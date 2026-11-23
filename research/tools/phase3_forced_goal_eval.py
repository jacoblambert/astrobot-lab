#!/usr/bin/env python3
"""Forced-goal navigation evaluation for the nav-controller overhaul.

Drives a fixed set of map-frame goals (offsets from the start pose, map axes)
directly to NavigateToPose -- bypassing the exploration manager -- to isolate
and quantify controller smoothness/reliability on repeatable hard scenarios
(goals routing near/around known rocks). Records per-goal:
  - success, endpoint error, duration
  - rotate-in-place episodes   (from /cmd_vel: |wz|>thr & |vx|~0 sustained)
  - max number_of_recoveries   (NavigateToPose feedback)
  - max SLAM-vs-GT error        (/diagnostics nav.slam_vs_gt.position_error_m)
  - cmd reversals               (sign flips of vx / wz = oscillation proxy)
and prints a PASS/FAIL summary against smoothness criteria.

Run the autonomy stack with phase3_nav.launch.py (SLAM+Nav2+MPPI, no manager).
"""
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class GoalResult:
    index: int
    kind: str
    target_x: float
    target_y: float
    status: int
    error_code: int
    duration_sec: float
    endpoint_error_m: float | None
    rotate_in_place_episodes: int
    max_recoveries: int
    max_slam_err_m: float
    cmd_reversals: int


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def normalize_angle(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def parse_goal(value: str) -> Pose2D:
    fields = [float(p.strip()) for p in value.split(",")]
    if len(fields) == 2:
        fields.append(0.0)
    if len(fields) != 3:
        raise argparse.ArgumentTypeError("goals must be dx,dy[,dyaw]")
    return Pose2D(fields[0], fields[1], fields[2])


class ForcedGoalEval(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("phase3_forced_goal_eval")
        self.args = args
        self.latest_pose: Pose2D | None = None
        # per-goal accumulators (reset by reset_goal_metrics)
        self._rip_episodes = 0
        self._in_rip = 0          # consecutive cmd_vel samples in rotate-in-place state
        self._cmd_reversals = 0
        self._last_vx_sign = 0
        self._last_wz_sign = 0
        self._max_recoveries = 0
        self._max_slam_err = 0.0
        self.create_subscription(PoseStamped, args.pose_topic, self._pose_cb, 20)
        self.create_subscription(Twist, args.cmd_vel_topic, self._cmd_cb, 50)
        self.create_subscription(String, args.diagnostics_topic, self._diag_cb, 10)
        self.client = ActionClient(self, NavigateToPose, args.action_name)

    def _pose_cb(self, msg: PoseStamped) -> None:
        imu = Pose2D(msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(msg.pose.orientation))
        c, s = math.cos(imu.yaw), math.sin(imu.yaw)
        self.latest_pose = Pose2D(
            imu.x + c * self.args.base_offset_x - s * self.args.base_offset_y,
            imu.y + s * self.args.base_offset_x + c * self.args.base_offset_y,
            normalize_angle(imu.yaw + self.args.base_yaw_offset),
        )

    def _cmd_cb(self, msg: Twist) -> None:
        vx, wz = msg.linear.x, msg.angular.z
        # rotate-in-place: rotating hard with ~no forward motion
        if abs(wz) > self.args.rip_wz_thr and abs(vx) < self.args.rip_vx_thr:
            self._in_rip += 1
            if self._in_rip == self.args.rip_min_samples:  # episode onset (count once)
                self._rip_episodes += 1
        else:
            self._in_rip = 0
        # oscillation: sign flips (ignore near-zero)
        vs = 1 if vx > 0.02 else (-1 if vx < -0.02 else 0)
        ws = 1 if wz > 0.05 else (-1 if wz < -0.05 else 0)
        if vs != 0 and self._last_vx_sign != 0 and vs != self._last_vx_sign:
            self._cmd_reversals += 1
        if ws != 0 and self._last_wz_sign != 0 and ws != self._last_wz_sign:
            self._cmd_reversals += 1
        if vs != 0:
            self._last_vx_sign = vs
        if ws != 0:
            self._last_wz_sign = ws

    def _diag_cb(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            err = float(d.get("nav", {}).get("slam_vs_gt", {}).get("position_error_m", 0.0))
        except (ValueError, TypeError, AttributeError):
            return
        self._max_slam_err = max(self._max_slam_err, err)

    def _feedback_cb(self, feedback_msg) -> None:
        try:
            self._max_recoveries = max(self._max_recoveries, int(feedback_msg.feedback.number_of_recoveries))
        except AttributeError:
            pass

    def reset_goal_metrics(self) -> None:
        self._rip_episodes = 0
        self._in_rip = 0
        self._cmd_reversals = 0
        self._last_vx_sign = 0
        self._last_wz_sign = 0
        self._max_recoveries = 0
        self._max_slam_err = 0.0

    def wait_for_pose(self, timeout_sec: float) -> Pose2D:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose is not None:
                return self.latest_pose
        raise TimeoutError(f"no pose on {self.args.pose_topic}")

    def teleport(self, x: float, y: float, z: float) -> None:
        pub = self.create_publisher(PoseStamped, self.args.teleport_topic, 1)
        msg = PoseStamped()
        msg.header.frame_id = self.args.robot_namespace
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.w = 1.0
        for _ in range(5):
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.2)

    def send_goal(self, index: int, kind: str, pose: Pose2D) -> GoalResult:
        self.reset_goal_metrics()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.args.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        if not self.client.wait_for_server(timeout_sec=self.args.server_timeout):
            raise TimeoutError(f"{self.args.action_name} unavailable")
        start = time.monotonic()
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.server_timeout)
        handle = future.result()
        if handle is None or not handle.accepted:
            return GoalResult(index, kind, pose.x, pose.y, GoalStatus.STATUS_REJECTED, -1, 0.0, None,
                              self._rip_episodes, self._max_recoveries, self._max_slam_err, self._cmd_reversals)
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.goal_timeout)
        duration = time.monotonic() - start
        if not result_future.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            return GoalResult(index, kind, pose.x, pose.y, GoalStatus.STATUS_CANCELED, -2, duration, None,
                              self._rip_episodes, self._max_recoveries, self._max_slam_err, self._cmd_reversals)
        res = result_future.result()
        endpoint_err = None
        if self.latest_pose is not None:
            endpoint_err = math.hypot(self.latest_pose.x - pose.x, self.latest_pose.y - pose.y)
        return GoalResult(index, kind, pose.x, pose.y, res.status, res.result.error_code, duration, endpoint_err,
                          self._rip_episodes, self._max_recoveries, self._max_slam_err, self._cmd_reversals)


# Default goal set (offsets from start, map axes) routing toward/around the
# seed-42 rock region (~map (-1.6,-6.7) from spawn) and back -- forces the hard
# maneuvers that previously caused rotate-in-place / wedging.
DEFAULT_GOALS = [
    Pose2D(0.0, -4.0, 0.0),
    Pose2D(-1.5, -7.0, 0.0),
    Pose2D(1.5, -7.0, 0.0),
    Pose2D(0.0, -9.0, 0.0),
    Pose2D(-3.0, -5.0, 0.0),
    Pose2D(3.0, -5.0, 0.0),
]


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--pose-topic", default="/astrobot_0/slam/pose_corrected")
    p.add_argument("--cmd-vel-topic", default="/astrobot_0/cmd_vel")
    p.add_argument("--diagnostics-topic", default="/astrobot_0/diagnostics")
    p.add_argument("--action-name", default="/astrobot_0/navigate_to_pose")
    p.add_argument("--teleport-topic", default="/OmniLRS/Robots/Teleport")
    p.add_argument("--robot-namespace", default="astrobot_0")
    p.add_argument("--frame-id", default="map")
    p.add_argument("--server-timeout", type=float, default=30.0)
    p.add_argument("--goal-timeout", type=float, default=120.0)
    p.add_argument("--pose-timeout", type=float, default=40.0)
    p.add_argument("--return-home-every", type=int, default=2)
    p.add_argument("--teleport", default="", help="x,y,z world teleport before sweep (optional)")
    p.add_argument("--base-offset-x", type=float, default=0.264)
    p.add_argument("--base-offset-y", type=float, default=0.017)
    p.add_argument("--base-yaw-offset", type=float, default=math.pi)
    p.add_argument("--rip-wz-thr", type=float, default=0.25, help="rotate-in-place: |wz| above this")
    p.add_argument("--rip-vx-thr", type=float, default=0.03, help="rotate-in-place: |vx| below this")
    p.add_argument("--rip-min-samples", type=int, default=30, help="~1.5s at 20Hz to count an episode")
    p.add_argument("--goal", action="append", type=parse_goal, default=[], help="offset dx,dy[,dyaw] from start. Repeatable.")
    p.add_argument("--output", default="")
    # pass/fail criteria
    p.add_argument("--crit-success-rate", type=float, default=0.90)
    p.add_argument("--crit-max-rip-per-goal", type=float, default=1.0)
    p.add_argument("--crit-max-recoveries-per-goal", type=float, default=1.0)
    p.add_argument("--crit-max-slam-err", type=float, default=0.8)
    p.add_argument("--crit-endpoint-err", type=float, default=0.6)
    args = p.parse_args()
    goals = args.goal if args.goal else DEFAULT_GOALS

    rclpy.init()
    node = ForcedGoalEval(args)
    try:
        if args.teleport:
            tx, ty, tz = [float(v) for v in args.teleport.split(",")]
            node.teleport(tx, ty, tz)
            time.sleep(2.0)
        home = node.wait_for_pose(args.pose_timeout)
        node.get_logger().info(f"home(map) x={home.x:.2f} y={home.y:.2f} yaw={home.yaw:.2f}")
        results: list[GoalResult] = []
        index = 0
        for i, off in enumerate(goals, 1):
            index += 1
            target = Pose2D(home.x + off.x, home.y + off.y, off.yaw)  # translate-only, map axes
            node.get_logger().info(f"goal {index}: forced x={target.x:.2f} y={target.y:.2f}")
            r = node.send_goal(index, "forced", target)
            node.get_logger().info(
                f"  -> status={r.status} err={r.endpoint_error_m} dur={r.duration_sec:.1f}s "
                f"rip={r.rotate_in_place_episodes} recov={r.max_recoveries} "
                f"slam_err={r.max_slam_err_m:.2f} reversals={r.cmd_reversals}"
            )
            results.append(r)
            if args.return_home_every > 0 and i % args.return_home_every == 0:
                index += 1
                node.get_logger().info(f"goal {index}: return_home x={home.x:.2f} y={home.y:.2f}")
                results.append(node.send_goal(index, "return_home", home))

        succ = [r for r in results if r.status == GoalStatus.STATUS_SUCCEEDED]
        n = len(results)
        success_rate = len(succ) / n if n else 0.0
        rip_per_goal = sum(r.rotate_in_place_episodes for r in results) / n if n else 0.0
        recov_per_goal = sum(r.max_recoveries for r in results) / n if n else 0.0
        max_slam = max((r.max_slam_err_m for r in results), default=0.0)
        endpoint_errs = [r.endpoint_error_m for r in succ if r.endpoint_error_m is not None]
        max_endpoint = max(endpoint_errs, default=0.0)
        criteria = {
            "success_rate": (success_rate, success_rate >= args.crit_success_rate),
            "rip_per_goal": (rip_per_goal, rip_per_goal <= args.crit_max_rip_per_goal),
            "recoveries_per_goal": (recov_per_goal, recov_per_goal <= args.crit_max_recoveries_per_goal),
            "max_slam_err_m": (max_slam, max_slam <= args.crit_max_slam_err),
            "max_endpoint_err_m": (max_endpoint, max_endpoint <= args.crit_endpoint_err),
        }
        overall_pass = all(ok for _, ok in criteria.values())
        summary = {
            "home": asdict(home),
            "total_goals": n,
            "successes": len(succ),
            "success_rate": success_rate,
            "rip_per_goal": rip_per_goal,
            "recoveries_per_goal": recov_per_goal,
            "max_slam_err_m": max_slam,
            "max_endpoint_err_m": max_endpoint,
            "criteria": {k: {"value": v, "pass": ok} for k, (v, ok) in criteria.items()},
            "overall_pass": overall_pass,
            "results": [asdict(r) for r in results],
        }
        out = json.dumps(summary, indent=2, sort_keys=True)
        print(out)
        print(f"\nOVERALL_PASS={overall_pass}  success_rate={success_rate:.2f} rip/goal={rip_per_goal:.2f} "
              f"recov/goal={recov_per_goal:.2f} max_slam_err={max_slam:.2f} max_endpoint_err={max_endpoint:.2f}")
        if args.output:
            Path(args.output).write_text(out + "\n", encoding="utf-8")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
