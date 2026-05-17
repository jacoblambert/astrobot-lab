#!/usr/bin/env python3
"""Run Phase 3 short-horizon SLAM-frame navigation probes."""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


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
    error_msg: str
    duration_sec: float
    endpoint_error_m: float | None


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def normalize_angle(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    return Pose2D(
        x=a.x + c * b.x - s * b.y,
        y=a.y + s * b.x + c * b.y,
        yaw=normalize_angle(a.yaw + b.yaw),
    )


class Phase3ProbeSweep(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("phase3_probe_sweep")
        self.args = args
        self.latest_pose: Pose2D | None = None
        self.create_subscription(PoseStamped, args.pose_topic, self._pose_cb, 20)
        self.client = ActionClient(self, NavigateToPose, args.action_name)

    def _pose_cb(self, msg: PoseStamped) -> None:
        imu_pose = Pose2D(
            msg.pose.position.x,
            msg.pose.position.y,
            yaw_from_quaternion(msg.pose.orientation),
        )
        imu_to_base = Pose2D(self.args.base_offset_x, self.args.base_offset_y, self.args.base_yaw_offset)
        self.latest_pose = compose(imu_pose, imu_to_base)

    def wait_for_pose(self, timeout_sec: float) -> Pose2D:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose is not None:
                return self.latest_pose
        raise TimeoutError(f"no pose received on {self.args.pose_topic}")

    def send_goal(self, index: int, kind: str, pose: Pose2D) -> GoalResult:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.args.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose.x
        goal_msg.pose.pose.position.y = pose.y
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        if not self.client.wait_for_server(timeout_sec=self.args.server_timeout):
            raise TimeoutError(f"{self.args.action_name} action server unavailable")

        start = time.monotonic()
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.server_timeout)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return GoalResult(index, kind, pose.x, pose.y, GoalStatus.STATUS_REJECTED, -1, "goal rejected", 0.0, None)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.goal_timeout)
        duration = time.monotonic() - start
        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return GoalResult(index, kind, pose.x, pose.y, GoalStatus.STATUS_CANCELED, -2, "goal timeout", duration, None)

        action_result = result_future.result()
        endpoint_error = None
        if self.latest_pose is not None:
            endpoint_error = math.hypot(self.latest_pose.x - pose.x, self.latest_pose.y - pose.y)

        return GoalResult(
            index=index,
            kind=kind,
            target_x=pose.x,
            target_y=pose.y,
            status=action_result.status,
            error_code=action_result.result.error_code,
            error_msg=action_result.result.error_msg,
            duration_sec=duration,
            endpoint_error_m=endpoint_error,
        )


def parse_relative_goal(value: str) -> Pose2D:
    fields = [float(part.strip()) for part in value.split(",")]
    if len(fields) == 2:
        fields.append(0.0)
    if len(fields) != 3:
        raise argparse.ArgumentTypeError("relative goals must be dx,dy[,dyaw]")
    return Pose2D(fields[0], fields[1], fields[2])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default="/glim_rosnode/pose_corrected")
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--frame-id", default="glim_map")
    parser.add_argument("--server-timeout", type=float, default=30.0)
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--pose-timeout", type=float, default=30.0)
    parser.add_argument("--return-home-every", type=int, default=3)
    parser.add_argument("--output", default="")
    parser.add_argument("--base-offset-x", type=float, default=0.264)
    parser.add_argument("--base-offset-y", type=float, default=0.017)
    parser.add_argument("--base-yaw-offset", type=float, default=math.pi)
    parser.add_argument(
        "--relative-goal",
        action="append",
        type=parse_relative_goal,
        default=[],
        help="Relative SLAM-frame probe goal as dx,dy[,dyaw]. Repeatable.",
    )
    args = parser.parse_args()
    if not args.relative_goal:
        args.relative_goal = [
            Pose2D(1.0, 0.0, 0.0),
            Pose2D(0.0, 1.0, 0.0),
            Pose2D(-1.0, 0.0, 0.0),
            Pose2D(0.0, -1.0, 0.0),
            Pose2D(1.5, 0.5, 0.0),
            Pose2D(-1.5, -0.5, 0.0),
        ]

    rclpy.init()
    node = Phase3ProbeSweep(args)
    try:
        home = node.wait_for_pose(args.pose_timeout)
        results: list[GoalResult] = []
        index = 0
        probe_count = 0
        for offset in args.relative_goal:
            index += 1
            probe_count += 1
            target = compose(home, offset)
            node.get_logger().info(f"goal {index}: probe x={target.x:.2f} y={target.y:.2f}")
            results.append(node.send_goal(index, "probe", target))
            if args.return_home_every > 0 and probe_count % args.return_home_every == 0:
                index += 1
                node.get_logger().info(f"goal {index}: return_home x={home.x:.2f} y={home.y:.2f}")
                results.append(node.send_goal(index, "return_home", home))

        successes = [r for r in results if r.status == GoalStatus.STATUS_SUCCEEDED]
        summary = {
            "home": asdict(home),
            "total_goals": len(results),
            "successes": len(successes),
            "success_rate": len(successes) / len(results) if results else 0.0,
            "return_home_total": sum(1 for r in results if r.kind == "return_home"),
            "return_home_successes": sum(
                1 for r in results if r.kind == "return_home" and r.status == GoalStatus.STATUS_SUCCEEDED
            ),
            "results": [asdict(result) for result in results],
        }
        output = json.dumps(summary, indent=2, sort_keys=True)
        print(output)
        if args.output:
            Path(args.output).write_text(output + "\n", encoding="utf-8")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
