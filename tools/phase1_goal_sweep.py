#!/usr/bin/env python3
import argparse
import math
import random
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def compose_pose(a: Pose2D, b: Pose2D) -> Pose2D:
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    return Pose2D(
        x=a.x + c * b.x - s * b.y,
        y=a.y + s * b.x + c * b.y,
        yaw=a.yaw + b.yaw,
    )


class GoalSweepNode(Node):
    def __init__(self, pose_topic: Optional[str] = None) -> None:
        super().__init__("phase1_goal_sweep")
        self.map_msg: Optional[OccupancyGrid] = None
        self.odom_msg: Optional[Odometry] = None
        self.pose_msg: Optional[PoseStamped] = None
        self.pose_topic = pose_topic
        self.tf_msgs = []
        self.tf_static_msgs = []
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 50)
        if self.pose_topic:
            self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 50)
        self.tf_sub = self.create_subscription(TFMessage, "/tf", self._tf_cb, 100)
        self.tf_static_sub = self.create_subscription(TFMessage, "/tf_static", self._tf_static_cb, 100)
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_msg = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.pose_msg = msg

    def _tf_cb(self, msg: TFMessage) -> None:
        self.tf_msgs.extend(msg.transforms)
        self.tf_msgs = self.tf_msgs[-500:]

    def _tf_static_cb(self, msg: TFMessage) -> None:
        self.tf_static_msgs.extend(msg.transforms)
        self.tf_static_msgs = self.tf_static_msgs[-200:]

    def latest_transform(self, parent: str, child: str):
        all_tf = self.tf_msgs + self.tf_static_msgs
        matches = [t for t in all_tf if t.header.frame_id == parent and t.child_frame_id == child]
        return matches[-1] if matches else None

    def current_pose_in_map(self) -> Pose2D:
        if self.pose_msg is not None:
            return Pose2D(
                x=self.pose_msg.pose.position.x,
                y=self.pose_msg.pose.position.y,
                yaw=yaw_from_quaternion(self.pose_msg.pose.orientation),
            )

        map_to_odom = self.latest_transform("map", "odom")
        if map_to_odom is None or self.odom_msg is None:
            raise RuntimeError("missing map->odom transform or /odom")
        p1 = Pose2D(
            x=map_to_odom.transform.translation.x,
            y=map_to_odom.transform.translation.y,
            yaw=yaw_from_quaternion(map_to_odom.transform.rotation),
        )
        odom_pose = self.odom_msg.pose.pose
        p2 = Pose2D(
            x=odom_pose.position.x,
            y=odom_pose.position.y,
            yaw=yaw_from_quaternion(odom_pose.orientation),
        )
        return compose_pose(p1, p2)

    def occupancy_at(self, x: float, y: float) -> Optional[int]:
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        col = int(math.floor((x - info.origin.position.x) / info.resolution))
        row = int(math.floor((y - info.origin.position.y) / info.resolution))
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return 100
        return int(self.map_msg.data[row * info.width + col])

    def map_xy_for_cell(self, row: int, col: int) -> tuple[float, float]:
        if self.map_msg is None:
            raise RuntimeError("missing map")
        info = self.map_msg.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    def cell_is_clear(self, row: int, col: int, clearance_cells: int, free_threshold: int) -> bool:
        if self.map_msg is None:
            return False
        info = self.map_msg.info
        for rr in range(row - clearance_cells, row + clearance_cells + 1):
            for cc in range(col - clearance_cells, col + clearance_cells + 1):
                if rr < 0 or rr >= info.height or cc < 0 or cc >= info.width:
                    return False
                occ = int(self.map_msg.data[rr * info.width + cc])
                if occ < 0 or occ > free_threshold:
                    return False
        return True

    def sample_goals_from_map(
        self,
        start: Pose2D,
        free_count: int,
        occupied_count: int,
        min_distance: float,
        max_distance: float,
        clearance_m: float,
        free_threshold: int,
        occupied_threshold: int,
        seed: int,
    ) -> list[tuple[float, float, str]]:
        if self.map_msg is None:
            raise RuntimeError("missing map")

        rng = random.Random(seed)
        info = self.map_msg.info
        clearance_cells = max(0, int(math.ceil(clearance_m / info.resolution)))
        free_cells = []
        occupied_cells = []
        for row in range(info.height):
            for col in range(info.width):
                occ = int(self.map_msg.data[row * info.width + col])
                x, y = self.map_xy_for_cell(row, col)
                dist = math.hypot(x - start.x, y - start.y)
                if dist < min_distance or dist > max_distance:
                    continue
                if 0 <= occ <= free_threshold and self.cell_is_clear(row, col, clearance_cells, free_threshold):
                    free_cells.append((row, col))
                elif occ >= occupied_threshold:
                    occupied_cells.append((row, col))

        rng.shuffle(free_cells)
        rng.shuffle(occupied_cells)
        goals = []
        for row, col in free_cells[:free_count]:
            x, y = self.map_xy_for_cell(row, col)
            goals.append((x, y, "free"))
        for row, col in occupied_cells[:occupied_count]:
            x, y = self.map_xy_for_cell(row, col)
            goals.append((x, y, "occupied"))
        return goals

    def wait_for_data(self, timeout_sec: float = 10.0) -> None:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            has_pose_source = self.pose_msg is not None or (
                self.odom_msg is not None and self.latest_transform("map", "odom")
            )
            if self.map_msg is not None and has_pose_source:
                return
        raise RuntimeError("timed out waiting for map and pose source")

    def send_goal(self, x: float, y: float, yaw: float, timeout_sec: float) -> tuple[int, Pose2D, int, str]:
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("navigate_to_pose server unavailable")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return -1, self.current_pose_in_map(), -1, "goal rejected"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            return GoalStatus.STATUS_ABORTED, self.current_pose_in_map(), -2, "timeout"

        result = result_future.result()
        nav_result = result.result
        return result.status, self.current_pose_in_map(), int(nav_result.error_code), nav_result.error_msg


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--auto-free", type=int, default=0, help="Sample this many free-space goals from /map.")
    parser.add_argument("--auto-occupied", type=int, default=0, help="Sample this many occupied goals from /map.")
    parser.add_argument("--min-distance", type=float, default=1.0)
    parser.add_argument("--max-distance", type=float, default=8.0)
    parser.add_argument("--clearance", type=float, default=0.35, help="Required free radius around auto-free goals.")
    parser.add_argument("--free-threshold", type=int, default=0)
    parser.add_argument("--occupied-threshold", type=int, default=50)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--pose-topic",
        default=None,
        help="Optional PoseStamped topic in map frame for route start and endpoint evaluation, e.g. /gt/base_link_pose.",
    )
    parser.add_argument(
        "--relative-goals",
        nargs="+",
        default=None,
        help="Relative dx,dy goals from current pose.",
    )
    parser.add_argument(
        "--relative-goal",
        action="append",
        dest="relative_goal_items",
        default=[],
        help="Single relative dx,dy goal from current pose. May be repeated and supports negative values as --relative-goal=-1.0,0.0.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = GoalSweepNode(pose_topic=args.pose_topic)
    try:
        node.wait_for_data()
        relative_goals = list(args.relative_goals or [])
        relative_goals.extend(args.relative_goal_items)
        if not relative_goals and not args.auto_free and not args.auto_occupied:
            relative_goals = [
                "0.8,0.0",
                "0.0,0.8",
                "-0.8,0.0",
                "0.0,-0.8",
                "0.8,0.8",
                "-0.8,0.8",
            ]

        start = node.current_pose_in_map()
        print(f"START x={start.x:.3f} y={start.y:.3f} yaw={start.yaw:.3f}")
        goals = []
        if args.auto_free or args.auto_occupied:
            goals.extend(
                node.sample_goals_from_map(
                    start=start,
                    free_count=args.auto_free,
                    occupied_count=args.auto_occupied,
                    min_distance=args.min_distance,
                    max_distance=args.max_distance,
                    clearance_m=args.clearance,
                    free_threshold=args.free_threshold,
                    occupied_threshold=args.occupied_threshold,
                    seed=args.seed,
                )
            )
            print(
                f"AUTO_GOALS free={sum(1 for _, _, kind in goals if kind == 'free')} "
                f"occupied={sum(1 for _, _, kind in goals if kind == 'occupied')} "
                f"min_distance={args.min_distance:.2f} max_distance={args.max_distance:.2f} "
                f"clearance={args.clearance:.2f} seed={args.seed}"
            )
        for item in relative_goals:
            dx_str, dy_str = item.split(",")
            dx = float(dx_str)
            dy = float(dy_str)
            goals.append((start.x + dx, start.y + dy, f"relative({dx:.3f},{dy:.3f})"))

        overall_ok = True
        free_results = []
        occupied_results = []
        for idx, (goal_x, goal_y, goal_kind) in enumerate(goals, start=1):
            goal_occ = node.occupancy_at(goal_x, goal_y)
            heading = math.atan2(goal_y - start.y, goal_x - start.x)
            print(
                f"GOAL {idx} target=({goal_x:.3f},{goal_y:.3f}) kind={goal_kind} "
                f"map_occ={goal_occ}"
            )
            status, pose, error_code, error_msg = node.send_goal(goal_x, goal_y, heading, timeout_sec=args.timeout)
            err = math.hypot(goal_x - pose.x, goal_y - pose.y)
            print(
                f"RESULT {idx} status={status} end=({pose.x:.3f},{pose.y:.3f}) "
                f"err={err:.3f} end_occ={node.occupancy_at(pose.x, pose.y)} "
                f"error_code={error_code} error_msg={error_msg!r}"
            )
            if goal_occ is not None and goal_occ >= 50:
                occupied_results.append((status, err, error_code))
                if status == GoalStatus.STATUS_SUCCEEDED:
                    overall_ok = False
            else:
                free_results.append((status, err, error_code))
                if status != GoalStatus.STATUS_SUCCEEDED or err > 0.35:
                    overall_ok = False
        if free_results:
            free_success = [item for item in free_results if item[0] == GoalStatus.STATUS_SUCCEEDED]
            free_errors = [item[1] for item in free_success]
            print(
                f"SUMMARY_FREE attempted={len(free_results)} succeeded={len(free_success)} "
                f"min_err={min(free_errors) if free_errors else float('nan'):.3f} "
                f"max_err={max(free_errors) if free_errors else float('nan'):.3f} "
                f"mean_err={sum(free_errors) / len(free_errors) if free_errors else float('nan'):.3f}"
            )
        if occupied_results:
            occupied_blocked = [item for item in occupied_results if item[0] != GoalStatus.STATUS_SUCCEEDED]
            print(
                f"SUMMARY_OCCUPIED attempted={len(occupied_results)} blocked={len(occupied_blocked)} "
                f"error_codes={sorted(set(item[2] for item in occupied_blocked))}"
            )
        return 0 if overall_ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
