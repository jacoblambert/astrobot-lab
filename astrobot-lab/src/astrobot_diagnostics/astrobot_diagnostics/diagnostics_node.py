#!/usr/bin/env python3
"""Publish lightweight structured diagnostics for navigation and exploration.

The output topic is intentionally JSON on std_msgs/String for now. It is easy to
record in rosbags and lets us evolve fields while Phase 4 failure modes are
still changing.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quaternion(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform_xy(x: float, y: float, tx: float, ty: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def inverse_transform_xy(x: float, y: float, tx: float, ty: float, yaw: float) -> tuple[float, float]:
    dx = x - tx
    dy = y - ty
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class DiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__("diagnostics_node")
        self.schema_version = "astrobot_diagnostics.v1"
        self.started_at = time.monotonic()

        self.publish_topic = self.declare_parameter("publish_topic", "diagnostics").value
        self.period_sec = float(self.declare_parameter("period_sec", 2.0).value)
        self.gt_occupied_threshold = int(self.declare_parameter("gt_occupied_threshold", 50).value)
        self.high_cost_threshold = int(self.declare_parameter("high_cost_threshold", 80).value)
        self.lethal_cost_threshold = int(self.declare_parameter("lethal_cost_threshold", 100).value)
        self.corridor_radius_m = float(self.declare_parameter("path_corridor_radius_m", 0.35).value)
        self.goal_radius_m = float(self.declare_parameter("goal_radius_m", 0.75).value)
        self.progress_timeout_sec = float(self.declare_parameter("progress_timeout_sec", 35.0).value)
        self.progress_epsilon_m = float(self.declare_parameter("progress_epsilon_m", 0.10).value)
        self.alignment_yaw_mode = self.declare_parameter("alignment_yaw_mode", "zero").value
        self.alignment_yaw = float(self.declare_parameter("alignment_yaw", 0.0).value)

        self.maps: dict[str, OccupancyGrid] = {}
        self.latest_path: Path | None = None
        self.selected_goal: PoseStamped | None = None
        self.gt_pose: PoseStamped | None = None
        self.slam_pose: PoseStamped | None = None
        self.initial_gt_to_map: tuple[float, float, float] | None = None
        self.status: dict[str, Any] = {}
        self.nav_feedback: dict[str, Any] = {}
        self.last_distance_remaining: float | None = None
        self.last_progress_at = time.monotonic()
        self.cmd_vel: dict[str, float] = {}
        self.cmd_vel_nav: dict[str, float] = {}
        self.action_status: list[dict[str, Any]] = []
        self.last_failure_count = 0
        self.previous_nav_sample: dict[str, Any] | None = None
        self.latest_motion_diagnostics: dict[str, Any] = {}

        self.pub = self.create_publisher(String, self.publish_topic, 10)

        for name, topic in {
            "bev": "slam/bev_costmap",
            "global": "global_costmap/costmap",
            "local": "local_costmap/costmap",
            "gt": "/gt/map",
        }.items():
            self.create_subscription(OccupancyGrid, topic, lambda msg, key=name: self._map_cb(key, msg), 2)

        self.create_subscription(Path, "plan", self._path_cb, 5)
        self.create_subscription(PoseStamped, "exploration/selected_goal", self._selected_goal_cb, 10)
        self.create_subscription(String, "exploration/status", self._status_cb, 10)
        self.create_subscription(PoseStamped, "/gt/base_link_pose", self._gt_pose_cb, 10)
        self.create_subscription(PoseStamped, "slam/pose_corrected", self._slam_pose_cb, 10)
        self.create_subscription(Twist, "cmd_vel", lambda msg: self._twist_cb(msg, "cmd_vel"), 10)
        self.create_subscription(Twist, "cmd_vel_nav", lambda msg: self._twist_cb(msg, "cmd_vel_nav"), 10)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", self._action_status_cb, 10)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "navigate_to_pose/_action/feedback",
            self._nav_feedback_cb,
            10,
        )
        self.create_timer(self.period_sec, self._tick)

    def _map_cb(self, key: str, msg: OccupancyGrid) -> None:
        self.maps[key] = msg

    def _path_cb(self, msg: Path) -> None:
        self.latest_path = msg

    def _selected_goal_cb(self, msg: PoseStamped) -> None:
        self.selected_goal = msg

    def _status_cb(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status = {"raw": msg.data}

    def _gt_pose_cb(self, msg: PoseStamped) -> None:
        self.gt_pose = msg
        self._maybe_lock_alignment()

    def _slam_pose_cb(self, msg: PoseStamped) -> None:
        self.slam_pose = msg
        self._maybe_lock_alignment()

    def _twist_cb(self, msg: Twist, key: str) -> None:
        target = {
            "linear_x": float(msg.linear.x),
            "linear_y": float(msg.linear.y),
            "angular_z": float(msg.angular.z),
        }
        if key == "cmd_vel":
            self.cmd_vel = target
        else:
            self.cmd_vel_nav = target

    def _action_status_cb(self, msg: GoalStatusArray) -> None:
        self.action_status = [
            {
                "status": int(status.status),
                "goal_id": "".join(f"{b:02x}" for b in status.goal_info.goal_id.uuid[:4]),
            }
            for status in msg.status_list[-4:]
        ]

    def _nav_feedback_cb(self, msg: NavigateToPose.Impl.FeedbackMessage) -> None:
        feedback = msg.feedback
        distance = float(feedback.distance_remaining)
        if self.last_distance_remaining is None or distance < self.last_distance_remaining - self.progress_epsilon_m:
            self.last_progress_at = time.monotonic()
        self.last_distance_remaining = distance
        pose = feedback.current_pose.pose
        self.nav_feedback = {
            "distance_remaining_m": distance,
            "recoveries": int(feedback.number_of_recoveries),
            "navigation_time_sec": int(feedback.navigation_time.sec) + 1e-9 * int(feedback.navigation_time.nanosec),
            "estimated_time_remaining_sec": int(feedback.estimated_time_remaining.sec)
            + 1e-9 * int(feedback.estimated_time_remaining.nanosec),
            "current_pose": [float(pose.position.x), float(pose.position.y), yaw_from_quaternion(pose.orientation)],
            "seconds_since_progress": float(time.monotonic() - self.last_progress_at),
        }

    @staticmethod
    def _pose_tuple(msg: PoseStamped) -> Pose2D:
        return Pose2D(float(msg.pose.position.x), float(msg.pose.position.y), yaw_from_quaternion(msg.pose.orientation))

    def _maybe_lock_alignment(self) -> None:
        if self.initial_gt_to_map is not None or self.gt_pose is None or self.slam_pose is None:
            return
        gt = self._pose_tuple(self.gt_pose)
        slam = self._pose_tuple(self.slam_pose)
        yaw = self.alignment_yaw
        if self.alignment_yaw_mode == "pose":
            yaw = math.atan2(math.sin(slam.yaw - gt.yaw), math.cos(slam.yaw - gt.yaw))
        sx, sy = transform_xy(gt.x, gt.y, 0.0, 0.0, yaw)
        self.initial_gt_to_map = (slam.x - sx, slam.y - sy, yaw)
        self.get_logger().info(
            "locked gt->map alignment "
            f"x={self.initial_gt_to_map[0]:.3f} y={self.initial_gt_to_map[1]:.3f} yaw={self.initial_gt_to_map[2]:.3f}"
        )

    def _tick(self) -> None:
        record = {
            "schema": self.schema_version,
            "stamp": self.get_clock().now().nanoseconds * 1e-9,
            "elapsed_sec": time.monotonic() - self.started_at,
            "status": self.status,
            "nav": self._nav_record(),
            "goal": self._goal_record(),
            "maps": self._map_records(),
            "classification": self._classify(),
        }
        msg = String()
        msg.data = json.dumps(record, sort_keys=True, separators=(",", ":"))
        self.pub.publish(msg)

        failures = int(self.status.get("failures", 0) or 0)
        if failures > self.last_failure_count:
            self.get_logger().warn(f"diagnostic failure snapshot: {json.dumps(record['classification'], sort_keys=True)}")
        self.last_failure_count = failures

    def _nav_record(self) -> dict[str, Any]:
        output = {
            "feedback": self.nav_feedback,
            "action_status": self.action_status,
            "cmd_vel": self.cmd_vel,
            "cmd_vel_nav": self.cmd_vel_nav,
            "path_pose_count": len(self.latest_path.poses) if self.latest_path is not None else 0,
        }
        if self.gt_pose is not None and self.slam_pose is not None and self.initial_gt_to_map is not None:
            gt = self._pose_tuple(self.gt_pose)
            slam = self._pose_tuple(self.slam_pose)
            pred_x, pred_y = transform_xy(gt.x, gt.y, *self.initial_gt_to_map)
            output["slam_vs_gt"] = {
                "position_error_m": math.hypot(pred_x - slam.x, pred_y - slam.y),
                "alignment": {
                    "x": self.initial_gt_to_map[0],
                    "y": self.initial_gt_to_map[1],
                    "yaw": self.initial_gt_to_map[2],
                },
            }
            motion = self._motion_record(gt, slam, pred_x, pred_y, output["slam_vs_gt"]["position_error_m"])
            output["motion"] = motion
            self.latest_motion_diagnostics = motion
        return output

    def _motion_record(
        self, gt: Pose2D, slam: Pose2D, aligned_gt_x: float, aligned_gt_y: float, position_error_m: float
    ) -> dict[str, Any]:
        now = time.monotonic()
        sample = {
            "time": now,
            "gt": gt,
            "slam": slam,
            "aligned_gt_x": aligned_gt_x,
            "aligned_gt_y": aligned_gt_y,
            "position_error_m": position_error_m,
        }
        previous = self.previous_nav_sample
        self.previous_nav_sample = sample
        if previous is None:
            return {"position_error_m": position_error_m}

        dt = max(1e-6, now - float(previous["time"]))
        previous_gt = previous["gt"]
        previous_slam = previous["slam"]
        gt_delta_m = math.hypot(gt.x - previous_gt.x, gt.y - previous_gt.y)
        slam_delta_m = math.hypot(slam.x - previous_slam.x, slam.y - previous_slam.y)
        aligned_gt_delta_m = math.hypot(
            aligned_gt_x - float(previous["aligned_gt_x"]), aligned_gt_y - float(previous["aligned_gt_y"])
        )
        error_delta_m = position_error_m - float(previous["position_error_m"])
        gt_yaw_delta_rad = wrap_angle(gt.yaw - previous_gt.yaw)
        slam_yaw_delta_rad = wrap_angle(slam.yaw - previous_slam.yaw)
        yaw_delta_error_rad = wrap_angle(slam_yaw_delta_rad - gt_yaw_delta_rad)
        cmd_linear = abs(float(self.cmd_vel.get("linear_x", 0.0)))
        cmd_angular = abs(float(self.cmd_vel.get("angular_z", 0.0)))
        gt_stall_with_cmd = cmd_linear > 0.08 and gt_delta_m / dt < 0.03

        return {
            "position_error_m": position_error_m,
            "dt_sec": dt,
            "error_delta_m": error_delta_m,
            "error_rate_mps": error_delta_m / dt,
            "gt_delta_m": gt_delta_m,
            "slam_delta_m": slam_delta_m,
            "aligned_gt_delta_m": aligned_gt_delta_m,
            "delta_mismatch_m": abs(slam_delta_m - aligned_gt_delta_m),
            "gt_speed_mps": gt_delta_m / dt,
            "slam_speed_mps": slam_delta_m / dt,
            "gt_yaw_delta_rad": gt_yaw_delta_rad,
            "slam_yaw_delta_rad": slam_yaw_delta_rad,
            "yaw_delta_error_rad": yaw_delta_error_rad,
            "cmd_linear_abs": cmd_linear,
            "cmd_angular_abs": cmd_angular,
            "gt_stall_with_cmd": gt_stall_with_cmd,
        }

    def _goal_record(self) -> dict[str, Any]:
        if self.selected_goal is None:
            return {}
        pose = self.selected_goal.pose
        return {
            "frame": self.selected_goal.header.frame_id,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": yaw_from_quaternion(pose.orientation),
        }

    def _map_records(self) -> dict[str, Any]:
        records: dict[str, Any] = {}
        path_points = self._path_points()
        goal_xy = self._goal_xy()
        for name, grid in self.maps.items():
            threshold = self.gt_occupied_threshold if name == "gt" else self.high_cost_threshold
            points = path_points
            if name == "gt" and points is not None:
                points = self._map_points_to_gt(points)
            records[name] = {
                "frame": grid.header.frame_id,
                "resolution": float(grid.info.resolution),
                "width": int(grid.info.width),
                "height": int(grid.info.height),
                "path": self._path_stats(grid, points, threshold),
                "goal": self._goal_stats(grid, goal_xy, threshold, name == "gt"),
            }
        return records

    def _path_points(self) -> list[tuple[float, float]] | None:
        if self.latest_path is None or not self.latest_path.poses:
            return None
        return [(float(p.pose.position.x), float(p.pose.position.y)) for p in self.latest_path.poses]

    def _goal_xy(self) -> tuple[float, float] | None:
        if self.selected_goal is None:
            return None
        return float(self.selected_goal.pose.position.x), float(self.selected_goal.pose.position.y)

    def _map_points_to_gt(self, points: list[tuple[float, float]] | None) -> list[tuple[float, float]] | None:
        if points is None or self.initial_gt_to_map is None:
            return None
        tx, ty, yaw = self.initial_gt_to_map
        return [inverse_transform_xy(x, y, tx, ty, yaw) for x, y in points]

    def _sample_grid(self, grid: OccupancyGrid, x: float, y: float) -> int | None:
        info = grid.info
        col = int(math.floor((x - info.origin.position.x) / info.resolution))
        row = int(math.floor((y - info.origin.position.y) / info.resolution))
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return None
        data = np.asarray(grid.data, dtype=np.int16).reshape((info.height, info.width))
        return int(data[row, col])

    def _path_stats(
        self, grid: OccupancyGrid, points: list[tuple[float, float]] | None, occupied_threshold: int
    ) -> dict[str, Any]:
        if not points:
            return {}
        info = grid.info
        data = np.asarray(grid.data, dtype=np.int16).reshape((info.height, info.width))
        radius_cells = max(0, int(math.ceil(self.corridor_radius_m / info.resolution)))
        visited: set[tuple[int, int]] = set()
        for x, y in points:
            col = int(math.floor((x - info.origin.position.x) / info.resolution))
            row = int(math.floor((y - info.origin.position.y) / info.resolution))
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    if dr * dr + dc * dc > radius_cells * radius_cells:
                        continue
                    rr = row + dr
                    cc = col + dc
                    if 0 <= rr < info.height and 0 <= cc < info.width:
                        visited.add((rr, cc))
        if not visited:
            return {"cells": 0}
        vals = np.asarray([data[r, c] for r, c in visited], dtype=np.int16)
        known = vals >= 0
        occupied = vals >= occupied_threshold
        high = vals >= self.high_cost_threshold
        lethal = vals >= self.lethal_cost_threshold
        return {
            "cells": int(vals.size),
            "known_fraction": float(np.count_nonzero(known) / vals.size),
            "unknown_cells": int(np.count_nonzero(~known)),
            "occupied_or_high_cells": int(np.count_nonzero(occupied)),
            "occupied_or_high_fraction": float(np.count_nonzero(occupied) / vals.size),
            "lethal_cells": int(np.count_nonzero(lethal)),
            "high_cost_cells": int(np.count_nonzero(high)),
            "max": int(vals.max()),
            "mean_known": float(vals[known].mean()) if np.any(known) else None,
        }

    def _goal_stats(
        self, grid: OccupancyGrid, goal_xy: tuple[float, float] | None, occupied_threshold: int, convert_to_gt: bool
    ) -> dict[str, Any]:
        if goal_xy is None:
            return {}
        x, y = goal_xy
        if convert_to_gt and self.initial_gt_to_map is not None:
            x, y = inverse_transform_xy(x, y, *self.initial_gt_to_map)
        info = grid.info
        data = np.asarray(grid.data, dtype=np.int16).reshape((info.height, info.width))
        col = int(math.floor((x - info.origin.position.x) / info.resolution))
        row = int(math.floor((y - info.origin.position.y) / info.resolution))
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return {"inside": False}
        radius_cells = max(1, int(math.ceil(self.goal_radius_m / info.resolution)))
        r0 = max(0, row - radius_cells)
        r1 = min(info.height, row + radius_cells + 1)
        c0 = max(0, col - radius_cells)
        c1 = min(info.width, col + radius_cells + 1)
        window = data[r0:r1, c0:c1]
        occupied = window >= occupied_threshold
        return {
            "inside": True,
            "cell": [row, col],
            "value": int(data[row, col]),
            "near_occupied_or_high_cells": int(np.count_nonzero(occupied)),
            "window_cells": int(window.size),
            "max": int(window.max()),
            "mean_known": float(window[window >= 0].mean()) if np.any(window >= 0) else None,
        }

    def _classify(self) -> dict[str, Any]:
        reasons: list[str] = []
        confidence = "low"
        maps = self._map_records()
        gt_path = maps.get("gt", {}).get("path", {})
        bev_path = maps.get("bev", {}).get("path", {})
        global_path = maps.get("global", {}).get("path", {})
        local_goal = maps.get("local", {}).get("goal", {})
        gt_goal = maps.get("gt", {}).get("goal", {})
        feedback = self.nav_feedback
        motion = self.latest_motion_diagnostics

        if motion.get("error_delta_m", 0.0) > 1.0 and motion.get("error_rate_mps", 0.0) > 0.25:
            reasons.append("slam_pose_error_jump")
            confidence = "high"
        elif motion.get("position_error_m", 0.0) > 2.0 and motion.get("error_rate_mps", 0.0) > 0.05:
            reasons.append("slam_pose_error_drifting")
            confidence = "high"
        if motion.get("delta_mismatch_m", 0.0) > 0.75:
            reasons.append("slam_gt_increment_mismatch")
            confidence = "high"
        if motion.get("gt_stall_with_cmd", False):
            reasons.append("gt_motion_stalled_while_commanded")
            confidence = "medium"
        if gt_path.get("occupied_or_high_fraction", 0.0) > 0.01:
            reasons.append("planned_path_intersects_gt_obstacle_corridor")
            confidence = "medium"
        if gt_goal.get("near_occupied_or_high_cells", 0) > 0:
            reasons.append("selected_goal_near_gt_obstacle")
            confidence = "medium"
        if bev_path.get("occupied_or_high_fraction", 0.0) > 0.05:
            reasons.append("planned_path_intersects_bev_high_cost")
            confidence = "medium"
        if global_path.get("occupied_or_high_fraction", 0.0) > 0.05:
            reasons.append("planned_path_intersects_nav2_global_high_cost")
            confidence = "medium"
        if local_goal.get("near_occupied_or_high_cells", 0) > 0:
            reasons.append("goal_area_high_cost_in_local_costmap")
        if feedback.get("seconds_since_progress", 0.0) > self.progress_timeout_sec:
            reasons.append("nav2_feedback_no_distance_progress")
            confidence = "high" if reasons else "medium"
        if feedback.get("distance_remaining_m", 0.0) < 0.75 and feedback.get("seconds_since_progress", 0.0) > 10.0:
            reasons.append("near_goal_progress_checker_or_goal_tolerance_issue")
            confidence = "medium"

        if not reasons:
            reasons.append("no_active_fault_detected")

        return {
            "primary": reasons[0],
            "reasons": reasons,
            "confidence": confidence,
        }


def main() -> None:
    rclpy.init()
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
