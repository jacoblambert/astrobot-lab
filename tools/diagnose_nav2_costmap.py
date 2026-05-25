#!/usr/bin/env python3
import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CostmapDiagnostic(Node):
    def __init__(self, args):
        super().__init__("phase4_costmap_diagnostic")
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.maps: dict[str, OccupancyGrid] = {}
        self.status = {}
        self.selected_goal: PoseStamped | None = None
        self.cmd_vel = {}
        self.cmd_vel_nav = {}
        self.nav_feedback = {}
        self.started = time.monotonic()
        self.records = []

        for name, topic in {
            "local": f"/{args.namespace}/local_costmap/costmap",
            "global": f"/{args.namespace}/global_costmap/costmap",
            "bev": f"/{args.namespace}/slam/bev_costmap",
        }.items():
            self.create_subscription(OccupancyGrid, topic, lambda msg, key=name: self._map_cb(key, msg), 1)

        self.create_subscription(String, f"/{args.namespace}/exploration/status", self._status_cb, 10)
        self.create_subscription(PoseStamped, f"/{args.namespace}/exploration/selected_goal", self._goal_cb, 10)
        self.create_subscription(Twist, f"/{args.namespace}/cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(Twist, f"/{args.namespace}/cmd_vel_nav", self._cmd_vel_nav_cb, 10)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            f"/{args.namespace}/navigate_to_pose/_action/feedback",
            self._nav_feedback_cb,
            10,
        )
        self.timer = self.create_timer(args.period, self._tick)

    def _map_cb(self, key: str, msg: OccupancyGrid) -> None:
        self.maps[key] = msg

    def _status_cb(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status = {"raw": msg.data}

    def _goal_cb(self, msg: PoseStamped) -> None:
        self.selected_goal = msg

    @staticmethod
    def _twist_summary(msg: Twist) -> dict:
        return {
            "linear_x": float(msg.linear.x),
            "linear_y": float(msg.linear.y),
            "angular_z": float(msg.angular.z),
        }

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self.cmd_vel = self._twist_summary(msg)

    def _cmd_vel_nav_cb(self, msg: Twist) -> None:
        self.cmd_vel_nav = self._twist_summary(msg)

    def _nav_feedback_cb(self, msg: NavigateToPose.Impl.FeedbackMessage) -> None:
        feedback = msg.feedback
        pose = feedback.current_pose.pose
        self.nav_feedback = {
            "distance_remaining": float(feedback.distance_remaining),
            "number_of_recoveries": int(feedback.number_of_recoveries),
            "current_pose": [
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quat(pose.orientation),
            ],
            "navigation_time_sec": int(feedback.navigation_time.sec)
            + 1e-9 * int(feedback.navigation_time.nanosec),
            "estimated_time_remaining_sec": int(feedback.estimated_time_remaining.sec)
            + 1e-9 * int(feedback.estimated_time_remaining.nanosec),
        }

    def _lookup_pose(self, frame: str):
        try:
            transform = self.tf_buffer.lookup_transform(frame, f"{self.args.namespace}/base_link", rclpy.time.Time())
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(frame, "base_link", rclpy.time.Time())
            except TransformException:
                return None
        t = transform.transform.translation
        q = transform.transform.rotation
        return (t.x, t.y, yaw_from_quat(q))

    def _window_stats(self, msg: OccupancyGrid, pose):
        x, y, yaw = pose
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape((info.height, info.width))
        col = int(math.floor((x - info.origin.position.x) / info.resolution))
        row = int(math.floor((y - info.origin.position.y) / info.resolution))
        radius_cells = int(math.ceil(self.args.footprint_radius / info.resolution))
        forward_cells = int(math.ceil(self.args.forward_distance / info.resolution))

        def sample_mask(max_forward: float, lateral: float):
            coords = []
            span = max(radius_cells, forward_cells) + 2
            for rr in range(max(0, row - span), min(info.height, row + span + 1)):
                for cc in range(max(0, col - span), min(info.width, col + span + 1)):
                    wx = info.origin.position.x + (cc + 0.5) * info.resolution
                    wy = info.origin.position.y + (rr + 0.5) * info.resolution
                    dx = wx - x
                    dy = wy - y
                    fwd = math.cos(yaw) * dx + math.sin(yaw) * dy
                    side = -math.sin(yaw) * dx + math.cos(yaw) * dy
                    if -self.args.rear_distance <= fwd <= max_forward and abs(side) <= lateral:
                        coords.append((rr, cc))
            return coords

        footprint = sample_mask(self.args.front_distance, self.args.footprint_radius)
        forward = sample_mask(self.args.forward_distance, self.args.footprint_radius)

        def summarize(coords):
            if not coords:
                return {"cells": 0}
            vals = np.asarray([data[rr, cc] for rr, cc in coords], dtype=np.int16)
            return {
                "cells": int(vals.size),
                "unknown": int(np.count_nonzero(vals < 0)),
                "lethal": int(np.count_nonzero(vals >= self.args.lethal_threshold)),
                "high": int(np.count_nonzero(vals >= self.args.high_threshold)),
                "max": int(vals.max()),
                "mean_known": float(vals[vals >= 0].mean()) if np.any(vals >= 0) else None,
            }

        center = None
        if 0 <= row < info.height and 0 <= col < info.width:
            center = int(data[row, col])
        return {
            "frame": msg.header.frame_id,
            "pose": [x, y, yaw],
            "cell": [row, col],
            "center": center,
            "footprint": summarize(footprint),
            "forward": summarize(forward),
        }

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started
        record = {"t": elapsed, "status": self.status}
        if self.cmd_vel:
            record["cmd_vel"] = self.cmd_vel
        if self.cmd_vel_nav:
            record["cmd_vel_nav"] = self.cmd_vel_nav
        if self.nav_feedback:
            record["nav_feedback"] = self.nav_feedback
        if self.selected_goal is not None:
            goal = self.selected_goal.pose.position
            record["selected_goal"] = {
                "frame": self.selected_goal.header.frame_id,
                "x": goal.x,
                "y": goal.y,
            }
        for name, msg in self.maps.items():
            frame = msg.header.frame_id
            pose = self._lookup_pose(frame)
            if pose is None:
                continue
            record[name] = self._window_stats(msg, pose)
            if self.selected_goal is not None and self.selected_goal.header.frame_id == frame:
                goal = self.selected_goal.pose.position
                record[f"{name}_goal_distance"] = math.hypot(goal.x - pose[0], goal.y - pose[1])
        print(json.dumps(record, sort_keys=True), flush=True)
        self.records.append(record)
        if elapsed >= self.args.duration:
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="astrobot_0")
    parser.add_argument("--duration", type=float, default=240.0)
    parser.add_argument("--period", type=float, default=2.0)
    parser.add_argument("--footprint-radius", type=float, default=0.58)
    parser.add_argument("--rear-distance", type=float, default=0.58)
    parser.add_argument("--front-distance", type=float, default=0.58)
    parser.add_argument("--forward-distance", type=float, default=0.90)
    parser.add_argument("--lethal-threshold", type=int, default=100)
    parser.add_argument("--high-threshold", type=int, default=80)
    args = parser.parse_args()

    rclpy.init()
    node = CostmapDiagnostic(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
