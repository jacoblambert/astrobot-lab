#!/usr/bin/env python3
"""Monitor a Phase 4 exploration run and write a compact summary."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Float32, String


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Phase4ExplorationEval(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("phase4_exploration_eval")
        self.args = args
        self.started_at = time.monotonic()
        self.initial_known = None
        self.latest_known = 0
        self.latest_gt_pose = None
        self.latest_slam_pose = None
        self.latest_status = {}
        self.best_sample = 0.0
        self.latest_sample_value = None
        self.resource_samples = []
        self.sample_path = []
        self.selected_goals = []
        self.visited_gt_cells = {}
        self.resource_map_info = None
        self.resource_map_data = None
        self.map_times = []

        self.create_subscription(OccupancyGrid, args.bev_topic, self._bev_cb, 10)
        self.create_subscription(PoseStamped, args.gt_pose_topic, self._gt_pose_cb, 10)
        self.create_subscription(PoseStamped, args.slam_pose_topic, self._slam_pose_cb, 10)
        self.create_subscription(String, args.status_topic, self._status_cb, 10)
        self.create_subscription(Float32, args.resource_sample_topic, self._sample_cb, 10)
        self.create_subscription(PoseStamped, args.resource_sample_pose_topic, self._sample_pose_cb, 10)
        self.create_subscription(PoseStamped, args.selected_goal_topic, self._selected_goal_cb, 10)
        self.create_subscription(OccupancyGrid, args.resource_map_topic, self._resource_map_cb, 10)

    def _bev_cb(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int16)
        known = int(np.count_nonzero(data >= 0))
        if self.initial_known is None and known > 0:
            self.initial_known = known
        self.latest_known = known
        self.map_times.append(time.monotonic())
        self.map_times = self.map_times[-100:]

    def _gt_pose_cb(self, msg: PoseStamped) -> None:
        self.latest_gt_pose = [msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(msg.pose.orientation)]

    def _slam_pose_cb(self, msg: PoseStamped) -> None:
        self.latest_slam_pose = [msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(msg.pose.orientation)]

    def _status_cb(self, msg: String) -> None:
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_status = {"raw": msg.data}

    def _sample_cb(self, msg: Float32) -> None:
        value = float(msg.data)
        self.latest_sample_value = value
        self.best_sample = max(self.best_sample, value)
        self.resource_samples.append([time.monotonic() - self.started_at, value])

    def _sample_pose_cb(self, msg: PoseStamped) -> None:
        value = self.latest_sample_value
        sample = {
            "t": time.monotonic() - self.started_at,
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "yaw": yaw_from_quaternion(msg.pose.orientation),
            "value": value,
        }
        cell = self._resource_cell(sample["x"], sample["y"])
        if cell is not None:
            row, col = cell
            sample["cell"] = [row, col]
            if value is not None:
                key = f"{row},{col}"
                self.visited_gt_cells[key] = {
                    "row": row,
                    "col": col,
                    "x": sample["x"],
                    "y": sample["y"],
                    "best_value": max(value, self.visited_gt_cells.get(key, {}).get("best_value", -math.inf)),
                    "last_t": sample["t"],
                }
        self.sample_path.append(sample)
        self.sample_path = self.sample_path[-5000:]

    def _selected_goal_cb(self, msg: PoseStamped) -> None:
        self.selected_goals.append(
            {
                "t": time.monotonic() - self.started_at,
                "frame_id": msg.header.frame_id,
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
                "yaw": yaw_from_quaternion(msg.pose.orientation),
            }
        )

    def _resource_map_cb(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int16)
        self.resource_map_data = data.reshape((msg.info.height, msg.info.width)).copy()
        max_index = int(np.argmax(data)) if data.size else 0
        max_row = max_index // msg.info.width if msg.info.width else 0
        max_col = max_index % msg.info.width if msg.info.width else 0
        self.resource_map_info = {
            "frame_id": msg.header.frame_id,
            "resolution": float(msg.info.resolution),
            "origin": [float(msg.info.origin.position.x), float(msg.info.origin.position.y)],
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "max_cell": [max_row, max_col],
            "max_xy": [
                float(msg.info.origin.position.x + (max_col + 0.5) * msg.info.resolution),
                float(msg.info.origin.position.y + (max_row + 0.5) * msg.info.resolution),
            ],
        }

    def _resource_cell(self, x: float, y: float) -> tuple[int, int] | None:
        if self.resource_map_info is None:
            return None
        resolution = self.resource_map_info["resolution"]
        origin_x, origin_y = self.resource_map_info["origin"]
        col = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))
        if 0 <= row < self.resource_map_info["height"] and 0 <= col < self.resource_map_info["width"]:
            return row, col
        return None

    def done(self) -> bool:
        if time.monotonic() - self.started_at >= self.args.duration:
            return True
        return self.latest_status.get("state") == "finished"

    def summary(self) -> dict:
        map_rate_hz = 0.0
        if len(self.map_times) >= 2:
            dt = self.map_times[-1] - self.map_times[0]
            if dt > 0.0:
                map_rate_hz = (len(self.map_times) - 1) / dt
        initial_known = self.initial_known or 0
        coverage_gain = 0.0
        if initial_known > 0:
            coverage_gain = (self.latest_known - initial_known) / initial_known
        return {
            "duration_sec": time.monotonic() - self.started_at,
            "initial_known_cells": initial_known,
            "latest_known_cells": self.latest_known,
            "coverage_gain_fraction": coverage_gain,
            "best_sample": self.best_sample,
            "latest_gt_pose": self.latest_gt_pose,
            "latest_slam_pose": self.latest_slam_pose,
            "latest_status": self.latest_status,
            "bev_map_rate_hz": map_rate_hz,
            "resource_sample_count": len(self.resource_samples),
            "resource_samples_tail": self.resource_samples[-20:],
            "resource_map": self.resource_map_info,
            "selected_goals": self.selected_goals,
            "visited_gt_cell_count": len(self.visited_gt_cells),
            "visited_gt_cells_tail": list(self.visited_gt_cells.values())[-20:],
            "top_resource_cells": sorted(
                self.visited_gt_cells.values(), key=lambda item: item["best_value"], reverse=True
            )[:20],
            "sample_path_tail": self.sample_path[-50:],
        }

    def write_resource_compare(self, output: str) -> None:
        if self.resource_map_data is None or self.resource_map_info is None:
            return
        rows = []
        cols = []
        values = []
        for item in self.visited_gt_cells.values():
            rows.append(item["row"])
            cols.append(item["col"])
            values.append(item["best_value"])
        path_x = [item["x"] for item in self.sample_path if item.get("value") is not None]
        path_y = [item["y"] for item in self.sample_path if item.get("value") is not None]
        path_value = [item["value"] for item in self.sample_path if item.get("value") is not None]
        np.savez_compressed(
            output,
            gt_resource_grid=self.resource_map_data.astype(np.int16),
            resolution=np.array([self.resource_map_info["resolution"]], dtype=np.float32),
            origin=np.array(self.resource_map_info["origin"], dtype=np.float32),
            collected_rows=np.asarray(rows, dtype=np.int32),
            collected_cols=np.asarray(cols, dtype=np.int32),
            collected_values=np.asarray(values, dtype=np.float32),
            path_x=np.asarray(path_x, dtype=np.float32),
            path_y=np.asarray(path_y, dtype=np.float32),
            path_values=np.asarray(path_value, dtype=np.float32),
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=900.0)
    parser.add_argument("--bev-topic", default="/astrobot_0/slam/bev_costmap")
    parser.add_argument("--gt-pose-topic", default="/gt/base_link_pose")
    parser.add_argument("--slam-pose-topic", default="/astrobot_0/slam/pose_corrected")
    parser.add_argument("--status-topic", default="/astrobot_0/exploration/status")
    parser.add_argument("--resource-sample-topic", default="/astrobot_0/resource/water/sample")
    parser.add_argument("--resource-sample-pose-topic", default="/astrobot_0/resource/water/sample_pose")
    parser.add_argument("--resource-map-topic", default="/gt/resource_maps/water")
    parser.add_argument("--selected-goal-topic", default="/astrobot_0/exploration/selected_goal")
    parser.add_argument("--output", default="")
    parser.add_argument("--resource-compare-output", default="")
    args = parser.parse_args()

    rclpy.init()
    node = Phase4ExplorationEval(args)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        summary = node.summary()
        text = json.dumps(summary, indent=2, sort_keys=True)
        print(text)
        if args.output:
            Path(args.output).write_text(text + "\n", encoding="utf-8")
        if args.resource_compare_output:
            node.write_resource_compare(args.resource_compare_output)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
