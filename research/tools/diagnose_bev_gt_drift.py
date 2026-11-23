#!/usr/bin/env python3
"""Track SLAM BEV map consistency against simulator GT over time.

The diagnostic locks the initial gt_map -> SLAM map alignment from the first
GT and SLAM pose pair, then compares later BEV costmaps to /gt/map through that
fixed transform. If SLAM/BEV drifts relative to the simulator GT, local rock
IoU and free-space consistency should degrade over time.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform_points(x: np.ndarray, y: np.ndarray, tx: float, ty: float, yaw: float) -> tuple[np.ndarray, np.ndarray]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def inverse_transform_point(x: float, y: float, tx: float, ty: float, yaw: float) -> tuple[float, float]:
    dx = x - tx
    dy = y - ty
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def compose_transform(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    bx, by = transform_points(np.asarray([b[0]]), np.asarray([b[1]]), a[0], a[1], a[2])
    return float(bx[0]), float(by[0]), math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2]))


def invert_transform(t: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y = inverse_transform_point(0.0, 0.0, t[0], t[1], t[2])
    return x, y, -t[2]


class BevGtDriftDiagnostic(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("bev_gt_drift_diagnostic")
        self.args = args
        self.started_at = time.monotonic()
        self.gt_map: OccupancyGrid | None = None
        self.bev_map: OccupancyGrid | None = None
        self.gt_pose: PoseStamped | None = None
        self.slam_pose: PoseStamped | None = None
        self.latest_status: dict = {}
        self.initial_gt_to_slam: tuple[float, float, float] | None = None
        self.last_sample_at = 0.0
        self.csv_file = None
        self.csv_writer = None

        if args.csv:
            Path(args.csv).parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = open(args.csv, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.DictWriter(
                self.csv_file,
                fieldnames=[
                    "t",
                    "pose_error_m",
                    "yaw_error_rad",
                    "eval_cells",
                    "gt_occ_cells",
                    "pred_occ_cells",
                    "iou",
                    "precision",
                    "recall",
                    "false_blocked_free_rate",
                    "missed_obstacle_rate",
                    "pred_known_rate",
                    "mission_coverage_fraction",
                    "mission_reachable_cells",
                    "goals",
                    "successes",
                    "failures",
                    "state",
                ],
            )
            self.csv_writer.writeheader()

        self.create_subscription(OccupancyGrid, args.gt_map_topic, self._gt_map_cb, 2)
        self.create_subscription(OccupancyGrid, args.bev_topic, self._bev_cb, 2)
        self.create_subscription(PoseStamped, args.gt_pose_topic, self._gt_pose_cb, 10)
        self.create_subscription(PoseStamped, args.slam_pose_topic, self._slam_pose_cb, 10)
        self.create_subscription(String, args.status_topic, self._status_cb, 10)
        self.create_timer(args.period_sec, self._sample)

    def _gt_map_cb(self, msg: OccupancyGrid) -> None:
        self.gt_map = msg

    def _bev_cb(self, msg: OccupancyGrid) -> None:
        self.bev_map = msg

    def _gt_pose_cb(self, msg: PoseStamped) -> None:
        self.gt_pose = msg
        self._maybe_initialize_alignment()

    def _slam_pose_cb(self, msg: PoseStamped) -> None:
        self.slam_pose = msg
        self._maybe_initialize_alignment()

    def _status_cb(self, msg: String) -> None:
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_status = {"state": "invalid_status_json"}

    def _maybe_initialize_alignment(self) -> None:
        if self.initial_gt_to_slam is not None or self.gt_pose is None or self.slam_pose is None:
            return
        gt = self._pose_tuple(self.gt_pose)
        slam = self._pose_tuple(self.slam_pose)
        if self.args.alignment_yaw_mode == "pose":
            yaw = math.atan2(math.sin(slam[2] - gt[2]), math.cos(slam[2] - gt[2]))
        else:
            yaw = self.args.alignment_yaw
        sx, sy = transform_points(np.asarray([gt[0]]), np.asarray([gt[1]]), 0.0, 0.0, yaw)
        self.initial_gt_to_slam = (slam[0] - float(sx[0]), slam[1] - float(sy[0]), yaw)
        self.get_logger().info(
            "locked initial gt_map->slam_map alignment "
            f"x={self.initial_gt_to_slam[0]:.3f} y={self.initial_gt_to_slam[1]:.3f} yaw={self.initial_gt_to_slam[2]:.3f}"
        )

    @staticmethod
    def _pose_tuple(msg: PoseStamped) -> tuple[float, float, float]:
        return float(msg.pose.position.x), float(msg.pose.position.y), yaw_from_quaternion(msg.pose.orientation)

    def _sample(self) -> None:
        if (
            self.gt_map is None
            or self.bev_map is None
            or self.gt_pose is None
            or self.slam_pose is None
            or self.initial_gt_to_slam is None
        ):
            return

        result = self._compare_window()
        if result is None:
            return

        status = self.latest_status
        result.update(
            {
                "mission_coverage_fraction": float(status.get("mission_coverage_fraction", 0.0)),
                "mission_reachable_cells": int(status.get("mission_reachable_cells", 0)),
                "goals": int(status.get("goals", 0)),
                "successes": int(status.get("successes", 0)),
                "failures": int(status.get("failures", 0)),
                "state": str(status.get("state", "")),
            }
        )

        if self.csv_writer is not None:
            self.csv_writer.writerow(result)
            self.csv_file.flush()

        if self.args.jsonl:
            with open(self.args.jsonl, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(result, sort_keys=True) + "\n")

        self.get_logger().info(
            "bev_gt "
            f"t={result['t']:.1f}s pose_err={result['pose_error_m']:.2f}m "
            f"iou={result['iou']:.3f} precision={result['precision']:.3f} recall={result['recall']:.3f} "
            f"false_blocked={result['false_blocked_free_rate']:.3f} missed={result['missed_obstacle_rate']:.3f} "
            f"known={result['pred_known_rate']:.3f} goals={result['goals']}"
        )

    def _compare_window(self) -> dict | None:
        assert self.gt_map is not None
        assert self.bev_map is not None
        assert self.gt_pose is not None
        assert self.slam_pose is not None
        assert self.initial_gt_to_slam is not None

        gt_info = self.gt_map.info
        bev_info = self.bev_map.info
        gt_data = np.asarray(self.gt_map.data, dtype=np.int16).reshape((gt_info.height, gt_info.width))
        bev_data = np.asarray(self.bev_map.data, dtype=np.int16).reshape((bev_info.height, bev_info.width))

        gt_x, gt_y, gt_yaw = self._pose_tuple(self.gt_pose)
        slam_x, slam_y, slam_yaw = self._pose_tuple(self.slam_pose)
        pred_slam_x, pred_slam_y, pred_slam_yaw = compose_transform(self.initial_gt_to_slam, (gt_x, gt_y, gt_yaw))
        pose_error = math.hypot(pred_slam_x - slam_x, pred_slam_y - slam_y)
        yaw_error = math.atan2(math.sin(pred_slam_yaw - slam_yaw), math.cos(pred_slam_yaw - slam_yaw))

        radius = self.args.window_radius_m
        gt_res = gt_info.resolution
        center_col = int(math.floor((gt_x - gt_info.origin.position.x) / gt_res))
        center_row = int(math.floor((gt_y - gt_info.origin.position.y) / gt_res))
        r_cells = int(math.ceil(radius / gt_res))
        r0 = max(0, center_row - r_cells)
        r1 = min(gt_info.height, center_row + r_cells + 1)
        c0 = max(0, center_col - r_cells)
        c1 = min(gt_info.width, center_col + r_cells + 1)
        if r0 >= r1 or c0 >= c1:
            return None

        rows = np.arange(r0, r1, dtype=np.float64)
        cols = np.arange(c0, c1, dtype=np.float64)
        gt_xs = gt_info.origin.position.x + (cols + 0.5) * gt_res
        gt_ys = gt_info.origin.position.y + (rows + 0.5) * gt_res
        x_grid, y_grid = np.meshgrid(gt_xs, gt_ys)
        dist_mask = (x_grid - gt_x) ** 2 + (y_grid - gt_y) ** 2 <= radius**2

        slam_x_grid, slam_y_grid = transform_points(
            x_grid,
            y_grid,
            self.initial_gt_to_slam[0],
            self.initial_gt_to_slam[1],
            self.initial_gt_to_slam[2],
        )
        bev_col = np.floor((slam_x_grid - bev_info.origin.position.x) / bev_info.resolution).astype(np.int64)
        bev_row = np.floor((slam_y_grid - bev_info.origin.position.y) / bev_info.resolution).astype(np.int64)
        inside = (
            (bev_row >= 0)
            & (bev_row < bev_info.height)
            & (bev_col >= 0)
            & (bev_col < bev_info.width)
        )
        sampled = np.full((r1 - r0, c1 - c0), -1, dtype=np.int16)
        sampled[inside] = bev_data[bev_row[inside], bev_col[inside]]

        gt_window = gt_data[r0:r1, c0:c1]
        eval_mask = dist_mask & inside & (gt_window >= 0)
        known_eval = eval_mask & (sampled >= 0)
        if not np.any(eval_mask):
            return None

        gt_occ = eval_mask & (gt_window >= self.args.gt_occupied_threshold)
        gt_free = eval_mask & (gt_window >= 0) & (gt_window < self.args.gt_occupied_threshold)
        pred_known = eval_mask & (sampled >= 0)
        pred_occ = pred_known & (sampled >= self.args.pred_occupied_threshold)

        gt_occ_d = self._dilate(gt_occ, self.args.dilation_cells)
        pred_occ_d = self._dilate(pred_occ, self.args.dilation_cells)
        tp = int(np.count_nonzero(pred_occ & gt_occ_d))
        fp = int(np.count_nonzero(pred_occ & ~gt_occ_d))
        fn = int(np.count_nonzero(gt_occ & ~pred_occ_d))
        union = int(np.count_nonzero(eval_mask & (gt_occ_d | pred_occ_d)))
        intersection = int(np.count_nonzero(eval_mask & gt_occ_d & pred_occ_d))

        false_blocked = int(np.count_nonzero(gt_free & pred_occ))
        missed = int(np.count_nonzero(gt_occ & ~pred_occ_d))
        gt_free_count = int(np.count_nonzero(gt_free))
        gt_occ_count = int(np.count_nonzero(gt_occ))

        return {
            "t": time.monotonic() - self.started_at,
            "pose_error_m": pose_error,
            "yaw_error_rad": yaw_error,
            "eval_cells": int(np.count_nonzero(eval_mask)),
            "gt_occ_cells": gt_occ_count,
            "pred_occ_cells": int(np.count_nonzero(pred_occ)),
            "iou": intersection / union if union else 0.0,
            "precision": tp / (tp + fp) if tp + fp else 0.0,
            "recall": tp / (tp + fn) if tp + fn else 0.0,
            "false_blocked_free_rate": false_blocked / gt_free_count if gt_free_count else 0.0,
            "missed_obstacle_rate": missed / gt_occ_count if gt_occ_count else 0.0,
            "pred_known_rate": int(np.count_nonzero(pred_known)) / int(np.count_nonzero(eval_mask)),
        }

    @staticmethod
    def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
        if radius <= 0:
            return mask
        result = np.zeros_like(mask, dtype=bool)
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if dr * dr + dc * dc > radius * radius:
                    continue
                src_r0 = max(0, -dr)
                src_r1 = min(mask.shape[0], mask.shape[0] - dr)
                src_c0 = max(0, -dc)
                src_c1 = min(mask.shape[1], mask.shape[1] - dc)
                dst_r0 = max(0, dr)
                dst_r1 = min(mask.shape[0], mask.shape[0] + dr)
                dst_c0 = max(0, dc)
                dst_c1 = min(mask.shape[1], mask.shape[1] + dc)
                result[dst_r0:dst_r1, dst_c0:dst_c1] |= mask[src_r0:src_r1, src_c0:src_c1]
        return result

    def destroy_node(self) -> bool:
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gt-map-topic", default="/gt/map")
    parser.add_argument("--bev-topic", default="/astrobot_0/slam/bev_costmap")
    parser.add_argument("--gt-pose-topic", default="/gt/base_link_pose")
    parser.add_argument("--slam-pose-topic", default="/astrobot_0/slam/pose_corrected")
    parser.add_argument("--status-topic", default="/astrobot_0/exploration/status")
    parser.add_argument("--window-radius-m", type=float, default=8.0)
    parser.add_argument("--period-sec", type=float, default=5.0)
    parser.add_argument("--gt-occupied-threshold", type=int, default=50)
    parser.add_argument("--pred-occupied-threshold", type=int, default=50)
    parser.add_argument("--dilation-cells", type=int, default=2)
    parser.add_argument("--alignment-yaw-mode", choices=["zero", "pose", "manual"], default="zero")
    parser.add_argument("--alignment-yaw", type=float, default=0.0)
    parser.add_argument("--csv", default="")
    parser.add_argument("--jsonl", default="")
    args = parser.parse_args()

    rclpy.init()
    node = BevGtDriftDiagnostic(args)
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
