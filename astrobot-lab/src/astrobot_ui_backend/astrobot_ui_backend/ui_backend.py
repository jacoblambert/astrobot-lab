#!/usr/bin/env python3
"""Read-only ROS 2 to FastAPI bridge for the Astrobot console."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np
import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from pydantic import BaseModel
from sensor_msgs.msg import Imu, PointCloud2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def yaw_from_quaternion(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class TeleopGoalRequest(BaseModel):
    x: float
    y: float
    yaw: float | None = None
    frame_id: str = "map"


def pose_msg_to_dict(msg: PoseStamped) -> dict[str, float]:
    return {
        "x": float(msg.pose.position.x),
        "y": float(msg.pose.position.y),
        "yaw": yaw_from_quaternion(msg.pose.orientation),
        "stamp": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
    }


# GLIM's slam/pose_corrected is the IMU sensor pose: offset and pi-rotated from
# base_link (must match slam_map_to_odom_tf_node / tools/map_gt_alignment.py).
# Converting at ingestion keeps the displayed rover pose, trajectory, home pose
# and the GT alignment fit all consistently in base_link/map coordinates.
IMU_BASE_X = 0.264
IMU_BASE_Y = 0.017
IMU_BASE_YAW = math.pi


def slam_pose_to_base(pose: dict[str, float]) -> dict[str, float]:
    """Convert a SLAM (IMU-frame) pose dict to the base_link pose in map coords."""
    yaw = float(pose.get("yaw", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    out = dict(pose)
    out["x"] = float(pose["x"]) + c * IMU_BASE_X - s * IMU_BASE_Y
    out["y"] = float(pose["y"]) + s * IMU_BASE_X + c * IMU_BASE_Y
    out["yaw"] = normalize_angle(yaw + IMU_BASE_YAW)
    return out


def path_msg_to_list(msg: Path, limit: int = 600) -> list[dict[str, float]]:
    poses = msg.poses
    if len(poses) > limit:
        stride = max(1, int(math.ceil(len(poses) / limit)))
        poses = poses[::stride]
    return [pose_msg_to_dict(pose) for pose in poses]


def occupancy_grid_to_dict(
    msg: OccupancyGrid,
    max_cells: int = 200_000,
    gt_to_map_transform: tuple[float, float, float] | None = None,
) -> dict[str, Any]:
    width = int(msg.info.width)
    height = int(msg.info.height)
    stride = max(1, int(math.ceil(math.sqrt(max(1, width * height) / max_cells))))
    resolution = float(msg.info.resolution) * stride
    source_frame = msg.header.frame_id
    display_frame = source_frame
    origin_x = float(msg.info.origin.position.x)
    origin_y = float(msg.info.origin.position.y)
    origin_yaw = yaw_from_quaternion(msg.info.origin.orientation)
    if source_frame == "gt_map":
        # OmniLRS GT/resource maps use positive yard coordinates. For UI overlay,
        # display them in the robot-local SLAM/Nav2 map frame anchored at start.
        display_frame = "map"
        if gt_to_map_transform is not None:
            offset_x, offset_y, offset_yaw = gt_to_map_transform
            cos_yaw = math.cos(offset_yaw)
            sin_yaw = math.sin(offset_yaw)
            transformed_x = offset_x + cos_yaw * origin_x - sin_yaw * origin_y
            transformed_y = offset_y + sin_yaw * origin_x + cos_yaw * origin_y
            origin_x = transformed_x
            origin_y = transformed_y
            origin_yaw = normalize_angle(origin_yaw + offset_yaw)
        else:
            origin_x -= 0.5 * float(width) * float(msg.info.resolution)
            origin_y -= 0.5 * float(height) * float(msg.info.resolution)
    if stride > 1:
        display_width = int(math.ceil(width / stride))
        display_height = int(math.ceil(height / stride))
        data = [
            int(msg.data[min(row * stride, height - 1) * width + min(col * stride, width - 1)])
            for row in range(display_height)
            for col in range(display_width)
        ]
    else:
        display_width = width
        display_height = height
        data = [int(value) for value in msg.data]

    # OmniLRS publishes GT/resource maps with yaw≈±π (axes flipped vs standard).
    # Rotate 180°: flip data and move origin to the corner that becomes the new (0,0).
    if source_frame == "gt_map" and abs(abs(origin_yaw) - math.pi) < 0.15:
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)
        dw, dh = display_width, display_height
        # Corner (dw-1, dh-1) in the flipped grid becomes the new origin
        new_origin_x = origin_x + c * (dw - 1) * resolution - s * (dh - 1) * resolution
        new_origin_y = origin_y + s * (dw - 1) * resolution + c * (dh - 1) * resolution
        origin_x = new_origin_x
        origin_y = new_origin_y
        origin_yaw = normalize_angle(origin_yaw + math.pi)  # → ≈0
        data = [data[(dh - 1 - row) * dw + (dw - 1 - col)] for row in range(dh) for col in range(dw)]

    return {
        "frame": display_frame,
        "sourceFrame": source_frame,
        "resolution": resolution,
        "width": display_width,
        "height": display_height,
        "sourceWidth": width,
        "sourceHeight": height,
        "downsample": stride,
        "origin": {
            "x": origin_x,
            "y": origin_y,
            "yaw": origin_yaw,
        },
        "data": data,
        "stamp": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
    }


def image_msg_to_bgr(msg: Image) -> np.ndarray | None:
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
        "8uc1": 1,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None or msg.height == 0 or msg.width == 0:
        return None
    dtype = np.uint8
    row_bytes = int(msg.step)
    expected_row_bytes = int(msg.width) * channels
    raw = np.frombuffer(msg.data, dtype=dtype)
    if raw.size < row_bytes * int(msg.height):
        return None
    rows = raw[: row_bytes * int(msg.height)].reshape((int(msg.height), row_bytes))
    pixels = rows[:, :expected_row_bytes].reshape((int(msg.height), int(msg.width), channels))
    if encoding == "rgb8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return pixels.copy()
    if encoding == "rgba8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)


def severity_from_diagnostics(diagnostics: dict[str, Any], mission: dict[str, Any]) -> tuple[str, str]:
    classification = diagnostics.get("classification", {}) if isinstance(diagnostics, dict) else {}
    primary = str(classification.get("primary", "no_active_fault_detected"))
    confidence = str(classification.get("confidence", "low"))
    state = str(mission.get("state", "waiting_for_inputs"))

    critical_reasons = {
        "slam_pose_error_jump",
        "slam_pose_error_drifting",
        "slam_gt_increment_mismatch",
        "gt_motion_stalled_while_commanded",
    }
    warning_reasons = {
        "planned_path_intersects_gt_obstacle_corridor",
        "planned_path_intersects_bev_high_cost",
        "planned_path_intersects_nav2_global_high_cost",
        "selected_goal_near_gt_obstacle",
        "nav2_feedback_no_distance_progress",
        "near_goal_progress_checker_or_goal_tolerance_issue",
    }

    if primary in critical_reasons and confidence in {"medium", "high"}:
        return "critical", primary
    if primary in warning_reasons or int(mission.get("failures", 0) or 0) > 0:
        return "warning", primary
    if state == "finished":
        return "nominal", "mission finished"
    if mission.get("returning_home"):
        return "nominal", "returning home"
    if mission.get("active_goal"):
        return "nominal", f"navigating: {mission.get('active_goal_kind', 'goal')}"
    return "nominal", state.replace("_", " ")


@dataclass
class TopicFreshness:
    topic: str
    first_stamp: float = 0.0
    stamp: float = 0.0
    count: int = 0


@dataclass
class BridgeState:
    maps: dict[str, dict[str, Any]] = field(default_factory=dict)
    poses: dict[str, dict[str, float]] = field(default_factory=dict)
    trajectories: dict[str, list[dict[str, float]]] = field(default_factory=dict)
    plan: list[dict[str, float]] = field(default_factory=list)
    goal: dict[str, float] | None = None
    diagnostics: dict[str, Any] = field(default_factory=dict)
    mission: dict[str, Any] = field(default_factory=dict)
    camera_images: dict[str, Image] = field(default_factory=dict)
    topic_freshness: dict[str, TopicFreshness] = field(default_factory=dict)
    series: dict[str, list[float]] = field(default_factory=dict)
    started_at: float = field(default_factory=time.monotonic)


class UiBridgeNode(Node):
    def __init__(self, robot_namespace: str, trajectory_limit: int = 1500) -> None:
        super().__init__("ui_backend")
        self.robot_namespace = robot_namespace.strip("/")
        self.trajectory_limit = trajectory_limit
        self._lock = threading.RLock()
        self._state = BridgeState()
        self._last_grid_conversion: dict[str, float] = {}
        self._latest_grid_msgs: dict[str, OccupancyGrid] = {}
        self._grid_conversion_min_period_sec = 0.5
        self._gt_home_pose: dict[str, float] | None = None
        self._slam_home_pose: dict[str, float] | None = None
        self._gt_to_map_transform: tuple[float, float, float] | None = None
        # Windowed least-squares gt->map alignment (positions only -- yaw diffs from
        # single quaternions are fragile; the IMU-frame pi rotation poisoned them).
        self._align_pairs: list[tuple[float, float, float, float]] = []  # (gt_x, gt_y, map_x, map_y)
        self._align_fitted = False
        self._last_gt_pose: dict[str, float] | None = None

        self._subscribe_grid("obstacle", "slam/bev_costmap")
        self._subscribe_grid("mission", "exploration/mission_grid")
        self._subscribe_grid("water", "/gt/resource_maps/water")
        self._subscribe_grid("gt", "/gt/map")
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            PointCloud2,
            "lidar_0/pointcloud/filtered",
            self._freshness_cb("lidar_0/pointcloud/filtered"),
            sensor_qos,
        )
        self.create_subscription(Imu, "imu", self._freshness_cb("imu"), sensor_qos)
        self.create_subscription(PointCloud2, "slam/prob_voxel_map", self._freshness_cb("slam/prob_voxel_map"), sensor_qos)
        self.create_subscription(PoseStamped, "slam/pose_corrected", self._pose_cb, 20)
        self.create_subscription(PoseStamped, "/gt/base_link_pose", self._gt_pose_cb, 10)
        self.create_subscription(PoseStamped, "exploration/selected_goal", self._goal_cb, 10)
        self.create_subscription(Path, "plan", self._plan_cb, 5)
        self.create_subscription(String, "diagnostics", self._diagnostics_cb, 10)
        self.create_subscription(String, "exploration/status", self._mission_cb, 10)
        self.create_subscription(Image, "camera_0/image_raw", self._camera_cb("camera_0/image_raw"), sensor_qos)
        self.create_subscription(Image, "/front_camera/mono/rgb", self._camera_cb("camera_0/image_raw"), sensor_qos)
        self.teleop_goal_pub = self.create_publisher(PoseStamped, "teleop/goal", 10)

    def _touch(self, topic: str) -> None:
        freshness = self._state.topic_freshness.setdefault(topic, TopicFreshness(topic=topic))
        now = time.time()
        if freshness.first_stamp <= 0.0:
            freshness.first_stamp = now
        freshness.stamp = now
        freshness.count += 1
        hz = self._topic_hz(freshness)
        series_name = self._topic_series_name(topic)
        if series_name:
            self._append_series(series_name, hz)
        if topic == "slam/bev_costmap":
            self._append_series("Map rate", hz)
        elif topic == "diagnostics":
            self._append_series("Diagnostics Hz", hz)
        elif topic.startswith("camera_"):
            self._append_series("Camera Hz", hz)

    @staticmethod
    def _topic_hz(freshness: TopicFreshness) -> float:
        elapsed = max(1e-6, freshness.stamp - freshness.first_stamp)
        return float(freshness.count / elapsed)

    def _subscribe_grid(self, layer_name: str, topic: str) -> None:
        self.create_subscription(OccupancyGrid, topic, lambda msg, key=layer_name, source=topic: self._grid_cb(key, source, msg), 2)

    @staticmethod
    def _topic_series_name(topic: str) -> str | None:
        return {
            "lidar_0/pointcloud/filtered": "PointCloud Hz",
            "imu": "IMU Hz",
            "slam/pose_corrected": "SLAM pose Hz",
            "slam/prob_voxel_map": "Voxel map Hz",
            "slam/bev_costmap": "BEV map Hz",
        }.get(topic)

    def _freshness_cb(self, topic: str):
        def callback(_: Any) -> None:
            with self._lock:
                self._touch(topic)

        return callback

    def _grid_cb(self, layer_name: str, topic: str, msg: OccupancyGrid) -> None:
        now = time.monotonic()
        with self._lock:
            self._latest_grid_msgs[layer_name] = msg
            last = self._last_grid_conversion.get(layer_name, 0.0)
            if now - last < self._grid_conversion_min_period_sec:
                self._touch(topic)
                return
            self._last_grid_conversion[layer_name] = now
            self._state.maps[layer_name] = occupancy_grid_to_dict(
                msg, gt_to_map_transform=self._gt_to_map_transform
            )
            self._touch(topic)

    def _pose_cb(self, msg: PoseStamped) -> None:
        # Convert IMU-frame SLAM pose to base_link once, at ingestion.
        pose = slam_pose_to_base(pose_msg_to_dict(msg))
        with self._lock:
            if self._slam_home_pose is None:
                self._slam_home_pose = pose
                self._update_gt_to_map_offset()
            # Collect matched (gt, map/base) position pairs for the LS alignment fit.
            if not self._align_fitted and self._last_gt_pose is not None:
                if abs(float(self._last_gt_pose["stamp"]) - float(pose["stamp"])) <= 0.15:
                    self._align_pairs.append(
                        (float(self._last_gt_pose["x"]), float(self._last_gt_pose["y"]),
                         float(pose["x"]), float(pose["y"]))
                    )
                    if len(self._align_pairs) > 600:
                        del self._align_pairs[: len(self._align_pairs) - 600]
                    # Yaw is only observable once the trajectory has spatial spread
                    # (pairs collected while parked at spawn are degenerate).
                    if len(self._align_pairs) >= 60:
                        xs = [p[2] for p in self._align_pairs]
                        ys = [p[3] for p in self._align_pairs]
                        if (max(xs) - min(xs)) + (max(ys) - min(ys)) >= 1.5:
                            self._fit_gt_to_map()
            self._state.poses[self.robot_namespace] = pose
            trajectory = self._state.trajectories.setdefault(self.robot_namespace, [])
            trajectory.append(pose)
            if len(trajectory) > self.trajectory_limit:
                del trajectory[: len(trajectory) - self.trajectory_limit]
            self._touch("slam/pose_corrected")

    def _gt_pose_cb(self, msg: PoseStamped) -> None:
        pose = pose_msg_to_dict(msg)
        with self._lock:
            self._last_gt_pose = pose
            if self._gt_home_pose is None:
                self._gt_home_pose = pose
                self._update_gt_to_map_offset()
            self._touch("/gt/base_link_pose")

    def _update_gt_to_map_offset(self) -> None:
        """Initial gt->map transform: translation-only from the home poses.

        Empirically map->world is near-pure translation (LS fits across runs:
        yaw < 1 deg, rms < 5 cm), so this is already close; the windowed LS fit
        in _fit_gt_to_map() refines yaw+translation once enough pairs exist.
        Never derive yaw from single-pose quaternion differences (the IMU-frame
        pi rotation made that produce a bogus ~180 deg yaw before).
        """
        if self._gt_home_pose is None or self._align_fitted:
            return
        slam_home = self._slam_home_pose or {"x": 0.0, "y": 0.0}
        self._gt_to_map_transform = (
            float(slam_home["x"]) - float(self._gt_home_pose["x"]),
            float(slam_home["y"]) - float(self._gt_home_pose["y"]),
            0.0,
        )
        self._rerender_gt_layers()

    def _fit_gt_to_map(self) -> None:
        """Least-squares planar yaw+translation (gt -> map) over collected pairs.

        Same math as slam_eval/compare_tum_trajectories.align_yaw_translation /
        tools/map_gt_alignment.py (reference = map/base positions, estimate = gt).
        """
        pairs = self._align_pairs
        n = len(pairs)
        if n < 10:
            return
        gt_cx = sum(p[0] for p in pairs) / n
        gt_cy = sum(p[1] for p in pairs) / n
        mp_cx = sum(p[2] for p in pairs) / n
        mp_cy = sum(p[3] for p in pairs) / n
        sxx = sxy = 0.0
        for gx, gy, mx, my in pairs:
            ex, ey = gx - gt_cx, gy - gt_cy
            rx, ry = mx - mp_cx, my - mp_cy
            sxx += ex * rx + ey * ry
            sxy += ex * ry - ey * rx
        yaw = math.atan2(sxy, sxx)
        c, s = math.cos(yaw), math.sin(yaw)
        self._gt_to_map_transform = (
            mp_cx - (c * gt_cx - s * gt_cy),
            mp_cy - (s * gt_cx + c * gt_cy),
            yaw,
        )
        self._align_fitted = True
        self._align_pairs = []
        self._rerender_gt_layers()

    def _rerender_gt_layers(self) -> None:
        for layer_name, msg in self._latest_grid_msgs.items():
            if msg.header.frame_id == "gt_map":
                self._state.maps[layer_name] = occupancy_grid_to_dict(
                    msg, gt_to_map_transform=self._gt_to_map_transform
                )

    def _goal_cb(self, msg: PoseStamped) -> None:
        with self._lock:
            self._state.goal = pose_msg_to_dict(msg)
            self._touch("exploration/selected_goal")

    def _plan_cb(self, msg: Path) -> None:
        with self._lock:
            self._state.plan = path_msg_to_list(msg)
            self._touch("plan")

    def _diagnostics_cb(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            parsed = {"raw": msg.data}
        with self._lock:
            self._state.diagnostics = parsed
            nav = parsed.get("nav", {}) if isinstance(parsed, dict) else {}
            feedback = nav.get("feedback", {}) if isinstance(nav, dict) else {}
            motion = nav.get("motion", {}) if isinstance(nav, dict) else {}
            self._append_series("SLAM error", motion.get("position_error_m"))
            self._append_series("Goal distance", feedback.get("distance_remaining_m"))
            self._touch("diagnostics")

    def _mission_cb(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            parsed = {"state": msg.data}
        with self._lock:
            self._state.mission = parsed
            self._append_series("Coverage", parsed.get("mission_coverage_fraction"))
            self._append_series("Resource", parsed.get("best_sample"))
            self._touch("exploration/status")

    def _camera_cb(self, topic: str):
        def callback(msg: Image) -> None:
            with self._lock:
                self._state.camera_images[topic] = msg
                self._touch(topic)

        return callback

    def _append_series(self, name: str, value: Any, limit: int = 180) -> None:
        if value is None:
            return
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return
        values = self._state.series.setdefault(name, [])
        values.append(numeric)
        if len(values) > limit:
            del values[: len(values) - limit]

    def snapshot(self, include_maps: bool = True) -> dict[str, Any]:
        with self._lock:
            mission_raw = dict(self._state.mission)
            diagnostics = dict(self._state.diagnostics)
            severity, status_text = severity_from_diagnostics(diagnostics, mission_raw)
            events = self._events(severity, status_text, diagnostics, mission_raw)
            robot = {
                "name": self.robot_namespace,
                "severity": severity,
                "statusText": status_text,
                "pose": self._state.poses.get(self.robot_namespace),
                "goal": self._state.goal,
                "trajectory": list(self._state.trajectories.get(self.robot_namespace, [])),
                "plan": list(self._state.plan),
                "diagnostics": diagnostics.get("classification", diagnostics),
            }
            mission = {
                "mode": str(mission_raw.get("mission_mode", "unknown")),
                "state": str(mission_raw.get("state", "waiting_for_inputs")),
                "elapsedSec": float(time.monotonic() - self._state.started_at),
                "coverageFraction": float(mission_raw.get("mission_coverage_fraction", 0.0) or 0.0),
                "bestSample": float(mission_raw.get("best_sample", 0.0) or 0.0),
                "goals": int(mission_raw.get("goals", 0) or 0),
                "successes": int(mission_raw.get("successes", 0) or 0),
                "failures": int(mission_raw.get("failures", 0) or 0),
                "returningHome": bool(mission_raw.get("returning_home", False)) and mission_raw.get("state") != "finished",
                "details": mission_raw,
            }
            camera_topics = [
                f"/{self.robot_namespace}/{topic.topic}"
                for topic in self._state.topic_freshness.values()
                if topic.topic.startswith("camera_")
            ]
            return {
                "schemaVersion": "astrobot.console.v1",
                "stamp": time.time(),
                "connected": True,
                "robots": [robot],
                "mission": mission,
                "maps": dict(self._state.maps) if include_maps else {},
                "cameraTopics": sorted(camera_topics),
                "series": {key: list(value) for key, value in self._state.series.items()},
                "topics": {
                    key: {"stamp": value.stamp, "count": value.count, "hz": self._topic_hz(value)}
                    for key, value in self._state.topic_freshness.items()
                },
                "events": events,
            }

    def map_layer(self, layer: str) -> dict[str, Any] | None:
        with self._lock:
            value = self._state.maps.get(layer)
            return dict(value) if value is not None else None

    def map_metadata(self) -> dict[str, Any]:
        with self._lock:
            return {
                name: {
                    "frame": grid.get("frame", ""),
                    "width": grid.get("width", 0),
                    "height": grid.get("height", 0),
                    "resolution": grid.get("resolution", 0.0),
                    "stamp": grid.get("stamp", 0.0),
                    "cells": int(grid.get("width", 0)) * int(grid.get("height", 0)),
                }
                for name, grid in self._state.maps.items()
            }

    def camera_metadata(self) -> list[dict[str, Any]]:
        with self._lock:
            output = []
            for topic, msg in self._state.camera_images.items():
                freshness = self._state.topic_freshness.get(topic)
                output.append(
                    {
                        "key": topic.split("/", 1)[0],
                        "topic": f"/{self.robot_namespace}/{topic}",
                        "width": int(msg.width),
                        "height": int(msg.height),
                        "encoding": msg.encoding,
                        "stamp": freshness.stamp if freshness else 0.0,
                        "hz": self._topic_hz(freshness) if freshness else 0.0,
                    }
                )
            return sorted(output, key=lambda item: item["topic"])

    def camera_jpeg(self, camera_key: str, max_width: int = 640, quality: int = 72) -> bytes | None:
        with self._lock:
            topic = camera_key
            if topic.startswith(f"{self.robot_namespace}/"):
                topic = topic[len(self.robot_namespace) + 1 :]
            if topic.startswith(f"/{self.robot_namespace}/"):
                topic = topic[len(self.robot_namespace) + 2 :]
            if topic not in self._state.camera_images:
                topic = f"{camera_key}/image_raw"
            msg = self._state.camera_images.get(topic)
            if msg is None:
                return None
            image = image_msg_to_bgr(msg)
        if image is None:
            return None
        if max_width > 0 and image.shape[1] > max_width:
            scale = max_width / float(image.shape[1])
            image = cv2.resize(image, (max_width, max(1, int(round(image.shape[0] * scale)))), interpolation=cv2.INTER_AREA)
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(max(1, min(95, quality)))]
        ok, encoded = cv2.imencode(".jpg", image, encode_params)
        return encoded.tobytes() if ok else None

    def publish_teleop_goal(self, request: TeleopGoalRequest) -> dict[str, Any]:
        frame_id = request.frame_id.strip() or "map"
        if frame_id != "map":
            raise ValueError("teleop goals must use frame_id='map'")
        yaw = float(request.yaw) if request.yaw is not None else 0.0
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(request.x)
        msg.pose.position.y = float(request.y)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.teleop_goal_pub.publish(msg)
        self._touch("teleop/goal")
        return {
            "ok": True,
            "robot": self.robot_namespace,
            "topic": f"/{self.robot_namespace}/teleop/goal",
            "goal": {"x": float(request.x), "y": float(request.y), "yaw": yaw, "frame_id": frame_id},
            "stamp": time.time(),
        }

    def _events(
        self,
        severity: str,
        status_text: str,
        diagnostics: dict[str, Any],
        mission_raw: dict[str, Any],
    ) -> list[dict[str, Any]]:
        now = time.time()
        events: list[dict[str, Any]] = []
        classification = diagnostics.get("classification", {}) if isinstance(diagnostics, dict) else {}
        primary = str(classification.get("primary", "no_active_fault_detected"))
        if primary != "no_active_fault_detected":
            events.append({"stamp": now, "severity": severity, "text": primary, "source": "diagnostics"})
        if mission_raw.get("returning_home"):
            events.append({"stamp": now, "severity": "nominal", "text": "returning home", "source": "mission"})
        if bool(mission_raw.get("pending_goal_check", False)):
            events.append({"stamp": now, "severity": "warning", "text": "planner precheck pending", "source": "exploration"})
        if int(mission_raw.get("precheck_rejections", 0) or 0) > 0:
            events.append(
                {
                    "stamp": now,
                    "severity": "warning",
                    "text": f"precheck rejections: {mission_raw.get('precheck_rejections')}",
                    "source": "exploration",
                }
            )
        if not events and status_text:
            events.append({"stamp": now, "severity": severity, "text": status_text, "source": "robot"})
        return events[:4]


def create_app(node: UiBridgeNode) -> FastAPI:
    app = FastAPI(title="Astrobot UI Backend", version="0.1.0")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.get("/api/health")
    def health() -> dict[str, Any]:
        return {"ok": True, "robot": node.robot_namespace, "stamp": time.time()}

    @app.get("/api/state")
    def state(include_maps: bool = False) -> dict[str, Any]:
        return node.snapshot(include_maps=include_maps)

    @app.get("/api/maps")
    def maps() -> dict[str, Any]:
        return node.snapshot()["maps"]

    @app.get("/api/maps/metadata")
    def map_metadata() -> dict[str, Any]:
        return node.map_metadata()

    @app.get("/api/maps/{layer}")
    def map_layer(layer: str) -> dict[str, Any]:
        value = node.map_layer(layer)
        if value is None:
            raise HTTPException(status_code=404, detail=f"unknown map layer: {layer}")
        return value

    @app.get("/api/robots")
    def robots() -> list[dict[str, Any]]:
        return node.snapshot()["robots"]

    @app.post("/api/robots/{robot}/teleop_goal")
    def teleop_goal(robot: str, request: TeleopGoalRequest) -> dict[str, Any]:
        if robot.strip("/") != node.robot_namespace:
            raise HTTPException(status_code=404, detail=f"unknown robot: {robot}")
        try:
            return node.publish_teleop_goal(request)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.get("/api/cameras")
    def cameras() -> list[dict[str, Any]]:
        return node.camera_metadata()

    @app.get("/api/cameras/{camera_key}/snapshot.jpg")
    def camera_snapshot(camera_key: str, max_width: int = 640, quality: int = 72) -> Response:
        jpeg = node.camera_jpeg(camera_key, max_width=max_width, quality=quality)
        if jpeg is None:
            raise HTTPException(status_code=404, detail=f"no JPEG snapshot available for camera: {camera_key}")
        return Response(content=jpeg, media_type="image/jpeg")

    @app.websocket("/ws/telemetry")
    async def telemetry(websocket: WebSocket) -> None:
        await websocket.accept()
        last_map_send = 0.0
        try:
            while True:
                now = time.monotonic()
                include_maps = now - last_map_send > 2.0
                if include_maps:
                    last_map_send = now
                await websocket.send_json(node.snapshot(include_maps=include_maps))
                await asyncio.sleep(0.5)
        except WebSocketDisconnect:
            return

    return app


def spin_node(node: UiBridgeNode) -> None:
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--robot-namespace", default="astrobot_0")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = UiBridgeNode(robot_namespace=args.robot_namespace)
    thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    thread.start()
    try:
        uvicorn.run(create_app(node), host=args.host, port=args.port, log_level="info")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
