#!/usr/bin/env python3
"""Read-only ROS 2 to FastAPI bridge for the Astrobot console."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np
import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from pydantic import BaseModel
from sensor_msgs.msg import Imu, PointCloud2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:  # astrobot_interfaces is optional for the legacy console
    from astrobot_interfaces.msg import LocalizationStatus
except ImportError:  # pragma: no cover
    LocalizationStatus = None

# Per-goal bookkeeping the event log would drown in (goal_sent already marks each goal).
QUIET_MISSION_EVENTS = {"goal_accepted", "planner_precheck_started"}
# Rates are message arrivals per sim second over the last RATE_WINDOW_SIM_S: Isaac runs below real
# time, so wall-clock rates would track the sim speed rather than the stack (shown separately).
RATE_WINDOW_SIM_S = 10.0
SERIES_PERIOD_WALL_S = 1.0
# Trails keep a point per TRAIL_SPACING_M of travel (the newest pose always replaces the tail), so
# the SLAM and GT trails span the same stretch of the run whatever their publish rates.
TRAIL_SPACING_M = 0.1
TRAIL_LIMIT = 3000
LOCALIZATION_STATES = ("starting", "healthy", "degraded", "stopping", "reinitializing", "rebuilding", "faulted")

# Per-stack inputs. Rates come from small topics only (raw, never deserialized): a console
# subscribing to point clouds adds DDS/CPU load to a behaviour run (observer effect).
STACKS: dict[str, dict[str, Any]] = {
    "legacy": {
        "maps": {"obstacle": "slam/bev_costmap"},
        "rates": [
            ("lidar_0/pointcloud/filtered", PointCloud2, "LiDAR Hz"),
            ("imu", Imu, "IMU Hz"),
            ("slam/prob_voxel_map", PointCloud2, "Voxel map Hz"),
        ],
        "map_rates": {"obstacle": "BEV map Hz"},
    },
    "v2": {
        # global_costmap: Nav2 costs on the persistent mosaic; exploration_map: observed
        # free/occupied/unknown; planner_costmap: the 6 m window Nav2's local costmap reads.
        "maps": {
            "obstacle": "/terrain_shadow/global_costmap",
            "explore": "/terrain_shadow/exploration_map",
            "local": "/terrain_shadow/planner_costmap",
        },
        "rates": [("imu_physics", Imu, "IMU Hz")],
        "map_rates": {"local": "Terrain window Hz", "obstacle": "Mosaic Hz"},
        # RTK-anchored map = world - home (v2_slam.launch.py synthetic_rtk home_xy -20, -20): the
        # GT overlay and SLAM error use this exact transform instead of a fit to the SLAM path.
        "gt_to_map": (20.0, 20.0, 0.0),
    },
}


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
    max_cells: int = 400_000,
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
    grid = np.frombuffer(bytes(msg.data), dtype=np.int8, count=width * height).reshape(height, width)
    if stride > 1:
        # Max-pool (unknown -1 < free 0 < cost): striding would drop small rocks between samples.
        pad_h, pad_w = -height % stride, -width % stride
        grid = np.pad(grid, ((0, pad_h), (0, pad_w)), constant_values=-1)
        grid = grid.reshape(grid.shape[0] // stride, stride, grid.shape[1] // stride, stride).max(axis=(1, 3))
    display_height, display_width = grid.shape

    # OmniLRS publishes GT/resource maps with yaw≈±π (axes flipped vs standard).
    # Rotate 180°: flip data and move origin to the corner that becomes the new (0,0).
    if source_frame == "gt_map" and abs(abs(origin_yaw) - math.pi) < 0.15:
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)
        dw, dh = display_width, display_height
        # The far corner of the grid (dw, dh cells along its axes) becomes the new origin
        new_origin_x = origin_x + c * dw * resolution - s * dh * resolution
        new_origin_y = origin_y + s * dw * resolution + c * dh * resolution
        origin_x = new_origin_x
        origin_y = new_origin_y
        origin_yaw = normalize_angle(origin_yaw + math.pi)  # → ≈0
        grid = grid[::-1, ::-1]

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
        "data": grid.astype(int).ravel().tolist(),
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
    stamp: float = 0.0
    count: int = 0
    arrivals: deque = field(default_factory=lambda: deque(maxlen=4000))  # sim arrival times


def trail_append(trail: list[dict[str, float]], pose: dict[str, float]) -> None:
    """A point per TRAIL_SPACING_M of travel; the newest pose replaces the tail until then."""
    tail, previous = (trail[-1], trail[-2]) if len(trail) >= 2 else (None, None)
    if tail is not None and math.hypot(tail["x"] - previous["x"], tail["y"] - previous["y"]) < TRAIL_SPACING_M:
        trail[-1] = pose
        return
    trail.append(pose)
    if len(trail) > TRAIL_LIMIT:
        del trail[: len(trail) - TRAIL_LIMIT]


def compact_pose(pose: dict[str, float]) -> dict[str, float]:
    return {"x": round(pose["x"], 3), "y": round(pose["y"], 3), "yaw": round(pose["yaw"], 3),
            "stamp": round(pose.get("stamp", 0.0), 2)}


def windowed_rate(stamps: deque, now: float, window: float = RATE_WINDOW_SIM_S) -> float:
    """Arrivals per second over the last `window` seconds (0 once the topic has gone quiet)."""
    while stamps and now - stamps[0] > window:
        stamps.popleft()
    if len(stamps) < 2 or now - stamps[-1] > window:
        return 0.0
    return (len(stamps) - 1) / max(1e-6, stamps[-1] - stamps[0])


def load_gt_rocks(path: str, gt_to_map: tuple[float, float, float]) -> list[dict[str, float]]:
    """Rock footprints (axis-aligned world bounds) from the sim's evaluator scene inventory."""
    with open(path) as handle:
        objects = json.load(handle).get("objects", [])
    ox, oy, yaw = gt_to_map
    if abs(yaw) > 1e-6:
        raise ValueError("scene rocks need a translation-only gt->map transform")
    return [{"x0": round(o["minimum"][0] + ox, 3), "y0": round(o["minimum"][1] + oy, 3),
             "x1": round(o["maximum"][0] + ox, 3), "y1": round(o["maximum"][1] + oy, 3)} for o in objects]


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
    gt_trajectory: list[dict[str, float]] = field(default_factory=list)
    localization: dict[str, Any] | None = None
    terrain: dict[str, Any] | None = None
    event_log: list[dict[str, Any]] = field(default_factory=list)


class UiBridgeNode(Node):
    def __init__(self, robot_namespace: str) -> None:
        super().__init__("ui_backend")
        self.robot_namespace = robot_namespace.strip("/")
        self._lock = threading.RLock()
        self._state = BridgeState()
        self._last_grid_conversion: dict[str, float] = {}
        self._latest_grid_msgs: dict[str, OccupancyGrid] = {}
        self._map_versions: dict[str, int] = {}
        self._grid_content: dict[str, tuple[Any, ...]] = {}
        self._grid_conversion_min_period_sec = 0.5
        self._gt_home_pose: dict[str, float] | None = None
        self._slam_home_pose: dict[str, float] | None = None
        self._gt_to_map_transform: tuple[float, float, float] | None = None
        # Windowed least-squares gt->map alignment (positions only -- yaw diffs from
        # single quaternions are fragile; the IMU-frame pi rotation poisoned them).
        self._align_pairs: list[tuple[float, float, float, float]] = []  # (gt_x, gt_y, map_x, map_y)
        self._align_fitted = False
        self._last_gt_pose: dict[str, float] | None = None
        self._clock_samples: list[tuple[float, float]] = []  # (wall, sim) for the sim speed
        self._terrain_received: deque = deque()  # (sim time, chunks received) for the chunk rate
        self._last_series_sample = 0.0
        self._candidates: list[dict[str, float]] = []
        self.gt_rocks: list[dict[str, float]] = []
        self._mission_started_sim: float | None = None  # from exploration events (stamp - elapsed_sec)

        # legacy: BEV mapper (phase2_slam). v2: v2 terrain publisher + supervisor (slam_stack:=v2).
        self.stack = str(self.declare_parameter("stack", "legacy").value)
        if self.stack not in STACKS:
            raise ValueError(f"unknown stack {self.stack!r}; expected one of {sorted(STACKS)}")
        stack = STACKS[self.stack]
        self.footprint = {
            "halfLength": float(self.declare_parameter("footprint_half_length_m", 0.495).value),
            "halfWidth": float(self.declare_parameter("footprint_half_width_m", 0.335).value),
        }
        maps = dict(stack["maps"])
        override = str(self.declare_parameter("obstacle_map_topic", "").value)
        if override:
            maps["obstacle"] = override
        self._map_rate_names = {maps[k]: name for k, name in stack["map_rates"].items() if k in maps}
        self.rate_series = [name for _, _, name in stack["rates"]] + list(stack["map_rates"].values()) + ["SLAM pose Hz"]
        if stack.get("gt_to_map") is not None:
            self._gt_to_map_transform = tuple(float(v) for v in stack["gt_to_map"])
            self._align_fitted = True  # known, never refit from the SLAM path
        scene = str(self.declare_parameter("gt_scene_path", "").value)
        if scene and self._gt_to_map_transform is not None:
            try:
                self.gt_rocks = load_gt_rocks(scene, self._gt_to_map_transform)
            except (OSError, ValueError, KeyError) as exc:
                self.get_logger().warning(f"GT rocks unavailable from {scene}: {exc}")
        for layer_name, topic in maps.items():
            self._subscribe_grid(layer_name, topic)
        self._subscribe_grid("mission", "exploration/mission_grid")
        self._subscribe_grid("water", "/gt/resource_maps/water")
        self._subscribe_grid("gt", "/gt/map")
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._rate_names = {topic: name for topic, _, name in stack["rates"]} | {"slam/pose_corrected": "SLAM pose Hz"}
        for topic, msg_type, _ in stack["rates"]:
            self.create_subscription(msg_type, topic, self._freshness_cb(topic), sensor_qos, raw=True)
        if self.stack == "v2":
            self.rate_series.insert(0, "Native chunks Hz")
            self.create_subscription(String, "/terrain_shadow/status", self._terrain_cb, 10)
            if LocalizationStatus is not None:
                self.create_subscription(LocalizationStatus, "localization/status", self._localization_cb, 10)
        self.create_subscription(String, "exploration/events", self._mission_event_cb, 20)
        self.create_subscription(PoseStamped, "slam/pose_corrected", self._pose_cb, 20)
        self.create_subscription(PoseStamped, "/gt/base_link_pose", self._gt_pose_cb, 10)
        self.create_subscription(PoseStamped, "exploration/selected_goal", self._goal_cb, 10)
        self.create_subscription(PoseArray, "exploration/frontiers", self._candidates_cb, 10)
        self.create_subscription(Path, "plan", self._plan_cb, 5)
        self.create_subscription(String, "diagnostics", self._diagnostics_cb, 10)
        self.create_subscription(String, "exploration/status", self._mission_cb, 10)
        self.create_subscription(Image, "camera_0/image_raw", self._camera_cb("camera_0/image_raw"), sensor_qos)
        self.create_subscription(Image, "/front_camera/mono/rgb", self._camera_cb("camera_0/image_raw"), sensor_qos)
        self.teleop_goal_pub = self.create_publisher(PoseStamped, "teleop/goal", 10)

    def _touch(self, topic: str) -> None:
        freshness = self._state.topic_freshness.setdefault(topic, TopicFreshness(topic=topic))
        now = time.time()
        freshness.stamp = now
        freshness.count += 1
        freshness.arrivals.append(self._rate_clock())
        self._sample_series()

    def _rate_clock(self) -> float:
        sim = self.get_clock().now().nanoseconds * 1e-9
        return sim if sim > 0 else time.monotonic()

    def _topic_hz(self, freshness: TopicFreshness) -> float:
        return windowed_rate(freshness.arrivals, self._rate_clock())

    def _sample_series(self) -> None:
        """Rate and sim-speed series at a fixed wall cadence (not per message or per client)."""
        wall = time.monotonic()
        if wall - self._last_series_sample < SERIES_PERIOD_WALL_S:
            return
        self._last_series_sample = wall
        for topic, freshness in self._state.topic_freshness.items():
            name = self._rate_names.get(topic) or self._map_rate_names.get(topic)
            if topic == "diagnostics":
                name = "Diagnostics Hz"
            elif topic.startswith("camera_"):
                name = "Camera Hz"
            if name:
                self._append_series(name, self._topic_hz(freshness))
        chunks = self._terrain_received
        if len(chunks) >= 2:
            (t0, n0), (t1, n1) = chunks[0], chunks[-1]
            self._append_series("Native chunks Hz", (n1 - n0) / max(1e-6, t1 - t0))
        self._append_series("Sim speed", self._sim_speed())

    def _subscribe_grid(self, layer_name: str, topic: str) -> None:
        self.create_subscription(OccupancyGrid, topic, lambda msg, key=layer_name, source=topic: self._grid_cb(key, source, msg), 2)

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
            # GT/resource maps are republished unchanged: re-sending ~1 MB layers would be pure load.
            info = msg.info
            content = (msg.header.frame_id, info.width, info.height, info.resolution, info.origin.position.x,
                       info.origin.position.y, info.origin.orientation.z, info.origin.orientation.w, bytes(msg.data))
            if self._grid_content.get(layer_name) != content:
                self._grid_content[layer_name] = content
                self._set_map(layer_name, occupancy_grid_to_dict(msg, gt_to_map_transform=self._gt_to_map_transform))
            self._touch(topic)

    def _set_map(self, layer_name: str, value: dict[str, Any]) -> None:
        self._state.maps[layer_name] = value
        self._map_versions[layer_name] = self._map_versions.get(layer_name, 0) + 1

    def changed_maps(self, sent: dict[str, int]) -> tuple[dict[str, Any], dict[str, int]]:
        """Layers converted since the versions a client was sent (a 600 x 600 mosaic is ~1 MB of JSON)."""
        with self._lock:
            changed = {k: v for k, v in self._state.maps.items() if self._map_versions.get(k, 0) != sent.get(k)}
            return changed, dict(self._map_versions)

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
            if self.stack != "legacy":
                gt = self._gt_in_map(self._last_gt_pose)
                if gt is not None and abs(gt["stamp"] - pose["stamp"]) <= 0.15:
                    self._append_series("SLAM error", math.hypot(pose["x"] - gt["x"], pose["y"] - gt["y"]))
            trail_append(self._state.trajectories.setdefault(self.robot_namespace, []), pose)
            self._touch("slam/pose_corrected")

    def _gt_pose_cb(self, msg: PoseStamped) -> None:
        pose = pose_msg_to_dict(msg)
        with self._lock:
            self._last_gt_pose = pose
            if self._gt_home_pose is None:
                self._gt_home_pose = pose
                self._update_gt_to_map_offset()
            trail_append(self._state.gt_trajectory, pose)
            self._touch("/gt/base_link_pose")

    def _gt_in_map(self, pose: dict[str, float] | None) -> dict[str, float] | None:
        """A GT (gt_map) pose in the display map frame: evaluator overlay only, never fed back."""
        if pose is None or self._gt_to_map_transform is None:
            return None
        tx, ty, yaw = self._gt_to_map_transform
        c, s = math.cos(yaw), math.sin(yaw)
        return {"x": tx + c * pose["x"] - s * pose["y"], "y": ty + s * pose["x"] + c * pose["y"],
                "yaw": normalize_angle(pose["yaw"] + yaw), "stamp": pose["stamp"]}

    def _localization_cb(self, msg: Any) -> None:
        state = int(msg.state)
        value = {
            "state": LOCALIZATION_STATES[state] if state < len(LOCALIZATION_STATES) else str(state),
            "motionPermitted": bool(msg.motion_permitted),
            "reason": str(msg.reason),
            "epoch": int(msg.epoch),
            "stamp": time.time(),
        }
        with self._lock:
            previous = self._state.localization
            if previous is None or (previous["state"], previous["reason"]) != (value["state"], value["reason"]):
                severity = ("nominal" if value["motionPermitted"] else
                            "critical" if value["state"] == "faulted" else "warning")
                self._log_event(severity, f"localization {value['state']}: {value['reason']} (epoch {value['epoch']})",
                                "supervisor")
            self._state.localization = value
            self._touch("localization/status")

    def _terrain_cb(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        sim_now = self.get_clock().now().nanoseconds * 1e-9
        evidence = status.get("evidence_ns")
        received = int(status.get("received", 0) or 0)
        with self._lock:
            previous = self._state.terrain
            value = {
                "valid": bool(status.get("valid")),
                "failure": status.get("failure"),
                "epoch": status.get("epoch"),
                "received": received,
                "projected": int(status.get("projected", 0) or 0),
                "skippedSequences": int(status.get("skipped_sequences", 0) or 0),
                "lateChunks": int(status.get("late_chunks", 0) or 0),
                "evidenceLagSec": (sim_now - evidence * 1e-9) if evidence and evidence > 0 and sim_now > 0 else None,
                "peakRssMib": status.get("peak_process_rss_mib"),
                "stamp": now,
            }
            if previous is not None and previous["valid"] != value["valid"]:
                self._log_event("nominal" if value["valid"] else "warning",
                                f"terrain map {'valid' if value['valid'] else 'invalid'}"
                                + (f": {value['failure']}" if value["failure"] else ""), "terrain")
            chunks = self._terrain_received
            if chunks and (received < chunks[-1][1] or sim_now < chunks[-1][0]):
                chunks.clear()  # publisher restart or sim clock reset
            if not chunks or sim_now > chunks[-1][0]:  # statuses within one /clock tick share a stamp
                chunks.append((sim_now, received))
            while chunks and sim_now - chunks[0][0] > RATE_WINDOW_SIM_S:
                chunks.popleft()
            self._state.terrain = value
            self._touch("/terrain_shadow/status")

    def _mission_event_cb(self, msg: String) -> None:
        try:
            record = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = str(record.get("event", "event"))
        if isinstance(record.get("stamp"), (int, float)) and isinstance(record.get("elapsed_sec"), (int, float)):
            self._mission_started_sim = float(record["stamp"]) - float(record["elapsed_sec"])
        if event in QUIET_MISSION_EVENTS:
            return
        detail = " ".join(str(record[k]) for k in ("kind", "reason") if record.get(k))
        severity = ("critical" if event == "mission_failed" else
                    "warning" if any(word in event for word in ("fail", "blacklist", "reject", "unavailable", "cancel"))
                    else "nominal")
        with self._lock:
            self._log_event(severity, f"{event.replace('_', ' ')}{(' · ' + detail) if detail else ''}", "mission")

    def _log_event(self, severity: str, text: str, source: str, limit: int = 60) -> None:
        log = self._state.event_log
        log.append({"stamp": time.time(), "severity": severity, "text": text, "source": source})
        if len(log) > limit:
            del log[: len(log) - limit]

    def _sim_speed(self) -> float | None:
        """Sim seconds per wall second over the last ~10 s (Isaac runs below real time)."""
        sim_now = self.get_clock().now().nanoseconds * 1e-9
        if sim_now <= 0:
            return None
        wall = time.monotonic()
        samples = self._clock_samples
        if not samples or wall - samples[-1][0] >= 0.5:
            samples.append((wall, sim_now))
        while len(samples) > 2 and wall - samples[1][0] > 10.0:
            samples.pop(0)
        (w0, s0), (w1, s1) = samples[0], samples[-1]
        return (s1 - s0) / (w1 - w0) if w1 - w0 >= 2.0 else None

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
                self._set_map(layer_name, occupancy_grid_to_dict(msg, gt_to_map_transform=self._gt_to_map_transform))

    def _goal_cb(self, msg: PoseStamped) -> None:
        with self._lock:
            self._state.goal = pose_msg_to_dict(msg)
            self._touch("exploration/selected_goal")

    def _candidates_cb(self, msg: PoseArray) -> None:
        # exploration_manager publishes its ranked candidates (best first) each planning cycle.
        with self._lock:
            self._candidates = [{"x": round(p.position.x, 3), "y": round(p.position.y, 3)} for p in msg.poses]
            self._touch("exploration/frontiers")

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
            if self.stack == "legacy":  # other stacks score SLAM against their exact GT transform
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
                "trajectory": [compact_pose(p) for p in self._state.trajectories.get(self.robot_namespace, [])],
                "plan": list(self._state.plan),
                "diagnostics": diagnostics.get("classification", diagnostics),
                "footprint": dict(self.footprint),
                "gtPose": self._gt_in_map(self._last_gt_pose),
                "gtTrajectory": [compact_pose(p) for p in map(self._gt_in_map, self._state.gt_trajectory) if p is not None],
                # the list from the last goal selection is stale once the mission heads home
                "candidates": [] if mission_raw.get("returning_home") or mission_raw.get("state") in ("finished", "failed")
                else list(self._candidates),
            }
            mission = {
                "mode": str(mission_raw.get("mission_mode", "unknown")),
                "state": str(mission_raw.get("state", "waiting_for_inputs")),
                # mission sim time when the manager's events give its start, else since backend start
                "elapsedSec": (float(self.get_clock().now().nanoseconds * 1e-9 - self._mission_started_sim)
                               if self._mission_started_sim is not None
                               else float(time.monotonic() - self._state.started_at)),
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
            sim_speed = self._sim_speed()
            sim_time = self.get_clock().now().nanoseconds * 1e-9
            return {
                "schemaVersion": "astrobot.console.v1",
                "stamp": time.time(),
                "connected": True,
                "stack": self.stack,
                "simTime": sim_time if sim_time > 0 else None,
                "simSpeed": sim_speed,
                "rateSeries": list(self.rate_series),
                "localization": dict(self._state.localization) if self._state.localization else None,
                "terrain": dict(self._state.terrain) if self._state.terrain else None,
                "gtRocks": self.gt_rocks,
                "dds": {"domain": os.environ.get("ROS_DOMAIN_ID", "0"),
                        "rmw": os.environ.get("RMW_IMPLEMENTATION", "default").replace("rmw_", "").replace("_cpp", "")},
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
        return events[:4] + list(reversed(self._state.event_log))[:40]


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
        sent: dict[str, int] = {}
        try:
            while True:
                frame = node.snapshot(include_maps=False)
                frame["maps"], sent = node.changed_maps(sent)
                await websocket.send_json(frame)
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
        # An open browser WebSocket would otherwise hold a Ctrl-C'd backend (and port 8080) forever.
        uvicorn.run(create_app(node), host=args.host, port=args.port, log_level="info", timeout_graceful_shutdown=3)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
