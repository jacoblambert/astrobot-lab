#!/usr/bin/env python3
"""Frontier and resource-aware single-robot exploration manager."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import asdict, dataclass

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class Candidate:
    x: float
    y: float
    info_gain: float
    resource_score: float
    terrain_cost: float
    distance: float
    total_score: float
    cells: int


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
    return Pose2D(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalize_angle(a.yaw + b.yaw))


class ExplorationManager(Node):
    def __init__(self) -> None:
        super().__init__("exploration_manager")
        self.robot_namespace = self.declare_parameter("robot_namespace", "astrobot_0").value.strip("/")
        self.mission_mode = self.declare_parameter("mission_mode", "explore_radius").value
        self.map_topic = self.declare_parameter("map_topic", "slam/bev_costmap").value
        self.pose_topic = self.declare_parameter("pose_topic", "slam/pose_corrected").value
        self.resource_names = [
            name.strip()
            for name in self.declare_parameter("resource_names", "water").value.split(",")
            if name.strip()
        ]
        self.resource_weights = self._parse_resource_weights(
            self.declare_parameter("resource_weights", "water:1.0").value
        )
        self.action_name = self.declare_parameter("action_name", "navigate_to_pose").value
        self.planner_action_name = self.declare_parameter("planner_action_name", "compute_path_to_pose").value
        self.frame_id = self.declare_parameter("frame_id", "map").value

        self.base_offset = Pose2D(
            self.declare_parameter("base_offset_x", 0.264).value,
            self.declare_parameter("base_offset_y", 0.017).value,
            self.declare_parameter("base_yaw_offset", math.pi).value,
        )
        self.home_radius_m = float(self.declare_parameter("home_radius_m", 8.0).value)
        self.coverage_goal_fraction = float(self.declare_parameter("coverage_goal_fraction", 0.30).value)
        self.sample_success_threshold = float(self.declare_parameter("sample_success_threshold", 0.75).value)
        self.max_goals = int(self.declare_parameter("max_goals", 30).value)
        self.max_duration_sec = float(self.declare_parameter("max_duration_sec", 900.0).value)
        self.goal_timeout_sec = float(self.declare_parameter("goal_timeout_sec", 120.0).value)
        self.goal_timeout_per_meter_sec = float(self.declare_parameter("goal_timeout_per_meter_sec", 18.0).value)
        self.goal_timeout_max_sec = float(self.declare_parameter("goal_timeout_max_sec", 300.0).value)
        self.cycle_period_sec = float(self.declare_parameter("cycle_period_sec", 2.0).value)

        self.free_cost_max = int(self.declare_parameter("free_cost_max", 35).value)
        self.lethal_cost_min = int(self.declare_parameter("lethal_cost_min", 80).value)
        self.min_frontier_cells = int(self.declare_parameter("min_frontier_cells", 4).value)
        self.max_frontiers = int(self.declare_parameter("max_frontiers", 80).value)
        self.goal_clearance_cells = int(self.declare_parameter("goal_clearance_cells", 3).value)
        self.goal_clearance_radius_m = float(self.declare_parameter("goal_clearance_radius_m", 0.75).value)
        self.goal_clearance_cost_min = int(self.declare_parameter("goal_clearance_cost_min", 80).value)
        self.max_goal_distance_m = float(self.declare_parameter("max_goal_distance_m", 5.0).value)
        self.blacklist_radius_m = float(self.declare_parameter("blacklist_radius_m", 0.75).value)
        self.blacklist_timeout_sec = float(self.declare_parameter("blacklist_timeout_sec", 0.0).value)
        self.min_goal_distance_m = float(self.declare_parameter("min_goal_distance_m", 0.75).value)
        self.recent_goal_radius_m = float(self.declare_parameter("recent_goal_radius_m", 0.75).value)
        self.recent_goal_timeout_sec = float(self.declare_parameter("recent_goal_timeout_sec", 240.0).value)

        self.info_weight = float(self.declare_parameter("info_weight", 1.0).value)
        self.resource_weight_scale = float(self.declare_parameter("resource_weight_scale", 4.0).value)
        self.distance_weight = float(self.declare_parameter("distance_weight", 0.8).value)
        self.over_distance_weight = float(self.declare_parameter("over_distance_weight", 2.0).value)
        self.terrain_weight = float(self.declare_parameter("terrain_weight", 0.025).value)
        self.resource_influence_radius_m = float(self.declare_parameter("resource_influence_radius_m", 3.0).value)
        self.resource_decay_radius_m = float(self.declare_parameter("resource_decay_radius_m", 1.5).value)
        self.max_resource_candidates = int(self.declare_parameter("max_resource_candidates", 20).value)
        self.resource_explore_radius_m = float(self.declare_parameter("resource_explore_radius_m", 3.0).value)
        self.resource_min_explore_step_m = float(self.declare_parameter("resource_min_explore_step_m", 0.75).value)
        self.use_resource_candidates_in_explore_radius = bool(
            self.declare_parameter("use_resource_candidates_in_explore_radius", False).value
        )
        self.planner_precheck_enabled = bool(self.declare_parameter("planner_precheck_enabled", True).value)
        self.planner_precheck_timeout_sec = float(self.declare_parameter("planner_precheck_timeout_sec", 8.0).value)
        self.planner_id = self.declare_parameter("planner_id", "GridBased").value
        self.planner_precheck_corridor_radius_m = float(
            self.declare_parameter("planner_precheck_corridor_radius_m", 0.35).value
        )
        self.planner_precheck_max_cost = int(self.declare_parameter("planner_precheck_max_cost", 70).value)
        self.planner_precheck_high_cost = int(self.declare_parameter("planner_precheck_high_cost", 50).value)
        self.planner_precheck_max_high_cost_fraction = float(
            self.declare_parameter("planner_precheck_max_high_cost_fraction", 0.04).value
        )
        self.breadcrumb_return_enabled = bool(self.declare_parameter("breadcrumb_return_enabled", True).value)
        self.breadcrumb_spacing_m = float(self.declare_parameter("breadcrumb_spacing_m", 0.75).value)
        self.breadcrumb_max_home_radius_factor = float(
            self.declare_parameter("breadcrumb_max_home_radius_factor", 1.25).value
        )
        self.return_breadcrumb_stride_m = float(self.declare_parameter("return_breadcrumb_stride_m", 1.5).value)
        self.return_breadcrumb_min_distance_m = float(
            self.declare_parameter("return_breadcrumb_min_distance_m", 0.75).value
        )
        self.mission_cell_size_m = float(self.declare_parameter("mission_cell_size_m", 1.0).value)
        self.mission_coverage_min_known_fraction = float(
            self.declare_parameter("mission_coverage_min_known_fraction", 0.30).value
        )
        self.mission_candidate_weight = float(self.declare_parameter("mission_candidate_weight", 3.0).value)
        self.mission_candidate_limit = int(self.declare_parameter("mission_candidate_limit", 80).value)

        self.latest_map: OccupancyGrid | None = None
        self.grid: np.ndarray | None = None
        self.latest_path: Path | None = None
        self.latest_pose: Pose2D | None = None
        self.home_pose: Pose2D | None = None
        self.breadcrumbs: list[Pose2D] = []
        self.return_waypoints: list[Pose2D] = []
        self.visited_mission_cells: set[tuple[int, int]] = set()
        self.latest_mission_grid: OccupancyGrid | None = None
        self.latest_mission_total_cells = 0
        self.latest_mission_reachable_cells = 0
        self.latest_mission_max_reachable_cells = 0
        self.latest_mission_covered_cells = 0
        self.latest_mission_observed_cells = 0
        self.latest_mission_visited_cells = 0
        self.latest_mission_coverage_fraction = 0.0
        self.latest_mission_observed_fraction = 0.0
        self.start_known_cells = 0
        self.started_at = time.monotonic()
        self.goal_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.precheck_rejection_count = 0
        self.safety_cancellation_count = 0
        self.returning_home = False
        self.finished = False
        self.active_goal = False
        self.active_goal_handle = None
        self.active_goal_started_at = 0.0
        self.active_goal_timeout_sec = 0.0
        self.active_goal_distance = 0.0
        self.active_goal_pose: Pose2D | None = None
        self.active_goal_kind = ""
        self.cancel_requested = False
        self.active_goal_safety_cancelled = False
        self.pending_goal_check = False
        self.pending_goal_check_started_at = 0.0
        self.pending_goal_check_pose: Pose2D | None = None
        self.pending_goal_check_kind = ""
        self.pending_goal_check_candidate: Candidate | None = None
        self.best_sample = 0.0
        self.best_sample_resource = ""
        self.blacklist: list[tuple[float, float, float]] = []
        self.recent_goals: list[tuple[float, float, float]] = []
        self.resource_maps: dict[str, np.ndarray] = {}
        self.last_resource_sample: dict[str, float] = {}
        self.latest_reachable_cells = 0
        self.latest_resource_candidates = 0

        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 5)
        self.create_subscription(Path, "plan", self._path_cb, 5)
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 20)
        for resource_name in self.resource_names:
            self.create_subscription(
                Float32,
                f"resource/{resource_name}/sample",
                lambda msg, name=resource_name: self._resource_cb(name, msg),
                10,
            )

        self.frontier_pub = self.create_publisher(PoseArray, "exploration/frontiers", 10)
        self.selected_goal_pub = self.create_publisher(PoseStamped, "exploration/selected_goal", 10)
        self.mission_grid_pub = self.create_publisher(OccupancyGrid, "exploration/mission_grid", 2)
        self.status_pub = self.create_publisher(String, "exploration/status", 10)
        self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        self.planner_client = ActionClient(self, ComputePathToPose, self.planner_action_name)
        self.create_timer(self.cycle_period_sec, self._cycle)
        self.get_logger().info(
            f"exploration manager mode={self.mission_mode} map={self.map_topic} action={self.action_name}"
        )

    @staticmethod
    def _parse_resource_weights(value: str) -> dict[str, float]:
        weights: dict[str, float] = {}
        for item in value.split(","):
            item = item.strip()
            if not item:
                continue
            if ":" not in item:
                weights[item] = 1.0
                continue
            name, raw_weight = item.split(":", 1)
            weights[name.strip()] = float(raw_weight)
        return weights

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        for name in self.resource_names:
            if name not in self.resource_maps or self.resource_maps[name].shape != self.grid.shape:
                self.resource_maps[name] = np.full(self.grid.shape, np.nan, dtype=np.float32)

    def _path_cb(self, msg: Path) -> None:
        self.latest_path = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        imu_pose = Pose2D(msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(msg.pose.orientation))
        self.latest_pose = compose(imu_pose, self.base_offset)
        if self.home_pose is None:
            self.home_pose = self.latest_pose
            self.breadcrumbs.append(self.latest_pose)
        elif (
            self.breadcrumb_spacing_m > 0.0
            and self._pose_is_inside_breadcrumb_radius(self.latest_pose)
            and self._pose_distance(self.latest_pose, self.breadcrumbs[-1]) >= self.breadcrumb_spacing_m
        ):
            self.breadcrumbs.append(self.latest_pose)
        self._mark_mission_cell_visited(self.latest_pose)

    def _resource_cb(self, name: str, msg: Float32) -> None:
        self.last_resource_sample[name] = float(msg.data)
        self.best_sample = max(self.best_sample, float(msg.data))
        if self.best_sample == float(msg.data):
            self.best_sample_resource = name
        if self.latest_pose is None or self.latest_map is None or name not in self.resource_maps:
            return
        cell = self._world_to_cell(self.latest_pose.x, self.latest_pose.y)
        if cell is None:
            return
        row, col = cell
        self.resource_maps[name][row, col] = float(msg.data)

    def _cycle(self) -> None:
        if self.pending_goal_check:
            self._check_planner_precheck_timeout()
            self._publish_status()
            return
        if self.active_goal:
            self._check_active_goal_safety()
            self._check_goal_timeout()
            self._publish_status()
            return
        if self.finished:
            self._publish_status()
            return
        if self.latest_map is None or self.grid is None or self.latest_pose is None or self.home_pose is None:
            self._publish_status(state="waiting_for_inputs")
            return
        if self.start_known_cells == 0:
            self.start_known_cells = self._known_cells_in_radius()

        elapsed = time.monotonic() - self.started_at
        if elapsed >= self.max_duration_sec or self.goal_count >= self.max_goals:
            self._send_return_home("budget_exhausted")
            return
        if self.mission_mode == "sample_return" and self.best_sample >= self.sample_success_threshold:
            self._send_return_home("resource_found")
            return
        if self.mission_mode == "explore_radius" and self._coverage_goal_progress() >= self.coverage_goal_fraction:
            self._send_return_home("coverage_goal_met")
            return

        candidates = self._find_candidates()
        self._publish_frontiers(candidates)
        if not candidates:
            self._send_return_home("no_candidates")
            return
        best = candidates[0]
        self._send_goal(Pose2D(best.x, best.y, self.latest_pose.yaw), "explore", best)

    def _send_return_home(self, reason: str) -> None:
        if self.returning_home:
            return
        self.returning_home = True
        self.get_logger().info(f"returning home: {reason}")
        self.return_waypoints = self._build_return_waypoints()
        self.get_logger().info(f"return path waypoints={len(self.return_waypoints)}")
        self._send_next_return_waypoint()

    def _send_next_return_waypoint(self) -> None:
        while self.return_waypoints:
            waypoint = self._project_return_waypoint(self.return_waypoints.pop(0))
            if waypoint is None:
                continue
            self._send_goal(waypoint, "return_home", None)
            return
        self.finished = True
        self._publish_status(state="finished")

    def _build_return_waypoints(self) -> list[Pose2D]:
        if self.home_pose is None:
            return []
        if not self.breadcrumb_return_enabled or self.latest_pose is None or len(self.breadcrumbs) < 2:
            return [self.home_pose]

        waypoints: list[Pose2D] = []
        last = self.latest_pose
        stride = max(self.return_breadcrumb_stride_m, self.return_breadcrumb_min_distance_m)
        for pose in reversed(self.breadcrumbs[:-1]):
            if not self._pose_is_inside_breadcrumb_radius(pose):
                continue
            if self._pose_distance(pose, self.latest_pose) < self.return_breadcrumb_min_distance_m:
                continue
            if self._pose_distance(pose, last) < stride:
                continue
            if self._pose_distance(pose, self.home_pose) < self.return_breadcrumb_min_distance_m:
                continue
            waypoints.append(Pose2D(pose.x, pose.y, pose.yaw))
            last = pose
        if not waypoints or self._pose_distance(waypoints[-1], self.home_pose) >= self.return_breadcrumb_min_distance_m:
            waypoints.append(self.home_pose)
        return waypoints

    def _pose_is_inside_breadcrumb_radius(self, pose: Pose2D) -> bool:
        if self.home_pose is None or self.breadcrumb_max_home_radius_factor <= 0.0:
            return True
        max_radius = self.home_radius_m * self.breadcrumb_max_home_radius_factor
        return self._pose_distance(pose, self.home_pose) <= max_radius

    def _project_return_waypoint(self, pose: Pose2D) -> Pose2D | None:
        if self.grid is None or self.latest_map is None or self.latest_pose is None:
            return pose
        known_free = (self.grid >= 0) & (self.grid <= self.free_cost_max)
        reachable_free = self._reachable_free_mask(known_free)
        target = self._world_to_cell(pose.x, pose.y)
        if target is None:
            return None
        max_radius_cells = max(8, int(math.ceil(1.5 / self.latest_map.info.resolution)))
        projected = self._nearest_safe_free_cell(target[0], target[1], reachable_free, max_radius_cells)
        if projected is None:
            self.get_logger().warn(f"skipping unreachable return waypoint x={pose.x:.2f} y={pose.y:.2f}")
            return None
        row, col = projected
        x, y = self._cell_to_world(row, col)
        return Pose2D(x, y, pose.yaw)

    def _send_goal(self, pose: Pose2D, kind: str, candidate: Candidate | None) -> None:
        if pose is None:
            return
        if self.planner_precheck_enabled and kind == "explore":
            self._start_planner_precheck(pose, kind, candidate)
            return
        self._send_nav_goal(pose, kind, candidate)

    def _send_nav_goal(self, pose: Pose2D, kind: str, candidate: Candidate | None) -> None:
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self._publish_status(state="action_unavailable")
            return
        self.active_goal = True
        self.active_goal_handle = None
        self.active_goal_started_at = time.monotonic()
        self.active_goal_distance = self._distance_from_robot(pose)
        self.active_goal_timeout_sec = self._timeout_for_distance(self.active_goal_distance)
        self.active_goal_pose = pose
        self.active_goal_kind = kind
        self.cancel_requested = False
        self.active_goal_safety_cancelled = False
        self.goal_count += 1
        if kind == "explore":
            self._remember_goal(pose.x, pose.y)
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self._publish_selected_goal(goal.pose)
        score = candidate.total_score if candidate is not None else 0.0
        self.get_logger().info(
            f"goal {self.goal_count}: {kind} x={pose.x:.2f} y={pose.y:.2f} score={score:.2f} "
            f"distance={self.active_goal_distance:.1f}m timeout={self.active_goal_timeout_sec:.0f}s"
        )

        send_future = self.action_client.send_goal_async(goal)
        send_future.add_done_callback(lambda future, sent_at=time.monotonic(), k=kind, p=pose: self._goal_response(future, sent_at, k, p))

    def _start_planner_precheck(self, pose: Pose2D, kind: str, candidate: Candidate | None) -> None:
        if not self.planner_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("planner precheck unavailable; sending navigation goal directly")
            self._send_nav_goal(pose, kind, candidate)
            return

        self.pending_goal_check = True
        self.pending_goal_check_started_at = time.monotonic()
        self.pending_goal_check_pose = pose
        self.pending_goal_check_kind = kind
        self.pending_goal_check_candidate = candidate

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.pose.position.x = pose.x
        goal_pose.pose.position.y = pose.y
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = goal_pose
        plan_goal.planner_id = self.planner_id
        plan_goal.use_start = False
        future = self.planner_client.send_goal_async(plan_goal)
        future.add_done_callback(self._planner_precheck_response)

    def _planner_precheck_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._reject_pending_goal_check("planner_rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._planner_precheck_result)

    def _planner_precheck_result(self, future) -> None:
        if not self.pending_goal_check or self.pending_goal_check_pose is None:
            return
        result = future.result().result
        pose = self.pending_goal_check_pose
        kind = self.pending_goal_check_kind
        candidate = self.pending_goal_check_candidate
        if result.error_code != 0 or not result.path.poses:
            reason = result.error_msg or f"planner_error_{result.error_code}"
            self._reject_pending_goal_check(reason)
            return
        goal_error = self._goal_quality_error(pose)
        if goal_error:
            self._reject_pending_goal_check(goal_error)
            return
        quality_error = self._planned_path_quality_error(result.path)
        if quality_error:
            self._reject_pending_goal_check(quality_error)
            return
        self.pending_goal_check = False
        self.pending_goal_check_started_at = 0.0
        self.pending_goal_check_pose = None
        self.pending_goal_check_kind = ""
        self.pending_goal_check_candidate = None
        self._send_nav_goal(pose, kind, candidate)

    def _reject_pending_goal_check(self, reason: str) -> None:
        if self.pending_goal_check_pose is not None:
            pose = self.pending_goal_check_pose
            self.blacklist.append((pose.x, pose.y, time.monotonic()))
            self.precheck_rejection_count += 1
            self.get_logger().warn(f"planner precheck failed x={pose.x:.2f} y={pose.y:.2f}: {reason}")
        self.pending_goal_check = False
        self.pending_goal_check_started_at = 0.0
        self.pending_goal_check_pose = None
        self.pending_goal_check_kind = ""
        self.pending_goal_check_candidate = None

    def _check_planner_precheck_timeout(self) -> None:
        if not self.pending_goal_check or self.planner_precheck_timeout_sec <= 0.0:
            return
        elapsed = time.monotonic() - self.pending_goal_check_started_at
        if elapsed >= self.planner_precheck_timeout_sec:
            self._reject_pending_goal_check(f"timeout_{elapsed:.1f}s")

    def _goal_response(self, future, sent_at: float, kind: str, pose: Pose2D) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.failure_count += 1
            self.blacklist.append((pose.x, pose.y, time.monotonic()))
            self._clear_active_goal()
            return
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._goal_result(result, sent_at, kind, pose))

    def _goal_result(self, future, sent_at: float, kind: str, pose: Pose2D) -> None:
        result = future.result()
        status = result.status
        duration = time.monotonic() - sent_at
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.success_count += 1
            if kind == "return_home":
                if self.return_waypoints:
                    self.get_logger().info(f"return waypoint reached in {duration:.1f}s")
                    self._clear_active_goal()
                    self._send_next_return_waypoint()
                    return
                self.finished = True
                self.get_logger().info(f"return home succeeded in {duration:.1f}s")
        else:
            if kind == "explore" and self.active_goal_safety_cancelled:
                self.safety_cancellation_count += 1
                self.get_logger().warn(f"goal safety-canceled kind={kind} status={status} duration={duration:.1f}s")
            else:
                self.failure_count += 1
                self.blacklist.append((pose.x, pose.y, time.monotonic()))
                self.get_logger().warn(f"goal failed kind={kind} status={status} duration={duration:.1f}s")
            if kind == "return_home" and self.return_waypoints:
                self.get_logger().warn("return waypoint failed; trying next breadcrumb")
                self._clear_active_goal()
                self._send_next_return_waypoint()
                return
            elif kind == "return_home":
                self.finished = True
        self._clear_active_goal()
        self._publish_status(state="finished" if self.finished else "running")

    def _check_goal_timeout(self) -> None:
        if (
            self.cancel_requested
            or self.active_goal_handle is None
            or self.active_goal_pose is None
            or self.active_goal_timeout_sec <= 0.0
        ):
            return
        elapsed = time.monotonic() - self.active_goal_started_at
        if elapsed < self.active_goal_timeout_sec:
            return
        self.cancel_requested = True
        pose = self.active_goal_pose
        self.blacklist.append((pose.x, pose.y, time.monotonic()))
        self.get_logger().warn(
            f"goal timeout kind={self.active_goal_kind} x={pose.x:.2f} y={pose.y:.2f} "
            f"after {elapsed:.1f}s; canceling"
        )
        self.active_goal_handle.cancel_goal_async()

    def _check_active_goal_safety(self) -> None:
        if (
            self.cancel_requested
            or self.active_goal_handle is None
            or self.active_goal_kind != "explore"
            or self.active_goal_pose is None
        ):
            return
        goal_error = self._goal_quality_error(self.active_goal_pose)
        if goal_error:
            self._cancel_active_goal_for_safety(goal_error)
            return
        if self.latest_path is None or not self.latest_path.poses:
            return
        path_error = self._planned_path_quality_error(self.latest_path)
        if path_error:
            self._cancel_active_goal_for_safety(path_error)

    def _cancel_active_goal_for_safety(self, reason: str) -> None:
        if self.active_goal_handle is None or self.active_goal_pose is None:
            return
        pose = self.active_goal_pose
        self.blacklist.append((pose.x, pose.y, time.monotonic()))
        self.cancel_requested = True
        self.active_goal_safety_cancelled = True
        self.get_logger().warn(
            f"active goal became unsafe kind={self.active_goal_kind} x={pose.x:.2f} y={pose.y:.2f}: {reason}; canceling"
        )
        self.active_goal_handle.cancel_goal_async()

    def _clear_active_goal(self) -> None:
        self.active_goal = False
        self.active_goal_handle = None
        self.active_goal_started_at = 0.0
        self.active_goal_timeout_sec = 0.0
        self.active_goal_distance = 0.0
        self.active_goal_pose = None
        self.active_goal_kind = ""
        self.cancel_requested = False
        self.active_goal_safety_cancelled = False

    def _distance_from_robot(self, pose: Pose2D) -> float:
        if self.latest_pose is None:
            return 0.0
        return self._pose_distance(pose, self.latest_pose)

    @staticmethod
    def _pose_distance(a: Pose2D, b: Pose2D) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _timeout_for_distance(self, distance: float) -> float:
        if self.goal_timeout_sec <= 0.0:
            return 0.0
        timeout = self.goal_timeout_sec + self.goal_timeout_per_meter_sec * max(0.0, distance)
        if self.goal_timeout_max_sec > 0.0:
            return min(timeout, self.goal_timeout_max_sec)
        return timeout

    def _coverage_goal_progress(self) -> float:
        if self.latest_mission_reachable_cells > 0:
            return self.latest_mission_coverage_fraction
        return self._coverage_gain_fraction()

    def _find_candidates(self) -> list[Candidate]:
        assert self.grid is not None
        known_free = (self.grid >= 0) & (self.grid <= self.free_cost_max)
        reachable_free = self._reachable_free_mask(known_free)
        self.latest_reachable_cells = int(np.count_nonzero(reachable_free))
        if self.latest_reachable_cells == 0:
            return []
        mission_candidates = self._mission_candidates(reachable_free)
        if self.mission_mode == "explore_radius" and mission_candidates:
            return mission_candidates
        unknown = self.grid < 0
        frontier = unknown & self._dilate_bool(reachable_free, 1)
        frontier &= self._inside_home_radius_mask()

        clusters = self._cluster_mask(frontier, self.max_frontiers)
        candidates: list[Candidate] = []
        for cluster in clusters:
            if len(cluster) < self.min_frontier_cells:
                continue
            goal_cell = self._nearest_free_goal_cell(cluster, reachable_free)
            if goal_cell is None:
                continue
            row, col = goal_cell
            x, y = self._cell_to_world(row, col)
            if self._is_blacklisted(x, y):
                continue
            terrain_cost = float(max(0, self.grid[row, col]))
            if terrain_cost >= self.lethal_cost_min:
                continue
            if self._near_unsafe(row, col):
                continue
            distance = math.hypot(x - self.latest_pose.x, y - self.latest_pose.y)
            if not self._goal_is_worth_sending(x, y, distance):
                continue
            if distance > self.max_goal_distance_m > 0.0:
                staged_cell = self._staged_goal_cell(x, y, reachable_free)
                if staged_cell is None:
                    continue
                row, col = staged_cell
                x, y = self._cell_to_world(row, col)
                if self._is_blacklisted(x, y):
                    continue
                terrain_cost = float(max(0, self.grid[row, col]))
                if terrain_cost >= self.lethal_cost_min or self._near_unsafe(row, col):
                    continue
                distance = math.hypot(x - self.latest_pose.x, y - self.latest_pose.y)
                if not self._goal_is_worth_sending(x, y, distance):
                    continue
            over_distance = max(0.0, distance - self.max_goal_distance_m)
            resource_score = self._resource_score_near(row, col)
            info_gain = math.sqrt(float(len(cluster)))
            total = (
                self.info_weight * info_gain
                + self.resource_weight_scale * resource_score
                - self.distance_weight * distance
                - self.over_distance_weight * over_distance
                - self.terrain_weight * terrain_cost
            )
            candidates.append(Candidate(x, y, info_gain, resource_score, terrain_cost, distance, total, len(cluster)))
        occupied = {self._world_to_cell(candidate.x, candidate.y) for candidate in candidates}
        occupied.discard(None)
        mission_occupied = set(occupied)
        for candidate in mission_candidates:
            cell = self._world_to_cell(candidate.x, candidate.y)
            if cell is None or cell in mission_occupied:
                continue
            mission_occupied.add(cell)
            candidates.append(candidate)
        resource_candidates = []
        if self.mission_mode != "explore_radius" or self.use_resource_candidates_in_explore_radius:
            resource_candidates = self._resource_exploitation_candidates(reachable_free, mission_occupied)
        self.latest_resource_candidates = len(resource_candidates)
        candidates.extend(resource_candidates)
        candidates.sort(key=lambda c: c.total_score, reverse=True)
        return candidates

    def _mission_candidates(self, reachable_free: np.ndarray) -> list[Candidate]:
        mission = self._update_mission_grid(reachable_free)
        self._publish_mission_grid()
        if mission is None or self.latest_pose is None or self.latest_map is None:
            return []

        origin_x, origin_y, width, height, data = mission
        candidates: list[Candidate] = []
        resolution = self.latest_map.info.resolution
        search_radius = max(2, int(math.ceil(self.mission_cell_size_m / resolution)))
        for row in range(height):
            for col in range(width):
                value = data[row, col]
                if value < 0 or value >= 80:
                    continue
                x = origin_x + (col + 0.5) * self.mission_cell_size_m
                y = origin_y + (row + 0.5) * self.mission_cell_size_m
                if self._is_blacklisted(x, y):
                    continue
                target = self._world_to_cell(x, y)
                if target is None:
                    continue
                nav_cell = self._safe_free_cell_in_mission_cell(row, col, reachable_free)
                if nav_cell is None:
                    nav_cell = self._nearest_safe_free_cell(target[0], target[1], reachable_free, search_radius)
                if nav_cell is None:
                    continue
                nav_row, nav_col = nav_cell
                goal_x, goal_y = self._cell_to_world(nav_row, nav_col)
                if self._is_blacklisted(goal_x, goal_y):
                    continue
                goal_mission_cell = self._mission_cell_for_xy(goal_x, goal_y)
                if goal_mission_cell is None or goal_mission_cell in self.visited_mission_cells:
                    continue
                terrain_cost = float(max(0, self.grid[nav_row, nav_col]))
                if terrain_cost >= self.lethal_cost_min or self._near_unsafe(nav_row, nav_col):
                    continue
                distance = math.hypot(goal_x - self.latest_pose.x, goal_y - self.latest_pose.y)
                if not self._goal_is_worth_sending(goal_x, goal_y, distance):
                    continue
                if distance > self.max_goal_distance_m > 0.0:
                    staged_cell = self._staged_goal_cell(goal_x, goal_y, reachable_free)
                    if staged_cell is None:
                        continue
                    nav_row, nav_col = staged_cell
                    goal_x, goal_y = self._cell_to_world(nav_row, nav_col)
                    if self._is_blacklisted(goal_x, goal_y):
                        continue
                    terrain_cost = float(max(0, self.grid[nav_row, nav_col]))
                    if terrain_cost >= self.lethal_cost_min or self._near_unsafe(nav_row, nav_col):
                        continue
                    distance = math.hypot(goal_x - self.latest_pose.x, goal_y - self.latest_pose.y)
                    if not self._goal_is_worth_sending(goal_x, goal_y, distance):
                        continue
                resource_score = self._resource_score_near(nav_row, nav_col)
                total = (
                    self.mission_candidate_weight
                    + self.resource_weight_scale * resource_score
                    - self.distance_weight * distance
                    - self.terrain_weight * terrain_cost
                )
                candidates.append(Candidate(goal_x, goal_y, 1.0, resource_score, terrain_cost, distance, total, 1))

        candidates.sort(key=lambda c: c.total_score, reverse=True)
        return candidates[: self.mission_candidate_limit]

    def _update_mission_grid(self, reachable_free: np.ndarray):
        if self.latest_map is None or self.grid is None or self.home_pose is None or self.mission_cell_size_m <= 0.0:
            return None
        radius = self.home_radius_m
        cell_size = self.mission_cell_size_m
        width = max(1, int(math.ceil((2.0 * radius) / cell_size)))
        height = width
        origin_x = self.home_pose.x - 0.5 * width * cell_size
        origin_y = self.home_pose.y - 0.5 * height * cell_size
        data = np.full((height, width), -1, dtype=np.int16)

        total_cells = 0
        reachable_cells = 0
        covered_cells = 0
        observed_cells = 0
        visited_cells = 0
        resolution = self.latest_map.info.resolution
        for row in range(height):
            y0 = origin_y + row * cell_size
            y1 = y0 + cell_size
            cy = y0 + 0.5 * cell_size
            for col in range(width):
                x0 = origin_x + col * cell_size
                x1 = x0 + cell_size
                cx = x0 + 0.5 * cell_size
                if math.hypot(cx - self.home_pose.x, cy - self.home_pose.y) > radius:
                    continue
                total_cells += 1
                c0 = max(0, int(math.floor((x0 - self.latest_map.info.origin.position.x) / resolution)))
                c1 = min(self.grid.shape[1], int(math.ceil((x1 - self.latest_map.info.origin.position.x) / resolution)))
                r0 = max(0, int(math.floor((y0 - self.latest_map.info.origin.position.y) / resolution)))
                r1 = min(self.grid.shape[0], int(math.ceil((y1 - self.latest_map.info.origin.position.y) / resolution)))
                if r0 >= r1 or c0 >= c1:
                    continue
                window = self.grid[r0:r1, c0:c1]
                reachable_window = reachable_free[r0:r1, c0:c1]
                known_fraction = float(np.count_nonzero(window >= 0)) / float(window.size)
                reachable = bool(np.any(reachable_window))
                visited = (row, col) in self.visited_mission_cells
                observed = known_fraction >= self.mission_coverage_min_known_fraction
                if not reachable and not visited:
                    data[row, col] = -1
                    continue
                reachable_cells += 1
                if visited:
                    visited_cells += 1
                    data[row, col] = 80
                elif observed:
                    data[row, col] = 50
                else:
                    data[row, col] = 0
                if visited:
                    covered_cells += 1
                if observed:
                    observed_cells += 1

        self.latest_mission_total_cells = total_cells
        self.latest_mission_reachable_cells = reachable_cells
        self.latest_mission_covered_cells = covered_cells
        self.latest_mission_observed_cells = observed_cells
        self.latest_mission_visited_cells = visited_cells
        self.latest_mission_max_reachable_cells = max(self.latest_mission_max_reachable_cells, reachable_cells)
        coverage_denominator = max(self.latest_mission_max_reachable_cells, reachable_cells)
        self.latest_mission_coverage_fraction = covered_cells / coverage_denominator if coverage_denominator > 0 else 0.0
        self.latest_mission_observed_fraction = (
            self.latest_mission_observed_cells / reachable_cells if reachable_cells > 0 else 0.0
        )

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = float(cell_size)
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = data.astype(np.int8).reshape(-1).tolist()
        self.latest_mission_grid = msg
        return origin_x, origin_y, width, height, data

    def _publish_mission_grid(self) -> None:
        if self.latest_mission_grid is not None:
            self.latest_mission_grid.header.stamp = self.get_clock().now().to_msg()
            self.mission_grid_pub.publish(self.latest_mission_grid)

    def _resource_exploitation_candidates(
        self, reachable_free: np.ndarray, occupied: set[tuple[int, int]]
    ) -> list[Candidate]:
        if (
            self.latest_map is None
            or self.latest_pose is None
            or self.max_resource_candidates <= 0
            or not self.resource_maps
        ):
            return []

        combined = np.full(reachable_free.shape, np.nan, dtype=np.float32)
        known_any = np.zeros(reachable_free.shape, dtype=bool)
        for name, resource_grid in self.resource_maps.items():
            weight = self.resource_weights.get(name, 1.0)
            finite = np.isfinite(resource_grid)
            if not np.any(finite):
                continue
            known_any |= finite
            weighted = np.where(finite, resource_grid * weight, np.nan)
            combined = np.fmax(combined, weighted)
        finite_cells = np.argwhere(np.isfinite(combined))
        if finite_cells.size == 0:
            return []

        values = combined[np.isfinite(combined)]
        top_count = min(self.max_resource_candidates, values.size)
        top_indices = np.argpartition(values, -top_count)[-top_count:]
        top_rows_cols = finite_cells[top_indices]

        radius = max(1, int(math.ceil(self.resource_explore_radius_m / self.latest_map.info.resolution)))
        min_step_cells = max(1.0, self.resource_min_explore_step_m / self.latest_map.info.resolution)
        inside_radius = self._inside_home_radius_mask()
        candidates: list[Candidate] = []

        for source_row, source_col in top_rows_cols.tolist():
            best: Candidate | None = None
            r0 = max(0, source_row - radius)
            r1 = min(reachable_free.shape[0], source_row + radius + 1)
            c0 = max(0, source_col - radius)
            c1 = min(reachable_free.shape[1], source_col + radius + 1)
            for row in range(r0, r1):
                for col in range(c0, c1):
                    if (row, col) in occupied:
                        continue
                    if not reachable_free[row, col] or not inside_radius[row, col] or known_any[row, col]:
                        continue
                    if self._near_unsafe(row, col):
                        continue
                    source_dist_cells = math.hypot(row - source_row, col - source_col)
                    if source_dist_cells < min_step_cells or source_dist_cells > radius:
                        continue
                    x, y = self._cell_to_world(row, col)
                    if self._is_blacklisted(x, y):
                        continue
                    terrain_cost = float(max(0, self.grid[row, col]))
                    if terrain_cost >= self.lethal_cost_min:
                        continue
                    distance = math.hypot(x - self.latest_pose.x, y - self.latest_pose.y)
                    if not self._goal_is_worth_sending(x, y, distance):
                        continue
                    if distance > self.max_goal_distance_m > 0.0:
                        continue
                    resource_score = self._resource_score_near(row, col)
                    if resource_score <= 0.0:
                        continue
                    total = (
                        self.resource_weight_scale * resource_score
                        - self.distance_weight * distance
                        - self.terrain_weight * terrain_cost
                    )
                    candidate = Candidate(x, y, 0.0, resource_score, terrain_cost, distance, total, 1)
                    if best is None or candidate.total_score > best.total_score:
                        best = candidate
            if best is not None:
                cell = self._world_to_cell(best.x, best.y)
                if cell is not None:
                    occupied.add(cell)
                candidates.append(best)
        return candidates

    def _staged_goal_cell(self, frontier_x: float, frontier_y: float, known_free: np.ndarray) -> tuple[int, int] | None:
        assert self.latest_pose is not None
        dx = frontier_x - self.latest_pose.x
        dy = frontier_y - self.latest_pose.y
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return None
        scale = min(1.0, self.max_goal_distance_m / distance)
        target_x = self.latest_pose.x + dx * scale
        target_y = self.latest_pose.y + dy * scale
        target_cell = self._world_to_cell(target_x, target_y)
        if target_cell is None:
            return None
        return self._nearest_safe_free_cell(target_cell[0], target_cell[1], known_free, max_radius_cells=10)

    def _nearest_safe_free_cell(
        self, row: int, col: int, known_free: np.ndarray, max_radius_cells: int
    ) -> tuple[int, int] | None:
        best: tuple[int, int] | None = None
        best_dist2 = float("inf")
        for radius in range(max_radius_cells + 1):
            r0 = max(0, row - radius)
            r1 = min(known_free.shape[0], row + radius + 1)
            c0 = max(0, col - radius)
            c1 = min(known_free.shape[1], col + radius + 1)
            for rr in range(r0, r1):
                for cc in range(c0, c1):
                    if not known_free[rr, cc] or self._near_unsafe(rr, cc):
                        continue
                    dist2 = (rr - row) ** 2 + (cc - col) ** 2
                    if dist2 < best_dist2:
                        best = (rr, cc)
                        best_dist2 = dist2
            if best is not None:
                return best
        return None

    def _safe_free_cell_in_mission_cell(
        self, mission_row: int, mission_col: int, reachable_free: np.ndarray
    ) -> tuple[int, int] | None:
        if self.latest_map is None or self.home_pose is None or self.grid is None:
            return None
        cell_size = self.mission_cell_size_m
        resolution = self.latest_map.info.resolution
        width = max(1, int(math.ceil((2.0 * self.home_radius_m) / cell_size)))
        origin_x = self.home_pose.x - 0.5 * width * cell_size
        origin_y = self.home_pose.y - 0.5 * width * cell_size
        x0 = origin_x + mission_col * cell_size
        y0 = origin_y + mission_row * cell_size
        x1 = x0 + cell_size
        y1 = y0 + cell_size
        c0 = max(0, int(math.floor((x0 - self.latest_map.info.origin.position.x) / resolution)))
        c1 = min(self.grid.shape[1], int(math.ceil((x1 - self.latest_map.info.origin.position.x) / resolution)))
        r0 = max(0, int(math.floor((y0 - self.latest_map.info.origin.position.y) / resolution)))
        r1 = min(self.grid.shape[0], int(math.ceil((y1 - self.latest_map.info.origin.position.y) / resolution)))
        if r0 >= r1 or c0 >= c1:
            return None
        rows, cols = np.nonzero(reachable_free[r0:r1, c0:c1])
        if rows.size == 0:
            return None
        center_x = x0 + 0.5 * cell_size
        center_y = y0 + 0.5 * cell_size
        best: tuple[int, int] | None = None
        best_dist2 = float("inf")
        for local_row, local_col in zip(rows.tolist(), cols.tolist()):
            row = r0 + local_row
            col = c0 + local_col
            if self._near_unsafe(row, col):
                continue
            x, y = self._cell_to_world(row, col)
            dist2 = (x - center_x) ** 2 + (y - center_y) ** 2
            if dist2 < best_dist2:
                best = (row, col)
                best_dist2 = dist2
        return best

    def _cluster_mask(self, mask: np.ndarray, max_clusters: int) -> list[list[tuple[int, int]]]:
        visited = np.zeros(mask.shape, dtype=bool)
        clusters: list[list[tuple[int, int]]] = []
        rows, cols = np.nonzero(mask)
        for start_row, start_col in zip(rows.tolist(), cols.tolist()):
            if visited[start_row, start_col]:
                continue
            queue = deque([(start_row, start_col)])
            visited[start_row, start_col] = True
            cluster: list[tuple[int, int]] = []
            while queue:
                row, col = queue.popleft()
                cluster.append((row, col))
                for nr, nc in self._neighbors4(row, col):
                    if 0 <= nr < mask.shape[0] and 0 <= nc < mask.shape[1] and mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        queue.append((nr, nc))
            clusters.append(cluster)
            if len(clusters) >= max_clusters:
                break
        clusters.sort(key=len, reverse=True)
        return clusters

    def _nearest_free_goal_cell(self, cluster: list[tuple[int, int]], known_free: np.ndarray) -> tuple[int, int] | None:
        candidates = set()
        for row, col in cluster:
            for nr, nc in self._neighbors8(row, col):
                if 0 <= nr < known_free.shape[0] and 0 <= nc < known_free.shape[1] and known_free[nr, nc]:
                    candidates.add((nr, nc))
        if not candidates:
            return None
        centroid_row = sum(row for row, _ in cluster) / len(cluster)
        centroid_col = sum(col for _, col in cluster) / len(cluster)
        return min(candidates, key=lambda cell: (cell[0] - centroid_row) ** 2 + (cell[1] - centroid_col) ** 2)

    def _reachable_free_mask(self, known_free: np.ndarray) -> np.ndarray:
        assert self.latest_pose is not None
        start = self._world_to_cell(self.latest_pose.x, self.latest_pose.y)
        if start is None:
            return np.zeros_like(known_free, dtype=bool)
        start_cell = self._nearest_safe_free_cell(start[0], start[1], known_free, max_radius_cells=12)
        if start_cell is None:
            return np.zeros_like(known_free, dtype=bool)

        reachable = np.zeros_like(known_free, dtype=bool)
        queue = deque([start_cell])
        reachable[start_cell[0], start_cell[1]] = True
        while queue:
            row, col = queue.popleft()
            for nr, nc in self._neighbors4(row, col):
                if (
                    0 <= nr < known_free.shape[0]
                    and 0 <= nc < known_free.shape[1]
                    and known_free[nr, nc]
                    and not reachable[nr, nc]
                    and not self._near_unsafe(nr, nc)
                ):
                    reachable[nr, nc] = True
                    queue.append((nr, nc))
        return reachable

    def _resource_score_near(self, row: int, col: int) -> float:
        if self.latest_map is None:
            return 0.0
        radius = max(1, int(math.ceil(self.resource_influence_radius_m / self.latest_map.info.resolution)))
        sigma_cells = max(1.0, self.resource_decay_radius_m / self.latest_map.info.resolution)
        total = 0.0
        for name, resource_grid in self.resource_maps.items():
            weight = self.resource_weights.get(name, 1.0)
            r0 = max(0, row - radius)
            r1 = min(resource_grid.shape[0], row + radius + 1)
            c0 = max(0, col - radius)
            c1 = min(resource_grid.shape[1], col + radius + 1)
            window = resource_grid[r0:r1, c0:c1]
            if np.all(np.isnan(window)):
                continue
            rows, cols = np.indices(window.shape)
            rr = rows + r0
            cc = cols + c0
            dist2 = (rr - row) ** 2 + (cc - col) ** 2
            decay = np.exp(-0.5 * dist2 / (sigma_cells * sigma_cells))
            known = np.nan_to_num(window, nan=0.0)
            total += weight * float(np.max(known * decay))
        return total

    def _coverage_gain_fraction(self) -> float:
        if self.start_known_cells <= 0:
            return 0.0
        return max(0.0, (self._known_cells_in_radius() - self.start_known_cells) / self.start_known_cells)

    def _known_cells_in_radius(self) -> int:
        if self.grid is None:
            return 0
        mask = self._inside_home_radius_mask()
        return int(np.count_nonzero((self.grid >= 0) & mask))

    def _inside_home_radius_mask(self) -> np.ndarray:
        assert self.latest_map is not None
        assert self.grid is not None
        if self.home_pose is None:
            return np.ones(self.grid.shape, dtype=bool)
        height, width = self.grid.shape
        rows, cols = np.indices((height, width))
        xs = self.latest_map.info.origin.position.x + (cols + 0.5) * self.latest_map.info.resolution
        ys = self.latest_map.info.origin.position.y + (rows + 0.5) * self.latest_map.info.resolution
        return ((xs - self.home_pose.x) ** 2 + (ys - self.home_pose.y) ** 2) <= self.home_radius_m**2

    def _mark_mission_cell_visited(self, pose: Pose2D) -> None:
        cell = self._mission_cell_for_xy(pose.x, pose.y)
        if cell is not None:
            self.visited_mission_cells.add(cell)

    def _mission_cell_for_xy(self, x: float, y: float) -> tuple[int, int] | None:
        if self.home_pose is None or self.mission_cell_size_m <= 0.0:
            return None
        radius = self.home_radius_m
        cell_size = self.mission_cell_size_m
        width = max(1, int(math.ceil((2.0 * radius) / cell_size)))
        height = width
        origin_x = self.home_pose.x - 0.5 * width * cell_size
        origin_y = self.home_pose.y - 0.5 * height * cell_size
        col = int(math.floor((x - origin_x) / cell_size))
        row = int(math.floor((y - origin_y) / cell_size))
        if 0 <= row < height and 0 <= col < width:
            return row, col
        return None

    def _is_blacklisted(self, x: float, y: float) -> bool:
        now = time.monotonic()
        if self.blacklist_timeout_sec > 0.0:
            self.blacklist = [item for item in self.blacklist if now - item[2] <= self.blacklist_timeout_sec]
        return any(math.hypot(x - bx, y - by) <= self.blacklist_radius_m for bx, by, _ in self.blacklist)

    def _goal_is_worth_sending(self, x: float, y: float, distance: float) -> bool:
        if distance < self.min_goal_distance_m:
            return False
        return not self._is_recent_goal(x, y)

    def _is_recent_goal(self, x: float, y: float) -> bool:
        now = time.monotonic()
        self.recent_goals = [item for item in self.recent_goals if now - item[2] <= self.recent_goal_timeout_sec]
        return any(math.hypot(x - gx, y - gy) <= self.recent_goal_radius_m for gx, gy, _ in self.recent_goals)

    def _remember_goal(self, x: float, y: float) -> None:
        self.recent_goals.append((x, y, time.monotonic()))

    def _near_lethal(self, row: int, col: int) -> bool:
        assert self.grid is not None
        r = self.goal_clearance_cells
        r0 = max(0, row - r)
        r1 = min(self.grid.shape[0], row + r + 1)
        c0 = max(0, col - r)
        c1 = min(self.grid.shape[1], col + r + 1)
        return bool(np.any(self.grid[r0:r1, c0:c1] >= self.lethal_cost_min))

    def _near_unsafe(self, row: int, col: int) -> bool:
        assert self.grid is not None
        if self.latest_map is None:
            return self._near_lethal(row, col)
        radius_cells = max(
            self.goal_clearance_cells,
            int(math.ceil(self.goal_clearance_radius_m / self.latest_map.info.resolution)),
        )
        r0 = max(0, row - radius_cells)
        r1 = min(self.grid.shape[0], row + radius_cells + 1)
        c0 = max(0, col - radius_cells)
        c1 = min(self.grid.shape[1], col + radius_cells + 1)
        return bool(np.any(self.grid[r0:r1, c0:c1] >= self.goal_clearance_cost_min))

    def _planned_path_quality_error(self, path) -> str:
        if self.grid is None or self.latest_map is None:
            return ""
        if not path.poses:
            return "empty_path"
        info = self.latest_map.info
        radius_cells = max(0, int(math.ceil(self.planner_precheck_corridor_radius_m / info.resolution)))
        cells: set[tuple[int, int]] = set()
        for pose_stamped in path.poses:
            cell = self._world_to_cell(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            if cell is None:
                return "path_outside_costmap"
            row, col = cell
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    if dr * dr + dc * dc > radius_cells * radius_cells:
                        continue
                    rr = row + dr
                    cc = col + dc
                    if 0 <= rr < self.grid.shape[0] and 0 <= cc < self.grid.shape[1]:
                        cells.add((rr, cc))
        if not cells:
            return "path_no_costmap_cells"
        values = np.asarray([self.grid[row, col] for row, col in cells], dtype=np.int16)
        unknown_fraction = float(np.count_nonzero(values < 0) / values.size)
        if unknown_fraction > 0.02:
            return f"path_unknown_fraction_{unknown_fraction:.2f}"
        known = values[values >= 0]
        if known.size == 0:
            return "path_all_unknown"
        max_cost = int(np.max(known))
        high_fraction = float(np.count_nonzero(known >= self.planner_precheck_high_cost) / known.size)
        if max_cost >= self.planner_precheck_max_cost:
            return f"path_max_cost_{max_cost}"
        if high_fraction > self.planner_precheck_max_high_cost_fraction:
            return f"path_high_cost_fraction_{high_fraction:.2f}"
        return ""

    def _goal_quality_error(self, pose: Pose2D) -> str:
        if self.grid is None or self.latest_map is None:
            return ""
        cell = self._world_to_cell(pose.x, pose.y)
        if cell is None:
            return "goal_outside_costmap"
        row, col = cell
        radius_cells = max(
            self.goal_clearance_cells,
            int(math.ceil(self.goal_clearance_radius_m / self.latest_map.info.resolution)),
        )
        r0 = max(0, row - radius_cells)
        r1 = min(self.grid.shape[0], row + radius_cells + 1)
        c0 = max(0, col - radius_cells)
        c1 = min(self.grid.shape[1], col + radius_cells + 1)
        window = self.grid[r0:r1, c0:c1]
        if window.size == 0:
            return "goal_no_costmap_cells"
        if self.grid[row, col] < 0:
            return "goal_unknown"
        max_cost = int(np.max(window))
        if max_cost >= self.goal_clearance_cost_min:
            return f"goal_near_high_cost_{max_cost}"
        return ""

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        if self.latest_map is None:
            return None
        col = int(math.floor((x - self.latest_map.info.origin.position.x) / self.latest_map.info.resolution))
        row = int(math.floor((y - self.latest_map.info.origin.position.y) / self.latest_map.info.resolution))
        if row < 0 or col < 0 or row >= self.latest_map.info.height or col >= self.latest_map.info.width:
            return None
        return row, col

    def _cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        assert self.latest_map is not None
        x = self.latest_map.info.origin.position.x + (col + 0.5) * self.latest_map.info.resolution
        y = self.latest_map.info.origin.position.y + (row + 0.5) * self.latest_map.info.resolution
        return float(x), float(y)

    @staticmethod
    def _dilate_bool(mask: np.ndarray, radius: int) -> np.ndarray:
        result = np.zeros_like(mask, dtype=bool)
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
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

    @staticmethod
    def _neighbors4(row: int, col: int):
        yield row - 1, col
        yield row + 1, col
        yield row, col - 1
        yield row, col + 1

    @staticmethod
    def _neighbors8(row: int, col: int):
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr != 0 or dc != 0:
                    yield row + dr, col + dc

    def _publish_frontiers(self, candidates: list[Candidate]) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for candidate in candidates[: self.max_frontiers]:
            pose = Pose()
            pose.position.x = candidate.x
            pose.position.y = candidate.y
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.frontier_pub.publish(msg)

    def _publish_selected_goal(self, pose: PoseStamped) -> None:
        self.selected_goal_pub.publish(pose)

    def _publish_status(self, state: str = "running") -> None:
        summary = {
            "state": "finished" if self.finished else state,
            "mission_mode": self.mission_mode,
            "goals": self.goal_count,
            "successes": self.success_count,
            "failures": self.failure_count,
            "precheck_rejections": self.precheck_rejection_count,
            "safety_cancellations": self.safety_cancellation_count,
            "returning_home": self.returning_home,
            "active_goal": self.active_goal,
            "active_goal_kind": self.active_goal_kind,
            "active_goal_distance": self.active_goal_distance,
            "active_goal_timeout_sec": self.active_goal_timeout_sec,
            "pending_goal_check": self.pending_goal_check,
            "coverage_gain_fraction": self._coverage_goal_progress() if self.start_known_cells > 0 else 0.0,
            "raw_known_coverage_gain_fraction": self._coverage_gain_fraction() if self.start_known_cells > 0 else 0.0,
            "mission_cell_size_m": self.mission_cell_size_m,
            "mission_total_cells": self.latest_mission_total_cells,
            "mission_reachable_cells": self.latest_mission_reachable_cells,
            "mission_max_reachable_cells": self.latest_mission_max_reachable_cells,
            "mission_covered_cells": self.latest_mission_covered_cells,
            "mission_observed_cells": self.latest_mission_observed_cells,
            "mission_visited_cells": self.latest_mission_visited_cells,
            "mission_coverage_fraction": self.latest_mission_coverage_fraction,
            "mission_observed_fraction": self.latest_mission_observed_fraction,
            "best_sample": self.best_sample,
            "best_sample_resource": self.best_sample_resource,
            "last_resource_sample": self.last_resource_sample,
            "blacklist_size": len(self.blacklist),
            "recent_goal_size": len(self.recent_goals),
            "breadcrumbs": len(self.breadcrumbs),
            "return_waypoints": len(self.return_waypoints),
            "reachable_cells": self.latest_reachable_cells,
            "resource_candidates": self.latest_resource_candidates,
        }
        self.status_pub.publish(String(data=json.dumps(summary, sort_keys=True)))


def main() -> None:
    rclpy.init()
    node = ExplorationManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
