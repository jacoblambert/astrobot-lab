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
import rclpy.duration
import rclpy.time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String
from astrobot_core.mission import ActionGeneration, home_reached
from astrobot_interfaces.msg import LocalizationStatus
from exploration_manager.reachability import clearance_components, component_near
from scipy.ndimage import distance_transform_edt
from tf2_ros import Buffer, TransformListener


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
        self.teleop_goal_topic = self.declare_parameter("teleop_goal_topic", "teleop/goal").value
        self.frame_id = self.declare_parameter("frame_id", "map").value

        self.base_offset = Pose2D(
            self.declare_parameter("base_offset_x", 0.264).value,
            self.declare_parameter("base_offset_y", 0.017).value,
            self.declare_parameter("base_yaw_offset", math.pi).value,
        )
        self.home_radius_m = float(self.declare_parameter("home_radius_m", 8.0).value)
        # Home-reached tolerance for the RETURN phase: declare the return successful when the rover
        # is within this of home, even if it can't reach exact (0,0) -- e.g. a rock right next to
        # spawn blocks the last ~0.7m (observed seed7: stuck at 0.67m -> return_failed -> gate FAIL
        # though best_sample was 0.86). More lenient than near_goal_success_radius_m (0.65), kept
        # under the gate's home_tol (1.0m) so a mission-declared "home" still GT-scores as home.
        self.home_reached_radius_m = float(self.declare_parameter("home_reached_radius_m", 0.9).value)
        # > 0: if a mapped obstacle lies within home_approach_clearance_m of home (seed 7 spawns
        # over a rock, so Smac can never end a path there), return to the nearest known-free cell
        # with that clearance within this radius instead. Keep radius + Nav2 xy goal tolerance
        # under home_reached_radius_m, which a return_home success must still satisfy.
        self.home_approach_radius_m = float(self.declare_parameter("home_approach_radius_m", 0.0).value)
        self.home_approach_clearance_m = float(self.declare_parameter("home_approach_clearance_m", 0.5).value)
        # Optional NavigateToPose behavior tree for return_home (v2: a tighter goal checker).
        self.return_behavior_tree = str(self.declare_parameter("return_behavior_tree", "").value)
        self.coverage_goal_fraction = float(self.declare_parameter("coverage_goal_fraction", 0.30).value)
        self.mission_visit_margin_m = float(self.declare_parameter("mission_visit_margin_m", 0.0).value)
        self.sample_success_threshold = float(self.declare_parameter("sample_success_threshold", 0.75).value)
        self.max_goals = int(self.declare_parameter("max_goals", 500).value)
        self.max_duration_sec = float(self.declare_parameter("max_duration_sec", 900.0).value)
        self.goal_timeout_sec = float(self.declare_parameter("goal_timeout_sec", 120.0).value)
        self.goal_timeout_per_meter_sec = float(self.declare_parameter("goal_timeout_per_meter_sec", 18.0).value)
        self.goal_timeout_max_sec = float(self.declare_parameter("goal_timeout_max_sec", 300.0).value)
        self.cycle_period_sec = float(self.declare_parameter("cycle_period_sec", 2.0).value)
        self.near_goal_success_radius_m = float(self.declare_parameter("near_goal_success_radius_m", 0.65).value)

        self.free_cost_max = int(self.declare_parameter("free_cost_max", 35).value)
        self.lethal_cost_min = int(self.declare_parameter("lethal_cost_min", 80).value)
        self.min_frontier_cells = int(self.declare_parameter("min_frontier_cells", 4).value)
        self.max_frontiers = int(self.declare_parameter("max_frontiers", 80).value)
        self.goal_clearance_cells = int(self.declare_parameter("goal_clearance_cells", 3).value)
        self.goal_clearance_radius_m = float(self.declare_parameter("goal_clearance_radius_m", 0.75).value)
        # Reveal: a front-mounted LiDAR cannot see the ground beside a stationary rover, so a local
        # map (v2) starts, and restarts after a localization reset, with nothing reachable. Send a
        # short goal (ahead, left, right, back) through that blind ring instead of ending the
        # mission with no_candidates; Nav2 still checks it. 0 disables (legacy BEV map).
        self.reveal_distance_m = float(self.declare_parameter("reveal_distance_m", 0.0).value)
        # > 0: when the rover's own cell lacks goal clearance, seed reachability from clear cells
        # within this distance through known-free cells (v2: unseen patches beside the rover).
        self.reachable_seed_distance_m = float(self.declare_parameter("reachable_seed_distance_m", 0.0).value)
        self.reveal_max_attempts = int(self.declare_parameter("reveal_max_attempts", 4).value)
        self.reveal_attempts = 0
        self.goal_clearance_cost_min = int(self.declare_parameter("goal_clearance_cost_min", 80).value)
        self.max_goal_distance_m = float(self.declare_parameter("max_goal_distance_m", 5.0).value)
        self.blacklist_radius_m = float(self.declare_parameter("blacklist_radius_m", 0.75).value)
        self.blacklist_timeout_sec = float(self.declare_parameter("blacklist_timeout_sec", 0.0).value)
        self.stall_blacklist_radius_m = float(self.declare_parameter("stall_blacklist_radius_m", 0.85).value)
        self.stall_blacklist_timeout_sec = float(
            self.declare_parameter("stall_blacklist_timeout_sec", 180.0).value
        )
        self.precheck_reject_radius_m = float(self.declare_parameter("precheck_reject_radius_m", 0.35).value)
        self.precheck_reject_timeout_sec = float(
            self.declare_parameter("precheck_reject_timeout_sec", 60.0).value
        )
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
        # Resource gradient-ascent bias (sample_return): with POINT-only resource sensing a
        # far frontier toward the peak scores ~0 (never sensed near it), so the rover only
        # inches between already-sensed cells. This biases frontier selection toward the
        # direction of increasing SENSED resource (estimated from the sparse sensed field),
        # letting it commit UP-gradient toward the unseen peak. 0 disables.
        self.resource_gradient_weight = float(self.declare_parameter("resource_gradient_weight", 10.0).value)
        self.resource_gradient_radius_m = float(self.declare_parameter("resource_gradient_radius_m", 6.0).value)
        # Distance-projected gradient COMMIT (sample_return Tier 1): once a gradient exists, reward
        # going FAR up-gradient toward the unseen peak (gradient_bias * distance) instead of inching
        # cell-by-cell. Also weights the synthesized straight-up-gradient goal. 0 disables the commit.
        self.resource_gradient_commit_weight = float(
            self.declare_parameter("resource_gradient_commit_weight", 6.0).value
        )
        # Mid-flight re-evaluation (sample_return): while driving to a far goal, keep re-running goal
        # selection each cycle so (a) the exploration overlays update as-you-go (not only after the
        # goal) and (b) we PREEMPT toward a clearly-better goal if the gradient shifts as new samples
        # arrive. Preemption is conservative -- only after a cooldown, only if the new best is a
        # different spot AND beats the active goal's score by a margin -- to avoid goal thrashing.
        self.replan_while_active = bool(self.declare_parameter("replan_while_active", True).value)
        self.replan_cooldown_sec = float(self.declare_parameter("replan_cooldown_sec", 12.0).value)
        self.replan_min_shift_m = float(self.declare_parameter("replan_min_shift_m", 2.0).value)
        self.replan_margin = float(self.declare_parameter("replan_margin", 8.0).value)
        # Far-from-home penalty (sample_return acquisition): prefer exploring NEAR home
        # rather than wandering far out when there's no resource signal yet (keeps the
        # rover gathering samples locally so the gradient engages early, and bounds the
        # return trip). Kept small so the resource_score + gradient terms dominate it once
        # a peak is sensed and pull the rover outward toward it.
        self.home_penalty_weight = float(self.declare_parameter("home_penalty_weight", 0.5).value)
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
        self.planner_precheck_max_unknown_fraction = float(
            self.declare_parameter("planner_precheck_max_unknown_fraction", 0.02).value
        )
        # Stage 2 (precheck anti-trap). The flat 0.02 unknown gate TRAPS the rover at spawn: every path
        # into unexplored space crosses >2% unknown -> rejected -> 800+ rejects -> no_candidates near
        # spawn. But exploration MUST enter unknown. Fix = DISTANCE-SCALE the unknown tolerance: NEAR
        # goals are permissive (the rover maps + re-plans as it drives the short hop -> low risk), FAR
        # goals stay strict (planner_precheck_max_unknown_fraction above is the far/strict bound). Plus a
        # consecutive-rejection backstop: after N rejects in a row, force-send the best candidate WITHOUT
        # precheck to escape the trap (reset on any dispatched goal).
        self.planner_precheck_unknown_near_fraction = float(
            self.declare_parameter("planner_precheck_unknown_near_fraction", 0.15).value
        )
        self.planner_precheck_unknown_near_dist_m = float(
            self.declare_parameter("planner_precheck_unknown_near_dist_m", 2.0).value
        )
        self.planner_precheck_unknown_far_dist_m = float(
            self.declare_parameter("planner_precheck_unknown_far_dist_m", 4.0).value
        )
        self.planner_precheck_force_move_count = int(
            self.declare_parameter("planner_precheck_force_move_count", 10).value
        )
        self.breadcrumb_return_enabled = bool(self.declare_parameter("breadcrumb_return_enabled", True).value)
        self.use_through_poses_return = bool(self.declare_parameter("use_through_poses_return", False).value)
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
        self.started_at = self._now()
        self._generation = ActionGeneration()
        self.require_localization = bool(self.declare_parameter("require_localization", True).value)
        self._health = None
        self._health_wall = 0.0
        self._resume_goal = None
        self._pending_return_reason = None
        self._map_epoch = 0
        self._map_minimum_stamp = 0.0
        self._component_cache = None
        self.goal_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.precheck_rejection_count = 0
        self.consecutive_precheck_rejects = 0  # Stage 2: streak since last dispatched goal (force-move backstop)
        self._force_next_goal = False  # Stage 2: bypass precheck on the next goal to escape a spawn trap
        self.safety_cancellation_count = 0
        self.returning_home = False
        self.return_reason = ""
        self.finished = False
        self.mission_failed = False
        self.active_goal = False
        self.active_goal_handle = None
        self.active_goal_started_at = 0.0
        self.active_goal_timeout_sec = 0.0
        self.active_goal_distance = 0.0
        self.active_goal_pose: Pose2D | None = None
        self.active_goal_kind = ""
        self.active_goal_score = 0.0
        self.active_goal_cancel_reason = ""
        self.active_goal_cancel_stall_pose: Pose2D | None = None
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
        self.stall_blacklist: list[tuple[float, float, float]] = []
        self.precheck_rejects: list[tuple[float, float, float]] = []
        self.recent_goals: list[tuple[float, float, float]] = []
        self.resource_maps: dict[str, np.ndarray] = {}
        self.last_resource_sample: dict[str, float] = {}
        self.latest_reachable_cells = 0
        self.latest_resource_candidates = 0

        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 5)
        self.create_subscription(Path, "plan", self._path_cb, 5)
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 20)
        self.create_subscription(PoseStamped, self.teleop_goal_topic, self._teleop_goal_cb, 10)
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
        self.events_pub = self.create_publisher(String, "exploration/events", 20)
        self.through_poses_action_name = self.declare_parameter(
            "through_poses_action_name", "navigate_through_poses"
        ).value
        self.slam_watchdog_enabled = bool(self.declare_parameter("slam_watchdog_enabled", True).value)
        self.slam_watchdog_max_offset_m = float(self.declare_parameter("slam_watchdog_max_offset_m", 2.0).value)
        self.slam_odom_frame = self.declare_parameter("slam_odom_frame", "slam_odom").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        # Execution monitoring. The legacy distance-timeout + mid-flight path-quality
        # safety-cancel were band-aids for a flaky controller; default OFF now that
        # MPPI + velocity smoothing handle execution. Genuine stalls are caught by
        # NavigateToPose feedback (no distance progress / excessive recoveries) and
        # Nav2's own BT recoveries.
        self.legacy_safety_cancel = bool(self.declare_parameter("legacy_safety_cancel", False).value)
        self.progress_monitor_enabled = bool(self.declare_parameter("progress_monitor_enabled", True).value)
        self.progress_stall_timeout_sec = float(self.declare_parameter("progress_stall_timeout_sec", 15.0).value)
        self.progress_max_recoveries = int(self.declare_parameter("progress_max_recoveries", 4).value)
        self.goal_max_runtime_sec = float(self.declare_parameter("goal_max_runtime_sec", 240.0).value)
        self.motion_stall_timeout_sec = float(self.declare_parameter("motion_stall_timeout_sec", 12.0).value)
        self.motion_stall_cmd_linear_min = float(self.declare_parameter("motion_stall_cmd_linear_min", 0.05).value)
        self.motion_stall_cmd_angular_min = float(self.declare_parameter("motion_stall_cmd_angular_min", 0.15).value)
        self.motion_stall_odom_linear_max = float(self.declare_parameter("motion_stall_odom_linear_max", 0.015).value)
        self.motion_stall_odom_angular_max = float(self.declare_parameter("motion_stall_odom_angular_max", 0.04).value)
        self.motion_stall_msg_timeout_sec = float(self.declare_parameter("motion_stall_msg_timeout_sec", 2.0).value)
        self.active_goal_best_distance = float("inf")
        self.active_goal_last_progress_at = 0.0
        self.active_goal_recoveries = 0
        self.latest_cmd_vel: Twist | None = None
        self.latest_cmd_vel_at = 0.0
        self.latest_odom: Odometry | None = None
        self.latest_odom_at = 0.0
        self.motion_stall_started_at = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LocalizationStatus, "localization/status", self._health_cb, 10)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(Odometry, "odom", self._odom_cb, 20)

        self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        self.planner_client = ActionClient(self, ComputePathToPose, self.planner_action_name)
        self.return_home_started_at = 0.0
        self.create_timer(self.cycle_period_sec, self._cycle)
        self.get_logger().info(
            f"exploration manager mode={self.mission_mode} map={self.map_topic} action={self.action_name}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _motion_allowed(self) -> bool:
        return (self._health is not None and self._health.motion_permitted and
                time.monotonic() - self._health_wall < 0.5)

    def _health_cb(self, msg: LocalizationStatus) -> None:
        self._health, self._health_wall = msg, time.monotonic()
        if msg.epoch != self._map_epoch:
            self._map_epoch = msg.epoch
            self._map_minimum_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.grid = self.latest_map = None
            self._component_cache = None
        if not msg.motion_permitted:
            if self.active_goal:
                self._resume_goal = (self.active_goal_pose, self.active_goal_kind, None)
                self.active_goal_cancel_reason = "localization_hold"
                self.cancel_requested = True
                if self.active_goal_handle is not None:
                    self.active_goal_handle.cancel_goal_async()
                self._clear_active_goal()
            elif self.pending_goal_check:
                self._resume_goal = (self.pending_goal_check_pose, self.pending_goal_check_kind,
                                     self.pending_goal_check_candidate)
            self.pending_goal_check = False
            self._generation.advance()
        if msg.state == LocalizationStatus.FAULTED:
            self._resume_goal = None
            self._mark_mission_failed("localization_fault")

    def _guard(self, generation, callback):
        def guarded(future):
            if not self._generation.current(generation):
                try:
                    handle = future.result()
                    if getattr(handle, "accepted", False):
                        handle.cancel_goal_async()
                except Exception:
                    pass
                return
            try:
                callback(future)
            except Exception as exc:
                self.get_logger().error(f"action callback failed: {exc}")
                returning = self.returning_home
                self.pending_goal_check = False
                self._clear_active_goal()
                if returning:
                    self._mark_mission_failed("return_failed")
        return guarded

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
        if msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 < self._map_minimum_stamp:
            return
        self._component_cache = None
        self.latest_map = msg
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        for name in self.resource_names:
            if name not in self.resource_maps or self.resource_maps[name].shape != self.grid.shape:
                self.resource_maps[name] = np.full(self.grid.shape, np.nan, dtype=np.float32)

    def _path_cb(self, msg: Path) -> None:
        self.latest_path = msg

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self.latest_cmd_vel = msg
        self.latest_cmd_vel_at = self._now()

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_at = self._now()

    def _teleop_goal_cb(self, msg: PoseStamped) -> None:
        pose = Pose2D(msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(msg.pose.orientation))
        if self.mission_mode != "teleop_click":
            self.get_logger().warn(
                f"ignoring teleop goal while mission_mode={self.mission_mode}: x={pose.x:.2f} y={pose.y:.2f}"
            )
            self._publish_event("teleop_goal_rejected", kind="teleop_goal", pose=pose, reason="wrong_mission_mode")
            return
        if self.finished:
            self._publish_event("teleop_goal_rejected", kind="teleop_goal", pose=pose, reason="mission_finished")
            return
        if self.pending_goal_check or self.active_goal or self.returning_home:
            self._publish_event("teleop_goal_rejected", kind="teleop_goal", pose=pose, reason="busy")
            return
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self._publish_event(
                "teleop_goal_rejected",
                kind="teleop_goal",
                pose=pose,
                reason=f"unsupported_frame:{msg.header.frame_id}",
            )
            return
        if self.latest_pose is not None and abs(pose.yaw) < 1e-9:
            pose = Pose2D(pose.x, pose.y, self.latest_pose.yaw)
        self.get_logger().info(f"teleop goal x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.2f}")
        self._publish_event("teleop_goal_received", kind="teleop_goal", pose=pose)
        self._send_goal(pose, "teleop_goal", None)

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
        if self.finished:
            self._publish_status()
            return
        if self.require_localization and not self._motion_allowed():
            self._publish_status(state="localization_hold")
            return
        if self.home_pose is not None and not self.returning_home and self.mission_mode != "teleop_click":
            if self._now() - self.started_at >= self.max_duration_sec:
                self._pending_return_reason = "budget_exhausted"
            elif self.mission_mode == "sample_return" and self.best_sample >= self.sample_success_threshold:
                self._pending_return_reason = "resource_found"
            if self._pending_return_reason is not None:
                self._resume_goal = None
                if self.pending_goal_check:
                    self._generation.advance()
                    self.pending_goal_check = False
                if self.active_goal:
                    if not self.cancel_requested:
                        self.cancel_requested = True
                        self.active_goal_cancel_reason = "mission_return"
                        if self.active_goal_handle is not None:
                            self.active_goal_handle.cancel_goal_async()
                    self._publish_status(state="canceling_for_return")
                    return  # Wait for completion/cancellation before sending home.
                reason, self._pending_return_reason = self._pending_return_reason, None
                self._send_return_home(reason)
                return
        if self._resume_goal is not None and not self.active_goal and not self.pending_goal_check:
            pose, kind, candidate = self._resume_goal
            self._resume_goal = None
            self._send_goal(pose, kind, candidate)
            return
        if self.pending_goal_check:
            self._check_planner_precheck_timeout()
            self._publish_status()
            return
        if self.active_goal:
            if self.legacy_safety_cancel:
                self._check_active_goal_safety()
                self._check_goal_timeout()
            if self.progress_monitor_enabled:
                self._check_goal_progress()
            self._maybe_replan_active()
            self._publish_status()
            return
        if self.finished:
            self._publish_status()
            return
        if self.returning_home:
            self._publish_status()
            return
        if self.latest_map is None or self.grid is None or self.latest_pose is None or self.home_pose is None:
            self._publish_status(state="waiting_for_inputs")
            return
        if self.start_known_cells == 0:
            self.start_known_cells = self._known_cells_in_radius()
        if self.mission_mode == "teleop_click":
            self._publish_frontiers([])
            if self.slam_watchdog_enabled and self._slam_diverged():
                self._publish_status(state="slam_diverged")
                return
            self._publish_status(state="awaiting_command")
            return

        elapsed = self._now() - self.started_at
        if elapsed >= self.max_duration_sec or self.goal_count >= self.max_goals:
            self._send_return_home("budget_exhausted")
            return
        if self.slam_watchdog_enabled and self._slam_diverged():
            self._send_return_home("slam_diverged")
            return
        if self.mission_mode == "sample_return" and self.best_sample >= self.sample_success_threshold:
            self._send_return_home("resource_found")
            return
        if self.mission_mode == "explore_radius" and self._coverage_goal_progress() >= self.coverage_goal_fraction:
            self._send_return_home("coverage_goal_met")
            return

        candidates = self._find_candidates()
        self._publish_frontiers(candidates)
        if candidates:
            self.reveal_attempts = 0
        if not candidates:
            if self._send_reveal_goal():
                return
            self._send_return_home("no_candidates")
            return
        best = candidates[0]
        self._send_goal(Pose2D(best.x, best.y, self.latest_pose.yaw), "explore", best)

    def _send_reveal_goal(self) -> bool:
        """No candidates (blind ring, or only a small seeded pocket is reachable): a short goal into
        the unseen ring, not the end of the mission. Bounded per stuck episode; the counter resets
        once candidates exist (a reachable pocket without candidates ended seed 411, Oct 28)."""
        if self.reveal_distance_m <= 0.0 or self.reveal_attempts >= self.reveal_max_attempts:
            return False
        offset = (0.0, math.pi / 2, -math.pi / 2, math.pi)[self.reveal_attempts % 4]
        self.reveal_attempts += 1
        yaw = self.latest_pose.yaw + offset  # latest_pose is base_link (composed in _pose_cb)
        pose = Pose2D(self.latest_pose.x + self.reveal_distance_m * math.cos(yaw),
                      self.latest_pose.y + self.reveal_distance_m * math.sin(yaw), yaw)
        cell = self._world_to_cell(pose.x, pose.y)
        if cell is not None and self.grid[cell] >= self.lethal_cost_min:
            return self._send_reveal_goal()
        self._publish_event("reveal_goal", kind="reveal", pose=pose, attempt=self.reveal_attempts)
        self._send_goal(pose, "reveal", None)
        return True

    def _send_return_home(self, reason: str) -> None:
        if self.returning_home or self.home_pose is None:
            return
        self.returning_home = True
        self.return_reason = reason
        self.return_waypoints = []
        self.return_home_started_at = self._now()
        self._send_goal(self._home_goal_pose(), "return_home", None)

    def _home_goal_pose(self) -> Pose2D:
        home = self.home_pose
        if self.home_approach_radius_m <= 0.0 or self.grid is None or self.latest_map is None:
            return home
        cell = self._world_to_cell(home.x, home.y)
        if cell is None:
            return home
        res = self.latest_map.info.resolution
        reach = int(math.ceil((self.home_approach_radius_m + self.home_approach_clearance_m) / res)) + 1
        r0, c0 = max(0, cell[0] - reach), max(0, cell[1] - reach)
        window = self.grid[r0:cell[0] + reach + 1, c0:cell[1] + reach + 1]
        clearance = distance_transform_edt(window < self.lethal_cost_min) * res
        rows, cols = np.indices(window.shape)
        xs = self.latest_map.info.origin.position.x + (cols + c0 + 0.5) * res
        ys = self.latest_map.info.origin.position.y + (rows + r0 + 0.5) * res
        offset = np.hypot(xs - home.x, ys - home.y)
        if clearance[cell[0] - r0, cell[1] - c0] > self.home_approach_clearance_m:
            return home
        ok = ((window >= 0) & (window <= self.free_cost_max) & (clearance > self.home_approach_clearance_m)
              & (offset <= self.home_approach_radius_m))
        if not ok.any():
            return home
        best = np.unravel_index(np.argmin(np.where(ok, offset, np.inf)), ok.shape)
        pose = Pose2D(float(xs[best]), float(ys[best]), home.yaw)
        self.get_logger().info(f"return home: obstacle at home; approach x={pose.x:.2f} y={pose.y:.2f} "
                               f"({offset[best]:.2f} m from home)")
        self._publish_event("home_approach", kind="return_home", pose=pose)
        return pose

    def _pose_is_inside_breadcrumb_radius(self, pose: Pose2D) -> bool:
        if self.home_pose is None or self.breadcrumb_max_home_radius_factor <= 0.0:
            return True
        max_radius = self.home_radius_m * self.breadcrumb_max_home_radius_factor
        return self._pose_distance(pose, self.home_pose) <= max_radius

    def _is_home_reached(self) -> bool:
        return home_reached(self.latest_pose, self.home_pose, self.home_reached_radius_m)

    def _line_of_sight_clear(self, ax: float, ay: float, bx: float, by: float) -> bool:
        """True if the straight line a->b passes only through KNOWN, low-cost, safe cells -- a
        conservative test for 'is the target directly reachable'. Samples every half cell. Used to
        decide whether to skip breadcrumbs and go straight home; if the line isn't clear we fall back
        to the (safe) breadcrumb path, so a false negative just keeps the old behaviour."""
        if self.grid is None or self.latest_map is None:
            return False
        res = self.latest_map.info.resolution
        dist = math.hypot(bx - ax, by - ay)
        steps = max(1, int(math.ceil(dist / (0.5 * res))))
        for i in range(steps + 1):
            t = i / steps
            cell = self._world_to_cell(ax + (bx - ax) * t, ay + (by - ay) * t)
            if cell is None:
                return False
            row, col = cell
            v = int(self.grid[row, col])
            if v < 0 or v > self.free_cost_max:  # unknown or too costly
                return False
            if self._near_unsafe(row, col):
                return False
        return True

    def _mark_mission_failed(self, reason: str) -> None:
        if self.mission_failed:
            return
        self.return_reason = reason
        self.mission_failed = True
        self.finished = True
        self.get_logger().warn(f"mission failed: {reason}")
        self._publish_event("mission_failed", kind="return_home", reason=reason)

    def _send_goal(self, pose: Pose2D, kind: str, candidate: Candidate | None) -> None:
        if pose is None:
            return
        if self.require_localization and not self._motion_allowed():
            self._resume_goal = (pose, kind, candidate)
            return
        if self.planner_precheck_enabled and kind not in ("return_home", "reveal"):
            self._start_planner_precheck(pose, kind, candidate)
        else:
            # Home is a normal global Nav2 goal, including its bounded recovery for blocked starts.
            self._send_nav_goal(pose, kind, candidate)

    def _send_nav_goal(self, pose: Pose2D, kind: str, candidate: Candidate | None) -> None:
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            if kind == "return_home":
                self._mark_mission_failed("return_failed")
            self._publish_status(state="action_unavailable")
            self._publish_event("action_unavailable", kind=kind, pose=pose, candidate=candidate)
            return
        self._generation.advance()
        self.active_goal = True
        self.active_goal_handle = None
        self.active_goal_started_at = self._now()
        self.active_goal_distance = self._distance_from_robot(pose)
        self.active_goal_timeout_sec = self._timeout_for_distance(self.active_goal_distance)
        self.active_goal_pose = pose
        self.active_goal_kind = kind
        self.active_goal_score = candidate.total_score if candidate is not None else 0.0
        self.active_goal_cancel_reason = ""
        self.active_goal_cancel_stall_pose = None
        self.cancel_requested = False
        self.active_goal_safety_cancelled = False
        self.active_goal_best_distance = float("inf")
        self.active_goal_last_progress_at = self._now()
        self.active_goal_recoveries = 0
        self.motion_stall_started_at = 0.0
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
        if kind == "return_home" and self.return_behavior_tree:
            goal.behavior_tree = self.return_behavior_tree
        self._publish_selected_goal(goal.pose)
        score = candidate.total_score if candidate is not None else 0.0
        self.get_logger().info(
            f"goal {self.goal_count}: {kind} x={pose.x:.2f} y={pose.y:.2f} score={score:.2f} "
            f"distance={self.active_goal_distance:.1f}m timeout={self.active_goal_timeout_sec:.0f}s"
        )
        self._publish_event(
            "goal_sent",
            kind=kind,
            pose=pose,
            candidate=candidate,
            goal_index=self.goal_count,
            distance_m=self.active_goal_distance,
            timeout_sec=self.active_goal_timeout_sec,
        )

        generation = self._generation.value
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=lambda msg: self._goal_feedback(msg) if self._generation.current(generation) else None)
        send_future.add_done_callback(self._guard(generation, lambda future, sent_at=self._now(), k=kind, p=pose: self._goal_response(future, sent_at, k, p)))

    def _goal_feedback(self, feedback_msg) -> None:
        # NavigateToPose feedback: distance_remaining, number_of_recoveries, etc.
        fb = feedback_msg.feedback
        try:
            self.active_goal_recoveries = int(fb.number_of_recoveries)
            dist = float(fb.distance_remaining)
        except AttributeError:
            return
        # Record progress whenever the remaining distance meaningfully decreases.
        if dist > 0.0 and dist < self.active_goal_best_distance - 0.05:
            self.active_goal_best_distance = dist
            self.active_goal_last_progress_at = self._now()

    def _check_goal_progress(self) -> None:
        # Nav2 owns progress and its two bounded recovery attempts. Backing away is useful motion.
        if self.cancel_requested or self.active_goal_handle is None:
            return
        if self.goal_max_runtime_sec > 0 and self._now() - self.active_goal_started_at > self.goal_max_runtime_sec:
            self.active_goal_cancel_reason = "runtime_ceiling"
            self.cancel_requested = True
            self.active_goal_handle.cancel_goal_async()

    def _maybe_replan_active(self) -> None:
        """Mid-flight re-evaluation for sample_return explore goals: re-run candidate selection each
        cycle so the exploration overlays update as-you-go (not only after the goal completes), and
        PREEMPT toward a clearly-better goal if the resource gradient shifts as new samples arrive.
        Conservative (cooldown + score margin + min spatial shift) to avoid goal thrashing; the
        preempt just cancels the goal -> the next cycle re-selects the new best."""
        if not self.replan_while_active or self.mission_mode != "sample_return":
            return
        if self.active_goal_kind != "explore" or self.cancel_requested:
            return
        if self.active_goal_handle is None or self.active_goal_pose is None:
            return
        if self.grid is None or self.latest_pose is None:
            return
        candidates = self._find_candidates()
        self._publish_frontiers(candidates)  # refresh overlays mid-flight (fixes "map updates late")
        if not candidates:
            return
        now = self._now()
        if (now - self.active_goal_started_at) < self.replan_cooldown_sec:
            return
        if self._distance_from_robot(self.active_goal_pose) <= 2.0 * self.near_goal_success_radius_m:
            return  # almost at the current goal -- let it finish
        best = candidates[0]
        shift = math.hypot(best.x - self.active_goal_pose.x, best.y - self.active_goal_pose.y)
        if shift < self.replan_min_shift_m or best.total_score < self.active_goal_score + self.replan_margin:
            return
        self.get_logger().info(
            f"mid-flight re-plan: better goal score={best.total_score:.1f} (active {self.active_goal_score:.1f}) "
            f"shifted {shift:.1f}m -> preempting"
        )
        self._publish_event(
            "goal_preempted",
            kind="explore",
            reason="replan",
            new_score=best.total_score,
            old_score=self.active_goal_score,
            shift_m=shift,
        )
        self.active_goal_cancel_reason = "replan"
        self.cancel_requested = True
        self.active_goal_handle.cancel_goal_async()

    def _motion_stall_state(self, now: float) -> tuple[bool, float]:
        if self.motion_stall_timeout_sec <= 0.0:
            self.motion_stall_started_at = 0.0
            return False, 0.0
        if self.latest_cmd_vel is None or self.latest_odom is None:
            self.motion_stall_started_at = 0.0
            return False, 0.0
        if (
            now - self.latest_cmd_vel_at > self.motion_stall_msg_timeout_sec
            or now - self.latest_odom_at > self.motion_stall_msg_timeout_sec
        ):
            self.motion_stall_started_at = 0.0
            return False, 0.0

        cmd_linear = abs(self.latest_cmd_vel.linear.x)
        cmd_angular = abs(self.latest_cmd_vel.angular.z)
        odom_linear = abs(self.latest_odom.twist.twist.linear.x)
        odom_angular = abs(self.latest_odom.twist.twist.angular.z)
        commanded_motion = (
            cmd_linear >= self.motion_stall_cmd_linear_min
            or cmd_angular >= self.motion_stall_cmd_angular_min
        )
        odom_stationary = (
            odom_linear <= self.motion_stall_odom_linear_max
            and odom_angular <= self.motion_stall_odom_angular_max
        )
        if not (commanded_motion and odom_stationary):
            self.motion_stall_started_at = 0.0
            return False, 0.0
        if self.motion_stall_started_at <= 0.0:
            self.motion_stall_started_at = now
        duration = now - self.motion_stall_started_at
        return duration >= self.motion_stall_timeout_sec, duration

    def _start_planner_precheck(self, pose: Pose2D, kind: str, candidate: Candidate | None) -> None:
        if not self.planner_client.wait_for_server(timeout_sec=1.0):
            self._publish_event("planner_unavailable", kind=kind, pose=pose)
            if kind == "return_home":
                self._mark_mission_failed("return_failed")
            return

        self._generation.advance()
        self.pending_goal_check = True
        self.pending_goal_check_started_at = self._now()
        self.pending_goal_check_pose = pose
        self.pending_goal_check_kind = kind
        self.pending_goal_check_candidate = candidate
        self._publish_event("planner_precheck_started", kind=kind, pose=pose, candidate=candidate)

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
        future.add_done_callback(self._guard(self._generation.value, self._planner_precheck_response))

    def _planner_precheck_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._reject_pending_goal_check("planner_rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._guard(self._generation.value, self._planner_precheck_result))

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
        self.consecutive_precheck_rejects = 0  # Stage 2: a passing precheck breaks the reject streak
        self._send_nav_goal(pose, kind, candidate)

    def _reject_pending_goal_check(self, reason: str) -> None:
        if self.pending_goal_check_pose is not None:
            pose = self.pending_goal_check_pose
            self.precheck_rejects.append((pose.x, pose.y, self._now()))
            self.precheck_rejection_count += 1
            self.get_logger().warn(f"planner precheck failed x={pose.x:.2f} y={pose.y:.2f}: {reason}")
            self._publish_event(
                "planner_precheck_rejected",
                kind=self.pending_goal_check_kind,
                pose=pose,
                candidate=self.pending_goal_check_candidate,
                reason=reason,
                precheck_rejections=self.precheck_rejection_count,
            )
        self._generation.advance()
        self.consecutive_precheck_rejects += 1
        self._force_next_goal = False
        if self.pending_goal_check_kind == "return_home":
            self._mark_mission_failed("return_failed")
        self.pending_goal_check = False
        self.pending_goal_check_started_at = 0.0
        self.pending_goal_check_pose = None
        self.pending_goal_check_kind = ""
        self.pending_goal_check_candidate = None

    def _check_planner_precheck_timeout(self) -> None:
        if not self.pending_goal_check or self.planner_precheck_timeout_sec <= 0.0:
            return
        elapsed = self._now() - self.pending_goal_check_started_at
        if elapsed >= self.planner_precheck_timeout_sec:
            self._reject_pending_goal_check(f"timeout_{elapsed:.1f}s")

    def _goal_response(self, future, sent_at: float, kind: str, pose: Pose2D) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.failure_count += 1
            self._add_goal_blacklist(pose, "action_rejected")
            self._publish_event(
                "goal_rejected",
                kind=kind,
                pose=pose,
                reason="action_rejected",
                failures=self.failure_count,
            )
            self._clear_active_goal()
            if kind == "return_home":
                self._mark_mission_failed("return_failed")
            return
        self._publish_event("goal_accepted", kind=kind, pose=pose)
        self.active_goal_handle = goal_handle
        if self.cancel_requested:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._guard(self._generation.value, lambda result: self._goal_result(result, sent_at, kind, pose)))

    def _goal_result(self, future, sent_at: float, kind: str, pose: Pose2D) -> None:
        result = future.result()
        status = result.status
        duration = self._now() - sent_at
        cancel_reason = self.active_goal_cancel_reason
        cancel_stall_pose = self.active_goal_cancel_stall_pose
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.success_count += 1
            self._publish_event(
                "goal_succeeded",
                kind=kind,
                pose=pose,
                status=int(status),
                duration_sec=duration,
                successes=self.success_count,
            )
            if kind == "return_home":
                if self._is_home_reached():
                    self.finished = True
                else:
                    self._mark_mission_failed("return_failed")

        else:
            if cancel_reason in {"replan", "localization_hold", "mission_return"}:
                # Deliberate mid-flight preemption toward a better goal -- not a failure: don't count
                # it or blacklist the spot; the next cycle re-selects the new best.
                self.get_logger().info("goal preempted for re-plan; reselecting next cycle")
                self._publish_event("goal_preempted_done", kind=kind, pose=pose)
                self._clear_active_goal()
                self._publish_status(state="running")
                return
            if cancel_reason in {"motion_stall", "no_progress", "too_many_recoveries"}:
                self.failure_count += 1
                self._add_stall_blacklist(cancel_stall_pose, cancel_reason, pose, kind)
                self.get_logger().warn(
                    f"goal stalled kind={kind} status={status} duration={duration:.1f}s reason={cancel_reason}"
                )
                self._publish_event(
                    "goal_failed",
                    kind=kind,
                    pose=pose,
                    status=int(status),
                    duration_sec=duration,
                    failures=self.failure_count,
                    reason=cancel_reason,
                    stall_pose=asdict(cancel_stall_pose) if cancel_stall_pose is not None else {},
                )
            elif kind == "explore" and self.active_goal_safety_cancelled:
                self.safety_cancellation_count += 1
                self.get_logger().warn(f"goal safety-canceled kind={kind} status={status} duration={duration:.1f}s")
                self._publish_event(
                    "goal_safety_cancelled",
                    kind=kind,
                    pose=pose,
                    status=int(status),
                    duration_sec=duration,
                    safety_cancellations=self.safety_cancellation_count,
                )
            else:
                self.failure_count += 1
                self._add_goal_blacklist(pose, cancel_reason or f"status_{status}")
                self.get_logger().warn(f"goal failed kind={kind} status={status} duration={duration:.1f}s")
                self._publish_event(
                    "goal_failed",
                    kind=kind,
                    pose=pose,
                    status=int(status),
                    duration_sec=duration,
                    failures=self.failure_count,
                )
            if kind == "return_home":
                # The final home goal failed (can't reach exact (0,0)). If we're already within the
                # home-reached tolerance -- e.g. a rock right next to spawn blocks the last bit -- call
                # it home (good enough) rather than failing the whole mission.
                if self._is_home_reached():
                    self.finished = True
                    dist_home = self._pose_distance(self.latest_pose, self.home_pose)
                    self.get_logger().info(
                        f"return home: close enough ({dist_home:.2f}m <= {self.home_reached_radius_m:.2f}m); "
                        f"obstacle blocks exact home -- accepting as reached"
                    )
                    self._publish_event("return_home_close_enough", kind="return_home", home_dist=dist_home)
                else:
                    self.return_reason = "return_failed"
                    self._mark_mission_failed("return_failed")
        self._clear_active_goal()
        self._publish_status(state="awaiting_command" if self.mission_mode == "teleop_click" else "running")

    def _check_goal_timeout(self) -> None:
        if (
            self.cancel_requested
            or self.active_goal_handle is None
            or self.active_goal_pose is None
            or self.active_goal_timeout_sec <= 0.0
        ):
            return
        elapsed = self._now() - self.active_goal_started_at
        if elapsed < self.active_goal_timeout_sec:
            return
        self.cancel_requested = True
        pose = self.active_goal_pose
        self.active_goal_cancel_reason = "timeout"
        self.active_goal_cancel_stall_pose = self.latest_pose
        self.get_logger().warn(
            f"goal timeout kind={self.active_goal_kind} x={pose.x:.2f} y={pose.y:.2f} "
            f"after {elapsed:.1f}s; canceling"
        )
        self._publish_event(
            "goal_cancel_requested",
            kind=self.active_goal_kind,
            pose=pose,
            reason="timeout",
            runtime_sec=elapsed,
            timeout_sec=self.active_goal_timeout_sec,
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
        self._add_goal_blacklist(pose, f"safety:{reason}")
        self.active_goal_cancel_reason = f"safety:{reason}"
        self.active_goal_cancel_stall_pose = self.latest_pose
        self.cancel_requested = True
        self.active_goal_safety_cancelled = True
        self.get_logger().warn(
            f"active goal became unsafe kind={self.active_goal_kind} x={pose.x:.2f} y={pose.y:.2f}: {reason}; canceling"
        )
        self._publish_event(
            "goal_cancel_requested",
            kind=self.active_goal_kind,
            pose=pose,
            reason=f"safety:{reason}",
        )
        self.active_goal_handle.cancel_goal_async()

    def _clear_active_goal(self) -> None:
        self._generation.advance()
        self.active_goal = False
        self.active_goal_handle = None
        self.active_goal_started_at = 0.0
        self.active_goal_timeout_sec = 0.0
        self.active_goal_distance = 0.0
        self.active_goal_pose = None
        self.active_goal_kind = ""
        self.active_goal_cancel_reason = ""
        self.active_goal_cancel_stall_pose = None
        self.cancel_requested = False
        self.active_goal_safety_cancelled = False

    def _slam_diverged(self) -> bool:
        # A global frame offset is not an estimator-health metric. The
        # supervisor checks timestamped motion and absolute-reference agreement.
        if self.require_localization:
            return not self._motion_allowed()
        if not self.slam_watchdog_enabled or self.slam_watchdog_max_offset_m <= 0.0:
            return False
        try:
            tf = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            dx = tf.transform.translation.x
            dy = tf.transform.translation.y
            offset = math.hypot(dx, dy)
            if offset > self.slam_watchdog_max_offset_m:
                self.get_logger().warn(
                    f"SLAM watchdog: map->odom offset={offset:.2f}m > {self.slam_watchdog_max_offset_m:.1f}m"
                )
                return True
        except Exception:
            pass
        return False

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
        return self.latest_mission_coverage_fraction

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
        if self.mission_mode == "sample_return":
            # 3-tier in-situ resource seeking (gradient -> expand-from-best-known -> search).
            return self._sample_return_candidates(reachable_free, mission_candidates)
        unknown = self.grid < 0
        frontier = unknown & self._dilate_bool(reachable_free, 1)
        frontier &= self._inside_home_radius_mask()

        clusters = self._cluster_mask(frontier, self.max_frontiers)
        # Up-gradient bias for sample_return: commit toward the rising-resource direction
        # (the unseen peak) rather than only toward already-sensed-nearby frontiers.
        grad_dir = (
            self._resource_gradient_dir()
            if (self.mission_mode == "sample_return" and self.resource_gradient_weight > 0.0)
            else None
        )
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
            gradient_bias = 0.0
            if grad_dir is not None and distance > 1e-3:
                fdx = (x - self.latest_pose.x) / distance
                fdy = (y - self.latest_pose.y) / distance
                gradient_bias = fdx * grad_dir[0] + fdy * grad_dir[1]  # [-1, 1]
            home_dist = (
                math.hypot(x - self.home_pose.x, y - self.home_pose.y)
                if self.home_pose is not None
                else 0.0
            )
            total = (
                self.info_weight * info_gain
                + self.resource_weight_scale * resource_score
                + self.resource_gradient_weight * gradient_bias
                - self.distance_weight * distance
                - self.over_distance_weight * over_distance
                - self.terrain_weight * terrain_cost
                - self.home_penalty_weight * home_dist
            )
            candidates.append(Candidate(x, y, info_gain, resource_score, terrain_cost, distance, total, len(cluster)))
        # sample_return uses a single resource-weighted frontier search: the
        # frontier candidates above already score expected resource (via
        # _resource_score_near) minus travel cost. Explore-then-exploit emerges
        # from that score (flat resource -> distance dominates -> coverage; sensed
        # resource -> bias toward the high-resource frontier), and blacklist-driven
        # rerouting (finite blacklist_timeout) walks around obstacle barriers over
        # time. No separate mission-grid or greedy-climb merge here.
        if self.mission_mode != "sample_return":
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
            if self.use_resource_candidates_in_explore_radius:
                resource_candidates = self._resource_exploitation_candidates(reachable_free, mission_occupied)
            self.latest_resource_candidates = len(resource_candidates)
            candidates.extend(resource_candidates)
        else:
            self.latest_resource_candidates = 0
        candidates.sort(key=lambda c: c.total_score, reverse=True)
        return candidates

    def _sample_return_candidates(
        self, reachable_free: np.ndarray, mission_candidates: list[Candidate]
    ) -> list[Candidate]:
        """3-tier in-situ resource seeking (point-sensing only, NO ground-truth map):
        1) a gradient is available  -> follow it toward the unseen peak;
        2) some resource sensed but no gradient yet -> greedily expand from the highest-known
           sensed point into its unexplored reachable neighbours (climb + densify samples so a
           gradient engages);
        3) nothing usable (dead-zone / everything above filtered) -> systematic coverage search
           outward from home until a sample is sensed.
        Each tier yields MULTIPLE ranked candidates so a single precheck rejection reroutes next
        cycle instead of ending the mission with no_candidates."""
        self.latest_resource_candidates = 0
        grad_dir = self._resource_gradient_dir() if self.resource_gradient_weight > 0.0 else None

        # TIER 1 -- follow the strongest gradient toward the unseen peak.
        if grad_dir is not None:
            tier1 = self._gradient_candidates(reachable_free, grad_dir)
            if tier1:
                return tier1

        # TIER 2 -- greedily expand from the best-known sensed point(s).
        if self._has_any_sensed_resource():
            tier2 = self._resource_exploitation_candidates(reachable_free, set())
            if tier2:
                tier2.sort(key=lambda c: c.total_score, reverse=True)
                self.latest_resource_candidates = len(tier2)
                return tier2

        # TIER 3 -- dead-zone / nothing reachable above: systematic outward search from home
        # (reuses the proven mission-grid coverage) so the rover keeps moving + sampling until a
        # gradient (Tier 1) or a best-known point (Tier 2) engages -- never a premature no_candidates.
        if mission_candidates:
            return mission_candidates

        # Last resort: raw frontier (keep moving rather than declaring no_candidates).
        return self._frontier_candidates_sample_return(reachable_free, grad_dir)

    def _has_any_sensed_resource(self) -> bool:
        return any(bool(np.any(np.isfinite(g))) for g in self.resource_maps.values())

    def _gradient_candidates(
        self, reachable_free: np.ndarray, grad_dir: "tuple[float, float]"
    ) -> list[Candidate]:
        """Tier 1: frontier candidates scored with the gradient + distance-projected commit term,
        plus a synthesized straight-up-gradient goal so the rover commits toward the peak even when
        no frontier happens to lie up-gradient (avoids inching/stalling)."""
        cands = self._frontier_candidates_sample_return(reachable_free, grad_dir)
        synth = self._upgradient_goal(reachable_free, grad_dir)
        if synth is not None:
            cands.append(synth)
        cands.sort(key=lambda c: c.total_score, reverse=True)
        return cands

    def _upgradient_goal(
        self, reachable_free: np.ndarray, grad_dir: "tuple[float, float]"
    ) -> "Candidate | None":
        """A committing goal one max-step straight up-gradient from the rover, snapped to a safe
        reachable cell (so its planned path stays in known space -> better precheck odds)."""
        if self.latest_pose is None or self.grid is None:
            return None
        tx = self.latest_pose.x + grad_dir[0] * self.max_goal_distance_m
        ty = self.latest_pose.y + grad_dir[1] * self.max_goal_distance_m
        staged = self._staged_goal_cell(tx, ty, reachable_free)
        if staged is None:
            return None
        row, col = staged
        x, y = self._cell_to_world(row, col)
        if self._is_blacklisted(x, y) or self._near_unsafe(row, col):
            return None
        terrain_cost = float(max(0, self.grid[row, col]))
        if terrain_cost >= self.lethal_cost_min:
            return None
        distance = math.hypot(x - self.latest_pose.x, y - self.latest_pose.y)
        if not self._goal_is_worth_sending(x, y, distance):
            return None
        resource_score = self._resource_score_near(row, col)
        total = (
            self.resource_weight_scale * resource_score
            + self.resource_gradient_weight  # bias == 1 by construction (straight up-gradient)
            + self.resource_gradient_commit_weight * min(distance, self.max_goal_distance_m)
            - self.distance_weight * distance
            - self.terrain_weight * terrain_cost
        )
        return Candidate(x, y, 0.0, resource_score, terrain_cost, distance, total, 1)

    def _frontier_candidates_sample_return(
        self, reachable_free: np.ndarray, grad_dir: "tuple[float, float] | None"
    ) -> list[Candidate]:
        """Resource-weighted frontier search for sample_return -- mirrors the explore_radius
        frontier loop in _find_candidates, plus the distance-projected gradient commit term when a
        gradient exists. Kept separate so the explore_radius path stays untouched."""
        assert self.grid is not None and self.latest_pose is not None
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
            gradient_bias = 0.0
            if grad_dir is not None and distance > 1e-3:
                fdx = (x - self.latest_pose.x) / distance
                fdy = (y - self.latest_pose.y) / distance
                gradient_bias = fdx * grad_dir[0] + fdy * grad_dir[1]  # [-1, 1]
            home_dist = (
                math.hypot(x - self.home_pose.x, y - self.home_pose.y)
                if self.home_pose is not None
                else 0.0
            )
            total = (
                self.info_weight * info_gain
                + self.resource_weight_scale * resource_score
                + self.resource_gradient_weight * gradient_bias
                - self.distance_weight * distance
                - self.over_distance_weight * over_distance
                - self.terrain_weight * terrain_cost
                - self.home_penalty_weight * home_dist
            )
            if grad_dir is not None:
                total += self.resource_gradient_commit_weight * gradient_bias * min(
                    distance, self.max_goal_distance_m
                )
            candidates.append(
                Candidate(x, y, info_gain, resource_score, terrain_cost, distance, total, len(cluster))
            )
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

        if self.mission_mode == "explore_radius":
            # Stage 3: greedy NEAREST-UNVISITED sweep instead of the global total_score sort. The rover
            # takes candidates[0] each cycle, so a global-score sort made it teleport to the highest-score
            # cell anywhere in the radius -> long back-and-forth (path_efficiency ~0.32) -> coverage starved
            # in the time budget. Distance ascending (bucketed to 0.5m so near-equidistant cells fall back
            # to score) yields a contiguous lawnmower-ish sweep that expands outward from home. Coverage,
            # not resource, is the explore_radius objective, so resource/terrain only break ties.
            candidates.sort(key=lambda c: (round(c.distance / 0.5), -c.total_score))
        else:
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
        coverage_denominator = total_cells
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
        for radius in range(max_radius_cells + 1):
            best: tuple[int, int] | None = None
            best_key: tuple[float, ...] | None = None
            r0 = max(0, row - radius)
            r1 = min(known_free.shape[0], row + radius + 1)
            c0 = max(0, col - radius)
            c1 = min(known_free.shape[1], col + radius + 1)
            for rr in range(r0, r1):
                for cc in range(c0, c1):
                    if not known_free[rr, cc] or self._near_unsafe(rr, cc):
                        continue
                    key = self._cell_safety_key(rr, cc, row, col)
                    if best_key is None or key < best_key:
                        best = (rr, cc)
                        best_key = key
            if best is not None:
                return best
        return None

    def _cell_safety_key(self, row: int, col: int, target_row: int, target_col: int) -> tuple[float, ...]:
        assert self.grid is not None
        if self.latest_map is None:
            dist2 = (row - target_row) ** 2 + (col - target_col) ** 2
            return (0.0, 0.0, float(dist2))
        radius_cells = max(
            self.goal_clearance_cells,
            int(math.ceil(self.goal_clearance_radius_m / self.latest_map.info.resolution)),
        )
        r0 = max(0, row - radius_cells)
        r1 = min(self.grid.shape[0], row + radius_cells + 1)
        c0 = max(0, col - radius_cells)
        c1 = min(self.grid.shape[1], col + radius_cells + 1)
        window = self.grid[r0:r1, c0:c1]
        known = window[window >= 0]
        if known.size == 0:
            return (1000.0, 1000.0, float((row - target_row) ** 2 + (col - target_col) ** 2))
        max_cost = float(np.max(known))
        high_cells = float(np.count_nonzero(known >= self.planner_precheck_high_cost))
        mean_cost = float(np.mean(known))
        dist2 = float((row - target_row) ** 2 + (col - target_col) ** 2)
        return (max_cost, high_cells, mean_cost, dist2)

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
        target = self._world_to_cell(center_x, center_y)
        if target is None:
            target = ((r0 + r1) // 2, (c0 + c1) // 2)
        best: tuple[int, int] | None = None
        best_key: tuple[float, ...] | None = None
        for local_row, local_col in zip(rows.tolist(), cols.tolist()):
            row = r0 + local_row
            col = c0 + local_col
            if self._near_unsafe(row, col):
                continue
            x, y = self._cell_to_world(row, col)
            key = self._cell_safety_key(row, col, target[0], target[1])
            # Keep a weak preference for the mission-cell center after clearance.
            key = (*key[:3], (x - center_x) ** 2 + (y - center_y) ** 2)
            if best_key is None or key < best_key:
                best = (row, col)
                best_key = key
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
        if self._component_cache is None:
            self._component_cache = clearance_components(
                known_free, self.latest_map.info.resolution, self.goal_clearance_radius_m)
        steps = int(math.ceil(self.reachable_seed_distance_m / self.latest_map.info.resolution))
        return component_near(self._component_cache, known_free, start, steps)

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

    def _resource_gradient_dir(self) -> "tuple[float, float] | None":
        """World-frame unit direction of increasing SENSED resource near the robot,
        estimated from the sparse sensed-sample field (finite-difference of value vs.
        offset over visited cells in a window). Point-sensing means a far frontier toward
        the peak scores ~0 resource; this lets sample_return commit UP-gradient toward the
        unseen peak instead of inching cell-by-cell. Returns None if too few samples / no
        slope. (cell->world is affine with no rotation: world dx ~ Dcol, dy ~ Drow, same
        scale, so the cell-space direction maps straight to world.)"""
        if self.latest_map is None or self.latest_pose is None or not self.resource_maps:
            return None
        res = self.latest_map.info.resolution
        radius = max(2, int(math.ceil(self.resource_gradient_radius_m / res)))
        cell = self._world_to_cell(self.latest_pose.x, self.latest_pose.y)
        if cell is None:
            return None
        r0c, c0c = cell
        pr: list[np.ndarray] = []
        pc: list[np.ndarray] = []
        pv: list[np.ndarray] = []
        for name, grid in self.resource_maps.items():
            weight = self.resource_weights.get(name, 1.0)
            r0 = max(0, r0c - radius)
            r1 = min(grid.shape[0], r0c + radius + 1)
            c0 = max(0, c0c - radius)
            c1 = min(grid.shape[1], c0c + radius + 1)
            win = grid[r0:r1, c0:c1]
            mask = ~np.isnan(win)
            if int(np.count_nonzero(mask)) < 3:
                continue
            rr, cc = np.nonzero(mask)
            pr.append(rr + r0)
            pc.append(cc + c0)
            pv.append(win[rr, cc].astype(np.float64) * weight)
        if not pr:
            return None
        rows = np.concatenate(pr).astype(np.float64)
        cols = np.concatenate(pc).astype(np.float64)
        vals = np.concatenate(pv)
        drow = rows - float(r0c)
        dcol = cols - float(c0c)
        dist = np.hypot(drow, dcol)
        ok = dist > 1e-6
        if int(np.count_nonzero(ok)) < 3:
            return None
        drow = drow[ok]
        dcol = dcol[ok]
        dist = dist[ok]
        wv = vals[ok] - float(np.mean(vals[ok]))
        grow = float(np.sum(wv * drow / dist))
        gcol = float(np.sum(wv * dcol / dist))
        norm = math.hypot(gcol, grow)
        if norm < 1e-6:
            return None
        return (gcol / norm, grow / norm)

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
        m = self.mission_visit_margin_m
        # A pose within the localization error of a cell edge may lie in the neighbour: count the
        # cell only when the pose is this far inside it, so online coverage cannot read above GT
        # on corner-clipped cells (seeds 42/411: 37 online vs 36 GT cells at the 0.70 stop).
        if m > 0.0 and cell is not None and not (
                self._mission_cell_for_xy(pose.x - m, pose.y - m) == cell == self._mission_cell_for_xy(pose.x + m, pose.y + m)):
            return
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

    @staticmethod
    def _dedup_spatial_memory(
        memory: list[tuple[float, float, float]], x: float, y: float, radius: float
    ) -> list[tuple[float, float, float]]:
        keep_radius = max(radius * 0.5, 1.0e-3)
        return [item for item in memory if math.hypot(x - item[0], y - item[1]) > keep_radius]

    def _add_goal_blacklist(self, pose: Pose2D, reason: str) -> None:
        self.blacklist = self._dedup_spatial_memory(self.blacklist, pose.x, pose.y, self.blacklist_radius_m)
        self.blacklist.append((pose.x, pose.y, self._now()))
        self._publish_event("goal_blacklisted", kind=self.active_goal_kind or "goal", pose=pose, reason=reason)

    def _add_stall_blacklist(
        self, stall_pose: Pose2D | None, reason: str, goal_pose: Pose2D, kind: str
    ) -> None:
        if stall_pose is None:
            self._add_goal_blacklist(goal_pose, f"stall_without_pose:{reason}")
            return
        self.stall_blacklist = self._dedup_spatial_memory(
            self.stall_blacklist, stall_pose.x, stall_pose.y, self.stall_blacklist_radius_m
        )
        self.stall_blacklist.append((stall_pose.x, stall_pose.y, self._now()))
        self._publish_event(
            "stall_blacklisted",
            kind=kind,
            pose=stall_pose,
            reason=reason,
            goal_pose=asdict(goal_pose),
            stall_blacklist_size=len(self.stall_blacklist),
        )

    def _prune_blacklists(self) -> None:
        now = self._now()
        if self.blacklist_timeout_sec > 0.0:
            self.blacklist = [item for item in self.blacklist if now - item[2] <= self.blacklist_timeout_sec]
        if self.stall_blacklist_timeout_sec > 0.0:
            self.stall_blacklist = [
                item for item in self.stall_blacklist if now - item[2] <= self.stall_blacklist_timeout_sec
            ]
        if self.precheck_reject_timeout_sec > 0.0:
            self.precheck_rejects = [
                item for item in self.precheck_rejects if now - item[2] <= self.precheck_reject_timeout_sec
            ]

    def _is_blacklisted(self, x: float, y: float) -> bool:
        self._prune_blacklists()
        if any(math.hypot(x - bx, y - by) <= self.blacklist_radius_m for bx, by, _ in self.blacklist):
            return True
        if any(
            math.hypot(x - bx, y - by) <= self.stall_blacklist_radius_m
            for bx, by, _ in self.stall_blacklist
        ):
            return True
        return any(
            math.hypot(x - bx, y - by) <= self.precheck_reject_radius_m for bx, by, _ in self.precheck_rejects
        )

    def _goal_is_worth_sending(self, x: float, y: float, distance: float) -> bool:
        if distance < self.min_goal_distance_m:
            return False
        return not self._is_recent_goal(x, y)

    def _is_recent_goal(self, x: float, y: float) -> bool:
        now = self._now()
        self.recent_goals = [item for item in self.recent_goals if now - item[2] <= self.recent_goal_timeout_sec]
        return any(math.hypot(x - gx, y - gy) <= self.recent_goal_radius_m for gx, gy, _ in self.recent_goals)

    def _remember_goal(self, x: float, y: float) -> None:
        self.recent_goals.append((x, y, self._now()))

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
        # Stage 2: distance-scale the unknown tolerance. Goal distance = straight line between the planned
        # path's start (rover) and end (goal). Near goals are permissive, far goals strict; linear interp.
        try:
            p0 = path.poses[0].pose.position
            pn = path.poses[-1].pose.position
            goal_dist = math.hypot(pn.x - p0.x, pn.y - p0.y)
        except (IndexError, AttributeError):
            goal_dist = self.planner_precheck_unknown_far_dist_m
        near_d = self.planner_precheck_unknown_near_dist_m
        far_d = max(self.planner_precheck_unknown_far_dist_m, near_d + 1e-3)
        near_f = self.planner_precheck_unknown_near_fraction
        far_f = self.planner_precheck_max_unknown_fraction
        if goal_dist <= near_d:
            unknown_tol = near_f
        elif goal_dist >= far_d:
            unknown_tol = far_f
        else:
            t = (goal_dist - near_d) / (far_d - near_d)
            unknown_tol = near_f + t * (far_f - near_f)
        if unknown_fraction > unknown_tol:
            return f"path_unknown_fraction_{unknown_fraction:.2f}_tol{unknown_tol:.2f}_d{goal_dist:.1f}"
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

    def _candidate_dict(self, candidate: Candidate | None) -> dict:
        if candidate is None:
            return {}
        return {
            "x": candidate.x,
            "y": candidate.y,
            "info_gain": candidate.info_gain,
            "resource_score": candidate.resource_score,
            "terrain_cost": candidate.terrain_cost,
            "distance": candidate.distance,
            "total_score": candidate.total_score,
            "cells": candidate.cells,
        }

    def _publish_event(self, event: str, **fields) -> None:
        self._prune_blacklists()
        pose = fields.pop("pose", None)
        candidate = fields.pop("candidate", None)
        record = {
            "event": event,
            "stamp": self.get_clock().now().nanoseconds * 1e-9,
            "elapsed_sec": self._now() - self.started_at,
            "mission_mode": self.mission_mode,
            "goals": self.goal_count,
            "successes": self.success_count,
            "failures": self.failure_count,
            "precheck_rejections": self.precheck_rejection_count,
            "safety_cancellations": self.safety_cancellation_count,
            "coverage": self._coverage_goal_progress() if self.start_known_cells > 0 else 0.0,
            "mission_coverage": self.latest_mission_coverage_fraction,
            "blacklist_size": len(self.blacklist),
            "stall_blacklist_size": len(self.stall_blacklist),
            "precheck_reject_memory_size": len(self.precheck_rejects),
            **fields,
        }
        if pose is not None:
            record["pose"] = {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
        if candidate is not None:
            record["candidate"] = self._candidate_dict(candidate)
        self.events_pub.publish(String(data=json.dumps(record, sort_keys=True)))

    def _publish_status(self, state: str = "running") -> None:
        self._prune_blacklists()
        status_state = "failed" if self.mission_failed else ("finished" if self.finished else state)
        _, motion_stall_duration = self._motion_stall_state(self._now()) if self.active_goal else (False, 0.0)
        summary = {
            "state": status_state,
            "mission_mode": self.mission_mode,
            "goals": self.goal_count,
            "successes": self.success_count,
            "failures": self.failure_count,
            "precheck_rejections": self.precheck_rejection_count,
            "safety_cancellations": self.safety_cancellation_count,
            "returning_home": self.returning_home,
            "return_reason": self.return_reason,
            "active_goal": self.active_goal,
            "active_goal_kind": self.active_goal_kind,
            "active_goal_distance": self.active_goal_distance,
            "active_goal_timeout_sec": self.active_goal_timeout_sec,
            "motion_stall_duration_sec": motion_stall_duration,
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
            "stall_blacklist_size": len(self.stall_blacklist),
            "precheck_reject_memory_size": len(self.precheck_rejects),
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
