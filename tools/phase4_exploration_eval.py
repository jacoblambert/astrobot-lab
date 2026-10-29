#!/usr/bin/env python3
"""Monitor a Phase 4 exploration run and write a compact summary."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
from astrobot_core.evaluation import FixedCoverage, mission_verdict
from astrobot_interfaces.msg import LocalizationStatus
from astrobot_core.integrity import PoseSample, PoseValidator
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32, String


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def path_length(path: NavPath) -> float:
    total = 0.0
    previous = None
    for pose_stamped in path.poses:
        p = pose_stamped.pose.position
        current = (float(p.x), float(p.y))
        if previous is not None:
            total += math.hypot(current[0] - previous[0], current[1] - previous[1])
        previous = current
    return total


class Phase4ExplorationEval(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("phase4_exploration_eval", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.args = args
        self.started_at = time.monotonic()
        self.started_sim = None
        self.last_sim = None
        self.sim_duration = 0.0
        self.clock_fault = None
        self.localization_events = []
        self.last_localization_key = None
        self.initial_known = None
        self.latest_known = 0
        self.latest_gt_pose = None
        self.gt_validator = PoseValidator()
        self.reference_faults = []
        self.last_gt_received_at = None
        # The nominal home is kept for comparison with legacy gates; with --home-from-gt-start the
        # scored geometry is anchored at the first valid GT pose, as the manager anchors its own.
        self.nominal_coverage = FixedCoverage(args.home_x, args.home_y, args.coverage_radius, args.coverage_cell_size)
        self.home = None if getattr(args, "home_from_gt_start", False) else (args.home_x, args.home_y)
        self.physical_coverage = None if self.home is None else self.nominal_coverage
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
        self.events = []
        self.diagnostics = []
        self.goal_records = []
        self.active_goal_record = None
        self.latest_diagnostics = {}
        self.latest_feedback = {}
        self.latest_cmd_vel = {}
        self.latest_path_stats = {}
        self.latest_grids = {}
        self.gt_history = []
        self.snapshot_count = 0
        self.last_snapshot_at = 0.0
        self.record_dir = Path(args.record_dir) if args.record_dir else None
        self.goal_events_file = None
        self.raw_events_file = None
        self.diagnostics_file = None
        if self.record_dir:
            self.record_dir.mkdir(parents=True, exist_ok=True)
            self.goal_events_file = (self.record_dir / "goal_events.jsonl").open("w", encoding="utf-8")
            self.raw_events_file = (self.record_dir / "raw_exploration_events.jsonl").open("w", encoding="utf-8")
            self.diagnostics_file = (self.record_dir / "diagnostics.jsonl").open("w", encoding="utf-8")
        # Pose timeseries for divergence fingerprinting. Recorded IN this node (which
        # is already subscribed to GT + SLAM pose) so the study adds NO extra
        # processes -> no observer effect on CPU-bound GLIM. Each: (stamp,x,y,z,qx,qy,qz,qw).
        self.gt_ts = []
        self.slam_ts = []

        self.create_subscription(OccupancyGrid, args.bev_topic, self._bev_cb, 10)
        self.create_subscription(LocalizationStatus, args.localization_topic, self._localization_cb, 20)
        self.create_subscription(OccupancyGrid, args.gt_map_topic, lambda msg: self._grid_snapshot_cb("gt", msg), 2)
        self.create_subscription(PoseStamped, args.gt_pose_topic, self._gt_pose_cb, 10)
        self.create_subscription(PoseStamped, args.slam_pose_topic, self._slam_pose_cb, 10)
        self.create_subscription(String, args.status_topic, self._status_cb, 10)
        self.create_subscription(Float32, args.resource_sample_topic, self._sample_cb, 10)
        self.create_subscription(PoseStamped, args.resource_sample_pose_topic, self._sample_pose_cb, 10)
        self.create_subscription(PoseStamped, args.selected_goal_topic, self._selected_goal_cb, 10)
        self.create_subscription(OccupancyGrid, args.resource_map_topic, self._resource_map_cb, 10)
        self.create_subscription(String, args.event_topic, self._event_cb, 20)
        self.create_subscription(String, args.diagnostics_topic, self._diagnostics_cb, 20)
        self.create_subscription(NavPath, args.plan_topic, self._plan_cb, 5)
        self.create_subscription(Twist, args.cmd_vel_topic, self._cmd_vel_cb, 10)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            args.nav_feedback_topic,
            self._nav_feedback_cb,
            10,
        )
        for key, topic in {
            "global": args.global_costmap_topic,
            "local": args.local_costmap_topic,
        }.items():
            self.create_subscription(OccupancyGrid, topic, lambda msg, name=key: self._grid_snapshot_cb(name, msg), 2)

    def _bev_cb(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int16)
        known = int(np.count_nonzero(data >= 0))
        if self.initial_known is None and known > 0:
            self.initial_known = known
        self.latest_known = known
        self.map_times.append(time.monotonic())
        self.map_times = self.map_times[-100:]
        self._grid_snapshot_cb("bev", msg)

    @staticmethod
    def _tum_row(msg: PoseStamped):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.position, msg.pose.orientation
        return (st, p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def _gt_pose_cb(self, msg: PoseStamped) -> None:
        row = self._tum_row(msg)
        # Keep raw evidence even when invalid; a faulty reference invalidates the trial.
        if self.args.record_dir:
            self.gt_ts.append(row)
        sample = PoseSample(row[0], tuple(row[1:4]), tuple(row[4:8]))
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0 and msg.header.frame_id == "gt_map":
            return  # no /clock yet: a valid pose would read as a future stamp (seed-7 gate, Oct 27)
        reason = ("wrong_frame" if msg.header.frame_id != "gt_map" else
                  self.gt_validator.validate(sample, now))
        if reason:
            self.reference_faults.append({"stamp": row[0], "reason": reason})
            return
        previous = self.latest_gt_pose
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self.latest_gt_pose = [msg.pose.position.x, msg.pose.position.y, yaw]
        self.last_gt_received_at = time.monotonic()
        if self.home is None:
            self.home = (msg.pose.position.x, msg.pose.position.y)
            self.physical_coverage = FixedCoverage(*self.home, self.args.coverage_radius, self.args.coverage_cell_size)
        self.physical_coverage.observe(msg.pose.position.x, msg.pose.position.y)
        if self.nominal_coverage is not self.physical_coverage:
            self.nominal_coverage.observe(msg.pose.position.x, msg.pose.position.y)
        if self.active_goal_record is not None and previous is not None:
            record = self.active_goal_record
            record["executed_length_m"] = record.get("executed_length_m", 0.0) + math.hypot(
                msg.pose.position.x - previous[0], msg.pose.position.y - previous[1])
            direct = float(record.get("distance_m") or 0.0)
            if direct > 0.05:
                record["path_efficiency_ratio"] = record["executed_length_m"] / direct
        now = time.monotonic()
        self.gt_history.append((now, float(msg.pose.position.x), float(msg.pose.position.y), yaw))
        self.gt_history = [item for item in self.gt_history if now - item[0] <= self.args.spin_window_sec]
        self._maybe_capture_spin(now)

    def _slam_pose_cb(self, msg: PoseStamped) -> None:
        self.latest_slam_pose = [msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(msg.pose.orientation)]
        if self.args.record_dir:
            self.slam_ts.append(self._tum_row(msg))

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

    def _event_cb(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            event = {"event": "malformed", "raw": msg.data}
        event.setdefault("eval_t", time.monotonic() - self.started_at)
        self.events.append(event)
        self._update_goal_records(event)
        self._write_jsonl(self.raw_events_file, event)
        if event.get("event") in {
            "planner_precheck_rejected",
            "goal_sent",
            "goal_accepted",
            "goal_cancel_requested",
            "goal_succeeded",
            "goal_failed",
            "goal_safety_cancelled",
        } and self.goal_records:
            self._rewrite_goal_records()
        if event.get("event") == "goal_cancel_requested":
            reason = str(event.get("reason", ""))
            if reason in {"no_progress", "timeout"} or reason.startswith("safety:"):
                self._capture_snapshot(f"cancel_{reason.replace(':', '_')}", event)

    def _diagnostics_cb(self, msg: String) -> None:
        try:
            diagnostic = json.loads(msg.data)
        except json.JSONDecodeError:
            diagnostic = {"raw": msg.data}
        diagnostic.setdefault("eval_t", time.monotonic() - self.started_at)
        self.latest_diagnostics = diagnostic
        self.diagnostics.append(diagnostic)
        self.diagnostics = self.diagnostics[-self.args.max_diagnostics_records :]
        self._write_jsonl(self.diagnostics_file, diagnostic)
        primary = str((diagnostic.get("classification") or {}).get("primary", ""))
        if primary in {
            "gt_motion_stalled_while_commanded",
            "nav2_feedback_no_distance_progress",
            "near_goal_progress_checker_or_goal_tolerance_issue",
        }:
            self._capture_snapshot(primary, {"event": "diagnostics", "classification": diagnostic.get("classification")})

    def _nav_feedback_cb(self, msg: NavigateToPose.Impl.FeedbackMessage) -> None:
        feedback = msg.feedback
        self.latest_feedback = {
            "distance_remaining_m": float(feedback.distance_remaining),
            "recoveries": int(feedback.number_of_recoveries),
            "navigation_time_sec": int(feedback.navigation_time.sec) + 1e-9 * int(feedback.navigation_time.nanosec),
        }
        if self.active_goal_record is not None:
            self.active_goal_record["last_feedback"] = self.latest_feedback

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self.latest_cmd_vel = {
            "linear_x": float(msg.linear.x),
            "angular_z": float(msg.angular.z),
        }

    def _plan_cb(self, msg: NavPath) -> None:
        length = path_length(msg)
        self.latest_path_stats = {
            "pose_count": len(msg.poses),
            "length_m": length,
            "frame_id": msg.header.frame_id,
        }
        if self.active_goal_record is not None:
            self.active_goal_record["path"] = dict(self.latest_path_stats)
            self.active_goal_record.setdefault("initial_planned_length_m", length)
            self.active_goal_record["remaining_planned_length_m"] = length

    def _grid_snapshot_cb(self, key: str, msg: OccupancyGrid) -> None:
        self.latest_grids[key] = msg

    @staticmethod
    def _write_jsonl(handle, item: dict) -> None:
        if handle is None:
            return
        handle.write(json.dumps(item, sort_keys=True) + "\n")
        handle.flush()

    def _rewrite_goal_records(self) -> None:
        if self.goal_events_file is None:
            return
        self.goal_events_file.seek(0)
        self.goal_events_file.truncate(0)
        self.goal_events_file.write("".join(json.dumps(item, sort_keys=True) + "\n" for item in self.goal_records))
        self.goal_events_file.flush()

    def _update_goal_records(self, event: dict) -> None:
        event_name = event.get("event")
        if event_name == "planner_precheck_rejected":
            self.goal_records.append(
                {
                    "event": event_name,
                    "kind": event.get("kind"),
                    "failure_class": "planner_reject",
                    "reason": event.get("reason"),
                    "pose": event.get("pose"),
                    "candidate": event.get("candidate"),
                    "elapsed_sec": event.get("elapsed_sec"),
                    "coverage": event.get("coverage"),
                    "mission_coverage": event.get("mission_coverage"),
                }
            )
            return
        if event_name != "goal_sent" and self.active_goal_record is None:
            return
        if event_name == "goal_sent":
            self.active_goal_record = {
                "goal_index": event.get("goal_index", event.get("goals")),
                "kind": event.get("kind"),
                "pose": event.get("pose"),
                "candidate": event.get("candidate"),
                "distance_m": event.get("distance_m"),
                "timeout_sec": event.get("timeout_sec"),
                "sent_elapsed_sec": event.get("elapsed_sec"),
                "sent_coverage": event.get("coverage"),
                "sent_mission_coverage": event.get("mission_coverage"),
            }
            self.goal_records.append(self.active_goal_record)
            return
        record = self.active_goal_record
        if event_name == "goal_accepted":
            record["accepted_elapsed_sec"] = event.get("elapsed_sec")
        elif event_name == "goal_cancel_requested":
            record["cancel_reason"] = event.get("reason")
            record["cancel_elapsed_sec"] = event.get("elapsed_sec")
            record["failure_class"] = self._classify_goal_event(event, record)
        elif event_name in {"goal_succeeded", "goal_failed", "goal_safety_cancelled"}:
            record["result_event"] = event_name
            record["status"] = event.get("status")
            record["duration_sec"] = event.get("duration_sec")
            record["result_elapsed_sec"] = event.get("elapsed_sec")
            record["end_coverage"] = event.get("coverage")
            record["end_mission_coverage"] = event.get("mission_coverage")
            record["failure_class"] = self._classify_goal_event(event, record)
            record["last_feedback"] = self.latest_feedback
            self.active_goal_record = None

    @staticmethod
    def _classify_goal_event(event: dict, record: dict) -> str:
        if event.get("event") == "goal_succeeded":
            return "success"
        reason = str(event.get("reason") or record.get("cancel_reason") or "")
        if reason == "no_progress":
            return "controller_stall"
        if reason == "timeout":
            return "timeout"
        if reason.startswith("safety:"):
            return "unsafe_path_cancel"
        if event.get("event") == "goal_safety_cancelled":
            return "unsafe_path_cancel"
        if event.get("event") == "planner_precheck_rejected":
            return "planner_reject"
        return "nav2_failure"

    def _maybe_capture_spin(self, now: float) -> None:
        if len(self.gt_history) < 10 or now - self.last_snapshot_at < self.args.snapshot_cooldown_sec:
            return
        first = self.gt_history[0]
        last = self.gt_history[-1]
        displacement = math.hypot(last[1] - first[1], last[2] - first[2])
        yaw_sum = sum(abs(angle_delta(a[3], b[3])) for a, b in zip(self.gt_history, self.gt_history[1:]))
        if displacement <= self.args.spin_max_displacement_m and yaw_sum >= math.radians(self.args.spin_min_yaw_deg):
            self._capture_snapshot("spin_episode", {"event": "spin_episode", "yaw_sum_rad": yaw_sum, "displacement_m": displacement})

    def _capture_snapshot(self, reason: str, trigger: dict) -> None:
        if not self.args.record_dir:
            return
        now = time.monotonic()
        if now - self.last_snapshot_at < self.args.snapshot_cooldown_sec:
            return
        self.last_snapshot_at = now
        self.snapshot_count += 1
        output_dir = self.record_dir / "snapshots" if self.record_dir else Path(self.args.record_dir) / "snapshots"
        output_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "reason": np.asarray([reason]),
            "trigger_json": np.asarray([json.dumps(trigger, sort_keys=True)]),
            "latest_status_json": np.asarray([json.dumps(self.latest_status, sort_keys=True)]),
            "latest_diagnostics_json": np.asarray([json.dumps(self.latest_diagnostics, sort_keys=True)]),
            "latest_feedback_json": np.asarray([json.dumps(self.latest_feedback, sort_keys=True)]),
            "latest_cmd_vel_json": np.asarray([json.dumps(self.latest_cmd_vel, sort_keys=True)]),
            "gt_history": np.asarray(self.gt_history, dtype=np.float64),
        }
        if self.latest_gt_pose is not None:
            payload["latest_gt_pose"] = np.asarray(self.latest_gt_pose, dtype=np.float64)
        if self.latest_slam_pose is not None:
            payload["latest_slam_pose"] = np.asarray(self.latest_slam_pose, dtype=np.float64)
        for key, grid in self.latest_grids.items():
            payload[f"{key}_data"] = np.asarray(grid.data, dtype=np.int16).reshape((grid.info.height, grid.info.width))
            payload[f"{key}_meta"] = np.asarray(
                [grid.info.resolution, grid.info.origin.position.x, grid.info.origin.position.y],
                dtype=np.float64,
            )
            payload[f"{key}_frame"] = np.asarray([grid.header.frame_id])
        np.savez_compressed(output_dir / f"{self.snapshot_count:03d}_{reason}.npz", **payload)

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

    def _localization_cb(self, msg) -> None:
        key = (msg.epoch, msg.state, msg.reason)
        if key != self.last_localization_key:
            self.localization_events.append({"epoch": msg.epoch, "state": msg.state, "reason": msg.reason,
                                             "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9})
            self.last_localization_key = key

    def done(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now > 0:
            if self.started_sim is None:
                self.started_sim = now
            if self.last_sim is not None and now < self.last_sim:
                self.clock_fault = "clock_reversed"
                return True
            self.last_sim = now
            self.sim_duration = now - self.started_sim
        if time.monotonic() - self.started_at >= self.args.wall_timeout:
            self.clock_fault = "wall_timeout"
            return True
        if self.sim_duration >= self.args.duration:
            return True
        return self.latest_status.get("state") in {"finished", "failed"}

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
            "duration_sec": self.sim_duration,
            "duration_timebase": "simulation",
            "wall_duration_sec": time.monotonic() - self.started_at,
            "clock_fault": self.clock_fault,
            "localization_events": self.localization_events,
            "spontaneous_localization_faults": sum(e["state"] in (3, 6) for e in self.localization_events),
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
            "goal_event_count": len(self.events),
            "goal_summary": self.goal_summary(),
            "failure_class_counts": self.failure_class_counts(),
            "diagnostics_tail": self.diagnostics[-20:],
            "snapshot_count": self.snapshot_count,
            "visited_gt_cell_count": len(self.visited_gt_cells),
            "visited_gt_cells_tail": list(self.visited_gt_cells.values())[-20:],
            "top_resource_cells": sorted(
                self.visited_gt_cells.values(), key=lambda item: item["best_value"], reverse=True
            )[:20],
            "sample_path_tail": self.sample_path[-50:],
            "verdict": self.verdict(),
        }

    def goal_summary(self) -> dict:
        completed = [g for g in self.goal_records if g.get("result_event") or g.get("failure_class") == "planner_reject"]
        ratios = [float(g["path_efficiency_ratio"]) for g in completed if g.get("path_efficiency_ratio") is not None]
        durations = [float(g["duration_sec"]) for g in completed if g.get("duration_sec") is not None]
        return {
            "total_records": len(self.goal_records),
            "completed_records": len(completed),
            "path_efficiency_median": float(np.median(ratios)) if ratios else None,
            "path_efficiency_p90": float(np.percentile(ratios, 90)) if ratios else None,
            "duration_median_sec": float(np.median(durations)) if durations else None,
            "duration_p90_sec": float(np.percentile(durations, 90)) if durations else None,
        }

    def failure_class_counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for record in self.goal_records:
            cls = record.get("failure_class")
            if not cls:
                continue
            counts[str(cls)] = counts.get(str(cls), 0) + 1
        if self.latest_status.get("return_reason") == "slam_diverged":
            counts["slam_guard"] = counts.get("slam_guard", 0) + 1
        return counts

    def verdict(self) -> dict | None:
        """Independent physical coverage and home scoring; corrupt references fail closed."""
        mode = self.args.mission_mode
        if not mode:
            return None
        status = self.latest_status or {}
        gt = self.latest_gt_pose
        home_dist = (
            math.hypot(gt[0] - self.home[0], gt[1] - self.home[1])
            if gt is not None and self.home is not None
            else None
        )
        reference_valid = (not self.reference_faults and self.clock_fault is None and self.last_gt_received_at is not None
                           and time.monotonic() - self.last_gt_received_at <= 1.0)
        coverage = self.physical_coverage.fraction if self.physical_coverage is not None else 0.0
        result = mission_verdict(mode, status, home_dist, coverage, self.best_sample,
                                 self.args.home_tol, self.args.coverage_gate, self.args.sample_gate, reference_valid)
        result["reference_faults"] = self.reference_faults
        result["clock_fault"] = self.clock_fault
        result["localization_fault_free"] = not any(e["state"] in (3, 6) for e in self.localization_events)
        if not result["localization_fault_free"]:
            result["reasons"].append("localization_fault_observed")
            result["pass"] = False
        result["metrics"]["online_coverage_fraction"] = status.get("mission_coverage_fraction")
        result["metrics"]["home_xy"] = list(self.home) if self.home is not None else None
        result["metrics"]["nominal_home_coverage_fraction"] = self.nominal_coverage.fraction
        result["metrics"]["nominal_home_distance_m"] = (
            math.hypot(gt[0] - self.args.home_x, gt[1] - self.args.home_y) if gt is not None else None)
        return result

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
    parser.add_argument("--duration", type=float, default=900.0, help="Simulation seconds, not wall time")
    parser.add_argument("--wall-timeout", type=float, default=0.0, help="Infrastructure watchdog; default 4*duration+60")
    parser.add_argument("--localization-topic", default="/astrobot_0/localization/status")
    parser.add_argument("--bev-topic", default="/astrobot_0/slam/bev_costmap")
    parser.add_argument("--gt-map-topic", default="/gt/map")
    parser.add_argument("--gt-pose-topic", default="/gt/base_link_pose")
    parser.add_argument("--slam-pose-topic", default="/astrobot_0/slam/pose_corrected")
    parser.add_argument("--status-topic", default="/astrobot_0/exploration/status")
    parser.add_argument("--resource-sample-topic", default="/astrobot_0/resource/water/sample")
    parser.add_argument("--resource-sample-pose-topic", default="/astrobot_0/resource/water/sample_pose")
    parser.add_argument("--resource-map-topic", default="/gt/resource_maps/water")
    parser.add_argument("--selected-goal-topic", default="/astrobot_0/exploration/selected_goal")
    parser.add_argument("--event-topic", default="/astrobot_0/exploration/events")
    parser.add_argument("--diagnostics-topic", default="/astrobot_0/diagnostics")
    parser.add_argument("--plan-topic", default="/astrobot_0/plan")
    parser.add_argument("--cmd-vel-topic", default="/astrobot_0/cmd_vel")
    parser.add_argument("--nav-feedback-topic", default="/astrobot_0/navigate_to_pose/_action/feedback")
    parser.add_argument("--global-costmap-topic", default="/astrobot_0/global_costmap/costmap")
    parser.add_argument("--local-costmap-topic", default="/astrobot_0/local_costmap/costmap")
    parser.add_argument("--output", default="")
    parser.add_argument("--goal-events-output", default="")
    parser.add_argument("--diagnostics-output", default="")
    parser.add_argument("--resource-compare-output", default="")
    # Gate-verdict args (verdict is computed only when --mission-mode is given,
    # so existing callers are unaffected). Home defaults to the lunaryard spawn.
    parser.add_argument("--mission-mode", default="", choices=["", "explore_radius", "sample_return"])
    parser.add_argument("--home-x", type=float, default=20.0)
    parser.add_argument("--home-y", type=float, default=18.0)
    parser.add_argument("--home-from-gt-start", action="store_true",
                        help="score coverage and home distance about the rover's first GT pose, not --home-x/y")
    parser.add_argument("--coverage-gate", type=float, default=0.70)
    parser.add_argument("--coverage-radius", type=float, default=8.0)
    parser.add_argument("--coverage-cell-size", type=float, default=2.0)
    parser.add_argument("--sample-gate", type=float, default=0.75)
    parser.add_argument("--home-tol", type=float, default=1.0)
    parser.add_argument("--max-diagnostics-records", type=int, default=4000)
    parser.add_argument("--spin-window-sec", type=float, default=10.0)
    parser.add_argument("--spin-max-displacement-m", type=float, default=0.20)
    parser.add_argument("--spin-min-yaw-deg", type=float, default=90.0)
    parser.add_argument("--snapshot-cooldown-sec", type=float, default=30.0)
    # If set, write gt_base.tum + slam_corrected.tum (recorded in-node, no extra
    # processes) into this dir for divergence fingerprinting.
    parser.add_argument("--record-dir", default="")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.wall_timeout <= 0:
        args.wall_timeout = 4*args.duration + 60

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
        if args.goal_events_output:
            Path(args.goal_events_output).write_text(
                "".join(json.dumps(item, sort_keys=True) + "\n" for item in node.goal_records),
                encoding="utf-8",
            )
        if args.diagnostics_output:
            Path(args.diagnostics_output).write_text(
                "".join(json.dumps(item, sort_keys=True) + "\n" for item in node.diagnostics),
                encoding="utf-8",
            )
        if args.record_dir:
            d = Path(args.record_dir)
            d.mkdir(parents=True, exist_ok=True)
            (d / "goal_events.jsonl").write_text(
                "".join(json.dumps(item, sort_keys=True) + "\n" for item in node.goal_records),
                encoding="utf-8",
            )
            (d / "raw_exploration_events.jsonl").write_text(
                "".join(json.dumps(item, sort_keys=True) + "\n" for item in node.events),
                encoding="utf-8",
            )
            (d / "diagnostics.jsonl").write_text(
                "".join(json.dumps(item, sort_keys=True) + "\n" for item in node.diagnostics),
                encoding="utf-8",
            )
            for name, ts in (("gt_base.tum", node.gt_ts), ("slam_corrected.tum", node.slam_ts)):
                (d / name).write_text(
                    "".join(f"{r[0]:.4f} {r[1]:.5f} {r[2]:.5f} {r[3]:.5f} "
                            f"{r[4]:.6f} {r[5]:.6f} {r[6]:.6f} {r[7]:.6f}\n" for r in ts),
                    encoding="utf-8",
                )
        if args.resource_compare_output:
            node.write_resource_compare(args.resource_compare_output)
        for handle in (node.goal_events_file, node.raw_events_file, node.diagnostics_file):
            if handle is not None:
                handle.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
