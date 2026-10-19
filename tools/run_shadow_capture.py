#!/usr/bin/env python3
"""Record before launching pose-only GLIM; lease a guarded simulator driver.

Start this before the opt-in shadow simulator. Never publishes cmd_vel. All
received required messages are synchronously written before readiness checks.
Output is collection evidence only, not moving-terrain or navigation acceptance.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from archive_guard import allocate_run, check_run
from capture_polling import StorageCheck
from audit_shadow_inputs import transform7
from export_shadow_trajectory import pose_matrix, revision_record, stamp_ns
from live_revision_gate import LiveRevisionGate
from shadow_capture_gate import PhysicsIMUReadiness, ShadowLease, validate_shadow_geometry
from run_stationary_revision_check import stop
from selected_lidar_profile import TOPIC as SELECTED_TOPIC, required_topics as selected_topics, validate_config


TOPICS = {
    "/clock": "rosgraph_msgs/msg/Clock", "/tf": "tf2_msgs/msg/TFMessage", "/tf_static": "tf2_msgs/msg/TFMessage",
    "/gt/base_link_pose": "geometry_msgs/msg/PoseStamped", "/astrobot_0/odom": "nav_msgs/msg/Odometry",
    "/astrobot_0/imu": "sensor_msgs/msg/Imu", "/rtk/pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/astrobot_0/localization/absolute_pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/astrobot_0/localization/status": "astrobot_interfaces/msg/LocalizationStatus",
    "/astrobot_0/cmd_vel": "geometry_msgs/msg/Twist",
    "/terrain_shadow/request": "std_msgs/msg/String",
}
for suffix in ("raw", "filtered"):
    TOPICS["/astrobot_0/lidar_0/pointcloud/" + suffix] = "sensor_msgs/msg/PointCloud2"
for suffix, kind in (("revisions", "astrobot_interfaces/msg/SlamRevision"),
                     ("observations", "astrobot_interfaces/msg/SlamObservation"),
                     ("pose_corrected", "geometry_msgs/msg/PoseStamped"),
                     ("lidar_pose_corrected", "geometry_msgs/msg/PoseStamped"),
                     ("aligned_points_corrected", "sensor_msgs/msg/PointCloud2"),
                     ("points_corrected", "sensor_msgs/msg/PointCloud2")):
    TOPICS["/astrobot_0/slam_raw/" + suffix] = kind

# Isaac publishes rigid sensor transforms on /tf. Record /tf_static when
# present, but require actual transform agreement, not a specific transport.
REQUIRED_TOPICS = set(TOPICS) - {"/tf_static", "/astrobot_0/cmd_vel"}
# Optional comparison stream. Its absence cannot invalidate historical captures;
# it is not an estimator input. Explicit paired experiments require its readiness.
TOPICS["/astrobot_0/imu_physics"] = "sensor_msgs/msg/Imu"
TOPICS["/astrobot_0/imu_physics/clock"] = "std_msgs/msg/String"
# Optional actuator telemetry for turn-drift diagnosis; never estimator input
# or grounds for accepting a run without the required GT/native streams.
TOPICS["/astrobot_0/joint_states"] = "sensor_msgs/msg/JointState"
TERRAIN_TOPICS = {"/terrain_shadow/native_chunks": "sensor_msgs/msg/PointCloud2",
                  "/terrain_shadow/native_complete": "std_msgs/msg/String",
                  "/terrain_shadow/costmap": "nav_msgs/msg/OccupancyGrid",
                  "/terrain_shadow/classes": "nav_msgs/msg/OccupancyGrid",
                  "/terrain_shadow/status": "std_msgs/msg/String"}
TOPICS.update(TERRAIN_TOPICS)  # Optional on historical/localization-only captures.
TOPICS["/terrain_shadow/contact_advisory"] = "std_msgs/msg/String"
TOPICS["/terrain_shadow/planner_costmap"] = "nav_msgs/msg/OccupancyGrid"
TOPICS["/terrain_shadow/uncertainty_mask"] = "nav_msgs/msg/OccupancyGrid"
TOPICS["/terrain_trial/proposal"] = "std_msgs/msg/String"
TOPICS[SELECTED_TOPIC] = "sensor_msgs/msg/PointCloud2"


def config_fingerprint(config):
    files = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(config.glob("*.json"))}
    if not files:
        raise ValueError("empty GLIM configuration")
    return hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest(), files


class Capture(Node):
    def __init__(self, run, config, require_physics_imu=False, natural_stationary=False, natural_motion="",
                 terrain_library=None, selected_lidar=False, terrain_v2=False, execution_profile="default"):
        super().__init__("terrain_shadow_capture", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.run, self.config = run, config
        self.natural_stationary = natural_stationary
        self.natural_motion = natural_motion
        from natural_motion import execution_budget
        self.execution_budget = execution_budget(natural_motion, execution_profile)
        self.execution_profile = execution_profile
        self.selected_lidar = selected_lidar
        if selected_lidar:
            if not natural_motion or terrain_library is not None and not terrain_v2:
                raise ValueError("selected LiDAR requires guarded motion and explicit v2 for terrain")
            validate_config(config)
        self.terrain_library_sha256 = (hashlib.sha256(Path(terrain_library).read_bytes()).hexdigest()
                                       if terrain_library is not None else None)
        self.terrain_status = None
        self.terrain_status_wall = 0.
        self.terrain_first_stamp = None
        self.gt_samples = deque(maxlen=1800)
        self.first_frame = self.initial_output_check = None
        if natural_stationary and natural_motion:
            raise ValueError("stationary and motion acquisitions are mutually exclusive")
        if (natural_stationary or natural_motion) and not require_physics_imu:
            raise ValueError("natural recovery holdout requires physics IMU readiness")
        self.physics = PhysicsIMUReadiness() if require_physics_imu else None
        self.required_topics = REQUIRED_TOPICS | ({"/astrobot_0/imu_physics", "/astrobot_0/imu_physics/clock"}
                                                  if require_physics_imu else set())
        if selected_lidar:
            self.required_topics = selected_topics(self.required_topics)
        self.digest, self.config_files = config_fingerprint(config)
        selected = json.loads((config / "config.json").read_text())["global"]["config_sensors"]
        self.extrinsic = transform7(json.loads((config / selected).read_text())["sensors"]["T_lidar_imu"])
        self.writer = SequentialWriter()
        self.writer.open(StorageOptions(uri=str(run / "bag"), storage_id="mcap", max_bagfile_size=1024**3),
                         ConverterOptions("", ""))
        self.counts = Counter()
        self.processing_max_s = {}
        self.slow_processing_count = 0
        self.slow_processing = deque(maxlen=128)
        self.recorder_sources = {str(Path(__file__).with_name(name)):
            hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in ("run_shadow_capture.py", "live_revision_gate.py", "export_shadow_trajectory.py",
                         "shadow_terrain.py", "archive_guard.py", "shadow_capture_gate.py",
                         "selected_lidar_profile.py", "natural_motion.py", "local_terrain_seal.py",
                         "capture_polling.py")}
        self.failure = None
        self.request = self.status = self.revision = self.timeline = self.active = None
        self.finished_stamp = self.finished_wall = None
        self.sequence = 0
        self.last_clock_wall = self.status_wall = self.request_wall = 0.
        self.subscriptions_owned = []
        for index, (topic, kind) in enumerate({**TOPICS, "/terrain_shadow/grant": "std_msgs/msg/String"}.items()):
            self.writer.create_topic(TopicMetadata(id=index, name=topic, type=kind, serialization_format="cdr"))
            if topic == "/terrain_shadow/grant":
                continue  # Written synchronously immediately before publication.
            transient = topic.endswith(("revisions", "observations")) or topic == "/tf_static"
            reliable = transient or topic in TERRAIN_TOPICS
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
            self.subscriptions_owned.append(self.create_subscription(
                get_message(kind), topic, lambda msg, t=topic: self.receive(t, msg), qos))
        self.grants = self.create_publisher(String, "/terrain_shadow/grant", 1)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def record(self, topic, message):
        self.writer.write(topic, serialize_message(message), time.time_ns())
        self.counts[topic] += 1

    def note_processing(self, stage, started, frames=None):
        elapsed = time.monotonic()-started
        self.processing_max_s[stage] = max(elapsed, self.processing_max_s.get(stage, 0.))
        if elapsed > .05:
            self.slow_processing_count += 1
            self.slow_processing.append(dict(stage=stage, wall_s=elapsed, frames=frames,
                                             wall_end_ns=time.time_ns()))

    def receive(self, topic, message):
        started = time.monotonic()
        try:
            self.record(topic, message)
            if topic == "/clock":
                self.last_clock_wall = time.monotonic()
            elif topic == "/terrain_shadow/status" and self.terrain_library_sha256 is not None:
                status = json.loads(message.data)
                if (status.get("navigation_authority") is not False or status.get("failure") is not None
                        or status.get("library_sha256") != self.terrain_library_sha256):
                    raise ValueError("terrain shadow failed or changed its frozen library: " + str(status.get("failure")))
                self.terrain_status, self.terrain_status_wall = status, time.monotonic()
            elif topic == "/terrain_shadow/native_chunks" and self.terrain_library_sha256 is not None:
                if self.terrain_first_stamp is None:
                    self.terrain_first_stamp = stamp_ns(message.header.stamp)/1e9
            elif topic == "/gt/base_link_pose" and self.natural_motion:
                stamp = stamp_ns(message.header.stamp)
                if message.header.frame_id != "gt_map" or (self.gt_samples and stamp <= self.gt_samples[-1][0]):
                    raise ValueError("invalid GT frame or nonmonotonic GT stamp")
                self.gt_samples.append((stamp, pose_matrix(message.pose)))
            elif topic == "/astrobot_0/imu_physics" and self.physics is not None:
                a, w = message.linear_acceleration, message.angular_velocity
                self.physics.sample(stamp_ns(message.header.stamp), time.monotonic(), message.header.frame_id,
                                    message.orientation_covariance[0], (a.x, a.y, a.z, w.x, w.y, w.z))
            elif topic == "/astrobot_0/imu_physics/clock" and self.physics is not None:
                self.physics.clock(json.loads(message.data), time.monotonic())
            elif topic == "/terrain_shadow/request":
                request = json.loads(message.data)
                if ("lidar_profile" in request) != self.selected_lidar:
                    raise ValueError("recorder and simulator LiDAR profiles differ")
                if self.selected_lidar:
                    from selected_lidar_profile import declaration
                    if request["lidar_profile"] != declaration():
                        raise ValueError("recorder and simulator selected geometry profiles differ")
                ShadowLease(request["capture"])
                if self.terrain_library_sha256 is not None and request.get("native_shadow_publishing") is not True:
                    raise ValueError("live terrain capture requires opt-in native publication before motion")
                if self.natural_stationary:
                    from natural_holdout import validate_request
                    validate_request(request)
                if self.natural_motion:
                    from natural_motion import validate_motion_request, validate_execution_budget
                    validate_motion_request(request)
                    validate_execution_budget(request, self.execution_profile)
                    if request["pattern"] != self.natural_motion:
                        raise ValueError("simulator route differs from declared recorder route")
                    if self.request is not None and {k: v for k, v in request.items() if k != "stamp"} != {
                            k: v for k, v in self.request.items() if k != "stamp"}:
                        raise ValueError("natural motion request changed")
                if self.request is not None and request["capture"] != self.request["capture"]:
                    raise ValueError("multiple simulator capture identities")
                self.request, self.request_wall = request, time.monotonic()
            elif topic.endswith("/status"):
                self.status, self.status_wall = message, time.monotonic()
            elif topic.endswith("/revisions"):
                record = revision_record(message)
                # Revision chain validation here is not the offline causal
                # receipt bound. The exporter derives that from recorded /clock.
                record["arrival_ns"] = max(self.get_clock().now().nanoseconds,
                                           max((f["stamp_ns"] for f in record["frames"]), default=0),
                                           self.timeline.last_arrival_ns if self.timeline else 0)
                if self.timeline is None:
                    self.timeline = LiveRevisionGate(record["session"], record["epoch"])
                self.timeline.apply_record(record)
                if self.natural_motion and self.first_frame is None and record["frames"]:
                    self.first_frame = min(record["frames"], key=lambda f: f["stamp_ns"])
                self.revision = message
            elif topic == "/astrobot_0/cmd_vel":
                moving = any(abs(v) > 1e-9 for v in (message.linear.x, message.linear.y, message.linear.z,
                                                    message.angular.x, message.angular.y, message.angular.z))
                if moving and (self.natural_stationary or self.active is None or self.finished_stamp is not None):
                    raise ValueError("motion before shadow recording/readiness barrier or after finish barrier")
        except Exception as exc:
            self.failure = str(exc)
        finally:
            self.note_processing(topic, started, len(message.frames) if topic.endswith("/revisions") else None)

    def tf_matrix(self, child):
        transform = self.tf.lookup_transform("base_link", child, Time()).transform
        return transform7([transform.translation.x, transform.translation.y, transform.translation.z,
                           transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])

    def check_initial_output(self):
        if not self.natural_motion or self.initial_output_check is not None:
            return True
        if self.first_frame is None or not self.gt_samples or self.gt_samples[-1][0] < self.first_frame["stamp_ns"]:
            return False
        from natural_motion import verify_live_start
        result = verify_live_start(self.request, self.first_frame, list(self.gt_samples))
        with (self.run / "initial_output_check.json").open("x") as stream:
            json.dump(result, stream, indent=2)
        if not result["passed"]:
            raise ValueError("first live GLIM output exceeds 1 cm / 0.1 degree GT-start limit")
        self.initial_output_check = result
        return True

    def storage_key(self):
        return (str(self.run), str(self.config), self.digest,
                self.request["capture"] if self.request is not None else None)

    def check_storage(self, key):
        run, config, digest, capture = key
        check_run(Path(run))
        if config_fingerprint(Path(config))[0] != digest:
            raise ValueError("frozen GLIM configuration changed during capture")
        if capture is not None:
            check_run(Path(run).parent / capture)
        if self.execution_budget.get("diagnostic_only"):
            # Deliberately model measured long-archive overhead on the short
            # GC stress route. Never applied to nominal runs or hidden in timing.
            time.sleep(self.execution_budget["storage_check_delay_s"])

    def check_ready(self, storage_receipt=None):
        if self.failure:
            raise ValueError(self.failure)
        if self.status is not None and self.status.state == self.status.FAULTED:
            raise ValueError(f"shadow localization fault: {self.status.reason}")
        if storage_receipt is None:
            self.check_storage(self.storage_key())
        else:
            storage_receipt.consume(self.storage_key())
        publishers = self.get_publishers_info_by_topic("/astrobot_0/cmd_vel")
        if self.natural_stationary and publishers:
            raise ValueError("stationary holdout forbids all command publishers")
        owner = "natural_holdout_capture" if self.natural_motion else "terrain_calibration_drive"
        if any(p.node_name != owner for p in publishers) or len(publishers) > 1:
            raise ValueError("another command publisher is active")
        if self.request is None:
            return None
        capture = self.run.parent / self.request["capture"]
        if (capture / "failure.json").exists():
            raise ValueError("simulator capture failed")
        finish_path = capture / "acquisition_finished.json"
        if finish_path.exists():
            finished = json.loads(finish_path.read_text())
            if (finished.get("schema") != 1 or finished.get("capture") != capture.name
                    or finished.get("views") != (0 if self.natural_stationary or self.natural_motion else 6) or finished.get("commands_stopped") is not True
                    or self.active is None or not math.isfinite(finished["stamp"])
                    or finished["stamp"] > self.get_clock().now().nanoseconds / 1e9 + .05):
                raise ValueError("invalid acquisition finish barrier")
            if self.finished_stamp is None:
                self.finished_stamp, self.finished_wall = finished["stamp"], time.monotonic()
            if finished["stamp"] != self.finished_stamp:
                raise ValueError("acquisition finish barrier changed")
            if time.monotonic()-self.finished_wall > 20:
                raise TimeoutError("simulator archive sealing deadline exceeded")
        completion = capture / ("natural_holdout.json" if self.natural_stationary or self.natural_motion else "multiview.json")
        if completion.exists():
            result = json.loads(completion.read_text())
            if not result.get("complete") or self.active is None or self.finished_stamp is None:
                raise ValueError("invalid simulator completion")
            if self.terrain_library_sha256 is not None:
                if self.terrain_status is None or not self.terrain_status.get("completed"):
                    return "sealing"
                if (self.terrain_status.get("capture") != self.request["capture"] or
                        self.terrain_status.get("received") != result.get("chunks") or
                        self.terrain_status.get("integrated") != result.get("chunks")):
                    raise ValueError("live terrain completion differs from native capture")
            return "complete"
        if self.finished_stamp is not None:
            return "sealing"  # Never grants further motion while files close.
        now = self.get_clock().now().nanoseconds / 1e9
        wall = time.monotonic()
        checks = dict(
            pose_healthy=self.status is not None and self.status.state == self.status.HEALTHY,
            navigation_disabled=self.status is not None and not self.status.motion_permitted,
            epoch_matches=self.status is not None and self.revision is not None
                and self.status.epoch == self.revision.localization_epoch,
            pose_fresh=self.status is not None and 0 <= now-stamp_ns(self.status.last_pose_stamp)/1e9 <= .5,
            status_heartbeat=wall-self.status_wall <= .5, request_heartbeat=wall-self.request_wall <= .5,
            clock_heartbeat=wall-self.last_clock_wall <= .5,
            revisions_recorded=self.counts["/astrobot_0/slam_raw/revisions"] >= 10,
            observations_recorded=self.counts["/astrobot_0/slam_raw/observations"] >= 10,
            required_topics_recorded=all(self.counts[t] for t in self.required_topics),
            physics_imu_ready=self.physics is None or self.physics.ready(now, wall),
            first_output_verified=self.check_initial_output(),
            sole_driver=len(publishers) == (0 if self.natural_stationary else 1))
        self.last_readiness_checks = checks
        if self.terrain_library_sha256 is not None:
            checks["terrain_status_heartbeat"] = self.terrain_status is not None and wall-self.terrain_status_wall <= .5
            # Native capture starts only after the existing grant. Permit that
            # startup handshake, not a moving indefinite all-unknown mapper.
            checks["terrain_mapping_live"] = (self.terrain_first_stamp is None or now-self.terrain_first_stamp <= .4
                                               or self.terrain_status is not None and self.terrain_status.get("valid", False))
        ready = all(checks.values())
        if not ready:
            if self.active is not None:
                raise ValueError(f"shadow inputs became stale/unhealthy after arming: {[k for k,v in checks.items() if not v]}")
            return None
        if not self.revision.frames:
            raise ValueError("empty latest revision")
        frame = max(self.revision.frames, key=lambda f: stamp_ns(f.measurement_stamp))
        try:
            errors = validate_shadow_geometry(self.request, self.extrinsic, self.tf_matrix("Imu_Sensor"),
                                              self.tf_matrix("vlp16"), pose_matrix(frame.imu_from_lidar),
                                              natural_stationary=self.natural_stationary, natural_motion=bool(self.natural_motion))
        except TransformException:
            if self.active is not None:
                raise
            return None
        identity = (self.revision.session_id, self.revision.localization_epoch)
        if self.active is not None and identity != self.active:
            raise ValueError("estimator session changed during capture")
        return dict(identity=identity, errors=errors, stamp=now, capture=str(capture))

    def grant(self, ready):
        if self.active is None:
            link = dict(schema=1, request=self.request, recorder=self.run.name, identity=ready["identity"],
                        config_path=str(self.config), config_sha256=self.digest, config_files=self.config_files,
                        live_geometry_errors=ready["errors"], timing_mode="current_GLIM_global_shutter_baseline",
                        source_frame="map", home_xyz=[-20., -20., 0.],
                        motion_owner="none" if self.natural_stationary else (
                            "natural_holdout_capture" if self.natural_motion else "terrain_calibration_drive"),
                        physics_imu_required=self.physics is not None)
            with (self.run / "shadow_collection.json").open("x") as stream:
                json.dump(link, stream, indent=2)
            self.active = ready["identity"]
            if self.physics is not None:
                self.physics.armed = True
        self.sequence += 1
        message = String(data=json.dumps(dict(schema=1, scope="guarded_terrain_capture", ready=True,
            capture=self.request["capture"], recorder=self.run.name, session=self.active[0], epoch=self.active[1],
            config_sha256=self.digest, sequence=self.sequence, stamp=ready["stamp"])))
        self.record("/terrain_shadow/grant", message)
        self.grants.publish(message)

    def revoke(self):
        if self.active is not None:
            message = String(data=json.dumps(dict(schema=1, scope="guarded_terrain_capture", ready=False,
                                                 capture=self.request["capture"])))
            self.record("/terrain_shadow/grant", message)
            self.grants.publish(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--wall-seconds", type=float, default=900.)
    parser.add_argument("--require-physics-imu", action="store_true",
                        help="Require healthy paired 60 Hz IMU/clock before and throughout motion")
    parser.add_argument("--natural-stationary", action="store_true",
                        help="Independent seed-43 stationary fixture; forbids every command publisher")
    from natural_motion import PATTERNS, EXECUTION_PROFILES, execution_budget
    parser.add_argument("--natural-motion", choices=PATTERNS, default="",
                        help="Geometry-guarded natural acquisition with one-time live GT initialization")
    parser.add_argument("--execution-profile", choices=EXECUTION_PROFILES, default="default",
                        help="Explicit total wall-time budget; motion/heartbeat/pose guards remain unchanged")
    parser.add_argument("--terrain-library", type=Path,
                        help="Opt-in advisory native mapper; requires natural motion and native publication in Isaac")
    parser.add_argument("--selected-lidar", action="store_true", help="Level nonuniform native-window profile")
    parser.add_argument("--terrain-v2", action="store_true", help="Opt-in rolling native v2 advisory costmap")
    parser.add_argument("--isolated-gap-costs", action="store_true", help="Separate experimental planner costs for isolated unknown cells; no command-policy change")
    parser.add_argument("--contact-advisory", action="store_true", help="Opt-in current-pose contact diagnostic; guarded clear deployment, no commands")
    parser.add_argument("--contact-startup-turn-area", action="store_true", help="Declare fixed physical turning-circle deployment support, no added skid margin")
    parser.add_argument("--contact-current-turn-area", action="store_true", help="Assume turning area at actual current pose; hazards still veto, no inferred history")
    parser.add_argument("--contact-support-core", action="store_true", help="Explicit experimental smaller support region; full hazard vetoes remain")
    args = parser.parse_args()
    if args.isolated_gap_costs and not args.terrain_v2:
        parser.error("isolated-gap costs require v2 terrain")
    from costmap_scenarios import TRIAL_PATTERNS
    if args.natural_motion in TRIAL_PATTERNS and not (
            args.terrain_v2 and args.contact_advisory and args.contact_startup_turn_area and not args.contact_support_core):
        parser.error("terrain_step requires v2, contact advisory, startup turn area, and original full patches")
    if args.contact_advisory and not args.terrain_v2:
        parser.error("contact advisory requires the guarded v2 terrain experiment")
    if args.contact_support_core and not args.contact_advisory:
        parser.error("experimental support core requires contact advisory")
    if args.contact_startup_turn_area and not args.contact_advisory:
        parser.error("startup turning area requires contact advisory")
    if args.contact_current_turn_area and not args.contact_advisory:
        parser.error("current turning area requires contact advisory")
    try:
        budget = execution_budget(args.natural_motion, args.execution_profile)
    except ValueError as exc:
        parser.error(str(exc))
    from costmap_scenarios import recorder_wall_maximum
    wall_maximum = recorder_wall_maximum(args.natural_motion, args.execution_profile, budget)
    if not 30 <= args.wall_seconds <= wall_maximum:
        parser.error(f"wall limit must be 30..{wall_maximum:g} seconds for this execution profile")
    if args.natural_motion and (args.natural_stationary or not args.require_physics_imu):
        parser.error("natural motion requires physics IMU and excludes stationary mode")
    if args.terrain_library and not args.natural_motion:
        parser.error("live terrain integration currently requires a guarded natural-motion experiment")
    if args.terrain_v2:
        from selected_lidar_profile import declaration
        from build_local_terrain import validate_build
        if not args.selected_lidar or not args.terrain_library or declaration()["name"] != "selected_optical_v2":
            parser.error("v2 requires explicit optical profile, selected LiDAR and frozen terrain library")
        validate_build(args.terrain_library, args.archive_root)
    from costmap_scenarios import archive_limit
    run = allocate_run(args.archive_root, limit=archive_limit(args.natural_motion, args.execution_profile))
    print(f"Run: {run}", flush=True)
    rclpy.init()
    node = launch = manifest = None
    storage_checks = None
    passed, failure, start = False, None, time.monotonic()
    frozen_config = run / "config"
    command = ["ros2", "launch", "astrobot_launch", "terrain_shadow.launch.py", f"config_path:={frozen_config}"]
    try:
        shutil.copytree(args.config, frozen_config)
        terrain_library = None
        if args.terrain_library:
            terrain_library = run / "libterrain_shadow.so"
            shutil.copy2(args.terrain_library, terrain_library)
        node = Capture(run, frozen_config, args.require_physics_imu, args.natural_stationary, args.natural_motion,
                       terrain_library=terrain_library, selected_lidar=args.selected_lidar, terrain_v2=args.terrain_v2,
                       execution_profile=args.execution_profile)
        cuda = subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / "docker/check_cuda_runtime.py")],
                              capture_output=True, text=True, timeout=30)
        with (run / "cuda_preflight.json").open("x") as stream:
            json.dump(dict(returncode=cuda.returncode, stdout=cuda.stdout, stderr=cuda.stderr), stream, indent=2)
        if cuda.returncode:
            raise RuntimeError("capture container CUDA preflight failed; no estimator launch or motion grant")
        with (run / "shadow_launch.log").open("x") as log:
            env = dict(os.environ, ROS_LOG_DIR=str(run / "ros-log"))
            storage_checks = StorageCheck()
            while time.monotonic()-start < args.wall_seconds:
                rclpy.spin_once(node, timeout_sec=.02)
                # Teleport/static calibration precede this request. Do not
                # initialize an estimator on pre-teleport measurements.
                if launch is None and node.request is not None:
                    check_run(run.parent / node.request["capture"])
                    if args.natural_motion:
                        from natural_motion import seed_from_request
                        seed_path = run / "live_gt_seed.json"
                        with seed_path.open("x") as stream:
                            json.dump(seed_from_request(node.request), stream, indent=2)
                        command.append(f"shadow_seed_path:={seed_path}")
                    if terrain_library is not None:
                        import numpy as np
                        manifest_data = json.loads((run.parent / node.request["capture"] / "manifest.json").read_text())
                        sensor = manifest_data["sensor"]["sensor_attributes"]
                        base = np.asarray(node.request["world_from_base"])
                        origin = np.floor((base[:2, 3]-np.asarray(node.request["home_xyz"])[:2])/.1)*.1-10.
                        command.extend(["terrain_mapping:=true", f"terrain_library:={terrain_library}",
                            f"terrain_origin_x:={origin[0]}", f"terrain_origin_y:={origin[1]}",
                            f"terrain_min_range:={float(sensor['minRange'])}",
                            f"terrain_max_range:={float(sensor['maxRange'])}"])
                        if args.terrain_v2:
                            request_path = run / "terrain_request.json"
                            with request_path.open("x") as stream:
                                json.dump(node.request, stream, indent=2)
                            command.extend(["terrain_executable:=live_local_terrain.py",
                                            f"terrain_request_json:={request_path}"])
                            # v2 additionally verifies the immutable build's source
                            # manifest. The recorder's copied .so has the same hash.
                            command = [item for item in command if not item.startswith("terrain_library:=")]
                            command.append(f"terrain_library:={args.terrain_library}")
                            if args.isolated_gap_costs:
                                command.append("isolated_gap_costs:=true")
                            if args.contact_advisory:
                                command.extend(["contact_advisory:=true", "contact_deployment_confirmed:=true"])
                            if args.contact_current_turn_area:
                                command.append("contact_current_turn_area:=true")
                            if args.contact_startup_turn_area:
                                command.append("contact_startup_turn_area:=true")
                                if args.contact_support_core:
                                    command.append("contact_support_core:=true")
                    launch = subprocess.Popen(command, stdout=log, stderr=log, env=env, start_new_session=True)
                if launch is not None and launch.poll() is not None:
                    raise RuntimeError("shadow launch exited early")
                if node.failure:
                    raise ValueError(node.failure)
                receipt = storage_checks.poll(node.storage_key(), node.check_storage)
                if receipt is None:
                    continue
                node.note_processing("storage_check", receipt.started)
                checked = time.monotonic()
                try:
                    ready = node.check_ready(storage_receipt=receipt)
                finally:
                    node.note_processing("readiness", checked)
                if ready == "sealing":
                    continue
                if ready == "complete":
                    # Native acquisition has stopped, but a final scan pose may
                    # not yet bracket its last tick. Preserve a bounded postroll
                    # without adding native evidence or issuing another grant.
                    deadline = time.monotonic()+10.
                    while (node.get_clock().now().nanoseconds / 1e9 < node.finished_stamp + .2
                           or node.timeline is None or not node.timeline.stamps
                           or max(node.timeline.stamps) < round(node.finished_stamp*1e9)):
                        if time.monotonic() > deadline or launch.poll() is not None:
                            raise TimeoutError("final native tick lacks a recorded pose bracket")
                        rclpy.spin_once(node, timeout_sec=.02)
                        if node.failure:
                            raise ValueError(node.failure)
                    # Stop producing estimates while continuing to record, so
                    # the final revision also has a following clock receipt.
                    launch.send_signal(signal.SIGINT)
                    deadline = time.monotonic()+20.
                    while launch.poll() is None and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.02)
                    if launch.poll() is None:
                        raise TimeoutError("estimator shutdown did not finish before recording tail")
                    end = time.monotonic()+1.
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=.02)
                    if node.failure:
                        raise ValueError(node.failure)
                    if args.terrain_v2:
                        from local_terrain_seal import verify_seal
                        verify_seal(args.archive_root, node.terrain_status)
                        if args.contact_advisory:
                            from local_terrain.contact_runtime import verify_contact_seal
                            verify_contact_seal(args.archive_root, node.terrain_status)
                    passed = True
                    break
                if ready:
                    if manifest is None:
                        manifest = subprocess.Popen([sys.executable, str(Path(__file__).with_name("write_run_manifest.py")),
                            "--output", str(run / "runtime_manifest.json"), "--seed", "43" if args.natural_stationary or args.natural_motion else "42", "--launch-command", shlex.join(command)],
                            stdout=log, stderr=log, env=env, start_new_session=True)
                    elif manifest.poll() == 0:
                        node.grant(ready)
                    elif manifest.poll() is not None:
                        raise RuntimeError("runtime provenance collection failed")
            if not passed:
                raise TimeoutError("shadow collection deadline exceeded")
    except (Exception, KeyboardInterrupt) as exc:
        failure = str(exc) or "interrupted"
    finally:
        if node is not None:
            try:
                node.revoke()
            except Exception as exc:
                failure = failure or str(exc)
            if storage_checks is not None:
                storage_checks.close()
            counts = dict(node.counts)
            del node.writer  # Finalize MCAP and metadata before success is recorded.
            node.destroy_node()
        else:
            counts = {}
        stop(manifest)
        stop(launch)
        rclpy.try_shutdown()  # SIGINT can already have shut down the ROS context.
    result = dict(collection_completed=passed and failure is None, failure=failure, command=command,
                  counts=counts, wall_seconds=time.monotonic()-start, validation="NOT_RUN",
                  experiment="assisted_GLIM_guarded_native_shadow", no_navigation_authority=True,
                  physics_imu_required=args.require_physics_imu)
    result["natural_stationary"] = args.natural_stationary
    result["natural_motion"] = args.natural_motion
    result["execution_profile"] = args.execution_profile
    result["instrumented_gc_stress"] = budget.get("diagnostic_only", False)
    result["wall_limit_seconds"] = args.wall_seconds
    result["storage_worker"] = None if storage_checks is None else storage_checks.diagnostics()
    result["selected_lidar"] = args.selected_lidar
    result["terrain_shadow_required"] = bool(args.terrain_library)
    result["isolated_gap_costs"] = args.isolated_gap_costs
    result["contact_advisory_required"] = args.contact_advisory
    result["contact_startup_turn_area"] = args.contact_startup_turn_area
    result["experimental_contact_support_core"] = args.contact_support_core
    result["terrain_shadow_status"] = getattr(node, "terrain_status", None)
    if args.natural_motion:
        result["experiment"] = "natural_motion_gt_start"
        result["initialization"] = "one-time live GT seed; no ongoing GT corrections"
    if args.natural_stationary:
        result["experiment"] = "natural_stationary_recovery_holdout"
        result["initialization"] = "live supervisor normal start; exact GT-start qualification requires separate replay"
    required = REQUIRED_TOPICS | ({"/astrobot_0/imu_physics", "/astrobot_0/imu_physics/clock"}
                                 if args.require_physics_imu else set())
    if args.terrain_library:
        required |= set(TERRAIN_TOPICS)
    if args.contact_advisory:
        required.add("/terrain_shadow/contact_advisory")
    if args.isolated_gap_costs:
        required.update(("/terrain_shadow/planner_costmap", "/terrain_shadow/uncertainty_mask"))
    if args.natural_motion in TRIAL_PATTERNS:
        required.add("/terrain_trial/proposal")
    if args.selected_lidar:
        required = selected_topics(required)
    result["missing_required_topics"] = sorted(t for t in required if not counts.get(t))
    result["last_readiness_checks"] = getattr(node, "last_readiness_checks", None)
    result["recorder_source_sha256"] = getattr(node, "recorder_sources", {})
    result["recorder_processing"] = dict(maximum_s=getattr(node, "processing_max_s", {}),
        slow_threshold_s=.05, slow_count=getattr(node, "slow_processing_count", 0),
        last_128_slow_events=list(getattr(node, "slow_processing", [])), diagnostic_only=True)
    result["native_acquisition_finished_stamp"] = getattr(node, "finished_stamp", None)
    result["last_localization_status"] = (dict(state=node.status.state, reason=node.status.reason,
        epoch=node.status.epoch) if node is not None and node.status is not None else None)
    if result["collection_completed"]:
        try:
            check_run(run)
            if not (run / "bag/metadata.yaml").is_file() or not list((run / "bag").glob("*.mcap")):
                raise ValueError("recording did not finalize metadata/MCAP")
        except Exception as exc:
            result.update(collection_completed=False, failure=str(exc))
    with (run / "collection.json").open("x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result["collection_completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
