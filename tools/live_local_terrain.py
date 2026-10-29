#!/usr/bin/env python3
"""Opt-in native + GLIM v2 advisory grids. No GT labels or motion authority.

Start with the guarded recorder before revision 1. Archive raw bytes and actual
receipt order. Unknown is published at startup, failure, completion and shutdown.
The incremental policy cannot repair graph changes: its reference witness and
unsupported revision barriers fail closed, never silently keep a stale map.
"""

from array import array
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import resource
import time

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import GIB, allocate_run, check_run
from build_local_terrain import validate_build
from export_shadow_trajectory import revision_record, stamp_ns
from local_terrain.core import LocalConfig, LocalReconstructor
from local_terrain.costmap import inflate, map_from_grid
from local_terrain.nav2_contract import (DEFAULT_CONFIG as NAV2_CONFIG, Footprint, Nav2Config, clear_footprint,
                                         grid_pose, planner_costs)
from local_terrain.navigation_cost import PUBLISHED_POLICIES
from local_terrain.rolling_mapper import RollingTerrainMapper
from local_terrain.self_filter import chassis_returns
from local_terrain.global_stitch import GlobalStitch
from native_chunk_transport import decode
from selected_lidar_profile import profile_path, selected_setup, validate_request
from shadow_capture_gate import HEARTBEAT_S
from terrain_shadow_runtime import ReceiptScheduler

DEPLOYMENTS = ("capture", "nav")
DUPLICATE_AUTHORITY_GRACE_S = 10.0


class Discard:
    """Raw/event sink for nav runs without a raw archive (OutputDigest interface)."""

    def write(self, value):
        return len(value)

    def tell(self):
        return 0

    def flush(self):
        pass

    def close(self):
        pass

    def hexdigest(self):
        return None


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock as ROSClock, ClockType
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from astrobot_interfaces.msg import SlamRevision
    from astrobot_interfaces.srv import ResetMap
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import PointCloud2
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    class LocalNode(Node):
        def __init__(self):
            super().__init__("terrain_local_v2")
            parameter = lambda name, default: self.declare_parameter(name, default).value
            archive = Path(parameter("archive_root", "/archive"))
            library = Path(parameter("library", "")).resolve(strict=True)
            library_hashes = validate_build(library, archive)
            request_path = Path(parameter("request_json", "")).resolve(strict=True)
            request = json.loads(request_path.read_text())
            # capture: one recorder-granted capture. nav: static rover request; the capture id
            # is adopted from the sim stream, and the supervisor resets the map per GLIM epoch.
            self.deployment = parameter("deployment", "capture")
            if self.deployment not in DEPLOYMENTS:
                raise ValueError("undeclared terrain deployment")
            self.nav = self.deployment == "nav"
            if self.nav:
                if request.get("capture") is not None or request.get("pattern") != "nav":
                    raise ValueError("nav request must be the static pattern=nav request without a capture")
                request = dict(request, capture=None)
            validate_request(request)
            if request["lidar_profile"]["name"] != "selected_optical_v2":
                raise ValueError("live v2 requires optical-origin corrected sensing")
            self.request = request
            # planner_costmap: "" (not published), "isolated" (the legacy isolated_gap_costs
            # experiment) or "nav2_bounded_shadow" (the C1-gated Nav2 input). Every policy
            # publishes with the rover's own unpadded footprint cleared (nav2_contract).
            legacy = "isolated" if parameter("isolated_gap_costs", False) else ""
            self.planner_policy_name = parameter("planner_gap_policy", "") or legacy
            if self.planner_policy_name not in ("", *PUBLISHED_POLICIES) or legacy and self.planner_policy_name != legacy:
                raise ValueError("undeclared or conflicting planner gap policy")
            self.planner_policy = PUBLISHED_POLICIES.get(self.planner_policy_name)
            self.isolated_gap_costs = self.planner_policy is not None
            nav2_config = Nav2Config.from_yaml()
            self.planner_footprint = Footprint((60, 60), (0.0, 0.0), 0.1, nav2_config)
            self.setup, _ = selected_setup(profile_path(request["lidar_profile"]["name"]))
            config = LocalConfig(
                window_s=5,
                evidence_lifetime_s=60,
                observation_policy="frozen_observation",
                backend="prepared",
                map_policy="rolling_incremental",
                allow_bookkeeping=True,
                # GLIM bookkeeping revisions carry the whole trajectory (4,842 frames at 485 s):
                # 8192 ends a mission at ~14 min. Accept is linear, 167 ms at 32,768 (Oct 22).
                # Live nav stream: skip dropped/late chunks instead of faulting the map (seed-411
                # gate, Oct 27: one discontinuity forced a GLIM restart and wiped the mosaic).
                **(dict(max_bookkeeping_frames=32768, tolerate_stream_gaps=True) if self.nav else {}),
            )
            self.make_core = lambda: LocalReconstructor(
                lambda origin, lifetime: RollingTerrainMapper(
                    library, width=60, height=60, origin=origin, lifetime=lifetime
                ),
                request["base_from_imu"],
                config,
            )
            self.core = self.make_core()
            self.scheduler = ReceiptScheduler(self.core)
            self.archive_raw = parameter("archive_raw", True)
            if self.nav:
                limit = int(parameter("archive_quota_gib", 24 if self.archive_raw else 2)) * GIB
                # A nav run writes MBs without the raw archive; the 50 GiB default is sized for captures.
                reserve = dict(reserve=int(parameter("archive_reserve_gib", 20)) * GIB)
            else:
                from costmap_scenarios import archive_limit
                limit = archive_limit(request.get("pattern"), request.get("execution_budget", {}).get("profile", "default"))
                reserve = {}
            self.run = allocate_run(archive, limit=limit, **reserve)
            self.get_logger().info(f"Local v2 archive: {self.run}")
            directory = Path(__file__).resolve().parent
            paths = [
                Path(__file__).resolve(),
                request_path,
                profile_path(request["lidar_profile"]["name"]),
                NAV2_CONFIG,  # planner footprint (Nav2Config.from_yaml)
                *sorted((directory / "local_terrain").glob("*.py")),
                *[
                    directory / name
                    for name in (
                        "terrain_shadow_runtime.py",
                        "terrain_shadow_core.py",
                        "native_chunk_transport.py",
                        "shadow_terrain.py",
                        "revision_evidence.py",
                        "layered_terrain.py",
                        "raw_return_filter.py",
                        "selected_lidar_profile.py",
                        "costmap_scenarios.py",
                        "export_shadow_trajectory.py",
                        "local_terrain_seal.py",
                    )
                ],
            ]
            self.hashes = dict(library_hashes, **{str(p): sha(p) for p in paths})
            self.manifest = dict(
                schema=1,
                policy="live_local_v2",
                config=asdict(config),
                library=str(library),
                library_sha256=sha(library),
                input_sha256=self.hashes,
                request=request,
                inflation_radius_m=1.0,
                isolated_gap_costs=asdict(self.planner_policy) if self.isolated_gap_costs else None,
                planner_gap_policy=self.planner_policy_name or None,
                planner_footprint_cleared=asdict(nav2_config) if self.isolated_gap_costs else None,
                navigation_authority=False,
                acceptance=False,
                output_frame="map",
                native_self_filter="calibrated_chassis_only",
                deployment=self.deployment,
                archive_raw=self.archive_raw,
            )
            with (self.run / "manifest.json").open("x") as stream:
                json.dump(self.manifest, stream, indent=2)
            from local_terrain_seal import OutputDigest

            self.events = OutputDigest(self.run / "events.jsonl") if self.archive_raw else Discard()
            self.raw = OutputDigest(self.run / "native.bin", binary=True) if self.archive_raw else Discard()
            self.projections = OutputDigest(self.run / "projections.jsonl")
            self.map_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/costmap", 1)
            self.class_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/classes", 1)
            if self.isolated_gap_costs:
                self.planner_cost_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/planner_costmap", 1)
                self.uncertainty_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/uncertainty_mask", 1)
            self.status_pub = self.create_publisher(String, "/terrain_shadow/status", 10)
            # nav: epoch of the current GLIM child (None until the supervisor's first reset) and
            # the first native stamp to integrate (the epoch's first ODOM frame)
            self.epoch = self.minimum_ns = self.armed_ns = None
            self.stream_capture = None
            self.duplicate_authority_since = None
            self.mosaic = self.explore = None
            self.mosaic_half_extent = float(parameter("mosaic_half_extent_m", 30.0))
            if self.nav:
                if not self.isolated_gap_costs:
                    raise ValueError("nav deployment requires a planner gap policy")
                self.mosaic_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/global_costmap", 1)
                self.explore_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/exploration_map", 1)
                self.create_service(ResetMap, "/astrobot_0/slam/reset_map", self.reset_map)
                self.create_timer(1.0, self.publish_mosaic, clock=ROSClock(clock_type=ClockType.STEADY_TIME))
            self.contact_advisory = None
            if parameter("contact_advisory", False):
                if self.nav:
                    raise ValueError("contact advisory is capture-only")
                from local_terrain.contact_runtime import ContactRuntime

                self.contact_advisory = ContactRuntime(
                    self,
                    deployment_confirmed=parameter("contact_deployment_confirmed", False),
                    support_core=parameter("contact_support_core", False),
                    startup_turn_area=parameter("contact_startup_turn_area", False),
                    current_turn_area=parameter("contact_current_turn_area", False),
                )
            reliable = QoSProfile(
                depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
            self.create_subscription(SlamRevision, "/astrobot_0/slam_raw/revisions", self.revision, reliable)
            # 64 chunks = ~1 sim-s of callback stall before the reader drops (16 was ~0.27 sim-s).
            self.create_subscription(PointCloud2, "/terrain_shadow/native_chunks", self.chunk, 64)
            self.create_subscription(String, "/terrain_shadow/native_complete", self.complete, 1)
            self.create_subscription(Clock, "/clock", self.clock, 100)
            self.last_clock_wall = time.monotonic()
            self.last_grid_pose = np.eye(4)
            self.create_timer(0.1, self.watchdog, clock=ROSClock(clock_type=ClockType.STEADY_TIME))
            if rclpy.ok():
                self.publish(None)

        def event(self, kind, payload):
            check_run(self.run)
            self.events.write(
                json.dumps(dict(kind=kind, payload=payload, receipt_wall_ns=time.monotonic_ns()), allow_nan=False)
                + "\n"
            )

        def guarded(self, function):
            if self.core.failure:
                return
            try:
                function()
            except Exception as exc:
                self.core.fault(exc)
                self.scheduler.pending.clear()
                self.scheduler.bytes = 0
                self.get_logger().error(str(exc))
                try:
                    self.event("fault", str(exc))
                except Exception:
                    pass
                self.publish(None)

        def revision(self, message):
            def action():
                record = dict(revision_record(message), kind=message.kind)
                if self.nav:
                    # Only the epoch the supervisor reset the map for; a previous child's
                    # late revisions (or any before the first reset) never reach the core.
                    if self.epoch is None or record["epoch"] != self.epoch:
                        return
                    if self.armed_ns is None:
                        if record["kind"] != 1 or not record["frames"]:
                            return
                        self.armed_ns = max(self.minimum_ns, min(f["stamp_ns"] for f in record["frames"]))
                self.event("revision", record)
                self.scheduler.receive("revision", record, len(json.dumps(record)))
                if self.contact_advisory is not None:
                    self.contact_advisory.receive(record)

            self.guarded(action)

        def chunk(self, message):
            def action():
                chunk = decode(message)
                if self.nav:
                    if self.stream_capture is None:
                        self.stream_capture = chunk["capture"]
                    if chunk["capture"] != self.stream_capture:
                        raise ValueError("foreign native capture")
                    if self.armed_ns is None or chunk["stamp_ns"] < self.armed_ns:
                        return  # no pose state for this epoch yet
                    if self.core.capture is None and chunk["sequence"] > 0:
                        self.core.resume_after(chunk["capture"], chunk["sequence"] - 1)
                elif chunk["capture"] != self.request["capture"]:
                    raise ValueError("foreign native capture")
                data = bytes(message.data)
                offset = self.raw.tell()
                self.raw.write(data)
                self.raw.flush()
                self.event(
                    "chunk",
                    dict(
                        offset=offset,
                        size=len(data),
                        stamp_ns=chunk["stamp_ns"],
                        sequence=chunk["sequence"],
                        capture=chunk["capture"],
                        sha256=hashlib.sha256(data).hexdigest(),
                    ),
                )
                # Keep original wire bytes above. Zero rejected XYZ only in the
                # classifier input (outside valid range), preserving tick identity.
                points = chunk["points"].copy()
                rejected = chassis_returns(points, self.setup) | (chunk["intensities"] == 0)
                points[rejected] = 0
                chunk["points"] = points
                self.scheduler.receive("chunk", chunk, len(data))

            self.guarded(action)

        def complete(self, message):
            def action():
                payload = json.loads(message.data)
                self.event("complete", payload)
                self.scheduler.receive("complete", payload, len(message.data))

            self.guarded(action)

        def projection_line(self, record):
            """Without the raw archive nothing can be replayed, so the per-chunk association list
            (the whole 60 s evidence window, ~270 KB per record) is reduced to its length: full
            records filled the 2 GiB nav quota at ~800 sim-s (seed-411 gate, Oct 28)."""
            if not self.archive_raw and "associations" in record:
                record = dict(record, associations=None, association_count=len(record["associations"]))
            return json.dumps(record, allow_nan=False) + "\n"

        def clock(self, message):
            def action():
                self.last_clock_wall = time.monotonic()
                stamp = stamp_ns(message.clock)
                self.event("clock", stamp)
                layers = self.scheduler.clock(stamp)
                if layers is not None:
                    self.projections.write(self.projection_line(self.core.last_record))
                    self.publish(None if self.core.completed else layers)
                elif self.core.completed:
                    self.publish(None)
                if self.contact_advisory is not None and not self.core.completed:
                    self.contact_advisory.clock(stamp, layers, self.core.last_record)

            self.guarded(action)

        def status(self):
            value = self.core.status()
            value.update(
                policy="live_local_v2",
                archive=str(self.run),
                library_sha256=self.manifest["library_sha256"],
                capture=self.core.capture,
                integrated=self.core.integrated_sequence + 1,
                deployment=self.deployment,
                epoch=self.epoch,
                valid=value["valid"] and not self.core.completed,
                peak_process_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
            )
            return value

        def publish(self, layers):
            if layers is None and self.contact_advisory is not None:
                self.contact_advisory.invalidate("terrain output unavailable")
            classes = np.full((60, 60), -1, dtype=np.int8) if layers is None else layers[:, :, 0].astype(np.int8)
            if layers is not None:
                self.last_grid_pose = map_from_grid(self.core.last_record)
            outputs = [(self.class_pub, classes), (self.map_pub, inflate(classes))]
            if self.isolated_gap_costs:
                if layers is None:
                    planner_cost, uncertainty = classes.copy(), classes.copy()
                else:
                    planner_cost, mask = planner_costs(layers, self.core.last_record, self.planner_policy,
                                                       self.planner_footprint)
                    uncertainty = mask.astype(np.int8) * 100
                    if self.nav:
                        # Nav2's local costmap reads this window: send it from the mosaic so an
                        # obstacle out of view (behind the front LiDAR) or past the 60 s evidence
                        # lifetime stays lethal; MPPI reversed into such a 0.34 m rock (seed 411).
                        self.stitch(planner_cost, classes)
                        planner_cost = self.mosaic.window(self.last_grid_pose[:2, 3], planner_cost.shape)
                outputs.extend([(self.planner_cost_pub, planner_cost), (self.uncertainty_pub, uncertainty)])
            for publisher, values in outputs:
                message = OccupancyGrid()
                message.header.frame_id = "map"
                stamp = max(0, self.core.watermark_ns)
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
                message.info.width = message.info.height = 60
                message.info.resolution = 0.1
                p, q = message.info.origin.position, message.info.origin.orientation
                p.x, p.y, p.z = map(float, self.last_grid_pose[:3, 3])
                q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(self.last_grid_pose[:3, :3]).as_quat())
                message.data = array("b", np.ascontiguousarray(values, dtype=np.int8).tobytes())
                publisher.publish(message)
            self.status_pub.publish(String(data=json.dumps(self.status(), allow_nan=False)))

        def stitch(self, planner_cost, classes):
            """Two mosaics: Nav2 costs (global costmap) and observed classes (exploration map:
            0 free, 100 occupied, -1 unknown, as the legacy BEV map; no gap policy or taper).
            The exploration map also takes the ground under the unpadded body as free where nothing
            observed it: the front LiDAR never sees the spawn footprint, and that unknown patch left
            the rover with no 0.75 m-clear start cell (seed-411 gate, Oct 27)."""
            if not np.allclose(self.last_grid_pose[:3, :3], np.eye(3), atol=1e-9):
                raise ValueError("rotated planner window; the mosaic is axis-aligned")
            origin = self.last_grid_pose[:2, 3]
            if self.mosaic is None:
                centre = origin + 0.5 * np.array(planner_cost.shape[::-1]) * 0.1
                lo, hi = centre - self.mosaic_half_extent, centre + self.mosaic_half_extent
                self.mosaic, self.explore = GlobalStitch(lo, hi, origin, 0.1), GlobalStitch(lo, hi, origin, 0.1)
            self.mosaic.update(planner_cost, origin, classes != -1)
            body = clear_footprint(np.full(classes.shape, -1, np.int8), self.planner_footprint,
                                   grid_pose(self.core.last_record)) == 0
            explore = np.where(classes == 20, 0, classes).astype(np.int8)
            explore[body & (classes == -1)] = 0
            self.explore.update(explore, origin, classes != -1)

        def publish_mosaic(self):
            if self.mosaic is None:
                return
            for publisher, mosaic in ((self.mosaic_pub, self.mosaic), (self.explore_pub, self.explore)):
                message = OccupancyGrid()
                message.header.frame_id = "map"
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(max(0, self.core.watermark_ns), 10**9)
                message.info.height, message.info.width = mosaic.grid.shape
                message.info.resolution = 0.1
                message.info.origin.position.x, message.info.origin.position.y = map(float, mosaic.origin)
                message.info.origin.orientation.w = 1.0
                # array, not a 360k-element list: a slow publish backs up this node's /clock queue
                message.data = array("b", mosaic.grid.tobytes())
                publisher.publish(message)

        def reset_map(self, request, response):
            """Supervisor map reset before each GLIM child: fresh core and mosaic for the new epoch."""
            response.epoch = request.epoch
            if self.epoch is not None and request.epoch < self.epoch:
                response.success, response.reason = False, "stale_epoch"
                return response
            try:
                old = self.core
                self.core = self.make_core()
                self.scheduler = ReceiptScheduler(self.core)
                old.close()
                self.epoch, self.armed_ns, self.mosaic, self.explore = request.epoch, None, None, None
                self.minimum_ns = stamp_ns(request.minimum_stamp)
                self.event("reset", dict(epoch=request.epoch, minimum_ns=self.minimum_ns))
                self.publish(None)
                response.success = True
            except Exception as exc:
                self.get_logger().error(f"map reset failed: {exc}")
                response.success, response.reason = False, str(exc) or type(exc).__name__
            return response

        def watchdog(self):
            if self.core.failure or self.core.completed:
                self.publish(None)
                return

            def action():
                check_run(self.run)
                if self.core.capture is not None and time.monotonic() - self.last_clock_wall > HEARTBEAT_S:
                    raise ValueError("simulation wall heartbeat lost")
                if any(
                    self.count_publishers(t) > 1
                    for t in ("/terrain_shadow/native_chunks", "/astrobot_0/slam_raw/revisions")
                ):
                    # A supervisor GLIM restart briefly shows the old and new revision publishers
                    # in discovery; nav accepts only the reset epoch, so fault only if it persists
                    # (seed-7 gate, Oct 28: the transient faulted the freshly reset core).
                    now = time.monotonic()
                    if self.duplicate_authority_since is None:
                        self.duplicate_authority_since = now
                    if not self.nav or now - self.duplicate_authority_since > DUPLICATE_AUTHORITY_GRACE_S:
                        raise ValueError("multiple v2 input authorities")
                else:
                    self.duplicate_authority_since = None
                self.status_pub.publish(String(data=json.dumps(self.status(), allow_nan=False)))

            self.guarded(action)

        def finish(self):
            for path, digest in self.hashes.items():
                if sha(path) != digest:
                    self.core.fault("runtime input changed: " + path)
            self.core.close()
            if rclpy.ok():
                self.publish(None)
            for stream in (self.events, self.raw, self.projections):
                stream.close()
            if self.contact_advisory is not None:
                self.contact_advisory.finish()
            check_run(self.run)
            with (self.run / "collection.json").open("x") as stream:
                json.dump(
                    dict(
                        status=self.status(),
                        experiment_completed=self.core.completed and self.core.failure is None,
                        acceptance=False,
                        output_sha256={
                            name: output.hexdigest()
                            for name, output in zip(
                                ("events.jsonl", "native.bin", "projections.jsonl"),
                                (self.events, self.raw, self.projections),
                            )
                        },
                    ),
                    stream,
                    indent=2,
                )

    # A launcher SIGINT must not interrupt a core transaction and become a
    # KeyboardInterrupt fault. Finish the active callback, then seal while the
    # ROS context is still alive. Recorder's existing shutdown deadline remains.
    from local_terrain.shutdown import DeferredInterrupt

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    with DeferredInterrupt() as stop:
        try:
            node = LocalNode()
            while rclpy.ok() and not stop.requested:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            if node is not None:
                node.finish()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
