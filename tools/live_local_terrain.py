#!/usr/bin/env python3
"""Opt-in native + GLIM v2 advisory grids. No GT labels or motion authority.

Start with the guarded recorder before revision 1. Archive raw bytes and actual
receipt order. Unknown is published at startup, failure, completion and shutdown.
The incremental policy cannot repair graph changes: its reference witness and
unsupported revision barriers fail closed, never silently keep a stale map.
"""

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import resource
import time

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import allocate_run, check_run
from build_local_terrain import validate_build
from export_shadow_trajectory import revision_record, stamp_ns
from local_terrain.core import LocalConfig, LocalReconstructor
from local_terrain.costmap import inflate, map_from_grid
from local_terrain.nav2_contract import DEFAULT_CONFIG as NAV2_CONFIG, Footprint, Nav2Config, planner_costs
from local_terrain.navigation_cost import PUBLISHED_POLICIES
from local_terrain.rolling_mapper import RollingTerrainMapper
from local_terrain.self_filter import chassis_returns
from native_chunk_transport import decode
from selected_lidar_profile import profile_path, selected_setup, validate_request
from terrain_shadow_runtime import ReceiptScheduler


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
            )
            self.core = LocalReconstructor(
                lambda origin, lifetime: RollingTerrainMapper(
                    library, width=60, height=60, origin=origin, lifetime=lifetime
                ),
                request["base_from_imu"],
                config,
            )
            self.scheduler = ReceiptScheduler(self.core)
            from costmap_scenarios import archive_limit
            self.run = allocate_run(archive, limit=archive_limit(
                request.get("pattern"), request.get("execution_budget", {}).get("profile", "default")))
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
            )
            with (self.run / "manifest.json").open("x") as stream:
                json.dump(self.manifest, stream, indent=2)
            from local_terrain_seal import OutputDigest

            self.events = OutputDigest(self.run / "events.jsonl")
            self.raw = OutputDigest(self.run / "native.bin", binary=True)
            self.projections = OutputDigest(self.run / "projections.jsonl")
            self.map_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/costmap", 1)
            self.class_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/classes", 1)
            if self.isolated_gap_costs:
                self.planner_cost_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/planner_costmap", 1)
                self.uncertainty_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/uncertainty_mask", 1)
            self.status_pub = self.create_publisher(String, "/terrain_shadow/status", 10)
            self.contact_advisory = None
            if parameter("contact_advisory", False):
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
            self.create_subscription(PointCloud2, "/terrain_shadow/native_chunks", self.chunk, 16)
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
                self.event("revision", record)
                self.scheduler.receive("revision", record, len(json.dumps(record)))
                if self.contact_advisory is not None:
                    self.contact_advisory.receive(record)

            self.guarded(action)

        def chunk(self, message):
            def action():
                chunk = decode(message)
                if chunk["capture"] != self.request["capture"]:
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

        def clock(self, message):
            def action():
                self.last_clock_wall = time.monotonic()
                stamp = stamp_ns(message.clock)
                self.event("clock", stamp)
                layers = self.scheduler.clock(stamp)
                if layers is not None:
                    self.projections.write(json.dumps(self.core.last_record, allow_nan=False) + "\n")
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
                message.data = values.ravel().tolist()
                publisher.publish(message)
            self.status_pub.publish(String(data=json.dumps(self.status(), allow_nan=False)))

        def watchdog(self):
            if self.core.failure or self.core.completed:
                self.publish(None)
                return

            def action():
                check_run(self.run)
                if self.core.capture is not None and time.monotonic() - self.last_clock_wall > 0.5:
                    raise ValueError("simulation wall heartbeat lost")
                if any(
                    self.count_publishers(t) > 1
                    for t in ("/terrain_shadow/native_chunks", "/astrobot_0/slam_raw/revisions")
                ):
                    raise ValueError("multiple v2 input authorities")
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
