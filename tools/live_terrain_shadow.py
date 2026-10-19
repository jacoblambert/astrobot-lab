#!/usr/bin/env python3
"""Opt-in live native/first-published terrain mapper. NEVER publishes commands.

Archives actual callback order and all projection hashes for independent exact
replay. Outputs are advisory, never the navigation BEV topic. Start before GLIM
revision 1; late joining cannot reconstruct first-publication availability.
"""
import hashlib
import json
from pathlib import Path
import resource
import time

import numpy as np

from archive_guard import allocate_run, check_run
from export_shadow_trajectory import revision_record, stamp_ns
from layered_terrain import TerrainMapper
from native_chunk_transport import decode
from terrain_shadow_core import ShadowCore
from terrain_shadow_runtime import ReceiptScheduler


def sha_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.clock import Clock as ROSClock, ClockType
    from astrobot_interfaces.msg import SlamRevision
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    class ShadowNode(Node):
        def __init__(self):
            super().__init__("terrain_shadow_mapper")
            def parameter(name, default):
                return self.declare_parameter(name, default).value
            library = Path(parameter("library", "")).resolve(strict=True)
            if not library.is_file():
                raise ValueError("explicit frozen terrain library required")
            geometry = dict(width=200, height=200, resolution=.1, lifetime=30., interpolate=True,
                            origin=[parameter("origin_x", float("nan")), parameter("origin_y", float("nan"))])
            minimum, maximum = parameter("min_range", float("nan")), parameter("max_range", float("nan"))
            if not np.isfinite(geometry["origin"]).all():
                raise ValueError("explicit frozen map origin required")
            self.core = ShadowCore(lambda: TerrainMapper(library, **geometry), minimum=minimum, maximum=maximum)
            self.scheduler = ReceiptScheduler(self.core)
            self.run = allocate_run(parameter("archive_root", "/archive"))
            self.get_logger().info(f"Shadow archive: {self.run}")
            sources = [Path(__file__), *[Path(__file__).with_name(n) for n in
                       ("terrain_shadow_core.py", "terrain_shadow_runtime.py", "native_chunk_transport.py",
                        "layered_terrain.py", "shadow_terrain.py", "raw_return_filter.py",
                        "revision_evidence.py", "export_shadow_trajectory.py", "archive_guard.py")]]
            self.manifest = dict(schema=1, library=str(library), library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                geometry=geometry, minimum=minimum, maximum=maximum, navigation_authority=False,
                association_deadline_ns=200_000_000, projection_hz=10, output_frame="map")
            with (self.run / "manifest.json").open("x") as stream:
                json.dump(self.manifest, stream, indent=2)
            self.events = (self.run / "events.jsonl").open("x", buffering=1)
            self.raw = (self.run / "native.bin").open("xb")
            self.frames = (self.run / "projections.jsonl").open("x", buffering=1)
            self.map_pub = self.create_publisher(OccupancyGrid, "/terrain_shadow/costmap", 1)
            self.status_pub = self.create_publisher(String, "/terrain_shadow/status", 10)
            reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(SlamRevision, "/astrobot_0/slam_raw/revisions", self.revision, reliable)
            self.create_subscription(PointCloud2, "/terrain_shadow/native_chunks", self.chunk, 16)
            self.create_subscription(String, "/terrain_shadow/native_complete", self.complete, 1)
            self.create_subscription(Clock, "/clock", self.clock, 100)
            self.last_clock_wall = time.monotonic()
            self.create_timer(.1, self.watchdog, clock=ROSClock(clock_type=ClockType.STEADY_TIME))

        def event(self, kind, payload):
            check_run(self.run)
            self.events.write(json.dumps(dict(kind=kind, payload=payload, receipt_wall_ns=time.monotonic_ns()),
                                         allow_nan=False) + "\n")

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
                    pass  # Archive loss never permits a root-disk fallback.
                self.publish(None)

        def revision(self, message):
            def action():
                record = revision_record(message)
                record["kind"] = message.kind
                payload_size = len(json.dumps(record))
                self.event("revision", record)
                self.scheduler.receive("revision", record, payload_size)
            self.guarded(action)

        def chunk(self, message):
            def action():
                chunk = decode(message)
                payload = bytes(message.data)
                offset = self.raw.tell()
                self.raw.write(payload)
                self.raw.flush()
                self.event("chunk", dict(offset=offset, size=len(payload), stamp_ns=chunk["stamp_ns"],
                    capture=chunk["capture"], sequence=chunk["sequence"], sha256=hashlib.sha256(payload).hexdigest()))
                self.scheduler.receive("chunk", chunk, len(payload))
            self.guarded(action)

        def clock(self, message):
            def action():
                self.last_clock_wall = time.monotonic()
                stamp = stamp_ns(message.clock)
                self.event("clock", stamp)
                layers = self.scheduler.clock(stamp)
                if layers is not None:
                    record = dict(status=self.core.status(), associations=self.core.last_associations)
                    self.frames.write(json.dumps(record, allow_nan=False) + "\n")
                    self.publish(layers)
            self.guarded(action)

        def complete(self, message):
            def action():
                payload = json.loads(message.data)
                self.event("complete", payload)
                self.scheduler.receive("complete", payload, len(message.data))
            self.guarded(action)

        def publish(self, layers):
            message = OccupancyGrid()
            message.header.frame_id = "map"
            stamp = max(0, self.core.watermark_ns)
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            message.info.width = message.info.height = 200
            message.info.resolution = .1
            message.info.origin.position.x, message.info.origin.position.y = self.manifest["geometry"]["origin"]
            message.info.origin.orientation.w = 1.
            message.data = (np.full(40000, -1, dtype=np.int8) if layers is None else
                            layers[:, :, 0].astype(np.int8).ravel()).tolist()
            self.map_pub.publish(message)
            status = self.message_status()
            status.update(peak_process_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.,
                          receipt_pending=len(self.scheduler.pending), receipt_pending_bytes=self.scheduler.bytes)
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

        def message_status(self):
            return dict(self.core.status(), archive=str(self.run), library_sha256=self.manifest["library_sha256"])

        def watchdog(self):
            if self.core.failure:
                self.publish(None)
                return
            def action():
                check_run(self.run)
                if self.core.capture is not None and not self.core.completed and time.monotonic() - self.last_clock_wall > .5:
                    raise ValueError("simulation wall heartbeat lost")
                if (self.count_publishers("/terrain_shadow/native_chunks") > 1 or
                        self.count_publishers("/astrobot_0/slam_raw/revisions") > 1):
                    raise ValueError("multiple shadow input authorities")
                self.status_pub.publish(String(data=json.dumps(self.message_status(), allow_nan=False)))
            self.guarded(action)

        def finish(self):
            for path, expected in {**self.manifest["source_sha256"],
                    self.manifest["library"]: self.manifest["library_sha256"]}.items():
                if sha_file(path) != expected:
                    self.core.fault("runtime source/library changed: " + path)
            self.core.close()
            for stream in (self.raw, self.events, self.frames):
                stream.close()
            check_run(self.run)
            summary = dict(status=self.core.status(), experiment_completed=self.core.completed and not self.core.failure,
                           note="Software stream completion only; independent sensor and live qualification remain required.",
                           output_sha256={n: sha_file(self.run / n)
                                          for n in ("events.jsonl", "native.bin", "projections.jsonl")})
            with (self.run / "collection.json").open("x") as stream:
                json.dump(summary, stream, indent=2)

    rclpy.init()
    node = None
    try:
        node = ShadowNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.finish()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
