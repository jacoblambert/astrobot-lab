#!/usr/bin/env python3
"""Read-only bounded v2 topic probe; exact class/cost consistency, not terrain truth."""

import json
import time
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from local_terrain.costmap import inflate


def main():
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String

    output = allocate_run("/archive", limit=256 * 1024**2)
    print(f"Run: {output}", flush=True)
    rclpy.init()
    node = rclpy.create_node("read_only_v2_costmap_probe")
    rows, pending, statuses = [], {}, []
    failure = []

    def receive(message, kind):
        try:
            if message.header.frame_id != "map" or message.info.width != 60 or message.info.height != 60:
                raise ValueError("wrong map frame/dimensions")
            if abs(message.info.resolution - 0.1) > 1e-7:
                raise ValueError("wrong cell size")
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            pair = pending.setdefault(stamp, {})
            pair[kind] = message
            if len(pending) > 32:
                raise ValueError("unmatched grid publications exceed bound")
            if set(pair) != {"classes", "costmap"}:
                return
            classes = np.asarray(pair["classes"].data, dtype=np.int8).reshape(60, 60)
            costs = np.asarray(pair["costmap"].data, dtype=np.int8).reshape(60, 60)
            if pair["classes"].info != pair["costmap"].info or not np.array_equal(costs, inflate(classes)):
                raise ValueError("class/cost geometry or taper differs")
            pending.pop(stamp)
            if not (classes != -1).any():
                return  # Invalid/all-unknown output is correct but is not live-data success.
            rows.append(
                dict(
                    stamp_ns=stamp,
                    known=int((classes != -1).sum()),
                    occupied=int((classes == 100).sum()),
                    measured=int((classes == 0).sum()),
                    inferred=int((classes == 20).sum()),
                    inflated=int(((costs > classes) & (classes >= 0)).sum()),
                )
            )
            if len(rows) == 1:
                np.savez_compressed(output / "first_map.npz", classes=classes, costs=costs)
        except Exception as exc:
            failure.append(str(exc))

    node.create_subscription(OccupancyGrid, "/terrain_shadow/classes", lambda m: receive(m, "classes"), 10)
    node.create_subscription(OccupancyGrid, "/terrain_shadow/costmap", lambda m: receive(m, "costmap"), 10)
    node.create_subscription(String, "/terrain_shadow/status", lambda m: statuses.append(json.loads(m.data)), 10)
    start = time.monotonic()
    try:
        while time.monotonic() - start < 20 and len(rows) < 10 and not failure:
            rclpy.spin_once(node, timeout_sec=0.1)
        passed = len(rows) >= 10 and not failure and bool(statuses) and statuses[-1].get("valid") is True
        report = dict(
            passed=passed,
            rows=rows,
            status=statuses[-1] if statuses else None,
            failures=failure,
            acceptance=False,
            note=__doc__,
        )
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps(report), flush=True)
        if not passed:
            raise ValueError("live costmap probe did not pass")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
