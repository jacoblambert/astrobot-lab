#!/usr/bin/env python3
"""Upper-capacity bookkeeping callback microbenchmark, not live acceptance."""

import json
from pathlib import Path
import resource
import time

import numpy as np

from archive_guard import allocate_run, ARCHIVE_ROOT
from local_terrain.bookkeeping import BookkeepingLedger
from shadow_terrain import FramePose, PoseTimeline
from replay_local_terrain import sha


def main():
    output = allocate_run(ARCHIVE_ROOT)
    print(f"Run: {output}", flush=True)
    poses = [FramePose(i, i, np.eye(4), np.eye(4)) for i in range(8192)]
    ledger = BookkeepingLedger(8192)
    ledger.validate_odom(poses)
    ledger.commit_odom(poses)
    timeline = PoseTimeline("benchmark", 1)
    timeline.apply(session="benchmark", epoch=1, revision=1, parent=0, arrival_ns=8192, frames=[poses[0], *poses[-51:]])
    rows = [
        dict(
            frame_id=p.frame_id,
            stamp_ns=p.stamp_ns,
            map_from_imu=p.map_from_imu.tolist(),
            imu_from_lidar=p.imu_from_lidar.tolist(),
            deskew_recomputed=False,
        )
        for p in poses
    ]
    elapsed = []
    for revision in range(2, 34):
        record = dict(
            kind=2 if revision % 2 else 3,
            session="benchmark",
            epoch=1,
            revision=revision,
            parent=revision - 1,
            snapshot=False,
            frames=rows,
        )
        start = time.perf_counter()
        ledger.accept(record, 8192 + revision, timeline)
        elapsed.append((time.perf_counter() - start) * 1000)
    report = dict(
        acceptance=False,
        frames=8192,
        batches=32,
        all_callback_ms=elapsed,
        p95_ms=float(np.percentile(elapsed, 95)),
        max_ms=max(elapsed),
        peak_process_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
        retained_pose_array_bytes=sum(f.map_from_imu.nbytes + f.imu_from_lidar.nbytes for f in ledger.frames.values()),
        retained_interpolation_frames=len(timeline.frames),
        source_sha256={
            str(p): sha(p) for p in (Path(__file__), Path(__file__).parent / "local_terrain/bookkeeping.py")
        },
        note="All samples retained. Separate host upper-capacity microbenchmark; no DDS/deserialization, simulator, concurrent compute or live heartbeat qualification.",
    )
    with (output / "report.json").open("x") as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
