#!/usr/bin/env python3
"""Offline callback-processing diagnostic; not live heartbeat/compute acceptance."""

import argparse
import gc
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from archive_guard import allocate_run, check_run
from export_shadow_trajectory import apply_record, revision_record, stamp_ns
from live_revision_gate import LiveRevisionGate
from shadow_pose_replay import sha
from shadow_terrain import PoseTimeline


def profile(recorder, kind):
    from astrobot_interfaces.msg import SlamRevision
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from rosgraph_msgs.msg import Clock

    if not gc.isenabled():
        raise ValueError("processing diagnostic requires normal enabled GC")
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(recorder / "bag"), storage_id="mcap"), ConverterOptions("", ""))
    reader.set_filter(StorageFilter(topics=["/clock", "/astrobot_0/slam_raw/revisions"]))
    gate, clock, rows = None, 0, []
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        if topic == "/clock":
            clock = stamp_ns(deserialize_message(data, Clock).clock)
            continue
        started = time.monotonic()
        message = deserialize_message(data, SlamRevision)
        record = revision_record(message)
        converted = time.monotonic()
        if gate is None:
            gate = (PoseTimeline if kind == "reference" else LiveRevisionGate)(record["session"], record["epoch"])
        # Match the live chain validator, NOT offline receipt-bounded pose
        # association. No poses/availability claims are exported by this tool.
        record["arrival_ns"] = max(
            clock, gate.last_arrival_ns, max((f["stamp_ns"] for f in record["frames"]), default=0)
        )
        if kind == "reference":
            apply_record(gate, record)
        else:
            gate.apply_record(record)
        ended = time.monotonic()
        rows.append(
            dict(
                revision=message.revision,
                kind=message.kind,
                frames=len(message.frames),
                receipt_wall_ns=receipt,
                convert_s=converted - started,
                validate_s=ended - converted,
                total_s=ended - started,
            )
        )
    if gate is None:
        raise ValueError("no recorded revisions")
    state = dict(
        events=gate.events,
        stamps=gate.stamps,
        extrinsic=gate.extrinsic.tolist(),
        revision=gate.revision,
        last_arrival_ns=gate.last_arrival_ns,
    )
    return dict(
        mode=kind,
        callbacks=len(rows),
        processing_p50_p95_p99_max_s=np.percentile([r["total_s"] for r in rows], [50, 95, 99, 100]).tolist(),
        large_batches=[r for r in rows if r["frames"] > 100],
        retained_pose_versions=sum(map(len, gate.frames.values())) if kind == "reference" else 0,
        immutable_frame_identities=len(gate.stamps),
        validation_state_sha256=hashlib.sha256(json.dumps(state, sort_keys=True).encode()).hexdigest(),
        gc_enabled=gc.isenabled(),
        gc_thresholds=list(gc.get_threshold()),
        rows=rows,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recorder", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    check_run(args.recorder)
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        results = [profile(args.recorder, kind) for kind in ("reference", "live_gate")]
        if results[0]["validation_state_sha256"] != results[1]["validation_state_sha256"]:
            raise ValueError("live gate validation state differs from offline reference")
        inputs = [args.recorder / "collection.json", args.recorder / "bag/metadata.yaml"]
        inputs += sorted((args.recorder / "bag").glob("*.mcap"))
        sources = [
            Path(__file__).with_name(n)
            for n in (
                "profile_revision_validation.py",
                "live_revision_gate.py",
                "export_shadow_trajectory.py",
                "shadow_terrain.py",
            )
        ]
        result = dict(
            completed=True,
            acceptance=False,
            diagnostic_only=True,
            results=results,
            input_sha256={str(p): sha(p) for p in inputs},
            source_sha256={str(p): sha(p) for p in sources},
            note="Sequential offline callback benchmark, disk reads excluded. No live deadline acceptance or failed-input promotion.",
        )
        with (output / "revision_processing.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps([{k: v for k, v in r.items() if k != "rows"} for r in results], indent=2))
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
