#!/usr/bin/env python3
"""Export causal/revised IMU poses from a completed revision MCAP fixture.

Creates a fresh archive run, never changes the recording. Receipt-time simulation
bounds come from the next /clock message. No moving-terrain acceptance claim.
ROS imports are lazy so receipt-order/trajectory contracts run on host Python.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from shadow_terrain import FramePose, PoseTimeline, clock_bound_events


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def pose_matrix(pose):
    xyz = [pose.position.x, pose.position.y, pose.position.z]
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    if not np.isfinite([*xyz, *q]).all() or abs(q @ q - 1.) > 1e-6:
        raise ValueError("invalid recorded pose/quaternion")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(q).as_matrix()
    result[:3, 3] = xyz
    return result


def revision_record(msg):
    if msg.header.frame_id != "map":
        raise ValueError("revision must be in map frame")
    return dict(session=msg.session_id, epoch=msg.localization_epoch,
                revision=msg.revision, parent=msg.parent_revision, snapshot=msg.full_snapshot,
                frames=[dict(frame_id=f.frame_id, stamp_ns=stamp_ns(f.measurement_stamp),
                             map_from_imu=pose_matrix(f.map_from_imu).tolist(),
                             imu_from_lidar=pose_matrix(f.imu_from_lidar).tolist(),
                             deskew_recomputed=f.deskew_recomputed) for f in msg.frames])


def apply_record(timeline, record):
    """Native local returns do not inherit GLIM's preprocessed/deskewed points."""
    frames = [FramePose(f["frame_id"], f["stamp_ns"], f["map_from_imu"], f["imu_from_lidar"])
              for f in record["frames"]]
    return timeline.apply(**{key: record[key] for key in
                            ("session", "epoch", "revision", "parent", "arrival_ns", "snapshot")}, frames=frames)


def bag_events(bag):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rosgraph_msgs.msg import Clock
    from astrobot_interfaces.msg import SlamRevision

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id="mcap"), ConverterOptions("", ""))
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        if topic == "/clock":
            yield receipt, "clock", stamp_ns(deserialize_message(data, Clock).clock)
        elif topic == "/astrobot_0/slam_raw/revisions":
            yield receipt, "revision", revision_record(deserialize_message(data, SlamRevision))


def export(source, output):
    check_run(source)
    collection = json.loads((source / "collection.json").read_text())
    if not collection.get("collection_completed") or collection.get("failure"):
        raise ValueError("failed/incomplete recording is ineligible")
    if (source / "orchestration_failure.json").exists():
        raise ValueError("failed recording orchestration is ineligible")
    timeline, records = None, 0
    files = [source / "collection.json", source / "bag/metadata.yaml", *sorted((source / "bag").glob("*.mcap"))]
    if not files[-1].suffix == ".mcap":
        raise ValueError("missing MCAP input")
    hashes = {}
    for path in files:
        check_run(output)
        with path.open("rb") as stream:
            digest = hashlib.sha256()
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        hashes[str(path.relative_to(source))] = digest.hexdigest()
    clock_extent = {}
    def observed_events():
        for event in bag_events(source / "bag"):
            if event[1] == "clock":
                clock_extent["recording_end_clock_ns"] = event[2]
            yield event
    with (output / "trajectory.jsonl").open("x") as stream:
        for record, arrival, receipt in clock_bound_events(observed_events()):
            record.update(arrival_ns=arrival, bag_receipt_ns=receipt)
            if timeline is None:
                timeline = PoseTimeline(record["session"], record["epoch"])
            apply_record(timeline, record)
            stream.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
            records += 1
            if records % 100 == 0:
                check_run(output)
    if timeline is None or len(timeline.frames) < 2:
        raise ValueError("insufficient recorded graph frames")
    return dict(passed=True, source=str(source), input_sha256=hashes,
                **clock_extent,
                trajectory_sha256=hashlib.sha256((output / "trajectory.jsonl").read_bytes()).hexdigest(),
                session=timeline.session, epoch=timeline.epoch, revisions=records, frames=len(timeline.frames),
                receipt_bound="next recorded simulation clock; maximum clock gap 0.2 s",
                native_terrain_validation="NOT_RUN", moving_ros_scan_deskew_validated=False,
                retrospective_final_pose_used_for_causal_output=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive") if Path("/archive").is_dir() else ARCHIVE_ROOT)
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = export(args.source, output)
    except Exception as exc:
        result = dict(passed=False, failure=str(exc), source=str(args.source))
    with (output / "shadow_trajectory.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
