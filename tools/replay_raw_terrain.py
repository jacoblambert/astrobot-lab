#!/usr/bin/env python3
"""Streaming raw XYZ + timestamped GT replay. No ROS graph or GT terrain labels.

Requires ROS bag Python bindings, NumPy, SciPy and the layered_terrain library.
Stationary-only until simulator sensor timing is validated; moving bags fail
closed. This measures retention/coverage/runtime, NOT obstacle accuracy.
"""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import time

import numpy as np

from archive_guard import allocate_run, check_run
from fixture_alignment import ReferenceTrajectory, pose_matrix
from layered_terrain import TerrainMapper

RAW = "/astrobot_0/lidar_0/pointcloud/raw"
GT = "/gt/base_link_pose"


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def xyz_array(msg):
    """Honor PointCloud2 row padding, field offsets, endian and float width."""
    fields = {f.name: f for f in msg.fields}
    formats, offsets = [], []
    for name in ("x", "y", "z"):
        f = fields.get(name)
        if f is None or f.count != 1 or f.datatype not in (7, 8):
            raise ValueError("raw cloud needs scalar floating XYZ")
        size = 4 if f.datatype == 7 else 8
        if f.offset < 0 or f.offset + size > msg.point_step:
            raise ValueError("XYZ field outside point stride")
        formats.append((">" if msg.is_bigendian else "<") + f"f{size}")
        offsets.append(f.offset)
    if msg.row_step < msg.width * msg.point_step or len(msg.data) != msg.height * msg.row_step:
        raise ValueError("invalid cloud buffer/row stride")
    dtype = np.dtype(dict(names=["x", "y", "z"], formats=formats, offsets=offsets, itemsize=msg.point_step))
    rows = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    return np.column_stack([rows[name].ravel() for name in ("x", "y", "z")]).astype(np.float64)


def read_messages(bag, topics):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    reader.set_filter(StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, types[topic])


def reference_data(bag):
    rows, extrinsics = [], []
    for topic, msg in read_messages(bag, {GT, "/tf", "/tf_static"}):
        if topic == GT:
            if msg.header.frame_id != "gt_map":
                raise ValueError("unexpected GT frame")
            p, q = msg.pose.position, msg.pose.orientation
            rows.append((stamp_ns(msg)*1e-9, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        else:
            for tf in msg.transforms:
                if (tf.header.frame_id, tf.child_frame_id) == ("base_link", "vlp16"):
                    p, q = tf.transform.translation, tf.transform.rotation
                    extrinsics.append(pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
    trajectory = ReferenceTrajectory(rows)
    if not extrinsics:
        raise ValueError("missing recorded base_link -> vlp16 extrinsic")
    first = extrinsics[0]
    translation = max(np.linalg.norm(t[:3, 3]-first[:3, 3]) for t in extrinsics)
    rotation = max(np.linalg.norm(t[:3, :3]-first[:3, :3]) for t in extrinsics)
    if translation > .001 or rotation > .001:
        raise ValueError("base/LiDAR extrinsic is not constant within 1mm/0.001 matrix norm")
    start = pose_matrix(*trajectory.rows[0][1:])
    displacement = max(np.linalg.norm(np.asarray(r[1])-start[:3, 3]) for r in trajectory.rows)
    angle_matrix = max(np.linalg.norm(pose_matrix(*r[1:])[:3, :3]-start[:3, :3]) for r in trajectory.rows)
    if displacement > .005 or angle_matrix > .002:
        raise ValueError("moving fixture: validate scan acquisition timing/deskew before GT replay")
    return trajectory, first, dict(extrinsic_samples=len(extrinsics), base_from_lidar=first.tolist(),
        max_extrinsic_translation_m=translation, max_extrinsic_rotation_matrix_norm=rotation,
        max_base_displacement_m=displacement, max_base_rotation_matrix_norm=angle_matrix,
        timing_status="stationary_only_no_per_point_deskew")


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for data in iter(lambda: stream.read(1024*1024), b""):
            h.update(data)
    return h.hexdigest()


def run(args, output):
    trajectory, base_lidar, reference = reference_data(args.bag)
    anchor = np.array(args.anchor, dtype=float)
    if not np.isfinite(anchor).all():
        raise ValueError("nonfinite anchor")
    counters = dict(scans=0, boundary_scans=0, raw_points=0, invalid_points=0,
                    within_5m=0, within_10m=0, within_20m=0, outside_grid=0)
    variants = {"measured_accumulated": False, "inferred_accumulated": True}
    maps = {name: TerrainMapper(args.library, interpolate=interpolate) for name, interpolate in variants.items()}
    latencies = {name: [] for name in maps}
    frames, last = [], -1
    source_dir = Path(__file__).resolve().parents[1]
    provenance = {str(p.relative_to(source_dir)): digest(p) for p in [Path(__file__),
        source_dir/"tools/layered_terrain.py", source_dir/"tools/fixture_alignment.py",
        source_dir/"astrobot-lab/src/slam_mapping/include/slam_mapping/layered_terrain.hpp",
        source_dir/"astrobot-lab/src/slam_mapping/src/layered_terrain.cpp",
        source_dir/"astrobot-lab/src/slam_mapping/src/layered_terrain_c.cpp"]}
    manifest = dict(schema=1, kind="stationary_raw_gt_terrain_diagnostic", bag=str(args.bag.resolve()),
                    inputs={p.name: digest(p) for p in sorted(args.bag.glob("*.mcap"))},
                    library_sha256=digest(args.library), source_sha256=provenance, anchor=anchor.tolist(),
                    width=200, height=200, resolution=.1, origin=[-10,-10], lifetime=30,
                    reference=reference, variants=variants,
                    gt_terrain_used=False, acceptance_scoring=False, point_filter="finite_only_no_decimation")
    with (output/"manifest.json").open("x") as stream:
        json.dump(manifest, stream, indent=2)
    try:
        for _, msg in read_messages(args.bag, {RAW}):
            ts = stamp_ns(msg)
            if ts <= last:
                raise ValueError("nonmonotonic raw scan stamp")
            last = ts
            if msg.header.frame_id != "vlp16":
                raise ValueError("raw cloud not in verified LiDAR frame")
            seconds = ts*1e-9
            if not trajectory.times[0] <= seconds <= trajectory.times[-1]:
                counters["boundary_scans"] += 1
                continue
            points = xyz_array(msg)
            counters["raw_points"] += len(points)
            valid = np.isfinite(points).all(axis=1)
            counters["invalid_points"] += int((~valid).sum())
            points = points[valid]
            ranges = np.linalg.norm(points, axis=1)
            for distance in (5,10,20):
                counters[f"within_{distance}m"] += int((ranges <= distance).sum())
            world_lidar = trajectory.at(seconds) @ base_lidar
            world_lidar[:3, 3] -= anchor
            points = points @ world_lidar[:3, :3].T + world_lidar[:3, 3]
            counters["outside_grid"] += int((np.any((points[:, :2]<-10) | (points[:, :2]>=10), axis=1)).sum())
            yaw = np.arctan2(world_lidar[1, 0], world_lidar[0, 0])
            frame = dict(stamp_ns=ts)
            for name, mapper in maps.items():
                started = time.perf_counter()
                mapper.integrate(points, seconds, world_lidar[:3, 3], yaw)
                grid = mapper.project(seconds)
                elapsed = time.perf_counter()-started
                latencies[name].append(elapsed)
                frame[name] = dict(sha256=hashlib.sha256(grid[:, :, :2].tobytes()).hexdigest(), seconds=elapsed,
                                  reason_counts=np.bincount(grid[:, :, 1].astype(int).ravel(), minlength=8).tolist())
            frames.append(frame)
            counters["scans"] += 1
            if counters["scans"] % 25 == 0:
                check_run(output)
        if not counters["scans"]:
            raise ValueError("no usable raw scans")
        for name, mapper in maps.items():
            np.savez_compressed(output/f"{name}.npz", layers=mapper.project(frames[-1]["stamp_ns"]*1e-9))
        summary = dict(counters=counters, frames=frames,
            latency={name: dict(p50_ms=float(np.percentile(v,50)*1000), p95_ms=float(np.percentile(v,95)*1000),
                               p99_ms=float(np.percentile(v,99)*1000)) for name,v in latencies.items()},
            process_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
            note="No validated geometry oracle: cell counts are coverage diagnostics, not accuracy or navigation safety.")
        with (output/"summary.json").open("x") as stream:
            json.dump(summary, stream, indent=2)
        print(json.dumps({k:v for k,v in summary.items() if k != "frames"}, indent=2), flush=True)
    finally:
        for mapper in maps.values():
            mapper.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--library", required=True, type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--anchor", type=float, nargs=3, default=[20.,18.,0.])
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        run(args, output)
    except Exception as exc:
        with (output/"failure.json").open("x") as stream:
            json.dump(dict(type=type(exc).__name__, error=str(exc)), stream, indent=2)
        raise


if __name__ == "__main__":
    main()
