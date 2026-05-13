#!/usr/bin/env python3
"""Register local-frame PointCloud2 messages with pose messages before BEV comparison."""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2

from slam_eval.compare_bev_maps import (
    Alignment,
    compare_bev,
    compute_alignment,
    pointcloud_xyz,
    read_latest_messages,
    transform_points,
)


def stamp_ns(msg: Any, fallback: int) -> int:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
        return fallback
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_matrix(msg: PoseStamped) -> np.ndarray:
    p = msg.pose.position
    q = msg.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        x = y = z = 0.0
        w = 1.0
    else:
        x, y, z, w = x / n, y / n, z / n, w / n

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )
    mat[:3, 3] = (p.x, p.y, p.z)
    return mat


def transform_matrix(xyz: list[float], quat: list[float]) -> np.ndarray:
    msg = PoseStamped()
    msg.pose.position.x = xyz[0]
    msg.pose.position.y = xyz[1]
    msg.pose.position.z = xyz[2]
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    return pose_matrix(msg)


def read_registered_cloud(
    bag_path: Path,
    cloud_topic: str,
    pose_topic: str,
    sample_stride: int,
    max_messages: int,
    max_pose_dt: float,
    sensor_matrix: np.ndarray,
) -> tuple[np.ndarray, dict[str, Any]]:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id="sqlite3"), ConverterOptions("", ""))
    type_by_topic = {info.name: info.type for info in reader.get_all_topics_and_types()}
    missing = [topic for topic in (cloud_topic, pose_topic) if topic not in type_by_topic]
    if missing:
        raise RuntimeError(f"bag is missing required topic(s): {', '.join(missing)}")

    cloud_type = get_message(type_by_topic[cloud_topic])
    pose_type = get_message(type_by_topic[pose_topic])

    poses: list[tuple[int, np.ndarray]] = []
    clouds: list[tuple[int, PointCloud2]] = []
    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        if topic == pose_topic:
            msg = deserialize_message(data, pose_type)
            if not isinstance(msg, PoseStamped):
                raise RuntimeError(f"{pose_topic} is not geometry_msgs/msg/PoseStamped")
            poses.append((stamp_ns(msg, bag_time), pose_matrix(msg)))
        elif topic == cloud_topic:
            msg = deserialize_message(data, cloud_type)
            if not isinstance(msg, PointCloud2):
                raise RuntimeError(f"{cloud_topic} is not sensor_msgs/msg/PointCloud2")
            clouds.append((stamp_ns(msg, bag_time), msg))

    if not poses:
        raise RuntimeError(f"no poses found on {pose_topic}")
    if not clouds:
        raise RuntimeError(f"no clouds found on {cloud_topic}")

    pose_times = [item[0] for item in poses]
    registered = []
    seen = 0
    used = 0
    skipped_dt = 0
    max_pose_dt_ns = int(max_pose_dt * 1_000_000_000)
    for cloud_time, cloud in clouds:
        seen += 1
        if sample_stride > 1 and (seen - 1) % sample_stride != 0:
            continue
        idx = bisect.bisect_left(pose_times, cloud_time)
        candidates = []
        if idx < len(poses):
            candidates.append(poses[idx])
        if idx > 0:
            candidates.append(poses[idx - 1])
        pose_time, mat = min(candidates, key=lambda item: abs(item[0] - cloud_time))
        if abs(pose_time - cloud_time) > max_pose_dt_ns:
            skipped_dt += 1
            continue

        points = pointcloud_xyz(cloud)
        points = points @ sensor_matrix[:3, :3].T
        points += sensor_matrix[:3, 3]
        rotated = points @ mat[:3, :3].T
        rotated += mat[:3, 3]
        registered.append(rotated)
        used += 1
        if max_messages > 0 and used >= max_messages:
            break

    if not registered:
        raise RuntimeError("no clouds selected after pose matching")

    return (
        np.concatenate(registered, axis=0),
        {
            "cloud_topic": cloud_topic,
            "pose_topic": pose_topic,
            "cloud_messages_seen": seen,
            "cloud_messages_used": used,
            "cloud_messages_skipped_pose_dt": skipped_dt,
            "pose_messages": len(poses),
            "sample_stride": sample_stride,
            "max_messages": max_messages,
            "max_pose_dt": max_pose_dt,
        },
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Path to rosbag directory.")
    parser.add_argument("--gt-map-topic", default="/map")
    parser.add_argument("--cloud-topic", default="/glim_rosnode/points_corrected")
    parser.add_argument("--pose-topic", default="/glim_rosnode/pose_corrected")
    parser.add_argument("--reference-tum", help="GT TUM trajectory used to align GLIM map.")
    parser.add_argument("--estimate-tum", help="GLIM TUM trajectory used to align GLIM map.")
    parser.add_argument("--align", choices=["none", "first", "yaw"], default="first")
    parser.add_argument("--max-dt", type=float, default=0.1)
    parser.add_argument("--max-pose-dt", type=float, default=0.05)
    parser.add_argument("--sensor-xyz", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--sensor-quat", nargs=4, type=float, default=[0.0, 0.0, 0.0, 1.0])
    parser.add_argument("--sample-stride", type=int, default=4)
    parser.add_argument("--max-messages", type=int, default=0)
    parser.add_argument("--occupied-threshold", type=int, default=50)
    parser.add_argument("--height-threshold", type=float, default=0.20)
    parser.add_argument("--min-points-per-cell", type=int, default=4)
    parser.add_argument("--dilation-radius", type=int, default=2)
    parser.add_argument("--feature-dilation-radius", type=int, default=2)
    parser.add_argument("--ignore-border-cells", type=int, default=2)
    args = parser.parse_args()

    messages = read_latest_messages(Path(args.bag), {args.gt_map_topic})
    gt_map = messages[args.gt_map_topic]
    if not isinstance(gt_map, OccupancyGrid):
        raise RuntimeError(f"{args.gt_map_topic} is not nav_msgs/msg/OccupancyGrid")

    points, registration = read_registered_cloud(
        Path(args.bag),
        args.cloud_topic,
        args.pose_topic,
        max(1, args.sample_stride),
        max(0, args.max_messages),
        args.max_pose_dt,
        transform_matrix(args.sensor_xyz, args.sensor_quat),
    )

    alignment = Alignment(0.0, 0.0, 0.0, 0.0)
    if args.align != "none":
        if not args.reference_tum or not args.estimate_tum:
            raise RuntimeError("--reference-tum and --estimate-tum are required unless --align none")
        alignment = compute_alignment(Path(args.reference_tum), Path(args.estimate_tum), args.align, args.max_dt)

    result = compare_bev(
        gt_map,
        transform_points(points, alignment),
        args.occupied_threshold,
        args.height_threshold,
        args.min_points_per_cell,
        args.dilation_radius,
        args.feature_dilation_radius,
        args.ignore_border_cells,
    )
    result["alignment"] = {"mode": args.align, "yaw": alignment.yaw, "tx": alignment.tx, "ty": alignment.ty, "tz": alignment.tz}
    result["registration"] = registration
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
