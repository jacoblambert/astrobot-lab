#!/usr/bin/env python3
"""Compare an accumulated PointCloud2 stream against the simulator GT map in BEV."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
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


def read_accumulated_cloud(
    bag_path: Path,
    cloud_topic: str,
    sample_stride: int,
    max_messages: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id="sqlite3"), ConverterOptions("", ""))
    type_by_topic = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if cloud_topic not in type_by_topic:
        raise RuntimeError(f"bag is missing required topic: {cloud_topic}")
    msg_type = get_message(type_by_topic[cloud_topic])

    clouds = []
    seen = 0
    used = 0
    first_stamp = None
    last_stamp = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != cloud_topic:
            continue
        seen += 1
        if sample_stride > 1 and (seen - 1) % sample_stride != 0:
            continue
        msg = deserialize_message(data, msg_type)
        if not isinstance(msg, PointCloud2):
            raise RuntimeError(f"{cloud_topic} is not sensor_msgs/msg/PointCloud2")
        clouds.append(pointcloud_xyz(msg))
        used += 1
        first_stamp = timestamp if first_stamp is None else first_stamp
        last_stamp = timestamp
        if max_messages > 0 and used >= max_messages:
            break

    if not clouds:
        raise RuntimeError(f"no PointCloud2 messages selected from {cloud_topic}")

    return (
        np.concatenate(clouds, axis=0),
        {
            "cloud_topic": cloud_topic,
            "cloud_messages_seen": seen,
            "cloud_messages_used": used,
            "sample_stride": sample_stride,
            "max_messages": max_messages,
            "first_bag_time_ns": first_stamp,
            "last_bag_time_ns": last_stamp,
        },
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Path to rosbag directory.")
    parser.add_argument("--gt-map-topic", default="/map")
    parser.add_argument("--cloud-topic", default="/glim_rosnode/aligned_points_corrected")
    parser.add_argument("--reference-tum", help="GT TUM trajectory used to align GLIM map.")
    parser.add_argument("--estimate-tum", help="GLIM TUM trajectory used to align GLIM map.")
    parser.add_argument("--align", choices=["none", "first", "yaw"], default="first")
    parser.add_argument("--max-dt", type=float, default=0.1)
    parser.add_argument("--sample-stride", type=int, default=1)
    parser.add_argument("--max-messages", type=int, default=0)
    parser.add_argument("--occupied-threshold", type=int, default=50)
    parser.add_argument("--height-threshold", type=float, default=0.10)
    parser.add_argument("--min-points-per-cell", type=int, default=2)
    parser.add_argument("--dilation-radius", type=int, default=2)
    parser.add_argument("--feature-dilation-radius", type=int, default=2)
    parser.add_argument("--ignore-border-cells", type=int, default=2)
    args = parser.parse_args()

    messages = read_latest_messages(Path(args.bag), {args.gt_map_topic})
    gt_map = messages[args.gt_map_topic]
    if not isinstance(gt_map, OccupancyGrid):
        raise RuntimeError(f"{args.gt_map_topic} is not nav_msgs/msg/OccupancyGrid")

    points, accumulation = read_accumulated_cloud(
        Path(args.bag),
        args.cloud_topic,
        max(1, args.sample_stride),
        max(0, args.max_messages),
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
    result["accumulation"] = accumulation
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
