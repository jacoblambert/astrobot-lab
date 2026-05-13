#!/usr/bin/env python3
"""Measure whether point clouds contain obstacle-height structure above local ground."""

from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path
from typing import Any

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2

from slam_eval.compare_bev_maps import pointcloud_xyz


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def read_latest_messages(bag_path: Path, topics: set[str]) -> dict[str, Any]:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id="sqlite3"), ConverterOptions("", ""))
    type_by_topic = {info.name: info.type for info in reader.get_all_topics_and_types()}
    msg_type_by_topic = {topic: get_message(type_by_topic[topic]) for topic in topics if topic in type_by_topic}

    latest = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in msg_type_by_topic:
            latest[topic] = deserialize_message(data, msg_type_by_topic[topic])
    return latest


def local_height_stats(points: np.ndarray, cell_size: float, max_radius: float, min_points_per_cell: int) -> dict[str, Any]:
    bins: dict[tuple[int, int], list[float]] = collections.defaultdict(list)
    for x, y, z in points:
        if np.hypot(x, y) > max_radius:
            continue
        bins[(round(float(x) / cell_size), round(float(y) / cell_size))].append(float(z))

    heights: list[float] = []
    for zs in bins.values():
        if len(zs) < min_points_per_cell:
            continue
        ground = percentile(zs, 0.10)
        heights.extend(z - ground for z in zs)

    thresholds = [0.05, 0.10, 0.20, 0.40, 0.80]
    return {
        "cells": len(bins),
        "height_samples": len(heights),
        "height_p90": percentile(heights, 0.90),
        "height_p99": percentile(heights, 0.99),
        "height_max": max(heights) if heights else float("nan"),
        "count_above": {str(threshold): sum(1 for value in heights if value > threshold) for threshold in thresholds},
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Path to rosbag directory.")
    parser.add_argument("--topics", nargs="+", default=["/pointcloud", "/pointcloud/filtered", "/glim_rosnode/map"])
    parser.add_argument("--cell-size", type=float, default=0.5)
    parser.add_argument("--max-radius", type=float, default=15.0)
    parser.add_argument("--min-points-per-cell", type=int, default=4)
    args = parser.parse_args()

    messages = read_latest_messages(Path(args.bag), set(args.topics))
    result = {}
    for topic in args.topics:
        msg = messages.get(topic)
        if msg is None:
            result[topic] = {"present": False}
            continue
        if not isinstance(msg, PointCloud2):
            raise RuntimeError(f"{topic} is not PointCloud2")
        points = pointcloud_xyz(msg)
        result[topic] = {
            "present": True,
            "frame_id": msg.header.frame_id,
            "points": int(points.shape[0]),
            **local_height_stats(points, args.cell_size, args.max_radius, args.min_points_per_cell),
        }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
