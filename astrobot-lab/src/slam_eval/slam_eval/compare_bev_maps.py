#!/usr/bin/env python3
"""Compare a GLIM 3D map against the simulator GT occupancy grid in BEV.

The GLIM map is projected to the GT grid after applying the same planar
alignment used for trajectory evaluation. A cell is treated as occupied in the
GLIM BEV map when enough points land in the cell and the local height span is
large enough to plausibly represent a rock/obstacle instead of flat terrain.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2

from slam_eval.compare_tum_trajectories import align_by_first_pose, align_yaw_translation, nearest_index, read_tum


@dataclass(frozen=True)
class Alignment:
    yaw: float
    tx: float
    ty: float
    tz: float


def read_latest_messages(bag_path: Path, topics: set[str]) -> dict[str, Any]:
    reader = SequentialReader()
    metadata_path = bag_path / "metadata.yaml"
    storage_id = "sqlite3"
    if metadata_path.exists():
        metadata = yaml.safe_load(metadata_path.read_text()) or {}
        storage_id = metadata.get("rosbag2_bagfile_information", {}).get("storage_identifier", storage_id)
    reader.open(StorageOptions(uri=str(bag_path), storage_id=storage_id), ConverterOptions("", ""))
    type_by_topic = {info.name: info.type for info in reader.get_all_topics_and_types()}
    msg_type_by_topic = {topic: get_message(type_by_topic[topic]) for topic in topics if topic in type_by_topic}
    missing = sorted(topics - set(msg_type_by_topic))
    if missing:
        raise RuntimeError(f"bag is missing required topic(s): {', '.join(missing)}")

    latest = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in msg_type_by_topic:
            latest[topic] = deserialize_message(data, msg_type_by_topic[topic])
    return latest


def pointcloud_xyz(msg: PointCloud2) -> np.ndarray:
    offsets = {field.name: field.offset for field in msg.fields}
    if not {"x", "y", "z"}.issubset(offsets):
        raise RuntimeError("PointCloud2 must contain x/y/z fields")

    points = np.empty((msg.width * msg.height, 3), dtype=np.float64)
    row_step = msg.row_step
    point_step = msg.point_step
    data = msg.data
    index = 0
    for row in range(msg.height):
        row_base = row * row_step
        for col in range(msg.width):
            base = row_base + col * point_step
            x = struct.unpack_from("<f", data, base + offsets["x"])[0]
            y = struct.unpack_from("<f", data, base + offsets["y"])[0]
            z = struct.unpack_from("<f", data, base + offsets["z"])[0]
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points[index] = (x, y, z)
                index += 1
    return points[:index]


def compute_alignment(reference_tum: Path, estimate_tum: Path, mode: str, max_dt: float) -> Alignment:
    reference = read_tum(str(reference_tum))
    estimate = read_tum(str(estimate_tum))
    ref_times = [point.t for point in reference]

    pairs = []
    for est in estimate:
        idx = nearest_index(ref_times, est.t)
        ref = reference[idx]
        if abs(ref.t - est.t) <= max_dt:
            pairs.append((ref, est))
    if not pairs:
        raise RuntimeError("no timestamp matches for map alignment")

    if mode == "first":
        yaw, tx, ty, tz = align_by_first_pose(pairs[0])
    elif mode == "yaw":
        yaw, tx, ty, tz = align_yaw_translation(pairs)
    else:
        yaw, tx, ty, tz = 0.0, 0.0, 0.0, 0.0
    return Alignment(yaw, tx, ty, tz)


def transform_points(points: np.ndarray, alignment: Alignment) -> np.ndarray:
    c = math.cos(alignment.yaw)
    s = math.sin(alignment.yaw)
    out = np.empty_like(points)
    out[:, 0] = c * points[:, 0] - s * points[:, 1] + alignment.tx
    out[:, 1] = s * points[:, 0] + c * points[:, 1] + alignment.ty
    out[:, 2] = points[:, 2] + alignment.tz
    return out


def dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask
    padded = np.pad(mask, radius, mode="constant", constant_values=False)
    out = np.zeros_like(mask, dtype=bool)
    for dy in range(2 * radius + 1):
        for dx in range(2 * radius + 1):
            out |= padded[dy : dy + mask.shape[0], dx : dx + mask.shape[1]]
    return out


def compare_bev(
    gt_map: OccupancyGrid,
    points: np.ndarray,
    occupied_threshold: int,
    height_threshold: float,
    min_points_per_cell: int,
    dilation_radius: int,
    feature_dilation_radius: int,
    ignore_border_cells: int,
) -> dict[str, Any]:
    width = gt_map.info.width
    height = gt_map.info.height
    resolution = gt_map.info.resolution
    origin_x = gt_map.info.origin.position.x
    origin_y = gt_map.info.origin.position.y

    gt_data = np.array(gt_map.data, dtype=np.int16).reshape((height, width))
    known = gt_data >= 0
    gt_occ = gt_data >= occupied_threshold
    if ignore_border_cells > 0:
        gt_occ[:ignore_border_cells, :] = False
        gt_occ[-ignore_border_cells:, :] = False
        gt_occ[:, :ignore_border_cells] = False
        gt_occ[:, -ignore_border_cells:] = False

    col = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int64)
    row = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int64)
    inside = (row >= 0) & (row < height) & (col >= 0) & (col < width)
    row = row[inside]
    col = col[inside]
    z = points[inside, 2]

    counts = np.zeros((height, width), dtype=np.int32)
    min_z = np.full((height, width), np.inf, dtype=np.float64)
    max_z = np.full((height, width), -np.inf, dtype=np.float64)
    np.add.at(counts, (row, col), 1)
    np.minimum.at(min_z, (row, col), z)
    np.maximum.at(max_z, (row, col), z)

    observed = counts >= min_points_per_cell
    height_span = np.where(observed, max_z - min_z, 0.0)
    glim_occ = observed & (height_span >= height_threshold)

    # Allow a small spatial tolerance because GLIM map alignment and GT rock
    # rasterization are both approximate.
    gt_occ_eval = dilate(gt_occ, dilation_radius)
    glim_occ_eval = dilate(glim_occ, dilation_radius)
    observed_eval = dilate(observed, dilation_radius)
    eval_mask = known & observed_eval

    tp = int(np.count_nonzero(eval_mask & glim_occ & gt_occ_eval))
    fp = int(np.count_nonzero(eval_mask & glim_occ & ~gt_occ_eval))
    fn = int(np.count_nonzero(eval_mask & gt_occ & ~glim_occ_eval))
    intersection = int(np.count_nonzero(eval_mask & glim_occ_eval & gt_occ_eval))
    union = int(np.count_nonzero(eval_mask & (glim_occ_eval | gt_occ_eval)))

    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    iou = intersection / union if union else 0.0

    # Rock-focused metrics. The broad BEV IoU can look acceptable when the
    # cloud matches terrain/background but misses small obstacle structure. This
    # focus mask scores only GT occupied features and nearby predictions.
    gt_feature = dilate(gt_occ, feature_dilation_radius)
    glim_feature = dilate(glim_occ, feature_dilation_radius)
    feature_eval_mask = known & observed_eval & gt_feature
    feature_tp = int(np.count_nonzero(feature_eval_mask & glim_feature))
    feature_fn = int(np.count_nonzero(feature_eval_mask & ~glim_feature))
    feature_recall = feature_tp / (feature_tp + feature_fn) if feature_tp + feature_fn else 0.0

    pred_eval_mask = known & observed_eval & glim_feature
    pred_near_gt = int(np.count_nonzero(pred_eval_mask & gt_feature))
    pred_total = int(np.count_nonzero(pred_eval_mask))
    feature_precision = pred_near_gt / pred_total if pred_total else 0.0
    feature_f1 = (
        2.0 * feature_precision * feature_recall / (feature_precision + feature_recall)
        if feature_precision + feature_recall
        else 0.0
    )

    return {
        "gt_width": width,
        "gt_height": height,
        "resolution": resolution,
        "points_total": int(points.shape[0]),
        "points_inside_gt": int(z.shape[0]),
        "observed_cells": int(np.count_nonzero(observed)),
        "eval_cells": int(np.count_nonzero(eval_mask)),
        "gt_occupied_cells": int(np.count_nonzero(gt_occ)),
        "gt_occupied_cells_eval": int(np.count_nonzero(eval_mask & gt_occ)),
        "glim_occupied_cells": int(np.count_nonzero(glim_occ)),
        "glim_occupied_cells_eval": int(np.count_nonzero(eval_mask & glim_occ)),
        "height_threshold": height_threshold,
        "min_points_per_cell": min_points_per_cell,
        "dilation_radius": dilation_radius,
        "feature_dilation_radius": feature_dilation_radius,
        "ignore_border_cells": ignore_border_cells,
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "iou": iou,
        "feature_eval_cells": int(np.count_nonzero(feature_eval_mask)),
        "feature_tp": feature_tp,
        "feature_fn": feature_fn,
        "feature_precision": feature_precision,
        "feature_recall": feature_recall,
        "feature_f1": feature_f1,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Path to rosbag directory.")
    parser.add_argument("--gt-map-topic", default="/map")
    parser.add_argument("--glim-map-topic", default="/glim_rosnode/map")
    parser.add_argument("--reference-tum", help="GT TUM trajectory used to align GLIM map.")
    parser.add_argument("--estimate-tum", help="GLIM TUM trajectory used to align GLIM map.")
    parser.add_argument("--align", choices=["none", "first", "yaw"], default="first")
    parser.add_argument("--max-dt", type=float, default=0.1)
    parser.add_argument("--occupied-threshold", type=int, default=50)
    parser.add_argument("--height-threshold", type=float, default=0.25)
    parser.add_argument("--min-points-per-cell", type=int, default=3)
    parser.add_argument("--dilation-radius", type=int, default=1)
    parser.add_argument("--feature-dilation-radius", type=int, default=2)
    parser.add_argument("--ignore-border-cells", type=int, default=2)
    args = parser.parse_args()

    messages = read_latest_messages(Path(args.bag), {args.gt_map_topic, args.glim_map_topic})
    gt_map = messages[args.gt_map_topic]
    glim_map = messages[args.glim_map_topic]
    if not isinstance(gt_map, OccupancyGrid) or not isinstance(glim_map, PointCloud2):
        raise RuntimeError("unexpected message types for map comparison")

    alignment = Alignment(0.0, 0.0, 0.0, 0.0)
    if args.align != "none":
        if not args.reference_tum or not args.estimate_tum:
            raise RuntimeError("--reference-tum and --estimate-tum are required unless --align none")
        alignment = compute_alignment(Path(args.reference_tum), Path(args.estimate_tum), args.align, args.max_dt)

    points = transform_points(pointcloud_xyz(glim_map), alignment)
    result = compare_bev(
        gt_map,
        points,
        args.occupied_threshold,
        args.height_threshold,
        args.min_points_per_cell,
        args.dilation_radius,
        args.feature_dilation_radius,
        args.ignore_border_cells,
    )
    result["alignment"] = {"mode": args.align, "yaw": alignment.yaw, "tx": alignment.tx, "ty": alignment.ty, "tz": alignment.tz}
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
