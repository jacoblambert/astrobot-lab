#!/usr/bin/env python3
"""Compare a generated OccupancyGrid/costmap against the simulator GT map."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from nav_msgs.msg import OccupancyGrid

from slam_eval.compare_bev_maps import dilate, read_latest_messages


def resample_to_gt(gt: OccupancyGrid, pred: OccupancyGrid) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    gt_info = gt.info
    pred_info = pred.info
    gt_data = np.array(gt.data, dtype=np.int16).reshape((gt_info.height, gt_info.width))
    pred_data = np.array(pred.data, dtype=np.int16).reshape((pred_info.height, pred_info.width))

    rows = np.arange(gt_info.height, dtype=np.float64)
    cols = np.arange(gt_info.width, dtype=np.float64)
    xx = gt_info.origin.position.x + (cols + 0.5) * gt_info.resolution
    yy = gt_info.origin.position.y + (rows + 0.5) * gt_info.resolution
    x_grid, y_grid = np.meshgrid(xx, yy)
    pred_col = np.floor((x_grid - pred_info.origin.position.x) / pred_info.resolution).astype(np.int64)
    pred_row = np.floor((y_grid - pred_info.origin.position.y) / pred_info.resolution).astype(np.int64)
    inside = (
        (pred_row >= 0)
        & (pred_row < pred_info.height)
        & (pred_col >= 0)
        & (pred_col < pred_info.width)
    )

    sampled = np.full_like(gt_data, -1, dtype=np.int16)
    sampled[inside] = pred_data[pred_row[inside], pred_col[inside]]
    return gt_data, sampled, inside


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--gt-topic", default="/gt/map")
    parser.add_argument("--pred-topic", default="/astrobot_0/slam/bev_costmap")
    parser.add_argument("--gt-occupied-threshold", type=int, default=50)
    parser.add_argument("--pred-occupied-threshold", type=int, default=50)
    parser.add_argument("--dilation-radius", type=int, default=2)
    parser.add_argument("--feature-dilation-radius", type=int, default=2)
    parser.add_argument("--ignore-border-cells", type=int, default=2)
    args = parser.parse_args()

    messages = read_latest_messages(Path(args.bag), {args.gt_topic, args.pred_topic})
    gt = messages[args.gt_topic]
    pred = messages[args.pred_topic]
    if not isinstance(gt, OccupancyGrid) or not isinstance(pred, OccupancyGrid):
      raise RuntimeError("expected nav_msgs/msg/OccupancyGrid topics")

    gt_data, pred_data, inside = resample_to_gt(gt, pred)
    known = (gt_data >= 0) & inside & (pred_data >= 0)
    gt_occ = gt_data >= args.gt_occupied_threshold
    pred_occ = pred_data >= args.pred_occupied_threshold
    if args.ignore_border_cells > 0:
        b = args.ignore_border_cells
        gt_occ[:b, :] = False
        gt_occ[-b:, :] = False
        gt_occ[:, :b] = False
        gt_occ[:, -b:] = False

    gt_occ_eval = dilate(gt_occ, args.dilation_radius)
    pred_occ_eval = dilate(pred_occ, args.dilation_radius)
    eval_mask = known
    tp = int(np.count_nonzero(eval_mask & pred_occ & gt_occ_eval))
    fp = int(np.count_nonzero(eval_mask & pred_occ & ~gt_occ_eval))
    fn = int(np.count_nonzero(eval_mask & gt_occ & ~pred_occ_eval))
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    union = int(np.count_nonzero(eval_mask & (gt_occ_eval | pred_occ_eval)))
    intersection = int(np.count_nonzero(eval_mask & gt_occ_eval & pred_occ_eval))
    iou = intersection / union if union else 0.0

    gt_feature = dilate(gt_occ, args.feature_dilation_radius)
    pred_feature = dilate(pred_occ, args.feature_dilation_radius)
    feature_eval_mask = eval_mask & gt_feature
    feature_tp = int(np.count_nonzero(feature_eval_mask & pred_feature))
    feature_fn = int(np.count_nonzero(feature_eval_mask & ~pred_feature))
    feature_recall = feature_tp / (feature_tp + feature_fn) if feature_tp + feature_fn else 0.0
    pred_eval_mask = eval_mask & pred_feature
    pred_near_gt = int(np.count_nonzero(pred_eval_mask & gt_feature))
    pred_total = int(np.count_nonzero(pred_eval_mask))
    feature_precision = pred_near_gt / pred_total if pred_total else 0.0
    feature_f1 = (
        2.0 * feature_precision * feature_recall / (feature_precision + feature_recall)
        if feature_precision + feature_recall
        else 0.0
    )

    result = {
        "gt_topic": args.gt_topic,
        "pred_topic": args.pred_topic,
        "gt_resolution": gt.info.resolution,
        "pred_resolution": pred.info.resolution,
        "eval_cells": int(np.count_nonzero(eval_mask)),
        "gt_occupied_cells_eval": int(np.count_nonzero(eval_mask & gt_occ)),
        "pred_occupied_cells_eval": int(np.count_nonzero(eval_mask & pred_occ)),
        "pred_threshold": args.pred_occupied_threshold,
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "iou": iou,
        "feature_precision": feature_precision,
        "feature_recall": feature_recall,
        "feature_f1": feature_f1,
        "pred_cost_mean": float(np.mean(pred_data[eval_mask])) if np.any(eval_mask) else math.nan,
        "pred_cost_p95": float(np.percentile(pred_data[eval_mask], 95)) if np.any(eval_mask) else math.nan,
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
