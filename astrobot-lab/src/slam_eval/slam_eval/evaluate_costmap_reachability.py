#!/usr/bin/env python3
"""Evaluate whether a generated costmap preserves navigable free space."""

from __future__ import annotations

import argparse
import json
from collections import deque
from pathlib import Path

import numpy as np
from nav_msgs.msg import OccupancyGrid

from slam_eval.compare_bev_maps import read_latest_messages
from slam_eval.compare_occupancy_maps import resample_to_gt


def world_to_cell(msg: OccupancyGrid, x: float, y: float) -> tuple[int, int] | None:
    col = int(np.floor((x - msg.info.origin.position.x) / msg.info.resolution))
    row = int(np.floor((y - msg.info.origin.position.y) / msg.info.resolution))
    if row < 0 or row >= msg.info.height or col < 0 or col >= msg.info.width:
        return None
    return row, col


def nearest_free(free: np.ndarray, start: tuple[int, int], max_radius: int = 30) -> tuple[int, int] | None:
    rows, cols = free.shape
    sr, sc = start
    if 0 <= sr < rows and 0 <= sc < cols and free[sr, sc]:
        return start
    for radius in range(1, max_radius + 1):
        r0 = max(0, sr - radius)
        r1 = min(rows - 1, sr + radius)
        c0 = max(0, sc - radius)
        c1 = min(cols - 1, sc + radius)
        candidates = []
        for r in range(r0, r1 + 1):
            candidates.append((r, c0))
            candidates.append((r, c1))
        for c in range(c0 + 1, c1):
            candidates.append((r0, c))
            candidates.append((r1, c))
        for r, c in candidates:
            if free[r, c]:
                return r, c
    return None


def connected_component(free: np.ndarray, start: tuple[int, int]) -> np.ndarray:
    visited = np.zeros_like(free, dtype=bool)
    queue: deque[tuple[int, int]] = deque([start])
    visited[start] = True
    rows, cols = free.shape
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while queue:
        row, col = queue.popleft()
        for dr, dc in neighbors:
            rr = row + dr
            cc = col + dc
            if 0 <= rr < rows and 0 <= cc < cols and free[rr, cc] and not visited[rr, cc]:
                visited[rr, cc] = True
                queue.append((rr, cc))
    return visited


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--gt-topic", default="/gt/map")
    parser.add_argument("--pred-topic", default="/astrobot_0/slam/bev_costmap")
    parser.add_argument("--gt-occupied-threshold", type=int, default=50)
    parser.add_argument("--pred-blocked-threshold", type=int, default=50)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--goal-count", type=int, default=100)
    parser.add_argument("--seed", type=int, default=177)
    args = parser.parse_args()

    messages = read_latest_messages(Path(args.bag), {args.gt_topic, args.pred_topic})
    gt = messages[args.gt_topic]
    pred = messages[args.pred_topic]
    if not isinstance(gt, OccupancyGrid) or not isinstance(pred, OccupancyGrid):
        raise RuntimeError("expected nav_msgs/msg/OccupancyGrid topics")

    gt_data, pred_on_gt, inside = resample_to_gt(gt, pred)
    known = (gt_data >= 0) & inside & (pred_on_gt >= 0)
    gt_free = known & (gt_data < args.gt_occupied_threshold)
    gt_occ = known & (gt_data >= args.gt_occupied_threshold)
    pred_blocked_on_gt = known & (pred_on_gt >= args.pred_blocked_threshold)

    gt_free_count = int(np.count_nonzero(gt_free))
    gt_occ_count = int(np.count_nonzero(gt_occ))
    false_blocked = int(np.count_nonzero(gt_free & pred_blocked_on_gt))
    missed_obstacle = int(np.count_nonzero(gt_occ & ~pred_blocked_on_gt))

    pred_data = np.array(pred.data, dtype=np.int16).reshape((pred.info.height, pred.info.width))
    pred_free = (pred_data >= 0) & (pred_data < args.pred_blocked_threshold)
    pred_known = pred_data >= 0
    start_cell = world_to_cell(pred, args.start_x, args.start_y)
    if start_cell is None:
        raise RuntimeError("start is outside predicted costmap")
    start_free = nearest_free(pred_free, start_cell)
    component = np.zeros_like(pred_free, dtype=bool)
    if start_free is not None:
        component = connected_component(pred_free, start_free)

    rng = np.random.default_rng(args.seed)
    gt_free_rows, gt_free_cols = np.nonzero(gt_free)
    sample_count = min(args.goal_count, len(gt_free_rows))
    reachable = 0
    sampled = 0
    if sample_count > 0 and start_free is not None:
        chosen = rng.choice(len(gt_free_rows), size=sample_count, replace=False)
        for idx in chosen:
            row = int(gt_free_rows[idx])
            col = int(gt_free_cols[idx])
            x = gt.info.origin.position.x + (col + 0.5) * gt.info.resolution
            y = gt.info.origin.position.y + (row + 0.5) * gt.info.resolution
            pred_cell = world_to_cell(pred, x, y)
            if pred_cell is None:
                continue
            sampled += 1
            if component[pred_cell]:
                reachable += 1

    result = {
        "gt_topic": args.gt_topic,
        "pred_topic": args.pred_topic,
        "pred_blocked_threshold": args.pred_blocked_threshold,
        "eval_known_cells": int(np.count_nonzero(known)),
        "gt_free_cells": gt_free_count,
        "gt_occupied_cells": gt_occ_count,
        "pred_known_cells": int(np.count_nonzero(pred_known)),
        "pred_free_cells": int(np.count_nonzero(pred_free)),
        "pred_free_ratio_known": float(np.count_nonzero(pred_free) / np.count_nonzero(pred_known))
        if np.count_nonzero(pred_known)
        else 0.0,
        "false_blocked_gt_free_cells": false_blocked,
        "false_blocked_gt_free_rate": false_blocked / gt_free_count if gt_free_count else 0.0,
        "missed_gt_obstacle_cells": missed_obstacle,
        "missed_gt_obstacle_rate": missed_obstacle / gt_occ_count if gt_occ_count else 0.0,
        "start_cell_free": start_free is not None,
        "start_component_free_cells": int(np.count_nonzero(component)),
        "start_component_ratio_pred_free": float(np.count_nonzero(component) / np.count_nonzero(pred_free))
        if np.count_nonzero(pred_free)
        else 0.0,
        "sampled_gt_free_goals": sampled,
        "reachable_gt_free_goals": reachable,
        "reachable_gt_free_goal_rate": reachable / sampled if sampled else 0.0,
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
