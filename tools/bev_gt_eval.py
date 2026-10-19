#!/usr/bin/env python3
"""Honest BEV-vs-GT rock evaluation (live probe). Run INSIDE the dev container.

Subscribes /gt/map + the BEV costmap + both pose streams, fits the canonical
map<->world alignment from matched pose pairs (tools/map_gt_alignment.py — the
validated least-squares method, IMU->base aware), then scores:

- feature_recall: fraction of GT ROCKS (connected components of /gt/map) with at
  least one BEV obstacle cell (>= --obstacle-thresh, default 88 = the mapper's
  obstacle band floor) inside the rock's dilated footprint. Per-rock, because GT
  marks full footprints while LiDAR marks visible surfaces — cell-wise recall
  on footprints under-reads by construction (the bogus 0.6% measurement).
- cell_precision: fraction of BEV obstacle cells that fall within a dilated GT
  rock footprint (false-obstacle rate proxy).

Only rocks inside the BEV's KNOWN area count (can't recall what was never seen).
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_gt_alignment import fit_map_to_gt, world_to_map, imu_to_base, yaw_of  # noqa: E402


class BevGtEval(Node):
    def __init__(self, args):
        super().__init__("bev_gt_eval")
        self.args = args
        self.gt_rows = []     # (t,x,y,z,yaw) GT base
        self.slam_rows = []   # (t,x,y,z,yaw) SLAM (IMU frame)
        self.grids = {}
        self.create_subscription(PoseStamped, "/gt/base_link_pose",
                                 lambda m: self._pose(m, self.gt_rows), 10)
        self.create_subscription(PoseStamped, "/astrobot_0/slam/pose_corrected",
                                 lambda m: self._pose(m, self.slam_rows), 10)
        self.create_subscription(OccupancyGrid, "/gt/map", lambda m: self._grid("gt", m), 1)
        self.create_subscription(OccupancyGrid, args.bev_topic, lambda m: self._grid("bev", m), 1)

    @staticmethod
    def _pose(msg, rows):
        q = msg.pose.orientation
        rows.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                     msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                     yaw_of(q.x, q.y, q.z, q.w)))

    def _grid(self, key, msg):
        self.grids[key] = {
            "data": np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width),
            "res": msg.info.resolution,
            "ox": msg.info.origin.position.x,
            "oy": msg.info.origin.position.y,
        }


def label_components(mask):
    """4-connected component labeling (BFS); returns list of cell-index arrays."""
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    comps = []
    idx = np.argwhere(mask)
    for i0, j0 in idx:
        if seen[i0, j0]:
            continue
        q = deque([(i0, j0)])
        seen[i0, j0] = True
        cells = []
        while q:
            i, j = q.popleft()
            cells.append((i, j))
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if 0 <= ni < h and 0 <= nj < w and mask[ni, nj] and not seen[ni, nj]:
                    seen[ni, nj] = True
                    q.append((ni, nj))
        comps.append(np.asarray(cells))
    return comps


def score_grids(bev: dict, gt: dict, fit: dict, obstacle_thresh: int = 88,
                tolerance_m: float = 0.3, gt_label_res: float = 0.09,
                min_rock_cells: int = 4) -> dict:
    """Per-rock feature recall + cell precision of a BEV grid vs the GT rock map.

    bev/gt: {"data": HxW int8 array, "res", "ox", "oy"}; fit: map_gt_alignment fit.
    Shared by the live probe (this file) and the offline bag scorer (bev_bag_score.py).
    """
    B, bres, box, boy = bev["data"], bev["res"], bev["ox"], bev["oy"]
    G, gres, gox, goy = gt["data"], gt["res"], gt["ox"], gt["oy"]
    known = B >= 0
    obst = B >= obstacle_thresh

    stride = max(1, int(round(gt_label_res / gres)))
    Gs = G[::stride, ::stride]
    comps = label_components(Gs >= 99)
    tol_cells = max(1, int(round(tolerance_m / bres)))

    def gt_cell_to_bev(i, j):
        wx = gox + (j * stride + 0.5 * stride) * gres
        wy = goy + (i * stride + 0.5 * stride) * gres
        mx, my = world_to_map(wx, wy, fit)
        return int((my - boy) / bres), int((mx - box) / bres)

    recalled = total = 0
    for cells in comps:
        if len(cells) < min_rock_cells:
            continue
        in_known = hit = False
        for i, j in cells:
            bi, bj = gt_cell_to_bev(i, j)
            if not (0 <= bi < B.shape[0] and 0 <= bj < B.shape[1]):
                continue
            if known[bi, bj]:
                in_known = True
            s = obst[max(0, bi - tol_cells):bi + tol_cells + 1,
                     max(0, bj - tol_cells):bj + tol_cells + 1]
            if s.any():
                hit = True
                break
        if in_known or hit:
            total += 1
            recalled += int(hit)

    obs_cells = np.argwhere(obst)
    tp = 0
    gtol = max(1, int(round(tolerance_m / gres)))
    c, s = math.cos(fit["yaw"]), math.sin(fit["yaw"])
    for bi, bj in obs_cells:
        mx = box + (bj + 0.5) * bres
        my = boy + (bi + 0.5) * bres
        wx = c * mx - s * my + fit["tx"]
        wy = s * mx + c * my + fit["ty"]
        gi, gj = int((wy - goy) / gres), int((wx - gox) / gres)
        if 0 <= gi < G.shape[0] and 0 <= gj < G.shape[1]:
            sub = G[max(0, gi - gtol):gi + gtol + 1, max(0, gj - gtol):gj + gtol + 1]
            if (sub >= 99).any():
                tp += 1
    n_obs = len(obs_cells)
    return {
        "rocks_total": total, "rocks_recalled": recalled,
        "feature_recall": recalled / max(1, total),
        "cell_precision": tp / max(1, n_obs),
        "bev_obstacle_cells": int(n_obs), "known_cells": int(known.sum()),
    }


def evaluate(node, args):
    fit = fit_map_to_gt(node.gt_rows, node.slam_rows, window_s=args.fit_window)
    if fit is None:
        print("EVAL FAIL: no alignment fit (insufficient pose pairs)")
        return 1
    gt = node.grids.get("gt")
    bev = node.grids.get("bev")
    if gt is None or bev is None:
        print(f"EVAL FAIL: missing grids ({sorted(node.grids)})")
        return 1
    print(f"alignment: yaw={math.degrees(fit['yaw']):.2f}deg t=({fit['tx']:.2f},{fit['ty']:.2f}) "
          f"rms={fit['rms_m']:.3f}m n={fit['n_pairs']}")
    r = score_grids(bev, gt, fit, args.obstacle_thresh, args.tolerance_m,
                    args.gt_label_res, args.min_rock_cells)
    print(f"rocks (>= {args.min_rock_cells} cells, in explored area): {r['rocks_total']}")
    print(f"FEATURE RECALL: {r['rocks_recalled']}/{r['rocks_total']} = {r['feature_recall'] * 100:.1f}%")
    print(f"CELL PRECISION: {r['cell_precision'] * 100:.1f}% "
          f"(BEV obstacle cells within {args.tolerance_m}m of GT rock)")
    print(f"bev obstacle cells: {r['bev_obstacle_cells']}; known cells: {r['known_cells']}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bev-topic", default="/astrobot_0/slam/bev_costmap")
    ap.add_argument("--duration", type=float, default=60.0, help="collect this long, then score")
    ap.add_argument("--fit-window", type=float, default=45.0)
    ap.add_argument("--obstacle-thresh", type=int, default=88)
    ap.add_argument("--tolerance-m", type=float, default=0.3)
    ap.add_argument("--gt-label-res", type=float, default=0.09, help="GT subsample res for labeling")
    ap.add_argument("--min-rock-cells", type=int, default=4)
    args = ap.parse_args()
    rclpy.init()
    node = BevGtEval(args)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    rc = evaluate(node, args)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
