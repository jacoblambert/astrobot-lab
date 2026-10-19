#!/usr/bin/env python3
"""Score a recorded/replayed bag's BEV stream against GT. Run INSIDE the container.

Reads a bag containing /astrobot_0/slam/bev_costmap (sequence), /gt/map,
/gt/base_link_pose and /astrobot_0/slam/pose_corrected; fits the canonical
alignment (tools/map_gt_alignment.py), then reports:
  - final-grid per-rock FEATURE RECALL + CELL PRECISION (bev_gt_eval.score_grids)
  - FLICKER: per-cell obstacle-state toggles across the stream (total toggles/min)
    and 'transient' cells (became obstacle then vanished within --transient-frames)
Emits a single CSV row (also to --csv) so the param sweep can build a table.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
import rclpy.serialization as ser
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_gt_alignment import fit_map_to_gt, yaw_of  # noqa: E402
from bev_gt_eval import score_grids  # noqa: E402


def grid_dict(msg):
    return {"data": np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width),
            "res": msg.info.resolution, "ox": msg.info.origin.position.x,
            "oy": msg.info.origin.position.y}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir")
    ap.add_argument("--label", default="")
    ap.add_argument("--obstacle-thresh", type=int, default=88)
    ap.add_argument("--tolerance-m", type=float, default=0.3)
    ap.add_argument("--transient-frames", type=int, default=4)
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag_dir, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    gt_rows, slam_rows = [], []
    gt_grid = None
    last_bev = None
    # flicker accumulators (lazy init on first BEV)
    prev_obst = None
    became_on_frame = None
    toggles = None
    transient = None
    n_frames = 0
    t_first = t_last = None

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic == "/astrobot_0/slam/bev_costmap":
            msg = ser.deserialize_message(raw, OccupancyGrid)
            last_bev = grid_dict(msg)
            ob = last_bev["data"] >= args.obstacle_thresh
            if prev_obst is None:
                toggles = np.zeros(ob.shape, dtype=np.int32)
                transient = np.zeros(ob.shape, dtype=np.int32)
                became_on_frame = np.full(ob.shape, -1, dtype=np.int32)
            else:
                changed = ob != prev_obst
                toggles += changed
                turned_on = changed & ob
                turned_off = changed & ~ob
                became_on_frame[turned_on] = n_frames
                # cells turning off shortly after turning on = transient blips
                recent = (became_on_frame >= 0) & (n_frames - became_on_frame <= args.transient_frames)
                transient += (turned_off & recent)
            prev_obst = ob
            n_frames += 1
            t_last = t_ns
            if t_first is None:
                t_first = t_ns
        elif topic == "/gt/map":
            gt_grid = grid_dict(ser.deserialize_message(raw, OccupancyGrid))
        elif topic in ("/gt/base_link_pose", "/astrobot_0/slam/pose_corrected"):
            m = ser.deserialize_message(raw, PoseStamped)
            q = m.pose.orientation
            row = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                   m.pose.position.x, m.pose.position.y, m.pose.position.z,
                   yaw_of(q.x, q.y, q.z, q.w))
            (gt_rows if topic == "/gt/base_link_pose" else slam_rows).append(row)

    if last_bev is None or gt_grid is None:
        print("SCORE FAIL: bag missing bev or gt grids")
        return 1
    fit = fit_map_to_gt(gt_rows, slam_rows, window_s=45.0)
    if fit is None:
        print("SCORE FAIL: alignment fit failed")
        return 1

    r = score_grids(last_bev, gt_grid, fit, args.obstacle_thresh, args.tolerance_m)
    dur_min = max(1e-6, (t_last - t_first) / 1e9 / 60.0) if t_first is not None else 1e-6
    total_toggles = int(toggles.sum()) if toggles is not None else 0
    transient_total = int(transient.sum()) if transient is not None else 0
    row = {
        "label": args.label or os.path.basename(args.bag_dir.rstrip("/")),
        "feature_recall": round(r["feature_recall"], 4),
        "cell_precision": round(r["cell_precision"], 4),
        "rocks_total": r["rocks_total"],
        "bev_obstacle_cells": r["bev_obstacle_cells"],
        "toggles_per_min": round(total_toggles / dur_min, 1),
        "transient_blips": transient_total,
        "frames": n_frames,
        "align_yaw_deg": round(math.degrees(fit["yaw"]), 2),
        "align_rms_m": round(fit["rms_m"], 3),
    }
    hdr = ",".join(row.keys())
    line = ",".join(str(v) for v in row.values())
    print(hdr)
    print(line)
    if args.csv:
        new = not os.path.exists(args.csv)
        with open(args.csv, "a") as f:
            if new:
                f.write(hdr + "\n")
            f.write(line + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
