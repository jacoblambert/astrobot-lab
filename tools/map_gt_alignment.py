#!/usr/bin/env python3
"""Canonical SLAM-map <-> GT/world (gt_map) planar alignment.

One shared implementation so every consumer (eval probes, fingerprints, the
console backend, sweep scoring) agrees on the transform. The math is the
validated least-squares yaw+translation fit from
slam_eval/compare_tum_trajectories.py (`align_yaw_translation`, the method
behind Phase 2's 0.877 BEV rock recall) — NOT a naive spawn-offset guess.

Crucial detail: GLIM's `slam/pose_corrected` is the IMU sensor pose, which is
offset AND 180deg-rotated from base_link (slam_map_to_odom_tf_node params:
x 0.264, y 0.017, yaw pi). SLAM poses are converted to base frame BEFORE
fitting against GT base-link poses; skipping this is how the bogus 0.6% recall
measurement happened.

Library use:  fit = fit_map_to_gt(gt_rows, slam_rows); wx, wy = map_to_world(x, y, fit)
CLI:          map_gt_alignment.py <run_dir with gt_base.tum + slam_corrected.tum>
                                  [--window 30] [--json out.json]
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sys

# IMU -> base_link extrinsic (must match slam_map_to_odom_tf_node / phase3_nav launch).
IMU_BASE_X = 0.264
IMU_BASE_Y = 0.017
IMU_BASE_YAW = math.pi


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def load_tum(path: str):
    """Rows of (t, x, y, z, yaw). Accepts full TUM (8 col) or t x y [z]."""
    rows = []
    if not os.path.exists(path):
        return rows
    for ln in open(path):
        a = ln.split()
        try:
            if len(a) >= 8:
                rows.append((float(a[0]), float(a[1]), float(a[2]), float(a[3]),
                             yaw_of(float(a[4]), float(a[5]), float(a[6]), float(a[7]))))
            elif len(a) >= 3:
                rows.append((float(a[0]), float(a[1]), float(a[2]), 0.0, 0.0))
        except ValueError:
            continue
    return rows


def imu_to_base(x: float, y: float, yaw: float):
    """GLIM IMU-frame pose (in map coords) -> base_link pose (in map coords)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * IMU_BASE_X - s * IMU_BASE_Y,
            y + s * IMU_BASE_X + c * IMU_BASE_Y,
            wrap(yaw + IMU_BASE_YAW))


def fit_map_to_gt(gt_rows, slam_rows, window_s: float = 30.0, max_dt: float = 0.1,
                  slam_is_imu_frame: bool = True):
    """Least-squares planar (yaw, tx, ty) mapping SLAM-map coords -> GT/world coords.

    gt_rows / slam_rows: load_tum() output. Uses pairs within window_s of the first
    match (early window = SLAM still accurate, so the fit captures the FRAME offset,
    not accumulated drift). Returns dict or None.
    """
    if len(gt_rows) < 5 or len(slam_rows) < 5:
        return None
    gtt = [r[0] for r in gt_rows]

    def nearest(t):
        i = bisect.bisect_left(gtt, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(gt_rows)]
        return min(cands, key=lambda j: abs(gtt[j] - t)) if cands else None

    pairs = []  # (ref=(x,y), est=(x,y))
    t0 = None
    for t, x, y, _z, yw in slam_rows:
        j = nearest(t)
        if j is None or abs(gtt[j] - t) > max_dt:
            continue
        if t0 is None:
            t0 = t
        if t - t0 > window_s:
            break
        if slam_is_imu_frame:
            x, y, yw = imu_to_base(x, y, yw)
        g = gt_rows[j]
        pairs.append(((g[1], g[2]), (x, y)))
    if len(pairs) < 5:
        return None

    # align_yaw_translation (ported verbatim from slam_eval/compare_tum_trajectories.py)
    n = len(pairs)
    ref_cx = sum(r[0] for r, _ in pairs) / n
    ref_cy = sum(r[1] for r, _ in pairs) / n
    est_cx = sum(e[0] for _, e in pairs) / n
    est_cy = sum(e[1] for _, e in pairs) / n
    sxx = sxy = 0.0
    for (rx, ry), (ex, ey) in pairs:
        ex -= est_cx; ey -= est_cy
        rx -= ref_cx; ry -= ref_cy
        sxx += ex * rx + ey * ry
        sxy += ex * ry - ey * rx
    yw = math.atan2(sxy, sxx)
    c, s = math.cos(yw), math.sin(yw)
    tx = ref_cx - (c * est_cx - s * est_cy)
    ty = ref_cy - (s * est_cx + c * est_cy)

    rms = math.sqrt(sum((c * e[0] - s * e[1] + tx - r[0]) ** 2 +
                        (s * e[0] + c * e[1] + ty - r[1]) ** 2
                        for r, e in pairs) / n)
    return {"yaw": yw, "tx": tx, "ty": ty, "n_pairs": n, "rms_m": rms,
            "window_s": window_s, "slam_is_imu_frame": slam_is_imu_frame}


def map_to_world(x: float, y: float, fit) -> tuple:
    c, s = math.cos(fit["yaw"]), math.sin(fit["yaw"])
    return (c * x - s * y + fit["tx"], s * x + c * y + fit["ty"])


def world_to_map(x: float, y: float, fit) -> tuple:
    c, s = math.cos(fit["yaw"]), math.sin(fit["yaw"])
    dx, dy = x - fit["tx"], y - fit["ty"]
    return (c * dx + s * dy, -s * dx + c * dy)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", help="dir containing gt_base.tum + slam_corrected.tum")
    ap.add_argument("--window", type=float, default=30.0)
    ap.add_argument("--json", default="")
    args = ap.parse_args()
    gt = load_tum(os.path.join(args.run_dir, "gt_base.tum"))
    sl = load_tum(os.path.join(args.run_dir, "slam_corrected.tum"))
    fit = fit_map_to_gt(gt, sl, window_s=args.window)
    if fit is None:
        print("ALIGN FAIL: insufficient matched pairs", file=sys.stderr)
        return 1
    print(f"map->world: yaw={math.degrees(fit['yaw']):.2f}deg tx={fit['tx']:.3f} "
          f"ty={fit['ty']:.3f} (n={fit['n_pairs']}, rms={fit['rms_m']:.3f} m)")
    if args.json:
        json.dump(fit, open(args.json, "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
