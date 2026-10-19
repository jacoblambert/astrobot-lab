#!/usr/bin/env python3
"""Extract a structured divergence FINGERPRINT from one recorded gate-run dir.

Run dir (from phase4_mission_gate_run.sh --record) contains:
  gt_base.tum, slam_corrected.tum, map_odom.csv, glim.log, eval.json

Pure host-side (no ROS). We anchor SLAM (slam/pose_corrected, in the IMU sensor
frame) directly against GT base-link, fitting out the initial translation + yaw
(mirrors tools/slam_anchored_error.py). The IMU->base lever arm (~0.4 m) is left
unmodeled on purpose: it's well below the 2 m divergence threshold, so it's noise
for fingerprinting (we only care about >2 m events: jumps, runaway, yaw-slip).

Emits fingerprint.json + a one-line summary. The fields are chosen so a batch can
cluster diverged runs and answer "one failure mode or several?".
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import re
import sys

JUMP_THRESH = 2.0      # m: diverged
RUNAWAY_M = 50.0       # m: runaway (vs bounded drift)
TURN_RATE = 0.2        # rad/s: turning vs straight at onset
YAWSLIP_DEG = 30.0     # deg of relative yaw error -> yaw-slip
MAX_DT = 0.1           # s: GT/SLAM time match tolerance
MAX_SPEED = 2.0        # m/s: GT speed-gate (drop glitch pairings)
FIT_YAW_WINDOW = 30.0  # s of early data to fit the anchor yaw


def load_tum(p):
    rows = []
    if not os.path.exists(p):
        return rows
    for ln in open(p):
        a = ln.split()
        if len(a) >= 8:
            try:
                rows.append(tuple(float(v) for v in a[:8]))  # t x y z qx qy qz qw
            except ValueError:
                pass
        elif len(a) >= 3:
            try:
                rows.append((float(a[0]), float(a[1]), float(a[2]), 0, 0, 0, 0, 1))
            except ValueError:
                pass
    return rows


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def fingerprint(run_dir):
    gt = load_tum(os.path.join(run_dir, "gt_base.tum"))
    sl = load_tum(os.path.join(run_dir, "slam_corrected.tum"))
    fp = {
        "run": os.path.basename(run_dir.rstrip("/")),
        "diverged": None, "onset_t": None, "onset_gt_xy": None,
        "max_err_m": None, "max_rate_ms": None, "type": None,
        "rel_yaw_err_deg_at_onset": None,
        "motion_at_onset": None,
        "map_odom_max_m": None, "map_odom_cross_t": None,
        "imu_warn_count": None,
        "outcome": None, "note": None,
    }
    m = re.search(r"seed(\d+)", fp["run"])
    fp["seed"] = int(m.group(1)) if m else None
    m = re.match(r"(sample_return|explore_radius)", fp["run"])
    fp["mode"] = m.group(1) if m else None

    # --- map->odom watchdog (frame-clean, independent of GT) ---
    mo = os.path.join(run_dir, "map_odom.csv")
    if os.path.exists(mo):
        mx = 0.0
        cross = None
        for ln in open(mo):
            a = ln.split(",")
            if len(a) < 5 or a[0] == "sim_t":
                continue
            try:
                t = float(a[0]); mag = float(a[4])
            except ValueError:
                continue
            if mag > mx:
                mx = mag
            if cross is None and mag > JUMP_THRESH:
                cross = t
        fp["map_odom_max_m"] = round(mx, 3)
        fp["map_odom_cross_t"] = cross

    # --- GLIM console IMU-warning count (proxy for "IMU not constraining") ---
    gl = os.path.join(run_dir, "glim.log")
    if os.path.exists(gl):
        txt = open(gl, errors="ignore").read()
        fp["imu_warn_count"] = len(re.findall(r"IMU prediction is not good|prediction is not good", txt))

    # --- outcome from the eval verdict ---
    ej = os.path.join(run_dir, "eval.json")
    if os.path.exists(ej):
        try:
            d = json.load(open(ej))
            v = d.get("verdict") or {}
            mm = v.get("metrics") or {}
            fp["outcome"] = {
                "pass": v.get("pass"),
                "return_reason": mm.get("return_reason"),
                "best_sample": mm.get("best_sample"),
            }
        except Exception:
            pass

    if len(gt) < 5 or len(sl) < 5:
        fp["note"] = f"insufficient TUM data (gt={len(gt)}, slam={len(sl)})"
        # Fall back to the watchdog signal for the diverged flag if poses are missing.
        if fp["map_odom_max_m"] is not None:
            fp["diverged"] = fp["map_odom_max_m"] > JUMP_THRESH
        return fp

    gtt = [r[0] for r in gt]

    def nearest(t):
        i = bisect.bisect_left(gtt, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(gt)]
        return min(cands, key=lambda j: abs(gtt[j] - t)) if cands else None

    # matched pairs: (t, sx, sy, syaw, gx, gy, gyaw)
    pairs = []
    for r in sl:
        t, sx, sy = r[0], r[1], r[2]
        syaw = yaw_of(r[4], r[5], r[6], r[7])
        j = nearest(t)
        if j is not None and abs(gtt[j] - t) <= MAX_DT:
            g = gt[j]
            pairs.append((t, sx, sy, syaw, g[1], g[2], yaw_of(g[4], g[5], g[6], g[7])))
    if len(pairs) < 5:
        fp["note"] = f"too few matched pairs ({len(pairs)})"
        if fp["map_odom_max_m"] is not None:
            fp["diverged"] = fp["map_odom_max_m"] > JUMP_THRESH
        return fp

    # GT speed-gate (truth can't teleport) — drop glitch pairings
    clean = [pairs[0]]
    for k in range(1, len(pairs)):
        dt = pairs[k][0] - clean[-1][0]
        if dt <= 0:
            continue
        v = math.hypot(pairs[k][4] - clean[-1][4], pairs[k][5] - clean[-1][5]) / dt
        if v <= MAX_SPEED:
            clean.append(pairs[k])
    pairs = clean

    t0, sx0, sy0, syaw0, gx0, gy0, gyaw0 = pairs[0]
    # anchor yaw from early window (least-squares rotation aligning SLAM->GT deltas)
    win = [p for p in pairs if p[0] - t0 <= FIT_YAW_WINDOW]
    sxx = sxy = 0.0
    for t, sx, sy, sw, gx, gy, gw in win:
        a, b = sx - sx0, sy - sy0
        c, d = gx - gx0, gy - gy0
        sxx += a * c + b * d
        sxy += a * d - b * c
    yaw = math.atan2(sxy, sxx) if len(win) >= 5 else 0.0
    cy, sny = math.cos(yaw), math.sin(yaw)

    def world_of(sx, sy):
        a, b = sx - sx0, sy - sy0
        return (cy * a - sny * b + gx0, sny * a + cy * b + gy0)

    yaw_off0 = wrap(syaw0 - gyaw0)  # initial yaw offset to subtract
    prof = []  # (t_rel, pos_err, rel_yaw_err_rad, gx, gy, gyaw, t_abs)
    for t, sx, sy, sw, gx, gy, gw in pairs:
        wx, wy = world_of(sx, sy)
        pe = math.hypot(wx - gx, wy - gy)
        rye = wrap((sw - gw) - yaw_off0)
        prof.append((t - t0, pe, rye, gx, gy, gw, t))

    n = len(prof)
    mx = max(e for _, e, *_ in prof)
    first_cross_i = next((i for i, p in enumerate(prof) if p[1] > JUMP_THRESH), None)
    maxrate = 0.0
    for i in range(1, n):
        dt = prof[i][0] - prof[i - 1][0]
        if dt > 0:
            maxrate = max(maxrate, abs(prof[i][1] - prof[i - 1][1]) / dt)

    fp["max_err_m"] = round(mx, 3)
    fp["max_rate_ms"] = round(maxrate, 3)
    fp["diverged"] = mx > JUMP_THRESH

    if first_cross_i is not None:
        on = prof[first_cross_i]
        fp["onset_t"] = round(on[0], 2)
        fp["onset_gt_xy"] = [round(on[3], 2), round(on[4], 2)]
        fp["rel_yaw_err_deg_at_onset"] = round(math.degrees(on[2]), 1)
        # motion in a +/-1s window around onset (from GT)
        t_abs = on[6]
        wpts = [p for p in prof if abs(p[6] - t_abs) <= 1.0]
        if len(wpts) >= 2:
            dt = wpts[-1][6] - wpts[0][6]
            if dt > 0:
                spd = math.hypot(wpts[-1][3] - wpts[0][3], wpts[-1][4] - wpts[0][4]) / dt
                yr = wrap(wpts[-1][5] - wpts[0][5]) / dt
                fp["motion_at_onset"] = {
                    "speed_ms": round(spd, 3),
                    "yaw_rate_rps": round(yr, 3),
                    "turning": abs(yr) > TURN_RATE,
                }

    # classify type
    if not fp["diverged"]:
        fp["type"] = "none"
    elif mx > RUNAWAY_M:
        fp["type"] = "runaway"
    elif fp["rel_yaw_err_deg_at_onset"] is not None and abs(fp["rel_yaw_err_deg_at_onset"]) > YAWSLIP_DEG:
        fp["type"] = "yaw_slip"
    else:
        fp["type"] = "drift"
    return fp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--output", default="")  # default: <run_dir>/fingerprint.json
    args = ap.parse_args()
    fp = fingerprint(args.run_dir)
    out = args.output or os.path.join(args.run_dir, "fingerprint.json")
    json.dump(fp, open(out, "w"), indent=2)
    mo = fp.get("motion_at_onset") or {}
    print(f"{fp['run']}: diverged={fp['diverged']} type={fp['type']} "
          f"onset_t={fp['onset_t']} onset_xy={fp['onset_gt_xy']} "
          f"max_err={fp['max_err_m']} turning={mo.get('turning')} "
          f"yaw_err={fp['rel_yaw_err_deg_at_onset']}deg imu_warns={fp['imu_warn_count']} "
          f"map_odom_max={fp['map_odom_max_m']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
