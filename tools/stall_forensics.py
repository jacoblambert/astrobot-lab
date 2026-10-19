#!/usr/bin/env python3
"""Classify each stall snapshot by ROOT CAUSE, to confirm whether the rover hits rocks
because they're undetected (BEV-clear but GT-rock) vs map-misaligned vs drove-through-a-
mapped-rock vs terrain. Pure offline analysis of existing gate-run .npz snapshots -- no sim.

Each snapshot's `latest_diagnostics_json` already carries (computed live by
astrobot_diagnostics):
  - maps.gt.path.occupied_or_high_fraction : fraction of the PLANNED path that crosses
    GT rock cells (the truth: did the path go through a real rock?)
  - maps.bev.path.lethal_cells / occupied_or_high_cells : did the BEV (what the planner
    used) have an obstacle on that same path?
  - nav.motion.position_error_m : SLAM-vs-GT misalignment at the stall.
Plus we sample the GT rock map (`gt_data`) under the rover footprint at `latest_gt_pose`
to ask "is the rover physically on a rock right now?".

Classes:
  detection_miss        : GT rock on path BUT BEV path clear, small pose error
                          -> the planner routed through a rock the BEV never marked. (ROOT)
  misaligned_then_miss  : GT rock on path, BEV clear, BUT pose error > thresh
                          -> rock may be mapped-but-shifted (localization).
  drove_through_mapped  : GT rock on path AND BEV path also had lethal cells
                          -> map had it; planner/controller went anyway (nav issue).
  rock_under_rover_only : path clear of GT rock but a GT rock sits under the footprint
                          -> near-field / blind-cone contact (saw too late).
  terrain_not_rock      : no GT rock on path and none under the footprint
                          -> bog is terrain/traction, not a rock.

Usage:
  python3 tools/stall_forensics.py [run_dir ...]
  (default: rosbags/phase4_gate_runs/explore_radius_seed*_20260612_*)
"""
import glob
import json
import os
import sys

import numpy as np

POSE_ERR_MISALIGN_M = 0.40   # above this, attribute a BEV-clear/GT-rock stall to misalignment
FOOTPRINT_RADIUS_M = 0.50    # rover half-extent for "rock physically under the rover"
GT_LETHAL = 80               # GT-map cost >= this = rock
GT_PATH_FRAC_THRESH = 0.01   # >1% of path through GT rock = "GT rock on path"
BEV_LETHAL_CELLS_THRESH = 1  # >=1 lethal BEV cell on path = "BEV had it"


def classify(diag, gt_rock_under_rover):
    nav = diag.get("nav", {})
    maps = diag.get("maps", {})
    gt_path = maps.get("gt", {}).get("path", {})
    bev_path = maps.get("bev", {}).get("path", {})
    gt_frac = float(gt_path.get("occupied_or_high_fraction", 0.0))
    bev_lethal = int(bev_path.get("lethal_cells", 0)) + int(bev_path.get("occupied_or_high_cells", 0))
    pose_err = float(nav.get("motion", {}).get("position_error_m", 0.0))

    gt_on_path = gt_frac > GT_PATH_FRAC_THRESH
    bev_on_path = bev_lethal >= BEV_LETHAL_CELLS_THRESH

    if gt_on_path and not bev_on_path:
        return ("misaligned_then_miss" if pose_err > POSE_ERR_MISALIGN_M else "detection_miss",
                gt_frac, bev_lethal, pose_err)
    if gt_on_path and bev_on_path:
        return ("drove_through_mapped", gt_frac, bev_lethal, pose_err)
    # No GT rock on the planned path:
    if gt_rock_under_rover:
        return ("rock_under_rover_only", gt_frac, bev_lethal, pose_err)
    return ("terrain_not_rock", gt_frac, bev_lethal, pose_err)


def rock_under_footprint(z):
    """Sample gt_data under the rover footprint at latest_gt_pose. True if any rock cell."""
    if "gt_data" not in z.files or "gt_meta" not in z.files or "latest_gt_pose" not in z.files:
        return False
    gt = z["gt_data"]
    res, ox, oy = [float(v) for v in z["gt_meta"]]
    gx, gy = float(z["latest_gt_pose"][0]), float(z["latest_gt_pose"][1])
    cx = int(round((gx - ox) / res))
    cy = int(round((gy - oy) / res))
    r = max(1, int(round(FOOTPRINT_RADIUS_M / res)))
    y0, y1 = max(0, cy - r), min(gt.shape[0], cy + r + 1)
    x0, x1 = max(0, cx - r), min(gt.shape[1], cx + r + 1)
    if y1 <= y0 or x1 <= x0:
        return False
    return bool(np.any(gt[y0:y1, x0:x1] >= GT_LETHAL))


def main():
    run_dirs = sys.argv[1:] or sorted(glob.glob(
        "rosbags/phase4_gate_runs/explore_radius_seed*_20260612_*"))
    classes = ["detection_miss", "misaligned_then_miss", "drove_through_mapped",
               "rock_under_rover_only", "terrain_not_rock"]
    grand = {c: 0 for c in classes}
    total = 0
    for rd in run_dirs:
        snaps = sorted(glob.glob(os.path.join(rd, "**", "*gt_motion_stalled*.npz"), recursive=True))
        if not snaps:
            continue
        counts = {c: 0 for c in classes}
        for s in snaps:
            try:
                z = np.load(s, allow_pickle=True)
                diag = json.loads(str(z["latest_diagnostics_json"].reshape(-1)[0]))
            except Exception as e:
                print(f"  skip {os.path.basename(s)}: {e}")
                continue
            cls, _, _, _ = classify(diag, rock_under_footprint(z))
            counts[cls] += 1
            grand[cls] += 1
        n = sum(counts.values())
        total += n
        print(f"\n{os.path.basename(rd)}  ({n} stall snapshots)")
        for c in classes:
            if counts[c]:
                print(f"    {c:24} {counts[c]:3}  ({100*counts[c]/n:4.0f}%)")
    print(f"\n===== AGGREGATE ({total} stalls across {len(run_dirs)} run dirs) =====")
    for c in classes:
        if grand[c]:
            print(f"  {c:24} {grand[c]:3}  ({100*grand[c]/max(total,1):4.0f}%)")
    miss = grand["detection_miss"] + grand["misaligned_then_miss"]
    print(f"\n  DETECTION-RELATED (miss+misalign): {miss}/{total} ({100*miss/max(total,1):.0f}%)")
    print(f"  drove-through-mapped (nav):        {grand['drove_through_mapped']}/{total}")
    print(f"  terrain (not a rock):              {grand['terrain_not_rock']}/{total}")


if __name__ == "__main__":
    main()
