#!/usr/bin/env python3
"""GT body contacts with rock cores along a recorded GT trajectory (gate runs, post hoc).

Reads a TUM trajectory in the simulator world frame (phase4_exploration_eval.py --record-dir
writes gt_base.tum) and the sim nav run's evaluator geometry (natural_scene.json and the
terrain-only clearance_tile.npz). Scores the unpadded body at every sample. No ROS.

Rocks the body already overlaps at the first sample (the eval starts before the mission, so
that is the spawn pose: seed 7 spawns over a 0.12 m rock) are world content the rover cannot
avoid. Their contacts are reported as spawn_rock_contacts and left out of contacts_by_height.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import shapely

from archive_guard import ARCHIVE_ROOT
from audit_terrain_robustness import HAZARD_HEIGHTS, LARGE, Scene
from v2_nav_trial import body


def contacts(scene, tum):
    rocks = [(o["core"], o["height"], o["name"]) for o in scene.objects if o["height"] >= HAZARD_HEIGHTS[0]]
    tree = shapely.STRtree([c for c, _, _ in rocks])
    found = []
    for t, x, y, _, qx, qy, qz, qw in tum:
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        hits = tree.query(body(x, y, yaw), predicate="intersects")
        if len(hits):
            tallest = max(hits, key=lambda i: rocks[i][1])
            found.append((float(t), float(x), float(y), rocks[tallest][1], rocks[tallest][2]))
    return found


def summarize(found, samples, spawn=()):
    """found: contacts() rows; spawn: names of rocks under the body at the first sample."""
    avoidable = [c for c in found if c[4] not in spawn]
    return dict(samples=samples, contact_samples=len(avoidable),
                contacts_by_height={f"ge_{h:.2f}": sum(c[3] >= h for c in avoidable) for h in HAZARD_HEIGHTS},
                large_contact=any(c[3] >= LARGE for c in avoidable),
                rocks_touched=sorted({c[4] for c in avoidable}),
                first_contact=avoidable[0] if avoidable else None,
                spawn_rocks=sorted(spawn), spawn_rock_contacts=len(found) - len(avoidable))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tum", type=Path)
    parser.add_argument("--scene-run", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive") if Path("/archive").is_dir() else ARCHIVE_ROOT)
    args = parser.parse_args()
    matches = sorted(args.archive_root.glob(args.scene_run + "*"))
    if len(matches) != 1:
        raise SystemExit(f"{args.scene_run}: {len(matches)} archive matches")
    scene = Scene(matches[0], [0., 0., 0.])  # world frame, like the TUM
    tum = np.loadtxt(args.tum, ndmin=2)
    found = contacts(scene, tum)
    spawn = {c[4] for c in contacts(scene, tum[:1])}
    summary = dict(scene_run=matches[0].name, **summarize(found, len(tum), spawn))
    text = json.dumps(summary, indent=2)
    if args.output:
        args.output.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
