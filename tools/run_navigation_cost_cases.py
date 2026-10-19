#!/usr/bin/env python3
"""Frozen analytic planner cases, including deliberately hidden hazards.

These are controlled map/geometry fixtures, NOT claimed LiDAR observations.
GT footprints enter scoring only. A hidden-hazard collision is an expected
negative control, never removed to improve an aggregate safety percentage.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil

import numpy as np
from shapely import box

from archive_guard import allocate_run, check_run
from local_terrain.cost_planner import CostPlanner
from local_terrain.navigation_cost import GapPolicy, navigation_costs
from local_terrain.physx_reference import BodyBox
from local_terrain.planner_evaluation import path_sweep
from replay_local_terrain import sha


def scenarios():
    free = np.zeros((60, 60, 6))
    corridor = free.copy()
    corridor[:22, 30, 0] = corridor[38:, 30, 0] = 100
    corridor[33, 30, 0] = -1
    walls = [[0, -3, 0.1, -0.8], [0, 0.8, 0.1, 3]]
    seen = free.copy()
    seen[28:32, 29:31, 0] = 100
    minimum = free.copy()
    minimum[29:31, 29:31, 0] = 100
    hole = free.copy()
    hole[32:34, 30:32, 0] = -1
    wall = free.copy()
    wall[:, 30, 0] = 100
    return [
        dict(name="free_floor", layers=free, rocks=[], holes=[]),
        dict(name="seen_rock_detour", layers=seen, rocks=[[-0.1, -0.2, 0.1, 0.2]], holes=[]),
        dict(name="seen_10cm_high_15cm_wide_rock", layers=minimum, rocks=[[-0.075, -0.075, 0.075, 0.075]], holes=[]),
        dict(name="isolated_benign_gap", layers=corridor, rocks=walls, holes=[]),
        dict(name="hidden_8cm_high_6cm_wide_rock", layers=corridor, rocks=[*walls, [0.02, 0.32, 0.08, 0.38]], holes=[]),
        dict(name="hidden_10cm_hole", layers=corridor, rocks=walls, holes=[[0, 0.3, 0.1, 0.4]]),
        dict(name="known_20cm_hole", layers=hole, rocks=[], holes=[[0, 0.2, 0.2, 0.4]]),
        dict(name="solid_wall", layers=wall, rocks=[[0, -3, 0.1, 3]], holes=[]),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=256 * 1024**2)
    print(f"Run: {output}", flush=True)
    root = Path(__file__).parent
    paths = [
        Path(__file__),
        *[
            root / "local_terrain" / n
            for n in (
                "navigation_cost.py",
                "costmap.py",
                "cost_planner.py",
                "support_sweep.py",
                "physx_reference.py",
                "planner_evaluation.py",
            )
        ],
    ]
    hashes = {str(p): sha(p) for p in paths}
    for p in paths:
        target = output / "source" / p.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, target)
    patches = [
        BodyBox((x, y, -0.145), (0.04, 0.06, 0.001), "declared_wheel")
        for x in (-0.256, 0.256)
        for y in (-0.2854, 0.2854)
    ]
    body = BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "existing_body_guard")
    record = dict(origin=[-3, -3], resolution=0.1)
    start = np.eye(4)
    start[0, 3] = -1.4
    rows = []
    grids = {}
    try:
        for case in scenarios():
            row = dict(name=case["name"], rocks=case["rocks"], holes=case["holes"], policies={})
            grids[case["name"] + "_classes"] = case["layers"][:, :, 0]
            for policy, enabled in (("strict", False), ("isolated_gap", True)):
                costs, mask = navigation_costs(case["layers"], enabled=enabled)
                grids[case["name"] + "_" + policy] = costs
                result = CostPlanner(costs, record, start, patches, body).plan([1.4, 0])
                result.update(gap_cells=int(mask.sum()), rock_guard_intersections=[], hole_wheel_intersections=[])
                if result["found"]:
                    sweep = path_sweep(result["poses"], [body])
                    wheels = path_sweep(result["poses"], patches)
                    result["rock_guard_intersections"] = [
                        i for i, r in enumerate(case["rocks"]) if sweep.intersects(box(*r))
                    ]
                    result["hole_wheel_intersections"] = [
                        i for i, r in enumerate(case["holes"]) if wheels.intersects(box(*r))
                    ]
                row["policies"][policy] = result
            rows.append(row)
            print(
                json.dumps(
                    dict(
                        name=row["name"],
                        results={
                            p: {
                                k: v[k]
                                for k in ("found", "rock_guard_intersections", "hole_wheel_intersections", "wall_ms")
                            }
                            for p, v in row["policies"].items()
                        },
                    )
                ),
                flush=True,
            )
        np.savez_compressed(output / "grids.npz", **grids)
        if any(sha(p) != h for p, h in hashes.items()):
            raise ValueError("fixture sources changed")
        report = dict(
            completed=True,
            navigation_qualified=False,
            cases=rows,
            record=record,
            policy=asdict(GapPolicy()),
            input_sha256=hashes,
            grid_sha256=sha(output / "grids.npz"),
            limits=[
                __doc__,
                "No live sensing or vehicle tracking",
                "Hidden hazard exposures are expected evidence against blanket safety",
            ],
        )
        (output / "report.json").write_text(json.dumps(report, indent=2))
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), completed=False)))
        raise
    check_run(output)


if __name__ == "__main__":
    main()
