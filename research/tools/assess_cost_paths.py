#!/usr/bin/env python3
"""Matched costmap/planner experiment on exact-replayed native+GLIM maps.

All maps count for cost statistics. First decision of each fixed five-second
bin plans to 0.3/0.5/1.0 m in four cardinal directions, with strict/soft policy
on IDENTICAL maps, assumptions, poses, goals and search budgets. No commands.
"""

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import resource
import shutil
import time

import numpy as np
from shapely import MultiPoint, box

from archive_guard import allocate_run, check_run
from local_terrain.cost_planner import CostPlanner
from local_terrain.navigation_cost import GapPolicy, navigation_costs
from local_terrain.planner_evaluation import evaluate_path
from replay_contact_advisory import evaluate
from replay_local_terrain import sha


def gt_at(rows, stamp):
    # Same physics timestamp, allowing only integer-nanosecond representation.
    stamps = [round(row["stamp"] * 1e9) for row in rows]
    j = int(np.searchsorted(stamps, stamp))
    for i in (j, j - 1):
        if 0 <= i < len(rows) and abs(stamps[i] - stamp) <= 2:
            return rows[i]["world_from_base"]
    return None


def optimistic_control(layers, soft):
    """Diagnostic only: missing evidence is costly, explicit conflicts still block.

    This is deliberately more permissive than the proposed live policy. It
    measures the effect of unknown handling, NOT an inference of safe ground.
    """
    costs = soft.copy()
    missing = (layers[:, :, 0] == -1) & np.isin(layers[:, :, 1], (0, 4, 9, 12))
    costs[missing] = np.maximum(costs[missing], 90)
    return costs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    parser.add_argument(
        "--include-short-goals",
        action="store_true",
        help="Also test 0.1/0.2 m goals; retain every original 0.3/0.5/1.0 m goal",
    )
    parser.add_argument(
        "--optimistic-control", action="store_true", help="offline-only high-cost missing-ground control"
    )
    args = parser.parse_args()
    distances = (0.1, 0.2, 0.3, 0.5, 1.0) if args.include_short_goals else (0.3, 0.5, 1.0)
    check_run(args.source)
    manifest = json.loads((args.source / "manifest.json").read_text())
    request = manifest["request"]
    capture = args.archive_root / request["capture"]
    check_run(capture)
    completion = json.loads((capture / "natural_holdout.json").read_text())
    if not completion["complete"] or (capture / "failure.json").exists():
        raise ValueError("completed acquisition required")
    sources = [
        capture / name
        for name in (
            "natural_scene.json",
            "clearance.json",
            "clearance_tile.npz",
            "motion.jsonl",
            "natural_holdout.json",
        )
    ]
    sources += [Path(__file__), args.source / "manifest.json"]
    tools_root = Path(__file__).parent
    sources += [
        tools_root / "local_terrain" / name
        for name in (
            "navigation_cost.py",
            "costmap.py",
            "cost_planner.py",
            "planner_evaluation.py",
            "support_sweep.py",
            "physx_reference.py",
        )
    ]
    hashes = {str(p): sha(p) for p in sources}
    scene = json.loads((capture / "natural_scene.json").read_text())
    rocks = [(r["path"], box(*r["minimum"][:2], *r["maximum"][:2])) for r in scene["objects"]]
    if "demo_rocks" in request:
        path = capture / "demo_rocks.json"
        if sha(path) != request["demo_rocks"]["geometry_sha256"]:
            raise ValueError("changed demo rock reference")
        hashes[str(path)] = sha(path)
        demo = json.loads(path.read_text())
        if not demo["verified"]:
            raise ValueError("unverified demo geometry")
        rocks += [(r["path"], MultiPoint(np.asarray(r["points"])[:, :2]).convex_hull) for r in demo["objects"]]
    clearance = json.loads((capture / "clearance.json").read_text())
    if not clearance["passed"] or sha(capture / "clearance_tile.npz") != clearance["tile_sha256"]:
        raise ValueError("missing independent floor survey")
    with np.load(capture / "clearance_tile.npz", allow_pickle=False) as data:
        survey = MultiPoint(data["xy"]).convex_hull
    truth = [json.loads(s) for s in (capture / "motion.jsonl").read_text().splitlines()]
    if not truth or any(b["stamp"] <= a["stamp"] for a, b in zip(truth, truth[1:])):
        raise ValueError("ordered recorded GT execution poses required")
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    for p in sources:
        if p.is_relative_to(tools_root):
            target = output / "source" / p.relative_to(tools_root)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, target)
    contract = dict(
        policy=asdict(GapPolicy()),
        acquisition_pattern=request.get("pattern"),
        goals_m=list(distances),
        bearings_deg=[0, 90, 180, 270],
        sample_bin_s=5,
        search_expansions=4000,
        heuristic_weight=2,
        no_commands=True,
        scene_reference="projected natural USD bounds plus verified demo footprints",
        gt_tracking_reference="same-time GT start with perfect relative path tracking; no fitted alignment or planner GT inputs",
        acceptance=False,
        optimistic_control=args.optimistic_control,
        optimistic_cost=90 if args.optimistic_control else None,
    )
    (output / "manifest.json").write_text(
        json.dumps(dict(source=str(args.source), contract=contract, input_sha256=hashes), indent=2)
    )
    first = last_bin = None
    maps = 0
    gaps, cost_ms, rows, snapshots = [], [], [], {}
    try:
        with (output / "paths.jsonl").open("x") as sink:

            def observe(runtime, layers, record, actual):
                nonlocal first, last_bin, maps
                began = time.perf_counter()
                soft, mask = navigation_costs(layers)
                cost_ms.append((time.perf_counter() - began) * 1000)
                maps += 1
                gaps.append(int(mask.sum()))
                if first is None:
                    first = actual["query_ns"]
                bin_id = (actual["query_ns"] - first) // 5_000_000_000
                if bin_id == last_bin:
                    return
                last_bin = bin_id
                advisory = runtime.advisory
                start = advisory.context.local_from_base
                strict, _ = navigation_costs(layers, enabled=False)
                eligible = (layers[:, :, 0] == -1) & np.isin(layers[:, :, 1], (0, 4, 9, 12))
                row = dict(query_ns=actual["query_ns"], gap_cells=int(mask.sum()), plans={})
                gt = gt_at(truth, actual["query_ns"])
                snapshots[f"{bin_id}_cost"] = soft
                snapshots[f"{bin_id}_strict_cost"] = strict
                snapshots[f"{bin_id}_classes"] = layers[:, :, 0].astype(np.int8)
                row.update(
                    origin=record["origin"],
                    resolution=record["resolution"],
                    local_from_base=start.tolist(),
                    local_from_map=record["local_from_map"],
                    current_turn_polygons=[
                        list(polygon.exterior.coords)
                        for key, (polygon, _) in advisory.index.geometry.items()
                        if key[2] == "current_turn_operating_assumption"
                    ],
                )
                candidates = [("strict", strict), ("isolated_gap", soft)]
                if args.optimistic_control:
                    optimistic = optimistic_control(layers, soft)
                    candidates.append(("optimistic_control", optimistic))
                    snapshots[f"{bin_id}_optimistic_cost"] = optimistic
                    row["optimistic_missing_cells"] = int(eligible.sum())
                for policy, costs in candidates:
                    planner = CostPlanner(
                        costs,
                        record,
                        start,
                        advisory.patches,
                        advisory.body,
                        support_index=advisory.index,
                        support_eligible=eligible,
                    )
                    results = {}
                    for distance in distances:
                        for angle in (0, 90, 180, 270):
                            direction = np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))])
                            goal = start[:2, 3] + distance * direction
                            result = planner.plan(goal)
                            result["goal_local_xy"] = goal.tolist()
                            # Only AFTER search returns do independent GT geometry
                            # and actual pose enter the evaluation.
                            result["truth"] = evaluate_path(
                                result["poses"],
                                record,
                                request["home_xyz"],
                                rocks,
                                survey,
                                advisory.patches,
                                advisory.body,
                                gt_start=gt,
                            )
                            results[f"{distance:.1f}m_{angle}"] = result
                    row["plans"][policy] = results
                rows.append(row)
                sink.write(json.dumps(row) + "\n")
                sink.flush()
                print(
                    json.dumps(
                        dict(
                            bin=int(bin_id),
                            gap_cells=row["gap_cells"],
                            found={p: sum(r["found"] for r in v.values()) for p, v in row["plans"].items()},
                        )
                    ),
                    flush=True,
                )

            exact = evaluate(args.source, args.archive_root, output, observation_callback=observe)
        summaries = {}
        policies = ["strict", "isolated_gap"] + (["optimistic_control"] if args.optimistic_control else [])
        for policy in policies:
            plans = [p for r in rows for p in r["plans"][policy].values()]
            summaries[policy] = dict(
                goals=len(plans),
                found=sum(p["found"] for p in plans),
                reasons=dict(Counter(p["reason"] for p in plans)),
                search_ms_p50_p95_max=np.percentile([p["wall_ms"] for p in plans], [50, 95, 100]).tolist(),
                by_distance={
                    str(d): sum(
                        p["found"]
                        for r in rows
                        for name, p in r["plans"][policy].items()
                        if name.startswith(f"{d:.1f}m")
                    )
                    for d in distances
                },
                bins_with_a_path={
                    str(d): sum(
                        any(p["found"] for name, p in r["plans"][policy].items() if name.startswith(f"{d:.1f}m"))
                        for r in rows
                    )
                    for d in distances
                },
            )
            for label in ("map_frame", "gt_start_relative_tracking"):
                refs = [
                    p["truth"]["references"][label] for p in plans if p["found"] and label in p["truth"]["references"]
                ]
                summaries[policy][label] = dict(
                    evaluated=len(refs),
                    possible_rock_intersections=sum(bool(r["possible_rock_guard_intersections"]) for r in refs),
                    sampled_clear_paths=sum(r["sampled_clear_and_no_rock_guard_intersection"] for r in refs),
                    outside_survey=sum(not r["surveyed_floor_only"] for r in refs),
                )
        if any(sha(p) != h for p, h in hashes.items()):
            raise ValueError("changed planner evaluation inputs")
        np.savez_compressed(output / "grids.npz", **snapshots)
        report = dict(
            completed=True,
            navigation_qualified=False,
            maps=maps,
            decision_bins=len(rows),
            gap_cells_min_median_max=np.percentile(gaps, [0, 50, 100]).tolist(),
            costmap_ms_p50_p95_max=np.percentile(cost_ms, [50, 95, 100]).tolist(),
            peak_process_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
            policies=summaries,
            contract=contract,
            exact_replay=exact,
            input_sha256=hashes,
            output_sha256={name: sha(output / name) for name in ("paths.jsonl", "grids.npz")},
            limits=[
                "Offline paths, not executed motion",
                "Natural rock bounds are conservative, not exact mesh collisions",
                "Survey samples cannot rule out unsampled small holes",
                "Missing same-time GT is unverified, never silently extrapolated",
                "No hazard/compute promotion; mapper age is not oldest supporting evidence age",
                "Optimistic control, when requested, crosses missing evidence without a size bound; not a live policy",
            ],
        )
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k not in ("exact_replay", "input_sha256")}, indent=2))
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), completed=False)))
        raise
    check_run(output)


if __name__ == "__main__":
    main()
