#!/usr/bin/env python3
"""Fixed 1/2 m observed-free goals: tentative route versus supported prefix.

Exact recorded maps and causal current poses. No commands, GT planning inputs,
future map evidence or success-based goal selection. Every five-second bin.
"""

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import shutil

import numpy as np
from shapely import MultiPoint, box

from archive_guard import allocate_run, check_run
from assess_cost_paths import gt_at
from local_terrain.horizon_planner import FixedFreeGoals, HorizonPlanner, HorizonPolicy
from local_terrain.planner_evaluation import evaluate_path
from replay_contact_advisory import evaluate
from replay_local_terrain import sha


POLICIES = ("full_local_route", "tentative_route", "executable_prefix")


def summarize(rows, policy):
    plans = [p for row in rows for p in row["plans"][policy].values()]
    result = dict(
        goals=len(plans),
        found=sum(p["found"] for p in plans),
        reasons=dict(Counter(p["reason"] for p in plans)),
        by_distance={
            str(d): sum(
                p["found"] for r in rows for name, p in r["plans"][policy].items() if name.startswith(f"{d:.1f}m")
            )
            for d in (1.0, 2.0)
        },
        bins_with_a_path={
            str(d): sum(
                any(p["found"] for name, p in r["plans"][policy].items() if name.startswith(f"{d:.1f}m")) for r in rows
            )
            for d in (1.0, 2.0)
        },
        search_ms_p50_p95_max=np.percentile([p["wall_ms"] for p in plans], [50, 95, 100]).tolist(),
    )
    for label in ("map_frame", "gt_start_relative_tracking"):
        refs = [
            p["truth"]["references"][label] for p in plans if p["found"] and label in p["truth"].get("references", {})
        ]
        result[label] = dict(
            evaluated=len(refs),
            possible_rock_intersections=sum(bool(r["possible_rock_guard_intersections"]) for r in refs),
            outside_survey=sum(not r["surveyed_floor_only"] for r in refs),
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    parser.add_argument(
        "--goal-selection-delay-s",
        type=float,
        default=0.0,
        help="Declared observation wait before selecting fixed goals; pre-selection bins remain counted",
    )
    args = parser.parse_args()
    selection = FixedFreeGoals(args.goal_selection_delay_s)
    check_run(args.source)
    source_manifest = json.loads((args.source / "manifest.json").read_text())
    request = source_manifest["request"]
    capture = args.archive_root / request["capture"]
    check_run(capture)
    completed = json.loads((capture / "natural_holdout.json").read_text())
    if not completed["complete"] or (capture / "failure.json").exists():
        raise ValueError("completed acquisition required")
    sources = [
        capture / n
        for n in ("natural_holdout.json", "natural_scene.json", "clearance.json", "clearance_tile.npz", "motion.jsonl")
    ]
    root = Path(__file__).parent
    sources += [Path(__file__), root / "assess_cost_paths.py", args.source / "manifest.json"]
    sources += [
        root / "local_terrain" / n
        for n in (
            "horizon_planner.py",
            "cost_planner.py",
            "navigation_cost.py",
            "costmap.py",
            "planner_evaluation.py",
            "support_sweep.py",
            "physx_reference.py",
        )
    ]
    scene = json.loads((capture / "natural_scene.json").read_text())
    rocks = [(r["path"], box(*r["minimum"][:2], *r["maximum"][:2])) for r in scene["objects"]]
    if "demo_rocks" in request:
        path = capture / "demo_rocks.json"
        if sha(path) != request["demo_rocks"]["geometry_sha256"]:
            raise ValueError("changed demo geometry")
        sources.append(path)
        demo = json.loads(path.read_text())
        if not demo["verified"]:
            raise ValueError("unverified demo geometry")
        rocks += [(r["path"], MultiPoint(np.asarray(r["points"])[:, :2]).convex_hull) for r in demo["objects"]]
    clearance = json.loads((capture / "clearance.json").read_text())
    if not clearance["passed"] or sha(capture / "clearance_tile.npz") != clearance["tile_sha256"]:
        raise ValueError("independent floor survey required")
    with np.load(capture / "clearance_tile.npz", allow_pickle=False) as data:
        survey = MultiPoint(data["xy"]).convex_hull
    truth = [json.loads(s) for s in (capture / "motion.jsonl").read_text().splitlines()]
    if not truth or any(b["stamp"] <= a["stamp"] for a, b in zip(truth, truth[1:])):
        raise ValueError("ordered GT execution poses required")
    hashes = {str(p): sha(p) for p in sources}
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    for path in sources:
        if path.is_relative_to(root):
            dest = output / "source" / path.relative_to(root)
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, dest)
    contract = dict(
        policy=asdict(HorizonPolicy()),
        acquisition_pattern=request.get("pattern"),
        goals_m=[1.0, 2.0],
        bearings_deg=[0, 90, 180, 270],
        sample_bin_s=5,
        fixed_goals=True,
        goal_selection_delay_s=args.goal_selection_delay_s,
        selection="nearest initially measured/inferred free cell within 0.3 m; no GT or search success",
        goal_free_definition="target center and reached route endpoint are observed free; not a fully surveyed endpoint footprint",
        no_commands=True,
        acceptance=False,
        two_horizons=True,
        gt_tracking_reference="same-time GT start, perfect relative path tracking; not executed paths",
    )
    (output / "manifest.json").write_text(
        json.dumps(dict(source=str(args.source), contract=contract, input_sha256=hashes), indent=2)
    )
    rows, grids, gaps = [], {}, []
    first = last_bin = frame = None
    maps = 0
    try:
        with (output / "paths.jsonl").open("x") as sink:

            def observe(runtime, layers, record, actual):
                nonlocal first, last_bin, frame, maps
                maps += 1
                if first is None:
                    first = actual["query_ns"]
                    frame = np.asarray(record["local_from_map"])
                if not np.array_equal(frame, record["local_from_map"]):
                    raise ValueError("fixed-goal reference changed")
                bin_id = (actual["query_ns"] - first) // 5_000_000_000
                if bin_id == last_bin:
                    return
                last_bin = bin_id
                advisory = runtime.advisory
                start = advisory.context.local_from_base
                planner = HorizonPlanner(
                    layers, record, start, advisory.patches, advisory.body, support_index=advisory.index
                )
                selection.observe(planner, actual["query_ns"])
                goals, nominal_goals = selection.goals, selection.nominal
                gt = gt_at(truth, actual["query_ns"])
                row = dict(
                    query_ns=actual["query_ns"],
                    origin=record["origin"],
                    resolution=record["resolution"],
                    local_from_base=start.tolist(),
                    local_from_map=record["local_from_map"],
                    current_turn_polygons=[
                        list(p.exterior.coords)
                        for key, (p, _) in advisory.index.geometry.items()
                        if key[2] == "current_turn_operating_assumption"
                    ],
                    plans={p: {} for p in POLICIES},
                    goals=goals,
                    nominal_goals=nominal_goals,
                    prefix_details={},
                    gap_cells=int(planner.isolated_mask.sum()),
                    grid_keys={
                        "full_local_route": f"{bin_id}_strict_cost",
                        "tentative_route": f"{bin_id}_route_cost",
                        "executable_prefix": f"{bin_id}_cost",
                    },
                )
                grids[f"{bin_id}_cost"] = planner.local_costs
                grids[f"{bin_id}_strict_cost"] = planner.local_costs
                grids[f"{bin_id}_route_cost"] = planner.route_costs
                grids[f"{bin_id}_classes"] = layers[:, :, 0].astype(np.int8)
                gaps.append(row["gap_cells"])
                for name, goal in goals.items():
                    candidate = planner.plan(goal)
                    baseline = planner.full_local_route(goal)
                    candidates = (baseline, candidate["route"], candidate["prefix"])
                    for policy, result in zip(POLICIES, candidates):
                        if selection.selected_ns is None:
                            result["reason"] = "awaiting_initial_observation"
                        result["goal_local_xy"] = goal
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
                        row["plans"][policy][name] = result
                    candidate["prefix"]["tentative_poses"] = candidate["route"]["poses"]
                    row["prefix_details"][name] = {
                        k: v for k, v in candidate.items() if k not in ("prefix", "route", "checked_poses")
                    }
                rows.append(row)
                sink.write(json.dumps(row) + "\n")
                sink.flush()
                print(
                    json.dumps(
                        dict(
                            bin=int(bin_id),
                            found={p: sum(v["found"] for v in row["plans"][p].values()) for p in POLICIES},
                        )
                    ),
                    flush=True,
                )

            exact = evaluate(args.source, args.archive_root, output, observation_callback=observe)
        if any(sha(p) != h for p, h in hashes.items()):
            raise ValueError("evaluation inputs changed")
        np.savez_compressed(output / "grids.npz", **grids)
        executable = [v["executable_translation_m"] for r in rows for v in r["prefix_details"].values()]
        report = dict(
            completed=True,
            navigation_qualified=False,
            maps=maps,
            decision_bins=len(rows),
            contract=contract,
            fixed_goals=selection.goals,
            nominal_goals=selection.nominal,
            goal_selection_query_ns=selection.selected_ns,
            goal_missing_at_selection=sum(g is None for g in selection.goals.values()),
            gap_cells_min_median_max=np.percentile(gaps, [0, 50, 100]).tolist(),
            executable_translation_m_min_median_max=np.percentile(executable, [0, 50, 100]).tolist(),
            policies={p: summarize(rows, p) for p in POLICIES},
            exact_replay=exact,
            input_sha256=hashes,
            output_sha256={n: sha(output / n) for n in ("paths.jsonl", "grids.npz")},
            limits=[
                "No paths executed; fixed goals are not proven reached",
                "Unknown route cost never authorizes local motion",
                "Turning area is assumed support, not new LiDAR ground",
                "Prefix reserves 2 cm, not a braking certificate",
                "Natural rock bounds are conservative; missing GT/outside survey remains unverified",
            ],
        )
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(
            json.dumps(
                {k: v for k, v in report.items() if k not in ("exact_replay", "input_sha256", "output_sha256")},
                indent=2,
            )
        )
        check_run(output)
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), completed=False)))
        raise


if __name__ == "__main__":
    main()
