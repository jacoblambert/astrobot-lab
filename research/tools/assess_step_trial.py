#!/usr/bin/env python3
"""Exact controller replay plus actual progress for the bounded terrain trial.

No commands. Completed acquisition is distinct from reaching the goal. GT is
used only here to measure motion and consumer-time estimated-pose discrepancy.
The latter includes pose age; it is not same-timestamp SLAM trajectory accuracy.
"""

import argparse
import json
from pathlib import Path
from functools import partial

import numpy as np

from archive_guard import allocate_run, check_run
from local_terrain.step_trial import StepTrial, validate_proposal
from replay_contact_advisory import evaluate
from replay_local_terrain import sha
from costmap_scenarios import SCENARIOS, COSTMAP_PATTERNS, TRIAL_PATTERNS


def command_times(rows):
    result = dict(forward=0.0, reverse=0.0, turn=0.0, waiting=0.0, hold=0.0)
    for a, b in zip(rows, rows[1:]):
        mode = (
            "waiting"
            if a.get("execution_state") == "waiting_for_fresh_proposal"
            else "forward" if a["linear"] > 0 else "reverse" if a["linear"] < 0 else "turn" if a["angular"] else "hold"
        )
        result[mode] += b["stamp"] - a["stamp"]
    return result


def route_measurements(rows, request, demo=None):
    """GT evaluation only: never supplied to the online mapper/controller."""
    import math
    from natural_motion import HALF
    from shapely.geometry import MultiPoint, Polygon

    last = rows[-1]["proposal"]
    goals = last.get("goals_local_xy", [])
    reached = last.get("reached_goals", [])
    inverse = np.linalg.inv(last["local_from_map"])
    goals_world = []
    for goal in goals:
        p = inverse @ [*goal, last["local_from_base"][2][3], 1]
        goals_world.append((p[:3] + request["home_xyz"])[:2])
    legs = []
    for event in reached:
        row = next(r for r in rows if r["proposal"] and len(r["proposal"].get("reached_goals", [])) > event["index"])
        xy = np.asarray(row["world_from_base"])[:2, 3]
        legs.append(
            dict(
                index=event["index"],
                stamp=row["stamp"],
                gt_goal_error_m=float(np.linalg.norm(xy - goals_world[event["index"]])),
            )
        )
    clearance = None
    if demo:
        objects = [MultiPoint(np.asarray(o["points"])[:, :2]).convex_hull for o in demo["objects"]]
        distances = []
        corners = np.array([[-HALF[0], -HALF[1]], [HALF[0], -HALF[1]], [HALF[0], HALF[1]], [-HALF[0], HALF[1]]])
        for row in rows:
            pose = np.asarray(row["world_from_base"])
            angle = math.atan2(pose[1, 0], pose[0, 0])
            c, s = math.cos(angle), math.sin(angle)
            footprint = Polygon(corners @ np.array([[c, s], [-s, c]]) + pose[:2, 3])
            distances.append(min(footprint.distance(obstacle) for obstacle in objects))
        clearance = min(distances)
    return dict(
        goals_world_xy=[p.tolist() for p in goals_world],
        reached=legs,
        minimum_inflated_footprint_to_demo_rock_xy_m=clearance,
        clearance_scope="all executed ticks, XY convex hull of inserted rocks only; not all scene hazards",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    check_run(args.source)
    manifest = json.loads((args.source / "manifest.json").read_text())
    request = manifest["request"]
    if request["pattern"] not in TRIAL_PATTERNS:
        raise ValueError("explicit terrain-controlled trial required")
    capture = args.archive_root / request["capture"]
    check_run(capture)
    completion = json.loads((capture / "natural_holdout.json").read_text())
    collection = json.loads((args.source / "step_trial_collection.json").read_text())
    if not completion["complete"] or not collection["completed"] or (capture / "failure.json").exists():
        raise ValueError("completed acquisition required; do not score a failed launch as navigation")
    if sha(args.source / "step_trial.jsonl") != collection["output_sha256"]:
        raise ValueError("trial proposal log changed")
    expected = [json.loads(line) for line in (args.source / "step_trial.jsonl").open()]
    valid = [v for v in expected if v["valid"]]
    if not valid or len(valid) != collection["proposals"]:
        raise ValueError("trial denominator mismatch")
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    sources = [
        Path(__file__),
        Path(__file__).parent / "local_terrain/step_trial.py",
        args.source / "manifest.json",
        args.source / "step_trial.jsonl",
        args.source / "step_trial_collection.json",
        capture / "motion.jsonl",
        capture / "natural_holdout.json",
        capture / "step_trial_outcome.json",
    ]
    hashes = {str(p): sha(p) for p in sources}
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    planner = StepTrial(request["capture"])
    validator = validate_proposal
    if request["pattern"] in COSTMAP_PATTERNS:
        from local_terrain.costmap_trial import CostmapTrial, validate_proposal as costmap_validator

        scenario = request["pattern"] if request["pattern"] in SCENARIOS else None
        planner = CostmapTrial(request["capture"], 1.0 if request["pattern"] == "costmap1" else 2.0, scenario=scenario)
        validator = partial(costmap_validator, scenario=scenario)
        path = Path(__file__).parent / "local_terrain/costmap_trial.py"
        hashes[str(path)] = sha(path)
        path = Path(__file__).parent / "costmap_scenarios.py"
        hashes[str(path)] = sha(path)
    count = 0
    try:

        def observe(runtime, layers, record, actual):
            nonlocal count
            calculated = planner.update(runtime.advisory, actual["query_ns"])
            if calculated != valid[count]:
                raise ValueError(f"controller replay differs at proposal {count}")
            count += 1

        exact = evaluate(args.source, args.archive_root, output, observation_callback=observe)
        if count != len(valid):
            raise ValueError("missing replayed controller proposals")
        by_sequence = {v["sequence"]: v for v in valid}
        rows = [json.loads(line) for line in (capture / "motion.jsonl").open()]
        if not rows:
            raise ValueError("missing execution log")
        last_stamp = -1
        last_sequence = 0
        errors, ages = [], []
        for row in rows:
            if row["stamp"] <= last_stamp:
                raise ValueError("nonadvancing execution clock")
            last_stamp = row["stamp"]
            proposal = row["proposal"]
            state = row.get("execution_state", "checked_proposal")
            if state == "waiting_for_fresh_proposal":
                if row["linear"] or row["angular"]:
                    raise ValueError("motion while waiting for a fresh proposal")
                if proposal is None:
                    continue
            elif state != "checked_proposal":
                raise ValueError("undeclared execution state")
            sequence = proposal["sequence"]
            if sequence < last_sequence or proposal != by_sequence.get(sequence):
                raise ValueError("executed proposal not in exact mapper record")
            last_sequence = sequence
            if state == "waiting_for_fresh_proposal":
                if not (
                    row["proposal_wall_age_s"] > 0.5
                    or not 0 <= round(row["stamp"] * 1e9) - proposal["query_ns"] <= 200_000_000
                ):
                    raise ValueError("waiting row has no expired proposal")
                continue
            command = validator(
                proposal,
                request["capture"],
                (proposal["session"], proposal["epoch"]),
                round(row["stamp"] * 1e9),
                previous_sequence=sequence - 1,
            )
            if command != (row["linear"], row["angular"]):
                raise ValueError("executed command differs from checked proposal")
            estimated = np.linalg.inv(proposal["local_from_map"]) @ np.asarray(proposal["local_from_base"])
            estimated[:3, 3] += np.asarray(request["home_xyz"])
            truth = np.asarray(row["world_from_base"])
            errors.append(float(np.linalg.norm(estimated[:2, 3] - truth[:2, 3])))
            ages.append((round(row["stamp"] * 1e9) - proposal["pose_ns"]) / 1e9)
        outcome = json.loads((capture / "step_trial_outcome.json").read_text())
        if outcome["outcome"] != collection["outcome"] or rows[-1]["proposal"]["terminal"] != outcome["outcome"]:
            raise ValueError("trial terminal state mismatch")
        if rows[-1]["linear"] or rows[-1]["angular"]:
            raise ValueError("nonzero terminal command")
        poses = np.array([r["world_from_base"] for r in rows])
        start = np.asarray(request["world_from_base"])
        travel = poses[-1, :2, 3] - start[:2, 3]
        selected = next((v for v in reversed(valid) if v.get("goal_local_xy") is not None), None)
        world_goal = None
        if selected is not None:
            point = [*selected["goal_local_xy"], selected["local_from_base"][2][3], 1]
            world_goal = (np.linalg.inv(selected["local_from_map"]) @ point)[:3] + request["home_xyz"]
        command_seconds = command_times(rows)
        fragments = {}
        for label in ("first_refusal", "terminal_refusal"):
            path = args.source / (label + ".json")
            if not path.exists():
                continue
            value = json.loads(path.read_text())
            hashes[str(path)] = sha(path)
            fragments[label] = dict(
                query_ns=value["proposal"]["query_ns"],
                decision=value["proposal"]["decision"],
                area_cm2=sum(f["area_m2"] for f in value["fragments"]) * 10000,
                fragments=len(value["fragments"]),
                conflict_cells=value["conflict_cells"],
                terminal=value["terminal"],
            )
        demo = None
        if (capture / "demo_rocks.json").exists():
            path = capture / "demo_rocks.json"
            hashes[str(path)] = sha(path)
            demo = json.loads(path.read_text())
            if not demo.get("verified"):
                raise ValueError("unverified inserted obstacle geometry")
        report = dict(
            completed=True,
            navigation_qualified=False,
            outcome=outcome["outcome"],
            exact_replay=exact,
            controller_proposals_compared=count,
            execution_ticks=len(rows),
            nonzero_command_ticks=sum(bool(r["linear"] or r["angular"]) for r in rows),
            waiting_ticks=sum(r.get("execution_state") == "waiting_for_fresh_proposal" for r in rows),
            completed_estimated_steps=collection["steps"],
            gt_net_displacement_m=float(np.linalg.norm(travel)),
            gt_forward_progress_m=float(travel @ start[:2, 0]),
            gt_cumulative_xy_m=float(np.linalg.norm(np.diff(poses[:, :2, 3], axis=0), axis=1).sum()),
            estimated_remaining_goal_m=rows[-1]["proposal"]["remaining_goal_m"],
            gt_distance_to_fixed_goal_m=(
                None if world_goal is None else float(np.linalg.norm(poses[-1, :2, 3] - world_goal[:2]))
            ),
            fixed_goal_world_xy=None if world_goal is None else world_goal[:2].tolist(),
            scenario=request["pattern"],
            route_evaluation=route_measurements(rows, request, demo) if request["pattern"] in SCENARIOS else None,
            reached_route_goals=valid[-1].get("reached_goals"),
            gt_distance_to_start_m=float(np.linalg.norm(travel)),
            command_seconds=command_seconds,
            controller_ms_p50_p95_max=collection.get("controller_ms_p50_p95_max"),
            moving_proposals_with_unknown_wheel_cells=sum(
                bool(v["linear"] or v["angular"]) and v["decision"].get("unknown_wheel_cells", 0) > 0 for v in valid
            ),
            consumer_time_xy_error_p50_p95_max_m=np.percentile(errors, [50, 95, 100]).tolist(),
            consumer_pose_age_p50_p95_max_s=np.percentile(ages, [50, 95, 100]).tolist(),
            refusal=fragments,
            input_sha256=hashes,
            limits=[
                __doc__,
                "No native-ray/hazard-recall qualification in this report",
                "Independent GT emergency veto retained; no GT planner inputs",
                "2 cm stopping allowance is experimental, not calibrated",
            ],
        )
        if any(sha(p) != digest for p, digest in hashes.items()):
            raise ValueError("trial audit inputs changed")
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k not in ("input_sha256", "exact_replay")}, indent=2))
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(completed=False, error=repr(exc))))
        raise
    check_run(output)


if __name__ == "__main__":
    main()
