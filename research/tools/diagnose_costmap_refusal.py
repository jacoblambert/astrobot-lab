#!/usr/bin/env python3
"""One-command counterfactual on sealed refusal grids; never route acceptance."""

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from archive_guard import allocate_run, check_run
from local_terrain.costmap_trial import CostmapTrial, grid_route, validate_proposal
from local_terrain.physx_reference import BodyBox
from replay_local_terrain import sha


def propose(snapshot, value, initial, previous, patches):
    pose, stamp = snapshot["local_from_base"], value["query_ns"]
    record = dict(origin=snapshot["origin"], resolution=0.1, local_from_map=snapshot["local_from_map"])
    context = SimpleNamespace(
        local_from_base=pose,
        poses=lambda relative: [pose @ p for p in relative],
        **{k: value[k] for k in ("query_ns", "pose_ns", "evidence_ns", "session", "epoch", "revision")},
    )
    advisory = SimpleNamespace(
        context=context,
        record=record,
        layers=snapshot["layers"],
        patches=patches,
        body=BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "original_body_guard"),
    )
    trial = CostmapTrial(value["capture"], scenario=value["scenario"])
    trial.first_ns, trial.origin = initial["query_ns"], np.asarray(initial["local_from_base"])
    trial.reference, trial.identity = snapshot["local_from_map"], (value["session"], value["epoch"])
    trial.goals = [np.asarray(p) for p in value["goals_local_xy"]]
    trial.goal_index, trial.reached = value["goal_index"], value["reached_goals"]
    trial.goal, trial.turning = trial.goals[trial.goal_index], bool(previous["angular"])
    # Restore the preceding controller state; otherwise a diagnostic silently
    # chooses a new route instead of testing the recorded commitment policy.
    if previous.get("route_goal_xy") is not None:
        trial.route_goal = np.asarray(previous["route_goal_xy"])
        trial.route_points = [np.asarray(p) for p in previous["route_waypoints_xy"]]
        trial.route_generation = previous["route_generation"]
    # Preserve the frozen turn anchor/hysteresis. Resetting it at the query
    # pose can falsely make a blocked centering maneuver look like a safe turn.
    if value.get("turn_origin_xy") is not None:
        trial.turn_origin = np.asarray(value["turn_origin_xy"])
        trial.turn_id = value["turn_id"]
        trial.turning = True
        trial.recentering = value["turn_recentering"]
    # Reset the refusal timer for this single hypothetical query, not the live run.
    candidate = trial.update(advisory, stamp)
    validate_proposal(candidate, value["capture"], trial.identity, stamp, scenario=value["scenario"])
    return candidate


def validate_snapshot(snapshot, value, record):
    if hashlib.sha256(snapshot["layers"].tobytes()).hexdigest() != record["layers_sha256"]:
        raise ValueError("snapshot layers disagree with sealed map")
    for name in ("origin", "local_from_map"):
        if not np.array_equal(snapshot[name], record[name]):
            raise ValueError("snapshot frame disagrees with sealed map")
    # Consumer pose is the proposal's checked pose, not the map evidence pose.
    # Both are sealed separately; never fit or replace one with the other.
    for name in ("local_from_base", "local_from_map"):
        if not np.array_equal(snapshot[name], value[name]):
            raise ValueError("snapshot pose disagrees with sealed proposal")


def diagnose(source):
    paths = [
        source / n
        for n in (
            "collection.json",
            "projections.jsonl",
            "step_trial.jsonl",
            "step_trial_collection.json",
            "contact_manifest.json",
        )
    ]
    paths += [
        source / (label + suffix)
        for label in ("first_refusal", "terminal_refusal")
        for suffix in (".json", ".npz")
        if (source / (label + suffix)).exists()
    ]
    paths += [
        Path(__file__),
        Path(__file__).parent / "local_terrain/costmap_trial.py",
        Path(__file__).with_name("costmap_scenarios.py"),
    ]
    hashes = {str(p): sha(p) for p in paths}
    collection = json.loads((source / "collection.json").read_text())
    trial_seal = json.loads((source / "step_trial_collection.json").read_text())
    if (
        not collection["experiment_completed"]
        or collection.get("status", {}).get("failure") is not None
        or not trial_seal["completed"]
    ):
        raise ValueError("completed recording required; route may have refused")
    if sha(source / "projections.jsonl") != collection["output_sha256"]["projections.jsonl"]:
        raise ValueError("projection receipt changed")
    if sha(source / "step_trial.jsonl") != trial_seal["output_sha256"]:
        raise ValueError("recorded proposal changed")
    proposals = [json.loads(line) for line in (source / "step_trial.jsonl").open()]
    proposals = [v for v in proposals if v.get("valid")]
    records = {r["publication_ns"]: r for r in map(json.loads, (source / "projections.jsonl").open())}
    manifest = json.loads((source / "contact_manifest.json").read_text())
    patches = [BodyBox(tuple(p["center"]), tuple(p["half_extent"]), "recorded_wheel") for p in manifest["patches"]]
    results = {}
    for label in ("first_refusal", "terminal_refusal"):
        if not (source / (label + ".json")).exists():
            continue
        value = json.loads((source / (label + ".json")).read_text())["proposal"]
        index = proposals.index(value)
        previous = next((p for p in reversed(proposals[:index]) if p["decision"]["state"] == "available"), proposals[0])
        snapshot = dict(np.load(source / (label + ".npz")))
        record = records[value["query_ns"]]
        validate_snapshot(snapshot, value, record)
        baseline, _ = grid_route(
            snapshot["layers"],
            record,
            snapshot["local_from_base"][:2, 3],
            value["goal_local_xy"],
            math.hypot(0.85, 0.70),
            partial=True,
            wheel_radius=max(np.linalg.norm(p.center[:2]) + math.hypot(*p.half_extent[:2]) for p in patches),
        )
        times = []
        for _ in range(20):
            start = time.perf_counter()
            candidate = propose(snapshot, value, proposals[0], previous, patches)
            times.append((time.perf_counter() - start) * 1000)
        results[label] = dict(
            original=value["decision"],
            v3_route_exists=bool(baseline),
            candidate=candidate,
            candidate_ms_p50_p95_max=np.percentile(times, [50, 95, 100]).tolist(),
        )
    if any(sha(p) != digest for p, digest in hashes.items()):
        raise ValueError("counterfactual inputs changed")
    return dict(
        acceptance=False,
        live_motion_tested=False,
        input_sha256=hashes,
        results=results,
        scope="single-command frozen-map counterfactual; no future maps, no simulated execution",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    check_run(args.source)
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        report = diagnose(args.source)
        (output / "report.json").write_text(json.dumps(report, indent=2))
        check_run(output)
        print(
            json.dumps(
                {
                    k: dict(
                        command=[v["candidate"]["linear"], v["candidate"]["angular"]],
                        decision=v["candidate"]["decision"],
                        timings=v["candidate_ms_p50_p95_max"],
                    )
                    for k, v in report["results"].items()
                }
            )
        )
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
