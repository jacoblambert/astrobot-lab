#!/usr/bin/env python3
"""Attribute completed-trial delay and local-prefix limits; no commands or fixes.

GT bearings are evaluator-only counterfactuals, never controller inputs. Command
durations use each logged command until the next recorded tick; no extrapolated
terminal interval. Profiles are instrumented diagnostics, not compute gates.
"""

import argparse
from collections import Counter, defaultdict
import cProfile
import json
from pathlib import Path
import pstats
import shutil
import time

import numpy as np
from shapely import Polygon, box, intersection, union_all

from archive_guard import allocate_run, check_run
from local_terrain.contact_query import PreparedMotion
from local_terrain.horizon_planner import HorizonPlanner, length, translation_prefix
from local_terrain.support_sweep import swept_polygons, touched_cells
from replay_contact_advisory import evaluate
from replay_local_terrain import relocate, sha


def distribution(values):
    return (
        None
        if not len(values)
        else dict(zip(("min", "median", "p95", "max"), map(float, np.percentile(values, [0, 50, 95, 100]))))
    )


def execution_accounting(rows, world_goal_xy):
    if (
        len(rows) < 2
        or not np.isfinite([r["stamp"] for r in rows]).all()
        or any(b["stamp"] <= a["stamp"] for a, b in zip(rows, rows[1:]))
    ):
        raise ValueError("ordered nonempty execution intervals required")
    duration = defaultdict(float)
    denied = defaultdict(float)
    ticks = Counter()
    wall_ages, sim_ages, opposite = [], [], 0.0

    def mode(row):
        if row.get("execution_state") == "waiting_for_fresh_proposal":
            if row["linear"] or row["angular"]:
                raise ValueError("stale wait has nonzero command")
            return "stale_wait"
        if row.get("execution_state", "checked_proposal") != "checked_proposal":
            raise ValueError("unknown execution state")
        if row["linear"] and row["angular"]:
            raise ValueError("unexpected arc command")
        return "forward" if row["linear"] else "turn" if row["angular"] else "hold"

    for i, row in enumerate(rows):
        state = mode(row)
        ticks[state] += 1
        dt = rows[i + 1]["stamp"] - row["stamp"] if i + 1 < len(rows) else 0.0
        duration[state] += dt
        proposal = row.get("proposal")
        if state == "stale_wait":
            denied[
                "forward" if proposal and proposal["linear"] else "turn" if proposal and proposal["angular"] else "hold"
            ] += dt
            if proposal:
                sim_ages.append(row["stamp"] - proposal["query_ns"] / 1e9)
                wall_ages.append(row["proposal_wall_age_s"])
        elif state == "turn":
            actual = np.asarray(row["world_from_base"])
            delta = np.asarray(world_goal_xy) - actual[:2, 3]
            error = (np.arctan2(delta[1], delta[0]) - np.arctan2(actual[1, 0], actual[0, 0]) + np.pi) % (
                2 * np.pi
            ) - np.pi
            if abs(error) > 0.05 and np.sign(error) != np.sign(row["angular"]):
                opposite += dt
    total = rows[-1]["stamp"] - rows[0]["stamp"]
    return dict(
        execution_ticks=len(rows),
        interval_count=len(rows) - 1,
        logged_duration_s=total,
        seconds=dict(duration),
        percent={k: 100 * v / total for k, v in duration.items()},
        ticks=dict(ticks),
        stale_requested_seconds=dict(denied),
        stale_sim_age_s=distribution(sim_ages),
        stale_wall_age_s=distribution(wall_ages),
        opposite_gt_goal_turn_s=opposite,
        maximum_recorded_tick_gap_s=max(b["stamp"] - a["stamp"] for a, b in zip(rows, rows[1:])),
        last_tick_duration_extrapolated=False,
        counterfactual_bearings_not_control_inputs=True,
    )


def support_area(layers, record, poses, patches):
    polygons = [p for a, b in zip(poses, poses[1:]) for p in swept_polygons([a, b], patches)]
    cells, _ = touched_cells(polygons, record["origin"], record["resolution"])
    footprint = union_all([Polygon(p) for p in polygons])
    low = np.asarray(record["origin"]) + cells * record["resolution"]
    parts = intersection(
        footprint, box(low[:, 0], low[:, 1], low[:, 0] + record["resolution"], low[:, 1] + record["resolution"])
    )
    area = defaultdict(float)
    for (x, y), part in zip(cells, parts):
        if not (0 <= x < layers.shape[1] and 0 <= y < layers.shape[0]):
            raise ValueError("prefix area outside evaluated grid")
        key = "measured" if layers[y, x, 0] == 0 else "inferred" if layers[y, x, 0] == 20 else "not_raw_free"
        area[key] += float(part.area)
    return dict(area_m2=dict(area), raw_free_fraction=(area["measured"] + area["inferred"]) / footprint.area)


def inspect_prefix(planner, route, recorded_prefix, advisory, layers, record):
    poses = translation_prefix(route["poses"], planner.policy.local_horizon_m)
    checked, first_bad, extra = [poses[0]], None, 0.0
    for a, b in zip(poses, poses[1:]):
        if planner.local.check(planner.local.geometry([a, b])) is None:
            prepared = PreparedMotion(layers, record, [a, b], advisory.patches, advisory.body)
            first_bad = prepared.evaluate(advisory.index)
            d = length([a, b])
            if d > 1e-12:
                for n in range(1, int(np.floor(d / 0.01)) + 1):
                    prefix = translation_prefix([a, b], min(0.01 * n, d))
                    if planner.local.check(planner.local.geometry(prefix)) is None:
                        break
                    extra = min(0.01 * n, d)
            break
        checked.append(b)
    return dict(
        checked_translation_m=length(checked),
        subdivision_extra_m=extra,
        first_rejection=first_bad,
        support=support_area(layers, record, recorded_prefix["poses"], advisory.patches),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("trial_audit", type=Path)
    parser.add_argument("horizon", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    for path in (args.source, args.trial_audit, args.horizon):
        check_run(path)
        if (path / "failure.json").exists():
            raise ValueError("completed evidence only")
    read = lambda p: json.loads(p.read_text())
    trial = read(args.trial_audit / "report.json")
    horizon = read(args.horizon / "report.json")
    if not trial["completed"] or not horizon["completed"] or not trial["exact_replay"]["exact_advisory_decisions"]:
        raise ValueError("completed exact controller/map audits required")
    if relocate(read(args.horizon / "manifest.json")["source"], args.archive_root).resolve() != args.source.resolve():
        raise ValueError("horizon/source mismatch")
    request = read(args.source / "manifest.json")["request"]
    capture = args.archive_root / request["capture"]
    check_run(capture)
    if not read(capture / "natural_holdout.json")["complete"] or (capture / "failure.json").exists():
        raise ValueError("incomplete acquisition")
    sources = [
        Path(__file__),
        args.trial_audit / "report.json",
        args.horizon / "report.json",
        args.horizon / "manifest.json",
        args.horizon / "paths.jsonl",
        args.source / "manifest.json",
        args.source / "step_trial.jsonl",
        capture / "motion.jsonl",
        capture / "natural_holdout.json",
    ]
    audited = {relocate(name, args.archive_root).resolve(): digest for name, digest in trial["input_sha256"].items()}
    for path in (capture / "motion.jsonl", args.source / "step_trial.jsonl", args.source / "manifest.json"):
        if path.resolve() not in audited or sha(path) != audited[path.resolve()]:
            raise ValueError("changed audited trial inputs")
    if sha(args.horizon / "paths.jsonl") != horizon["output_sha256"]["paths.jsonl"]:
        raise ValueError("changed evaluated paths")
    root = Path(__file__).parent
    sources += [
        root / "local_terrain" / n
        for n in ("horizon_planner.py", "cost_planner.py", "contact_query.py", "support_sweep.py")
    ]
    hashes = {str(p): sha(p) for p in sources}
    proposals = [json.loads(s) for s in (args.source / "step_trial.jsonl").read_text().splitlines()]
    proposals = [p for p in proposals if p["valid"]]
    execution = [json.loads(s) for s in (capture / "motion.jsonl").read_text().splitlines()]
    if len(proposals) != trial["controller_proposals_compared"] or len(execution) != trial["execution_ticks"]:
        raise ValueError("trial denominators changed")
    initial = proposals[0]
    world = np.linalg.inv(initial["local_from_map"])
    world[:3, 3] += request["home_xyz"]
    goal = (world @ [*initial["goal_local_xy"], initial["local_from_base"][2][3], 1])[:2]
    accounting = execution_accounting(execution, goal)
    path_rows = [json.loads(s) for s in (args.horizon / "paths.jsonl").read_text().splitlines()]
    targets = {r["query_ns"]: r for r in path_rows}
    if len(targets) != horizon["decision_bins"]:
        raise ValueError("planning bin mismatch")
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    for p in sources:
        if p.is_relative_to(root):
            dest = output / "source" / p.relative_to(root)
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, dest)
    results, profiles, queries = [], [], []
    visited = set()
    try:

        def observe(runtime, layers, record, actual):
            stamp = actual["query_ns"]
            if stamp not in targets:
                return
            visited.add(stamp)
            row = targets[stamp]
            advisory = runtime.advisory
            start = advisory.context.local_from_base
            if not np.array_equal(start, row["local_from_base"]) or not np.array_equal(
                record["local_from_map"], row["local_from_map"]
            ):
                raise ValueError("recorded planner exposure changed")
            planner = HorizonPlanner(
                layers, record, start, advisory.patches, advisory.body, support_index=advisory.index
            )
            direct = np.eye(4)
            direct[0, 3] = 0.07
            began = time.perf_counter()
            decision = advisory.query([np.eye(4), direct], query_ns=stamp)
            queries.append(dict(query_ns=stamp, wall_ms=(time.perf_counter() - began) * 1000, state=decision["state"]))
            for name, prefix in row["plans"]["executable_prefix"].items():
                if not prefix["found"]:
                    continue
                detail = inspect_prefix(
                    planner, row["plans"]["tentative_route"][name], prefix, advisory, layers, record
                )
                results.append(dict(query_ns=stamp, goal=name, **detail))
            # First forward 2 m success and first forward 2 m exhausted search.
            name = "2.0m_0"
            expected = row["plans"]["tentative_route"][name]
            kind = expected["reason"]
            if kind in ("path", "search_budget") and kind not in [p["kind"] for p in profiles]:
                profile = cProfile.Profile()
                calculated = profile.runcall(planner.plan, row["goals"][name])
                if (
                    calculated["route"]["found"] != expected["found"]
                    or calculated["route"]["poses"] != expected["poses"]
                ):
                    raise ValueError("profile changed route output")
                profile.dump_stats(str(output / (kind + ".prof")))
                stats = pstats.Stats(profile)
                top = sorted(stats.stats.items(), key=lambda p: p[1][3], reverse=True)[:18]
                profiles.append(
                    dict(
                        kind=kind,
                        query_ns=stamp,
                        instrumented=True,
                        cold_route_cache=True,
                        functions=[
                            dict(file=k[0], line=k[1], name=k[2], calls=v[1], self_s=v[2], cumulative_s=v[3])
                            for k, v in top
                        ],
                    )
                )
            print(
                json.dumps(
                    dict(
                        bin=len(visited) - 1,
                        prefixes=sum(p["found"] for p in row["plans"]["executable_prefix"].values()),
                    )
                ),
                flush=True,
            )

        exact = evaluate(args.source, args.archive_root, output, observation_callback=observe)
        if len(visited) != len(targets) or len(results) != horizon["policies"]["executable_prefix"]["found"]:
            raise ValueError("missing diagnostic samples")
        if any(sha(p) != h for p, h in hashes.items()):
            raise ValueError("diagnostic inputs changed")
        report = dict(
            completed=True,
            diagnostic_only=True,
            navigation_qualified=False,
            execution=accounting,
            source_trial_forward_m=trial["gt_forward_progress_m"],
            world_goal_xy=goal.tolist(),
            measured_raw_support_fraction=distribution([r["support"]["raw_free_fraction"] for r in results]),
            subdivision_extra_m=distribution([r["subdivision_extra_m"] for r in results]),
            rejected_edge_reasons=dict(
                Counter(r["first_rejection"]["reason"] if r["first_rejection"] else "horizon" for r in results)
            ),
            fresh_seven_cm_query_ms=distribution([q["wall_ms"] for q in queries]),
            queries=queries,
            prefix_diagnostics=results,
            profiles=profiles,
            exact_replay=exact,
            input_sha256=hashes,
            limits=[
                "Recorded commands, not performance of the new planner in closed loop",
                "No extrapolated terminal tick or invented missing observations",
                "GT bearing disagreement diagnoses estimator/controller interaction, not a proposed GT controller",
                "Not-raw-free support includes explicit assumptions/retained support; no new free terrain inferred",
                "Profile and isolated query timings are not live compute acceptance",
                "1 cm subdivisions are diagnosis only; no live policy changed",
            ],
        )
        (output / "report.json").write_text(json.dumps(report, indent=2))
        check_run(output)
        print(
            json.dumps(
                {
                    k: v
                    for k, v in report.items()
                    if k not in ("exact_replay", "input_sha256", "prefix_diagnostics", "profiles", "queries")
                },
                indent=2,
            )
        )
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), completed=False)))
        raise


if __name__ == "__main__":
    main()
