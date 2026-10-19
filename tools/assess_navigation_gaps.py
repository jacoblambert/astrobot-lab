#!/usr/bin/env python3
"""Offline short-motion map availability and independent sampled-clear-floor scores.

Reads actual recorded maps, not a GT-pose reconstruction. No motion commands.
Support patches are declared assumptions, not measured tire contact areas.
Unknown beneath the body need not be ground support; occupied there still blocks.
No obstacle recall, stopping, uncertainty, or physical safety certification.
"""

import argparse
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import allocate_run, check_run
from diagnose_local_support import loaded_patches
from export_gap_examples import export
from local_terrain.physx_reference import BodyBox
from local_terrain.support_sweep import swept_polygons, touched_cells
from natural_motion import route, survey_points, validate_ground_tile, world_target, yaw
from replay_local_terrain import sha
from revision_evidence import validated_transform


def candidate_motions(start):
    """Fixed short straight/steering motions and observation turns, no reverse.

    Steering paths are two linear/SLERP segments, not continuous circular arcs.
    Every segment is conservatively swept by support_sweep, including the start.
    """
    start = validated_transform(start)
    result = {"stationary": [start]}
    for distance in (0.25, 0.5):
        for angle in (0.0, -np.pi / 6, np.pi / 6):
            poses = [start]
            for fraction in (0.5, 1.0):
                delta = np.eye(4)
                a = angle * fraction
                delta[:3, :3] = Rotation.from_euler("z", a).as_matrix()
                delta[:2, 3] = (
                    [distance * fraction, 0] if angle == 0
                    else [distance / angle * np.sin(a), distance / angle * (1 - np.cos(a))]
                )
                poses.append(start @ delta)
            result[f"move_{distance:.2f}_{round(np.degrees(angle)):+d}deg"] = poses
    for angle in (-np.pi / 6, np.pi / 6):
        delta = np.eye(4)
        delta[:3, :3] = Rotation.from_euler("z", angle).as_matrix()
        result[f"turn_{round(np.degrees(angle)):+d}deg"] = [start, start @ delta]
    return result


def cell_counts(classes, indices):
    classes = np.asarray(classes)
    if classes.ndim != 2 or not np.isin(classes, [-1, 0, 20, 100]).all():
        raise ValueError("invalid recorded classes")
    x, y = np.asarray(indices, dtype=int).T
    inside = (x >= 0) & (y >= 0) & (x < classes.shape[1]) & (y < classes.shape[0])
    values = np.full(len(x), -1, dtype=int)
    values[inside] = classes[y[inside], x[inside]]
    return dict(samples=len(x), free=int(np.isin(values, [0, 20]).sum()),
                occupied=int((values == 100).sum()), unknown=int((values == -1).sum()),
                outside=int((~inside).sum()))


def motion_score(classes, record, poses, patches, body, goal):
    def score(boxes):
        indices, _ = touched_cells(swept_polygons(poses, boxes), record["origin"], record["resolution"])
        return cell_counts(classes, indices)

    support, footprint = score(patches), score([body])
    blocked = support["occupied"] > 0 or footprint["occupied"] > 0
    state = "blocked" if blocked else "unknown" if support["unknown"] or footprint["outside"] else "available"
    progress = float(np.linalg.norm(poses[0][:2, 3] - goal) - np.linalg.norm(poses[-1][:2, 3] - goal))
    return dict(state=state, wheel_support=support, body_projection=footprint,
                progress_m=progress, toward_goal=progress >= 0.05,
                whole_footprint_available=state == "available" and footprint["unknown"] == 0,
                no_known_obstacle=not blocked and not footprint["outside"])


def floor_scores(classes, record, points, home):
    """Independent 5 cm survey points, including unknown/outside; no map-fitted GT."""
    transform = validated_transform(record["local_from_map"])
    local = (points - home) @ transform[:3, :3].T + transform[:3, 3]
    base = validated_transform(record["local_from_base"])
    relative = (local - base[:3, 3]) @ base[:3, :3]
    distance = np.linalg.norm(relative[:, :2], axis=1)
    ij = np.floor((local[:, :2] - record["origin"]) / record["resolution"]).astype(int)
    regions = {"whole_survey": np.ones(len(points), dtype=bool)}
    for low, high in ((0, 1), (1, 2), (2, 3)):
        for side, sign in (("front", 1), ("rear", -1)):
            regions[f"{side}_{low}_{high}m"] = (distance >= low) & (distance < high) & (sign * relative[:, 0] >= 0)
    return {key: cell_counts(classes, ij[mask]) for key, mask in regions.items()}


def summarize(rows):
    result = {}
    for name, selected in (
        ("all", rows), ("first_15s", [r for r in rows if r["elapsed_s"] < 15]),
        ("after_15s", [r for r in rows if r["elapsed_s"] >= 15]),
        ("translation", [r for r in rows if r["phase"] == "translate"]),
        ("rotation", [r for r in rows if r["phase"] == "rotate"]),
    ):
        counts = dict(samples=len(selected), progress_available=0, turn_available=0,
                      progress_or_turn=0, no_action=0, no_known_obstacle_progress=0,
                      whole_footprint_progress=0, stationary_supported=0)
        for row in selected:
            moves = [s for n, s in row["motions"].items() if n.startswith("move") and s["toward_goal"]]
            progress = any(s["state"] == "available" for s in moves)
            turn = any(s["state"] == "available" for n, s in row["motions"].items() if n.startswith("turn"))
            counts["progress_available"] += progress
            counts["turn_available"] += turn
            counts["progress_or_turn"] += progress or turn
            counts["no_action"] += not (progress or turn)
            counts["no_known_obstacle_progress"] += any(s["no_known_obstacle"] for s in moves)
            counts["whole_footprint_progress"] += any(s["whole_footprint_available"] for s in moves)
            counts["stationary_supported"] += row["motions"]["stationary"]["state"] == "available"
        result[name] = counts
    # Time between sampled failed decisions, NOT a claim about intervening maps.
    stretches, current = [], []
    for row in rows:
        usable = any(s["state"] == "available" and (n.startswith("turn") or s["toward_goal"])
                     for n, s in row["motions"].items() if n != "stationary")
        if not usable:
            current.append(row["elapsed_s"])
        elif current:
            stretches.append(current)
            current = []
    if current:
        stretches.append(current)
    result["no_action_sample_stretches"] = [dict(start_s=s[0], end_s=s[-1], samples=len(s)) for s in stretches]
    return result


def route_goal(origin, pattern, leg):
    """Next translation endpoint, using the driver's two declared route formats.

    Keep the original loop goals exact. Short routes return backwards, whereas
    candidate_motions deliberately has no reverse; report that limitation rather
    than silently changing the fixed motion candidates for those recordings.
    """
    targets = route(pattern)["targets"]
    if not isinstance(leg, (int, np.integer)) or not 0 <= leg <= len(targets):
        raise ValueError("invalid recorded route leg")
    if pattern == "loop16":
        phase = targets[leg][3] if leg < len(targets) else "hold"
        goal_leg = leg if leg % 2 == 0 else leg + 1
        heading = yaw(origin)
        planar = np.array([[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]])
        goal = targets[goal_leg][:2] if goal_leg < len(targets) else [2, 0]
        return phase, origin[:2, 3] + planar @ np.asarray(goal)
    rotating = lambda i: pattern.startswith("turn") or (pattern.startswith("combined") and i in (0, 3))
    phase = "hold" if leg == len(targets) else ("rotate" if rotating(leg) else "translate")
    for i in range(leg, len(targets)):
        if not rotating(i):
            return phase, world_target(origin, *targets[i])
    return phase, world_target(origin, 2., 0.)


def evaluate(recorder, mapper, capture, loaded, output):
    patches, hashes = loaded_patches(capture, loaded)
    paths = [capture / name for name in ("clearance.json", "clearance_tile.npz", "shadow_link.json",
                                         "natural_holdout.json", "motion.jsonl")]
    paths += [Path(__file__), Path(__file__).with_name("export_gap_examples.py"),
              Path(__file__).with_name("diagnose_local_support.py"), Path(__file__).with_name("natural_motion.py"),
              Path(__file__).with_name("local_terrain") / "support_sweep.py",
              Path(__file__).with_name("local_terrain") / "physx_reference.py"]
    hashes.update({str(p): sha(p) for p in paths})
    clearance, link, complete = [json.loads((capture / name).read_text()) for name in
                                  ("clearance.json", "shadow_link.json", "natural_holdout.json")]
    request = link["request"]
    mapper_request = json.loads((mapper / "manifest.json").read_text())["request"]
    if (not complete["complete"] or complete["request"] != request or request["capture"] != capture.name
            or {k: v for k, v in mapper_request.items() if k != "stamp"} != request
            or not clearance["passed"] or clearance["pattern"] != request["pattern"]
            or sha(paths[1]) != clearance["tile_sha256"]
            or any((capture / n).exists() for n in ("failure.json", "orchestration_failure.json"))):
        raise ValueError("completed matching route and independent surveyed floor required")
    origin = validated_transform(clearance["origin"])
    xy, shape = survey_points(origin, request["pattern"])
    with np.load(paths[1], allow_pickle=False) as data:
        if not np.array_equal(data["xy"], xy):
            raise ValueError("changed survey denominator")
        validate_ground_tile(xy, data["z"], shape, data["normal"])
        points = np.c_[xy, data["z"]]
    motion = [json.loads(line) for line in (capture / "motion.jsonl").open()]
    stamps = np.array([r["stamp"] for r in motion]) * 1e9
    if not len(stamps) or np.any(np.diff(stamps) <= 0):
        raise ValueError("ordered motion telemetry required")
    home = np.asarray(request["home_xyz"])
    # Declared inflated acquisition footprint; not a claim about exact collision shape.
    body = BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "declared_1.70x1.40m_planar_guard")
    contract = dict(motion_sampling="first recorded map per fixed 1-second evidence-time bin; all bins retained",
                    floor_sampling="every recorded valid map, all independent 5cm survey points",
                    minimum_goal_progress_m=0.05, patch_full_size_m=[0.08, 0.12, 0.002],
                    patches=[dict(center=p.center, half_extent=p.half_extent) for p in patches],
                    body_full_xy_m=[1.70, 1.40],
                    unknown_body_policy="not required as wheel support; occupied/outside still blocks",
                    pose_uncertainty_margin_m=0, navigation_authority=False,
                    goal="next translational route waypoint; next leg during rotation; final initial heading ray",
                    turns="30 degrees either way; availability not proof of visibility gain",
                    reference_limit="sampled clear route only, no obstacle recall or continuous floor proof",
                    route_pattern=request["pattern"],
                    reverse_limit="fixed candidates omit reverse; return legs of short routes may need a turn")
    (output / "manifest.json").write_text(json.dumps(dict(contract=contract, input_sha256=hashes), indent=2))
    rows, floors, first, last_bin = [], {}, None, -1
    with (output / "motion_scores.jsonl").open("x") as sink, (output / "floor_scores.jsonl").open("x") as fsink:
        def observer(classes, record):
            nonlocal first, last_bin
            stamp = record["evidence_ns"]
            if first is None:
                first = stamp
            elapsed = (stamp - first) / 1e9
            scores = floor_scores(classes, record, points, home)
            for region, counts in scores.items():
                total = floors.setdefault(region, dict.fromkeys(counts, 0))
                for k, v in counts.items():
                    total[k] += v
            fsink.write(json.dumps(dict(evidence_ns=stamp, elapsed_s=elapsed, regions=scores)) + "\n")
            bucket = (stamp - first) // 10**9
            if bucket <= last_bin:
                return
            last_bin = bucket
            index = int(np.searchsorted(stamps, stamp, side="right")) - 1
            leg = int(motion[index]["index"]) if index >= 0 else 0
            phase, goal_xy = route_goal(origin, request["pattern"], leg)
            if index < 0:
                phase = "hold"
            goal_map = np.r_[goal_xy, origin[2, 3]] - home
            transform = validated_transform(record["local_from_map"])
            goal = (transform[:3, :3] @ goal_map + transform[:3, 3])[:2]
            motions = {name: motion_score(classes, record, poses, patches, body, goal)
                       for name, poses in candidate_motions(record["local_from_base"]).items()}
            row = dict(evidence_ns=stamp, elapsed_s=elapsed, leg=leg, phase=phase, motions=motions)
            rows.append(row)
            sink.write(json.dumps(row) + "\n")
            if len(rows) % 30 == 0:
                print(f"Scored {len(rows)} fixed-time motion samples (+{elapsed:.1f}s).", flush=True)

        grid_report = export(recorder, mapper, output, grid_observer=observer)
    if any(sha(p) != h for p, h in hashes.items()):
        raise ValueError("diagnostic inputs changed")
    return dict(diagnostic_completed=True, acceptance=False, navigation_authority=False,
                contract=contract, summary=summarize(rows), floor_totals=floors, grids=grid_report,
                input_sha256=hashes, output_sha256={n: sha(output / n) for n in
                                                  ("motion_scores.jsonl", "floor_scores.jsonl", "manifest.json")})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("recorder", "mapper", "capture", "loaded"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    for name in ("recorder", "mapper", "capture", "loaded"):
        check_run(getattr(args, name))
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    started = time.monotonic()
    try:
        report = evaluate(args.recorder, args.mapper, args.capture, args.loaded, output)
        report["offline_wall_s"] = time.monotonic() - started
        check_run(output)
        (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
        print(json.dumps(report["summary"], indent=2), flush=True)
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
