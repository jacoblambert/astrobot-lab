#!/usr/bin/env python3
"""Does deliberate observation expose a supported exit beyond the startup prior?

Completed live source, exact raw AND advisory replay. Extra offline queries do
not command the rover, alter live output, or count turns inside the prior as
progress. Independent surveyed clear floor is evaluator-only; no obstacle recall.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from shapely import Polygon, union_all, difference

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from assess_navigation_gaps import cell_counts
from attribute_live_floor_support import REASONS
from local_terrain.pose_lookup import PublicationPoseLookup
from local_terrain.contact_support import patch_polygons
from local_terrain.support_sweep import swept_polygons
from natural_motion import survey_points, validate_ground_tile
from replay_contact_advisory import evaluate
from replay_local_terrain import sha


def exit_candidates():
    """Fixed 0.1..1.0 m straight prefixes both ways; every candidate retained."""
    result = {}
    for sign, direction in ((1, "forward"), (-1, "reverse")):
        for cm in range(10, 101, 10):
            end = np.eye(4)
            end[0, 3] = sign * cm / 100
            result[f"{direction}_{cm}cm"] = [np.eye(4), end]
    return result


def floor_bands(layers, record, points, home, start_xy):
    """Same fixed independent points at every timestamp, never trim unknowns."""
    transform = np.asarray(record["local_from_map"])
    local = (points - home) @ transform[:3, :3].T + transform[:3, 3]
    ij = np.floor((local[:, :2] - record["origin"]) / record["resolution"]).astype(int)
    radius = np.linalg.norm(points[:, :2] - start_xy, axis=1)
    result = {}
    for name, low, high in (
        ("startup", 0, np.hypot(0.5037, 0.3454)),
        ("outside_startup_to_1m", np.hypot(0.5037, 0.3454), 1),
        ("1_to_1_3m", 1, 1.3),
    ):
        selected = ij[(radius >= low) & (radius < high)]
        counts = cell_counts(layers[:, :, 0], selected)
        x, y = selected.T
        inside = (x >= 0) & (y >= 0) & (x < layers.shape[1]) & (y < layers.shape[0])
        codes = layers[y[inside], x[inside], 1]
        missing = layers[y[inside], x[inside], 0] == -1
        counts["unknown_reasons"] = {name: int(np.sum(missing & (codes == i))) for i, name in enumerate(REASONS)}
        result[name] = counts
    return result


def assess(source, archive, output):
    manifest = json.loads((source / "manifest.json").read_text())
    request = manifest["request"]
    capture = archive / request["capture"]
    paths = [
        capture / name for name in ("clearance.json", "clearance_tile.npz", "natural_holdout.json", "motion.jsonl")
    ]
    hashes = {str(p): sha(p) for p in [*paths, Path(__file__), Path(__file__).with_name("replay_contact_advisory.py")]}
    clearance = json.loads(paths[0].read_text())
    complete = json.loads(paths[2].read_text())
    if not clearance["passed"] or not complete["complete"] or clearance["pattern"] != request["pattern"]:
        raise ValueError("completed matching clear-floor acquisition required")
    if sha(paths[1]) != clearance["tile_sha256"]:
        raise ValueError("changed evaluator-only floor evidence")
    origin = np.asarray(clearance["origin"])
    expected, shape = survey_points(origin, request["pattern"])
    with np.load(paths[1], allow_pickle=False) as tile:
        if not np.array_equal(expected, tile["xy"]):
            raise ValueError("changed floor denominator")
        validate_ground_tile(expected, tile["z"], shape, tile["normal"])
        points = np.c_[expected, tile["z"]]
    motion = [json.loads(line) for line in paths[3].open()]
    poses = np.asarray([r["world_from_base"] for r in motion])
    yaw = np.unwrap(np.arctan2(poses[:, 1, 0], poses[:, 0, 0]))
    drift = np.linalg.norm(poses[:, :2, 3] - origin[:2, 3], axis=1)
    rows, first = [], None
    floor_totals = {}
    examples = []
    previous_yaw, accumulated_yaw, next_angle = None, 0.0, 0
    last_example = None
    source_copy = output / Path(__file__).name
    source_copy.write_bytes(Path(__file__).read_bytes())
    with (output / "observation.jsonl").open("x") as sink:

        def observer(runtime, layers, record, actual):
            nonlocal first, previous_yaw, accumulated_yaw, next_angle, last_example
            if first is None:
                first = actual["query_ns"]
            bands = floor_bands(layers, record, points, np.asarray(request["home_xyz"]), origin[:2, 3])
            for name, counts in bands.items():
                totals = floor_totals.setdefault(name, dict(samples=0, free=0, occupied=0, unknown=0, outside=0))
                for key in totals:
                    totals[key] += counts[key]
            row = dict(
                query_ns=actual["query_ns"], elapsed_s=(actual["query_ns"] - first) / 1e9, floor=bands, motions=None
            )
            base = runtime.advisory.context.local_from_base
            heading = np.arctan2(base[1, 0], base[0, 0])
            if previous_yaw is not None:
                accumulated_yaw += (heading - previous_yaw + np.pi) % (2 * np.pi) - np.pi
            previous_yaw = heading
            last_example = dict(
                elapsed_s=row["elapsed_s"],
                yaw_degrees=float(np.degrees(accumulated_yaw)),
                query_ns=actual["query_ns"],
                origin=record["origin"],
                resolution=record["resolution"],
                local_from_base=base.tolist(),
                classes=layers[:, :, 0].astype(int).tolist(),
                floor=bands,
            )
            if np.degrees(accumulated_yaw) >= next_angle:
                examples.append(dict(last_example, label=f"First map after {next_angle} degrees"))
                next_angle += 90
            if actual["motions"] is not None:
                advisory = runtime.advisory
                lookup = PublicationPoseLookup(runtime.timeline, actual["query_ns"])
                anchor = advisory.memory.anchors[0].stamp_ns
                startup_pose = (
                    np.asarray(record["local_from_map"])
                    @ lookup.associate(anchor).map_from_lidar
                    @ np.linalg.inv(runtime.timeline.extrinsic)
                    @ np.linalg.inv(runtime.node.core.base_from_imu)
                )
                # Use the un-eroded declared prior, so eroded border slivers
                # cannot masquerade as exits from the initial known-safe area.
                xyz = np.c_[advisory.memory.footprint, np.full(len(advisory.memory.footprint), advisory.memory.floor_z)]
                startup = Polygon((xyz @ startup_pose[:3, :3].T + startup_pose[:3, 3])[:, :2])
                assumed = union_all(
                    [startup, *[Polygon(p) for p in patch_polygons(advisory.context.local_from_base, advisory.patches)]]
                )
                decisions = {}
                goal = startup_pose[:2, 3] + 2 * startup_pose[:2, 0]
                for name, relative in exit_candidates().items():
                    decision = advisory.query(relative, query_ns=actual["query_ns"])
                    swept = union_all(
                        [Polygon(p) for p in swept_polygons(advisory.context.poses(relative), advisory.patches)]
                    )
                    outside = float(difference(swept, startup).area)
                    beyond_assumed = float(difference(swept, assumed).area)
                    endpoint = (advisory.context.local_from_base @ relative[-1])[:2, 3]
                    progress = float(
                        np.linalg.norm(goal - advisory.context.local_from_base[:2, 3]) - np.linalg.norm(goal - endpoint)
                    )
                    decisions[name] = dict(
                        decision,
                        outside_startup_wheel_area_m2=outside,
                        beyond_startup_and_current_wheels_m2=beyond_assumed,
                        supported_exit=decision["state"] == "available" and beyond_assumed > 0.001,
                        progress_toward_initial_forward_goal_m=progress,
                    )
                row["motions"] = decisions
            sink.write(json.dumps(row, allow_nan=False) + "\n")
            rows.append(row)

        exact = evaluate(source, archive, output, observation_callback=observer)
    examples.append(dict(last_example, label="Final map"))
    batches = [r for r in rows if r["motions"] is not None]
    if any(sha(p) != digest for p, digest in hashes.items()):
        raise ValueError("observation inputs changed")
    return dict(
        completed=True,
        acceptance=False,
        navigation_authority=False,
        exact_replay=exact,
        protocol=request["pattern"],
        exit_contract=dict(
            straight_distances_m=[i / 10 for i in range(1, 11)],
            directions=["forward", "reverse"],
            minimum_area_beyond_startup_and_current_wheels_m2=0.001,
            goal="2 m along the initial heading, in the same causal estimator frame",
            minimum_progress_toward_goal_m=0.05,
            commands_executed=False,
        ),
        maps=len(rows),
        decisions=len(batches),
        decisions_with_supported_exit=sum(any(d["supported_exit"] for d in r["motions"].values()) for r in batches),
        decisions_with_exit_toward_goal=sum(
            any(
                d["supported_exit"] and d["progress_toward_initial_forward_goal_m"] >= 0.05
                for d in r["motions"].values()
            )
            for r in batches
        ),
        first_supported_exit=next(
            (
                dict(elapsed_s=r["elapsed_s"], candidates=[n for n, d in r["motions"].items() if d["supported_exit"]])
                for r in batches
                if any(d["supported_exit"] for d in r["motions"].values())
            ),
            None,
        ),
        per_candidate={name: sum(r["motions"][name]["supported_exit"] for r in batches) for name in exit_candidates()},
        first_floor=rows[0]["floor"],
        final_floor=rows[-1]["floor"],
        all_map_floor=floor_totals,
        final_decisions=batches[-1]["motions"],
        examples=examples,
        physical_motion=dict(
            net_yaw_degrees=float(np.degrees(yaw[-1] - yaw[0])),
            max_origin_displacement_m=float(drift.max()),
            cumulative_translation_m=float(np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1).sum()),
        ),
        limits=[
            "GT acquisition driver, not terrain-controlled progress",
            "sampled clear floor, not obstacle recall",
            "actual skid does not enlarge the fixed startup assumption",
            "offline exit queries, no runtime acceptance",
            "independent native first-hit audit remains separate",
        ],
        input_sha256=hashes,
        output_sha256={"observation.jsonl": sha(output / "observation.jsonl")},
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        report = assess(args.source, args.archive_root, output)
        check_run(output)
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(
            json.dumps(
                {
                    k: report[k]
                    for k in (
                        "maps",
                        "decisions",
                        "decisions_with_supported_exit",
                        "per_candidate",
                        "physical_motion",
                        "final_floor",
                    )
                },
                indent=2,
            ),
            flush=True,
        )
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
