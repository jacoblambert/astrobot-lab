#!/usr/bin/env python3
"""Idealized move-observe-replan screen with the unchanged selected LiDAR.

Analytic flat floor plus chassis occlusion, exact poses, no wheel/mast occlusion,
no SLAM noise or skid. Each endpoint supplies ONE instantaneous complete scan.
Uses the frozen terrain classifier and full wheel/body sweep rules. Not native,
live, hazard or navigation qualification. No commands and no asset changes.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from shapely import Polygon, difference, union_all

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from assess_startup_observation import exit_candidates
from layered_terrain import TerrainMapper
from local_terrain.contact_index import IncrementalSupportIndex
from local_terrain.contact_query import PreparedMotion
from local_terrain.contact_runtime import startup_footprint
from local_terrain.contact_support import erode, patch_polygons
from local_terrain.mounting_study import beam_directions, first_intersections, valid_returns
from local_terrain.physx_reference import BodyBox
from local_terrain.ray_design import selected_setup
from local_terrain.support_sweep import swept_polygons
from replay_local_terrain import sha


HEIGHT = 0.1449231063


def pose(x=0.0, y=0.0, degrees=0.0):
    angle = np.radians(degrees)
    result = np.eye(4)
    result[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    result[:3, 3] = [x, y, HEIGHT]
    return result


def observe_route():
    """Reverse 5/10/15 cm, return, turn 15 degrees, repeat through 345 degrees.

    Return to center BEFORE turning. Every segment is checked against evidence
    already obtained. Translation 3 cm/s, yaw 0.1 rad/s, 0.1 s per endpoint scan.
    No support is granted just because a pose appears in this route.
    """
    rows = []
    for degrees in range(0, 360, 15):
        if degrees:
            for part in (degrees - 10, degrees - 5, degrees):
                rows.append((pose(degrees=part), np.radians(5) / 0.1, "turn"))
        direction = pose(degrees=degrees)[:2, 0]
        for distance in (-0.05, -0.10, -0.15, -0.10, -0.05, 0.0):
            xy = direction * distance
            rows.append((pose(*xy, degrees), 0.05 / 0.03, "reverse_observe_return"))
    return rows


def flat_returns(setup, elevations):
    azimuth = setup["horizontal_start_rad"] + np.arange(setup["horizontal_columns"]) * np.radians(
        setup["horizontal_resolution_deg"]
    )
    directions = beam_directions(azimuth, -np.radians(elevations))
    sensor = np.array(setup["base_from_lidar_translation_m"]) + [0, 0, HEIGHT]
    chassis = np.array(setup["chassis_exclusion_base_aabb_m"]) + [0, 0, HEIGHT]
    depth, labels = first_intersections(sensor, directions, [chassis])
    valid = valid_returns(depth, setup["min_range_m"], setup["max_range_m"]) & (labels == 0)
    return sensor + directions[valid] * depth[valid, None], sensor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    source = Path(__file__).parent / "local_terrain/selected_lidar_optical.json"
    setup, elevations = selected_setup(source)
    points, sensor = flat_returns(setup, elevations)
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    print(f"Run: {output}", flush=True)
    paths = [Path(__file__), source, args.library]
    paths += [
        Path(__file__).parent / "local_terrain" / name
        for name in (
            "contact_index.py",
            "contact_query.py",
            "contact_runtime.py",
            "contact_support.py",
            "mounting_study.py",
            "physx_reference.py",
            "ray_design.py",
            "support_sweep.py",
        )
    ]
    hashes = {str(p): sha(p) for p in paths}
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    patches = [
        BodyBox((x, y, -HEIGHT), (0.04, 0.06, 0.001), "wheel") for x in (-0.256, 0.256) for y in (-0.2854, 0.2854)
    ]
    body = BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "original_body_guard")
    footprint = startup_footprint(True)
    startup = Polygon(footprint)
    index = IncrementalSupportIndex()
    record = dict(origin=[-3.0, -3.0], resolution=0.1)
    current, stamp, rows, stopped = pose(), 0.0, [], None

    def support():
        polygons = [(erode(footprint, 0.02), "fixed_startup_with_original_erosion")]
        polygons += [(p, "current_wheels_only") for p in patch_polygons(current, patches)]
        index.update(polygons, record["origin"], record["resolution"])

    try:
        with TerrainMapper(args.library, width=60, height=60, origin=record["origin"], lifetime=60) as mapper:
            for number, (target, duration, action) in enumerate([(current, 0.0, "initial"), *observe_route()]):
                if number:
                    # Age existing evidence at arrival BEFORE granting this observation.
                    layers = mapper.project(stamp + duration)
                    support()
                    permission = PreparedMotion(layers, record, [current, target], patches, body).evaluate(index)
                    if permission["state"] != "available":
                        stopped = dict(index=number, action=action, decision=permission)
                        break
                current, stamp = target, stamp + duration + 0.1
                rotation = current[:3, :3]
                shift = current[:3, 3] - np.array([0, 0, HEIGHT])
                mapper.integrate(
                    points @ rotation.T + shift,
                    stamp,
                    rotation @ sensor + shift,
                    np.arctan2(rotation[1, 0], rotation[0, 0]),
                )
                layers = mapper.project(stamp)
                support()
                assumed = union_all([startup, *[Polygon(p) for p in patch_polygons(current, patches)]])
                decisions = {}
                for name, deltas in exit_candidates().items():
                    poses = [current @ delta for delta in deltas]
                    decision = PreparedMotion(layers, record, poses, patches, body).evaluate(index)
                    swept = union_all([Polygon(p) for p in swept_polygons(poses, patches)])
                    beyond = float(difference(swept, assumed).area)
                    decisions[name] = dict(decision, exit=decision["state"] == "available" and beyond > 0.001)
                rows.append(dict(index=number, stamp_s=stamp, action=action, base=current.tolist(), motions=decisions))
                if number % 30 == 0:
                    print(f"Observed {number + 1} poses", flush=True)
            np.savez_compressed(output / "final_grid.npz", layers=layers)
        if any(sha(path) != hashes[str(path)] for path in paths):
            raise ValueError("screen sources changed")
        (output / "decisions.json").write_text(json.dumps(rows, indent=2))
        exits = [r for r in rows if any(d["exit"] for d in r["motions"].values())]
        report = dict(
            completed=True,
            acceptance=False,
            navigation_authority=False,
            native=False,
            assumptions=__doc__,
            checked_observation_poses=len(rows),
            refused_next_move=stopped,
            exit_observations=len(exits),
            first_exit=exits[0] if exits else None,
            original_startup_radius_m=float(np.hypot(0.5037, 0.3454)),
            history_erosion_m=0.02,
            evidence_lifetime_s=60,
            valid_ground_returns_per_view=len(points),
            input_sha256=hashes,
            output_sha256={name: sha(output / name) for name in ("decisions.json", "final_grid.npz")},
        )
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k not in ("input_sha256", "first_exit")}, indent=2))
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), acceptance=False)))
        raise
    check_run(output)


if __name__ == "__main__":
    main()
