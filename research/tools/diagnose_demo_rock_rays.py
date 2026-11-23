#!/usr/bin/env python3
"""Inspect every saved demo-rock recast disagreement; never qualify or repair inputs."""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from ray_triangle_precision import intersections
from replay_local_terrain import sha


def mesh_hits(objects, origin, direction, distance):
    """Float64 intersections against the actual float32 authored mesh vertices."""
    hits = []
    for obj in objects:
        points = np.asarray(obj["points"], dtype=np.float32).astype(np.float64)
        triangles = points[np.asarray(obj["triangles"], dtype=int)]
        for hit in intersections(origin, direction, triangles, max_distance=distance):
            hits.append(dict(hit, collision=obj["path"]))
    return sorted(hits, key=lambda hit: hit["distance"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    check_run(args.capture)
    paths = [args.capture / name for name in ("native_timing_audit.json", "demo_rocks.json", "shadow_link.json")]
    audit, geometry, link = [json.loads(p.read_text()) for p in paths]
    if sha(paths[1]) != link["request"]["demo_rocks"]["geometry_sha256"]:
        raise ValueError("changed demo geometry")
    paths += [Path(__file__), Path(__file__).with_name("ray_triangle_precision.py")]
    hashes = {str(p): sha(p) for p in paths}
    cases, total = [], 0
    for tick, frame in enumerate(audit["frames"]):
        surface = frame["current_pose"]
        total += surface["selected"]
        for outlier in surface["outliers"]:
            origin, direction = np.asarray(outlier["ray_origin"]), np.asarray(outlier["ray_direction"])
            distance = outlier["cast_distance_m"]
            rays = [dict(offset_m=0.0, origin=origin.tolist())]
            rays += [
                dict(offset_m=r["offset_m"], origin=r["ray_origin"])
                for r in outlier["origin_counterfactuals"]
                if r["offset_m"]
            ]
            for ray in rays:
                ray["demo_mesh_intersections"] = mesh_hits(
                    geometry["objects"], ray["origin"], direction, distance - ray["offset_m"]
                )
                for hit in ray["demo_mesh_intersections"]:
                    hit["distance_from_sensor_m"] = hit["distance"] + ray["offset_m"]
            cases.append(dict(tick=tick, stamp=frame["observed_stamp"], original=outlier, float64_rays=rays))
    if any(sha(p) != hashes[str(p)] for p in paths):
        raise ValueError("changed diagnostic inputs")
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    report = dict(
        completed=True,
        acceptance=False,
        sampled_rays=total,
        disagreements=len(cases),
        cases=cases,
        input_sha256=hashes,
        limits=[
            "Only authored demo meshes; not an all-scene terrain oracle or PhysX cooking reproduction",
            "Saved reconstructed ray, not a replacement for original native rays/returns",
            "No native bytes, strict reference threshold, mapper output or qualification changed",
        ],
    )
    (output / "report.json").write_text(json.dumps(report, indent=2))
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(output)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
