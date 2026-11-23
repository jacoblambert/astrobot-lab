#!/usr/bin/env python3
"""Archive a primitive-level Nav2 wall-footprint check, not a mission test.

Uses the existing analytic wall fixture. Pose probes/continuous collision labels
are evaluator-only. A high soft cost is reported, never counted as a hard veto.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run
from layered_terrain import TerrainMapper
from nav2_shadow_probe import probe, rectangle_intersects_segment


def wall_case(library, executable):
    sys.path.insert(0, str(Path(__file__).resolve().parent / "test"))
    from test_terrain_diagnostics import surface

    rotation = np.array([[np.cos(.17), np.sin(.17)], [-np.sin(.17), np.cos(.17)]])
    points = surface("positive")
    points[:, :2] = points[:, :2] @ rotation
    origin = (-1.925, -1.925)
    with TerrainMapper(library, width=40, height=40, origin=origin) as mapper:
        mapper.integrate(points, 1., [0., 0., .5])
        layers = mapper.project(1.)
    poses = np.array([[x, y, yaw] for yaw in (0., .17, .61, np.pi/2)
                      for x in np.arange(-1.1, 1.11, .1) for y in np.arange(-.9, 1.11, .1)])
    wall = np.array([[.32, .3], [.32, .9]]) @ rotation
    result = probe(executable, layers[:, :, 0], poses, origin=origin)
    accepted = np.array([r["accepted"] for r in result["queries"]])
    area = np.array([r["area_accepted"] for r in result["queries"]])
    padded = np.array([rectangle_intersects_segment(p[:2], p[2], *wall) for p in poses])
    body = np.array([rectangle_intersects_segment(p[:2], p[2], *wall, half=(.495, .335)) for p in poses])
    control = probe(executable, np.zeros((40, 40), dtype=int), poses, origin=origin)
    missed = np.array([r["accepted"] for r in control["queries"]])
    summary = dict(passed=bool(accepted.any() and not (accepted & padded).any()),
                   scope="installed InflationLayer + center/perimeter cost primitives, not complete planner/controller",
                   queries=len(poses), accepted=int(accepted.sum()),
                   padded_footprint_intersections_accepted=int((accepted & padded).sum()),
                   body_footprint_intersections_accepted=int((accepted & body).sum()),
                   missed_wall_control_intersections_accepted=int((missed & body).sum()),
                   area_policy=dict(passed=bool(area.any() and not (area & padded).any()),
                       accepted=int(area.sum()), padded_footprint_intersections_accepted=int((area & padded).sum()),
                       body_footprint_intersections_accepted=int((area & body).sum()), installed=False),
                   failures=[dict(pose=poses[i].tolist(), body_collision=bool(body[i]), **result["queries"][i])
                             for i in np.flatnonzero(accepted & padded)],
                   soft_high_cost_is_not_hard_rejection=True, navigation_acceptance="NOT_RUN")
    arrays = dict(layers=layers, inflated=result["inflated"], poses=poses, accepted=accepted,
                  area_accepted=area, padded_collision=padded, body_collision=body, wall=wall)
    return summary, arrays


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive") if Path("/archive").is_dir() else ARCHIVE_ROOT)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)  # two small JSON/npz files
    print(f"Run: {output}", flush=True)
    try:
        sources = [args.library, args.probe, Path(__file__), Path(__file__).with_name("nav2_shadow_probe.py"),
                   Path(__file__).parent / "nav2_shadow/nav2_shadow_probe.cpp",
                   Path(__file__).parent / "nav2_shadow/footprint_area.hpp",
                   Path(__file__).parent / "test/test_terrain_diagnostics.py"]
        hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
        summary, arrays = wall_case(args.library, args.probe)
        summary["input_sha256"] = hashes
        with (output / "nav2_shadow_arrays.npz").open("xb") as stream:
            np.savez_compressed(stream, **arrays)
    except Exception as exc:
        summary = dict(passed=False, failure=str(exc))
    with (output / "nav2_shadow.json").open("x") as stream:
        json.dump(summary, stream, indent=2, allow_nan=False)
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
