#!/usr/bin/env python3
"""Flat-plane/chassis-box visibility screen; NOT native or navigation acceptance.

Same fixed beam budget, optical origin and software range cutoff. Intersect the
chassis BEFORE range filtering: rejected self returns do not reveal ground.
Wheels, mast, terrain relief and SLAM errors are excluded from this ideal model.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.mounting_study import beam_directions, first_intersections, valid_returns
from local_terrain.ray_design import selected_setup
from replay_local_terrain import sha


def screen(setup, elevations, *, base_x=0.0, sensor_y=0.0, height=0.1449231063):
    azimuth = setup["horizontal_start_rad"] + np.arange(setup["horizontal_columns"]) * np.radians(
        setup["horizontal_resolution_deg"]
    )
    directions = beam_directions(azimuth, -np.radians(elevations))
    sensor = np.array(setup["base_from_lidar_translation_m"], dtype=float)
    sensor[1] = sensor_y
    sensor += [base_x, 0, height]
    box = np.array(setup["chassis_exclusion_base_aabb_m"]) + [base_x, 0, height]
    depth, labels = first_intersections(sensor, directions, [box])
    valid = valid_returns(depth, setup["min_range_m"], setup["max_range_m"])
    ground = sensor + directions[valid & (labels == 0)] * depth[valid & (labels == 0), None]
    track = (np.abs(ground[:, 1]) >= 0.22) & (np.abs(ground[:, 1]) <= 0.35) & (ground[:, 0] > 0.4)
    return dict(
        rays=len(depth),
        chassis_first_hits=int((labels == 1).sum()),
        valid_ground_returns=len(ground),
        minimum_ground_radius_from_start_m=float(np.linalg.norm(ground[:, :2], axis=1).min()),
        first_front_wheel_strip_ground_x_m=float(ground[track, 0].min()),
        ground_returns_inside_startup_circle=int(
            (np.linalg.norm(ground[:, :2], axis=1) < np.hypot(0.5037, 0.3454)).sum()
        ),
        base_x_m=base_x,
        sensor_y_m=sensor_y,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    source = Path(__file__).parent / "local_terrain/selected_lidar_optical.json"
    setup, elevations = selected_setup(source)
    cases = {
        name: screen(setup, elevations, base_x=x, sensor_y=y)
        for name, x, y in (
            ("parked_selected_mount", 0.0, 0.0),
            ("reverse_15cm", -0.15, 0.0),
            ("reverse_20cm", -0.20, 0.0),
            ("hypothetical_side_mount", 0.0, 0.32),
        )
    }
    output = allocate_run(args.archive_root, limit=1024**2)
    paths = [
        Path(__file__),
        source,
        Path(__file__).parent / "local_terrain/mounting_study.py",
        Path(__file__).parent / "local_terrain/ray_design.py",
    ]
    report = dict(
        completed=True,
        acceptance=False,
        native_validation=False,
        cases=cases,
        assumption=__doc__,
        input_sha256={str(p): sha(p) for p in paths},
    )
    (output / "report.json").write_text(json.dumps(report, indent=2))
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    check_run(output)
    print(output)
    print(json.dumps(cases, indent=2))


if __name__ == "__main__":
    main()
