#!/usr/bin/env python3
"""Fixed-budget analytic downward-beam screen, NOT a sensor configuration change.

Preserve mounting, upper 25 rings, azimuths, range cutoff and chassis first hits.
Compare only seven lowest rings. Excludes wheels, mast, relief and pose errors;
this cannot qualify native sensor performance or motion safety.
"""

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from diagnose_startup_visibility import screen
from local_terrain.ray_design import selected_setup
from replay_local_terrain import sha


def variants(elevations):
    elevations = np.asarray(elevations)
    if elevations.shape != (32,):
        raise ValueError("32-ring baseline required")
    return {str(end): np.r_[elevations[:25], np.linspace(-35, end, 7)] for end in (-60, -65, -70, -75, -80)}


def forward_mount_screen(setup, elevations):
    """Separate hypothetical placement, not a new selected profile or asset edit."""
    candidate = copy.deepcopy(setup)
    candidate["base_from_lidar_translation_m"][0] = 0.52
    return {
        str(end): dict(
            screen(candidate, variants(elevations)[str(end)]),
            base_from_lidar_translation_m=candidate["base_from_lidar_translation_m"],
        )
        for end in (-60, -75, -80)
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    source = Path(__file__).parent / "local_terrain/selected_lidar_optical.json"
    setup, elevations = selected_setup(source)
    cases = {name: screen(setup, beams) for name, beams in variants(elevations).items()}
    output = allocate_run(args.archive_root, limit=1024**2)
    paths = [Path(__file__), source, Path(__file__).with_name("diagnose_startup_visibility.py")]
    paths += [Path(__file__).parent / "local_terrain" / name for name in ("ray_design.py", "mounting_study.py")]
    for path in paths:
        (output / path.name).write_bytes(path.read_bytes())
    report = dict(
        completed=True,
        acceptance=False,
        native_validation=False,
        sensor_changed=False,
        assumption=__doc__,
        cases=cases,
        hypothetical_forward_mount_cases=forward_mount_screen(setup, elevations),
        input_sha256={str(p): sha(p) for p in paths},
    )
    (output / "report.json").write_text(json.dumps(report, indent=2))
    check_run(output)
    print(output)
    print(json.dumps(cases, indent=2))


if __name__ == "__main__":
    main()
