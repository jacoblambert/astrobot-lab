#!/usr/bin/env python3
"""Bounded pose/transport audit using existing unchanged evaluators.

This writes trajectory and diagnostic arrays, not sensor recordings. Each of
the two outputs has a strict 1 GiB quota and the normal 50 GiB reserve. Capture
completion is not route success; a clean terrain refusal remains a refusal.
"""

import argparse
import json
import shutil
from pathlib import Path

from archive_guard import allocate_run, check_run
from export_shadow_trajectory import export
from measure_exported_glim import measure

DIAGNOSTIC_LIMIT = 1024**3


def audit(capture, recorder, archive):
    from restore_native_archive import restore
    restore(capture)  # Closed packed captures only; original per-tick bytes/hashes.
    trajectory = allocate_run(archive, limit=DIAGNOSTIC_LIMIT)
    print(f"Trajectory: {trajectory}", flush=True)
    shutil.copyfile(__file__, trajectory / Path(__file__).name)
    try:
        result = export(recorder, trajectory)
        with (trajectory / "shadow_trajectory.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        check_run(trajectory)
    except BaseException as exc:
        with (trajectory / "failure.json").open("x") as stream:
            json.dump(dict(error=repr(exc), acceptance=False), stream)
        raise
    output = allocate_run(archive, limit=DIAGNOSTIC_LIMIT)
    print(f"Pose audit: {output}", flush=True)
    shutil.copyfile(__file__, output / Path(__file__).name)
    try:
        result = measure(capture, recorder, trajectory, output)
        with (output / "pose_diagnostic.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        check_run(output)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=repr(exc), acceptance=False), stream)
        raise
    print(json.dumps(dict(trajectory=str(trajectory), output=str(output), results=result["results"])), flush=True)
    return trajectory, output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("recorder", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    audit(args.capture, args.recorder, args.archive_root)


if __name__ == "__main__":
    main()
