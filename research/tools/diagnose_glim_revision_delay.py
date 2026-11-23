#!/usr/bin/env python3
"""Fixed-deadline revision sensitivity; NOT consumer-time navigation accuracy."""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run
from export_shadow_trajectory import apply_record
from measure_exported_glim import measure
from measure_live_glim_accuracy import statistics
from shadow_pose_replay import sha
from shadow_terrain import PoseTimeline

DEADLINES_NS = (300_000_000, 500_000_000, 1_000_000_000)


def fixed_deadline(timeline, frames, truth, base_from_lidar, end_clock_ns, deadline_ns):
    """No trimming: one unavailable association makes aggregate error ineligible."""
    poses, associations, missing = [], [], []
    lidar_base = np.linalg.inv(base_from_lidar)
    for index, frame in enumerate(frames):
        stamp = round(frame["observed_stamp"] * 1e9)
        output = stamp + deadline_ns
        try:
            if output > end_clock_ns:
                raise ValueError("deadline extends beyond recorded receipt clock")
            association = timeline.associate(stamp, output, mode="revised")
            poses.append(association.map_from_lidar @ lidar_base)
            associations.append(dict(chunk=index, **association.record()))
        except ValueError as exc:
            missing.append(dict(chunk=index, acquisition_ns=stamp, error=str(exc)))
    report = dict(
        deadline_s=deadline_ns / 1e9,
        expected_samples=len(frames),
        associated_samples=len(poses),
        missing=missing,
        full_interval_available=not missing,
        statistics=None if missing else statistics(poses, truth),
    )
    return report, associations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "recorder", "trajectory"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        # Reuse the complete-recording identity, transport, GT and hash checks;
        # preserve independent native surface failures and baseline metrics.
        baseline = measure(args.capture, args.recorder, args.trajectory, output)
        frames = json.loads((args.capture / "native_timing_audit.json").read_text())["frames"]
        request = json.loads((args.capture / "shadow_link.json").read_text())["request"]
        exported = json.loads((args.trajectory / "shadow_trajectory.json").read_text())
        timeline = PoseTimeline(exported["session"], exported["epoch"])
        for line in (args.trajectory / "trajectory.jsonl").read_text().splitlines():
            apply_record(timeline, json.loads(line))
        with np.load(output / "pose_arrays.npz") as arrays:
            truth = arrays["gt_base"].copy()
        results = []
        with (output / "deadline_associations.jsonl").open("x") as stream:
            for deadline in DEADLINES_NS:
                result, rows = fixed_deadline(
                    timeline, frames, truth, request["base_from_lidar"], exported["recording_end_clock_ns"], deadline
                )
                results.append(result)
                for row in rows:
                    stream.write(json.dumps(dict(deadline_ns=deadline, **row), allow_nan=False) + "\n")
        report = dict(
            completed=True,
            acceptance=False,
            interpretation="Offline acquisition-pose sensitivity at fixed receipt deadlines, not current robot pose or live runtime acceptance",
            baseline=baseline,
            results=results,
            source_sha256=sha(Path(__file__)),
            associations_sha256=sha(output / "deadline_associations.jsonl"),
        )
        with (output / "revision_delay.json").open("x") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
        print(json.dumps(results, indent=2), flush=True)
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
