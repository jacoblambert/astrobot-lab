#!/usr/bin/env python3
"""Fixed 200 ms pose-publication sensitivity, not live latency or terrain acceptance."""
import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from diagnose_glim_accuracy import summary
from shadow_pose_replay import load_pose_sensitivity, sha
from shadow_terrain import PoseTimeline, integer


DEADLINE_NS = 200_000_000  # Two declared 10 Hz scan periods; never selected from errors.


def observation_horizon(timeline, recording_end_ns):
    end = timeline.last_arrival_ns if recording_end_ns is None else integer(recording_end_ns, "recording end")
    if end < timeline.last_arrival_ns:
        raise ValueError("recording end precedes a revision receipt")
    return end


def deadline_coverage(timeline, acquisition_ns, recording_end_ns=None):
    """Explain every missed deadline without dropping samples or enlarging it.

    Availability is the recorded *next-clock upper bound*, not a precise callback
    timestamp. Even a nanosecond excess is retained as unproven, never rounded to
    a passing deadline. Final-history lookup explains eventual availability only.
    """
    stamps = [integer(t, "acquisition") for t in acquisition_ns]
    if not stamps or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("empty or unordered acquisition sequence")
    horizon = observation_horizon(timeline, recording_end_ns)
    misses = []
    for index, stamp in enumerate(stamps):
        deadline = stamp + DEADLINE_NS
        try:
            if deadline > horizon:
                raise ValueError("recording does not cover the complete fixed deadline")
            timeline.associate(stamp, deadline, mode="revised")
        except ValueError as exc:
            row = dict(index=index, acquisition_ns=stamp, deadline_ns=deadline, reason=str(exc))
            try:
                eventual = timeline.associate(stamp, timeline.last_arrival_ns, mode="causal")
                row.update(first_available_upper_bound_ns=eventual.available_ns,
                           upper_bound_excess_ns=eventual.available_ns-deadline)
            except ValueError as eventual_error:
                row["eventual_failure"] = str(eventual_error)
            misses.append(row)
    return dict(samples=len(stamps), fixed_deadline_ns=DEADLINE_NS,
                strict_deadline_proven=not misses, missed_samples=len(misses), misses=misses,
                availability="next recorded simulation clock upper bound, not exact receipt time",
                samples_dropped=0, tolerance_added_ns=0)


def deadline_associations(timeline, acquisition_ns, recording_end_ns=None):
    stamps = [integer(t, "acquisition") for t in acquisition_ns]
    if not stamps or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("empty or unordered acquisition sequence")
    horizon = observation_horizon(timeline, recording_end_ns)
    results = []
    for stamp in stamps:
        deadline = stamp + DEADLINE_NS
        if deadline > horizon:
            raise ValueError("recording does not cover the complete fixed deadline")
        # Historical query excludes every revision arriving after the deadline,
        # even though the timeline contains the whole replay. Missing brackets
        # are a full-run failure; never trim the denominator or extrapolate.
        results.append(timeline.associate(stamp, deadline, mode="revised").record())
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("pair", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        from export_shadow_trajectory import apply_record
        for folder in (args.capture, args.pair):
            check_run(folder)
        native = json.loads((args.capture / "native_timing_audit.json").read_text())["frames"]
        # Validate all original input/output hashes and identities, not just poses.
        first, _ = load_pose_sensitivity(args.pair, args.capture, native, "first_published")
        report = json.loads((args.pair / "shadow_pair_audit.json").read_text())
        timeline = PoseTimeline(report["session"], report["epoch"])
        trajectory = args.pair / "trajectory.jsonl"
        if not trajectory.exists():
            candidates = [Path(p) for p in report["input_sha256"] if p.endswith("/trajectory.jsonl")]
            if len(candidates) != 1:
                raise ValueError("ambiguous live trajectory provenance")
            trajectory = candidates[0]
        if sha(trajectory) != report["input_sha256"].get(str(trajectory)):
            raise ValueError("unqualified trajectory")
        for line in trajectory.read_text().splitlines():
            apply_record(timeline, json.loads(line))
        recording_end_ns = None
        export_path = trajectory.with_name("shadow_trajectory.json")
        if export_path.exists() and str(export_path) in report["input_sha256"]:
            if sha(export_path) != report["input_sha256"][str(export_path)]:
                raise ValueError("unqualified recording horizon")
            recording_end_ns = json.loads(export_path.read_text()).get("recording_end_clock_ns")
        acquisitions = [round(f["observed_stamp"] * 1e9) for f in native]
        coverage = deadline_coverage(timeline, acquisitions, recording_end_ns)
        coverage["recording_end_clock_ns"] = recording_end_ns
        coverage["input_sha256"] = {str(args.pair / "shadow_pair_audit.json"): sha(args.pair / "shadow_pair_audit.json"),
                                    str(trajectory): sha(trajectory)}
        with (output / "deadline_coverage.json").open("x") as stream:
            json.dump(coverage, stream, indent=2, allow_nan=False)
        rows = deadline_associations(timeline, acquisitions, recording_end_ns)
        mounts = [Path(p) for p in report["input_sha256"] if p.endswith("/shadow_collection.json")]
        if len(mounts) != 1:
            raise ValueError("ambiguous physical mount")
        mount = json.loads(mounts[0].read_text())["request"]
        lidar_base = np.linalg.inv(np.asarray(mount["base_from_lidar"]))
        truth = np.array([f["world_from_lidar"] for f in native])
        truth[:, :3, 3] -= report["fixed_home"]
        truth = truth @ lidar_base
        poses = np.array([r["map_from_lidar"] for r in rows]) @ lidar_base
        with (output / "deadline_associations.jsonl").open("x") as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False) + "\n")
        with (output / "deadline_arrays.npz").open("xb") as stream:
            np.savez_compressed(stream, estimate_base=poses, truth_base=truth,
                                acquisition_ns=[r["acquisition_ns"] for r in rows])
        with (output / Path(__file__).name).open("xb") as stream:
            stream.write(Path(__file__).read_bytes())
        result = dict(completed=True, acceptance=False, fixed_deadline_ns=DEADLINE_NS,
                      samples=len(rows), first_published=summary(first @ lidar_base, truth),
                      deadline=summary(poses, truth), fitted_alignment=False,
                      live_timing_validated=False, terrain_acceptance=False,
                      note="Original replay receipt bounds, full sequence, only revisions available by each deadline. No pose publication policy installed.",
                      input_sha256={str(args.pair / "shadow_pair_audit.json"): sha(args.pair / "shadow_pair_audit.json"),
                                    str(output / Path(__file__).name): sha(output / Path(__file__).name)},
                      output_sha256={name: sha(output / name) for name in
                                     ("deadline_associations.jsonl", "deadline_arrays.npz")})
        with (output / "deadline_report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(result, indent=2))
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc)), stream)
        raise


if __name__ == "__main__":
    main()
