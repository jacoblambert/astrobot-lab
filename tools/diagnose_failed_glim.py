#!/usr/bin/env python3
"""Explicit failed-prefix pose diagnosis; never exports qualified replay inputs."""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from export_shadow_trajectory import apply_record, bag_events
from glim_gt_start import recorded_gt
from measure_live_glim_accuracy import statistics
from natural_holdout import validate_gt_snapshots
from shadow_pose_replay import sha
from shadow_terrain import PoseTimeline, clock_bound_events


def pair_failed_prefix(timeline, frames, request, gt_ns, gt_poses):
    mount = np.array(request["base_from_lidar"])
    expected = np.linalg.inv(request["base_from_imu"]) @ mount
    if not np.allclose(timeline.extrinsic, expected, atol=1e-4, rtol=0):
        raise ValueError("failed-prefix sensor/estimator frame mismatch")
    rows, paired, truth, estimates = [], [], [], {m: [] for m in ("causal", "revised")}
    for index, frame in enumerate(frames):
        stamp = round(frame["observed_stamp"] * 1e9)
        k = np.searchsorted(gt_ns, stamp)
        candidates = [j for j in (k - 1, k) if 0 <= j < len(gt_ns) and abs(int(gt_ns[j]) - stamp) <= 2]
        row = dict(chunk=index, acquisition_ns=stamp, missing=[])
        if len(candidates) != 1:
            row["missing"].append("exact_gt_snapshot")
        else:
            validate_gt_snapshots([frame], gt_ns, gt_poses, mount)
        associations = {}
        for mode in estimates:
            try:
                associations[mode] = timeline.associate(stamp, timeline.last_arrival_ns, mode=mode)
                row[mode] = associations[mode].record()
            except ValueError as exc:
                row["missing"].append(mode + ": " + str(exc))
        if not row["missing"]:
            reference = gt_poses[candidates[0]].copy()
            reference[:3, 3] -= request["home_xyz"]
            truth.append(reference)
            paired.append(index)
            for mode in estimates:
                estimates[mode].append(associations[mode].map_from_lidar @ np.linalg.inv(mount))
        rows.append(row)
    return rows, paired, truth, estimates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "recorder"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    # Small derived reports, not a new raw capture. Keep the same external-drive
    # and 50 GiB reserve checks, with a stricter 1 GiB output quota.
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        for folder in (args.capture, args.recorder):
            check_run(folder)
        collection = json.loads((args.recorder / "collection.json").read_text())
        failure = json.loads((args.capture / "failure.json").read_text())
        if collection.get("collection_completed") or not collection.get("failure"):
            raise ValueError("tool is only for explicitly failed recordings")
        link = json.loads((args.recorder / "shadow_collection.json").read_text())
        native_link = json.loads((args.capture / "shadow_link.json").read_text())
        request = native_link["request"]
        if (
            request != {k: v for k, v in link["request"].items() if k != "stamp"}
            or request["capture"] != args.capture.name
            or native_link["first_grant"]["recorder"] != args.recorder.name
            or request["home_xyz"] != [-20.0, -20.0, 0.0]
        ):
            raise ValueError("failed capture identity/frame mismatch")
        if {p.name: sha(p) for p in (args.recorder / "config").glob("*.json")} != link["config_files"]:
            raise ValueError("recorded configuration changed")
        timeline = PoseTimeline(*link["identity"])
        unbounded_tail = None
        try:
            for record, arrival, receipt in clock_bound_events(bag_events(args.recorder / "bag")):
                apply_record(timeline, dict(record, arrival_ns=arrival, bag_receipt_ns=receipt))
        except ValueError as exc:
            if str(exc) != "missing tail clock: revision availability unbounded":
                raise
            unbounded_tail = str(exc)  # Never use any unbounded pending revision.
        native = json.loads((args.capture / "native_timing_audit.json").read_text())
        gt_ns, gt_poses = recorded_gt(args.recorder)
        rows, paired, truth, estimates = pair_failed_prefix(timeline, native["frames"], request, gt_ns, gt_poses)
        with (output / "failed_associations.jsonl").open("x") as stream:
            for row in rows:
                stream.write(json.dumps(row, allow_nan=False) + "\n")
        results = {mode: statistics(poses, truth) for mode, poses in estimates.items()} if paired else {}
        with (output / "failed_prefix_arrays.npz").open("xb") as stream:
            np.savez_compressed(stream, native_indices=paired, gt_base=truth, **estimates)
        sources = [args.capture / n for n in ("failure.json", "native_timing_audit.json", "shadow_link.json")]
        sources += [args.recorder / n for n in ("collection.json", "shadow_collection.json", "bag/metadata.yaml")]
        sources += sorted((args.recorder / "bag").glob("*.mcap"))
        report = dict(
            completed_diagnostic=True,
            acceptance=False,
            source_collection_completed=False,
            source_failure=failure,
            expected_recorded_native_ticks=len(rows),
            paired_ticks=len(paired),
            missing=[dict(chunk=r["chunk"], reasons=r["missing"]) for r in rows if r["missing"]],
            unbounded_revision_tail=unbounded_tail,
            results=results,
            executed_native_duration_s=native["frames"][-1]["observed_stamp"] - native["frames"][0]["observed_stamp"],
            paired_path_length_m=(
                float(np.linalg.norm(np.diff(np.asarray(truth)[:, :3, 3], axis=0), axis=1).sum())
                if len(truth) > 1
                else None
            ),
            note="FAILED PREFIX ONLY. Missing associations and unexecuted route remain failures; no whole-route stability claim.",
            input_sha256={str(p): sha(p) for p in sources},
            source_sha256=sha(Path(__file__)),
            output_sha256={n: sha(output / n) for n in ("failed_associations.jsonl", "failed_prefix_arrays.npz")},
        )
        with (output / "failed_prefix.json").open("x") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
        check_run(output)
        print(json.dumps(report, indent=2))
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
