#!/usr/bin/env python3
"""Pose-only diagnostic for a completed recording; never terrain qualification.

Uses every archived acquisition and its exact published GT snapshot. A failed
surface audit remains reported as failed, independently of measured pose error.
No failed capture, missing pose bracket, fitted alignment or GT interpolation.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from audit_shadow_pair import compare_pose_sequence
from diagnose_glim_accuracy import relative_summary
from export_shadow_trajectory import apply_record
from glim_gt_start import recorded_gt
from measure_live_glim_accuracy import published_gt_at_native_ticks, statistics
from revision_evidence import validated_transform
from selected_native_audit import audit_transport, validate_native
from shadow_pose_replay import sha, validate_pose_rows
from shadow_terrain import PoseTimeline


def evaluate_poses(timeline, frames, request, gt_ns, gt_poses):
    lidar_base = np.linalg.inv(validated_transform(request["base_from_lidar"]))
    expected = np.linalg.inv(validated_transform(request["base_from_imu"])) @ np.linalg.inv(lidar_base)
    if not np.allclose(timeline.extrinsic, expected, atol=1e-4, rtol=0):
        raise ValueError("recorded GLIM/simulator extrinsics disagree")
    truth, gt_audit = published_gt_at_native_ticks(
        frames, np.asarray(gt_ns) / 1e9, gt_poses, request["base_from_lidar"]
    )
    truth[:, :3, 3] -= request["home_xyz"]
    rows, _ = compare_pose_sequence(timeline, frames, request["home_xyz"])
    times = np.array([round(f["observed_stamp"] * 1e9) / 1e9 for f in frames])
    arrays = dict(times=times, gt_base=truth)
    results = {}
    for mode in ("first_published", "final_revised"):
        poses = validate_pose_rows(rows, frames, mode, timeline.session, timeline.epoch) @ lidar_base
        results[mode] = dict(overall=statistics(poses, truth))
        if times[-1] - times[0] > 1:
            relative = relative_summary(times, poses, truth)
            relative["diagnostic_pose_targets_pass"] = relative.pop("targets_pass")
            results[mode]["relative_1s"] = relative
        arrays[mode] = poses
    return results, arrays, rows, gt_audit


def measure(capture, recorder, trajectory, output):
    read = lambda path: json.loads(path.read_text())
    for folder in (capture, recorder, trajectory):
        check_run(folder)
        if any((folder / name).exists() for name in ("failure.json", "orchestration_failure.json")):
            raise ValueError("failed capture is ineligible for completed-route pose diagnostic")
    collection = read(recorder / "collection.json")
    complete = read(capture / "natural_holdout.json")
    exported = read(trajectory / "shadow_trajectory.json")
    if not collection.get("collection_completed") or collection.get("failure") or not complete.get("complete"):
        raise ValueError("incomplete capture")
    if not exported.get("passed") or Path(exported["source"]) != recorder:
        raise ValueError("invalid trajectory export")
    for name, expected in exported["input_sha256"].items():
        if Path(name).is_absolute() or ".." in Path(name).parts or sha(recorder / name) != expected:
            raise ValueError("recording changed since export")
    if sha(trajectory / "trajectory.jsonl") != exported["trajectory_sha256"]:
        raise ValueError("trajectory changed")
    link = read(recorder / "shadow_collection.json")
    simulator = read(capture / "shadow_link.json")
    request = simulator["request"]
    if (
        request != {k: v for k, v in link["request"].items() if k != "stamp"}
        or request["capture"] != capture.name
        or simulator["first_grant"]["recorder"] != recorder.name
        or complete["request"] != request
        or request["home_xyz"] != [-20.0, -20.0, 0.0]
    ):
        raise ValueError("capture/recorder frame or identity mismatch")
    if sha(capture / "sensor_audit.json") != read(capture / "manifest.json")["sensor_audit_sha256"]:
        raise ValueError("stationary extrinsic audit changed")
    sensor = read(capture / "sensor_audit.json")
    if not sensor["passed"] or not sensor["extrinsic_tf_checked"]:
        raise ValueError("stationary extrinsic audit failed")
    native = read(capture / "native_timing_audit.json")
    frames = native["frames"]
    if complete["chunks"] != len(frames) or complete["last_stamp"] != frames[-1]["observed_stamp"]:
        raise ValueError("native acquisition tail missing")
    transport = audit_transport(capture, recorder)
    # Diagnostic separation, NOT a relaxed replacement for the qualification
    # gate. Preserve its exact result and every tick in the pose denominator.
    try:
        surface = dict(passed=True, report=validate_native(native, motion=request["pattern"], profile=request["lidar_profile"]["name"]))
    except ValueError as exc:
        surface = dict(passed=False, failure=str(exc))
    timeline = PoseTimeline(*link["identity"])
    for line in (trajectory / "trajectory.jsonl").read_text().splitlines():
        apply_record(timeline, json.loads(line))
    if (timeline.session, timeline.epoch) != (exported["session"], exported["epoch"]):
        raise ValueError("export session/epoch mismatch")
    gt_ns, gt_poses = recorded_gt(recorder)
    results, arrays, rows, gt_audit = evaluate_poses(timeline, frames, request, gt_ns, gt_poses)
    with (output / "pose_arrays.npz").open("xb") as stream:
        np.savez_compressed(stream, **arrays)
    with (output / "pose_associations.jsonl").open("x") as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    sources = [
        capture / n
        for n in (
            "natural_holdout.json",
            "native_timing_audit.json",
            "manifest.json",
            "sensor_audit.json",
            "selected_sensor.json",
            "selected_windows.jsonl",
            "shadow_link.json",
        )
    ]
    sources += [recorder / n for n in ("collection.json", "shadow_collection.json", "runtime_manifest.json")]
    sources += [trajectory / n for n in ("shadow_trajectory.json", "trajectory.jsonl")]
    sources += [
        Path(__file__),
        Path(__file__).with_name("measure_live_glim_accuracy.py"),
        Path(__file__).with_name("diagnose_glim_accuracy.py"),
    ]
    return dict(
        completed=True,
        acceptance=False,
        terrain_qualification=False,
        navigation_acceptance=False,
        protocol="completed selected-sensor route; physical IMU; one-time GT seed; synthetic RTK assistance",
        frame="same physical base_link, fixed home, exact published GT ticks, no fitted alignment",
        samples=len(frames),
        duration_s=float(arrays["times"][-1] - arrays["times"][0]),
        path_length_m=float(np.linalg.norm(np.diff(arrays["gt_base"][:, :3, 3], axis=0), axis=1).sum()),
        native_surface_gate=surface,
        gt_reference=gt_audit,
        transport=transport,
        results=results,
        input_sha256={str(p): sha(p) for p in sources},
        output_sha256={n: sha(output / n) for n in ("pose_arrays.npz", "pose_associations.jsonl")},
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "recorder", "trajectory"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = measure(args.capture, args.recorder, args.trajectory, output)
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise
    with (output / "pose_diagnostic.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
