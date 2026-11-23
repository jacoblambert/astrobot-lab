#!/usr/bin/env python3
"""Qualify a linked native/GLIM capture and report fixed-anchor pose errors.

No terrain mapping or navigation acceptance. First-published estimates retain
their availability delay; final corrected estimates are explicitly retrospective.
Every native chunk counts, with no fitted alignment, trimming or extrapolation.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import allocate_run, check_run
from export_shadow_trajectory import apply_record
from replay_native_terrain import validate_native_source
from score_terrain_calibration import validated_route
from shadow_terrain import PoseTimeline, pose_error
from shadow_capture_gate import ShadowLease


def read(path):
    return json.loads(path.read_text())


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_links(capture_name, recorder_name, simulator_link, recorder_link, drive):
    request, recorded = simulator_link["request"], recorder_link["request"]
    # Only the heartbeat timestamp varies. All geometry/identity fields agree.
    if request != {k: v for k, v in recorded.items() if k != "stamp"}:
        raise ValueError("capture/recorder geometry declarations differ")
    if (request["capture"] != capture_name or recorder_link["recorder"] != recorder_name
            or request["home_xyz"] != [-20., -20., 0.]
            or recorder_link["home_xyz"] != request["home_xyz"]
            or request["pattern"] != "wide_sweep15_return" or request["pitch_deg"] != 5.):
        raise ValueError("wrong capture/recorder/route/mount/home")
    grant = simulator_link["first_grant"]
    ShadowLease(capture_name).receive(grant, grant["stamp"], 0.)
    if (grant != drive["shadow_first_grant"] or drive["failure"] is not None
            or grant["capture"] != capture_name or grant["recorder"] != recorder_name
            or [grant["session"], grant["epoch"]] != recorder_link["identity"]
            or grant["config_sha256"] != recorder_link["config_sha256"]
            or grant["scope"] != "guarded_terrain_capture" or grant["ready"] is not True):
        raise ValueError("readiness identity/grant/driver failure mismatch")
    errors = list(recorder_link["live_geometry_errors"].values())
    if not errors or not np.isfinite(errors).all() or max(errors) > 1e-4:
        raise ValueError("live geometry qualification failed")
    for command in drive["commands"]:
        if (command["linear"] or command["angular"]) and command["stamp"] < grant["stamp"]:
            raise ValueError("driver moved before readiness grant")
    return grant


def compare_pose_sequence(timeline, frames, home):
    home = np.asarray(home, dtype=float)
    if home.shape != (3,) or not np.isfinite(home).all():
        raise ValueError("invalid fixed home")
    records = []
    previous = -1
    for index, frame in enumerate(frames):
        stamp = round(float(frame["observed_stamp"]) * 1e9)
        if stamp <= previous:
            raise ValueError("nonmonotonic native acquisition")
        previous = stamp
        reference = np.asarray(frame["world_from_lidar"], dtype=float).copy()
        reference[:3, 3] -= home
        row = dict(chunk=index, acquisition_ns=stamp)
        for mode in ("causal", "revised"):
            association = timeline.associate(stamp, timeline.last_arrival_ns, mode=mode)
            row[mode] = dict(error=pose_error(association.map_from_lidar, reference),
                             association=association.record(),
                             availability_delay_s=(association.available_ns-stamp)/1e9)
            row[mode]["position_xyz_error_m"] = (association.map_from_lidar[:3, 3]-reference[:3, 3]).tolist()
            row[mode]["world_rotation_vector_rad"] = Rotation.from_matrix(
                association.map_from_lidar[:3, :3] @ reference[:3, :3].T).as_rotvec().tolist()
        records.append(row)
    if not records:
        raise ValueError("empty native sequence")
    stats = {}
    for mode in ("causal", "revised"):
        stats[mode] = {}
        for metric in ("position_m", "rotation_rad", "availability_delay_s"):
            values = [r[mode][metric] if metric == "availability_delay_s" else r[mode]["error"][metric]
                      for r in records]
            stats[mode][metric] = dict(zip(("median", "p95", "maximum"), map(float, np.percentile(values, [50, 95, 100]))))
        translation = np.array([r[mode]["position_xyz_error_m"] for r in records])
        angles = np.array([r[mode]["world_rotation_vector_rad"] for r in records])
        stats[mode]["position_xyz_signed_median_m"] = np.median(translation, axis=0).tolist()
        stats[mode]["position_xyz_absolute_p95_m"] = np.percentile(abs(translation), 95, axis=0).tolist()
        stats[mode]["horizontal_position_p95_m"] = float(np.percentile(np.linalg.norm(translation[:, :2], axis=1), 95))
        stats[mode]["world_rotation_xyz_absolute_p95_rad"] = np.percentile(abs(angles), 95, axis=0).tolist()
    return records, stats


def audit(capture, recorder, trajectory, output):
    for folder in (capture, recorder, trajectory):
        check_run(folder)
        if (folder / "failure.json").exists() or (folder / "orchestration_failure.json").exists():
            raise ValueError("failed inputs are ineligible")
    collection = read(recorder / "collection.json")
    exported = read(trajectory / "shadow_trajectory.json")
    if not collection.get("collection_completed") or collection.get("failure") or not exported.get("passed"):
        raise ValueError("incomplete collection/trajectory")
    if Path(exported["source"]).name != recorder.name:
        raise ValueError("trajectory source differs from linked recording")
    for name, expected in exported["input_sha256"].items():
        if Path(name).is_absolute() or ".." in Path(name).parts or sha(recorder / name) != expected:
            raise ValueError("recording changed after trajectory export")
    if sha(trajectory / "trajectory.jsonl") != exported["trajectory_sha256"]:
        raise ValueError("exported trajectory changed")
    route = validated_route(read(capture / "multiview.json"))
    for folder in [capture, *(capture / view["path"] for view in route)]:
        sensor = read(folder / "sensor_audit.json")
        if not sensor["passed"] or not sensor["extrinsic_tf_checked"]:
            raise ValueError("native capture lacks sensor/extrinsic audit")
    link = read(recorder / "shadow_collection.json")
    config_files = {p.name: sha(p) for p in sorted((recorder / "config").glob("*.json"))}
    if (config_files != link["config_files"]
            or hashlib.sha256(json.dumps(config_files, sort_keys=True).encode()).hexdigest() != link["config_sha256"]):
        raise ValueError("frozen estimator configuration changed")
    grant = validate_links(capture.name, recorder.name, read(capture / "shadow_link.json"), link,
                           read(capture / "drive_log.json"))
    native = read(capture / "native_timing_audit.json")
    timing = validate_native_source(native)
    frames = native["frames"]
    if (frames[0]["observed_stamp"] < grant["stamp"] or frames[0]["observed_stamp"] > route[0]["stamp"]
            or frames[-1]["observed_stamp"] < route[-1]["stamp"] - 1e-6):
        raise ValueError("native acquisition does not cover the entire granted route")
    files = sorted((capture / "native_chunks").glob("chunk_*.npz"))
    if len(files) != len(frames) or any(f.name != row["file"] or sha(f) != row["sha256"] for f, row in zip(files, frames)):
        raise ValueError("native chunk identity/hash mismatch")
    timeline = PoseTimeline(*link["identity"])
    with (trajectory / "trajectory.jsonl").open() as stream:
        for line in stream:
            apply_record(timeline, json.loads(line))
    if ([exported["session"], exported["epoch"]] != link["identity"]
            or exported["frames"] != len(timeline.frames)):
        raise ValueError("trajectory summary identity/count mismatch")
    records, stats = compare_pose_sequence(timeline, frames, link["home_xyz"])
    with (output / "paired_poses.jsonl").open("x") as stream:
        for row in records:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    return dict(passed=True, chunks=len(records), fixed_home=link["home_xyz"], errors=stats, timing=timing,
                session=timeline.session, epoch=timeline.epoch,
                paired_poses_sha256=sha(output / "paired_poses.jsonl"),
                retrospective_revisions=True, causal_map_projection_evaluated=False,
                terrain_validation="NOT_RUN", navigation_acceptance=False,
                input_sha256={str(p): sha(p) for p in (recorder / "shadow_collection.json",
                    capture / "shadow_link.json", capture / "native_timing_audit.json",
                    capture / "drive_log.json", capture / "multiview.json", recorder / "collection.json",
                    trajectory / "shadow_trajectory.json", trajectory / "trajectory.jsonl")})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "recorder", "trajectory"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = audit(args.capture, args.recorder, args.trajectory, output)
    except Exception as exc:
        result = dict(passed=False, failure=str(exc), terrain_validation="NOT_RUN")
    with (output / "shadow_pair_audit.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
