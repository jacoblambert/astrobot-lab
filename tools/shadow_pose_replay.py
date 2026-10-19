"""Frozen pose-only sensitivity inputs. NOT a causal terrain scheduling policy."""
import hashlib
import json
from pathlib import Path

import numpy as np

from revision_evidence import validated_transform


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_pose_rows(rows, frames, mode, session, epoch):
    key = {"first_published": "causal", "final_revised": "revised"}.get(mode)
    if key is None or len(rows) != len(frames):
        raise ValueError("unknown pose mode or native chunk count mismatch")
    poses = []
    for index, (row, frame) in enumerate(zip(rows, frames)):
        stamp = round(float(frame["observed_stamp"])*1e9)
        association = row[key]["association"]
        lo, hi = association["bracket_ns"]
        if (row["chunk"] != index or row["acquisition_ns"] != stamp
                or association["acquisition_ns"] != stamp or association["mode"] != key
                or (association["session"], association["epoch"]) != (session, epoch)
                or not stamp <= association["available_ns"] <= association["output_ns"]
                or not lo <= stamp <= hi or hi-lo > 200_000_000 or association["valid"] is not True):
            raise ValueError("pose association identity/time/bracket mismatch")
        poses.append(validated_transform(association["map_from_lidar"]).copy())
    return np.array(poses)


def load_pose_sensitivity(folder, capture, frames, mode):
    folder, capture = Path(folder), Path(capture)
    report_path = folder / "shadow_pair_audit.json"
    report = json.loads(report_path.read_text())
    if not report.get("passed") or (folder / "failure.json").exists() or report["fixed_home"] != [-20., -20., 0.]:
        raise ValueError("failed/misaligned pose audit is ineligible")
    if report["chunks"] != len(frames):
        raise ValueError("pose audit chunk count differs")
    paths = report["input_sha256"]
    if str(capture / "native_timing_audit.json") not in paths:
        raise ValueError("pose audit belongs to another capture")
    for path, expected in paths.items():
        if sha(Path(path)) != expected:
            raise ValueError("paired source provenance changed")
    pose_file = folder / "paired_poses.jsonl"
    if sha(pose_file) != report["paired_poses_sha256"]:
        raise ValueError("paired pose sequence changed")
    rows = [json.loads(line) for line in pose_file.read_text().splitlines()]
    poses = validate_pose_rows(rows, frames, mode, report["session"], report["epoch"])
    return poses, dict(mode=mode, audit=str(folder), audit_sha256=sha(report_path),
        poses_sha256=sha(pose_file), acquisition_stamps_and_projection_schedule_unchanged=True,
        availability_delays_applied=False, causal_map_evaluation=False, revision_rebuild_evaluated=False,
        interpretation="Offline pose-only sensitivity: same native evidence/times; never live causal or compute acceptance")
