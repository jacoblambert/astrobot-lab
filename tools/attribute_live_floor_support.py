#!/usr/bin/env python3
"""Offline GT-pose substitution on exact live grids; NOT terrain truth or runtime acceptance.

Hold native inputs, integration/projection schedule, rolling-grid origins, local
reference and evidence lifetime fixed. Only acquisition poses change. A mirror
using the recorded SLAM placements must match every original layer hash before
comparing GT. PhysX floor samples remain evaluator-only. No fitted alignment,
resampling, future-revised SLAM poses, or input age changes.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from local_terrain.rolling_mapper import RollingTerrainMapper
from local_terrain.self_filter import chassis_returns
from raw_return_filter import valid_range_mask
from replay_local_terrain import decoded_chunk, sha
from revision_evidence import validated_transform
from score_live_floor_samples import sample_floor
from selected_lidar_profile import profile_path, selected_setup

REASONS = (
    "NO_RETURNS",
    "MEASURED_GROUND",
    "INFERRED_GROUND",
    "OBSTACLE",
    "NO_GROUND_MODEL",
    "DISCONNECTED_SURFACE",
    "LAYER_CAPACITY",
    "UNCERTAIN_RAISED",
    "NEGATIVE_EDGE",
    "INSUFFICIENT_AREA",
    "CLEARANCE_OCCLUDED",
    "POSITIVE_EDGE",
    "UNSUPPORTED_EXTENT",
    "UNRESOLVED_NORMAL",
)


def isolated_pose(frame, acquisition_ns, local_from_map, home):
    if round(frame["observed_stamp"] * 10**9) != acquisition_ns:
        raise ValueError("GT acquisition stamp mismatch; no nearest-sample association")
    pose = validated_transform(frame["world_from_lidar"]).copy()
    home = np.asarray(home, dtype=float)
    if home.shape != (3,) or not np.isfinite(home).all():
        raise ValueError("invalid declared home")
    pose[:3, 3] -= home
    return validated_transform(local_from_map) @ pose


def floor_reasons(layers, record, points, home):
    pose = validated_transform(record["local_from_map"])
    local = (points - home) @ pose[:3, :3].T + pose[:3, 3]
    ij = np.floor((local[:, :2] - record["origin"]) / record["resolution"]).astype(int)
    inside = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < layers.shape[1]) & (ij[:, 1] < layers.shape[0])
    cells = layers[ij[inside, 1], ij[inside, 0]]
    codes = cells[cells[:, 0] == -1, 1]
    if not np.isin(codes, np.arange(len(REASONS))).all():
        raise ValueError("unknown classifier reason")
    return dict(outside=int((~inside).sum()), **{name: int((codes == i).sum()) for i, name in enumerate(REASONS)})


def attribute(floor_run, pose_run, output):
    floor_path, pose_path = floor_run / "report.json", pose_run / "pose_diagnostic.json"
    floor, accuracy = [json.loads(p.read_text()) for p in (floor_path, pose_path)]
    if not (
        floor["diagnostic_completed"]
        and floor["exact_replay"]["passed"]
        and accuracy["completed"]
        and accuracy["native_surface_gate"]["passed"]
    ):
        raise ValueError("completed exact replay and independently audited pose input required")
    source, capture = Path(floor["source"]), Path(floor["capture"])
    hashes = {}
    for group in (floor["input_sha256"], floor["exact_replay"]["input_sha256"], accuracy["input_sha256"]):
        for path, digest in group.items():
            if path in hashes and hashes[path] != digest:
                raise ValueError("conflicting input provenance: " + path)
            hashes[path] = digest
    hashes[str(floor_run / "floor_samples.jsonl")] = floor["output_sha256"]["floor_samples.jsonl"]
    native_path = capture / "native_timing_audit.json"
    if str(native_path) not in accuracy["input_sha256"]:
        raise ValueError("pose diagnostic belongs to another capture")
    hashes.update({str(p): sha(p) for p in (floor_path, pose_path, Path(__file__))})
    for path, digest in hashes.items():
        if sha(path) != digest:
            raise ValueError("changed attribution input: " + path)
    frames = json.loads(native_path.read_text())["frames"]
    manifest = json.loads((source / "manifest.json").read_text())
    request, config = manifest["request"], manifest["config"]
    if len(frames) != accuracy["samples"] or request["capture"] != capture.name:
        raise ValueError("native pose denominator mismatch")
    with np.load(capture / "clearance_tile.npz", allow_pickle=False) as tile:
        points = np.column_stack((tile["xy"], tile["z"]))
    home = np.asarray(request["home_xyz"])
    setup, _ = selected_setup(profile_path(request["lidar_profile"]["name"]))
    chunks = {}
    for line in (source / "events.jsonl").open():
        event = json.loads(line)
        if event["kind"] == "chunk":
            payload = event["payload"]
            if payload["sequence"] != len(chunks):
                raise ValueError("missing or duplicated native input")
            chunks[payload["sequence"]] = payload
    if len(chunks) != len(frames):
        raise ValueError("incomplete native input")
    mappers = {}
    totals = {mode: dict(queries=0, free=0, occupied=0, unknown=0, outside=0) for mode in ("slam", "gt_pose")}
    reasons = {mode: {name: 0 for name in ("outside", "invalidated_publication", *REASONS)} for mode in totals}
    last_sequence, projected = -1, 0
    last_valid = None
    try:
        with (source / "native.bin").open("rb") as raw, (output / "attribution.jsonl").open("x") as result:
            with (floor_run / "floor_samples.jsonl").open() as scores:
                for line in (source / "projections.jsonl").open():
                    record = json.loads(line)
                    expected_score = json.loads(next(scores))
                    if not mappers:
                        for mode in totals:
                            mappers[mode] = RollingTerrainMapper(
                                manifest["library"],
                                width=config["cells"],
                                height=config["cells"],
                                resolution=config["resolution"],
                                origin=record["origin"],
                                lifetime=config["evidence_lifetime_s"],
                            )
                    for mapper in mappers.values():
                        mapper.shift_origin(record["origin"])
                    integrated = 0
                    for association in record["associations"]:
                        sequence, stamp = association["sequence"], association["acquisition_ns"]
                        if sequence <= last_sequence:
                            continue
                        if sequence != last_sequence + 1:
                            raise ValueError("missing integrated input")
                        chunk = decoded_chunk(chunks[sequence], raw)
                        if chunk["stamp_ns"] != stamp or chunk["capture"] != capture.name:
                            raise ValueError("wrong acquisition identity")
                        native = chunk["points"].copy()
                        native[chassis_returns(native, setup) | (chunk["intensities"] == 0)] = 0
                        keep, _ = valid_range_mask(native, config["minimum"], config["maximum"])
                        selected = native[keep].astype(np.float64)
                        poses = dict(
                            slam=validated_transform(association["local_from_lidar"]),
                            gt_pose=isolated_pose(frames[sequence], stamp, record["local_from_map"], home),
                        )
                        for mode, mapper in mappers.items():
                            pose = poses[mode]
                            world = selected @ pose[:3, :3].T + pose[:3, 3]
                            mapper.integrate(world, stamp / 1e9, pose[:3, 3], np.arctan2(pose[1, 0], pose[0, 0]))
                        last_sequence, integrated = sequence, integrated + 1
                    if integrated != record["integrated_inputs"]:
                        raise ValueError("integration schedule differs")
                    maps = {mode: mapper.project(record["evidence_ns"] / 1e9) for mode, mapper in mappers.items()}
                    if hashlib.sha256(maps["slam"].tobytes()).hexdigest() != record["layers_sha256"]:
                        raise ValueError("mirror does not reproduce exact live layers")
                    row = dict(
                        evidence_ns=record["evidence_ns"],
                        published_valid=expected_score["published_valid"],
                        counts={},
                        unknown_reasons={},
                    )
                    for mode, layers in maps.items():
                        if not row["published_valid"]:
                            layers = layers.copy()
                            layers[:, :, 0] = -1
                            layers[:, :, 1] = 0
                        counts = sample_floor(layers, record, points, home)
                        if mode == "slam" and any(counts[k] != expected_score[k] for k in counts):
                            raise ValueError("floor exposure differs from exact-replay reference")
                        if row["published_valid"]:
                            why = dict(floor_reasons(layers, record, points, home), invalidated_publication=0)
                        else:
                            why = dict.fromkeys(reasons[mode], 0)
                            why.update(
                                outside=counts["outside"], invalidated_publication=counts["queries"] - counts["outside"]
                            )
                        for key in totals[mode]:
                            totals[mode][key] += counts[key]
                        for key in reasons[mode]:
                            reasons[mode][key] += why[key]
                        row["counts"][mode], row["unknown_reasons"][mode] = counts, why
                    if row["published_valid"]:
                        last_valid = row
                        # Overwrite only this run's in-memory candidate; persist once after completion.
                        last_maps = {mode: value.copy() for mode, value in maps.items()}
                        last_record = record
                    result.write(json.dumps(row, allow_nan=False) + "\n")
                    projected += 1
                if scores.read() or raw.read(1):
                    raise ValueError("unconsumed reference/native tail")
        if projected != floor["projections"] or last_sequence + 1 != len(frames) or last_valid is None:
            raise ValueError("missing attribution exposure")
        np.savez_compressed(
            output / "last_valid_layers.npz",
            **last_maps,
            floor_world_points=points,
            home=home,
            local_from_map=last_record["local_from_map"],
            origin=last_record["origin"],
            local_from_base=last_record["local_from_base"],
            resolution=last_record["resolution"],
        )
        for path, digest in hashes.items():
            if sha(path) != digest:
                raise ValueError("input changed during attribution: " + path)
        return dict(
            diagnostic_completed=True,
            acceptance=False,
            navigation_authority=False,
            pose_substitution_only=True,
            gt_reconstruction_is_terrain_truth=False,
            exact_slam_mirror=True,
            projections=projected,
            native_chunks=len(frames),
            overall=totals,
            unknown_reasons=reasons,
            last_valid=last_valid,
            supported_fraction={mode: values["free"] / values["queries"] for mode, values in totals.items()},
            input_sha256=hashes,
            output_sha256={n: sha(output / n) for n in ("attribution.jsonl", "last_valid_layers.npz")},
            scope=__doc__,
        )
    finally:
        for mapper in mappers.values():
            mapper.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("floor_run", type=Path)
    parser.add_argument("pose_run", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    for path in (args.floor_run, args.pose_run):
        check_run(path)
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = attribute(args.floor_run, args.pose_run, output)
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps({k: v for k, v in result.items() if k not in ("input_sha256", "output_sha256")}, indent=2))
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
