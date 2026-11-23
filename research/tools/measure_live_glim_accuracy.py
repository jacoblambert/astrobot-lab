#!/usr/bin/env python3
"""Diagnostic pose accuracy of a linked live shadow run, including failed prefixes.

Never qualifies failed captures. GT is evaluator-only; fixed home, no fitted
alignment/time offset, no extrapolation, explicit missing-association counts.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import allocate_run, check_run
from diagnose_glim_accuracy import summary
from export_shadow_trajectory import apply_record, pose_matrix, stamp_ns
from live_terrain_shadow import sha_file
from natural_holdout import validate_gt_snapshots
from replay_live_terrain_shadow import verify_runtime_sources
from replay_native_terrain import validate_native_source
from revision_evidence import validated_transform
from shadow_terrain import PoseTimeline


def base_reference(world_from_lidar, base_from_lidar, home):
    home = np.asarray(home, dtype=float)
    if home.shape != (3,) or not np.isfinite(home).all():
        raise ValueError("invalid declared home")
    pose = validated_transform(world_from_lidar).copy()
    pose[:3, 3] -= home
    return pose @ np.linalg.inv(validated_transform(base_from_lidar))


def statistics(estimated, truth):
    result = summary(np.asarray(estimated), np.asarray(truth))
    xyz = np.asarray(estimated)[:, :3, 3] - np.asarray(truth)[:, :3, 3]
    angles = Rotation.from_matrix(np.swapaxes(np.asarray(truth)[:, :3, :3], 1, 2) @
                                  np.asarray(estimated)[:, :3, :3]).magnitude()
    result["position_3d_rmse_m"] = float(np.sqrt(np.mean(np.sum(xyz * xyz, axis=1))))
    result["rotation_geodesic_deg_p50_p95_max"] = np.rad2deg(np.percentile(angles, [50, 95, 100])).tolist()
    # Existing pose targets are diagnostic, never whole-route acceptance.
    result["diagnostic_pose_targets_pass"] = result.pop("targets_pass")
    return result


def validate_gt_frame(frame):
    # Matches glim_gt_start: this stack declares simulator world as gt_map,
    # distinct from GLIM's translated map frame.
    if frame != "gt_map":
        raise ValueError("unexpected simulator GT world frame")


def acquisition_seconds(frames):
    return np.array([round(f["observed_stamp"] * 1e9) / 1e9 for f in frames])


def published_gt_at_native_ticks(frames, times, poses, mount):
    # Existing native/GT contract allows only the 2 ns round-versus-truncate
    # representation difference. Select the actual same-tick snapshot; never
    # interpolate/extrapolate a missing final tick after an interrupted run.
    stamps = np.rint(np.asarray(times)*1e9).astype(np.int64)
    report = validate_gt_snapshots(frames, stamps, poses, mount)
    indices = []
    for frame in frames:
        stamp = round(frame["observed_stamp"]*1e9)
        k = np.searchsorted(stamps, stamp)
        indices.append(next(j for j in (k-1, k) if 0 <= j < len(stamps) and abs(int(stamps[j])-stamp) <= 2))
    return np.asarray(poses)[indices].copy(), report


def validate_projection_tail(collection, verified, projected):
    if verified != projected:
        raise ValueError("not every projected input pose was verified")
    tail = collection["status"]["integrated"] - projected
    if tail < 0 or (collection["experiment_completed"] and tail):
        raise ValueError("invalid completed integration/projection counts")
    # SIGINT can interrupt a callback after integration but before projection.
    # Such poses were never published as a map. Keep this gap explicit in a
    # FAILED/INCOMPLETE-run diagnostic, never claim they were reproduced.
    return tail


def gt_from_bag(bag):
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id="mcap"), ConverterOptions("", ""))
    times, poses = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/gt/base_link_pose":
            message = deserialize_message(data, PoseStamped)
            validate_gt_frame(message.header.frame_id)
            # gt_map_ros2._publish_gt_pose publishes WORLD coordinates, not the
            # home-translated SLAM map. Independently verify against USD below.
            times.append(stamp_ns(message.header.stamp) / 1e9)
            poses.append(pose_matrix(message.pose))
    return np.asarray(times), np.asarray(poses)


def timeline_from_events(path):
    timeline, pending, previous_clock, previous_wall = None, [], -1, -1
    terminal = None
    with path.open() as stream:
        for line in stream:
            event = json.loads(line)
            if terminal is not None or event["receipt_wall_ns"] < previous_wall:
                raise ValueError("events after fault or regressing callback receipt")
            previous_wall = event["receipt_wall_ns"]
            kind, payload = event["kind"], event["payload"]
            if kind == "revision":
                if payload["kind"] != 1:
                    raise ValueError("non-ODOM revision is not qualified for this diagnostic")
                pending.append(payload)
            elif kind == "clock":
                if payload < previous_clock or (previous_clock >= 0 and payload-previous_clock > 200_000_000):
                    raise ValueError("clock reset/gap: split diagnostic at barrier")
                previous_clock = payload
                for record in pending:
                    if timeline is None:
                        if record["revision"] != 1 or record["snapshot"]:
                            raise ValueError("missing first revision")
                        timeline = PoseTimeline(record["session"], record["epoch"])
                    apply_record(timeline, dict(record, arrival_ns=payload))
                pending.clear()
            elif kind == "fault":
                terminal = payload
            elif kind not in ("chunk", "complete"):
                raise ValueError("unknown live event")
    if timeline is None:
        raise ValueError("no bounded GLIM poses")
    return timeline, previous_clock, len(pending), terminal


def measure(capture, recorder, mapper, output):
    for folder in (capture, recorder, mapper):
        check_run(folder)
    read = lambda p: json.loads(p.read_text())
    cap_link = read(capture / "shadow_link.json")
    link = read(recorder / "shadow_collection.json")
    request = cap_link["request"]
    if (request != {k: v for k, v in link["request"].items() if k != "stamp"} or
            request["capture"] != capture.name or cap_link["first_grant"]["recorder"] != recorder.name):
        raise ValueError("capture/recorder identity or geometry mismatch")
    manifest, collection = read(mapper / "manifest.json"), read(mapper / "collection.json")
    if (collection["status"]["capture"] != capture.name or
            [collection["status"]["session"], collection["status"]["epoch"]] != link["identity"]):
        raise ValueError("mapper session/capture mismatch")
    verify_runtime_sources(manifest)
    if set(collection["output_sha256"]) != {"events.jsonl", "native.bin", "projections.jsonl"}:
        raise ValueError("incomplete mapper provenance")
    for name, expected in collection["output_sha256"].items():
        if sha_file(mapper / name) != expected:
            raise ValueError("mapper output changed")
    paths = [capture / n for n in ("shadow_link.json", "native_timing_audit.json", "sensor_audit.json")]
    paths += [recorder / n for n in ("shadow_collection.json", "collection.json", "bag/metadata.yaml")]
    paths += list(sorted((recorder / "bag").glob("*.mcap")))
    paths += [mapper / n for n in ("manifest.json", "collection.json", "events.jsonl", "projections.jsonl")]
    paths += [Path(__file__), Path(__file__).with_name("diagnose_glim_accuracy.py"),
              Path(__file__).with_name("natural_holdout.py"), Path(__file__).with_name("export_shadow_trajectory.py")]
    hashes = {str(p): sha_file(p) for p in paths}
    native = read(capture / "native_timing_audit.json")
    native_audit = validate_native_source(native)
    sensor = read(capture / "sensor_audit.json")
    if not sensor["passed"] or not sensor["extrinsic_tf_checked"]:
        raise ValueError("GT/sensor mount audit absent")
    frames = native["frames"]
    # Native archives use float seconds; ROS serializes the same acquisition
    # clock to integer nanoseconds. Use that exact representation, not a fitted
    # time offset or epsilon extrapolation at the last recorded tick.
    times = acquisition_seconds(frames)
    truth = np.array([base_reference(f["world_from_lidar"], request["base_from_lidar"], request["home_xyz"])
                      for f in frames])
    gt_times, world_base = gt_from_bag(recorder / "bag")
    published_gt, gt_tick_audit = published_gt_at_native_ticks(frames, gt_times, world_base, request["base_from_lidar"])
    published_gt[:, :3, 3] -= request["home_xyz"]
    gt_check = statistics(published_gt, truth)
    if (np.max(abs(published_gt[:, :3, 3] - truth[:, :3, 3])) > .001 or
            gt_check["rotation_geodesic_deg_p50_p95_max"][-1] > .0573):
        raise ValueError("published GT and native acquisition GT disagree")
    timeline, last_clock, pending, terminal = timeline_from_events(mapper / "events.jsonl")
    if (timeline.session, timeline.epoch) != tuple(link["identity"]):
        raise ValueError("revision timeline identity mismatch")
    expected_extrinsic = np.linalg.inv(validated_transform(request["base_from_imu"])) @ validated_transform(request["base_from_lidar"])
    if not np.allclose(timeline.extrinsic, expected_extrinsic, atol=1e-4, rtol=0):
        raise ValueError("GLIM IMU/LiDAR and simulator extrinsics disagree")
    used, projected_inputs = {}, 0
    with (mapper / "projections.jsonl").open() as stream:
        for line in stream:
            projection = json.loads(line)
            for association in projection["associations"]:
                if association["acquisition_ns"] in used:
                    raise ValueError("duplicate integrated acquisition")
                used[association["acquisition_ns"]] = association
            projected_inputs = projection["status"]["integrated"]
            if len(used) != projected_inputs:
                raise ValueError("projection association prefix incomplete")
    rows, estimated, references, matched = [], {m: [] for m in ("first_published", "final_revised")}, {}, set()
    arrays = {"times": times, "gt_base": truth}
    lidar_base = np.linalg.inv(validated_transform(request["base_from_lidar"]))
    for index, (frame, reference) in enumerate(zip(frames, truth)):
        stamp = round(frame["observed_stamp"] * 1e9)
        row = dict(chunk=index, stamp_ns=stamp, integrated=stamp in used)
        for name, mode in (("first_published", "causal"), ("final_revised", "revised")):
            deadline = min(last_clock, stamp + 200_000_000) if mode == "causal" else last_clock
            try:
                association = timeline.associate(stamp, deadline, mode=mode)
            except ValueError as exc:
                row[name] = dict(missing=str(exc))
                continue
            if mode == "causal" and stamp in used:
                if dict(sequence=index, **association.record()) != used[stamp]:
                    raise ValueError("diagnostic association differs from actually integrated pose")
                matched.add(stamp)
            pose = association.map_from_lidar @ lidar_base
            estimated[name].append(pose)
            references.setdefault(name, []).append((index, reference))
            row[name] = dict(association=association.record(), map_from_base=pose.tolist())
        rows.append(row)
    unpublished_tail = validate_projection_tail(collection, len(matched), projected_inputs)
    results = {}
    for name, poses in estimated.items():
        if not poses:
            raise ValueError("no associated poses for " + name)
        indices = np.array([i for i, _ in references[name]])
        gt = np.array([p for _, p in references[name]])
        poses = np.array(poses)
        arrays[name], arrays[name + "_indices"] = poses, indices
        first, last = times[indices] <= times[0]+3, times[indices] >= times[-1]-3
        results[name] = dict(overall=statistics(poses, gt), missing=len(frames)-len(poses),
                            first_3s=statistics(poses[first], gt[first]) if first.any() else None,
                            last_3s=statistics(poses[last], gt[last]) if last.any() else None)
    with (output / "poses.jsonl").open("x") as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    with (output / "accuracy_arrays.npz").open("xb") as stream:
        np.savez_compressed(stream, **arrays)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for name in estimated:
        indices = arrays[name + "_indices"]
        error = arrays[name][:, :3, 3] - truth[indices, :3, 3]
        angles = Rotation.from_matrix(np.swapaxes(truth[indices, :3, :3], 1, 2) @
                                      arrays[name][:, :3, :3]).magnitude()
        for ax, values in zip(axes, (np.linalg.norm(error[:, :2], axis=1), error[:, 2], np.rad2deg(angles))):
            ax.plot(times[indices]-times[0], values, label=name)
    for ax, label in zip(axes, ("Horizontal error (m)", "Signed height error (m)", "Rotation error (degrees)")):
        ax.set_ylabel(label); ax.grid(); ax.legend()
    axes[-1].set_xlabel("Seconds since first native acquisition")
    fig.suptitle("Fixed-frame diagnostic — source completed: " + str(collection["experiment_completed"]))
    fig.tight_layout(); fig.savefig(output / "accuracy.png"); plt.close(fig)
    for path, expected in hashes.items():
        if sha_file(path) != expected:
            raise ValueError("input changed during measurement: " + path)
    return dict(diagnostic_completed=True, acceptance=False, failed_sources_retained=True,
        frame="same physical base_link; world minus declared home; no fitted alignment or time shift",
        first_published_deadline_s=.2, final_revised_retrospective=True,
        source_mapper_completed=collection["experiment_completed"], source_mapper_failure=collection["status"]["failure"],
        source_recorder=read(recorder / "collection.json"), terminal_event=terminal,
        native_chunks=len(frames), integrated_poses_verified=len(matched),
        unprojected_shutdown_integrations=unpublished_tail, unbounded_tail_revisions=pending,
        duration_s=float(times[-1]-times[0]), gt_path_length_m=float(np.linalg.norm(np.diff(truth[:, :3, 3], axis=0), axis=1).sum()),
        gt_publication_vs_native=gt_check, gt_tick_audit=gt_tick_audit, native_geometry_audit=native_audit, results=results,
        input_sha256=hashes, mapper_output_sha256=collection["output_sha256"],
        output_sha256={n: sha_file(output / n) for n in ("poses.jsonl", "accuracy_arrays.npz", "accuracy.png")})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "recorder", "mapper"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = measure(args.capture, args.recorder, args.mapper, output)
        with (output / "accuracy_report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(result["results"], indent=2), flush=True)
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
