#!/usr/bin/env python3
"""Archive a fixed-clock IMU/frame diagnostic from an already qualified bag.

No simulator, estimator, fitted delay/axis/bias, input edits or terrain promotion.
Requires the sourced ROS overlay. Pure numerical contracts live in
test_imu_consistency.py; the existing pair audit anchors recording provenance.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import allocate_run, check_run
from export_shadow_trajectory import pose_matrix, stamp_ns
from imu_consistency import (consistency, interpolate_poses, residual_summary, orientation_clock_hypotheses,
                             paired_consistency, coincident_samples)
from revision_evidence import validated_transform
from shadow_pose_replay import load_pose_sensitivity, sha


def read(path):
    return json.loads(path.read_text())


def recorded_samples(bag, compare_physics=False):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rosidl_runtime_py.utilities import get_message

    topics = ["/gt/base_link_pose", "/astrobot_0/imu", "/astrobot_0/slam_raw/revisions",
              "/astrobot_0/slam_raw/lidar_pose_corrected", "/astrobot_0/lidar_0/pointcloud/filtered"]
    if compare_physics:
        topics += ["/astrobot_0/imu_physics", "/astrobot_0/imu_physics/clock"]
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id="mcap"), ConverterOptions("", ""))
    reader.set_filter(StorageFilter(topics=topics))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    gt, imu, clouds, published, revisions, identities, kinds = [], [], [], [], {}, set(), {}
    physics, clocks = [], []
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic.endswith("revisions"):
            if msg.header.frame_id != "map":
                raise ValueError("revision frame differs from map")
            identities.add((msg.session_id, msg.localization_epoch))
            kinds[str(msg.kind)] = kinds.get(str(msg.kind), 0) + 1
            for frame in msg.frames:
                revisions.setdefault(stamp_ns(frame.measurement_stamp), []).append(
                    pose_matrix(frame.map_from_imu) @ pose_matrix(frame.imu_from_lidar))
        elif topic.endswith("base_link_pose"):
            gt.append((stamp_ns(msg.header.stamp), pose_matrix(msg.pose), msg.header.frame_id))
        elif topic.endswith("imu_physics/clock"):
            clocks.append(json.loads(msg.data))
        elif topic.endswith("/imu_physics"):
            if msg.orientation_covariance[0] != -1:
                raise ValueError("physics IMU claims unqualified orientation")
            physics.append((stamp_ns(msg.header.stamp),
                            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], msg.header.frame_id))
        elif topic.endswith("/imu"):
            q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            if not np.isfinite(q).all() or abs(q @ q - 1.) > 1e-6 or msg.orientation_covariance[0] < 0:
                raise ValueError("recording lacks valid IMU orientation diagnostics")
            imu.append((stamp_ns(msg.header.stamp),
                        [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                        [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], q, msg.header.frame_id))
        elif topic.endswith("lidar_pose_corrected"):
            if msg.header.frame_id != "map":
                raise ValueError("published LiDAR pose frame differs from map")
            published.append((stamp_ns(msg.header.stamp), pose_matrix(msg.pose)))
        else:
            clouds.append(dict(stamp_ns=stamp_ns(msg.header.stamp), frame=msg.header.frame_id,
                               fields=[f.name for f in msg.fields], points=msg.width*msg.height))
    if not all((gt, imu, clouds, published, revisions)) or len(identities) != 1:
        raise ValueError("missing streams or mixed localization sessions")
    differences = []
    for stamp, pose in published:
        if stamp not in revisions:
            raise ValueError("published pose has no exact-stamp revision")
        differences.append(min(float(np.max(abs(pose - other))) for other in revisions[stamp]))
    if max(differences) > 1e-8:
        raise ValueError("published LiDAR pose differs from every matching revision")
    return gt, imu, clouds, dict(published_poses=len(published), revision_kinds=kinds,
        session_epoch=list(next(iter(identities))), maximum_published_revision_matrix_error=max(differences)), physics, clocks


def validate_physics_clock(stamps, clocks, start, end):
    if not clocks or any("terminal_fault" in c for c in clocks):
        raise ValueError("missing/faulted physics clock diagnostics")
    selected = [c for c in clocks if start <= round(c["monotonic_time"]*1e9) <= end]
    if not selected or any(c.get("valid") is not True for c in selected):
        raise ValueError("invalid physics sample clock")
    times = [round(c["monotonic_time"]*1e9) for c in selected]
    if times != list(stamps) or len(set(times)) != len(times):
        raise ValueError("physics samples and clock diagnostics must match one-to-one")
    if len({c["epoch"] for c in selected}) != 1:
        raise ValueError("physics clock reset inside acquisition")
    for c in selected:
        values = [c[k] for k in ("monotonic_time", "world_time", "sensor_time", "physics_dt")]
        if (not np.isfinite(values).all() or min(values) < 0 or abs(c["physics_dt"]-1/60) > 1e-8
                or abs(c["world_time"]-c["sensor_time"]-c["physics_dt"]) > 1e-3):
            raise ValueError("unqualified physics clock phase")
    return dict(samples=len(selected), epoch=selected[0]["epoch"], exact_sample_clock_pairs=True)


def audit(capture, recorder, pair, output, compare_physics=False):
    for folder in (capture, recorder, pair):
        check_run(folder)
    frames = read(capture / "native_timing_audit.json")["frames"]
    # Verifies qualified identities and every frozen upstream report/pose hash.
    load_pose_sensitivity(pair, capture, frames, "first_published")
    qualified = read(pair / "shadow_pair_audit.json")
    if str(recorder / "shadow_collection.json") not in qualified["input_sha256"]:
        raise ValueError("recorder does not belong to qualified pair")
    export_paths = [Path(p) for p in qualified["input_sha256"] if p.endswith("/shadow_trajectory.json")]
    if len(export_paths) != 1:
        raise ValueError("ambiguous trajectory provenance")
    exported = read(export_paths[0])
    if Path(exported["source"]) != recorder:
        raise ValueError("trajectory belongs to another recording")
    source_hashes = {}
    for relative, expected in exported["input_sha256"].items():
        path = recorder / relative
        actual = sha(path)
        if actual != expected:
            raise ValueError("recorded bag changed after qualification")
        source_hashes[str(path)] = actual
    collection = read(recorder / "shadow_collection.json")
    for name, expected in collection["config_files"].items():
        path = recorder / "config" / name
        if sha(path) != expected:
            raise ValueError("recorded configuration changed")
        source_hashes[str(path)] = expected
    config = recorder / "config"
    selection = read(config / "config.json")["global"]
    settings = read(config / selection["config_ros"])["glim_ros"]
    sensors = read(config / selection["config_sensors"])["sensors"]
    if compare_physics and settings["imu_topic"] not in ("imu", "/astrobot_0/imu"):
        raise ValueError("paired input experiment requires unchanged legacy GLIM IMU topic")
    if settings["imu_time_offset"] != 0 or settings["points_time_offset"] != 0 or settings["acc_scale"] != 1:
        raise ValueError("this diagnostic requires explicit zero time offsets and SI acceleration")
    gt, imu, clouds, wiring, physics_samples, physics_clocks = recorded_samples(recorder / "bag", compare_physics)
    if wiring["session_epoch"] != [qualified["session"], qualified["epoch"]]:
        raise ValueError("bag identity differs from qualified pair")
    if {x[2] for x in gt} != {"gt_map"} or {x[4] for x in imu} != {settings["imu_frame_id"]}:
        raise ValueError("unexpected GT or IMU frame")
    if {c["frame"] for c in clouds} != {settings["lidar_frame_id"]}:
        raise ValueError("unexpected input cloud frame")
    # Fixed acquisition interval, with GT bracketing margin. No error-based trim.
    start = int(np.floor(frames[0]["observed_stamp"]*1e9))
    end = int(np.ceil(frames[-1]["observed_stamp"]*1e9))
    gt = [x for x in gt if start-400_000_000 <= x[0] <= end+400_000_000]
    imu = [x for x in imu if start <= x[0] <= end]
    clouds = [c for c in clouds if start <= c["stamp_ns"] <= end]
    if not imu or not clouds:
        raise ValueError("missing acquisition-interval sensor samples")
    gt_times = np.array([x[0] for x in gt], dtype=np.int64)*1e-9
    gt_base = np.array([x[1] for x in gt])
    request = collection["request"]
    base_imu = validated_transform(request["base_from_imu"])
    base_lidar = validated_transform(request["base_from_lidar"])
    times = np.array([x[0] for x in imu], dtype=np.int64)*1e-9
    # Interpolate the BASE pose first, THEN apply the rotating sensor lever arm.
    imu_poses = interpolate_poses(gt_times, gt_base, times) @ base_imu
    acceleration, gyro, orientation = (np.array([x[k] for x in imu]) for k in (1, 2, 3))
    diagnostics = consistency(times, imu_poses, times, acceleration, gyro,
                              gravity=float(sensors["imu_gravity_magnitude"]))
    native_times = np.array([f["observed_stamp"] for f in frames])
    native_expected = interpolate_poses(gt_times, gt_base, native_times) @ base_lidar
    native_actual = np.array([f["world_from_lidar"] for f in frames])
    native_position = native_expected[:, :3, 3]-native_actual[:, :3, 3]
    native_rotation = (Rotation.from_matrix(native_expected[:, :3, :3]).inv() *
                       Rotation.from_matrix(native_actual[:, :3, :3])).as_rotvec()
    orientation_error = (Rotation.from_matrix(imu_poses[:, :3, :3]).inv() *
                         Rotation.from_quat(orientation)).as_rotvec()
    physics = {f["world_physics_dt"] for f in frames}
    rendering = {f["world_rendering_dt"] for f in frames}
    if len(physics) != 1 or len(rendering) != 1:
        raise ValueError("simulator clock periods changed")
    hypotheses = orientation_clock_hypotheses(gt_times, gt_base, base_imu, times, orientation,
        physics_dt=next(iter(physics)), rendering_dt=next(iter(rendering)))
    report = dict(schema=1, diagnostic_completed=True, acceptance=False, capture=str(capture),
        recorder=str(recorder), qualified_pair=str(pair), fixed_acquisition_ns=[start, end],
        fitted_alignment=False, fitted_time_offset=False, gt_used_by_estimator=False,
        imu_samples=len(times), imu_dt_p50_p95_max_s=np.percentile(np.diff(times), [50, 95, 100]).tolist(),
        physics_dt_s=sorted({f["world_physics_dt"] for f in frames}),
        rendering_dt_s=sorted({f["world_rendering_dt"] for f in frames}),
        acceleration_half_window_s=.2, acceleration_windows=len(diagnostics["centers"]),
        acceleration_edge_samples_without_full_window=len(times)-len(diagnostics["centers"]),
        imu_orientation_error_rad=residual_summary(orientation_error),
        orientation_clock_hypotheses=hypotheses, clock_hypothesis_applied_to_inputs=False,
        gyro_increment_error_rad_s=residual_summary(diagnostics["gyro_error_rad_s"]),
        acceleration_consistency_error_m_s2=residual_summary(diagnostics["acceleration_error_m_s2"]),
        acceleration_reference_m_s2=residual_summary(diagnostics["acceleration_reference_m_s2"]),
        acceleration_predicted_m_s2=residual_summary(diagnostics["acceleration_predicted_m_s2"]),
        native_gt_position_error_m=residual_summary(native_position),
        native_gt_rotation_error_rad=residual_summary(native_rotation), wiring=wiring,
        cloud_samples=len(clouds), cloud_fields=sorted({tuple(c["fields"]) for c in clouds}),
        cloud_dt_p50_p95_max_s=np.percentile(np.diff([c["stamp_ns"] for c in clouds])*1e-9, [50, 95, 100]).tolist(),
        configured_global_shutter=sensors["global_shutter_lidar"],
        note="No timing/axis/scale fit. IMU orientation is diagnostic only; GLIM uses acceleration/gyro. "
             "Finite cadence and GT interpolation contribute residuals. No calibrated IMU-error gate or terrain acceptance.")
    if compare_physics:
        physics_samples = [x for x in physics_samples if start <= x[0] <= end]
        if not physics_samples or {x[3] for x in physics_samples} != {settings["imu_frame_id"]}:
            raise ValueError("missing or wrong-frame physics IMU")
        clock_report = validate_physics_clock([x[0] for x in physics_samples], physics_clocks, start, end)
        stream_data = {"legacy": (times, acceleration, gyro), "physics": (
            np.array([x[0] for x in physics_samples], dtype=np.int64)*1e-9,
            np.array([x[1] for x in physics_samples]), np.array([x[2] for x in physics_samples]))}
        physics_ids, serialization_delta = coincident_samples([x[0] for x in imu], [x[0] for x in physics_samples])
        paired = paired_consistency(gt_times, gt_base, base_imu, stream_data,
                                    gravity=float(sensors["imu_gravity_magnitude"]))
        paired["shared_legacy_stamp_ns"] = np.array([x[0] for x in imu], dtype=np.int64)
        paired["shared_serialization_delta_ns"] = serialization_delta
        paired["shared_acceleration_difference_m_s2"] = acceleration-stream_data["physics"][1][physics_ids]
        paired["shared_gyro_difference_rad_s"] = gyro-stream_data["physics"][2][physics_ids]
        with (output / "paired_imu_residuals.npz").open("xb") as stream:
            np.savez_compressed(stream, **paired)
        report["paired_comparison"] = dict(physics_clock=clock_report, acceleration_windows=len(paired["centers"]),
            gyro_windows=len(paired["gyro_endpoints"])-1, common_window_centers=True,
            physics_orientation_used=False, input_samples={name: len(data[0]) for name, data in stream_data.items()},
            estimator_imu_topic=settings["imu_topic"],
            physics_readiness_required=collection.get("physics_imu_required", False),
            exact_shared_samples=int(np.count_nonzero(serialization_delta == 0)),
            coincident_samples=len(serialization_delta), serialization_tolerance_ns=1,
            serialization_delta_ns_counts={str(int(k)): int(v) for k, v in zip(*np.unique(serialization_delta, return_counts=True))},
            kinematic_timestamps_adjusted=False,
            gyro_edge_intervals_without_common_bracket=len(times)-len(paired["gyro_endpoints"]),
            shared_sample_differences={
                name: residual_summary(value) for name, value in paired.items() if "_difference_" in name},
            first_center_s=float(paired["centers"][0]), last_center_s=float(paired["centers"][-1]),
            summaries={name: residual_summary(value) for name, value in paired.items() if "_error_" in name},
            residual_sha256=sha(output / "paired_imu_residuals.npz"),
            note="Same GT windows and gyro endpoints, all measured ticks integrated; no fitted correction. "
                 "GT interpolation cannot resolve motion above the recorded GT sampling rate.")
    for path in (pair / "shadow_pair_audit.json", capture / "native_timing_audit.json",
                 recorder / "shadow_collection.json", Path(__file__), Path(__file__).with_name("imu_consistency.py")):
        source_hashes[str(path)] = sha(path)
    check_run(output)
    with (output / "imu_residuals.npz").open("xb") as stream:
        np.savez_compressed(stream, **diagnostics, imu_times=times, orientation_error_rad=orientation_error,
                            native_times=native_times, native_position_error_m=native_position,
                            native_rotation_error_rad=native_rotation)
    report.update(input_sha256=source_hashes, residual_sha256=sha(output / "imu_residuals.npz"))
    with (output / "imu_audit.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "recorder", "pair"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--compare-physics", action="store_true", help="Require and compare complete 60 Hz shadow IMU")
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        report = audit(args.capture, args.recorder, args.pair, output, args.compare_physics)
        print(json.dumps({k: v for k, v in report.items() if k != "input_sha256"}, indent=2))
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), type=type(exc).__name__), stream, indent=2)
        raise


if __name__ == "__main__":
    main()
