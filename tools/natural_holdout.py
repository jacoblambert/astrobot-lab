"""Declared stationary natural-scene acquisition; no motion or estimator tuning."""
import json
import hashlib
from pathlib import Path

import numpy as np

from revision_evidence import validated_transform

PATTERN = "natural_stationary"
SEED = 43
DURATION = 30.
HOME = [-20., -20., 0.]  # Existing fixed map origin, NOT fitted to the new spawn.


def native_hit_records(frame, directory):
    surface = frame["current_pose"]
    name = surface["hit_records_file"]
    if name != Path(frame["file"]).with_suffix(".hits.json").name or Path(name).name != name:
        raise ValueError("unexpected first-hit evidence path")
    payload = (directory / name).read_bytes()
    if hashlib.sha256(payload).hexdigest() != surface["hit_records_sha256"]:
        raise ValueError("first-hit evidence changed")
    records = json.loads(payload)
    if [r["point_index"] for r in records] != surface["selected_indices"]:
        raise ValueError("first-hit sample selection changed")
    return records


def validate_gt_snapshots(frames, times_ns, world_from_base, base_from_lidar):
    """Cross-check ROS GT against each audited physics acquisition, no fitting."""
    times = np.asarray(times_ns, dtype=np.int64)
    poses = np.asarray(world_from_base, dtype=float)
    mount = validated_transform(base_from_lidar)
    if len(times) < 2 or len(times) != len(poses) or not (np.diff(times) > 0).all():
        raise ValueError("invalid GT snapshot sequence")
    errors = []
    for frame in frames:
        stamp = round(frame["observed_stamp"]*1e9)
        k = np.searchsorted(times, stamp)
        candidates = [j for j in (k-1, k) if 0 <= j < len(times) and abs(int(times[j])-stamp) <= 2]
        # rclpy.Time uses truncation whereas native JSON stamps are rounded.
        # Two nanoseconds accommodates conversion only, never missing ticks.
        if len(candidates) != 1:
            raise ValueError("native tick lacks exact published GT snapshot")
        expected = validated_transform(poses[candidates[0]]) @ mount
        actual = validated_transform(frame["world_from_lidar"])
        position = float(np.linalg.norm(expected[:3, 3]-actual[:3, 3]))
        angle = float(np.arccos(np.clip((np.trace(expected[:3, :3].T@actual[:3, :3])-1.)/2., -1., 1.)))
        if position > .001 or angle > 1e-4:
            raise ValueError("published GT and native physics pose disagree")
        errors.append((position, angle))
    if not errors: raise ValueError("no GT/native snapshot pairs")
    maximum = np.max(errors, axis=0)
    return dict(pairs=len(errors), maximum_position_m=float(maximum[0]), maximum_rotation_rad=float(maximum[1]),
                association="exact measurement stamps (2 ns numeric conversion tolerance); no interpolation/fitting")


def validate_frozen_candidate(actual, expected, loaded, plugin_hashes):
    """Only the explicitly declared live normal-start pose may differ."""
    actual, expected = (json.loads(json.dumps(value)) for value in (actual, expected))
    selected = expected["config.json"]["global"]
    name = selected["config_odometry"]
    for configs in (actual, expected):
        pose = configs[name]["odometry_estimation"].pop("init_T_world_imu")
        if len(pose) != 7 or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1.) > 1e-5:
            raise ValueError("invalid declared initial pose")
    if actual != expected:
        raise ValueError("holdout changed frozen estimator settings beyond initial pose")
    paths = [expected[name]["odometry_estimation"]["so_name"]]
    paths += [p for p in expected[selected["config_ros"]]["glim_ros"]["extension_modules"]
              if p.endswith("/libclosed_window_abs.so")]
    if len(paths) != 2 or set(plugin_hashes) != set(paths):
        raise ValueError("frozen CPU/absolute plugin identity missing")
    for path in paths:
        records = [r for r in loaded if r["path"] == path]
        if len(records) != 1 or records[0]["sha256"] != plugin_hashes[path]:
            raise ValueError("frozen plugin not attested in live process maps")


def validate_loaded_dependencies(loaded, reference):
    actual = {r["path"]: r["sha256"] for r in loaded}
    if len(actual) != len(loaded) or not reference:
        raise ValueError("ambiguous/empty loaded dependency provenance")
    compared = 0
    for path, expected in reference.items():
        # Reader/serialization dependencies of the standalone MCAP probe may
        # legitimately be absent in a live ROS estimator process.
        probe_only = Path(path).name.startswith(("librosbag2_", "libservice_msgs__", "libyaml-cpp.so", "libmcap.so"))
        if path not in actual and probe_only:
            continue
        if actual.get(path) != expected:
            raise ValueError("frozen loaded dependency missing or changed: " + path)
        compared += 1
    return dict(matching_libraries=compared, missing_or_changed_reference_libraries=0)


def seal_capture(run, request, first_stamp, last_stamp, native):
    """Finish measurement before a blocking flush; success only after sealing."""
    validate_request(request)
    if not np.isfinite([first_stamp, last_stamp]).all() or not DURATION <= last_stamp-first_stamp <= DURATION+.04:
        raise ValueError("incomplete stationary acquisition")
    with (run / "acquisition_finished.json").open("x") as stream:
        json.dump(dict(schema=1, capture=run.name, views=0, commands_stopped=True, stamp=last_stamp), stream)
    native.close()
    with (run / "natural_holdout.json").open("x") as stream:
        json.dump(dict(complete=True, request=request, chunks=len(native.frames),
            first_stamp=first_stamp, last_stamp=last_stamp, command_publishers=0,
            stationary_only=True, validation="NOT_RUN"), stream, indent=2)


def validate_request(request):
    if (request.get("schema") != 1 or request.get("pattern") != PATTERN
            or request.get("seed") != SEED or request.get("pitch_deg") != 5.
            or request.get("home_xyz") != HOME or request.get("duration_s") != DURATION
            or request.get("motion_authority") is not False):
        raise ValueError("undeclared natural holdout acquisition")
    for key in ("base_from_imu", "base_from_lidar"):
        validated_transform(request[key])


def stationary_error(reference, current):
    a, b = validated_transform(reference), validated_transform(current)
    translation = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    rotation = float(np.arccos(np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1.) / 2., -1., 1.)))
    if translation > .005 or rotation > np.deg2rad(.1):
        raise ValueError("stationary holdout moved beyond 5 mm / 0.1 degree")
    return dict(translation_m=translation, rotation_rad=rotation)


def validate_stationary_native(audit, motion=False):
    from audit_native_terrain import summarize_frames
    report = summarize_frames(audit["frames"])
    current = report["current_pose"]
    if (not report["cadence_passed"] or audit["duplicate_native_reads"]
            or current["missing"] or current["maximum_m"] > .001 or current["samples"] < 10000
            or current["nonhorizontal_surface_samples"] < 100
            or abs(report["timeline_time_delta_ratio"] - 1.) > .005
            or abs(report["world_time_delta_ratio"] - 1.) > .005):
        raise ValueError("natural native cadence/current-pose geometry gate failed")
    frames = audit["frames"]
    times = np.array([f["observed_stamp"] for f in frames])
    duration_ok = (6. <= times[-1]-times[0] <= 200.) if motion else DURATION <= times[-1]-times[0] <= DURATION+.04
    if (not duration_ok or not np.allclose(np.diff(times), [f["world_rendering_dt"] for f in frames[1:]], rtol=.005, atol=1e-6)):
        raise ValueError("incomplete stationary duration or skipped native ticks")
    if not motion:
        for frame in frames:
            stationary_error(frames[0]["world_from_lidar"], frame["world_from_lidar"])
    report.update(stationary_only=not motion, eligible_for_moving_replay=motion, acceptance=False)
    return report
