#!/usr/bin/env python3
"""Qualify natural capture identity, geometry and live pose receipts.

Produces a pose-pair artifact for existing diagnostics/replay preparation. It
does not itself establish pose accuracy, terrain recall or normal-start acceptance.
"""
import argparse
import json
from pathlib import Path
import numpy as np

from archive_guard import allocate_run, check_run
from audit_shadow_pair import compare_pose_sequence, sha
from export_shadow_trajectory import apply_record
from natural_holdout import validate_request, validate_stationary_native, validate_frozen_candidate, native_hit_records
from natural_holdout import validate_gt_snapshots, validate_loaded_dependencies
from shadow_capture_gate import ShadowLease
from shadow_terrain import PoseTimeline


def read(path):
    return json.loads(path.read_text())


def audit(capture, recorder, trajectory, output, frozen, motion="", selected_lidar=False):
    for folder in (capture, recorder, trajectory, frozen):
        check_run(folder)
        if any((folder / name).exists() for name in ("failure.json", "orchestration_failure.json")):
            raise ValueError("failed holdout is ineligible")
    complete = read(capture / "natural_holdout.json")
    scene = read(capture / "natural_scene.json")
    if scene.get("seed") != 43 or not scene.get("objects"):
        raise ValueError("independent natural rock geometry absent")
    collection = read(recorder / "collection.json")
    link = read(recorder / "shadow_collection.json")
    simulator = read(capture / "shadow_link.json")
    exported = read(trajectory / "shadow_trajectory.json")
    request = simulator["request"]
    if request.get("execution_budget", {}).get("diagnostic_only"):
        raise ValueError("instrumented GC stress capture is not a nominal holdout qualification")
    if ("lidar_profile" in request) != selected_lidar or selected_lidar and not motion:
        raise ValueError("selected LiDAR requires explicit moving audit protocol")
    if motion:
        from natural_motion import validate_motion_request
        validate_motion_request(request)
        if request["pattern"] != motion or collection.get("natural_motion") != motion:
            raise ValueError("natural motion route mismatch")
    else:
        validate_request(request)
    if (not complete.get("complete") or complete.get("stationary_only") is not (not bool(motion))
            or complete.get("command_publishers") != (1 if motion else 0) or complete["request"] != request
            or not collection.get("collection_completed") or collection.get("failure")
            or collection.get("natural_stationary") is not (not bool(motion)) or not collection.get("physics_imu_required")
            or (not motion and collection["counts"].get("/astrobot_0/cmd_vel", 0) != 0)
            or request != {k: v for k, v in link["request"].items() if k != "stamp"}
            or request["capture"] != capture.name or link["recorder"] != recorder.name
            or link["motion_owner"] != ("natural_holdout_capture" if motion else "none") or link["home_xyz"] != request["home_xyz"]):
        raise ValueError("stationary completion/identity/motion contract failed")
    grant = simulator["first_grant"]
    ShadowLease(capture.name).receive(grant, grant["stamp"], 0.)
    geometry = link["live_geometry_errors"]
    if (set(geometry) != {"base_imu_tf", "base_lidar_tf", "glim_config", "glim_revision"}
            or not np.isfinite(list(geometry.values())).all()):
        raise ValueError("invalid live geometry audit")
    if (grant["recorder"] != recorder.name or [grant["session"], grant["epoch"]] != link["identity"]
            or grant["config_sha256"] != link["config_sha256"]
            or max(link["live_geometry_errors"].values()) > 1e-4):
        raise ValueError("recording grant or live geometry differs")
    sensor = read(capture / "sensor_audit.json")
    if not sensor["passed"] or not sensor["extrinsic_tf_checked"]:
        raise ValueError("stationary ROS sensor/TF audit failed")
    sealed_sensor_hash = read(capture / "manifest.json").get("sensor_audit_sha256")
    if ((selected_lidar and sealed_sensor_hash is None)
            or sealed_sensor_hash is not None and sha(capture / "sensor_audit.json") != sealed_sensor_hash):
        raise ValueError("stationary audit changed after pre-motion validation")
    if {p.name: sha(p) for p in (recorder / "config").glob("*.json")} != link["config_files"]:
        raise ValueError("recorded configuration changed")
    from run_paired_glim_replay import runtime_configs
    manifest = read(recorder / "runtime_manifest.json")
    configs = {k: json.loads(v) for k, v in runtime_configs(
        manifest, expected_imu="imu_physics", selected_lidar=selected_lidar).items()}
    expected = {p.name: read(p) for p in (frozen / "config").glob("*.json")}
    selected = expected["config.json"]["global"]
    plugins = [expected[selected["config_odometry"]]["odometry_estimation"]["so_name"]]
    plugins += [p for p in expected[selected["config_ros"]]["glim_ros"]["extension_modules"]
                if p.endswith("/libclosed_window_abs.so")]
    plugin_hashes = {p: sha(Path(p)) for p in plugins}
    processes = [p for p in manifest["processes"] if p["executable"]["resolved_path"].endswith("/glim_rosnode")]
    validate_frozen_candidate(configs, expected, processes[0]["loaded_code"]["libraries"], plugin_hashes)
    motion_sources = []
    if motion:
        from natural_motion import audit_motion_artifacts
        motion_sources = audit_motion_artifacts(capture, recorder, request, complete, grant, configs)
        heap_path = capture / "heap_checkpoint.json"
        if heap_path.exists():
            heap = read(heap_path)
            if heap.get("gc_enabled") is not True or heap.get("phase") != "before_request_and_motion_lease":
                raise ValueError("invalid pre-motion heap checkpoint")
            motion_sources.append(heap_path)
        from natural_motion import validate_ground_tile
        clearance = read(capture / "clearance.json")
        with np.load(capture / "clearance_tile.npz") as tile:
            checked = validate_ground_tile(tile["xy"], tile["z"], clearance["shape"], tile["normal"])
        for key, value in checked.items():
            actual = clearance.get(key)
            # Isaac and the ROS evaluator use different NumPy/LAPACK builds;
            # compare fitted scalars numerically, not at the last binary digit.
            matches = (isinstance(actual, (int, float)) and np.isclose(actual, value, rtol=1e-9, atol=1e-9)
                       if isinstance(value, float) else actual == value)
            if not matches:
                raise ValueError("retained clearance metrics do not reproduce: " + key)
    dependency_source = frozen / "loaded_libraries.json"
    if selected_lidar:
        from selected_lidar_profile import declaration, validate_config
        validate_config(frozen / "config")
        dependency_source = frozen / "profile.json"
        profile = read(dependency_source)
        if profile["profile"] != declaration(request["lidar_profile"]["name"]) or profile["config_sha256"] != {
                p.name: sha(p) for p in (frozen / "config").glob("*.json")}:
            raise ValueError("frozen selected profile changed")
        # New sensor/runtime experiment, not a matched-library old-sensor replay.
        # Actual loaded hashes remain attested; plugin/config equality above is
        # mandatory. Do not imply an exact historical binary A/B comparison.
        dependencies = dict(historical_binary_equivalence="NOT_CLAIMED",
            libraries=processes[0]["loaded_code"]["libraries"])
    else:
        dependencies = validate_loaded_dependencies(processes[0]["loaded_code"]["libraries"],
                                                   read(dependency_source))
    if not exported.get("passed") or Path(exported["source"]) != recorder:
        raise ValueError("unqualified trajectory export")
    for name, expected in exported["input_sha256"].items():
        if Path(name).is_absolute() or ".." in Path(name).parts or sha(recorder / name) != expected:
            raise ValueError("recorded source changed")
    if sha(trajectory / "trajectory.jsonl") != exported["trajectory_sha256"]:
        raise ValueError("trajectory changed")
    native = read(capture / "native_timing_audit.json")
    if selected_lidar:
        from selected_native_audit import validate_native, audit_transport
        timing = validate_native(native, motion=motion, profile=request["lidar_profile"]["name"])
        timing["transport"] = audit_transport(capture, recorder)
    else:
        timing = validate_stationary_native(native, motion=bool(motion))
    frames = native["frames"]
    if (complete["chunks"] != len(frames) or frames[0]["observed_stamp"] < grant["stamp"]
            or complete["first_stamp"] != frames[0]["observed_stamp"]
            or complete["last_stamp"] != frames[-1]["observed_stamp"]):
        raise ValueError("native acquisition preceded the grant or lost chunks")
    files = sorted((capture / "native_chunks").glob("chunk_*.npz"))
    if len(files) != len(frames) or any(p.name != f["file"] or sha(p) != f["sha256"] for p, f in zip(files, frames)):
        raise ValueError("native chunks changed")
    for frame in frames:
        native_hit_records(frame, capture / "native_chunks")
    from glim_gt_start import recorded_gt
    gt_times, gt_poses = recorded_gt(recorder)
    gt_reference = validate_gt_snapshots(frames, gt_times, gt_poses, request["base_from_lidar"])
    if motion:
        from natural_motion import verify_live_start
        first = read(recorder / "initial_output_check.json")
        checked = verify_live_start(request, first["first_frame"], list(zip(gt_times, gt_poses)))
        if not checked["passed"]:
            raise ValueError("offline first-output GT verification failed")
    timeline = PoseTimeline(*link["identity"])
    first_recorded_frame = None
    for line in (trajectory / "trajectory.jsonl").read_text().splitlines():
        record = json.loads(line)
        if first_recorded_frame is None and record["frames"]:
            first_recorded_frame = min(record["frames"], key=lambda f: f["stamp_ns"])
        apply_record(timeline, record)
    if motion and first_recorded_frame != first["first_frame"]:
        raise ValueError("live first-output check differs from recorded first frame")
    if [exported["session"], exported["epoch"]] != link["identity"] or exported["frames"] != len(timeline.frames):
        raise ValueError("trajectory identity/count mismatch")
    rows, errors = compare_pose_sequence(timeline, frames, request["home_xyz"])
    with (output / "paired_poses.jsonl").open("x") as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    sources = (capture / "natural_holdout.json", capture / "natural_scene.json", capture / "shadow_link.json", capture / "manifest.json",
               capture / "sensor_audit.json", capture / "native_timing_audit.json", recorder / "collection.json",
               recorder / "shadow_collection.json", recorder / "runtime_manifest.json",
               trajectory / "shadow_trajectory.json", trajectory / "trajectory.jsonl",
               dependency_source, *sorted((frozen / "config").glob("*.json")), *map(Path, plugins), *motion_sources)
    if selected_lidar:
        sources += (capture / "selected_sensor.json", capture / "selected_windows.jsonl",
                    Path(__file__).with_name("selected_native_audit.py"), Path(__file__))
    return dict(passed=True, experiment="natural_motion_gt_start" if motion else "natural_stationary_holdout", stationary_only=not bool(motion),
                chunks=len(rows), fixed_home=request["home_xyz"], session=timeline.session, epoch=timeline.epoch,
                errors=errors, timing=timing, paired_poses_sha256=sha(output / "paired_poses.jsonl"),
                frozen_candidate=str(frozen), plugin_sha256=plugin_hashes,
                gt_reference=gt_reference,
                frozen_dependencies=dependencies,
                input_sha256={str(p): sha(p) for p in sources}, initialization="live one-time GT seed" if motion else "live normal supervisor start",
                causal_map_projection_evaluated=False, terrain_validation="NOT_RUN", navigation_acceptance=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture", "recorder", "trajectory"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--frozen-candidate", type=Path, required=True)
    from natural_motion import PATTERNS
    parser.add_argument("--motion", choices=PATTERNS, default="")
    parser.add_argument("--selected-lidar", action="store_true")
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = audit(args.capture, args.recorder, args.trajectory, output, args.frozen_candidate, args.motion, args.selected_lidar)
    except Exception as exc:
        result = dict(passed=False, failure=str(exc), stationary_only=not bool(args.motion), navigation_acceptance=False)
    with (output / "shadow_pair_audit.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
