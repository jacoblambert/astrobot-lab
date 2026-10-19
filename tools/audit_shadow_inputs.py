#!/usr/bin/env python3
"""Pre-capture GLIM mount/timing audit against an audited native fixture.

No simulator launch or configuration mutation. The known base->IMU extrinsic is
explicit in the report; it must still be checked against live TF for a new run.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from revision_evidence import validated_transform
from shadow_terrain import pose_error


# Settled fixed-joint geometry, independently agreeing with live TF within
# 1.2 micrometres in recorder revision-816d4e50f04d4cb38f162d1bcd9d20ad.
# The older millimetre-rounded [.264,.017,.262] misses the actual origin by
# 0.547 mm. Authored USD alone is insufficient: physics resolves the D455 joint.
# This calibration evidence does NOT make that failed recording score-eligible.
BASE_FROM_IMU = [.26399862939321395, .017279625659938347, .2624700776230195, 0., 0., 1., 0.]
BASE_FROM_IMU_SOURCE = dict(recorder="revision-816d4e50f04d4cb38f162d1bcd9d20ad",
    capture="revision-ddd7ee425b2e423bab71e7af75d2647e", source="settled_USD_and_live_TF",
    robot_asset_sha256="357bd6cd9531cceb7971059c96d8a61052165eea7f8f6ff9bf28f35a5de1ca67",
    rotation_residual_rad=6.604777070249176e-08, next_capture_live_validation_required=True)


def transform7(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (7,) or not np.isfinite(value).all() or abs(value[3:] @ value[3:] - 1) > 1e-6:
        raise ValueError("expected xyz and normalized xyzw")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(value[3:]).as_matrix()
    matrix[:3, 3] = value[:3]
    return matrix


def audit_sensor_config(sensors, base_from_lidar, base_from_imu=BASE_FROM_IMU):
    # T_lidar_imu in GLIM is LiDAR <- IMU, the inverse of exported imu_from_lidar.
    expected = np.linalg.inv(validated_transform(base_from_lidar)) @ transform7(base_from_imu)
    actual = transform7(sensors["T_lidar_imu"])
    error = pose_error(actual, expected)
    # Same matrix tolerance as the live readiness gate; a loose archival audit
    # must not obscure a calibration that cannot pass live qualification.
    matrix_error = float(np.max(np.abs(actual - expected)))
    matches = matrix_error <= 1e-4
    return dict(mount_matches=matches, error=error,
                maximum_matrix_error=matrix_error,
                expected_T_lidar_imu=[*expected[:3, 3].tolist(), *Rotation.from_matrix(expected[:3, :3]).as_quat().tolist()],
                configured_T_lidar_imu=sensors["T_lidar_imu"],
                base_from_imu_assumption=base_from_imu,
                default_imu_calibration_source=BASE_FROM_IMU_SOURCE if np.array_equal(base_from_imu, BASE_FROM_IMU) else None,
                global_shutter_lidar=bool(sensors.get("global_shutter_lidar", False)),
                perpoint_autoconfiguration=bool(sensors.get("autoconf_perpoint_times", True)),
                moving_ros_full_scan_timing_validated=False,
                live_base_imu_tf_validation="REQUIRED", capture_ready=False)


def audit(capture, config):
    check_run(capture)
    if (capture / "failure.json").exists():
        raise ValueError("failed fixture is ineligible")
    manifest = json.loads((capture / "manifest.json").read_text())
    sensor_audit = json.loads((capture / "sensor_audit.json").read_text())
    if not sensor_audit.get("passed") or not sensor_audit.get("extrinsic_tf_checked"):
        raise ValueError("fixture lacks passing sensor/TF audit")
    selection = json.loads((config / "config.json").read_text())["global"]
    sensor_file = config / selection["config_sensors"]
    sensors = json.loads(sensor_file.read_text())["sensors"]
    result = audit_sensor_config(sensors, manifest["sensor"]["base_from_lidar"])
    result.update(capture=str(capture), config=str(config),
                  input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                                (capture / "manifest.json", capture / "sensor_audit.json", config / "config.json", sensor_file)},
                  note="Archived mount audit, not a fresh moving GLIM validation. Production configuration unchanged.")
    return result


def prepare_config(config, output, report):
    """Freeze an opt-in config with ONLY the measured mount correction.

    No effect on normal launches. Does not enable deskew/optimization or claim
    readiness. Sources and candidate hashes make this change reviewable.
    """
    if "expected_T_lidar_imu" not in report:
        raise ValueError("cannot prepare configuration without a valid mount audit")
    files = {path.name: path.read_bytes() for path in sorted(config.glob("*.json"))}
    selection = json.loads(files["config.json"])["global"]
    for section in ("sensors", "global_mapping", "sub_mapping"):
        name = selection["config_" + section]
        if Path(name).name != name or name not in files:
            raise ValueError("selected config must be an included local JSON file")
        if section != "sensors" and json.loads(files[name])[section].get("enable_optimization") is not False:
            raise ValueError("shadow baseline must keep graph optimization disabled")
    sensor_name = selection["config_sensors"]
    sensors = json.loads(files[sensor_name])
    if sensors["sensors"]["T_lidar_imu"] != report["configured_T_lidar_imu"]:
        raise ValueError("sensor config changed after audit")
    sensors["sensors"]["T_lidar_imu"] = report["expected_T_lidar_imu"]
    destination = output / "candidate-config"
    destination.mkdir(exist_ok=False)
    source_hashes, candidate_hashes = {}, {}
    for name, content in files.items():
        source_hashes[name] = hashlib.sha256(content).hexdigest()
        candidate = (json.dumps(sensors, indent=2, allow_nan=False) + "\n").encode() if name == sensor_name else content
        with (destination / name).open("xb") as stream:
            stream.write(candidate)
        candidate_hashes[name] = hashlib.sha256(candidate).hexdigest()
    return dict(path=str(destination), changed_file=sensor_name, changed_parameter="sensors.T_lidar_imu",
                source_sha256=source_hashes, candidate_sha256=candidate_hashes,
                production_changed=False, live_tf_and_timing_validation="REQUIRED")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] /
                        "astrobot-lab/src/glim_slam_launch/config/glim_astrobot")
    parser.add_argument("--archive-root", type=Path, default=Path("/archive") if Path("/archive").is_dir() else ARCHIVE_ROOT)
    parser.add_argument("--prepare-config", action="store_true", help="Freeze a mount-corrected experiment-only config in the new run")
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = audit(args.capture, args.config)
        if args.prepare_config:
            result["prepared_config"] = prepare_config(args.config, output, result)
    except Exception as exc:
        result = dict(mount_matches=False, capture_ready=False, failure=str(exc))
    with (output / "shadow_input_audit.json").open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2))
    return 0 if result["mount_matches"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
