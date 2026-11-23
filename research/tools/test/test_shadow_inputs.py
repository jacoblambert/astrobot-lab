"""Host NumPy/SciPy/pytest: experimental mount must not reuse untilted GLIM calibration."""
from pathlib import Path
import json
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_shadow_inputs import audit_sensor_config, prepare_config, BASE_FROM_IMU


@pytest.mark.parametrize("pitch", [0., 5., 10., 15.])
def test_mount_direction_lever_arm_and_global_shutter_are_explicit(pitch):
    sensor = {"T_lidar_imu": [-.136, .017, .012, 0., 0., 1., 0.],
              "global_shutter_lidar": True, "autoconf_perpoint_times": False}
    base_lidar = np.eye(4)
    base_lidar[:3, 3] = [.4, 0., .25]
    base_lidar[:3, :3] = Rotation.from_euler("y", pitch, degrees=True).as_matrix()
    rounded_legacy_imu = [.264, .017, .262, 0., 0., 1., 0.]
    report = audit_sensor_config(sensor, base_lidar, rounded_legacy_imu)
    assert report["mount_matches"] == (pitch == 0.)
    assert report["error"]["rotation_rad"] == pytest.approx(np.deg2rad(pitch), abs=1e-12)
    assert not report["capture_ready"] and not report["moving_ros_full_scan_timing_validated"]
    corrected = dict(sensor, T_lidar_imu=report["expected_T_lidar_imu"])
    assert audit_sensor_config(corrected, base_lidar, rounded_legacy_imu)["mount_matches"]
    assert sensor["T_lidar_imu"] == [-.136, .017, .012, 0., 0., 1., 0.]


def test_invalid_mount_rejected():
    invalid = np.eye(4)
    invalid[2, 2] = 2
    with pytest.raises(ValueError):
        audit_sensor_config({"T_lidar_imu": [0, 0, 0, 0, 0, 0, 1]}, invalid)


def test_rounded_imu_origin_is_not_the_strict_live_mount():
    from shadow_capture_gate import validate_shadow_geometry
    from audit_shadow_inputs import transform7
    imu = transform7(BASE_FROM_IMU)
    lidar = np.eye(4)
    lidar[:3, :3] = Rotation.from_euler("y", 5., degrees=True).as_matrix()
    lidar[:3, 3] = [.4, 0., .25]
    request = dict(schema=1, capture="revision-" + "a" * 32, pattern="wide_sweep15_return",
                   pitch_deg=5., home_xyz=[-20., -20., 0.], base_from_imu=imu.tolist(), base_from_lidar=lidar.tolist())
    rounded = np.linalg.inv(lidar) @ transform7([.264, .017, .262, 0., 0., 1., 0.])
    with pytest.raises(ValueError, match="extrinsic mismatch"):
        validate_shadow_geometry(request, rounded, imu, lidar, np.linalg.inv(rounded))
    exact = np.linalg.inv(lidar) @ imu
    assert max(validate_shadow_geometry(request, exact, imu, lidar, np.linalg.inv(exact)).values()) < 1e-12


def test_experiment_config_is_exclusive_and_changes_only_extrinsic(tmp_path):
    config, output = tmp_path / "source", tmp_path / "run"
    config.mkdir()
    output.mkdir()
    selection = {"config_sensors": "sensor.json", "config_global_mapping": "global.json", "config_sub_mapping": "sub.json"}
    source = {"config.json": {"global": selection}, "sensor.json": {"sensors": {"T_lidar_imu": [0, 0, 0, 0, 0, 0, 1],
                                                                               "global_shutter_lidar": True}},
              "global.json": {"global_mapping": {"enable_optimization": False}},
              "sub.json": {"sub_mapping": {"enable_optimization": False}}}
    for name, value in source.items():
        (config / name).write_text(json.dumps(value))
    report = {"configured_T_lidar_imu": [0, 0, 0, 0, 0, 0, 1], "expected_T_lidar_imu": [1, 0, 0, 0, 0, 0, 1]}
    result = prepare_config(config, output, report)
    for name, value in source.items():
        assert json.loads((config / name).read_text()) == value
        if name != "sensor.json":
            assert result["source_sha256"][name] == result["candidate_sha256"][name]
    candidate = json.loads((output / "candidate-config/sensor.json").read_text())["sensors"]
    assert candidate["T_lidar_imu"] == report["expected_T_lidar_imu"]
    assert candidate["global_shutter_lidar"] is True
    with pytest.raises(FileExistsError):
        prepare_config(config, output, report)
    bad_output = tmp_path / "bad"
    bad_output.mkdir()
    (config / "global.json").write_text(json.dumps({"global_mapping": {"enable_optimization": True}}))
    with pytest.raises(ValueError, match="optimization"):
        prepare_config(config, bad_output, report)
    assert not (bad_output / "candidate-config").exists()
