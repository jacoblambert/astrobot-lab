"""Opt-in must not silently reinterpret old recordings or zero timed windows."""

import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from selected_lidar_profile import TOPIC, declaration, profile_path, required_topics, selected_setup, validate_config
from natural_motion import validate_motion_request
from test_natural_motion import request


def selected_request():
    value = request()
    mount = np.eye(4)
    mount[:3, 3] = [0.4, 0, 0.25]
    value.update(lidar_profile=declaration(), pitch_deg=0.0, base_from_lidar=mount.tolist())
    return value


def test_legacy_and_selected_have_distinct_frozen_mount_contracts():
    validate_motion_request(request())
    validate_motion_request(selected_request())
    for field, value in [("pitch_deg", 5.0), ("lidar_profile", {}), ("base_from_lidar", np.eye(4).tolist())]:
        changed = selected_request()
        changed[field] = value
        with pytest.raises(ValueError):
            validate_motion_request(changed)
    changed = request()
    changed["pitch_deg"] = 0.0
    with pytest.raises(ValueError):
        validate_motion_request(changed)


def test_topic_replacement_keeps_all_nonsensor_requirements():
    original = {
        "/clock",
        "/gt/base_link_pose",
        "/astrobot_0/lidar_0/pointcloud/raw",
        "/astrobot_0/lidar_0/pointcloud/filtered",
    }
    assert required_topics(original) == {"/clock", "/gt/base_link_pose", TOPIC}
    assert len(original) == 4


def test_optical_profile_is_distinct_without_reinterpreting_old_setup(monkeypatch):
    old = declaration("selected_generic_v1")
    new = declaration("selected_optical_v2")
    setup, rays = selected_setup(profile_path("selected_optical_v2"))
    original, original_rays = selected_setup()
    np.testing.assert_array_equal(rays, original_rays)
    assert original["base_from_lidar_translation_m"] == [0.4, 0, 0.25]
    assert setup["base_from_lidar_translation_m"] == [0.4, 0, 0.45]
    assert setup["raycast_min_range_m"] == 0 and setup["min_range_m"] == 0.5
    assert old["setup_sha256"] != new["setup_sha256"]
    value = selected_request()
    value["lidar_profile"] = new
    with pytest.raises(ValueError):
        validate_motion_request(value)
    value["base_from_lidar"][2][3] = 0.45
    validate_motion_request(value)
    monkeypatch.setenv("ASTROBOT_SELECTED_LIDAR_PROFILE", "selected_optical_v2")
    assert declaration() == new
    # Historical protocol validation remains valid under a new live selection.
    historical = selected_request()
    historical["lidar_profile"] = old
    validate_motion_request(historical)


def test_time_policy_and_sensor_streams_are_explicit(tmp_path):
    sensors = dict(
        global_shutter_lidar=False,
        autoconf_perpoint_times=False,
        autoconf_prefer_frame_time=True,
        perpoint_relative_time=True,
        perpoint_time_scale=1.0,
    )
    ros = dict(points_topic=TOPIC, imu_topic="imu_physics", points_time_offset=0.0, imu_time_offset=0.0)
    (tmp_path / "config.json").write_text(json.dumps({"global": {"config_sensors": "config_sensors.json"}}))

    def write():
        (tmp_path / "config_sensors.json").write_text(json.dumps(dict(sensors=sensors)))
        (tmp_path / "config_ros.json").write_text(json.dumps(dict(glim_ros=ros)))

    write()
    validate_config(tmp_path)
    for key in ("global_shutter_lidar", "autoconf_perpoint_times", "perpoint_relative_time"):
        sensors[key] = not sensors[key]
        write()
        with pytest.raises(ValueError):
            validate_config(tmp_path)
        sensors[key] = not sensors[key]
    ros["points_time_offset"] = 0.1
    write()
    with pytest.raises(ValueError):
        validate_config(tmp_path)
