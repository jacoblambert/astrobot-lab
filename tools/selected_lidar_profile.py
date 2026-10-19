"""Explicit opt-in contract for the level, nonuniform sensor on the live rover."""

import hashlib
import json
import os

import numpy as np
from costmap_scenarios import SCENARIOS

from local_terrain.ray_design import SELECTED_SETUP, selected_setup

PROFILE = "selected_generic_v1"
PROFILES = {PROFILE: SELECTED_SETUP, "selected_optical_v2": SELECTED_SETUP.with_name("selected_lidar_optical.json")}
TOPIC = "/astrobot_0/lidar_selected/points"


def profile_path(name=None):
    name = name or os.environ.get("ASTROBOT_SELECTED_LIDAR_PROFILE", PROFILE)
    if name not in PROFILES:
        raise ValueError("unknown selected sensor profile")
    return PROFILES[name]


def acquisition_limits(pattern):
    from natural_motion import route

    route(pattern)  # No arbitrary duration extension through an unknown route.
    if pattern in SCENARIOS:
        return dict(max_duration_s=620, max_native_frames=38000)
    if pattern == "turn360":
        # Ten independently bounded 40 s legs plus settling/pre/postroll.
        # Explicit new observation dose, not an extension of any old route.
        return dict(max_duration_s=420, max_native_frames=26000)
    # Existing loop leg limits total 640 s, plus pre/postroll and settles.
    # These are storage/evaluation bounds, not modified driver deadlines.
    return (
        dict(max_duration_s=660, max_native_frames=40000)
        if pattern == "loop16"
        else dict(max_duration_s=200, max_native_frames=20000)
    )


def declaration(name=None):
    path = profile_path(name)
    return dict(
        name=next(key for key, value in PROFILES.items() if value == path),
        setup_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        physics_hz=60,
        rendering_hz=60,
        ticks_per_window=6,
        topic=TOPIC,
    )


def validate_request(request):
    profile = request.get("lidar_profile", {}).get("name", "invalid")
    if request.get("lidar_profile") != declaration(profile) or request.get("pitch_deg") != 0.0:
        raise ValueError("selected LiDAR protocol differs from frozen geometry/timing")
    if request.get("pattern") in ("loop16", "turn360", *SCENARIOS) and request.get("acquisition_limits") != acquisition_limits(request["pattern"]):
        raise ValueError("selected loop requires declared extended acquisition bounds")
    setup, _ = selected_setup(profile_path(profile))
    expected = np.eye(4)
    expected[:3, 3] = setup["base_from_lidar_translation_m"]
    actual = np.asarray(request["base_from_lidar"])
    if actual.shape != (4, 4) or not np.allclose(actual, expected, atol=1e-5, rtol=0):
        raise ValueError("selected LiDAR must be level at its declared base extrinsic")


def validate_config(config):
    selection = json.loads((config / "config.json").read_text())["global"]
    sensors = json.loads((config / selection["config_sensors"]).read_text())["sensors"]
    ros = json.loads((config / "config_ros.json").read_text())["glim_ros"]
    expected = dict(
        global_shutter_lidar=False,
        autoconf_perpoint_times=False,
        autoconf_prefer_frame_time=True,
        perpoint_relative_time=True,
        perpoint_time_scale=1.0,
    )
    if any(sensors.get(k) != v for k, v in expected.items()):
        raise ValueError("selected native windows require unchanged tick-relative point times")
    if (
        ros.get("points_topic") != TOPIC
        or ros.get("imu_topic") != "imu_physics"
        or ros.get("points_time_offset") != 0.0
        or ros.get("imu_time_offset") != 0.0
    ):
        raise ValueError("selected profile requires declared native topic/physical IMU without offsets")


def required_topics(topics):
    return (set(topics) - {"/astrobot_0/lidar_0/pointcloud/" + s for s in ("raw", "filtered")}) | {TOPIC}
