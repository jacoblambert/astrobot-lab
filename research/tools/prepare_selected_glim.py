#!/usr/bin/env python3
"""Freeze an opt-in native-window profile from a completed physical-IMU recorder."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from archive_guard import ARCHIVE_ROOT, GIB, allocate_run, check_capacity
from audit_shadow_inputs import audit_sensor_config
from selected_lidar_profile import TOPIC, PROFILES, declaration, profile_path, selected_setup, validate_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recorder", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--profile", choices=PROFILES, default="selected_generic_v1")
    args = parser.parse_args()
    # Read-only historical import: Linux st_dev can change after reconnecting the
    # same external filesystem. Never rewrite its old lease or resume that run.
    # New output gets a fresh device-bound lease and normal strict runtime checks.
    root = check_capacity(args.archive_root, 50 * GIB)
    if args.recorder.resolve().parent != root:
        raise ValueError("historical input must reside on the verified external output archive")
    historical_contract = json.loads((args.recorder / "archive_contract.json").read_text())
    if historical_contract.get("run") != args.recorder.name or historical_contract.get("external_required") is not True:
        raise ValueError("historical input lacks an external archive identity")
    source = args.recorder / "config"
    # Refuse an arbitrary production config or failed recorder as the reference.
    result = json.loads((args.recorder / "collection.json").read_text())
    if not result.get("collection_completed") or result.get("failure"):
        raise ValueError("completed moving recorder required")
    run = allocate_run(args.archive_root)
    print(run, flush=True)
    try:
        config = run / "config"
        shutil.copytree(source, config)
        selection = json.loads((config / "config.json").read_text())["global"]
        path = config / selection["config_sensors"]
        data = json.loads(path.read_text())
        mount = np.eye(4)
        setup, _ = selected_setup(profile_path(args.profile))
        mount[:3, 3] = setup["base_from_lidar_translation_m"]
        calibration = audit_sensor_config(data["sensors"], mount)
        data["sensors"].update(
            T_lidar_imu=calibration["expected_T_lidar_imu"],
            global_shutter_lidar=False,
            autoconf_perpoint_times=False,
            autoconf_prefer_frame_time=True,
            perpoint_relative_time=True,
            perpoint_time_scale=1.0,
        )
        path.write_text(json.dumps(data, indent=2) + "\n")
        ros_path = config / "config_ros.json"
        ros = json.loads(ros_path.read_text())
        if ros["glim_ros"]["imu_topic"] != "imu_physics":
            raise ValueError("reference must already use physical IMU")
        ros["glim_ros"]["points_topic"] = TOPIC
        ros_path.write_text(json.dumps(ros, indent=2) + "\n")
        validate_config(config)
        with (run / "profile.json").open("x") as stream:
            json.dump(
                dict(
                    profile=declaration(args.profile),
                    historical_archive_contract=historical_contract,
                    current_archive_device=root.stat().st_dev,
                    reference=str(args.recorder),
                    calibration=calibration,
                    source_config_sha256={
                        p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.glob("*.json")
                    },
                    config_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in config.glob("*.json")},
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    changes=["level mount", "native tick-relative timing", "explicit nonuniform point topic"],
                    live_tf_validation_required=True,
                    motion_qualified=False,
                ),
                stream,
                indent=2,
            )
    except Exception as exc:
        with (run / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc)), stream)
        raise


if __name__ == "__main__":
    main()
