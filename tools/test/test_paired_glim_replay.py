"""Host preparation/dispatch contracts; real replay requires the sourced overlay."""
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_paired_glim_replay import ABSOLUTE, IMUS, POINTS, TYPES, runtime_configs, validate_dispatch, validate_ab_settings, lidar_only_configs
from run_paired_glim_replay import execution_configs
from run_paired_glim_replay import absolute_candidate_config
from run_paired_glim_replay import gpu_configs
from run_paired_glim_replay import validate_inherited_candidates


def test_frozen_candidate_cannot_be_mislabeled_as_a_default_baseline():
    configs = {"config.json": {"global": {"config_odometry": "o.json", "config_ros": "r.json"}},
        "o.json": {"odometry_estimation": {"revise_target_poses": True}},
        "r.json": {"abs_constraint": {"enable": True, "require_closed_window": True}}}
    with pytest.raises(ValueError, match="CPU candidate"):
        validate_inherited_candidates(configs, "physics", "cpu", None, None)
    with pytest.raises(ValueError, match="absolute candidate"):
        validate_inherited_candidates(configs, "physics", "cpu", "cpu.so", None)
    validate_inherited_candidates(configs, "physics", "cpu", "cpu.so", "abs.so")
    validate_inherited_candidates(configs, "lidar_only", "cpu", None, None)


def test_gpu_backend_retains_seed_bias_lag_and_noise_policy():
    gpu = json.loads((Path(__file__).resolve().parents[2]/"astrobot-lab/src/glim_slam_launch/config/glim_astrobot/config_odometry_gpu.json").read_text())
    base = {"so_name": "libodometry_estimation_cpu.so", "init_T_world_imu": [1,2,3,0,0,0,1],
            "smoother_lag": 5., "fix_imu_bias": True, "num_threads": 1}
    original = {"config.json": {"global": {"config_odometry": "cpu.json"}},
                "cpu.json": {"odometry_estimation": base}, "config_odometry_gpu.json": gpu,
                "sensors.json": {"gravity": 1.62}}
    result = gpu_configs(original)
    candidate = result["config_odometry_gpu_trial.json"]["odometry_estimation"]
    for key in set(base)-{"so_name"}: assert candidate[key] == base[key]
    assert candidate["so_name"] == "libodometry_estimation_gpu.so"
    assert original["config.json"]["global"]["config_odometry"] == "cpu.json"
    assert result["sensors.json"] == original["sensors.json"]


def test_closed_window_preserves_fix_noise_frames_and_topics():
    config = {"glim_ros": {"extension_modules": ["librevision_module.so", "libabs_constraint_module.so"]},
              "abs_constraint": {"rtk_topic": ABSOLUTE, "assoc_tolerance": .05, "sigma_floor": .05}}
    patched = absolute_candidate_config(config, "/frozen/plugin.so", "closed-window")
    assert config["glim_ros"]["extension_modules"][-1] == "libabs_constraint_module.so"
    assert patched["glim_ros"]["extension_modules"] == ["librevision_module.so", "/frozen/plugin.so"]
    assert patched["abs_constraint"] == dict(config["abs_constraint"], require_closed_window=True)
    assert not absolute_candidate_config(config, "/frozen/plugin.so", "immediate")["abs_constraint"]["require_closed_window"]
    with pytest.raises(ValueError): absolute_candidate_config(config, "p", "unknown")


def test_already_frozen_absolute_plugin_can_be_replayed_but_ambiguous_modules_cannot():
    config = {"glim_ros": {"extension_modules": ["librevision_module.so", "/old/libclosed_window_abs.so"]},
              "abs_constraint": {"assoc_tolerance": .05}}
    result = absolute_candidate_config(config, "/new/libclosed_window_abs.so", "closed-window")
    assert result["glim_ros"]["extension_modules"][-1] == "/new/libclosed_window_abs.so"
    config["glim_ros"]["extension_modules"].append("libabs_constraint_module.so")
    with pytest.raises(ValueError, match="ambiguous"):
        absolute_candidate_config(config, "/new/libclosed_window_abs.so", "closed-window")


def test_serial_profile_changes_only_declared_worker_settings():
    config = {"config.json": {"global": {"config_preprocess": "pre.json", "config_odometry": "odom.json"}},
              "pre.json": {"preprocess": {"num_threads": 2, "resolution": .25}},
              "odom.json": {"odometry_estimation": {"num_threads": 6, "weight": 300}},
              "sensor.json": {"gravity": 1.62}}
    serial = execution_configs(config, "serial")
    assert config["pre.json"]["preprocess"]["num_threads"] == 2
    assert serial["pre.json"]["preprocess"] == {"num_threads": 1, "resolution": .25}
    assert serial["odom.json"]["odometry_estimation"] == {"num_threads": 1, "num_smoother_update_threads": 1, "weight": 300}
    assert serial["sensor.json"] == config["sensor.json"]
    assert execution_configs(config, "recorded") == config
    with pytest.raises(ValueError, match="undeclared"):
        execution_configs(config, "arbitrary")


def manifest():
    configs = {"config.json": {"global": {"config_"+k: k+".json" for k in
                                        ("ros", "sensors", "odometry", "sub_mapping", "global_mapping")}},
               **{k+".json": {} for k in ("sensors", "odometry", "sub_mapping", "global_mapping")},
               "ros.json": {"glim_ros": {"imu_topic": "imu", "points_topic": "lidar_0/pointcloud/filtered",
                                          "imu_time_offset": 0., "points_time_offset": 0.},
                            "abs_constraint": {"rtk_topic": ABSOLUTE, "enable": True}}}
    files = []
    for name, value in configs.items():
        content = json.dumps(value)
        files.append(dict(path="/original/"+name, content=content, sha256=hashlib.sha256(content.encode()).hexdigest()))
    return dict(processes=[dict(executable=dict(resolved_path="/installed/glim_rosnode"), runtime_files=files)])


def test_actual_launch_config_bytes_are_preserved_not_reinitialized():
    original = manifest()
    result = runtime_configs(original)
    assert result == {Path(f["path"]).name: f["content"] for f in original["processes"][0]["runtime_files"]}


def test_physics_capture_requires_explicit_input_wiring_declaration():
    original = manifest()
    for f in original["processes"][0]["runtime_files"]:
        if f["path"].endswith("/ros.json"):
            value = json.loads(f["content"])
            value["glim_ros"]["imu_topic"] = "imu_physics"
            f["content"] = json.dumps(value)
            f["sha256"] = hashlib.sha256(f["content"].encode()).hexdigest()
    with pytest.raises(ValueError, match="wiring"):
        runtime_configs(original)
    assert runtime_configs(original, expected_imu="imu_physics")
    with pytest.raises(ValueError, match="undeclared"):
        runtime_configs(original, expected_imu="arbitrary")


def test_selected_points_are_not_silently_accepted_as_legacy_wiring():
    from selected_lidar_profile import TOPIC
    original = manifest()
    for f in original["processes"][0]["runtime_files"]:
        if f["path"].endswith("/ros.json"):
            value = json.loads(f["content"])
            value["glim_ros"].update(imu_topic="imu_physics", points_topic=TOPIC)
            f["content"] = json.dumps(value)
            f["sha256"] = hashlib.sha256(f["content"].encode()).hexdigest()
    with pytest.raises(ValueError, match="wiring"):
        runtime_configs(original, expected_imu="imu_physics")
    with pytest.raises(ValueError, match="physical IMU"):
        runtime_configs(original, selected_lidar=True)
    assert runtime_configs(original, expected_imu="imu_physics", selected_lidar=True)


@pytest.mark.parametrize("fault", ["content", "duplicate", "missing", "processes"])
def test_broken_runtime_provenance_fails(fault):
    m = manifest()
    files = m["processes"][0]["runtime_files"]
    if fault == "content": files[0]["content"] += " "
    if fault == "duplicate": files.append(files[-1])
    if fault == "missing": files.pop()
    if fault == "processes": m["processes"] *= 2
    with pytest.raises(ValueError):
        runtime_configs(m)


@pytest.mark.parametrize("mode", list(IMUS))
def test_dispatch_counts_exclude_only_the_other_imu(mode):
    expected = {t: i+10 for i,t in enumerate(TYPES)}
    actual = {t:n for t,n in expected.items() if t != IMUS["physics" if mode == "legacy" else "legacy"]}
    dispatch = dict(completed=True, counts=actual, mode=mode)
    validate_dispatch(expected, dispatch, mode)
    for topic in (IMUS[mode], POINTS, ABSOLUTE, "/clock"):
        bad = dict(dispatch, counts=dict(actual, **{topic: actual[topic]-1}))
        with pytest.raises(ValueError, match="count"):
            validate_dispatch(expected, bad, mode)


def test_replay_allowlist_excludes_gt_navigation_and_recorded_estimator_outputs():
    assert set(TYPES) == {"/clock", POINTS, ABSOLUTE, *IMUS.values()}
    assert not any("gt" in t or "cmd_vel" in t or "slam_raw" in t for t in TYPES)


def test_lidar_only_dispatch_rejects_any_imu_or_rtk_consumption():
    expected = {t: 10 for t in TYPES}
    dispatch = dict(completed=True, mode="lidar_only", counts={POINTS: 10, "/clock": 10})
    validate_dispatch(expected, dispatch, "lidar_only")
    for extra in (ABSOLUTE, *IMUS.values()):
        with pytest.raises(ValueError, match="count"):
            validate_dispatch(expected, dict(dispatch, counts={**dispatch["counts"], extra: 1}), "lidar_only")


def test_lidar_only_config_preserves_cloud_geometry_but_removes_imu_assistance():
    configs = {k: json.loads(v) for k, v in runtime_configs(manifest()).items()}
    configs["sensors.json"] = {"sensors": {"global_shutter_lidar": True, "T_lidar_imu": [1, 2, 3]}}
    original = json.loads(json.dumps(configs))
    ct = {"odometry_estimation": {"so_name": "libodometry_estimation_ct.so"}}
    candidate = lidar_only_configs(configs, ct)
    assert configs == original
    assert candidate["sensors.json"] == original["sensors.json"]
    assert candidate["ros.json"]["abs_constraint"]["enable"] is False
    ros = candidate["ros.json"]["glim_ros"]
    assert ros["extension_modules"] == ["librevision_module.so"]
    assert not ros["enable_global_mapping"] and not ros["enable_local_mapping"]
    assert ros["imu_frame_id"] == "vlp16"
    assert candidate["config.json"]["global"]["config_odometry"] == "config_odometry_ct_trial.json"
    assert candidate["config_odometry_ct_trial.json"] == ct
    configs["sensors.json"]["sensors"]["global_shutter_lidar"] = False
    with pytest.raises(ValueError, match="rigid-scan"):
        lidar_only_configs(configs, ct)


@pytest.mark.parametrize("change", [None, "prepared_sha256", "probe_sha256", "library_sha256", "noise", "initialization"])
def test_ab_requires_same_binary_inputs_and_every_other_config(change):
    left = dict(mode="legacy", prepared="p", prepared_sha256="a", probe_sha256="b", library_sha256={"lib": "c"})
    right = dict(left, mode="physics")
    a = {"config_ros.json": {"glim_ros": {"imu_topic": "imu"}}, "config_sensors.json": {"noise": .005},
         "config_odometry.json": {"initialization": [0., 0., 0.]}}
    b = json.loads(json.dumps(a))
    b["config_ros.json"]["glim_ros"]["imu_topic"] = "imu_physics"
    if change in ("prepared_sha256", "probe_sha256", "library_sha256"): right[change] = "changed"
    if change == "noise": b["config_sensors.json"]["noise"] = .05
    if change == "initialization": b["config_odometry.json"]["initialization"][0] = .1
    if change:
        with pytest.raises(ValueError):
            validate_ab_settings(left, right, a, b)
    else:
        validate_ab_settings(left, right, a, b)
