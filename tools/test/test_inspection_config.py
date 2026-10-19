"""Read-only inspection must not silently turn into a legacy navigation run."""

from pathlib import Path
import importlib.util
import os
import re
import subprocess

import yaml
import pytest

ROOT = Path(__file__).resolve().parents[2]


def run_mocked_launcher(nodes="", query_exit=0, arguments=("start", "loop16"), launcher="run_selected_gui.sh",
                        execution_profile="default"):
    fixture = ROOT / "tools/test/fixtures/inspection_launcher_env.sh"
    env = dict(
        os.environ,
        PATH="",
        BASH_ENV=str(fixture),
        XAUTHORITY=str(fixture),
        INSPECTION_TEST_ROOT=str(ROOT),
        INSPECTION_TEST_NODES=nodes,
        INSPECTION_TEST_QUERY_EXIT=str(query_exit),
        SELECTED_EXECUTION_PROFILE=execution_profile,
    )
    return subprocess.run(
        ["/bin/bash", str(ROOT / "docker" / launcher), *arguments], env=env, capture_output=True, text=True
    )


@pytest.mark.parametrize("profile,route_name,accepted", [
    ("default", "loop16", True), ("loop16_slow_host_v1", "loop16", True),
    ("loop16_slow_host_v1", "straight_slow", False), ("unbounded", "loop16", False),
    ("turn45_bounded_v1", "turn45", True), ("turn45_bounded_v1", "turn15", False),
    ("default", "turn45", True),
])
def test_headless_execution_profile_is_explicit_and_route_scoped(profile, route_name, accepted):
    result = run_mocked_launcher("/terrain_shadow_capture", arguments=("start", route_name),
                                launcher="run_selected_headless.sh", execution_profile=profile)
    assert (result.returncode == 0) is accepted
    assert ("TEST_SIMULATOR_START" in result.stdout) is accepted


@pytest.mark.parametrize("profile,route_name,accepted", [
    ("default", "turn45", True), ("turn45_bounded_v1", "turn45", True),
    ("turn45_bounded_v1", "turn15", False), ("unbounded", "turn45", False),
])
def test_gui_bounded_turn_keeps_profile_and_recorder_guards(profile, route_name, accepted):
    result = run_mocked_launcher("/terrain_shadow_capture", arguments=("start", route_name),
                                execution_profile=profile)
    assert (result.returncode == 0) is accepted
    assert ("TEST_SIMULATOR_START" in result.stdout) is accepted
    if accepted:
        missing = run_mocked_launcher(arguments=("start", route_name), execution_profile=profile)
        assert missing.returncode == 1 and "TEST_SIMULATOR_START" not in missing.stdout
    env = yaml.safe_load((ROOT / "docker-compose.inspect.yml").read_text())["services"]["astrobot-sim"]["environment"]
    assert env["OMNILRS_CAPTURE_EXECUTION_PROFILE"] == "${SELECTED_EXECUTION_PROFILE:-default}"
    assert env["OMNILRS_IMPORT_STAT_CACHE"] == "${INSPECT_IMPORT_STAT_CACHE:-true}"


@pytest.mark.parametrize(
    "nodes,count",
    [
        ("", 0),
        ("/inspection_bridge\n/terrain_shadow_capture\n/other_node", 1),
        ("/terrain_shadow_capture\n/terrain_shadow_capture", 2),
        ("/other/terrain_shadow_capture\n/terrain_shadow_capture_extra", 0),
    ],
)
def test_recorder_count_without_host_search_utilities(nodes, count):
    result = run_mocked_launcher(nodes)
    assert "command not found" not in result.stderr
    assert result.returncode == (0 if count == 1 else 1)
    assert ("TEST_SIMULATOR_START" in result.stdout) == (count == 1)
    if count != 1:
        assert f"found {count}" in result.stderr


def test_node_query_failure_is_not_reported_as_missing_recorder():
    result = run_mocked_launcher("/terrain_shadow_capture", query_exit=3)
    assert result.returncode == 1
    assert "Could not query ROS nodes" in result.stderr
    assert "TEST_SIMULATOR_START" not in result.stdout


@pytest.mark.parametrize("route", ["loop16", "turn15", "straight_slow", "combined10"])
def test_declared_routes_keep_recorder_gate(route):
    ready = run_mocked_launcher("/terrain_shadow_capture", arguments=("start", route))
    assert ready.returncode == 0
    assert "TEST_SIMULATOR_START" in ready.stdout
    missing = run_mocked_launcher(arguments=("start", route))
    assert missing.returncode == 1
    assert "TEST_SIMULATOR_START" not in missing.stdout


@pytest.mark.parametrize(
    "nodes,code", [("", 1), ("/terrain_shadow_capture", 0), ("/terrain_shadow_capture\n/terrain_shadow_capture", 1)]
)
def test_headless_keeps_recorder_gate(nodes, code):
    result = run_mocked_launcher(nodes, launcher="run_selected_headless.sh")
    assert result.returncode == code
    assert ("TEST_SIMULATOR_START" in result.stdout) == (code == 0)


def test_headless_configuration_keeps_corrected_sensor_and_gt_separation():
    env = yaml.safe_load((ROOT / "docker-compose.selected-headless.yml").read_text())["services"]["astrobot-sim"][
        "environment"
    ]
    assert env["OMNILRS_HEADLESS"] == "true" and env["DISPLAY"] == ""
    assert env["OMNILRS_DISABLE_VIEWPORT_UPDATES"] == "true"
    assert env["OMNILRS_IMPORT_STAT_CACHE"] == "${SELECTED_IMPORT_STAT_CACHE:-true}"
    assert env["OMNILRS_SELECTED_EXPERIENCE"] == "${SELECTED_ISAAC_EXPERIENCE:-default}"
    assert env["ASTROBOT_SELECTED_LIDAR_PROFILE"] == "selected_optical_v2"
    assert env["OMNILRS_TERRAIN_NATIVE_PUBLISH"] == "true"
    for name in ("GT_TF_ENABLED", "GT_MAP_ENABLED", "CAMERA_ENABLED", "TERRAIN_SHADOW"):
        assert env["OMNILRS_" + name] == "false"
    assert "XAUTHORITY" not in env


def test_viewport_setting_is_explicit_headless_only_and_preserves_gui_default():
    path = ROOT / "OmniLRS/src/configurations/rendering_confs.py"
    spec = importlib.util.spec_from_file_location("viewport_rendering_config", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    values = yaml.safe_load((ROOT / "OmniLRS/cfg/rendering/ray_tracing.yaml").read_text())["renderer"]
    assert module.RendererConf(**values).disable_viewport_updates is False
    assert module.RendererConf(**values).path_stat_cache_enabled is True
    assert module.RendererConf(**dict(values, path_stat_cache_enabled=False)).path_stat_cache_enabled is False
    with pytest.raises(AssertionError, match="boolean"):
        module.RendererConf(**dict(values, path_stat_cache_enabled="false"))
    with pytest.raises(AssertionError, match="headless"):
        module.RendererConf(**dict(values, disable_viewport_updates=True))
    assert module.RendererConf(**dict(values, headless=True, disable_viewport_updates=True)).disable_viewport_updates
    with pytest.raises(AssertionError, match="boolean"):
        module.RendererConf(**dict(values, headless=True, disable_viewport_updates="true"))
    launcher = (ROOT / "docker/run_omnilrs.sh").read_text()
    assert "++rendering.renderer.disable_viewport_updates=${OMNILRS_DISABLE_VIEWPORT_UPDATES:-false}" in launcher
    assert "++rendering.renderer.path_stat_cache_enabled=${OMNILRS_IMPORT_STAT_CACHE:-true}" in launcher
    assert (
        "self.world.step(render=True)"
        in (ROOT / "OmniLRS/src/environments_wrappers/ros2/simulation_manager_ros2.py").read_text()
    )


def test_headless_rejects_undeclared_route_and_discovery_failure():
    invalid = run_mocked_launcher(arguments=("start", "loop100"), launcher="run_selected_headless.sh")
    assert invalid.returncode == 2 and "TEST_SIMULATOR_START" not in invalid.stdout
    failed = run_mocked_launcher("/terrain_shadow_capture", query_exit=3, launcher="run_selected_headless.sh")
    assert failed.returncode == 1 and "Could not query ROS nodes" in failed.stderr


def test_pasted_log_command_is_rejected_before_preflight():
    result = run_mocked_launcher(
        "/terrain_shadow_capture", arguments=("start", "loop16", "docker", "logs", "-f", "astrobot-sim-lunaryard")
    )
    assert result.returncode == 2
    assert "separate" in result.stderr
    assert "GUI preflight passed" not in result.stdout
    assert "TEST_SIMULATOR_START" not in result.stdout


def test_bridge_is_loopback_read_only_and_bounded():
    overlay = yaml.safe_load((ROOT / "docker-compose.inspect.yml").read_text())
    bridge = overlay["services"]["inspection-bridge"]
    assert bridge["ports"] == ["127.0.0.1:8766:8766"]
    assert "depends_on" not in bridge
    assert bridge["restart"] == "no"
    params = yaml.safe_load((ROOT / "config/foxglove_inspection.yaml").read_text())["/**"]["ros__parameters"]
    assert params["capabilities"] == ["connectionGraph"]
    assert params["num_threads"] == 2 and params["max_qos_depth"] == 2
    for key in ("client_topic_whitelist", "service_whitelist", "param_whitelist", "asset_uri_allowlist"):
        assert params[key] == ["a^"]


def test_current_topics_exposed_without_legacy_substitution():
    params = yaml.safe_load((ROOT / "config/foxglove_inspection.yaml").read_text())["/**"]["ros__parameters"]
    for topic in (
        "/tf",
        "/tf_static",
        "/clock",
        "/astrobot_0/lidar_selected/points",
        "/astrobot_0/slam_raw/revisions",
        "/astrobot_0/slam_raw/aligned_points_corrected",
        "/astrobot_0/slam/aligned_points_corrected",
        "/gt/base_link_pose",
        "/astrobot_0/imu_physics/clock",
        "/terrain_shadow/request",
        "/terrain_shadow/grant",
    ):
        assert any(re.fullmatch(pattern, topic) for pattern in params["topic_whitelist"])
    assert not any(re.fullmatch(pattern, "/astrobot_0/slam/bev_costmap") for pattern in params["topic_whitelist"])


def test_gui_keeps_selected_protocol_and_gt_separation():
    env = yaml.safe_load((ROOT / "docker-compose.inspect.yml").read_text())["services"]["astrobot-sim"]["environment"]
    for name in ("HEADLESS", "GT_TF_ENABLED", "GT_MAP_ENABLED", "TERRAIN_SHADOW"):
        assert env["OMNILRS_" + name] == "false"
    assert env["OMNILRS_TERRAIN_NATIVE_PUBLISH"] == "${INSPECT_TERRAIN_V2:-false}"
    assert env["ASTROBOT_SELECTED_LIDAR_PROFILE"] == "${ASTROBOT_SELECTED_LIDAR_PROFILE:-selected_generic_v1}"
    for name in ("NATURAL_HOLDOUT", "SELECTED_LIDAR", "PHYSICS_IMU", "ROCKS_ENABLED"):
        assert env["OMNILRS_" + name] == "true"
    assert env["OMNILRS_ENVIRONMENT_SEED"] == "43"
    assert env["OMNILRS_NATURAL_MOTION"] == "${INSPECT_ROUTE:-loop16}"
    assert env["XAUTHORITY"] == "/run/inspection.xauthority"


def test_start_helper_rejects_unknown_protocol_before_any_launch():
    result = subprocess.run(
        ["bash", str(ROOT / "docker/run_selected_gui.sh"), "start", "free_drive"], capture_output=True, text=True
    )
    assert result.returncode == 2
    assert "Only declared" in result.stderr


def test_start_helper_rejects_unknown_action():
    result = subprocess.run(["bash", str(ROOT / "docker/run_selected_gui.sh"), "reset"], capture_output=True, text=True)
    assert result.returncode == 2
    assert "Usage:" in result.stderr


@pytest.mark.parametrize("authorized", [False, True])
def test_container_x11_probe_requires_authorization_and_closes_connection(authorized):
    spec = importlib.util.spec_from_file_location("check_x11", ROOT / "docker/check_x11.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class Function:
        def __init__(self, value):
            self.value, self.calls = value, []

        def __call__(self, argument):
            self.calls.append(argument)
            return self.value

    class Library:
        XOpenDisplay = Function(123 if authorized else None)
        XCloseDisplay = Function(0)

    if authorized:
        assert module.probe_x11(lambda _: Library)["container_x11_authorized"]
        assert Library.XCloseDisplay.calls == [123]
    else:
        with pytest.raises(RuntimeError, match="Cannot open DISPLAY"):
            module.probe_x11(lambda _: Library)
        assert not Library.XCloseDisplay.calls
