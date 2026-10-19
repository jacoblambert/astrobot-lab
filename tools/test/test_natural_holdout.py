"""Host NumPy/SciPy acquisition contracts; no simulator or ROS required."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from natural_holdout import validate_request, stationary_error, validate_stationary_native, seal_capture, validate_frozen_candidate
from shadow_capture_gate import validate_shadow_geometry
from natural_holdout import validate_gt_snapshots
from natural_holdout import validate_loaded_dependencies


@pytest.mark.parametrize("failed_flush", [False, True])
def test_moving_completion_is_invisible_until_fully_written(tmp_path, monkeypatch, failed_flush):
    import importlib
    import json
    from types import SimpleNamespace
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[2] / "OmniLRS"))
    module = importlib.import_module("src.environments_wrappers.ros2.natural_holdout_capture")
    stopped = []
    driver = SimpleNamespace(stop=lambda: stopped.append(True), reached=[dict(index=1)])
    original_dump = json.dump

    def slow_dump(payload, stream, **kwargs):
        # Reproduce the old race deterministically: a reader runs in the middle
        # of serialization, before even the first JSON byte has been written.
        target = tmp_path / Path(stream.name).name.removesuffix(".pending")
        assert not target.exists()
        return original_dump(payload, stream, **kwargs)

    monkeypatch.setattr(json, "dump", slow_dump)

    def close():
        assert stopped == [True]
        assert json.loads((tmp_path / "acquisition_finished.json").read_text())["commands_stopped"]
        assert not (tmp_path / "natural_holdout.json").exists()
        if failed_flush:
            raise OSError("fixture seal failure")

    native = SimpleNamespace(close=close, frames=[None] * 7)
    if failed_flush:
        with pytest.raises(OSError, match="fixture seal failure"):
            module.seal_moving_capture(tmp_path, {"pattern": "straight_slow"}, 1., 8., native, driver)
        assert not (tmp_path / "natural_holdout.json").exists()
    else:
        module.seal_moving_capture(tmp_path, {"pattern": "straight_slow"}, 1., 8., native, driver)
        result = json.loads((tmp_path / "natural_holdout.json").read_text())
        assert result["complete"] and result["chunks"] == 7 and result["reached"] == driver.reached
        with pytest.raises(FileExistsError):
            module.seal_moving_capture(tmp_path, {}, 1., 8., native, driver)


def request():
    return dict(schema=1, pattern="natural_stationary", seed=43, pitch_deg=5., duration_s=30.,
                home_xyz=[-20., -20., 0.], motion_authority=False,
                capture="revision-" + "a" * 32,
                base_from_imu=np.eye(4).tolist(), base_from_lidar=np.eye(4).tolist())


def test_loaded_shared_core_cannot_change_behind_a_frozen_plugin():
    reference = {"/installed/libglim.so": "core", "/frozen/plugin.so": "plugin", "/ros/librosbag2_cpp.so": "reader"}
    loaded = [dict(path=p, sha256=h) for p, h in reference.items() if "rosbag2" not in p]
    assert validate_loaded_dependencies(loaded, reference)["matching_libraries"] == 2
    loaded[0]["sha256"] = "new core"
    with pytest.raises(ValueError, match="changed"):
        validate_loaded_dependencies(loaded, reference)
    with pytest.raises(ValueError, match="missing"):
        validate_loaded_dependencies(loaded[1:], reference)


@pytest.mark.parametrize("fault", [None, "gap", "frame", "extrinsic", "duplicate", "rotation"])
def test_published_gt_matches_every_native_tick_without_fitting(fault):
    frames = [dict(observed_stamp=1., world_from_lidar=np.eye(4).tolist())]
    times = np.array([1000000000, 1033333333])
    poses = np.tile(np.eye(4), (2, 1, 1))
    mount = np.eye(4)
    if fault == "gap": times += 1000000
    if fault == "frame": poses[0, 2, 3] = .002
    if fault == "extrinsic": mount[0, 3] = .4
    if fault == "duplicate": times[1] = times[0]
    if fault == "rotation":
        from scipy.spatial.transform import Rotation
        poses[0, :3, :3] = Rotation.from_euler("z", .001).as_matrix()
    if fault:
        with pytest.raises(ValueError): validate_gt_snapshots(frames, times, poses, mount)
    else:
        result = validate_gt_snapshots(frames, times-1, poses, mount)
        assert result["pairs"] == 1 and result["maximum_position_m"] == 0.


def test_failure_marker_precedes_fallible_archive_cleanup(tmp_path):
    import importlib.util
    import json
    from types import SimpleNamespace
    path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/natural_holdout_capture.py"
    spec = importlib.util.spec_from_file_location("natural_capture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    capture = module.NaturalHoldoutCapture.__new__(module.NaturalHoldoutCapture)
    capture.done, capture.run, capture.phase = False, tmp_path, "test"
    capture.failure = capture.selected_lidar = capture.sealing = None
    capture.max_tick_wall_gap = .8
    capture.lease = SimpleNamespace(sequence=4, sim_stamp=1., wall_stamp=None)
    def fail(stamp): raise ValueError("original failure")
    def cleanup():
        assert json.loads((tmp_path / "failure.json").read_text())["error"] == "original failure"
        raise OSError("flush failed")
    capture._tick = fail
    capture.close_diagnostics = cleanup
    capture.native = SimpleNamespace(close=cleanup)
    capture.tick(2.)
    assert capture.done
    assert json.loads((tmp_path / "failure.json").read_text())["error"] == "original failure"
    assert not (tmp_path / "natural_holdout.json").exists()


@pytest.mark.parametrize("fault", [None, "weight", "velocity", "missing_library", "wrong_library", "bad_quaternion"])
def test_natural_holdout_freezes_settings_and_loaded_code_except_declared_initial_pose(fault):
    import copy
    expected = {"config.json": {"global": {"config_odometry": "odom.json", "config_ros": "ros.json"}},
        "odom.json": {"odometry_estimation": {"so_name": "/frozen/cpu.so", "weight": 300.,
            "init_T_world_imu": [0, 0, 0, 0, 0, 0, 1], "init_v_world_imu": [0, 0, 0]}},
        "ros.json": {"glim_ros": {"extension_modules": ["/frozen/libclosed_window_abs.so"]}}}
    actual = copy.deepcopy(expected)
    actual["odom.json"]["odometry_estimation"]["init_T_world_imu"][0] = 40.
    hashes = {"/frozen/cpu.so": "a" * 64, "/frozen/libclosed_window_abs.so": "b" * 64}
    loaded = [dict(path=k, sha256=v) for k, v in hashes.items()]
    if fault == "weight": actual["odom.json"]["odometry_estimation"]["weight"] = 301.
    if fault == "velocity": actual["odom.json"]["odometry_estimation"]["init_v_world_imu"][0] = .1
    if fault == "missing_library": loaded.pop()
    if fault == "wrong_library": loaded[0]["sha256"] = "c" * 64
    if fault == "bad_quaternion": actual["odom.json"]["odometry_estimation"]["init_T_world_imu"][-1] = 2.
    if fault:
        with pytest.raises(ValueError): validate_frozen_candidate(actual, expected, loaded, hashes)
    else:
        validate_frozen_candidate(actual, expected, loaded, hashes)


def test_natural_fixture_is_explicit_and_cannot_unlock_calibration_or_motion():
    r = request()
    validate_request(r)
    validate_shadow_geometry(r, np.eye(4), np.eye(4), np.eye(4), np.eye(4), natural_stationary=True)
    with pytest.raises(ValueError):
        validate_shadow_geometry(r, np.eye(4), np.eye(4), np.eye(4), np.eye(4))
    for field, value in (("seed", 42), ("pitch_deg", 10.), ("duration_s", 20.), ("motion_authority", True)):
        with pytest.raises(ValueError):
            validate_request(dict(r, **{field: value}))


@pytest.mark.parametrize("failed_flush", [False, True])
def test_measurement_finish_precedes_archive_flush_but_success_never_does(tmp_path, failed_flush):
    class Native:
        frames = [None] * 901

        def close(self):
            assert (tmp_path / "acquisition_finished.json").exists()
            assert not (tmp_path / "natural_holdout.json").exists()
            if failed_flush:
                raise OSError("archive failure")

    if failed_flush:
        with pytest.raises(OSError):
            seal_capture(tmp_path, request(), 10., 40., Native())
        assert not (tmp_path / "natural_holdout.json").exists()
    else:
        seal_capture(tmp_path, request(), 10., 40., Native())
        assert (tmp_path / "natural_holdout.json").exists()
        with pytest.raises(FileExistsError):
            seal_capture(tmp_path, request(), 10., 40., Native())


def test_motion_is_not_hidden_by_fixed_pose_reanchoring():
    original = np.eye(4)
    moved = original.copy()
    moved[0, 3] = .006
    with pytest.raises(ValueError, match="moved"):
        stationary_error(original, moved)
    moved = original.copy()
    angle = np.deg2rad(.11)
    moved[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    with pytest.raises(ValueError, match="moved"):
        stationary_error(original, moved)


def native():
    surface = dict(selected=128, missing=0, residuals_m=[0.] * 128, nonhorizontal_surface_samples=2)
    return dict(duplicate_native_reads=0, frames=[dict(observed_stamp=10. + i / 30.,
        full_columns=1800, columns=600, configured_rotation_rate=10., world_time=i / 30.,
        timeline_time=i / 30., world_physics_dt=1/60., world_rendering_dt=1/30.,
        current_pose=dict(surface), previous_pose=dict(surface), world_from_lidar=np.eye(4).tolist())
        for i in range(901)])


def test_stationary_native_is_never_moving_or_terrain_acceptance():
    report = validate_stationary_native(native())
    assert report["stationary_only"] and not report["eligible_for_moving_replay"] and not report["acceptance"]


@pytest.mark.parametrize("fault", ["gap", "duration", "first_hit", "missing", "duplicate", "no_sides", "motion"])
def test_native_failure_cannot_become_holdout_evidence(fault):
    audit = native()
    if fault == "gap": del audit["frames"][50]
    if fault == "duration": audit["frames"] = audit["frames"][:-1]
    if fault == "first_hit": audit["frames"][50]["current_pose"]["residuals_m"] = [.0011]
    if fault == "missing": audit["frames"][50]["current_pose"]["missing"] = 1
    if fault == "duplicate": audit["duplicate_native_reads"] = 1
    if fault == "no_sides":
        for frame in audit["frames"]: frame["current_pose"]["nonhorizontal_surface_samples"] = 0
    if fault == "motion": audit["frames"][50]["world_from_lidar"][0][3] = .006
    with pytest.raises(ValueError):
        validate_stationary_native(audit)
