"""Host NumPy/SciPy contracts; no simulator, ROS, GPU or motion authority."""
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "astrobot-lab/src/astrobot_core"))
from natural_motion import (PATTERNS, ground_sample, motion_command, route, route_hash,
                            seed_from_request, survey_points, validate_ground_tile,
                            validate_motion_request, verify_live_start)
from astrobot_core.shadow_seed import validate_shadow_seed
from natural_motion import execution_budget, validate_execution_budget, leg_deadline


def test_survey_lookup_exact_argmin_boundaries_ties_and_immutable_input():
    from natural_motion import SurveyLookup
    from scipy.spatial.transform import Rotation
    rng = np.random.default_rng(411)
    for angle in (0., .37, -1.8):
        origin = np.eye(4)
        origin[:3, :3] = Rotation.from_euler("z", angle).as_matrix()
        points, _ = survey_points(origin, "loop16")
        saved = points.copy()
        lookup = SurveyLookup(points)
        low, high = points.min(axis=0), points.max(axis=0)
        queries = np.vstack([rng.uniform(low, high, size=(300, 2)), points[::37], low, high])
        points[:] = 1e9  # Subsequent caller mutation cannot change a frozen survey.
        for xy in queries:
            assert lookup.nearest(xy) == int(np.argmin(np.sum((saved-xy)**2, axis=1)))
        for xy in (low-[1e-12, 0], high+[0, 1e-12]):
            with pytest.raises(ValueError, match="left surveyed"):
                lookup.nearest(xy)
    assert SurveyLookup([[-1, 0], [1, 0], [-1, 0]]).nearest([0, 0]) == 0
    for points in ([], [[1, 2, 3]], [[np.nan, 0]]):
        with pytest.raises(ValueError, match="surveyed floor"):
            SurveyLookup(points)
    for xy in ([np.nan, 0], [0], [[0, 0]]):
        with pytest.raises(ValueError, match="floor query"):
            lookup.nearest(xy)


def request():
    world = np.eye(4)
    world[:3, 3] = [20., 18., .12]
    imu = np.eye(4)
    imu[:3, 3] = [.1, -.02, .2]
    return dict(schema=1, seed=43, pitch_deg=5., home_xyz=[-20., -20., 0.],
                capture="revision-"+"a"*32, pattern="straight_slow", motion_authority=True,
                route_sha256=route_hash("straight_slow"), world_from_base=world.tolist(),
                base_from_imu=imu.tolist(), base_from_lidar=np.eye(4).tolist(), seed_stamp=2.)


def test_explicit_slow_host_budget_keeps_legacy_route_and_all_leg_guards():
    original_route, original_hash = route("loop16"), route_hash("loop16")
    value = request()
    value.update(pattern="loop16", route_sha256=original_hash)
    validate_motion_request(value)
    assert validate_execution_budget(value) == dict(profile="default", max_wall_s=1800.0)
    value["execution_budget"] = execution_budget("loop16", "loop16_slow_host_v1")
    validate_motion_request(value)
    assert validate_execution_budget(value, "loop16_slow_host_v1")["max_wall_s"] == 3600
    assert route("loop16") == original_route and route_hash("loop16") == original_hash
    assert [leg_deadline("loop16", i) for i in range(8)] == [85, 75] * 4
    with pytest.raises(ValueError, match="execution profiles differ"):
        validate_execution_budget(value, "default")
    value.pop("execution_budget")
    with pytest.raises(ValueError, match="execution profiles differ"):
        validate_execution_budget(value, "loop16_slow_host_v1")


@pytest.mark.parametrize("declared", [
    {}, "loop16_slow_host_v1", dict(profile="unknown", max_wall_s=3600),
    dict(profile="loop16_slow_host_v1", max_wall_s=3601),
    dict(profile="loop16_slow_host_v1", max_wall_s=float("nan")),
    dict(profile="loop16_slow_host_v1", max_wall_s=3600, heartbeat_s=5),
])
def test_undeclared_execution_budget_rejected(declared):
    value = request()
    value.update(pattern="loop16", route_sha256=route_hash("loop16"), execution_budget=declared)
    with pytest.raises(ValueError):
        validate_motion_request(value)


def test_slow_host_profile_is_not_permission_to_extend_other_routes():
    for pattern in ("", "straight_slow", "turn15", "combined10"):
        with pytest.raises(ValueError, match="execution profile"):
            execution_budget(pattern, "loop16_slow_host_v1")


def test_gc_stress_profile_is_explicit_short_route_only_and_cannot_change_injection():
    budget = execution_budget("turn15", "turn15_gc_stress_v1")
    assert budget == dict(profile="turn15_gc_stress_v1", max_wall_s=900.0,
                          diagnostic_only=True, forced_gc_after_native_ticks=[240, 480, 720],
                          storage_check_delay_s=.11, archive_limit_gib=32)
    value = request()
    value.update(pattern="turn15", route_sha256=route_hash("turn15"), execution_budget=budget)
    validate_motion_request(value)
    with pytest.raises(ValueError, match="profiles differ"):
        validate_execution_budget(value, "default")
    for pattern in ("loop16", "straight_slow", "", "turn45"):
        with pytest.raises(ValueError, match="execution profile"):
            execution_budget(pattern, "turn15_gc_stress_v1")
    value["execution_budget"]["forced_gc_after_native_ticks"] = [241]
    with pytest.raises(ValueError, match="differs"):
        validate_motion_request(value)


def test_bounded_turn_profile_keeps_pure_yaw_and_original_guards():
    from costmap_scenarios import archive_limit
    original = route_hash("turn45")
    budget = execution_budget("turn45", "turn45_bounded_v1")
    assert budget == dict(profile="turn45_bounded_v1", max_wall_s=900.0, archive_limit_gib=16)
    value = request()
    value.update(pattern="turn45", route_sha256=original, execution_budget=budget)
    validate_motion_request(value)
    assert validate_execution_budget(value, "turn45_bounded_v1") == budget
    assert archive_limit("turn45", "turn45_bounded_v1") == 16 * 1024**3
    assert archive_limit("turn45") == 64 * 1024**3
    assert route_hash("turn45") == original
    for i in range(3):
        current = np.eye(4)
        current[0, 3] = .20
        assert motion_command(np.eye(4), current, "turn45", i)[0] == 0
        current[0, 3] = .251
        with pytest.raises(ValueError, match="25 cm"):
            motion_command(np.eye(4), current, "turn45", i)
    with pytest.raises(ValueError, match="profiles differ"):
        validate_execution_budget(value, "default")
    with pytest.raises(ValueError, match="profile"):
        execution_budget("turn15", "turn45_bounded_v1")
    with pytest.raises(ValueError, match="requires turn45"):
        archive_limit("turn15", "turn45_bounded_v1")


def test_live_seed_converts_world_base_to_fixed_map_imu_not_base_or_lidar():
    value = seed_from_request(request())
    np.testing.assert_allclose(value["pose"][:3], [40.1, 37.98, .32])
    assert validate_shadow_seed(value, True, 3.) == value["pose"]
    for shadow, now in [(False, 3.), (True, 1.), (True, 33.)]:
        with pytest.raises(ValueError):
            validate_shadow_seed(value, shadow, now)


def test_live_first_output_requires_measurement_time_stationary_window_and_frame_zero():
    req = request()
    samples = [(round(i*1e9/30), np.array(req["world_from_base"])) for i in range(31, 100)]
    pose = np.array(req["world_from_base"]) @ np.array(req["base_from_imu"])
    pose[:3, 3] -= req["home_xyz"]
    frame = dict(frame_id=0, stamp_ns=3_000_000_000, map_from_imu=pose.tolist())
    assert verify_live_start(req, frame, samples)["passed"]
    frame["map_from_imu"][2][3] += .03
    assert not verify_live_start(req, frame, samples)["passed"]
    frame["frame_id"] = 1
    with pytest.raises(ValueError, match="not recorded"):
        verify_live_start(req, frame, samples)
    frame["frame_id"] = 0
    with pytest.raises(ValueError, match="stationary window"):
        verify_live_start(req, frame, samples[-2:])


@pytest.mark.parametrize("pattern", PATTERNS)
def test_routes_are_bounded_and_full_survey_contains_turn_envelope(pattern):
    spec = route(pattern)
    from costmap_scenarios import SCENARIOS
    from natural_motion import WAYPOINT_LOOPS
    # transit50 runs at Nav2's MPPI vx_max (0.18), the declared speed-realistic route.
    assert spec["speed"] <= (.18 if pattern == "transit50" else .15) and spec["angular_speed"] == .1
    assert tuple(spec["targets"][-1]) == ((0., 0., 0., "rotate") if pattern in WAYPOINT_LOOPS else (0., 0.))
    points, shape = survey_points(np.eye(4), pattern)
    assert len(points) == math.prod(shape)
    assert np.all(points.min(axis=0) <= (-.9 if pattern == "straight_slow" else -1.35))
    normals = np.tile([0., 0., 1.], (len(points), 1))
    assert validate_ground_tile(points, np.zeros(len(points)), shape, normals)["geometry_only"]
    bad = np.zeros(shape)
    bad[1, 1] = .03
    with pytest.raises(ValueError, match="discontinuity"):
        validate_ground_tile(points, bad.ravel(), shape, normals)


def test_request_cannot_change_route_dose_or_insert_unknown_pattern():
    value = request()
    value["pattern"] = "combined15"
    with pytest.raises(ValueError):
        validate_motion_request(value)
    value["pattern"] = "freeform"
    with pytest.raises(ValueError):
        validate_motion_request(value)


def floor_fixture(slope=(0., 0.)):
    xy = np.array([(x, y) for x in np.arange(21)*.05 for y in np.arange(21)*.05])
    z = xy@np.asarray(slope)
    normal = np.array([-slope[0], -slope[1], 1.])
    normals = np.tile(normal/np.linalg.norm(normal), (len(xy), 1))
    return xy, z, (21, 21), normals


@pytest.mark.parametrize("slope", [(0., 0.), (.3, 0.), (0., -.3), (.2, -.2)])
def test_smooth_incline_is_not_a_height_discontinuity(slope):
    result = validate_ground_tile(*floor_fixture(slope))
    assert result["maximum_relative_discontinuity_m"] < 1e-12
    assert result["maximum_detrended_roughness_m"] < 1e-12
    assert result["maximum_patch_incline"] == pytest.approx(np.linalg.norm(slope))


@pytest.mark.parametrize("slope", [(0., 0.), (.3, 0.), (0., -.3)])
@pytest.mark.parametrize("height", [-.08, .08])
def test_eight_cm_step_or_drop_remains_unsafe_on_sloping_ground(slope, height):
    xy, z, shape, normals = floor_fixture(slope)
    z[xy[:, 0] >= .5] += height
    with pytest.raises(ValueError, match="relative discontinuity"):
        validate_ground_tile(xy, z, shape, normals)


def test_smooth_but_excessive_incline_is_still_rejected_separately():
    with pytest.raises(ValueError, match="absolute incline"):
        validate_ground_tile(*floor_fixture((.36, 0.)))


def test_slope_transition_and_roughness_are_not_averaged_away():
    xy, z, shape, normals = floor_fixture()
    z = .03*np.sin(2*np.pi*xy[:, 0]/.7)
    derivative = .03*2*np.pi/.7*np.cos(2*np.pi*xy[:, 0]/.7)
    normals[:, 0] = -derivative
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    with pytest.raises(ValueError, match="roughness"):
        validate_ground_tile(xy, z, shape, normals)


def test_floor_normals_missing_or_grid_permuted_fail_closed():
    xy, z, shape, normals = floor_fixture()
    with pytest.raises(ValueError, match="floor tile"):
        validate_ground_tile(xy, z, shape, normals[:-1])
    with pytest.raises(ValueError, match="ordered grid"):
        validate_ground_tile(xy[::-1], z, shape, normals)


@pytest.mark.parametrize("hits", [[], [(1., [0,0,0], [0,0,1], "/Lunaryard/Rocks/a")],
                                     [(1., [0,0,0], [0,0,.5], "/Lunaryard/Terrain/a")],
                                     [(1., [0,0,float("nan")], [0,0,1], "/Lunaryard/Terrain/a")]])
def test_ground_requires_actual_first_floor_hit_with_safe_normal(hits):
    with pytest.raises(ValueError):
        ground_sample(hits)


def test_motion_is_signed_bounded_and_rejects_lateral_skid():
    origin = np.eye(4)
    linear, angular, arrived = motion_command(origin, origin, "straight_slow", 0)
    assert (linear, angular, arrived) == (.05, 0., False)
    current = origin.copy()
    current[0, 3] = 2.
    assert motion_command(origin, current, "straight_slow", 1)[0] == -.05
    current[1, 3] = .081
    with pytest.raises(ValueError, match="envelope"):
        motion_command(origin, current, "straight_slow", 1)
    current = origin.copy()
    current[0, 3] = .251
    with pytest.raises(ValueError, match="skid"):
        motion_command(origin, current, "turn15", 0)


@pytest.mark.parametrize("error", [-.3, -.04, -.01201, 0., .01199, .01201, .04, .3])
def test_turn_controller_keeps_drive_above_deadband_without_widening_arrival(error):
    from scipy.spatial.transform import Rotation
    origin, current = np.eye(4), np.eye(4)
    current[:3, :3] = Rotation.from_euler("z", math.radians(15)-error).as_matrix()
    linear, angular, arrived = motion_command(origin, current, "turn15", 0)
    assert linear == 0. and arrived == (abs(error) < .012)
    if arrived:
        assert angular == 0.
    else:
        assert .04 <= abs(angular) <= .1
        assert angular*error > 0.


@pytest.mark.parametrize("sign", [-1., 1.])
def test_turn_endpoint_converges_with_lossy_drive_and_deadband(sign):
    # Synthetic actuator regression, not a simulator/traction qualification.
    from scipy.spatial.transform import Rotation
    origin, current = np.eye(4), np.eye(4)
    target = math.radians(15)
    angle = target-sign*.02
    for _ in range(90):
        current[:3, :3] = Rotation.from_euler("z", angle).as_matrix()
        _, command, arrived = motion_command(origin, current, "turn15", 0)
        if arrived:
            break
        angle += math.copysign(max(0., abs(command)-.008)*.3/30., command)
    assert arrived and abs(target-angle) < .012


@pytest.mark.parametrize("pattern", PATTERNS)
def test_invalid_pose_never_produces_motion(pattern):
    current = np.eye(4)
    current[0, 0] = np.nan
    with pytest.raises(ValueError):
        motion_command(np.eye(4), current, pattern, 0)


@pytest.mark.parametrize("yaw", [0., .25, 1.2, -1.7])
def test_straight_survey_contains_every_admitted_footprint_corner(yaw):
    from scipy.spatial.transform import Rotation
    from natural_motion import HALF
    origin = np.eye(4)
    origin[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    points, _ = survey_points(origin, "straight_slow")
    for heading in np.linspace(yaw-.15, yaw+.15, 61):
        r = Rotation.from_euler("z", heading).as_matrix()[:2, :2]
        for d in (-.1, 2.1):
            for lateral in (-.08, .08):
                center = origin[:2, :2] @ [d, lateral]
                for x in (-HALF[0], HALF[0]):
                    for y in (-HALF[1], HALF[1]):
                        xy = center+r@[x, y]
                        assert np.all(xy >= points.min(axis=0)) and np.all(xy <= points.max(axis=0))


def test_watchdog_false_lease_stops_without_waiting_for_simulator_tick():
    from types import SimpleNamespace
    import threading
    import time
    path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/natural_motion_drive.py"
    spec = importlib.util.spec_from_file_location("motion_driver_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    obj = module.NaturalMotionDrive.__new__(module.NaturalMotionDrive)
    obj.closed = threading.Event()
    obj.armed, obj.last_wall, obj.failure = True, time.monotonic(), None
    obj.owner = SimpleNamespace(stamp=1., lease=SimpleNamespace(ready=lambda *args: False))
    obj.lock = threading.RLock()
    stopped = []
    obj.publish = lambda: (stopped.append(True), obj.closed.set())
    obj.watchdog()
    assert stopped and obj.failure == "motion lease expired"


def test_heap_checkpoint_only_before_lease_and_does_not_change_gc_configuration(monkeypatch):
    import gc
    path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/natural_motion_drive.py"
    spec = importlib.util.spec_from_file_location("heap_checkpoint_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []
    monkeypatch.setattr(gc, "collect", lambda generation: calls.append(generation) or 3)
    extension_type = type("ExtensionLike", (), {"__module__": property(lambda self: "extension")})
    monkeypatch.setattr(gc, "get_objects", lambda: [extension_type()])
    thresholds, enabled = gc.get_threshold(), gc.isenabled()
    result = module.stationary_heap_checkpoint(-1, False, None)
    assert calls == [2] and result["collected"] == 3 and result["gc_enabled"]
    assert gc.get_threshold() == thresholds and gc.isenabled() == enabled
    for args in [(0, False, None), (-1, True, None), (-1, False, 0.)]:
        with pytest.raises(ValueError, match="pre-lease"):
            module.stationary_heap_checkpoint(*args)
    assert calls == [2]


@pytest.mark.parametrize("fault", [None, "seed", "runtime", "clearance", "target", "command", "preroll", "postroll"])
def test_motion_artifact_audit_rejects_changed_or_incomplete_evidence(tmp_path, fault):
    import hashlib
    import json
    from natural_motion import audit_motion_artifacts
    capture, recorder = tmp_path / "capture", tmp_path / "recorder"
    capture.mkdir(); recorder.mkdir()
    req = request()
    origin = np.asarray(req["world_from_base"])
    end = origin.copy(); end[0, 3] += 2.
    seed = seed_from_request(req)
    configs = {"config.json": {"global": {"config_odometry": "odom.json"}},
               "odom.json": {"odometry_estimation": {"init_T_world_imu": seed["pose"].copy()}}}
    (capture / "clearance_tile.npz").write_bytes(b"hash-only-unit-fixture")
    clearance = dict(passed=True, pattern=req["pattern"], origin=origin.tolist(),
                     tile_sha256=hashlib.sha256(b"hash-only-unit-fixture").hexdigest())
    complete = dict(last_stamp=12., reached=[dict(index=0, stamp=4., world_from_base=end.tolist()),
                                           dict(index=1, stamp=8., world_from_base=origin.tolist())])
    rows = [dict(stamp=3., index=0, linear=.05, angular=0., world_from_base=origin.tolist()),
            dict(stamp=5., index=1, linear=-.05, angular=0., world_from_base=end.tolist())]
    if fault == "seed": seed["pose"][2] += .1
    if fault == "runtime": configs["odom.json"]["odometry_estimation"]["init_T_world_imu"][0] += .1
    if fault == "clearance": clearance["tile_sha256"] = "changed"
    if fault == "target": complete["reached"].pop()
    if fault == "command": rows[0]["linear"] = .051
    if fault == "preroll": rows[0]["stamp"] = 2.9
    if fault == "postroll": complete["last_stamp"] = 10.
    (capture / "clearance.json").write_text(json.dumps(clearance))
    (capture / "motion.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
    (recorder / "live_gt_seed.json").write_text(json.dumps(seed))
    (recorder / "initial_output_check.json").write_text(json.dumps(dict(passed=True, ongoing_gt=False)))
    if fault:
        with pytest.raises(ValueError):
            audit_motion_artifacts(capture, recorder, req, complete, dict(stamp=0.), configs)
    else:
        assert len(audit_motion_artifacts(capture, recorder, req, complete, dict(stamp=0.), configs)) == 5
