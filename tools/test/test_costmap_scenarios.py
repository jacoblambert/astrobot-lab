"""Small deterministic navigation plants, not Isaac/SLAM acceptance."""

import math
from functools import partial

import numpy as np
import pytest

from test_costmap_trial import advisory
from costmap_scenarios import SCENARIOS, check_position, scenario_spec
from local_terrain.costmap_trial import CostmapTrial, grid_route, validate_proposal, visible
from natural_motion import motion_command, route, survey_points
from selected_lidar_profile import acquisition_limits


def test_long_trial_archive_quota_is_stricter_not_a_relaxed_reserve():
    from costmap_scenarios import archive_limit

    assert archive_limit("costmap1") == archive_limit("loop16") == 64 * 1024**3
    assert archive_limit("turn15", "turn15_gc_stress_v1") == 32 * 1024**3
    for pattern in SCENARIOS:
        assert archive_limit(pattern) == 32 * 1024**3
        assert route(pattern)["archive_limit_gib"] == 32


def test_recorder_total_budget_matches_declared_route_without_changing_legacy():
    from costmap_scenarios import recorder_wall_maximum
    from natural_motion import execution_budget

    for pattern in ("costmap1", "loop16", *SCENARIOS):
        budget = execution_budget(pattern)
        assert recorder_wall_maximum(pattern, "default", budget) == (3600 if pattern in SCENARIOS else 1800)


def scene(stamp, xy, yaw=0, rock=False):
    a = advisory(stamp, *xy)
    c, s = math.cos(yaw), math.sin(yaw)
    a.context.local_from_base[:2, :2] = [[c, -s], [s, c]]
    a.record["origin"] = (np.floor((np.asarray(xy) - 3) / 0.1) * 0.1).tolist()
    a.layers[:, :, 0] = -1  # Explicit missing ground, not ground-truth free.
    if rock:
        y, x = np.indices(a.layers.shape[:2])
        xy_grid = np.stack([x, y], axis=-1) * 0.1 + np.asarray(a.record["origin"]) + 0.05
        a.layers[np.linalg.norm(xy_grid - [1.8, 0], axis=-1) <= 0.3, 0] = 100
    return a


@pytest.mark.parametrize("pattern", SCENARIOS)
def test_declared_routes_cannot_use_gt_controller_and_have_bounded_recording(pattern):
    spec = route(pattern)
    assert spec["targets"][-1] == [0, 0]
    assert spec["controller_uses_gt"] is False and spec["independent_gt_emergency_veto"]
    with pytest.raises(ValueError, match="GT waypoint"):
        motion_command(np.eye(4), np.eye(4), pattern, 0)
    assert acquisition_limits(pattern) == dict(max_duration_s=620, max_native_frames=38000)
    points, shape = survey_points(np.eye(4), pattern)
    assert len(points) == math.prod(shape)
    bounds = spec["position_bounds_initial_xy"]
    assert (points.min(axis=0) < np.asarray(bounds)[[0, 2]] - 1).all()
    assert (points.max(axis=0) > np.asarray(bounds)[[1, 3]] + 1).all()


@pytest.mark.parametrize("pattern", SCENARIOS)
def test_small_closed_loop_plant_visits_every_goal_and_returns_home(pattern):
    controller = CostmapTrial("c", scenario=pattern)
    xy, angle, dt = np.zeros(2), 0.0, 0.5
    home = None
    minimum_clearance = float("inf")
    for tick in range(1201):
        stamp = 1_000_000_000 + tick * 500_000_000
        a = scene(stamp, xy, angle, rock=pattern == "costmap_detour8")
        value = controller.update(a, stamp)
        linear, angular = validate_proposal(value, "c", ("s", 1), stamp, scenario=pattern)
        home = value["home_local_xy"] if home is None else home
        assert value["home_local_xy"] == home == [0, 0]
        minimum_clearance = min(minimum_clearance, np.linalg.norm(xy - [1.8, 0]))
        if value["terminal"]:
            break
        xy += linear * dt * np.array([math.cos(angle), math.sin(angle)])
        angle += angular * dt
    assert value["terminal"] == "estimated_goal_reached", value
    assert [v["index"] for v in value["reached_goals"]] == list(range(len(SCENARIOS[pattern])))
    assert np.linalg.norm(xy) <= 0.1
    assert value["goal_local_xy"] == [0, 0]
    if pattern == "costmap_detour8":
        assert minimum_clearance > math.hypot(0.85, 0.7) + 0.3


def test_home_uses_initial_planar_pose_not_current_heading_or_cell_selection():
    trial = CostmapTrial("c", scenario="costmap_square8")
    a = scene(1_000_000_000, [1, 2], math.pi / 2)
    trial.update(a, a.context.query_ns)
    a = scene(4_000_000_000, [1.03, 2], math.pi / 2 + 0.03)
    result = trial.update(a, a.context.query_ns)
    assert np.allclose(result["goals_local_xy"], [[1, 4], [3, 4], [3, 2], [1, 2]])


def test_goal_outside_rolling_map_uses_partial_path_without_false_arrival():
    a = scene(1, [0, 0])
    path, blocked = grid_route(a.layers, a.record, [0, 0], [8, 0], 1.1, partial=True)
    assert path and 2.5 < path[-1][0] < 3
    assert all(visible(p, q, blocked, a.record) for p, q in zip(path, path[1:]))
    assert not grid_route(a.layers, a.record, [0, 0], [8, 0], 1.1)[0]


def test_dropoff_uses_wheel_extent_but_rock_keeps_body_clearance():
    a = scene(1, [0, 0])
    a.layers[22, 31, :2] = [-1, 8]  # Lateral 75 cm, outside rotating wheel extent.
    args = (a.layers, a.record, [0, 0], [2, 0], 1.1)
    assert not grid_route(*args)[0]  # Preserved legacy policy.
    assert grid_route(*args, wheel_radius=0.46)[0]
    a.layers[22, 31, 0] = 100
    assert not grid_route(*args, wheel_radius=0.46)[0]
    a.layers[22, 31, :2] = [-1, 0]
    a.layers[27, 32, :2] = [-1, 8]  # Actual wheel/rotation region stays blocked.
    assert not grid_route(*args, wheel_radius=0.46)[0]


def test_stall_counts_commanded_motion_not_intentional_waits(monkeypatch, tmp_path):
    import time
    from test_step_trial import fake_driver

    driver = fake_driver(monkeypatch, tmp_path)
    driver.spec = route("costmap_return4")
    # Exercise driver time accounting without involving a path-planning plant.
    driver.validate = lambda *args, **kwargs: (0.05, 0)
    driver.proposal["terminal"] = None
    for stamp, fresh in [(4.0, True), (4.1, False), (7.0, False), (10.0, True)]:
        driver.proposal["query_ns"] = round(stamp * 1e9)
        driver.proposal_wall = time.monotonic() - (0 if fresh else 1)
        driver.step(stamp, np.eye(4))
    assert driver.command_elapsed == pytest.approx(0.1)
    assert driver.mode == "translate" and np.array_equal(driver.mode_start, np.eye(4))
    driver.proposal["query_ns"] = 11_500_000_000
    driver.proposal_wall = time.monotonic()
    with pytest.raises(ValueError, match="motion stalled"):
        driver.step(11.5, np.eye(4))


@pytest.mark.parametrize("reason", [5, 6, 7, 8, 10, 11])
def test_support_warning_keeps_wheel_veto_without_body_sized_dead_zone(reason):
    from local_terrain.contact_query import PreparedMotion

    a = scene(1, [0, 0])
    a.layers[22, 31, :2] = [-1, reason]
    args = (a.layers, a.record, [0, 0], [2, 0], 1.1)
    kwargs = dict(wheel_radius=0.46, separate_support_conflicts=True)
    assert grid_route(*args, **kwargs)[0]
    a.layers[22, 31, 0] = 100
    assert not grid_route(*args, **kwargs)[0]  # Original rock margin is unchanged.
    a.layers[22, 31, :2] = [-1, 0]
    a.layers[27, 32, :2] = [-1, reason]
    assert not grid_route(*args, **kwargs)[0]
    sweep = PreparedMotion(a.layers, a.record, [np.eye(4)], a.patches, a.body)
    assert sweep.conflicts  # Exact wheel test independently retains the warning.


def test_one_command_counterfactual_keeps_original_sensor_pose_and_hazard_veto():
    from diagnose_costmap_refusal import propose

    original = CostmapTrial("c", scenario="costmap_square8")
    a = scene(1_000_000_000, [0, 0])
    initial = original.update(a, a.context.query_ns)
    a = scene(4_000_000_000, [0, 0])
    value = original.update(a, a.context.query_ns)
    snapshot = dict(
        layers=a.layers.copy(),
        local_from_base=a.context.local_from_base,
        local_from_map=a.record["local_from_map"],
        origin=a.record["origin"],
    )
    result = propose(snapshot, value, initial, value, a.patches)
    assert result["local_from_base"] == value["local_from_base"]
    assert result["linear"] == 0.05
    snapshot["layers"][30, 34, 0] = 100
    result = propose(snapshot, value, initial, value, a.patches)
    assert result["linear"] == result["angular"] == 0


def test_refusal_snapshot_uses_sealed_consumer_pose_not_map_evidence_pose():
    import hashlib
    from diagnose_costmap_refusal import validate_snapshot

    layers = np.zeros((2, 2, 6))
    snapshot = dict(layers=layers, origin=[0, 0], local_from_map=np.eye(4), local_from_base=np.eye(4))
    value = dict(local_from_map=np.eye(4), local_from_base=np.eye(4))
    record = dict(snapshot, layers_sha256=hashlib.sha256(layers.tobytes()).hexdigest())
    record["local_from_base"] = np.eye(4)
    record["local_from_base"][0, 3] = 0.001
    validate_snapshot(snapshot, value, record)
    value["local_from_base"][0, 3] = 0.001
    with pytest.raises(ValueError, match="sealed proposal"):
        validate_snapshot(snapshot, value, record)
    value["local_from_base"][0, 3] = 0
    snapshot["layers"][0, 0, 0] = 100
    with pytest.raises(ValueError, match="sealed map"):
        validate_snapshot(snapshot, value, record)


def test_exact_start_connector_removes_cell_rounding_without_shrinking_radii():
    from local_terrain.costmap_trial import CircularClearance

    a = scene(1, [0, 0])
    a.layers[29, 35, :2] = [-1, 8]  # Cell x=[.5,.6], y=[-.1,0].
    clearance = CircularClearance(a.layers, a.record, 1.1, 0.46)
    args = (a.layers, a.record, [0, 0], [-2, 0], 1.1)
    kwargs = dict(wheel_radius=0.46, separate_support_conflicts=True)
    assert not grid_route(*args, **kwargs)[0]
    path, blocked = grid_route(*args, **kwargs, start_clearance=clearance)
    assert path and np.array_equal(path[0], [0, 0])
    assert not any(np.array_equal(p, [0.05, 0.05]) for p in path)  # No invented centered pose.
    assert all(clearance(p, q) for p, q in zip(path, path[1:]))
    assert visible(path[0], path[-1], blocked, a.record, clearance=clearance)
    # Actual overlap still rejects; an occupied rock keeps the original 1.1 m.
    assert not clearance([0.05, 0], [0.05, 0])
    a.layers[29, 35, 0] = 100
    rock = CircularClearance(a.layers, a.record, 1.1, 0.46)
    assert not grid_route(*args, **kwargs, start_clearance=rock)[0]


def test_exact_clearance_rejects_segment_crossing_and_closed_corner_touch():
    from local_terrain.costmap_trial import CircularClearance

    a = scene(1, [0, 0])
    a.layers[30, 30, :2] = [-1, 8]  # Square [0,.1] x [0,.1].
    clearance = CircularClearance(a.layers, a.record, 1.1, 0.46)
    assert clearance([-1, 0.05], [-1, 0.05]) and clearance([1, 0.05], [1, 0.05])
    assert not clearance([-1, 0.05], [1, 0.05])  # Endpoint checks alone are insufficient.
    assert not clearance([-0.46, 0], [-0.46, -0.3])
    assert clearance([-0.46001, 0], [-0.46001, -0.3])


def test_occupied_padding_allows_only_a_certified_escape_from_current_cell():
    from local_terrain.costmap_trial import CircularClearance

    a = scene(1, [0, 0])
    a.layers[30, 43, :2] = [100, 3]
    start, goal = np.array([0.128, 0.013]), np.array([-2, 0.013])
    clearance = CircularClearance(a.layers, a.record, 1.1, 0.46)
    original = clearance.body_blocked.copy()
    args = (a.layers, a.record, start, goal, 1.1)
    kwargs = dict(wheel_radius=0.46, separate_support_conflicts=True)
    assert original[30, 31] and clearance(start, start)
    assert not grid_route(*args, **kwargs)[0]
    path, blocked = grid_route(*args, **kwargs, start_clearance=clearance)
    assert path and visible(start, goal, blocked, a.record, clearance=clearance)
    assert all(clearance(p, q) for p, q in zip(path, path[1:]))
    assert not visible(start, [0.14, 0.02], blocked, a.record, clearance=clearance)
    assert not visible(start, [0.4, 0.013], blocked, a.record, clearance=clearance)
    assert np.array_equal(clearance.body_blocked, original)
    # The exception belongs to the already-occupied start cell, not the rest
    # of the route, and cannot authorize approach through this padding.
    assert not visible(goal, start, original, a.record, clearance=clearance)


def test_route_validator_rejects_other_scenario_and_legacy_protocol():
    value = CostmapTrial("c", scenario="costmap_return4").update(scene(1, [0, 0]), 1)
    for pattern in (None, "costmap_square8", "invented"):
        with pytest.raises(ValueError):
            validate_proposal(value, "c", ("s", 1), 1, scenario=pattern)


def test_fixed_position_envelope_does_not_rotate_with_current_heading():
    start, current = np.eye(4), np.eye(4)
    start[:2, :2] = [[0, -1], [1, 0]]
    current[1, 3] = 2
    bounds = scenario_spec("costmap_return4")["position_bounds_initial_xy"]
    check_position(start, current, bounds)
    current[0, 3] = 1
    with pytest.raises(ValueError):
        check_position(start, current, bounds)


def test_wait_does_not_erase_turn_drift_guard(monkeypatch, tmp_path):
    import time
    from test_step_trial import fake_driver

    driver = fake_driver(monkeypatch, tmp_path)
    driver.spec = route("costmap_return4")
    driver.mode, driver.mode_start = "rotate", np.eye(4)
    driver.progress = (4.0, np.eye(4))
    driver.proposal_wall = time.monotonic() - 1
    driver.step(4.0, np.eye(4))
    assert driver.mode == "rotate" and np.array_equal(driver.mode_start, np.eye(4))
    controller = CostmapTrial("capture", scenario="costmap_return4")
    controller.update(scene(1_000_000_000, [0, 0], math.pi), 1_000_000_000)
    # A checked pure turn; GT is used below only to challenge the drift veto.
    controller.goal = np.array([0, 2])
    driver.proposal = controller.update(scene(4_000_000_000, [0, 0], math.pi), 4_000_000_000)
    driver.proposal_sequence = driver.proposal["sequence"]
    driver.proposal_wall = time.monotonic()
    driver.validate = partial(validate_proposal, scenario="costmap_return4")
    pose = np.eye(4)
    pose[0, 3] = 0.76  # Explicit v12 cumulative limit; old v11 was 0.25 m.
    with pytest.raises(ValueError, match="rotation skid"):
        driver.step(4.0, pose)


def test_obstacle_survey_does_not_disable_command_time_first_hit_veto(monkeypatch, tmp_path):
    from natural_motion import SurveyLookup
    from test_step_trial import fake_driver

    driver = fake_driver(monkeypatch, tmp_path)
    del driver.guard  # Exercise the real inherited guard, not fixture stub.
    driver.spec = route("costmap_detour8")
    driver.points, driver.shape = survey_points(np.eye(4), "costmap_detour8")
    driver.survey_lookup = SurveyLookup(driver.points)
    driver.heights = np.zeros(len(driver.points))
    driver.normals = np.tile([0, 0, 1], (len(driver.points), 1))
    driver.cast = lambda *args: [
        (1.6, [0, 0, 0.4], [0, 0, 1], "/DemoDetourRock/rock_0"),
        (2, [0, 0, 0], [0, 0, 1], "/Lunaryard/Terrain/mesh"),
    ]
    with pytest.raises(ValueError, match="contains rock"):
        driver.guard(np.eye(4))


def test_detour_rock_cannot_be_omitted_or_injected_into_other_routes():
    from demo_rock_profile import DETOUR_PROFILE, validate_profile, rounded_mesh

    assert validate_profile(DETOUR_PROFILE, "costmap_detour8") == DETOUR_PROFILE
    for name, pattern in [("", "costmap_detour8"), (DETOUR_PROFILE, "costmap_square8")]:
        with pytest.raises(ValueError):
            validate_profile(name, pattern)
    points, _ = rounded_mesh(0.4, 0.6)
    assert np.ptp(points, axis=0) == pytest.approx([0.6, 0.6, 0.4])


def test_detour_destination_clears_background_rock_without_removing_detour():
    # Evaluator-only frozen USD AABB from seed 43. Old (4, 0) goal was inside
    # its body margin; no runtime controller reads these truth coordinates.
    spec = scenario_spec("costmap_detour8")
    assert spec["route_layout"] == "wide_goal_south_v2"
    goal = np.array(spec["goals_initial_xy"][0])
    low, high = np.array([4.228, 0.194]), np.array([5.065, 1.071])
    distance = lambda xy: np.linalg.norm(np.maximum(np.maximum(low - xy, xy - high), 0))
    assert distance(np.array([4, 0])) < 0.31
    assert distance(goal) > math.hypot(0.85, 0.7) + 0.3
    rock = np.array([1.8, 0])
    nearest = np.clip(rock @ goal / (goal @ goal), 0, 1) * goal
    assert np.linalg.norm(rock - nearest) < math.hypot(0.85, 0.7) + 0.3
    assert np.linalg.norm(rock - goal) > math.hypot(0.85, 0.7) + 0.3


def test_proposal_from_ros_clock_waits_for_actual_simulator_tick(monkeypatch, tmp_path):
    import json
    from types import SimpleNamespace
    from test_step_trial import fake_driver

    driver = fake_driver(monkeypatch, tmp_path)
    driver.proposal_sequence, driver.reference, driver.last_revision = 0, None, -1
    driver.owner.stamp = 3.983333333
    driver.owner.node = SimpleNamespace(
        get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name="terrain_shadow_mapper")]
    )
    driver.receive_proposal(SimpleNamespace(data=json.dumps(driver.proposal)))
    assert driver.failure is None
    driver.step(3.983333333, np.eye(4))
    assert driver.commands[-1] == (0, 0)
    driver.step(4.0, np.eye(4))
    assert driver.commands[-1] == (0.02, 0)
    driver.step(4.200000001, np.eye(4))
    assert driver.commands[-1] == (0, 0)
