"""Retain safe routes; no cost-only switching, stale clearance, or false arrival."""

import copy
import math

import numpy as np
import pytest

from test_costmap_scenarios import scene
from local_terrain import costmap_trial as module


def started(monkeypatch, pattern="costmap_return4", yaw=0):
    calls = []
    original = module.grid_route

    def search(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    monkeypatch.setattr(module, "grid_route", search)
    trial = module.CostmapTrial("c", scenario=pattern)
    trial.update(scene(1_000_000_000, [0, 0]), 1_000_000_000)
    value = trial.update(scene(4_000_000_000, [0, 0], yaw), 4_000_000_000)
    assert value["route_generation"] == len(calls) == 1
    return trial, value, calls


def query(trial, a):
    value = trial.update(a, a.context.query_ns)
    module.validate_proposal(value, "c", ("s", 1), a.context.query_ns, scenario=trial.scenario)
    return value


def test_cheaper_alternative_new_visibility_and_pose_jitter_do_not_replan(monkeypatch):
    trial, first, calls = started(monkeypatch)
    for i in range(30):
        a = scene(4_100_000_000 + i * 100_000_000, [0.01 * math.sin(i), 0.01 * math.cos(i)])
        a.layers[:, :, 0] = 0 if i % 2 else -1
        a.layers[0, 0, 0] = 100  # Newly seen obstacle well away from this route.
        before = a.layers.copy()
        v = query(trial, a)
        assert v["route_waypoints_xy"] == first["route_waypoints_xy"]
        assert v["route_action"] == "retained"
        assert v["route_generation"] == len(calls) == 1
        assert v["goal_local_xy"] == [2, 0]
        assert np.array_equal(a.layers, before)


@pytest.mark.parametrize("kind", [(100, 3), (-1, 8), (-1, 7)])
def test_new_obstacle_or_support_conflict_replans_before_reusing_route(monkeypatch, kind):
    trial, first, calls = started(monkeypatch)
    a = scene(4_100_000_000, [0, 0])
    a.layers[30, 47, :2] = kind  # Future route blocked, current turning area clear.
    v = query(trial, a)
    assert len(calls) == 2 and v["route_action"] == "blocked"
    assert v["route_waypoints_xy"] != first["route_waypoints_xy"]
    assert v["goal_local_xy"] == first["goal_local_xy"]


def test_blocker_at_current_robot_stops_despite_committed_route(monkeypatch):
    trial, _, calls = started(monkeypatch)
    a = scene(4_100_000_000, [0, 0])
    a.layers[30, 31, :2] = [100, 3]
    v = query(trial, a)
    assert len(calls) == 2
    assert v["linear"] == v["angular"] == 0
    assert v["decision"]["state"] == "blocked"


def test_consumed_partial_route_extends_to_same_final_goal(monkeypatch):
    # Start aligned for the outbound straight leg: advancing to its endpoint
    # must not simulate teleporting several metres during a still-active turn.
    trial, first, calls = started(monkeypatch, "costmap_detour8", yaw=-0.46)
    assert first["linear"] == 0.05 and trial.turn_origin is None
    assert not np.array_equal(trial.route_points[-1], trial.goal)
    endpoint = trial.route_points[-1].copy()
    # Clear empty map gives a single straight committed segment to its frontier.
    assert len(trial.route_points) == 1
    a = scene(4_100_000_000, endpoint)
    v = query(trial, a)
    assert v["route_action"] == "local_route_consumed"
    assert len(calls) == v["route_generation"] == 2
    assert v["goal_index"] == 0 and not v["reached_goals"]
    assert v["goal_local_xy"] == [3.5, -1.5]


def test_actual_goal_arrival_starts_next_leg(monkeypatch):
    trial, _, calls = started(monkeypatch)
    v = query(trial, scene(4_100_000_000, [2, 0]))
    assert v["route_action"] == "goal_changed"
    assert v["route_generation"] == len(calls) == 2
    assert v["goal_index"] == 1 and v["goal_local_xy"] == [0, 0]
    assert v["route_waypoints_xy"][-1] == [0, 0]


def test_rolling_map_edge_is_not_a_new_blocker_but_observed_hazards_are(monkeypatch):
    trial, first, calls = started(monkeypatch)
    a = scene(4_100_000_000, [0, 0])
    a.record["origin"] = [-4, -3]  # Committed endpoint now at right edge/out of map.
    v = query(trial, a)
    assert v["route_waypoints_xy"] == first["route_waypoints_xy"]
    assert len(calls) == 1
    a = scene(4_200_000_000, [0, 0])
    a.record["origin"] = [-4, -3]
    a.layers[30, 57, :2] = [100, 3]
    v = query(trial, a)
    assert len(calls) == 2 and v["route_action"] == "blocked"


def test_reused_route_does_not_skip_immediate_3d_sweep(monkeypatch):
    trial, _, calls = started(monkeypatch)
    from local_terrain import contact_query

    class BlockedSweep:
        terminal = dict(state="blocked", reason="fixture_3d_conflict")

        def __init__(self, *args):
            pass

    monkeypatch.setattr(contact_query, "PreparedMotion", BlockedSweep)
    v = query(trial, scene(4_100_000_000, [0, 0]))
    assert len(calls) == 1  # Retained geometric route is NOT command authority.
    assert v["linear"] == v["angular"] == 0
    assert v["decision"]["reason"] == "fixture_3d_conflict"
    assert v["route_action"] == "immediate_sweep_blocked"
    assert not trial.route_points


def test_new_cheaper_detour_is_ignored_while_old_side_remains_clear(monkeypatch):
    trial = module.CostmapTrial("c", scenario="costmap_detour8")
    trial.goal = np.array([4.0, 0.0])
    a = scene(1, [0, 0])
    a.layers = np.zeros((100, 100, 6))
    a.layers[:, :, 0] = -1
    a.record["origin"] = [-3, -5]
    a.layers[47:53, 45:51, 0] = 100
    radius, wheel = 1.15, 0.51

    def calculate(controller):
        clearance = module.CircularClearance(a.layers, a.record, radius, wheel)
        return controller.committed_route(a.layers, a.record, np.zeros(2), radius, wheel, clearance)[0]

    old = calculate(trial)
    assert old and abs(old[1][1]) > 1
    # Make the opposite side cheaper, without changing any occupied cell.
    selected_rows = np.arange(100) < 50 if old[1][1] > 0 else np.arange(100) >= 50
    mask = selected_rows[:, None] & (a.layers[:, :, 0] != 100)
    a.layers[mask, 0] = 0
    fresh = module.CostmapTrial("c", scenario="costmap_detour8")
    fresh.goal = trial.goal.copy()
    alternative = calculate(fresh)
    assert alternative[1][1] * old[1][1] < 0  # Fresh search really switches sides.
    retained = calculate(trial)
    assert np.array_equal(retained, old)
    assert trial.route_generation == 1


def test_retention_policy_is_explicit_and_validated(monkeypatch):
    trial, value, _ = started(monkeypatch)
    bad = copy.deepcopy(value)
    bad["route_policy"] = "replan_every_tick"
    with pytest.raises(ValueError, match="retention policy"):
        module.validate_proposal(bad, "c", ("s", 1), value["query_ns"], scenario=trial.scenario)
