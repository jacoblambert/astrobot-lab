"""No goal route must not prevent a safe, bounded observation turn."""

import numpy as np
import pytest

from test_costmap_scenarios import scene
from local_terrain import costmap_trial


def setup(monkeypatch):
    original = costmap_trial.grid_route
    monkeypatch.setattr(costmap_trial, "grid_route", lambda layers, *a, **k: ([], np.zeros(layers.shape[:2], bool)))
    trial = costmap_trial.CostmapTrial("c", scenario="costmap_square8")
    trial.update(scene(1_000_000_000, [0, 0]), 1_000_000_000)
    return trial, original


def query(trial, seconds, xy=(0, 0), rock=False):
    stamp = round(seconds * 1e9)
    a = scene(stamp, xy)
    if rock:
        a.layers[30, 32, :2] = [100, 3]
    v = trial.update(a, stamp)
    costmap_trial.validate_proposal(v, "c", ("s", 1), stamp, scenario="costmap_square8")
    return v


def test_no_route_allows_observation_but_rock_still_vetoes(monkeypatch):
    trial, _ = setup(monkeypatch)
    first = query(trial, 4)
    assert first["angular"] == -0.1 and first["observing_for_route"]
    second = query(trial, 4.1)
    assert second["turn_id"] == first["turn_id"]
    blocked = query(trial, 4.2, rock=True)
    assert blocked["linear"] == blocked["angular"] == 0
    assert blocked["decision"]["state"] == "blocked"


def test_observation_follows_pose_without_recentering_or_resetting_anchor(monkeypatch):
    trial, _ = setup(monkeypatch)
    first = query(trial, 4)
    correction = query(trial, 4.1, xy=(0.08, 0))
    assert correction["linear"] == 0 and correction["angular"] == -0.1
    assert correction["turn_id"] == first["turn_id"]
    assert correction["turn_origin_xy"] == first["turn_origin_xy"]
    assert correction["observing_for_route"] and not correction["turn_recentering"]


def test_observation_budget_cannot_reset_on_transient_route_availability(monkeypatch):
    trial, original = setup(monkeypatch)
    query(trial, 4)
    monkeypatch.setattr(costmap_trial, "grid_route", original)
    assert not query(trial, 90)["observing_for_route"]
    monkeypatch.setattr(costmap_trial, "grid_route", lambda layers, *a, **k: ([], np.zeros(layers.shape[:2], bool)))
    # A missing NEW search no longer invalidates a retained clear route. Add an
    # actual blocker; the observation episode must still keep its original age.
    end = query(trial, 94, rock=True)
    assert end["linear"] == end["angular"] == 0
    assert not end["observing_for_route"]
    assert query(trial, 96.1, rock=True)["terminal"] == "terrain_refused"


def test_route_recovery_resumes_progress_after_new_observations(monkeypatch):
    trial, original = setup(monkeypatch)
    assert query(trial, 4)["observing_for_route"]
    monkeypatch.setattr(costmap_trial, "grid_route", original)
    v = query(trial, 5)
    assert not v["observing_for_route"] and v["linear"] == 0.05
    assert v["goal_local_xy"] == [2, 0]  # Never replace the mission goal.


@pytest.mark.parametrize("offset", [0.03, 0.08])
def test_blocked_correction_can_yaw_but_keeps_original_anchor(monkeypatch, offset):
    trial, _ = setup(monkeypatch)
    first = query(trial, 4)
    trial.recentering = True
    a = scene(4_100_000_000, [offset, 0])
    a.layers[33, 24, :2] = [-1, 8]  # Reverse sweep cannot fit, current turn can.
    v = trial.update(a, a.context.query_ns)
    assert v["angular"] == -0.1 and v["linear"] == 0
    assert v["turn_id"] == first["turn_id"] and v["turn_origin_xy"] == [0, 0]
    assert not v["turn_recentering"]


def test_frozen_counterfactual_preserves_turn_anchor():
    from diagnose_costmap_refusal import propose

    trial = costmap_trial.CostmapTrial("c", scenario="costmap_square8")
    initial = trial.update(scene(1_000_000_000, [0, 0]), 1_000_000_000)
    trial.goal = np.array([0.0, -2.0])
    trial.goals = [trial.goal.copy()]
    previous = query(trial, 4)
    a = scene(4_100_000_000, [0.08, 0])
    value = trial.update(a, a.context.query_ns)
    snapshot = dict(
        layers=a.layers,
        origin=a.record["origin"],
        local_from_base=a.context.local_from_base,
        local_from_map=a.record["local_from_map"],
    )
    candidate = propose(snapshot, value, initial, previous, a.patches)
    assert candidate["turn_origin_xy"] == value["turn_origin_xy"] == [0, 0]
    assert candidate["linear"] == value["linear"] == 0
    assert candidate["angular"] == value["angular"] == -0.1
