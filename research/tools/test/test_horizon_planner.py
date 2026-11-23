from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.horizon_planner import FixedFreeGoals, HorizonPlanner, HorizonPolicy, length, translation_prefix
from local_terrain.contact_index import IncrementalSupportIndex
from local_terrain.contact_runtime import startup_footprint
from test_cost_planner import BODY, PATCHES, fixture, collision
from shapely import box


def test_long_unknown_bridge_is_a_route_not_permission_to_cross_it():
    layers, record, start = fixture()
    layers[:, 29:32, 0] = -1
    before = layers.copy()
    planner = HorizonPlanner(layers, record, start, PATCHES, BODY)
    goal = planner.select_goal([1.4, 0])
    assert goal is not None
    assert not planner.local.plan(goal)["found"]
    result = planner.plan(goal)
    assert result["route"]["found"] and result["prefix"]["found"]
    assert result["checked_translation_m"] == pytest.approx(0.5)
    assert result["executable_translation_m"] == pytest.approx(0.48)
    assert not result["goal_reached"] and not result["navigation_authority"]
    assert collision(result["route"]["poses"], box(-0.1, -3, 0.2, 3), PATCHES)
    assert not collision(result["prefix"]["poses"], box(-0.1, -3, 0.2, 3), PATCHES)
    np.testing.assert_array_equal(layers, before)


@pytest.mark.parametrize("classification,reason", [(100, 1), (-1, 11), (-1, 7)])
def test_known_obstacle_dropoff_or_conflict_is_never_a_soft_route(classification, reason):
    layers, record, start = fixture()
    layers[:, 29:32, :2] = classification, reason
    result = HorizonPlanner(layers, record, start, PATCHES, BODY).plan([1.4, 0])
    assert not result["route"]["found"] and not result["prefix"]["found"]


def test_nearby_unknown_limits_prefix_even_when_long_route_exists():
    layers, record, start = fixture()
    layers[:, 21:24, 0] = -1
    result = HorizonPlanner(layers, record, start, PATCHES, BODY).plan([1.4, 0])
    assert result["route"]["found"]
    assert result["prefix"]["found"]
    assert 0.05 <= result["executable_translation_m"] < 0.4
    assert not collision(result["prefix"]["poses"], box(-0.9, -3, -0.6, 3), PATCHES)


def test_unknown_destination_is_not_silently_accepted_or_replaced():
    layers, record, start = fixture()
    layers[29:32, 43:46, 0] = -1
    planner = HorizonPlanner(layers, record, start, PATCHES, BODY)
    result = planner.plan([1.4, 0])
    assert not result["route"]["found"]
    assert result["route"]["reason"] == "goal_not_observed_free"
    assert result["goal_local_xy"] == [1.4, 0]
    assert planner.select_goal([1.4, 0], max_offset_m=0.05) is None


def test_actual_turning_area_does_not_follow_long_route_into_unknown():
    layers, record, start = fixture()
    layers[:, :, 0] = -1
    layers[25:35, 41:47, 0] = 0
    index = IncrementalSupportIndex()
    index.update([(startup_footprint(True) + start[:2, 3], "current_turn")], record["origin"], record["resolution"])
    planner = HorizonPlanner(layers, record, start, PATCHES, BODY, support_index=index)
    result = planner.plan([1.4, 0])
    assert result["route"]["found"] and result["prefix"]["found"]
    assert 0.05 <= result["executable_translation_m"] < 0.3
    assert length(result["route"]["poses"]) > 2
    assert not planner.local.plan([1.4, 0])["found"]


def test_known_obstacle_at_current_pose_overrides_turning_assumption():
    layers, record, start = fixture()
    layers[29:32, 15:18, 0] = 100
    index = IncrementalSupportIndex()
    index.update([(startup_footprint(True) + start[:2, 3], "current_turn")], record["origin"], record["resolution"])
    result = HorizonPlanner(layers, record, start, PATCHES, BODY, support_index=index).plan([1.4, 0])
    assert not result["route"]["found"] and not result["prefix"]["found"]


def test_prefix_reserve_cannot_cut_across_a_corner():
    a = np.eye(4)
    b = a.copy()
    b[0, 3] = 1
    c = b.copy()
    c[:2, :2] = [[0, -1], [1, 0]]
    d = c.copy()
    d[1, 3] = 1
    prefix = translation_prefix([a, b, c, d], 1.4)
    assert len(prefix) == 4
    np.testing.assert_array_equal(prefix[1], b)
    np.testing.assert_array_equal(prefix[2], c)
    np.testing.assert_allclose(prefix[-1][:2, 3], [1, 0.4])
    assert length(prefix) == pytest.approx(1.4)
    with pytest.raises(ValueError, match="separate"):
        translation_prefix([a, c], 0.5)


def test_goal_selection_translation_invariance_and_outside_rejection():
    layers, record, start = fixture()
    first = HorizonPlanner(layers, record, start, PATCHES, BODY)
    goal = first.select_goal([1.4, 0.0])
    shift = np.array([20, -18])
    start[:2, 3] += shift
    record["origin"] = (np.array(record["origin"]) + shift).tolist()
    second = HorizonPlanner(layers, record, start, PATCHES, BODY)
    # Unique off-center query avoids symmetric-cell floating-point tie ambiguity.
    a = first.select_goal([1.42, 0.02])
    b = second.select_goal(np.array([1.42, 0.02]) + shift)
    np.testing.assert_allclose(np.array(a) + shift, b)
    assert first.known_free(goal)
    assert not second.known_free([1000, 1000])
    assert not second.plan([1000, 1000])["route"]["found"]


@pytest.mark.parametrize(
    "kwargs",
    [dict(unknown_route_cost=100), dict(stopping_reserve_m=0), dict(local_horizon_m=2), dict(minimum_translation_m=1)],
)
def test_invalid_policy_fails(kwargs):
    with pytest.raises(ValueError):
        HorizonPolicy(**kwargs)


def test_observation_wait_freezes_goals_including_missing_without_future_retargeting():
    layers, record, start = fixture()
    planner = HorizonPlanner(layers, record, start, PATCHES, BODY)
    selection = FixedFreeGoals(5)
    selection.observe(planner, 100)
    assert selection.selected_ns is None and all(g is None for g in selection.goals.values())
    selection.observe(planner, 5_000_000_100)
    assert selection.selected_ns == 5_000_000_100
    saved = dict(selection.goals)
    assert saved["1.0m_0"] is not None
    assert saved["2.0m_180"] is None  # Outside this shifted start's map.
    start[0, 3] += 1
    layers[:, :, 0] = -1
    selection.observe(HorizonPlanner(layers, record, start, PATCHES, BODY), 6_000_000_100)
    assert selection.goals == saved
    with pytest.raises(ValueError, match="clock"):
        selection.observe(planner, 6_000_000_100)


def test_lattice_endpoint_stays_known_free_inside_original_goal_tolerance():
    layers, record, start = fixture()
    layers[:, :, 0] = -1
    layers[30, 44, 0] = 0
    planner = HorizonPlanner(layers, record, start, PATCHES, BODY)
    goal = [1.45, 0.05]
    endpoint = planner.lattice_goal(goal)
    assert endpoint is not None and planner.known_free(endpoint)
    assert np.linalg.norm(np.asarray(endpoint) - goal) <= 0.075
    assert goal == [1.45, 0.05]


def test_free_goal_center_can_have_no_clear_body_orientation():
    layers, record, start = fixture()
    layers[30, 48, 0] = 100
    planner = HorizonPlanner(layers, record, start, PATCHES, BODY)
    assert planner.known_free([1.4, 0])
    assert planner.endpoint_headings(planner.lattice_goal([1.4, 0])) == []
    result = planner.plan([1.4, 0])
    assert result["route"]["reason"] == "goal_footprint_unavailable"
    assert result["route"]["expanded"] == 0
    assert not result["prefix"]["found"]


def test_endpoint_precheck_preserves_successful_route_and_local_prefix_exactly():
    layers, record, start = fixture()
    layers[:, 29:32, 0] = -1
    off = HorizonPlanner(layers, record, start, PATCHES, BODY, policy=HorizonPolicy(endpoint_precheck=False))
    on = HorizonPlanner(layers, record, start, PATCHES, BODY)
    first, second = off.plan([1.4, 0]), on.plan([1.4, 0])
    for key in ("route", "prefix"):
        assert first[key]["found"] == second[key]["found"]
        np.testing.assert_array_equal(first[key]["poses"], second[key]["poses"])


def test_newly_seen_hazard_withdraws_local_permission_even_for_same_fixed_goal():
    layers, record, start = fixture()
    first = HorizonPlanner(layers, record, start, PATCHES, BODY).plan([1.4, 0])
    assert first["prefix"]["found"]
    layers[30, 20, 0] = 100
    second = HorizonPlanner(layers, record, start, PATCHES, BODY).plan([1.4, 0])
    assert not second["prefix"]["found"]
