"""User-declared navigation prior, not terrain-truth or sensor qualification."""

import copy
import math

import numpy as np
import pytest

from test_costmap_scenarios import scene
from costmap_scenarios import scenario_spec
from local_terrain.costmap_trial import (
    CircularClearance,
    CostmapTrial,
    current_turn_layers,
    grid_route,
    validate_proposal,
)


RADIUS = scenario_spec("costmap_square8")["current_turn_assumption_radius_m"]


@pytest.mark.parametrize("reason", [5, 6, 7, 8, 10, 11])
def test_current_circle_overrides_uncertainty_without_mutating_evidence(reason):
    a = scene(1, [0, 0])
    a.layers[32, 32, :2] = [-1, reason]
    a.layers[30, 36, :2] = [-1, reason]  # Only partly inside; cannot clear whole square.
    a.layers[29, 29, :2] = [100, 3]
    original = a.layers.copy()
    layers, receipt = current_turn_layers(a.layers, a.record, a.context.local_from_base, RADIUS)
    np.testing.assert_array_equal(a.layers, original)
    assert layers[32, 32, :2].tolist() == [-1, 0]
    assert layers[30, 36, :2].tolist() == [-1, reason]
    assert layers[29, 29, :2].tolist() == [100, 3]
    np.testing.assert_array_equal(layers[:, :, 2:], original[:, :, 2:])
    assert receipt["overridden_cells_xy"] == [[32, 32]]
    assert not receipt["terrain_evidence_changed"]


def test_current_prior_cannot_follow_hypothetical_path_or_persist_as_evidence():
    a = scene(1, [0, 0])
    a.layers[32, 32, :2] = [-1, 8]
    layers, _ = current_turn_layers(a.layers, a.record, a.context.local_from_base, RADIUS)
    assert layers[32, 32, 1] == 0
    pose = a.context.local_from_base.copy()
    pose[0, 3] = 2
    next_layers, receipt = current_turn_layers(a.layers, a.record, pose, RADIUS)
    assert next_layers[32, 32, 1] == 8 and receipt["overridden_cells_xy"] == []


def test_full_rotation_available_with_uncertainty_under_turn_circle():
    controller = CostmapTrial("c", scenario="costmap_square8")
    controller.update(scene(1_000_000_000, [0, 0]), 1_000_000_000)
    for index, yaw in enumerate(np.linspace(0, 2 * math.pi, 73)):
        stamp = 4_000_000_000 + index * 100_000_000
        a = scene(stamp, [0, 0], yaw)
        y, x = np.indices(a.layers.shape[:2])
        centers = np.stack([x, y], axis=-1) * 0.1 - 3 + 0.05
        a.layers[np.linalg.norm(centers, axis=-1) < 0.45, 1] = 8
        value = controller.update(a, stamp)
        validate_proposal(value, "c", ("s", 1), stamp, scenario="costmap_square8")
        assert value["decision"]["state"] == "available", value
        assert value["current_turn_assumption"]["overridden_cells_xy"]


def test_definitive_rock_inside_current_circle_still_stops():
    trial = CostmapTrial("c", scenario="costmap_return4")
    trial.update(scene(1_000_000_000, [0, 0]), 1_000_000_000)
    a = scene(4_000_000_000, [0, 0])
    a.layers[30, 32, :2] = [100, 3]
    value = trial.update(a, a.context.query_ns)
    assert value["linear"] == value["angular"] == 0
    assert value["decision"]["state"] == "blocked"


def test_closed_loop_square_with_new_uncertainty_inside_current_footprint():
    controller = CostmapTrial("c", scenario="costmap_square8")
    xy, yaw = np.zeros(2), 0.0
    assumed_queries = 0
    for tick in range(1201):
        stamp = 1_000_000_000 + tick * 500_000_000
        a = scene(stamp, xy, yaw)
        y, x = np.indices(a.layers.shape[:2])
        centers = np.stack([x, y], axis=-1) * 0.1 + a.record["origin"] + 0.05
        # Deliberate classifier faults arriving after movement; not GT evidence
        # fed into the controller. Test the declared precedence throughout a route.
        a.layers[np.linalg.norm(centers - xy, axis=-1) < 0.4, 1] = (5, 6, 8)[tick % 3]
        value = controller.update(a, stamp)
        linear, angular = validate_proposal(value, "c", ("s", 1), stamp, scenario="costmap_square8")
        assumed_queries += bool(value["current_turn_assumption"]["overridden_cells_xy"])
        if value["terminal"]:
            break
        xy += linear * 0.5 * np.array([math.cos(yaw), math.sin(yaw)])
        yaw += angular * 0.5
    assert value["terminal"] == "estimated_goal_reached", value
    assert len(value["reached_goals"]) == 4 and np.linalg.norm(xy) <= 0.1
    assert assumed_queries > 100


@pytest.mark.parametrize("occupied", [False, True])
def test_no_entry_into_corridor_without_turn_space(occupied):
    a = scene(1, [0, 0])
    # Walls start ahead, outside the actual current prior. A narrower rectangular
    # footprint could fit, but the all-heading envelope cannot.
    rows = (25, 34) if not occupied else (20, 39)
    for row in rows:
        a.layers[row, 37:, :2] = [100, 3] if occupied else [-1, 8]
    layers, _ = current_turn_layers(a.layers, a.record, a.context.local_from_base, RADIUS)
    clearance = CircularClearance(layers, a.record, math.hypot(0.85, 0.7), 0.46)
    path, _ = grid_route(
        layers,
        a.record,
        [0, 0],
        [2, 0],
        math.hypot(0.85, 0.7),
        wheel_radius=0.46,
        separate_support_conflicts=True,
        start_clearance=clearance,
    )
    assert not path


def test_validator_rejects_enlarged_or_shifted_assumption():
    v = CostmapTrial("c", scenario="costmap_square8").update(scene(1, [0, 0]), 1)
    for change in (dict(radius_m=10), dict(center_xy=[2, 0]), dict(terrain_evidence_changed=True)):
        bad = copy.deepcopy(v)
        bad["current_turn_assumption"].update(change)
        with pytest.raises(ValueError, match="assumption"):
            validate_proposal(bad, "c", ("s", 1), 1, scenario="costmap_square8")
