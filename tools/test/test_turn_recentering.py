"""V12 replaces precision centering with clearance-checked bounded turn drift.

Historical v11 centering contracts are preserved with the archived source.
The user explicitly authorized changed turn limits; rock/freshness guards remain.
"""

import copy
import math
import time
from functools import partial
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from local_terrain.costmap_trial import CostmapTrial, validate_proposal
from natural_motion import route
from test_costmap_scenarios import scene
from test_step_trial import fake_driver


def test_report_does_not_count_reverse_correction_as_forward():
    from assess_step_trial import command_times

    rows = [dict(stamp=i, linear=v, angular=0) for i, v in enumerate([0.05, -0.05, 0, 0])]
    assert command_times(rows) == dict(forward=1, reverse=1, turn=0, waiting=0, hold=1)


def turning_trial():
    controller = CostmapTrial("capture", scenario="costmap_square8")
    controller.update(scene(1_000_000_000, [0, 0]), 1_000_000_000)
    controller.goal = np.array([0.0, -2.0])
    controller.goals = [controller.goal.copy()]
    value = controller.update(scene(4_000_000_000, [0, 0]), 4_000_000_000)
    assert value["angular"] == -0.1 and value["turn_id"] == 1
    return controller, value


@pytest.mark.parametrize("x", [-0.08, 0.08, -0.26, 0.26])
def test_turn_does_not_chase_noisy_position_with_translation(x):
    controller, first = turning_trial()
    value = controller.update(scene(4_100_000_000, [x, 0]), 4_100_000_000)
    assert not value["turn_recentering"] and value["angular"] == -0.1 and value["linear"] == 0
    assert value["relative_poses"][-1][0][3] == 0
    assert value["turn_id"] == first["turn_id"] and value["turn_origin_xy"] == [0, 0]
    assert validate_proposal(value, "capture", ("s", 1), value["query_ns"], scenario="costmap_square8") == (0, -0.1)


def test_backward_hazard_blocks_translation_but_not_clear_observation_turn():
    controller, _ = turning_trial()
    a = scene(4_100_000_000, [0.08, 0])
    # Negative edge behind the current wheels, intersected by reverse sweep.
    a.layers[33, 25, :2] = [-1, 8]
    value = controller.update(a, a.context.query_ns)
    assert value["linear"] == 0 and value["angular"] == -0.1
    assert not value["turn_recentering"]
    # Verify the offered rotation against the original, unmodified warning.
    from local_terrain.contact_query import PreparedMotion

    poses = a.context.poses([np.asarray(p) for p in value["relative_poses"]])
    sweep = PreparedMotion(a.layers, a.record, poses, a.patches, a.body)
    assert sweep.terminal is None and not sweep.conflicts


@pytest.mark.parametrize(
    "mutation",
    [
        dict(turn_id=None, turn_origin_xy=None),
        dict(turn_recentering=True),
        dict(turn_origin_xy=[float("nan"), 0]),
        dict(turn_translation_limit_m=10),
        dict(turn_command_drift_allowance_m=0),
        dict(linear=-0.05, angular=0),
    ],
)
def test_turn_rejects_centering_and_changed_limits(mutation):
    controller, _ = turning_trial()
    value = controller.update(scene(4_100_000_000, [0.08, 0]), 4_100_000_000)
    value.update(mutation)
    with pytest.raises(ValueError):
        validate_proposal(value, "capture", ("s", 1), value["query_ns"], scenario="costmap_square8")


def test_wait_keeps_frozen_gt_anchor_with_explicit_larger_limit(monkeypatch, tmp_path):
    controller, first = turning_trial()
    driver = fake_driver(monkeypatch, tmp_path)
    driver.spec = route("costmap_square8")
    driver.validate = partial(validate_proposal, scenario="costmap_square8")

    def execute(value, gt_x):
        driver.proposal, driver.proposal_sequence = value, value["sequence"]
        driver.proposal_wall = time.monotonic()
        pose = np.eye(4)
        pose[0, 3] = gt_x
        driver.step(value["query_ns"] / 1e9, pose)

    execute(first, 0)
    for i in range(1, 27):
        stamp = 4_000_000_000 + i * 10_000_000
        rotation = controller.update(scene(stamp, [i * 0.01, 0]), stamp)
        execute(rotation, i * 0.01)
    # Old 25 cm rejection is explicitly no longer expected.
    assert driver.mode == "rotate" and driver.commands[-1] == (0, -0.1)
    driver.proposal_wall -= 1
    driver.step(4.27, np.eye(4))
    rotation = controller.update(scene(4_300_000_000, [0.30, 0]), 4_300_000_000)
    with pytest.raises(ValueError, match="rotation skid"):
        execute(rotation, 0.76)
    changed = copy.deepcopy(rotation)
    changed["turn_origin_xy"] = [0.01, 0]
    with pytest.raises(ValueError, match="anchor changed"):
        execute(changed, 0.2)


def test_slipping_turn_plant_can_finish_without_precision_centering():
    controller, _ = turning_trial()
    xy, angle, corrected, maximum = np.zeros(2), 0.0, 0, 0.0
    for tick in range(1, 600):
        stamp = 4_000_000_000 + tick * 100_000_000
        value = controller.update(scene(stamp, xy, angle), stamp)
        linear, angular = validate_proposal(value, "capture", ("s", 1), stamp, scenario="costmap_square8")
        if linear and not value["turn_recentering"]:
            break
        corrected += bool(value["turn_recentering"] and linear)
        body_v = np.array([linear - 0.07 * abs(angular), -0.025 * abs(angular)])
        c, s = math.cos(angle), math.sin(angle)
        xy += np.array([[c, -s], [s, c]]) @ body_v * 0.1
        angle += angular * 0.4 * 0.1
        maximum = max(maximum, np.linalg.norm(xy))
        assert np.linalg.norm(xy) < 0.75
    assert corrected == 0 and linear == 0.05 and not value["turn_recentering"]
    assert 0.1 < maximum < 0.75


@pytest.mark.parametrize("new_sequence", [False, True])
def test_gt_drift_allowance_cannot_be_bypassed_by_a_fresh_proposal(monkeypatch, tmp_path, new_sequence):
    controller, first = turning_trial()
    driver = fake_driver(monkeypatch, tmp_path)
    driver.spec = route("costmap_square8")
    driver.validate = partial(validate_proposal, scenario="costmap_square8")
    driver.proposal, driver.proposal_sequence = first, first["sequence"]
    driver.proposal_wall = time.monotonic()
    driver.step(4.0, np.eye(4))
    if new_sequence:
        driver.proposal = controller.update(scene(4_100_000_000, [0, 0]), 4_100_000_000)
        driver.proposal_sequence = driver.proposal["sequence"]
    drifted = np.eye(4)
    drifted[1, 3] = 0.051
    with pytest.raises(ValueError, match="command drift allowance"):
        driver.step(4.1, drifted)


def test_estimated_excessive_turn_translation_stops_without_recentering():
    controller, first = turning_trial()
    value = controller.update(scene(4_100_000_000, [0.76, 0]), 4_100_000_000)
    assert value["linear"] == value["angular"] == 0
    assert value["decision"]["reason"] == "turn_translation_limit"
    assert value["turn_origin_xy"] == first["turn_origin_xy"]


def test_rock_in_additional_slip_envelope_still_vetoes_turn():
    from local_terrain.costmap_trial import CircularClearance

    controller, _ = turning_trial()
    a = scene(4_100_000_000, [0, 0])
    a.record["origin"] = [-2.97, -3.0]
    a.layers[30, 41, :2] = [100, 3]  # Near edge x=1.13 m: old 1.101 m circle clears.
    assert CircularClearance(a.layers, a.record, math.hypot(0.85, 0.70), 0.46)([0, 0], [0, 0])
    value = controller.update(a, a.context.query_ns)
    assert value["linear"] == value["angular"] == 0
    assert value["decision"]["state"] == "blocked"
