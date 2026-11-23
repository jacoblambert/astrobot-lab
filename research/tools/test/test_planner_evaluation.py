import sys
from pathlib import Path

import numpy as np
import pytest
from shapely import box
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assess_cost_paths import gt_at, optimistic_control
from local_terrain.planner_evaluation import evaluate_path, path_sweep
from test_cost_planner import BODY, PATCHES


def test_current_localization_error_can_expose_collision_not_seen_in_nominal_path():
    start, end = np.eye(4), np.eye(4)
    start[0, 3], end[0, 3] = -1.4, 1.4
    gt = start.copy()
    gt[1, 3] = 1.2
    result = evaluate_path(
        [start, end],
        dict(local_from_map=np.eye(4)),
        [0, 0, 0],
        [("rock", box(-0.1, 1.4, 0.1, 1.6))],
        box(-5, -5, 5, 5),
        PATCHES,
        BODY,
        gt_start=gt,
    )
    assert result["references"]["map_frame"]["possible_rock_guard_intersections"] == []
    assert result["references"]["gt_start_relative_tracking"]["possible_rock_guard_intersections"] == ["rock"]
    np.testing.assert_array_equal(start[:2, 3], [-1.4, 0])  # Evaluation never changes planner path.


def test_missing_truth_and_outside_survey_are_not_clear_path_acceptance():
    pose = np.eye(4)
    result = evaluate_path([pose], dict(local_from_map=np.eye(4)), [0, 0, 0], [], box(2, 2, 3, 3), PATCHES, BODY)
    assert result["missing_current_gt"]
    assert not result["references"]["map_frame"]["sampled_clear_and_no_rock_guard_intersection"]
    assert not result["navigation_qualified"]


def test_gt_lookup_never_extrapolates_or_matches_a_neighbouring_tick():
    rows = [dict(stamp=1.0, world_from_base=np.eye(4).tolist())]
    assert gt_at(rows, 1_000_000_001) == rows[0]["world_from_base"]
    assert gt_at(rows, 1_000_000_003) is None
    assert gt_at(rows, 1_016_666_667) is None


def test_map_home_chain_is_applied_once():
    local_from_map = np.eye(4)
    local_from_map[:2, 3] = [-40, -38]
    result = evaluate_path(
        [np.eye(4)],
        dict(local_from_map=local_from_map),
        [-20, -20, 0],
        [("world_rock", box(20, 18, 20.1, 18.1))],
        box(15, 15, 25, 25),
        PATCHES,
        BODY,
    )
    assert result["references"]["map_frame"]["possible_rock_guard_intersections"] == ["world_rock"]


def test_multi_turn_route_uses_per_edge_budget_without_skipping_segments():
    poses = []
    for angle in np.arange(0, 721, 45):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler("z", angle, degrees=True).as_matrix()
        poses.append(pose)
    sweep = path_sweep(poses, PATCHES)
    assert sweep.area > 0 and sweep.intersects(box(0.25, 0.25, 0.3, 0.3))
    with pytest.raises(ValueError, match="bounded"):
        path_sweep([], PATCHES)


def test_optimistic_control_is_explicit_and_does_not_clear_occupied_or_conflicts():
    layers = np.zeros((3, 4, 6))
    layers[1, :, :2] = [[-1, 0], [-1, 4], [-1, 11], [100, 1]]
    original = layers[:, :, 0].astype(np.int8)
    before = original.copy()
    costs = optimistic_control(layers, original)
    np.testing.assert_array_equal(costs[1], [90, 90, -1, 100])
    np.testing.assert_array_equal(original, before)
