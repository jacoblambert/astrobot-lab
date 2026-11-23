"""Exact raw-index pose-displacement diagnostics; NumPy/SciPy, no ROS or GPU."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_terrain_pose_conflicts import conflict_cells, selected_returns


def test_all_conflicts_are_selected_without_tolerance_or_unknown_as_free():
    a, b = np.zeros((1, 4, 6)), np.zeros((1, 4, 6))
    a[0, :, 0] = [100, 100, 100, -1]
    b[0, :, 0] = [0, 20, -1, 0]
    np.testing.assert_array_equal(conflict_cells(a, b), [[0, 0], [0, 1]])
    b[0, 0, 0] = 50
    with pytest.raises(ValueError, match="costs"):
        conflict_cells(a, b)


def test_translation_moves_obstacle_evidence_not_its_original_point_identity():
    truth, estimate = np.eye(4), np.eye(4)
    estimate[0, 3] = .1
    rows = selected_returns(np.array([[.05, .05, .1], [-.05, .05, 0.]]),
                            truth, estimate, [[0, 0]], [0., 0.], .1)
    assert [r["point_index"] for r in rows] == [0, 1]
    assert rows[0]["gt_in_cell"] and not rows[0]["slam_in_cell"]
    assert rows[0]["slam_cell_yx"] == [0, 1]
    assert not rows[1]["gt_in_cell"] and rows[1]["slam_in_cell"]
    np.testing.assert_allclose(rows[0]["delta_xyz"], [.1, 0., 0.])


def test_rotation_is_applied_at_measurement_origin_not_fitted_to_surface():
    estimate = np.eye(4)
    estimate[:2, :2] = [[0, -1], [1, 0]]
    rows = selected_returns(np.array([[.15, .05, .08]]), np.eye(4), estimate, [[0, 1]], [0., 0.], .1)
    np.testing.assert_allclose(rows[0]["slam_xyz"], [-.05, .15, .08])
    np.testing.assert_allclose(rows[0]["delta_xyz"], [-.2, .1, 0.])


def test_no_conflicts_and_invalid_points_fail_without_inventing_correspondences():
    assert selected_returns(np.empty((0, 3)), np.eye(4), np.eye(4), [], [0., 0.], .1) == []
    with pytest.raises(ValueError, match="points"):
        selected_returns(np.array([[np.nan, 0, 0]]), np.eye(4), np.eye(4), [], [0., 0.], .1)
