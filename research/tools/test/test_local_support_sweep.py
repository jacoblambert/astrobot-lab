from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.physx_reference import BodyBox
from local_terrain.support_sweep import diagnostic_motions, support_availability, swept_polygons, touched_cells


PATCH = BodyBox((0, 0, 0), (0.04, 0.04, 0.001), "analytic_assumed_support_patch")


def pose(x=0.05, y=0.05, yaw=0):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    value[:2, 3] = x, y
    return value


def grid():
    value = np.zeros((20, 20, 6))
    value[:, :, 2] = 0.1
    return value


def test_clear_class_sweep_is_not_a_motion_or_age_certificate():
    result = support_availability(grid(), [-1, -1], 0.1, [pose(), pose(0.55)], [PATCH])
    assert result["support_class_state"] == "supported"
    assert result["cells"] == result["free_cells"] == 6
    for key in (
        "acceptance",
        "navigation_authority",
        "terrain_truth_evaluated",
        "body_clearance_evaluated",
        "support_freshness_certified",
        "relative_pose_uncertainty_evaluated",
    ):
        assert result[key] is False


@pytest.mark.parametrize("cost,expected", [(-1, "unknown"), (100, "blocked")])
def test_interior_hazard_or_hole_not_hidden_by_clear_endpoints(cost, expected):
    layers = grid()
    layers[10, 13, :2] = cost, 7
    result = support_availability(layers, [-1, -1], 0.1, [pose(), pose(0.55)], [PATCH])
    assert result["support_class_state"] == expected
    assert result["rejection_reasons"] == {"7": 1}


def test_outside_grid_keeps_full_denominator():
    result = support_availability(grid(), [-1, -1], 0.1, [pose(0.95), pose(1.15)], [PATCH])
    assert result["support_class_state"] == "unknown"
    assert result["cells"] == 3 and result["outside_cells"] == 2 and result["unknown_cells"] == 2


def test_touching_boundary_counts_both_sides():
    square = np.array([[0, 0], [0.1, 0], [0.1, 0.1], [0, 0.1]])
    cells, _ = touched_cells([square], [0, 0], 0.1)
    assert set(map(tuple, cells)) == {(x, y) for x in (-1, 0, 1) for y in (-1, 0, 1)}


@pytest.mark.parametrize("yaw", [0.2, 0.7, 1.57])
@pytest.mark.parametrize("phase", [0, 0.025, 0.075])
def test_rotating_off_center_patch_samples_all_inside_certified_cell_union(yaw, phase):
    patch = BodyBox((0.256, 0.2854, -0.1449), (0.04, 0.06, 0.001), "analytic_test")
    first, last = pose(phase, phase), pose(0.3 + phase, phase, yaw)
    last[:3, :3] = Rotation.from_euler("xyz", [0.1, -0.2, yaw]).as_matrix()
    cells, _ = touched_cells(swept_polygons([first, last], [patch]), [-1, -1], 0.1)
    covered = set(map(tuple, cells))
    interpolate = Slerp([0, 1], Rotation.from_matrix(np.array([first[:3, :3], last[:3, :3]])))
    points = (
        np.array([[x, y, z] for x in (-0.04, 0, 0.04) for y in (-0.06, 0, 0.06) for z in (-0.001, 0.001)])
        + patch.center
    )
    for t in np.linspace(0, 1, 151):
        world = interpolate(t).apply(points) + first[:3, 3] * (1 - t) + last[:3, 3] * t
        indices = np.floor((world[:, :2] + 1) / 0.1).astype(int)
        assert set(map(tuple, indices)) <= covered


def test_all_patches_checked_not_only_center_or_nearest_wheel():
    patches = [PATCH, BodyBox((0, 0.3, 0), PATCH.half_extent, "second")]
    layers = grid()
    layers[13, 10, :2] = -1, 5
    result = support_availability(layers, [-1, -1], 0.1, [pose()], patches)
    assert result["support_class_state"] == "unknown" and result["cells"] == 2


def test_work_budget_and_bad_inputs_fail_before_partial_results():
    with pytest.raises(ValueError, match="budget"):
        support_availability(grid(), [-1, -1], 0.1, [pose(), pose(1000)], [PATCH], max_cell_tests=10)
    with pytest.raises(ValueError):
        support_availability(grid(), [-1, -1], 0, [pose()], [PATCH])
    with pytest.raises(ValueError):
        support_availability(grid(), [-1, -1], 0.1, [], [PATCH])
    layers = grid()
    layers[0, 0, 0] = np.nan
    with pytest.raises(ValueError):
        support_availability(layers, [-1, -1], 0.1, [pose()], [PATCH])


def test_declared_motions_use_base_frame_and_keep_stationary():
    motions = diagnostic_motions(pose(yaw=np.pi / 2))
    assert set(motions) == {"stationary", "forward_10cm", "left_0.1rad", "right_0.1rad"}
    np.testing.assert_allclose(motions["forward_10cm"][-1][:2, 3], [0.05, 0.15])
