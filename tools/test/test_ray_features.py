from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from measure_ray_features import object_metrics


def test_missing_structure_does_not_pass():
    result = object_metrics(np.zeros((100, 3)), ([10, -1, 0], [13, 1, 2]))
    assert result["points"] == 0 and not result["usable"]


def test_many_duplicate_points_do_not_replace_spatial_extent():
    result = object_metrics(np.tile([10, 0, 1], (100, 1)), ([10, -1, 0], [13, 1, 2]))
    assert result["points"] == 100 and result["voxels"] == 1 and not result["usable"]


def test_object_metrics_are_evaluator_only_and_exclude_ground_outside_box():
    obj = np.column_stack([np.full(20, 10), np.linspace(-1, 1, 20), np.ones(20)])
    points = np.vstack([obj, np.zeros((100, 3))])
    before = points.copy()
    result = object_metrics(points, ([10, -1, 0], [13, 1, 2]))
    assert result["usable"] and result["points"] == 20
    np.testing.assert_array_equal(points, before)
