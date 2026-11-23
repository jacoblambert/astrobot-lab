import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.costmap import inflate, map_from_grid


def test_cost_taper_preserves_classes_and_unknown():
    classes = np.zeros((21, 21))
    classes[10, 10] = 100
    classes[10, 11] = -1
    classes[0, 0] = 20
    before = classes.copy()
    costs = inflate(classes)
    assert costs[10, 10] == 100
    assert costs[10, 11] == -1
    assert costs[0, 0] == 20
    assert costs[10, 9] > costs[10, 8] > costs[10, 1] > costs[10, 0] == 0
    assert np.count_nonzero(costs == 100) == 1
    np.testing.assert_array_equal(classes, before)


@pytest.mark.parametrize("value", [-1, 0, 20, 100])
def test_uniform_grid_has_no_phantom_boundary_obstacle(value):
    classes = np.full((60, 60), value)
    np.testing.assert_array_equal(inflate(classes), classes)


@pytest.mark.parametrize("kwargs", [dict(radius=0), dict(radius=float("nan")), dict(resolution=-1)])
def test_bad_geometry_rejected(kwargs):
    with pytest.raises(ValueError):
        inflate(np.zeros((2, 2)), **kwargs)


def test_nonclass_values_rejected():
    with pytest.raises(ValueError):
        inflate([[0, 50]])


def test_map_grid_preserves_full_frame_rotation_translation_and_corner():
    local_from_map = np.eye(4)
    local_from_map[:3, :3] = Rotation.from_euler("xyz", [0.1, -0.2, 0.3]).as_matrix()
    local_from_map[:3, 3] = [-40, -38, 0.1]
    result = map_from_grid(dict(origin=[-3.0, -3.0], local_from_map=local_from_map))
    corner = local_from_map @ result @ [0, 0, 0, 1]
    np.testing.assert_allclose(corner, [-3, -3, 0, 1], atol=1e-12)
    np.testing.assert_allclose((local_from_map @ result)[:3, :3], np.eye(3), atol=1e-12)
