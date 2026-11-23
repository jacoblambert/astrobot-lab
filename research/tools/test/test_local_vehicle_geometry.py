from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.inspect_vehicle import cube_bounds


def test_chassis_cube_uses_size_not_double_scaled_authored_extent():
    pose = np.eye(4)
    pose[:3, :3] = np.diag([1.0074, 0.5709, 0.2675])
    pose[2, 3] = 0.12498
    result = cube_bounds(1.0, pose)
    np.testing.assert_allclose(result["minimum"], [-0.5037, -0.28545, -0.00877])
    np.testing.assert_allclose(result["maximum"], [0.5037, 0.28545, 0.25873])
    # These coordinates are INSIDE the primitive but OUTSIDE the old cached box.
    assert result["maximum"][1] > 0.25 and result["minimum"][2] < 0.04


def test_rotated_nonuniform_cube_retains_all_corners():
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", [0.3, -0.2, 0.8]).as_matrix() @ np.diag([1.2, 0.5, 0.2])
    matrix[:3, 3] = [1, 2, 3]
    bounds = cube_bounds(2, matrix)
    radius = np.abs(matrix[:3, :3]).sum(axis=1)
    np.testing.assert_allclose(bounds["minimum"], matrix[:3, 3] - radius)
    np.testing.assert_allclose(bounds["maximum"], matrix[:3, 3] + radius)


@pytest.mark.parametrize("fault", ["size", "nan", "singular", "projective"])
def test_invalid_primitive_bound_rejected(fault):
    transform = np.eye(4)
    size = 0 if fault == "size" else 1
    if fault == "nan":
        transform[0, 3] = np.nan
    if fault == "singular":
        transform[0, 0] = 0
    if fault == "projective":
        transform[3, 0] = 1
    with pytest.raises(ValueError):
        cube_bounds(size, transform)
