from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.reference_mesh import HeightMeshReference


VERTICES = np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]], dtype=float)
TRIANGLES = np.array([[0, 1, 2], [0, 2, 3]])
PATCH = np.array([[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]])


@pytest.mark.parametrize("yaw", [0, 0.37, 1.57])
@pytest.mark.parametrize("phase", [0, 0.025, 0.05, 0.075])
def test_continuous_sloped_patch_transforms_without_raster_or_fit(yaw, phase):
    vertices = VERTICES.copy()
    vertices[:, 2] = 0.1 * vertices[:, 0] + 0.2 * vertices[:, 1] + 0.4
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    pose[:3, 3] = [40, 38, 1]
    patch = PATCH + phase
    transformed = patch @ pose[:2, :2].T + pose[:2, 3]
    result = HeightMeshReference(vertices, TRIANGLES, pose).height_patch(transformed)
    assert result["complete"] and not result["acceptance"]
    expected = 1.4 + patch @ [0.1, 0.2]
    assert result["minimum_height_m"] == pytest.approx(min(expected))
    assert result["maximum_height_m"] == pytest.approx(max(expected))
    assert result["maximum_slope"] == pytest.approx(np.hypot(0.1, 0.2))
    assert result["covered_area_m2"] == pytest.approx(0.04)


def test_missing_triangle_is_unknown_not_interpolated_free():
    report = HeightMeshReference(VERTICES, TRIANGLES[:1], np.eye(4)).height_patch(PATCH)
    assert not report["complete"] and report["reason"] == "uncovered_reference_area"
    assert report["covered_area_m2"] == pytest.approx(0.02)


def test_full_se3_reference_preserves_physical_height_and_slope():
    vertices = VERTICES.copy()
    vertices[:, 2] = vertices[:, :2] @ [0.1, 0.2] + 0.4
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", [0.12, -0.07, 0.31]).as_matrix()
    pose[:3, 3] = [-20, -20, 2]
    contact = np.c_[PATCH, PATCH @ [0.1, 0.2] + 0.4]
    contact = contact @ pose[:3, :3].T + pose[:3, 3]
    normal = pose[:3, :3] @ [-0.1, -0.2, 1]
    report = HeightMeshReference(vertices, TRIANGLES, pose).height_patch(contact[:, :2])
    assert report["complete"]
    assert report["minimum_height_m"] == pytest.approx(contact[:, 2].min())
    assert report["maximum_height_m"] == pytest.approx(contact[:, 2].max())
    assert report["maximum_slope"] == pytest.approx(np.linalg.norm(normal[:2]) / abs(normal[2]))


def test_extreme_finite_geometry_cannot_overflow_into_a_valid_reference():
    with pytest.raises(ValueError):
        HeightMeshReference(VERTICES * 1e200, TRIANGLES, np.eye(4))


def test_overlap_cannot_numerically_cancel_a_hole_in_area_sum():
    report = HeightMeshReference(VERTICES, np.array([[0, 1, 2], [0, 1, 2]]), np.eye(4)).height_patch(PATCH)
    assert not report["complete"] and report["reason"] == "overlapping_or_multilevel_reference"


def test_multilevel_reference_does_not_choose_bottom_or_top():
    vertices = np.vstack([VERTICES, VERTICES + [0, 0, 0.2]])
    report = HeightMeshReference(vertices, np.vstack([TRIANGLES, TRIANGLES + 4]), np.eye(4)).height_patch(PATCH)
    assert report["reason"] == "overlapping_or_multilevel_reference"


def test_vertical_wall_requires_volume_reference():
    vertices = np.array([[0, -1, 0], [0, 1, 0], [0, 0, 1]])
    report = HeightMeshReference(vertices, np.array([[0, 1, 2]]), np.eye(4)).height_patch(PATCH)
    assert report["reason"] == "vertical_face_requires_volume_reference"


def test_reference_extent_and_work_budget_remain_unknown():
    scene = HeightMeshReference(VERTICES, TRIANGLES, np.eye(4))
    assert not scene.height_patch(PATCH + [1, 0])["complete"]
    limited = HeightMeshReference(VERTICES, TRIANGLES, np.eye(4), max_patch_triangles=1)
    assert limited.height_patch(PATCH)["reason"] == "reference_work_budget"


@pytest.mark.parametrize(
    "vertices,triangles",
    [
        (VERTICES * np.nan, TRIANGLES),
        (VERTICES, TRIANGLES.astype(float)),
        (VERTICES, np.array([[0, 1, 4]])),
        (VERTICES, np.array([[0, 0, 0]])),
    ],
)
def test_invalid_geometry_rejected(vertices, triangles):
    with pytest.raises(ValueError):
        HeightMeshReference(vertices, triangles, np.eye(4))


def test_concave_contact_polygon_rejected_and_input_arrays_owned():
    vertices = VERTICES.copy()
    scene = HeightMeshReference(vertices, TRIANGLES, np.eye(4))
    vertices[:] = 100
    assert scene.height_patch(PATCH)["minimum_height_m"] == 0
    with pytest.raises(ValueError):
        scene.height_patch(np.array([[0, 0], [1, 0], [0.1, 0.1], [0, 1]]))
