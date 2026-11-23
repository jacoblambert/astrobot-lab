import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ray_triangle_precision import intersections


def test_known_plane_order_backfaces_and_no_sensor_mutation():
    triangles = np.array([[[-1, -1, 0], [1, -1, 0], [0, 1, 0]], [[-1, -1, 1], [0, 1, 1], [1, -1, 1]]], dtype=float)
    original = triangles.copy()
    hits = intersections([0, 0, 2], [0, 0, -1], triangles)
    assert [h["triangle"] for h in hits] == [1, 0]
    assert [h["distance"] for h in hits] == [1.0, 2.0]
    np.testing.assert_array_equal(triangles, original)


def test_parallel_degenerate_outside_and_behind_are_not_hits():
    triangles = np.array([[[-1, -1, 0], [1, -1, 0], [0, 1, 0]], [[0, 0, 0]] * 3])
    assert not intersections([0, 0, 2], [1, 0, 0], triangles)
    assert not intersections([0, 0, 2], [0, 0, 1], triangles)
    assert not intersections([2, 0, 2], [0, 0, -1], triangles)
    assert not intersections([0, 0, 2], [0, 0, -1], triangles, max_distance=1)


def test_grazing_ray_known_intersection():
    triangle = np.array([[[-10, -10, 0], [10, -10, 0], [0, 10, 0]]])
    direction = np.array([1.0, 0.0, -1e-6])
    direction /= np.linalg.norm(direction)
    result = intersections([-1, 0, 1e-6], direction, triangle)
    assert len(result) == 1
    np.testing.assert_allclose(result[0]["position"], [0, 0, 0], atol=1e-12)


def test_archived_seed43_grazing_triangle_reference():
    # Copied, not fitted, from the regenerated float32 terrain vertices in
    # revision-7db0e84fa4b04c5c87b7a23c2d0f6d2a. Its full-mesh PhysX control
    # exactly reproduced the saved failure in revision-f369a1c1a2da4387b564f2b99e319676.
    triangles = [
        [
            [17.780000686645508, 27.81999969482422, -0.22116601467132568],
            [17.760000228881836, 27.81999969482422, -0.2199397087097168],
            [17.760000228881836, 27.799999237060547, -0.21774940192699432],
        ]
    ]
    origin = [22.09556123768997, 16.76841678284039, 0.7542820444308753]
    direction = [-0.36325421539559494, 0.9280865429455261, -0.08186417898417417]
    hit = intersections(origin, direction, triangles)[0]
    assert hit["distance"] == pytest.approx(11.907916333742474, abs=1e-10)
    # A reproducer for the engine discrepancy, NOT an instruction to accept it.
    physx_distance = 11.906257629394531
    assert 0.0016 < hit["distance"] - physx_distance < 0.0017


@pytest.mark.parametrize("direction", [[0, 0, 0], [0, 0, 2], [0, 0, float("nan")]])
def test_invalid_directions_rejected(direction):
    with pytest.raises(ValueError):
        intersections([0, 0, 1], direction, np.zeros((1, 3, 3)))
