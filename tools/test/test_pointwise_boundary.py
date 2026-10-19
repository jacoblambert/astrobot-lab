"""Density must not turn millimetre roughness into a dropoff; do not hide outliers."""

import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper

BASELINE = os.environ.get("TERRAIN_LIBRARY")
CANDIDATE = os.environ.get("TERRAIN_POINTWISE_LIBRARY")
pytestmark = pytest.mark.skipif(not BASELINE or not CANDIDATE, reason="explicit frozen baseline and candidate required")


def rough_floor():
    y, x = np.indices((190, 190))
    # Three millimetres peak-to-peak, far below the unchanged hazard height.
    return np.c_[x.ravel() * 0.02 - 1.895, y.ravel() * 0.02 - 1.895, (((x + y) % 2) * 0.003 - 0.0015).ravel()]


@pytest.mark.parametrize("phase", [0.0, 0.025, 0.05, 0.075])
@pytest.mark.parametrize("repeat", [1, 4])
def test_benign_native_roughness_not_amplified_by_return_count(phase, repeat):
    points = np.repeat(rough_floor(), repeat, axis=0)
    results = []
    for library in (BASELINE, CANDIDATE):
        with TerrainMapper(library, width=40, height=40, origin=(-2 + phase, -2 + phase)) as mapper:
            mapper.integrate(points, 1.0, [0, 0, 0.5])
            results.append(mapper.project(1.0))
    assert (results[0][8:32, 8:32, 1] == 8).any(), "lost density-dependent false-boundary reproducer"
    assert not (results[1][8:32, 8:32, 1] == 8).any()
    assert np.isin(results[1][8:32, 8:32, 0], [0, 20]).all()


@pytest.mark.parametrize("height", [-0.02, -0.008, 0.008, 0.02])
@pytest.mark.parametrize("duplicates", [1, 100])
def test_single_real_deviation_is_not_diluted_by_many_ground_returns(height, duplicates):
    floor = rough_floor()
    floor[:, 2] = 0
    local = np.array([[0.32, 0.32, 0], [0.34, 0.32, 0], [0.32, 0.34, 0], [0.34, 0.34, 0]])
    points = np.r_[floor, np.repeat(local, duplicates, axis=0), [[0.33, 0.33, height]]]
    with TerrainMapper(CANDIDATE, width=40, height=40, origin=(-2, -2)) as mapper:
        mapper.integrate(points, 1.0, [0, 0, 0.5])
        result = mapper.project(1.0)
    assert result[23, 23, 0] == -1
    assert result[23, 23, 1] == 8


def test_boundary_evidence_expires_not_overwritten_by_later_smooth_scan():
    floor = rough_floor()
    floor[:, 2] = 0
    cfg = dict(width=40, height=40, origin=(-2, -2), lifetime=1.2)
    with TerrainMapper(CANDIDATE, **cfg) as retained, TerrainMapper(CANDIDATE, **cfg) as fresh:
        retained.integrate(np.r_[floor, [[0.33, 0.33, -0.02]]], 1.0, [0, 0, 0.5])
        assert retained.project(1.0)[23, 23, 1] == 8
        for mapper in (retained, fresh):
            mapper.integrate(floor, 2.1, [0, 0, 0.5])
        assert retained.project(2.1)[23, 23, 1] == 8
        np.testing.assert_array_equal(retained.project(2.3), fresh.project(2.3))
