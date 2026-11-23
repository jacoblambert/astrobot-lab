"""Analytic registration contracts; no GT-fed correction or acceptance claim.

Run with TERRAIN_LIBRARY pointing to the built terrain core. The strict xfail is
an OPEN robustness failure, not permission to promote an all-occupied mapper.
"""
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper

LIBRARY = os.environ.get("TERRAIN_LIBRARY")
pytestmark = pytest.mark.skipif(not LIBRARY, reason="set TERRAIN_LIBRARY to built terrain library")


def flat_returns(obstacle=False):
    x, y = np.meshgrid(np.arange(-1., 1.001, .02), np.arange(-1., 1.001, .02))
    z = np.zeros_like(x)
    if obstacle:
        z[(abs(x - .6) < .12) & (abs(y) < .12)] = .08
    return np.column_stack([x.ravel(), y.ravel(), z.ravel()])


def accumulate(offsets, phase, obstacle=False):
    points = flat_returns(obstacle)
    with TerrainMapper(LIBRARY, width=40, height=40, origin=(-2. + phase, -2. + phase)) as mapper:
        for stamp, offset in enumerate(offsets, 1):
            shifted = points + [0., 0., offset]
            mapper.integrate(shifted, float(stamp), [0., 0., .4 + offset])
            result = mapper.project(float(stamp))
    return result


@pytest.mark.parametrize("phase", [0., .025, .05, .075])
@pytest.mark.parametrize("offset", [-.05, 0., .05])
def test_fixed_vertical_gauge_does_not_create_obstacles(phase, offset):
    layers = accumulate([offset] * 3, phase)
    assert np.all(layers[15:25, 15:25, 0] == 0)
    np.testing.assert_allclose(layers[15:25, 15:25, 2], offset, atol=1e-7)


@pytest.mark.parametrize("phase", [0., .025, .05, .075])
@pytest.mark.parametrize("offset", [-.05, .05])
def test_true_eight_cm_object_survives_a_fixed_vertical_gauge(phase, offset):
    layers = accumulate([offset] * 3, phase, obstacle=True)
    x, y = np.floor((np.array([.6, 0.]) - (-2. + phase)) / .1).astype(int)
    assert layers[y, x, 0] == 100


@pytest.mark.parametrize("phase", [0., .025, .05, .075])
@pytest.mark.xfail(strict=True, reason="OPEN: cross-time floor registration error is treated as obstacle height")
def test_varying_registration_must_not_certify_flat_ground_as_obstacle(phase):
    layers = accumulate([-.05, 0., .05], phase)
    # Unknown is acceptable in this diagnostic; unqualified free space is not
    # required. A fix must ALSO keep the separate real-obstacle contract above.
    assert not np.any(layers[15:25, 15:25, 0] == 100)
