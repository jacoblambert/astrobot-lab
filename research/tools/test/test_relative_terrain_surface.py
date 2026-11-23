"""Frozen C++ mapper: smooth incline versus relative obstacle height.

Host NumPy/pytest, TERRAIN_LIBRARY required. Analytic geometry contracts only;
not real LiDAR coverage, moving SLAM, terrain recall or vehicle certification.
"""
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper

LIBRARY = os.environ.get("TERRAIN_LIBRARY")
pytestmark = pytest.mark.skipif(not LIBRARY, reason="set TERRAIN_LIBRARY to frozen mapper")


@pytest.mark.parametrize("slope", [(0., 0.), (.18, 0.), (0., -.18), (.2, -.2), (.3, 0.)])
@pytest.mark.parametrize("phase", [0., .025, .05, .075])
def test_incline_is_free_but_eight_cm_object_is_not_absorbed(slope, phase):
    x, y = np.meshgrid(np.arange(-1.19, 1.20, .02), np.arange(-1.19, 1.20, .02))
    z = slope[0]*x + slope[1]*y
    origin = (-2.+phase, -2.+phase)
    for obstacle in (False, True):
        heights = z.copy()
        if obstacle:
            heights[(abs(x-.6) < .12) & (abs(y) < .12)] += .08
        points = np.column_stack((x.ravel(), y.ravel(), heights.ravel()))
        with TerrainMapper(LIBRARY, width=40, height=40, origin=origin) as mapper:
            for stamp in (1., 2., 3.):
                mapper.integrate(points, stamp, [0., 0., .5])
            layers = mapper.project(3.)
        center = np.floor((np.array([-.4, 0.])-origin)/.1).astype(int)
        assert layers[center[1], center[0], 0] == 0, "smooth incline was rejected as obstacle/unsupported"
        rock = np.floor((np.array([.6, 0.])-origin)/.1).astype(int)
        assert layers[rock[1], rock[0], 0] == (100 if obstacle else 0)
