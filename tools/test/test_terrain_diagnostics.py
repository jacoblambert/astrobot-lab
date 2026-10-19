"""Opt-in diagnostic target contracts; no ROS/GT or simulator required.

Set TERRAIN_DIAGNOSTIC_LIBRARY and TERRAIN_LIBRARY to freshly built diagnostic
and default targets. Ordinary host runs skip the native contracts if absent.
"""
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper


@pytest.fixture
def libraries():
    paths = [os.environ.get(name) for name in ("TERRAIN_LIBRARY", "TERRAIN_DIAGNOSTIC_LIBRARY")]
    if not all(paths):
        pytest.skip("requires default and diagnostic terrain targets")
    return paths


def surface(kind):
    x, y = np.meshgrid(np.arange(-1.9, 1.9, .015), np.arange(-1.9, 1.9, .015))
    x, y = x.ravel(), y.ravel()
    floor = np.c_[x, y, np.zeros(len(x))]
    if kind == "hole":
        floor = floor[~((abs(x - .6) < .3) & (abs(y - .6) < .3))]
    elif kind == "positive":
        # The foreground face occludes ground in its own coarse cell. Dense
        # ground mixed into that cell would hide the wall-normal reproducer.
        floor = floor[~((x >= .3) & (x < .4) & (y >= .3) & (y < .9))]
    if kind == "flat":
        return floor
    yy, zz = np.meshgrid(np.arange(.3, .9, .012), np.arange(.002, .10, .004))
    z = zz.ravel() * (-1 if kind == "hole" else 1)
    wall = np.c_[np.full(len(z), .32), yy.ravel(), z]
    return np.r_[floor, wall]


@pytest.mark.parametrize("kind", ["flat", "positive", "hole"])
@pytest.mark.parametrize("phase", [0., .025, .05, .075])
@pytest.mark.parametrize("rotation", [0., .17, .61])
@pytest.mark.parametrize("slope", [-.15, 0., .15])
def test_diagnostic_projection_preserves_layers_edges_and_expiry(libraries, kind, phase, rotation, slope):
    points = surface(kind)
    c, s = np.cos(rotation), np.sin(rotation)
    points[:, :2] = points[:, :2] @ np.array([[c, s], [-s, c]])
    points[:, 2] += slope * points[:, 0]
    origin = (-2. + phase, -2. + phase)
    # Independent geometric probes, evaluator-only: do not pass these positions
    # or labels to either mapper. Unknown is allowed; false-free is not.
    if kind == "hole":
        px, py = np.meshgrid(np.arange(.31, .9, .02), np.arange(.31, .9, .02))
        probes = np.c_[px.ravel(), py.ravel()]
    elif kind == "positive":
        probes = np.c_[np.full(30, .32), np.linspace(.31, .89, 30)]
    else:
        probes = np.empty((0, 2))
    probes = probes @ np.array([[c, s], [-s, c]])
    indices = np.floor((probes - origin) / .1).astype(int)
    with TerrainMapper(libraries[0], width=40, height=40, origin=origin) as ordinary, \
         TerrainMapper(libraries[1], width=40, height=40, origin=origin) as diagnostic:
        with pytest.raises(ValueError, match="successful projection"):
            diagnostic.diagnostics()
        with pytest.raises(ValueError, match="not built"):
            ordinary.diagnostics()
        for mapper in (ordinary, diagnostic):
            mapper.integrate(points, 1., [0., 0., .5])
        for stamp in (1., 31., 31.001):
            a, b = ordinary.project(stamp), diagnostic.project(stamp)
            assert np.array_equal(a, b, equal_nan=True)
            cells, corners = diagnostic.diagnostics()
            assert cells.shape == (40, 40, 4) and corners.shape == (41, 41, 8)
            assert cells.dtype == corners.dtype == np.uint32
            # Reads must not mutate either outputs or the diagnostics snapshot.
            assert np.array_equal(diagnostic.diagnostics()[0], cells)
            if len(indices):
                assert not np.isin(b[indices[:, 1], indices[:, 0], 0], [0, 20]).any()
            if stamp > 31.:
                assert not cells.any() and not corners.any()
            elif kind == "flat":
                assert not cells[:, :, 0].any()
                assert (corners[:, :, 0] & 4).any()  # adjacent-cell certificate
            elif kind == "hole":
                # These bits refer to the ESTIMATED plane, not the analytic GT.
                # On positive slopes the pre-existing local fit can select the
                # lower wall as ground and label the rim occupied instead.
                # Keep all sloped hole probes non-free above; do not require a
                # particular estimator-relative diagnostic label in that case.
                if slope == 0.:
                    assert (cells[:, :, 0] & 1).any()
                    assert (cells[:, :, 1] & 1).any()
            else:
                assert (cells[:, :, 0] & 4).any()  # actual raised obstacle
                # Removing the wall guard merely because the face points up
                # failed the rotated wall probes. Retain conservative extent
                # protection until a replacement certifies the whole cell.
        diagnostic.close()
        with pytest.raises(RuntimeError, match="closed"):
            diagnostic.diagnostics()
