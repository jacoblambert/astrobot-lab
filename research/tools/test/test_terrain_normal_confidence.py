"""Opt-in normal-confidence candidate regressions, no simulator or GT-fed correction.

TERRAIN_LIBRARY is the candidate; BASELINE_TERRAIN_LIBRARY is the frozen core.
Analytic evaluator geometry is never given to the mapper as a label.
"""
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper
from test_terrain_diagnostics import surface

LIBRARY = os.environ.get("TERRAIN_LIBRARY")
BASELINE = os.environ.get("BASELINE_TERRAIN_LIBRARY")
pytestmark = pytest.mark.skipif(not LIBRARY or not BASELINE, reason="candidate and frozen baseline required")


def jittered_ring_fixture(rotation):
    x, y = np.meshgrid(np.arange(-1.89, 1.90, .02), np.arange(-1.89, 1.90, .02))
    floor = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    # Replace one observed ground patch by repeated ring segments with <=1.5 cm
    # height and <=0.8 cm lateral registration error, not a physical wall.
    floor = floor[~((floor[:, 0] >= .3) & (floor[:, 0] < .4) &
                   (floor[:, 1] >= .3) & (floor[:, 1] < .4))]
    frames = []
    c, s = np.cos(rotation), np.sin(rotation)
    r = np.array([[c, -s], [s, c]])
    for angle in np.arange(8)*np.pi/4:
        xx = np.arange(.305, .396, .005)
        ring = np.c_[xx, np.full(len(xx), .35+.008*np.sin(angle)), np.full(len(xx), .015*np.cos(angle))]
        points = np.r_[floor, ring]
        points[:, :2] = points[:, :2]@r.T
        frames.append(points)
    return frames


@pytest.mark.parametrize("phase", [0., .025, .05, .075])
@pytest.mark.parametrize("rotation", [0., .17, .61])
def test_small_registration_thickness_does_not_assert_a_wall_normal(phase, rotation):
    outputs = []
    for library in (BASELINE, LIBRARY):
        with TerrainMapper(library, width=40, height=40, origin=(-2.+phase, -2.+phase)) as mapper:
            for index, points in enumerate(jittered_ring_fixture(rotation)):
                stamp = 1.+index*.1
                mapper.integrate(points, stamp, [0., 0., .5])
            outputs.append(mapper.project(stamp))
    assert not np.any(outputs[1][:, :, 0] == 100)
    # Candidate may still be unknown for insufficient support, but cannot claim
    # a resolved wall solely from volumetric registration spread.
    assert np.count_nonzero(outputs[1][:, :, 1] == 8) <= np.count_nonzero(outputs[0][:, :, 1] == 8)
    if phase == 0. and rotation == 0.:
        assert np.count_nonzero(outputs[0][:, :, 1] == 8) > 0, "lost baseline reproducer"
        assert np.count_nonzero(outputs[1][:, :, 1] == 8) == 0


@pytest.mark.parametrize("kind", ["positive", "hole"])
@pytest.mark.parametrize("phase", [0., .025, .05, .075])
@pytest.mark.parametrize("rotation", [0., .17, .61])
@pytest.mark.parametrize("thickness", [.003, .008, .015])
@pytest.mark.parametrize("visible_height", [.02, .04, .06, .10])
@pytest.mark.parametrize("gate", ["absolute_safety", "no_new_false_free"])
def test_thick_observed_faces_do_not_become_drivable(kind, phase, rotation, thickness, visible_height, gate):
    outcomes = face_outcomes(kind, phase, rotation, thickness, visible_height)
    if gate == "no_new_false_free":
        assert not np.any(outcomes[1] & ~outcomes[0]), "new hazard probes became free, even within already-failing cases"
    else:
        assert not any(mask.any() for mask in outcomes), [int(mask.sum()) for mask in outcomes]


def face_outcomes(kind, phase, rotation, thickness, visible_height):
    """A noisy/rough true face also has volumetric covariance: don't erase it.

    Surface-normal X perturbations stay within the original occluded cell.
    The independent wall/void probes remain fixed, never mapper inputs.
    """
    points, cells, origin = face_fixture(kind, phase, rotation, thickness, visible_height)
    outcomes = []
    for library in (BASELINE, LIBRARY):
        with TerrainMapper(library, width=40, height=40, origin=origin) as mapper:
            mapper.integrate(points, 1., [0., 0., .5])
            layers = mapper.project(1.)
        outcomes.append(np.isin(layers[cells[:, 1], cells[:, 0], 0], [0, 20]))
    return outcomes


def face_fixture(kind, phase, rotation, thickness, visible_height):
    points = surface(kind)
    # A highest return below 8 cm is not proof of an obstacle's actual top.
    # Include truncated faces; full-height returns alone exercise occupancy,
    # not the uncertain-normal safety decision under test.
    points = points[np.abs(points[:, 2]) <= visible_height]
    wall = points[:, 2] != 0.
    rng = np.random.default_rng(913)
    points[wall, 0] += rng.uniform(-thickness, thickness, np.count_nonzero(wall))
    c, s = np.cos(rotation), np.sin(rotation)
    rotation_matrix = np.array([[c, s], [-s, c]])
    points[:, :2] = points[:, :2] @ rotation_matrix
    if kind == "positive":
        probes = np.c_[np.full(30, .32), np.linspace(.31, .89, 30)]
    else:
        x, y = np.meshgrid(np.arange(.31, .9, .02), np.arange(.31, .9, .02))
        probes = np.c_[x.ravel(), y.ravel()]
    origin = np.array([-2.+phase, -2.+phase])
    cells = np.floor((probes @ rotation_matrix-origin)/.1).astype(int)
    return points, cells, origin
