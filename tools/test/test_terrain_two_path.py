"""Two-path native surface/boundary experiment; no promotion from these tests.

TERRAIN_TWO_PATH_LIBRARY is the candidate; TERRAIN_LIBRARY is the frozen parent.
NumPy/pytest only. Every independent probe remains fixed under registration
perturbations. Absolute safety and nonregression are separate gates.
"""
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper
from test_terrain_normal_confidence import face_fixture, jittered_ring_fixture

BASELINE = os.environ.get("TERRAIN_LIBRARY")
CANDIDATE = os.environ.get("TERRAIN_TWO_PATH_LIBRARY")
pytestmark = pytest.mark.skipif(not BASELINE or not CANDIDATE, reason="baseline and two-path candidate required")


@pytest.mark.parametrize("kind", ["positive", "hole"])
@pytest.mark.parametrize("phase", [0., .025, .05, .075])
@pytest.mark.parametrize("rotation", [0., .17, .61])
@pytest.mark.parametrize("height", [.02, .04, .06, .10])
@pytest.mark.parametrize("thickness", [.003, .008, .015])
@pytest.mark.parametrize("motion", [False, True])
def test_native_faces_absolute_safety_and_per_probe_nonregression(kind, phase, rotation, height, thickness, motion):
    points, cells, origin = face_fixture(kind, phase, rotation, thickness, height)
    offsets = ([0., 0., 0.],)*3 if not motion else ([0., 0., 0.], [.008, -.006, .012], [-.006, .008, -.012])
    results = []
    for library in (BASELINE, CANDIDATE):
        masks = []
        with TerrainMapper(library, width=40, height=40, origin=origin) as mapper:
            for index, offset in enumerate(offsets):
                stamp = float(index+1)
                mapper.integrate(points+offset, stamp, np.array([0., 0., .5])+offset)
                layers = mapper.project(stamp)
                masks.append(np.isin(layers[cells[:, 1], cells[:, 0], 0], [0, 20]))
        results.append(np.array(masks))
    assert not np.any(results[1] & ~results[0]), "new false-free probes"
    assert not results[1].any(), "absolute partial-face safety failure remains"


@pytest.mark.parametrize("phase", [0., .025, .05, .075])
def test_rigid_flat_registration_is_not_a_physical_face(phase):
    frames = jittered_ring_fixture(0.)
    # Unlike the earlier local-thickness diagnostic, every point in one input
    # lies on the SAME plane: a rigid Z gauge error, not a real local step.
    outputs = []
    for library in (BASELINE, CANDIDATE):
        with TerrainMapper(library, width=40, height=40, origin=(-2.+phase, -2.+phase)) as mapper:
            for index, points in enumerate(frames):
                points = points.copy()
                points[:, 2] = points[-1, 2]
                stamp = 1.+index*.1
                mapper.integrate(points, stamp, [0., 0., .5+points[-1, 2]])
            outputs.append(mapper.project(stamp))
    assert not np.any(outputs[1][:, :, 0] == 100)
    if phase == 0.:
        assert np.any(outputs[0][:, :, 1] == 8), "lost false-normal reproducer"
        assert np.count_nonzero(outputs[1][:, :, 1] == 8) < np.count_nonzero(outputs[0][:, :, 1] == 8)
    assert np.count_nonzero(outputs[1][:, :, 0] == 0) >= np.count_nonzero(outputs[0][:, :, 0] == 0)


def test_boundary_warning_expires_without_ground_rejuvenation():
    points, cells, origin = face_fixture("positive", .025, .17, .008, .02)
    x, y = np.meshgrid(np.arange(-1.89, 1.90, .02), np.arange(-1.89, 1.90, .02))
    floor = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    cfg = dict(width=40, height=40, origin=origin, lifetime=1.2)
    with TerrainMapper(CANDIDATE, **cfg) as retained, TerrainMapper(CANDIDATE, **cfg) as fresh:
        retained.integrate(points, 1., [0., 0., .5])
        before = retained.project(1.)
        assert not np.isin(before[cells[:, 1], cells[:, 0], 0], [0, 20]).any()
        for mapper in (retained, fresh):
            mapper.integrate(floor, 2.1, [0., 0., .5])
        live = retained.project(2.1)
        assert not np.isin(live[cells[:, 1], cells[:, 0], 0], [0, 20]).any()
        np.testing.assert_array_equal(retained.project(2.3), fresh.project(2.3))


@pytest.mark.parametrize("phase", [0., .025, .05, .075])
@pytest.mark.parametrize("rotation", [0., .17, .61])
@pytest.mark.parametrize("motion", [False, True])
def test_resolved_end_does_not_clear_the_rest_of_a_partly_observed_face(phase, rotation, motion):
    points, cells, origin = face_fixture("positive", phase, rotation, .008, .02)
    yy, zz = np.meshgrid(np.arange(.31, .385, .01), np.arange(.08, .125, .005))
    resolved_end = np.c_[np.full(yy.size, .32), yy.ravel(), zz.ravel()]
    c, s = np.cos(rotation), np.sin(rotation)
    resolved_end[:, :2] = resolved_end[:, :2] @ np.array([[c, s], [-s, c]])
    points = np.r_[points, resolved_end]
    offsets = ([0., 0., 0.],)*3 if not motion else ([0., 0., 0.], [.008, -.006, .012], [-.006, .008, -.012])
    with TerrainMapper(CANDIDATE, width=40, height=40, origin=origin) as mapper:
        for index, offset in enumerate(offsets):
            stamp = float(index+1)
            mapper.integrate(points+offset, stamp, np.array([0., 0., .5])+offset)
            layers = mapper.project(stamp)
            assert (layers[:, :, 0] == 100).any(), "fixture lost its resolved obstacle end"
            assert not np.isin(layers[cells[:, 1], cells[:, 0], 0], [0, 20]).any()
