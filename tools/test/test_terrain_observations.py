"""Rigid-observation evidence contracts, not terrain acceptance.

TERRAIN_OBSERVATION_LIBRARY: experimental observation diagnostics.
TERRAIN_LIBRARY: frozen, uninstrumented baseline. Optional
TERRAIN_OBSERVATION_CANDIDATE: uninstrumented candidate.
"""
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper
from replay_natural_terrain import observation_attribution
from test_terrain_normal_confidence import face_fixture, jittered_ring_fixture

BASELINE = os.environ.get("TERRAIN_LIBRARY")
DIAGNOSTIC = os.environ.get("TERRAIN_OBSERVATION_LIBRARY")
CANDIDATE = os.environ.get("TERRAIN_OBSERVATION_CANDIDATE")
native = pytest.mark.skipif(not BASELINE or not DIAGNOSTIC, reason="baseline and observation build required")


def patch_frames(area=False):
    frames = jittered_ring_fixture(0.)
    if area:
        for index, points in enumerate(frames):
            ring = points[-19:]
            assert len(ring) == 19
            frames[index] = np.r_[points[:-19], ring-[0., .003, 0.], ring, ring+[0., .003, 0.]]
    return frames


@native
@pytest.mark.parametrize("area", [False, True])
def test_within_between_decomposition_and_exact_baseline(area):
    frames = patch_frames(area)
    with TerrainMapper(BASELINE, width=40, height=40, origin=(-2., -2.)) as baseline, \
         TerrainMapper(DIAGNOSTIC, width=40, height=40, origin=(-2., -2.)) as diagnostic:
        with pytest.raises(ValueError, match="successful projection"):
            diagnostic.observations()
        with pytest.raises(ValueError, match="not built"):
            baseline.observations()
        for index, points in enumerate(frames):
            stamp = 1.+index*.1
            for mapper in (baseline, diagnostic):
                mapper.integrate(points, stamp, [0., 0., .5])
            np.testing.assert_array_equal(baseline.project(stamp), diagnostic.project(stamp))
        row = diagnostic.observations()[23, 23]
        assert row[0] == 1
        assert row[1:5].sum() == 8
        assert row[3 if area else 1] == 8
        assert row[8] == (1 if area else 0)
        assert row[7] > 0.
        np.testing.assert_allclose(row[5], row[6]+row[7], atol=1e-12)
        groups = [p[(p[:, 0] >= .3)&(p[:, 0] < .4)&(p[:, 1] >= .3)&(p[:, 1] < .4)] for p in frames]
        all_points = np.concatenate(groups)
        total = np.sum((all_points-all_points.mean(axis=0))**2)
        within = sum(np.sum((p-p.mean(axis=0))**2) for p in groups)
        np.testing.assert_allclose(row[5:8], [total, within, total-within], atol=1e-10)
        assert np.array_equal(diagnostic.observations(), diagnostic.observations())
        diagnostic.project(stamp+30.001)
        assert not diagnostic.observations().any()


@native
def test_observation_buckets_expire_without_rejuvenating_votes():
    points = patch_frames(True)[0]
    geometry = dict(width=40, height=40, origin=(-2., -2.), lifetime=1.2)
    with TerrainMapper(DIAGNOSTIC, **geometry) as retained, TerrainMapper(DIAGNOSTIC, **geometry) as fresh:
        retained.integrate(points, 1., [0., 0., .5])
        retained.integrate(points, 2.1, [0., 0., .5])
        fresh.integrate(points, 2.1, [0., 0., .5])
        np.testing.assert_array_equal(retained.project(2.3), fresh.project(2.3))
        np.testing.assert_allclose(retained.observations(), fresh.observations(), atol=1e-10)


@native
@pytest.mark.skipif(not CANDIDATE, reason="candidate not requested")
@pytest.mark.parametrize("area", [False, True])
def test_candidate_requires_resolved_ground_not_single_ring(area):
    results = []
    for library in (BASELINE, CANDIDATE):
        with TerrainMapper(library, width=40, height=40, origin=(-2., -2.)) as mapper:
            for index, points in enumerate(patch_frames(area)):
                stamp = 1.+index*.1
                mapper.integrate(points, stamp, [0., 0., .5])
            results.append(mapper.project(stamp))
    if not area:
        np.testing.assert_array_equal(*results)
    else:
        assert (results[0][:, :, 1] == 8).any()
        assert not (results[1][:, :, 1] == 8).any()
        assert not (results[1][:, :, 0] == 100).any()


def test_eligible_source_attribution_requires_no_competing_source_or_below_plane():
    layers = np.zeros((5, 5, 6)); layers[:, :, 1] = 8
    cells = np.zeros((5, 5, 4), dtype=np.uint32)
    observations = np.zeros((5, 5, 13))
    observations[0, 0, [0, 8]] = 1
    result = observation_attribution(layers, layers, cells, observations)
    assert result["lost_cells_with_only_qualified_sources"] == 4
    cells[0, 0, 1] = 1
    assert observation_attribution(layers, layers, cells, observations)["lost_cells_with_only_qualified_sources"] == 3
    observations[1, 1, 0] = 1
    assert observation_attribution(layers, layers, cells, observations)["lost_cells_with_only_qualified_sources"] == 0
    with pytest.raises(ValueError, match="invalid observation"):
        observation_attribution(layers, layers, cells, observations[:, :, :12])


def test_spatial_attribution_is_separate_from_cell_local_evidence():
    layers = np.zeros((5, 5, 6)); layers[:, :, 1] = 8
    cells = np.zeros((5, 5, 4), dtype=np.uint32)
    observations = np.zeros((5, 5, 22))
    observations[2, 2, [0, 17]] = 1
    result = observation_attribution(layers, layers, cells, observations)
    assert result["qualified_ground_only_layers"] == 0
    assert result["lost_cells_with_only_qualified_sources"] == 0
    assert result["spatial_context"]["qualified_ground_only_layers"] == 1
    assert result["spatial_context"]["lost_cells_with_only_qualified_sources"] == 9
    assert result["spatial_context"]["diagnostic_only"]


@native
@pytest.mark.skipif(not CANDIDATE, reason="candidate not requested")
@pytest.mark.parametrize("kind", ["positive", "hole"])
@pytest.mark.parametrize("phase", [0., .025, .05, .075])
@pytest.mark.parametrize("rotation", [0., .17, .61])
@pytest.mark.parametrize("height", [.02, .04, .06, .10])
@pytest.mark.parametrize("moving_registration", [False, True])
def test_repeated_partial_faces_have_no_new_false_free_probes(kind, phase, rotation, height, moving_registration):
    points, cells, origin = face_fixture(kind, phase, rotation, .008, height)
    offsets = ([0., 0., 0.],)*3 if not moving_registration else ([0., 0., 0.], [.008, -.006, .012], [-.006, .008, -.012])
    results = []
    for library in (BASELINE, CANDIDATE):
        sequence = []
        with TerrainMapper(library, width=40, height=40, origin=origin) as mapper:
            for index, offset in enumerate(offsets):
                mapper.integrate(points+offset, float(index+1), np.array([0., 0., .5])+offset)
                layers = mapper.project(float(index+1))
                sequence.append(np.isin(layers[cells[:, 1], cells[:, 0], 0], [0, 20]))
        results.append(np.array(sequence))
    assert not np.any(results[1] & ~results[0]), "new hazard probes became free during repeated evidence"
