"""Opt-in persistent lattice shifts: no backfill, stamp refresh or resurrection."""

import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.rolling_mapper import RollingTerrainMapper
from local_terrain.core import LocalConfig, LocalReconstructor
from test_local_terrain import feed, revision, transform


LIBRARY = os.environ.get("TERRAIN_ROLLING_LIBRARY")
pytestmark = pytest.mark.skipif(not LIBRARY, reason="explicit --rolling-grid library required")


def mapper():
    return RollingTerrainMapper(LIBRARY, width=60, height=60, origin=(-3, -3), lifetime=30)


def cloud():
    # Avoid exact lattice boundaries; all source geometry is far inside both
    # grid extents, so a small shift does not alter native neighborhood inputs.
    x, y = np.meshgrid(np.arange(-1.473, 1.5, 0.033), np.arange(-1.479, 1.5, 0.033))
    z = 0.04 * x + 0.03 * y
    z[(abs(x - 0.8) < 0.08) & (abs(y) < 0.12)] += 0.15
    return np.c_[x.ravel(), y.ravel(), z.ravel()]


@pytest.mark.parametrize("offset", [(1, 0), (-1, 0), (0, 2), (2, -2)])
def test_interior_ground_and_hazard_keep_world_location_age_and_height(offset):
    with mapper() as terrain:
        for stamp in (0.0, 0.1, 0.2):
            terrain.integrate(cloud(), stamp, [0, 0, 0.5])
        before = terrain.project(1.0)
        assert (before[:, :, 0] == 100).any() and (before[:, :, 0] == 0).any()
        dx, dy = offset
        terrain.shift_origin((-3 + 0.1 * dx, -3 + 0.1 * dy))
        after = terrain.project(1.0)
        np.testing.assert_allclose(
            after[10:50, 10:50], before[10 + dy : 50 + dy, 10 + dx : 50 + dx], atol=1e-6, rtol=0, equal_nan=True
        )
        terrain.shift_origin((-3, -3))
        np.testing.assert_allclose(terrain.project(1.0), before, atol=1e-6, rtol=0, equal_nan=True)
        terrain.project(30.1)
        # Every bucket started at 0.0. Shifting must not make it young again.
        assert np.all(terrain.project(30.3)[:, :, 0] == -1)


def test_pending_native_votes_flush_into_original_cells_before_shift():
    with mapper() as deferred, mapper() as flushed:
        for index in range(4):
            points = cloud()[index::4]
            for terrain in (deferred, flushed):
                terrain.integrate(points, index * 0.1, [0, 0, 0.5])
            flushed.project(index * 0.1)
        for terrain in (deferred, flushed):
            terrain.shift_origin((-2.9, -3.1))
        # 3 * 0.1 is slightly later than literal 0.3; retain the strict clock
        # guard instead of rounding/relaxing production timestamps for a test.
        assert deferred.project(0.4).tobytes() == flushed.project(0.4).tobytes()


def test_evicted_region_and_returning_grid_are_unknown_not_historical_backfill():
    with mapper() as terrain:
        terrain.integrate(cloud(), 0.0, [0, 0, 0.5])
        assert (terrain.project(0.0)[:, :, 0] >= 0).any()
        terrain.shift_origin((10, 10))
        assert np.all(terrain.project(0.1)[:, :, 0] == -1)
        terrain.shift_origin((-3, -3))
        assert np.all(terrain.project(0.2)[:, :, 0] == -1)
        terrain.integrate(cloud(), 0.3, [0, 0, 0.5])
        assert (terrain.project(0.3)[:, :, 0] == 100).any()


@pytest.mark.parametrize("origin", [(-2.95, -3), (float("nan"), -3), (1e100, -3)])
def test_invalid_shift_does_not_modify_existing_projection(origin):
    with mapper() as terrain:
        terrain.integrate(cloud(), 0.0, [0, 0, 0.5])
        before = terrain.project(0.0).tobytes()
        with pytest.raises(ValueError, match="integer cells"):
            terrain.shift_origin(origin)
        assert terrain.project(0.0).tobytes() == before


def test_rolling_core_follows_rover_without_reintegrating_old_inputs():
    factory = lambda origin, lifetime: RollingTerrainMapper(
        LIBRARY, width=60, height=60, origin=origin, lifetime=lifetime
    )
    config = LocalConfig(
        window_s=30,
        cells=60,
        observation_policy="frozen_observation",
        map_policy="rolling_incremental",
        backend="prepared",
    )
    core = LocalReconstructor(factory, np.eye(4), config)
    try:
        for i in range(50):
            feed(core, i)
        assert core.last_record["origin"][0] > -3
        assert core.integrated_sequence == 47 and core.last_record["integrated_inputs"] == 1
        assert core.persistent_mapper.handle is not None
    finally:
        core.close()
    assert core.last_result is None


@pytest.mark.parametrize("lifetime", [30, 60])
def test_short_raw_retention_does_not_expire_persistent_terrain_early(lifetime):
    factory = lambda origin, duration: RollingTerrainMapper(
        LIBRARY, width=60, height=60, origin=origin, lifetime=duration
    )
    cores = [
        LocalReconstructor(
            factory,
            np.eye(4),
            LocalConfig(
                window_s=window,
                cells=60,
                observation_policy="frozen_observation",
                map_policy="rolling_incremental",
                backend="prepared",
                evidence_lifetime_s=lifetime,
            ),
        )
        for window in (5, 30)
    ]
    raw = cloud()[::4].astype(np.float32)
    try:
        for i in range(335):
            stamp = i * 100_000_000
            for core in cores:
                core.revision(revision(i + 1, [(i, stamp, transform(0.001 * i))]), stamp)
                core.chunk(dict(capture="raw-retention", sequence=i, stamp_ns=stamp, points=raw), stamp)
                core.advance(stamp)
            if i >= 2:
                assert cores[0].last_result.tobytes() == cores[1].last_result.tobytes()
                assert cores[0].last_record["local_from_base"] == cores[1].last_record["local_from_base"]
        assert len(cores[0].observations) <= 53 and len(cores[1].observations) >= 300
        assert cores[0].bytes < cores[1].bytes / 5
        assert cores[0].last_record["evidence_lifetime_s"] == lifetime
    finally:
        for core in cores:
            core.close()
