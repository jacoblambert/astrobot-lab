"""Separate fixed-grid/frozen-placement algorithm; never naive equivalence claims."""

import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper
from local_terrain.core import LocalConfig, LocalReconstructor
from raw_return_filter import valid_range_mask
from test_local_terrain import FakeMapper, feed, revision, transform


def config(**kwargs):
    return LocalConfig(
        window_s=30, cells=200, observation_policy="frozen_observation", map_policy="fixed_incremental", **kwargs
    )


class PersistentFake(FakeMapper):
    def project(self, stamp):
        if self.interrupt:
            raise KeyboardInterrupt("persistent fixture interruption")
        return np.zeros((200, 200, 6), dtype=np.float32)


@pytest.mark.parametrize("bad", [dict(window_s=5), dict(cells=60), dict(observation_policy="coherent")])
def test_incremental_requires_explicit_experiment_contract(bad):
    values = dict(window_s=30, cells=200, observation_policy="frozen_observation", map_policy="fixed_incremental")
    values.update(bad)
    with pytest.raises(ValueError, match="requires"):
        LocalConfig(**values)


@pytest.mark.parametrize(
    "values",
    [
        dict(evidence_lifetime_s=60),
        dict(map_policy="fixed_incremental", evidence_lifetime_s=60),
        dict(map_policy="rolling_incremental", evidence_lifetime_s=120),
    ],
)
def test_lifetime_cannot_silently_extend_other_policies(values):
    with pytest.raises(ValueError):
        LocalConfig(**values)


def test_one_mapper_fixed_origin_each_observation_integrated_once_and_close_invalidates():
    core = LocalReconstructor(PersistentFake, np.eye(4), config())
    for i in range(20):
        feed(core, i)
    mapper = core.persistent_mapper
    assert not mapper.closed
    assert core.integrated_sequence == 17
    np.testing.assert_allclose(mapper.stamps, np.arange(18) / 10)
    np.testing.assert_allclose(core.last_record["origin"], [-10, -10])
    assert core.last_record["integrated_inputs"] == 1
    assert core.last_record["policy"] == "local_fixed_incremental_frozen_v1"
    assert core.last_record["support_freshness_certified"] is False
    core.close()
    assert mapper.closed and core.persistent_mapper is None and core.last_result is None


def test_old_pose_revision_does_not_reintegrate_frozen_returns():
    core = LocalReconstructor(PersistentFake, np.eye(4), config())
    for i in range(5):
        feed(core, i)
    points = [p.copy() for p in core.persistent_mapper.points]
    # An old non-reference pose can be revised without changing the map gauge.
    core.revision(revision(6, [(1, 100_000_000, transform(0.3))]), 400_000_000)
    feed(core, 5, number=7)
    assert len(core.persistent_mapper.points) == len(points) + 1
    for original, retained in zip(points, core.persistent_mapper.points):
        np.testing.assert_array_equal(original, retained)
    core.close()


def test_recent_pose_updates_cannot_walk_the_frame_away_from_frozen_evidence():
    core = LocalReconstructor(PersistentFake, np.eye(4), config())
    for i in range(5):
        feed(core, i)
    gauge = core.local_from_map.copy()
    points = core.persistent_mapper.points[0].copy()
    assert core.anchor == 0
    core.revision(revision(6, [(2, 200_000_000, transform(0.12))]), 400_000_000)
    feed(core, 5, number=7)
    np.testing.assert_array_equal(core.local_from_map, gauge)
    np.testing.assert_array_equal(core.persistent_mapper.points[0], points)
    assert core.last_record["reference_policy"] == "fixed_map_witness"
    core.close()


def test_fixed_reference_witness_jitter_cannot_ratchet_into_a_gauge_shift():
    core = LocalReconstructor(PersistentFake, np.eye(4), config())
    for i in range(5):
        feed(core, i)
    mapper = core.persistent_mapper
    gauge = core.local_from_map.copy()
    core.revision(revision(6, [(0, 0, transform(0.75e-6))]), 400_000_000)
    np.testing.assert_array_equal(core.local_from_map, gauge)
    with pytest.raises(ValueError, match="reference witness"):
        core.revision(revision(7, [(0, 0, transform(1.5e-6))]), 400_000_000)
    assert mapper.closed and core.last_result is None and core.failure


@pytest.mark.parametrize("fault", ["project", "epoch", "clock"])
def test_partial_failure_releases_persistent_state_and_all_published_output(fault):
    core = LocalReconstructor(PersistentFake, np.eye(4), config())
    for i in range(5):
        feed(core, i)
    mapper = core.persistent_mapper
    with pytest.raises((ValueError, KeyboardInterrupt)):
        if fault == "project":
            mapper.interrupt = True
            feed(core, 5)
        elif fault == "epoch":
            record = revision(6, [(5, 500_000_000, transform())])
            record["epoch"] = 1
            core.revision(record, 500_000_000)
        else:
            core.advance(900_000_000)
    assert core.failure and mapper.closed and core.persistent_mapper is None and core.last_result is None
    with pytest.raises(ValueError):
        core.advance(1_000_000_000)


@pytest.mark.skipif(not os.environ.get("TERRAIN_LIBRARY"), reason="real opt-in C++ library required")
def test_real_incremental_core_matches_direct_immutable_bucket_stream_through_expiry():
    library = os.environ["TERRAIN_LIBRARY"]
    factory = lambda origin, lifetime: TerrainMapper(library, width=200, height=200, origin=origin, lifetime=lifetime)
    core = LocalReconstructor(factory, np.eye(4), config(backend="prepared"))
    reference = factory((-10, -10), 30)
    x, y = np.meshgrid(np.linspace(0.5, 1.5, 21), np.linspace(-0.4, 0.4, 17))
    raw = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)].astype(np.float32)
    keep, _ = valid_range_mask(raw, 0.5, 100)
    selected = raw[keep].astype(float)
    last = -1
    try:
        for i in range(335):
            stamp = i * 100_000_000
            core.revision(revision(i + 1, [(i, stamp, transform(0.001 * i))]), stamp)
            core.chunk(dict(capture="fixture", sequence=i, stamp_ns=stamp, points=raw), stamp)
            result = core.advance(stamp)
            if result is None:
                continue
            for association in core.last_record["associations"]:
                if association["sequence"] <= last:
                    continue
                pose = np.asarray(association["local_from_lidar"])
                world = selected @ pose[:3, :3].T + pose[:3, 3]
                reference.integrate(
                    world, association["acquisition_ns"] / 1e9, pose[:3, 3], np.arctan2(pose[1, 0], pose[0, 0])
                )
                last = association["sequence"]
            assert result.tobytes() == reference.project(core.watermark_ns / 1e9).tobytes()
        assert core.last_record["observations"] <= 301
        assert core.integrated_sequence == last == 332
    finally:
        core.close()
        reference.close()
