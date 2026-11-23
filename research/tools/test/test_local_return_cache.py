from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.return_cache import ReturnTransformCache
from raw_return_filter import valid_range_mask


def points():
    raw = np.random.default_rng(42).normal(size=(100, 3)).astype(np.float32)
    raw.flags.writeable = False
    return raw


def test_range_and_transform_cache_is_bit_exact_and_revision_sensitive():
    raw = points()
    cache = ReturnTransformCache(0.5, 100, retain_transforms=True)
    pose = np.eye(4)
    for step in range(10):
        pose[:3, :3] = Rotation.from_euler("z", step / 10).as_matrix()
        pose[:3, 3] = [step * 1e-9, 0.12345, -0.04]
        keep, _ = valid_range_mask(raw, 0.5, 100)
        expected = raw[keep].astype(np.float64) @ pose[:3, :3].T + pose[:3, 3]
        actual = cache.transform(0, 123, raw, pose)
        np.testing.assert_array_equal(actual, expected)
        assert not actual.flags.writeable
        assert cache.transform(0, 123, raw, pose) is actual
    assert cache.filters == 1 and cache.hits == cache.transforms == 10


def test_default_cache_keeps_only_filtered_returns_not_repeated_world_arrays():
    raw = points()
    cache = ReturnTransformCache(0.5, 100)
    world = cache.transform(0, 0, raw, np.eye(4))
    assert cache.bytes == world.nbytes
    assert cache.entries[0][-1] is None
    np.testing.assert_array_equal(cache.transform(0, 0, raw, np.eye(4)), world)
    assert cache.filters == 1 and cache.transforms == 2 and cache.hits == 0


@pytest.mark.parametrize("budget", [0, 1, 10_000])
def test_cache_limits_fallback_and_expiry(budget):
    cache = ReturnTransformCache(0.5, 100, max_bytes=budget, max_entries=2)
    for i in range(6):
        cache.transform(i, i, points(), np.eye(4))
        assert cache.bytes <= budget and len(cache.entries) <= 2
    cache.prune([5])
    assert set(cache.entries) <= {5}
    cache.clear()
    assert cache.bytes == 0 and not cache.entries


@pytest.mark.parametrize("change", ["stamp", "raw", "mutable"])
def test_cache_identity_or_mutable_evidence_rejected(change):
    raw = points()
    cache = ReturnTransformCache(0.5, 100)
    cache.transform(0, 1, raw, np.eye(4))
    if change == "mutable":
        raw.flags.writeable = True
    with pytest.raises(ValueError):
        cache.transform(0, 2 if change == "stamp" else 1, points() if change == "raw" else raw, np.eye(4))
