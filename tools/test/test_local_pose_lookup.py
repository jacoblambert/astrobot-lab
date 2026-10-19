from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.pose_lookup import PublicationPoseLookup
from shadow_terrain import FramePose, PoseTimeline


def timeline():
    rng = np.random.default_rng(42)
    timeline = PoseTimeline("test", 0)
    mount = np.eye(4)
    mount[:3, 3] = [0.4, 0.01, 0.25]
    for rev in range(1, 5):
        frames = []
        for i in range(20):
            pose = np.eye(4)
            pose[:3, 3] = rng.normal(size=3)
            pose[:3, :3] = Rotation.from_rotvec(rng.normal(size=3)).as_matrix()
            frames.append(FramePose(i, i * 100_000_000, pose, mount))
        timeline.apply(
            session="test",
            epoch=0,
            revision=rev,
            parent=rev - 1,
            arrival_ns=2_000_000_000 + rev * 100_000_000,
            frames=frames,
        )
    return timeline


@pytest.mark.parametrize("publication", [2_100_000_000, 2_200_000_000, 2_300_000_000, 2_400_000_000])
def test_prepared_lookup_exactly_matches_causal_revision_history(publication):
    source = timeline()
    lookup = PublicationPoseLookup(source, publication)
    for stamp in range(0, 1_900_000_001, 25_000_000):
        expected = source.associate(stamp, publication, mode="revised")
        actual = lookup.associate(stamp)
        assert actual.record() == expected.record()
        np.testing.assert_array_equal(actual.map_from_lidar, expected.map_from_lidar)


@pytest.mark.parametrize("kind", ["future", "before_known", "extrapolate", "gap"])
def test_prepared_lookup_preserves_rejection_boundaries(kind):
    source = timeline()
    output = 2_000_000_000 if kind == "before_known" else 2_400_000_000
    stamp = 100_000_000
    if kind == "future":
        stamp = output + 1
    elif kind == "extrapolate":
        stamp = 1_900_000_001
    elif kind == "gap":
        source.frames.pop(1)
        source.frames.pop(2)
    lookup = PublicationPoseLookup(source, output)
    with pytest.raises(ValueError):
        lookup.associate(stamp)
    with pytest.raises(ValueError):
        source.associate(stamp, output, mode="revised")


def test_prepared_lookup_does_not_observe_future_mutation():
    source = timeline()
    lookup = PublicationPoseLookup(source, 2_400_000_000)
    expected = lookup.associate(50_000_000).record()
    source.frames.clear()
    assert lookup.associate(50_000_000).record() == expected


def test_interpolator_reuse_is_local_to_one_publication_pair(monkeypatch):
    import local_terrain.pose_lookup as module

    source = timeline()
    original = module.Slerp
    constructed = []

    def counted(*args):
        constructed.append(1)
        return original(*args)

    monkeypatch.setattr(module, "Slerp", counted)
    lookup = PublicationPoseLookup(source, 2_300_000_000)
    for stamp in (25_000_000, 50_000_000, 75_000_000):
        result = lookup.associate(stamp)
        np.testing.assert_array_equal(
            result.map_from_lidar, source.associate(stamp, 2_300_000_000, mode="revised").map_from_lidar
        )
    assert len(constructed) == 1
    lookup.associate(125_000_000)
    assert len(constructed) == 2
    newer = PublicationPoseLookup(source, 2_400_000_000)
    result = newer.associate(50_000_000)
    assert len(constructed) == 3
    np.testing.assert_array_equal(
        result.map_from_lidar, source.associate(50_000_000, 2_400_000_000, mode="revised").map_from_lidar
    )
    assert not np.array_equal(result.map_from_lidar, lookup.associate(50_000_000).map_from_lidar)
