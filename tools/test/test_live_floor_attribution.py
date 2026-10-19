import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from attribute_live_floor_support import isolated_pose, floor_reasons


def test_pose_substitution_uses_exact_tick_and_full_home_transform():
    pose = np.eye(4)
    pose[:3, 3] = [20, 18, 0.4]
    gauge = np.eye(4)
    gauge[:3, 3] = [-40, -38, -0.1]
    frame = dict(observed_stamp=1.5, world_from_lidar=pose.tolist())
    result = isolated_pose(frame, 1_500_000_000, gauge, [-20, -20, 0])
    np.testing.assert_allclose(result[:3, 3], [0, 0, 0.3], atol=1e-15)
    np.testing.assert_array_equal(frame["world_from_lidar"], pose)
    with pytest.raises(ValueError, match="stamp mismatch"):
        isolated_pose(frame, 1_500_000_001, gauge, [-20, -20, 0])


def test_reasons_count_unknown_only_and_retain_outside():
    layers = np.zeros((1, 3, 6))
    layers[0, :, 0] = [-1, -1, 0]
    layers[0, :, 1] = [8, 12, 1]
    record = dict(local_from_map=np.eye(4), origin=[0, 0], resolution=0.1)
    points = np.array([[0.05, 0.05, 0], [0.15, 0.05, 0], [0.25, 0.05, 0], [1, 1, 0]])
    counts = floor_reasons(layers, record, points, np.zeros(3))
    assert counts["outside"] == counts["NEGATIVE_EDGE"] == counts["UNSUPPORTED_EXTENT"] == 1
    assert sum(counts.values()) == 3
