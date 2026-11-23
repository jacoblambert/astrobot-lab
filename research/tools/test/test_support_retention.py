import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assess_support_retention import PastGroundUnion


def frame(stamp=1, origin=(0, 0)):
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = -1
    return layers, dict(
        session="s",
        epoch=1,
        local_from_map=np.eye(4).tolist(),
        origin=list(origin),
        resolution=0.1,
        publication_ns=stamp,
    )


def test_only_previously_free_cells_are_reused_without_mutating_raw():
    memory = PastGroundUnion()
    layers, record = frame()
    layers[4, 4, 0] = 0
    memory.update(layers, record)
    layers, record = frame(2)
    before = layers.copy()
    result, count = memory.update(layers, record)
    assert count == 1 and result[4, 4, 0] == 20 and result[4, 5, 0] == -1
    np.testing.assert_array_equal(before, layers)


@pytest.mark.parametrize("cost,reason", [(100, 3), (-1, 8), (-1, 11), (-1, 5)])
def test_current_hazards_veto_and_erase_old_support(cost, reason):
    memory = PastGroundUnion()
    layers, record = frame()
    layers[4, 4, 0] = 0
    memory.update(layers, record)
    layers, record = frame(2)
    layers[4, 4, :2] = [cost, reason]
    result, count = memory.update(layers, record)
    assert count == 0 and result[4, 4, 0] == cost
    result, count = memory.update(*frame(3))
    assert count == 0


def test_rolling_eviction_does_not_resurrect_support():
    memory = PastGroundUnion()
    layers, record = frame()
    layers[4, 4, 0] = 0
    memory.update(layers, record)
    result, count = memory.update(*frame(2, (0.1, 0)))
    assert count == 1 and result[4, 3, 0] == 20
    memory.update(*frame(3, (10, 0)))
    assert memory.update(*frame(4))[1] == 0


@pytest.mark.parametrize("change", [dict(epoch=2), dict(publication_ns=1), dict(origin=[0.01, 0])])
def test_changed_identity_phase_or_clock_is_rejected(change):
    memory = PastGroundUnion()
    memory.update(*frame())
    layers, record = frame(2)
    with pytest.raises(ValueError):
        memory.update(layers, dict(record, **change))


def test_pose_frame_change_is_not_silently_transported():
    memory = PastGroundUnion()
    memory.update(*frame())
    layers, record = frame(2)
    record["local_from_map"][0][3] = 0.01
    with pytest.raises(ValueError, match="reference"):
        memory.update(layers, record)
