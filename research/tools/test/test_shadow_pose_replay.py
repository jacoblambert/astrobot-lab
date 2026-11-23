"""Host NumPy/pytest: frozen pose-only replay cannot substitute/mis-time chunks."""
from copy import deepcopy
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from shadow_pose_replay import validate_pose_rows


def inputs():
    association = dict(session="s", epoch=1, acquisition_ns=50_000_000, available_ns=120_000_000,
                       output_ns=300_000_000, bracket_ns=[0, 100_000_000], mode="causal",
                       map_from_lidar=np.eye(4).tolist(), valid=True)
    row = dict(chunk=0, acquisition_ns=50_000_000, causal=dict(association=association),
               revised=dict(association=dict(association, mode="revised")))
    return [row], [dict(observed_stamp=.05)]


def test_unchanged_native_time_and_distinct_retrospective_poses():
    rows, frames = inputs()
    rows[0]["revised"]["association"]["map_from_lidar"] = np.eye(4).tolist()
    rows[0]["revised"]["association"]["map_from_lidar"][0][3] = .2
    assert validate_pose_rows(rows, frames, "first_published", "s", 1)[0, 0, 3] == 0.
    assert validate_pose_rows(rows, frames, "final_revised", "s", 1)[0, 0, 3] == .2
    assert frames == [dict(observed_stamp=.05)]


@pytest.mark.parametrize("change", ["chunk", "stamp", "epoch", "mode", "count", "future", "extrapolate", "gap", "matrix"])
def test_substitution_gaps_and_bad_transforms_rejected(change):
    rows, frames = inputs()
    a = rows[0]["causal"]["association"]
    if change == "chunk": rows[0]["chunk"] = 1
    elif change == "stamp": frames[0]["observed_stamp"] = .051
    elif change == "epoch": a["epoch"] = 2
    elif change == "mode": a["mode"] = "revised"
    elif change == "count": rows = []
    elif change == "future": a["available_ns"] = 400_000_000
    elif change == "extrapolate": a["bracket_ns"] = [60_000_000, 100_000_000]
    elif change == "gap": a["bracket_ns"] = [0, 300_000_000]
    else: a["map_from_lidar"][0][0] = 2.
    with pytest.raises(ValueError):
        validate_pose_rows(rows, frames, "first_published", "s", 1)
