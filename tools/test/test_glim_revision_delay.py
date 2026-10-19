from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_glim_revision_delay import fixed_deadline
from shadow_terrain import FramePose, PoseTimeline


def fixture():
    timeline = PoseTimeline("test", 1)
    pose = np.eye(4)
    pose[0, 3] = 0.1
    timeline.apply(
        session="test", epoch=1, revision=1, parent=0, arrival_ns=100_000_000, frames=[FramePose(0, 0, pose, np.eye(4))]
    )
    timeline.apply(
        session="test",
        epoch=1,
        revision=2,
        parent=1,
        arrival_ns=400_000_000,
        frames=[FramePose(0, 0, np.eye(4), np.eye(4))],
    )
    return timeline


def test_fixed_deadline_cannot_leak_later_revision():
    for deadline, expected in [(300_000_000, 0.1), (500_000_000, 0.0)]:
        result, rows = fixed_deadline(
            fixture(), [dict(observed_stamp=0)], [np.eye(4)], np.eye(4), 1_000_000_000, deadline
        )
        assert result["full_interval_available"]
        assert result["statistics"]["horizontal_m"]["absolute_p95"] == expected
        assert rows[0]["available_ns"] <= rows[0]["output_ns"] == deadline


def test_missing_tail_forbids_partial_aggregate():
    result, _ = fixed_deadline(
        fixture(),
        [dict(observed_stamp=0), dict(observed_stamp=0.2)],
        [np.eye(4), np.eye(4)],
        np.eye(4),
        600_000_000,
        500_000_000,
    )
    assert result["expected_samples"] == 2 and result["associated_samples"] == 1
    assert result["statistics"] is None
    assert result["missing"][0]["chunk"] == 1


def test_too_early_deadline_does_not_use_first_future_pose():
    result, _ = fixed_deadline(fixture(), [dict(observed_stamp=0)], [np.eye(4)], np.eye(4), 1_000_000_000, 50_000_000)
    assert result["statistics"] is None and result["associated_samples"] == 0
