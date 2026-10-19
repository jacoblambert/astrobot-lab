from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_failed_glim import pair_failed_prefix
from shadow_terrain import FramePose, PoseTimeline


def test_failed_tail_and_missing_gt_remain_in_denominator():
    timeline = PoseTimeline("test", 1)
    pose = np.eye(4)
    pose[0, 3] = 0.03
    timeline.apply(
        session="test",
        epoch=1,
        revision=1,
        parent=0,
        arrival_ns=100_000_000,
        frames=[FramePose(0, 0, pose, np.eye(4)), FramePose(1, 100_000_000, pose, np.eye(4))],
    )
    frames = [dict(observed_stamp=t, world_from_lidar=np.eye(4).tolist()) for t in (0, 0.05, 0.1, 0.2)]
    request = dict(base_from_lidar=np.eye(4), base_from_imu=np.eye(4), home_xyz=[0, 0, 0])
    rows, paired, truth, estimates = pair_failed_prefix(
        timeline, frames, request, np.array([0, 100_000_000, 200_000_000]), np.tile(np.eye(4), (3, 1, 1))
    )
    assert len(rows) == 4 and paired == [0, 2]
    assert "exact_gt_snapshot" in rows[1]["missing"]
    assert rows[-1]["missing"]
    assert np.allclose(np.asarray(estimates["causal"])[:, 0, 3], 0.03)
