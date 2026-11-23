import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_revision_batches import summarize


def event(revision, kind=1, x=0, frame_id=0):
    pose = np.eye(4)
    pose[0, 3] = x
    return dict(
        kind="revision",
        payload=dict(
            session="test",
            epoch=1,
            revision=revision,
            parent=revision - 1,
            kind=kind,
            snapshot=False,
            frames=[
                dict(
                    frame_id=frame_id,
                    stamp_ns=10**9,
                    map_from_imu=pose.tolist(),
                    imu_from_lidar=np.eye(4).tolist(),
                    deskew_recomputed=False,
                )
            ],
        ),
    )


def test_tiny_pose_roundoff_is_measured_not_hidden_and_motion_is_distinct():
    result = summarize([event(1), event(2, kind=2, x=1e-14), event(3, kind=3, x=0.1)])
    assert result["non_odom_batches"][0]["max_xyz_delta_m"] == 1e-14
    assert result["non_odom_batches"][1]["max_xyz_delta_m"] == pytest.approx(0.1)
    assert not result["acceptance"]


def test_unknown_frames_and_gaps_not_silently_called_noop():
    result = summarize([event(1), event(2, kind=2, frame_id=1)])
    assert result["non_odom_batches"][0]["unknown_frames"] == 1
    assert result["non_odom_batches"][0]["max_xyz_delta_m"] is None
    with pytest.raises(ValueError, match="chain gap"):
        summarize([event(1), event(3)])
