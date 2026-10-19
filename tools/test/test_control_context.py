import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.control_context import current_context, displacement_bound
from shadow_terrain import FramePose, PoseTimeline


def setup():
    timeline = PoseTimeline("s", 1)
    first, last, extrinsic, base_from_imu, local_from_map = [np.eye(4) for _ in range(5)]
    first[:3, 3] = [40, 38, 0.5]
    last[:] = first
    last[0, 3] += 0.02
    last[:3, :3] = Rotation.from_euler("z", 0.02).as_matrix()
    extrinsic[:3, 3] = [0.4, 0, 0.45]
    base_from_imu[:3, 3] = [0.1, 0, 0.2]
    local_from_map[:3, 3] = [-40, -38, -0.5]
    # PoseTimeline internals are explicit fixture input; actual replay uses
    # validated revision callbacks and the normal bookkeeping ledger.
    timeline.revision = 2
    timeline.extrinsic = extrinsic
    timeline.frames = {
        0: [(0, 1, FramePose(0, 0, first, extrinsic))],
        1: [(180_000_000, 2, FramePose(1, 150_000_000, last, extrinsic))],
    }
    record = dict(
        session="s", epoch=1, revision=2, evidence_ns=0, publication_ns=200_000_000, local_from_map=local_from_map
    )
    return timeline, record, base_from_imu


def test_uses_latest_available_pose_exact_base_chain_not_map_evidence_pose():
    timeline, record, base_from_imu = setup()
    context = current_context(timeline, record, base_from_imu, 200_000_000)
    assert context.pose_ns == 150_000_000 and context.pose_age_s == 0.05
    expected = record["local_from_map"] @ timeline.frames[1][0][2].map_from_imu @ np.linalg.inv(base_from_imu)
    np.testing.assert_allclose(context.local_from_base, expected, atol=1e-14)
    step = np.eye(4)
    step[0, 3] = 0.1
    np.testing.assert_allclose(context.poses([np.eye(4), step])[-1], expected @ step, atol=1e-14)
    assert not context.local_from_base.flags.writeable
    with pytest.raises(ValueError, match="identity"):
        context.poses([step])


@pytest.mark.parametrize("problem", ["epoch", "revision", "future_map", "stale_map", "stale_pose"])
def test_stale_future_or_mixed_generation_cannot_authorize_query(problem):
    timeline, record, base_from_imu = setup()
    now = 200_000_000
    if problem in ("epoch", "revision"):
        record[problem] += 1
    elif problem == "future_map":
        record["publication_ns"] += 1
    elif problem == "stale_map":
        now = 600_000_000
    else:
        now = 400_000_000
    with pytest.raises(ValueError):
        current_context(timeline, record, base_from_imu, now)


def test_future_received_pose_never_leaks_or_gets_retimestamped():
    timeline, record, base_from_imu = setup()
    row = timeline.frames[1][0]
    timeline.frames[1] = [(201_000_000, row[1], row[2])]
    context = current_context(timeline, record, base_from_imu, 200_000_000)
    assert context.pose_ns == 0 and context.pose_age_s == 0.2


def test_rotation_and_stale_motion_contribute_to_displacement_bound():
    bound = displacement_bound(
        radius_m=1, translation_error_m=0.02, angle_error_rad=0.03, speed_m_s=0.1, yaw_rate_rad_s=0.1, age_s=0.2
    )
    assert bound == pytest.approx(0.04 + 2 * np.sin(0.025))
    with pytest.raises(ValueError):
        displacement_bound(
            radius_m=1, translation_error_m=-1, angle_error_rad=0, speed_m_s=0, yaw_rate_rad_s=0, age_s=0
        )
