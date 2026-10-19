from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_tf_fixture import correction_series, held_correction_errors
from fixture_alignment import pose_matrix


def test_same_time_full_se3_and_held_correction():
    odom = [(1., [0, 0, 0], [0, 0, 0, 1]), (1.1, [.01, .02, .03], [0, 0, 0, 1])]
    slam = [(1.05, [1, 2, 3], [.2, 0, 0, np.sqrt(.96)])]
    extrinsic = pose_matrix([.264, .017, -.262], [0, 0, 1, 0])
    correction = correction_series(slam, odom, extrinsic)
    expected = pose_matrix(*slam[0][1:]) @ extrinsic @ np.linalg.inv(pose_matrix([.005, .01, .015], [0, 0, 0, 1]))
    np.testing.assert_allclose(correction[0][1], expected, atol=1e-12)
    assert held_correction_errors(correction, [(1.2, expected)])[0] < 1e-12
    planar = expected.copy()
    planar[2, 3] = 0
    assert held_correction_errors(correction, [(1.2, planar)])[0] > 1


@pytest.mark.parametrize('timestamp', [1.1, 2.])
def test_tf_cannot_match_future_or_expired_pose(timestamp):
    with pytest.raises(ValueError, match='causal fresh'):
        held_correction_errors([(1.05, np.eye(4))], [(timestamp, np.eye(4))])
