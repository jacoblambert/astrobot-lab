from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fixture_alignment import ReferenceTrajectory, pose_matrix, reanchor_scan, validate_fixture_epochs


def test_reference_interpolates_translation_and_shortest_rotation():
    trajectory = ReferenceTrajectory([
        (1., [0,0,0], Rotation.from_euler("z", 3.13).as_quat()),
        (1.1, [.02,0,0], Rotation.from_euler("z", -3.13).as_quat())])
    middle = trajectory.at(1.05)
    assert middle[0,3] == pytest.approx(.01)
    assert middle[0,0] == pytest.approx(-1)
    with pytest.raises(ValueError, match="extrapolation"):
        trajectory.at(.9)


def test_invalid_reference_and_large_time_gaps_are_rejected():
    with pytest.raises(ValueError, match="kinematic"):
        ReferenceTrajectory([(1.,[0,0,0],[0,0,0,1]),(1.1,[5,0,0],[0,0,0,1])])
    trajectory = ReferenceTrajectory([(1.,[0,0,0],[0,0,0,1]),(2.,[0,0,0],[0,0,0,1])])
    with pytest.raises(ValueError, match="gap"):
        trajectory.at(1.5)
    with pytest.raises(ValueError, match="timestamp"):
        trajectory.at(float("nan"))
    with pytest.raises(ValueError, match="gap limit"):
        ReferenceTrajectory(trajectory.rows, max_gap=float("nan"))


def test_reset_epochs_cannot_be_combined_without_a_reset_barrier():
    assert validate_fixture_epochs([3, 3]) == [3]
    assert validate_fixture_epochs([]) == []  # Caller reports unavailable evidence.
    with pytest.raises(ValueError, match="reset barriers"):
        validate_fixture_epochs([3, 4])


def test_se3_reanchor_corrects_cloud_and_true_lidar_origin():
    gt = pose_matrix([20,18,.1], Rotation.from_euler("xyz", [.2,-.3,.4]).as_quat())
    anchored = gt.copy(); anchored[:3,3] -= [20,18,0]
    true_imu = anchored @ pose_matrix([.264,.017,.262],[0,0,1,0])
    imu_lidar = pose_matrix([-.136,.017,.012],[0,0,1,0])
    error = pose_matrix([2,-1,.5], Rotation.from_euler("xyz", [-.1,.2,.5]).as_quat())
    wrong_imu = error @ true_imu
    local_points = np.array([[1.,2.,.3],[-1.,2.,-.2]])
    wrong_points = local_points @ wrong_imu[:3,:3].T + wrong_imu[:3,3]
    corrected, origin = reanchor_scan(wrong_points, wrong_imu, wrong_imu@imu_lidar, gt)
    expected = local_points @ true_imu[:3,:3].T + true_imu[:3,3]
    np.testing.assert_allclose(corrected, expected, atol=1e-12)
    np.testing.assert_allclose(origin, true_imu@imu_lidar, atol=1e-12)
    assert np.linalg.norm(origin[:3,3]-anchored[:3,3]) > .1  # Not the base-link ray origin.
