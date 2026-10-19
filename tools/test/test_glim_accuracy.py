"""Host NumPy/SciPy/pytest: no alignment, stage identity, temporal accuracy."""
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_glim_accuracy import summary, relative_summary, first_sustained, trace_stages
from compare_glim_diagnostics import motion_bound, compare_poses


def test_fixed_offset_remains_in_absolute_score_not_relative_motion():
    times = np.arange(61)/30.
    gt = np.tile(np.eye(4), (len(times), 1, 1))
    gt[:, 0, 3] = times*.15
    est = gt.copy(); est[:, 0, 3] += .4
    assert summary(est, gt)["horizontal_m"]["absolute_p95"] == pytest.approx(.4)
    assert relative_summary(times, est, gt)["horizontal_m"]["absolute_max"] < 1e-12


def test_angle_wrap_does_not_create_360_degree_errors():
    gt = np.eye(4)[None, ...]; est = gt.copy()
    gt[0, :3, :3] = Rotation.from_euler("z", 179.9, degrees=True).as_matrix()
    est[0, :3, :3] = Rotation.from_euler("z", -179.9, degrees=True).as_matrix()
    assert summary(est, gt)["rpy_deg"]["absolute_p95"][2] == pytest.approx(.2)


def test_sustained_failure_is_not_one_spike_or_disjoint_samples():
    assert first_sustained([0,.5,1,1.5,2], [.2,0,.2,.2,.2]) == 1
    assert first_sustained([0,.5,1,1.5,2], [.2,0,.2,0,.2]) is None


def test_trace_distinguishes_scan_prior_from_yaw_prior_and_gps():
    pose = dict(frame_id=3, stamp_ns=100, map_from_imu=np.eye(4).tolist())
    key = (ord("x") << 56)+3
    rows = [dict(stage="pre_smoother", poses=[pose], factors=[
        dict(keys=[key], sigmas=[.1]*6, prior_pose=pose["map_from_imu"]),
        dict(keys=[key], sigmas=[1000,1000,.01,1000,1000,1000], prior_pose=pose["map_from_imu"]),
        dict(keys=[key], gps_position=[1,2,3])])]
    stages, factors = trace_stages(rows)
    assert stages["scan_match_target"][3]["stamp_ns"] == 100
    assert factors == dict(gps_factors=1, scan_prior_sigmas=[.1])
    with pytest.raises(ValueError):
        trace_stages([dict(pose, stage="optimized")]*2)


def test_motion_bound_has_no_fitted_delay_and_replay_comparison_retains_offset():
    times=np.arange(61)/30.
    poses=np.tile(np.eye(4),(len(times),1,1));poses[:,0,3]=times*.15
    result=motion_bound(times,poses,duration=.1,radius=5.)
    np.testing.assert_allclose(result["endpoint_bound_p95_max_m"],[.015,.015],atol=1e-12)
    assert compare_poses(poses,poses)["equivalent_1um_1urad"]
    shifted=poses.copy();shifted[:,0,3]+=.01
    assert not compare_poses(poses,shifted)["equivalent_1um_1urad"]


def test_anisotropic_scan_prior_is_not_dropped_as_heading():
    pose = dict(frame_id=3, stamp_ns=100, map_from_imu=np.eye(4).tolist())
    key = (ord("x") << 56)+3
    stages, factors = trace_stages([dict(stage="pre_smoother", poses=[pose], factors=[
        dict(keys=[key], sigmas=[.01,.02,.2,.3,.4,.05], prior_pose=pose["map_from_imu"]),
        dict(keys=[key], sigmas=[1000,1000,.01,1000,1000,1000], prior_pose=pose["map_from_imu"])])])
    assert stages["scan_match_target"][3]["stamp_ns"] == 100
    assert factors["anisotropic_scan_priors"] == 1
