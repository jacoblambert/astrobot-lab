"""Host NumPy/SciPy/pytest. Analytic evaluator contracts, no ROS/GPU."""
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_consistency import (consistency, interpolate_poses, residual_summary, series,
                             triangular_acceleration, orientation_clock_hypotheses)


def motion():
    t = np.arange(201) * .01
    yaw_rate = .3
    base = Rotation.from_euler("z", yaw_rate*t)
    sensor = base * Rotation.from_euler("z", np.pi)
    lever = base.apply(np.tile([.264, .017, .262], (len(t), 1)))
    poses = np.tile(np.eye(4), (len(t), 1, 1))
    poses[:, :3, :3] = sensor.as_matrix()
    poses[:, :3, 3] = lever + np.column_stack([.1*t*t, 0*t, 0*t])
    world_acc = -yaw_rate**2 * lever
    world_acc[:, 2] = 0.
    world_acc += [.2, 0., 0.]
    acceleration = sensor.inv().apply(world_acc + [0., 0., 1.62])
    gyro = np.tile([0., 0., yaw_rate], (len(t), 1))
    return t, poses, acceleration, gyro


def test_full_mount_rotation_and_rotating_lever_arm_kinematics():
    t, poses, acceleration, gyro = motion()
    result = consistency(t, poses, t, acceleration, gyro)
    np.testing.assert_allclose(result["gyro_error_rad_s"], 0., atol=1e-12)
    np.testing.assert_allclose(result["acceleration_error_m_s2"], 0., atol=1e-6)
    assert len(result["centers"]) > 100


def test_wrong_imu_axes_and_gravity_are_visible_not_fitted_away():
    t, poses, acceleration, gyro = motion()
    wrong = acceleration.copy()
    wrong[:, :2] *= -1
    bad_axes = consistency(t, poses, t, wrong, gyro)
    assert residual_summary(bad_axes["acceleration_error_m_s2"])["norm_p50_p95_max"][0] > .3
    wrong_g = consistency(t, poses, t, acceleration, gyro, gravity=9.81)
    np.testing.assert_allclose(wrong_g["acceleration_error_m_s2"][:, 2], -8.19, atol=1e-10)


def test_acceleration_triangle_integrates_linear_signal_exactly():
    t = np.array([0., .13, .37, .52, .89, 1.])
    a = np.column_stack([2*t+1, -t, 0*t])
    np.testing.assert_allclose(triangular_acceleration(t, a, .5, .4), [2., -.5, 0.], atol=1e-14)


@pytest.mark.parametrize("kind", ["duplicate", "regress", "gap", "nan", "short", "negative"])
def test_invalid_series_fail_instead_of_dropping_samples(kind):
    t = np.array([0., .02, .04])
    if kind == "duplicate": t[1] = 0.
    if kind == "regress": t[1] = .05
    if kind == "gap": t[2] = .5
    if kind == "nan": t[1] = np.nan
    if kind == "short": t = t[:1]
    if kind == "negative": t[0] = -.01
    with pytest.raises(ValueError):
        series(t, np.zeros((len(t), 3)))


def test_pose_extrapolation_invalid_rotation_and_short_windows_rejected():
    t, poses, a, w = motion()
    for query in [[-.001], [2.001], [np.nan]]:
        with pytest.raises(ValueError):
            interpolate_poses(t, poses, query)
    malformed = poses.copy()
    malformed[5, 0, 0] = 2.
    with pytest.raises(ValueError):
        interpolate_poses(t, malformed, [.1])
    with pytest.raises(ValueError, match="insufficient"):
        consistency(t[:10], poses[:10], t[:10], a[:10], w[:10])


def test_interpolation_owns_output_and_preserves_rotation_direction():
    t = [0., .05]
    poses = np.tile(np.eye(4), (2, 1, 1))
    poses[1, :3, :3] = Rotation.from_euler("z", .1).as_matrix()
    poses[1, 0, 3] = .2
    output = interpolate_poses(t, poses, [.025])
    np.testing.assert_allclose(output[0, :3, :3], Rotation.from_euler("z", .05).as_matrix(), atol=1e-12)
    assert output[0, 0, 3] == .1
    output[0, 0, 3] = 9.
    assert poses[1, 0, 3] == .2


def test_clock_hypotheses_report_every_declared_delay_without_retiming():
    t, poses, _, _ = motion()
    stamps = t[10:-10].copy()
    q = Rotation.from_euler("z", .3*(stamps-.02)+np.pi).as_quat()
    report = orientation_clock_hypotheses(t, poses, np.eye(4), stamps, q,
                                         physics_dt=.01, rendering_dt=.02)
    assert list(report) == ["header_time", "previous_physics_tick", "previous_render_tick"]
    assert report["header_time"]["error_rad"]["norm_p50_p95_max"][1] == pytest.approx(.006)
    assert report["previous_physics_tick"]["error_rad"]["norm_p50_p95_max"][1] == pytest.approx(.003)
    assert report["previous_render_tick"]["error_rad"]["norm_p50_p95_max"][2] < 1e-12
    np.testing.assert_array_equal(stamps, t[10:-10])
