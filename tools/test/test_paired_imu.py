"""Host NumPy/SciPy/pytest: equal-window IMU comparisons, no fitted timing."""
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu_consistency import paired_consistency, coincident_samples
from audit_shadow_imu import validate_physics_clock
from shadow_capture_gate import PhysicsIMUReadiness


def fixture():
    times = np.arange(121)/60
    poses = np.tile(np.eye(4), (len(times), 1, 1))
    acceleration = np.tile([0., 0., 1.62], (len(times), 1))
    gyro = np.zeros_like(acceleration)
    streams = {"legacy": (times[::2], acceleration[::2].copy(), gyro[::2].copy()),
               "physics": (times, acceleration.copy(), gyro.copy())}
    return times, poses, streams


def test_same_windows_not_two_unmatched_rate_dependent_percentiles():
    times, poses, streams = fixture()
    result = paired_consistency(times, poses, np.eye(4), streams)
    assert result["legacy_acceleration_error_m_s2"].shape == result["physics_acceleration_error_m_s2"].shape
    for key, value in result.items():
        if "_error_" in key:
            np.testing.assert_allclose(value, 0., atol=1e-12)
    np.testing.assert_array_equal(result["gyro_endpoints"], times[::2])


def test_coincident_diagnostic_allows_only_nanosecond_serialization_not_time_fitting():
    physics = np.arange(7, dtype=np.int64)*16_666_667
    legacy = physics[::2]+[0, 1, -1, 0]
    ids, differences = coincident_samples(legacy, physics)
    np.testing.assert_array_equal(ids, [0, 2, 4, 6])
    np.testing.assert_array_equal(differences, [0, -1, 1, 0])
    for bad in (legacy+2, legacy+1_000_000, legacy[::-1], legacy.astype(float)):
        with pytest.raises(ValueError):
            coincident_samples(bad, physics)


def test_full_rate_preserves_alternating_signal_lost_by_decimation():
    times, poses, streams = fixture()
    for name, (_, a, _) in streams.items():
        a[:, 0] = -1 if name == "legacy" else np.where(np.arange(len(a)) % 2, 1., -1.)
    result = paired_consistency(times, poses, np.eye(4), streams)
    np.testing.assert_allclose(result["legacy_acceleration_error_m_s2"][:, 0], -1., atol=1e-12)
    np.testing.assert_allclose(result["physics_acceleration_error_m_s2"][:, 0], 0., atol=1e-12)


def test_paired_rotating_lever_and_sensor_axes_use_base_then_mount():
    times, poses, streams = fixture()
    rate = .7
    base_rotation = Rotation.from_euler("z", rate*times)
    poses[:, :3, :3] = base_rotation.as_matrix()
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler("xyz", [.1, -.2, np.pi]).as_matrix()
    mount[:3, 3] = [.4, .05, .3]
    world_acc = base_rotation.apply(np.tile(-rate**2*mount[:3, 3], (len(times), 1)))
    world_acc[:, 2] = 0.
    sensor = base_rotation*Rotation.from_matrix(mount[:3, :3])
    acceleration = sensor.inv().apply(world_acc + [0., 0., 1.62])
    gyro = sensor.inv().apply(np.tile([0., 0., rate], (len(times), 1)))
    streams = {"legacy": (times[::2], acceleration[::2], gyro[::2]),
               "physics": (times, acceleration, gyro)}
    # Deliberately only 30 Hz base GT. The lever must rotate after interpolation.
    result = paired_consistency(times[::2], poses[::2], mount, streams)
    for name in streams:
        np.testing.assert_allclose(result[name+"_gyro_error_rad_s"], 0., atol=1e-12)
        np.testing.assert_allclose(result[name+"_acceleration_error_m_s2"], 0., atol=1e-5)


@pytest.mark.parametrize("stream", ["legacy", "physics"])
def test_a_missing_tick_is_a_failure_not_interpolated_away(stream):
    times, poses, streams = fixture()
    streams[stream] = tuple(np.delete(v, 10, axis=0) for v in streams[stream])
    with pytest.raises(ValueError, match="cadence"):
        paired_consistency(times, poses, np.eye(4), streams)


def test_clock_rows_must_pair_exactly_without_resets_or_faults():
    clocks = [dict(monotonic_time=1+i/60, world_time=1+i/60, sensor_time=1+(i-1)/60,
                   physics_dt=1/60, epoch=0, valid=True) for i in range(10)]
    stamps = [round(c["monotonic_time"]*1e9) for c in clocks]
    assert validate_physics_clock(stamps, clocks, stamps[0], stamps[-1])["samples"] == 10
    for bad in (clocks[:-1], clocks+[dict(terminal_fault="stopped")],
                [dict(c, epoch=i % 2) for i, c in enumerate(clocks)], [dict(c, sensor_time=0.) for c in clocks]):
        with pytest.raises(ValueError):
            validate_physics_clock(stamps, bad, stamps[0], stamps[-1])


def ready_gate():
    gate = PhysicsIMUReadiness()
    for i in range(60):
        t = 1+i/60
        gate.sample(round(t*1e9), t, "Imu_Sensor", -1, (0., 0., 1.62, 0., 0., 0.))
        gate.clock(dict(monotonic_time=t, world_time=t, sensor_time=t-1/60,
                        physics_dt=1/60, epoch=1, valid=True), t)
    return gate


def test_motion_gate_requires_matching_fresh_second_and_fails_closed():
    gate = ready_gate()
    assert gate.ready(2., 2.)
    assert not gate.ready(3., 3.)
    gate.clocks.pop()
    assert not gate.ready(2., 2.)
    gate.armed = True
    with pytest.raises(ValueError, match="cadence"):
        gate.sample(round(2.1e9), 2.1, "Imu_Sensor", -1, (0.,)*6)


@pytest.mark.parametrize("change", [dict(valid=False), dict(epoch=2), dict(terminal_fault="stopped"),
                                   dict(sensor_time=0.)])
def test_armed_gate_rejects_bad_clock(change):
    gate = ready_gate()
    gate.armed = True
    status = dict(monotonic_time=2., world_time=2., sensor_time=2.-1/60,
                  physics_dt=1/60, epoch=1, valid=True)
    with pytest.raises(ValueError):
        gate.clock(dict(status, **change), 2.)
