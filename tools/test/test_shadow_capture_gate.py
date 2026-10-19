"""Host-only lease contracts. No ROS/Isaac; runtime coupling needs integration."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from shadow_capture_gate import ShadowLease, validate_shadow_geometry

CAPTURE = "revision-" + "a" * 32


def grant(**kwargs):
    return dict(dict(schema=1, scope="guarded_terrain_capture", capture=CAPTURE,
                     recorder="revision-" + "b" * 32, session="session", epoch=1,
                     config_sha256="c" * 64, ready=True, sequence=1, stamp=10.), **kwargs)


def test_unarmed_stale_latched_message_and_fresh_grant():
    lease = ShadowLease(CAPTURE)
    assert not lease.ready(10., 100.)
    assert not lease.receive(grant(capture="another"), 10., 100.)
    assert not lease.ready(10., 100.)
    assert lease.receive(grant(), 10., 100.)
    assert lease.ready(10.4, 100.4)
    assert lease.first_grant == grant()


def test_duplicate_does_not_refresh_wall_lease():
    lease = ShadowLease(CAPTURE)
    lease.receive(grant(), 10., 100.)
    assert not lease.receive(grant(), 10.4, 100.4)
    with pytest.raises(ValueError, match="expired"):
        lease.ready(10.4, 100.6)
    with pytest.raises(ValueError):
        lease.receive(grant(sequence=2, stamp=10.4), 10.4, 100.6)


@pytest.mark.parametrize("changes", [dict(ready=False), dict(epoch=2), dict(session="new"),
                                     dict(config_sha256="d" * 64), dict(recorder="revision-" + "d" * 32),
                                     dict(stamp=9.), dict(stamp=11.), dict(stamp=float("nan")),
                                     dict(sequence=True), dict(scope="navigation")])
def test_faults_cannot_rearm(changes):
    lease = ShadowLease(CAPTURE)
    lease.receive(grant(), 10., 100.)
    with pytest.raises(ValueError):
        lease.receive(grant(**changes), 10.1, 100.1)
    with pytest.raises(ValueError):
        lease.ready(10.1, 100.1)


@pytest.mark.parametrize("sim,wall", [(10.6, 100.1), (10.1, 100.6), (9., 100.1), (10.1, 99.)])
def test_independent_clock_expiry_and_reversal(sim, wall):
    lease = ShadowLease(CAPTURE)
    lease.receive(grant(), 10., 100.)
    with pytest.raises(ValueError):
        lease.ready(sim, wall)


def test_advancing_heartbeat_keeps_fixed_identity():
    lease = ShadowLease(CAPTURE)
    for i in range(20):
        lease.receive(grant(sequence=i, stamp=10 + .1*i), 10 + .1*i, 100 + .1*i)
        assert lease.ready(10 + .1*i, 100 + .1*i)
    assert lease.first_grant["sequence"] == 0


def test_late_fresh_grant_cannot_revive_lease_without_prior_poll():
    lease = ShadowLease(CAPTURE)
    lease.receive(grant(), 10., 100.)
    with pytest.raises(ValueError, match="expired"):
        lease.receive(grant(sequence=2, stamp=10.6), 10.6, 100.6)


def geometry():
    import numpy as np
    from scipy.spatial.transform import Rotation
    imu, lidar = np.eye(4), np.eye(4)
    imu[:3, :3] = Rotation.from_euler("z", 180, degrees=True).as_matrix()
    imu[:3, 3] = [.264, .017, .262]
    lidar[:3, :3] = Rotation.from_euler("y", 5, degrees=True).as_matrix()
    lidar[:3, 3] = [.12747165, 0., .26189884]
    extrinsic = np.linalg.inv(imu) @ lidar
    request = dict(schema=1, capture=CAPTURE, pattern="wide_sweep15_return", pitch_deg=5.,
                   home_xyz=[-20., -20., 0.], base_from_imu=imu.tolist(), base_from_lidar=lidar.tolist())
    return request, np.linalg.inv(extrinsic), imu, lidar, extrinsic


def test_geometry_checks_both_lever_arm_and_mount_direction():
    assert max(validate_shadow_geometry(*geometry()).values()) < 1e-12


@pytest.mark.parametrize("index", [1, 2, 3, 4])
def test_every_live_geometry_link_must_match(index):
    args = list(geometry())
    args[index][0, 3] += .002
    with pytest.raises(ValueError, match="extrinsic mismatch"):
        validate_shadow_geometry(*args)


@pytest.mark.parametrize("changes", [dict(home_xyz=[20., 18., 0.]), dict(pitch_deg=0.),
                                    dict(pattern="wide_sweep15"), dict(capture="../elsewhere")])
def test_geometry_cannot_silently_change_experiment(changes):
    args = list(geometry())
    args[0].update(changes)
    with pytest.raises(ValueError):
        validate_shadow_geometry(*args)
def test_validation_pause_keeps_actual_receipt_time_without_late_revival():
    lease = ShadowLease(CAPTURE)
    lease.receive(grant(sequence=0), 10., .8)
    times = iter([1., 1.354])
    assert lease.receive_checked(lambda: grant(sequence=1), 10., clock=lambda: next(times))
    assert lease.wall_stamp == 1.  # Not artificially freshened to 1.354.
    assert lease.ready(10., 1.49)
    with pytest.raises(ValueError, match="expired"):
        lease.ready(10., 1.501)


@pytest.mark.parametrize("arrival,completion", [(1.301, 1.4), (1., 1.501)])
def test_checked_receipt_rejects_expired_entry_or_slow_validation(arrival, completion):
    lease = ShadowLease(CAPTURE)
    lease.receive(grant(sequence=0), 10., .8)
    times = iter([arrival, completion])
    with pytest.raises(ValueError, match="expired"):
        lease.receive_checked(lambda: grant(sequence=1), 10., clock=lambda: next(times))
    assert lease.failure


def test_bad_publisher_validation_cannot_refresh_lease():
    lease = ShadowLease(CAPTURE)
    lease.receive(grant(sequence=0), 10., .8)
    def reject():
        raise ValueError("wrong publisher")
    with pytest.raises(ValueError, match="wrong publisher"):
        lease.receive_checked(reject, 10., clock=lambda: 1.)
    assert lease.wall_stamp == .8 and lease.sequence == 0 and lease.failure
