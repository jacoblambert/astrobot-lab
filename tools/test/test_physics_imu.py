"""Host-only sample clock/payload contracts; no simulator or ROS required."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/physics_imu_ros2.py"
spec = importlib.util.spec_from_file_location("physics_imu_ros2", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def sample(clock, monotonic, world=None, sensor=None, valid=True):
    world = monotonic if world is None else world
    return clock.observe(monotonic=monotonic, world=world, sensor=world-1/60 if sensor is None else sensor,
                         dt=1/60, valid=valid)


def test_physics_ticks_and_world_reset_preserve_monotonic_header_clock():
    clock = module.PhysicsSampleClock()
    assert sample(clock, 1.)["epoch"] == 0
    assert sample(clock, 1.+1/60)["epoch"] == 0
    result = sample(clock, 1.1, world=.05)
    assert result["epoch"] == 1 and result["monotonic_time"] == 1.1
    assert result["sensor_time"] == pytest.approx(.05-1/60)
    assert not result["orientation_available"]
    sample(clock, 1.1+1/60, world=.05+1/60)


@pytest.mark.parametrize("kind", ["duplicate", "backwards", "skipped", "stale", "wrong_phase", "nan"])
def test_unqualified_sensor_clock_fails_closed(kind):
    clock = module.PhysicsSampleClock()
    sample(clock, 1.)
    kwargs = dict(monotonic=1.+1/60)
    if kind == "duplicate": kwargs["monotonic"] = 1.
    if kind == "backwards": kwargs["monotonic"] = .5
    if kind == "skipped": kwargs["monotonic"] = 1.+2/60
    if kind == "stale": kwargs["sensor"] = 1.-1/60
    if kind == "wrong_phase": kwargs["sensor"] = 1.+1/60
    if kind == "nan": kwargs["sensor"] = float("nan")
    with pytest.raises(ValueError):
        sample(clock, **kwargs)


def test_invalid_startup_sample_is_explicit_and_not_reused():
    clock = module.PhysicsSampleClock()
    assert not sample(clock, 1., sensor=0., valid=False)["valid"]
    assert sample(clock, 1.+1/60)["valid"]


def message():
    return NS(header=NS(stamp=NS(sec=0, nanosec=0), frame_id=""),
              linear_acceleration=NS(), angular_velocity=NS(), orientation=NS(w=0.),
              orientation_covariance=[0.]*9)


def reading():
    return NS(lin_acc_x=.1, lin_acc_y=.2, lin_acc_z=1.62, ang_vel_x=.01, ang_vel_y=.02, ang_vel_z=.03)


def test_payload_preserves_vectors_and_does_not_claim_stale_orientation():
    result = module.fill_imu(message(), reading(), 4.033333333, "Imu_Sensor")
    assert (result.header.stamp.sec, result.header.stamp.nanosec) == (4, 33333333)
    assert result.header.frame_id == "Imu_Sensor"
    assert result.linear_acceleration.z == 1.62 and result.angular_velocity.z == .03
    assert result.orientation_covariance[0] == -1.
    assert result.orientation.w == 1.


def test_nonfinite_payload_rejected_before_publication():
    value = reading()
    value.ang_vel_y = float("inf")
    with pytest.raises(ValueError):
        module.fill_imu(message(), value, 1., "Imu_Sensor")


def test_callback_fault_stops_shadow_without_touching_legacy():
    sent = []
    publisher = NS(publish=lambda m: sent.append(m))
    value = reading()
    value.is_valid, value.time = True, .5  # Deliberately inconsistent sensor clock.
    shadow = object.__new__(module.PhysicsIMUShadow)
    shadow.failed = False
    shadow.interface = NS(get_sensor_reading=lambda *a, **kw: value)
    shadow.core_clock = NS(get_sim_time_monotonic=lambda: 1.)
    shadow.world = NS(current_time=1.)
    shadow.entries = [("/robot/Imu_Sensor", module.PhysicsSampleClock(), publisher, publisher)]
    shadow.node = NS(get_logger=lambda: NS(error=lambda value: None))
    shadow.Imu, shadow.String = message, lambda **kw: NS(**kw)
    shadow.tick(1/60)
    assert shadow.failed and len(sent) == 1 and "terminal_fault" in sent[0].data
    shadow.tick(1/60)
    assert len(sent) == 1


def test_callback_keeps_both_physics_samples_in_each_render_interval():
    sent, statuses = [], []
    value = reading()
    value.is_valid = True
    shadow = object.__new__(module.PhysicsIMUShadow)
    shadow.failed = False
    shadow.interface = NS(get_sensor_reading=lambda *a, **kw: value)
    shadow.world = NS(current_time=0.)
    shadow.core_clock = NS(get_sim_time_monotonic=lambda: shadow.world.current_time)
    shadow.entries = [("/robot/Imu_Sensor", module.PhysicsSampleClock(), NS(publish=sent.append),
                       NS(publish=statuses.append))]
    shadow.node = NS(get_logger=lambda: NS(error=lambda value: None))
    shadow.Imu, shadow.String = message, lambda **kw: NS(**kw)
    for step in range(60):
        shadow.world.current_time = 1.+step/60
        value.time = shadow.world.current_time-1/60
        value.lin_acc_x = 1. if step % 2 else -1.
        shadow.tick(1/60)
    assert not shadow.failed and len(sent) == len(statuses) == 60
    assert sum(m.linear_acceleration.x/60 for m in sent) == pytest.approx(0.)
    # A render-rate decimator would alias this zero-net signal into a bias.
    assert abs(sum(m.linear_acceleration.x/30 for m in sent[::2])) == pytest.approx(1.)


def test_stationary_capture_gate_rejects_a_skipped_physics_sample():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from check_physics_imu_capture import summarize
    rows = []
    for topic, hz in (("/astrobot_0/imu", 30), ("/astrobot_0/imu_physics", 60)):
        rows += [dict(topic=topic, stamp=i/hz, acceleration=[0, 0, 1.62], gyro=[0, 0, 0],
                      orientation_covariance_0=-1) for i in range(hz+1)]
    rows += [dict(topic="/astrobot_0/imu_physics/clock", data=dict(valid=True)) for _ in range(61)]
    assert summarize(rows, 1.)["wiring_passed"]
    rows.pop(33)
    with pytest.raises(ValueError, match="cadence"):
        summarize(rows, 1.)


def test_stationary_comparison_retains_startup_but_uses_fixed_common_interval():
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from check_physics_imu_capture import summarize
    rows = []
    for topic, hz, stamps in (("/astrobot_0/imu", 30, [0., .2]+[1+i/30 for i in range(31)]),
                              ("/astrobot_0/imu_physics", 60, [1+i/60 for i in range(61)])):
        rows += [dict(topic=topic, stamp=t, acceleration=[0, 0, 1.62], gyro=[0, 0, 0],
                      orientation_covariance_0=-1) for t in stamps]
    rows += [dict(topic="/astrobot_0/imu_physics/clock", data=dict(valid=True)) for _ in range(61)]
    result = summarize(rows, 1.)
    assert result["common_interval_s"] == [1., 2.]
    assert result["streams"]["/astrobot_0/imu"]["outside_common_interval"] == 2
    assert result["streams"]["/astrobot_0/imu"]["recorded_maximum_gap_s"] == pytest.approx(.8)
    assert len(rows) == 155  # Inputs are not modified or discarded.
