"""Small actual ROS/MCAP recorder contracts; no simulator or motion commands.

Source the built interfaces/launch overlay and use isolated DDS domain 179.
Temporary files are tiny unit-test fixtures, not acquisition/archive substitutes.
"""
from pathlib import Path
import json
import sys
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("astrobot_interfaces.msg")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from archive_guard import allocate_run
from run_shadow_capture import Capture, TOPICS, REQUIRED_TOPICS
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from astrobot_interfaces.msg import LocalizationStatus


@pytest.fixture
def capture(tmp_path, request):
    config = tmp_path / "config"
    config.mkdir()
    (config / "config.json").write_text(json.dumps({"global": {"config_sensors": "config_sensors.json"}}))
    (config / "config_sensors.json").write_text(json.dumps(dict(sensors=dict(T_lidar_imu=[0, 0, 0, 0, 0, 0, 1]))))
    run = allocate_run(tmp_path, limit=1024**2, reserve=0, external=False)
    rclpy.init()
    node = Capture(run, config, require_physics_imu=getattr(request, "param", False))
    try:
        yield node
    finally:
        del node.writer
        node.destroy_node()
        rclpy.shutdown()


def test_recorder_registers_required_topics_and_finalizes_actual_mcap(capture):
    capture.record("/terrain_shadow/grant", String(data="test-only"))
    run = capture.run
    # Writer destruction is what the collector uses; reopen after finalization.
    writer = capture.writer
    del capture.writer
    del writer
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(run / "bag"), storage_id="mcap"), ConverterOptions("", ""))
    assert set(TOPICS) < {t.name for t in reader.get_all_topics_and_types()}
    assert reader.read_next()[0] == "/terrain_shadow/grant"
    assert (run / "bag/metadata.yaml").is_file()
    capture.writer = None


def test_dynamic_sensor_tf_does_not_require_nonexistent_static_publisher():
    assert "/tf" in REQUIRED_TOPICS and "/tf_static" not in REQUIRED_TOPICS
    assert "/tf_static" in TOPICS  # Still archived if supplied.


def test_async_storage_checks_actual_files_and_receipt_is_single_use(capture):
    from capture_polling import StorageCheck
    worker = StorageCheck()
    try:
        key = capture.storage_key()
        assert worker.poll(key, capture.check_storage) is None
        worker.future.result(timeout=1)
        receipt = worker.poll(key, capture.check_storage)
        assert capture.check_ready(storage_receipt=receipt) is None
        with pytest.raises(ValueError, match="reused"):
            capture.check_ready(storage_receipt=receipt)
        (capture.config / "config_sensors.json").write_text('{}')
        assert worker.poll(key, capture.check_storage) is None
        with pytest.raises(ValueError, match="configuration changed"):
            worker.future.result(timeout=1)
        with pytest.raises(ValueError, match="configuration changed"):
            worker.poll(key, capture.check_storage)
    finally:
        worker.close()


@pytest.mark.parametrize("recorder_profile", ["default", "loop16_slow_host_v1"])
@pytest.mark.parametrize("simulator_profile", ["default", "loop16_slow_host_v1"])
def test_execution_budget_must_match_before_recorder_accepts_request(capture, recorder_profile, simulator_profile):
    from test_natural_motion import request
    from natural_motion import execution_budget, route_hash
    capture.natural_motion, capture.execution_profile = "loop16", recorder_profile
    value = request()
    value.update(pattern="loop16", route_sha256=route_hash("loop16"))
    if simulator_profile != "default":
        value["execution_budget"] = execution_budget("loop16", simulator_profile)
    capture.receive("/terrain_shadow/request", String(data=json.dumps(value)))
    if recorder_profile == simulator_profile:
        assert capture.failure is None and capture.request == value
    else:
        assert "execution profiles differ" in capture.failure and capture.request is None


@pytest.mark.parametrize("profile", ["default", "turn15_gc_stress_v1"])
def test_gc_stress_request_requires_explicit_recorder_match(capture, profile):
    from test_natural_motion import request
    from natural_motion import execution_budget, route_hash
    capture.natural_motion, capture.execution_profile = "turn15", profile
    value = request()
    value.update(pattern="turn15", route_sha256=route_hash("turn15"),
                 execution_budget=execution_budget("turn15", "turn15_gc_stress_v1"))
    capture.receive("/terrain_shadow/request", String(data=json.dumps(value)))
    if profile == "default":
        assert "profiles differ" in capture.failure
    else:
        assert capture.failure is None and capture.request == value


@pytest.mark.parametrize("profile", ["default", "turn45_bounded_v1"])
def test_bounded_turn_request_requires_explicit_recorder_match(capture, profile):
    from test_natural_motion import request
    from natural_motion import execution_budget, route_hash
    capture.natural_motion, capture.execution_profile = "turn45", profile
    value = request()
    value.update(pattern="turn45", route_sha256=route_hash("turn45"),
                 execution_budget=execution_budget("turn45", "turn45_bounded_v1"))
    capture.receive("/terrain_shadow/request", String(data=json.dumps(value)))
    if profile == "default":
        assert "profiles differ" in capture.failure
    else:
        assert capture.failure is None and capture.request == value


def test_wheel_telemetry_is_optional_not_an_initialization_requirement():
    from run_shadow_capture import TOPICS, REQUIRED_TOPICS
    assert TOPICS["/astrobot_0/joint_states"] == "sensor_msgs/msg/JointState"
    assert "/astrobot_0/joint_states" not in REQUIRED_TOPICS


def test_large_submap_revision_is_recorded_and_validated_without_pose_history(capture):
    from astrobot_interfaces.msg import SlamRevision, SlamFramePose
    from live_revision_gate import LiveRevisionGate
    msg = SlamRevision()
    msg.header.frame_id = "map"
    msg.session_id, msg.localization_epoch = "batch-test", 1
    msg.revision, msg.parent_revision, msg.kind = 1, 0, SlamRevision.SUBMAP
    for i in range(1835):
        frame = SlamFramePose()
        frame.frame_id, frame.submap_id = i, 0
        frame.measurement_stamp.nanosec = i
        frame.map_from_imu.orientation.w = frame.imu_from_lidar.orientation.w = 1.
        msg.frames.append(frame)
    capture.receive("/astrobot_0/slam_raw/revisions", msg)
    assert capture.failure is None
    assert capture.counts["/astrobot_0/slam_raw/revisions"] == 1
    assert isinstance(capture.timeline, LiveRevisionGate)
    assert len(capture.timeline.stamps) == 1835
    assert capture.processing_max_s["/astrobot_0/slam_raw/revisions"] > 0
    assert any(p.endswith("live_revision_gate.py") for p in capture.recorder_sources)


def test_live_terrain_request_requires_native_publisher_and_preserves_failure(capture):
    capture.terrain_library_sha256 = "frozen"
    request = dict(capture="revision-" + "a" * 32)
    capture.receive("/terrain_shadow/request", String(data=json.dumps(request)))
    assert "native publication" in capture.failure
    assert capture.counts["/terrain_shadow/request"] == 1


@pytest.mark.parametrize("status", [dict(failure="deadline"), dict(failure=None, library_sha256="wrong"),
                                    dict(failure=None, library_sha256="frozen", navigation_authority=True)])
def test_live_terrain_failure_or_changed_identity_stops_capture(capture, status):
    capture.terrain_library_sha256 = "frozen"
    capture.receive("/terrain_shadow/status", String(data=json.dumps(status)))
    assert "terrain shadow failed" in capture.failure


def test_live_terrain_status_waiting_is_not_navigation_authority(capture):
    capture.terrain_library_sha256 = "frozen"
    status = dict(failure=None, library_sha256="frozen", navigation_authority=False, valid=False)
    capture.receive("/terrain_shadow/status", String(data=json.dumps(status)))
    assert capture.failure is None and capture.terrain_status == status


def test_actual_ros_callback_finishes_before_deferred_sigint(capture):
    import signal
    import time
    from rclpy.clock import Clock, ClockType
    from local_terrain.shutdown import DeferredInterrupt

    phases = []
    with DeferredInterrupt() as stop:
        def callback():
            phases.append("begin")
            signal.raise_signal(signal.SIGINT)
            phases.append("committed")

        timer = capture.create_timer(0.001, callback, clock=Clock(clock_type=ClockType.STEADY_TIME))
        deadline = time.monotonic() + 2
        while not stop.requested and time.monotonic() < deadline:
            rclpy.spin_once(capture, timeout_sec=0.05)
        capture.destroy_timer(timer)
        assert phases == ["begin", "committed"]
        status = dict(failure=None, library_sha256="frozen", navigation_authority=False,
                      valid=False, completed=True, closed=True)
        capture.terrain_library_sha256 = "frozen"
        capture.receive("/terrain_shadow/status", String(data=json.dumps(status)))
        assert capture.failure is None and capture.terrain_status == status


def test_empty_mapper_failure_is_not_accepted_as_successful_shutdown(capture):
    capture.terrain_library_sha256 = "frozen"
    status = dict(failure="", library_sha256="frozen", navigation_authority=False,
                  valid=False, completed=True, closed=True)
    capture.receive("/terrain_shadow/status", String(data=json.dumps(status)))
    assert "terrain shadow failed" in capture.failure


def test_natural_stationary_mode_never_accepts_a_motion_command(capture):
    capture.natural_stationary = True
    capture.active = ("test", 1)
    command = Twist()
    command.linear.x = .01
    capture.receive("/astrobot_0/cmd_vel", command)
    assert capture.failure is not None
    assert capture.counts["/astrobot_0/cmd_vel"] == 1  # Preserve rejected evidence.


def test_natural_motion_first_output_gate_records_actual_gt_and_refuses_missing_first_frame(capture):
    from geometry_msgs.msg import PoseStamped
    from test_natural_motion import request
    capture.natural_motion = "straight_slow"
    capture.request = request()
    assert not capture.check_initial_output()
    for i in range(31, 100):
        msg = PoseStamped()
        msg.header.frame_id = "gt_map"
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(i*1e9/30), 1_000_000_000)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 20., 18., .12
        msg.pose.orientation.w = 1.
        capture.receive("/gt/base_link_pose", msg)
    assert capture.failure is None
    capture.first_frame = dict(frame_id=1, stamp_ns=3_000_000_000)
    with pytest.raises(ValueError, match="not recorded"):
        capture.check_initial_output()
    assert capture.initial_output_check is None


def test_natural_motion_only_accepts_immutable_matching_route(capture):
    from test_natural_motion import request
    capture.natural_motion = "straight_slow"
    value = request()
    capture.receive("/terrain_shadow/request", String(data=json.dumps(value)))
    assert capture.failure is None
    value["seed_stamp"] += .1
    capture.receive("/terrain_shadow/request", String(data=json.dumps(value)))
    assert "request changed" in capture.failure


def test_natural_stationary_mode_rejects_a_command_publisher_even_if_stopped(capture):
    capture.natural_stationary = True
    capture.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(node_name="terrain_calibration_drive")]
    with pytest.raises(ValueError, match="forbids all command publishers"):
        capture.check_ready()


@pytest.mark.parametrize("capture", [True], indirect=True)
def test_paired_capture_records_and_validates_actual_ros_messages(capture):
    assert {"/astrobot_0/imu_physics", "/astrobot_0/imu_physics/clock"} <= capture.required_topics
    for i in range(60):
        t = 1+i/60
        msg = Imu()
        msg.header.frame_id = "Imu_Sensor"
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(t*1e9), 1_000_000_000)
        msg.orientation_covariance[0] = -1.
        msg.linear_acceleration.z = 1.62
        capture.receive("/astrobot_0/imu_physics", msg)
        capture.receive("/astrobot_0/imu_physics/clock", String(data=json.dumps(dict(
            monotonic_time=t, world_time=t, sensor_time=t-1/60, physics_dt=1/60, epoch=1, valid=True))))
    assert capture.failure is None
    import time
    assert capture.physics.ready(2., time.monotonic())
    capture.physics.armed = True
    capture.receive("/astrobot_0/imu_physics/clock", String(data='{"terminal_fault":"test"}'))
    assert "publisher fault" in capture.failure
    assert capture.counts["/astrobot_0/imu_physics/clock"] == 61


def test_terminal_localization_fault_does_not_wait_for_capture_deadline(capture):
    capture.status = LocalizationStatus(state=LocalizationStatus.FAULTED, reason="estimator_exited")
    with pytest.raises(ValueError, match="localization fault: estimator_exited"):
        capture.check_ready()


def test_nonzero_command_before_barrier_is_recorded_and_fails(capture):
    message = Twist()
    message.angular.z = .01
    capture.receive("/astrobot_0/cmd_vel", message)
    assert capture.counts["/astrobot_0/cmd_vel"] == 1
    assert "motion before" in capture.failure
    with pytest.raises(ValueError, match="motion before"):
        capture.check_ready()


def test_config_changes_fail_closed(capture):
    (capture.config / "config_sensors.json").write_text("{}")
    with pytest.raises(ValueError, match="configuration changed"):
        capture.check_ready()


def test_other_command_publisher_refused_even_before_request(capture, monkeypatch):
    monkeypatch.setattr(capture, "get_publishers_info_by_topic", lambda _: [SimpleNamespace(node_name="controller")])
    with pytest.raises(ValueError, match="another command publisher"):
        capture.check_ready()


def test_malformed_request_recorded_without_arming(capture):
    capture.receive("/terrain_shadow/request", String(data='{"capture":"../elsewhere"}'))
    assert capture.counts["/terrain_shadow/request"] == 1
    assert capture.failure and capture.active is None


def test_armed_input_loss_and_simulator_failure_are_terminal(capture, monkeypatch):
    simulator = allocate_run(capture.run.parent, limit=1024**2, reserve=0, external=False)
    capture.request = {"capture": simulator.name}
    capture.active = ("session", 1)
    monkeypatch.setattr(capture, "get_publishers_info_by_topic", lambda _: [])
    with pytest.raises(ValueError, match="stale/unhealthy"):
        capture.check_ready()
    (simulator / "failure.json").write_text('{}')
    with pytest.raises(ValueError, match="simulator capture failed"):
        capture.check_ready()


def test_finish_barrier_allows_publisher_teardown_but_no_new_motion(capture, monkeypatch):
    simulator = allocate_run(capture.run.parent, limit=1024**2, reserve=0, external=False)
    capture.request = {"capture": simulator.name}
    capture.active = ("session", 1)
    monkeypatch.setattr(capture, "get_publishers_info_by_topic", lambda _: [])
    (simulator / "acquisition_finished.json").write_text(json.dumps(dict(schema=1, capture=simulator.name,
        views=6, stamp=0., commands_stopped=True)))
    assert capture.check_ready() == "sealing"
    (simulator / "multiview.json").write_text('{"complete": true}')
    assert capture.check_ready() == "complete"
    message = Twist()
    message.linear.x = .01
    capture.receive("/astrobot_0/cmd_vel", message)
    assert "after finish" in capture.failure


def test_absent_driver_is_not_excused_by_an_incomplete_finish_marker(capture, monkeypatch):
    simulator = allocate_run(capture.run.parent, limit=1024**2, reserve=0, external=False)
    capture.request = {"capture": simulator.name}
    capture.active = ("session", 1)
    monkeypatch.setattr(capture, "get_publishers_info_by_topic", lambda _: [])
    (simulator / "acquisition_finished.json").write_text(json.dumps(dict(schema=1, capture=simulator.name,
        views=5, stamp=0., commands_stopped=True)))
    with pytest.raises(ValueError, match="invalid acquisition finish"):
        capture.check_ready()


def test_driver_never_publishes_nonzero_without_fresh_lease(monkeypatch):
    import importlib.util
    import time
    from shadow_capture_gate import ShadowLease
    path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/terrain_calibration_drive.py"
    spec = importlib.util.spec_from_file_location("shadow_driver_contract", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    driver = object.__new__(module.CalibrationDrive)
    driver.shadow_lease = ShadowLease("revision-" + "a" * 32)
    driver.shadow_sim = 10.
    commands = []
    driver.pub = SimpleNamespace(publish=commands.append)
    with pytest.raises(ValueError, match="not armed"):
        driver._publish(.15, 0.)
    assert commands[-1].linear.x == commands[-1].angular.z == 0.
    wall = time.monotonic()
    driver.shadow_lease.receive(dict(schema=1, scope="guarded_terrain_capture", ready=True,
        capture=driver.shadow_lease.capture, recorder="revision-" + "b" * 32, session="s", epoch=1,
        config_sha256="c" * 64, sequence=1, stamp=10.), 10., wall)
    driver._publish(.15, 0.)
    assert commands[-1].linear.x == .15
    monkeypatch.setattr(module.time, "monotonic", lambda: wall + .6)
    with pytest.raises(ValueError, match="expired"):
        driver._publish(.15, 0.)
    assert commands[-1].linear.x == commands[-1].angular.z == 0.
    driver._publish()  # Zero remains possible after failure.
