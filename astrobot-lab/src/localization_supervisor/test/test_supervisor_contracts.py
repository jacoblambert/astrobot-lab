"""Real ROS node callbacks with deterministic input/futures; no estimator child."""
from concurrent.futures import Future
from array import array
from types import SimpleNamespace as NS
from unittest.mock import Mock
import json

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from localization_supervisor.node import LocalizationSupervisor, Status


@pytest.fixture
def supervisor():
    rclpy.init()
    node = LocalizationSupervisor()
    node.now_sec = lambda: 10.0
    node.last_sim = 9.9
    node.clock_paused = False
    node._launch_child = Mock()
    node.health_pub = Mock()
    node.pose_pub = Mock()
    node.cancel_client = Mock()
    node.cancel_client.service_is_ready.return_value = False
    yield node
    node.destroy_node()
    rclpy.shutdown()


def pose(stamp=10, x=0):
    msg = PoseStamped()
    msg.header.stamp.sec = stamp
    msg.header.frame_id = "map"
    msg.pose.position.x = float(x)
    msg.pose.orientation.w = 1.0
    return msg


def test_live_gt_seed_rejects_reuse_before_any_child_launch(supervisor):
    supervisor.shadow_mode = True
    supervisor.shadow_seed_path = "/unused-seed"
    supervisor.shadow_seed_consumed = True
    with pytest.raises(ValueError, match="cannot be reused"):
        LocalizationSupervisor._launch_child(supervisor, [0, 0, 0, 0, 0, 0, 1])


def test_live_gt_seed_never_overrides_nonshadow_launch(supervisor, tmp_path):
    seed = tmp_path / "seed.json"
    seed.write_text("{}")
    supervisor.shadow_mode = False
    supervisor.shadow_seed_path = str(seed)
    supervisor.shadow_seed_consumed = False
    with pytest.raises(ValueError, match="forbidden outside"):
        LocalizationSupervisor._launch_child(supervisor, [0, 0, 0, 0, 0, 0, 1])
    assert not supervisor.shadow_seed_consumed


def test_terminal_fault_invalidates_pending_reset(supervisor):
    node = supervisor
    pending = Future()
    node.reset_future = pending
    node._fault("injected", terminal=True)
    assert pending.cancelled()
    node._tick()
    node._launch_child.assert_not_called()
    assert node.state == Status.FAULTED
    assert not node.health_pub.publish.call_args.args[0].motion_permitted


def test_bad_pose_is_not_published_and_clears_output_queues(supervisor):
    node = supervisor
    node.state = Status.HEALTHY
    node.pending_clouds.append(object())
    node._pose(pose(x=float("nan")))
    node.pose_pub.publish.assert_not_called()
    assert not node.pending_clouds and node.state == Status.STOPPING


def test_duplicate_frame_revision_does_not_trigger_restart(supervisor):
    node = supervisor
    node.state = Status.REBUILDING
    node._pose(pose())
    node._pose(pose())
    assert node.state == Status.REBUILDING
    assert node.pose_pub.publish.call_count == 1


def test_rebuild_needs_local_fresh_support(supervisor):
    node = supervisor
    node.last_pose = pose(x=5)
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.header.stamp.sec = 10
    msg.info.resolution = .1
    msg.info.width, msg.info.height = 100, 20
    msg.data = [-1]*2000
    msg.data[:30] = array('b', [0]*30)  # Free cells exist, but nowhere near the rover.
    node._map(msg)
    assert not node.map_ready
    for row in range(5):
        msg.data[row*100+45:row*100+50] = array('b', [0]*5)
    node._map(msg)
    assert node.map_ready
    msg.header.stamp.sec = 8
    node._map(msg)
    assert not node.map_ready


@pytest.mark.parametrize("field,value", [("linear.y", .03), ("linear.z", .03),
                                         ("angular.x", .1), ("angular.y", .1), ("linear.x", float("nan"))])
def test_restart_requires_stationary_finite_six_axis_twist(supervisor, field, value):
    node = supervisor
    node.odom = Odometry()
    node.odom.header.stamp.sec = 10
    group, axis = field.split(".")
    setattr(getattr(node.odom.twist.twist, group), axis, value)
    node.stationary_since = 1
    assert node._stationary_reference(10) is None
    assert node.stationary_since is None


def test_failed_reinitializations_are_bounded(supervisor):
    node = supervisor
    node._stationary_reference = lambda _: [0,0,0,0,0,0,1]
    node.reset_client = Mock()
    node.reset_client.service_is_ready.return_value = True
    def rejected(_):
        future = Future()
        future.set_result(NS(success=False, epoch=node.epoch))
        return future
    node.reset_client.call_async.side_effect = rejected
    for _ in range(3):
        node._tick()
    assert node.reset_client.call_async.call_count == 2
    assert node.state == Status.FAULTED
    node._launch_child.assert_not_called()


def test_wrong_epoch_reset_ack_does_not_launch(supervisor):
    node = supervisor
    node.state = Status.REINITIALIZING
    node.reset_future = Future()
    node.reset_future.set_result(NS(success=True, epoch=999))
    node._tick()
    assert node.state == Status.STOPPING
    node._launch_child.assert_not_called()


def test_mapping_capacity_fault_is_terminal(supervisor):
    supervisor.state = Status.HEALTHY
    supervisor._map_integrity(Bool(data=False))
    assert supervisor.state == Status.FAULTED
    assert supervisor.reason == "map_capacity_exceeded"


def test_stale_map_holds_even_when_pose_is_fresh(supervisor):
    supervisor.state = Status.HEALTHY
    supervisor.last_pose = pose()
    supervisor.map_stamp = 8.0
    supervisor._tick()
    assert supervisor.state == Status.STOPPING
    assert supervisor.reason == "map_stale"


@pytest.mark.parametrize("section", ["sub_mapping", "global_mapping"])
def test_optimization_requires_revision_consumer(supervisor, tmp_path, section):
    node = supervisor
    node.config_path = tmp_path
    files = {"config_" + key: key + ".json" for key in ("sub_mapping", "global_mapping")}
    (tmp_path / "config.json").write_text(json.dumps({"global": files}))
    for key in ("sub_mapping", "global_mapping"):
        (tmp_path / files["config_" + key]).write_text(json.dumps({key: {"enable_optimization": key == section}}))
    LocalizationSupervisor._launch_child(node, [0, 0, 0, 0, 0, 0, 1])
    assert node.state == Status.FAULTED
    assert node.reason == "revision_consumer_required"
    assert node.process is None


def test_missing_revision_config_fails_closed(supervisor, tmp_path):
    supervisor.config_path = tmp_path
    LocalizationSupervisor._launch_child(supervisor, [0, 0, 0, 0, 0, 0, 1])
    assert supervisor.state == Status.FAULTED
    assert supervisor.reason == "invalid_mapping_revision_config"


def test_shadow_initializes_without_map_services_but_never_permits_navigation(supervisor):
    node = supervisor
    node.shadow_mode = True
    reference = [0, 0, 0, 0, 0, 0, 1]
    node._stationary_reference = lambda _: reference
    node.reset_client = Mock()
    node.reset_client.service_is_ready.return_value = False
    node._tick()
    node._launch_child.assert_called_once_with(reference)
    node.reset_client.call_async.assert_not_called()
    assert node.epoch == 1
    node.state = Status.REBUILDING
    node.last_pose = pose()
    node.healthy_since = 7.
    node._tick()
    assert node.state == Status.HEALTHY
    assert node.reason == "shadow_pose_ready_not_navigation"
    assert not node.health_pub.publish.call_args.args[0].motion_permitted


def test_shadow_fault_is_terminal_and_does_not_restart(supervisor):
    supervisor.shadow_mode = True
    supervisor.state = Status.REBUILDING
    supervisor._fault("injected")
    supervisor._tick()
    assert supervisor.state == Status.FAULTED
    supervisor._launch_child.assert_not_called()


def test_shadow_rejects_navigation_action_activity(supervisor):
    supervisor.shadow_mode = True
    supervisor.nav_active = True
    supervisor._tick()
    assert supervisor.state == Status.FAULTED
    assert supervisor.reason == "navigation_active_in_shadow_mode"
    supervisor._launch_child.assert_not_called()


def fix(t, x=0., y=0., z=0., sigma=.075):
    from geometry_msgs.msg import PoseWithCovarianceStamped

    msg = PoseWithCovarianceStamped()
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
    msg.header.frame_id = "map"
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = float(x), float(y), float(z)
    msg.pose.pose.orientation.w = 1.0
    cov = [0.0] * 36
    cov[0] = cov[7] = sigma**2
    cov[14] = (2 * sigma) ** 2  # synthetic_rtk sigma_z_scale
    cov[21] = cov[28] = 1e6
    cov[35] = 1e-4
    msg.pose.covariance = cov
    return msg


def feed(node, t, msg):
    node.now_sec = lambda: t
    node._fix(msg)


def test_vertical_variance_does_not_widen_the_horizontal_jump_gate(supervisor):
    # sigma 7.5 cm (55 m from home): a 1 m horizontal jump between consecutive fixes is
    # ~9 per-axis sigma. The old 3-D trace gate (z sigma 2x) admitted it (1.67 m).
    feed(supervisor, 9.6, fix(9.6))
    feed(supervisor, 9.633, fix(9.633, x=1.0))
    assert len(supervisor.fixes) == 1
    feed(supervisor, 9.666, fix(9.666, x=.1, z=.3))  # plausible noise + motion: accepted
    assert len(supervisor.fixes) == 2


def test_persistent_outlier_stays_rejected_while_glim_covers_the_interval(supervisor):
    for k in range(21):
        supervisor.recent_poses.append((9.0 + k * .05, (0., 0., 0.)))  # stationary GLIM
    feed(supervisor, 9.5, fix(9.5))
    for t in (9.55, 9.7, 9.85, 9.95):  # the speed bound alone would admit 9.95 / 10.1+
        feed(supervisor, t, fix(t, x=1.0))
    assert [f.pose.pose.position.x for f in supervisor.fixes] == [0.]


def test_one_noisy_outlier_fix_cannot_capture_the_gate(supervisor):
    # Fix-to-fix gating chains: once one burst fix slips in, the rest follow it. Against
    # GLIM, every later burst fix is still 1 m from the estimate.
    for k in range(21):
        supervisor.recent_poses.append((9.0 + k * .05, (0., 0., 0.)))
    feed(supervisor, 9.5, fix(9.5, x=.5))  # burst fix pulled back by noise: inside 0.55 m
    for t in (9.55, 9.6, 9.65):
        feed(supervisor, t, fix(t, x=1.0))
    assert [f.pose.pose.position.x for f in supervisor.fixes] == [.5]


def test_real_motion_follows_glim_displacement_across_a_gap(supervisor):
    for k in range(21):
        supervisor.recent_poses.append((9.0 + k * .05, (.5 * k * .05, 0., 0.)))  # 0.5 m/s
    feed(supervisor, 9.2, fix(9.2, x=.1))
    feed(supervisor, 9.95, fix(9.95, x=.475))  # 0.75 s gap, moved with GLIM
    assert len(supervisor.fixes) == 2
    (x, _, _), uncovered = supervisor._estimated_position(9.95)
    assert abs(x - .475) < 1e-9 and uncovered == 0.
    assert supervisor._estimated_position(10.6) is None  # GLIM silent > 0.5 s: speed bound instead


@pytest.mark.parametrize("x,z,faults", [(.6, 0., True), (0., .6, False)])
def test_disagreement_fault_is_per_axis(supervisor, x, z, faults):
    # sigma 7.5 cm: horizontal limit max(0.5, 0.45) = 0.5 m, vertical max(0.5, 6 * 0.15) = 0.9 m
    node = supervisor
    node.state = Status.HEALTHY
    feed(node, 10.0, fix(10.0))
    node._pose(_pose_at(10.0, x=x, z=z))
    assert (node.reason == "absolute_position_disagreement") is faults
    assert (node.state == Status.STOPPING) is faults


def _pose_at(t, x=0., z=0.):
    msg = pose(x=x)
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
    msg.pose.position.z = float(z)
    return msg



def test_map_topic_defaults_to_legacy_bev_and_is_configurable():
    from rclpy.parameter import Parameter

    rclpy.init()
    try:
        legacy = LocalizationSupervisor()
        assert legacy.map_topic == "slam/bev_costmap"
        assert legacy.get_subscriptions_info_by_topic("/slam/bev_costmap")
        legacy.destroy_node()
        v2 = LocalizationSupervisor(parameter_overrides=[Parameter("map_topic", value="/terrain_shadow/planner_costmap")])
        assert v2.map_topic == "/terrain_shadow/planner_costmap"
        assert v2.get_subscriptions_info_by_topic("/terrain_shadow/planner_costmap")
        v2.destroy_node()
    finally:
        rclpy.shutdown()


def test_one_late_map_message_does_not_erase_the_last_fresh_map(supervisor):
    node = supervisor
    node.last_pose = pose(x=5)
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.header.stamp.sec = 10
    msg.info.resolution = .1
    msg.info.width, msg.info.height = 100, 20
    msg.data = array('b', [0]*2000)
    node._map(msg)
    assert node.map_ready and node.map_stamp == 10
    msg.header.stamp.sec = 8  # 2 s old on arrival: rejected, but the 10 s map still counts
    node._map(msg)
    assert not node.map_ready and node.map_stamp == 10
