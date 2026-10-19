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
