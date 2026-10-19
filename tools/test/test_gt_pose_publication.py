"""Exercise actual publisher methods with tiny transport fakes, no Isaac/ROS.

Extract only these methods to avoid importing Isaac extension modules on host.
The tested method bodies are the production source, not a copied implementation.
"""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


def publisher(on_tick):
    source = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/gt_map_ros2.py"
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ROS_GTMapPublisher")
    names = {"_configure_pose_publication", "publish_captured_pose", "_publish_gt_pose", "_configure_grid_publication"}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len(methods) == len(names)
    def message(): return NS(header=NS(), pose=NS(position=NS(), orientation=NS()))
    env = dict(PoseStamped=message, Time=lambda seconds: NS(to_msg=lambda: seconds))
    code = ast.Module(body=[ast.ClassDef(name="Publisher", bases=[], keywords=[], body=methods, decorator_list=[])], type_ignores=[])
    exec(compile(ast.fix_missing_locations(code), str(source), "exec"), env)
    node = env["Publisher"]()
    node.timers, node.messages = [], []
    node.create_timer = lambda interval, callback: node.timers.append((interval, callback))
    node.gt_pose_pub = NS(publish=node.messages.append)
    node.robot_name, node.map_frame = "robot", "gt_map"
    node.snapshot = (12., (1., 2., 3.), (0., 0., 0., 1.))
    node.environment_controller = NS(get_gt_robot_pose_snapshot=lambda name: node.snapshot)
    node._configure_pose_publication(30., on_tick)
    return node


def test_default_timer_mode_does_not_also_publish_on_tick():
    node = publisher(False)
    assert len(node.timers) == 1 and node.timers[0][0] == pytest.approx(1/30)
    node.publish_captured_pose()
    assert not node.messages
    node.timers[0][1]()
    assert len(node.messages) == 1


def test_tick_mode_preserves_each_snapshot_without_timer_or_duplicate():
    node = publisher(True)
    assert not node.timers
    for i in range(31):
        node.snapshot = (12.+i/30., (1., 2., 3.), (0., 0., 0., 1.))
        node.publish_captured_pose()
        node.publish_captured_pose()
    assert len(node.messages) == 31
    assert [m.header.stamp for m in node.messages] == [12.+i/30 for i in range(31)]
    assert node.messages[0].header.frame_id == "gt_map"
    assert node.messages[0].pose.position.z == 3.
    assert node.messages[0].pose.orientation.w == 1.


def test_no_snapshot_never_becomes_a_current_clock_pose():
    node = publisher(True)
    node.snapshot = None
    node.publish_captured_pose()
    assert not node.messages


def test_grid_ablation_does_not_disable_or_retime_ground_truth_poses():
    for enabled in (True, False):
        node = publisher(True)
        node._publish_map = lambda: None
        node._configure_grid_publication(1., enabled)
        assert len(node.timers) == int(enabled)
        node.publish_captured_pose()
        assert len(node.messages) == 1 and node.messages[0].header.stamp == 12.
