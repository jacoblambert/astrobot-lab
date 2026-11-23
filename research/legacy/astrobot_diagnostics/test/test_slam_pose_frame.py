"""The diagnostics SLAM-vs-GT comparison uses base_link, not GLIM's IMU pose."""
import math

import pytest

pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseStamped  # noqa: E402

from astrobot_diagnostics.diagnostics_node import IMU_BASE_X, IMU_BASE_Y, DiagnosticsNode  # noqa: E402


def imu_pose(x, y, yaw):
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


@pytest.mark.parametrize("base_yaw", [0.0, 1.0, math.pi / 2, -2.5])
def test_slam_pose_is_composed_to_base_link(base_yaw):
    # base at (5, 3, base_yaw): the IMU sits at the base pose composed with the inverse lever arm
    imu_yaw = base_yaw - math.pi
    c, s = math.cos(imu_yaw), math.sin(imu_yaw)
    imu_x = 5.0 - (c * IMU_BASE_X - s * IMU_BASE_Y)
    imu_y = 3.0 - (s * IMU_BASE_X + c * IMU_BASE_Y)
    base = DiagnosticsNode._slam_base_tuple(imu_pose(imu_x, imu_y, imu_yaw))
    assert base.x == pytest.approx(5.0) and base.y == pytest.approx(3.0)
    assert math.cos(base.yaw - base_yaw) == pytest.approx(1.0)


def test_an_in_place_turn_moves_the_imu_but_not_the_base():
    a = DiagnosticsNode._slam_base_tuple(imu_pose(0.0, 0.0, 0.0))
    turned_yaw = math.pi / 2
    # rotate the IMU about the base point a
    dx, dy = -IMU_BASE_X, -IMU_BASE_Y  # IMU relative to base at imu yaw 0
    c, s = math.cos(turned_yaw), math.sin(turned_yaw)
    b = DiagnosticsNode._slam_base_tuple(imu_pose(a.x + c * dx - s * dy, a.y + s * dx + c * dy, turned_yaw))
    assert math.hypot(b.x - a.x, b.y - a.y) == pytest.approx(0.0, abs=1e-9)
