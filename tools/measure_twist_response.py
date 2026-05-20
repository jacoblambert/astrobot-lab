#!/usr/bin/env python3
import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose_pose(a: Pose2D, b: Pose2D) -> Pose2D:
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    return Pose2D(
        x=a.x + c * b.x - s * b.y,
        y=a.y + s * b.x + c * b.y,
        yaw=normalize_angle(a.yaw + b.yaw),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class TwistProbe(Node):
    def __init__(self) -> None:
        super().__init__("measure_twist_response")
        self.odom_msg: Optional[Odometry] = None
        self.tf_msgs = []
        self.tf_static_msgs = []
        self.create_subscription(Odometry, "/astrobot_0/odom", self._odom_cb, 50)
        self.create_subscription(TFMessage, "/tf", self._tf_cb, 200)
        self.create_subscription(TFMessage, "/tf_static", self._tf_static_cb, 50)
        self.mode_pub = self.create_publisher(String, "/astrobot_0/control/mode", 10)
        self.cmd_pub = self.create_publisher(Twist, "/astrobot_0/cmd_vel_teleop", 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom_msg = msg

    def _tf_cb(self, msg: TFMessage) -> None:
        self.tf_msgs.extend(msg.transforms)
        self.tf_msgs = self.tf_msgs[-1000:]

    def _tf_static_cb(self, msg: TFMessage) -> None:
        self.tf_static_msgs.extend(msg.transforms)
        self.tf_static_msgs = self.tf_static_msgs[-400:]

    def latest_transform(self, parent: str, child: str):
        all_tf = self.tf_msgs + self.tf_static_msgs
        matches = [t for t in all_tf if t.header.frame_id == parent and t.child_frame_id == child]
        return matches[-1] if matches else None

    def wait_for_pose(self, timeout_sec: float = 10.0) -> None:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_msg is not None and self.latest_transform("gt_map", "odom"):
                return
        raise RuntimeError("timed out waiting for gt_map->odom and /astrobot_0/odom")

    def current_pose_in_map(self) -> Pose2D:
        map_to_odom = self.latest_transform("gt_map", "odom")
        if map_to_odom is None or self.odom_msg is None:
            raise RuntimeError("missing gt_map->odom transform or /astrobot_0/odom")

        p1 = Pose2D(
            x=map_to_odom.transform.translation.x,
            y=map_to_odom.transform.translation.y,
            yaw=yaw_from_quaternion(map_to_odom.transform.rotation),
        )
        odom_pose = self.odom_msg.pose.pose
        p2 = Pose2D(
            x=odom_pose.position.x,
            y=odom_pose.position.y,
            yaw=yaw_from_quaternion(odom_pose.orientation),
        )
        return compose_pose(p1, p2)

    def publish_mode(self, mode: str, count: int = 5) -> None:
        msg = String()
        msg.data = mode
        for _ in range(count):
            self.mode_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_twist_for_duration(self, linear_x: float, angular_z: float, duration_sec: float, rate_hz: float) -> None:
        period = 1.0 / rate_hz
        end_time = time.time() + duration_sec
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        while time.time() < end_time:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.cmd_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--linear-x", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.8)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--settle", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = TwistProbe()
    try:
        node.wait_for_pose()
        node.publish_mode("teleop")
        start = node.current_pose_in_map()
        t0 = time.time()
        node.publish_twist_for_duration(args.linear_x, args.angular_z, args.duration, args.rate)
        while time.time() - t0 < args.duration + args.settle:
            rclpy.spin_once(node, timeout_sec=0.05)
        end = node.current_pose_in_map()
        dx = end.x - start.x
        dy = end.y - start.y
        dist = math.hypot(dx, dy)
        dyaw = normalize_angle(end.yaw - start.yaw)
        total = max(args.duration, 1e-6)
        print(
            "cmd_vx=%.3f cmd_wz=%.3f duration=%.2f dist=%.3f dyaw=%.3f avg_v=%.3f avg_wz=%.3f"
            % (args.linear_x, args.angular_z, args.duration, dist, dyaw, dist / total, dyaw / total)
        )
        print(
            "start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)"
            % (start.x, start.y, start.yaw, end.x, end.y, end.yaw)
        )
        return 0
    finally:
        try:
            node.publish_mode("auto")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
