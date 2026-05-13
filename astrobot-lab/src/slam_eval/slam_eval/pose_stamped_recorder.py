#!/usr/bin/env python3
"""Record a PoseStamped topic as a TUM trajectory."""

from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


class PoseStampedRecorder(Node):
    def __init__(self, topic: str, output: Path) -> None:
        super().__init__("pose_stamped_recorder")
        self._output = output
        self._output.parent.mkdir(parents=True, exist_ok=True)
        self._file = self._output.open("w", encoding="utf-8")
        self._count = 0
        self._subscription = self.create_subscription(PoseStamped, topic, self._on_pose, 100)
        self.get_logger().info(f"recording {topic} to {self._output}")

    def _on_pose(self, msg: PoseStamped) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        self._file.write(f"{stamp:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
        self._count += 1
        if self._count % 100 == 0:
            self._file.flush()

    def destroy_node(self) -> bool:
        self._file.flush()
        self._file.close()
        self.get_logger().info(f"wrote {self._count} poses to {self._output}")
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/glim_rosnode/pose", help="PoseStamped topic to record.")
    parser.add_argument("--output", required=True, help="Output TUM trajectory path.")
    args = parser.parse_args(remove_ros_args()[1:])

    rclpy.init()
    node = PoseStampedRecorder(args.topic, Path(args.output))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
