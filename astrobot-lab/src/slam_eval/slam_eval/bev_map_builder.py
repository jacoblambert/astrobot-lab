#!/usr/bin/env python3
"""Build a regular BEV OccupancyGrid from GLIM-frame PointCloud2 scans."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from slam_eval.compare_bev_maps import pointcloud_xyz


class BevMapBuilder(Node):
    def __init__(self) -> None:
        super().__init__("bev_map_builder")
        self.input_topic = str(self.declare_parameter("input_topic", "/glim_rosnode/aligned_points_corrected").value)
        self.output_topic = str(self.declare_parameter("output_topic", "/slam/bev_map").value)
        self.resolution = float(self.declare_parameter("resolution", 0.10).value)
        self.width_m = float(self.declare_parameter("width_m", 40.0).value)
        self.height_m = float(self.declare_parameter("height_m", 40.0).value)
        self.origin_x = float(self.declare_parameter("origin_x", -20.0).value)
        self.origin_y = float(self.declare_parameter("origin_y", -20.0).value)
        self.height_threshold = float(self.declare_parameter("height_threshold", 0.02).value)
        self.min_points_per_cell = int(self.declare_parameter("min_points_per_cell", 2).value)
        self.max_scans = int(self.declare_parameter("max_scans", 800).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 1.0).value)

        self.width = max(1, int(math.ceil(self.width_m / self.resolution)))
        self.height = max(1, int(math.ceil(self.height_m / self.resolution)))
        self.scans: list[np.ndarray] = []
        self.frame_id: Optional[str] = None

        self.map_pub = self.create_publisher(OccupancyGrid, self.output_topic, 1)
        self.create_subscription(PointCloud2, self.input_topic, self._cloud_cb, 10)
        self.create_timer(1.0 / max(self.publish_rate_hz, 0.1), self._publish_map)
        self.get_logger().info(
            "BEV map builder: %s -> %s resolution=%.2f size=%dx%d height_threshold=%.3f max_scans=%d"
            % (
                self.input_topic,
                self.output_topic,
                self.resolution,
                self.width,
                self.height,
                self.height_threshold,
                self.max_scans,
            )
        )

    def _cloud_cb(self, msg: PointCloud2) -> None:
        points = pointcloud_xyz(msg)
        if points.size == 0:
            return
        self.frame_id = msg.header.frame_id or self.frame_id
        self.scans.append(points)
        if self.max_scans > 0 and len(self.scans) > self.max_scans:
            del self.scans[: len(self.scans) - self.max_scans]

    def _publish_map(self) -> None:
        if not self.scans:
            return
        points = np.concatenate(self.scans, axis=0)
        col = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(np.int64)
        inside = (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)
        row = row[inside]
        col = col[inside]
        z = points[inside, 2]

        counts = np.zeros((self.height, self.width), dtype=np.int32)
        min_z = np.full((self.height, self.width), np.inf, dtype=np.float64)
        max_z = np.full((self.height, self.width), -np.inf, dtype=np.float64)
        np.add.at(counts, (row, col), 1)
        np.minimum.at(min_z, (row, col), z)
        np.maximum.at(max_z, (row, col), z)

        observed = counts >= self.min_points_per_cell
        occupied = observed & ((max_z - min_z) >= self.height_threshold)
        data = np.full((self.height, self.width), -1, dtype=np.int8)
        data[observed] = 0
        data[occupied] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id or "glim_map"
        msg.info = MapMetaData()
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = data.reshape(-1).tolist()
        self.map_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BevMapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
