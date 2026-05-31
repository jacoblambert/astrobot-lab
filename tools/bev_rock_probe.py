#!/usr/bin/env python3
"""Subscribe to the (regenerated) BEV costmap during replay and log the cost at a
fixed map-frame location over time. Writes CSV incrementally (flush per msg), so
it survives an abrupt kill — no bag finalization needed.

ros2 run-style: pass params via --ros-args -p rock_x:= -p rock_y:= -p radius:= -p csv:=
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class Probe(Node):
    def __init__(self):
        super().__init__("bev_rock_probe")
        self.rock_x = self.declare_parameter("rock_x", -1.62).value
        self.rock_y = self.declare_parameter("rock_y", -6.73).value
        self.rock_world_x = self.declare_parameter("rock_world_x", 18.43).value
        self.rock_world_y = self.declare_parameter("rock_world_y", 11.42).value
        self.radius = self.declare_parameter("radius", 0.75).value
        self.csv = self.declare_parameter("csv", "/tmp/probe.csv").value
        self.f = open(self.csv, "w")
        self.f.write("stamp,cost,gt_dist,total_obstacle_cells\n")
        self.f.flush()
        self.gt_dist = float("nan")
        self.create_subscription(OccupancyGrid, "/astrobot_0/slam/bev_costmap", self.bev_cb, 10)
        self.create_subscription(PoseStamped, "/gt/base_link_pose", self.gt_cb, 10)
        self.get_logger().info(f"probe rock_map=({self.rock_x},{self.rock_y}) r={self.radius} csv={self.csv}")

    def gt_cb(self, m):
        self.gt_dist = math.hypot(m.pose.position.x - self.rock_world_x, m.pose.position.y - self.rock_world_y)

    def bev_cb(self, g):
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        w, h = g.info.width, g.info.height
        cx = int((self.rock_x - ox) / res)
        cy = int((self.rock_y - oy) / res)
        rc = max(0, int(math.ceil(self.radius / res)))
        best = -128
        for dy in range(-rc, rc + 1):
            for dx in range(-rc, rc + 1):
                x, y = cx + dx, cy + dy
                if 0 <= x < w and 0 <= y < h:
                    best = max(best, g.data[y * w + x])
        # global saturation proxy: total cells marked obstacle (>=88) in the whole grid
        total_obs = sum(1 for v in g.data if v >= 88)
        stamp = g.header.stamp.sec + g.header.stamp.nanosec * 1e-9
        self.f.write(f"{stamp:.3f},{best},{self.gt_dist:.2f},{total_obs}\n")
        self.f.flush()


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()


if __name__ == "__main__":
    main()
