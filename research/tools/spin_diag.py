#!/usr/bin/env python3
"""Live spin-in-place diagnostic. Run INSIDE the dev container alongside a mission.

Watches GT pose; when the rover is stationary-but-rotating (a spin episode), snapshots
the BEV costmap + Nav2 local costmap + recent cmd_vel and saves them (npz) for offline
analysis of WHY the controller is boxed in (phantom obstacle vs inflation walling).

Exits after --episodes captures (default 2) or --duration seconds.
"""
from __future__ import annotations

import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SpinDiag(Node):
    def __init__(self, args):
        super().__init__("spin_diag")
        self.args = args
        self.hist = []          # (t, x, y, yaw) GT
        self.cmds = []          # (t, vx, wz)
        self.grids = {}         # topic -> (msg-derived dict), latest
        self.captures = 0
        self.last_capture_t = 0.0
        self.create_subscription(PoseStamped, "/gt/base_link_pose", self._gt_cb, 10)
        self.create_subscription(Twist, args.cmd_vel_topic, self._cmd_cb, 10)
        for key, topic in (("bev", args.bev_topic), ("local", args.local_costmap_topic)):
            self.create_subscription(OccupancyGrid, topic, lambda m, k=key: self._grid_cb(k, m), 1)

    def _grid_cb(self, key, msg):
        self.grids[key] = {
            "data": np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width),
            "res": msg.info.resolution,
            "ox": msg.info.origin.position.x,
            "oy": msg.info.origin.position.y,
            "frame": msg.header.frame_id,
        }

    def _cmd_cb(self, msg):
        self.cmds.append((time.monotonic(), msg.linear.x, msg.angular.z))
        self.cmds = self.cmds[-400:]

    def _gt_cb(self, msg):
        t = time.monotonic()
        self.hist.append((t, msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation)))
        self.hist = [h for h in self.hist if t - h[0] <= 10.0]
        if len(self.hist) < 20 or t - self.last_capture_t < 30.0:
            return
        # spin = stationary (<0.2m over the window) + cumulative |dyaw| > 90deg
        disp = math.hypot(self.hist[-1][1] - self.hist[0][1], self.hist[-1][2] - self.hist[0][2])
        cum = sum(abs((b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi)
                  for a, b in zip(self.hist, self.hist[1:]))
        if disp < 0.2 and math.degrees(cum) > 90.0:
            self._capture(t)

    def _capture(self, t):
        self.last_capture_t = t
        self.captures += 1
        x, y = self.hist[-1][1], self.hist[-1][2]
        out = {
            "gt_x": x, "gt_y": y, "gt_yaw": self.hist[-1][3],
            "cmds": np.asarray(self.cmds, dtype=np.float64),
            "hist": np.asarray(self.hist, dtype=np.float64),
        }
        for key, g in self.grids.items():
            out[f"{key}_data"] = g["data"]
            out[f"{key}_meta"] = np.asarray([g["res"], g["ox"], g["oy"]], dtype=np.float64)
            out[f"{key}_frame"] = np.asarray([g["frame"]])
        path = f"{self.args.output_prefix}_ep{self.captures}.npz"
        np.savez_compressed(path, **out)
        self.get_logger().warn(f"SPIN EPISODE captured -> {path} (GT {x:.1f},{y:.1f})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bev-topic", default="/astrobot_0/slam/bev_costmap")
    ap.add_argument("--local-costmap-topic", default="/astrobot_0/local_costmap/costmap")
    ap.add_argument("--cmd-vel-topic", default="/astrobot_0/cmd_vel")
    ap.add_argument("--output-prefix", default="/workspace/astrobot-lab/rosbags/spin_diag")
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--duration", type=float, default=1200.0)
    args = ap.parse_args()
    rclpy.init()
    node = SpinDiag(args)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and node.captures < args.episodes and time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"spin_diag done: {node.captures} episode(s) captured")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
