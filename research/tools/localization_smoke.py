#!/usr/bin/env python3
"""Read-only stationary integration probe. Never publishes motion or goals."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from astrobot_interfaces.msg import LocalizationStatus
from exploration_manager.reachability import clearance_components, component_at


class Probe(Node):
    def __init__(self):
        super().__init__("localization_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.counts, self.events = Counter(), []
        self.ages = {"gt": [], "slam": []}
        self.gt, self.slam, self.grid = [], None, None
        self.slam_pose = None
        self.nonzero_commands = 0
        self.last_key = None
        self.create_subscription(PoseStamped, "/gt/base_link_pose", lambda m: self.pose("gt", m), 10)
        self.create_subscription(PoseStamped, "/astrobot_0/slam/pose_corrected", lambda m: self.pose("slam", m), 10)
        self.create_subscription(LocalizationStatus, "/astrobot_0/localization/status", self.health, 20)
        self.create_subscription(Twist, "/astrobot_0/cmd_vel", self.command, 10)
        self.create_subscription(OccupancyGrid, "/astrobot_0/slam/bev_costmap", self.map, qos_profile_sensor_data)

    def pose(self, name, msg):
        self.counts[name] += 1
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.get_clock().now().nanoseconds:
            self.ages[name].append(self.get_clock().now().nanoseconds*1e-9-ts)
        if name == "gt":
            self.gt.append((msg.pose.position.x, msg.pose.position.y))
        else:
            self.slam = msg.pose.position
            self.slam_pose = msg.pose

    def health(self, msg):
        self.counts["health"] += 1
        key = (msg.epoch, msg.state, msg.reason, msg.motion_permitted)
        if key != self.last_key:
            self.events.append({"epoch": msg.epoch, "state": msg.state, "reason": msg.reason,
                                "motion_permitted": msg.motion_permitted})
            self.last_key = key

    def command(self, msg):
        self.counts["command"] += 1
        if any(not math.isfinite(v) or abs(v) > 1e-6 for v in
               (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)):
            self.nonzero_commands += 1

    def map(self, msg):
        self.counts["map"] += 1
        self.grid = msg

    def summary(self):
        result = {"counts": dict(self.counts), "health_transitions": self.events,
                  "nonzero_commands": self.nonzero_commands,
                  "gt_displacement_m": math.dist(self.gt[0], self.gt[-1]) if self.gt else None,
                  "pose_age_sec": {name: {"min": min(ages), "p95": float(np.percentile(ages, 95)), "max": max(ages)}
                                   for name, ages in self.ages.items() if ages}}
        if self.grid is not None:
            msg = self.grid
            grid = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
            result["map_cells"] = {"unknown": int(np.count_nonzero(grid < 0)),
                                   "free": int(np.count_nonzero((grid >= 0) & (grid < 30))),
                                   "lethal": int(np.count_nonzero(grid >= 100))}
            if self.slam is not None:
                start = (math.floor((self.slam.y-msg.info.origin.position.y)/msg.info.resolution),
                         math.floor((self.slam.x-msg.info.origin.position.x)/msg.info.resolution))
                labels = clearance_components((grid >= 0) & (grid <= 30), msg.info.resolution, .75)
                result["reachable_cells_near_imu"] = int(component_at(labels, start).sum())
                q = self.slam_pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                bx = self.slam.x + math.cos(yaw)*.264 - math.sin(yaw)*.017
                by = self.slam.y + math.sin(yaw)*.264 + math.cos(yaw)*.017
                base_cell = (math.floor((by-msg.info.origin.position.y)/msg.info.resolution),
                             math.floor((bx-msg.info.origin.position.x)/msg.info.resolution))
                result["base_xy"] = [bx, by]
                result["reachable_cells_at_base"] = int(component_at(labels, base_cell).sum())
                result["clearance_safe_cells_total"] = int(np.count_nonzero(labels))
                row, col = base_cell
                result["base_map_window_3m"] = ["".join("?" if v<0 else "#" if v>=100 else "." if v<=30 else "+" for v in line)
                    for line in grid[max(0,row-15):row+16, max(0,col-15):col+16]]
        result["pass"] = (self.counts["gt"] > 10 and self.counts["slam"] > 10 and self.nonzero_commands == 0
                          and self.last_key is not None and self.last_key[1] == LocalizationStatus.HEALTHY
                          and result.get("gt_displacement_m", math.inf) < .1
                          and self.ages["gt"] and max(self.ages["gt"]) < .5)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wall-seconds", type=float, default=60)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    end = time.monotonic()+args.wall_seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        result = node.summary()
        text = json.dumps(result, indent=2)
        with args.output.open("x") as output:
            output.write(text+"\n")
        print(text)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result["pass"] else 1)


if __name__ == "__main__":
    main()
