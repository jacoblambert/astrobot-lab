#!/usr/bin/env python3
"""Teleop capture driver: wander the on-map interior through feature-poor terrain to
provoke (on-map) SLAM drift/divergence, while staying inside the terrain bounds and
recovering from getting stuck against rocks. Prints world pose + GT-vs-SLAM gap; exits
when the gap exceeds --gap-exit (a real divergence) or after --max-sec.

Run INSIDE the dev container with GLIM live + a raw bag recorder already running.
"""
import math
import time
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


class Driver(Node):
    def __init__(self, args):
        super().__init__("capture_teleop")
        self.a = args
        self.home = (20.0, 18.0)
        self.lo, self.hi = args.bound_lo, args.bound_hi
        self.g = None      # GT world (x,y)
        self.s = None      # SLAM (home-relative)
        self.pub = self.create_publisher(Twist, "/astrobot_0/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/gt/base_link_pose",
                                 lambda m: setattr(self, "g", (m.pose.position.x, m.pose.position.y)), 10)
        self.create_subscription(PoseStamped, "/astrobot_0/slam/pose_corrected",
                                 lambda m: setattr(self, "s", (m.pose.position.x, m.pose.position.y)), 10)
        self.mode = "FWD"
        self.mode_until = 0.0
        self.last_eval = 0.0
        self.last_pos = None
        self.t0 = time.time()
        self.diverged = False
        self.create_timer(0.1, self.tick)        # 10 Hz command
        self.create_timer(2.0, self.evaluate)    # 2 s decisions

    def gap(self):
        if not (self.g and self.s):
            return None
        gr = (self.g[0] - self.home[0], self.g[1] - self.home[1])
        return math.hypot(gr[0] - self.s[0], gr[1] - self.s[1])

    def tick(self):
        t = Twist()
        if self.mode == "FWD":
            t.linear.x = 0.45
        elif self.mode == "TURN":
            t.angular.z = 0.6
        elif self.mode == "BACK":
            t.linear.x = -0.3
        self.pub.publish(t)

    def evaluate(self):
        now = time.time()
        if now - self.t0 > self.a.max_sec:
            self.get_logger().info("max-sec reached, stopping")
            self.diverged = True  # signal main loop to exit (capture whatever we have)
            return
        if self.g is None:
            return
        gap = self.gap()
        wx, wy = self.g
        gaps = f"{gap:.2f}" if gap is not None else "?"
        self.get_logger().info(f"world=({wx:.1f},{wy:.1f}) gap={gaps} mode={self.mode}")

        if gap is not None and gap >= self.a.gap_exit:
            self.get_logger().info(f">>> ON-MAP DIVERGENCE gap={gap:.2f} at world=({wx:.1f},{wy:.1f})")
            self.diverged = True
            return

        # Stay in an active timed maneuver
        if now < self.mode_until:
            return

        near_edge = wx < self.lo or wx > self.hi or wy < self.lo or wy > self.hi
        moved = None if self.last_pos is None else math.hypot(wx - self.last_pos[0], wy - self.last_pos[1])
        self.last_pos = (wx, wy)

        if near_edge:
            self.mode, self.mode_until = "TURN", now + 2.6     # ~90 deg back inward
        elif moved is not None and moved < 0.15:
            # stuck against a rock -> back up then turn
            if self.mode != "BACK":
                self.mode, self.mode_until = "BACK", now + 3.0
            else:
                self.mode, self.mode_until = "TURN", now + 2.6
        else:
            self.mode, self.mode_until = "FWD", now + 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gap-exit", type=float, default=2.5)
    ap.add_argument("--max-sec", type=float, default=420.0)
    ap.add_argument("--bound-lo", type=float, default=8.0)
    ap.add_argument("--bound-hi", type=float, default=32.0)
    args = ap.parse_args()
    rclpy.init()
    n = Driver(args)
    try:
        while rclpy.ok() and not n.diverged:
            rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        n.pub.publish(Twist())  # stop
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
