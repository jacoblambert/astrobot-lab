#!/usr/bin/env python3
"""Drive the rover through world-frame waypoints via /astrobot_0/cmd_vel using GT pose
feedback (diagnostic tool -- no nav stack needed). Used to REPRODUCE known stall/bog
locations under the Isaac GUI (headless=off) so the physics can be inspected visually.

On bog detection (commanded forward but GT speed < threshold for bog_sec), it announces
loudly, keeps commanding for inspect_sec so wheels/contact can be watched in the GUI,
then stops and holds position (robot left in place for terrain inspection).

Example (the seed42 trap at (13.6,13.45), approach from the southeast like the mission did):
  python3 tools/drive_route.py --waypoints "18.5,16.5 16.5,14.5 15.0,12.8 14.3,13.0 13.6,13.45 12.9,13.9"
"""
import math
import time
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RouteDriver(Node):
    def __init__(self, args):
        super().__init__("drive_route")
        self.a = args
        self.wps = []
        for tok in args.waypoints.split():
            x, y = tok.split(",")
            self.wps.append((float(x), float(y)))
        self.idx = 0
        self.pose = None  # (x, y, yaw) world
        self.hist = []    # (t, x, y) for speed estimate
        self.bog_since = None
        self.state = "DRIVE"          # DRIVE -> INSPECT -> HOLD
        self.state_until = 0.0
        self.wp_started = time.time()
        self.pub = self.create_publisher(Twist, "/astrobot_0/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/gt/base_link_pose", self._gt, 10)
        self.create_timer(0.1, self.tick)    # 10 Hz command, like the controller
        self.create_timer(1.0, self.report)

    def _gt(self, msg):
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw_from_quat(msg.pose.orientation))
        now = time.time()
        self.hist.append((now, self.pose[0], self.pose[1]))
        while self.hist and now - self.hist[0][0] > 3.0:
            self.hist.pop(0)

    def gt_speed(self):
        if len(self.hist) < 2:
            return None
        (t0, x0, y0), (t1, x1, y1) = self.hist[0], self.hist[-1]
        dt = max(t1 - t0, 1e-3)
        return math.hypot(x1 - x0, y1 - y0) / dt

    def tick(self):
        t = Twist()
        now = time.time()
        if self.pose is None or self.state == "DONE":
            self.pub.publish(t)
            return
        if self.state == "HOLD":
            self.pub.publish(t)  # zeros -- hold for inspection
            return

        x, y, yaw = self.pose
        wx, wy = self.wps[self.idx]
        dist = math.hypot(wx - x, wy - y)
        if dist < self.a.wp_tol and self.state == "DRIVE":
            self.idx += 1
            self.wp_started = now
            if self.idx >= len(self.wps):
                self.get_logger().info("ROUTE COMPLETE (no terminal bog) -- stopping")
                self.state = "DONE"
                self.pub.publish(Twist())
                return
            wx, wy = self.wps[self.idx]

        bearing = math.atan2(wy - y, wx - x)
        err = ang_diff(bearing, yaw)
        if abs(err) > 0.6:
            t.angular.z = max(-self.a.wz, min(self.a.wz, 1.2 * err))
        else:
            t.linear.x = self.a.speed
            t.angular.z = max(-self.a.wz, min(self.a.wz, 1.0 * err))
        self.pub.publish(t)

        # Bog detection: commanding forward, not moving.
        spd = self.gt_speed()
        if self.state == "DRIVE" and t.linear.x > 0.08 and spd is not None and spd < self.a.bog_speed:
            if self.bog_since is None:
                self.bog_since = now
            elif now - self.bog_since > self.a.bog_sec:
                self.get_logger().warn(
                    f"\n{'='*64}\n>>> BOG REPRODUCED at world=({x:.2f},{y:.2f}) yaw={yaw:.2f}\n"
                    f">>> commanded {t.linear.x:.2f} m/s, GT speed {spd:.3f} m/s\n"
                    f">>> WATCH THE WHEELS/CONTACT NOW -- holding command {self.a.inspect_sec:.0f}s, then stopping\n{'='*64}"
                )
                self.state = "INSPECT"
                self.state_until = now + self.a.inspect_sec
        elif spd is not None and spd > self.a.bog_speed * 1.5:
            self.bog_since = None

        if self.state == "INSPECT" and now > self.state_until:
            self.get_logger().warn(">>> inspection window over -- HOLDING (zeros). Robot left in place. Ctrl-C when done.")
            self.state = "HOLD"

        # waypoint give-up (don't churn forever before the target zone)
        if self.state == "DRIVE" and now - self.wp_started > self.a.wp_timeout:
            self.get_logger().warn(f"waypoint {self.idx} ({wx:.1f},{wy:.1f}) timeout -- skipping")
            self.idx += 1
            self.wp_started = now
            if self.idx >= len(self.wps):
                self.state = "DONE"

    def report(self):
        if self.pose is None:
            self.get_logger().info("waiting for /gt/base_link_pose ...")
            return
        spd = self.gt_speed()
        wx, wy = self.wps[min(self.idx, len(self.wps) - 1)]
        self.get_logger().info(
            f"[{self.state}] pos=({self.pose[0]:.2f},{self.pose[1]:.2f}) -> wp{self.idx}({wx:.1f},{wy:.1f}) "
            f"gt_speed={spd if spd is None else round(spd,3)}"
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--waypoints", required=True, help='"x1,y1 x2,y2 ..." world frame')
    ap.add_argument("--speed", type=float, default=0.15)
    ap.add_argument("--wz", type=float, default=0.6)
    ap.add_argument("--wp-tol", type=float, default=0.45)
    ap.add_argument("--wp-timeout", type=float, default=90.0)
    ap.add_argument("--bog-speed", type=float, default=0.03)
    ap.add_argument("--bog-sec", type=float, default=6.0)
    ap.add_argument("--inspect-sec", type=float, default=25.0)
    args = ap.parse_args()
    rclpy.init()
    n = RouteDriver(args)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
