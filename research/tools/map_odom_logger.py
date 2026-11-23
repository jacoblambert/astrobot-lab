#!/usr/bin/env python3
"""Log the map->odom correction magnitude over (sim) time -- the GLIM watchdog
signal. A bounded map->odom = stable SLAM; a step jump / >2m = the watchdog-trip
divergence. Frame-clean (no GT alignment needed): it IS GLIM's own correction.

Writes CSV "sim_t,dx,dy,yaw,mag" (one row per tick) and prints the running max.
Usage: map_odom_logger.py --output map_odom.csv [--map map] [--odom odom] [--hz 5]
Tries a few common (namespaced) frame-name pairs if the given one isn't found.
"""
import argparse, math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


class MapOdomLogger(Node):
    def __init__(self, args):
        super().__init__("map_odom_logger")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.f = open(args.output, "w", buffering=1)
        self.f.write("sim_t,dx,dy,yaw,mag\n")
        self.maxmag = 0.0
        # candidate (parent, child) pairs to try
        ns = args.namespace.strip("/")
        self.cands = []
        for m, o in [(args.map, args.odom)]:
            self.cands.append((m, o))
            if ns:
                self.cands.append((f"{ns}/{m}", f"{ns}/{o}"))
        self.locked = None
        self.timer = self.create_timer(1.0 / args.hz, self.tick)

    def tick(self):
        pairs = [self.locked] if self.locked else self.cands
        for parent, child in pairs:
            try:
                tr = self.buf.lookup_transform(parent, child, Time())
            except Exception:
                continue
            self.locked = (parent, child)
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            x = tr.transform.translation.x
            y = tr.transform.translation.y
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            mag = math.hypot(x, y)
            self.maxmag = max(self.maxmag, mag)
            self.f.write(f"{t:.3f},{x:.4f},{y:.4f},{yaw:.4f},{mag:.4f}\n")
            if mag > 1.0 and mag >= self.maxmag - 1e-6:
                self.get_logger().warn(f"map->odod mag={mag:.2f} (max={self.maxmag:.2f})")
            return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    ap.add_argument("--map", default="map")
    ap.add_argument("--odom", default="odom")
    ap.add_argument("--namespace", default="astrobot_0")
    ap.add_argument("--hz", type=float, default=5.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = MapOdomLogger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.flush()
        node.f.close()
        node.get_logger().info(f"map->odom max mag = {node.maxmag:.3f} m")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
