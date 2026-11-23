#!/usr/bin/env python3
"""Live stall probe: is the rock the rover is wedged on DETECTED in the BEV, or invisible to the LiDAR?
Reads map->base_link TF (rover pose+heading in map frame), the BEV costmap, and the filtered pointcloud;
samples BEV cost + counts LiDAR points in a forward cone (the direction the rover is being commanded)."""
import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data

class Probe(Node):
    def __init__(self):
        super().__init__('stall_probe')
        self.bev = None
        self.cloud_pts = []
        self.create_subscription(OccupancyGrid, '/astrobot_0/slam/bev_costmap', self._bev, 1)
        self.create_subscription(PointCloud2, '/astrobot_0/lidar_0/pointcloud/filtered', self._cloud, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/astrobot_0/lidar_0/pointcloud/raw', self._cloud_raw, qos_profile_sensor_data)
        self.cloud_raw_pts = []
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self)
    def _bev(self, m): self.bev = m
    def _cloud(self, m):
        try: self.cloud_pts = [(p[0], p[1], p[2]) for p in pc2.read_points(m, ('x','y','z'), skip_nans=True)]
        except Exception: pass
    def _cloud_raw(self, m):
        try: self.cloud_raw_pts = [(p[0], p[1], p[2]) for p in pc2.read_points(m, ('x','y','z'), skip_nans=True)]
        except Exception: pass

rclpy.init()
p = Probe()
t0 = time.time()
while time.time() - t0 < 6.0 and (p.bev is None or (not p.cloud_pts and not p.cloud_raw_pts)):
    rclpy.spin_once(p, timeout_sec=0.2)

# rover pose+heading in map frame
yaw = rx = ry = None
try:
    tf = p.tfbuf.lookup_transform('map', 'base_link', rclpy.time.Time())
    rx, ry = tf.transform.translation.x, tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    print(f"rover map-frame pose: x={rx:.2f} y={ry:.2f} yaw={math.degrees(yaw):.0f}deg")
except Exception as e:
    print(f"TF map->base_link failed: {e}")

# BEV cost sampled forward along heading (0.3..1.5m)
if p.bev is not None and yaw is not None:
    g = p.bev.info; import numpy as np
    arr = np.array(p.bev.data, dtype=np.int16).reshape(g.height, g.width)
    print(f"BEV: {g.width}x{g.height} res={g.resolution:.2f} origin=({g.origin.position.x:.1f},{g.origin.position.y:.1f})")
    print("forward BEV cost (dist: cost) -- 100=lethal rock, -1=unknown, 0=free:")
    line = []
    for d in [0.3,0.5,0.7,0.9,1.1,1.3,1.5]:
        wx, wy = rx+d*math.cos(yaw), ry+d*math.sin(yaw)
        c = int((wx-g.origin.position.x)/g.resolution); r = int((wy-g.origin.position.y)/g.resolution)
        v = int(arr[r,c]) if (0<=r<g.height and 0<=c<g.width) else 'OOB'
        line.append(f"{d}m:{v}")
    print("   " + "  ".join(line))
else:
    print("BEV or yaw missing")

# LiDAR points in a forward cone (within 2.0m, +-40deg of heading). Check both filtered and raw:
# raw sees rock but filtered/BEV doesn't -> preprocessor crops it; neither sees it -> geometric blind cone.
def forward_cone(pts):
    fwd = 0; closest = 99.0
    for (x,y,z) in pts:
        d = math.hypot(x,y); ang = math.atan2(y, x)
        if d < 2.0 and abs(ang) < math.radians(40):
            fwd += 1; closest = min(closest, d)
    return fwd, closest
for name, pts in (("filtered", p.cloud_pts), ("raw", p.cloud_raw_pts)):
    if pts:
        fwd, closest = forward_cone(pts)
        print(f"LiDAR {name}: {len(pts)} pts; {fwd} in forward cone(<2m,+-40deg)"
              + (f", closest={closest:.2f}m" if fwd else " -> ZERO ahead = blind"))
    else:
        print(f"LiDAR {name}: no cloud received")
# height analysis of forward-cone points (lidar frame; lidar mounted ~0.25m above base, so ground z~-0.25)
if p.cloud_raw_pts:
    fz = [z for (x,y,z) in p.cloud_raw_pts if math.hypot(x,y) < 1.5 and abs(math.atan2(y,x)) < math.radians(40)]
    if fz:
        fz.sort()
        zmin, zmax = fz[0], fz[-1]
        print(f"forward-cone(<1.5m) point z: n={len(fz)} min={zmin:.2f} max={zmax:.2f} spread={zmax-zmin:.2f}m "
              f"(obstacle_height gate=0.11m; ground~-0.25). >0.11 spread above ground => SHOULD mark obstacle")
        bands = {}
        for z in fz:
            b = round(z, 1); bands[b] = bands.get(b, 0) + 1
        print("   z histogram (0.1m bins):", {k: bands[k] for k in sorted(bands)})
p.destroy_node(); rclpy.shutdown() if rclpy.ok() else None
