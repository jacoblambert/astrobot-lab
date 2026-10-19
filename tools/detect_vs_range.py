#!/usr/bin/env python3
"""Detection-rate-vs-range: cross-check LiDAR-derived rock cells against what the BEV actually marks.
Distinguishes 'rock marked at range then lost up close' (clearing / persist-radius) from 'never marked'
(ground-reference rides up on wide rocks / seed gate). Uses the map-frame corrected cloud the mapper
consumes, so rock cells are directly comparable to the BEV grid."""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n = Node('detect_vs_range')
state = {'cloud': None, 'bev': None, 'odom': None}
n.create_subscription(PointCloud2, '/astrobot_0/slam/aligned_points_corrected',
                      lambda m: state.update(cloud=[(p[0],p[1],p[2]) for p in pc2.read_points(m,('x','y','z'),skip_nans=True)]),
                      qos_profile_sensor_data)
n.create_subscription(OccupancyGrid, '/astrobot_0/slam/bev_costmap', lambda m: state.update(bev=m), 1)
n.create_subscription(Odometry, '/astrobot_0/slam/odom_corrected', lambda m: state.update(odom=m), qos_profile_sensor_data)
t0 = time.time()
while time.time()-t0 < 8 and (state['cloud'] is None or state['bev'] is None or state['odom'] is None):
    rclpy.spin_once(n, timeout_sec=0.2)
if not all(state.values()):
    print("missing:", [k for k,v in state.items() if v is None]); n.destroy_node(); rclpy.shutdown(); raise SystemExit
pts = state['cloud']; bev = state['bev']; od = state['odom'].pose.pose.position
print(f"map-frame cloud: {len(pts)} pts; rover map pos=({od.x:.2f},{od.y:.2f})")
CELL = 0.10
# per-cell max/min z in map frame
cmax, cmin = {}, {}
for (x,y,z) in pts:
    k = (round(x/CELL), round(y/CELL))
    if k not in cmax or z > cmax[k]: cmax[k] = z
    if k not in cmin or z < cmin[k]: cmin[k] = z
# robust local ground = min cmin over a 1.0m window (radius 5 cells) -- immune to a rock filling a small window
def robust_ground(k, R=5):
    best = None
    for dx in range(-R, R+1):
        for dy in range(-R, R+1):
            v = cmin.get((k[0]+dx, k[1]+dy))
            if v is not None and (best is None or v < best): best = v
    return best
g = bev.info
barr = np.array(bev.data, dtype=np.int16).reshape(g.height, g.width)
def bev_cost(x, y):
    c = int((x-g.origin.position.x)/g.resolution); r = int((y-g.origin.position.y)/g.resolution)
    return int(barr[r,c]) if (0<=r<g.height and 0<=c<g.width) else None
# bin rock cells by distance from rover; detected if BEV cost >= 88 (obstacle_cost_min)
bins = {(0,1):[], (1,2):[], (2,3):[], (3,5):[], (5,8):[]}
for k, mx in cmax.items():
    gr = robust_ground(k)
    if gr is None: continue
    h = mx - gr
    if h < 0.15: continue                 # not a rock cell
    wx, wy = k[0]*CELL, k[1]*CELL
    d = math.hypot(wx-od.x, wy-od.y)
    cost = bev_cost(wx, wy)
    if cost is None: continue
    for (lo,hi) in bins:
        if lo <= d < hi:
            bins[(lo,hi)].append((h, cost)); break
print(f"\nrock cells (>=0.15m above robust local ground) cross-checked vs BEV cost (detected = cost>=88):")
print(f"{'range':>8} {'n':>5} {'detected%':>10} {'median_h':>9} {'median_cost':>12}")
for (lo,hi), vals in bins.items():
    if not vals:
        print(f"{lo}-{hi}m{'':>3} {0:>5}"); continue
    hs = sorted(v[0] for v in vals); cs = sorted(v[1] for v in vals)
    det = sum(1 for _,c in vals if c >= 88)/len(vals)*100
    print(f"{lo}-{hi}m{'':>3} {len(vals):>5} {det:>9.0f}% {hs[len(hs)//2]:>9.2f} {cs[len(cs)//2]:>12}")

# PRECISION: of BEV obstacle cells (cost>=88) within 8m of rover, how many have a real LiDAR rock nearby?
def has_rock_near(wx, wy, R=1):
    k = (round(wx/CELL), round(wy/CELL))
    for dx in range(-R, R+1):
        for dy in range(-R, R+1):
            kk = (k[0]+dx, k[1]+dy)
            if kk in cmax:
                gr = robust_ground(kk)
                if gr is not None and (cmax[kk]-gr) >= 0.15:
                    return True
    return False
tp = fp = 0
for r in range(g.height):
    for c in range(g.width):
        if barr[r,c] < 88: continue
        wx = g.origin.position.x + (c+0.5)*g.resolution; wy = g.origin.position.y + (r+0.5)*g.resolution
        if math.hypot(wx-od.x, wy-od.y) > 8.0: continue
        if has_rock_near(wx, wy): tp += 1
        else: fp += 1
if tp+fp:
    print(f"\nPRECISION: {tp} BEV-obstacle cells w/ real rock, {fp} without -> precision={tp/(tp+fp)*100:.0f}% "
          f"(FP cells may be slopes/over-marking; r=2 baseline was ~0.995)")
n.destroy_node(); rclpy.shutdown()
