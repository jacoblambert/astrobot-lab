#!/usr/bin/env python3
"""Audit near-spawn rocks: find rocks in the LiDAR cloud and check which are MISSING from the GT rock map
(/gt/map) and/or the BEV costmap. Tests the user's observation that GT itself omits a small rock near spawn
(which would bias every GT-based eval). Run inside dev while the stack is live."""
import sys, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/workspace/astrobot-lab/tools')
from map_gt_alignment import imu_to_base, yaw_of

rclpy.init(); n = Node('spawn_rock_audit')
S = {'bev': None, 'gt': None, 'slam': None, 'gtp': None}
pts = []
n.create_subscription(OccupancyGrid, '/astrobot_0/slam/bev_costmap', lambda m: S.update(bev=m), 1)
n.create_subscription(OccupancyGrid, '/gt/map', lambda m: S.update(gt=m), 1)
n.create_subscription(PoseStamped, '/astrobot_0/slam/pose_corrected', lambda m: S.update(slam=m), 10)
n.create_subscription(PoseStamped, '/gt/base_link_pose', lambda m: S.update(gtp=m), 10)
def cb(m):
    global pts
    pts = [(p[0], p[1], p[2]) for p in pc2.read_points(m, ('x', 'y', 'z'), skip_nans=True)]
n.create_subscription(PointCloud2, '/astrobot_0/slam/aligned_points_corrected', cb, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 8 and (S['bev'] is None or S['gt'] is None or S['slam'] is None or S['gtp'] is None or not pts):
    rclpy.spin_once(n, timeout_sec=0.2)
if S['bev'] is None or S['gt'] is None or S['slam'] is None or S['gtp'] is None or not pts:
    print("missing:", [k for k in ('bev', 'gt', 'slam', 'gtp') if S[k] is None], "cloud", len(pts)); sys.exit(1)
sp = S['slam'].pose.position; so = S['slam'].pose.orientation
sbx, sby, syaw = imu_to_base(sp.x, sp.y, yaw_of(so.x, so.y, so.z, so.w))
gp = S['gtp'].pose.position; go = S['gtp'].pose.orientation; gyaw = yaw_of(go.x, go.y, go.z, go.w)
dyaw = gyaw - syaw; c, s = math.cos(dyaw), math.sin(dyaw)
tx = gp.x - (c * sbx - s * sby); ty = gp.y - (s * sbx + c * sby)
P = np.array(pts)  # map frame
# per-cell (0.1m) max-z + low ground (global low pctl as floor) within 5m of rover
CELL = 0.10; near = P[np.hypot(P[:, 0] - sbx, P[:, 1] - sby) < 5.0]
ground = np.percentile(near[:, 2], 10) if len(near) else 0.0
cmax = {}
for x, y, z in near:
    k = (round(x / CELL), round(y / CELL))
    if k not in cmax or z > cmax[k]: cmax[k] = z
B = np.array(S['bev'].data, np.int16).reshape(S['bev'].info.height, S['bev'].info.width); bi = S['bev'].info
G = np.array(S['gt'].data, np.int16).reshape(S['gt'].info.height, S['gt'].info.width); gi = S['gt'].info
def in_bev(mx, my):
    bj = int((mx - bi.origin.position.x) / bi.resolution); br = int((my - bi.origin.position.y) / bi.resolution)
    t = 3
    return 0 <= br < B.shape[0] and 0 <= bj < B.shape[1] and (B[max(0, br - t):br + t + 1, max(0, bj - t):bj + t + 1] >= 88).any()
def in_gt(mx, my):
    wx = c * mx - s * my + tx; wy = s * mx + c * my + ty
    gj = int((wx - gi.origin.position.x) / gi.resolution); gr = int((wy - gi.origin.position.y) / gi.resolution)
    t = int(round(0.3 / gi.resolution))
    return 0 <= gr < G.shape[0] and 0 <= gj < G.shape[1] and (G[max(0, gr - t):gr + t + 1, max(0, gj - t):gj + t + 1] >= 99).any()
rock_cells = [(k[0] * CELL, k[1] * CELL, cmax[k] - ground) for k in cmax if (cmax[k] - ground) >= 0.12]
print(f"rover map=({sbx:.2f},{sby:.2f}) ground_z~{ground:.2f}; cloud rock-cells(>0.12m,<5m)={len(rock_cells)}")
nb = ng = neither = 0
neither_list = []
for mx, my, h in rock_cells:
    b = in_bev(mx, my); g = in_gt(mx, my)
    nb += b; ng += g
    if not b and not g:
        neither += 1; neither_list.append((mx, my, h))
print(f"  in BEV: {nb}/{len(rock_cells)}   in GT: {ng}/{len(rock_cells)}   in NEITHER: {neither}")
if neither_list:
    neither_list.sort(key=lambda r: -r[2])
    print("  tallest cloud-rock cells missing from BOTH GT and BEV (map x,y,height):")
    for mx, my, h in neither_list[:8]:
        print(f"    ({mx:.2f},{my:.2f}) h={h:.2f}m  dist_from_rover={math.hypot(mx-sbx,my-sby):.2f}m")
n.destroy_node(); rclpy.shutdown()
