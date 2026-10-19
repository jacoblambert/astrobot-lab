#!/usr/bin/env python3
"""Diagnose the seed-411 spawn trap: are the BEV obstacle cells near spawn REAL rocks (on the GT rock map)
or FALSE ground/mound-as-rock? Aligns BEV(map) -> GT(gt_map) from the current SLAM/GT pose pair (early =
low drift near spawn) and classifies near-rover BEV obstacle cells against /gt/map. Run inside dev."""
import sys, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
sys.path.insert(0, '/workspace/astrobot-lab/tools')
from map_gt_alignment import imu_to_base, yaw_of

rclpy.init(); n = Node('diag_spawn')
S = {'bev': None, 'gt': None, 'slam': None, 'gtp': None, 'status': None}
n.create_subscription(OccupancyGrid, '/astrobot_0/slam/bev_costmap', lambda m: S.update(bev=m), 1)
n.create_subscription(OccupancyGrid, '/gt/map', lambda m: S.update(gt=m), 1)
n.create_subscription(PoseStamped, '/astrobot_0/slam/pose_corrected', lambda m: S.update(slam=m), 10)
n.create_subscription(PoseStamped, '/gt/base_link_pose', lambda m: S.update(gtp=m), 10)
n.create_subscription(String, '/astrobot_0/exploration/status', lambda m: S.update(status=m), 1)
t0 = time.time()
while time.time() - t0 < 8 and not all(v is not None for k, v in S.items() if k != 'status'):
    rclpy.spin_once(n, timeout_sec=0.2)
if not all(v is not None for k, v in S.items() if k != 'status'):
    print("missing:", [k for k, v in S.items() if v is None]); sys.exit(1)
# alignment map->gt from the current pose pair (slam is IMU frame -> base)
sp = S['slam'].pose.position; sbx, sby, syaw = imu_to_base(sp.x, sp.y, yaw_of(S['slam'].pose.orientation.x, S['slam'].pose.orientation.y, S['slam'].pose.orientation.z, S['slam'].pose.orientation.w))
gp = S['gtp'].pose.position; gyaw = yaw_of(S['gtp'].pose.orientation.x, S['gtp'].pose.orientation.y, S['gtp'].pose.orientation.z, S['gtp'].pose.orientation.w)
dyaw = gyaw - syaw; c, s = math.cos(dyaw), math.sin(dyaw)
tx = gp.x - (c * sbx - s * sby); ty = gp.y - (s * sbx + c * sby)
print(f"rover map=({sbx:.2f},{sby:.2f}) gt=({gp.x:.2f},{gp.y:.2f}) align dyaw={math.degrees(dyaw):.1f}deg t=({tx:.2f},{ty:.2f})")
B = np.array(S['bev'].data, np.int16).reshape(S['bev'].info.height, S['bev'].info.width); bi = S['bev'].info
G = np.array(S['gt'].data, np.int16).reshape(S['gt'].info.height, S['gt'].info.width); gi = S['gt'].info
def map_to_gt(mx, my): return (c * mx - s * my + tx, s * mx + c * my + ty)
tol = max(1, int(round(0.3 / gi.resolution)))
obs = np.argwhere(B >= 88)
real = false = nearspawn = 0
for r, cc in obs:
    mx = bi.origin.position.x + (cc + 0.5) * bi.resolution; my = bi.origin.position.y + (r + 0.5) * bi.resolution
    if math.hypot(mx - sbx, my - sby) > 8.0: continue
    nearspawn += 1
    wx, wy = map_to_gt(mx, my)
    gj = int((wx - gi.origin.position.x) / gi.resolution); gr = int((wy - gi.origin.position.y) / gi.resolution)
    if 0 <= gr < G.shape[0] and 0 <= gj < G.shape[1] and (G[max(0, gr - tol):gr + tol + 1, max(0, gj - tol):gj + tol + 1] >= 99).any():
        real += 1
    else:
        false += 1
print(f"near-spawn (<8m) BEV obstacle cells: {nearspawn}  REAL(on GT rock)={real}  FALSE(ground/mound)={false}  -> precision={real/max(1,nearspawn):.2f}")
if S['status'] is not None:
    import json
    try:
        d = json.loads(S['status'].data)
        print("status:", {k: d.get(k) for k in ['reachable_cells', 'mission_reachable_cells', 'mission_coverage_fraction', 'goals', 'state']})
    except Exception as e:
        print("status parse failed:", e)
n.destroy_node(); rclpy.shutdown()
