#!/usr/bin/env python3
"""Offline: did the BEV costmap ever mark the rock the robot got stuck on?

Reads the seed-42 divergence bag and determines whether the rock at the stall
location was (A) marked-then-erased or (B) never-marked in slam/bev_costmap.
"""
import json
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1] if len(sys.argv) > 1 else \
    "/workspace/astrobot-lab/rosbags/phase4_seed42_divergence_20260531_174810"

TYPES = {
    "/gt/base_link_pose": "geometry_msgs/msg/PoseStamped",
    "/astrobot_0/slam/pose_corrected": "geometry_msgs/msg/PoseStamped",
    "/astrobot_0/diagnostics": "std_msgs/msg/String",
    "/astrobot_0/slam/bev_costmap": "nav_msgs/msg/OccupancyGrid",
    "/gt/map": "nav_msgs/msg/OccupancyGrid",
}


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def open_reader(path):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return r


gt_poses, slam_poses, diag = [], [], []
bev = []  # (t, OccupancyGrid)
gt_map = None
msgcls = {t: get_message(c) for t, c in TYPES.items()}

reader = open_reader(BAG)
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic not in TYPES:
        continue
    m = deserialize_message(data, msgcls[topic])
    if topic == "/gt/base_link_pose":
        gt_poses.append((stamp_s(m.header), m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)))
    elif topic == "/astrobot_0/slam/pose_corrected":
        slam_poses.append((stamp_s(m.header), m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)))
    elif topic == "/astrobot_0/diagnostics":
        try:
            d = json.loads(m.data)
        except Exception:
            continue
        nav = d.get("nav", {})
        svg = nav.get("slam_vs_gt", {})
        mot = nav.get("motion", {})
        fb = nav.get("feedback", {})
        diag.append((
            float(d.get("stamp", 0.0)),
            svg.get("position_error_m"),
            mot.get("gt_delta_m"),
            fb.get("seconds_since_progress"),
            d.get("status", {}).get("state"),
        ))
    elif topic == "/astrobot_0/slam/bev_costmap":
        bev.append((stamp_s(m.header), m))
    elif topic == "/gt/map":
        gt_map = m

gt_poses.sort(); slam_poses.sort(); diag.sort(); bev.sort(key=lambda x: x[0])
print(f"counts: gt_pose={len(gt_poses)} slam_pose={len(slam_poses)} diag={len(diag)} bev={len(bev)} gt_map={'yes' if gt_map else 'no'}")
if not (gt_poses and slam_poses and diag and bev and gt_map):
    print("MISSING DATA — abort"); sys.exit(1)


def nearest(seq, t):
    return min(seq, key=lambda r: abs(r[0] - t))


def occ_sample(grid, wx, wy, radius):
    """Max cell value within radius (m) of world (wx,wy). Returns (maxval, any_observed)."""
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    w, h = grid.info.width, grid.info.height
    cx = int((wx - ox) / res); cy = int((wy - oy) / res)
    rc = max(0, int(math.ceil(radius / res)))
    best = -128; observed = False
    for dy in range(-rc, rc + 1):
        for dx in range(-rc, rc + 1):
            x, y = cx + dx, cy + dy
            if 0 <= x < w and 0 <= y < h:
                v = grid.data[y * w + x]
                if v >= 0:
                    observed = True
                best = max(best, v)
    return best, observed


# 1) stall time = first diag where err>2.0
t_stall = None
for t, err, gtd, nop, st in diag:
    if err is not None and err > 2.0:
        t_stall = t; break
if t_stall is None:
    t_stall = max(diag, key=lambda r: (r[3] or 0))[0]
gstall = nearest(gt_poses, t_stall)
print(f"\n=== STALL ===\nt_stall(sim)={t_stall:.2f}  GT stall pos=({gstall[1]:.2f},{gstall[2]:.2f})")

# 2) gt->map offset from a clean pre-divergence diag time (err small)
clean = [d for d in diag if d[1] is not None and d[1] < 0.6 and d[0] < t_stall]
t0 = clean[-1][0] if clean else gt_poses[len(gt_poses)//4][0]
g0, s0 = nearest(gt_poses, t0), nearest(slam_poses, t0)
offx, offy = s0[1] - g0[1], s0[2] - g0[2]
rock_map = (gstall[1] + offx, gstall[2] + offy)
print(f"clean t0={t0:.2f} err≈{[d[1] for d in clean[-1:]]}  gt->map offset=({offx:.3f},{offy:.3f})")
print(f"stall in map frame ≈ ({rock_map[0]:.2f},{rock_map[1]:.2f})")

# 3) GT rock near stall — find the nearest occupied GT cell (the actual obstacle)
res = gt_map.info.resolution
ox, oy = gt_map.info.origin.position.x, gt_map.info.origin.position.y
w, h = gt_map.info.width, gt_map.info.height
nearest_rock = None; rock_cells = 0; rock_gt = None
cx = int((gstall[1]-ox)/res); cy = int((gstall[2]-oy)/res)
rc = int(math.ceil(2.5/res))
for dy in range(-rc, rc+1):
    for dx in range(-rc, rc+1):
        x, y = cx+dx, cy+dy
        if 0 <= x < w and 0 <= y < h and gt_map.data[y*w+x] >= 50:
            rock_cells += 1
            d = math.hypot(dx*res, dy*res)
            if nearest_rock is None or d < nearest_rock:
                nearest_rock = d
                rock_gt = (ox + (x + 0.5) * res, oy + (y + 0.5) * res)
print(f"\n=== GT MAP near stall (radius 2.5m) ===\noccupied(>=50) cells={rock_cells}  nearest_rock_dist={nearest_rock if nearest_rock is not None else 'NONE'}")
print(f"nearest GT rock world pos={rock_gt}")
# use earliest CLEAN offset (more accurate than near-stall) for mapping the rock
clean_early = [d for d in diag if d[1] is not None and d[1] < 0.4]
te = clean_early[0][0] if clean_early else t0
ge, se = nearest(gt_poses, te), nearest(slam_poses, te)
offx_e, offy_e = se[1] - ge[1], se[2] - ge[2]
print(f"early-clean offset @t={te:.1f} (err≈{clean_early[0][1] if clean_early else '?'}): ({offx_e:.3f},{offy_e:.3f})")
if rock_gt is not None:
    rock_map = (rock_gt[0] + offx_e, rock_gt[1] + offy_e)
    print(f"==> SAMPLING BEV AT THE ROCK, map frame ≈ ({rock_map[0]:.2f},{rock_map[1]:.2f})")

# 4) BEV cost at rock_map over time (approach window, map frame valid pre/at divergence)
print("\n=== BEV cost at stall location over time (radius 0.5m) ===")
print(f"{'t_sim':>9} {'gt_dist_to_stall':>16} {'bev_max_cost':>12} {'observed':>9}")
ever_marked = -128
for t, grid in bev:
    if t > t_stall + 6:
        break
    g = nearest(gt_poses, t)
    gt_dist = math.hypot(g[1]-gstall[1], g[2]-gstall[2])
    cost, obs = occ_sample(grid, rock_map[0], rock_map[1], 0.75)
    ever_marked = max(ever_marked, cost)
    print(f"{t:9.2f} {gt_dist:16.2f} {cost:12d} {str(obs):>9}")

print("\n=== VERDICT ===")
print(f"max BEV cost ever seen at stall cell (approach window): {ever_marked}")
if ever_marked >= 50:
    print("=> rock WAS marked at some point then lost  ==> CASE A (marked-then-erased)")
elif ever_marked > 0:
    print(f"=> rock only ever reached low/partial cost ({ever_marked})  ==> weak detection")
else:
    print("=> rock NEVER reached obstacle cost in BEV  ==> CASE B (never-marked)")
