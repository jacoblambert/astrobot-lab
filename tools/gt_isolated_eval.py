#!/usr/bin/env python3
"""GT-pose-ISOLATED BEV rock-detection eval (drift-free).

The fixtures' SLAM drifts (~2 m), so SLAM-pose alignment of BEV->GT fails. Here we RE-ANCHOR each
aligned scan into the GT world frame using the GT pose: Delta(t) = GT_base(t) o SLAM_base(t)^-1, applied to
the map-frame points. The drift is removed per-scan, so the resulting BEV is in GT world coords and is
scored against /gt/map with IDENTITY alignment (zero registration error). This ISOLATES the ground-fit
algorithm (only use_gpf_ground varies). Reports per-cell recall/precision + recall bucketed by rock height.

Run INSIDE the dev container:
  python3 tools/gt_isolated_eval.py <bag> --gpf true|false [--label L]
"""
import sys, argparse, subprocess, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, '/workspace/astrobot-lab/tools')
from map_gt_alignment import imu_to_base, yaw_of

ap = argparse.ArgumentParser()
ap.add_argument('bag'); ap.add_argument('--gpf', default='true'); ap.add_argument('--label', default='')
ap.add_argument('--obstacle-thresh', type=int, default=88); ap.add_argument('--tolerance-m', type=float, default=0.3)
ap.add_argument('--obstacle-height', type=float, default=0.11)
ap.add_argument('--legacy-unsafe', action='store_true',
                help='Archived diagnostic only; receipt-time/yaw-only alignment is invalid for current gates')
A = ap.parse_args()
if not A.legacy_unsafe:
    ap.error('This archived evaluator uses bag receipt times, yaw-only re-anchoring, zero output stamps, '
             'and a base-link ray origin. It cannot score the repaired mapper. Use replay_surface_fixture.py '
             'with --gt-isolate for measurement-time full-SE(3) mapper replay; obstacle recall scoring remains separate. '
             '--legacy-unsafe is only for reproducing archived experiments, never for gate results.')
print('WARNING: LEGACY INVALID-FOR-GATES evaluation; output is not current-system accuracy evidence.', file=sys.stderr)
LABEL = A.label or f"gpf{A.gpf}_oh{A.obstacle_height}"

# ---- read the fixture ----
def yw(q): return yaw_of(q.x, q.y, q.z, q.w)
r = SequentialReader(); r.open(StorageOptions(uri=A.bag, storage_id='mcap'), ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}; cls = {n: get_message(t) for n, t in types.items()}
scans = []   # (t, Nx3 map-frame points)
slp = []     # (t, base x,y,z,yaw)  (SLAM, IMU->base converted)
gtp = []     # (t, base x,y,z,yaw)  (GT)
gtmap = None
CLOUD = '/astrobot_0/slam/aligned_points_corrected'
while r.has_next():
    tp, data, t = r.read_next(); ts = t * 1e-9
    if tp == CLOUD:
        m = deserialize_message(data, cls[tp])
        pts = np.array([(p[0], p[1], p[2]) for p in pc2.read_points(m, ('x', 'y', 'z'), skip_nans=True)], np.float64)
        scans.append((ts, pts))
    elif tp == '/astrobot_0/slam/pose_corrected':
        m = deserialize_message(data, cls[tp]); p = m.pose.position
        bx, by, byaw = imu_to_base(p.x, p.y, yw(m.pose.orientation)); slp.append((ts, bx, by, p.z, byaw))
    elif tp == '/gt/base_link_pose':
        m = deserialize_message(data, cls[tp]); p = m.pose.position; gtp.append((ts, p.x, p.y, p.z, yw(m.pose.orientation)))
    elif tp == '/gt/map':
        gtmap = deserialize_message(data, cls[tp])
print(f"[{LABEL}] scans={len(scans)} slam_poses={len(slp)} gt_poses={len(gtp)} gt_map={'ok' if gtmap else 'MISSING'}")
if not scans or not slp or not gtp or gtmap is None: sys.exit("missing topics")

def nearest(rows, ts):
    ti = min(range(len(rows)), key=lambda i: abs(rows[i][0] - ts)); return rows[ti]

# ---- re-anchor every scan into GT world frame; also build a per-cell height map (max_z - low ground) ----
GR = gtmap.info.resolution; GOX = gtmap.info.origin.position.x; GOY = gtmap.info.origin.position.y
G = np.array(gtmap.data, np.int16).reshape(gtmap.info.height, gtmap.info.width)
BRES = 0.10
reanchored = []   # (origin xyz, Nx3 gt-world points)
for ts, pts in scans:
    if len(pts) == 0: continue
    _, bx, by, bz, byaw = nearest(slp, ts); _, gx, gy, gz, gyaw = nearest(gtp, ts)
    d = gyaw - byaw; cs, sn = math.cos(d), math.sin(d)
    dx = pts[:, 0] - bx; dy = pts[:, 1] - by
    px = cs * dx - sn * dy + gx; py = sn * dx + cs * dy + gy; pz = pts[:, 2] - bz + gz
    reanchored.append(((gx, gy, gz), np.column_stack([px, py, pz])))
print(f"[{LABEL}] re-anchored {len(reanchored)} scans into GT frame")

# ---- drive the real C++ mapper on the re-anchored clouds ----
GPF = A.gpf
mapper = subprocess.Popen(
    ["ros2", "run", "slam_mapping", "probabilistic_voxel_mapper_node", "--ros-args",
     "-p", "use_sim_time:=false", "-p", "input_topic:=/eval/cloud", "-p", "pose_topic:=/eval/pose",
     "-p", "costmap_output_topic:=/eval/bev", "-p", "voxel_output_topic:=/eval/vox", "-p", "frame_id:=gt_map",
     "-p", "voxel_size:=0.05", "-p", "bev_resolution:=0.10",
     "-p", "width_m:=40.0", "-p", "height_m:=40.0", "-p", "origin_x:=0.0", "-p", "origin_y:=0.0",
     "-p", "hit_log_odds:=0.5", "-p", "miss_log_odds:=-0.20", "-p", "occupied_threshold:=1.0",
     "-p", "free_threshold:=-0.8", "-p", "max_range:=35.0", "-p", "point_stride:=3", "-p", "max_voxels:=6000000",
     "-p", "decay_factor:=0.999", "-p", "ground_window_radius:=2", "-p", "ground_percentile:=0.10",
     "-p", "low_z_percentile:=0.10", "-p", "high_z_percentile:=0.90", "-p", "use_plane_fit:=true",
     "-p", f"use_gpf_ground:={GPF}", "-p", "gpf_seed_fraction:=0.30", "-p", "gpf_inlier_band:=0.05",
     "-p", "gpf_iterations:=3", "-p", "use_ground_floor:=false", "-p", "min_occupied_voxels_per_cell:=3",
     "-p", f"obstacle_height:={A.obstacle_height}", "-p", "lethal_obstacle_height:=0.35",
     "-p", "obstacle_cost_min:=100", "-p", "obstacle_cost_max:=100",
     "-p", "terrain_risk_height:=0.055", "-p", "terrain_risk_cost_max:=14",
     "-p", "obstacle_inflation_radius:=0.0", "-p", "publish_voxel_map:=false", "-p", "publish_rate_hz:=5.0"],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
rclpy.init(); node = Node('gt_iso_eval')
cloud_pub = node.create_publisher(PointCloud2, '/eval/cloud', 10)
pose_pub = node.create_publisher(PoseStamped, '/eval/pose', 10)
bev = {'m': None}
node.create_subscription(OccupancyGrid, '/eval/bev', lambda m: bev.update(m=m), 10)
time.sleep(4.0)  # let the mapper come up
hdr_frame = 'gt_map'
for i, (origin, pg) in enumerate(reanchored):
    ps = PoseStamped(); ps.header.frame_id = hdr_frame
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(origin[0]), float(origin[1]), float(origin[2])
    ps.pose.orientation.w = 1.0
    pose_pub.publish(ps)
    cloud = pc2.create_cloud_xyz32(ps.header, pg.astype(np.float32).tolist())
    cloud.header.frame_id = hdr_frame
    cloud_pub.publish(cloud)
    rclpy.spin_once(node, timeout_sec=0.0)
    time.sleep(0.02)
# settle: let the mapper integrate + publish the final BEV
t0 = time.time()
while time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.2)
m = bev['m']
mapper.terminate()
if m is None:
    node.destroy_node(); rclpy.shutdown(); sys.exit("no BEV produced")

# ---- per-cell score (IDENTITY: BEV and GT both in GT world coords) ----
B = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
BOX = m.info.origin.position.x; BOY = m.info.origin.position.y; BRES = m.info.resolution
OBST = A.obstacle_thresh; TOL = A.tolerance_m
# GT rock cells -> world centers
grock = np.argwhere(G >= 99)
gw = np.column_stack([GOX + (grock[:, 1] + 0.5) * GR, GOY + (grock[:, 0] + 0.5) * GR])  # (x,y)
# build a per-rock-cell HEIGHT from the re-anchored cloud (max_z - 10th pct over a 0.5m window)
allpts = np.vstack([pg for _, pg in reanchored])
# coarse height grid at BRES over the gt-world extent
def cellkey(x, y): return (int((x - BOX) / BRES), int((y - BOY) / BRES))
cmax = {}; clow = {}
for x, y, z in allpts:
    k = cellkey(x, y)
    if k not in cmax or z > cmax[k]: cmax[k] = z
    clow.setdefault(k, []).append(z)
def cell_obstacle(x, y):
    bj = int((x - BOX) / BRES); bi = int((y - BOY) / BRES)
    tol = int(round(TOL / BRES))
    sub = B[max(0, bi - tol):bi + tol + 1, max(0, bj - tol):bj + tol + 1]
    return sub.size > 0 and (sub >= OBST).any()
def rock_near(x, y):
    gj = int((x - GOX) / GR); gi = int((y - GOY) / GR); tol = int(round(TOL / GR))
    sub = G[max(0, gi - tol):gi + tol + 1, max(0, gj - tol):gj + tol + 1]
    return sub.size > 0 and (sub >= 99).any()
# RECALL per GT rock cell (subsample to BRES to dedupe), bucketed by height
seen = set(); buckets = {'<0.15': [0, 0], '0.15-0.35': [0, 0], '>0.35': [0, 0], 'all': [0, 0]}
ground_all = np.percentile(allpts[:, 2], 10) if len(allpts) else 0.0
for x, y in gw:
    k = cellkey(x, y)
    if k in seen: continue
    seen.add(k)
    hit = cell_obstacle(x, y)
    # height of this rock cell: cell max_z minus a low local ground (global 10th pct is a robust floor here)
    h = (cmax.get(k, ground_all) - ground_all)
    b = '<0.15' if h < 0.15 else ('0.15-0.35' if h < 0.35 else '>0.35')
    for key in (b, 'all'):
        buckets[key][1] += 1; buckets[key][0] += int(hit)
# PRECISION over BEV obstacle cells
obs = np.argwhere(B >= OBST); tp = 0
for bi, bj in obs:
    wx = BOX + (bj + 0.5) * BRES; wy = BOY + (bi + 0.5) * BRES
    if rock_near(wx, wy): tp += 1
prec = tp / max(1, len(obs))
print(f"\n===== GT-ISOLATED SCORE [{LABEL}] (use_gpf_ground={GPF}) =====")
print(f"BEV obstacle cells={len(obs)}  precision(on GT rock)={prec:.3f}")
print(f"per-cell RECALL by rock height (recalled/total):")
for key in ('<0.15', '0.15-0.35', '>0.35', 'all'):
    rc, tt = buckets[key]
    print(f"   {key:>10}: {rc}/{tt} = {rc/max(1,tt):.2f}")
node.destroy_node(); rclpy.shutdown()
