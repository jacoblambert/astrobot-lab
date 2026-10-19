#!/usr/bin/env python3
"""Measure actual rock heights the LiDAR sees + per-cell ground roughness (the obstacle_height-gate
false-positive driver). The gate marks a 0.1m BEV cell obstacle if (hi-lo) of its points >= obstacle_height.
So: rock cells should have large vertical spread; the FP risk is STEEP TERRAIN cells (slope*cellsize)."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n = Node('rock_height_probe')
pts = []
def cb(m):
    global pts
    pts = [(p[0], p[1], p[2]) for p in pc2.read_points(m, ('x','y','z'), skip_nans=True)]
n.create_subscription(PointCloud2, '/astrobot_0/lidar_0/pointcloud/raw', cb, qos_profile_sensor_data)
import time
t0 = time.time()
while time.time()-t0 < 6 and not pts:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"raw cloud: {len(pts)} pts (lidar frame; lidar ~0.25m above base)")
if pts:
    zs = sorted(z for _,_,z in pts)
    ground = zs[int(0.10*len(zs))]   # robust ground estimate = 10th percentile z
    print(f"estimated ground z = {ground:.2f} (lidar frame)")
    CELL = 0.10  # match BEV resolution
    cells = {}   # (cx,cy) -> list of z
    for (x,y,z) in pts:
        d = math.hypot(x,y)
        if d > 8.0: continue
        key = (round(x/CELL), round(y/CELL))
        cells.setdefault(key, []).append(z)
    # per-cell vertical spread (what the gate sees) and height above ground
    rock_cells = []   # (dist, height_above_ground, spread)
    ground_spreads = []
    for (cx,cy), czs in cells.items():
        if len(czs) < 2: continue
        spread = max(czs) - min(czs)
        h = max(czs) - ground
        d = math.hypot(cx*CELL, cy*CELL)
        if h >= 0.11:                 # cell rises >=gate above ground => candidate rock
            rock_cells.append((d, h, spread))
        elif h < 0.05:                # near-ground cell => measures terrain roughness
            ground_spreads.append(spread)
    rock_cells.sort()
    print(f"\nROCK cells (>=0.11m above ground), within 8m: {len(rock_cells)}")
    if rock_cells:
        hs = sorted(h for _,h,_ in rock_cells)
        print(f"  rock-cell HEIGHT above ground: min={hs[0]:.2f} median={hs[len(hs)//2]:.2f} max={hs[-1]:.2f}")
        ss = sorted(s for _,_,s in rock_cells)
        print(f"  rock-cell SPREAD (gate sees this): min={ss[0]:.2f} median={ss[len(ss)//2]:.2f} max={ss[-1]:.2f}")
        nd, nh, nsp = rock_cells[0]
        print(f"  NEAREST rock cell: dist={nd:.2f}m height={nh:.2f}m spread={nsp:.2f}m")
        print(f"  rock-cell height histogram (0.1m bins): ", end="")
        bins={}
        for h in hs: b=round(h,1); bins[b]=bins.get(b,0)+1
        print({k:bins[k] for k in sorted(bins)})
    if ground_spreads:
        gs = sorted(ground_spreads)
        print(f"\nGROUND cells (<0.05m above ground): {len(gs)}  per-cell SPREAD (FP risk vs gate 0.11):")
        print(f"  min={gs[0]:.3f} median={gs[len(gs)//2]:.3f} p90={gs[int(0.9*len(gs))]:.3f} max={gs[-1]:.3f}")
        for thr in (0.06, 0.08, 0.10):
            frac = sum(1 for s in gs if s >= thr)/len(gs)
            print(f"  ground cells with spread >= {thr:.2f} (would FALSE-mark if gate={thr}): {frac*100:.1f}%")
n.destroy_node(); rclpy.shutdown()
