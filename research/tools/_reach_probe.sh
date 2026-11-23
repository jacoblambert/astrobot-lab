#!/bin/bash
# Replay the fixture through the mapper at a given (obstacle_height, min_voxels), then
# REPLICATE the exploration_manager reachability flood-fill on the resulting BEV grid
# (known_free cost in [0,free_cost_max]; minus _near_unsafe = within goal_clearance_radius
# of a cell >= goal_clearance_cost_min; 4-conn flood from spawn cell). Reports reachable
# free-cell count + whether any unknown frontier is reachable. This is what decides
# explore_radius "no_candidates". Run INSIDE dev container.
# Usage: _reach_probe.sh <bag> <obstacle_height> <min_voxels>
set -e
source /etc/ros_setup.sh
source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
BAG="$1"; OH="$2"; MINV="$3"
pkill -f probabilistic_voxel_mapper_node 2>/dev/null || true
sleep 1
ros2 run slam_mapping probabilistic_voxel_mapper_node --ros-args \
  -p use_sim_time:=true \
  -p input_topic:=/astrobot_0/slam/aligned_points_corrected \
  -p pose_topic:=/astrobot_0/slam/pose_corrected \
  -p costmap_output_topic:=/sweep/bev_costmap -p voxel_output_topic:=/sweep/vox \
  -p frame_id:=map -p voxel_size:=0.05 -p bev_resolution:=0.10 \
  -p width_m:=40.0 -p height_m:=40.0 -p origin_x:=-20.0 -p origin_y:=-20.0 \
  -p hit_log_odds:=0.5 -p miss_log_odds:=-0.20 -p min_log_odds:=-3.0 -p max_log_odds:=4.0 \
  -p occupied_threshold:=1.0 -p free_threshold:=-0.8 -p max_range:=35.0 \
  -p point_stride:=3 -p ray_stride:=1 -p min_z:=-2.0 -p max_z:=4.0 \
  -p max_voxels:=6000000 -p decay_factor:=0.998 -p decay_min_abs_log_odds:=0.05 \
  -p ground_window_radius:=2 -p ground_percentile:=0.10 \
  -p low_z_percentile:=0.10 -p high_z_percentile:=0.90 \
  -p min_occupied_voxels_per_cell:=$MINV \
  -p obstacle_height:=$OH -p lethal_obstacle_height:=0.35 \
  -p obstacle_cost_min:=100 -p obstacle_cost_max:=100 \
  -p terrain_risk_height:=0.06 -p terrain_risk_cost_max:=12 \
  -p slope_cost_max:=28.0 -p slope_min:=0.12 -p slope_max:=0.50 -p slope_window_cells:=3 \
  -p obstacle_inflation_radius:=0.70 -p obstacle_inflation_cost:=28 \
  -p obstacle_inflation_min_cost:=8 -p obstacle_inflation_exponent:=1.0 \
  -p publish_voxel_map:=false -p publish_rate_hz:=2.0 \
  > /tmp/reach_mapper.log 2>&1 &
MAP=$!
sleep 4
ros2 bag play "$BAG" --clock --rate 3.0 \
  --topics /astrobot_0/slam/aligned_points_corrected /astrobot_0/slam/pose_corrected \
  > /tmp/reach_play.log 2>&1
sleep 3
python3 - "$OH" "$MINV" <<'PY'
import rclpy, time, sys
from collections import deque
import numpy as np
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = rclpy.create_node("reach"); got = {}
n.create_subscription(OccupancyGrid, "/sweep/bev_costmap", lambda m: got.update(m=m), 1)
t = time.time()
while time.time()-t < 8 and "m" not in got: rclpy.spin_once(n, timeout_sec=0.2)
if "m" not in got: print("NO GRID"); sys.exit(1)
m = got["m"]; res = m.info.resolution; ox, oy = m.info.origin.position.x, m.info.origin.position.y
g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
# phase4 params
FREE_MAX = 30; CLR_COST = 50; CLR_R = int(np.ceil(0.90/res))  # 9 cells
known_free = (g >= 0) & (g <= FREE_MAX)
# unsafe = within CLR_R (Chebyshev approx via iterative dilation) of cost>=CLR_COST
hard = g >= CLR_COST
unsafe = hard.copy()
for _ in range(CLR_R):
    sh = unsafe.copy()
    sh[1:,:] |= unsafe[:-1,:]; sh[:-1,:] |= unsafe[1:,:]
    sh[:,1:] |= unsafe[:,:-1]; sh[:,:-1] |= unsafe[:,1:]
    unsafe = sh
trav = known_free & ~unsafe
# spawn cell: world (20,18) -> map (0,0) -> grid index
sr = int((0 - oy)/res); sc = int((0 - ox)/res)
# nearest trav cell within 12 (mimic _nearest_safe_free_cell)
start = None
for rad in range(13):
    for dr in range(-rad, rad+1):
        for dc in range(-rad, rad+1):
            r,c = sr+dr, sc+dc
            if 0<=r<g.shape[0] and 0<=c<g.shape[1] and trav[r,c]:
                start=(r,c); break
        if start: break
    if start: break
reach = np.zeros_like(trav)
if start:
    q=deque([start]); reach[start]=True
    while q:
        r,c=q.popleft()
        for nr,nc in ((r+1,c),(r-1,c),(r,c+1),(r,c-1)):
            if 0<=nr<g.shape[0] and 0<=nc<g.shape[1] and trav[nr,nc] and not reach[nr,nc]:
                reach[nr,nc]=True; q.append((nr,nc))
# frontier reachable? unknown cell adjacent to a reachable cell
unknown = g < 0
fr = np.zeros_like(reach)
fr[1:,:]|=reach[:-1,:]; fr[:-1,:]|=reach[1:,:]; fr[:,1:]|=reach[:,:-1]; fr[:,:-1]|=reach[:,1:]
frontier = unknown & fr
print(f"OH={sys.argv[1]} MINV={sys.argv[2]} | obstacles(>=50)={int(hard.sum())} unsafe_cells={int(unsafe.sum())} "
      f"known_free={int(known_free.sum())} REACHABLE={int(reach.sum())} frontier_reachable={int(frontier.sum())} "
      f"start={'OK' if start else 'NONE'}")
rclpy.shutdown()
PY
kill -TERM $MAP 2>/dev/null || true
