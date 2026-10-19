#!/bin/bash
# Replay the rock fixture through the BEV mapper and report obstacle cells in the
# SPAWN region (world 3x3 m box around 20,18) vs the wedge rock vs global. Tests
# whether a given obstacle_height + min_occupied_voxels_per_cell rings the spawn in
# with spurious near-field (LiDAR blind-cone) obstacles -> exploration manager
# no_candidates. Run INSIDE the dev container.
# Usage: _spawn_ring_probe.sh <bag> <obstacle_height> <min_voxels>
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
  > /tmp/ring_mapper.log 2>&1 &
MAP=$!
sleep 4
ros2 bag play "$BAG" --clock --rate 3.0 \
  --topics /astrobot_0/slam/aligned_points_corrected /astrobot_0/slam/pose_corrected \
  > /tmp/ring_play.log 2>&1
sleep 3
python3 - "$OH" "$MINV" <<'PY'
import rclpy, time, sys, numpy as np
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = rclpy.create_node("ring"); got = {}
n.create_subscription(OccupancyGrid, "/sweep/bev_costmap", lambda m: got.update(m=m), 1)
t = time.time()
while time.time()-t < 8 and "m" not in got: rclpy.spin_once(n, timeout_sec=0.2)
if "m" not in got: print("NO GRID"); sys.exit(1)
m = got["m"]; res = m.info.resolution; ox, oy = m.info.origin.position.x, m.info.origin.position.y
a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
FREE_MAX = 35  # exploration_manager free_cost_max
def box(wx0,wx1,wy0,wy1):
    x0=int((wx0-20-ox)/res); x1=int((wx1-20-ox)/res); y0=int((wy0-18-oy)/res); y1=int((wy1-18-oy)/res)
    return a[y0:y1, x0:x1]
spawn = box(18.5,21.5,16.5,19.5)   # 3x3 m around spawn (robot start; blind-cone near-field)
rock  = box(13.0,14.4,12.65,14.05) # wedge rock
print(f"OH={sys.argv[1]} MINV={sys.argv[2]} | "
      f"SPAWN_box cells>FREE35={int((spawn>FREE_MAX).sum())} cost100={int((spawn>=100).sum())} (n={spawn.size}) | "
      f"ROCK cost100={int((rock>=100).sum())} | GLOBAL cost100={int((a>=100).sum())}")
rclpy.shutdown()
PY
kill -TERM $MAP 2>/dev/null || true
