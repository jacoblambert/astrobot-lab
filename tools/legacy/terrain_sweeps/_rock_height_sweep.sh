#!/bin/bash
# One arm of the small-rock detection sweep: replay a recorded approach bag through the
# BEV mapper with a chosen obstacle_height (+ optional obstacle_cost_min), then report
# cost stats at the wedge-rock box and global obstacle counts (false-positive check).
# Mirrors the CURRENT phase2_slam.launch.py mapper params (2026-06-11: soft inflation
# 0.70/28, slope cost 28) except the knobs under test. Run INSIDE the dev container.
# Usage: _rock_height_sweep.sh <bag> <obstacle_height> [obstacle_cost_min]
set -e
source /etc/ros_setup.sh
source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
BAG="$1"; OH="$2"; OCMIN="${3:-100}"

pkill -f probabilistic_voxel_mapper_node 2>/dev/null || true
sleep 1

ros2 run slam_mapping probabilistic_voxel_mapper_node --ros-args \
  -p use_sim_time:=true \
  -p input_topic:=/astrobot_0/slam/aligned_points_corrected \
  -p pose_topic:=/astrobot_0/slam/pose_corrected \
  -p costmap_output_topic:=/sweep/bev_costmap \
  -p voxel_output_topic:=/sweep/prob_voxel_map \
  -p frame_id:=map \
  -p voxel_size:=0.05 -p bev_resolution:=0.10 \
  -p width_m:=40.0 -p height_m:=40.0 -p origin_x:=-20.0 -p origin_y:=-20.0 \
  -p hit_log_odds:=0.5 -p miss_log_odds:=-0.20 -p min_log_odds:=-3.0 -p max_log_odds:=4.0 \
  -p occupied_threshold:=1.0 -p free_threshold:=-0.8 -p max_range:=35.0 \
  -p point_stride:=3 -p ray_stride:=1 -p min_z:=-2.0 -p max_z:=4.0 \
  -p max_voxels:=6000000 -p decay_factor:=0.998 -p decay_min_abs_log_odds:=0.05 \
  -p ground_window_radius:=2 -p ground_percentile:=0.10 \
  -p low_z_percentile:=0.10 -p high_z_percentile:=0.90 \
  -p min_occupied_voxels_per_cell:=1 \
  -p obstacle_height:=$OH -p lethal_obstacle_height:=0.35 \
  -p obstacle_cost_min:=$OCMIN -p obstacle_cost_max:=100 \
  -p terrain_risk_height:=0.06 -p terrain_risk_cost_max:=12 \
  -p slope_cost_max:=28.0 -p slope_min:=0.12 -p slope_max:=0.50 -p slope_window_cells:=3 \
  -p obstacle_inflation_radius:=0.70 -p obstacle_inflation_cost:=28 \
  -p obstacle_inflation_min_cost:=8 -p obstacle_inflation_exponent:=1.0 \
  -p publish_voxel_map:=false -p publish_rate_hz:=2.0 \
  > /tmp/sweep_mapper.log 2>&1 &
MAP=$!
sleep 4

ros2 bag play "$BAG" --clock --rate 3.0 \
  --topics /astrobot_0/slam/aligned_points_corrected /astrobot_0/slam/pose_corrected \
  > /tmp/sweep_play.log 2>&1
sleep 3

python3 - "$OH" "$OCMIN" <<'PY'
import rclpy, time, sys, numpy as np
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = rclpy.create_node("sweep_probe"); got = {}
n.create_subscription(OccupancyGrid, "/sweep/bev_costmap", lambda m: got.update(m=m), 1)
t = time.time()
while time.time() - t < 8 and "m" not in got:
    rclpy.spin_once(n, timeout_sec=0.2)
if "m" not in got:
    print("NO GRID"); sys.exit(1)
m = got["m"]; res = m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
def box(wx0, wx1, wy0, wy1):  # world -> map = world - (20,18)
    x0 = int((wx0-20-ox)/res); x1 = int((wx1-20-ox)/res)
    y0 = int((wy0-18-oy)/res); y1 = int((wy1-18-oy)/res)
    return a[y0:y1, x0:x1]
rock = box(13.0, 14.4, 12.65, 14.05)   # wedge-rock complex
known = a[a >= 0]
print(f"OH={sys.argv[1]} OCMIN={sys.argv[2]} | rock: max={rock.max()} cells>=80={(rock>=80).sum()} cells>=50={(rock>=50).sum()} "
      f"| GLOBAL known={known.size} cells>=80={(a>=80).sum()} cells>=50={(a>=50).sum()} mean_known={known.mean():.1f}")
rclpy.shutdown()
PY
kill -TERM $MAP 2>/dev/null || true
