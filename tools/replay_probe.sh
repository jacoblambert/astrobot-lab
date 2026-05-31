#!/bin/bash
# Replay clouds through the mapper with chosen log-odds; a live probe logs BEV cost
# at the rock location to CSV (flushed per msg, no bag finalization needed).
# Usage: replay_probe.sh ORIG CSV HIT MISS MAXLO MINOCC
set -e
source /etc/ros_setup.sh
source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
ORIG="$1"; CSV="$2"; HIT="$3"; MISS="$4"; MAXLO="$5"; MINOCC="$6"; DECAY="${7:-1.0}"

ros2 run slam_mapping probabilistic_voxel_mapper_node --ros-args \
  -p use_sim_time:=true \
  -p input_topic:=/astrobot_0/slam/aligned_points_corrected \
  -p pose_topic:=/astrobot_0/slam/pose_corrected \
  -p costmap_output_topic:=/astrobot_0/slam/bev_costmap \
  -p voxel_output_topic:=/astrobot_0/slam/prob_voxel_map -p frame_id:=map \
  -p voxel_size:=0.05 -p bev_resolution:=0.10 \
  -p width_m:=40.0 -p height_m:=40.0 -p origin_x:=-20.0 -p origin_y:=-20.0 \
  -p hit_log_odds:=$HIT -p miss_log_odds:=$MISS -p min_log_odds:=-3.0 -p max_log_odds:=$MAXLO \
  -p occupied_threshold:=1.0 -p free_threshold:=-0.8 -p max_range:=35.0 \
  -p point_stride:=3 -p ray_stride:=1 \
  -p use_bounds:=true -p min_x:=-25.0 -p max_x:=25.0 -p min_y:=-25.0 -p max_y:=25.0 -p min_z:=-2.0 -p max_z:=4.0 \
  -p max_voxels:=4000000 -p decay_factor:=$DECAY -p decay_min_abs_log_odds:=0.05 \
  -p ground_window_radius:=2 -p ground_percentile:=0.10 -p low_z_percentile:=0.10 -p high_z_percentile:=0.90 \
  -p min_occupied_voxels_per_cell:=$MINOCC -p obstacle_height:=0.12 -p lethal_obstacle_height:=0.35 \
  -p obstacle_cost_min:=88 -p obstacle_cost_max:=100 -p terrain_risk_height:=0.06 -p terrain_risk_cost_max:=8 \
  -p obstacle_inflation_radius:=0.85 -p obstacle_inflation_cost:=62 -p obstacle_inflation_min_cost:=8 -p obstacle_inflation_exponent:=1.35 \
  -p publish_voxel_map:=false -p publish_rate_hz:=2.0 \
  > /tmp/mapper.log 2>&1 &
MAP=$!

python3 /workspace/astrobot-lab/tools/bev_rock_probe.py --ros-args \
  -p use_sim_time:=true -p rock_x:=-1.62 -p rock_y:=-6.73 \
  -p rock_world_x:=18.43 -p rock_world_y:=11.42 -p radius:=0.75 -p csv:=$CSV \
  > /tmp/probe.log 2>&1 &
PROBE=$!
sleep 4

ros2 bag play "$ORIG" --clock --rate 2.0 \
  --topics /astrobot_0/slam/aligned_points_corrected /astrobot_0/slam/pose_corrected \
           /gt/base_link_pose /gt/map /astrobot_0/diagnostics

sleep 2
# CSV already flushed per-msg; abrupt kill is safe. Kill by PID / parent-PID only.
pkill -INT -P "$MAP" 2>/dev/null || true; kill -INT "$MAP" "$PROBE" 2>/dev/null || true
sleep 2
pkill -9 -P "$MAP" 2>/dev/null || true; kill -9 "$MAP" "$PROBE" 2>/dev/null || true
echo "mapper integrate lines: $(grep -c 'integrate points' /tmp/mapper.log || echo 0)"
echo "CSV rows: $(wc -l < $CSV)"
echo "DONE -> $CSV"
