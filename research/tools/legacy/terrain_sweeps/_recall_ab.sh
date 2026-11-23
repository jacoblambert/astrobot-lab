#!/bin/bash
# Recall A/B for the BEV mapper: replay a fixture (aligned_points + pose + /gt/map) through the
# mapper with chosen recall knobs (min_voxels, point_stride, seed_confidence_log_odds,
# single_cell_height_gate), holding ALL other params at the current phase2_slam values, then
# score vs the GT rock map. Run INSIDE the dev container. Invoke BARE (no pkill wrapper).
# Usage: _recall_ab.sh <fixture_bag> <min_voxels> <point_stride> <seed_conf_lo> <single_cell:true|false> <label>
set -e
source /etc/ros_setup.sh
source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
BAG="$1"; MINV="$2"; PSTRIDE="$3"; SEEDLO="$4"; SCG="$5"; LABEL="$6"
OUT=/workspace/astrobot-lab/rosbags/recall_ab_${LABEL}
rm -rf "$OUT"
pkill -f probabilistic_voxel_mapper_node 2>/dev/null || true
sleep 1

ros2 run slam_mapping probabilistic_voxel_mapper_node --ros-args \
  -p use_sim_time:=true \
  -p input_topic:=/astrobot_0/slam/aligned_points_corrected \
  -p pose_topic:=/astrobot_0/slam/pose_corrected \
  -p costmap_output_topic:=/astrobot_0/slam/bev_costmap -p voxel_output_topic:=/sweep/vox \
  -p frame_id:=map -p voxel_size:=0.05 -p bev_resolution:=0.10 \
  -p width_m:=40.0 -p height_m:=40.0 -p origin_x:=-20.0 -p origin_y:=-20.0 \
  -p hit_log_odds:=0.5 -p miss_log_odds:=-0.20 -p min_log_odds:=-3.0 -p max_log_odds:=4.0 \
  -p occupied_threshold:=1.0 -p free_threshold:=-0.8 -p max_range:=35.0 \
  -p point_stride:=$PSTRIDE -p ray_stride:=1 -p min_z:=-2.0 -p max_z:=4.0 \
  -p max_voxels:=6000000 -p decay_factor:=0.998 -p decay_min_abs_log_odds:=0.05 \
  -p ground_window_radius:=2 -p ground_percentile:=0.10 \
  -p low_z_percentile:=0.10 -p high_z_percentile:=0.90 \
  -p use_plane_fit:=true -p plane_fit_trim:=0.05 \
  -p min_occupied_voxels_per_cell:=$MINV \
  -p seed_confidence_log_odds:=$SEEDLO -p single_cell_height_gate:=$SCG \
  -p obstacle_height:=0.085 -p lethal_obstacle_height:=0.35 \
  -p obstacle_cost_min:=100 -p obstacle_cost_max:=100 \
  -p terrain_risk_height:=0.055 -p terrain_risk_cost_max:=14 \
  -p slope_cost_max:=0.0 \
  -p obstacle_inflation_radius:=0.75 -p obstacle_inflation_cost:=32 \
  -p obstacle_inflation_min_cost:=10 -p obstacle_inflation_exponent:=1.0 \
  -p publish_voxel_map:=false -p publish_rate_hz:=2.0 \
  > /tmp/recall_mapper_${LABEL}.log 2>&1 &
MAP=$!
sleep 4
ros2 bag record --storage mcap -o "$OUT" \
  /astrobot_0/slam/bev_costmap /gt/map /gt/base_link_pose /astrobot_0/slam/pose_corrected \
  > /tmp/recall_rec_${LABEL}.log 2>&1 &
sleep 2
ros2 bag play "$BAG" --clock --rate 3.0 \
  --topics /astrobot_0/slam/aligned_points_corrected /astrobot_0/slam/pose_corrected \
           /gt/map /gt/base_link_pose \
  > /tmp/recall_play_${LABEL}.log 2>&1
sleep 2
pkill -INT -f "ros2 bag record" 2>/dev/null || true
sleep 4
pkill -f probabilistic_voxel_mapper_node 2>/dev/null || true
sleep 1
if [ ! -f "$OUT/metadata.yaml" ]; then ros2 bag reindex -s mcap "$OUT" >/dev/null 2>&1 || true; fi
echo "===== SCORE: $LABEL (minv=$MINV stride=$PSTRIDE seed_lo=$SEEDLO scg=$SCG) ====="
python3 /workspace/astrobot-lab/tools/bev_bag_score.py "$OUT" --label "$LABEL" --obstacle-thresh 88
