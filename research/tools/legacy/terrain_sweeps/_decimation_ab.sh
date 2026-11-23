#!/bin/bash
# Decimation A/B: replay the density fixture's filtered cloud + imu through the FULL phase2
# stack (GLIM regenerates aligned_points at the chosen config's decimation -> BEV mapper),
# recording bev + /gt/map + poses, then score recall vs the GT rock map. Run BARE inside the
# dev container, with the SIM STOPPED (else live /gt/map contaminates the replay).
# Usage: _decimation_ab.sh <glim_config_abs_path> <label>
set -e
source /etc/ros_setup.sh
source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
CFG="$1"; LABEL="$2"
BAG=/workspace/astrobot-lab/rosbags/density_fixture
OUT=/workspace/astrobot-lab/rosbags/decim_ab_${LABEL}
rm -rf "$OUT"
pkill -f "glim_rosnode|probabilistic_voxel_mapper_node|synthetic_rtk|pointcloud_crop" 2>/dev/null || true
sleep 2

# Full phase2 (GLIM + mapper + synthetic_rtk) with the chosen GLIM config. GLIM consumes the
# replayed filtered cloud + imu; synthetic_rtk consumes the replayed /gt/base_link_pose.
ros2 launch astrobot_launch phase2_slam.launch.py \
  glim_config_path:=$CFG use_sim_time:=true start_synthetic_rtk:=true \
  > /tmp/decim_stack_${LABEL}.log 2>&1 &
sleep 22

ros2 bag record --storage mcap -o "$OUT" \
  /astrobot_0/slam/bev_costmap /gt/map /gt/base_link_pose /astrobot_0/slam/pose_corrected \
  > /tmp/decim_rec_${LABEL}.log 2>&1 &
sleep 2

ros2 bag play "$BAG" --clock --rate 1.0 \
  --topics /astrobot_0/lidar_0/pointcloud/filtered /astrobot_0/imu /gt/map /gt/base_link_pose \
  > /tmp/decim_play_${LABEL}.log 2>&1
sleep 3
pkill -INT -f "ros2 bag record" 2>/dev/null || true
sleep 4
pkill -f "glim_rosnode|probabilistic_voxel_mapper_node|synthetic_rtk|pointcloud_crop|ros2 launch" 2>/dev/null || true
sleep 2
if [ ! -f "$OUT/metadata.yaml" ]; then ros2 bag reindex -s mcap "$OUT" >/dev/null 2>&1 || true; fi
echo "===== SCORE decimation: $LABEL ($CFG) ====="
python3 /workspace/astrobot-lab/tools/bev_bag_score.py "$OUT" --label "$LABEL" --obstacle-thresh 88
