#!/usr/bin/env bash
# Offline A/B: replay a recorded phase-4 bag (raw LiDAR + IMU + GT) through the
# CURRENT GLIM (frame pinning + abs_constraint module), with synthetic_rtk fed
# from the bag's /gt/base_link_pose, and score GLIM pose_corrected vs GT.
# Run with abs_constraint.enable=true (treatment) vs a config copy with
# enable=false (baseline) to measure whether RTK fusion reduces divergence.
# No sim needed -- pure replay, reproducible. Run INSIDE the dev container.
#
# Usage: rtk_replay_ab.sh <bag_dir> <config_path> <name> [rate]
set -eo pipefail

BAG_DIR="${1:?usage: rtk_replay_ab.sh <bag_dir> <config_path> <name> [rate]}"
CONFIG_PATH="${2:?config_path}"
NAME="${3:?name}"
RATE="${4:-1.0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/workspace/astrobot-lab/rosbags/rtk_ab_${NAME}_${STAMP}"
mkdir -p "$OUT"

source /etc/ros_setup.sh
source /workspace/astrobot-lab/astrobot-lab/install/setup.bash

cleanup() {
  kill "${GLIM_PID:-}" "${RTK_PID:-}" "${GTREC_PID:-}" "${SLREC_PID:-}" 2>/dev/null || true
  pkill -INT -f "glim_rosnode|pointcloud_crop_filter|pose_stamped_recorder|synthetic_rtk|ros2 bag play" 2>/dev/null || true
  sleep 2
}
trap cleanup EXIT

echo "===== RTK A/B: name=$NAME bag=$BAG_DIR config=$CONFIG_PATH rate=$RATE ====="
echo "out=$OUT"

# 1) GLIM (loads abs_constraint module per config_ros extension_modules)
ros2 launch glim_slam_launch glim_live.launch.py config_path:="$CONFIG_PATH" > "$OUT/glim.log" 2>&1 &
GLIM_PID=$!
sleep 8

# 2) synthetic_rtk off the bag's GT (home-relative output, matches pinned frame)
ros2 run synthetic_rtk synthetic_rtk --ros-args \
  -p use_sim_time:=true \
  -p gt_topic:=/gt/base_link_pose -p output_topic:=/rtk/pose \
  -p world_frame:=map -p home_xy:="[20.0, 18.0]" \
  -p antenna_offset_xyz:="[0.264, 0.017, 0.262]" \
  -p sigma0:=0.02 -p sigma_per_m:=0.001 -p dropout_prob:=0.0 -p latency_s:=0.0 \
  > "$OUT/synthetic_rtk.log" 2>&1 &
RTK_PID=$!

# 3) recorders: GT base + GLIM corrected pose
ros2 run slam_eval pose_stamped_recorder --topic /gt/base_link_pose \
  --output "$OUT/gt_base.tum" --ros-args -p use_sim_time:=true > "$OUT/rec_gt.log" 2>&1 &
GTREC_PID=$!
ros2 run slam_eval pose_stamped_recorder --topic /astrobot_0/slam/pose_corrected \
  --output "$OUT/slam_corrected.tum" --ros-args -p use_sim_time:=true > "$OUT/rec_slam.log" 2>&1 &
SLREC_PID=$!
sleep 3

# 4) play raw inputs + GT + clock (crop filter in glim_live produces /filtered)
ros2 bag play "$BAG_DIR" \
  --topics /clock /gt/base_link_pose /astrobot_0/imu /astrobot_0/lidar_0/pointcloud/raw \
  --rate "$RATE" > "$OUT/bag_play.log" 2>&1
sleep 6
cleanup
trap - EXIT

# 5) convert GLIM IMU-frame pose -> base frame, then score vs GT (yaw + first align)
ros2 run slam_eval transform_tum --input "$OUT/slam_corrected.tum" \
  --output "$OUT/slam_corrected_base.tum" --xyz 0.264 0.017 -0.262 --quat 0 0 1 0 || true

{
  echo "=== corrected_base YAW align ==="
  ros2 run slam_eval compare_tum_trajectories \
    --reference "$OUT/gt_base.tum" --estimate "$OUT/slam_corrected_base.tum" \
    --max-dt 0.1 --align yaw
  echo "=== corrected_base FIRST align (anchored, exposes drift/jumps) ==="
  ros2 run slam_eval compare_tum_trajectories \
    --reference "$OUT/gt_base.tum" --estimate "$OUT/slam_corrected_base.tum" \
    --max-dt 0.1 --align first
} | tee "$OUT/metrics.txt"

echo "--- abs_constraint diagnostics (last) ---"
grep "abs_constraint:" "$OUT/glim.log" | tail -3 | tee -a "$OUT/metrics.txt"
echo "--- smoother health ---"
grep -ciE "corrupt|nan|diverge" "$OUT/glim.log" | sed 's/^/corruption_hits=/' | tee -a "$OUT/metrics.txt"
echo "Wrote $OUT"
