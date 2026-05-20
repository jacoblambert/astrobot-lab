#!/usr/bin/env bash
set -euo pipefail

OUT_ROOT="${1:-/workspace/astrobot-lab/rosbags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/phase3_failure_repro_${STAMP}"
SUMMARY="${OUT_DIR}/probe_summary.json"
INITIAL_REST_SEC="${PHASE3_INITIAL_REST_SEC:-20}"
GOAL_TIMEOUT="${PHASE3_GOAL_TIMEOUT:-220}"

mkdir -p "${OUT_DIR}"
echo "Recording Phase 3 failure-repro bag to ${OUT_DIR}"

ros2 bag record --storage sqlite3 -o "${OUT_DIR}/bag" \
  /clock \
  /tf \
  /tf_static \
  /astrobot_0/lidar_0/pointcloud/raw \
  /astrobot_0/lidar_0/pointcloud/filtered \
  /astrobot_0/imu \
  /astrobot_0/odom \
  /gt/base_link_pose \
  /gt/map \
  /astrobot_0/cmd_vel \
  /astrobot_0/cmd_vel_nav \
  /astrobot_0/control/status \
  /astrobot_0/slam/prob_voxel_map \
  /astrobot_0/slam/bev_costmap \
  /astrobot_0/global_costmap/costmap \
  /astrobot_0/global_costmap/costmap_raw \
  /astrobot_0/local_costmap/costmap \
  /astrobot_0/local_costmap/costmap_raw \
  /astrobot_0/plan \
  /astrobot_0/slam/pose \
  /astrobot_0/slam/pose_corrected \
  /astrobot_0/slam/odom \
  /astrobot_0/slam/odom_corrected \
  /astrobot_0/slam/aligned_points \
  /astrobot_0/slam/aligned_points_corrected \
  /astrobot_0/slam/points \
  /astrobot_0/slam/points_corrected \
  /astrobot_0/slam/glim_map \
  /astrobot_0/navigate_to_pose/_action/status \
  /astrobot_0/navigate_to_pose/_action/feedback \
  /astrobot_0/navigate_to_pose/_action/result \
  /astrobot_0/navigate_to_pose/_action/goal > "${OUT_DIR}/bag_record.log" 2>&1 &
BAG_PID=$!

cleanup() {
  kill "${BAG_PID:-}" 2>/dev/null || true
  wait "${BAG_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

echo "Waiting ${INITIAL_REST_SEC}s before motion for GLIM initialization"
sleep "${INITIAL_REST_SEC}"

python3 /workspace/astrobot-lab/tools/phase3_probe_sweep.py \
  --goal-timeout "${GOAL_TIMEOUT}" \
  --return-home-every 3 \
  --relative-goal=3.0,0.0 \
  --relative-goal=2.1,2.1 \
  --relative-goal=0.0,3.0 \
  --relative-goal=-2.1,2.1 \
  --relative-goal=-3.0,0.0 \
  --relative-goal=-2.1,-2.1 \
  --relative-goal=0.0,-3.0 \
  --relative-goal=2.1,-2.1 \
  --relative-goal=2.7,1.2 \
  --relative-goal=-2.7,1.2 \
  --relative-goal=-1.2,2.7 \
  --relative-goal=1.2,-2.7 \
  --relative-goal=2.4,1.8 \
  --relative-goal=-2.4,-1.8 \
  --relative-goal=0.0,2.6 \
  --output "${SUMMARY}" | tee "${OUT_DIR}/probe_stdout.txt"

sleep 3
cleanup
trap - EXIT

echo "Wrote bag: ${OUT_DIR}/bag"
echo "Wrote summary: ${SUMMARY}"
