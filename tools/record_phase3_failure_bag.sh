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
  /pointcloud \
  /pointcloud/filtered \
  /imu \
  /odom \
  /gt/base_link_pose \
  /map \
  /cmd_vel \
  /cmd_vel_nav \
  /control/status \
  /slam/prob_voxel_map \
  /slam/bev_costmap \
  /global_costmap/costmap \
  /global_costmap/costmap_raw \
  /local_costmap/costmap \
  /local_costmap/costmap_raw \
  /plan \
  /glim_rosnode/pose \
  /glim_rosnode/pose_corrected \
  /glim_rosnode/odom \
  /glim_rosnode/odom_corrected \
  /glim_rosnode/aligned_points \
  /glim_rosnode/aligned_points_corrected \
  /glim_rosnode/points \
  /glim_rosnode/points_corrected \
  /glim_rosnode/map \
  /navigate_to_pose/_action/status \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/result \
  /navigate_to_pose/_action/goal > "${OUT_DIR}/bag_record.log" 2>&1 &
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
