#!/usr/bin/env bash
set -eo pipefail

SRC_DIR="${1:?usage: run_glim_replay_eval.sh <phase2_bag_dir> [config_path] [name] [rate]}"
CONFIG_PATH="${2:-/workspace/astrobot-lab/astrobot-lab/src/glim_slam_launch/config/glim_astrobot}"
NAME="${3:-default}"
RATE="${4:-1.0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="/workspace/astrobot-lab/rosbags/glim_replay_eval_${NAME}_${STAMP}"
EVAL_BAG="${OUT_DIR}/bag"

if [[ ! -d "${SRC_DIR}/bag" ]]; then
  echo "Source bag directory must contain a bag/ subdirectory: ${SRC_DIR}" >&2
  exit 2
fi

mkdir -p "${OUT_DIR}"
cp "${SRC_DIR}/gt_map_base_link.tum" "${OUT_DIR}/gt_map_base_link.tum"

cleanup() {
  kill "${BAG_RECORD_PID:-}" "${RAW_PID:-}" "${CORR_PID:-}" "${GLIM_LAUNCH_PID:-}" 2>/dev/null || true
  wait "${BAG_RECORD_PID:-}" "${RAW_PID:-}" "${CORR_PID:-}" "${GLIM_LAUNCH_PID:-}" 2>/dev/null || true
  pkill -INT -f "glim_rosnode|pointcloud_crop_filter_node|pose_stamped_recorder|ros2 bag record|ros2 bag play" 2>/dev/null || true
}
trap cleanup EXIT

echo "Output: ${OUT_DIR}"
echo "Config: ${CONFIG_PATH}"
echo "Rate: ${RATE}"

ros2 launch glim_slam_launch glim_live.launch.py config_path:="${CONFIG_PATH}" > "${OUT_DIR}/glim.log" 2>&1 &
GLIM_LAUNCH_PID=$!
sleep 8

ros2 run slam_eval pose_stamped_recorder \
  --topic /astrobot_0/slam/pose \
  --output "${OUT_DIR}/glim_pose.tum" \
  --ros-args -p use_sim_time:=true > "${OUT_DIR}/pose_recorder.log" 2>&1 &
RAW_PID=$!

ros2 run slam_eval pose_stamped_recorder \
  --topic /astrobot_0/slam/pose_corrected \
  --output "${OUT_DIR}/glim_pose_corrected.tum" \
  --ros-args -p use_sim_time:=true > "${OUT_DIR}/pose_corrected_recorder.log" 2>&1 &
CORR_PID=$!

ros2 bag record --storage sqlite3 -o "${EVAL_BAG}" \
  /gt/map \
  /astrobot_0/lidar_0/pointcloud/filtered \
  /astrobot_0/slam/aligned_points_corrected \
  /astrobot_0/slam/points_corrected \
  /astrobot_0/slam/glim_map \
  /astrobot_0/slam/pose \
  /astrobot_0/slam/pose_corrected \
  /tf \
  /tf_static \
  /clock > "${OUT_DIR}/bag_record.log" 2>&1 &
BAG_RECORD_PID=$!

sleep 3
ros2 bag play "${SRC_DIR}/bag" --topics /clock /gt/map /astrobot_0/imu /astrobot_0/lidar_0/pointcloud/raw --rate "${RATE}" > "${OUT_DIR}/bag_play.log" 2>&1
sleep 8

cleanup
trap - EXIT

ros2 run slam_eval transform_tum \
  --input "${OUT_DIR}/glim_pose.tum" \
  --output "${OUT_DIR}/glim_pose_base.tum" \
  --xyz 0.264 0.017 -0.262 \
  --quat 0 0 1 0

ros2 run slam_eval transform_tum \
  --input "${OUT_DIR}/glim_pose_corrected.tum" \
  --output "${OUT_DIR}/glim_pose_corrected_base.tum" \
  --xyz 0.264 0.017 -0.262 \
  --quat 0 0 1 0

{
  echo "raw_base_first"
  ros2 run slam_eval compare_tum_trajectories \
    --reference "${OUT_DIR}/gt_map_base_link.tum" \
    --estimate "${OUT_DIR}/glim_pose_base.tum" \
    --max-dt 0.1 \
    --align first
  echo "raw_base_yaw"
  ros2 run slam_eval compare_tum_trajectories \
    --reference "${OUT_DIR}/gt_map_base_link.tum" \
    --estimate "${OUT_DIR}/glim_pose_base.tum" \
    --max-dt 0.1 \
    --align yaw
  echo "corrected_base_first"
  ros2 run slam_eval compare_tum_trajectories \
    --reference "${OUT_DIR}/gt_map_base_link.tum" \
    --estimate "${OUT_DIR}/glim_pose_corrected_base.tum" \
    --max-dt 0.1 \
    --align first
  echo "corrected_base_yaw"
  ros2 run slam_eval compare_tum_trajectories \
    --reference "${OUT_DIR}/gt_map_base_link.tum" \
    --estimate "${OUT_DIR}/glim_pose_corrected_base.tum" \
    --max-dt 0.1 \
    --align yaw
} | tee "${OUT_DIR}/trajectory_base_metrics.txt"

ros2 run slam_eval evaluate_loop_closure \
  --reference "${OUT_DIR}/gt_map_base_link.tum" \
  --estimate "${OUT_DIR}/glim_pose_corrected_base.tum" \
  --max-dt 0.1 \
  --revisit-radius 0.75 \
  --min-separation-sec 30.0 | tee "${OUT_DIR}/loop_closure_metrics.txt"

ros2 run slam_eval compare_bev_maps \
  --bag "${EVAL_BAG}" \
  --reference-tum "${OUT_DIR}/gt_map_base_link.tum" \
  --estimate-tum "${OUT_DIR}/glim_pose_corrected_base.tum" \
  --align yaw \
  --height-threshold 0.10 \
  --min-points-per-cell 2 \
  --dilation-radius 2 \
  --feature-dilation-radius 2 \
  --ignore-border-cells 2 | tee "${OUT_DIR}/bev_metrics.json"

ros2 run slam_eval analyze_pointcloud_height \
  --bag "${EVAL_BAG}" \
  --topics /astrobot_0/lidar_0/pointcloud/filtered /astrobot_0/slam/aligned_points_corrected /astrobot_0/slam/glim_map \
  --cell-size 0.5 \
  --max-radius 15.0 \
  --min-points-per-cell 4 | tee "${OUT_DIR}/pointcloud_height_metrics.json"

echo "Wrote ${OUT_DIR}"
