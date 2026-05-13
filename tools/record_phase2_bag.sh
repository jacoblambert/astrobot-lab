#!/usr/bin/env bash
set -euo pipefail

ROUTE="${1:-short_loop}"
OUT_ROOT="${2:-/workspace/astrobot-lab/rosbags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/phase2_${ROUTE}_${STAMP}"
GT_TUM="${OUT_DIR}/gt_map_base_link.tum"
GLIM_TUM="${OUT_DIR}/glim_pose.tum"
GLIM_CORRECTED_TUM="${OUT_DIR}/glim_pose_corrected.tum"
INITIAL_REST_SEC="${PHASE2_INITIAL_REST_SEC:-10}"
GT_POSE_TOPIC="${PHASE2_GT_POSE_TOPIC:-/gt/base_link_pose}"

case "${ROUTE}" in
  short_loop)
    GOALS=(
      "--relative-goal=2.0,0.0"
      "--relative-goal=0.0,2.0"
      "--relative-goal=-2.0,0.0"
      "--relative-goal=0.0,-2.0"
      "--relative-goal=1.5,1.5"
      "--relative-goal=-1.5,-1.5"
    )
    ;;
  loop_closure)
    GOALS=(
      "--relative-goal=4.0,0.0"
      "--relative-goal=4.0,3.0"
      "--relative-goal=0.0,5.0"
      "--relative-goal=-4.0,3.0"
      "--relative-goal=-4.0,0.0"
      "--relative-goal=0.0,-3.0"
      "--relative-goal=0.0,0.0"
    )
    ;;
  rocks_double_loop)
    # GT-map-selected route for a rocks-enabled Lunaryard spawn near x=20, y=18.
    # Waypoints are clear at 0.75 m radius and feature-rich within a 4 m annulus.
    GOALS=(
      "--relative-goal=-4.0,3.0"
      "--relative-goal=1.0,3.0"
      "--relative-goal=5.0,4.0"
      "--relative-goal=4.0,0.0"
      "--relative-goal=1.0,2.0"
      "--relative-goal=-4.0,2.0"
      "--relative-goal=-4.0,3.0"
      "--relative-goal=1.0,3.0"
      "--relative-goal=5.0,4.0"
      "--relative-goal=4.0,0.0"
      "--relative-goal=1.0,2.0"
      "--relative-goal=-4.0,2.0"
      "--relative-goal=0.0,0.0"
    )
    ;;
  rocks_stress)
    GOALS=(
      "--auto-free"
      "10"
      "--auto-occupied"
      "0"
      "--min-distance"
      "1.0"
      "--max-distance"
      "5.0"
      "--clearance"
      "0.60"
      "--seed"
      "278"
    )
    ;;
  rocks_map_eval)
    GOALS=(
      "--auto-free"
      "4"
      "--auto-occupied"
      "0"
      "--min-distance"
      "2.0"
      "--max-distance"
      "5.0"
      "--clearance"
      "1.40"
      "--seed"
      "179"
    )
    ;;
  rocks_random_long)
    GOALS=(
      "--auto-free"
      "20"
      "--auto-occupied"
      "0"
      "--min-distance"
      "1.0"
      "--max-distance"
      "5.0"
      "--clearance"
      "0.60"
      "--seed"
      "4242"
    )
    ;;
  *)
    echo "Unknown route '${ROUTE}'. Use short_loop, loop_closure, rocks_double_loop, rocks_stress, rocks_map_eval, or rocks_random_long." >&2
    exit 2
    ;;
esac

mkdir -p "${OUT_DIR}"
echo "Recording Phase 2 bag to ${OUT_DIR}"

ros2 run slam_eval pose_stamped_recorder --topic "${GT_POSE_TOPIC}" --output "${GT_TUM}" --ros-args -p use_sim_time:=true &
GT_PID=$!

ros2 run slam_eval pose_stamped_recorder --topic /glim_rosnode/pose --output "${GLIM_TUM}" --ros-args -p use_sim_time:=true &
GLIM_PID=$!

ros2 run slam_eval pose_stamped_recorder --topic /glim_rosnode/pose_corrected --output "${GLIM_CORRECTED_TUM}" --ros-args -p use_sim_time:=true &
GLIM_CORRECTED_PID=$!

ros2 bag record --storage sqlite3 -o "${OUT_DIR}/bag" \
  /pointcloud \
  /pointcloud/filtered \
  /imu \
  /odom \
  /tf \
  /tf_static \
  /map \
  "${GT_POSE_TOPIC}" \
  /clock \
  /cmd_vel \
  /cmd_vel_nav \
  /control/status \
  /glim_rosnode/pose \
  /glim_rosnode/pose_corrected \
  /glim_rosnode/odom \
  /glim_rosnode/odom_corrected \
  /glim_rosnode/aligned_points_corrected \
  /glim_rosnode/points_corrected \
  /glim_rosnode/map \
  /navigate_to_pose/_action/status &
BAG_PID=$!

cleanup() {
  kill "${BAG_PID}" "${GT_PID}" "${GLIM_PID}" "${GLIM_CORRECTED_PID}" 2>/dev/null || true
  wait "${BAG_PID}" "${GT_PID}" "${GLIM_PID}" "${GLIM_CORRECTED_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "Waiting ${INITIAL_REST_SEC}s before motion for LiDAR/IMU initialization"
sleep "${INITIAL_REST_SEC}"
SWEEP_STATUS=0
set +e
python3 /workspace/astrobot-lab/tools/phase1_goal_sweep.py --timeout 120 --pose-topic "${GT_POSE_TOPIC}" "${GOALS[@]}" | tee "${OUT_DIR}/route_result.txt"
SWEEP_STATUS=${PIPESTATUS[0]}
set -e
sleep 3

cleanup
trap - EXIT
echo "Wrote bag: ${OUT_DIR}/bag"
echo "Wrote GT TUM: ${GT_TUM}"
echo "Wrote GLIM TUM: ${GLIM_TUM}"
echo "Wrote GLIM corrected TUM: ${GLIM_CORRECTED_TUM}"
exit "${SWEEP_STATUS}"
