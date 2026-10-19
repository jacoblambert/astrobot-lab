#!/bin/bash
# Per-rung SLAM-stability validation. Run INSIDE the dev container, with the sim +
# phase3_nav stack already up (SLAM-frame nav, no exploration manager -> SLAM is the
# only variable). Records GT + GLIM TUM, drives a deterministic forced-goal
# trajectory, then computes ATE + motion-mode error breakdown (+ loop closure).
# The recorded GT-vs-SLAM TUM is the authoritative signal (phase3_nav has no
# diagnostics node). Stability verdict = no large jump + bounded max error + completed.
#
# Usage: slam_validation_run.sh TAG "GOAL1 GOAL2 ..." [--loop] [--goal-timeout S] [--return-home-every N]
#   GOALs are "dx,dy[,dyaw]" offsets from the start pose (passed to phase3_forced_goal_eval).
# NOTE: no `set -u` — the ROS/colcon setup scripts reference unbound vars.
source /etc/ros_setup.sh
source /workspace/astrobot-lab/astrobot-lab/install/setup.bash

TAG="$1"; GOALS="$2"; shift 2
LOOP=0; GTMO=150; RHE=2
while [ $# -gt 0 ]; do
  case "$1" in
    --loop) LOOP=1;;
    --goal-timeout) GTMO="$2"; shift;;
    --return-home-every) RHE="$2"; shift;;
  esac; shift
done

OUT=/workspace/astrobot-lab/rosbags/slamval_$TAG
mkdir -p "$OUT"
GTT="$OUT/gt_base.tum"; SLT="$OUT/slam_corrected.tum"; SLB="$OUT/slam_corrected_base.tum"

echo "===== SLAM-VAL rung=$TAG goals=[$GOALS] loop=$LOOP ====="
# Live TUM recorders (use_sim_time so stamps match for comparison)
ros2 run slam_eval pose_stamped_recorder --topic /gt/base_link_pose --output "$GTT" \
  --ros-args -p use_sim_time:=true > "$OUT/rec_gt.log" 2>&1 &
GTPID=$!
ros2 run slam_eval pose_stamped_recorder --topic /astrobot_0/slam/pose_corrected --output "$SLT" \
  --ros-args -p use_sim_time:=true > "$OUT/rec_slam.log" 2>&1 &
SLPID=$!
# Authoritative watchdog signal: map->odom correction magnitude (frame-clean).
python3 /workspace/astrobot-lab/tools/map_odom_logger.py --output "$OUT/map_odom.csv" \
  > "$OUT/map_odom.log" 2>&1 &
MOPID=$!
sleep 2

# Gate on TF readiness: Nav2 rejects goals (error 102) until GLIM has connected
# map->odom so the full map->odom->base_link chain exists. Wait for it.
echo "waiting for map->base_link TF chain..."
TF_OK=0
for i in $(seq 1 40); do
  if timeout 6 ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true 2>/dev/null | grep -q "Translation"; then
    TF_OK=1; echo "TF chain ready (waited ${i}x)"; break
  fi
  sleep 2
done
[ "$TF_OK" = "1" ] || echo "WARN: map->base_link TF not confirmed; proceeding anyway"

# GT-free guard: if any frame has >1 parent (e.g. odom <- {map, gt_map}), the GT
# map->odom TF is leaking in (OMNILRS_GT_TF_ENABLED not false). That both breaks
# nav AND means GT is contaminating the run -- ABORT loudly rather than score it.
python3 - <<'PYEOF'
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage
rclpy.init()
n = Node("gt_guard"); n.set_parameters([Parameter("use_sim_time", value=True)])
par = {}
n.create_subscription(TFMessage, "/tf",
    lambda m: [par.setdefault(t.child_frame_id, set()).add(t.header.frame_id) for t in m.transforms], 200)
t0 = time.time()
while time.time() - t0 < 4:
    rclpy.spin_once(n, timeout_sec=0.05)
# Only the nav-chain frames matter: gt_map leaking in shows up as odom/base_link
# gaining a second parent (gt_map->odom). Imu_Sensor legitimately has two parents
# (base_link mount + GLIM's slam_odom->Imu_Sensor) -- that's benign, ignore it.
navchain = {c: p for c, p in par.items() if c in ("odom", "base_link") and len(p) > 1}
gt = "gt_map" in {pp for ps in par.values() for pp in ps}
rclpy.shutdown()
if navchain or gt:
    print(f"GT-GUARD FAIL: navchain-multi-parent={navchain} gt_map_in_tree={gt} "
          f"-- set OMNILRS_GT_TF_ENABLED=false. ABORTING rung.")
    sys.exit(3)
print("GT-guard OK: odom/base_link single-parent; gt_map absent from estimation tree.")
PYEOF
if [ $? -ne 0 ]; then
  kill -9 "$GTPID" "$SLPID" "$MOPID" 2>/dev/null
  echo "===== ABORTED rung=$TAG (GT leak) ====="; exit 3
fi

# Drive the deterministic trajectory (also logs live per-goal results).
GOAL_ARGS=""
# Use --goal=VALUE (equals form): a bare "--goal -3,-3" makes argparse treat the
# negative-leading "-3,-3" as an option flag ("expected one argument" error -> null run).
for g in $GOALS; do GOAL_ARGS="$GOAL_ARGS --goal=$g"; done
python3 /workspace/astrobot-lab/tools/phase3_forced_goal_eval.py \
  --goal-timeout "$GTMO" --return-home-every "$RHE" $GOAL_ARGS \
  --output "$OUT/forced_goal.json" > "$OUT/forced_goal.log" 2>&1
echo "trajectory done"

# Stop recorders cleanly (single processes; SIGINT closes the TUM files).
kill -INT "$GTPID" "$SLPID" "$MOPID" 2>/dev/null
for i in $(seq 1 10); do kill -0 "$GTPID" 2>/dev/null || break; sleep 0.5; done
kill -9 "$GTPID" "$SLPID" "$MOPID" 2>/dev/null
echo "gt_poses=$(wc -l < "$GTT" 2>/dev/null) slam_poses=$(wc -l < "$SLT" 2>/dev/null) map_odom_ticks=$(wc -l < "$OUT/map_odom.csv" 2>/dev/null)"

# IMU-frame GLIM pose -> base frame, then compare to GT base-link.
ros2 run slam_eval transform_tum --input "$SLT" --output "$SLB" \
  --xyz 0.264 0.017 -0.262 --quat 0 0 1 0 > "$OUT/transform.log" 2>&1

echo "--- ATE (compare_tum_trajectories, align yaw) ---"
ros2 run slam_eval compare_tum_trajectories --reference "$GTT" --estimate "$SLB" \
  --max-dt 0.1 --align yaw | tee "$OUT/ate.txt"
echo "--- motion-mode error breakdown + top error-growth (analyze_slam_error) ---"
ros2 run slam_eval analyze_slam_error --reference "$GTT" --estimate "$SLB" \
  --max-dt 0.1 --align yaw --csv "$OUT/error_timeline.csv" --top 8 | tee "$OUT/analyze.txt"
if [ "$LOOP" = "1" ]; then
  echo "--- loop/revisit consistency (evaluate_loop_closure) ---"
  ros2 run slam_eval evaluate_loop_closure --reference "$GTT" --estimate "$SLB" \
    --max-dt 0.1 --revisit-radius 0.75 --min-separation-sec 20 --sample-step 5 | tee "$OUT/loop.txt"
fi

# AUTHORITATIVE verdict: origin-anchored GT-vs-SLAM error (the global-align ATE
# above is context-only -- it smears half-healthy/half-diverged runs).
echo "--- origin-anchored error profile (authoritative) ---"
python3 /workspace/astrobot-lab/tools/slam_anchored_error.py \
  --reference "$GTT" --estimate "$SLB" --max-dt 0.1 --jump-thresh 2.0 | tee "$OUT/anchored.txt"

# Cross-check against the watchdog signal: map->odom correction magnitude.
echo "--- map->odom watchdog signal ---"
awk -F, 'NR>1 {if($5>mx)mx=$5; n++} END{printf "map_odom: ticks=%d max_mag=%.3f m (watchdog=2.0)\n", n, mx}' \
  "$OUT/map_odom.csv" 2>/dev/null | tee "$OUT/map_odom_summary.txt"
echo "===== END rung=$TAG ====="
