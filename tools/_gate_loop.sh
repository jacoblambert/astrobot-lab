#!/bin/bash
# Validate the GLIM scan-gate on the intermittently-diverging seed411 by running N explore trials,
# each a fresh sim, capturing max position_error + scan-gate fire count + slam_diverged per trial.
# Host-side. Appends one line per trial to rosbags/gate_loop_results.txt.
# Usage: _gate_loop.sh <n_trials> <run_seconds_wall>
set -o pipefail
cd "$(dirname "$0")/.."
N="${1:-4}"; RUNSEC="${2:-560}"
COMPOSE="docker compose -f docker-compose.lunaryard.yml"
RES=rosbags/gate_loop_results.txt
echo "=== gate loop: $N trials, ${RUNSEC}s each, seed411 ===" | tee -a "$RES"
for trial in $(seq 1 "$N"); do
  echo "[trial $trial] bring up sim seed411 (headless)" | tee -a "$RES"
  OMNILRS_HEADLESS=true OMNILRS_ENVIRONMENT_SEED=411 OMNILRS_ROCKS_ENABLED=true OMNILRS_GT_TF_ENABLED=false \
    $COMPOSE up -d --force-recreate astrobot-sim astrobot-dev >/dev/null 2>&1
  # wait for sim
  $COMPOSE exec -T astrobot-dev bash -lc '
    source /etc/ros_setup.sh; source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
    ros2 daemon stop >/dev/null 2>&1; sleep 2
    for i in $(seq 1 100); do timeout 4 ros2 topic echo --once /gt/base_link_pose >/dev/null 2>&1 && break; sleep 3; done' >/dev/null 2>&1
  # launch mission
  $COMPOSE exec -d astrobot-dev bash -lc \
    'source /etc/ros_setup.sh && source /workspace/astrobot-lab/astrobot-lab/install/setup.bash && ros2 launch astrobot_launch phase4_exploration.launch.py mission_mode:=explore_radius start_diagnostics:=true > /tmp/gateloop.log 2>&1'
  sleep 32
  $COMPOSE exec -d astrobot-dev bash -lc '
    source /etc/ros_setup.sh && source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
    python3 - <<PY > /tmp/peloop.log 2>&1
import rclpy, json
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("p"); f=open("/tmp/peloop.csv","w"); f.write("stamp,pe\n")
def cb(m):
    if "position_error" not in m.data: return
    try: d=json.loads(m.data)
    except: return
    st=d.get("stamp"); pe=d.get("nav",{}).get("motion",{}).get("position_error_m")
    if st is not None and pe is not None: f.write("{},{}\n".format(st,pe)); f.flush()
n.create_subscription(String,"/astrobot_0/diagnostics",cb,50); rclpy.spin(n)
PY
'
  sleep "$RUNSEC"
  # collect verdict
  VERDICT=$($COMPOSE exec -T astrobot-dev bash -lc '
    mx=$(awk -F, "NR>1{if(\$2>m)m=\$2}END{printf \"%.2f\",m}" /tmp/peloop.csv 2>/dev/null)
    st=$(tail -1 /tmp/peloop.csv 2>/dev/null | cut -d, -f1)
    gf=$(grep -c "scan gate:" /tmp/gateloop.log 2>/dev/null)
    sd=$(grep -c slam_diverged /tmp/gateloop.log 2>/dev/null)
    gl=$(grep -cE "goal [0-9]+:" /tmp/gateloop.log 2>/dev/null)
    echo "simT=$st maxPosErr=$mx gateFires=$gf slamDiverged=$sd goals=$gl"
  ' 2>/dev/null | grep -v "TERM environ")
  echo "[trial $trial] $VERDICT" | tee -a "$RES"
  $COMPOSE exec -T astrobot-dev bash -lc 'pkill -f ros2 2>/dev/null; pkill -f glim_rosnode 2>/dev/null; true' >/dev/null 2>&1
  sleep 4
done
echo "=== gate loop done ===" | tee -a "$RES"
