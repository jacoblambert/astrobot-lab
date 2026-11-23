#!/bin/bash
# Run one Astrobot mission in simulation: lunar yard (seed) -> localization, terrain, Nav2 and the
# mission manager in astrobot-dev -> optional ground-truth scoring and Ops console. Ctrl-C (or the
# end of a scored run) tears the simulator down (stop + rm); astrobot-dev keeps running.
#
# Usage: scripts/run_mission.sh <seed> <explore_radius|sample_return|teleop_click> [--eval] [--console] [--camera]
#        [<launch_arg>:=<value> ...]
#   --eval     GT verdict (coverage / sample / home) and rock contacts; ends when the eval does.
#   --console  Ops console backend; then: cd ui/astrobot-console && npm run dev
#   --camera   render the front camera for the console (extra Isaac load; never with --eval gates).
#   name:=value  passed to mission.launch.py, e.g. the RTK fault injection
#              rtk_occlusion_period_s:=300 rtk_occlusion_duration_s:=10
# Outputs: runs/<stamp>-seed<seed>-<mission>/ (mission.log, eval.json, contacts.json, result.json,
#          sim/ with the scene geometry).
# NOTE: no `set -u` (ROS setup scripts reference unbound variables).
set -o pipefail
cd "$(dirname "$0")/.." || exit 2

SEED="${1:?usage: $0 <seed> <mission> [--eval] [--console] [--camera]}"
MISSION="${2:?usage: $0 <seed> <mission> [--eval] [--console] [--camera]}"
EVAL=0; CONSOLE=0; CAMERA=false; LAUNCH_ARGS=""
for flag in "${@:3}"; do
  case "$flag" in
    --eval) EVAL=1 ;;
    --console) CONSOLE=1 ;;
    --camera) CAMERA=true ;;
    *:=*) LAUNCH_ARGS="$LAUNCH_ARGS $flag" ;;
    *) echo "unknown flag $flag" >&2; exit 2 ;;
  esac
done
case "$MISSION" in
  explore_radius) EVAL_DUR=2600 ;;
  sample_return) EVAL_DUR=1500; export OMNILRS_RESOURCE_CENTERS="${OMNILRS_RESOURCE_CENTERS:-11.875,12.133,8.0,1.0}" ;;
  teleop_click) EVAL=0 ;;
  *) echo "mission must be explore_radius, sample_return or teleop_click" >&2; exit 2 ;;
esac

COMPOSE=(docker compose -f docker-compose.yml)
SIM=astrobot-sim
DEV=astrobot-dev
WS=/workspace/astrobot-lab
SRC="source /etc/ros_setup.sh >/dev/null 2>&1 && source $WS/astrobot-lab/install/setup.bash && cd $WS"

if [ "$(docker inspect $SIM --format '{{.State.Running}}' 2>/dev/null)" = true ]; then
  echo "Simulator already running; refusing to replace an active world." >&2; exit 2
fi
RUN=runs/$(date +%Y%m%d-%H%M%S)-seed$SEED-$MISSION
mkdir -p "$RUN"
echo "Run: $RUN"

teardown() {
  trap - EXIT INT TERM
  echo "[mission] tearing down (sim stop + rm)"
  docker exec $DEV pkill -INT -f "[m]ission_eval --mission-mode" >/dev/null 2>&1
  docker exec $DEV pkill -INT -f "[r]os2 launch astrobot_bringup" >/dev/null 2>&1
  sleep 10
  "${COMPOSE[@]}" stop $SIM >/dev/null 2>&1; "${COMPOSE[@]}" rm -f $SIM >/dev/null 2>&1
}
trap teardown EXIT
trap 'teardown; exit 130' INT TERM

if [ "$(docker inspect $DEV --format '{{.State.Running}}' 2>/dev/null)" != true ]; then
  echo "[mission] starting $DEV (first start builds the workspace)"
  "${COMPOSE[@]}" up -d --no-deps $DEV >/dev/null || { echo "ABORT: dev container failed" >&2; exit 2; }
fi
docker exec $DEV bash -c "$SRC && for i in \$(seq 1 120); do ros2 pkg prefix astrobot_bringup >/dev/null 2>&1 && exit 0; sleep 5; done; exit 1" \
  || { echo "ABORT: workspace not built in $DEV" >&2; exit 2; }

echo "[mission $MISSION seed$SEED] sim up"
# The simulator writes its evaluator geometry (natural_scene.json, clearance_tile.npz) into the run.
SCENE=$RUN/sim
OMNILRS_ENVIRONMENT_SEED=$SEED OMNILRS_CAMERA_ENABLED=$CAMERA OMNILRS_RUN_DIR=/$SCENE \
  "${COMPOSE[@]}" up -d --no-deps $SIM >/dev/null 2>&1 || { echo "ABORT: sim up failed" >&2; exit 2; }
for _ in $(seq 1 90); do
  docker logs $SIM 2>&1 | grep -q "SELECTED_NAV tile" && break
  docker ps --format '{{.Names}}' | grep -qx $SIM || { echo "ABORT: sim exited during boot" >&2; exit 2; }
  docker logs $SIM 2>&1 | grep -qE "SELECTED_NAV FAILED|Traceback" && { echo "ABORT: sim stream failed" >&2; exit 2; }
  sleep 5
done
[ -f "$SCENE/clearance_tile.npz" ] || { echo "ABORT: no sim scene in $SCENE" >&2; exit 2; }
echo "[mission $MISSION seed$SEED] scene $SCENE"

if [ "$CONSOLE" = 1 ]; then
  docker exec -d $DEV bash -c "$SRC && ros2 launch astrobot_bringup console.launch.py gt_scene_path:=$SCENE/natural_scene.json > $RUN/console.log 2>&1"
fi
if [ "$EVAL" = 1 ]; then
  # The eval starts before the mission so its first GT pose is the spawn (home anchor, spawn rocks).
  echo "[mission $MISSION seed$SEED] GT-scored eval -> $RUN/eval.json"
  docker exec -d $DEV bash -c "$SRC && ros2 run astrobot_eval mission_eval --mission-mode $MISSION --duration $EVAL_DUR \
    --home-from-gt-start --output $RUN/eval.json --record-dir $RUN/record > $RUN/eval_stdout.log 2>&1; touch $RUN/eval.done"
  sleep 15
fi
echo "[mission $MISSION seed$SEED] launch"
docker exec -d $DEV bash -c "$SRC && ros2 launch astrobot_bringup mission.launch.py mission:=$MISSION$LAUNCH_ARGS > $RUN/mission.log 2>&1"

if [ "$CONSOLE" = 1 ]; then
  echo "  Ops console: cd ui/astrobot-console && npm run dev, then open http://localhost:5173"
fi

if [ "$EVAL" = 0 ]; then
  echo "  Running until Ctrl-C (mission status: ros2 topic echo /astrobot_0/exploration/status)."
  while true; do sleep 60; done
fi

until [ -f "$RUN/eval.done" ]; do sleep 20; done
docker exec $DEV bash -c "$SRC && ros2 run astrobot_eval score_contacts $RUN/record/gt_base.tum --scene $SCENE \
  --output $RUN/contacts.json > /dev/null"
python3 - "$RUN" "$MISSION" "$SEED" "$SCENE" <<'PYEOF'
import json, sys
from pathlib import Path
run, mission, seed, scene = Path(sys.argv[1]), sys.argv[2], int(sys.argv[3]), sys.argv[4]
verdict = (json.loads((run / "eval.json").read_text()).get("verdict") or {}) if (run / "eval.json").exists() else {}
contacts = json.loads((run / "contacts.json").read_text()) if (run / "contacts.json").exists() else {}
result = dict(mission=mission, seed=seed, scene=scene, verdict=verdict, contacts=contacts,
              passed=bool(verdict.get("pass")) and contacts.get("contacts_by_height", {}).get("ge_0.10", 1) == 0)
(run / "result.json").write_text(json.dumps(result, indent=2))
print(f"===== {mission} seed={seed}: {'PASS' if result['passed'] else 'FAIL'} "
      f"reasons={verdict.get('reasons')} contacts={contacts.get('contacts_by_height')} =====")
PYEOF
