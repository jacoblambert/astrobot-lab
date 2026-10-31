#!/bin/bash
# Live v2 demo/review session: sim nav stream (selected LiDAR) -> v2 mission in astrobot-selected-dev
# (the gated configuration, tools/v2_mission_env.sh) -> Ops console backend (stack:=v2). Optional
# GT-scored eval (--eval) as in the gates. Runs until Ctrl-C, then tears down the sim only (stop + rm;
# never `down`, the dev container is in the same project). Logs go under the archive run printed as
# "Run:".
#
# Usage: tools/v2_demo.sh <seed> <explore_radius|sample_return|teleop_click> [--eval] [--camera]
#   --camera renders the sim's front camera for the console (extra Isaac render load: demo only).
# Then on the host: cd ui/astrobot-console && ASTROBOT_BACKEND=http://<printed ip>:8080 npm run dev
# NOTE: the console is an extra process beside the stack; gate numbers come from v2_gate_run.sh only.
set -o pipefail
cd "$(dirname "$0")/.." || exit 2

SEED="${1:-42}"; MODE="${2:-explore_radius}"; EVAL=0; CAMERA=false
for flag in "${@:3}"; do
  case "$flag" in
    --eval) EVAL=1 ;;
    --camera) CAMERA=true ;;
    *) echo "unknown flag $flag"; exit 2 ;;
  esac
done
case "$MODE" in
  explore_radius|sample_return|teleop_click) ;;
  *) echo "Usage: $0 <seed> <explore_radius|sample_return|teleop_click> [--eval] [--camera]"; exit 2 ;;
esac
COMPOSE=(docker compose -f docker-compose.lunaryard.yml -f docker-compose.archive.yml -f docker-compose.selected-nav.yml)
source tools/v2_mission_env.sh
if [ "$(docker inspect astrobot-sim-lunaryard --format '{{.State.Running}}' 2>/dev/null)" = true ]; then
  echo "Simulator already running; refusing to replace an active world." >&2; exit 2
fi
RUN=$(docker exec $DEV bash -c "$SRC && python3 -c 'from archive_guard import allocate_run, GIB; print(allocate_run(\"/archive\", limit=2*GIB, reserve=20*GIB))'") \
  || { echo "ABORT: archive allocation failed"; exit 2; }
echo "Run: $RUN"

teardown() {
  trap - EXIT INT TERM
  echo "[demo] tearing down (sim stop + rm)"
  docker exec $DEV pkill -INT -f "[p]hase4_exploration_eval.py --mission-mode" >/dev/null 2>&1
  docker exec $DEV pkill -INT -f "[r]os2 launch astrobot_launch phase4_exploration" >/dev/null 2>&1
  docker exec $DEV pkill -INT -f "[r]os2 launch astrobot_launch phase5_gui" >/dev/null 2>&1
  sleep 10
  docker exec $DEV pkill -KILL -f "[a]strobot_ui_backend/ui_backend" >/dev/null 2>&1
  "${COMPOSE[@]}" stop astrobot-sim >/dev/null 2>&1; "${COMPOSE[@]}" rm -f astrobot-sim >/dev/null 2>&1
  exit 0
}
trap teardown EXIT INT TERM

echo "[demo $MODE seed$SEED] sim up"
OMNILRS_CAMERA_ENABLED=$CAMERA "${COMPOSE[@]}" up -d --no-deps astrobot-sim >/dev/null 2>&1 || { echo "ABORT: sim up failed"; exit 2; }
SCENE=""
for i in $(seq 1 90); do
  SCENE=$(docker logs astrobot-sim-lunaryard 2>&1 | grep -o "SELECTED_NAV run=/archive/revision-[0-9a-f]*" | tail -1 | cut -d/ -f3)
  docker logs astrobot-sim-lunaryard 2>&1 | grep -q "SELECTED_NAV tile" && break
  docker ps --format '{{.Names}}' | grep -q astrobot-sim-lunaryard || { echo "ABORT: sim exited during boot"; exit 2; }
  docker logs astrobot-sim-lunaryard 2>&1 | grep -qE "SELECTED_NAV FAILED|Traceback" && { echo "ABORT: sim stream failed"; exit 2; }
  sleep 5
done
[ -n "$SCENE" ] || { echo "ABORT: no sim nav run"; exit 2; }
echo "[demo $MODE seed$SEED] scene $SCENE"

docker exec -d $DEV bash -c "$SRC && ros2 launch astrobot_launch phase5_gui.launch.py stack:=v2 gt_scene_path:=/archive/$SCENE/natural_scene.json > $RUN/ui_backend.log 2>&1"
if [ "$EVAL" = 1 ] && [ "$MODE" != "teleop_click" ]; then
  echo "[demo $MODE seed$SEED] GT-scored eval -> $RUN/eval.json"
  docker exec -d $DEV bash -c "$SRC && python3 tools/phase4_exploration_eval.py --mission-mode $MODE --duration $EVAL_DUR \
    --home-from-gt-start --bev-topic /terrain_shadow/exploration_map --output $RUN/eval.json --record-dir $RUN/record > $RUN/eval_stdout.log 2>&1"
  sleep 15
fi
echo "[demo $MODE seed$SEED] launch v2 mission"
docker exec -d $DEV bash -c "$SRC && ros2 launch astrobot_launch phase4_exploration.launch.py $V2_ARGS $LAUNCH_ARGS > $RUN/mission_launch.log 2>&1"

IP=$(docker inspect $DEV --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}')
cat <<MSG

  Ops console:  cd ui/astrobot-console && ASTROBOT_BACKEND=http://$IP:8080 npm run dev
                then open http://localhost:5173  (backend API: http://$IP:8080/api/state)
  Ctrl-C here tears the session down.

MSG

# Mission state transitions from the console backend (no extra ROS subscribers).
last=""; printed=0
while true; do
  line=$(curl -s -m 3 "http://$IP:8080/api/state" | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit()
m, loc = d['mission'], d.get('localization') or {}
print(f\"mission={m['state']} coverage={m['coverageFraction']:.2f} sample={m['bestSample']:.2f} goals={m['successes']}/{m['goals']} localization={loc.get('state', '-')}:{loc.get('reason', '-')}\")
" 2>/dev/null)
  # on a mission/localization state change, and once a minute for progress
  if [ -n "$line" ] && { [ "${line%% coverage*}|${line##*localization=}" != "$last" ] || [ $((SECONDS - printed)) -ge 60 ]; }; then
    echo "[demo $(date +%H:%M:%S)] $line"
    last="${line%% coverage*}|${line##*localization=}"; printed=$SECONDS
  fi
  sleep 10
done
