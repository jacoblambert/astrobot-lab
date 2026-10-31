#!/bin/bash
# Phase 4 mission gate on the v2 stack (N3 T5) for ONE (seed, mission_mode), one attempt.
# Host-side: sim nav stream up (selected LiDAR) -> v2 phase4 launch in astrobot-selected-dev ->
# supervisor validated -> GT-scored eval (in-node recording only) -> offline GT rock contacts ->
# teardown of the sim only (stop + rm; never `down`, the dev container is in the same project).
# All output under the archive run printed as "Run:".
#
# Usage: tools/v2_gate_run.sh <seed> <explore_radius|sample_return>
# NOTE: no `set -u` (ROS setup scripts reference unbound vars).
set -o pipefail
cd "$(dirname "$0")/.." || exit 2

SEED="$1"; MODE="$2"
if [ "$MODE" != "explore_radius" ] && [ "$MODE" != "sample_return" ]; then
  echo "ERROR: mode must be explore_radius or sample_return (got '$MODE')"; exit 2
fi
COMPOSE=(docker compose -f docker-compose.lunaryard.yml -f docker-compose.archive.yml -f docker-compose.selected-nav.yml)
# DEV, SRC, SHARE, V2_ARGS, LAUNCH_ARGS, EVAL_DUR and the sim environment (shared with v2_demo.sh).
source tools/v2_mission_env.sh
if [ "$(docker inspect astrobot-sim-lunaryard --format '{{.State.Running}}' 2>/dev/null)" = true ]; then
  echo "Simulator already running; refusing to replace an active world." >&2; exit 2
fi
RUN=$(docker exec $DEV bash -c "$SRC && python3 -c 'from archive_guard import allocate_run, GIB; print(allocate_run(\"/archive\", limit=2*GIB, reserve=20*GIB))'") \
  || { echo "ABORT: archive allocation failed"; exit 2; }
echo "Run: $RUN"
teardown() {
  docker exec $DEV pkill -INT -f "[p]hase4_exploration_eval.py --mission-mode" >/dev/null 2>&1
  docker exec $DEV pkill -INT -f "[r]os2 launch astrobot_launch phase4_exploration" >/dev/null 2>&1
  sleep 10
  "${COMPOSE[@]}" stop astrobot-sim >/dev/null 2>&1; "${COMPOSE[@]}" rm -f astrobot-sim >/dev/null 2>&1
}
trap teardown EXIT

echo "[$MODE seed$SEED] sim up"
"${COMPOSE[@]}" up -d --no-deps astrobot-sim >/dev/null 2>&1 || { echo "ABORT: sim up failed"; exit 2; }
SCENE=""
for i in $(seq 1 90); do
  SCENE=$(docker logs astrobot-sim-lunaryard 2>&1 | grep -o "SELECTED_NAV run=/archive/revision-[0-9a-f]*" | tail -1 | cut -d/ -f3)
  docker logs astrobot-sim-lunaryard 2>&1 | grep -q "SELECTED_NAV tile" && break
  docker ps --format '{{.Names}}' | grep -q astrobot-sim-lunaryard || { echo "ABORT: sim exited during boot"; exit 2; }
  docker logs astrobot-sim-lunaryard 2>&1 | grep -qE "SELECTED_NAV FAILED|Traceback" && { echo "ABORT: sim stream failed"; exit 2; }
  sleep 5
done
[ -n "$SCENE" ] || { echo "ABORT: no sim nav run"; exit 2; }
echo "[$MODE seed$SEED] scene $SCENE"

# The eval starts before the mission so its first GT pose is the spawn (home anchor, spawn rocks):
# when bring-up outlasts the manager's start delay the rover is already moving at "validated".
echo "[$MODE seed$SEED] GT-scored eval (in-node recording only)"
docker exec -d $DEV bash -c "$SRC && python3 tools/phase4_exploration_eval.py --mission-mode $MODE --duration $EVAL_DUR \
  --home-from-gt-start --bev-topic /terrain_shadow/exploration_map --output $RUN/eval.json --record-dir $RUN/record > $RUN/eval_stdout.log 2>&1; touch $RUN/eval.done"
sleep 15
echo "[$MODE seed$SEED] launch v2 mission"
docker exec -d $DEV bash -c "$SRC && ros2 launch astrobot_launch phase4_exploration.launch.py $V2_ARGS $LAUNCH_ARGS > $RUN/mission_launch.log 2>&1"
docker exec $DEV bash -c "$SRC && for i in \$(seq 1 120); do s=\$(timeout 5 ros2 topic echo --once /astrobot_0/localization/status 2>/dev/null | grep -E '^reason'); case \"\$s\" in *validated*) exit 0;; *budget*|*terminal*) exit 1;; esac; sleep 3; done; exit 1" \
  || { echo "ABORT: supervisor never validated"; exit 2; }

echo "[$MODE seed$SEED] mission running"
until docker exec $DEV test -f $RUN/eval.done; do sleep 20; done
docker exec $DEV bash -c "$SRC && python3 tools/score_gt_contacts.py $RUN/record/gt_base.tum --scene-run $SCENE --output $RUN/contacts.json > /dev/null"
docker exec $DEV bash -c "cp -f \$(ls -td /root/.ros/log/*/ 2>/dev/null | head -1)launch.log $RUN/ros_launch.log 2>/dev/null; true"

docker exec -i $DEV python3 - "$RUN" "$MODE" "$SEED" "$SCENE" <<'PYEOF'
import json, sys
from pathlib import Path
run, mode, seed, scene = Path(sys.argv[1]), sys.argv[2], int(sys.argv[3]), sys.argv[4]
verdict = (json.loads((run / "eval.json").read_text()).get("verdict") or {}) if (run / "eval.json").exists() else {}
contacts = json.loads((run / "contacts.json").read_text()) if (run / "contacts.json").exists() else {}
result = dict(mode=mode, seed=seed, stack="v2", scene_run=scene, verdict=verdict, contacts=contacts,
              passed=bool(verdict.get("pass")) and contacts.get("contacts_by_height", {}).get("ge_0.10", 1) == 0)
(run / "gate.json").write_text(json.dumps(result, indent=2))
print(f"===== V2 GATE mode={mode} seed={seed}: {'PASS' if result['passed'] else 'FAIL'} "
      f"reasons={verdict.get('reasons')} contacts={contacts.get('contacts_by_height')} =====")
PYEOF
