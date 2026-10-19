#!/bin/bash
# Phase 4 gate run for ONE (seed, mission_mode). Host-side: full fresh bring-up ->
# scene-load wait -> settle -> launch -> post-launch preflight (clean GLIM init +
# GT-leak guard) -> GT-scored eval verdict -> teardown. Up to (retries+1) attempts;
# Retries retain diagnostics but cannot erase a failed/invalid first attempt.
#
# Usage: tools/phase4_mission_gate_run.sh <seed> <explore_radius|sample_return> [--retries N] [--record DIR]
# Writes rosbags/phase4_gate_<mode>_seed<seed>.json plus per-attempt artifacts.
# NOTE: no `set -u` (ROS setup scripts reference unbound vars).
set -o pipefail
cd "$(dirname "$0")/.." || exit 2   # repo root (astrobot-lab/)

SEED="$1"; MODE="$2"; shift 2
RETRIES=0
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RECORD_REL="phase4_gate_runs/${MODE}_seed${SEED}_${RUN_ID}"   # relative to rosbags/; per-attempt artifacts are always recorded
while [ $# -gt 0 ]; do
  case "$1" in
    --retries) RETRIES="$2"; shift;;
    --record) RECORD_REL="$2"; shift;;
  esac; shift
done

if [ "$MODE" != "explore_radius" ] && [ "$MODE" != "sample_return" ]; then
  echo "ERROR: mode must be explore_radius or sample_return (got '$MODE')"; exit 2
fi

COMPOSE="docker compose -f docker-compose.lunaryard.yml"
WS=/workspace/astrobot-lab/astrobot-lab
COUT=/workspace/astrobot-lab/rosbags          # container path (host-mounted)
HOUT=rosbags                                   # host path
RESULT="$HOUT/phase4_gate_${MODE}_seed${SEED}.json"

# Scenario env + launch args (validated this session).
COMMON_ENV="OMNILRS_ROCKS_ENABLED=true OMNILRS_GT_TF_ENABLED=false OMNILRS_ENVIRONMENT_SEED=$SEED"
if [ "$MODE" = "sample_return" ]; then
  SCEN_ENV="OMNILRS_RESOURCE_ENABLED=true OMNILRS_RESOURCE_MODE=deterministic OMNILRS_RESOURCE_CENTERS=11.875,12.133,8.0,1.0"
  # max_duration 1200 + eval 1500: with REAL rocks nav is slower (prechecks/cancels)
  # and the resource_found trigger only fires between goals, so missions can take
  # 15-20 min; the old 1000s eval cap cut off runs that then completed successfully.
  LAUNCH_ARGS="mission_mode:=sample_return sample_success_threshold:=0.75 resource_weight_scale:=25.0 info_weight:=0.3 resource_gradient_weight:=10.0 home_penalty_weight:=0.5 home_radius_m:=19.0 max_goal_distance_m:=5.0 max_duration_sec:=1200.0 exploration_start_delay:=35.0"
  EVAL_DUR=1500
else
  # explore_radius coverage is slow (~1 cell/goal); give it a big budget so it can
  # reach the 70% gate, and an eval cap that comfortably exceeds it + return-home.
  SCEN_ENV="OMNILRS_RESOURCE_ENABLED=true"
  # coverage_goal_fraction MUST match the gate (0.70): with a stricter mission
  # self-goal (0.80) a run that reaches the gate level keeps exploring instead of
  # returning and times out not_finished (seen: 75% coverage, 37/38 goals, FAIL).
  LAUNCH_ARGS="mission_mode:=explore_radius coverage_goal_fraction:=0.70 home_radius_m:=8.0 max_goal_distance_m:=5.0 mission_cell_size_m:=2.0 mission_candidate_limit:=120 max_duration_sec:=1800.0 breadcrumb_return_enabled:=true use_through_poses_return:=false return_breadcrumb_stride_m:=3.0 return_breadcrumb_min_distance_m:=1.0 exploration_start_delay:=35.0"
  EVAL_DUR=2600
fi

# attempt_run <n> -> 0 PASS, 1 FAIL(scored), 2 ABORT(infra/preflight retryable)
attempt_run() {
  local n="$1"; local tag="${MODE}_seed${SEED}_${RUN_ID}_a${n}"
  local label="${MODE}_seed${SEED}_a${n}"
  echo "----- attempt $n: mode=$MODE seed=$SEED -----"
  # Use a unique artifact root per invocation. Previous runs may contain
  # container-owned files, and host-side cleanup can silently leave stale JSON.
  mkdir -p "$HOUT/$RECORD_REL/$label" || { echo "[$label] ABORT: artifact dir not writable"; return 2; }
  : > "$HOUT/phase4_gate_${tag}.json" || { echo "[$tag] ABORT: result file not writable"; return 2; }

  echo "[$tag] bring up (fresh)"; env $COMMON_ENV $SCEN_ENV $COMPOSE up -d --force-recreate >/dev/null 2>&1 \
    || { echo "[$tag] ABORT: compose up failed"; return 2; }

  echo "[$tag] wait for scene load"
  local loaded=0
  for i in $(seq 1 18); do
    sleep 10
    [ "$($COMPOSE logs astrobot-sim 2>&1 | grep -c 'GT rock map')" -gt 0 ] && { loaded=1; break; }
  done
  [ "$loaded" = 1 ] || { echo "[$tag] ABORT: scene did not load"; $COMPOSE down >/dev/null 2>&1; return 2; }

  # Always bring up the operator UIs (user directive 2026-06-04): foxglove bridge
  # (:8765) + astrobot console backend (:8080) on every run, so the scene/SLAM can
  # be watched live. (Frontend: ui/astrobot-console `npm run dev -- --host` on the
  # host, port 5173 — keep it running; it reconnects across stack restarts.)
  $COMPOSE exec -d astrobot-dev bash -lc \
    "source /etc/ros_setup.sh && cd $WS && source install/setup.bash && \
     ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765 > /tmp/foxglove.log 2>&1"
  $COMPOSE exec -d astrobot-dev bash -lc \
    "source /etc/ros_setup.sh && cd $WS && source install/setup.bash && \
     ros2 launch astrobot_launch phase5_gui.launch.py host:=0.0.0.0 port:=8080 > /tmp/ui_backend.log 2>&1"

  echo "[$tag] settle gate (GT x stable)"
  $COMPOSE exec -T astrobot-dev bash -lc '
    source /etc/ros_setup.sh
    prev=""; stable=0
    for k in $(seq 1 12); do
      x=$(timeout 5 ros2 topic echo --once /gt/base_link_pose 2>/dev/null | grep -A1 "position:" | grep "x:" | tr -d " x:")
      [ -z "$x" ] && { sleep 2; continue; }
      if [ -n "$prev" ] && awk "BEGIN{exit !(($x-($prev))^2 < 0.0001)}"; then
        stable=$((stable+1)); [ "$stable" -ge 3 ] && { echo "settled x=$x"; exit 0; }
      else stable=0; fi
      prev="$x"; sleep 2
    done
    echo "ERROR: GT not confirmed stable"; exit 2' \
    || { echo "[$tag] ABORT: settling failed"; $COMPOSE down >/dev/null 2>&1; return 2; }

  echo "[$tag] launch mission"
  $COMPOSE exec -d astrobot-dev bash -lc \
    "source /etc/ros_setup.sh && cd $WS && source install/setup.bash && \
     ros2 launch astrobot_launch phase4_exploration.launch.py $LAUNCH_ARGS > /tmp/gate_${tag}.log 2>&1"

  # Diagnostic recording is done INSIDE the eval node (already subscribed to GT,
  # SLAM pose, Nav2 feedback, diagnostics, and costmaps) -> NO extra recorder
  # processes, so no observer-effect CPU load on CPU-bound GLIM.
  local CREC="$COUT/$RECORD_REL/$label"
  $COMPOSE exec -T astrobot-dev bash -lc \
    "source /etc/ros_setup.sh && source $WS/install/setup.bash && cd /workspace/astrobot-lab && \
     python3 tools/write_run_manifest.py --output $CREC/manifest.json --seed $SEED \
       --launch-command 'phase4_exploration.launch.py $LAUNCH_ARGS'" \
    || { echo "[$tag] ABORT: provenance capture failed"; $COMPOSE down >/dev/null 2>&1; return 2; }
  echo "[$tag] recording diagnostics (in-eval) -> rosbags/$RECORD_REL/$label"

  echo "[$tag] wait for GLIM, then preflight (GLIM alive + init sane + GT-leak guard)"
  sleep 30
  $COMPOSE exec -T astrobot-dev bash -lc \
    "source /etc/ros_setup.sh && cd /workspace/astrobot-lab && source astrobot-lab/install/setup.bash && \
     python3 tools/phase4_gate_preflight.py"
  local pf=$?
  if [ "$pf" -ne 0 ]; then echo "[$tag] ABORT: preflight rc=$pf (retryable)"; $COMPOSE down >/dev/null 2>&1; return 2; fi

  echo "[$tag] run mission + GT-scored eval (blocks until finished/timeout)"
  $COMPOSE exec -T astrobot-dev bash -lc \
    "source /etc/ros_setup.sh && cd /workspace/astrobot-lab && source astrobot-lab/install/setup.bash && \
     python3 tools/phase4_exploration_eval.py --mission-mode $MODE --duration $EVAL_DUR \
       --output $COUT/phase4_gate_${tag}.json --record-dir $CREC > /tmp/eval_${tag}.log 2>&1"

  # Stash the GLIM console log + eval verdict into the run dir (the eval already wrote
  # the TUMs there via --record-dir) so each <dir> is a self-contained record.
  $COMPOSE exec -T astrobot-dev bash -lc \
    "mkdir -p $CREC; cp -f /tmp/gate_${tag}.log $CREC/mission_launch.log 2>/dev/null; \
     cp -f /tmp/eval_${tag}.log $CREC/eval_stdout.log 2>/dev/null; \
     cp -f /tmp/foxglove.log $CREC/foxglove.log 2>/dev/null; \
     cp -f /tmp/ui_backend.log $CREC/ui_backend.log 2>/dev/null; \
     cp -f $COUT/phase4_gate_${tag}.json $CREC/eval.json 2>/dev/null; true"

  echo "[$tag] tear down"; $COMPOSE down >/dev/null 2>&1

  # Verdict (read the host-mounted eval JSON).
  local v
  v=$(python3 -c "import json;d=json.load(open('$HOUT/phase4_gate_${tag}.json'));x=d.get('verdict') or {};print('PASS' if x.get('pass') else 'FAIL');print(','.join(x.get('reasons',[])))" 2>/dev/null)
  local res="${v%%$'\n'*}"; local reasons="${v#*$'\n'}"
  echo "[$tag] verdict: ${res:-NOJSON} reasons=[$reasons]"
  [ "$res" = "PASS" ] && return 0 || return 1
}

FINAL="FAIL"; PASS_ATTEMPT=0; LAST_RC=1; N=0
for a in $(seq 1 $((RETRIES + 1))); do
  N="$a"; attempt_run "$a"; LAST_RC=$?
  if [ "$LAST_RC" = 0 ]; then
    PASS_ATTEMPT="$a"
    [ "$a" = 1 ] && FINAL="PASS"
    break
  fi
  [ "$LAST_RC" = 2 ] && echo "(attempt $a was an infra/preflight abort)"
done

# Final per-(seed,mode) record.
python3 - "$RESULT" "$MODE" "$SEED" "$FINAL" "$N" "$PASS_ATTEMPT" "$LAST_RC" "$RECORD_REL" "$RUN_ID" <<'PYEOF'
import json, sys
from pathlib import Path
path, mode, seed, final, attempts, pass_attempt, last_rc, record_rel, run_id = sys.argv[1:10]
attempt_records = []
for idx in range(1, int(attempts) + 1):
    tag = f"{mode}_seed{seed}_{run_id}_a{idx}"
    label = f"{mode}_seed{seed}_a{idx}"
    eval_path = Path("rosbags") / f"phase4_gate_{tag}.json"
    rec = {"attempt": idx, "artifact_dir": str(Path("rosbags") / record_rel / label)}
    try:
        data = json.load(open(eval_path))
        rec.update({
            "verdict": data.get("verdict"),
            "latest_status": data.get("latest_status"),
            "goal_summary": data.get("goal_summary"),
            "failure_class_counts": data.get("failure_class_counts"),
            "snapshot_count": data.get("snapshot_count"),
        })
    except Exception as exc:
        rec["error"] = str(exc)
    attempt_records.append(rec)
json.dump({
    "mode": mode, "seed": int(seed), "result": final,
    "attempts": int(attempts), "pass_attempt": int(pass_attempt),
    "last_rc": int(last_rc),
    "artifact_root": str(Path("rosbags") / record_rel),
    "attempt_records": attempt_records,
}, open(path, "w"), indent=2)
print(f"\n===== GATE RESULT mode={mode} seed={seed}: {final} (attempts={attempts}) =====")
PYEOF

[ "$FINAL" = "PASS" ] && exit 0 || exit 1
