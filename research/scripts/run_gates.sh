#!/bin/bash
# The six 0.1 mission gates on the current tree, one simulator session at a time:
# seeds 42 / 411 / 7 x explore_radius / sample_return through scripts/run_mission.sh --eval.
#   research/scripts/run_gates.sh [<seed>:<mission> ...]     (default: all six)
# Each run writes runs/<stamp>-seed<seed>-<mission>/result.json; the summary goes to
# runs/gates-<stamp>.log. Stops at the first run that aborts (infrastructure, not a gate FAIL).
cd "$(dirname "$0")/../.." || exit 2
GATES=("$@")
[ ${#GATES[@]} -gt 0 ] || GATES=(42:explore_radius 411:explore_radius 7:explore_radius
                                 42:sample_return 411:sample_return 7:sample_return)
LOG=runs/gates-$(date +%Y%m%d-%H%M%S).log
for gate in "${GATES[@]}"; do
  seed=${gate%%:*}; mission=${gate#*:}
  if docker ps --format '{{.Names}}' | grep -qx astrobot-sim; then
    echo "ABORT: a simulator is already running" | tee -a "$LOG"; exit 2
  fi
  echo "$(date +%T) start seed=$seed $mission" | tee -a "$LOG"
  scripts/run_mission.sh "$seed" "$mission" --eval > "$LOG.$seed-$mission.out" 2>&1
  rc=$?
  grep -E "^Run:|=====|ABORT" "$LOG.$seed-$mission.out" | tee -a "$LOG"
  [ "$rc" = 0 ] || { echo "$(date +%T) run aborted rc=$rc" | tee -a "$LOG"; exit "$rc"; }
  sleep 20
done
echo "$(date +%T) done" | tee -a "$LOG"
