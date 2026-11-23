#!/bin/bash
# Mapper param sweep: replay the tuning bag through the voxel mapper across a grid of
# (hit_log_odds x decay_factor), score each replay (recall/precision/flicker), build a
# CSV. Run INSIDE the dev container. Direction per user: accumulate slower + decay slower.
#
# Usage: bev_sweep.sh BAG OUTDIR ["HITS"] ["DECAYS"]
#   defaults: HITS="0.4 0.5 0.6"  DECAYS="0.99 0.995 1.0"  (+ a baseline 0.85/0.97 run)
set -e -o pipefail
source /etc/ros_setup.sh
source /workspace/astrobot-lab/astrobot-lab/install/setup.bash
BAG="$1"; OUTDIR="$2"
HITS="${3:-0.4 0.5 0.6}"
DECAYS="${4:-0.99 0.995 1.0}"
MISS=-0.20; MAXLO=4.0; MINOCC=1; OCCTH=1.0
CSV="$OUTDIR/sweep.csv"
mkdir -p "$OUTDIR"; rm -f "$CSV"

run_one() {
  local hit="$1" decay="$2" label="h${1}_d${2}"
  echo "===== sweep $label ====="
  bash /workspace/astrobot-lab/tools/replay_voxel_experiment.sh \
    "$BAG" "$OUTDIR/replay_$label" "$hit" "$MISS" "$MAXLO" "$MINOCC" "$decay" "$OCCTH" \
    | tail -2
  python3 /workspace/astrobot-lab/tools/bev_bag_score.py "$OUTDIR/replay_$label" \
    --label "$label" --csv "$CSV" | tail -1
  # keep the CSV, drop the (large) replay bag to save disk
  rm -rf "$OUTDIR/replay_$label"
}

# Baseline = current live params, for reference.
run_one 0.85 0.97
for hit in $HITS; do
  for decay in $DECAYS; do
    run_one "$hit" "$decay"
  done
done

echo; echo "===== SWEEP TABLE ====="
column -s, -t "$CSV" 2>/dev/null || cat "$CSV"
