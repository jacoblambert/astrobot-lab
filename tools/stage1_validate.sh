#!/usr/bin/env bash
# Stage-1 (nav-robustness) validation: sample_return trio + per-seed nav-health extraction.
# Stage-1 attribution metrics: MPPI "Optimizer fail to compute path" count (mission_launch.log) +
# controller_stall (GT failure_class_counts) should DROP; planner_reject stays high until Stage 2.
# Also a regression check vs the pre-Stage-1 trio (42 FAIL/411 PASS/7 PASS).
set -u
cd /home/jacoblambert/astrobot-lab
SUMMARY=/tmp/stage1_summary.txt
: > "$SUMMARY"

for SEED in 42 411 7; do
  echo "===== STAGE1 sample_return seed=$SEED START =====" | tee -a "$SUMMARY"
  tools/phase4_mission_gate_run.sh "$SEED" sample_return --retries 0 2>&1 | tail -15
  python3 - "$SEED" <<'PY' | tee -a "$SUMMARY"
import json, sys, glob, os
seed = sys.argv[1]
try:
    d = json.load(open(f'rosbags/phase4_gate_sample_return_seed{seed}.json'))
except Exception as e:
    print(f"SEED {seed} | RESULT JSON MISSING ({e})"); sys.exit()
res = d.get('result')
root = d.get('artifact_root', '')
rec = (d.get('attempt_records') or [{}])[-1]
m = (rec.get('verdict') or {}).get('metrics', {})
fcc = rec.get('failure_class_counts', {})
# MPPI optimizer-fail count from the mission launch log(s) under the artifact root
mppi = 0
for lg in glob.glob(os.path.join(root, '**', 'mission_launch.log'), recursive=True):
    try:
        with open(lg, errors='ignore') as f:
            txt = f.read()
        mppi += txt.count('Optimizer fail to compute')
    except Exception:
        pass
print(f"SEED {seed} | {res} best_sample={m.get('best_sample'):.3f} home_dist={m.get('home_dist'):.3f} "
      f"state={m.get('state')} reason={m.get('return_reason')}"
      if m.get('best_sample') is not None else f"SEED {seed} | {res} (no metrics)")
print(f"   failure_class_counts={fcc}  MPPI_optimizer_fail={mppi}")
PY
  echo | tee -a "$SUMMARY"
done

echo "===== STAGE1 VALIDATION COMPLETE =====" | tee -a "$SUMMARY"
echo "--- pre-Stage-1 baseline was: 42 FAIL(0.457,trapped) / 411 PASS(0.931) / 7 PASS(0.924); seed7 had controller_stall=2 planner_reject=143 ---" | tee -a "$SUMMARY"
