#!/usr/bin/env bash
# Stage-2 (precheck anti-trap) validation: sample_return trio. Seeds 42 & 7 were trapped at spawn
# (planner_reject floods -> budget_exhausted, never moved). Stage 2 = distance-scaled unknown tolerance
# + force-move escape hatch. Expect: 42/7 LEAVE spawn (planner_reject drops, best_sample up, motion),
# home_dist stays <=1.0 (the KEY RISK: near-relaxation must not cause far-wander), 411 still PASS,
# Stage-1 nav health holds (MPPI_fail=0, controller_stall=0).
set -u
cd /home/jacoblambert/astrobot-lab
SUMMARY=/tmp/stage2_summary.txt
: > "$SUMMARY"

for SEED in 42 411 7; do
  echo "===== STAGE2 sample_return seed=$SEED START =====" | tee -a "$SUMMARY"
  tools/phase4_mission_gate_run.sh "$SEED" sample_return --retries 0 2>&1 | tail -12
  python3 - "$SEED" <<'PY' | tee -a "$SUMMARY"
import json, sys, glob, os
seed = sys.argv[1]
try:
    d = json.load(open(f'rosbags/phase4_gate_sample_return_seed{seed}.json'))
except Exception as e:
    print(f"SEED {seed} | RESULT JSON MISSING ({e})"); sys.exit()
res = d.get('result'); root = d.get('artifact_root', '')
rec = (d.get('attempt_records') or [{}])[-1]
m = (rec.get('verdict') or {}).get('metrics', {})
fcc = rec.get('failure_class_counts', {})
mppi = forced = preck = unkrej = 0
for lg in glob.glob(os.path.join(root, '**', 'mission_launch.log'), recursive=True):
    try:
        txt = open(lg, errors='ignore').read()
    except Exception:
        continue
    mppi  += txt.count('Optimizer fail to compute')
    forced+= txt.count('force-send (precheck bypassed)')
    preck += txt.count('planner precheck failed')
    unkrej+= txt.count('path_unknown_fraction')
bs = m.get('best_sample'); hd = m.get('home_dist')
head = (f"SEED {seed} | {res} best_sample={bs:.3f} home_dist={hd:.3f} state={m.get('state')} "
        f"reason={m.get('return_reason')}") if bs is not None else f"SEED {seed} | {res} (no metrics)"
print(head)
print(f"   fail_class={fcc}  MPPI_fail={mppi} forced_bypass={forced} precheck_rej={preck} unknown_rej={unkrej}")
PY
  echo | tee -a "$SUMMARY"
done
echo "===== STAGE2 VALIDATION COMPLETE =====" | tee -a "$SUMMARY"
echo "--- Stage-1 baseline: 42 FAIL(0.457,276 precheck) / 411 PASS(0.849) / 7 FAIL(0.454,277 precheck); MPPI_fail=0 all ---" | tee -a "$SUMMARY"
