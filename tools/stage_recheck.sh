#!/usr/bin/env bash
# Re-validate after the inflation_radius 0.60->0.75 fix (>= circumscribed radius 0.737). Confirms:
# (1) computeCircumscribedCost ERROR flood is GONE, (2) seed-42 return-home succeeds (home_dist<=1.0),
# (3) MPPI_fail stays 0, (4) 411/7 still PASS. Sample_return trio.
set -u
cd /home/jacoblambert/astrobot-lab
SUMMARY=/tmp/recheck_summary.txt
: > "$SUMMARY"
for SEED in 42 411 7; do
  echo "===== RECHECK sample_return seed=$SEED START =====" | tee -a "$SUMMARY"
  tools/phase4_mission_gate_run.sh "$SEED" sample_return --retries 0 2>&1 | tail -10
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
mppi = circ = forced = preck = 0
for lg in glob.glob(os.path.join(root, '**', 'mission_launch.log'), recursive=True):
    try: txt = open(lg, errors='ignore').read()
    except Exception: continue
    mppi += txt.count('Optimizer fail to compute')
    circ += txt.count('computeCircumscribedCost')
    forced += txt.count('force-send (precheck bypassed)')
    preck += txt.count('planner precheck failed')
bs = m.get('best_sample'); hd = m.get('home_dist')
print((f"SEED {seed} | {res} best_sample={bs:.3f} home_dist={hd:.3f} state={m.get('state')} "
       f"reason={m.get('return_reason')}") if bs is not None else f"SEED {seed} | {res} (no metrics)")
print(f"   fail_class={fcc}  MPPI_fail={mppi} circumscribed_err={circ} forced_bypass={forced} precheck_rej={preck}")
PY
  echo | tee -a "$SUMMARY"
done
echo "===== RECHECK COMPLETE =====" | tee -a "$SUMMARY"
echo "--- Stage-2 (inflation 0.60): 42 FAIL(return_failed 3.27m, 1049 circ_err) / 411 PASS / 7 PASS(0.975) ---" | tee -a "$SUMMARY"
