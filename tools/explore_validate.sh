#!/usr/bin/env bash
# explore_radius gate trio (seeds 42/411/7). Coverage gate = mission_coverage_fraction >= 0.70.
# Usage: explore_validate.sh <label>   (label tags the summary file, e.g. baseline / stage3)
set -u
cd /home/jacoblambert/astrobot-lab
LABEL="${1:-explore}"
SUMMARY=/tmp/explore_${LABEL}_summary.txt
: > "$SUMMARY"
for SEED in 42 411 7; do
  echo "===== EXPLORE[$LABEL] seed=$SEED START =====" | tee -a "$SUMMARY"
  tools/phase4_mission_gate_run.sh "$SEED" explore_radius --retries 0 2>&1 | tail -10
  python3 - "$SEED" <<'PY' | tee -a "$SUMMARY"
import json, sys, glob, os
seed = sys.argv[1]
try:
    d = json.load(open(f'rosbags/phase4_gate_explore_radius_seed{seed}.json'))
except Exception as e:
    print(f"SEED {seed} | RESULT JSON MISSING ({e})"); sys.exit()
res = d.get('result'); root = d.get('artifact_root', '')
rec = (d.get('attempt_records') or [{}])[-1]
m = (rec.get('verdict') or {}).get('metrics', {})
fcc = rec.get('failure_class_counts', {})
gs = rec.get('goal_summary', {})
mppi = circ = forced = preck = 0
for lg in glob.glob(os.path.join(root, '**', 'mission_launch.log'), recursive=True):
    try: txt = open(lg, errors='ignore').read()
    except Exception: continue
    mppi += txt.count('Optimizer fail to compute'); circ += txt.count('computeCircumscribedCost')
    forced += txt.count('force-send (precheck bypassed)'); preck += txt.count('planner precheck failed')
cov = m.get('mission_coverage_fraction')
print(f"SEED {seed} | {res} coverage={cov if cov is None else round(cov,3)} state={m.get('state')} "
      f"reason={m.get('return_reason')}")
print(f"   goal_summary={gs}")
print(f"   fail_class={fcc}  MPPI_fail={mppi} circumscribed_err={circ} forced_bypass={forced} precheck_rej={preck}")
PY
  echo | tee -a "$SUMMARY"
done
echo "===== EXPLORE[$LABEL] COMPLETE =====" | tee -a "$SUMMARY"
