#!/bin/bash
# Phase 4 gate SUITE: runs the per-(seed,mode) gate harness across seeds x modes and
# prints a PASS/FAIL matrix + writes an aggregate JSON. Host-side; each run does its
# own fresh bring-up/teardown (no retries by default). Budgets use simulation time.
#
# Usage: tools/phase4_gate_suite.sh [--seeds "42 411 7"] [--modes "explore_radius sample_return"] [--retries N]
set -o pipefail
cd "$(dirname "$0")/.." || exit 2

SEEDS="42 411 7"; MODES="explore_radius sample_return"; RETRIES=0
while [ $# -gt 0 ]; do
  case "$1" in
    --seeds) SEEDS="$2"; shift;;
    --modes) MODES="$2"; shift;;
    --retries) RETRIES="$2"; shift;;
  esac; shift
done

HOUT=rosbags
SUITE="$HOUT/phase4_gate_suite.json"
: > "$HOUT/.gate_suite_rows"

echo "===== PHASE 4 GATE SUITE  seeds=[$SEEDS] modes=[$MODES] retries=$RETRIES ====="
for mode in $MODES; do
  for seed in $SEEDS; do
    echo; echo "########## $mode / seed $seed ##########"
    tools/phase4_mission_gate_run.sh "$seed" "$mode" --retries "$RETRIES"
    rc=$?
    rj="$HOUT/phase4_gate_${mode}_seed${seed}.json"
    echo "$mode $seed $rc $rj" >> "$HOUT/.gate_suite_rows"
  done
done

echo; echo "===== SUITE SUMMARY ====="
python3 - "$SUITE" "$HOUT/.gate_suite_rows" <<'PYEOF'
import json, sys
suite_path, rows_path = sys.argv[1:3]
results = []
for line in open(rows_path):
    mode, seed, rc, rj = line.split()
    rec = {"mode": mode, "seed": int(seed), "result": "PASS" if rc == "0" else "FAIL"}
    try:
        d = json.load(open(rj))
        rec.update({
            "attempts": d.get("attempts"),
            "pass_attempt": d.get("pass_attempt"),
            "artifact_root": d.get("artifact_root"),
        })
        merged = {}
        for attempt in d.get("attempt_records", []):
            for key, value in (attempt.get("failure_class_counts") or {}).items():
                merged[key] = merged.get(key, 0) + int(value)
        rec["failure_class_counts"] = merged
        rec["attempt_records"] = d.get("attempt_records", [])
    except Exception:
        pass
    results.append(rec)

passed = sum(1 for r in results if r["result"] == "PASS")
json.dump({"results": results, "passed": passed, "total": len(results)},
          open(suite_path, "w"), indent=2)

w = max(len(r["mode"]) for r in results) if results else 14
print(f"{'mode'.ljust(w)}  seed  result  attempts  failure_classes")
for r in results:
    classes = ",".join(f"{k}:{v}" for k, v in sorted((r.get("failure_class_counts") or {}).items()))
    print(f"{r['mode'].ljust(w)}  {str(r['seed']).rjust(4)}  {r['result'].ljust(6)}  {r.get('attempts','?')}         {classes}")
print(f"\n{passed}/{len(results)} gates PASSED  ->  {suite_path}")
PYEOF
summary_rc=$?
failed=0
while read -r mode seed rc result_path; do
  [ "$rc" = "0" ] || failed=1
done < "$HOUT/.gate_suite_rows"
rm -f "$HOUT/.gate_suite_rows"
[ "$summary_rc" = "0" ] && [ "$failed" = "0" ]
