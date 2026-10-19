#!/bin/bash
# SLAM-divergence characterization STUDY. Runs the phase4 gate harness in --record
# mode across seeds x N repeats, extracts a divergence fingerprint per run, and
# aggregates to answer: ONE failure mode recurring, or SEVERAL? one place or many?
#
# Usage: tools/slam_divergence_study.sh [--seeds "42 411 7"] [--mode sample_return]
#                                       [--reps 5] [--tag run1]
# Each run does its own fresh bring-up/teardown (retries 0 -> one mission per rep, so
# every rep is an independent sample). Expect ~reps*seeds*~8min unattended.
set -o pipefail
cd "$(dirname "$0")/.." || exit 2

SEEDS="42 411 7"; MODE="sample_return"; REPS=5; STAG="$(printf 's%s' "$$")"
while [ $# -gt 0 ]; do
  case "$1" in
    --seeds) SEEDS="$2"; shift;;
    --mode) MODE="$2"; shift;;
    --reps) REPS="$2"; shift;;
    --tag) STAG="$2"; shift;;
  esac; shift
done

HOUT=rosbags
STUDYREL="divergence_study/${STAG}"
STUDYDIR="$HOUT/$STUDYREL"
mkdir -p "$STUDYDIR"
SUMMARY="$STUDYDIR/study_summary.json"

echo "===== SLAM DIVERGENCE STUDY  mode=$MODE seeds=[$SEEDS] reps=$REPS tag=$STAG ====="
for seed in $SEEDS; do
  for rep in $(seq 1 "$REPS"); do
    runrel="$STUDYREL/seed${seed}_r${rep}"
    echo; echo "########## $MODE seed $seed rep $rep -> rosbags/$runrel ##########"
    # retries 0 => exactly one mission attempt per rep (independent sample for stats).
    # The harness records artifacts into rosbags/$runrel/<tag> (its per-attempt subdir).
    tools/phase4_mission_gate_run.sh "$seed" "$MODE" --retries 0 --record "$runrel"
    # Extract the fingerprint from the attempt dir (a1) the harness created.
    adir="$HOUT/$runrel/${MODE}_seed${seed}_a1"
    if [ -d "$adir" ]; then
      python3 tools/slam_divergence_fingerprint.py "$adir" \
        --output "$STUDYDIR/fp_seed${seed}_r${rep}.json"
    else
      echo "WARN: no artifact dir $adir (run may have aborted in preflight)"
    fi
  done
done

echo; echo "===== STUDY AGGREGATE ====="
python3 - "$SUMMARY" "$STUDYDIR" <<'PYEOF'
import glob, json, sys
from collections import Counter, defaultdict
summary_path, sdir = sys.argv[1:3]
fps = []
for f in sorted(glob.glob(f"{sdir}/fp_*.json")):
    try:
        fps.append(json.load(open(f)))
    except Exception:
        pass

def region(xy):
    if not xy:
        return "?"
    return f"({round(xy[0]/3)*3},{round(xy[1]/3)*3})"  # coarse 3m grid bucket

per_seed = defaultdict(lambda: {"n": 0, "diverged": 0})
sigs = Counter()
imu_corr = {"diverged": 0, "diverged_with_imu_warn": 0}
rows = []
for fp in fps:
    s = fp.get("seed")
    per_seed[s]["n"] += 1
    d = bool(fp.get("diverged"))
    if d:
        per_seed[s]["diverged"] += 1
        mo = fp.get("motion_at_onset") or {}
        sig = (fp.get("type"), bool(mo.get("turning")), region(fp.get("onset_gt_xy")))
        sigs[sig] += 1
        imu_corr["diverged"] += 1
        if (fp.get("imu_warn_count") or 0) > 0:
            imu_corr["diverged_with_imu_warn"] += 1
    rows.append({
        "run": fp.get("run"), "seed": s, "diverged": d, "type": fp.get("type"),
        "onset_xy": fp.get("onset_gt_xy"), "max_err_m": fp.get("max_err_m"),
        "turning": (fp.get("motion_at_onset") or {}).get("turning"),
        "yaw_err_deg": fp.get("rel_yaw_err_deg_at_onset"),
        "imu_warns": fp.get("imu_warn_count"),
        "return_reason": (fp.get("outcome") or {}).get("return_reason"),
    })

out = {
    "n_runs": len(fps),
    "per_seed": {str(k): v for k, v in per_seed.items()},
    "signature_counts": {f"{t}|turning={tu}|region={r}": c for (t, tu, r), c in sigs.most_common()},
    "imu_warn_correlation": imu_corr,
    "runs": rows,
}
json.dump(out, open(summary_path, "w"), indent=2)

print(f"runs={len(fps)}")
print("per-seed divergence rate:")
for s, v in sorted(per_seed.items(), key=lambda x: str(x[0])):
    print(f"  seed {s}: {v['diverged']}/{v['n']}")
print("divergence signatures (type | turning | onset-region 3m-bucket)  -> count:")
for (t, tu, r), c in sigs.most_common():
    print(f"  {t:8} | turning={str(tu):5} | region={r:12} -> {c}")
n_sig = len(sigs)
print(f"\n{n_sig} distinct signature(s) across {imu_corr['diverged']} divergences "
      f"-> {'ONE dominant mode' if n_sig <= 1 else ('FEW modes' if n_sig <= 3 else 'MANY scattered modes')}")
print(f"IMU-warning preceded {imu_corr['diverged_with_imu_warn']}/{imu_corr['diverged']} divergences")
print(f"-> {summary_path}")
PYEOF
