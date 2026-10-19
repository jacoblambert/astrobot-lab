#!/usr/bin/env python3
"""Fixed-window relative pose errors from an audited accuracy diagnostic.

No GT feedback, fitting, time shifts or missing-sample interpolation. First
publication and final-revised diagnostics remain distinct, neither constitutes
terrain truth or a coherent-local-map acceptance result.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run
from local_terrain.evaluation import relative_error
from replay_local_terrain import sha


def window_errors(times, truth, estimated, indices, seconds):
    times, truth, estimated, indices = map(np.asarray, (times, truth, estimated, indices))
    if (
        times.ndim != 1
        or not np.isfinite(times).all()
        or not (np.diff(times) > 0).all()
        or truth.shape != (len(times), 4, 4)
        or estimated.shape != (len(indices), 4, 4)
        or indices.dtype.kind not in "iu"
        or len(set(indices.tolist())) != len(indices)
        or (indices < 0).any()
        or (indices >= len(times)).any()
    ):
        raise ValueError("invalid exact-time trajectory")
    lookup = {int(index): pose for index, pose in zip(indices, estimated)}
    values, unavailable = [], 0
    for i, stamp in enumerate(times):
        wanted = stamp - seconds
        if wanted < times[0] - 1e-6:
            continue
        k = int(np.searchsorted(times, wanted))
        matches = [j for j in (k - 1, k) if 0 <= j < len(times) and abs(times[j] - wanted) <= 1e-6]
        # Only floating simulator cadence representation is tolerated, never a
        # substantial fraction of the 33 ms native acquisition interval.
        if len(matches) != 1 or i not in lookup or matches[0] not in lookup:
            unavailable += 1
            continue
        j = matches[0]
        values.append(relative_error(lookup[j], lookup[i], truth[j], truth[i]))
    if not values:
        raise ValueError("no exact fixed-window endpoint pairs")
    result = dict(window_s=seconds, pairs=len(values), missing_pairs=unavailable)
    for key in ("translation_m", "rotation_rad"):
        array = np.array([value[key] for value in values])
        result[key] = dict(p95=float(np.percentile(array, 95)), maximum=float(array.max()))
    # Conservative effect at declared distances; this is not a statistical
    # uncertainty certificate and does not model all correlated surface noise.
    result["endpoint_displacement_bound_p95_m"] = {
        str(radius): float(
            np.percentile([v["translation_m"] + 2 * radius * np.sin(v["rotation_rad"] / 2) for v in values], 95)
        )
        for radius in (2, 3)
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("accuracy_run", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    report_path = args.accuracy_run / "accuracy_report.json"
    array_path = args.accuracy_run / "accuracy_arrays.npz"
    try:
        report_hash = sha(report_path)
        original = json.loads(report_path.read_text())
        if not original["diagnostic_completed"] or sha(array_path) != original["output_sha256"]["accuracy_arrays.npz"]:
            raise ValueError("accuracy diagnostic incomplete or arrays changed")
        with np.load(array_path, allow_pickle=False) as data:
            results = {
                mode: [
                    window_errors(data["times"], data["gt_base"], data[mode], data[mode + "_indices"], seconds)
                    for seconds in (1, 2, 5)
                ]
                for mode in ("first_published", "final_revised")
            }
        if sha(report_path) != report_hash or sha(array_path) != original["output_sha256"]["accuracy_arrays.npz"]:
            raise ValueError("accuracy inputs changed during diagnostic")
        result = dict(
            acceptance=False,
            terrain_truth_evaluated=False,
            results=results,
            source_mapper_completed=original["source_mapper_completed"],
            source_mapper_failure=original["source_mapper_failure"],
            input_sha256={str(report_path): report_hash, str(array_path): sha(array_path)},
            source_sha256={str(Path(__file__).resolve()): sha(Path(__file__))},
        )
        with (output / "relative_motion.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(results, indent=2))
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
