#!/usr/bin/env python3
"""Read-only native chunk cadence/pose diagnostics; never terrain acceptance."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def summarize_frames(frames):
    if len(frames) < 2:
        raise ValueError("native timing audit needs multiple chunks")
    stamps = np.array([f["observed_stamp"] for f in frames])
    if not np.isfinite(stamps).all() or not (np.diff(stamps) > 0).all():
        raise ValueError("nonmonotonic native chunk clock")
    columns = {f["full_columns"] for f in frames}
    rates = {f["configured_rotation_rate"] for f in frames}
    if len(columns) != 1 or min(columns) <= 0 or len(rates) != 1 or min(rates) <= 0:
        raise ValueError("sensor dimensions or rotation rate changed during audit")
    rate = float(sum(f["columns"] for f in frames[1:]) / (stamps[-1] - stamps[0]) / next(iter(columns)))
    nominal = next(iter(rates))
    result = dict(chunks=len(frames), observed_scan_rate_hz=rate, configured_scan_rate_hz=nominal,
                  cadence_passed=abs(rate / nominal - 1.) <= .05,
                  observed_tick_median_s=float(np.median(np.diff(stamps))),
                  eligible_for_moving_replay=False, acceptance=False)
    for clock in ("world_time", "timeline_time"):
        times = np.array([f[clock] for f in frames])
        if not np.isfinite(times).all():
            raise ValueError("nonfinite simulator clock")
        result[clock + "_delta_ratio"] = float((times[-1] - times[0]) / (stamps[-1] - stamps[0]))
    for key in ("world_physics_dt", "world_rendering_dt"):
        result[key] = sorted({f[key] for f in frames})
    for phase in ("current_pose", "previous_pose"):
        records = [f[phase] for f in frames if f.get(phase) is not None]
        residuals = np.array([e for r in records for e in r["residuals_m"]])
        if not len(residuals) or not np.isfinite(residuals).all():
            raise ValueError("missing or nonfinite native surface audit")
        result[phase] = dict(p95_m=float(np.percentile(residuals, 95)),
                             p99_m=float(np.percentile(residuals, 99)), maximum_m=float(residuals.max()),
                             missing=sum(r["missing"] for r in records), samples=len(residuals),
                             nonhorizontal_surface_samples=sum(r["nonhorizontal_surface_samples"] for r in records))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    if (args.capture / "failure.json").exists():
        raise ValueError("failed capture is ineligible")
    audit = json.loads((args.capture / "native_timing_audit.json").read_text())
    files = sorted((args.capture / "native_chunks").glob("chunk_*.npz"))
    if len(files) != len(audit["frames"]):
        raise ValueError("native chunk count differs from audit")
    for file, frame in zip(files, audit["frames"]):
        if file.name != frame["file"] or hashlib.sha256(file.read_bytes()).hexdigest() != frame["sha256"]:
            raise ValueError("native input changed after capture")
    report = summarize_frames(audit["frames"])
    report["duplicate_native_reads"] = audit["duplicate_native_reads"]
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
