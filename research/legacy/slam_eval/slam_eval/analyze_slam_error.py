#!/usr/bin/env python3
"""Analyze SLAM trajectory error over time and motion state."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path

from slam_eval.compare_tum_trajectories import (
    align_by_first_pose,
    align_yaw_translation,
    nearest_index,
    read_tum,
    transform_xyz_yaw,
    wrap_angle,
    yaw,
)


@dataclass(frozen=True)
class Sample:
    t: float
    error: float
    yaw_error: float
    gt_speed: float
    gt_yaw_rate: float
    error_rate: float
    mode: str


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[int(q * (len(ordered) - 1))]


def motion_mode(speed: float, yaw_rate: float, turn_thresh: float, speed_thresh: float) -> str:
    turning = abs(yaw_rate) >= turn_thresh
    moving = speed >= speed_thresh
    if turning and moving:
        return "arc"
    if turning:
        return "turn"
    if moving:
        return "linear"
    return "stop"


def build_samples(reference_path: Path, estimate_path: Path, max_dt: float, align: str) -> tuple[list[Sample], float]:
    reference = read_tum(str(reference_path))
    estimate = read_tum(str(estimate_path))
    ref_times = [point.t for point in reference]

    pairs = []
    for est in estimate:
        idx = nearest_index(ref_times, est.t)
        ref = reference[idx]
        if abs(ref.t - est.t) <= max_dt:
            pairs.append((ref, est))
    if len(pairs) < 3:
        raise RuntimeError("not enough timestamp matches")

    if align == "first":
        yaw_offset, tx, ty, tz = align_by_first_pose(pairs[0])
    elif align == "yaw":
        yaw_offset, tx, ty, tz = align_yaw_translation(pairs)
    else:
        yaw_offset, tx, ty, tz = 0.0, 0.0, 0.0, 0.0

    raw_rows = []
    for ref, est in pairs:
        x, y, z, est_yaw = transform_xyz_yaw(est, yaw_offset, tx, ty, tz)
        err = math.sqrt((x - ref.x) ** 2 + (y - ref.y) ** 2 + (z - ref.z) ** 2)
        raw_rows.append((ref, err, abs(wrap_angle(est_yaw - yaw(ref)))))

    samples: list[Sample] = []
    for idx, (ref, err, yaw_err) in enumerate(raw_rows):
        if idx == 0:
            prev_ref, prev_err, _ = raw_rows[idx]
            next_ref = raw_rows[idx + 1][0]
        else:
            prev_ref, prev_err, _ = raw_rows[idx - 1]
            next_ref = ref
        dt = max(1e-6, next_ref.t - prev_ref.t)
        speed = math.hypot(next_ref.x - prev_ref.x, next_ref.y - prev_ref.y) / dt
        yaw_rate = wrap_angle(yaw(next_ref) - yaw(prev_ref)) / dt
        error_rate = (err - prev_err) / max(1e-6, ref.t - prev_ref.t) if idx else 0.0
        mode = motion_mode(speed, yaw_rate, turn_thresh=0.25, speed_thresh=0.05)
        samples.append(Sample(ref.t, err, yaw_err, speed, yaw_rate, error_rate, mode))
    return samples, yaw_offset


def print_group(name: str, samples: list[Sample]) -> None:
    if not samples:
        print(f"{name}: count=0")
        return
    errors = [sample.error for sample in samples]
    yaw_errors = [sample.yaw_error for sample in samples]
    positive_rates = [sample.error_rate for sample in samples if sample.error_rate > 0.0]
    print(
        f"{name}: count={len(samples)} mean={sum(errors)/len(errors):.4f} "
        f"p50={percentile(errors, 0.50):.4f} p90={percentile(errors, 0.90):.4f} "
        f"p95={percentile(errors, 0.95):.4f} max={max(errors):.4f} "
        f"yaw_mean={sum(yaw_errors)/len(yaw_errors):.4f} "
        f"mean_positive_error_rate={sum(positive_rates)/len(positive_rates):.4f}"
        if positive_rates
        else f"{name}: count={len(samples)} mean={sum(errors)/len(errors):.4f} p95={percentile(errors, 0.95):.4f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True)
    parser.add_argument("--estimate", required=True)
    parser.add_argument("--max-dt", type=float, default=0.1)
    parser.add_argument("--align", choices=["first", "yaw", "none"], default="yaw")
    parser.add_argument("--csv", help="Optional output CSV path.")
    parser.add_argument("--top", type=int, default=12, help="Number of largest error-growth samples to print.")
    args = parser.parse_args()

    samples, yaw_offset = build_samples(Path(args.reference), Path(args.estimate), args.max_dt, args.align)
    print(f"matched={len(samples)} align={args.align} yaw_offset={yaw_offset:.6f}")
    print_group("all", samples)
    for mode in ("stop", "linear", "turn", "arc"):
        print_group(mode, [sample for sample in samples if sample.mode == mode])

    print("top_error_growth:")
    for sample in sorted(samples, key=lambda s: s.error_rate, reverse=True)[: args.top]:
        print(
            f"t={sample.t:.3f} mode={sample.mode} err={sample.error:.4f} "
            f"derr_dt={sample.error_rate:.4f} speed={sample.gt_speed:.4f} "
            f"yaw_rate={sample.gt_yaw_rate:.4f} yaw_err={sample.yaw_error:.4f}"
        )

    if args.csv:
        out = Path(args.csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["t", "error", "yaw_error", "gt_speed", "gt_yaw_rate", "error_rate", "mode"])
            for sample in samples:
                writer.writerow(
                    [
                        f"{sample.t:.9f}",
                        f"{sample.error:.9f}",
                        f"{sample.yaw_error:.9f}",
                        f"{sample.gt_speed:.9f}",
                        f"{sample.gt_yaw_rate:.9f}",
                        f"{sample.error_rate:.9f}",
                        sample.mode,
                    ]
                )
        print(f"wrote_csv={out}")


if __name__ == "__main__":
    main()
