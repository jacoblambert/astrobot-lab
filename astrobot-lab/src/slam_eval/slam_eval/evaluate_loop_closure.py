#!/usr/bin/env python3
"""Evaluate SLAM-frame consistency at physical revisits.

This does not align the SLAM map to GT. Instead, it finds timestamp pairs where
GT says the robot revisited the same physical area and measures the distance
between those two poses in the SLAM estimate. That makes the metric suitable for
SLAM maps whose global frame is allowed to differ from simulator GT.
"""

from __future__ import annotations

import argparse
import math

from slam_eval.compare_tum_trajectories import TrajectoryPoint, nearest_index, read_tum, wrap_angle, yaw


def distance_xy(a: TrajectoryPoint, b: TrajectoryPoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def matched_pairs(
    reference: list[TrajectoryPoint],
    estimate: list[TrajectoryPoint],
    max_dt: float,
) -> list[tuple[TrajectoryPoint, TrajectoryPoint]]:
    ref_times = [point.t for point in reference]
    pairs = []
    for est in estimate:
        idx = nearest_index(ref_times, est.t)
        ref = reference[idx]
        if abs(ref.t - est.t) <= max_dt:
            pairs.append((ref, est))
    return pairs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True, help="GT TUM trajectory.")
    parser.add_argument("--estimate", required=True, help="SLAM TUM trajectory in its own frame.")
    parser.add_argument("--max-dt", type=float, default=0.1)
    parser.add_argument("--revisit-radius", type=float, default=0.75, help="GT distance threshold for a revisit.")
    parser.add_argument("--min-separation-sec", type=float, default=30.0, help="Minimum time between revisit poses.")
    parser.add_argument("--sample-step", type=int, default=10, help="Use every Nth matched pose to limit pair count.")
    parser.add_argument("--max-pairs", type=int, default=200000, help="Stop after collecting this many revisit pairs.")
    args = parser.parse_args()

    pairs = matched_pairs(read_tum(args.reference), read_tum(args.estimate), args.max_dt)
    if not pairs:
        raise RuntimeError("no timestamp matches; check clocks or increase --max-dt")

    step = max(args.sample_step, 1)
    samples = pairs[::step]
    slam_distances: list[float] = []
    gt_distances: list[float] = []
    yaw_deltas: list[float] = []
    separations: list[float] = []

    for i, (ref_a, est_a) in enumerate(samples):
        for ref_b, est_b in samples[i + 1 :]:
            separation = ref_b.t - ref_a.t
            if separation < args.min_separation_sec:
                continue
            gt_distance = distance_xy(ref_a, ref_b)
            if gt_distance > args.revisit_radius:
                continue

            slam_distances.append(distance_xy(est_a, est_b))
            gt_distances.append(gt_distance)
            yaw_deltas.append(abs(wrap_angle(yaw(est_b) - yaw(est_a))))
            separations.append(separation)
            if len(slam_distances) >= args.max_pairs:
                break
        if len(slam_distances) >= args.max_pairs:
            break

    if not slam_distances:
        raise RuntimeError(
            "no revisit pairs found; increase --revisit-radius, reduce --min-separation-sec, or use a looped route"
        )

    close_1m = sum(1 for value in slam_distances if value <= 1.0) / len(slam_distances)
    close_15m = sum(1 for value in slam_distances if value <= 1.5) / len(slam_distances)
    close_2m = sum(1 for value in slam_distances if value <= 2.0) / len(slam_distances)
    mean = sum(slam_distances) / len(slam_distances)
    rms = math.sqrt(sum(value * value for value in slam_distances) / len(slam_distances))
    yaw_mean = sum(yaw_deltas) / len(yaw_deltas)
    yaw_rms = math.sqrt(sum(value * value for value in yaw_deltas) / len(yaw_deltas))

    print(
        f"matched={len(pairs)} samples={len(samples)} revisits={len(slam_distances)} "
        f"revisit_radius={args.revisit_radius:.3f} min_separation={args.min_separation_sec:.1f}"
    )
    print(
        f"slam_revisit_dist mean={mean:.4f} rms={rms:.4f} "
        f"p50={percentile(slam_distances, 0.50):.4f} p90={percentile(slam_distances, 0.90):.4f} "
        f"p95={percentile(slam_distances, 0.95):.4f} max={max(slam_distances):.4f}"
    )
    print(
        f"gt_revisit_dist mean={sum(gt_distances) / len(gt_distances):.4f} "
        f"p95={percentile(gt_distances, 0.95):.4f} max={max(gt_distances):.4f}"
    )
    print(
        f"slam_revisit_yaw mean={yaw_mean:.4f} rms={yaw_rms:.4f} "
        f"p95={percentile(yaw_deltas, 0.95):.4f} max={max(yaw_deltas):.4f}"
    )
    print(f"close_rate_1m={close_1m:.4f} close_rate_1_5m={close_15m:.4f} close_rate_2m={close_2m:.4f}")
    print(
        f"separation_sec mean={sum(separations) / len(separations):.1f} "
        f"p50={percentile(separations, 0.50):.1f} max={max(separations):.1f}"
    )


if __name__ == "__main__":
    main()
