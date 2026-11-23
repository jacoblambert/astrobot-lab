#!/usr/bin/env python3
"""Whole-recording catastrophic-error screen; not navigation qualification."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from archive_guard import allocate_run, check_run
from measure_live_glim_accuracy import statistics
from revision_evidence import validated_transform
from shadow_pose_replay import sha

SCREEN = dict(
    horizontal_m=0.5, rotation_deg=5.0, sustained_s=1.0, residual_step_m=0.5, residual_step_deg=5.0, jump_window_s=0.1
)


def excursions(times, values, threshold):
    indices = np.flatnonzero(values > threshold)
    groups = np.split(indices, np.flatnonzero(np.diff(indices) != 1) + 1)
    return [
        dict(
            start_s=float(times[g[0]]),
            end_s=float(times[g[-1]]),
            duration_s=float(times[g[-1]] - times[g[0]]),
            peak=float(np.max(values[g])),
        )
        for g in groups
        if len(g)
    ]


def assess(times, estimate, truth):
    times, estimate, truth = map(np.asarray, (times, estimate, truth))
    if (
        len(times) < 2
        or times.ndim != 1
        or not np.isfinite(times).all()
        or np.any(np.diff(times) <= 0)
        or np.max(np.diff(times)) > 0.2
        or estimate.shape != truth.shape
        or truth.shape != (len(times), 4, 4)
    ):
        raise ValueError("incomplete/nonmonotonic/gapped pose sequence")
    for poses in (estimate, truth):
        for pose in poses:
            validated_transform(pose)
    residual = estimate[:, :3, 3] - truth[:, :3, 3]
    xy = np.linalg.norm(residual[:, :2], axis=1)
    angular = Rotation.from_matrix(estimate[:, :3, :3] @ np.swapaxes(truth[:, :3, :3], 1, 2))
    angles = np.rad2deg(angular.magnitude())
    steps = np.linalg.norm(np.diff(residual, axis=0), axis=1)
    angle_steps = np.rad2deg((angular[:-1].inv() * angular[1:]).magnitude())
    # Native 60 Hz interpolation must not divide a 10 Hz estimator jump into
    # six apparently benign increments. Also measure changes over one scan.
    elapsed = times - times[0]
    eligible = elapsed >= SCREEN["jump_window_s"]
    if np.any(eligible):
        earlier = elapsed[eligible] - SCREEN["jump_window_s"]
        before = np.column_stack([np.interp(earlier, elapsed, residual[:, k]) for k in range(3)])
        scan_steps = np.linalg.norm(residual[eligible] - before, axis=1)
        scan_angles = np.rad2deg((Slerp(elapsed, angular)(earlier).inv() * angular[eligible]).magnitude())
    else:
        scan_steps, scan_angles = steps, angle_steps
    position_events = excursions(times, xy, SCREEN["horizontal_m"])
    rotation_events = excursions(times, angles, SCREEN["rotation_deg"])
    failures = dict(
        sustained_position=any(e["duration_s"] >= SCREEN["sustained_s"] for e in position_events),
        sustained_rotation=any(e["duration_s"] >= SCREEN["sustained_s"] for e in rotation_events),
        position_error_jump=bool(max(np.max(steps), np.max(scan_steps)) > SCREEN["residual_step_m"]),
        rotation_error_jump=bool(max(np.max(angle_steps), np.max(scan_angles)) > SCREEN["residual_step_deg"]),
    )
    bins = []
    for indices in np.array_split(np.arange(len(times)), 4):
        if not len(indices):
            continue
        bins.append(
            dict(
                start_s=float(times[indices[0]]),
                end_s=float(times[indices[-1]]),
                statistics=statistics(estimate[indices], truth[indices]),
            )
        )
    return dict(
        samples=len(times),
        thresholds=SCREEN,
        catastrophic_screen_pass=not any(failures.values()),
        failures=failures,
        horizontal_excursions=position_events,
        rotation_excursions=rotation_events,
        horizontal_p50_p95_p99_max_m=np.percentile(xy, [50, 95, 99, 100]).tolist(),
        rotation_p50_p95_p99_max_deg=np.percentile(angles, [50, 95, 99, 100]).tolist(),
        maximum_residual_step_m=float(np.max(steps)),
        maximum_residual_step_deg=float(np.max(angle_steps)),
        maximum_scan_interval_residual_step_m=float(np.max(scan_steps)),
        maximum_scan_interval_residual_step_deg=float(np.max(scan_angles)),
        start_end_error_change_m=float(np.linalg.norm(residual[-1] - residual[0])),
        equal_count_time_quarters=bins,
        overall=statistics(estimate, truth),
        navigation_acceptance=False,
        error_bound_is_empirical_only=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("diagnostic", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        check_run(args.diagnostic)
        source = args.diagnostic / "pose_diagnostic.json"
        report = json.loads(source.read_text())
        if not report.get("completed") or (args.diagnostic / "failure.json").exists():
            raise ValueError("failed diagnostic cannot establish full-route stability")
        for name, expected in report["input_sha256"].items():
            if sha(Path(name)) != expected:
                raise ValueError("diagnostic input changed: " + name)
        for name, expected in report["output_sha256"].items():
            if Path(name).name != name or sha(args.diagnostic / name) != expected:
                raise ValueError("diagnostic output changed")
        with np.load(args.diagnostic / "pose_arrays.npz") as arrays:
            times, truth = arrays["times"], arrays["gt_base"]
            results = {mode: assess(times, arrays[mode], truth) for mode in ("first_published", "final_revised")}
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, axes = plt.subplots(1, 3, figsize=(15, 4))
            axes[0].plot(truth[:, 0, 3], truth[:, 1, 3], label="GT")
            for mode in results:
                pose = arrays[mode]
                axes[0].plot(pose[:, 0, 3], pose[:, 1, 3], label=mode, alpha=0.8)
                axes[1].plot(times - times[0], np.linalg.norm(pose[:, :2, 3] - truth[:, :2, 3], axis=1), label=mode)
                error = Rotation.from_matrix(pose[:, :3, :3] @ np.swapaxes(truth[:, :3, :3], 1, 2))
                axes[2].plot(times - times[0], np.rad2deg(error.magnitude()), label=mode)
            axes[0].set(xlabel="Map X (m)", ylabel="Map Y (m)", title="Fixed-frame trajectory")
            axes[0].set_aspect("equal", adjustable="datalim")
            axes[1].set(xlabel="Acquisition elapsed (s)", ylabel="Horizontal error (m)")
            axes[2].set(xlabel="Acquisition elapsed (s)", ylabel="Orientation error (degrees)")
            for ax in axes:
                ax.grid()
                ax.legend()
            fig.tight_layout()
            fig.savefig(output / "stability.png")
            plt.close(fig)
        result = dict(
            completed=True,
            acceptance=False,
            results=results,
            native_surface_gate=report["native_surface_gate"],
            protocol=report["protocol"],
            duration_s=report["duration_s"],
            path_length_m=report["path_length_m"],
            input_sha256={str(source): sha(source)},
            source_sha256=sha(Path(__file__)),
            plot_sha256=sha(output / "stability.png"),
            note="Completed recording only. Health/route gates are separate; no long-term or unaided guarantee.",
        )
        with (output / "stability.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(result, indent=2))
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
