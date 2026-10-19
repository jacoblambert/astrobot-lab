#!/usr/bin/env python3
"""Inspect saved rejected sensor arrays; never repair them or qualify the stream."""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from replay_local_terrain import sha


def inspect_arrays(points, depths, intensities, azimuth, zenith):
    # Use the original array precision for exact agreement with the live gate.
    directions = np.column_stack([np.cos(azimuth) * np.cos(zenith), np.sin(azimuth) * np.cos(zenith), -np.sin(zenith)])
    residual = np.linalg.norm(points - directions * depths[:, None], axis=1)
    hit = intensities > 0
    bad = np.flatnonzero(hit & (residual > 0.001))
    worst = sorted(map(int, bad), key=lambda k: (-float(residual[k]), k))[:20]
    return dict(
        samples=len(points),
        hits=int(hit.sum()),
        nonfinite_values={
            name: int((~np.isfinite(value)).sum())
            for name, value in zip(
                ("points", "depths", "intensities", "azimuth", "zenith"), (points, depths, intensities, azimuth, zenith)
            )
        },
        threshold_m=0.001,
        association_bad_hits=len(bad),
        hit_residual_p50_p95_p99_max_m=(
            list(map(float, np.percentile(residual[hit], [50, 95, 99, 100]))) if hit.any() else []
        ),
        worst_hits=[
            dict(
                index=k,
                residual_m=float(residual[k]),
                depth_m=float(depths[k]),
                point_norm_m=float(np.linalg.norm(points[k])),
                point=points[k].tolist(),
                expected_point=(directions[k] * depths[k]).tolist(),
                azimuth_rad=float(azimuth[k]),
                zenith_rad=float(zenith[k]),
            )
            for k in worst
        ],
        failed_prefix_only=True,
        acceptance=False,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument(
        "--check-current-adapter", action="store_true", help="test the saved tick only; no run promotion"
    )
    args = parser.parse_args()
    check_run(args.capture)
    metadata = args.capture / "rejected_native_tick.json"
    rejection = json.loads(metadata.read_text())
    file = args.capture / "rejected_native_tick.npz"
    if (
        rejection["file"] != file.name
        or sha(file) != rejection["sha256"]
        or rejection["acceptance"]
        or rejection["published"]
    ):
        raise ValueError("rejection identity/hash mismatch")
    hashes = {str(p): sha(p) for p in (metadata, file, Path(__file__))}
    with np.load(file, allow_pickle=False) as data:
        report = inspect_arrays(*(data[n] for n in ("points", "depths", "intensities", "azimuth", "zenith")))
        report.update(stamp=float(data["stamp"]), producer_sequence=int(data["producer_sequence"]))
        if args.check_current_adapter:
            import generic_lidar_adapter
            from selected_lidar_profile import profile_path

            sensor = args.capture / "selected_sensor.json"
            hashes[str(sensor)] = sha(sensor)
            hashes[str(Path(generic_lidar_adapter.__file__))] = sha(Path(generic_lidar_adapter.__file__))
            profile = json.loads(sensor.read_text())["profile"]["name"]
            setup = profile_path(profile)
            hashes[str(setup)] = sha(setup)
            adapter = generic_lidar_adapter.NativeWindow(setup)
            adapter.next_sequence = report["producer_sequence"]  # Isolated fragment, not a fabricated stream.
            arrays = [data[k] for k in ("points", "depths", "intensities", "azimuth", "zenith")]
            before = [a.tobytes() for a in arrays]
            error = None
            try:
                adapter.push(report["producer_sequence"], report["stamp"], *arrays)
            except ValueError as exc:
                error = str(exc)
            if [a.tobytes() for a in arrays] != before:
                raise ValueError("adapter modified native arrays")
            report["current_adapter_fragment"] = dict(
                passed=error is None,
                error=error,
                raw_arrays_unchanged=True,
                policy=generic_lidar_adapter.ASSOCIATION_POLICY,
                diagnostics=adapter.association_diagnostics,
                note="One saved rejected tick; no stream continuity or moving acceptance claimed.",
            )
    if any(sha(p) != digest for p, digest in hashes.items()):
        raise ValueError("rejected input changed during inspection")
    report.update(input_sha256=hashes, original_error=rejection["error"])
    output = allocate_run(args.archive_root)
    with (output / "report.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(f"Run: {output}", flush=True)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
