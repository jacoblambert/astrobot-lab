#!/usr/bin/env python3
"""Archive a bounded ideal mount/yaw screen from verified observability inputs."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.mounting_study import (
    PHASES,
    PITCHES,
    first_intersections,
    hazard_boxes,
    sampled_cone_visibility,
    sampling_metrics,
    sensor_mount,
    valid_returns,
    world_rays,
)
from local_terrain.observability import minimum_forward_history


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("observability_run", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--diagnose-failed-prefix", action="store_true")
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        archive = args.archive_root.resolve(strict=True)
        source = args.observability_run.resolve(strict=True)
        if not source.is_relative_to(archive) or (source / "failure.json").exists():
            raise ValueError("successful observability diagnostic on archive required")
        report = json.loads((source / "report.json").read_text())
        if report["source_failed"] and not args.diagnose_failed_prefix:
            raise ValueError("failed source requires explicit diagnostic mode")
        hashes = {str(source / "report.json"): sha(source / "report.json"), **report["input_sha256"]}
        for name, digest in hashes.items():
            if not Path(name).resolve(strict=True).is_relative_to(archive) or sha(name) != digest:
                raise ValueError("input provenance changed or outside archive")
        manifests = [json.loads(Path(p).read_text()) for p in hashes if Path(p).name == "manifest.json"]
        capture = next(m for m in manifests if "sensor" in m)
        sensor = capture["sensor"]
        attrs = sensor["sensor_attributes"]
        mount = np.asarray(sensor["base_from_lidar"])
        if not np.allclose(mount, sensor_mount(5), atol=1e-8, rtol=0):
            raise ValueError("study assumes the audited baseline 5 degree mount")
        audit_path = next(Path(p) for p in hashes if Path(p).name == "native_timing_audit.json")
        audit = json.loads(audit_path.read_text())
        full_columns = int(audit["frames"][0]["full_columns"])
        azimuth = -np.pi + np.arange(full_columns) * np.deg2rad(float(attrs["horizontalResolution"]))
        zenith = np.deg2rad(report["native_zenith_degrees"])
        height = report["base_height_m"]
        wheel_xy = [w["xy_base_m"] for w in report["wheels"]]
        minimum, maximum = float(attrs["minRange"]), float(attrs["maxRange"])
        contract = dict(
            protocol="ideal_mount_sampling_v1",
            pitches_deg=list(PITCHES),
            phases_m=list(PHASES),
            base_height_m=height,
            baseline_mount=mount.tolist(),
            wheel_xy=wheel_xy,
            native_zenith_radians=zenith.tolist(),
            sensor_attributes=attrs,
            full_columns=full_columns,
            azimuth_phase="synthetic -pi + index * configured resolution, recorded column count",
            hazard_boxes=hazard_boxes(),
            source_failed=report["source_failed"],
            input_sha256=hashes,
            acceptance=False,
            navigation_authority=False,
            assumptions="level flat ground, no occlusion/noise, ideal instantaneous full views; not mapper input",
            dose="25 full views in both arms; stationary repeats identical geometry, yaw samples 0:15:360 degrees",
        )
        sources = [
            Path(__file__),
            Path(__file__).with_name("local_terrain") / "mounting_study.py",
            Path(__file__).with_name("local_terrain") / "observability.py",
            Path(__file__).with_name("revision_evidence.py"),
            Path(__file__).with_name("archive_guard.py"),
        ]
        contract["source_sha256"] = {str(p): sha(p) for p in sources}
        for path in sources:
            target = output / "source" / path.relative_to(Path(__file__).parent)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
        with (output / "manifest.json").open("x") as stream:
            json.dump(contract, stream, indent=2, allow_nan=False)
        results = []
        for pitch in PITCHES:
            candidate = sensor_mount(pitch)
            for mode in ("stationary", "virtual_yaw360"):
                yaws = np.zeros(25) if mode == "stationary" else np.deg2rad(np.arange(0, 361, 15))
                points = []
                for yaw in yaws:
                    origin, direction = world_rays(candidate, height, azimuth, zenith, yaw)
                    depth, _ = first_intersections(origin, direction)
                    valid = valid_returns(depth, minimum, maximum)
                    points.append(origin + depth[valid, None] * direction[valid])
                points = np.concatenate(points)
                metrics = {str(phase): sampling_metrics(points, wheel_xy, phase) for phase in PHASES}
                origin, direction = world_rays(candidate, height, azimuth, zenith)
                depth, labels = first_intersections(origin, direction, contract["hazard_boxes"])
                valid = valid_returns(depth, minimum, maximum)
                result = dict(
                    pitch_deg=pitch,
                    mode=mode,
                    metrics=metrics,
                    initial_wheel_centers_in_sampled_cone=sampled_cone_visibility(
                        candidate, height, wheel_xy, zenith, yaws, minimum, maximum
                    ),
                    wheel_prior_forward_distance_m=[
                        minimum_forward_history(candidate, height, xy, zenith, minimum=minimum, maximum=maximum)
                        for xy in wheel_xy
                    ],
                    stationary_hazard_ray_counts=[int(((labels == i) & valid).sum()) for i in range(1, 9)],
                )
                results.append(result)
                print(
                    json.dumps({k: result[k] for k in ("pitch_deg", "mode", "initial_wheel_centers_in_sampled_cone")}),
                    flush=True,
                )
        if any(sha(p) != digest for p, digest in {**hashes, **contract["source_sha256"]}.items()):
            raise ValueError("study inputs or source changed during evaluation")
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(
                dict(results=results, acceptance=False, navigation_authority=False), stream, indent=2, allow_nan=False
            )
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
