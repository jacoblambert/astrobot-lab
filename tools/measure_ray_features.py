#!/usr/bin/env python3
"""Bounded analytic distant structures plus installed GLIM random-grid sampler."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.mounting_study import first_intersections, valid_returns, world_rays
from local_terrain.ray_design import ray_mount

SEEDS = (7, 42, 179, 411, 177)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def object_metrics(points, box, resolution=0.25):
    low, high = np.asarray(box)
    inside = ((points >= low - 1e-5) & (points <= high + 1e-5)).all(axis=1)
    selected = points[inside]
    span = np.ptp(selected, axis=0) if len(selected) else np.zeros(3)
    voxels = len(np.unique(np.floor(selected / resolution).astype(np.int64), axis=0))
    return dict(
        points=len(selected),
        voxels=voxels,
        xy_span_m=float(np.linalg.norm(span[:2])),
        height_span_m=float(span[2]),
        usable=bool(len(selected) >= 8 and voxels >= 4 and np.linalg.norm(span[:2]) >= 1),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("level_study", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=2 * 1024**3)
    print(f"Run: {output}", flush=True)
    try:
        archive, study = args.archive_root.resolve(strict=True), args.level_study.resolve(strict=True)
        if study.parent != archive or (study / "failure.json").exists():
            raise ValueError("completed level screen required")
        manifest = json.loads((study / "manifest.json").read_text())
        if manifest["protocol"] != "single_lidar_ray_design_v2_level":
            raise ValueError("zero-pitch experiment required")
        geometry = manifest["geometry"]
        mount = ray_mount(geometry, require_level=True)
        sensor_world = mount[:3, 3] + [0, 0, geometry["base_height_m"]]
        root = Path(__file__).resolve().parents[1]
        config_path = root / "astrobot-lab/src/glim_slam_launch/config/glim_astrobot/config_preprocess.json"
        preprocess = json.loads(config_path.read_text())["preprocess"]
        if (
            preprocess["downsample_resolution"] != 0.25
            or preprocess["random_downsample_target"] != 20000
            or preprocess["num_threads"] != 2
            or not preprocess["use_random_grid_downsampling"]
            or preprocess["distance_near_thresh"] != 0.5
            or preprocess["distance_far_thresh"] != 100
            or preprocess["enable_cropbox_filter"]
            or preprocess["enable_outlier_removal"]
        ):
            raise ValueError("frozen sampler contract changed")
        sources = [
            Path(__file__),
            root / "tools/glim_shadow/ray_sampling_probe.cpp",
            config_path,
            root / "astrobot-lab/src/glim/src/glim/preprocess/cloud_preprocessor.cpp",
            root / "tools/local_terrain/mounting_study.py",
            root / "tools/local_terrain/ray_design.py",
            root / "tools/revision_evidence.py",
            root / "tools/archive_guard.py",
        ]
        hashes = {str(p): sha(p) for p in sources}
        for p in sources:
            target = output / "source" / p.relative_to(root)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, target)
        inputs = {str(study / name): sha(study / name) for name in ("manifest.json", "report.json")}
        image = subprocess.check_output(
            ["docker", "image", "inspect", "--format", "{{.Id}}", "astrobot-dev:latest"], text=True
        ).strip()
        base = [
            "docker",
            "run",
            "--rm",
            "--network",
            "none",
            "--user",
            f"{os.getuid()}:{os.getgid()}",
            "--mount",
            f"type=bind,source={output},target=/run",
            "--entrypoint",
            "/bin/bash",
            image,
        ]
        command = "g++ -std=c++17 -O2 -I/usr/local/include -I/usr/include/eigen3 /run/source/tools/glim_shadow/ray_sampling_probe.cpp -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lgtsam_points -o /run/sampler && sha256sum /usr/local/lib/libgtsam_points.so.1.2.1 /usr/local/lib/libgtsam_points_cuda.so.1.2.1 && ldd /run/sampler"
        with (output / "build.log").open("x") as log:
            subprocess.run(base + ["-lc", command], stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)
        cases, jobs = [], []
        step = np.deg2rad(float(geometry["sensor_attributes"]["horizontalResolution"]))
        for layout in ("baseline", "mid_dense_10_to_-60"):
            zenith = -np.deg2rad(manifest["layouts"][layout])
            for distance in (10, 20, 30, 40):
                for bearing in range(0, 360, 45):
                    center = distance * np.array([np.cos(np.deg2rad(bearing)), np.sin(np.deg2rad(bearing))])
                    box = ([*(center - 1.5), 0.0], [*(center + 1.5), 2.0])
                    for phase in (0.0, 0.5):
                        az = -np.pi + (np.arange(geometry["full_columns"]) + phase) * step
                        origin, rays = world_rays(mount, geometry["base_height_m"], az, zenith)
                        depth, _ = first_intersections(origin, rays, [box])
                        valid = valid_returns(depth, 0.5, 100)
                        world = origin + depth[valid, None] * rays[valid]
                        local = world - sensor_world  # identity mounting rotation, verified above
                        index = len(cases)
                        filename = f"input_{index:03}.bin"
                        with (output / filename).open("xb") as stream:
                            stream.write(np.asarray(local, dtype="<f8").tobytes())
                        case = dict(
                            layout=layout,
                            distance_m=distance,
                            bearing_deg=bearing,
                            phase_columns=phase,
                            box=box,
                            input=filename,
                            input_sha256=sha(output / filename),
                            raw=object_metrics(world, box),
                            samples=[],
                        )
                        for seed in SEEDS:
                            result = f"sample_{index:03}_{seed}.bin"
                            jobs.append(f"/run/{filename} /run/{result} {seed} 0.25 20000 2\n")
                            case["samples"].append(dict(seed=seed, file=result))
                        cases.append(case)
        with (output / "jobs.txt").open("x") as stream:
            stream.writelines(jobs)
        with (output / "manifest.json").open("x") as stream:
            json.dump(
                dict(
                    protocol="far_structure_sampling_v1",
                    source_sha256=hashes,
                    input_sha256=inputs,
                    container_image=image,
                    build_command=command,
                    preprocess=preprocess,
                    seeds=SEEDS,
                    sensor_world=sensor_world.tolist(),
                    cases=cases,
                    acceptance=False,
                ),
                stream,
                indent=2,
            )
        check_run(output)
        with (output / "sampling.log").open("x") as log:
            subprocess.run(
                base + ["-lc", "/run/sampler /run/jobs.txt"],
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
                timeout=240,
            )
        for case in cases:
            for sample in case["samples"]:
                path = output / sample["file"]
                points = np.fromfile(path, dtype="<f8").reshape(-1, 3) + sensor_world
                sample.update(metrics=object_metrics(points, case["box"]), sha256=sha(path), total_points=len(points))
        result = dict(
            cases=cases,
            structure_screen_passed=all(s["metrics"]["usable"] for c in cases for s in c["samples"]),
            sampler_sha256=sha(output / "sampler"),
            acceptance=False,
            slam_qualified=False,
            warning="analytic static structures and exact downsampling stage, not GLIM odometry or natural-scene qualification",
        )
        if any(sha(p) != v for p, v in {**hashes, **inputs}.items()):
            raise ValueError("input/source changed")
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps({k: v for k, v in result.items() if k != "cases"}), flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
