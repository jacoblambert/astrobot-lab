#!/usr/bin/env python3
"""Archive native-ray timed-cloud integration through installed GLIM preprocessing.

No odometry, IMU synthesis, GT transforms, publication or moving qualification.
Each settled view is independent; six original native ticks make one window.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from assess_ray_terrain import grouped_cases
from generic_lidar_adapter import NativeWindow, FIELDS
from local_terrain.ray_design import SELECTED_SETUP, ray_mount, validate_selected_setup


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("native_fixture", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=2 * 1024**3)
    print(f"Run: {output}", flush=True)
    try:
        archive, native = args.archive_root.resolve(strict=True), args.native_fixture.resolve(strict=True)
        if native.parent != archive or (native / "failure.json").exists():
            raise ValueError("completed external native fixture required")
        manifest = json.loads((native / "manifest.json").read_text())
        report = json.loads((native / "report.json").read_text())
        if not report["fixture_passed"] or not report["equal_dose"]:
            raise ValueError("native geometry/dose gate failed")
        geometry = manifest["study"]["geometry"]
        validate_selected_setup(geometry, manifest["study"]["layouts"]["mid_dense_10_to_-60"])
        expected = ray_mount(geometry, require_level=True).copy()
        expected[2, 3] += geometry["base_height_m"]
        grouped_cases(report, manifest, expected, False, True)
        inputs = {str(native / n): sha(native / n) for n in ("manifest.json", "report.json")}
        root = Path(__file__).resolve().parents[1]
        sources = [
            Path(__file__),
            root / "tools/generic_lidar_adapter.py",
            root / "tools/assess_ray_terrain.py",
            root / "tools/local_terrain/ray_design.py",
            root / "tools/local_terrain/mounting_study.py",
            root / "tools/revision_evidence.py",
            root / "tools/archive_guard.py",
            SELECTED_SETUP,
            root / "tools/glim_shadow/native_input_probe.cpp",
        ]
        config_dir = root / "astrobot-lab/src/glim_slam_launch/config/glim_astrobot"
        sources += sorted(config_dir.glob("*.json"))
        hashes = {str(p): sha(p) for p in sources}
        for p in sources:
            target = output / "source" / p.relative_to(root)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, target)
        (output / "config").mkdir()
        for p in config_dir.glob("*.json"):
            payload = json.loads(p.read_text())
            if p.name == "config_sensors.json":
                payload["sensors"].update(
                    global_shutter_lidar=False,
                    autoconf_perpoint_times=False,
                    autoconf_prefer_frame_time=True,
                    perpoint_relative_time=True,
                    perpoint_time_scale=1.0,
                )
            with (output / "config" / p.name).open("x") as stream:
                json.dump(payload, stream, indent=2)
        jobs = []
        for case in report["cases"]:
            adapter = NativeWindow()
            for sequence, frame in enumerate(case["frames"]):
                path = (native / case["case_directory"] / frame["file"]).resolve(strict=True)
                if not path.is_relative_to(native) or sha(path) != frame["sha256"]:
                    raise ValueError("native evidence changed or escaped archive")
                inputs[str(path)] = frame["sha256"]
                with np.load(path, allow_pickle=False) as raw:
                    if float(raw["observed_stamp"]) != frame["stamp"]:
                        raise ValueError("native stamp association changed")
                    window = adapter.push(
                        sequence,
                        frame["stamp"],
                        raw["points"],
                        raw["depth"],
                        raw["intensity"],
                        raw["azimuth"],
                        raw["zenith"],
                    )
                if window is not None:
                    target = output / f"window_{len(jobs):03}.bin"
                    with target.open("xb") as stream:
                        stream.write(window.pop("data"))
                    jobs.append(
                        dict(
                            window,
                            file="/run/" + target.name,
                            sha256=sha(target),
                            fields=FIELDS,
                            native_case=case["case_directory"],
                        )
                    )
            adapter.finish()
        if len(jobs) != 96:
            raise ValueError("incomplete declared yaw-window set")
        with (output / "jobs.json").open("x") as stream:
            json.dump(jobs, stream, indent=2)
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
        # One Jazzy header prefix; no mission setup or mixed Space ROS exports.
        command = (
            "g++ -std=c++17 -O2 -I/opt/astrobot/glim_source/include/glim -I/usr/local/include "
            "-I/usr/include/eigen3 -I/opt/ros/jazzy/include/sensor_msgs -I/opt/ros/jazzy/include/std_msgs "
            "-I/opt/ros/jazzy/include/builtin_interfaces -I/opt/ros/jazzy/include/rosidl_runtime_cpp "
            "-I/opt/ros/jazzy/include/rosidl_typesupport_interface "
            "-I/opt/ros/jazzy/include/rcutils "
            "-I/opt/ros/jazzy/include/rosidl_runtime_c /run/source/tools/glim_shadow/native_input_probe.cpp "
            "-L/opt/astrobot/glim_source/lib -Wl,-rpath,/opt/astrobot/glim_source/lib -L/usr/local/lib "
            "-Wl,-rpath,/usr/local/lib -lglim -lgtsam_points -lspdlog -lfmt -o /run/probe && "
            "sha256sum /opt/astrobot/glim_source/lib/libglim.so "
            "/opt/astrobot/glim_source/include/glim/glim/util/ros_cloud_converter.hpp && ldd /run/probe"
        )
        with (output / "manifest.json").open("x") as stream:
            json.dump(
                dict(
                    protocol="generic_native_glim_input_v1",
                    source_sha256=hashes,
                    input_sha256=inputs,
                    config_sha256={p.name: sha(p) for p in (output / "config").glob("*.json")},
                    image=image,
                    build_command=command,
                    acceptance=False,
                    moving_qualified=False,
                    timing="native tick stamps rounded to ROS ns; no sub-tick times invented",
                    note=__doc__,
                ),
                stream,
                indent=2,
            )
        with (output / "build.log").open("x") as log:
            subprocess.run(base + ["-lc", command], stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)
        check_run(output)
        with (output / "probe.log").open("x") as log:
            subprocess.run(
                base + ["-lc", "/run/probe /run/config /run/jobs.json timed /run/converted.jsonl"],
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
                timeout=120,
            )
        with (output / "negative-control.log").open("x") as log:
            negative = subprocess.run(
                base
                + ["-lc", "/run/probe /run/config /run/jobs.json global-shutter-negative-control /run/negative.jsonl"],
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=30,
            )
        if (
            negative.returncode != 1
            or "preprocessing erased/changed native acquisition extent"
            not in (output / "negative-control.log").read_text()
        ):
            raise ValueError("global-shutter negative control did not reject erased acquisition times")
        records = [json.loads(line) for line in (output / "converted.jsonl").read_text().splitlines()]
        if len(records) != len(jobs) or not all(r["passed"] for r in records):
            raise ValueError("not every window passed GLIM input integration")
        if any(sha(p) != digest for p, digest in {**hashes, **inputs}.items()):
            raise ValueError("source/input changed")
        with (output / "report.json").open("x") as stream:
            json.dump(
                dict(
                    input_contract_passed=True,
                    windows=len(records),
                    negative_control_rejected=True,
                    input_points=sum(j["points"] for j in jobs),
                    output_points_min=min(r["output_points"] for r in records),
                    output_points_max=max(r["output_points"] for r in records),
                    probe_sha256=sha(output / "probe"),
                    acceptance=False,
                    moving_slam_qualified=False,
                ),
                stream,
                indent=2,
            )
        check_run(output)
        print((output / "report.json").read_text(), flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
