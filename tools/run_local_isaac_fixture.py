#!/usr/bin/env python3
"""Bounded isolated Isaac fixtures with archive/GPU preflight and result checks.

No mission containers, ROS networking, drivers or host hardware are modified.
Isaac can exit zero after exceptions during shutdown; its inner evidence is
mandatory. Every attempt receives a fresh outer archive log and inner run.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import time

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run

FIXTURES = {
    "body": "probe_local_physx_reference.py",
    "optical": "probe_lidar_optical_origin.py",
    "contact": "probe_local_loaded_contact.py",
    "contact_awake": "probe_local_loaded_contact.py",
    "contact_points": "probe_local_loaded_contact.py",
    "support_hazards": "probe_support_hazards.py",
    "mounting": "probe_local_mounting.py",
    "ray_design": "probe_local_ray_design.py",
    "ray_terrain_views": "probe_local_ray_design.py",
    "ray_terrain_yaw": "probe_local_ray_design.py",
    "native_precision": "probe_native_precision.py",
}


def verify_result(archive, log, exit_code):
    archive = Path(archive).resolve(strict=True)
    names = re.findall(r"^Run: /archive/(revision-[a-f0-9]+)$", log, flags=re.MULTILINE)
    if len(names) != 1:
        raise ValueError("missing or ambiguous inner fixture run")
    inner = (archive / names[0]).resolve(strict=True)
    if not inner.is_relative_to(archive) or inner.parent != archive:
        raise ValueError("inner fixture escaped archive")
    if exit_code != 0 or (inner / "failure.json").exists():
        raise ValueError("fixture failed; exit zero alone is not success")
    report = json.loads((inner / "report.json").read_text())
    manifest = inner / "manifest.json"
    if report.get("fixture_passed") is not True or report.get("acceptance") is not False:
        raise ValueError("missing successful diagnostic report")
    if report.get("rover_qualified") is not False or not manifest.is_file():
        raise ValueError("fixture cannot claim rover qualification")
    return dict(
        fixture_passed=True,
        inner_run=str(inner),
        report_sha256=hashlib.sha256((inner / "report.json").read_bytes()).hexdigest(),
        manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
        acceptance=False,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", choices=FIXTURES)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--study-run", type=Path, help="completed screen; required for mounting/ray_design fixtures")
    parser.add_argument("--rejection-run", type=Path, help="failed native capture; precision diagnostic only")
    args = parser.parse_args()
    if (args.fixture == "native_precision") != (args.rejection_run is not None):
        parser.error("--rejection-run is required exactly for native_precision")
    if (args.fixture in ("mounting", "ray_design", "ray_terrain_views", "ray_terrain_yaw")) != (
        args.study_run is not None
    ):
        parser.error("--study-run is required exactly for mounting/ray fixtures")
    output = allocate_run(args.archive_root, limit=1024**3)
    archive = args.archive_root.resolve(strict=True)
    root = Path(__file__).resolve().parents[1]
    image = "isaac-sim-omnilrs:latest"
    name = "astrobot-fixture-" + output.name
    print(f"Launch: {output}", flush=True)
    sources = [Path(__file__), root / "tools/archive_guard.py", root / "docker/check_cuda_runtime.py"]
    for path in sources:
        shutil.copyfile(path, output / path.name)
    command = [
        "docker",
        "run",
        "--rm",
        "--network",
        "none",
        "--gpus",
        "all",
        "--name",
        name,
        "-e",
        "ACCEPT_EULA=Y",
        "-e",
        "PRIVACY_CONSENT=Y",
        "--mount",
        f"type=bind,source={archive},target=/archive",
        "--mount",
        f"type=bind,source={root}/tools,target=/workspace/terrain-tools,readonly",
        "--mount",
        f"type=bind,source={root}/OmniLRS,target=/workspace/omnilrs,readonly",
        "--entrypoint",
        "/isaac-sim/python.sh",
        image,
        "/workspace/terrain-tools/" + FIXTURES[args.fixture],
    ]
    if args.fixture in ("contact_awake", "contact_points"):
        command.append("--awake")
    if args.fixture == "contact_points":
        command.append("--contact-points")
    if args.fixture == "ray_terrain_views":
        command.append("--multiview")
    if args.fixture == "ray_terrain_yaw":
        command.append("--observation-yaw")
    preflight = [
        ["nvidia-smi"],
        ["python3", str(root / "docker/check_cuda_runtime.py")],
        [
            "docker",
            "run",
            "--rm",
            "--network",
            "none",
            "--gpus",
            "all",
            "--mount",
            f"type=bind,source={root}/docker/check_cuda_runtime.py,target=/check.py,readonly",
            "--entrypoint",
            "/isaac-sim/python.sh",
            image,
            "/check.py",
        ],
    ]
    try:
        if args.rejection_run is not None:
            rejected = args.rejection_run.resolve(strict=True)
            if rejected.parent != archive or not (rejected / "rejected_native_recast.json").is_file():
                raise ValueError("precision diagnostic requires archived same-tick rejection recast")
            command.extend(["--rejection-run", "/archive/" + rejected.name])
        if args.study_run is not None:
            study = args.study_run.resolve(strict=True)
            if study.parent != archive or (study / "failure.json").exists():
                raise ValueError("mounting study must be a completed run directly on the archive")
            command.extend(["--study-run", "/archive/" + study.name])
        running = subprocess.check_output(["docker", "ps", "--format", "{{.Names}}"], text=True, timeout=10)
        if any(re.search(r"astrobot|omnilrs|isaac", row, re.IGNORECASE) for row in running.splitlines()):
            raise ValueError("stop the existing task stack/fixture before an isolated fixture")
        image_id = subprocess.check_output(
            ["docker", "image", "inspect", "--format", "{{.Id}}", image], text=True, timeout=10
        ).strip()
        if not re.fullmatch(r"sha256:[a-f0-9]{64}", image_id):
            raise ValueError("cannot freeze the local Isaac image identity")
        command[command.index(image)] = image_id
        preflight[-1][preflight[-1].index(image)] = image_id
        with (output / "launch.json").open("x") as stream:
            json.dump(
                dict(
                    command=command,
                    preflight=preflight,
                    image_tag=image,
                    image_id=image_id,
                    source_sha256={p.name: hashlib.sha256((output / p.name).read_bytes()).hexdigest() for p in sources},
                    timeout_s=240,
                    acceptance=False,
                ),
                stream,
                indent=2,
            )
        with (output / "preflight.log").open("x") as log:
            for check in preflight:
                subprocess.run(check, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=30)
        check_run(output)
        start = time.monotonic()
        with (output / "container.log").open("x") as log:
            try:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=240)
            except subprocess.TimeoutExpired:
                # This unique name belongs solely to the just-launched fixture.
                subprocess.run(
                    ["docker", "stop", "--time", "10", name],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=30,
                    check=True,
                )
                raise
        with (output / "exit.json").open("x") as stream:
            json.dump(dict(exit_code=result.returncode, elapsed_s=time.monotonic() - start), stream)
        checked = verify_result(archive, (output / "container.log").read_text(), result.returncode)
        check_run(output)
        with (output / "result.json").open("x") as stream:
            json.dump(checked, stream, indent=2)
        print(json.dumps(checked), flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
