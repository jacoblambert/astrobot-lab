#!/usr/bin/env python3
"""Record source/config/binary provenance inside the ROS execution environment.

Run after stack startup, before evaluation. Does not mutate ROS parameters or
record arbitrary environment variables. Existing manifests are never overwritten.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess


EXECUTABLES = {"glim_rosnode", "probabilistic_voxel_mapper_node", "slam_map_to_odom_tf_node",
               "localization_supervisor", "command_mux", "exploration_manager", "synthetic_rtk",
               "controller_server", "planner_server", "bt_navigator", "behavior_server"}


def run(*args, cwd=None):
    result = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=20)
    return {"returncode": result.returncode, "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}


def fingerprint(path):
    path = Path(path)
    with path.open("rb") as source:
        digest = hashlib.sha256()
        for block in iter(lambda: source.read(1024*1024), b""):
            digest.update(block)
    return {"path": str(path), "resolved_path": str(path.resolve()), "size": path.stat().st_size,
            "sha256": digest.hexdigest()}


def configuration(path):
    return {**fingerprint(path), "content": Path(path).read_text()}


def mapped_library_paths(maps):
    """Unique file-backed shared libraries; never silently attest deleted code."""
    paths = set()
    for line in maps.splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6 or not fields[5].startswith("/") or ".so" not in fields[5]:
            continue
        if fields[5].endswith(" (deleted)"):
            raise ValueError("process maps a deleted shared library")
        paths.add(fields[5])
    return sorted(paths)


def loaded_libraries(proc):
    before = (proc / "maps").read_text()
    paths = mapped_library_paths(before)
    records = [fingerprint(path) for path in paths]
    if mapped_library_paths((proc / "maps").read_text()) != paths:
        raise ValueError("loaded library set changed during provenance collection")
    return dict(maps_sha256=hashlib.sha256(before.encode()).hexdigest(), libraries=records)


def collect(repo, seed, launch_command):
    result = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
              "seed": seed, "launch_command": launch_command, "repo": str(repo),
              "git": run("git", "rev-parse", "HEAD", cwd=repo),
              "worktree": run("git", "status", "--porcelain=v1", cwd=repo),
              "submodules": {}, "configurations": [], "processes": [], "python_modules": {}, "source_files": [],
              "environment": {k: os.environ.get(k) for k in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
                  "CYCLONEDDS_URI", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH", "PYTHONPATH")}}
    for relative in ("OmniLRS", "astrobot-lab/src/glim", "astrobot-lab/src/glim_ros2"):
        result["submodules"][relative] = {"head": run("git", "rev-parse", "HEAD", cwd=repo/relative),
                                         "worktree": run("git", "status", "--porcelain=v1", cwd=repo/relative)}
    for relative in ("astrobot-lab/src/astrobot_launch/config", "astrobot-lab/src/glim_slam_launch/config"):
        for path in sorted((repo/relative).rglob("*")):
            if path.is_file() and path.suffix in (".yaml", ".json"):
                result["configurations"].append(configuration(path))
    for package in ("astrobot_core", "localization_supervisor", "basic_control", "exploration_manager",
                    "synthetic_rtk", "slam_mapping", "astrobot_launch", "glim", "glim_ros2"):
        for path in sorted((repo/"astrobot-lab/src"/package).rglob("*")):
            if path.is_file() and path.suffix in (".py", ".cpp", ".hpp", ".h", ".xml"):
                result["source_files"].append(fingerprint(path))
    for name in ("localization_supervisor.node", "basic_control.command_mux", "exploration_manager.exploration_manager_node",
                 "astrobot_core.integrity", "synthetic_rtk.synthetic_rtk_node"):
        try:
            spec = importlib.util.find_spec(name)
            result["python_modules"][name] = fingerprint(spec.origin) if spec else None
        except (ImportError, ModuleNotFoundError) as exc:
            result["python_modules"][name] = {"error": str(exc)}
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc/"cmdline").read_bytes().decode().strip("\0").split("\0")
            if not argv or not any(Path(arg).name in EXECUTABLES for arg in argv[:2]):
                continue
            rec = {"pid": int(proc.name), "argv": argv, "executable": fingerprint(proc/"exe"), "runtime_files": []}
            if any(Path(arg).name == "glim_rosnode" for arg in argv[:2]):
                rec["loaded_code"] = loaded_libraries(proc)
            for index, arg in enumerate(argv):
                if arg == "--params-file" and index+1 < len(argv):
                    rec["runtime_files"].append(configuration(argv[index+1]))
                if arg.startswith("config_path:="):
                    for config in sorted(Path(arg.split(":=", 1)[1]).glob("*.json")):
                        rec["runtime_files"].append(configuration(config))
            result["processes"].append(rec)
        except (OSError, UnicodeError) as exc:
            # Processes can exit between enumeration and inspection.
            result.setdefault("process_scan_errors", []).append({"pid": proc.name, "error": str(exc)})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--launch-command", required=True)
    args = parser.parse_args()
    result = collect(args.repo.resolve(), args.seed, args.launch_command)
    with args.output.open("x") as output:
        json.dump(result, output, indent=2)
        output.write("\n")
    print(args.output)


if __name__ == "__main__":
    main()
