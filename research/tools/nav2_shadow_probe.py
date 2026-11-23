"""Subprocess adapter for installed Nav2 primitives, never a live costmap."""
import json
import os
from pathlib import Path
import subprocess

import numpy as np


def probe(executable, costs, poses, *, origin=(-2., -2.), resolution=.1, motions=None):
    costs, poses = np.asarray(costs), np.asarray(poses, dtype=float)
    motions = np.empty((0, 6)) if motions is None else np.asarray(motions, dtype=float)
    if (costs.ndim != 2 or max(costs.shape) > 200 or min(costs.shape) < 1
            or not np.isin(costs, [-1, 0, 20, 100]).all() or poses.ndim != 2
            or poses.shape[1] != 3 or len(poses) > 50000 or not np.isfinite(poses).all()
            or np.asarray(origin).shape != (2,) or not np.isfinite(origin).all() or resolution != .1
            or motions.ndim != 2 or motions.shape[1] != 6 or len(motions) > 1000
            or not np.isfinite(motions).all()):
        raise ValueError("invalid shadow grid/pose input")
    height, width = costs.shape
    data = f"{width} {height} {resolution} {origin[0]} {origin[1]}\n"
    data += " ".join(map(str, costs.astype(int).ravel())) + f"\n{len(poses)}\n"
    data += "\n".join(" ".join(map(str, row)) for row in poses) + "\n"
    data += str(len(motions)) + "\n" + "\n".join(" ".join(map(str, row)) for row in motions) + "\n"
    env = dict(os.environ, ROS_DOMAIN_ID="179", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
               CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces>'
               '<NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast>'
               '</General></Domain></CycloneDDS>')
    # Archive runs use the host UID; the container's default ROS log directory
    # may be root-owned. This standalone probe needs stderr, not global log files.
    completed = subprocess.run([str(Path(executable).resolve(strict=True)), "--ros-args", "--log-level", "error",
                                "--disable-external-lib-logs", "--disable-rosout-logs"],
                               input=data, text=True, capture_output=True, env=env, timeout=30, check=False)
    if completed.returncode:
        raise RuntimeError(f"Nav2 probe exit {completed.returncode}: {completed.stderr[-4000:]}")
    lines = [line for line in completed.stdout.splitlines() if line.startswith('{"inflated":')]
    if len(lines) != 1:
        raise ValueError("Nav2 probe did not emit one complete result")
    result = json.loads(lines[0])
    result["inflated"] = np.asarray(result["inflated"], dtype=np.uint8).reshape(costs.shape)
    if len(result["queries"]) != len(poses):
        raise ValueError("missing footprint query results")
    if len(result.get("motions", [])) != len(motions):
        raise ValueError("missing swept-footprint results; rebuild the standalone probe")
    return result


def rectangle_intersects_segment(center, yaw, a, b, half=(.595, .435)):
    """Independent continuous XY fixture collision oracle, not mapper input."""
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    start, end = (np.asarray([a, b]) - np.asarray(center)) @ rotation
    direction = end - start
    lower, upper = 0., 1.
    for axis in range(2):
        if abs(direction[axis]) < 1e-12:
            if abs(start[axis]) > half[axis]:
                return False
        else:
            ends = sorted(((-half[axis] - start[axis]) / direction[axis],
                           (half[axis] - start[axis]) / direction[axis]))
            lower, upper = max(lower, ends[0]), min(upper, ends[1])
            if lower > upper:
                return False
    return True
