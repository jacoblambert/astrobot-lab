#!/usr/bin/env python3
"""USD-only attachment audit; mesh bounds can disprove contact, not certify it.

No Isaac world/GPU is constructed and no asset/profile is changed. The selected
profile translates the entire vlp16 parent, including its original support mesh.
Report that visual support relative to actual mesh vertices, not stale extents.
Strength, fasteners and collision/optical suitability are NOT established.
"""

import argparse
import json
import os
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from replay_local_terrain import sha


def vertical_separation(mount, surfaces):
    """Positive value disproves attachment to all supplied XY-overlapping meshes."""
    mount = np.asarray(mount)
    tops = [
        np.asarray(s)[1, 2]
        for s in surfaces
        if np.all(mount[0, :2] <= np.asarray(s)[1, :2]) and np.all(np.asarray(s)[0, :2] <= mount[1, :2])
    ]
    return float(mount[0, 2] - max(tops)) if tops else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("asset", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(args.asset))
    cache = UsdGeom.XformCache()
    inverse = cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/husky_robot/base_link")).GetInverse()
    names = [
        "/husky_robot/base_link/visuals",
        "/husky_robot/top_plate_link/visuals",
        "/husky_robot/user_rail_link/visuals",
        "/husky_robot/base_link/vlp16/mount",
        "/husky_robot/base_link/vlp16/velodyne_vlp16/Geometry/Mesh_0",
    ]
    bounds = {}
    for name in names:
        prim = stage.GetPrimAtPath(name)
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim) * inverse).T
        vertices = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
        transformed = vertices @ matrix[:3, :3].T + matrix[:3, 3]
        bounds[name] = np.array([transformed.min(axis=0), transformed.max(axis=0)])
    profile = Path(__file__).parent / "local_terrain/selected_lidar_optical.json"
    selected = np.asarray(json.loads(profile.read_text())["base_from_lidar_translation_m"])
    mount_prim = stage.GetPrimAtPath("/husky_robot/base_link/vlp16")
    authored = np.asarray((cache.GetLocalToWorldTransform(mount_prim) * inverse).ExtractTranslation())
    support = bounds[names[3]]
    moved = support + selected - authored
    output = allocate_run(args.archive_root, limit=1024**2)
    sources = [
        Path(__file__),
        args.asset,
        profile,
        Path(__file__).parent.parent / "OmniLRS/src/environments_wrappers/ros2/natural_holdout_capture.py",
    ]
    report = dict(
        completed=True,
        acceptance=False,
        geometry_only=True,
        asset_changed=False,
        input_sha256={str(p): sha(p) for p in sources},
        authored_mesh_bounds={name: box.tolist() for name, box in bounds.items()},
        authored_sensor_xyz=authored.tolist(),
        selected_sensor_xyz=selected.tolist(),
        selected_support_bounds=moved.tolist(),
        selected_support_gap_above_body_plate_m=vertical_separation(moved, [bounds[n] for n in names[:3]]),
        interpretation="Positive gap disproves current modeled attachment; overlap would NOT prove a usable mount.",
        limits=__doc__,
    )
    (output / "report.json").write_text(json.dumps(report, indent=2))
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    check_run(output)
    # The image's Python installation is root-only. Only newly allocated owned
    # evidence is reassigned; repository/assets are mounted read-only.
    if os.getuid() == 0:
        for path in [*output.iterdir(), output]:
            os.chown(path, 1000, 1000)
    print(output)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
