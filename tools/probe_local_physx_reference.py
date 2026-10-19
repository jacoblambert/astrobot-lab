#!/usr/bin/env python3
"""Isolated Isaac active-collision reference fixture; no ROS, rover or commands.

Run only after NVIDIA/CUDA preflight in this image, with the opt-in archive bind.
This validates the scene-query adapter on analytic boxes, not rover qualification.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time

from archive_guard import allocate_run, check_run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument(
        "--asset",
        type=Path,
        default=Path("/workspace/omnilrs/assets/USD_Assets/robots/ros2_husky_PhysX_vlp16_mono_depth_imu.usd"),
    )
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    paths = [
        Path(__file__),
        Path(__file__).with_name("archive_guard.py"),
        Path(__file__).with_name("revision_evidence.py"),
        Path(__file__).with_name("local_terrain") / "physx_reference.py",
    ]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    asset_hash = hashlib.sha256(args.asset.read_bytes()).hexdigest()
    for path in paths:
        target = output / "source" / path.relative_to(Path(__file__).parent)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    with (output / "manifest.json").open("x") as stream:
        json.dump(
            dict(
                source_sha256=hashes,
                asset=str(args.asset),
                asset_sha256=asset_hash,
                protocol="isolated_static_boxes_v3",
                acceptance=False,
            ),
            stream,
            indent=2,
        )
    app = None
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "width": 64, "height": 64})
        import numpy as np
        from isaacsim.core.api import World
        from omni.physx import get_physx_scene_query_interface
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
        from scipy.spatial.transform import Rotation
        from local_terrain.physx_reference import BodyBox, body_clearance

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
        world.get_physics_context().set_gravity(-1.62)

        def cube(path, center, half):
            prim = UsdGeom.Cube.Define(world.stage, path)
            prim.CreateSizeAttr(2.0)
            prim.AddTranslateOp().Set(Gf.Vec3d(*center))
            prim.AddScaleOp().Set(Gf.Vec3f(*half))
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())

        cube("/World/floor", [0.0, 0.0, -0.1], [5.0, 5.0, 0.1])
        cube("/World/thin_wall", [1.0, 0.0, 0.5], [0.025, 0.5, 0.5])
        cube("/World/robot/body", [-3.0, 0.0, 0.5], [0.1, 0.1, 0.1])
        cube("/World/robot_other/body", [-3.0, 1.0, 0.5], [0.1, 0.1, 0.1])
        cube("/World/rotation_marker", [-1.0, 2.45, 0.5], [0.02, 0.02, 0.1])
        # Preserve the actual asset's Cube size, authored extent, scale and
        # collision flags. Its cached extent may disagree with primitive size.
        source_stage = Usd.Stage.Open(str(args.asset))
        parent = UsdGeom.Xform.Define(world.stage, "/World/AssetFrame")
        parent.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))
        asset_body = "/World/AssetFrame/body"
        if not Sdf.CopySpec(
            source_stage.GetRootLayer(), "/husky_robot/base_link/collisions", world.stage.GetRootLayer(), asset_body
        ):
            raise ValueError("actual body collision prim copy failed")
        body = UsdGeom.Cube(world.stage.GetPrimAtPath(asset_body))
        with (output / "body_asset.json").open("x") as stream:
            json.dump(
                dict(
                    size=body.GetSizeAttr().Get(),
                    authored_extent=[list(v) for v in body.GetExtentAttr().Get()],
                    local_transform=[list(v) for v in body.GetLocalTransformation()],
                    source_sha256=asset_hash,
                ),
                stream,
                indent=2,
            )
        world.reset()
        for _ in range(5):
            world.step(render=False)
        query = get_physx_scene_query_interface()
        box = BodyBox((0.0, 0.0, 0.0), (0.1, 0.1, 0.1), "isolated_exact_box_v1")
        token = lambda: ("static_boxes_v3", int(world.current_time_step_index))

        def pose(x, y=0.0, z=0.5):
            value = np.eye(4)
            value[:3, 3] = [x, y, z]
            return value

        definitions = [
            ("clear", [pose(0), pose(0.5)], True, []),
            ("midway_wall", [pose(0), pose(2)], False, ["/World/thin_wall"]),
            ("initial_overlap", [pose(1)], False, ["/World/thin_wall"]),
            ("floor_not_excluded", [pose(0, z=0)], False, ["/World/floor"]),
            ("self_excluded", [pose(-3)], True, []),
            ("other_robot_retained", [pose(-3, 1)], False, ["/World/robot_other/body"]),
        ]
        results = []
        definitions = [(name, poses, [box], clear, collisions) for name, poses, clear, collisions in definitions]
        long_box = BodyBox((0, 0, 0), (0.5, 0.05, 0.05), "isolated_exact_long_box_v2")
        first, middle, last = [pose(-1, 2) for _ in range(3)]
        middle[:3, :3] = Rotation.from_euler("z", np.pi / 2).as_matrix()
        last[:3, :3] = Rotation.from_euler("z", np.pi).as_matrix()
        definitions.extend(
            [
                ("long_box_unrotated_clear", [first], [long_box], True, []),
                ("long_box_yaw_quaternion", [middle], [long_box], False, ["/World/rotation_marker"]),
                ("rotation_interior_collision", [first, last], [long_box], False, ["/World/rotation_marker"]),
            ]
        )
        probe = BodyBox((0, 0, 0), (0.003, 0.003, 0.003), "tiny_asset_volume_probe_v3")
        definitions.extend(
            [
                ("asset_body_y_outside_cached_extent", [pose(0, 0.25, 1.125)], [probe], False, [asset_body]),
                ("asset_body_z_below_cached_extent", [pose(0, 0, 1.04)], [probe], False, [asset_body]),
                ("asset_body_y_outside_primitive", [pose(0, 0.31, 1.125)], [probe], True, []),
                ("asset_body_z_below_primitive", [pose(0, 0, 0.98)], [probe], True, []),
            ]
        )
        for name, poses, boxes, clear, collisions in definitions:
            start = time.perf_counter()
            result = body_clearance(query, poses, boxes, scene_token=token, robot_root="/World/robot")
            result.update(case=name, wall_ms=(time.perf_counter() - start) * 1000)
            results.append(result)
            with (output / f"{name}.json").open("x") as stream:
                json.dump(result, stream, indent=2)
            if result["body_envelope_clear"] != clear or result["possible_collisions"] != collisions:
                raise ValueError("active collision fixture mismatch: " + name)
        check_run(output)
        if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest for p, digest in hashes.items()):
            raise ValueError("reference fixture source changed")
        if hashlib.sha256(args.asset.read_bytes()).hexdigest() != asset_hash:
            raise ValueError("reference body asset changed")
        with (output / "report.json").open("x") as stream:
            json.dump(
                dict(fixture_passed=True, cases=results, acceptance=False, rover_qualified=False), stream, indent=2
            )
        print(f"LOCAL_PHYSX_REFERENCE PASSED {output}", flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
