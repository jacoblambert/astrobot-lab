#!/usr/bin/env python3
"""Six isolated, zero-command loaded-rover cases; no ROS, GT-fed policy or driving."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time

from archive_guard import allocate_run, check_run
from local_terrain.support_cases import loaded_cases, summarize_case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--omnilrs", type=Path, default=Path("/workspace/omnilrs"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    root = Path(__file__).parent
    sources = [Path(__file__), root / "archive_guard.py", root / "local_terrain/support_cases.py"]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    for path in sources:
        target = output / "source" / path.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    app = None
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "width": 64, "height": 64})
        import numpy as np
        from isaacsim.core.api import World
        from isaacsim.core.prims import RigidPrim
        from omni.physx import get_physx_simulation_interface
        from pxr import Gf, PhysicsSchemaTools, Sdf, UsdGeom, UsdPhysics, UsdUtils

        asset = args.omnilrs / "assets/USD_Assets/robots/ros2_husky_PhysX_vlp16_mono_depth_imu.usd"
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
        world.get_physics_context().set_gravity(-1.62)
        cases = loaded_cases()
        disabled, paths = [], []
        for index, case in enumerate(cases):
            offset = np.array([index * 4.0, 0.0, 0.0])
            prefix = f"/World/case_{index}"
            for number, (low, high) in enumerate(case["boxes"]):
                low, high = np.asarray(low) + offset, np.asarray(high) + offset
                shape = UsdGeom.Cube.Define(world.stage, f"{prefix}/surface_{number}")
                shape.CreateSizeAttr(2.0)
                shape.AddTranslateOp().Set(Gf.Vec3d(*map(float, (low + high) / 2)))
                shape.AddScaleOp().Set(Gf.Vec3f(*map(float, (high - low) / 2)))
                UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            robot_root = prefix + "/robot"
            robot = UsdGeom.Xform.Define(world.stage, robot_root)
            if not robot.GetPrim().GetReferences().AddReference(str(asset)):
                raise ValueError("missing original rover asset")
            robot.ClearXformOpOrder()
            robot.AddTranslateOp().Set(Gf.Vec3d(float(offset[0]), 0.0, 0.25))
            wheels, bases, joints = [], [], []
            for prim in list(world.stage.Traverse()):
                if not str(prim.GetPath()).startswith(robot_root + "/"):
                    continue
                if prim.GetTypeName() in ("OmniGraph", "Lidar", "Camera"):
                    disabled.append(str(prim.GetPath()))
                if prim.IsA(UsdPhysics.RevoluteJoint) and "wheel" in prim.GetName():
                    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                    if not drive:
                        raise ValueError("missing original angular drive")
                    drive.GetTargetVelocityAttr().Set(0.0)
                    drive.GetStiffnessAttr().Set(0.0)
                    drive.GetDampingAttr().Set(10000.0)
                    joints.append(str(prim.GetPath()))
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    if "wheel" in prim.GetName():
                        wheels.append(str(prim.GetPath()))
                    if prim.GetName() == "base_link":
                        bases.append(str(prim.GetPath()))
            if len(wheels) != 4 or len(bases) != 1 or len(joints) != 4:
                raise ValueError("unexpected rover structure")
            paths.append([bases[0], *sorted(wheels)])
            case["world_offset"] = offset.tolist()
            case["body_paths"] = paths[-1]
        for path in sorted(disabled, key=lambda p: p.count("/"), reverse=True):
            world.stage.GetPrimAtPath(path).SetActive(False)
        layer_hashes = {
            layer.realPath: hashlib.sha256(Path(layer.realPath).read_bytes()).hexdigest()
            for layer in world.stage.GetUsedLayers()
            if layer.realPath and Path(layer.realPath).is_file()
        }
        manifest = dict(
            protocol="isolated_loaded_support_cases_v1",
            cases=cases,
            source_sha256=hashes,
            layer_sha256=layer_hashes,
            gravity_m_s2=-1.62,
            steps=480,
            physics_dt=1 / 60,
            settling_summary_last_ticks=60,
            wake_base_each_tick=True,
            disable_stabilization=False,
            drive=dict(stiffness=0.0, damping=10000.0, target_velocity=0.0),
            acceptance=False,
        )
        with (output / "manifest.json").open("x") as stream:
            json.dump(manifest, stream, indent=2)
        views = []
        for index, case_paths in enumerate(paths):
            group = []
            for number, path in enumerate(case_paths):
                view = RigidPrim(
                    prim_paths_expr=path,
                    name=f"case_{index}_body_{number}",
                    reset_xform_properties=False,
                    track_contact_forces=True,
                    disable_stablization=False,
                )
                world.scene.add(view)
                group.append(view)
            views.append(group)
        world.reset()
        simulation = get_physx_simulation_interface()
        stage_id = UsdUtils.StageCache.Get().GetId(world.stage).ToLongInt()
        ids = [PhysicsSchemaTools.sdfPathToInt(Sdf.Path(p[0])) for p in paths]
        rows = [[] for _ in cases]
        started = time.monotonic()
        with (output / "states.jsonl").open("x") as stream:
            for step in range(480):
                if time.monotonic() - started > 60:
                    raise ValueError("bounded fixture physics deadline")
                for body in ids:
                    simulation.wake_up(stage_id, body)
                world.step(render=False)
                for index, group in enumerate(views):
                    positions, orientations, forces, velocities = [], [], [], []
                    for view in group:
                        position, orientation = view.get_world_poses()
                        positions.append(np.asarray(position)[0] - cases[index]["world_offset"])
                        orientations.append(np.asarray(orientation)[0])
                        forces.append(np.asarray(view.get_net_contact_forces(dt=1 / 60))[0])
                        velocities.append(np.asarray(view.get_velocities())[0])
                    if not all(np.isfinite(v).all() for v in (positions, orientations, forces, velocities)):
                        raise ValueError("nonfinite physics state")
                    if np.linalg.norm(positions[0][:2]) > 1.0 or not -0.8 < positions[0][2] < 1.0:
                        raise ValueError("isolated fixture containment guard")
                    row = dict(
                        case=cases[index]["name"],
                        step=step + 1,
                        positions=np.asarray(positions).tolist(),
                        orientations_wxyz=np.asarray(orientations).tolist(),
                        contact_forces_n=np.asarray(forces).tolist(),
                        velocities=np.asarray(velocities).tolist(),
                    )
                    stream.write(json.dumps(row) + "\n")
                    rows[index].append(row)
                if step % 60 == 0:
                    stream.flush()
        results = {c["name"]: summarize_case(r) for c, r in zip(cases, rows)}
        control = results["flat_control"]
        if control["settled_max_speed_m_s"] >= 0.005 or min(control["mean_wheel_up_force_n"]) <= 0.1:
            raise ValueError("flat loaded baseline failed")
        if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != h for p, h in {**hashes, **layer_hashes}.items()):
            raise ValueError("fixture source/asset changed")
        check_run(output)
        report = dict(
            fixture_passed=True,
            acceptance=False,
            rover_qualified=False,
            cases=results,
            elapsed_physics_wall_s=time.monotonic() - started,
            states_sha256=hashlib.sha256((output / "states.jsonl").read_bytes()).hexdigest(),
            limits=["stationary only", "no LiDAR or GLIM", "no contact area or safe-step qualification"],
        )
        with (output / "report.json").open("x") as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps(report), flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
