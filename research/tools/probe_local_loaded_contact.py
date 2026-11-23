#!/usr/bin/env python3
"""Isolated zero-command rover settling diagnostic, NOT driving qualification.

Known planar floor, lunar gravity, original physical asset, command/sensor graphs
disabled. No ROS, velocity commands, terrain labels or mission stack. Eight
simulated seconds maximum; displacement and finite-state guards remain active.
This measures loaded transforms/contact forces, not a tire contact patch or
braking/step/tipping envelope. Requires archive and image CUDA preflight.
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
    parser.add_argument("--omnilrs", type=Path, default=Path("/workspace/omnilrs"))
    parser.add_argument(
        "--awake", action="store_true", help="explicit force-characterization protocol; wake base each tick"
    )
    parser.add_argument("--contact-points", action="store_true", help="record floor contact manifold; not area truth")
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    asset = args.omnilrs / "assets/USD_Assets/robots/ros2_husky_PhysX_vlp16_mono_depth_imu.usd"
    sources = [Path(__file__), Path(__file__).with_name("archive_guard.py")]
    if args.contact_points:
        sources.append(Path(__file__).with_name("contact_manifold.py"))
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    for path in sources:
        shutil.copyfile(path, output / path.name)
    app = None
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "width": 64, "height": 64})
        import numpy as np
        from isaacsim.core.api import World
        from isaacsim.core.prims import RigidPrim
        from omni.physx import get_physx_simulation_interface
        from pxr import Gf, PhysicsSchemaTools, Sdf, UsdGeom, UsdPhysics, UsdUtils

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
        world.get_physics_context().set_gravity(-1.62)
        floor = UsdGeom.Cube.Define(world.stage, "/World/floor")
        floor.CreateSizeAttr(2.0)
        floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.1))
        floor.AddScaleOp().Set(Gf.Vec3f(5, 5, 0.1))
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        robot = UsdGeom.Xform.Define(world.stage, "/World/robot")
        if not robot.GetPrim().GetReferences().AddReference(str(asset)):
            raise ValueError("robot asset reference failed")
        robot.ClearXformOpOrder()
        robot.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.25))
        disabled, joints, wheels, bases = [], [], [], []
        # Read all prim handles before deactivating any parent: deactivation
        # expires descendant handles even if traversal was copied into a list.
        for prim in list(world.stage.Traverse()):
            if not str(prim.GetPath()).startswith("/World/robot/"):
                continue
            if prim.GetTypeName() in ("OmniGraph", "Lidar", "Camera"):
                disabled.append(str(prim.GetPath()))
            if prim.IsA(UsdPhysics.RevoluteJoint) and "wheel" in prim.GetName():
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                if not drive:
                    raise ValueError("missing wheel angular drive")
                # Explicit stationary fixture; same zero-stiffness / 10000
                # damping as lunaryard_40m, NOT a motion-controller experiment.
                drive.GetTargetVelocityAttr().Set(0.0)
                drive.GetStiffnessAttr().Set(0.0)
                drive.GetDampingAttr().Set(10000.0)
                joints.append(str(prim.GetPath()))
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                if "wheel" in prim.GetName():
                    wheels.append(str(prim.GetPath()))
                if prim.GetName() == "base_link":
                    bases.append(str(prim.GetPath()))
        if len(wheels) != 4 or len(joints) != 4 or len(bases) != 1 or not disabled:
            raise ValueError("unexpected rover rigid-body/drive/graph structure")
        for path in sorted(disabled, key=lambda p: p.count("/"), reverse=True):
            world.stage.GetPrimAtPath(path).SetActive(False)
        layer_hashes = {
            layer.realPath: hashlib.sha256(Path(layer.realPath).read_bytes()).hexdigest()
            for layer in world.stage.GetUsedLayers()
            if layer.realPath and Path(layer.realPath).is_file()
        }
        manifest = dict(
            source_sha256=hashes,
            asset_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
            layer_sha256=layer_hashes,
            disabled=disabled,
            joints=joints,
            wheels=sorted(wheels),
            base=bases[0],
            gravity_m_s2=-1.62,
            physics_dt=1 / 60,
            max_steps=480,
            max_base_xy_displacement_m=0.05,
            protocol="isolated_zero_command_loaded_contact_v2",
            wake_base_each_tick=args.awake,
            detailed_floor_contacts=args.contact_points,
            contact_buffer_capacity=256 if args.contact_points else 0,
            instrumentation="RigidPrim contact tracking sets per-body sleep threshold zero; articulation sleep is separate",
            disable_stabilization=False,
            drive=dict(stiffness=0.0, damping=10000.0, target_velocity=0.0),
            acceptance=False,
        )
        with (output / "manifest.json").open("x") as stream:
            json.dump(manifest, stream, indent=2)
        views = []
        for index, path in enumerate([bases[0], *sorted(wheels)]):
            view = RigidPrim(
                prim_paths_expr=path,
                name=f"contact_{index}",
                reset_xform_properties=False,
                track_contact_forces=True,
                disable_stablization=False,
                **(
                    dict(contact_filter_prim_paths_expr=["/World/floor"], max_contact_count=256)
                    if args.contact_points
                    else {}
                ),
            )
            world.scene.add(view)
            views.append(view)
        world.reset()
        simulation = get_physx_simulation_interface()
        stage_id = UsdUtils.StageCache.Get().GetId(world.stage).ToLongInt()
        body_ids = [PhysicsSchemaTools.sdfPathToInt(Sdf.Path(p)) for p in [bases[0], *sorted(wheels)]]
        initial = np.asarray(views[0].get_world_poses()[0])[0].copy()
        rows = []
        start = time.monotonic()
        with (output / "contacts.jsonl").open("x") as stream:
            for step in range(480):
                if time.monotonic() - start > 60:
                    raise ValueError("stationary contact fixture wall deadline")
                if args.awake:
                    simulation.wake_up(stage_id, body_ids[0])
                world.step(render=False)
                positions, orientations, forces, velocities = [], [], [], []
                for view in views:
                    position, orientation = view.get_world_poses()
                    positions.append(np.asarray(position)[0])
                    orientations.append(np.asarray(orientation)[0])
                    forces.append(np.asarray(view.get_net_contact_forces(dt=1 / 60))[0])
                    velocities.append(np.asarray(view.get_velocities())[0])
                if not all(np.isfinite(value).all() for value in (positions, orientations, forces, velocities)):
                    raise ValueError("nonfinite active loaded state")
                row = dict(
                    step=step + 1,
                    positions=np.asarray(positions).tolist(),
                    orientations_wxyz=np.asarray(orientations).tolist(),
                    contact_forces_n=np.asarray(forces).tolist(),
                    velocities=np.asarray(velocities).tolist(),
                    sleeping=[simulation.is_sleeping(stage_id, body_id) for body_id in body_ids],
                )
                if args.contact_points:
                    from contact_manifold import decode_contacts

                    row["floor_contacts"] = [
                        decode_contacts(view.get_contact_force_data(dt=1 / 60), capacity=256) for view in views
                    ]
                stream.write(json.dumps(row) + "\n")
                stream.flush()
                rows.append(row)
                if np.linalg.norm(positions[0][:2] - initial[:2]) > 0.05 or not -0.2 < positions[0][2] < 1:
                    raise ValueError("stationary loaded-pose safety guard")
        final = rows[-60:]  # Declared settling interval; ALL startup rows retained.
        speed = max(np.linalg.norm(np.asarray(r["velocities"])[0, :3]) for r in final)
        support = np.asarray([r["contact_forces_n"] for r in final])[:, 1:, 2]
        passed = bool(speed < 0.005 and np.all(np.mean(support, axis=0) > 0.1))
        check_run(output)
        for path, digest in {**hashes, **layer_hashes}.items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
                raise ValueError("contact fixture source/asset changed")
        report = dict(
            fixture_passed=passed,
            acceptance=False,
            rover_qualified=False,
            final_second_max_base_speed_m_s=float(speed),
            mean_wheel_up_force_n=np.mean(support, axis=0).tolist(),
            mean_base_height_m=float(np.mean([r["positions"][0][2] for r in final])),
            mean_wheel_height_m=np.mean([np.asarray(r["positions"])[1:, 2] for r in final], axis=0).tolist(),
            frames=len(rows),
            wake_base_each_tick=args.awake,
            final_sleeping=rows[-1]["sleeping"],
            contacts_sha256=hashlib.sha256((output / "contacts.jsonl").read_bytes()).hexdigest(),
            wall_s=time.monotonic() - start,
            unresolved=["contact patch extent", "braking", "safe attitude/step", "natural active terrain"],
        )
        if args.contact_points:
            from contact_manifold import summarize_contacts

            report["contact_manifold"] = summarize_contacts(final)
            if any(w["missing_contact_frames"] for w in report["contact_manifold"]["wheels"]):
                raise ValueError("missing loaded wheel manifold in declared settled interval")
        with (output / "report.json").open("x") as stream:
            json.dump(report, stream, indent=2)
        if not passed:
            raise ValueError("stationary support or settling contract failed")
        print(f"LOADED_CONTACT PASSED {output}", flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
