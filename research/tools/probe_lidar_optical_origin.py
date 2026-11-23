#!/usr/bin/env python3
"""Stationary native/optical-origin audit of actual rover collision geometry.

Copies every active collider with its complete base-relative affine transform;
no dynamics, ROS, navigation, or commands. Original and candidate mounts/raycast
minimums are distinct cases. Existing acquisition/terrain gates are not changed.
"""

import hashlib
import json
from pathlib import Path

from archive_guard import allocate_run, check_run


def audit_passed(cases):
    """Execution alone is insufficient: both zero-origin raised controls must pass."""
    names = {row["case"] for row in cases}
    expected = {name + suffix for name in ("original", "raised_045", "raised_065") for suffix in ("_min05", "_min0")}
    if len(cases) != 6 or names != expected:
        return False
    for row in cases:
        if row["native_samples"] != 127932:
            return False
        if row["case"].startswith("raised_") and row["case"].endswith("_min0"):
            if (
                not row["geometry_passed"]
                or row["origin_overlaps"]
                or row["mask_mismatch"]
                or row["hidden_body_returns"]
                or row["max_depth_error_m"] > 0.001
                or row["valid_returns"] <= 0
            ):
                return False
        if row["case"].startswith("original") and not row["origin_overlaps"]:
            return False
    return True


def main():
    run = allocate_run("/archive", limit=2 * 1024**3)
    print(f"Run: {run}", flush=True)
    asset = Path("/workspace/omnilrs/assets/USD_Assets/robots/ros2_husky_PhysX_vlp16_mono_depth_imu.usd")
    source = Path(__file__)
    paths = [source, asset, source.parent / "local_terrain/selected_lidar.json"]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    cases = [
        (name, xyz, minimum)
        for name, xyz in (("original", [0.4, 0, 0.25]), ("raised_045", [0.4, 0, 0.45]), ("raised_065", [0.4, 0, 0.65]))
        for minimum in (0.5, 0.0)
    ]
    with (run / "manifest.json").open("x") as out:
        json.dump(
            dict(
                protocol="actual_static_colliders_optical_origin_v1",
                source_sha256=hashes,
                cases=cases,
                measurement_minimum_m=0.5,
                base_height_m=0.145,
                acceptance=False,
                note=__doc__,
            ),
            out,
            indent=2,
        )
    app = None
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "width": 64, "height": 64})
        import numpy as np
        import omni.kit.commands
        from isaacsim.core.api import World
        from isaacsim.core.utils.extensions import enable_extension
        from omni.physx import get_physx_scene_query_interface
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
        from local_terrain.ray_design import selected_setup, native_pattern_indices

        enable_extension("isaacsim.sensors.physx")
        from isaacsim.sensors.physx import _range_sensor

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
        world.get_physics_context().set_gravity(-1.62)
        stage = Usd.Stage.Open(str(asset))
        flattened = stage.Flatten()
        cache = UsdGeom.XformCache()
        base = stage.GetPrimAtPath("/husky_robot/base_link")
        if not base:
            raise ValueError("missing known base frame")
        inverse = cache.GetLocalToWorldTransform(base).GetInverse()
        colliders = []
        UsdGeom.Xform.Define(world.stage, "/World/Robot")
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                continue
            if not prim.IsA(UsdGeom.Gprim):
                raise ValueError("unhandled non-geometric collider: " + str(prim.GetPath()))
            transform = np.asarray(cache.GetLocalToWorldTransform(prim) * inverse).T.copy()
            transform[2, 3] += 0.145
            target = f"/World/Robot/collider_{len(colliders)}"
            if not Sdf.CopySpec(flattened, prim.GetPath(), world.stage.GetRootLayer(), target):
                raise ValueError("collider copy failed")
            copied = world.stage.GetPrimAtPath(target)
            copied.RemoveAPI(UsdPhysics.RigidBodyAPI)
            xform = UsdGeom.Xformable(copied)
            xform.ClearXformOpOrder()
            xform.AddTransformOp(opSuffix="audit").Set(Gf.Matrix4d(transform.T.tolist()))
            colliders.append(
                dict(
                    source=str(prim.GetPath()),
                    target=target,
                    type=prim.GetTypeName(),
                    world_from_geometry=transform.tolist(),
                )
            )
        if not colliders:
            raise ValueError("no active colliders copied")
        with (run / "colliders.json").open("x") as out:
            json.dump(colliders, out, indent=2)
        floor = UsdGeom.Cube.Define(world.stage, "/World/Floor")
        floor.CreateSizeAttr(2.0)
        floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.1))
        floor.AddScaleOp().Set(Gf.Vec3f(110, 110, 0.1))
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        setup, elevations = selected_setup()
        azimuth = setup["horizontal_start_rad"] + np.arange(setup["horizontal_columns"]) * np.deg2rad(
            setup["horizontal_resolution_deg"]
        )
        aa, zz = np.meshgrid(azimuth, -np.deg2rad(elevations), indexing="ij")
        pattern = np.tile(np.stack([aa.ravel(), zz.ravel()]), (1, 4)).astype(np.float32)
        api = _range_sensor.acquire_generic_sensor_interface()
        query = get_physx_scene_query_interface()
        previous = None
        results = []
        for name, xyz, minimum in cases:
            label = name + ("_min05" if minimum else "_min0")
            if previous is not None:
                previous.SetActive(False)
            path = "/World/Sensor_" + label
            success, sensor = omni.kit.commands.execute(
                "RangeSensorCreateGeneric",
                path=path,
                parent=None,
                min_range=minimum,
                max_range=100.0,
                sampling_rate=639680,
                draw_points=False,
                draw_lines=False,
            )
            if not success:
                raise ValueError("native sensor construction failed")
            previous = sensor.GetPrim()
            origin = np.asarray(xyz) + [0, 0, 0.145]
            xf = UsdGeom.Xformable(previous)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*origin))
            world.reset()
            for _ in range(6):
                if api.send_next_batch(path):
                    api.set_next_batch_rays(path, pattern)
                world.step(render=True)
            overlaps = []

            def overlap(hit):
                overlaps.append(str(hit.collision))
                return True

            query.overlap_sphere(0.001, tuple(origin), overlap, False)
            fields = {k: [] for k in ("azimuth", "zenith", "depth", "intensity", "points", "tick")}
            for tick in range(12):
                if api.send_next_batch(path):
                    api.set_next_batch_rays(path, pattern)
                world.step(render=True)
                for key, method in [
                    ("azimuth", "get_azimuth_data"),
                    ("zenith", "get_zenith_data"),
                    ("depth", "get_linear_depth_data"),
                    ("intensity", "get_intensity_data"),
                    ("points", "get_point_cloud_data"),
                ]:
                    fields[key].append(
                        np.asarray(getattr(api, method)(path)).copy().reshape(-1, 3)
                        if key == "points"
                        else np.asarray(getattr(api, method)(path)).copy().reshape(-1)
                    )
                fields["tick"].append(np.full(len(fields["depth"][-1]), tick))
            arrays = {k: np.concatenate(v) for k, v in fields.items()}
            beam = native_pattern_indices(arrays["azimuth"], arrays["zenith"], azimuth, elevations)
            if len(np.unique(beam)) != len(azimuth) * 32:
                raise ValueError("missing ray sectors")
            direction = np.column_stack(
                [
                    np.cos(arrays["azimuth"]) * np.cos(arrays["zenith"]),
                    np.sin(arrays["azimuth"]) * np.cos(arrays["zenith"]),
                    -np.sin(arrays["zenith"]),
                ]
            )
            # Every sampled ray gets a zero-origin independent first-hit query.
            unique, indices = np.unique(beam, return_index=True)
            distances = np.full(len(azimuth) * 32, np.inf)
            body_hit = np.zeros(len(distances), bool)
            for b, i in zip(unique, indices):
                hit = query.raycast_closest(tuple(origin), tuple(map(float, direction[i])), 100.0)
                if hit.get("hit"):
                    distances[b] = float(hit["distance"])
                    body_hit[b] = str(hit["collision"]).startswith("/World/Robot/")
            optical = distances[beam]
            actual = (arrays["intensity"] > 0) & (arrays["depth"] > 0.50001) & (arrays["depth"] < 100.0)
            expected = (optical > 0.50001) & (optical < 100.0)
            mismatch = actual != expected
            both = actual & expected
            error = np.abs(arrays["depth"][both] - optical[both])
            bypass = actual & body_hit[beam] & (optical <= 0.50001)
            row = dict(
                case=label,
                mount=xyz,
                raycast_minimum_m=minimum,
                origin_overlaps=overlaps,
                native_samples=len(beam),
                valid_returns=int(actual.sum()),
                mask_mismatch=int(mismatch.sum()),
                hidden_body_returns=int(bypass.sum()),
                max_depth_error_m=float(error.max(initial=0)),
                optical_origin_clear=not overlaps,
                rings=[
                    dict(
                        elevation_deg=float(e),
                        samples=int((beam % 32 == r).sum()),
                        valid=int((actual & (beam % 32 == r)).sum()),
                        bypass=int((bypass & (beam % 32 == r)).sum()),
                    )
                    for r, e in enumerate(elevations)
                ],
            )
            np.savez_compressed(
                run / (label + ".npz"),
                **arrays,
                beam=beam,
                optical_depth=optical,
                optical_body_hit=body_hit[beam],
                optical_origin=origin,
            )
            row["native_sha256"] = hashlib.sha256((run / (label + ".npz")).read_bytes()).hexdigest()
            row["geometry_passed"] = not overlaps and row["mask_mismatch"] == 0 and row["max_depth_error_m"] <= 0.001
            results.append(row)
            with (run / (label + ".json")).open("x") as out:
                json.dump(row, out, indent=2)
            print(json.dumps({k: v for k, v in row.items() if k != "rings"}), flush=True)
            check_run(run)
        if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != h for p, h in hashes.items()):
            raise ValueError("audit source/asset changed")
        with (run / "report.json").open("x") as out:
            json.dump(
                dict(
                    cases=results,
                    fixture_passed=audit_passed(results),
                    rover_qualified=False,
                    acceptance=False,
                    note="Known embedded baseline and both raised zero-origin controls checked. No SLAM/terrain qualification.",
                ),
                out,
                indent=2,
            )
    except BaseException as exc:
        with (run / "failure.json").open("x") as out:
            json.dump(dict(error=str(exc), acceptance=False), out)
        raise
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
