#!/usr/bin/env python3
"""Stationary Isaac custom-ray sensor check; no rover, SLAM or navigation.

Uses Generic range sensor explicit ray tables, not the legacy symmetric LiDAR.
Acquisition is experimental: preserve native samples/ticks and fail on unmatched
directions, missing sectors, geometry mismatches or unequal acquisition dose.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

from archive_guard import allocate_run, check_run


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--study-run", type=Path, required=True)
    parser.add_argument("--multiview", action="store_true", help="six settled sensor-only views; no rover motion")
    parser.add_argument("--observation-yaw", action="store_true", help="24 settled base-yaw sensor-only views")
    args = parser.parse_args()
    if args.multiview and args.observation_yaw:
        parser.error("choose exactly one declared view protocol")
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    app = None
    try:
        study = args.study_run.resolve(strict=True)
        if study.parent != args.archive_root.resolve(strict=True) or (study / "failure.json").exists():
            raise ValueError("completed archived ray-design screen required")
        inputs = {str(study / name): sha(study / name) for name in ("manifest.json", "report.json")}
        source = json.loads((study / "manifest.json").read_text())
        if source["protocol"] not in ("single_lidar_ray_design_v1", "single_lidar_ray_design_v2_level"):
            raise ValueError("unexpected source protocol")
        contract = source["geometry"]
        from local_terrain.ray_design import (
            TERRAIN_VIEWS_XY,
            TERRAIN_YAWS_DEG,
            SELECTED_SETUP,
            validate_selected_setup,
            terrain_view_transform,
        )

        diverse = args.multiview or args.observation_yaw
        views = (
            ((0, 0),) * len(TERRAIN_YAWS_DEG)
            if args.observation_yaw
            else (TERRAIN_VIEWS_XY if args.multiview else ((0, 0),))
        )
        yaws = TERRAIN_YAWS_DEG if args.observation_yaw else (0,) * len(views)
        if diverse and source["protocol"] != "single_lidar_ray_design_v2_level":
            raise ValueError("multiview terrain requires explicit level geometry")
        if source["protocol"] == "single_lidar_ray_design_v2_level":
            validate_selected_setup(contract, source["layouts"]["mid_dense_10_to_-60"])
        paths = [
            Path(__file__),
            Path(__file__).with_name("archive_guard.py"),
            Path(__file__).with_name("revision_evidence.py"),
            SELECTED_SETUP,
        ]
        paths += [Path(__file__).with_name("local_terrain") / name for name in ("mounting_study.py", "ray_design.py")]
        hashes = {str(p): sha(p) for p in paths}
        for p in paths:
            target = output / "source" / p.relative_to(Path(__file__).parent)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, target)
        with (output / "manifest.json").open("x") as stream:
            json.dump(
                dict(
                    protocol=(
                        "generic_ray_layout_settled_yaw_v1"
                        if args.observation_yaw
                        else (
                            "generic_ray_layout_static_multiview_v2_triangle_boxes"
                            if args.multiview
                            else "generic_ray_layout_stationary_v3_explicit_mount"
                        )
                    ),
                    base_translations_xy_m=views,
                    base_yaws_deg=yaws,
                    motion_note="settled sensor placements only; continuous clock within scene; no rover",
                    hazard_collision="explicit triangle mesh" if diverse else "USD cube",
                    study=source,
                    source_sha256=hashes,
                    input_sha256=inputs,
                    tick_seconds=1 / 60,
                    recorded_ticks=12,
                    warmup_ticks=6,
                    rays_per_second=contract["full_columns"] * 32 * 10,
                    geometry_tolerance_m=0.002,
                    acceptance=False,
                    note=__doc__,
                ),
                stream,
                indent=2,
            )
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "width": 64, "height": 64})
        import numpy as np
        import omni.kit.commands
        from isaacsim.core.api import World
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import Gf, UsdGeom, UsdPhysics

        enable_extension("isaacsim.sensors.physx")
        from isaacsim.sensors.physx import _range_sensor
        from local_terrain.mounting_study import (
            azimuth_coverage,
            first_intersections,
            sampling_metrics,
            PHASES,
            valid_returns,
        )
        from local_terrain.ray_design import native_pattern_indices, ray_mount

        world = World(stage_units_in_meters=1, physics_dt=1 / 60, rendering_dt=1 / 60)
        world.get_physics_context().set_gravity(-1.62)

        def cube(path, low, high, triangle_mesh=False):
            low, high = np.asarray(low), np.asarray(high)
            if triangle_mesh:
                # Exact closed box surfaces; avoid PhysX convex-cube grazing-hit
                # inflation. Same declared bounds and strict ray/depth gate.
                prim = UsdGeom.Mesh.Define(world.stage, path)
                vertices = [[x, y, z] for z in (low[2], high[2]) for y in (low[1], high[1]) for x in (low[0], high[0])]
                triangles = [
                    0,
                    2,
                    3,
                    0,
                    3,
                    1,
                    4,
                    5,
                    7,
                    4,
                    7,
                    6,
                    0,
                    1,
                    5,
                    0,
                    5,
                    4,
                    2,
                    6,
                    7,
                    2,
                    7,
                    3,
                    0,
                    4,
                    6,
                    0,
                    6,
                    2,
                    1,
                    3,
                    7,
                    1,
                    7,
                    5,
                ]
                prim.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in vertices])
                prim.CreateFaceVertexCountsAttr([3] * 12)
                prim.CreateFaceVertexIndicesAttr(triangles)
                prim.CreateSubdivisionSchemeAttr("none")
                UsdPhysics.MeshCollisionAPI.Apply(prim.GetPrim()).CreateApproximationAttr("none")
                return UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
            prim = UsdGeom.Cube.Define(world.stage, path)
            prim.CreateSizeAttr(2)
            prim.AddTranslateOp().Set(Gf.Vec3d(*map(float, (low + high) / 2)))
            prim.AddScaleOp().Set(Gf.Vec3f(*map(float, (high - low) / 2)))
            return UsdPhysics.CollisionAPI.Apply(prim.GetPrim())

        cube("/World/floor", [-6, -6, -0.1], [6, 6, 0])
        hazards = [
            cube(f"/World/hazard_{i}", low, high, triangle_mesh=diverse)
            for i, (low, high) in enumerate(contract["hazard_boxes"])
        ]
        attrs = contract["sensor_attributes"]
        transform = ray_mount(contract, require_level=source["protocol"] == "single_lidar_ray_design_v2_level").copy()
        transform[2, 3] += contract["base_height_m"]
        initial_transform = transform.copy()
        api = _range_sensor.acquire_generic_sensor_interface()
        with (output / "interface.json").open("x") as stream:
            json.dump(
                {name: str(getattr(api, name).__doc__) for name in dir(api) if not name.startswith("_")},
                stream,
                indent=2,
            )
        required = ("get_azimuth_data", "get_zenith_data", "get_point_cloud_data", "get_num_samples_ticked")
        if any(not hasattr(api, name) for name in required):
            raise ValueError("custom sensor lacks required native association methods: " + repr(required))
        az = -np.pi + np.arange(contract["full_columns"]) * np.deg2rad(float(attrs["horizontalResolution"]))
        results = []
        previous_sensor = None
        for name, elevation in source["layouts"].items():
            if args.observation_yaw and name != "mid_dense_10_to_-60":
                continue
            if args.multiview and name not in ("baseline", "mid_dense_10_to_-60"):
                continue
            aa, zz = np.meshgrid(az, -np.deg2rad(elevation), indexing="ij")
            pattern = np.tile(np.stack([aa.ravel(), zz.ravel()]), (1, 4)).astype(np.float32)
            for scene, view_index, xy in ((s, i, v) for s in ("flat", "boxes") for i, v in enumerate(views)):
                transform = terrain_view_transform(initial_transform, xy, yaws[view_index])
                case_directory = f"{name}_{scene}" + (f"_view_{view_index}" if diverse else "")
                if previous_sensor is not None:
                    previous_sensor.SetActive(False)
                path = "/World/CustomLidar_" + case_directory.replace("-", "minus").replace(".", "_")
                success, sensor = omni.kit.commands.execute(
                    "RangeSensorCreateGeneric",
                    path=path,
                    parent=None,
                    min_range=float(attrs["minRange"]),
                    max_range=float(attrs["maxRange"]),
                    sampling_rate=contract["full_columns"] * 32 * 10,
                    draw_points=False,
                    draw_lines=False,
                )
                if not success or not sensor:
                    raise ValueError("custom-ray sensor creation failed")
                previous_sensor = sensor.GetPrim()
                xform = UsdGeom.Xformable(previous_sensor)
                xform.ClearXformOpOrder()
                xform.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))
                for collision in hazards:
                    collision.GetCollisionEnabledAttr().Set(scene == "boxes")
                if view_index == 0:
                    world.reset()
                actual_transform = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(previous_sensor)).T
                if not np.allclose(actual_transform, transform, atol=1e-8, rtol=0):
                    raise ValueError("actual USD sensor mount disagrees with experimental transform")
                folder = output / case_directory
                folder.mkdir()
                frames, points, angles = [], [], []
                counts = np.zeros(8, int)
                beam_seen = np.zeros(contract["full_columns"] * 32, bool)
                for tick in range(18):
                    if api.send_next_batch(path):
                        api.set_next_batch_rays(path, pattern)
                    world.step(render=True)
                    if tick < 6:
                        continue
                    a = np.asarray(api.get_azimuth_data(path), float).reshape(-1).copy()
                    z = np.asarray(api.get_zenith_data(path), float).reshape(-1).copy()
                    depth = np.asarray(api.get_linear_depth_data(path), float).reshape(-1).copy()
                    intensity = np.asarray(api.get_intensity_data(path)).reshape(-1).copy()
                    xyz = np.asarray(api.get_point_cloud_data(path), float).reshape(-1, 3).copy()
                    n = int(api.get_num_samples_ticked(path))
                    raw = folder / f"native_{tick:02}.npz"
                    np.savez_compressed(
                        raw,
                        azimuth=a,
                        zenith=z,
                        depth=depth,
                        intensity=intensity,
                        points=xyz,
                        observed_stamp=float(world.current_time),
                        world_from_lidar=transform,
                    )
                    if not n or any(len(v) != n for v in (a, z, depth, intensity, xyz)):
                        raise ValueError("custom native sample dimensions disagree")
                    beam_seen[native_pattern_indices(a, z, az, elevation)] = True
                    directions = (
                        np.column_stack([np.cos(a) * np.cos(z), np.sin(a) * np.cos(z), -np.sin(z)])
                        @ transform[:3, :3].T
                    )
                    expected, labels = first_intersections(
                        transform[:3, 3], directions, contract["hazard_boxes"] if scene == "boxes" else ()
                    )
                    expected_hit = valid_returns(expected, float(attrs["minRange"]), 5)
                    actual_hit = (intensity > 0) & valid_returns(depth, float(attrs["minRange"]), 5)
                    both = expected_hit & actual_hit
                    world_points = xyz @ transform[:3, :3].T + transform[:3, 3]
                    err = np.linalg.norm(
                        world_points[both] - (transform[:3, 3] + directions[both] * expected[both, None]), axis=1
                    )
                    row = dict(
                        tick=tick,
                        stamp=float(world.current_time),
                        samples=n,
                        mask_mismatches=int((expected_hit != actual_hit).sum()),
                        max_xyz_error_m=float(err.max(initial=0)),
                        max_depth_error_m=float(np.abs(depth[both] - expected[both]).max(initial=0)),
                    )
                    row.update(file=raw.name, sha256=sha(raw))
                    frames.append(row)
                    points.append(world_points[actual_hit & (np.abs(world_points[:, 2]) < 0.002)])
                    angles.extend(a.tolist())
                    for i in range(8):
                        counts[i] += int((both & (labels == i + 1)).sum())
                coverage = azimuth_coverage(angles, float(attrs["horizontalResolution"]))
                passed = (
                    bool(beam_seen.all())
                    and coverage["complete_azimuth"]
                    and all(
                        r["mask_mismatches"] == 0 and max(r["max_xyz_error_m"], r["max_depth_error_m"]) <= 0.002
                        for r in frames
                    )
                )
                result = dict(
                    layout=name,
                    scene=scene,
                    view_index=view_index,
                    base_translation_xy_m=xy,
                    base_yaw_deg=yaws[view_index],
                    case_directory=case_directory,
                    actual_world_from_lidar=actual_transform.tolist(),
                    frames=frames,
                    **coverage,
                    geometry_passed=passed,
                    unique_beams_observed=int(beam_seen.sum()),
                    expected_unique_beams=len(beam_seen),
                    hazard_return_counts=counts.tolist(),
                    metrics={str(p): sampling_metrics(np.concatenate(points), contract["wheel_xy"], p) for p in PHASES},
                )
                with (folder / "report.json").open("x") as stream:
                    json.dump(result, stream, indent=2, allow_nan=False)
                results.append(result)
                print(json.dumps(dict(layout=name, scene=scene, geometry_passed=passed)), flush=True)
                check_run(output)
        doses = {tuple(f["samples"] for f in r["frames"]) for r in results}
        passed = len(doses) == 1 and all(r["geometry_passed"] for r in results)
        if any(sha(p) != digest for p, digest in {**inputs, **hashes}.items()):
            raise ValueError("fixture source/input changed")
        with (output / "report.json").open("x") as stream:
            json.dump(
                dict(
                    fixture_passed=passed,
                    cases=results,
                    equal_dose=len(doses) == 1,
                    acceptance=False,
                    rover_qualified=False,
                ),
                stream,
                indent=2,
            )
        if not passed:
            raise ValueError("custom-ray geometry or acquisition gate failed")
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
