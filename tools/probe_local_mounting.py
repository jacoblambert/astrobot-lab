#!/usr/bin/env python3
"""Isolated stationary native-LiDAR geometry check, NOT rover qualification.

No rover, ROS, SLAM, command publisher, teleoperation or navigation. Actual native
10 Hz sensor chunks in an analytic plane/box scene; poses are constant per case.
This validates sampling predictions, not self-occlusion or a startup maneuver.
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
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    app = None
    try:
        study = args.study_run.resolve(strict=True)
        if not study.is_relative_to(args.archive_root.resolve(strict=True)) or (study / "failure.json").exists():
            raise ValueError("completed archived mounting screen required")
        inputs = {str(study / name): sha(study / name) for name in ("manifest.json", "report.json")}
        contract = json.loads((study / "manifest.json").read_text())
        if contract["protocol"] != "ideal_mount_sampling_v1" or contract["acceptance"] is not False:
            raise ValueError("unexpected mounting study contract")
        sources = [
            Path(__file__),
            Path(__file__).with_name("archive_guard.py"),
            Path(__file__).with_name("revision_evidence.py"),
            Path(__file__).with_name("local_terrain") / "mounting_study.py",
        ]
        hashes = {str(p): sha(p) for p in sources}
        for path in sources:
            target = output / "source" / path.relative_to(Path(__file__).parent)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
        manifest = dict(
            protocol="isolated_native_mount_sampling_v1",
            study=contract,
            input_sha256=inputs,
            source_sha256=hashes,
            warning=__doc__,
            warmup_ticks_per_case=6,
            measured_ticks_per_case=6,
            physics_dt=1 / 60,
            rendering_dt=1 / 30,
            maximum_scored_range_m=5.0,
            geometry_tolerance_m=0.002,
            acceptance=False,
        )
        with (output / "manifest.json").open("x") as stream:
            json.dump(manifest, stream, indent=2, allow_nan=False)

        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "width": 64, "height": 64})
        import numpy as np
        import omni.kit.commands
        from isaacsim.core.api import World
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        enable_extension("isaacsim.sensors.physx")
        from isaacsim.sensors.physx import _range_sensor
        from local_terrain.mounting_study import (
            PHASES,
            azimuth_coverage,
            first_intersections,
            sampling_metrics,
            sensor_mount,
            valid_returns,
            world_rays,
        )

        world = World(stage_units_in_meters=1, physics_dt=1 / 60, rendering_dt=1 / 30)
        world.get_physics_context().set_gravity(-1.62)

        def cube(path, low, high):
            low, high = np.asarray(low), np.asarray(high)
            cube = UsdGeom.Cube.Define(world.stage, path)
            cube.CreateSizeAttr(2)
            cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, (low + high) / 2)))
            cube.AddScaleOp().Set(Gf.Vec3f(*map(float, (high - low) / 2)))
            return UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        cube("/World/floor", [-6, -6, -0.1], [6, 6, 0])
        hazards = [cube(f"/World/hazard_{i}", low, high) for i, (low, high) in enumerate(contract["hazard_boxes"])]
        attrs = contract["sensor_attributes"]
        lidar_path = "/World/Lidar"
        success, sensor = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=lidar_path,
            parent=None,
            min_range=float(attrs["minRange"]),
            max_range=float(attrs["maxRange"]),
            horizontal_fov=float(attrs["horizontalFov"]),
            vertical_fov=float(attrs["verticalFov"]),
            horizontal_resolution=float(attrs["horizontalResolution"]),
            vertical_resolution=float(attrs["verticalResolution"]),
            rotation_rate=float(attrs["rotationRate"]),
            high_lod=True,
            draw_points=False,
            draw_lines=False,
        )
        if not success or not sensor:
            raise ValueError("native LiDAR creation failed")
        xform = UsdGeom.Xformable(world.stage.GetPrimAtPath(lidar_path))
        xform.ClearXformOpOrder()
        transform_op = xform.AddTransformOp()
        interface = _range_sensor.acquire_lidar_sensor_interface()
        results = []
        for pitch in contract["pitches_deg"]:
            mount = sensor_mount(pitch)
            expected_transform = mount.copy()
            expected_transform[2, 3] += contract["base_height_m"]
            transform_op.Set(Gf.Matrix4d(expected_transform.T.tolist()))
            for scene in ("flat", "boxes"):
                for collision in hazards:
                    collision.GetCollisionEnabledAttr().Set(scene == "boxes")
                world.reset()
                for _ in range(manifest["warmup_ticks_per_case"]):
                    world.step(render=True)
                actual_transform = np.asarray(
                    UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(xform.GetPrim())
                ).T
                if not np.allclose(actual_transform, expected_transform, atol=1e-8, rtol=0):
                    raise ValueError("native fixture mount transform mismatch")
                case = output / f"pitch_{pitch}_{scene}"
                case.mkdir()
                points, errors, frames, observed_columns, hazard_counts = [], [], [], [], np.zeros(8, dtype=int)
                for tick in range(manifest["measured_ticks_per_case"]):
                    world.step(render=True)
                    azimuth = np.asarray(interface.get_azimuth_data(lidar_path), float).copy()
                    zenith = np.asarray(interface.get_zenith_data(lidar_path), float).copy()
                    depth = np.asarray(interface.get_linear_depth_data(lidar_path), float).copy().reshape(-1)
                    intensity = np.asarray(interface.get_intensity_data(lidar_path)).copy().reshape(-1)
                    local_points = np.asarray(interface.get_point_cloud_data(lidar_path), float).copy().reshape(-1, 3)
                    columns = int(interface.get_num_cols_ticked(lidar_path))
                    if columns != len(azimuth) or depth.size != columns * len(zenith) or not columns:
                        raise ValueError("unexpected native chunk dimensions")
                    if not np.allclose(zenith, contract["native_zenith_radians"], atol=1e-7, rtol=0):
                        raise ValueError("native beam elevations changed")
                    if int(interface.get_num_cols(lidar_path)) != contract["full_columns"]:
                        raise ValueError("native full-column count changed")
                    origin, directions = world_rays(mount, contract["base_height_m"], azimuth, zenith)
                    predicted, labels = first_intersections(
                        origin, directions, contract["hazard_boxes"] if scene == "boxes" else ()
                    )
                    expected_hit = valid_returns(predicted, float(attrs["minRange"]), 5.0)
                    actual_hit = (intensity > 0) & valid_returns(depth, float(attrs["minRange"]), 5.0)
                    # All predicted / measured <=5m returns count, including mismatches.
                    mismatch = int(np.count_nonzero(expected_hit != actual_hit))
                    both = expected_hit & actual_hit
                    residual = np.abs(predicted[both] - depth[both])
                    world_points = local_points @ mount[:3, :3].T + origin
                    xyz_error = np.linalg.norm(
                        world_points[both] - (origin + directions[both] * predicted[both, None]), axis=1
                    )
                    record = dict(
                        tick=tick,
                        stamp=float(world.current_time),
                        columns=columns,
                        hit_mask_mismatches=mismatch,
                        max_depth_error_m=float(residual.max(initial=0)),
                        max_xyz_error_m=float(xyz_error.max(initial=0)),
                        expected_hits=int(expected_hit.sum()),
                        measured_hits=int(actual_hit.sum()),
                    )
                    raw = case / f"native_{tick:02}.npz"
                    np.savez_compressed(
                        raw,
                        azimuth=azimuth,
                        zenith=zenith,
                        depth=depth,
                        intensity=intensity,
                        points=local_points,
                        observed_stamp=record["stamp"],
                        world_from_lidar=expected_transform,
                    )
                    record["file"], record["sha256"] = raw.name, sha(raw)
                    frames.append(record)
                    errors.append(
                        mismatch == 0 and record["max_depth_error_m"] <= 0.002 and record["max_xyz_error_m"] <= 0.002
                    )
                    observed_columns.extend(azimuth.tolist())
                    # Ground classification belongs ONLY to this analytic fixture evaluator.
                    points.append(world_points[actual_hit & (np.abs(world_points[:, 2]) < 0.002)])
                    for i in range(8):
                        hazard_counts[i] += int((both & (labels == i + 1)).sum())
                angular = azimuth_coverage(observed_columns, float(attrs["horizontalResolution"]))
                ground = np.concatenate(points)
                result = dict(
                    pitch_deg=pitch,
                    scene=scene,
                    frames=frames,
                    **angular,
                    native_geometry_passed=all(errors) and angular["complete_azimuth"],
                    hazard_return_counts=hazard_counts.tolist(),
                    ground_hit_metrics={
                        str(phase): sampling_metrics(ground, contract["wheel_xy"], phase) for phase in PHASES
                    },
                )
                with (case / "report.json").open("x") as stream:
                    json.dump(result, stream, indent=2, allow_nan=False)
                results.append(result)
                print(
                    json.dumps(
                        {k: result[k] for k in ("pitch_deg", "scene", "native_geometry_passed", "hazard_return_counts")}
                    ),
                    flush=True,
                )
                check_run(output)
        if any(sha(p) != digest for p, digest in {**inputs, **hashes}.items()):
            raise ValueError("fixture input/source changed")
        passed = all(case["native_geometry_passed"] for case in results)
        with (output / "report.json").open("x") as stream:
            json.dump(
                dict(fixture_passed=passed, cases=results, acceptance=False, rover_qualified=False), stream, indent=2
            )
        if not passed:
            raise ValueError("native geometry / acquisition completeness mismatch; retain all cases")
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
