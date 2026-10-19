#!/usr/bin/env python3
"""Isolated seed-43 collider/raycast precision diagnostic, never an estimator.

Reproduce a saved rejected ray without driving a rover or changing live guards.
Preserve every case, including unsuccessful collider hypotheses.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

from archive_guard import allocate_run, check_run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--rejection-run", type=Path, required=True)
    parser.add_argument("--math-only", action="store_true", help="no Isaac; local float64 triangle diagnostic only")
    args = parser.parse_args()
    check_run(args.rejection_run)
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    app = None
    try:
        import numpy as np
        from omegaconf import OmegaConf

        root = Path("/workspace/omnilrs")
        os.chdir(root)
        sys.path.insert(0, str(root))
        from src.configurations.procedural_terrain_confs import MoonYardConf
        from src.terrain_management.terrain_generation import GenerateProceduralMoonYard

        config_path = root / "cfg/environment/lunaryard_40m.yaml"
        source = args.rejection_run / "rejected_native_recast.json"
        recast = json.loads(source.read_text())
        cfg = OmegaConf.create(dict(environment=OmegaConf.load(config_path)))
        cfg.environment.seed = 43
        config = OmegaConf.to_container(cfg.environment.terrain_manager.moon_yard, resolve=True)
        paths = [
            Path(__file__),
            source,
            config_path,
            root / "src/terrain_management/terrain_generation.py",
            root / "src/configurations/procedural_terrain_confs.py",
            Path(__file__).with_name("ray_triangle_precision.py"),
        ]
        hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
        dem, _, _ = GenerateProceduralMoonYard(MoonYardConf(**config)).randomize()
        dem = np.flip(dem, 0).astype(np.float32)
        height, width = dem.shape
        points = np.empty((height, width, 3), np.float32)
        points[:, :, 0] = np.arange(width, dtype=np.float64)[None, :] * 0.02
        points[:, :, 1] = np.arange(height, dtype=np.float64)[:, None] * 0.02
        points[:, :, 2] = dem
        y, x = np.mgrid[1:height, 1:width]
        a, b, c, d = (y - 1) * width + x, y * width + x, (y - 1) * width + x - 1, y * width + x - 1
        faces = np.stack([a, b, c, b, d, c], axis=-1).astype(np.int32).reshape(-1, 3)
        points = points.reshape(-1, 3)
        manifest = dict(
            protocol="isolated_native_precision_v1",
            math_only=args.math_only,
            input_sha256=hashes,
            dem_sha256=hashlib.sha256(dem.tobytes()).hexdigest(),
            vertices=len(points),
            triangles=len(faces),
            acceptance=False,
            note="Regenerated seed-43 terrain; reproduction must be measured, not assumed.",
        )
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
        from ray_triangle_precision import intersections

        references = []
        for rejected in recast["inspected"]:
            for label, ray in rejected["rays"].items():
                target = np.asarray(ray["first_hits"][0]["position"])
                # Local diagnostic only: this is NOT an all-scene first-hit oracle.
                selected_vertices = (np.abs(points[:, :2] - target[:2]) < 0.2).all(axis=1)
                selected_faces = np.flatnonzero(selected_vertices[faces].all(axis=1))
                triangles = points[faces[selected_faces]]
                hits = intersections(ray["origin"], ray["direction"], triangles)
                row = dict(
                    index=rejected["index"],
                    ray=label,
                    local_triangles=len(triangles),
                    hits=hits,
                    all_scene_reference=False,
                    acceptance=False,
                )
                if hits:
                    row["physx_distance_error_m"] = ray["first_hits"][0]["distance_m"] - hits[0]["distance"]
                    row["physx_position_error_m"] = float(np.linalg.norm(target - hits[0]["position"]))
                references.append(row)
                np.savez(
                    output / f"triangles_{rejected['index']}_{label}.npz",
                    triangles=triangles,
                    face_indices=selected_faces,
                    origin=ray["origin"],
                    direction=ray["direction"],
                )
        (output / "math_reference.json").write_text(json.dumps(references, indent=2))
        if args.math_only:
            if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != v for p, v in hashes.items()):
                raise ValueError("diagnostic input changed")
            check_run(output)
            (output / "report.json").write_text(
                json.dumps(
                    dict(
                        fixture_passed=True,
                        acceptance=False,
                        rover_qualified=False,
                        math_only=True,
                        references=references,
                    ),
                    indent=2,
                )
            )
            return
        from isaacsim import SimulationApp

        app = SimulationApp(dict(headless=True, disable_viewport_updates=True, width=64, height=64))
        from isaacsim.core.api import World
        from omni.physx import get_physx_scene_query_interface
        from omni.physx.scripts import utils
        from pxr import Gf, UsdGeom

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
        query = get_physx_scene_query_interface()
        results = []
        for name, mode, center in [
            ("baseline", "meshSimplification", 0.0),
            ("centered", "meshSimplification", 20.0),
            ("unsimplified", "none", 0.0),
        ]:
            if world.stage.GetPrimAtPath("/World/terrain"):
                world.stop()
                world.stage.RemovePrim("/World/terrain")
                app.update()
            mesh = UsdGeom.Mesh.Define(world.stage, "/World/terrain")
            local = points.copy()
            local[:, :2] -= center
            mesh.CreatePointsAttr(local)
            mesh.CreateFaceVertexIndicesAttr(faces.ravel())
            mesh.CreateFaceVertexCountsAttr(np.full(len(faces), 3, np.int32))
            mesh.AddTranslateOp().Set(Gf.Vec3d(center, center, 0.0))
            utils.setCollider(mesh.GetPrim(), approximationShape=mode)
            world.reset()
            for _ in range(3):
                world.step(render=False)
            rows = []
            for rejected in recast["inspected"]:
                for label, ray in rejected["rays"].items():
                    origin = np.asarray(ray["origin"])
                    base_dir = np.asarray(ray["direction"])
                    # Fixed perturbation grid, all results retained, no best-ray selection.
                    for dx in (-1e-5, 0.0, 1e-5):
                        for dz in (-1e-5, 0.0, 1e-5):
                            direction = base_dir + [dx, 0.0, dz]
                            direction /= np.linalg.norm(direction)
                            hit = query.raycast_closest(tuple(origin), tuple(direction), 100.0, True)
                            row = dict(
                                index=rejected["index"],
                                ray=label,
                                dx=dx,
                                dz=dz,
                                origin=origin.tolist(),
                                direction=direction.tolist(),
                                hit=bool(hit["hit"]),
                            )
                            if hit["hit"]:
                                distance, position = float(hit["distance"]), np.asarray(hit["position"])
                                row.update(
                                    distance=distance,
                                    position=position.tolist(),
                                    normal=list(hit["normal"]),
                                    residual_m=float(np.linalg.norm(position - origin - direction * distance)),
                                    archived_position_error_m=float(
                                        np.linalg.norm(position - np.asarray(ray["first_hits"][0]["position"]))
                                    ),
                                )
                                if name == "baseline" and dx == dz == 0.0:
                                    reference = ray["first_hits"][0]
                                    if (
                                        row["archived_position_error_m"] > 1e-6
                                        or abs(distance - reference["distance_m"]) > 1e-6
                                    ):
                                        raise ValueError("regenerated baseline does not reproduce archived PhysX ray")
                            rows.append(row)
            result = dict(
                case=name,
                mode=mode,
                center=center,
                rows=rows,
                max_residual_m=max(r.get("residual_m", 0.0) for r in rows),
            )
            results.append(result)
            (output / (name + ".json")).write_text(json.dumps(result, indent=2))
            print(name, result["max_residual_m"], flush=True)
        if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != v for p, v in hashes.items()):
            raise ValueError("diagnostic input changed")
        check_run(output)
        (output / "report.json").write_text(
            json.dumps(
                dict(
                    fixture_passed=True,
                    acceptance=False,
                    rover_qualified=False,
                    cases=results,
                    note="Diagnostic executed; case residuals determine hypotheses, not rover acceptance.",
                ),
                indent=2,
            )
        )
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    main()
