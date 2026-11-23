#!/usr/bin/env python3
"""Stationary native zero-pitch ray-layout comparison through current C++ terrain.

GT geometry is evaluator-only. No moving/SLAM/safe-motion/runtime acceptance.
All native ticks and grid phases remain counted, including startup and failures.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from build_local_terrain import validate_build
from layered_terrain import TerrainMapper
from local_terrain.mounting_study import PHASES, region_masks, valid_returns
from local_terrain.ray_design import (
    TERRAIN_VIEWS_XY,
    TERRAIN_YAWS_DEG,
    ray_mount,
    validate_selected_setup,
    SELECTED_SETUP,
    terrain_view_transform,
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reference_masks(boxes, phase):
    yy, xx = np.indices((60, 60))
    low_x, low_y = -3 + phase + xx * 0.1, -3 + phase + yy * 0.1
    objects = []
    for low, high in boxes:
        mask = (low_x < high[0]) & (low_x + 0.1 > low[0]) & (low_y < high[1]) & (low_y + 0.1 > low[1])
        objects.append(mask)
    hazard = np.logical_or.reduce(objects) if objects else np.zeros((60, 60), bool)
    return ~hazard, objects


def range_regions(phase):
    """Additive base-origin range/sector reports; never change old denominators."""
    yy, xx = np.indices((60, 60))
    x, y = -3 + phase + (xx + 0.5) * 0.1, -3 + phase + (yy + 0.5) * 0.1
    radius = np.hypot(x, y)
    regions = {}
    for low, high in ((0, 1), (1, 2), (2, 3)):
        band = (radius >= low) & (radius < high)
        regions[f"range_{low}_{high}m"] = band
        regions[f"front_range_{low}_{high}m"] = band & (x >= np.abs(y))
    return regions


def score(layers, boxes, phase, wheel_xy):
    free, objects = reference_masks(boxes, phase)
    costs = layers[:, :, 0]
    if not np.isin(costs, [-1, 0, 20, 100]).all():
        raise ValueError("unknown terrain cost class")
    free_map, occupied_map = np.isin(costs, [0, 20]), costs == 100
    regions = {"all": np.ones((60, 60), bool), **region_masks(wheel_xy, phase), **range_regions(phase)}
    return dict(
        false_free_cells=int((free_map & ~free).sum()),
        false_occupied_cells=int((occupied_map & free).sum()),
        obstacle_objects_detected=int(sum((occupied_map & m).any() for m in objects)),
        obstacle_objects=len(objects),
        regions={
            name: dict(
                reference_free_cells=int((mask & free).sum()),
                correctly_free_cells=int((mask & free & free_map).sum()),
                free_fraction=float(free_map[mask & free].mean()) if (mask & free).any() else None,
                unknown_free_cells=int((mask & free & (costs == -1)).sum()),
                false_occupied_cells=int((mask & free & occupied_map).sum()),
                unknown_reasons={
                    str(int(k)): int(v)
                    for k, v in zip(*np.unique(layers[:, :, 1][mask & free & (costs == -1)], return_counts=True))
                },
            )
            for name, mask in regions.items()
        },
        reason_counts={str(int(k)): int(v) for k, v in zip(*np.unique(layers[:, :, 1], return_counts=True))},
    )


def grouped_cases(report, manifest, expected, multiview, observation_yaw=False):
    """Validate complete declared views and original epochs before any assimilation."""
    protocol = (
        "generic_ray_layout_settled_yaw_v1"
        if observation_yaw
        else (
            "generic_ray_layout_static_multiview_v2_triangle_boxes"
            if multiview
            else "generic_ray_layout_stationary_v3_explicit_mount"
        )
    )
    if manifest["protocol"] != protocol:
        raise ValueError("native protocol does not match requested assessment")
    diverse = multiview or observation_yaw
    views = ((0, 0),) * len(TERRAIN_YAWS_DEG) if observation_yaw else (TERRAIN_VIEWS_XY if multiview else ((0, 0),))
    yaws = TERRAIN_YAWS_DEG if observation_yaw else (0,) * len(views)
    if observation_yaw and not np.array_equal(manifest["base_yaws_deg"], yaws):
        raise ValueError("undeclared yaw protocol")
    if diverse and not np.array_equal(manifest["base_translations_xy_m"], views):
        raise ValueError("undeclared view protocol")
    layouts = ("mid_dense_10_to_-60",) if observation_yaw else ("baseline", "mid_dense_10_to_-60")
    selected = [c for c in report["cases"] if c["layout"] in layouts]
    if len(selected) != 2 * len(layouts) * len(views):
        raise ValueError("incomplete layout/scene/view set")
    result = []
    for layout in layouts:
        for scene in ("flat", "boxes"):
            group = [c for c in selected if c["layout"] == layout and c["scene"] == scene]
            if len(group) != len(views):
                raise ValueError("duplicate or missing scene")
            frames = []
            previous = -np.inf
            for index, (case, xy) in enumerate(zip(group, views)):
                pose = terrain_view_transform(expected, xy, yaws[index])
                if (
                    not case["geometry_passed"]
                    or len(case["frames"]) != 12
                    or not np.allclose(case["actual_world_from_lidar"], pose, atol=1e-8, rtol=0)
                ):
                    raise ValueError("incomplete case or incorrect level pose")
                if diverse and (case["view_index"] != index or not np.array_equal(case["base_translation_xy_m"], xy)):
                    raise ValueError("view sequence changed")
                if observation_yaw and case["base_yaw_deg"] != yaws[index]:
                    raise ValueError("incorrect base yaw")
                directory = f"{layout}_{scene}" + (f"_view_{index}" if diverse else "")
                if case.get("case_directory", directory) != directory:
                    raise ValueError("unexpected native case directory")
                for frame in case["frames"]:
                    if not np.isfinite(frame["stamp"]) or frame["stamp"] <= previous:
                        raise ValueError("nonmonotonic/reset native clock; cannot merge epochs")
                    previous = frame["stamp"]
                    frames.append(
                        dict(frame, directory=directory, expected_pose=pose, view_index=index, yaw_deg=yaws[index])
                    )
            result.append(dict(layout=layout, scene=scene, frames=frames))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("native_fixture", type=Path)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--multiview", action="store_true", help="explicit six-view sensor-only protocol")
    parser.add_argument("--observation-yaw", action="store_true", help="explicit 24-view sensor-only yaw protocol")
    args = parser.parse_args()
    if args.multiview and args.observation_yaw:
        parser.error("choose exactly one view protocol")
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        archive = args.archive_root.resolve(strict=True)
        native = args.native_fixture.resolve(strict=True)
        if native.parent != archive or (native / "failure.json").exists():
            raise ValueError("completed native fixture on external archive required")
        manifest = json.loads((native / "manifest.json").read_text())
        report = json.loads((native / "report.json").read_text())
        if (
            not report["fixture_passed"]
            or not report["equal_dose"]
            or manifest["study"]["protocol"] != "single_lidar_ray_design_v2_level"
        ):
            raise ValueError("complete validated level sensor fixture required")
        geometry = manifest["study"]["geometry"]
        setup = validate_selected_setup(geometry, manifest["study"]["layouts"]["mid_dense_10_to_-60"])
        expected = ray_mount(geometry, require_level=True).copy()
        expected[2, 3] += geometry["base_height_m"]
        hashes = {str(native / name): sha(native / name) for name in ("manifest.json", "report.json")}
        hashes.update(validate_build(args.library, archive))
        paths = [
            Path(__file__),
            Path(__file__).with_name("layered_terrain.py"),
            Path(__file__).with_name("build_local_terrain.py"),
            Path(__file__).with_name("local_terrain") / "mounting_study.py",
            Path(__file__).with_name("local_terrain") / "ray_design.py",
            Path(__file__).with_name("revision_evidence.py"),
            Path(__file__).with_name("archive_guard.py"),
            SELECTED_SETUP,
        ]
        source_hashes = {str(p): sha(p) for p in paths}
        for p in paths:
            target = output / "source" / p.relative_to(Path(__file__).parent)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, target)
        cases = []
        selected = grouped_cases(report, manifest, expected, args.multiview, args.observation_yaw)
        with (output / "manifest.json").open("x") as stream:
            json.dump(
                dict(
                    protocol=(
                        "settled_yaw_native_ray_terrain_v1"
                        if args.observation_yaw
                        else (
                            "static_multiview_native_ray_terrain_v1"
                            if args.multiview
                            else "stationary_native_ray_terrain_v2_all_traversable_classes"
                        )
                    ),
                    input_sha256=hashes,
                    source_sha256=source_hashes,
                    phases=PHASES,
                    lifetime_s=60,
                    grid=[60, 60, 0.1],
                    acceptance=False,
                    selected_sensor=setup,
                    range_note="cell centers relative to original base XY, additive ranges; all unknowns counted",
                    note=__doc__,
                ),
                stream,
                indent=2,
            )
        for case in selected:
            boxes = geometry["hazard_boxes"] if case["scene"] == "boxes" else []
            for phase in PHASES:
                frames = []
                folder = output / f"{case['layout']}_{case['scene']}_{phase:g}"
                folder.mkdir()
                with TerrainMapper(
                    args.library, width=60, height=60, origin=(-3 + phase, -3 + phase), lifetime=60
                ) as mapper:
                    previous = -np.inf
                    for index, frame in enumerate(case["frames"]):
                        path = (native / frame["directory"] / frame["file"]).resolve(strict=True)
                        if not path.is_relative_to(native) or sha(path) != frame["sha256"]:
                            raise ValueError("native evidence changed or escaped archive")
                        hashes[str(path)] = frame["sha256"]
                        with np.load(path, allow_pickle=False) as raw:
                            stamp = float(raw["observed_stamp"])
                            pose = raw["world_from_lidar"]
                            if (
                                stamp != frame["stamp"]
                                or stamp <= previous
                                or not np.allclose(pose, frame["expected_pose"], atol=1e-8, rtol=0)
                            ):
                                raise ValueError("native time/level transform mismatch")
                            valid = (raw["intensity"] > 0) & valid_returns(raw["depth"], 0.5, 100)
                            points = raw["points"][valid].astype(float) @ pose[:3, :3].T + pose[:3, 3]
                        previous = stamp
                        mapper.integrate(points, stamp, pose[:3, 3], yaw=np.deg2rad(frame["yaw_deg"]))
                        layers = mapper.project(stamp)
                        dest = folder / f"map_{index:02}.npz"
                        np.savez_compressed(dest, layers=layers, stamp=stamp, origin=[-3 + phase, -3 + phase])
                        frames.append(
                            dict(
                                stamp=stamp,
                                view_index=frame["view_index"],
                                native_file=str(path),
                                native_sha256=frame["sha256"],
                                points=len(points),
                                map_sha256=sha(dest),
                                **score(layers, boxes, phase, geometry["wheel_xy"]),
                            )
                        )
                result = dict(layout=case["layout"], scene=case["scene"], phase=phase, frames=frames)
                cases.append(result)
                print(
                    json.dumps(
                        dict(
                            layout=case["layout"],
                            scene=case["scene"],
                            phase=phase,
                            final_free=frames[-1]["regions"]["all"]["free_fraction"],
                            objects=frames[-1]["obstacle_objects_detected"],
                            false_free_exposure=sum(f["false_free_cells"] for f in frames),
                        )
                    ),
                    flush=True,
                )
                check_run(output)
        if any(sha(p) != v for p, v in {**hashes, **source_hashes}.items()):
            raise ValueError("source or input changed")
        with (output / "report.json").open("x") as stream:
            json.dump(
                dict(cases=cases, input_sha256=hashes, acceptance=False, terrain_qualified=False),
                stream,
                indent=2,
                allow_nan=False,
            )
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
