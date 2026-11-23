#!/usr/bin/env python3
"""Frozen swept-patch availability on every publication of a declared replay.

Four assumed 80x120 mm rectangular patches at measured wheel XY. This is a
geometry/usefulness diagnostic, NOT measured tire contact or vehicle feasibility.
No body-clearance, independent terrain truth, pose uncertainty or freshness pass.
No input, pose, class, query exposure or retention is changed by the observer.
"""

import argparse
import json
from pathlib import Path
import shutil
import time

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.physx_reference import BodyBox
from local_terrain.support_sweep import diagnostic_motions, support_availability
from replay_local_terrain import replay, sha


def loaded_patches(capture, loaded):
    paths = [capture / "manifest.json", loaded / "manifest.json", loaded / "report.json", loaded / "contacts.jsonl"]
    hashes = {str(p): sha(p) for p in paths}
    capture_manifest, manifest, report = [json.loads(p.read_text()) for p in paths[:3]]
    if (loaded / "failure.json").exists() or report.get("fixture_passed") is not True:
        raise ValueError("successful loaded characterization required")
    if sha(paths[3]) != report["contacts_sha256"]:
        raise ValueError("loaded contact evidence changed")
    assets = [
        digest
        for p, digest in capture_manifest["source_asset_sha256"].items()
        if Path(p).name == "ros2_husky_PhysX_vlp16_mono_depth_imu.usd"
    ]
    if assets != [manifest["asset_sha256"]]:
        raise ValueError("different capture and loaded rover assets")
    rows = [json.loads(line) for line in paths[3].read_text().splitlines()]
    if len(rows) != 480 or [r["step"] for r in rows] != list(range(1, 481)):
        raise ValueError("incomplete loaded evidence")
    positions = np.asarray(rows[-1]["positions"], dtype=float)
    q = rows[-1]["orientations_wxyz"][0]
    if positions.shape != (5, 3) or not np.isfinite(positions).all() or len(manifest["wheels"]) != 4:
        raise ValueError("four finite loaded wheel positions required")
    xy = Rotation.from_quat([*q[1:], q[0]]).inv().apply(positions[1:] - positions[0])[:, :2]
    height = report["mean_base_height_m"]
    if not np.isfinite(height) or not 0 < height < 1:
        raise ValueError("invalid loaded height")
    patches = [
        BodyBox((float(x), float(y), -height), (0.04, 0.06, 0.001), manifest["asset_sha256"] + ":assumed_patch:" + name)
        for (x, y), name in zip(xy, manifest["wheels"])
    ]
    return patches, hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("capture", type=Path)
    parser.add_argument("loaded_contact", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--window", type=int, choices=(2, 5, 30), required=True)
    parser.add_argument("--max-projections", type=int, default=320)
    parser.add_argument("--candidate-library", type=Path, required=True)
    parser.add_argument("--diagnose-failed-prefix", action="store_true")
    parser.add_argument("--evidence-lifetime", type=int, choices=(30, 60))
    parser.add_argument(
        "--map-policy",
        choices=("rolling_rebuild", "fixed_incremental", "rolling_incremental"),
        default="rolling_rebuild",
    )
    args = parser.parse_args()
    if args.max_projections <= 0:
        parser.error("positive declared prefix required")
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        archive = args.archive_root.resolve(strict=True)
        source, capture, loaded = [p.resolve(strict=True) for p in (args.source, args.capture, args.loaded_contact)]
        if not all(p.is_relative_to(archive) for p in (source, capture, loaded)):
            raise ValueError("all diagnostic inputs must reside on declared archive")
        patches, input_hashes = loaded_patches(capture, loaded)
        # Freeze all local candidate/replay dependencies, including observer and
        # geometry helper sources. No mutable live-code-only manifest.
        names = [
            "diagnose_local_support.py",
            "replay_local_terrain.py",
            "archive_guard.py",
            "build_local_terrain.py",
            "layered_terrain.py",
            "native_chunk_transport.py",
            "revision_evidence.py",
            "shadow_terrain.py",
            "raw_return_filter.py",
            "terrain_shadow_runtime.py",
            "terrain_shadow_core.py",
        ]
        sources = [Path(__file__).with_name(n) for n in names]
        sources += sorted(Path(__file__).with_name("local_terrain").glob("*.py"))
        source_hashes = {str(p.resolve()): sha(p) for p in sources}
        for path in sources:
            target = output / "source" / path.relative_to(Path(__file__).parent)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
        contract = dict(
            assumed_patch_full_size_m=[0.08, 0.12, 0.002],
            patches=[
                dict(center=p.center, half_extent=p.half_extent, source_identity=p.source_identity) for p in patches
            ],
            motions=["stationary", "forward_10cm", "left_0.1rad", "right_0.1rad"],
            scope=__doc__,
            acceptance=False,
        )
        with (output / "manifest.json").open("x") as stream:
            json.dump(
                dict(
                    arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                    source_sha256=source_hashes,
                    input_sha256=input_hashes,
                    contract=contract,
                ),
                stream,
                indent=2,
            )
        totals = {
            name: dict(queries=0, supported=0, blocked=0, unknown=0, cells=0, free_cells=0, outside_cells=0)
            for name in contract["motions"]
        }
        with (output / "support.jsonl").open("x") as sink:

            def observer(layers, record):
                start = time.perf_counter()
                results = {}
                for name, poses in diagnostic_motions(record["local_from_base"]).items():
                    result = support_availability(layers, record["origin"], 0.1, poses, patches)
                    results[name] = result
                    counts = totals[name]
                    counts["queries"] += 1
                    counts[result["support_class_state"]] += 1
                    for key in ("cells", "free_cells", "outside_cells"):
                        counts[key] += result[key]
                sink.write(
                    json.dumps(
                        dict(
                            evidence_ns=record["evidence_ns"],
                            publication_ns=record["publication_ns"],
                            layers_sha256=record["layers_sha256"],
                            motions=results,
                            diagnostic_ms=(time.perf_counter() - start) * 1000,
                        ),
                        allow_nan=False,
                    )
                    + "\n"
                )

            result = replay(
                source,
                capture,
                archive,
                output,
                args.window,
                args.diagnose_failed_prefix,
                args.max_projections,
                backend="prepared",
                candidate_library=args.candidate_library,
                projection_observer=observer,
                map_policy=args.map_policy,
                evidence_lifetime=args.evidence_lifetime,
                cells=200 if args.map_policy == "fixed_incremental" else 60,
                observation_policy="frozen_observation" if args.map_policy != "rolling_rebuild" else "coherent",
            )
        for path, digest in {**source_hashes, **input_hashes}.items():
            if sha(path) != digest:
                raise ValueError("support diagnostic source/input changed")
        for counts in totals.values():
            counts["supported_fraction"] = counts["supported"] / counts["queries"]
            counts["free_cell_fraction"] = counts["free_cells"] / counts["cells"]
        result.update(
            support_availability=totals,
            support_sha256=sha(output / "support.jsonl"),
            support_contract=contract,
            compute_acceptance=False,
            limitation="assumed-patch map-class coverage; NOT independently safe motion availability",
        )
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(totals, indent=2), flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
