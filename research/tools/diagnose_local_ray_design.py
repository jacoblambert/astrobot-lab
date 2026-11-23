#!/usr/bin/env python3
"""Archived fixed-budget vertical-ray redesign; geometry only, not qualification."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.ray_design import evaluate, hazard_bank, layouts, level_geometry, ray_mount


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mount_study", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--diagnose-failed-prefix", action="store_true")
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        archive, source = args.archive_root.resolve(strict=True), args.mount_study.resolve(strict=True)
        if not source.is_relative_to(archive) or (source / "failure.json").exists():
            raise ValueError("completed mounting study on archive required")
        contract = json.loads((source / "manifest.json").read_text())
        if contract["protocol"] != "ideal_mount_sampling_v1":
            raise ValueError("unexpected source protocol")
        if contract["source_failed"] and not args.diagnose_failed_prefix:
            raise ValueError("failed historical source requires explicit diagnostic flag")
        inputs = {str(source / name): sha(source / name) for name in ("manifest.json", "report.json")}
        inputs.update(contract["input_sha256"])
        for path, digest in inputs.items():
            if not Path(path).resolve(strict=True).is_relative_to(archive) or sha(path) != digest:
                raise ValueError("source input changed or outside archive")
        contract = level_geometry(contract)
        ray_mount(contract, require_level=True)
        designs = layouts(contract["native_zenith_radians"])
        sources = [
            Path(__file__),
            Path(__file__).with_name("archive_guard.py"),
            Path(__file__).with_name("revision_evidence.py"),
        ]
        sources += [Path(__file__).with_name("local_terrain") / name for name in ("mounting_study.py", "ray_design.py")]
        hashes = {str(p): sha(p) for p in sources}
        for p in sources:
            target = output / "source" / p.relative_to(Path(__file__).parent)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, target)
        manifest = dict(
            protocol="single_lidar_ray_design_v2_level",
            geometry=contract,
            layouts=designs,
            hazard_bank=hazard_bank(),
            shifted_hazard_bank=hazard_bank(shifted=True),
            input_sha256=inputs,
            source_sha256=hashes,
            acceptance=False,
            navigation_authority=False,
            note="Zero-pitch mount: sensor elevations equal base elevations, positive up. Recorded 5-degree calibration retained separately, not used for rays. Fixed count/range/regions. No occlusion/noise/SLAM.",
        )
        with (output / "manifest.json").open("x") as stream:
            json.dump(manifest, stream, indent=2, allow_nan=False)
        results = {}
        for name, elevation in designs.items():
            results[name] = evaluate(contract, elevation)
            with (output / f"{name}.json").open("x") as stream:
                json.dump(results[name], stream, indent=2, allow_nan=False)
            print(
                json.dumps(
                    dict(
                        layout=name,
                        rays=results[name]["rays_per_revolution"],
                        below_min=results[name]["plane_rays_below_minimum"],
                        hazard_cases_hit=sum(c["ray_hits"] > 0 for c in results[name]["hazard_bank"]),
                    )
                ),
                flush=True,
            )
            check_run(output)
        if any(sha(p) != digest for p, digest in {**inputs, **hashes}.items()):
            raise ValueError("input/source changed during screen")
        with (output / "report.json").open("x") as stream:
            json.dump(
                dict(results=results, acceptance=False, navigation_authority=False), stream, indent=2, allow_nan=False
            )
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
