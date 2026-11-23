#!/usr/bin/env python3
"""Single candidate query microbenchmark; not coupled mapper/live qualification.

Fixed dense-flat and unknown/support profiles, all warmup and measured timings
retained. Does not select the best repeat or alter a geometry/safety threshold.
"""

import argparse
import hashlib
import json
from pathlib import Path
import resource
import shutil
import time

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.contact_index import IncrementalSupportIndex
from local_terrain.contact_query import PreparedMotion, evaluate_support_core
from local_terrain.physx_reference import BodyBox


def profiles():
    from scipy.spatial.transform import Rotation

    patches = [
        BodyBox((x, y, -0.145), (0.04, 0.06, 0.001), "assumed_wheel")
        for x in (-0.256, 0.256)
        for y in (-0.2854, 0.2854)
    ]
    core = [BodyBox(p.center, (0.02, 0.03, 0.001), "experimental_core") for p in patches]
    body = BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "original_guard")
    record = dict(origin=[-3, -3], resolution=0.1)
    motions = []
    for distance, angle in ((0.05, 0), (0, -5), (0, 5)):
        end = np.eye(4)
        end[0, 3] = distance
        end[:3, :3] = Rotation.from_euler("z", angle, degrees=True).as_matrix()
        motions.append([np.eye(4), end])
    square = np.array([[-0.5037, -0.3454], [0.5037, -0.3454], [0.5037, 0.3454], [-0.5037, 0.3454]])
    for name, classes, polygons in (
        ("dense_flat", 0, []),
        ("unknown_with_startup", -1, [(square, "startup")]),
    ):
        layers = np.zeros((60, 60, 6))
        layers[:, :, 0] = classes
        index = IncrementalSupportIndex().update(polygons, record["origin"], record["resolution"])
        yield name, layers, record, patches, core, body, index, motions


def benchmark(*, warmup=12, samples=120):
    if type(warmup) is not int or type(samples) is not int or warmup < 0 or samples < 1:
        raise ValueError("nonnegative warmup and positive samples required")
    results, rows = {}, []
    for name, layers, record, patches, core, body, index, motions in profiles():
        for policy in ("full_patch", "support_core"):
            timings = []
            for tick in range(warmup + samples):
                poses = motions[tick % len(motions)]
                started = time.perf_counter_ns()
                full = PreparedMotion(layers, record, poses, patches, body)
                decision = (
                    evaluate_support_core(full, PreparedMotion(layers, record, poses, core, body), index)
                    if policy == "support_core"
                    else full.evaluate(index)
                )
                milliseconds = (time.perf_counter_ns() - started) / 1e6
                rows.append(
                    dict(
                        profile=name,
                        policy=policy,
                        tick=tick,
                        warmup=tick < warmup,
                        motion=tick % 3,
                        query_ms=milliseconds,
                        state=decision["state"],
                    )
                )
                if tick >= warmup:
                    timings.append(milliseconds)
            p50, p95, p99, maximum = map(float, np.percentile(timings, [50, 95, 99, 100]))
            results[name + "/" + policy] = dict(
                samples=samples,
                p50_ms=p50,
                p95_ms=p95,
                p99_ms=p99,
                max_ms=maximum,
                standalone_50_100_ms_budget_pass=p95 < 50 and p99 < 100,
            )
    return results, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    print(f"Run: {output}", flush=True)
    try:
        import shapely

        root = Path(__file__).parent
        sources = [Path(__file__)] + [
            root / "local_terrain" / n
            for n in (
                "contact_query.py",
                "contact_index.py",
                "contact_support.py",
                "support_sweep.py",
                "physx_reference.py",
            )
        ]
        hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
        for path in sources:
            target = output / "source" / path.relative_to(root)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
        manifest = dict(
            protocol="single_contact_query_v1",
            warmup=12,
            samples=120,
            source_sha256=hashes,
            shapely_version=shapely.__version__,
            geos_version=shapely.geos_version_string,
            acceptance=False,
        )
        with (output / "manifest.json").open("x") as stream:
            json.dump(manifest, stream, indent=2)
        results, rows = benchmark()
        with (output / "timings.jsonl").open("x") as stream:
            for row in rows:
                stream.write(json.dumps(row) + "\n")
        if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != h for p, h in hashes.items()):
            raise ValueError("benchmark source changed")
        report = dict(
            completed=True,
            results=results,
            peak_process_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
            acceptance=False,
            navigation_authority=False,
            limits=[
                "synthetic fixed profiles",
                "single candidate only",
                "excludes memory update and pose lookup",
                "excludes terrain mapper, ROS and controller",
            ],
            timings_sha256=hashlib.sha256((output / "timings.jsonl").read_bytes()).hexdigest(),
        )
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps(report, indent=2), flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
