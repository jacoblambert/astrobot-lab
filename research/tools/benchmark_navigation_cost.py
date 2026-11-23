#!/usr/bin/env python3
"""Fixed existing resource inputs plus candidate cost layer; not live acceptance."""

import argparse
import json
from pathlib import Path
import resource
import shutil
import time

import numpy as np

from archive_guard import allocate_run, check_run
from layered_terrain import TerrainMapper
from local_terrain.navigation_cost import navigation_costs
from replay_local_terrain import sha
from stress_terrain_resources import stress_chunks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--archive-root", type=Path, required=True)
    parser.add_argument("--profile", choices=("layer_capacity", "dense_flat"), required=True)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=256 * 1024**2)
    print(f"Run: {output}", flush=True)
    root = Path(__file__).parent
    sources = [
        Path(__file__),
        root / "stress_terrain_resources.py",
        root / "layered_terrain.py",
        root / "local_terrain/navigation_cost.py",
        root / "local_terrain/costmap.py",
    ]
    hashes = {str(p): sha(p) for p in [*sources, args.library]}
    for p in sources:
        target = output / "source" / p.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, target)
    rows = []
    try:
        chunks = stress_chunks(args.profile)
        with TerrainMapper(args.library) as mapper:
            for i in range(320):
                started = time.perf_counter()
                mapper.integrate(chunks[i % 10], i / 10, [0, 0, 0.5])
                layers = mapper.project(i / 10)
                projected = time.perf_counter()
                costs, mask = navigation_costs(layers)
                ended = time.perf_counter()
                rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
                rows.append(
                    dict(
                        stamp=i / 10,
                        mapper_ms=(projected - started) * 1000,
                        cost_ms=(ended - projected) * 1000,
                        total_ms=(ended - started) * 1000,
                        rss_mib=rss,
                        gap_cells=int(mask.sum()),
                    )
                )
                if rss > 2048:
                    raise ValueError("existing 2 GiB diagnostic stop bound exceeded")
                if (i + 1) % 50 == 0:
                    print(f"{i+1}/320 inputs", flush=True)
        summary = dict(
            profile=args.profile,
            inputs=320,
            points_per_input=48000,
            grid_cells=200,
            mapper_ms_p50_p95_p99_max=np.percentile([r["mapper_ms"] for r in rows], [50, 95, 99, 100]).tolist(),
            cost_ms_p50_p95_p99_max=np.percentile([r["cost_ms"] for r in rows], [50, 95, 99, 100]).tolist(),
            total_ms_p50_p95_p99_max=np.percentile([r["total_ms"] for r in rows], [50, 95, 99, 100]).tolist(),
            peak_process_rss_mib=rss,
            memory_passed=rss <= 1024,
            compute_passed=bool(
                np.percentile([r["total_ms"] for r in rows], 95) < 50
                and np.percentile([r["total_ms"] for r in rows], 99) < 100
            ),
            navigation_qualified=False,
            acceptance=False,
            limits="Kernel plus cost layer only; no GLIM/ROS/search. Existing fixed input/gates unchanged.",
            input_sha256=hashes,
        )
        if any(sha(Path(p)) != h for p, h in hashes.items()):
            raise ValueError("changed benchmark inputs")
        np.savez_compressed(output / "final_layers.npz", layers=layers, costs=costs, uncertainty=mask)
        (output / "summary.json").write_text(json.dumps(dict(**summary, frames=rows), indent=2))
        print(json.dumps(summary, indent=2), flush=True)
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), frames=rows)))
        raise
    check_run(output)


if __name__ == "__main__":
    main()
