#!/usr/bin/env python3
"""Compare both writers on a fixed 100-tick prefix; failed sources stay failed."""

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from archive_guard import allocate_run, check_run
from native_archive_io import write_native_npz


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    check_run(args.source)
    frames = json.loads((args.source / "native_timing_audit.json").read_text())["frames"][:100]
    if len(frames) != 100:
        raise ValueError("100 source ticks required; do not shorten the comparison")
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    paths = [Path(__file__), Path(__file__).with_name("native_archive_io.py"), args.source / "native_timing_audit.json"]
    hashes = {str(p): sha(p) for p in paths}
    rows = []
    try:
        for i, frame in enumerate(frames):
            source = args.source / "native_chunks" / frame["file"]
            if source.name != frame["file"] or sha(source) != frame["sha256"]:
                raise ValueError("native source hash/path mismatch")
            with np.load(source, allow_pickle=False) as data:
                arrays = {name: data[name] for name in data.files}
            timings, digests = {}, {}
            # Alternate order without selecting or discarding any trial.
            for kind in ("legacy", "candidate") if i % 2 == 0 else ("candidate", "legacy"):
                target = output / f"{i:03d}_{kind}.npz"
                start = time.perf_counter()
                if kind == "legacy":
                    np.savez_compressed(target, **arrays)
                    digest = sha(target)
                else:
                    digest = write_native_npz(target, **arrays)
                timings[kind] = (time.perf_counter() - start) * 1000
                digests[kind] = digest
                if sha(target) != digest:
                    raise ValueError("independent disk hash mismatch")
            if digests["legacy"] != digests["candidate"] or sha(source) != frame["sha256"]:
                raise ValueError("writer bytes differ or source changed")
            rows.append(dict(index=i, input_sha256=frame["sha256"], output_sha256=digests["candidate"], ms=timings))
        if any(sha(Path(p)) != value for p, value in hashes.items()):
            raise ValueError("benchmark inputs changed")
        check_run(output)
        result = dict(
            passed=True,
            acceptance=False,
            failed_source_remains_ineligible=True,
            input_sha256=hashes,
            numpy_version=np.__version__,
            rows=rows,
            timing_ms={
                name: dict(
                    zip(("p50", "p95", "max"), map(float, np.percentile([r["ms"][name] for r in rows], [50, 95, 100])))
                )
                for name in ("legacy", "candidate")
            },
            note="Offline filesystem-cache timing; not concurrent Isaac deadline acceptance.",
        )
        (output / "report.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result["timing_ms"]), flush=True)
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
