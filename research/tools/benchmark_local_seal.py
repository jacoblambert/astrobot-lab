#!/usr/bin/env python3
"""Verify bounded shutdown hashing on a large archived payload, not live acceptance."""

import argparse
import json
from pathlib import Path
import time

from archive_guard import allocate_run, check_run, ARCHIVE_ROOT
from local_terrain_seal import OutputDigest
from replay_local_terrain import sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    check_run(args.source.parent)
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    target = output / "native.bin"
    writer = OutputDigest(target, binary=True)
    started = time.perf_counter()
    with args.source.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            writer.write(block)
    write_s = time.perf_counter() - started
    started = time.perf_counter()
    writer.close()
    digest = writer.hexdigest()
    seal_ms = (time.perf_counter() - started) * 1000
    # Deliberately independent full disk reads after the bounded writer close.
    started = time.perf_counter()
    source_digest, target_digest = sha(args.source), sha(target)
    verify_s = time.perf_counter() - started
    passed = digest == source_digest == target_digest and seal_ms < 100
    report = dict(
        passed=passed,
        acceptance=False,
        bytes=writer.bytes,
        source=str(args.source),
        source_sha256=source_digest,
        written_sha256=digest,
        disk_sha256=target_digest,
        write_s=write_s,
        writer_seal_ms=seal_ms,
        independent_readback_s=verify_s,
        code_sha256={str(p): sha(p) for p in (Path(__file__), Path(__file__).with_name("local_terrain_seal.py"))},
        note="Large-file writer test only. Source eligibility unchanged; no simulator, complete mapper shutdown or live timing acceptance.",
    )
    check_run(output)
    with (output / "report.json").open("x") as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
