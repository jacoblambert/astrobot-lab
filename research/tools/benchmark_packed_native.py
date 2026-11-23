#!/usr/bin/env python3
"""All-tick byte verification and paired directory-scan timing; no motion.

A failed source remains failed. This only tests the storage representation and
exact quota enumeration overhead, not simulator deadline or navigation acceptance.
"""

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import tarfile
import time

from archive_guard import allocate_run, archive_size, check_run
from native_archive_io import PackedNativeArchive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    check_run(args.source)
    source_audit = args.source / "native_timing_audit.json"
    audit_hash = hashlib.sha256(source_audit.read_bytes()).hexdigest()
    frames = json.loads(source_audit.read_text())["frames"]
    output = allocate_run(args.archive_root, limit=8 * 1024**3)
    print(f"Run: {output}", flush=True)
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    directory = output / "native_chunks"
    directory.mkdir()
    try:
        packed = PackedNativeArchive(directory)
        for i, frame in enumerate(frames):
            for name, expected in (
                (frame["current_pose"]["hit_records_file"], frame["current_pose"]["hit_records_sha256"]),
                (frame["file"], frame["sha256"]),
            ):
                if Path(name).name != name:
                    raise ValueError("invalid logical input path")
                payload = (args.source / "native_chunks" / name).read_bytes()
                if hashlib.sha256(payload).hexdigest() != expected:
                    raise ValueError("source member changed")
                packed.write(name, payload)
            if i % 1200 == 0:
                check_run(output)
        packed.close()
        # Independent standard TAR reader, every member and its original hash.
        with tarfile.open(directory / "native.tar", "r|") as archive:
            count = 0
            for member in archive:
                frame = frames[count // 2]
                name, expected = (
                    (frame["current_pose"]["hit_records_file"], frame["current_pose"]["hit_records_sha256"])
                    if count % 2 == 0
                    else (frame["file"], frame["sha256"])
                )
                if member.name != name or hashlib.sha256(archive.extractfile(member).read()).hexdigest() != expected:
                    raise ValueError("packed logical bytes differ")
                count += 1
            if count != 2 * len(frames):
                raise ValueError("packed member denominator differs")
        timings = {"legacy_ms": [], "packed_ms": []}
        sizes = {}
        for i in range(10):
            for name, path in (("legacy_ms", args.source / "native_chunks"), ("packed_ms", directory))[
                :: 1 if i % 2 == 0 else -1
            ]:
                started = time.perf_counter()
                sizes[name] = archive_size(path)
                timings[name].append((time.perf_counter() - started) * 1000)
        if hashlib.sha256(source_audit.read_bytes()).hexdigest() != audit_hash:
            raise ValueError("source audit changed")
        check_run(output)
        report = dict(
            passed=True,
            acceptance=False,
            failed_source_remains_failed=True,
            source=str(args.source),
            source_audit_sha256=audit_hash,
            ticks=len(frames),
            verified_logical_files=count,
            live_file_counts=dict(legacy=count, packed=1),
            sizes=sizes,
            all_timings_ms=timings,
            median_ms={name: statistics.median(values) for name, values in timings.items()},
            scope="offline cache-warm paired quota enumeration; not live compute acceptance",
        )
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
