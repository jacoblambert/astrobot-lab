#!/usr/bin/env python3
"""Materialize byte-identical logical native files from a closed packed capture.

Never run during acquisition. Validate the whole source before creating files;
never overwrite, remove, or modify recorded evidence. Existing audit/replay
tools then read the original logical NPZ/JSON bytes with their original hashes.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import tarfile
import tempfile

from archive_guard import check_capacity, check_run
from native_archive_io import MAX_ARRAY_BYTES


def digest(path):
    with path.open("rb") as stream:
        result = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
        return result.hexdigest()


def members(path, expected):
    count = 0
    with tarfile.open(path, "r|") as archive:
        for entry in archive:
            if count >= len(expected) or entry.name != expected[count][0] or not entry.isfile():
                raise ValueError("unexpected packed native member/order/type")
            if not 0 < entry.size <= MAX_ARRAY_BYTES + 65536:
                raise ValueError("unbounded packed native member")
            payload = archive.extractfile(entry).read()
            if len(payload) != entry.size or hashlib.sha256(payload).hexdigest() != expected[count][1]:
                raise ValueError("packed native logical bytes changed")
            count += 1
            yield entry.name, payload
    if count != len(expected):
        raise ValueError("missing packed native members")


def restore(run):
    run = Path(run)
    storage_path = run / "native_storage.json"
    if not storage_path.exists():
        return dict(layout="legacy", materialized=0)
    check_run(run)
    if not ((run / "natural_holdout.json").exists() or (run / "failure.json").exists()):
        raise ValueError("closed native capture required before materialization")
    link = json.loads((run / "shadow_link.json").read_text())
    recorder = link["first_grant"]["recorder"]
    if not re.fullmatch(r"revision-[0-9a-f]{32}", recorder):
        raise ValueError("invalid recorder identity")
    collection = json.loads((run.parent / recorder / "collection.json").read_text())
    if type(collection.get("collection_completed")) is not bool:
        raise ValueError("closed recorder required before materialization")
    metadata = json.loads(storage_path.read_text())
    if (
        metadata.get("schema") != 1
        or metadata.get("layout") != "packed_native_tar_v1"
        or metadata.get("file") != "native_chunks/native.tar"
        or metadata.get("logical_bytes_unchanged") is not True
    ):
        raise ValueError("unknown native storage contract")
    directory, packed = run / "native_chunks", run / metadata["file"]
    if directory.is_symlink() or packed.is_symlink() or not packed.is_file():
        raise ValueError("packed native source must be a regular local file")
    before = packed.stat()
    if before.st_size != metadata["bytes"] or digest(packed) != metadata["sha256"]:
        raise ValueError("packed native source changed")
    frames = json.loads((run / "native_timing_audit.json").read_text())["frames"]
    if not 0 < len(frames) <= 100000 or metadata["members"] != 2 * len(frames):
        raise ValueError("packed native denominator mismatch")
    expected = []
    for index, frame in enumerate(frames):
        name = f"chunk_{index:06}"
        surface = frame["current_pose"]
        if frame["file"] != name + ".npz" or surface["hit_records_file"] != name + ".hits.json":
            raise ValueError("unexpected logical native paths")
        expected.extend([(name + ".hits.json", surface["hit_records_sha256"]), (name + ".npz", frame["sha256"])])
    # First pass validates every member and exact capacity before any writes.
    missing, payload_bytes, tar_bytes = 0, 0, 1024
    for name, payload in members(packed, expected):
        payload_bytes += len(payload)
        tar_bytes += 512 + ((len(payload) + 511) // 512) * 512
        target = directory / name
        if target.is_symlink():
            raise ValueError("logical native destination is a symlink")
        if target.exists():
            if not target.is_file() or digest(target) != hashlib.sha256(payload).hexdigest():
                raise ValueError("existing logical native file differs; refusing overwrite")
        else:
            missing += len(payload)
    if payload_bytes != metadata["payload_bytes"] or tar_bytes != before.st_size:
        raise ValueError("packed native size/footer contract mismatch")
    contract = json.loads((run / "archive_contract.json").read_text())
    if check_run(run) + missing >= contract["limit_bytes"]:
        raise ValueError("materialization exceeds native archive quota")
    check_capacity(contract["root"], contract["reserve_bytes"] + missing, external=contract["external_required"])
    created = 0
    for name, payload in members(packed, expected):
        target = directory / name
        if target.exists():
            continue  # Independently checked in the first pass; never overwrite.
        with tempfile.NamedTemporaryFile(prefix=".native-", dir=directory, delete=False) as stream:
            temporary = Path(stream.name)
            try:
                if stream.write(payload) != len(payload):
                    raise OSError("partial native materialization write")
                stream.flush()
                os.fsync(stream.fileno())
                os.chmod(temporary, 0o644)
                os.link(temporary, target)  # Atomic no-overwrite installation.
                if digest(target) != hashlib.sha256(payload).hexdigest():
                    raise ValueError("materialized native bytes differ on disk")
                created += 1
            finally:
                temporary.unlink()
    after = packed.stat()
    if (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns) != (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
        before.st_ctime_ns,
    ):
        raise ValueError("packed source changed during materialization")
    check_run(run)
    return dict(
        layout=metadata["layout"],
        materialized=created,
        logical_files=len(expected),
        packed_sha256=metadata["sha256"],
        source_retained=True,
        acceptance=False,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    print(json.dumps(restore(args.capture)), flush=True)


if __name__ == "__main__":
    main()
