#!/usr/bin/env python3
"""Explicit, lossless cold storage for closed recording files. Never prune runs.

Pack retains metadata and verified original bytes in .zst; replay requires
restore first. Only explicit MCAP/native.bin paths in external revision runs
are accepted. No wildcard expansion, recursive deletion, or automatic eviction.
"""

import argparse
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
from pathlib import Path

from archive_guard import ARCHIVE_ROOT, GIB, check_capacity


def stream_digest(stream):
    result = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        result.update(block)
    return result.hexdigest()


def digest(path):
    with path.open("rb") as stream:
        return stream_digest(stream)


def decoded_digest(path):
    with subprocess.Popen(["zstd", "-q", "-dc", "--", str(path)], stdout=subprocess.PIPE) as process:
        result = stream_digest(process.stdout)
        if process.wait():
            raise ValueError("compressed stream failed validation")
    return result


def validate_path(path, root):
    path, root = Path(path).absolute(), Path(root).resolve(strict=True)
    if path != path.resolve():
        raise ValueError("symlinks/noncanonical paths are not allowed")
    relative = path.relative_to(root)
    parts = relative.parts
    if not parts or not re.fullmatch(r"revision-[0-9a-f]{32}", parts[0]):
        raise ValueError("explicit revision-run file required")
    if not (
        len(parts) == 2
        and parts[1] == "native.bin"
        or len(parts) == 3
        and parts[1] == "bag"
        and parts[2].endswith(".mcap")
    ):
        raise ValueError("only MCAP or native.bin data may enter cold storage")
    run = root / parts[0]
    closed = json.loads((run / "collection.json").read_text())
    if not any(k in closed for k in ("collection_completed", "experiment_completed")):
        raise ValueError("closed recording seal required, including failed outcomes")
    return path


def unchanged(path, before):
    after = path.stat()
    if (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns) != (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
        before.st_ctime_ns,
    ):
        raise ValueError("source changed; retaining original")


def pack(path, root):
    path = validate_path(path, root)
    before = path.stat()
    if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
        raise ValueError("regular singly linked source required")
    check_capacity(root, 50 * GIB + before.st_size + GIB)
    compressed, metadata = Path(str(path) + ".zst"), Path(str(path) + ".cold.json")
    if compressed.exists() or metadata.exists():
        raise ValueError("cold outputs already exist; inspect rather than overwrite")
    original_sha = digest(path)
    subprocess.run(["zstd", "-q", "-3", "-T2", "--no-progress", "-o", str(compressed), "--", str(path)], check=True)
    if decoded_digest(compressed) != original_sha:
        raise ValueError("round-trip checksum mismatch; retaining original")
    unchanged(path, before)
    if compressed.stat().st_size >= before.st_size:
        raise ValueError("compression did not save space; retaining original")
    record = dict(
        schema=1,
        original=str(path),
        sha256=original_sha,
        original_bytes=before.st_size,
        compressed_bytes=compressed.stat().st_size,
        compressed_sha256=digest(compressed),
        mode=stat.S_IMODE(before.st_mode),
        mtime_ns=before.st_mtime_ns,
        uid=before.st_uid,
        gid=before.st_gid,
        verified_round_trip=True,
        evidence_deleted=False,
    )
    with metadata.open("x") as stream:
        json.dump(record, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    with compressed.open("rb") as stream:
        os.fsync(stream.fileno())
    # Persist replacement names before removing the recoverable hot copy.
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
        unchanged(path, before)
        path.unlink()
        os.fsync(directory)
    finally:
        os.close(directory)
    return record


def restore(path, root):
    path = validate_path(path, root)
    compressed, metadata = Path(str(path) + ".zst"), Path(str(path) + ".cold.json")
    record = json.loads(metadata.read_text())
    if record["original"] != str(path) or path.exists():
        raise ValueError("restore target differs or already exists")
    if digest(compressed) != record["compressed_sha256"]:
        raise ValueError("compressed archive checksum mismatch")
    check_capacity(root, 50 * GIB + record["original_bytes"] + GIB)
    temporary = Path(str(path) + ".restore-partial")
    if temporary.exists():
        raise ValueError("partial restore exists; inspect before retry")
    subprocess.run(["zstd", "-q", "-d", "-o", str(temporary), "--", str(compressed)], check=True)
    if temporary.stat().st_size != record["original_bytes"] or digest(temporary) != record["sha256"]:
        raise ValueError("restored bytes differ; refusing activation")
    os.chmod(temporary, record["mode"])
    os.utime(temporary, ns=(record["mtime_ns"], record["mtime_ns"]))
    with temporary.open("rb") as stream:
        os.fsync(stream.fileno())
    os.link(temporary, path)  # Atomic no-overwrite activation.
    temporary.unlink()
    return dict(restored=str(path), sha256=record["sha256"], compressed_copy_retained=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("pack", "restore"))
    parser.add_argument("paths", nargs="+", type=Path)
    args = parser.parse_args()
    action = pack if args.action == "pack" else restore
    for path in args.paths:
        print(json.dumps(action(path, ARCHIVE_ROOT)), flush=True)
    print(json.dumps(dict(free_gib=shutil.disk_usage(ARCHIVE_ROOT).free / GIB)), flush=True)


if __name__ == "__main__":
    main()
