"""The launcher exiting or last live status is not proof of an archive seal."""

import hashlib
import json
import os
from pathlib import Path


class OutputDigest:
    """Hash exactly the bytes successfully written, without a shutdown re-read.

    Independent consumers still re-hash disk files before accepting a seal.
    Keep native evidence immutable and bound shutdown work regardless of length.
    """

    def __init__(self, path, *, binary=False):
        self.binary = binary
        self.stream = (
            Path(path).open("xb") if binary else Path(path).open("x", encoding="utf-8", newline="", buffering=1)
        )
        self.digest = hashlib.sha256()
        self.bytes = 0
        self.closed = False

    def write(self, value):
        encoded = value if self.binary else value.encode("utf-8")
        count = self.stream.write(value)
        if count != len(value):
            raise OSError("partial evidence write")
        self.digest.update(encoded)
        self.bytes += len(encoded)
        return count

    def tell(self):
        return self.stream.tell()

    def flush(self):
        self.stream.flush()

    def close(self):
        if self.closed:
            return
        self.stream.flush()
        if os.fstat(self.stream.fileno()).st_size != self.bytes:
            raise ValueError("evidence file size changed outside writer")
        self.stream.close()
        self.closed = True

    def hexdigest(self):
        if not self.closed:
            raise ValueError("unclosed evidence cannot be sealed")
        return self.digest.hexdigest()


def verify_seal(archive, live_status):
    archive = Path(archive).resolve(strict=True)
    run = Path(live_status["archive"]).resolve(strict=True)
    if run.parent != archive:
        raise ValueError("v2 seal escaped archive")
    report = json.loads((run / "collection.json").read_text())
    status = report["status"]
    if (
        report.get("experiment_completed") is not True
        or report.get("acceptance") is not False
        or status.get("failure") is not None
        or status.get("closed") is not True
        or status.get("valid") is not False
    ):
        raise ValueError("v2 mapper did not seal successfully")
    for field in ("capture", "received", "integrated", "projected", "library_sha256"):
        if status.get(field) != live_status.get(field):
            raise ValueError("v2 sealed state differs from live completion: " + field)
    hashes = report["output_sha256"]
    if set(hashes) != {"events.jsonl", "native.bin", "projections.jsonl"}:
        raise ValueError("v2 seal lacks complete payload hashes")
    for name, expected in hashes.items():
        digest = hashlib.sha256()
        with (run / name).open("rb") as stream:
            for block in iter(lambda: stream.read(1024**2), b""):
                digest.update(block)
        if digest.hexdigest() != expected:
            raise ValueError("v2 sealed payload changed: " + name)
    return report
