import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain_seal import OutputDigest, verify_seal


@pytest.mark.parametrize("binary", [False, True])
def test_streamed_digest_matches_exact_disk_bytes_without_shutdown_read(tmp_path, monkeypatch, binary):
    path = tmp_path / "output"
    writer = OutputDigest(path, binary=binary)
    values = [b"\x00\xff" * 1024, b"tail"] if binary else ['{"text":"µ🌕"}\n', "tail\n"]
    expected = b"".join(values) if binary else "".join(values).encode("utf-8")
    for value in values:
        writer.write(value)
        writer.flush()
    assert writer.tell() == len(expected)
    with pytest.raises(ValueError, match="unclosed"):
        writer.hexdigest()
    original = Path.open
    with monkeypatch.context() as patch:
        patch.setattr(Path, "open", lambda *a, **kw: pytest.fail("shutdown must not re-read evidence"))
        writer.close()
        assert writer.hexdigest() == hashlib.sha256(expected).hexdigest()
    with original(path, "rb") as stream:
        assert stream.read() == expected
    with pytest.raises(ValueError):
        writer.write(values[0])


def test_streamed_digest_detects_external_append_and_never_overwrites(tmp_path):
    path = tmp_path / "output"
    writer = OutputDigest(path, binary=True)
    writer.write(b"native")
    writer.flush()
    with path.open("ab") as stream:
        stream.write(b"unexpected")
    with pytest.raises(ValueError, match="size changed"):
        writer.close()
    writer.stream.close()
    with pytest.raises(FileExistsError):
        OutputDigest(path)


@pytest.mark.parametrize("fault", [None, "unsealed", "unclosed", "failure", "count", "modified"])
def test_seal_not_just_live_completion(tmp_path, fault):
    run = tmp_path / "revision-test"
    run.mkdir()
    live = dict(
        archive=str(run),
        capture="capture",
        received=3,
        integrated=3,
        projected=2,
        library_sha256="abc",
        valid=False,
        closed=True,
        failure=None,
    )
    status = live.copy()
    if fault == "unclosed":
        status["closed"] = False
    if fault == "failure":
        status["failure"] = "test"
    if fault == "count":
        status["integrated"] = 2
    hashes = {}
    for name in ("events.jsonl", "native.bin", "projections.jsonl"):
        (run / name).write_bytes(b"payload")
        hashes[name] = hashlib.sha256(b"payload").hexdigest()
    if fault != "unsealed":
        (run / "collection.json").write_text(
            json.dumps(dict(status=status, acceptance=False, experiment_completed=True, output_sha256=hashes))
        )
    if fault == "modified":
        (run / "native.bin").write_bytes(b"changed")
    if fault:
        with pytest.raises((ValueError, FileNotFoundError)):
            verify_seal(tmp_path, live)
    else:
        assert verify_seal(tmp_path, live)["experiment_completed"]
