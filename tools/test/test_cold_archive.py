import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cold_archive as cold


def fixture(tmp_path, monkeypatch):
    run = tmp_path / ("revision-" + "a" * 32)
    (run / "bag").mkdir(parents=True)
    (run / "collection.json").write_text(json.dumps(dict(collection_completed=False, failure="preserved")))
    path = run / "bag/bag_0.mcap"
    path.write_bytes(b"unchanged failed recording bytes\0" * 10000)
    monkeypatch.setattr(cold, "check_capacity", lambda *a: None)
    return path


def test_lossless_pack_and_restore_preserves_failed_evidence(tmp_path, monkeypatch):
    path = fixture(tmp_path, monkeypatch)
    original, old = path.read_bytes(), path.stat()
    record = cold.pack(path, tmp_path)
    assert not path.exists() and record["sha256"] == cold.decoded_digest(Path(str(path) + ".zst"))
    assert record["compressed_bytes"] < record["original_bytes"]
    cold.restore(path, tmp_path)
    assert path.read_bytes() == original and path.stat().st_mtime_ns == old.st_mtime_ns
    with pytest.raises(ValueError, match="already exist"):
        cold.pack(path, tmp_path)
    with pytest.raises(ValueError, match="already exists"):
        cold.restore(path, tmp_path)


@pytest.mark.parametrize("problem", ["symlink", "outside", "metadata", "unsealed"])
def test_rejects_unresolved_or_unapproved_targets(tmp_path, monkeypatch, problem):
    path = fixture(tmp_path, monkeypatch)
    if problem == "symlink":
        link = path.with_name("link.mcap")
        link.symlink_to(path)
        path = link
    elif problem == "outside":
        path = tmp_path.parent / "other.mcap"
    elif problem == "metadata":
        path = path.parent.parent / "collection.json"
    else:
        (path.parent.parent / "collection.json").unlink()
    with pytest.raises((ValueError, FileNotFoundError)):
        cold.pack(path, tmp_path)


def test_changed_source_is_not_removed(tmp_path, monkeypatch):
    path = fixture(tmp_path, monkeypatch)
    original = cold.decoded_digest

    def mutate(compressed):
        value = original(compressed)
        with path.open("ab") as stream:
            stream.write(b"late writer")
        return value

    monkeypatch.setattr(cold, "decoded_digest", mutate)
    with pytest.raises(ValueError, match="source changed"):
        cold.pack(path, tmp_path)
    assert path.exists()


def test_corrupt_compressed_data_cannot_replace_original(tmp_path, monkeypatch):
    path = fixture(tmp_path, monkeypatch)
    cold.pack(path, tmp_path)
    Path(str(path) + ".zst").write_bytes(b"corrupt")
    with pytest.raises(ValueError, match="checksum mismatch"):
        cold.restore(path, tmp_path)
    assert not path.exists()


def test_failed_round_trip_preserves_original(tmp_path, monkeypatch):
    path = fixture(tmp_path, monkeypatch)
    original = path.read_bytes()
    monkeypatch.setattr(cold, "decoded_digest", lambda _: "wrong")
    with pytest.raises(ValueError, match="round-trip checksum"):
        cold.pack(path, tmp_path)
    assert path.read_bytes() == original
    assert not Path(str(path) + ".cold.json").exists()


def test_reserve_failure_does_not_start_compression(tmp_path, monkeypatch):
    path = fixture(tmp_path, monkeypatch)

    def reject(*args):
        raise ValueError("reserve")

    monkeypatch.setattr(cold, "check_capacity", reject)
    with pytest.raises(ValueError, match="reserve"):
        cold.pack(path, tmp_path)
    assert path.exists() and not Path(str(path) + ".zst").exists()
