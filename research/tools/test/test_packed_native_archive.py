"""Storage-only change: exact NPZ/first-hit bytes and strict closed-run restore."""

import hashlib
import io
import json
from pathlib import Path
import sys
import tarfile

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from archive_guard import allocate_run
from native_archive_io import PackedNativeArchive, write_native_npz
from restore_native_archive import restore


def fixture(tmp_path, ticks=3):
    run = allocate_run(tmp_path, limit=16 * 1024**2, reserve=0, external=False)
    recorder = allocate_run(tmp_path, limit=16 * 1024**2, reserve=0, external=False)
    (recorder / "collection.json").write_text(json.dumps(dict(collection_completed=True)))
    (run / "shadow_link.json").write_text(json.dumps(dict(first_grant=dict(recorder=recorder.name))))
    (run / "natural_holdout.json").write_text(json.dumps(dict(complete=True)))
    directory = run / "native_chunks"
    directory.mkdir()
    packed = PackedNativeArchive(directory)
    frames, originals = [], {}
    for i in range(ticks):
        hit_name, npz_name = f"chunk_{i:06}.hits.json", f"chunk_{i:06}.npz"
        hits = json.dumps([dict(point_index=0, residual_m=0.0001)]).encode()
        packed.write(hit_name, hits)
        arrays = dict(points=np.array([[i, 2, 3]], np.float32), depths=np.array([5.0]), observed_stamp=i / 60)
        legacy = io.BytesIO()
        np.savez_compressed(legacy, **arrays)
        digest = write_native_npz(directory / npz_name, archive=packed, **arrays)
        originals.update({hit_name: hits, npz_name: legacy.getvalue()})
        assert digest == hashlib.sha256(legacy.getvalue()).hexdigest()
        frames.append(
            dict(
                file=npz_name,
                sha256=digest,
                current_pose=dict(hit_records_file=hit_name, hit_records_sha256=hashlib.sha256(hits).hexdigest()),
            )
        )
    packed.close()
    (run / "native_timing_audit.json").write_text(json.dumps(dict(frames=frames)))
    return run, recorder, originals


def test_one_live_file_and_exact_idempotent_materialization(tmp_path):
    run, recorder, originals = fixture(tmp_path, ticks=1200)
    directory = run / "native_chunks"
    assert [p.name for p in directory.iterdir()] == ["native.tar"]
    packed_digest = hashlib.sha256((directory / "native.tar").read_bytes()).hexdigest()
    result = restore(run)
    assert result["materialized"] == 2400 and result["source_retained"]
    for name, expected in originals.items():
        assert (directory / name).read_bytes() == expected
    assert restore(run)["materialized"] == 0
    assert hashlib.sha256((directory / "native.tar").read_bytes()).hexdigest() == packed_digest
    assert not list(directory.glob(".native-*"))


@pytest.mark.parametrize("which", ["capture", "recorder"])
def test_no_materialization_beside_live_capture(tmp_path, which):
    run, recorder, _ = fixture(tmp_path)
    (run / "natural_holdout.json" if which == "capture" else recorder / "collection.json").unlink()
    with pytest.raises((ValueError, FileNotFoundError)):
        restore(run)
    assert len(list((run / "native_chunks").iterdir())) == 1


def test_closed_failed_capture_can_restore_without_becoming_accepted(tmp_path):
    run, recorder, _ = fixture(tmp_path)
    (run / "natural_holdout.json").unlink()
    (run / "failure.json").write_text('{"error":"original failure"}')
    (recorder / "collection.json").write_text('{"collection_completed":false}')
    assert restore(run)["acceptance"] is False
    assert json.loads((run / "failure.json").read_text())["error"] == "original failure"
    assert not (run / "natural_holdout.json").exists()


@pytest.mark.parametrize("fault", ["hash", "logical_hash", "missing", "path", "size", "overwrite", "symlink", "quota"])
def test_corruption_or_unsafe_output_refuses_before_writing(tmp_path, fault):
    run, _, originals = fixture(tmp_path)
    directory = run / "native_chunks"
    metadata_path = run / "native_storage.json"
    metadata = json.loads(metadata_path.read_text())
    audit_path = run / "native_timing_audit.json"
    audit = json.loads(audit_path.read_text())
    if fault == "hash":
        metadata["sha256"] = "0" * 64
    elif fault == "logical_hash":
        audit["frames"][-1]["sha256"] = "0" * 64
    elif fault == "missing":
        audit["frames"].pop()
    elif fault == "path":
        audit["frames"][0]["file"] = "../bad.npz"
    elif fault == "size":
        metadata["payload_bytes"] += 1
    elif fault == "overwrite":
        (directory / "chunk_000000.npz").write_bytes(b"user data")
    elif fault == "symlink":
        (directory / "chunk_000000.npz").symlink_to(directory / "native.tar")
    elif fault == "quota":
        path = run / "archive_contract.json"
        contract = json.loads(path.read_text())
        contract["limit_bytes"] = 100
        path.write_text(json.dumps(contract))
    metadata_path.write_text(json.dumps(metadata))
    audit_path.write_text(json.dumps(audit))
    before = set(directory.iterdir())
    with pytest.raises(ValueError):
        restore(run)
    assert set(directory.iterdir()) == before
    if fault == "overwrite":
        assert (directory / "chunk_000000.npz").read_bytes() == b"user data"


def test_packer_rejects_reorder_duplicate_traversal_incomplete_and_overwrite(tmp_path):
    directory = tmp_path / "native_chunks"
    directory.mkdir()
    packed = PackedNativeArchive(directory)
    for name in ("../chunk_000000.hits.json", "chunk_000000.npz", "chunk_000001.hits.json"):
        with pytest.raises(ValueError):
            packed.write(name, b"test")
    packed.write("chunk_000000.hits.json", b"[]")
    with pytest.raises(ValueError):
        packed.write("chunk_000000.hits.json", b"[]")
    with pytest.raises(ValueError, match="incomplete"):
        packed.close()
    packed.write("chunk_000000.npz", b"exact bytes")
    packed.close()
    with pytest.raises(FileExistsError):
        PackedNativeArchive(directory)
    with pytest.raises(ValueError):
        packed.write("chunk_000001.hits.json", b"[]")


def test_packer_retains_no_per_member_objects(tmp_path):
    directory = tmp_path / "native_chunks"
    directory.mkdir()
    packed = PackedNativeArchive(directory)
    for i in range(1200):
        packed.write(f"chunk_{i:06}.hits.json", b"[]")
        packed.write(f"chunk_{i:06}.npz", b"unchanged")
    assert not any(isinstance(v, (list, dict, set)) for v in vars(packed).values())
    packed.close()
    with tarfile.open(directory / "native.tar") as archive:
        assert len(archive.getmembers()) == 2400


def test_packer_propagates_short_write(tmp_path):
    directory = tmp_path / "native_chunks"
    directory.mkdir()
    packed = PackedNativeArchive(directory)
    original = packed.stream

    class Short:
        def write(self, value):
            return max(0, len(value) - 1)

    packed.stream = Short()
    with pytest.raises(OSError, match="partial"):
        packed.write("chunk_000000.hits.json", b"[]")
    original.close()
