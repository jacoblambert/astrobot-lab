import hashlib
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from native_archive_io import MAX_ARRAY_BYTES, write_native_npz


def test_exact_legacy_npz_bytes_and_no_readback(tmp_path, monkeypatch):
    arrays = dict(
        points=np.arange(31983, dtype=np.float32).reshape(-1, 3),
        depths=np.linspace(0, 100, 10661, dtype=np.float32),
        intensities=np.full(10661, 255, np.uint8),
        observed_stamp=83.95,
        world_from_lidar=np.eye(4),
    )
    original = {k: np.asarray(v).copy() for k, v in arrays.items()}
    legacy, candidate = tmp_path / "legacy.npz", tmp_path / "candidate.npz"
    np.savez_compressed(legacy, **arrays)
    with monkeypatch.context() as m:
        m.setattr(Path, "read_bytes", lambda *args: pytest.fail("writer must not reread disk"))
        real_open = Path.open

        def only_write(self, mode, *args, **kwargs):
            assert mode == "xb"
            return real_open(self, mode, *args, **kwargs)

        m.setattr(Path, "open", only_write)
        digest = write_native_npz(candidate, **arrays)
    assert candidate.read_bytes() == legacy.read_bytes()
    assert digest == hashlib.sha256(candidate.read_bytes()).hexdigest()
    with np.load(candidate, allow_pickle=False) as result:
        for name, array in original.items():
            np.testing.assert_array_equal(result[name], array)
            np.testing.assert_array_equal(arrays[name], array)


def test_refuses_overwrite_and_out_of_bound_or_object_arrays(tmp_path):
    path = tmp_path / "native.npz"
    path.write_bytes(b"original")
    with pytest.raises(FileExistsError):
        write_native_npz(path, points=np.zeros((1, 3)))
    assert path.read_bytes() == b"original"
    for value in [np.zeros(MAX_ARRAY_BYTES + 1, dtype=np.uint8), np.array([object()])]:
        with pytest.raises(ValueError):
            write_native_npz(tmp_path / "not_written.npz", data=value)
        assert not (tmp_path / "not_written.npz").exists()


def test_write_error_propagates_without_success_digest(tmp_path, monkeypatch):
    def broken(*args, **kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(Path, "open", broken)
    with pytest.raises(OSError, match="disk full"):
        write_native_npz(tmp_path / "native.npz", points=np.zeros((1, 3)))


@pytest.mark.parametrize("fault", ["short", "close"])
def test_short_write_or_close_error_cannot_return_hash(tmp_path, monkeypatch, fault):
    class Stream:
        def __enter__(self):
            return self

        def write(self, data):
            return len(data) - (fault == "short")

        def __exit__(self, *args):
            if fault == "close":
                raise OSError("flush failed")

    monkeypatch.setattr(Path, "open", lambda *args: Stream())
    with pytest.raises(OSError, match="partial native|flush failed"):
        write_native_npz(tmp_path / "native.npz", points=np.zeros((1, 3)))
