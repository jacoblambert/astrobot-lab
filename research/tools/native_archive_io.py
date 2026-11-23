"""Bounded native NPZ serialization without a synchronous disk read-back.

Legacy file/schema bytes stay unchanged. This is an intermediate I/O fix, not
the diagnostic-topic migration. Offline evaluators still independently hash disk.
"""

import hashlib
import io
import json
from pathlib import Path
import tarfile

import numpy as np

MAX_ARRAY_BYTES = 2 * 1024**2


class PackedNativeArchive:
    """Two exact logical files per tick in one bounded-memory append-only TAR.

    Do not retain TarInfo objects or an index on the simulator's GC heap.
    Existing per-tick hashes/filenames stay unchanged. Offline materialization
    validates the entire sealed stream before creating their byte-identical files.
    """

    def __init__(self, directory):
        self.directory = Path(directory)
        if self.directory.name != "native_chunks":
            raise ValueError("packed native output requires native_chunks directory")
        self.path = self.directory / "native.tar"
        self.stream = self.path.open("xb", buffering=0)
        self.digest = hashlib.sha256()
        self.members = self.bytes = self.payload_bytes = 0
        self.closed = False

    def _write(self, payload):
        if self.stream.write(payload) != len(payload):
            raise OSError("partial packed native write")
        self.digest.update(payload)
        self.bytes += len(payload)

    def write(self, name, payload):
        suffix = "hits.json" if self.members % 2 == 0 else "npz"
        expected = f"chunk_{self.members // 2:06}.{suffix}"
        if self.closed or name != expected or not 0 < len(payload) <= MAX_ARRAY_BYTES + 65536:
            raise ValueError("invalid packed native member/order/size")
        entry = tarfile.TarInfo(name)
        entry.size, entry.mode = len(payload), 0o644
        self._write(entry.tobuf(format=tarfile.USTAR_FORMAT))
        self._write(payload)
        self._write(b"\0" * (-len(payload) % 512))
        self.members += 1
        self.payload_bytes += len(payload)

    def close(self):
        if self.closed:
            return
        if self.members % 2:
            raise ValueError("incomplete packed native tick")
        self._write(b"\0" * 1024)
        self.stream.close()
        self.closed = True
        with (self.directory.parent / "native_storage.json").open("x") as stream:
            json.dump(
                dict(
                    schema=1,
                    layout="packed_native_tar_v1",
                    file="native_chunks/native.tar",
                    members=self.members,
                    payload_bytes=self.payload_bytes,
                    bytes=self.bytes,
                    sha256=self.digest.hexdigest(),
                    logical_bytes_unchanged=True,
                ),
                stream,
                indent=2,
            )


def write_native_npz(path, *, archive=None, **arrays):
    values = {key: np.asarray(value) for key, value in arrays.items()}
    if any(value.dtype.hasobject for value in values.values()):
        raise ValueError("native archive cannot contain object arrays")
    if sum(value.nbytes for value in values.values()) > MAX_ARRAY_BYTES:
        raise ValueError("native serialization exceeds per-tick memory bound")
    with io.BytesIO() as encoded:
        np.savez_compressed(encoded, **values)
        # getbuffer avoids making a second copy of the compressed record.
        with encoded.getbuffer() as payload:
            if len(payload) > MAX_ARRAY_BYTES + 65536:
                raise ValueError("native serialized record exceeds memory bound")
            digest = hashlib.sha256(payload).hexdigest()
            if archive is not None:
                archive.write(Path(path).name, payload)
            else:
                with Path(path).open("xb") as stream:
                    if stream.write(payload) != len(payload):
                        raise OSError("partial native evidence write")
            return digest
