"""Host NumPy/SciPy tests; optional TERRAIN_LIBRARY exercises the real C++ core."""
import copy
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from native_chunk_transport import DTYPE, FIELDS, decode, encode
from terrain_shadow_core import ShadowCore
from terrain_shadow_runtime import ReceiptScheduler

CAPTURE = "revision-" + "a" * 32


def wire(points=None, sequence=0):
    points = np.array([[1., 2., 3.], [4., 5., 6.]], dtype="f4") if points is None else points
    data = encode(CAPTURE, sequence, points, np.linalg.norm(points, axis=1), np.ones(len(points), dtype="u1"))
    return NS(header=NS(frame_id="vlp16", stamp=NS(sec=1, nanosec=23)), height=1, width=len(points),
              point_step=40, row_step=len(data), is_bigendian=False, is_dense=True, data=data,
              fields=[NS(name=n, offset=o, datatype=t, count=c) for n, o, t, c in FIELDS])


def test_native_wire_preserves_every_return_identity_and_has_no_gt():
    cloud = wire()
    decoded = decode(cloud)
    assert decoded["capture"] == CAPTURE and decoded["sequence"] == 0
    assert decoded["stamp_ns"] == 1_000_000_023
    assert np.array_equal(decoded["points"], [[1., 2., 3.], [4., 5., 6.]])
    assert set(DTYPE.names) == {"x", "y", "z", "depth", "intensity", "sequence", "capture_uuid"}


@pytest.mark.parametrize("mutation", ["frame", "endian", "stride", "fields", "sequence", "uuid", "nan", "stamp"])
def test_native_wire_rejects_malformed_or_mixed_messages(mutation):
    cloud = wire()
    if mutation == "frame": cloud.header.frame_id = "map"
    if mutation == "endian": cloud.is_bigendian = True
    if mutation == "stride": cloud.point_step = 39
    if mutation == "fields": cloud.fields[0].offset = 1
    if mutation == "stamp": cloud.header.stamp.nanosec = 10**9
    if mutation in ("sequence", "uuid", "nan"):
        rows = np.frombuffer(cloud.data, dtype=DTYPE).copy()
        if mutation == "sequence": rows["sequence"][1] = 1
        if mutation == "uuid": rows["capture_uuid"][1, 0] = 0
        if mutation == "nan": rows["x"][1] = np.nan
        cloud.data = rows.tobytes()
    with pytest.raises(ValueError): decode(cloud)


class FakeMapper:
    def __init__(self): self.stamp = -1.; self.count = 0; self.closed = False
    def integrate(self, points, stamp, sensor, yaw):
        assert stamp >= self.stamp, "projection outran delayed acquisition"
        self.stamp = stamp
        self.count += len(points)
    def project(self, stamp):
        assert stamp >= self.stamp
        self.stamp = stamp
        return np.full((2, 2, 6), self.count, dtype="f4")
    def close(self): self.closed = True


def revision(index, *, stamp=None, kind=1, epoch=0):
    pose = np.eye(4)
    pose[0, 3] = .01 * index
    return dict(kind=kind, session="test", epoch=epoch, revision=index+1, parent=index, snapshot=False,
                frames=[dict(frame_id=index, stamp_ns=index*100_000_000 if stamp is None else stamp,
                             map_from_imu=pose.tolist(), imu_from_lidar=np.eye(4).tolist())])


def chunk(index, *, stamp=None):
    return dict(capture=CAPTURE, sequence=index, stamp_ns=index*100_000_000 if stamp is None else stamp,
                points=np.array([[1., 0., 0.], [1., .1, 0.], [1., 0., .1]]))


def stream(core, n=6):
    runtime = ReceiptScheduler(core)
    results = []
    runtime.clock(0)
    for i in range(n):
        runtime.receive("revision", revision(i), 300)
        runtime.receive("chunk", chunk(i), 120)
        result = runtime.clock((i+1)*100_000_000)
        if result is not None: results.append((core.status(), result.copy()))
    runtime.receive("complete", dict(capture=CAPTURE, sequence=n-1, stamp_ns=(n-1)*100_000_000), 100)
    result = runtime.clock((n+1)*100_000_000)
    if result is not None: results.append((core.status(), result.copy()))
    return results


def test_delayed_watermark_is_deterministic_and_integrates_complete_prefix():
    a, b = ShadowCore(FakeMapper), ShadowCore(FakeMapper)
    first, second = stream(a), stream(b)
    assert a.completed and a.integrated == 6 and a.projected == 6
    assert [x[0]["evidence_stamp_ns"] for x in first] == [i*100_000_000 for i in range(6)]
    for (s, x), (t, y) in zip(first, second):
        assert s["layers_sha256"] == t["layers_sha256"]
        assert np.array_equal(x, y)


@pytest.mark.parametrize("mode", ["missing", "late", "duplicate", "out_of_order", "clock_reset", "clock_gap",
                                 "epoch", "global", "submap", "overflow", "receipt_overflow", "extrinsic"])
def test_faults_clear_evidence_and_latch(mode):
    core = ShadowCore(FakeMapper, max_chunks=1 if mode == "overflow" else 128)
    core.revision(revision(0), 0)
    core.chunk(chunk(0), 0)
    mapper = None
    with pytest.raises(ValueError):
        if mode == "missing":
            core.chunk(chunk(1), 100_000_000)
            core.advance(100_000_000); core.advance(200_000_000); core.advance(300_000_000)
        elif mode == "late": core.chunk(chunk(1), 300_000_001)
        elif mode == "duplicate": core.chunk(chunk(0), 0)
        elif mode == "out_of_order": core.chunk(chunk(2), 200_000_000)
        elif mode == "clock_reset": core.advance(100_000_000); core.advance(0)
        elif mode == "clock_gap": core.advance(0); core.advance(200_000_001)
        elif mode == "epoch": core.revision(revision(1, epoch=1), 100_000_000)
        elif mode == "global": core.revision(revision(1, kind=3), 100_000_000)
        elif mode == "submap": core.revision(revision(1, kind=2), 100_000_000)
        elif mode == "overflow": core.chunk(chunk(1), 100_000_000)
        elif mode == "receipt_overflow": ReceiptScheduler(core, max_events=0).receive("chunk", chunk(1), 120)
        elif mode == "extrinsic":
            value = revision(1); value["frames"][0]["imu_from_lidar"][0][3] = .1
            core.revision(value, 100_000_000)
    assert core.failure and core.mapper is None and not core.status()["valid"] and not core.pending
    with pytest.raises(ValueError, match="latched"): core.advance(300_000_000)


def test_future_pose_is_not_backdated_and_normal_revisions_do_not_reposition_first_pose():
    core = ShadowCore(FakeMapper)
    core.revision(revision(0), 0)
    old = np.asarray(core.timeline.frames[0][0][2].map_from_imu).copy()
    changed = revision(1)
    changed["frames"] = copy.deepcopy(revision(0)["frames"])
    changed["frames"][0]["map_from_imu"][0][3] = 100.
    core.revision(changed, 100_000_000)
    assert np.array_equal(core.timeline.frames[0][0][2].map_from_imu, old)
    with pytest.raises(ValueError, match="measurement after arrival"):
        core.revision(revision(2, stamp=300_000_000), 200_000_000)


def test_pose_storage_remains_bounded_on_long_sequence():
    core = ShadowCore(FakeMapper, max_frames=10)
    stream(core, n=320)
    assert core.completed and core.integrated == 320 and len(core.timeline.frames) <= 4
    assert len(core.timeline.events) <= 128


def test_completion_cannot_hide_missing_tail():
    core = ShadowCore(FakeMapper)
    core.revision(revision(0), 0); core.chunk(chunk(0), 0)
    core.complete(dict(capture=CAPTURE, sequence=1, stamp_ns=100_000_000), 100_000_000)
    core.advance(100_000_000); core.advance(200_000_000)
    with pytest.raises(ValueError, match="incomplete native stream"):
        core.advance(300_000_000)


def test_real_mapper_delayed_clock_and_replay_hashes():
    library = os.environ.get("TERRAIN_LIBRARY")
    if not library: pytest.skip("requires frozen terrain library")
    from layered_terrain import TerrainMapper
    make = lambda: TerrainMapper(library, width=40, height=40, origin=(-2., -2.))
    a, b = ShadowCore(make), ShadowCore(make)
    try:
        first, second = stream(a), stream(b)
        assert a.completed and b.completed
        assert [s["layers_sha256"] for s, _ in first] == [s["layers_sha256"] for s, _ in second]
    finally:
        a.close(); b.close()
