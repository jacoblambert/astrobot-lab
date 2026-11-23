"""Recorder-only revision validation, without an offline pose-history heap.

The immutable MCAP remains authoritative. This gate cannot answer pose queries.
It retains every frame's timestamp and every revision's digest, but no past pose
versions. Batch SE(3) checks use the reference validator's unchanged tolerances.
"""

import hashlib
import json

import numpy as np

from shadow_terrain import integer


def rigid_batch(values):
    values = np.asarray(values, dtype=np.float64)
    if (
        values.ndim != 3
        or values.shape[1:] != (4, 4)
        or not np.isfinite(values).all()
        or not np.allclose(values[:, 3], [0, 0, 0, 1], atol=1e-10, rtol=0)
        or not np.allclose(np.swapaxes(values[:, :3, :3], 1, 2) @ values[:, :3, :3], np.eye(3), atol=1e-8, rtol=0)
        or np.any(np.abs(np.linalg.det(values[:, :3, :3]) - 1) > 1e-8)
    ):
        raise ValueError("pose is not finite rigid SE(3)")
    return values


class LiveRevisionGate:
    def __init__(self, session, epoch):
        if not isinstance(session, str) or not session:
            raise ValueError("invalid session")
        self.session, self.epoch = session, integer(epoch, "epoch")
        self.revision, self.last_arrival_ns = 0, -1
        self.frame_stamps, self.stamps, self.events = {}, {}, {}
        self.extrinsic = None

    def apply_record(self, record):
        if (record["session"], record["epoch"]) != (self.session, self.epoch):
            raise ValueError("session/epoch reset barrier required")
        revision = integer(record["revision"], "revision")
        parent = integer(record["parent"], "parent")
        arrival = integer(record["arrival_ns"], "arrival")
        if not revision or parent >= revision:
            raise ValueError("invalid revision sequence")
        frames = record["frames"]
        ids = [integer(f["frame_id"], "frame id") for f in frames]
        times = [integer(f["stamp_ns"], "measurement stamp") for f in frames]
        if len(set(ids)) != len(ids):
            raise ValueError("duplicate frame in revision")
        if frames:
            poses = rigid_batch([f["map_from_imu"] for f in frames])
            mounts = rigid_batch([f["imu_from_lidar"] for f in frames])
        else:
            poses = mounts = np.empty((0, 4, 4))
        # Exactly the offline timeline's canonical digest, including float
        # normalization. Conflicting duplicates remain failures indefinitely.
        normalized = sorted(zip(ids, times, poses.tolist(), mounts.tolist()))
        payload = [parent, bool(record["snapshot"]), normalized]
        digest = hashlib.sha256(json.dumps(payload, allow_nan=False).encode()).hexdigest()
        if revision in self.events:
            if self.events[revision] != digest:
                raise ValueError("conflicting duplicate revision")
            return False
        if (
            arrival < self.last_arrival_ns
            or revision <= self.revision
            or (not record["snapshot"] and (parent != self.revision or revision != self.revision + 1))
        ):
            raise ValueError("revision gap/order: authoritative snapshot required")
        if record["snapshot"] and not set(self.frame_stamps).issubset(ids):
            raise ValueError("snapshot omitted known frames")
        extrinsic = mounts[0] if self.extrinsic is None and frames else self.extrinsic
        if frames and not np.allclose(mounts, extrinsic, rtol=0, atol=1e-10):
            raise ValueError("sensor extrinsic changed within session")
        stamps = dict(self.stamps)
        for frame_id, stamp in zip(ids, times):
            if stamp > arrival:
                raise ValueError("measurement after arrival: clock domains disagree")
            if stamp in stamps and stamps[stamp] != frame_id:
                raise ValueError("ambiguous duplicate measurement timestamp")
            if frame_id in self.frame_stamps and self.frame_stamps[frame_id] != stamp:
                raise ValueError("immutable frame timestamp changed")
            stamps[stamp] = frame_id
        # All checks precede mutation, just as in the offline reference.
        self.frame_stamps.update(zip(ids, times))
        self.stamps = stamps
        if extrinsic is not None:
            self.extrinsic = extrinsic.copy()
        self.events[revision] = digest
        self.revision, self.last_arrival_ns = revision, arrival
        return True
