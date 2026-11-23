"""Bounded canonical ODOM pose ledger for proving non-ODOM batches are no-ops.

Keep references after the reconstruction interpolation window expires: retained
terrain can outlive raw observations. Bookkeeping never updates these references,
so tolerance cannot accumulate through repeated tiny corrections. This supports
no-change deltas only, not snapshots, epoch changes or real global corrections.
"""

import hashlib
import json

import numpy as np

from shadow_terrain import integer

ROUNDING_ATOL = 1e-10


class BookkeepingLedger:
    def __init__(self, capacity):
        self.capacity = capacity
        self.frames = {}
        self.stamps = {}
        self.events = {}
        self.accepted = 0

    def validate_odom(self, frames):
        if len(self.frames) + len({f.frame_id for f in frames} - self.frames.keys()) > self.capacity:
            raise ValueError("canonical pose ledger budget exceeded")
        for frame in frames:
            old = self.frames.get(frame.frame_id)
            if old is not None and old.stamp_ns != frame.stamp_ns:
                raise ValueError("immutable historical frame timestamp changed")
            if frame.stamp_ns in self.stamps and self.stamps[frame.stamp_ns] != frame.frame_id:
                raise ValueError("ambiguous historical measurement timestamp")

    def commit_odom(self, frames):
        for frame in frames:
            self.frames[frame.frame_id] = frame
            self.stamps[frame.stamp_ns] = frame.frame_id

    def accept(self, record, arrival_ns, timeline):
        if timeline is None:
            raise ValueError("bookkeeping before ODOM prefix")
        if record["snapshot"] or record["kind"] not in (2, 3):
            raise ValueError("unsupported bookkeeping kind/snapshot")
        if (record["session"], record["epoch"]) != (timeline.session, timeline.epoch):
            raise ValueError("session/epoch reset barrier required")
        revision = integer(record["revision"], "revision")
        parent = integer(record["parent"], "parent")
        if not record["frames"] or len(record["frames"]) > self.capacity:
            raise ValueError("bookkeeping batch budget/empty payload")
        # Digest includes kind and deskew metadata, unlike the interpolation
        # timeline's deliberately empty no-op advancement below.
        digest = hashlib.sha256(json.dumps(record, sort_keys=True, allow_nan=False).encode()).hexdigest()
        if revision in self.events:
            if digest != self.events[revision]:
                raise ValueError("conflicting duplicate bookkeeping revision")
            return False
        if parent != timeline.revision or revision != parent + 1 or arrival_ns < timeline.last_arrival_ns:
            raise ValueError("bookkeeping revision gap/order")
        ids, old_frames = set(), []
        for row in record["frames"]:
            frame_id = integer(row["frame_id"], "frame id")
            stamp = integer(row["stamp_ns"], "measurement stamp")
            if frame_id in ids:
                raise ValueError("duplicate frame in bookkeeping revision")
            ids.add(frame_id)
            old = self.frames.get(frame_id)
            if old is None:
                raise ValueError("bookkeeping frame lacks canonical ODOM reference")
            if stamp != old.stamp_ns or stamp > arrival_ns:
                raise ValueError("bookkeeping measurement stamp changed/future")
            if row.get("deskew_recomputed") is not False:
                raise ValueError("bookkeeping invalidates deskew")
            old_frames.append(old)
        for field in ("imu_from_lidar", "map_from_imu"):
            values = np.asarray([row[field] for row in record["frames"]], dtype=np.float64)
            canonical = np.asarray([getattr(old, field) for old in old_frames])
            # Canonical transforms were validated on ODOM admission. Equality
            # at rounding precision proves rigidity too; batch comparison avoids
            # thousands of per-frame NumPy validation calls in the live callback.
            if values.shape != canonical.shape or not np.isfinite(values).all():
                raise ValueError("invalid bookkeeping transform shape/nonfinite")
            if not np.allclose(values, canonical, rtol=0, atol=ROUNDING_ATOL):
                raise ValueError(f"bookkeeping changes canonical {field}; rebuild/reset required")
        # Validate the ENTIRE original batch before committing. Advance the
        # revision chain without importing old frames into the recent window or
        # rewriting pose availability, frozen geometry, ages or numeric poses.
        timeline.apply(
            session=record["session"],
            epoch=record["epoch"],
            revision=revision,
            parent=parent,
            arrival_ns=arrival_ns,
            frames=[],
        )
        timeline.events = dict(list(timeline.events.items())[-128:])
        self.events[revision] = digest
        self.events = dict(list(self.events.items())[-128:])
        self.accepted += 1
        return True
