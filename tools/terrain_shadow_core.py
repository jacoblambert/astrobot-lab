"""Bounded causal shadow scheduler, independent of ROS and all GT labels.

First-published poses only. A latched fault destroys map evidence; restart the
experiment explicitly, rather than silently resuming a broken sequence. A 200ms
delayed acquisition watermark prevents projection from outrunning delayed input.
"""
from collections import deque
import hashlib
import time

import numpy as np

from raw_return_filter import valid_range_mask
from shadow_terrain import FramePose, PoseTimeline, integer

DEADLINE_NS = 200_000_000
PERIOD_NS = 100_000_000


class ShadowCore:
    def __init__(self, mapper_factory, *, minimum=.5, maximum=100., max_chunks=128,
                 max_bytes=32 * 1024**2, max_frames=2048):
        valid_range_mask(np.empty((0, 3)), minimum, maximum)
        self.factory, self.minimum, self.maximum = mapper_factory, minimum, maximum
        self.max_chunks, self.max_bytes, self.max_frames = max_chunks, max_bytes, max_frames
        self.mapper = self.timeline = None
        self.pending = deque()
        self.pending_bytes = 0
        self.capture = None
        self.last_sequence = self.last_input_ns = self.last_clock_ns = self.watermark_ns = -1
        self.last_integrated_ns = -1
        self.failure = None
        self.received = self.integrated = self.projected = 0
        self.last_compute_ms = 0.
        self.last_associations = []
        self.last_result = None
        self.completion = None
        self.completed = False

    def close(self):
        if self.mapper is not None:
            self.mapper.close()
            self.mapper = None

    def fault(self, reason):
        if self.failure is None:
            self.failure = str(reason)
        self.close()
        self.pending.clear()
        self.pending_bytes = 0
        self.last_result = None

    def _check(self):
        if self.failure is not None:
            raise ValueError("latched shadow failure: " + self.failure)

    def revision(self, record, arrival_ns):
        self._check()
        try:
            integer(arrival_ns, "arrival")
            if record["kind"] != 1:
                raise ValueError("non-odometry revision: reconstruction not qualified")
            if len(record["frames"]) > self.max_frames:
                raise ValueError("revision payload frame budget exceeded")
            if self.timeline is None:
                if record["revision"] != 1 or record["snapshot"]:
                    raise ValueError("missing first-published revision prefix; restart capture")
                self.timeline = PoseTimeline(record["session"], record["epoch"])
            frames = [FramePose(f["frame_id"], f["stamp_ns"], f["map_from_imu"], f["imu_from_lidar"])
                      for f in record["frames"]]
            self.timeline.apply(**{k: record[k] for k in ("session", "epoch", "revision", "parent", "snapshot")},
                                arrival_ns=arrival_ns, frames=frames)
            # Only original publications are mapping inputs. Ordinary later
            # local-odometry revisions remain recorded but cannot rejuvenate or
            # reposition already integrated evidence in this declared branch.
            for key, history in self.timeline.frames.items():
                self.timeline.frames[key] = history[:1]
            if len(self.timeline.frames) > self.max_frames:
                raise ValueError("causal pose window budget exceeded")
            self.timeline.events = dict(list(self.timeline.events.items())[-128:])
        except Exception as exc:
            self.fault(exc)
            raise

    def chunk(self, chunk, arrival_ns):
        self._check()
        try:
            stamp = integer(chunk["stamp_ns"], "acquisition")
            sequence = integer(chunk["sequence"], "sequence")
            integer(arrival_ns, "arrival")
            if self.capture is None:
                self.capture = chunk["capture"]
            if (chunk["capture"] != self.capture or sequence != self.last_sequence + 1
                    or stamp <= self.last_input_ns or stamp <= self.watermark_ns):
                raise ValueError("native capture/sequence/stamp gap or duplicate")
            if not stamp <= arrival_ns <= stamp + DEADLINE_NS:
                raise ValueError("native input outside acquisition deadline")
            if self.completion is not None and sequence > self.completion[1]:
                raise ValueError("native chunk after declared completion")
            points = np.array(chunk["points"], dtype=np.float64, copy=True)
            keep, _ = valid_range_mask(points, self.minimum, self.maximum)
            points = np.ascontiguousarray(points[keep])
            points.flags.writeable = False
            if len(self.pending) >= self.max_chunks or self.pending_bytes + points.nbytes > self.max_bytes:
                raise ValueError("native input queue budget exceeded")
            self.pending.append((stamp, sequence, points))
            self.pending_bytes += points.nbytes
            self.last_sequence, self.last_input_ns = sequence, stamp
            self.received += 1
        except Exception as exc:
            self.fault(exc)
            raise

    def complete(self, payload, arrival_ns):
        self._check()
        try:
            value = (payload["capture"], integer(payload["sequence"], "completion sequence"),
                     integer(payload["stamp_ns"], "completion stamp"))
            if self.completion is not None or self.capture not in (None, value[0]):
                raise ValueError("duplicate/conflicting native completion")
            if not value[2] <= arrival_ns <= value[2] + DEADLINE_NS:
                raise ValueError("native completion outside acquisition deadline")
            if self.last_sequence > value[1]:
                raise ValueError("completion omitted received chunks")
            self.completion = value
        except Exception as exc:
            self.fault(exc)
            raise

    def advance(self, now_ns):
        self._check()
        try:
            integer(now_ns, "clock")
            if now_ns < self.last_clock_ns:
                raise ValueError("simulation clock reset")
            if self.last_clock_ns >= 0 and now_ns - self.last_clock_ns > DEADLINE_NS:
                raise ValueError("simulation clock gap")
            self.last_clock_ns = now_ns
            if self.completed:
                return None
            target = ((now_ns - DEADLINE_NS) // PERIOD_NS) * PERIOD_NS
            if self.capture is None or target < 0:
                return None
            if now_ns - self.last_input_ns > DEADLINE_NS and self.completion is None:
                raise ValueError("native input stream stale")
            if target <= self.watermark_ns:
                return None
            if self.watermark_ns < 0 and (not self.pending or target < self.pending[0][0]):
                return None
            if self.timeline is None:
                raise ValueError("no causal pose timeline by projection deadline")
            start = time.perf_counter()
            if self.mapper is None:
                self.mapper = self.factory()
            self.last_associations = []
            while self.pending and self.pending[0][0] <= target:
                stamp, sequence, points = self.pending[0]
                assoc = self.timeline.associate(stamp, stamp + DEADLINE_NS, mode="causal")
                pose = assoc.map_from_lidar
                world = points @ pose[:3, :3].T + pose[:3, 3]
                self.mapper.integrate(world, stamp / 1e9, pose[:3, 3], np.arctan2(pose[1, 0], pose[0, 0]))
                self.last_associations.append(dict(sequence=sequence, **assoc.record()))
                self.pending.popleft()
                self.pending_bytes -= points.nbytes
                self.integrated += 1
                self.last_integrated_ns = stamp
            layers = self.mapper.project(target / 1e9)
            self.watermark_ns, self.projected = target, self.projected + 1
            self.last_compute_ms = (time.perf_counter() - start) * 1000
            self.last_result = layers
            if self.completion is not None and target >= self.completion[2]:
                if (self.capture, self.last_sequence, self.last_input_ns) != self.completion or self.pending:
                    raise ValueError("incomplete native stream at final watermark")
                self.completed = True
            # Keep a predecessor for every future interpolation, not the entire
            # run history. Stale revisions cannot be used to integrate old data.
            ordered = sorted(self.timeline.frames, key=lambda k: self.timeline.frames[k][0][2].stamp_ns)
            old = [k for k in ordered if self.timeline.frames[k][0][2].stamp_ns < target]
            for key in old[:-1]:
                stamp = self.timeline.frames.pop(key)[0][2].stamp_ns
                self.timeline.stamps.pop(stamp)
            return layers
        except Exception as exc:
            self.fault(exc)
            raise

    def status(self):
        layers = self.last_result
        return dict(schema=1, valid=layers is not None and self.failure is None, failure=self.failure,
                    completed=self.completed,
                    capture=self.capture, session=None if self.timeline is None else self.timeline.session,
                    epoch=None if self.timeline is None else self.timeline.epoch,
                    pose_policy="first_published_only", evidence_stamp_ns=self.watermark_ns,
                    clock_ns=self.last_clock_ns, last_acquisition_ns=self.last_input_ns,
                    last_integrated_ns=self.last_integrated_ns, received=self.received,
                    integrated=self.integrated, projected=self.projected, pending=len(self.pending),
                    pending_bytes=self.pending_bytes, compute_ms=self.last_compute_ms,
                    layers_sha256=None if layers is None else hashlib.sha256(layers.tobytes()).hexdigest(),
                    navigation_authority=False)
