"""Offline native-chunk shadow contracts. No ROS publishers or GT terrain input.

Arrival timestamps must be conservative simulation-time receipt bounds, NOT
measurement stamps or unconverted wall-clock bag timestamps. The first-published
trajectory and the revised trajectory deliberately remain separate.
"""
import bisect
from dataclasses import dataclass
import hashlib
import json

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from revision_evidence import validated_transform


def integer(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError(f"invalid {name}: expected nonnegative integer")
    return int(value)


def immutable_pose(value):
    pose = validated_transform(value).copy()
    pose.flags.writeable = False
    return pose


@dataclass(frozen=True)
class FramePose:
    frame_id: int
    stamp_ns: int
    map_from_imu: np.ndarray
    imu_from_lidar: np.ndarray

    def __post_init__(self):
        integer(self.frame_id, "frame id")
        integer(self.stamp_ns, "measurement stamp")
        for name in ("map_from_imu", "imu_from_lidar"):
            object.__setattr__(self, name, immutable_pose(getattr(self, name)))


@dataclass(frozen=True)
class Association:
    session: str
    epoch: int
    acquisition_ns: int
    available_ns: int
    output_ns: int
    mode: str
    frame_ids: tuple
    revisions: tuple
    bracket_ns: tuple
    map_from_lidar: np.ndarray

    def record(self):
        return dict(session=self.session, epoch=self.epoch, acquisition_ns=self.acquisition_ns,
                    available_ns=self.available_ns, output_ns=self.output_ns, mode=self.mode,
                    frame_ids=list(self.frame_ids), revisions=list(self.revisions),
                    bracket_ns=list(self.bracket_ns), map_from_lidar=self.map_from_lidar.tolist(), valid=True)


class PoseTimeline:
    """Small offline timeline, one immutable session/epoch; reset creates a new one.

    Updates are atomically validated. Querying historical output times after
    ingesting a full bag must produce the same answer as online event delivery.
    This reference retains history; it is not a bounded live revision consumer.
    """
    def __init__(self, session, epoch, max_gap_ns=200_000_000):
        if not isinstance(session, str) or not session:
            raise ValueError("invalid session")
        self.session, self.epoch = session, integer(epoch, "epoch")
        self.max_gap_ns = integer(max_gap_ns, "maximum gap")
        if not self.max_gap_ns:
            raise ValueError("maximum gap must be positive")
        self.revision = 0
        self.last_arrival_ns = -1
        self.frames, self.stamps, self.events = {}, {}, {}
        self.extrinsic = None

    def apply(self, *, session, epoch, revision, parent, arrival_ns, frames, snapshot=False):
        if (session, epoch) != (self.session, self.epoch):
            raise ValueError("session/epoch reset barrier required")
        revision, parent = integer(revision, "revision"), integer(parent, "parent")
        arrival_ns = integer(arrival_ns, "arrival")
        if not revision or parent >= revision:
            raise ValueError("invalid revision sequence")
        frames = list(frames)
        if len({f.frame_id for f in frames}) != len(frames):
            raise ValueError("duplicate frame in revision")
        payload = [parent, bool(snapshot), [(f.frame_id, f.stamp_ns, f.map_from_imu.tolist(),
                                           f.imu_from_lidar.tolist()) for f in sorted(frames, key=lambda f: f.frame_id)]]
        digest = hashlib.sha256(json.dumps(payload, allow_nan=False).encode()).hexdigest()
        if revision in self.events:
            if self.events[revision] != digest:
                raise ValueError("conflicting duplicate revision")
            return False  # Retransmission must not change original availability.
        if (arrival_ns < self.last_arrival_ns or revision <= self.revision or
                (not snapshot and (parent != self.revision or revision != self.revision + 1))):
            raise ValueError("revision gap/order: authoritative snapshot required")
        if snapshot and not set(self.frames).issubset(f.frame_id for f in frames):
            raise ValueError("snapshot omitted known frames")
        stamps = dict(self.stamps)
        extrinsic = self.extrinsic
        for frame in frames:
            if frame.stamp_ns > arrival_ns:
                raise ValueError("measurement after arrival: clock domains disagree")
            if extrinsic is None:
                extrinsic = frame.imu_from_lidar
            if not np.allclose(extrinsic, frame.imu_from_lidar, rtol=0, atol=1e-10):
                raise ValueError("sensor extrinsic changed within session")
            if frame.stamp_ns in stamps and stamps[frame.stamp_ns] != frame.frame_id:
                raise ValueError("ambiguous duplicate measurement timestamp")
            if frame.frame_id in self.frames and self.frames[frame.frame_id][0][2].stamp_ns != frame.stamp_ns:
                raise ValueError("immutable frame timestamp changed")
            stamps[frame.stamp_ns] = frame.frame_id
        # No partial state updates before every frame passes.
        for frame in frames:
            self.frames.setdefault(frame.frame_id, []).append((arrival_ns, revision, frame))
        self.stamps, self.extrinsic = stamps, extrinsic
        self.events[revision] = digest
        self.revision, self.last_arrival_ns = revision, arrival_ns
        return True

    def associate(self, acquisition_ns, output_ns, *, mode="causal"):
        acquisition_ns, output_ns = integer(acquisition_ns, "acquisition"), integer(output_ns, "output")
        if mode not in {"causal", "revised"} or acquisition_ns > output_ns:
            raise ValueError("invalid mode or acquisition after output")
        samples = []
        for history in self.frames.values():
            known = [row for row in history if row[0] <= output_ns]
            if known:
                samples.append(known[0] if mode == "causal" else known[-1])
        samples.sort(key=lambda row: row[2].stamp_ns)
        times = [row[2].stamp_ns for row in samples]
        index = bisect.bisect_left(times, acquisition_ns)
        if index < len(times) and times[index] == acquisition_ns:
            before = after = samples[index]
            pose = before[2].map_from_imu.copy()
        else:
            if index == 0 or index == len(times):
                raise ValueError("missing causal bracket: extrapolation forbidden")
            before, after = samples[index - 1], samples[index]
            gap = after[2].stamp_ns - before[2].stamp_ns
            if gap > self.max_gap_ns:
                raise ValueError("pose interpolation gap exceeds 0.2-second contract")
            fraction = (acquisition_ns - before[2].stamp_ns) / gap
            pose = np.eye(4)
            pose[:3, 3] = (1 - fraction) * before[2].map_from_imu[:3, 3] + fraction * after[2].map_from_imu[:3, 3]
            pose[:3, :3] = Slerp([0., 1.], Rotation.from_matrix(np.array([
                before[2].map_from_imu[:3, :3], after[2].map_from_imu[:3, :3]])))([fraction]).as_matrix()[0]
        # Interpolate the IMU body pose FIRST. Linear interpolation of LiDAR
        # translations incorrectly cuts across its rotating off-center lever arm.
        lidar = immutable_pose(pose @ before[2].imu_from_lidar)
        return Association(self.session, self.epoch, acquisition_ns, max(before[0], after[0]),
                           output_ns, mode, (before[2].frame_id, after[2].frame_id),
                           (before[1], after[1]), (before[2].stamp_ns, after[2].stamp_ns), lidar)


STRESS_PROFILES = {"baseline": (0., 0.), "range_1cm": (.01, 0.),
                   "range_3cm": (.03, 0.), "dropout_10pct": (0., .1)}


def clock_bound_events(events, max_pending=10000, max_clock_gap_ns=200_000_000):
    """Yield revision payloads with a conservative arrival bound from /clock.

    Input: (bag_receipt_ns, 'clock'|'revision', simulation_ns|payload), in receipt
    order. The NEXT clock publication bounds availability; the previous clock can
    falsely backdate an estimate. Missing tail clock/reset/order is a hard error.
    This is an engineering receipt bound, not precise callback latency telemetry.
    """
    previous_receipt, previous_clock, pending = -1, None, []
    for receipt, kind, payload in events:
        integer(receipt, "bag receipt")
        if receipt < previous_receipt:
            raise ValueError("bag receipt order regressed")
        previous_receipt = receipt
        if kind == "clock":
            stamp = integer(payload, "simulation clock")
            if previous_clock is not None:
                if stamp < previous_clock:
                    raise ValueError("clock reset: split at epoch barrier")
                if stamp - previous_clock > max_clock_gap_ns:
                    raise ValueError("clock receipt gap exceeds association contract")
            for event_receipt, event in pending:
                yield event, stamp, event_receipt
            pending.clear()
            previous_clock = stamp
        elif kind == "revision":
            if previous_clock is None:
                raise ValueError("revision before first clock: missing recording prefix")
            pending.append((receipt, payload))
            if len(pending) > max_pending:
                raise ValueError("pending arrival-bound budget exceeded")
        else:
            raise ValueError("unsupported receipt event")
    if pending:
        raise ValueError("missing tail clock: revision availability unbounded")


def perturb_native(points, *, profile, seed, chunk_id):
    """Stable per-chunk synthetic stress; call upstream of BOTH consumers.

    Returns retained finite, positive-range XYZ and original indices. No clipping
    to a ground model or sensor range; the normal range filter remains downstream.
    Does not mutate input or use GT. This is not a calibrated hardware noise model.
    """
    if profile not in STRESS_PROFILES:
        raise ValueError("undeclared stress profile")
    integer(seed, "stress seed")
    integer(chunk_id, "chunk identity")
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("expected Nx3 native points")
    ranges = np.linalg.norm(points, axis=1)
    indices = np.flatnonzero(np.isfinite(points).all(axis=1) & np.isfinite(ranges) & (ranges > 0))
    xyz = points[indices].copy()
    sigma, dropout = STRESS_PROFILES[profile]
    rng = np.random.default_rng(np.random.SeedSequence([seed, chunk_id]))
    if sigma:
        radii = ranges[indices] + rng.normal(0., sigma, len(indices))
        xyz *= (radii / ranges[indices])[:, None]
        keep = radii > 0
        xyz, indices = xyz[keep], indices[keep]
    if dropout:
        keep = rng.random(len(indices)) >= dropout
        xyz, indices = xyz[keep], indices[keep]
    return xyz, indices


def pose_error(estimated, reference):
    """Fixed-anchor full-SE(3) error, no fitted alignment."""
    estimated, reference = validated_transform(estimated), validated_transform(reference)
    return dict(position_m=float(np.linalg.norm(estimated[:3, 3] - reference[:3, 3])),
                rotation_rad=float(Rotation.from_matrix(reference[:3, :3].T @ estimated[:3, :3]).magnitude()))


def reconstruct_native(chunks, associations, *, library, output_ns, projection_stamps_ns=(), **geometry):
    """Small-fixture full-prefix reconstruction oracle, NOT a live mapper.

    Caller supplies every original acquisition and original projection tick up
    to the watermark, including expired evidence. Cropping to now-30s can change
    immutable bucket boundaries and falsely retain returns from expired buckets.
    Stream chunks from disk for large fixtures. No raw point storage in this helper.
    """
    from layered_terrain import TerrainMapper

    integer(output_ns, "output")
    ticks = [integer(t, "projection stamp") for t in projection_stamps_ns]
    if ticks != sorted(set(ticks)) or any(t > output_ns for t in ticks):
        raise ValueError("invalid projection schedule")
    cursor, previous, count = 0, -1, 0
    identity = None
    with TerrainMapper(library, **geometry) as mapper:
        for chunk_id, stamp_ns, points in chunks:
            integer(chunk_id, "chunk id")
            integer(stamp_ns, "measurement")
            if not previous < stamp_ns <= output_ns:
                raise ValueError("nonmonotonic/future native chunk")
            assoc = associations[chunk_id]
            if assoc.acquisition_ns != stamp_ns or assoc.available_ns > output_ns:
                raise ValueError("missing or future pose association")
            current = (assoc.session, assoc.epoch, assoc.mode)
            if identity is not None and identity != current:
                raise ValueError("mixed session/epoch/mode reconstruction")
            identity = current
            points = np.asarray(points, dtype=float)
            if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
                raise ValueError("native points must be finite Nx3 after range filtering")
            while cursor < len(ticks) and ticks[cursor] < stamp_ns:
                mapper.project(ticks[cursor] / 1e9)
                cursor += 1
            pose = assoc.map_from_lidar
            world = points @ pose[:3, :3].T + pose[:3, 3]
            mapper.integrate(world, stamp_ns / 1e9, pose[:3, 3], np.arctan2(pose[1, 0], pose[0, 0]))
            previous, count = stamp_ns, count + 1
        if not count:
            raise ValueError("empty native reconstruction")
        for tick in ticks[cursor:]:
            mapper.project(tick / 1e9)
        return mapper.project(output_ns / 1e9)
