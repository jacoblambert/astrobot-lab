"""Bounded, publication-causal native reconstruction with coherent ODOM updates.

This is a distinct experimental policy, NOT the frozen first-publication core.
Each output reconstructs the window using the committed pose state available at
that output. A shared IMU anchor absorbs gauge corrections into local_from_map;
relative corrections remain in observation placement. No estimator uncertainty
certificate or motion authority is implied by valid geometry.
"""

from collections import deque
from dataclasses import dataclass
import hashlib
import time

import numpy as np

from native_chunk_transport import MAX_POINTS
from raw_return_filter import valid_range_mask
from revision_evidence import validated_transform
from shadow_terrain import FramePose, PoseTimeline, integer
from local_terrain.pose_lookup import PublicationPoseLookup
from local_terrain.return_cache import ReturnTransformCache
from local_terrain.bookkeeping import BookkeepingLedger

DELAY_NS = 200_000_000
PERIOD_NS = 100_000_000


@dataclass(frozen=True)
class LocalConfig:
    window_s: int = 5
    resolution: float = 0.1
    cells: int = 60
    max_bytes: int = 256 * 1024**2
    max_chunks: int = 2048
    max_frames: int = 2048
    minimum: float = 0.5
    maximum: float = 100.0
    profile: bool = False
    observation_policy: str = "coherent"
    backend: str = "reference"
    map_policy: str = "rolling_rebuild"
    evidence_lifetime_s: int | None = None
    allow_bookkeeping: bool = False
    max_bookkeeping_frames: int = 8192
    # Live navigation stream: a dropped or late chunk is skipped and counted, and stale input
    # pauses projection (the supervisor's map freshness gate owns outages). Captures stay strict.
    tolerate_stream_gaps: bool = False

    def __post_init__(self):
        if type(self.window_s) is not int or self.window_s not in (2, 5, 30):
            raise ValueError("declared windows are 2, 5 and 30 seconds")
        if self.resolution != 0.1 or type(self.cells) is not int or self.cells not in (60, 200):
            raise ValueError("declared diagnostic grids are 60x60 or 200x200 / 0.1 m")
        if self.observation_policy not in ("coherent", "frozen_observation"):
            raise ValueError("unknown observation placement policy")
        if self.backend not in ("reference", "prepared"):
            raise ValueError("unknown reconstruction backend")
        if self.map_policy not in ("rolling_rebuild", "fixed_incremental", "rolling_incremental"):
            raise ValueError("unknown map accumulation policy")
        if self.evidence_lifetime_s is not None and (
            type(self.evidence_lifetime_s) is not int
            or self.evidence_lifetime_s not in (30, 60)
            or self.map_policy != "rolling_incremental"
        ):
            raise ValueError("separate 30/60 s evidence lifetime is a rolling incremental experiment only")
        if self.map_policy == "fixed_incremental" and (
            self.window_s != 30 or self.cells != 200 or self.observation_policy != "frozen_observation"
        ):
            raise ValueError("incremental experiment requires fixed 20 m / 30 s / frozen observations")
        if self.map_policy == "rolling_incremental" and (
            self.window_s not in (5, 30)
            or self.cells != 60
            or self.observation_policy != "frozen_observation"
            or self.window_s == 5
            and self.evidence_lifetime_s is None
        ):
            raise ValueError(
                "rolling incremental requires 6 m, frozen observations and explicit evidence lifetime for 5 s raw"
            )
        if type(self.allow_bookkeeping) is not bool:
            raise ValueError("invalid bookkeeping policy")
        for value in (self.max_bytes, self.max_chunks, self.max_frames, self.max_bookkeeping_frames):
            if type(value) is not int or value <= 0:
                raise ValueError("invalid resource bound")
        valid_range_mask(np.empty((0, 3)), self.minimum, self.maximum)


class LocalReconstructor:
    def __init__(self, factory, base_from_imu, config=None):
        self.config = config or LocalConfig()
        self.factory = factory  # factory(origin, lifetime) -> terrain mapper
        self.base_from_imu = validated_transform(base_from_imu).copy()
        self.timeline = None
        self.observations = deque()
        self.frozen_placements = {}
        self.return_cache = (
            ReturnTransformCache(self.config.minimum, self.config.maximum, max_entries=self.config.max_chunks)
            if self.config.backend == "prepared"
            else None
        )
        self.bytes = 0
        self.capture = None
        self.sequence = self.last_input_ns = self.last_clock_ns = self.watermark_ns = -1
        self.last_arrival_ns = -1
        self.local_from_map = None
        self.anchor = None
        self.failure = None
        self.closed = False
        self.last_result = None
        self.last_record = None
        self.received = self.projected = 0
        self.skipped_sequences = self.late_chunks = 0
        self.completion = None
        self.completed = False
        self.persistent_mapper = None
        self.fixed_origin = None
        self.integrated_sequence = -1
        self.reference_witness = None
        self.bookkeeping = (
            BookkeepingLedger(self.config.max_bookkeeping_frames) if self.config.allow_bookkeeping else None
        )

    def _release_persistent(self):
        if self.persistent_mapper is not None:
            self.persistent_mapper.close()
            self.persistent_mapper = None

    def _check(self):
        if self.failure or self.closed:
            raise ValueError(self.failure or "local reconstruction closed")

    def fault(self, reason):
        # KeyboardInterrupt and other exceptions may stringify to "". An
        # empty failure must never look like a successfully completed run.
        if self.failure is None:
            self.failure = str(reason) or repr(reason) or type(reason).__name__
        self.last_result = self.last_record = None
        self.observations.clear()
        self.frozen_placements.clear()
        if self.return_cache is not None:
            self.return_cache.clear()
        self.bytes = 0
        self._release_persistent()

    def close(self):
        # Even a successfully sealed experiment must not leave an authoritative
        # live-looking output behind. Saved diagnostic records remain separate.
        self.closed = True
        self.last_result = self.last_record = None
        self.observations.clear()
        self.frozen_placements.clear()
        if self.return_cache is not None:
            self.return_cache.clear()
        self.bytes = 0
        self._release_persistent()

    def _arrival(self, stamp):
        stamp = integer(stamp, "receipt bound")
        if stamp < self.last_arrival_ns:
            raise ValueError("receipt order regressed")
        self.last_arrival_ns = stamp

    def revision(self, record, arrival_ns):
        self._check()
        try:
            self._arrival(arrival_ns)
            if self.bookkeeping is not None and record["kind"] in (2, 3):
                self.bookkeeping.accept(record, arrival_ns, self.timeline)
                return
            if record["kind"] != 1 or record["snapshot"]:
                raise ValueError("only contiguous ODOM deltas are supported")
            if len(record["frames"]) > self.config.max_frames:
                raise ValueError("revision frame budget exceeded")
            if self.timeline is None:
                if record["revision"] != 1:
                    raise ValueError("missing revision prefix")
                self.timeline = PoseTimeline(record["session"], record["epoch"])
            frames = [
                FramePose(f["frame_id"], f["stamp_ns"], f["map_from_imu"], f["imu_from_lidar"])
                for f in record["frames"]
            ]
            if self.bookkeeping is not None:
                self.bookkeeping.validate_odom(frames)
                if record["revision"] in self.bookkeeping.events:
                    raise ValueError("ODOM conflicts with bookkeeping revision")
            old_anchor = None
            if self.anchor is not None:
                if self.anchor not in self.timeline.frames:
                    raise ValueError("shared anchor lost")
                old_anchor = self.timeline.frames[self.anchor][-1][2].map_from_imu
            changed = self.timeline.apply(
                **{key: record[key] for key in ("session", "epoch", "revision", "parent", "snapshot")},
                arrival_ns=arrival_ns,
                frames=frames,
            )
            if changed:
                if self.bookkeeping is not None:
                    self.bookkeeping.commit_odom(frames)
                # Apply the complete batch first. Partial validation cannot
                # publish a half-updated transform or reconstructed map.
                if old_anchor is not None:
                    new_anchor = self.timeline.frames[self.anchor][-1][2].map_from_imu
                    if self.config.map_policy == "rolling_rebuild":
                        self.local_from_map = self.local_from_map @ old_anchor @ np.linalg.inv(new_anchor)
                    elif not np.allclose(new_anchor, self.reference_witness, atol=1e-6, rtol=0):
                        # Frozen evidence cannot follow a continually re-anchored
                        # local reference without transporting/rebinning ALL old
                        # geometry. Use a fixed map reference, and fail closed on
                        # witness changes. Compare to the INITIAL witness, never
                        # ratchet a tolerance through successive small updates.
                        # 1e-6 matrix-entry tolerance only admits numeric jitter;
                        # it is not a localization uncertainty or correction budget.
                        raise ValueError("fixed-map reference witness changed; rebuild/reset required")
                for key, history in self.timeline.frames.items():
                    self.timeline.frames[key] = history[-1:]
                self.timeline.events = dict(list(self.timeline.events.items())[-128:])
                self._prune_frames()
                if len(self.timeline.frames) > self.config.max_frames:
                    raise ValueError("pose frame budget exceeded")
        except BaseException as exc:
            self.fault(exc)
            raise

    def _prune_frames(self):
        if self.timeline is None or self.watermark_ns < 0:
            return
        cutoff = self.watermark_ns - self.config.window_s * 10**9
        old = sorted((h[-1][2].stamp_ns, key) for key, h in self.timeline.frames.items() if h[-1][2].stamp_ns < cutoff)
        # One predecessor is necessary for interpolation at the window edge.
        for stamp, key in old[:-1]:
            if key != self.anchor:
                self.timeline.frames.pop(key)
                self.timeline.stamps.pop(stamp)

    def resume_after(self, capture, sequence):
        """Join a running native stream (nav map reset): the next chunk is sequence + 1."""
        self._check()
        if self.capture is not None or self.received:
            raise ValueError("stream already joined")
        if not isinstance(capture, str) or not capture:
            raise ValueError("invalid capture identity")
        self.capture, self.sequence = capture, integer(sequence, "sequence")

    def chunk(self, chunk, arrival_ns):
        self._check()
        try:
            self._arrival(arrival_ns)
            stamp = integer(chunk["stamp_ns"], "acquisition")
            sequence = integer(chunk["sequence"], "sequence")
            capture = chunk["capture"]
            if not isinstance(capture, str) or not capture:
                raise ValueError("invalid capture identity")
            if self.capture is None:
                self.capture = capture
            tolerant = self.config.tolerate_stream_gaps
            if (
                capture != self.capture
                or sequence <= self.sequence
                or not tolerant and sequence != self.sequence + 1
                or stamp <= self.last_input_ns
                or self.completion is not None
            ):
                raise ValueError("native identity/sequence/stamp discontinuity")
            late = stamp <= self.watermark_ns or arrival_ns > stamp + DELAY_NS
            if tolerant and late:
                self.skipped_sequences += sequence - self.sequence - 1
                self.late_chunks += 1
                self.sequence = sequence
                return
            if stamp <= self.watermark_ns:
                raise ValueError("native identity/sequence/stamp discontinuity")
            if not stamp <= arrival_ns <= stamp + DELAY_NS:
                raise ValueError("native acquisition deadline exceeded")
            raw = np.asarray(chunk["points"])
            if raw.ndim != 2 or raw.shape[1] != 3 or not 0 < len(raw) <= MAX_POINTS or not np.isfinite(raw).all():
                raise ValueError("invalid native return array")
            # Preserve native float32 XYZ; no quantization beyond wire format.
            points = np.array(raw, dtype=np.float32, copy=True)
            if not np.isfinite(points).all():
                raise ValueError("native float32 overflow")
            if len(self.observations) >= self.config.max_chunks or self.bytes + points.nbytes > self.config.max_bytes:
                raise ValueError("retained native budget exceeded")
            points.flags.writeable = False
            self.observations.append((stamp, sequence, points))
            self.bytes += points.nbytes
            self.skipped_sequences += sequence - self.sequence - 1
            self.sequence, self.last_input_ns = sequence, stamp
            self.received += 1
        except BaseException as exc:
            self.fault(exc)
            raise

    def complete(self, payload, arrival_ns):
        self._check()
        try:
            self._arrival(arrival_ns)
            value = (
                payload["capture"],
                integer(payload["sequence"], "last sequence"),
                integer(payload["stamp_ns"], "last acquisition"),
            )
            if self.completion is not None or value != (self.capture, self.sequence, self.last_input_ns):
                raise ValueError("incomplete or duplicate native completion")
            if not value[2] <= arrival_ns <= value[2] + DELAY_NS:
                raise ValueError("completion deadline exceeded")
            self.completion = value
        except BaseException as exc:
            self.fault(exc)
            raise

    def advance(self, now_ns):
        self._check()
        mapper = None
        try:
            now_ns = integer(now_ns, "publication bound")
            if (
                now_ns < max(self.last_clock_ns, self.last_arrival_ns)
                or self.last_clock_ns >= 0
                and now_ns - self.last_clock_ns > DELAY_NS
            ):
                raise ValueError("publication clock reset/gap or future receipt")
            self.last_clock_ns = now_ns
            if self.completed or self.capture is None:
                return None
            if now_ns - self.last_input_ns > DELAY_NS and self.completion is None:
                if self.config.tolerate_stream_gaps:
                    return None
                raise ValueError("native input stale")
            target = (now_ns - DELAY_NS) // PERIOD_NS * PERIOD_NS
            if target <= self.watermark_ns or target < 0 or target < self.observations[0][0]:
                return None
            if self.timeline is None:
                raise ValueError("no pose state by deadline")
            start = time.perf_counter()
            cutoff = target - self.config.window_s * 10**9
            while self.observations and self.observations[0][0] < cutoff:
                _, expired_sequence, expired_points = self.observations.popleft()
                self.bytes -= expired_points.nbytes
                self.frozen_placements.pop(expired_sequence, None)
            # State is latest-at-publication, not final-revised offline state.
            if self.config.backend == "prepared":
                lookup = PublicationPoseLookup(self.timeline, now_ns)
                associate = lookup.associate
                self.return_cache.prune(sequence for _, sequence, _ in self.observations)
            else:
                associate = lambda stamp: self.timeline.associate(stamp, now_ns, mode="revised")
            current = associate(target)
            map_from_imu = current.map_from_lidar @ np.linalg.inv(self.timeline.extrinsic)
            map_from_base = map_from_imu @ np.linalg.inv(self.base_from_imu)
            if self.local_from_map is None:
                self.local_from_map = np.eye(4)
                self.local_from_map[:3, 3] = -map_from_base[:3, 3]
            local_from_base = self.local_from_map @ map_from_base
            r = self.config.resolution
            origin = np.floor(local_from_base[:2, 3] / r) * r - self.config.cells * r / 2
            incremental = self.config.map_policy != "rolling_rebuild"
            if incremental:
                if self.persistent_mapper is None:
                    self.fixed_origin = origin.copy()
                    self.persistent_mapper = self.factory(
                        self.fixed_origin, self.config.evidence_lifetime_s or self.config.window_s
                    )
                if self.config.map_policy == "fixed_incremental":
                    origin = self.fixed_origin
                else:
                    self.persistent_mapper.shift_origin(origin)
                mapper = self.persistent_mapper
            else:
                mapper = self.factory(origin, self.config.window_s)
            associations = []
            integrated_inputs = 0
            stages = dict(association=0.0, transform=0.0, integrate=0.0, project=0.0)
            for stamp, sequence, points in self.observations:
                if stamp > target:
                    break
                stage_start = time.perf_counter() if self.config.profile else 0.0
                frozen = self.frozen_placements.get(sequence)
                if frozen is None:
                    association = associate(stamp)
                    pose = self.local_from_map @ association.map_from_lidar
                    association_record = dict(sequence=sequence, **association.record())
                    if self.config.observation_policy == "frozen_observation":
                        # Freeze the observation's first USED local placement, not
                        # historical map-frame coordinates across gauge changes.
                        # Current-rover pose/revision handling is identical to the
                        # coherent arm. This is NOT the original live baseline.
                        self.frozen_placements[sequence] = (pose.copy(), association_record.copy())
                else:
                    pose, association_record = frozen
                associations.append(dict(**association_record, local_from_lidar=pose.tolist()))
                if incremental and sequence <= self.integrated_sequence:
                    continue
                if self.config.profile:
                    stages["association"] += time.perf_counter() - stage_start
                    stage_start = time.perf_counter()
                if self.return_cache is None:
                    keep, _ = valid_range_mask(points, self.config.minimum, self.config.maximum)
                    world = points[keep].astype(np.float64) @ pose[:3, :3].T + pose[:3, 3]
                else:
                    world = self.return_cache.transform(sequence, stamp, points, pose)
                if self.config.profile:
                    stages["transform"] += time.perf_counter() - stage_start
                    stage_start = time.perf_counter()
                mapper.integrate(world, stamp / 1e9, pose[:3, 3], np.arctan2(pose[1, 0], pose[0, 0]))
                integrated_inputs += 1
                if incremental:
                    self.integrated_sequence = sequence
                if self.config.profile:
                    stages["integrate"] += time.perf_counter() - stage_start
            stage_start = time.perf_counter() if self.config.profile else 0.0
            layers = mapper.project(target / 1e9)
            if self.config.profile:
                stages["project"] = time.perf_counter() - stage_start
            if layers.shape != (self.config.cells, self.config.cells, 6):
                raise ValueError("unexpected layered output shape")
            if not np.isin(layers[:, :, 0], [-1, 0, 20, 100]).all():
                raise ValueError("invalid terrain costs")
            if not incremental:
                mapper.close()
            mapper = None
            record = dict(
                schema=1,
                policy=(
                    "local_" + self.config.map_policy + "_frozen_v1"
                    if incremental
                    else (
                        "local_latest_available_odom_v1"
                        if self.config.observation_policy == "coherent"
                        else "local_frozen_observation_v1"
                    )
                ),
                map_policy=self.config.map_policy,
                integrated_inputs=integrated_inputs,
                support_freshness_certified=False,
                relative_pose_uncertainty_evaluated=False,
                reference_policy="fixed_map_witness" if incremental else "shared_recent_anchor",
                reference_witness_entry_tolerance=1e-6 if incremental else None,
                observation_policy=self.config.observation_policy,
                backend=self.config.backend,
                current_rover_policy="latest_available_odom",
                cells=self.config.cells,
                acceptance=False,
                navigation_authority=False,
                evidence_ns=target,
                publication_ns=now_ns,
                age_s=(now_ns - target) / 1e9,
                origin=origin.tolist(),
                resolution=r,
                local_from_map=self.local_from_map.tolist(),
                local_from_base=local_from_base.tolist(),
                session=self.timeline.session,
                epoch=self.timeline.epoch,
                revision=self.timeline.revision,
                window_s=self.config.window_s,
                evidence_lifetime_s=self.config.evidence_lifetime_s or self.config.window_s,
                historical_pose_repair_available=not incremental,
                retained_bytes=self.bytes,
                observations=len(associations),
                received=self.received,
                associations=associations,
                layers_sha256=hashlib.sha256(layers.tobytes()).hexdigest(),
                compute_ms=(time.perf_counter() - start) * 1000,
            )
            if self.config.profile:
                record["stage_ms"] = {key: value * 1000 for key, value in stages.items()}
                record["instrumented_compute_not_acceptance"] = True
            if self.return_cache is not None:
                record["exact_cache"] = dict(
                    bytes=self.return_cache.bytes,
                    entries=len(self.return_cache.entries),
                    hits=self.return_cache.hits,
                    transforms=self.return_cache.transforms,
                    filters=self.return_cache.filters,
                )
            # Commit only a completed projection. No partial update is visible.
            if incremental:
                if self.anchor is None:
                    # Oldest session frame, NOT the continually revised recent
                    # rover pose. The map-to-local transform remains constant.
                    self.anchor = min(self.timeline.frames, key=lambda k: self.timeline.frames[k][-1][2].stamp_ns)
                    self.reference_witness = self.timeline.frames[self.anchor][-1][2].map_from_imu.copy()
            else:
                self.anchor = current.frame_ids[0]
            self.watermark_ns = target
            self.last_result, self.last_record = layers, record
            self.projected += 1
            self.completed = self.completion is not None and target >= self.completion[2]
            self._prune_frames()
            return layers
        except BaseException as exc:
            self.fault(exc)
            raise
        finally:
            if mapper is not None and self.config.map_policy == "rolling_rebuild":
                mapper.close()

    def status(self):
        return dict(
            valid=self.last_result is not None and not self.failure and not self.closed,
            failure=self.failure,
            completed=self.completed,
            closed=self.closed,
            retained_bytes=self.bytes,
            received=self.received,
            projected=self.projected,
            skipped_sequences=self.skipped_sequences,
            late_chunks=self.late_chunks,
            evidence_ns=self.watermark_ns,
            navigation_authority=False,
            acceptance=False,
        )
