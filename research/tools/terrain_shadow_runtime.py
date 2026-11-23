"""Receipt-order adapter shared by live ROS and exact event replay."""
from collections import deque

from terrain_shadow_core import DEADLINE_NS


class ReceiptScheduler:
    def __init__(self, core, *, max_events=128, max_bytes=32 * 1024**2):
        self.core = core
        self.pending = deque()
        self.bytes = 0
        self.max_events, self.max_bytes = max_events, max_bytes

    def receive(self, kind, payload, size):
        self.core._check()
        if kind not in ("revision", "chunk", "complete") or size < 0:
            self.core.fault("invalid receipt event")
            raise ValueError(self.core.failure)
        if len(self.pending) >= self.max_events or self.bytes + size > self.max_bytes:
            self.core.fault("receipt queue budget exceeded")
            self.pending.clear()
            self.bytes = 0
            raise ValueError(self.core.failure)
        self.pending.append((kind, payload, size))
        self.bytes += size

    def clock(self, stamp):
        # A revision/native callback is bounded by the NEXT received clock,
        # exactly as archived replay. Never backdate to the previous clock.
        if (stamp < self.core.last_clock_ns or
                self.core.last_clock_ns >= 0 and stamp - self.core.last_clock_ns > DEADLINE_NS):
            self.core.fault("receipt clock reset/gap")
        try:
            self.core._check()
            while self.pending:
                kind, payload, _ = self.pending[0]
                if kind == "revision" and max((f["stamp_ns"] for f in payload.get("frames") or ()), default=-1) > stamp:
                    # Received while this subscriber's /clock queue lagged (live load): the
                    # first clock at/after its newest measurement is the receipt bound. Later
                    # receipts keep their order behind it. Streams that never hit this replay
                    # unchanged (it previously faulted "measurement after arrival").
                    break
                self.pending.popleft()
                if kind == "revision":
                    self.core.revision(payload, stamp)
                elif kind == "chunk":
                    self.core.chunk(payload, stamp)
                else:
                    self.core.complete(payload, stamp)
            return self.core.advance(stamp)
        except Exception:
            self.pending.clear()
            raise
        finally:
            self.bytes = sum(size for _, _, size in self.pending)
