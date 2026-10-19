"""Reserve callback-processing time between expensive capture readiness checks.

This schedules work only. It does not refresh a heartbeat, grant a lease, cache
an archive check, or change any freshness/safety threshold.
"""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import time


@dataclass
class StorageReceipt:
    key: tuple
    started: float
    completed: float
    consumed: bool = False

    def consume(self, expected):
        if self.consumed or self.key != expected:
            raise ValueError("reused or mismatched capture storage check")
        self.consumed = True
        if not 0 <= time.monotonic() - self.started <= 0.5:
            raise ValueError("capture storage check exceeded freshness deadline")


class StorageCheck:
    """One exact filesystem check in flight; ROS state stays on its executor.

    Every grant consumes a new result. No cached success, background grant,
    unbounded work queue, or relaxation of a stale/failed check is permitted.
    """

    def __init__(self):
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="capture_storage")
        self.future = None
        self.key = None
        self.started = None
        self.worker_started = self.worker_completed = None
        self.maximum_work_s = self.maximum_queue_s = self.maximum_result_wait_s = 0.0
        self.checks = 0

    def _check(self, check, key):
        self.worker_started = time.monotonic()
        self.maximum_queue_s = max(self.maximum_queue_s, self.worker_started - self.started)
        try:
            return check(key)
        finally:
            self.worker_completed = time.monotonic()
            self.maximum_work_s = max(self.maximum_work_s, self.worker_completed - self.worker_started)
            self.checks += 1

    def diagnostics(self):
        now = time.monotonic()
        return dict(
            checks=self.checks,
            maximum_work_s=self.maximum_work_s,
            maximum_queue_s=self.maximum_queue_s,
            maximum_result_wait_s=self.maximum_result_wait_s,
            pending_age_s=None if self.future is None else now - self.started,
            worker_started=self.worker_started,
            worker_completed=self.worker_completed,
            future_done=None if self.future is None else self.future.done(),
        )

    def poll(self, key, check):
        if self.future is None:
            self.key, self.started = key, time.monotonic()
            self.worker_started = self.worker_completed = None
            self.future = self.worker.submit(self._check, check, key)
            return None
        if time.monotonic() - self.started > 0.5:
            raise ValueError(f"capture storage check exceeded freshness deadline: {self.diagnostics()}")
        if not self.future.done():
            return None
        self.future.result()  # Propagate errors even if the request identity changed.
        receipt = StorageReceipt(self.key, self.started, time.monotonic())
        self.maximum_result_wait_s = max(self.maximum_result_wait_s, receipt.completed - self.worker_completed)
        self.future = None
        return receipt if key == self.key else None

    def close(self):
        # A stalled filesystem must not prevent revocation and failure sealing.
        # The worker owns no ROS state or writer; its late result cannot grant.
        self.worker.shutdown(wait=False, cancel_futures=True)


class ReadinessCadence:
    INTERVAL_S = 0.1

    def __init__(self):
        self.next_check = 0.0

    def due(self, now):
        return now >= self.next_check

    def finished(self, now):
        # Measured from completion, not start. A 100+ ms check must not cause
        # another check immediately after only one ROS callback was serviced.
        self.next_check = now + self.INTERVAL_S
