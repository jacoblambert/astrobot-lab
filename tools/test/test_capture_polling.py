"""Deterministic executor-load regression; not a live throughput benchmark."""

from collections import deque
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_polling import ReadinessCadence
from capture_polling import StorageCheck, StorageReceipt


@pytest.mark.parametrize("duration", [0.0, 0.08, 0.10, 0.11, 0.18])
def test_check_cost_cannot_consume_the_next_callback_interval(duration):
    cadence = ReadinessCadence()
    assert cadence.due(1.0)
    cadence.finished(1.0 + duration)
    assert not cadence.due(1.0 + duration)
    assert not cadence.due(1.0 + duration + 0.099)
    assert cadence.due(1.0 + duration + 0.101)


def exercise(*, legacy=False, stop_heartbeat=False):
    # 200 callbacks/s, each taking 1 ms; heartbeat every 200 ms. A 110 ms
    # exact archive scan is representative of the failed loop's final checks.
    # This queue model uses synthetic monotonic time and the actual policy;
    # it doesn't infer source emission times from recorder receipt timestamps.
    cadence = ReadinessCadence()
    now = last_heartbeat = next_message = last_check = 0.0
    queue, checked_ages = deque(), []
    sequence = 0
    while now < 2.0:
        while next_message <= now:
            heartbeat = sequence % 40 == 0
            if not (stop_heartbeat and heartbeat and next_message >= 0.4):
                queue.append(heartbeat)
            next_message += 0.005
            sequence += 1
        if queue:
            if queue.popleft():
                last_heartbeat = now
        now += 0.001
        if (now - last_check >= 0.1) if legacy else cadence.due(now):
            last_check = now
            now += 0.11
            checked_ages.append(now - last_heartbeat)
            cadence.finished(now)
            if now - last_heartbeat > 0.5:
                return True, checked_ages
    return False, checked_ages


def test_slow_checks_no_longer_starve_a_healthy_queued_heartbeat():
    old_failed, old_ages = exercise(legacy=True)
    failed, ages = exercise()
    assert old_failed and max(old_ages) > 0.5
    assert not failed and max(ages) <= 0.5
    assert len(ages) >= 8  # Checks still run; they are neither cached nor skipped.


def test_missing_heartbeat_still_fails_the_unchanged_half_second_limit():
    failed, ages = exercise(stop_heartbeat=True)
    assert failed and ages[-1] > 0.5


def test_background_storage_is_single_flight_and_requires_a_new_consumed_result():
    import threading
    import time

    started, finish = threading.Event(), threading.Event()
    calls = []

    def check(key):
        calls.append(key)
        started.set()
        assert finish.wait(1)

    worker = StorageCheck()
    try:
        assert worker.poll(("capture",), check) is None
        assert started.wait(1)
        for _ in range(20):
            assert worker.poll(("capture",), check) is None
        assert calls == [("capture",)]
        finish.set()
        receipt = None
        until = time.monotonic() + 0.3
        while receipt is None and time.monotonic() < until:
            receipt = worker.poll(("capture",), check)
            time.sleep(0.001)
        assert receipt is not None
        receipt.consume(("capture",))
        with pytest.raises(ValueError, match="reused"):
            receipt.consume(("capture",))
        assert worker.poll(("capture",), check) is None  # Not cached success.
    finally:
        finish.set()
        worker.close()


def test_background_storage_failure_and_identity_change_cannot_authorize():
    worker = StorageCheck()
    try:

        def fail(key):
            raise OSError("drive gone")

        worker.poll(("old",), fail)
        with pytest.raises(OSError, match="drive gone"):
            worker.future.result(timeout=1)
        with pytest.raises(OSError, match="drive gone"):
            worker.poll(("new",), fail)
    finally:
        worker.close()
    worker = StorageCheck()
    try:
        worker.poll(("old",), lambda key: None)
        worker.future.result(timeout=1)
        assert worker.poll(("new",), lambda key: None) is None
        assert worker.future is None
    finally:
        worker.close()


def test_storage_receipt_rejects_stale_or_changed_identity():
    import time

    with pytest.raises(ValueError, match="freshness"):
        StorageReceipt(("a",), time.monotonic() - 0.6, time.monotonic()).consume(("a",))
    with pytest.raises(ValueError, match="mismatched"):
        StorageReceipt(("a",), time.monotonic(), time.monotonic()).consume(("b",))


def test_stalled_storage_cannot_silently_continue_and_close_does_not_wait():
    import threading
    import time

    finish = threading.Event()
    worker = StorageCheck()
    try:
        worker.poll(("a",), lambda key: finish.wait(2))
        worker.started -= 0.6
        with pytest.raises(ValueError, match="freshness"):
            worker.poll(("a",), lambda key: None)
        before = time.monotonic()
        worker.close()
        assert time.monotonic() - before < 0.1
    finally:
        finish.set()


def test_background_checks_measure_actual_work_without_caching_success():
    calls = []
    worker = StorageCheck()
    try:
        worker.poll(("a",), lambda key: calls.append(key))
        worker.future.result(timeout=1)
        receipt = worker.poll(("a",), lambda key: calls.append(key))
        assert receipt is not None
        receipt.consume(("a",))
        assert calls == [("a",)] and worker.future is None
        assert worker.poll(("a",), lambda key: calls.append(key)) is None
        worker.future.result(timeout=1)
        result = worker.diagnostics()
        assert result["checks"] == 2 and result["future_done"]
        assert result["maximum_work_s"] >= 0 and result["maximum_result_wait_s"] >= 0
    finally:
        worker.close()
