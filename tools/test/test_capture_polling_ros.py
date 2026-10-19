"""Isolated DDS scheduling stress fixture; no simulator or command topics."""

from pathlib import Path
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_polling import ReadinessCadence, StorageCheck


@pytest.mark.parametrize("policy", ["legacy", "completion", "async"])
def test_real_ros_callbacks_get_time_between_slow_readiness_checks(policy):
    rclpy.init()
    sender = rclpy.create_node("polling_fixture_sender")
    receiver = rclpy.create_node("polling_fixture_receiver")
    stop = threading.Event()
    received = [0] * 32
    heartbeat = [time.monotonic()]
    publishers, subscriptions = [], []
    thread = None
    storage = StorageCheck()
    checks = 0

    def callback(index, message):
        received[index] += 1
        if index == 31:
            heartbeat[0] = time.monotonic()

    try:
        for i in range(32):
            topic = f"/polling_fixture/topic_{i}"
            publishers.append(sender.create_publisher(String, topic, 100))
            subscriptions.append(receiver.create_subscription(String, topic, lambda msg, i=i: callback(i, msg), 100))
        deadline = time.monotonic() + 5.0
        while not all(p.get_subscription_count() == 1 for p in publishers):
            assert time.monotonic() < deadline, "isolated DDS discovery failed"
            time.sleep(0.01)

        def publish():
            while not stop.is_set():
                for publisher in publishers:
                    publisher.publish(String(data="synthetic scheduling load"))
                stop.wait(0.2)

        thread = threading.Thread(target=publish)
        thread.start()
        deadline = time.monotonic() + 5.0
        while not all(received):
            rclpy.spin_once(receiver, timeout_sec=0.01)
            assert time.monotonic() < deadline, "fixture never became ready"
        cadence = ReadinessCadence()
        last_check = 0.0
        failed = False
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(receiver, timeout_sec=0.001)
            now = time.monotonic()
            if policy == "async":
                receipt = storage.poll(("synthetic",), lambda key: time.sleep(.11))
                if receipt is not None:
                    receipt.consume(("synthetic",))
                    checks += 1
                    failed = time.monotonic() - heartbeat[0] > .5
                    if failed:
                        break
                continue
            due = now - last_check >= 0.1 if policy == "legacy" else cadence.due(now)
            if due:
                last_check = now
                time.sleep(0.11)  # Measured long-archive check cost; not simulated sensor latency.
                failed = time.monotonic() - heartbeat[0] > 0.5
                cadence.finished(time.monotonic())
                if failed:
                    break
        assert failed is (policy == "legacy")
        if policy != "legacy":
            assert min(received) >= 6
        if policy == "async":
            assert checks >= 10  # Every result is a fresh full check, not a timer renewal.
    finally:
        storage.close()
        stop.set()
        if thread is not None:
            thread.join(timeout=2.0)
        receiver.destroy_node()
        sender.destroy_node()
        rclpy.shutdown()
