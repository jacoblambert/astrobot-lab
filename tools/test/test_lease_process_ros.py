"""Real DDS, isolated domain: receiver must survive Isaac's GIL-held pauses."""

import ctypes
import json
from pathlib import Path
import subprocess
import sys
import time

import pytest

pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lease_process import LeaseProcess
from shadow_capture_gate import HEARTBEAT_S
from test_shadow_capture_gate import CAPTURE, grant


SENDER = """
import json, sys, time
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
rclpy.init(args=[])
node = rclpy.create_node(sys.argv[2])
pub = node.create_publisher(String, "/lease_process_fixture/grant", 1)
clock = node.create_publisher(Clock, "/clock", 1)
deadline = time.monotonic()+8
while pub.get_subscription_count()!=1 or clock.get_subscription_count()!=1:
    if time.monotonic()>deadline: raise RuntimeError("DDS discovery timeout")
    time.sleep(.01)
stamp=Clock(); stamp.clock.sec=10
clock.publish(stamp)
time.sleep(.05)
payload=json.loads(sys.argv[1])
for seq in range(100):
    clock.publish(stamp)
    payload["sequence"]=seq
    if sys.argv[3] == "revoke" and seq >= 1: payload["ready"]=False
    pub.publish(String(data=json.dumps(payload)))
    time.sleep(.24)
"""


@pytest.fixture
def receiver():
    lease = LeaseProcess(CAPTURE, grant_topic="/lease_process_fixture/grant")
    yield lease
    lease.close()
    assert lease.process.poll() is not None


@pytest.fixture
def sender():
    processes = []

    def start(name="terrain_shadow_capture", mode="normal"):
        process = subprocess.Popen([sys.executable, "-c", SENDER, json.dumps(grant()), name, mode])
        processes.append(process)
        return process

    yield start
    for process in processes:
        process.terminate()
        process.wait(timeout=5)


def armed(receiver):
    deadline = time.monotonic() + 8
    while not receiver.ready(10.0, time.monotonic()):
        assert time.monotonic() < deadline
        time.sleep(0.005)


def pause_gil():
    # Start ~180 ms after a receipt. The next 360 ms pause crosses the old
    # 500 ms deadline if ROS reception shares this interpreter's GIL.
    libc = ctypes.PyDLL(None)
    libc.usleep.argtypes, libc.usleep.restype = [ctypes.c_uint], ctypes.c_int
    time.sleep(0.18)
    assert libc.usleep(360000) == 0


def test_separate_receiver_survives_observed_gap_plus_gc(receiver, sender):
    sender()
    armed(receiver)
    previous = receiver.sequence
    pause_gil()
    assert receiver.ready(10.0, time.monotonic())
    assert receiver.sequence >= previous + 2
    assert time.monotonic() - receiver.wall_stamp < 0.3


def test_watchdog_and_simulation_queries_cannot_cross_responses(receiver, sender):
    from concurrent.futures import ThreadPoolExecutor

    sender()
    armed(receiver)

    def query_many():
        for _ in range(120):
            assert receiver.ready(10.0, time.monotonic())

    with ThreadPoolExecutor(max_workers=3) as pool:
        tasks = [pool.submit(query_many) for _ in range(3)]
        for task in tasks:
            task.result(timeout=8)
    assert receiver.failure is None


@pytest.mark.parametrize("mode", ["normal", "revoke", "two_pauses"])
def test_gc_while_decoding_snapshot_requires_fresh_query_not_false_expiry(receiver, sender, monkeypatch, mode):
    import lease_process

    sender(mode="revoke" if mode == "revoke" else "normal")
    armed(receiver)
    original = lease_process.receive
    replies = []

    def delayed(channel):
        value = original(channel)
        replies.append(value)
        if len(replies) == 1 or mode == "two_pauses":
            libc = ctypes.PyDLL(None)
            libc.usleep.argtypes, libc.usleep.restype = [ctypes.c_uint], ctypes.c_int
            assert libc.usleep(360000) == 0
        return value

    monkeypatch.setattr(lease_process, "receive", delayed)
    if mode != "normal":
        with pytest.raises(ValueError, match="revoked" if mode == "revoke" else "deadline"):
            receiver.ready(10.0, time.monotonic())
        assert receiver.failure
        return
    assert receiver.ready(10.0, time.monotonic())
    assert len(replies) == 2
    assert receiver.sequence > replies[0]["sequence"]


def test_revocation_during_gc_is_not_hidden_by_later_snapshots(receiver, sender):
    sender(mode="revoke")
    armed(receiver)
    pause_gil()
    with pytest.raises(ValueError, match="revoked"):
        receiver.ready(10.0, time.monotonic())


def test_dead_sender_expires_and_late_restart_cannot_revive(receiver, sender):
    process = sender()
    armed(receiver)
    process.terminate()
    process.wait(timeout=5)
    time.sleep(HEARTBEAT_S + .05)
    sender()
    with pytest.raises(ValueError, match="expired"):
        receiver.ready(10.0, time.monotonic())


def test_wrong_publisher_cannot_arm(receiver, sender):
    sender(name="untrusted_fixture")
    with pytest.raises(ValueError, match="identified recorder"):
        armed(receiver)


def test_child_death_cannot_reuse_last_good_snapshot(receiver, sender):
    sender()
    armed(receiver)
    receiver.process.terminate()
    receiver.process.wait(timeout=5)
    with pytest.raises(ValueError, match="receiver exited"):
        receiver.ready(10.0, time.monotonic())


def test_main_simulation_clock_is_independently_checked(receiver, sender):
    sender()
    armed(receiver)
    with pytest.raises(ValueError, match="expired or clock reversed"):
        receiver.ready(10.5 + HEARTBEAT_S, time.monotonic())


def test_child_hang_times_out_without_reusing_snapshot(receiver, sender):
    import os
    import signal

    sender()
    armed(receiver)
    os.kill(receiver.process.pid, signal.SIGSTOP)
    try:
        start = time.monotonic()
        with pytest.raises(ValueError, match="timed out"):
            receiver.ready(10.0, start)
        assert time.monotonic() - start < 0.5
    finally:
        os.kill(receiver.process.pid, signal.SIGCONT)
