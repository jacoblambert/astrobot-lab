"""Isolated real DDS lease reception while the simulation thread is busy.

This is an I/O-service fixture. Real GIL-holding GC is tested separately in the
explicit Isaac stress protocol; sleeping here is not presented as GC emulation.
"""

from pathlib import Path
import sys
import time
import json

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lease_callbacks import LeaseCallbacks
from shadow_capture_gate import HEARTBEAT_S, ShadowLease
from test_shadow_capture_gate import CAPTURE, grant


def test_background_receives_real_grants_without_main_spin_and_dead_sender_still_expires():
    rclpy.init()
    sender = rclpy.create_node("lease_fixture_sender")
    receiver = rclpy.create_node("lease_fixture_receiver")
    executor = SingleThreadedExecutor()
    executor.add_node(receiver)
    lease, failures = ShadowLease(CAPTURE), []
    def receive(message):
        try:
            lease.receive(json.loads(message.data), 10., time.monotonic())
        except Exception as exc:
            failures.append(str(exc))
    receiver.create_subscription(String, "/lease_fixture/grant", receive, 1)
    publisher = sender.create_publisher(String, "/lease_fixture/grant", 1)
    pump = LeaseCallbacks(executor, failures.append)
    try:
        end = time.monotonic()+5
        while publisher.get_subscription_count() != 1:
            assert time.monotonic()<end
            time.sleep(.01)
        for seq in range(12):
            publisher.publish(String(data=json.dumps(grant(sequence=seq))))
            time.sleep(.06)  # No main-thread executor spin at any point.
        assert not failures and lease.sequence == 11
        assert lease.ready(10., time.monotonic())
        time.sleep(HEARTBEAT_S + .05)
        with pytest.raises(ValueError, match="expired"):
            lease.ready(10., time.monotonic())
        publisher.publish(String(data=json.dumps(grant(sequence=12))))
        end = time.monotonic()+1
        while not failures and time.monotonic()<end:
            time.sleep(.01)
        assert failures and lease.sequence == 11  # No late revival.
    finally:
        pump.close()
        assert not pump.thread.is_alive()
        executor.remove_node(receiver)
        receiver.destroy_node()
        sender.destroy_node()
        rclpy.shutdown()


def test_executor_exception_is_reported_and_thread_terminates():
    class Broken:
        def spin_once(self, **kwargs):
            raise RuntimeError("broken callback executor")
        def wake(self):
            pass
    failures = []
    pump = LeaseCallbacks(Broken(), failures.append)
    pump.thread.join(timeout=1)
    pump.close()
    assert failures == ["broken callback executor"]


@pytest.mark.parametrize("checked", [False, True])
def test_validation_gil_pause_uses_receipt_not_completion_time(checked):
    import ctypes
    import threading

    rclpy.init()
    sender = rclpy.create_node("lease_validation_sender")
    receiver = rclpy.create_node("lease_validation_receiver")
    executor = SingleThreadedExecutor()
    executor.add_node(receiver)
    lease, failures, completed = ShadowLease(CAPTURE), [], threading.Event()
    libc = ctypes.PyDLL(None)
    libc.usleep.argtypes, libc.usleep.restype = [ctypes.c_uint], ctypes.c_int

    def receive(message):
        payload = json.loads(message.data)
        def validate():
            if payload["sequence"] == 1:
                assert libc.usleep(350000) == 0
            return payload
        try:
            if checked:
                lease.receive_checked(validate, 10.)
            else:
                value = validate()
                lease.receive(value, 10., time.monotonic())
        except Exception as exc:
            failures.append(str(exc))
        finally:
            if payload["sequence"] == 1:
                completed.set()

    receiver.create_subscription(String, "/lease_fixture/validation", receive, 1)
    publisher = sender.create_publisher(String, "/lease_fixture/validation", 1)
    pump = LeaseCallbacks(executor, failures.append)
    try:
        deadline = time.monotonic()+5
        while publisher.get_subscription_count() != 1:
            assert time.monotonic() < deadline
            time.sleep(.01)
        publisher.publish(String(data=json.dumps(grant(sequence=0))))
        while lease.sequence < 0:
            assert time.monotonic() < deadline
            time.sleep(.001)
        time.sleep(.22 + HEARTBEAT_S - .5)  # pause + validation exceed the heartbeat by 70 ms
        publisher.publish(String(data=json.dumps(grant(sequence=1))))
        assert completed.wait(2)
        if checked:
            assert not failures and lease.sequence == 1
            assert .35 <= time.monotonic()-lease.wall_stamp < .5
            assert lease.ready(10., time.monotonic())
        else:
            assert failures and "expired" in failures[0] and lease.sequence == 0
    finally:
        pump.close()
        executor.remove_node(receiver)
        receiver.destroy_node()
        sender.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("background", [False, True])
def test_fixed_gil_pause_plus_io_reproduces_queued_lease_failure(background):
    import ctypes
    import subprocess

    rclpy.init()
    receiver = rclpy.create_node("lease_gil_fixture_receiver")
    executor = SingleThreadedExecutor()
    executor.add_node(receiver)
    lease, failures = ShadowLease(CAPTURE), []
    def receive(message):
        try:
            lease.receive(json.loads(message.data), 10., time.monotonic())
        except Exception as exc:
            failures.append(str(exc))
    receiver.create_subscription(String, "/lease_fixture/gil_grant", receive, 1)
    pump = LeaseCallbacks(executor, failures.append) if background else None
    # Separate process: sender keeps running while PyDLL holds this interpreter's
    # GIL. No simulation, commands or publisher outside isolated ROS domain 179.
    program = '''
import json, sys, time
import rclpy
from std_msgs.msg import String
rclpy.init()
node=rclpy.create_node("lease_gil_fixture_sender")
pub=node.create_publisher(String,"/lease_fixture/gil_grant",1)
deadline=time.monotonic()+5
while pub.get_subscription_count()!=1:
    if time.monotonic()>deadline: raise RuntimeError("DDS discovery timed out")
    time.sleep(.01)
payload=json.loads(sys.argv[1])
for seq in range(100):
    payload["sequence"]=seq
    pub.publish(String(data=json.dumps(payload)))
    time.sleep(.1)
'''
    sender = subprocess.Popen([sys.executable, "-c", program, json.dumps(grant())],
                              stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        end = time.monotonic()+6
        while lease.sequence < 0:
            assert time.monotonic()<end and sender.poll() is None
            if background:
                time.sleep(.001)
            else:
                executor.spin_once(timeout_sec=.01)
        time.sleep(.12)  # Lease phase at tick entry; old main loop does not spin.
        libc = ctypes.PyDLL(None)  # Unlike CDLL, intentionally retains the GIL.
        libc.usleep.argtypes, libc.usleep.restype = [ctypes.c_uint], ctypes.c_int
        assert libc.usleep(round((.4 + HEARTBEAT_S - .5)*1e6)) == 0  # 60 ms past the heartbeat
        time.sleep(.04)  # Remaining native file I/O; main still does not spin.
        if background:
            assert not failures and lease.sequence >= 4
            assert lease.ready(10., time.monotonic())
        else:
            with pytest.raises(ValueError, match="expired"):
                lease.ready(10., time.monotonic())
    finally:
        sender.terminate()
        sender.communicate(timeout=5)
        if pump:
            pump.close()
        executor.remove_node(receiver)
        receiver.destroy_node()
        rclpy.shutdown()
