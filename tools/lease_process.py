"""Receive capture leases outside Isaac's GC/GIL; never publish commands/grants.

The child validates real DDS callback receipts with the unchanged ShadowLease.
The parent requests a fresh snapshot and independently rechecks its age. No
cached permission survives child death, revocation, or an expired heartbeat.
"""

import argparse
import json
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import time

from shadow_capture_gate import HEARTBEAT_S, ShadowLease

MAX_PACKET = 32768
FIELDS = ("identity", "sequence", "sim_stamp", "wall_stamp", "failure", "first_grant")


def send(channel, value):
    payload = json.dumps(value, allow_nan=False).encode()
    if len(payload) >= MAX_PACKET or channel.send(payload) != len(payload):
        raise ValueError("invalid lease IPC write")


def receive(channel):
    payload, _, flags, _ = channel.recvmsg(MAX_PACKET)
    if not payload or flags & socket.MSG_TRUNC:
        raise ValueError("closed/truncated lease IPC")
    return json.loads(payload)


class LeaseProcess(ShadowLease):
    """Synchronous, bounded control-plane query; child alone owns its ROS node."""

    def __init__(self, capture, *, command=None, grant_topic="/terrain_shadow/grant"):
        super().__init__(capture)
        self.channel, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.query = 0
        environment = os.environ.copy()
        # Kit extends sys.path at runtime; those ROS paths need to reach the
        # fresh interpreter too. Do not initialize Kit/PhysX in the child.
        environment["PYTHONPATH"] = os.pathsep.join(sys.path)
        try:
            self.process = subprocess.Popen(
                [
                    *(command or [sys.executable]),
                    str(Path(__file__).resolve()),
                    "--fd",
                    str(child.fileno()),
                    "--capture",
                    capture,
                    "--grant-topic",
                    grant_topic,
                ],
                pass_fds=(child.fileno(),),
                env=environment,
            )
        finally:
            child.close()
        self.channel.settimeout(10.0)  # Pre-authority process startup only.
        try:
            if receive(self.channel) != {"started": True}:
                raise ValueError("lease receiver failed startup")
        except BaseException:
            self.close()
            raise
        self.channel.settimeout(0.2)

    def ready(self, now_sim, now_wall):
        # The simulation thread and independent motion watchdog both query.
        # One request/reply pair must be atomic, just like ShadowLease.ready.
        with self._lock:
            return self._query_ready(now_sim)

    def _query_ready(self, now_sim):
        if self.failure:
            raise ValueError(self.failure)
        try:
            started = time.monotonic()
            for attempt in range(2):
                if self.process.poll() is not None:
                    raise ValueError("lease receiver exited")
                self.query += 1
                send(self.channel, dict(query=self.query, now_sim=now_sim))
                result = receive(self.channel)
                now = time.monotonic()
                age = now - result["checked_wall"]
                if result.get("query") != self.query or age < 0:
                    raise ValueError("mismatched/future lease IPC response")
                if result["failure"]:
                    raise ValueError(result["failure"])
                if now - started > 0.5:
                    raise ValueError("lease IPC query deadline exceeded")
                receipt_age = None if result["wall_stamp"] is None else now - result["wall_stamp"]
                if age <= 0.2 and (receipt_age is None or 0 <= receipt_age <= HEARTBEAT_S):
                    for name in FIELDS:
                        setattr(self, name, result[name])
                    # Both clocks are rechecked after IPC. Child continuity is
                    # authoritative even when a simulator GC delayed decoding.
                    return super().ready(now_sim, time.monotonic())
                # Do NOT use the old snapshot. One fresh query can establish
                # that the child continuously received grants during parent GC.
                # Child expiry is sticky and never retried/revived. The whole
                # read remains bounded by the 500 ms IPC deadline.
            raise ValueError("stale lease IPC response after bounded refresh")
        except Exception as exc:
            self.failure = str(exc) or "lease receiver query failed"
            raise ValueError(self.failure) from exc

    def close(self):
        with self._lock:
            self._close_process()

    def _close_process(self):
        self.channel.close()  # EOF tells the child to stop; no further authority.
        try:
            self.process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)


def serve(channel, capture, grant_topic):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = rclpy.create_node("natural_holdout_lease_receiver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    lease = ShadowLease(capture)
    sim_stamp = 0.0

    def clock(message):
        nonlocal sim_stamp
        stamp = message.clock.sec + message.clock.nanosec / 1e9
        if stamp < sim_stamp and lease.identity is not None:
            lease.failure = "lease receiver simulation clock reversed"
        sim_stamp = stamp

    def grant(message):
        def checked():
            publishers = node.get_publishers_info_by_topic(grant_topic)
            if len(publishers) != 1 or publishers[0].node_name != "terrain_shadow_capture":
                raise ValueError("holdout requires exactly one identified recorder")
            return json.loads(message.data)

        try:
            lease.receive_checked(checked, sim_stamp)
        except Exception as exc:
            lease.failure = str(exc) or "lease receiver callback failed"

    node.create_subscription(Clock, "/clock", clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(String, grant_topic, grant, 1)
    send(channel, dict(started=True))
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.001)
            if lease.identity is not None and not lease.failure:
                try:
                    lease.ready(sim_stamp, time.monotonic())
                except Exception:
                    pass  # Sticky lease.failure is returned; never revived.
            if select.select([channel], [], [], 0)[0]:
                try:
                    request = receive(channel)
                except ValueError:
                    break
                # Query also checks actual simulation-thread time. Receiving
                # DDS /clock late cannot turn stale data into motion authority.
                try:
                    lease.ready(request["now_sim"], time.monotonic())
                except Exception:
                    pass
                send(
                    channel,
                    dict(
                        query=request["query"],
                        checked_wall=time.monotonic(),
                        **{name: getattr(lease, name) for name in FIELDS},
                    ),
                )
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        channel.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fd", type=int, required=True)
    parser.add_argument("--capture", required=True)
    parser.add_argument("--grant-topic", default="/terrain_shadow/grant")
    args = parser.parse_args()
    serve(socket.socket(fileno=args.fd), args.capture, args.grant_topic)
