#!/usr/bin/env python3
"""Bounded, read-only Foxglove smoke probe. No third-party WebSocket package.

Checks handshake/capabilities and receives one CDR message on each requested
topic. This is transport evidence, not pose/terrain accuracy or UI acceptance.
"""

import argparse
import base64
import hashlib
import json
import os
import socket
import time

DEFAULT_TOPICS = (
    "/astrobot_0/slam_raw/aligned_points_corrected",
    "/astrobot_0/lidar_selected/points",
    "/astrobot_0/localization/status",
    "/gt/base_link_pose",
    "/terrain_shadow/request",
)


def client_frame(opcode, body):
    mask = os.urandom(4)
    size = len(body)
    if size < 126:
        header = bytes([0x80 | opcode, 0x80 | size])
    elif size < 65536:
        header = bytes([0x80 | opcode, 0xFE]) + size.to_bytes(2, "big")
    else:
        raise ValueError("probe request too large")
    return header + mask + bytes(value ^ mask[index % 4] for index, value in enumerate(body))


def probe(host="127.0.0.1", port=8766, topics=DEFAULT_TOPICS, timeout=20):
    needed, received, channels = set(topics), set(), {}
    capabilities = None
    deadline = time.monotonic() + timeout
    with socket.create_connection((host, port), timeout=timeout) as connection:
        buffer = bytearray()

        def receive():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Missing live topics: {sorted(needed - received)}")
            connection.settimeout(remaining)
            data = connection.recv(65536)
            if not data:
                raise ConnectionError("bridge closed the connection")
            buffer.extend(data)

        def take(length):
            while len(buffer) < length:
                receive()
            result = bytes(buffer[:length])
            del buffer[:length]
            return result

        key = base64.b64encode(os.urandom(16)).decode()
        connection.sendall(
            (
                f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
                "Sec-WebSocket-Protocol: foxglove.sdk.v1, foxglove.websocket.v1\r\n\r\n"
            ).encode()
        )
        while b"\r\n\r\n" not in buffer:
            receive()
            if len(buffer) > 1024 * 1024:
                raise ValueError("unbounded handshake")
        header = take(buffer.index(b"\r\n\r\n") + 4)
        expected = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
        if not header.startswith(b"HTTP/1.1 101 ") or expected not in header:
            raise ValueError(f"WebSocket handshake rejected: {header!r}")
        while capabilities is None or received != needed:
            first, second = take(2)
            if not first & 0x80 or second & 0x80:
                raise ValueError("probe does not accept fragmented or masked server frames")
            length = second & 127
            if length in (126, 127):
                length = int.from_bytes(take(2 if length == 126 else 8), "big")
            if length > 16 * 1024 * 1024:
                raise ValueError("probe frame exceeds 16 MiB")
            payload = take(length)
            opcode = first & 15
            if opcode == 9:
                connection.sendall(client_frame(10, payload))
            elif opcode == 2:
                if len(payload) >= 13 and payload[0] == 1:
                    received.add(channels[int.from_bytes(payload[1:5], "little")])
            elif opcode == 1:
                message = json.loads(payload)
                if message.get("op") == "serverInfo":
                    capabilities = message["capabilities"]
                    if "connectionGraph" not in capabilities or set(capabilities) - {"connectionGraph", "time"}:
                        raise ValueError(f"Unexpected bridge capabilities: {capabilities}")
                if message.get("op") == "advertise":
                    subscriptions = []
                    for channel in message["channels"]:
                        if channel["topic"] in needed:
                            channels[channel["id"]] = channel["topic"]
                            subscriptions.append(dict(id=channel["id"], channelId=channel["id"]))
                    if subscriptions:
                        connection.sendall(
                            client_frame(1, json.dumps(dict(op="subscribe", subscriptions=subscriptions)).encode())
                        )
            elif opcode == 8:
                raise ConnectionError("bridge sent close before all requested topics arrived")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Missing live topics: {sorted(needed - received)}")
        connection.sendall(client_frame(8, (1000).to_bytes(2, "big")))
    return dict(passed=True, capabilities=capabilities, received_topics=sorted(received), accuracy_acceptance=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--handshake-only", action="store_true")
    args = parser.parse_args()
    print(json.dumps(probe(args.host, args.port, () if args.handshake_only else DEFAULT_TOPICS), indent=2))
