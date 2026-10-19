import base64
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from probe_inspection_bridge import client_frame, probe


@pytest.mark.parametrize("size", [0, 3, 125, 126, 900])
def test_client_frames_are_masked_and_roundtrip(size):
    body = bytes(index % 255 for index in range(size))
    frame = client_frame(1, body)
    assert frame[0] == 0x81 and frame[1] & 0x80
    offset = 2 if size < 126 else 4
    encoded_size = frame[1] & 127 if offset == 2 else int.from_bytes(frame[2:4], "big")
    assert encoded_size == size
    mask = frame[offset : offset + 4]
    assert bytes(value ^ mask[i % 4] for i, value in enumerate(frame[offset + 4 :])) == body


def test_client_frame_bound():
    with pytest.raises(ValueError, match="too large"):
        client_frame(1, b"x" * 65536)


@pytest.mark.parametrize("unsafe", [False, True])
def test_real_probe_checks_capabilities_before_accepting_data(monkeypatch, unsafe):
    def frame(opcode, payload):
        return bytes([0x80 | opcode, 126]) + len(payload).to_bytes(2, "big") + payload

    class Connection:
        buffer = b""
        sent = []

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def settimeout(self, timeout):
            assert 0 < timeout <= 20

        def recv(self, size):
            # Deliberately split headers and frames across small network reads.
            result, self.buffer = self.buffer[:7], self.buffer[7:]
            return result

        def sendall(self, data):
            self.sent.append(data)
            if data.startswith(b"GET "):
                assert b"foxglove.sdk.v1, foxglove.websocket.v1" in data
                key = data.split(b"Sec-WebSocket-Key: ")[1].split(b"\r\n")[0]
                digest = base64.b64encode(hashlib.sha1(key + b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest())
                self.buffer = b"HTTP/1.1 101 Switching Protocols\r\nSec-WebSocket-Accept: " + digest + b"\r\n\r\n"
                caps = ["connectionGraph", "time"] + (["clientPublish"] if unsafe else [])
                self.buffer += frame(1, json.dumps(dict(op="serverInfo", capabilities=caps)).encode())
                self.buffer += frame(1, json.dumps(dict(op="advertise", channels=[dict(id=1, topic="/test")])).encode())
                self.buffer += frame(2, b"\x01" + (1).to_bytes(4, "little") + bytes(8) + b"cdr")

    connection = Connection()
    monkeypatch.setattr("probe_inspection_bridge.socket.create_connection", lambda *a, **k: connection)
    if unsafe:
        with pytest.raises(ValueError, match="Unexpected bridge capabilities"):
            probe(topics=["/test"])
    else:
        result = probe(topics=["/test"])
        assert result["passed"] and result["received_topics"] == ["/test"]
        assert not result["accuracy_acceptance"]
