"""Real ROS serialization/MCAP, tiny synthetic fixtures; no simulation authority."""

import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from generic_lidar_adapter import NativeWindow, message, DTYPE
from selected_lidar_profile import TOPIC, declaration
from selected_native_audit import audit_transport
from test_generic_lidar_adapter import tick
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import PointCloud2
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata


@pytest.mark.parametrize("fault", [None, "tail", "beam", "stride"])
@pytest.mark.parametrize("policy", ["nearest_ns_v1", "isaac_clock_ns_v2"])
def test_every_native_tick_has_exact_recorded_window_including_boundaries(tmp_path, fault, policy):
    capture, recorder = tmp_path / "capture", tmp_path / "recorder"
    capture.mkdir()
    recorder.mkdir()
    (capture / "native_chunks").mkdir()
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=str(recorder / "bag"), storage_id="mcap"), ConverterOptions("", ""))
    writer.create_topic(TopicMetadata(id=0, name=TOPIC, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
    adapter = NativeWindow(timestamp_policy=policy)
    (capture / "selected_sensor.json").write_text(json.dumps(dict(profile=declaration(), timestamp_policy=policy)))
    frames, windows = [], []
    for i in range(12):
        arrays = tick(adapter)
        acquired = 2 + i / 60
        # Archive boundaries intentionally don't match six-tick windows.
        if 2 <= i <= 9:
            path = capture / "native_chunks" / f"chunk_{len(frames):06}.npz"
            np.savez(
                path,
                **dict(zip(("points", "depths", "intensities", "azimuth", "zenith"), arrays)),
                observed_stamp=acquired,
                world_from_lidar=np.eye(4),
            )
            frames.append(
                dict(
                    file=path.name,
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    observed_stamp=acquired,
                    producer_sequence=i,
                    world_from_lidar=np.eye(4).tolist(),
                )
            )
        window = adapter.push(i, acquired, *arrays)
        if window is None:
            continue
        if fault == "beam" and i == 5:
            rows = np.frombuffer(window["data"], dtype=DTYPE).copy()
            rows["beam_id"][:] = 0
            window["data"] = rows.tobytes()
        msg = message(window)
        serialized = serialize_message(msg)
        restored = deserialize_message(serialized, PointCloud2)
        assert len(restored.data) == 32 * restored.width
        assert bytes(restored.data) == window["data"]
        windows.append(dict((k, v) for k, v in window.items() if k != "data"))
        windows[-1]["sha256"] = hashlib.sha256(window["data"]).hexdigest()
        if fault == "stride":
            msg.data = msg.data[:-2]
            serialized = serialize_message(msg)
        if not (fault == "tail" and i == 11):
            writer.write(TOPIC, serialized, i * 10**9)
    del writer
    (capture / "native_timing_audit.json").write_text(json.dumps(dict(frames=frames)))
    (capture / "selected_windows.jsonl").write_text("".join(json.dumps(w) + "\n" for w in windows))
    if fault:
        with pytest.raises(ValueError):
            audit_transport(capture, recorder)
    else:
        result = audit_transport(capture, recorder)
        assert result["native_ticks"] == 8 and result["recorded_windows"] == 2
        assert result["points_checked"] == 8 * 10661
