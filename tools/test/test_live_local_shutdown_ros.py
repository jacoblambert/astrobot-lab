"""Opt-in real ROS SIGINT regression; isolated domain 179, no sensor/motion data."""

import json
import hashlib
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import pytest


def test_sigint_seals_real_node_without_false_completion():
    request = os.environ.get("TERRAIN_LIVE_REQUEST")
    library = os.environ.get("TERRAIN_LIBRARY")
    if not request or not library:
        pytest.skip("sourced ROS + explicit archived TERRAIN_LIVE_REQUEST and TERRAIN_LIBRARY required")
    command = [
        sys.executable,
        str(Path(__file__).resolve().parents[1] / "live_local_terrain.py"),
        "--ros-args",
        "-p",
        "request_json:=" + request,
        "-p",
        "library:=" + library,
    ]
    loopback = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>'
    process = subprocess.Popen(
        command,
        env=dict(os.environ, ROS_DOMAIN_ID="179", CYCLONEDDS_URI=loopback),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        time.sleep(4)  # Bounded startup, no mission clocks or publishers in this domain.
        assert process.poll() is None
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=5)  # Same SIGTERM escalation budget as launch.
        assert process.returncode == 0, output
        matches = re.findall(r"Local v2 archive: (/archive/revision-[a-f0-9]+)", output)
        assert len(matches) == 1, output
        report = json.loads((Path(matches[0]) / "collection.json").read_text())
        assert report["status"]["closed"] and report["status"]["failure"] is None
        assert report["status"]["received"] == 0 and not report["experiment_completed"]
        assert set(report["output_sha256"]) == {"events.jsonl", "native.bin", "projections.jsonl"}
        for name, expected in report["output_sha256"].items():
            assert hashlib.sha256((Path(matches[0]) / name).read_bytes()).hexdigest() == expected
        print("Shutdown fixture:", matches[0])
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)
