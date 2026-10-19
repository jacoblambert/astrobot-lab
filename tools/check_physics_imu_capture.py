#!/usr/bin/env python3
"""Bounded stationary wiring check for the opt-in physics IMU topic.

Start before a simulator with OMNILRS_PHYSICS_IMU=true and no command publisher.
Records JSONL on the external archive, sends no commands, starts no processes.
This checks cadence/transport only, NOT moving IMU accuracy or terrain acceptance.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from archive_guard import allocate_run, check_run


def summarize(rows, seconds):
    result, streams = {}, {}
    for topic, hz in (("/astrobot_0/imu", 30.), ("/astrobot_0/imu_physics", 60.)):
        messages = [r for r in rows if r["topic"] == topic]
        stamps = np.array([r["stamp"] for r in messages])
        if len(stamps) < 2 or not np.isfinite(stamps).all() or np.any(np.diff(stamps) <= 0):
            raise ValueError("missing/nonmonotonic IMU stream")
        streams[topic] = (messages, stamps, hz)
    # The opt-in publisher starts after world warm-up/reset. Keep every startup
    # sample, but compare cadence only on the common interval, fixed by stream
    # availability, never by observed residuals or missing-sample locations.
    start = max(stamps[0] for _, stamps, _ in streams.values())
    end = min(stamps[-1] for _, stamps, _ in streams.values())
    for topic, (all_messages, all_stamps, hz) in streams.items():
        keep = (all_stamps >= start) & (all_stamps <= end)
        messages = [m for m, selected in zip(all_messages, keep) if selected]
        stamps = all_stamps[keep]
        if len(stamps) < 2:
            raise ValueError("missing common IMU interval")
        if stamps[-1]-stamps[0] < seconds-.1 or np.max(abs(np.diff(stamps)-1/hz)) > 1e-4:
            raise ValueError("IMU stream duration/cadence failure")
        vectors = np.array([r["acceleration"] + r["gyro"] for r in messages])
        if not np.isfinite(vectors).all():
            raise ValueError("nonfinite IMU vectors")
        if topic.endswith("imu_physics") and any(r["orientation_covariance_0"] != -1 for r in messages):
            raise ValueError("physics stream claims unqualified orientation")
        result[topic] = dict(samples=len(stamps), duration_s=float(stamps[-1]-stamps[0]),
                             recorded_samples=len(all_stamps), outside_common_interval=int((~keep).sum()),
                             recorded_maximum_gap_s=float(np.max(np.diff(all_stamps))),
                             maximum_dt_error_s=float(np.max(abs(np.diff(stamps)-1/hz))))
    statuses = [r["data"] for r in rows if r["topic"].endswith("/clock")]
    if not statuses or any("terminal_fault" in r for r in statuses):
        raise ValueError("missing/faulted physics clock diagnostics")
    valid = [s for s in statuses if s.get("valid")]
    if len(valid) < result["/astrobot_0/imu_physics"]["samples"]-1:
        raise ValueError("missing physics sample clock diagnostics")
    return dict(wiring_passed=True, acceptance=False, moving_accuracy_validated=False,
                no_commands_sent=True, streams=result, clock_samples=len(statuses),
                common_interval_s=[float(start), float(end)],
                startup_samples_retained=True, evaluation_scope="common stream-availability interval only")


def collect(output, seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String

    rclpy.init()
    node = Node("stationary_physics_imu_check")
    rows, first, last, faults = [], {}, {}, []
    started = time.monotonic()
    with (output / "samples.jsonl").open("x") as stream:
        def receive(topic, message):
            if topic.endswith("/clock"):
                data = json.loads(message.data)
                row = dict(topic=topic, data=data)
                if "terminal_fault" in data:
                    faults.append(data["terminal_fault"])
            else:
                stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
                row = dict(topic=topic, stamp=stamp, frame=message.header.frame_id,
                    acceleration=[message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z],
                    gyro=[message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z],
                    orientation_covariance_0=message.orientation_covariance[0])
                first.setdefault(topic, stamp)
                last[topic] = stamp
            rows.append(row)
            stream.write(json.dumps(row, allow_nan=False)+"\n")

        subscriptions = []
        for topic, kind in (("/astrobot_0/imu", Imu), ("/astrobot_0/imu_physics", Imu),
                            ("/astrobot_0/imu_physics/clock", String)):
            subscriptions.append(node.create_subscription(kind, topic,
                lambda message, topic=topic: receive(topic, message), qos_profile_sensor_data))
        try:
            while True:
                if node.count_publishers("/astrobot_0/cmd_vel"):
                    raise ValueError("stationary check refuses a command publisher")
                rclpy.spin_once(node, timeout_sec=.1)
                if faults:
                    raise ValueError(faults[0])
                if time.monotonic()-started > 240:
                    raise TimeoutError("stationary IMU check exceeded startup/collection deadline")
                if len(last) == 2 and all(last[t]-first[t] >= seconds for t in last):
                    break
            check_run(output)
            return summarize(rows, seconds)
        finally:
            node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        report = collect(output, 10.)
        source = Path(__file__).resolve().parents[1]
        report["source_sha256"] = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(__file__).resolve(), source / "OmniLRS/src/environments_wrappers/ros2/physics_imu_ros2.py",
                      source / "OmniLRS/src/environments_wrappers/ros2/simulation_manager_ros2.py")}
        report["samples_sha256"] = hashlib.sha256((output / "samples.jsonl").read_bytes()).hexdigest()
        with (output / "summary.json").open("x") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
        print(json.dumps(report, indent=2), flush=True)
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), type=type(exc).__name__), stream, indent=2)
        raise


if __name__ == "__main__":
    main()
