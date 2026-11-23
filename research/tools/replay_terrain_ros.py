#!/usr/bin/env python3
"""Paced ROS replay of a raw terrain run (events.jsonl + native.bin) into a live terrain node.

Publishes /clock, the SLAM revisions, the native chunks and calls slam/reset_map in recorded
order, pacing each event so the node handles them in that order. Records every planner window
the node publishes and compares it with an offline terrain_replay output directory.

  replay_terrain_ros.py <run_dir> <terrain_replay_out_dir>

with the node running as: ros2 run astrobot_terrain terrain --ros-args -r __ns:=/astrobot_0
  --params-file <astrobot_bringup>/config/robot.yaml -p use_sim_time:=true
"""

import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from astrobot_interfaces.msg import SlamFramePose, SlamRevision
from astrobot_interfaces.srv import ResetMap
from scipy.spatial.transform import Rotation

FIELDS = (("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1), ("depth", 12, 7, 1), ("intensity", 16, 2, 1),
          ("sequence", 20, 6, 1), ("capture_uuid", 24, 2, 16))


def stamp(msg, ns):
    msg.sec, msg.nanosec = divmod(int(ns), 10**9)


def pose(msg, matrix):
    matrix = np.asarray(matrix)
    msg.position.x, msg.position.y, msg.position.z = map(float, matrix[:3, 3])
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(
        float, Rotation.from_matrix(matrix[:3, :3]).as_quat())


def main(run, offline):
    run, offline = Path(run), Path(offline)
    rclpy.init()
    node = rclpy.create_node("terrain_replay_ros")
    clock_pub = node.create_publisher(Clock, "/clock", 100)
    revisions = node.create_publisher(SlamRevision, "/astrobot_0/slam_raw/revisions", QoSProfile(
        depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    chunks = node.create_publisher(PointCloud2, "/astrobot_0/lidar_selected/chunks", 64)
    reset = node.create_client(ResetMap, "/astrobot_0/slam/reset_map")
    windows, statuses = {}, []
    node.create_subscription(OccupancyGrid, "/astrobot_0/terrain/local_costmap",
                             lambda m: windows.__setitem__(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec,
                                                           np.asarray(m.data, np.int8).reshape(60, 60)), 10)
    node.create_subscription(String, "/astrobot_0/terrain/status", lambda m: statuses.append(json.loads(m.data)), 10)
    if not reset.wait_for_service(timeout_sec=20):
        raise SystemExit("terrain node not up")
    time.sleep(1.0)  # discovery
    raw = (run / "native.bin").open("rb")

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.001)

    for line in (run / "events.jsonl").open():
        event = json.loads(line)
        kind, payload = event["kind"], event["payload"]
        if kind == "reset":
            request = ResetMap.Request(epoch=payload["epoch"])
            stamp(request.minimum_stamp, payload["minimum_ns"])
            future = reset.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.result().success, future.result()
        elif kind == "revision":
            msg = SlamRevision(session_id=payload["session"], localization_epoch=payload["epoch"],
                               revision=payload["revision"], parent_revision=payload["parent"], kind=payload["kind"],
                               full_snapshot=payload["snapshot"])
            msg.header.frame_id = "map"
            for f in payload["frames"]:
                frame = SlamFramePose(frame_id=f["frame_id"], submap_id=-1, deskew_recomputed=f["deskew_recomputed"])
                stamp(frame.measurement_stamp, f["stamp_ns"])
                pose(frame.map_from_imu, f["map_from_imu"])
                pose(frame.imu_from_lidar, f["imu_from_lidar"])
                msg.frames.append(frame)
            revisions.publish(msg)
        elif kind == "chunk":
            raw.seek(payload["offset"])
            data = raw.read(payload["size"])
            cloud = PointCloud2(height=1, width=payload["size"] // 40, is_bigendian=False, is_dense=True,
                                point_step=40, row_step=payload["size"], data=data)
            cloud.header.frame_id = "vlp16"
            stamp(cloud.header.stamp, payload["stamp_ns"])
            cloud.fields = [PointField(name=n, offset=o, datatype=t, count=c) for n, o, t, c in FIELDS]
            chunks.publish(cloud)
        elif kind == "clock":
            msg = Clock()
            stamp(msg.clock, payload)
            clock_pub.publish(msg)
        spin(0.004)  # paced: the node handles events in recorded order
    spin(2.0)
    expected = np.frombuffer((offline / "windows.bin").read_bytes(), np.int8).reshape(-1, 60, 60)
    stamps = [json.loads(line)["stamp_ns"] for line in (offline / "windows.jsonl").open()]
    matched = [k for k in stamps if k in windows]
    differing = sum(int((windows[k] != expected[i]).sum()) for i, k in enumerate(stamps) if k in windows)
    last = statuses[-1] if statuses else {}
    print(json.dumps(dict(offline_windows=len(stamps), ros_windows_matched=len(matched), cells_differing=differing,
                          status=dict((key, last.get(key)) for key in ("valid", "failure", "epoch", "received",
                                                                      "projected", "late_chunks")))))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if len(matched) == len(stamps) and differing == 0 and last.get("valid") else 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
