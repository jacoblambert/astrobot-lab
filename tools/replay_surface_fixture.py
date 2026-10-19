#!/usr/bin/env python3
"""Step-and-ack replay through the live C++ mapper and shared mapper_surface.yaml.

Requires aligned clouds and exact matching lidar_pose_corrected measurements.
Runs only in a separate ROS domain, never alongside the live domain 177. This
isolates mapper repeatability. --gt-isolate removes pose error with timestamped
full-SE(3) reference alignment; neither mode by itself scores obstacle recall.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from astrobot_interfaces.msg import LocalizationStatus
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2
from fixture_alignment import ReferenceTrajectory, pose_matrix, reanchor_scan, validate_fixture_epochs


def ns(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


def message_matrix(msg):
    p,q=msg.pose.position,msg.pose.orientation
    return pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])


def read_fixture(path, gt_isolate=False, home_xyz=(20.,18.,0.),
                 base_imu_xyz=(.264,.017,.262), base_imu_quaternion=(0.,0.,1.,0.)):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id="mcap"), ConverterOptions("", ""))
    clouds, poses, imu_poses, gt_rows = {}, {}, {}, []
    epochs = set()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/astrobot_0/slam/aligned_points_corrected":
            msg = deserialize_message(data, PointCloud2)
            if ns(msg) in clouds:
                raise ValueError("duplicate cloud measurement stamp")
            clouds[ns(msg)] = msg
        elif topic == "/astrobot_0/slam/lidar_pose_corrected":
            msg = deserialize_message(data, PoseStamped)
            if ns(msg) in poses:
                raise ValueError("duplicate or revised LiDAR pose measurement stamp")
            poses[ns(msg)] = msg
        elif gt_isolate and topic == "/astrobot_0/slam/pose_corrected":
            msg = deserialize_message(data, PoseStamped)
            if ns(msg) in imu_poses:
                raise ValueError("duplicate or revised IMU pose measurement stamp")
            imu_poses[ns(msg)] = msg
        elif topic == "/astrobot_0/localization/status":
            epochs.add(deserialize_message(data, LocalizationStatus).epoch)
        elif gt_isolate and topic == "/gt/base_link_pose":
            msg = deserialize_message(data, PoseStamped)
            if msg.header.frame_id != "gt_map":
                raise ValueError("GT reference is not in gt_map frame")
            p,q=msg.pose.position,msg.pose.orientation
            gt_rows.append((ns(msg)*1e-9,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
    if not clouds or any(ts not in poses for ts in clouds):
        raise ValueError("fixture requires every cloud to have an exact timestamped LiDAR pose")
    metadata = {"source_frame_count": len(clouds), "gt_isolated": gt_isolate, "excluded_boundary_stamps": [],
                "localization_epochs": validate_fixture_epochs(epochs),
                "epoch_validation": "single_epoch" if epochs else "unavailable_no_health_topic"}
    if not gt_isolate:
        return [(poses[ts], clouds[ts]) for ts in sorted(clouds)], metadata
    if any(ts not in imu_poses for ts in clouds):
        raise ValueError("GT isolation needs an exact IMU pose for every scan")
    reference = ReferenceTrajectory(gt_rows)
    frames = []
    for ts in sorted(clouds):
        if not reference.times[0] <= ts*1e-9 <= reference.times[-1]:
            # Recording boundaries need not align across topics. Explicitly
            # account for them; never extrapolate or hide interior reference gaps.
            metadata["excluded_boundary_stamps"].append(ts)
            continue
        pose,cloud=poses[ts],clouds[ts]
        if pose.header.frame_id != "map" or imu_poses[ts].header.frame_id != "map" or cloud.header.frame_id != "map":
            raise ValueError("SLAM fixture is not in map frame")
        points = point_cloud2.read_points_numpy(cloud, field_names=["x","y","z"], skip_nans=False)
        points, lidar = reanchor_scan(points,message_matrix(imu_poses[ts]),message_matrix(pose),
            reference.at(ts*1e-9),home_xyz,base_imu_xyz,base_imu_quaternion)
        output_pose=copy.deepcopy(pose)
        output_pose.pose.position.x,output_pose.pose.position.y,output_pose.pose.position.z=map(float,lidar[:3,3])
        q=Rotation.from_matrix(lidar[:3,:3]).as_quat()
        output_pose.pose.orientation.x,output_pose.pose.orientation.y,output_pose.pose.orientation.z,output_pose.pose.orientation.w=map(float,q)
        output_cloud=point_cloud2.create_cloud_xyz32(copy.deepcopy(cloud.header),points.astype(np.float32))
        frames.append((output_pose,output_cloud))
    if not frames:
        raise ValueError("no scans inside the validated GT trajectory")
    metadata.update(home_xyz=list(home_xyz),base_imu_xyz=list(base_imu_xyz),
                    base_imu_quaternion=list(base_imu_quaternion),reference_interpolation="linear_xyz_slerp_no_extrapolation")
    return frames,metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--params", type=Path, default=Path(__file__).resolve().parents[1]/
                        "astrobot-lab/src/astrobot_launch/config/mapper_surface.yaml")
    parser.add_argument("--domain-id", type=int, default=179)
    parser.add_argument("--gt-isolate", action="store_true")
    parser.add_argument("--home-xyz", type=float, nargs=3, default=[20.,18.,0.])
    parser.add_argument("--base-imu-xyz", type=float, nargs=3, default=[.264,.017,.262])
    parser.add_argument("--base-imu-quaternion", type=float, nargs=4, default=[0.,0.,1.,0.])
    parser.add_argument("--obstacle-memory-radius", type=float, default=None,
                        help="Experimental override; navigation memory is off in the baseline YAML")
    args = parser.parse_args()
    if args.domain_id == 177 or not 0 <= args.domain_id <= 232:
        parser.error("use an isolated ROS domain other than live domain 177")
    frames,metadata = read_fixture(args.bag,args.gt_isolate,args.home_xyz,args.base_imu_xyz,args.base_imu_quaternion)
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init()
    node = Node("surface_fixture_replay", parameter_overrides=[Parameter("use_sim_time", value=True)])
    latest = {"map": None, "integrity": True}
    node.create_subscription(OccupancyGrid, "/replay/bev", lambda m: latest.update(map=m), 10)
    node.create_subscription(Bool, "/replay/slam/map_integrity", lambda m: latest.update(integrity=m.data), 10)
    cloud_pub = node.create_publisher(PointCloud2, "/replay/cloud", 10)
    pose_pub = node.create_publisher(PoseStamped, "/replay/pose", 10)
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    executable = Path(get_package_prefix("slam_mapping"))/"lib/slam_mapping/probabilistic_voxel_mapper_node"
    log = (args.output/"mapper.log").open("x")
    command = [str(executable), "--ros-args", "-r", "__ns:=/replay",
        "--params-file", str(args.params), "-p", "use_sim_time:=true", "-p", "input_topic:=/replay/cloud",
        "-p", "pose_topic:=/replay/pose", "-p", "costmap_output_topic:=/replay/bev",
        "-p", "publish_voxel_map:=false", "-p", "publish_after_integration:=true"]
    if args.obstacle_memory_radius is not None:
        command += ["-p", f"obstacle_memory_radius:={args.obstacle_memory_radius}"]
    child = subprocess.Popen(command, stdout=log, stderr=log)
    latencies, hashes = [], []
    try:
        until = time.monotonic()+10
        while cloud_pub.get_subscription_count() == 0 or pose_pub.get_subscription_count() == 0:
            if child.poll() is not None or time.monotonic() > until:
                raise RuntimeError("mapper startup failed")
            rclpy.spin_once(node, timeout_sec=.05)
        for pose, cloud in frames:
            if cloud.header.frame_id != "map" or pose.header.frame_id != "map":
                raise ValueError("aligned fixture must be in map frame")
            clock_pub.publish(Clock(clock=cloud.header.stamp))
            pose_pub.publish(pose)
            cloud_pub.publish(cloud)
            started = time.monotonic()
            while latest["map"] is None or ns(latest["map"]) != ns(cloud):
                if child.poll() is not None or time.monotonic()-started > 10:
                    raise RuntimeError(f"unacknowledged frame {ns(cloud)}")
                rclpy.spin_once(node, timeout_sec=.01)
            if not latest["integrity"]:
                raise RuntimeError("mapper capacity exceeded")
            latencies.append(time.monotonic()-started)
            hashes.append(hashlib.sha256(bytes(latest["map"].data)).hexdigest())
        msg = latest["map"]
        grid = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        np.savez_compressed(args.output/"final_map.npz", grid=grid, resolution=msg.info.resolution,
                            origin=[msg.info.origin.position.x, msg.info.origin.position.y])
        summary = {"frames_received": len(frames), "frames_acknowledged": len(hashes), "unacknowledged": 0,
                   **metadata, "mapper_command": command,
                   "frame_map_sha256": hashes, "final_map_sha256": hashes[-1],
                   "params_sha256": hashlib.sha256(args.params.read_bytes()).hexdigest(),
                   "mapper_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
                   "alignment_sha256": hashlib.sha256(Path(__file__).with_name("fixture_alignment.py").read_bytes()).hexdigest(),
                   "mapper_executable": str(executable.resolve()),
                   "latency_p95_sec": float(np.percentile(latencies, 95)), "domain_id": args.domain_id,
                   "note": "mapper replay only; not obstacle recall or mission validation; per-frame projection cadence"}
        (args.output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
        print(json.dumps({k:v for k,v in summary.items() if k != "frame_map_sha256"}, indent=2))
    finally:
        child.terminate()
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
