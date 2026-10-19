"""Opt-in ROS/DDS integration, external archive required; NEVER moves a robot.

Source ROS + astrobot_interfaces; set ROS_DOMAIN_ID=179,
TERRAIN_SHADOW_ROS_TEST=1 and TERRAIN_LIBRARY to the frozen candidate.
"""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.mark.parametrize("fault", [None, "clock_reset", "missing_chunk"])
def test_real_ros_stream_archive_replay_and_invalid_map(fault):
    if os.environ.get("TERRAIN_SHADOW_ROS_TEST") != "1":
        pytest.skip("explicit isolated ROS integration opt-in required")
    assert os.environ.get("ROS_DOMAIN_ID") == "179"
    library = Path(os.environ["TERRAIN_LIBRARY"])
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from astrobot_interfaces.msg import SlamRevision, SlamFramePose
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from archive_guard import allocate_run
    from native_chunk_transport import message
    from replay_live_terrain_shadow import replay

    run = allocate_run("/archive")
    print(f"ROS shadow contract: {run}", flush=True)
    rclpy.init()
    node = rclpy.create_node("terrain_shadow_contract")
    revision_pub = node.create_publisher(SlamRevision, "/astrobot_0/slam_raw/revisions",
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    chunk_pub = node.create_publisher(PointCloud2, "/terrain_shadow/native_chunks", 16)
    clock_pub = node.create_publisher(Clock, "/clock", 100)
    complete_pub = node.create_publisher(String, "/terrain_shadow/native_complete", 1)
    statuses, maps = [], []
    node.create_subscription(String, "/terrain_shadow/status", lambda m: statuses.append(json.loads(m.data)), 10)
    node.create_subscription(OccupancyGrid, "/terrain_shadow/costmap", maps.append, 10)
    log = (run / "node.log").open("x")
    executable = os.environ.get("TERRAIN_SHADOW_EXECUTABLE", str(Path(__file__).parents[1] / "live_terrain_shadow.py"))
    process = subprocess.Popen([sys.executable, executable,
        "--ros-args", "-p", "use_sim_time:=true", "-p", f"library:={library}",
        "-p", "origin_x:=-10.0", "-p", "origin_y:=-10.0", "-p", "min_range:=0.5", "-p", "max_range:=100.0",
        "--disable-external-lib-logs", "--disable-rosout-logs"], stdout=log, stderr=subprocess.STDOUT)

    def spin_for(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            assert process.poll() is None, (run / "node.log").read_text()
            rclpy.spin_once(node, timeout_sec=.01)

    def clock(stamp):
        value = Clock()
        value.clock.sec, value.clock.nanosec = divmod(stamp, 10**9)
        clock_pub.publish(value)

    try:
        deadline = time.monotonic() + 15.
        while min(revision_pub.get_subscription_count(), chunk_pub.get_subscription_count(),
                  complete_pub.get_subscription_count()) < 1:
            assert time.monotonic() < deadline, (run / "node.log").read_text()
            spin_for(.02)
        clock(0)
        spin_for(.04)
        xy = np.arange(-3., 3.01, .07)
        x, y = np.meshgrid(xy, xy)
        points = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, -.5)]).astype("f4")
        depths = np.linalg.norm(points, axis=1).astype("f4")
        for index in range(6):
            rev = SlamRevision()
            rev.header.frame_id = "map"
            rev.session_id = run.name
            rev.kind = 1
            rev.revision, rev.parent_revision = index+1, index
            pose = SlamFramePose()
            pose.frame_id = index
            pose.measurement_stamp.sec, pose.measurement_stamp.nanosec = divmod(index*100_000_000, 10**9)
            pose.submap_id = -1
            pose.map_from_imu.orientation.w = pose.imu_from_lidar.orientation.w = 1.
            pose.map_from_imu.position.z = .5
            pose.map_from_imu.position.x = index * .01
            pose.imu_from_lidar.position.x = .264  # Nonzero lever arm, no GT subscription.
            rev.frames = [pose]
            revision_pub.publish(rev)
            if not (fault == "missing_chunk" and index == 4):
                chunk_pub.publish(message(run.name, index, index*100_000_000, points, depths,
                                          np.ones(len(points), dtype="u1")))
            spin_for(.04)
            clock((index+1)*100_000_000)
            spin_for(.10)
        if fault == "clock_reset":
            clock(100_000_000)
        elif fault is None:
            complete_pub.publish(String(data=json.dumps(dict(capture=run.name, sequence=5, stamp_ns=500_000_000))))
            spin_for(.03)
            clock(700_000_000)
        spin_for(.15)
        assert statuses and maps, (run / "node.log").read_text()
        if fault:
            assert statuses[-1]["failure"] and not statuses[-1]["valid"]
            assert set(maps[-1].data) == {-1}
        else:
            assert statuses[-1]["completed"] and statuses[-1]["integrated"] == 6, statuses[-1]
            assert any(0 <= value < 100 for value in maps[-1].data), "vacuous all-unknown map"
            assert maps[-1].header.stamp.nanosec == 500_000_000
        assert not node.get_publishers_info_by_topic("/astrobot_0/cmd_vel")
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    assert process.returncode == 0, (run / "node.log").read_text()
    match = re.search(r"Shadow archive: (\/archive\/revision-[0-9a-f]{32})", (run / "node.log").read_text())
    assert match
    source = Path(match[1])
    if fault:
        with pytest.raises(ValueError, match="failed/incomplete"):
            replay(source)
        result = dict(expected_fault=fault, run=str(source), invalid_map=True)
    else:
        result = replay(source)
        assert result["exact_match"] and result["chunks"] == 6 and result["projections"] == 6
    with (run / "contract.json").open("x") as stream:
        json.dump(result, stream, indent=2)
