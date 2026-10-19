"""One-time, stationary measurement-time GT initialization; never trajectory fitting."""
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from imu_consistency import interpolate_poses, series
from revision_evidence import validated_transform


def initial_state(times_ns, base_poses, first_cloud_ns, home, base_from_imu, base_from_lidar):
    times_ns = np.asarray(times_ns, dtype=np.int64)
    poses = np.asarray(base_poses, dtype=float)
    # Require the entire preceding stationary second and a bounded first-cloud bracket.
    lo = np.searchsorted(times_ns, first_cloud_ns - 1_000_000_000, side="right") - 1
    hi = np.searchsorted(times_ns, first_cloud_ns, side="left")
    if lo < 0 or hi >= len(times_ns) or hi <= lo:
        raise ValueError("missing stationary window or first-cloud GT bracket")
    t = times_ns[lo:hi+1] / 1e9
    p = poses[lo:hi+1]
    series(t, p, max_gap=.075)
    dt = np.diff(t)
    speed = np.linalg.norm(np.diff(p[:, :3, 3], axis=0), axis=1) / dt
    rotations = Rotation.from_matrix(p[:, :3, :3])
    omega = (rotations[:-1].inv() * rotations[1:]).magnitude() / dt
    if speed.max() > .01 or omega.max() > .02:
        raise ValueError("GT initialization window is not stationary")
    world_base = interpolate_poses(t, p, [first_cloud_ns/1e9])[0]
    map_world = np.eye(4)
    home = np.asarray(home, dtype=float)
    if home.shape != (3,) or not np.isfinite(home).all():
        raise ValueError("invalid fixed home")
    map_world[:3, 3] = -home
    base_imu = validated_transform(base_from_imu)
    base_lidar = validated_transform(base_from_lidar)
    map_imu = map_world @ world_base @ base_imu
    map_lidar = map_world @ world_base @ base_lidar
    lidar_imu = np.linalg.inv(base_lidar) @ base_imu
    np.testing.assert_allclose(map_imu @ np.linalg.inv(lidar_imu), map_lidar, rtol=0, atol=1e-10)
    return dict(mode="gt-start", stamp_ns=int(first_cloud_ns),
        gt_bracket_ns=[int(times_ns[hi-1]), int(times_ns[hi])],
        stationary_samples=len(t), maximum_speed_m_s=float(speed.max()), maximum_angular_speed_rad_s=float(omega.max()),
        map_from_world=map_world.tolist(), world_from_base=world_base.tolist(), base_from_imu=base_imu.tolist(),
        base_from_lidar=base_lidar.tolist(), map_from_imu=map_imu.tolist(), map_from_lidar=map_lidar.tolist(),
        init_T_world_imu=[*map_imu[:3, 3].tolist(), *Rotation.from_matrix(map_imu[:3, :3]).as_quat().tolist()],
        init_v_world_imu=[0., 0., 0.], bias_policy="unchanged recorded estimator configuration",
        ongoing_direct_gt=False, trajectory_alignment=False)


def recorded_gt(recorder):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from geometry_msgs.msg import PoseStamped
    from export_shadow_trajectory import pose_matrix, stamp_ns

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(Path(recorder)/"bag"), storage_id="mcap"), ConverterOptions("", ""))
    reader.set_filter(StorageFilter(topics=["/gt/base_link_pose"]))
    times, poses = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, PoseStamped)
        if msg.header.frame_id != "gt_map":
            raise ValueError("unexpected simulator GT world frame")
        stamp = stamp_ns(msg.header.stamp)
        if times and stamp <= times[-1]:
            raise ValueError("duplicate or reversing GT timestamps")
        times.append(stamp)
        poses.append(pose_matrix(msg.pose))
    return np.array(times, dtype=np.int64), np.array(poses)


def verify_first_output(records, seed):
    from shadow_terrain import pose_error

    first = min((f for r in records for f in r["frames"]), key=lambda f: f["stamp_ns"])
    if first["stamp_ns"] != seed["stamp_ns"]:
        raise ValueError("GLIM first frame did not use the declared seed measurement")
    error = pose_error(np.asarray(first["map_from_imu"]), np.asarray(seed["map_from_imu"]))
    result = dict(stamp_ns=first["stamp_ns"], **error,
        rotation_deg=float(np.rad2deg(error["rotation_rad"])))
    result["passed"] = error["position_m"] <= .01 and result["rotation_deg"] <= .1
    return result
