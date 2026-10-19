"""GT-only evaluator for recorded IMU kinematics; never estimator input.

No fitted axis, bias, scale or time offset. Residuals are diagnostics, NOT sensor
acceptance thresholds: finite cadence, filtering and interpolation contribute.
"""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from revision_evidence import validated_transform


def series(times, values, *, max_gap=.075):
    times, values = np.asarray(times, dtype=float), np.asarray(values, dtype=float)
    if (times.ndim != 1 or len(times) < 2 or len(values) != len(times)
            or not np.isfinite(times).all() or not np.isfinite(values).all()
            or times[0] < 0 or not np.isfinite(max_gap) or max_gap <= 0
            or np.any(np.diff(times) <= 0) or np.any(np.diff(times) > max_gap)):
        raise ValueError("invalid, duplicate, regressing or gapped measurement series")
    return times, values


def interpolate_poses(times, poses, queries, *, max_gap=.075):
    times, poses = series(times, poses, max_gap=max_gap)
    if poses.shape != (len(times), 4, 4):
        raise ValueError("expected SE(3) pose series")
    for pose in poses:
        validated_transform(pose)
    queries = np.asarray(queries, dtype=float)
    if (queries.ndim != 1 or not len(queries) or not np.isfinite(queries).all()
            or queries.min() < times[0] or queries.max() > times[-1]):
        raise ValueError("pose extrapolation or invalid queries")
    result = np.tile(np.eye(4), (len(queries), 1, 1))
    result[:, :3, :3] = Slerp(times, Rotation.from_matrix(poses[:, :3, :3]))(queries).as_matrix()
    for axis in range(3):
        result[:, axis, 3] = np.interp(queries, times, poses[:, axis, 3])
    return result


def triangular_acceleration(times, world_acceleration, center, half_window):
    """Exact integral of piecewise-linear acceleration times triangle weight.

    Dividing by h**2 gives the acceleration counterpart of
    (p(t-h)-2p(t)+p(t+h))/h**2. No noisy second derivative at one physics tick.
    """
    if (not np.isfinite([center, half_window]).all() or half_window <= 0
            or center-half_window < times[0] or center+half_window > times[-1]):
        raise ValueError("acceleration window requires bounded measurements")
    knots = np.unique(np.r_[center-half_window,
        times[(times > center-half_window) & (times < center+half_window)], center, center+half_window])
    a = np.column_stack([np.interp(knots, times, world_acceleration[:, axis]) for axis in range(3)])
    weight = np.maximum(0., half_window-abs(knots-center))[:, None]
    integral = np.sum(np.diff(knots)[:, None] / 6. *
        (2*a[:-1]*weight[:-1] + a[:-1]*weight[1:] + a[1:]*weight[:-1] + 2*a[1:]*weight[1:]), axis=0)
    return integral / half_window**2


def consistency(gt_times, world_from_imu, imu_times, acceleration, gyro, *, gravity=1.62, half_window=.2):
    gt_times, world_from_imu = series(gt_times, world_from_imu)
    imu_times, acceleration = series(imu_times, acceleration)
    _, gyro = series(imu_times, gyro)
    if acceleration.shape != (len(imu_times), 3) or gyro.shape != acceleration.shape:
        raise ValueError("expected IMU XYZ vectors")
    if not np.isfinite([gravity, half_window]).all() or gravity <= 0 or half_window <= 0:
        raise ValueError("invalid gravity or interval")
    pose = interpolate_poses(gt_times, world_from_imu, imu_times)
    rotation = Rotation.from_matrix(pose[:, :3, :3])
    dt = np.diff(imu_times)
    predicted = Rotation.from_rotvec((gyro[:-1] + gyro[1:]) * .5 * dt[:, None])
    actual = rotation[:-1].inv() * rotation[1:]
    gyro_error = (predicted.inv() * actual).as_rotvec() / dt[:, None]
    world_acc = rotation.apply(acceleration) + [0., 0., -gravity]
    centers = imu_times[(imu_times >= imu_times[0]+half_window) &
                        (imu_times <= imu_times[-1]-half_window)]
    if not len(centers):
        raise ValueError("insufficient bounded acceleration windows")
    queries = (centers[:, None] + [-half_window, 0., half_window]).ravel()
    positions = interpolate_poses(gt_times, world_from_imu, queries)[:, :3, 3].reshape(-1, 3, 3)
    truth = (positions[:, 0] - 2*positions[:, 1] + positions[:, 2]) / half_window**2
    estimated = np.array([triangular_acceleration(imu_times, world_acc, t, half_window) for t in centers])
    return dict(gyro_times=imu_times[1:], gyro_error_rad_s=gyro_error, centers=centers,
                acceleration_error_m_s2=estimated-truth, acceleration_predicted_m_s2=estimated,
                acceleration_reference_m_s2=truth)


def residual_summary(values):
    values = np.asarray(values, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or not len(values) or not np.isfinite(values).all():
        raise ValueError("expected finite nonempty residual vectors")
    return dict(samples=len(values), mean_xyz=np.mean(values, axis=0).tolist(),
                rms_xyz=np.sqrt(np.mean(values**2, axis=0)).tolist(),
                norm_p50_p95_max=np.percentile(np.linalg.norm(values, axis=1), [50, 95, 100]).tolist())


def orientation_clock_hypotheses(gt_times, gt_base, base_from_imu, imu_times, quaternion,
                                 *, physics_dt, rendering_dt):
    """Three code-derived clock hypotheses, all reported; never select a winner.

    The imported graph reads physics sensor data on a playback tick but stamps
    it with current simulation time. These checks can expose that mismatch;
    they cannot substitute for recording the actual sensor measurement clock.
    """
    if not np.isfinite([physics_dt, rendering_dt]).all() or not 0 < physics_dt <= rendering_dt:
        raise ValueError("invalid simulator clock periods")
    base_from_imu = validated_transform(base_from_imu)
    times, quaternion = series(imu_times, quaternion)
    if quaternion.shape != (len(times), 4) or np.any(abs(np.sum(quaternion**2, axis=1)-1) > 1e-6):
        raise ValueError("invalid IMU orientation")
    measured = Rotation.from_quat(quaternion)
    result = {}
    for name, delay in (("header_time", 0.), ("previous_physics_tick", physics_dt),
                        ("previous_render_tick", rendering_dt)):
        poses = interpolate_poses(gt_times, gt_base, times-delay) @ base_from_imu
        error = (Rotation.from_matrix(poses[:, :3, :3]).inv() * measured).as_rotvec()
        result[name] = dict(reference_delay_s=delay, error_rad=residual_summary(error))
    return result


def paired_consistency(gt_times, gt_base, base_from_imu, streams, *, gravity=1.62, half_window=.2):
    """Compare 30/60 Hz inputs on identical legacy endpoints and GT windows.

    Every measured sample is retained for integration. No rate conversion, fitted
    delay, orientation-message input or error-dependent window selection. Gyro
    uses products of trapezoidal body-rate increments, not a sum of world rates.
    """
    if set(streams) != {"legacy", "physics"}:
        raise ValueError("both declared IMU streams are required")
    if not np.isfinite([gravity, half_window]).all() or min(gravity, half_window) <= 0:
        raise ValueError("invalid physical constants")
    base_from_imu = validated_transform(base_from_imu)
    validated = {}
    for name, hz in (("legacy", 30.), ("physics", 60.)):
        times, acceleration, gyro = streams[name]
        times, acceleration = series(times, acceleration)
        _, gyro = series(times, gyro)
        if acceleration.shape != (len(times), 3) or gyro.shape != acceleration.shape:
            raise ValueError("expected IMU XYZ vectors")
        if np.max(abs(np.diff(times)-1/hz)) > 1e-5:
            raise ValueError(f"{name} IMU cadence failure; cannot drop or interpolate missing ticks")
        validated[name] = (times, acceleration, gyro)
    lo = max(v[0][0] for v in validated.values())
    hi = min(v[0][-1] for v in validated.values())
    legacy_times = validated["legacy"][0]
    centers = legacy_times[(legacy_times >= lo+half_window) & (legacy_times <= hi-half_window)]
    endpoints = legacy_times[(legacy_times >= lo) & (legacy_times <= hi)]
    if len(centers) == 0 or len(endpoints) < 2:
        raise ValueError("insufficient common bounded IMU windows")
    queries = (centers[:, None] + [-half_window, 0., half_window]).ravel()
    positions = (interpolate_poses(gt_times, gt_base, queries) @ base_from_imu)[:, :3, 3].reshape(-1, 3, 3)
    acceleration_reference = (positions[:, 0]-2*positions[:, 1]+positions[:, 2])/half_window**2
    reference_rot = Rotation.from_matrix((interpolate_poses(gt_times, gt_base, endpoints) @ base_from_imu)[:, :3, :3])
    reference_delta = reference_rot[:-1].inv()*reference_rot[1:]
    result = dict(centers=centers, gyro_endpoints=endpoints,
                  acceleration_reference_m_s2=acceleration_reference)
    for name, (times, acceleration, gyro) in validated.items():
        poses = interpolate_poses(gt_times, gt_base, times) @ base_from_imu
        world_acc = Rotation.from_matrix(poses[:, :3, :3]).apply(acceleration) + [0., 0., -gravity]
        estimated = np.array([triangular_acceleration(times, world_acc, t, half_window) for t in centers])
        deltas = []
        for start, end in zip(endpoints[:-1], endpoints[1:]):
            knots = np.r_[start, times[(times > start) & (times < end)], end]
            rates = np.column_stack([np.interp(knots, times, gyro[:, axis]) for axis in range(3)])
            delta = Rotation.identity()
            for vector in (rates[:-1]+rates[1:])*.5*np.diff(knots)[:, None]:
                delta = delta*Rotation.from_rotvec(vector)
            deltas.append(delta.as_quat())
        error = (Rotation.from_quat(deltas).inv()*reference_delta).as_rotvec()/np.diff(endpoints)[:, None]
        result[name+"_acceleration_error_m_s2"] = estimated-acceleration_reference
        result[name+"_gyro_error_rad_s"] = error
    return result


def coincident_samples(legacy_ns, physics_ns):
    """Match only ROS nanosecond quantization, never a fitted sensor-time offset.

    Isaac's graph conversion and Python round() can differ by one nanosecond.
    Preserve signed differences; these indices are diagnostic only and MUST NOT
    retimestamp either stream or alter the kinematic integration windows.
    """
    legacy, physics = np.asarray(legacy_ns), np.asarray(physics_ns)
    for stamps in (legacy, physics):
        if (stamps.ndim != 1 or not len(stamps) or stamps.dtype.kind not in "iu"
                or np.any(stamps < 0) or np.any(np.diff(stamps.astype(np.int64)) <= 0)):
            raise ValueError("expected ordered integer nanosecond stamps")
    right = np.searchsorted(physics, legacy)
    candidates = np.column_stack([np.clip(right-1, 0, len(physics)-1), np.clip(right, 0, len(physics)-1)])
    delta = physics[candidates].astype(np.int64)-legacy[:, None].astype(np.int64)
    ids = candidates[np.arange(len(legacy)), np.argmin(abs(delta), axis=1)]
    differences = physics[ids].astype(np.int64)-legacy.astype(np.int64)
    if len(np.unique(ids)) != len(ids) or np.any(abs(differences) > 1):
        raise ValueError("streams do not coincide within fixed one-nanosecond serialization bound")
    return ids, differences
