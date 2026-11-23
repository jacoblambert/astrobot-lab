"""Measurement-time, full-SE(3) re-anchoring for diagnostic mapper replay only."""
import bisect
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from astrobot_core.integrity import PoseSample, PoseValidator


def validate_fixture_epochs(epochs):
    """A replay without reset events must not combine distinct map epochs."""
    epochs = sorted(set(epochs))
    if len(epochs) > 1:
        raise ValueError("fixture spans localization epochs; split it at reset barriers")
    return epochs


def pose_matrix(position, quaternion):
    position, quaternion = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    if (position.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(position).all()
            or not np.isfinite(quaternion).all() or abs(quaternion@quaternion-1) > .02):
        raise ValueError("invalid SE(3) pose")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = position
    return matrix


class ReferenceTrajectory:
    def __init__(self, rows, max_gap=.2):
        """Rows: (measurement seconds, xyz, quaternion xyzw). No extrapolation."""
        if not np.isfinite(max_gap) or max_gap <= 0:
            raise ValueError("invalid GT interpolation gap limit")
        self.rows = sorted(rows, key=lambda row: row[0])
        if len(self.rows) < 2:
            raise ValueError("at least two valid GT measurements required")
        validator = PoseValidator()
        for ts, position, quaternion in self.rows:
            reason = validator.validate(PoseSample(ts, tuple(position), tuple(quaternion)), ts)
            if reason:
                raise ValueError(f"invalid GT reference at {ts}: {reason}")
        self.times = [row[0] for row in self.rows]
        self.max_gap = max_gap

    def at(self, stamp):
        if not np.isfinite(stamp):
            raise ValueError("invalid query timestamp")
        index = bisect.bisect_left(self.times, stamp)
        if index < len(self.times) and self.times[index] == stamp:
            return pose_matrix(*self.rows[index][1:])
        if index == 0 or index == len(self.times):
            raise ValueError("GT reference extrapolation is forbidden")
        before, after = self.rows[index-1], self.rows[index]
        if after[0]-before[0] > self.max_gap:
            raise ValueError("GT interpolation gap exceeds limit")
        fraction = (stamp-before[0])/(after[0]-before[0])
        position = (1-fraction)*np.asarray(before[1])+fraction*np.asarray(after[1])
        rotation = Slerp([before[0], after[0]], Rotation.from_quat([before[2], after[2]]))([stamp])[0]
        return pose_matrix(position, rotation.as_quat())


def reanchor_scan(points, map_imu, map_lidar, gt_base, home_xyz=(20.,18.,0.),
                  base_imu_xyz=(.264,.017,.262), base_imu_quaternion=(0.,0.,1.,0.)):
    """Transform BOTH cloud and ray origin into fixed GT-minus-home coordinates.

    No fitting to the accumulated map; each delta uses the same measurement time
    as the scan. The lever arm and roll/pitch are retained in the composition.
    """
    home_xyz = np.asarray(home_xyz, dtype=float)
    if home_xyz.shape != (3,) or not np.isfinite(home_xyz).all():
        raise ValueError("invalid fixed home translation")
    anchored_base = gt_base.copy()
    anchored_base[:3, 3] -= home_xyz
    true_imu = anchored_base @ pose_matrix(base_imu_xyz, base_imu_quaternion)
    delta = true_imu @ np.linalg.inv(map_imu)
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    corrected = np.full(points.shape, np.nan)
    valid = np.isfinite(points).all(axis=1)
    corrected[valid] = points[valid] @ delta[:3, :3].T + delta[:3, 3]
    return corrected, delta @ map_lidar
