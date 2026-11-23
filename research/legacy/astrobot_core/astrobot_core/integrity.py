"""Measurement-time validation. Origin coordinates are not an invalidity sentinel."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    position: tuple
    quaternion: tuple  # xyzw

    @property
    def yaw(self):
        x, y, z, w = self.quaternion
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rotate_vector(q, vector):
    """Apply an xyzw unit quaternion without dropping roll/pitch."""
    x, y, z, w = q
    vx, vy, vz = vector
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


class PoseValidator:
    def __init__(self, max_age=0.5, max_speed=2.0, max_angular_speed=4.0, position_slack=0.05):
        self.max_age = max_age
        self.max_speed = max_speed
        self.max_angular_speed = max_angular_speed
        self.position_slack = position_slack
        self.previous = None

    def reset(self):
        self.previous = None

    def validate(self, sample, now):
        values = (sample.stamp, now, *sample.position, *sample.quaternion)
        if not all(math.isfinite(v) for v in values):
            return "non_finite"
        norm = sum(v * v for v in sample.quaternion)
        if abs(norm - 1.0) > 0.02:
            return "quaternion_norm"
        age = now - sample.stamp
        if age < -0.05:
            return "future_stamp"
        if age > self.max_age:
            return "stale"
        if self.previous is not None:
            dt = sample.stamp - self.previous.stamp
            if dt <= 0:
                return "non_increasing_stamp"
            displacement = math.sqrt(sum((a - b) ** 2 for a, b in zip(sample.position, self.previous.position)))
            if displacement > self.position_slack + self.max_speed * dt:
                return "kinematic_position"
            dot = abs(sum(a * b for a, b in zip(sample.quaternion, self.previous.quaternion)))
            angle = 2 * math.acos(min(1.0, dot / math.sqrt(norm * sum(v*v for v in self.previous.quaternion))))
            if angle > 0.05 + self.max_angular_speed * dt:
                return "kinematic_rotation"
        self.previous = sample
        return ""
