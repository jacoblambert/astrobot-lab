import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def compose_pose(a: Pose2D, b: Pose2D) -> Pose2D:
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    return Pose2D(
        x=a.x + c * b.x - s * b.y,
        y=a.y + s * b.x + c * b.y,
        yaw=math.atan2(math.sin(a.yaw + b.yaw), math.cos(a.yaw + b.yaw)),
    )
