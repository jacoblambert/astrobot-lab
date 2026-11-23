"""Deterministic recovery budgets and stationary orientation initialization."""
import math


class RecoveryBudget:
    def __init__(self, limit=2):
        self.limit = limit
        self.attempts = 0

    def reset(self):
        self.attempts = 0

    def consume(self):
        if self.attempts >= self.limit:
            return False
        self.attempts += 1
        return True


def orientation_from_gravity(acceleration, yaw, gravity=1.62):
    ax, ay, az = acceleration
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if not all(math.isfinite(v) for v in (*acceleration, yaw)) or not 0.5*gravity <= norm <= 1.5*gravity:
        raise ValueError("stationary lunar gravity observation required")
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
