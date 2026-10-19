"""Small action/terminal-state invariants shared by mission code and fixtures."""
import math


class ActionGeneration:
    def __init__(self):
        self.value = 0

    def advance(self):
        self.value += 1
        return self.value

    def current(self, value):
        return self.value == value


def home_reached(pose, home, tolerance):
    if pose is None or home is None:
        return False
    distance = math.hypot(pose.x - home.x, pose.y - home.y)
    return math.isfinite(distance) and distance <= tolerance
