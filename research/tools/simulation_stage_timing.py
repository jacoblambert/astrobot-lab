"""Bounded passive wall-time attribution; never alters simulator/guard policy."""

import time


class StageTiming:
    def __init__(self, clock=time.monotonic, limit=12):
        self.clock, self.limit = clock, limit
        self.stages = {}

    def call(self, name, stamp, function, *args, **kwargs):
        began = self.clock()
        try:
            return function(*args, **kwargs)
        finally:
            elapsed = self.clock() - began
            stage = self.stages.setdefault(name, dict(calls=0, total_s=0.0, maximum_s=0.0, worst=[]))
            stage["calls"] += 1
            stage["total_s"] += elapsed
            stage["maximum_s"] = max(stage["maximum_s"], elapsed)
            if len(stage["worst"]) < self.limit or elapsed > stage["worst"][-1]["wall_s"]:
                stage["worst"].append(dict(stamp=stamp, wall_s=elapsed))
                stage["worst"].sort(key=lambda v: v["wall_s"], reverse=True)
                del stage["worst"][self.limit :]

    def report(self):
        return dict(stages=self.stages, nested_intervals_not_additive=True, diagnostic_only=True)
