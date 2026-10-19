from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation_stage_timing import StageTiming


def test_timing_preserves_calls_exceptions_and_bounded_worst_intervals():
    values = iter([0, 1, 2, 2.5, 3, 5, 6, 6.1])
    timer = StageTiming(clock=lambda: next(values), limit=2)
    assert timer.call("step", 10, lambda value, **kw: (value, kw), 4, render=True) == (4, dict(render=True))
    timer.call("step", 11, lambda: None)
    with pytest.raises(ValueError, match="original"):
        timer.call("step", 12, lambda: (_ for _ in ()).throw(ValueError("original")))
    timer.call("step", 13, lambda: None)
    stage = timer.report()["stages"]["step"]
    assert stage["calls"] == 4
    assert stage["maximum_s"] == 2
    assert stage["worst"] == [dict(stamp=12, wall_s=2), dict(stamp=10, wall_s=1)]
    assert timer.report()["nested_intervals_not_additive"]
