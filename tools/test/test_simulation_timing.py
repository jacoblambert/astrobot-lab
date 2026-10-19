import importlib.util
from pathlib import Path

import pytest

path = Path(__file__).resolve().parents[2] / "OmniLRS/src/configurations/simulation_timing.py"
spec = importlib.util.spec_from_file_location("simulation_timing", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_world_and_physics_context_use_the_same_actual_timestep():
    timing = module.simulation_timing(.0333, .0333, .016666)
    assert timing["physics_dt"] == 1 / 60 and timing["conflicting_environment_dt"]
    assert timing["rendering_dt"] == 1 / 30 and timing["physics_substeps"] == 2
    assert int(timing["rendering_dt"] / timing["physics_dt"]) == 2
    assert not module.simulation_timing(.016666, .0333, .016666)["conflicting_environment_dt"]


@pytest.mark.parametrize("values", [(0., .03, .01), (.03, float("nan"), .01), (.03, .03, -1.),
                                    (.03, .01, .03), (.03, .03, None), (.03, .025, .016666)])
def test_invalid_clock_contract_fails_before_simulator_start(values):
    with pytest.raises(ValueError):
        module.simulation_timing(*values)
