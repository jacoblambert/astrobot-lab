import math
import pytest
from astrobot_core.recovery import RecoveryBudget, orientation_from_gravity
from astrobot_core.integrity import rotate_vector
from astrobot_core.mission import ActionGeneration, home_reached
from types import SimpleNamespace as NS


def test_restart_budget_is_bounded():
    budget = RecoveryBudget(2)
    assert budget.consume() and budget.consume()
    assert not budget.consume()
    budget.reset()
    assert budget.consume()


def test_initialization_preserves_heading_and_gravity():
    pitch = .3
    acc = (-1.62*math.sin(pitch), 0, 1.62*math.cos(pitch))
    q = orientation_from_gravity(acc, 1.2)
    world = rotate_vector(q, acc)
    assert world == pytest.approx((0, 0, 1.62), abs=1e-10)
    with pytest.raises(ValueError):
        orientation_from_gravity((0,0,9.81),0)


def test_action_identity_and_home_validity():
    generation = ActionGeneration()
    old = generation.advance()
    generation.advance()
    assert not generation.current(old)
    assert not home_reached(NS(x=math.nan,y=0),NS(x=0,y=0),1)
    assert not home_reached(NS(x=1.5,y=0),NS(x=0,y=0),1)
