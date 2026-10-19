import math
import time
from geometry_msgs.msg import Twist
from basic_control.command_mux import CommandMux
from astrobot_interfaces.msg import LocalizationStatus


def mux():
    node = CommandMux.__new__(CommandMux)
    node.estop = False
    node.mode = "auto"
    node.require_localization = True
    node._localization_allowed = True
    node._localization_stamp = time.monotonic()
    node._nav_stamp = time.monotonic()
    node._teleop_stamp = None
    node._nav_cmd = Twist()
    node._nav_cmd.linear.x = .1
    node._teleop_cmd = None
    node.nav_timeout_sec = node.teleop_timeout_sec = .5
    return node


def test_localization_loss_inhibits_nav_and_teleop():
    node = mux()
    assert node._select_command()[0].linear.x == .1
    node._localization_stamp = time.monotonic() - 1
    assert node._select_command()[0].linear.x == 0
    node._localization_stamp = time.monotonic()
    node._localization_allowed = False
    assert node._select_command()[0].linear.x == 0


def test_nonfinite_command_is_zero():
    node = mux()
    for value in (math.nan, math.inf, -math.inf):
        command = Twist()
        command.linear.x = value
        assert node._limit_twist(command).linear.x == 0


def test_reenable_requires_a_new_command():
    node = mux()
    node._localization_allowed = False
    node._localization_callback(LocalizationStatus(motion_permitted=True))
    assert node._select_command()[0].linear.x == 0
    assert node._nav_cmd is None


def test_reenable_after_heartbeat_gap_discards_cached_commands():
    node = mux()
    node._localization_stamp = time.monotonic() - 1
    node._localization_callback(LocalizationStatus(motion_permitted=True))
    assert node._nav_cmd is None
