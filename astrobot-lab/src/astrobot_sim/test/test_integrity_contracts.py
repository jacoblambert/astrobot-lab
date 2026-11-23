# Copyright 2026 Jacob Lambert
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from astrobot_sim.integrity import PoseSample, PoseValidator, rotate_vector


def pose(t, x=0, q=(0, 0, 0, 1)):
    return PoseSample(t, (x, 0, 0), q)


def test_origin_is_valid_but_teleport_is_not():
    validator = PoseValidator()
    assert validator.validate(pose(1), 1) == ""
    assert validator.validate(pose(1.1, .01), 1.1) == ""
    assert validator.validate(pose(1.2, 20), 1.2) == "kinematic_position"
    assert validator.validate(pose(1.3, .02), 1.3) == ""


def test_bad_measurements_do_not_advance_validator():
    validator = PoseValidator()
    assert validator.validate(pose(1), 1) == ""
    for sample, now, reason in [(pose(1), 1, "non_increasing_stamp"),
                                (pose(2, math.nan), 2, "non_finite"),
                                (pose(2, q=(0, 0, 0, 0)), 2, "quaternion_norm"),
                                (pose(2), 3, "stale"), (pose(4), 2, "future_stamp")]:
        assert validator.validate(sample, now) == reason
        assert validator.previous.stamp == 1
    validator.reset()
    assert validator.validate(pose(.1, 30), .1) == ""


def test_pitch_rotates_vertical_lever_arm():
    q = (0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4))
    rotated = rotate_vector(q, (0, 0, 1))
    assert abs(rotated[0] - 1) < 1e-12
    assert abs(rotated[2]) < 1e-12
