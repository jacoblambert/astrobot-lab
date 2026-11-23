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
from astrobot_eval.evaluation import FixedCoverage, mission_verdict


def test_fixed_coverage_does_not_shrink_to_reachable_island():
    grid = FixedCoverage(0, 0, 8, 2)
    for x, y in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
        grid.observe(x, y)
    assert len(grid.cells) == 52
    assert grid.fraction == 4 / 52
    grid.observe(math.nan, 0)
    grid.observe(100, 100)
    assert grid.fraction == 4 / 52


def test_verdict_failure_reasons_always_fail():
    result = mission_verdict("explore_radius", {"state": "finished", "return_reason": "slam_diverged"}, 0, .8, .8)
    assert not result["pass"] and "slam_diverged" in result["reasons"]
    failed_return = mission_verdict("explore_radius", {"state": "failed", "returning_home": True}, 5, .8, .8)
    assert "false_success" not in failed_return["reasons"]
    invalid = mission_verdict("explore_radius", {"state": "finished"}, 0, .8, .8, reference_valid=False)
    assert not invalid["pass"] and invalid["validity"] == "infrastructure_invalid"
