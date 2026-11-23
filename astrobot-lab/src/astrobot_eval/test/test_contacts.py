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

import numpy as np

from astrobot_eval.contacts import BODY, body, contacts, ellipse, erode, intersects, summarize


class FakeScene:
    def __init__(self, objects):
        self.objects = [dict(core=c, height=h, name=n) for c, h, n in objects]


def circle(x, y, r):
    return ellipse(x, y, r, r)


def test_body_is_the_rotated_footprint():
    rect = body(1.0, 2.0, math.pi / 2)
    assert np.allclose(sorted(rect[:, 0]), [1 - BODY[1]] * 2 + [1 + BODY[1]] * 2)
    assert np.allclose(sorted(rect[:, 1]), [2 - BODY[0]] * 2 + [2 + BODY[0]] * 2)


def test_separating_axis_intersection():
    rect = body(0, 0, 0)
    assert intersects(rect, circle(0.6, 0, 0.2))  # overlaps the front edge
    assert not intersects(rect, circle(0.8, 0, 0.2))
    assert not intersects(body(0, 0, math.pi / 4), circle(0.62, 0.62, 0.1))  # beyond the rotated corner


def test_erosion_shrinks_and_keeps_thin_footprints():
    core = erode(circle(0, 0, 0.2), 0.05)
    assert abs(np.linalg.norm(core, axis=1).max() - 0.15) < 2e-3
    thin = ellipse(0, 0, 0.3, 0.03)
    assert erode(thin, 0.05) is not None and np.allclose(erode(thin, 0.05), thin)


def test_gt_contacts_score_the_tum_body_against_tall_cores():
    scene = FakeScene([(circle(5, 0, .2), .3, "tall"), (circle(0, 5, .2), .05, "pebble")])
    tum = np.array([[0, 0, 0, 0, 0, 0, 0, 1], [1, 4.4, 0, 0, 0, 0, 0, 1], [2, 0, 4.5, 0, 0, 0, 0, 1]], float)
    found = contacts(scene, tum)
    assert [(c[0], c[3], c[4]) for c in found] == [(1.0, .3, "tall")]


def test_rocks_under_the_spawn_body_are_reported_apart_from_avoidable_contacts():
    scene = FakeScene([(circle(0.3, 0, .15), .12, "spawn"), (circle(5, 0, .2), .3, "tall")])
    tum = np.array([[0, 0, 0, 0, 0, 0, 0, 1], [1, .5, 0, 0, 0, 0, 0, 1], [2, 4.4, 0, 0, 0, 0, 0, 1]], float)
    found = contacts(scene, tum)
    spawn = {c[4] for c in contacts(scene, tum[:1])}
    result = summarize(found, len(tum), spawn)
    assert spawn == {"spawn"} and result["spawn_rock_contacts"] == 2
    assert result["contact_samples"] == 1 and result["contacts_by_height"]["ge_0.10"] == 1
    assert result["rocks_touched"] == ["tall"] and result["first_contact"][4] == "tall"
    assert summarize(found, len(tum))["contact_samples"] == 3
