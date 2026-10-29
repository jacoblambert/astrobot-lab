import math

import numpy as np
from shapely import Point

from nav2_replay_check import BODY
from v2_nav_trial import GoalSampler, body, slam_error


class FakeScene:
    def __init__(self, objects, extent=20.):
        self.objects = [dict(core=c, height=h) for c, h in objects]
        self.tile_xy = np.array([[0., 0.], [extent, extent]])


def test_body_is_the_rotated_footprint():
    rect = body(1., 2., math.pi / 2)
    lo_x, lo_y, hi_x, hi_y = rect.bounds
    assert np.allclose([hi_x - lo_x, hi_y - lo_y], [2 * BODY[1], 2 * BODY[0]])
    assert np.allclose(rect.centroid.coords[0], (1., 2.))


def test_goals_stay_in_range_inside_the_margin_and_clear_of_tall_rocks():
    rocks = [(Point(x, y).buffer(.3), .25) for x in range(4, 17, 2) for y in range(4, 17, 2)]
    pebble = (Point(10, 13).buffer(.3), .05)  # below the hazard floor: may be under a goal
    sampler = GoalSampler(FakeScene(rocks + [pebble]), np.random.default_rng(1))
    for _ in range(50):
        x, y, yaw = sampler(10., 10., 3., 8.)
        assert 3. <= math.hypot(x - 10, y - 10) <= 8. and 3. <= x <= 17. and 3. <= y <= 17.
        footprint = body(x, y, yaw, .30)
        assert not any(footprint.intersects(core) for core, h in rocks)


def test_slam_error_interpolates_gt_at_slam_stamps():
    gt = [(0., 0., 0., 0.), (1., 1., 0., 0.)]
    slam = [(0.5, 0.5, 0.1, 0.), (1., 1., 0.2, 0.)]
    error = slam_error(gt, slam)
    assert error["samples"] == 2 and np.isclose(error["max_m"], .2) and np.isclose(error["p50_m"], .15)
    assert slam_error([], slam) is None


def test_gt_contacts_score_the_tum_body_against_tall_cores():
    from score_gt_contacts import contacts

    scene = FakeScene([(Point(5, 0).buffer(.2), .3), (Point(0, 5).buffer(.2), .05)])
    for o, name in zip(scene.objects, ("tall", "pebble")):
        o["name"] = name
    tum = np.array([[0, 0, 0, 0, 0, 0, 0, 1], [1, 4.4, 0, 0, 0, 0, 0, 1], [2, 0, 4.5, 0, 0, 0, 0, 1]], float)
    found = contacts(scene, tum)
    assert [(c[0], c[3], c[4]) for c in found] == [(1.0, .3, "tall")]


def test_rocks_under_the_spawn_body_are_reported_apart_from_avoidable_contacts():
    from score_gt_contacts import contacts, summarize

    scene = FakeScene([(Point(0.3, 0).buffer(.15), .12), (Point(5, 0).buffer(.2), .3)])
    for o, name in zip(scene.objects, ("spawn", "tall")):
        o["name"] = name
    tum = np.array([[0, 0, 0, 0, 0, 0, 0, 1], [1, .5, 0, 0, 0, 0, 0, 1], [2, 4.4, 0, 0, 0, 0, 0, 1]], float)
    found = contacts(scene, tum)
    spawn = {c[4] for c in contacts(scene, tum[:1])}
    result = summarize(found, len(tum), spawn)
    assert spawn == {"spawn"} and result["spawn_rock_contacts"] == 2
    assert result["contact_samples"] == 1 and result["contacts_by_height"]["ge_0.10"] == 1
    assert result["rocks_touched"] == ["tall"] and result["first_contact"][4] == "tall"
    assert summarize(found, len(tum))["contact_samples"] == 3
