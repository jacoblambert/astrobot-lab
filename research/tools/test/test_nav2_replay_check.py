import numpy as np
from shapely import Point, box

from nav2_replay_check import BODY, GOAL_TOLERANCE, collisions, gt_grid, params, swept, transit


class FakeScene:
    def __init__(self, *objects):
        self.objects = [dict(core=core, height=h) for core, h in objects]


def test_transit_stops_at_goal_tolerance():
    path = np.c_[np.linspace(0, 2.4, 25), np.zeros(25)]
    kept = transit(path)
    assert np.linalg.norm(kept[-1] - path[-1]) > GOAL_TOLERANCE
    assert np.linalg.norm(path[len(kept)] - path[-1]) <= GOAL_TOLERANCE
    assert len(transit(path[:3])) == 1  # a goal inside the tolerance keeps only the start pose


def test_heading_change_is_swept_as_in_place_turn():
    # rock beside the corner arc of a quarter turn, clear of both straight footprints
    corner = np.hypot(*BODY)
    rock = Point(corner * np.cos(np.pi / 4 + .6), corner * np.sin(np.pi / 4 + .6)).buffer(.03)
    straight = box(-BODY[0], -BODY[1], BODY[0], BODY[1])
    assert not straight.intersects(rock)
    scene = FakeScene((rock, .4))
    path = np.array([[0., 0.], [0., .01]])  # rover facing +x, path leaves along +y
    tallest, _, where = collisions(scene, swept(path, 0.))
    assert tallest == .4 and where < 1


def test_collisions_report_tallest_and_large_clearance():
    scene = FakeScene((Point(3, 0).buffer(.2), .25), (Point(0, 3).buffer(.2), .05))
    path = np.c_[np.linspace(0, 1, 11), np.zeros(11)]
    tallest, clearance, where = collisions(scene, swept(path, 0.))
    assert tallest == 0 and where is None
    assert np.isclose(clearance, 3 - .2 - 1 - BODY[0], atol=1e-2)
    tallest, _, where = collisions(scene, swept(np.c_[np.linspace(0, 2.4, 25), np.zeros(25)], 0.))
    assert tallest == .25 and where > .5


def test_gt_grid_marks_only_tall_cores():
    scene = FakeScene((box(.2, .2, .4, .4), .3), (box(.6, .6, .8, .8), .05))
    grid = gt_grid(scene, (0., 0.), (10, 10), .1, .10)
    assert grid[2:4, 2:4].tolist() == [[100, 100], [100, 100]]
    assert (grid == 100).sum() == 4


def test_params_rewire_static_layer_and_frames():
    out = params(__import__("nav2_replay_check").CONFIG, "gt", ("gt_map", "gt_base_link"), "/nav2_replay/gt_grid")
    costmap = out["/gt/global_costmap/global_costmap"]["ros__parameters"]
    assert costmap["static_layer"]["map_topic"] == "/nav2_replay/gt_grid"
    assert (costmap["global_frame"], costmap["robot_base_frame"]) == ("gt_map", "gt_base_link")
    assert costmap["inflation_layer"]["inflation_radius"] == .75  # production semantics kept
    assert out["/gt/planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] is False
