#!/usr/bin/env python3
"""Candidate-ranking parity: Python exploration_manager._find_candidates vs C++ astrobot_exploration.

Random exploration maps (rocks, unknown regions, a visited trail, resource samples) are ranked by
both implementations for explore_radius and sample_return; candidates, scores and coverage stats
are compared. Run inside the dev container (ROS sourced, astrobot_exploration built):

  exploration_parity.py <candidates_tool> [cases]
"""

import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np

REPO = Path(__file__).resolve().parents[2]
# The Python reference implementation (kept on dev under research/legacy).
sys.path[:0] = [str(REPO / "research/legacy/exploration_manager"), str(REPO / "research/legacy/astrobot_core")]
from exploration_manager.exploration_manager_node import ExplorationManager, Pose2D

# Effective v2 gate values (node defaults + launch defaults + per-mode gate arguments).
COMMON = dict(free_cost_max=30, lethal_cost_min=80, goal_clearance_cost_min=50, goal_clearance_cells=3,
              goal_clearance_radius_m=0.75, min_frontier_cells=4, max_frontiers=80, max_goal_distance_m=5.0,
              min_goal_distance_m=0.75, info_weight=1.0, distance_weight=0.8, over_distance_weight=2.0,
              terrain_weight=0.025, home_penalty_weight=0.5, mission_coverage_min_known_fraction=0.3,
              mission_candidate_weight=3.0, mission_visit_margin_m=0.15, reachable_seed_distance_m=1.0,
              resource_weight_scale=5.0, resource_influence_radius_m=4.0, resource_decay_radius_m=1.75,
              resource_gradient_weight=10.0, resource_gradient_radius_m=6.0, resource_gradient_commit_weight=6.0,
              max_resource_candidates=20, resource_explore_radius_m=3.0, resource_min_explore_step_m=0.75,
              blacklist_radius_m=0.75, recent_goal_radius_m=0.75)
MODES = {
    "explore_radius": dict(home_radius_m=8.0, mission_cell_size_m=2.0, mission_candidate_limit=120),
    "sample_return": dict(home_radius_m=19.0, mission_cell_size_m=1.0, mission_candidate_limit=80,
                          resource_weight_scale=25.0, info_weight=0.3),
}


def fixture(rng, mode):
    w = h = 300
    res, origin = 0.1, (-15.0, -15.0)
    grid = np.full((h, w), -1, np.int16)
    # Known disc around the rover's past with a gradient of costs and rocks.
    yy, xx = np.mgrid[0:h, 0:w]
    wx, wy = origin[0] + (xx + 0.5) * res, origin[1] + (yy + 0.5) * res
    rx, ry = rng.uniform(-3, 3, 2)
    known = (wx - rx) ** 2 + (wy - ry) ** 2 < rng.uniform(4, 8) ** 2
    grid[known] = rng.integers(0, 25, known.sum())
    for _ in range(rng.integers(5, 20)):
        cx, cy, r = rng.uniform(-8, 8), rng.uniform(-8, 8), rng.uniform(0.2, 0.6)
        rock = known & ((wx - cx) ** 2 + (wy - cy) ** 2 < r * r)
        grid[rock] = 100
        halo = known & ~rock & ((wx - cx) ** 2 + (wy - cy) ** 2 < (r + 0.4) ** 2)
        grid[halo] = np.maximum(grid[halo], rng.integers(40, 80))
    for _ in range(rng.integers(0, 4)):  # unknown holes inside the known area
        cx, cy, r = rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(0.3, 1.5)
        grid[(wx - cx) ** 2 + (wy - cy) ** 2 < r * r] = -1
    home = (0.0, 0.0, 0.0)
    # Trail of poses (first is home), staying on free ground near the rover.
    poses, x, y = [home], 0.0, 0.0
    for _ in range(rng.integers(1, 60)):
        x, y = x + rng.normal(0, 0.4), y + rng.normal(0, 0.4)
        poses.append((x, y, rng.uniform(-math.pi, math.pi)))
    cells = []
    if mode == "sample_return":
        for _ in range(rng.integers(0, 40)):
            row, col = rng.integers(100, 200, 2)
            cells.append([int(row), int(col), float(rng.uniform(0, 0.7))])
    params = dict(COMMON, **MODES[mode])
    return dict(mode=mode, now=100.0, params=params,
                map=dict(width=w, height=h, resolution=res, origin=list(origin), data=grid.ravel().tolist()),
                poses=[list(p) for p in poses], resource_cells=cells, blacklist=[])


def python_candidates(f):
    node = ExplorationManager.__new__(ExplorationManager)
    for key, value in f["params"].items():
        setattr(node, key, value)
    node.planner_precheck_high_cost = 30
    node.mission_mode = f["mode"]
    node.frame_id = "map"
    node.use_resource_candidates_in_explore_radius = False
    node.resource_weights = {"water": 1.0}
    node.blacklist, node.stall_blacklist, node.precheck_rejects, node.recent_goals = [], [], [], []
    node.blacklist_timeout_sec, node.stall_blacklist_timeout_sec, node.precheck_reject_timeout_sec = 300.0, 180.0, 60.0
    node.recent_goal_timeout_sec, node.stall_blacklist_radius_m, node.precheck_reject_radius_m = 240.0, 0.85, 0.35
    node.visited_mission_cells = set()
    node._component_cache = None
    node.latest_mission_max_reachable_cells = node.latest_reachable_cells = 0
    node.latest_mission_total_cells = node.latest_mission_reachable_cells = node.latest_mission_covered_cells = 0
    node.latest_mission_observed_cells = 0
    node.latest_mission_coverage_fraction = 0.0
    node.latest_mission_grid = None
    node.mission_grid_pub = Mock()
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: __import__("builtin_interfaces.msg").msg.Time()))
    node._now = lambda: f["now"]
    m = f["map"]
    node.latest_map = NS(info=NS(resolution=m["resolution"], width=m["width"], height=m["height"],
                                 origin=NS(position=NS(x=m["origin"][0], y=m["origin"][1]))))
    node.grid = np.asarray(m["data"], np.int16).reshape(m["height"], m["width"])
    node.home_pose = None
    for x, y, yaw in f["poses"]:
        pose = Pose2D(x, y, yaw)
        if node.home_pose is None:
            node.home_pose = pose
        node.latest_pose = pose
        node._mark_mission_cell_visited(pose)
    grid = np.full(node.grid.shape, np.nan, np.float32)
    for row, col, value in f["resource_cells"]:
        grid[row, col] = value
    node.resource_maps = {"water": grid}
    candidates = node._find_candidates()
    stats = dict(total=node.latest_mission_total_cells, reachable=node.latest_mission_reachable_cells,
                 covered=node.latest_mission_covered_cells, observed=node.latest_mission_observed_cells,
                 coverage=node.latest_mission_coverage_fraction, reachable_cells=node.latest_reachable_cells)
    return [[c.x, c.y, c.info_gain, c.resource_score, c.terrain_cost, c.distance, c.total_score, c.cells]
            for c in candidates], stats


def main(tool, cases=40):
    rng = np.random.default_rng(11)
    summary = dict(cases=0, identical=0, same_best=0, stats_equal=0, differences=[])
    with tempfile.TemporaryDirectory() as tmp:
        for case in range(int(cases)):
            mode = ("explore_radius", "sample_return")[case % 2]
            f = fixture(rng, mode)
            path = Path(tmp) / "fixture.json"
            path.write_text(json.dumps(f))
            cpp = json.loads(subprocess.run([tool, str(path)], capture_output=True, text=True, check=True).stdout)
            py, py_stats = python_candidates(f)
            cc = cpp["candidates"]
            summary["cases"] += 1
            same = len(py) == len(cc) and all(np.allclose(a, b, rtol=0, atol=1e-9) for a, b in zip(py, cc))
            summary["identical"] += same
            summary["same_best"] += bool(py and cc and np.allclose(py[0][:2], cc[0][:2])) or (not py and not cc)
            stats_equal = all(abs(py_stats[k] - cpp["stats"][k]) < 1e-12 for k in py_stats)
            summary["stats_equal"] += stats_equal
            if not same or not stats_equal:
                summary["differences"].append(dict(case=case, mode=mode, python=len(py), cpp=len(cc),
                                                   python_best=py[0][:2] if py else None,
                                                   cpp_best=cc[0][:2] if cc else None,
                                                   python_stats=py_stats, cpp_stats=cpp["stats"]))
    print(json.dumps(summary, indent=1))
    return 0 if summary["identical"] == summary["cases"] and summary["stats_equal"] == summary["cases"] else 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
