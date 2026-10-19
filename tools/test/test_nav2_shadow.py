"""Requires standalone NAV2_SHADOW_PROBE; wall cases also TERRAIN_LIBRARY.

Optional REJECTED_TERRAIN_LIBRARY runs the frozen rejected wall candidate as a
diagnostic, not permission to promote it. No simulator or motion publishers.
"""
import os
import hashlib
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav2_shadow_probe import probe, rectangle_intersects_segment


@pytest.fixture
def executable():
    value = os.environ.get("NAV2_SHADOW_PROBE")
    if not value:
        pytest.skip("requires installed-Nav2 standalone probe")
    return value


def test_real_inflation_gradient_unknown_and_footprint(executable):
    costs = np.zeros((40, 40), dtype=int)
    costs[20, 20] = 100
    costs[5, 5] = -1
    result = probe(executable, costs, [[.05, .05, 0], [1.05, .05, 0], [-1.45, -1.45, 0]])
    values = result["inflated"][20]
    assert values[20] == 254
    assert values[21] == 253
    assert 0 < values[26] < values[25] < values[24] <= 253
    assert values[30] == 0
    assert result["inflated"][5, 5] == 255
    assert [r["accepted"] for r in result["queries"]] == [False, True, False]


@pytest.mark.parametrize("library_variable", ["TERRAIN_LIBRARY", "REJECTED_TERRAIN_LIBRARY"])
def test_rotated_wall_inflation_footprint_vs_continuous_geometry(executable, library_variable):
    from layered_terrain import TerrainMapper
    from test_terrain_diagnostics import surface

    library = os.environ.get(library_variable)
    if not library:
        pytest.skip(f"requires {library_variable}")
    c, s = np.cos(.17), np.sin(.17)
    rotation = np.array([[c, s], [-s, c]])
    points = surface("positive")
    points[:, :2] = points[:, :2] @ rotation
    origin = (-1.925, -1.925)
    with TerrainMapper(library, width=40, height=40, origin=origin) as mapper:
        mapper.integrate(points, 1., [0., 0., .5])
        costs = mapper.project(1.)[:, :, 0]
    poses = np.array([[x, y, yaw] for yaw in (0., .17, .61, np.pi / 2)
                      for x in np.arange(-1.1, 1.11, .1) for y in np.arange(-.9, 1.11, .1)])
    wall = np.array([[.32, .3], [.32, .9]]) @ rotation
    colliding = np.array([rectangle_intersects_segment(p[:2], p[2], *wall) for p in poses])
    assert colliding.any() and (~colliding).any()
    result = probe(executable, costs, poses, origin=origin)
    accepted = np.array([r["accepted"] for r in result["queries"]])
    assert accepted.any(), "all-blocked maps cannot establish useful safety"
    # A completely missed wall cannot be rescued by inflation.
    control = probe(executable, np.zeros_like(costs), poses, origin=origin)
    assert any(r["accepted"] and collision for r, collision in zip(control["queries"], colliding))
    failures = int((accepted & colliding).sum())
    known = {"69d182cadf025fa8f8c11934566fad0322f4d4d7c8f72f1f79faa14b6b34f9ff",
             "7a13403fad68b6ab9cf445cb05fe9f4e586841ccd25de9b152c73d3f339a9c85"}
    if failures and hashlib.sha256(Path(library).read_bytes()).hexdigest() in known:
        # Only this measured safety failure is expected, never process errors,
        # missing controls, all-blocked output or arbitrary future libraries.
        pytest.xfail(f"Frozen core accepts {failures} wall-intersecting padded footprints; safety gate remains failed")
    assert failures == 0


def test_continuous_wall_oracle_orientation_and_separation():
    assert rectangle_intersects_segment([0, 0], 0, [-1, 0], [1, 0])
    assert not rectangle_intersects_segment([0, 0], 0, [-1, .5], [1, .5])
    assert rectangle_intersects_segment([0, 0], np.pi / 2, [-1, .5], [1, .5])


def test_probe_constants_match_both_current_nav2_costmaps():
    # This diagnostic intentionally freezes current policy. A later YAML change
    # must update the probe and its provenance, not silently keep stale constants.
    path = Path(__file__).resolve().parents[2] / "astrobot-lab/src/astrobot_launch/config/nav2_phase3_bev.yaml"
    config = path.read_text()
    assert config.count('footprint: "[[0.495, 0.335], [0.495, -0.335], [-0.495, -0.335], [-0.495, 0.335]]"') == 2
    for parameter in ["footprint_padding: 0.10", "inflation_radius: 0.75", "cost_scaling_factor: 2.0",
                      "lethal_cost_threshold: 100", "trinary_costmap: false"]:
        assert config.count(parameter) == 2


@pytest.mark.parametrize("hazard", [100, -1])
def test_area_policy_checks_interior_hazards_and_keeps_soft_costs_traversable(executable, hazard):
    costs = np.zeros((40, 40), dtype=int)
    costs[20, 20] = hazard
    queries = [[-.45, .05, 0.], [-1.05, .05, 0.], [1.8, 0., 0.]]
    result = probe(executable, costs, queries)
    assert result["queries"][0]["accepted"], "lost perimeter-only blind-spot control"
    assert not result["queries"][0]["area_accepted"]
    assert result["queries"][1]["area_accepted"]
    assert not result["queries"][2]["area_accepted"], "outside-map body accepted"
    soft = probe(executable, np.full((40, 40), 20), [[0., 0., .61]])
    assert soft["queries"][0]["area_accepted"], "soft terrain cost became a hard veto"


def test_area_policy_blocks_continuous_wall_collisions_without_all_blocked_output(executable):
    from run_nav2_shadow_check import wall_case
    library = os.environ.get("TERRAIN_LIBRARY")
    if not library:
        pytest.skip("requires TERRAIN_LIBRARY")
    result, arrays = wall_case(library, executable)
    assert result["missed_wall_control_intersections_accepted"] > 0
    assert result["area_policy"]["passed"]
    assert not np.any(arrays["area_accepted"] & ~arrays["accepted"]), "veto expanded primitive acceptance"
