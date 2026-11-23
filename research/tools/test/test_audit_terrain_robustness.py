import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import audit_terrain_robustness as audit


def test_profile_parsing():
    assert audit.parse_profile("slam") == ("slam", dict(h=0.0, z=0.0, y=0.0, r=0.0, tau=5.0, seed=0))
    base, noise = audit.parse_profile("gt+h15,z1.4,y0.35,r0.6,tau0.5,seed2")
    assert base == "gt" and noise == dict(h=15.0, z=1.4, y=0.35, r=0.6, tau=0.5, seed=2.0)
    with pytest.raises(ValueError):
        audit.parse_profile("odom")
    with pytest.raises(ValueError):
        audit.parse_profile("gt+q3")


def test_ou_error_matches_requested_p95():
    err = audit.OUError(15, 2, 0.5, 0.6, tau=1.0, seed=3, t0=0, t1=4000)
    x = err.x[1000:]
    assert np.percentile(np.hypot(x[:, 0], x[:, 1]), 95) == pytest.approx(0.15, rel=0.08)
    assert np.percentile(np.abs(x[:, 2]), 95) == pytest.approx(0.02, rel=0.08)
    assert np.degrees(np.percentile(np.abs(x[:, 3]), 95)) == pytest.approx(0.5, rel=0.08)
    assert np.degrees(np.percentile(np.abs(x[:, 4]), 95)) == pytest.approx(0.6, rel=0.08)
    # deterministic in stamp: the anchor frame must see the same error every revision
    assert np.array_equal(err(12.345), err(12.345))


def test_perturb_rotates_about_body_not_map_origin():
    T = np.eye(4)
    T[:3, 3] = [40.0, 38.0, 0.1]
    out = audit.perturb(T, np.array([0.1, 0.0, 0.0, np.deg2rad(2), 0.0, 0.0]))
    # yaw error must not swing the body 40 m from the map origin
    assert np.allclose(out[:3, 3], [40.1, 38.0, 0.1])
    assert Rotation.from_matrix(out[:3, :3]).as_euler("zyx", degrees=True)[0] == pytest.approx(2)
    tilted = audit.perturb(T, np.array([0, 0, 0, 0, 0, np.deg2rad(1)]))
    # body pitch error tilts the body x axis, i.e. a range-proportional height error
    assert tilted[2, 0] == pytest.approx(-np.sin(np.deg2rad(1)))


def _grid(n=60, res=0.1):
    ij = np.stack(np.meshgrid(np.arange(n), np.arange(n), indexing="xy"), -1).reshape(-1, 2)
    rel = (ij + 0.5) * res - n * res / 2  # rover at grid centre
    return ij, rel


def test_corridor_policies_on_synthetic_grid():
    scorer = audit.Scorer.__new__(audit.Scorer)
    ij, rel = _grid()
    cls = np.zeros(len(rel))
    reason = np.ones(len(rel), int)
    hazard = np.zeros(len(rel))
    ahead = (rel[:, 0] > 0.5) & (rel[:, 0] < 1.5) & (np.abs(rel[:, 1]) < 0.4)
    # plain missing evidence ahead (overlapping 22.5 deg corridors see it too): strict blocks those headings
    cls[ahead & (rel[:, 0] > 1.0)] = -1
    reason[ahead & (rel[:, 0] > 1.0)] = 0
    occ_dist = np.full(len(rel), np.inf)
    out = scorer.actions(rel, 0.0, cls, reason, hazard, hazard, occ_dist)
    assert out["optimistic"]["available"] == 16 and 12 <= out["strict"]["available"] < 16
    assert out["bounded"]["available"] == out["optimistic"]["available"]
    # a hidden rock in that unknown patch is a bounded/optimistic false-safe, never strict
    hazard[ahead & (rel[:, 0] > 1.2)] = 0.3
    out = scorer.actions(rel, 0.0, cls, reason, hazard, hazard, occ_dist)
    assert out["strict"]["gt_unsafe"] == 0 and out["bounded"]["gt_unsafe"] >= 1
    # unknown in an occupied cell's shadow is not accepted by the bounded policy
    shadowed = np.where(cls == -1, 0.2, np.inf)
    out = scorer.actions(rel, 0.0, cls, reason, hazard, hazard, shadowed)
    assert out["bounded"]["gt_unsafe"] == 0


def test_eroded_footprint_drops_the_approximate_outline_only():
    from shapely import Point, affinity

    ellipse = affinity.scale(Point(0, 0).buffer(1.0, 32), 0.4, 0.2)
    core = audit.eroded(ellipse)
    # a cell centre 2 cm inside the far tip (the instance_38 artifact) is not a hazard label
    assert ellipse.contains(Point(0.38, 0)) and not core.contains(Point(0.38, 0))
    assert core.contains(Point(0.2, 0))
    tiny = Point(0, 0).buffer(0.04)
    assert audit.eroded(tiny).equals(tiny)


class _Scene:
    def __init__(self, objects, survey):
        from shapely import prepare

        self.objects, self.survey = objects, survey
        for o in objects:
            prepare(o["poly"])
            prepare(o["core"])
        prepare(survey)

    def in_survey(self, xy):
        from shapely import contains_xy

        return contains_xy(self.survey, xy[:, 0], xy[:, 1])


def _frame(rover_x, t, marked, phantom=None):
    """Rover at map (rover_x, 0) facing +x, grid centred on it, identity local/map alignment."""
    origin = np.array([rover_x - 3.0, -3.0])
    layers = np.zeros((60, 60, 6))
    layers[:, :, 1] = 1
    centres = origin + (np.indices((60, 60))[::-1].transpose(1, 2, 0) + 0.5) * 0.1  # [row, col] -> (x, y)
    if marked:
        on = (np.abs(centres[..., 0] - 3.0) <= 0.2) & (np.abs(centres[..., 1]) <= 0.2)
        layers[on, 0] = 100
    if phantom is not None:
        near = np.hypot(centres[..., 0] - phantom[0], centres[..., 1] - phantom[1]) < 0.08
        layers[near, 0] = 100
    pose = np.eye(4)
    pose[0, 3] = rover_x
    return layers, dict(evidence_ns=int(t * 1e9), origin=origin.tolist(), local_from_base=pose.tolist()), pose


def _scorer():
    from shapely import box

    from local_terrain.nav2_contract import Nav2Config

    rock = box(2.8, -0.2, 3.2, 0.2)
    scene = _Scene([dict(name="rock", poly=rock, core=rock, height=0.3, in_survey=True)], box(-5, -5, 10, 5))
    return audit.Scorer(scene, 60, 0.1, nav2=Nav2Config())


def test_first_mark_and_unmarked_within_2m_on_a_synthetic_approach():
    scorer = _scorer()
    scorer(*_frame(-2.0, 0.0, marked=False))  # startup frame, far away
    for i, x in enumerate([0.5, 0.9, 1.3, 1.6, 2.0]):  # footprint edge distance 2.3 .. 0.8 m
        scorer(*_frame(x, 10.0 + i, marked=x >= 1.3))
    s = scorer.summary()
    rock = s["objects"]["rock"]
    assert rock["first_mark_dist_m"] == pytest.approx(1.5)
    assert rock["within_2m_frames"] == 4 and rock["within_2m_unmarked"] == 1  # x=0.9 (1.9 m) unmarked
    assert rock["danger_frames"] == 1 and rock["danger_unmarked"] == 0
    recall = s["c4_recall"][">=0.20m"]
    assert recall["within_2m_unmarked_frames"] == 1 and recall["min_first_mark_dist_m"] == pytest.approx(1.5)


def test_open_floor_phantom_vs_cells_next_to_an_object():
    scorer = _scorer()
    scorer(*_frame(0.0, 0.0, marked=False))
    scorer(*_frame(0.0, 10.0, marked=False, phantom=(1.0, 1.0)))  # open floor
    scorer(*_frame(0.0, 11.0, marked=False, phantom=(2.7, 0.0)))  # 0.1 m from the rock: not a phantom
    rows = scorer.rows[1:]
    assert rows[0]["open_floor_phantom"] >= 1 and rows[0]["phantom_near"] >= 1
    assert rows[1]["flat_phantom"] == 0 and rows[1]["open_floor_phantom"] == 0


def test_nav2_rows_and_unknown_crossing_counts():
    scorer = _scorer()
    layers, record, gt = _frame(0.0, 0.0, marked=True)
    layers[25:35, 36:44, 0] = -1  # plain-missing patch 0.6-1.4 m ahead
    layers[25:35, 36:44, 1] = 0
    scorer(layers, record, gt)
    nav2 = scorer.rows[0]["nav2"]
    assert nav2["nav2_strict"]["available"] < nav2["nav2_bounded"]["available"]
    assert nav2["nav2_bounded"]["crossed_unknown"] > 0
    assert nav2["nav2_bounded"]["crossed_unknown_hazard"] == 0


def test_suite_checks_and_regressions():
    import terrain_audit_suite as suite

    checks = [dict(criterion="C1", name="acts", profiles=["slam"], metric=["a", "b"], reduce="min", op=">=",
                   value=0.9, tolerance=0.01),
              dict(criterion="C4", name="missed", profiles=["*"], metric=["m"], reduce="sum", op="==", value=0)]
    reports = [dict(profile="slam", source="/x/revision-1", a=dict(b=0.95), m=0),
               dict(profile="gt+h15", source="/x/revision-1", a=dict(b=0.5), m=1)]
    rows, criteria = suite.score(dict(checks=checks), reports)
    assert rows[0]["value"] == 0.95 and rows[0]["jobs"] == 1 and rows[0]["passed"]
    assert rows[1]["value"] == 1 and not rows[1]["passed"] and criteria == dict(C1=True, C4=False)
    baseline = dict(checks=[dict(rows[0], value=0.99), dict(rows[1], value=1)])
    rows, _ = suite.score(dict(checks=checks), reports, baseline)
    assert rows[0]["regressed"] and not rows[1]["regressed"]


def test_glim_profile_parsing():
    assert audit.parse_profile("glim:revision-3452078552bf")[0] == "glim:revision-3452078552bf"
    base, noise = audit.parse_profile("glim:revision-34520785+z1")
    assert base == "glim:revision-34520785" and noise["z"] == 1.0
    with pytest.raises(ValueError):
        audit.parse_profile("glim:../../etc")
