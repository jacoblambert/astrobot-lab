import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from shapely import box

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.phase_support import PHASES, PhaseSupport
from local_terrain.physx_reference import BodyBox


def fixture():
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = -1
    record = dict(
        origin=[0.0, 0.0],
        resolution=0.1,
        local_from_map=np.eye(4).tolist(),
        session="s",
        epoch=1,
        revision=3,
        evidence_ns=100,
        publication_ns=110,
    )
    grids = [(layers.copy(), dict(record, origin=list(offset))) for offset in PHASES[1:]]
    index = SimpleNamespace(origin=np.array([0.0, 0.0]), unions={})
    return layers, record, grids, index


def test_approved_partial_cell_does_not_fill_unseen_remainder():
    layers, record, grids, index = fixture()
    grids[0][0][2, 2, 0] = 0
    candidate = PhaseSupport(index, grids, record)
    cover = candidate.get((2, 2))
    assert cover.intersection(box(0.2, 0.2, 0.3, 0.3)).area == pytest.approx(0.005)
    assert not cover.covers(box(0.2, 0.2, 0.3, 0.3))
    assert candidate.get((20, 20)) is None
    assert (layers[:, :, 0] == -1).all()


@pytest.mark.parametrize("field,value", [("epoch", 2), ("revision", 4), ("publication_ns", 111), ("evidence_ns", 99)])
def test_mixed_phase_generation_is_rejected(field, value):
    _, record, grids, index = fixture()
    grids[0][1][field] = value
    with pytest.raises(ValueError, match="generation"):
        PhaseSupport(index, grids, record)


@pytest.mark.parametrize("cost,reason", [(100, 3), (-1, 8), (-1, 11)])
def test_phase_obstacles_and_holes_veto_otherwise_available_support(cost, reason):
    layers, record, grids, index = fixture()
    layers[:, :, 0] = 0
    grids[0][0][2, 2, :2] = [cost, reason]
    pose = np.eye(4)
    pose[:2, 3] = [0.28, 0.25]
    patch = BodyBox((0, 0, 0), (0.01, 0.01, 0.001), "fixture")
    result = PhaseSupport(index, grids, record).query(layers, record, [pose], [patch], patch)
    assert result["state"] != "available"


def test_free_strip_supports_only_covered_patch():
    layers, record, grids, index = fixture()
    grids[0][0][2, 2, 0] = 0
    patch = BodyBox((0, 0, 0), (0.01, 0.01, 0.001), "fixture")
    candidate = PhaseSupport(index, grids, record)
    pose = np.eye(4)
    pose[:2, 3] = [0.28, 0.25]
    assert candidate.query(layers, record, [pose], [patch], patch)["state"] == "available"
    pose[0, 3] = 0.21
    assert candidate.query(layers, record, [pose], [patch], patch)["state"] == "unknown"
