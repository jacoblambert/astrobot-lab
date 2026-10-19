"""Exact C++ optimization checks against a separately built dense-stencil oracle."""

import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from layered_terrain import TerrainMapper

LIBRARY = os.environ.get("TERRAIN_LIBRARY")
REFERENCE = os.environ.get("TERRAIN_REFERENCE_LIBRARY")
pytestmark = pytest.mark.skipif(not LIBRARY or not REFERENCE, reason="set candidate and frozen reference libraries")


@pytest.mark.parametrize("cells", [1, 7, 60, 80])
@pytest.mark.parametrize("phase", [0.0, 0.025, 0.075])
@pytest.mark.parametrize("project_each", [True, False])
@pytest.mark.parametrize("point_count", [2400, 20000])
def test_ordered_sparse_native_patches_exact_through_layers_boundaries_and_expiry(
    cells, phase, project_each, point_count
):
    rng = np.random.default_rng(12091)
    extent = cells * 0.1
    origin = (-extent / 2 + phase, -extent / 2 + phase)
    xy = rng.uniform(-extent / 2 - 0.1, extent / 2 + 0.1, (point_count, 2))
    z = 0.1 * xy[:, 0] + rng.normal(0, 0.002, len(xy))
    # More than three simultaneous layers, thin faces and non-uniform sparsity.
    z[::9] += 0.1
    z[1::13] += 0.2
    z[2::17] += 0.3
    z[3::19] -= 0.2
    points = np.column_stack([xy, z])
    with TerrainMapper(LIBRARY, width=cells, height=cells, origin=origin, lifetime=1.2) as candidate, TerrainMapper(
        REFERENCE, width=cells, height=cells, origin=origin, lifetime=1.2
    ) as reference:
        for index, stamp in enumerate((0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.9, 1.4, 2.1, 2.9)):
            native = points[index % 6 :: 6]
            for mapper in (candidate, reference):
                mapper.integrate(native, stamp, [index * 0.08, 0, 0.5], index * 0.03)
            # byte equality retains NaN payloads as well as every geometry layer.
            if project_each or index in (5, 9):
                assert candidate.project(stamp).tobytes() == reference.project(stamp).tobytes()
        assert candidate.project(4.2).tobytes() == reference.project(4.2).tobytes()


@pytest.mark.parametrize("inputs,touched", [(2055, 1), (720, 1000)])
def test_deferred_input_count_and_memory_bounds_flush_without_changing_votes(inputs, touched):
    # First case crosses 2048 pending inputs. Second exceeds 64 MiB even using
    # only the 100-byte ObservationMoments payload (720*1000*100), excluding
    # row indices/alignment. Both cases stay below the large-input bypass.
    x, y = np.meshgrid(np.arange(40), np.arange(25))
    xy = np.c_[x.ravel(), y.ravel()][:touched] * 0.1 - [2, 1.2] + [0.023, 0.037]
    points = np.c_[xy, 0.04 * xy[:, 0]]
    with TerrainMapper(LIBRARY, width=60, height=60, origin=(-3, -3), lifetime=30) as candidate, TerrainMapper(
        REFERENCE, width=60, height=60, origin=(-3, -3), lifetime=30
    ) as reference:
        for i in range(inputs):
            stamp = i / 10000  # one immutable bucket; no projection-triggered flush
            for mapper in (candidate, reference):
                mapper.integrate(points, stamp, [0, 0, 0.5])
        assert candidate.project(1.0).tobytes() == reference.project(1.0).tobytes()
        assert candidate.project(31.0).tobytes() == reference.project(31.0).tobytes()
