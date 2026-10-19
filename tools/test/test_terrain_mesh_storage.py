"""Actual terrain grid method, isolated from Isaac imports; host NumPy/pytest."""
import ast
import gc
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def grid_method():
    source = Path(__file__).resolve().parents[2] / "OmniLRS/src/terrain_management/terrain_manager.py"
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "TerrainManager")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "buildGrid")
    isolated = ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[]))
    namespace = dict(np=np)
    exec(compile(isolated, str(source), "exec"), namespace)
    return namespace["buildGrid"]


@pytest.mark.parametrize("width,height", [(2, 2), (3, 7), (8, 4), (17, 19)])
def test_grid_indices_are_untracked_numeric_storage_with_identical_topology(width, height):
    obj = SimpleNamespace(_sim_width=width, _sim_length=height, _grid_size=.02, _indices=[],
                          gridIndex=lambda x, y, stride: y*stride+x)
    grid_method()(obj)
    expected, uv = [], []
    for y in range(1, height):
        for x in range(1, width):
            corners = [(x, y-1), (x, y), (x-1, y-1), (x, y), (x-1, y), (x-1, y-1)]
            expected.extend(b*width+a for a, b in corners)
            uv.extend(corners)
    assert isinstance(obj._indices, np.ndarray) and obj._indices.dtype == np.int64
    assert not gc.is_tracked(obj._indices), "millions of scalar indices retained in cyclic-GC traversal"
    np.testing.assert_array_equal(obj._indices, np.array(expected, dtype=np.int64))
    np.testing.assert_array_equal(obj._sim_uvs, np.array(uv, dtype=np.float32)*.02)
    expected_vertices = np.array([(x*.02, y*.02, 0.) for y in range(height) for x in range(width)], dtype=np.float32)
    np.testing.assert_array_equal(obj._sim_verts, expected_vertices)
    assert obj._indices.nbytes == 6*(width-1)*(height-1)*8
