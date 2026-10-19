import base64
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from render_cost_paths import packed


def test_report_preserves_cost_values_unknown_sign_and_row_order():
    grid = np.zeros((60, 60), np.int8)
    grid[0, :5] = [-1, 0, 20, 70, 100]
    grid[-1, -1] = 90
    decoded = np.frombuffer(base64.b64decode(packed(grid)), np.int8).reshape(60, 60)
    np.testing.assert_array_equal(decoded, grid)
    with pytest.raises(ValueError, match="60x60"):
        packed(np.zeros((50, 50)))
