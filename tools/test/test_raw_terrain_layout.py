import sys
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay_raw_terrain import xyz_array


@pytest.mark.parametrize("big", [False, True])
def test_padding_field_order_and_endian(big):
    data = bytearray(64)
    dtype = np.dtype(dict(names=["x","y","z"], formats=[(">" if big else "<")+"f4"]*3,
                         offsets=[8,0,4], itemsize=16))
    rows = np.ndarray((2,1), dtype=dtype, buffer=data, strides=(32,16))
    rows["x"] = 1; rows["y"] = 2; rows["z"] = 3
    msg = NS(fields=[NS(name=n, offset=o, count=1, datatype=7) for n,o in zip(["x","y","z"],[8,0,4])],
             width=1, height=2, point_step=16, row_step=32, data=data, is_bigendian=big)
    np.testing.assert_equal(xyz_array(msg), [[1,2,3],[1,2,3]])
    msg.row_step = 8
    with pytest.raises(ValueError):
        xyz_array(msg)
