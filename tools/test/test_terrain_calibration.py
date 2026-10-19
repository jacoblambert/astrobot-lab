import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]/"OmniLRS/src/environments_wrappers/ros2"


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cal = load("terrain_calibration")
sensor = load("terrain_sensor_audit")


def test_required_set_and_reserved_corridor_are_fixed():
    spec = cal.fixture_spec()
    assert len(spec["boxes"]) == 15
    assert sum(b["required"] for b in spec["boxes"]) == 12
    for b in spec["boxes"]:
        assert np.linalg.norm(b["center"]) < 5.
        assert abs(b["center"][1])-b["width"]/2 > .7


def test_geometry_error_and_filled_hole_fail_calibration():
    spec = cal.fixture_spec()
    checks = cal.query_checks(spec)

    def cast(origin, direction, distance):
        match = next(c for c in checks if np.allclose(c["xy"], origin[:2], rtol=0, atol=1e-8))
        if match["expected_z"] is None:
            return []
        z = match["expected_z"]
        return [(2-z, (origin[0],origin[1],z), (0,0,1), "/TerrainCalibration/test", "", 0)]

    assert cal.validate_queries(spec,cast)["passed"]

    def bad(origin, direction, distance):
        hits = cast(origin,direction,distance)
        return hits if hits else [(2., (*origin[:2],0.), (0,0,1), "/hidden_floor", "", 0)]

    assert not cal.validate_queries(spec,bad)["passed"]


def test_sensor_layout_is_explicit_and_preserves_invalid_returns():
    msg = NS(fields=[NS(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate("xyz")],
             point_step=12, row_step=24, width=2, height=1, is_bigendian=False,
             data=np.array([[1,2,3],[np.nan,0,0]],dtype="<f4").tobytes())
    points = sensor.cloud_xyz(msg)
    assert points.shape == (2,3)
    assert np.isnan(points[1,0])
    msg.point_step = 16
    with pytest.raises(ValueError):
        sensor.cloud_xyz(msg)
