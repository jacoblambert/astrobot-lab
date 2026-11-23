import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from terrain_metrics import calibration_gates


def metric():
    return dict(object_count=12,detected_objects=12,flat_cells=100,dropoff_cells=10,
                flat_free_fraction=.95,flat_false_occupied_fraction=.01,
                obstacle_false_free_fraction=0.,dropoff_free_cells=0)


def gates(sequence,**kwargs):
    return calibration_gates(sequence,required_objects=12,p95_ms=49.,p99_ms=99.,peak_rss_mib=300.,**kwargs)


def test_development_pass_is_never_promotion():
    result=gates([metric()])
    assert result["geometry_passed"] and result["sampled_compute_passed"]
    assert not result["promotion_allowed"]


@pytest.mark.parametrize("field,value",[("detected_objects",11),("flat_cells",0),("dropoff_cells",0),
    ("flat_free_fraction",.949),("flat_free_fraction",float("nan")),("flat_false_occupied_fraction",.011),
    ("obstacle_false_free_fraction",.001),("dropoff_free_cells",1)])
def test_gate_failures_cannot_be_hidden(field,value):
    item=metric(); item[field]=value
    assert not gates([item])["geometry_passed"]


def test_final_safe_map_cannot_hide_earlier_free_hole():
    earlier=metric();earlier["dropoff_free_cells"]=1
    assert not gates([earlier,metric()])["geometry_passed"]
    assert not gates([])["geometry_passed"]


@pytest.mark.parametrize("earlier_value", [.001, None, float("nan")])
def test_final_safe_map_cannot_hide_earlier_free_positive_obstacle(earlier_value):
    earlier=metric(); earlier["obstacle_false_free_fraction"]=earlier_value
    result=gates([earlier,metric()])
    assert result["geometry"]["final_positive_false_free"]
    assert not result["geometry"]["sequence_positive_safe"] and not result["geometry_passed"]


def test_missing_or_nonfinite_budget_is_not_a_pass():
    result=calibration_gates([metric()],required_objects=12,p95_ms=float("nan"),p99_ms=None,peak_rss_mib=0.)
    assert not result["sampled_compute_passed"] and not any(result["sampled_compute"].values())
