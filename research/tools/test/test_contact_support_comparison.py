import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from compare_contact_support import compare_rows


def row():
    return dict(
        evidence_ns=0,
        elapsed_s=0,
        history_anchors=1,
        missing_pose_anchors=0,
        diagnostic_query_wall_s=1,
        variants=dict(tracks=dict(forward=dict(state="available", new_ground_area_m2=0.0, unknown_cells=0))),
        retrace=dict(tracks=dict(state="unknown")),
    )


@pytest.mark.parametrize(
    "change", ["none", "timing", "area_rounding", "state", "area", "missing", "stamp", "candidate"]
)
def test_comparison_preserves_exposures_and_decisions(change):
    a = [row()]
    b = copy.deepcopy(a)
    if change == "timing":
        b[0]["diagnostic_query_wall_s"] = 0.1
    if change == "area_rounding":
        b[0]["variants"]["tracks"]["forward"]["new_ground_area_m2"] = 1e-14
    if change == "state":
        b[0]["variants"]["tracks"]["forward"]["state"] = "unknown"
    if change == "area":
        b[0]["variants"]["tracks"]["forward"]["new_ground_area_m2"] = 1e-9
    if change == "missing":
        b.clear()
    if change == "stamp":
        b[0]["evidence_ns"] = 1
    if change == "candidate":
        b[0]["variants"]["tracks"].clear()
    if change in ("none", "timing", "area_rounding"):
        assert compare_rows(a, b) == 1
    else:
        with pytest.raises(ValueError):
            compare_rows(a, b)
