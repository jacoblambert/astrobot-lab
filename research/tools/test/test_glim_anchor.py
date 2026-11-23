"""Host NumPy/SciPy/pytest; origin/gauge diagnosis cannot hide drift."""
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_glim_anchor import diagnose


def poses():
    p=np.tile(np.eye(4),(21,1,1))
    p[:,:3,:3]=Rotation.from_euler("z",np.linspace(0,1,21)).as_matrix()
    p[:,0,3]=np.linspace(0,2,21)
    return p


@pytest.mark.parametrize("side",["world","body"])
def test_fixed_gauge_or_sensor_origin_is_removed_by_its_own_hypothesis(side):
    gt=poses(); offset=np.eye(4)
    offset[:3,3]=[.4,.1,-.2]; offset[:3,:3]=Rotation.from_euler("xyz",[.1,-.1,.2]).as_matrix()
    est=offset@gt if side=="world" else gt@offset
    report=diagnose(est,gt)
    key="after_initial_world_gauge_only" if side=="world" else "after_initial_body_extrinsic_only"
    assert report[key]["position_m"]["maximum"]<1e-12
    assert report[key]["rotation_deg"]["maximum"]<1e-12


def test_zero_start_does_not_remove_subsequent_drift():
    gt=poses(); est=gt.copy(); est[:,1,3]+=np.linspace(0,.7,len(gt))
    report=diagnose(est,gt)
    assert report["raw"]["position_m"]["initial"]==0.
    for key in ("raw","after_initial_world_gauge_only","after_initial_body_extrinsic_only"):
        assert report[key]["position_m"]["final"]==pytest.approx(.7)
    assert not report["corrections_applied_to_mapper"]


def test_same_mount_both_sides_preserves_identical_trajectory():
    gt=poses(); mount=np.eye(4); mount[:3,3]=[.4,.1,.25]
    assert diagnose(gt@mount,gt@mount)["raw"]["position_m"]["maximum"]==0.
