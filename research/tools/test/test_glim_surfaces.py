"""Host NumPy/pytest: independent surface selection never fits pose error away."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from diagnose_glim_surfaces import surface_mask, displacement_stats


def test_box_shell_excludes_floor_interior_and_other_objects():
    p=np.array([[0,0,.3],[.5,0,.1],[0,0,0],[0,0,.2],[.6,0,.3]])
    assert surface_mask(p,[0,0],1.,.3).tolist()==[True,True,False,False,False]


def test_constant_displacement_is_not_aligned_away():
    result=displacement_stats([np.tile([.3,.4,0],(20,1))])
    assert result["eligible"] and not result["p95_pass"]
    assert result["displacement_p95_m"]==.5
    assert not displacement_stats([])["eligible"]
