"""Small synthetic heap ownership fixture, without Isaac or motion."""

import gc
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulator_heap_profile import profile_heap


def test_heap_profile_preserves_gc_and_reports_owners(monkeypatch):
    payload = list(range(2000))
    owner = {"mesh_indices": payload}
    monkeypatch.setattr(gc, "get_objects", lambda: [payload, owner])
    monkeypatch.setattr(gc, "get_referrers", lambda value: [owner] if value is payload else [])
    before = gc.get_threshold(), gc.isenabled()
    result = profile_heap(-1, False, None)
    assert (gc.get_threshold(), gc.isenabled()) == before
    assert result["acceptance"] is False and result["tracked_objects"] == 2
    row = result["largest_containers"][0]
    assert row["length"] == 2000
    assert row["owners"][0]["keys_referring_to_object"] == ["mesh_indices"]


@pytest.mark.parametrize("args", [(0, False, None), (-1, True, None), (-1, False, 0.0)])
def test_heap_profile_rejects_any_motion_authority(args):
    with pytest.raises(ValueError, match="pre-lease"):
        profile_heap(*args)


def test_profile_hook_aborts_before_request_publication():
    source = (Path(__file__).resolve().parents[2] /
              "OmniLRS/src/environments_wrappers/ros2/natural_holdout_capture.py").read_text()
    start = source.index('if os.getenv("OMNILRS_HEAP_PROFILE_ONLY"')
    assert start < source.index('self.request = dict(')
    assert 'raise RuntimeError("stationary heap diagnostic complete; no motion requested")' in source[start:]
