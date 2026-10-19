import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from astrobot_core.shadow_seed import shadow_dump_path


def test_shadow_dump_uses_current_external_run_not_tmp_or_stale_device(tmp_path, monkeypatch):
    run = tmp_path / ("revision-" + "a" * 32)
    run.mkdir()
    seed = run / "live_gt_seed.json"
    seed.write_text("{}")
    contract = dict(run=run.name, root=str(tmp_path), device=987654, external_required=True)
    path = run / "archive_contract.json"
    path.write_text(json.dumps(contract))
    original = Path.stat
    monkeypatch.setattr(
        Path, "stat", lambda self, **kw: SimpleNamespace(st_dev=987654) if self == tmp_path else original(self, **kw)
    )
    assert shadow_dump_path(seed, 1) == run / "glim_dump_epoch_1"
    contract["device"] -= 1
    path.write_text(json.dumps(contract))
    with pytest.raises(ValueError, match="lease"):
        shadow_dump_path(seed, 1)
    contract["device"] += 1
    contract["external_required"] = False
    path.write_text(json.dumps(contract))
    with pytest.raises(ValueError, match="lease"):
        shadow_dump_path(seed, 1)
