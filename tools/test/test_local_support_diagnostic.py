import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnose_local_support import loaded_patches


def inputs(tmp_path):
    capture, loaded = tmp_path / "capture", tmp_path / "loaded"
    capture.mkdir()
    loaded.mkdir()
    (capture / "manifest.json").write_text(
        json.dumps({"source_asset_sha256": {"/assets/ros2_husky_PhysX_vlp16_mono_depth_imu.usd": "fixture-asset"}})
    )
    (loaded / "manifest.json").write_text(
        json.dumps({"asset_sha256": "fixture-asset", "wheels": ["fl", "fr", "rl", "rr"]})
    )
    positions = [
        [0, 0, 0.14],
        [0.256, 0.2854, 0.177],
        [0.256, -0.2854, 0.177],
        [-0.256, 0.2854, 0.177],
        [-0.256, -0.2854, 0.177],
    ]
    rows = [dict(step=i, positions=positions, orientations_wxyz=[[1, 0, 0, 0]] * 5) for i in range(1, 481)]
    (loaded / "contacts.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (loaded / "report.json").write_text(
        json.dumps(
            dict(
                fixture_passed=True,
                mean_base_height_m=0.14,
                contacts_sha256=hashlib.sha256((loaded / "contacts.jsonl").read_bytes()).hexdigest(),
            )
        )
    )
    return capture, loaded


def test_loaded_positions_not_joint_or_cached_bbox_supply_patch_centers(tmp_path):
    patches, hashes = loaded_patches(*inputs(tmp_path))
    assert len(patches) == len(hashes) == 4
    np.testing.assert_allclose(patches[0].center, [0.256, 0.2854, -0.14])
    assert patches[0].half_extent == (0.04, 0.06, 0.001)
    assert ":assumed_patch:" in patches[0].source_identity


@pytest.mark.parametrize("fault", ["failure", "report", "asset", "contacts", "height", "missing_tail"])
def test_invalid_or_changed_geometry_provenance_is_rejected(tmp_path, fault):
    capture, loaded = inputs(tmp_path)
    report = json.loads((loaded / "report.json").read_text())
    if fault == "failure":
        (loaded / "failure.json").write_text("{}")
    elif fault == "report":
        report["fixture_passed"] = False
    elif fault == "asset":
        manifest = json.loads((loaded / "manifest.json").read_text())
        manifest["asset_sha256"] = "different-asset"
        (loaded / "manifest.json").write_text(json.dumps(manifest))
    elif fault == "contacts":
        (loaded / "contacts.jsonl").write_text("changed")
    elif fault == "height":
        report["mean_base_height_m"] = float("nan")
    else:
        rows = (loaded / "contacts.jsonl").read_text().splitlines()
        (loaded / "contacts.jsonl").write_text("\n".join(rows[:-1]) + "\n")
        report["contacts_sha256"] = hashlib.sha256((loaded / "contacts.jsonl").read_bytes()).hexdigest()
    (loaded / "report.json").write_text(json.dumps(report))
    with pytest.raises(ValueError):
        loaded_patches(capture, loaded)
