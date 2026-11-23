import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from render_dropoff_fix import render
from replay_local_terrain import sha


def fixture(tmp_path):
    source, comparison, output = [tmp_path / name for name in ("source", "comparison", "output")]
    for folder in (source, comparison, output):
        folder.mkdir()
    classes = np.zeros((60, 60), dtype=np.int8)
    classes[29:31, 29:31] = -1
    original_grid = source / "grid.npz"
    np.savez_compressed(original_grid, classes=classes)
    score = dict(samples=10, progress_available=5, progress_or_turn=6)
    floor = dict(samples=100, free=90, occupied=0)
    regions = [f"{side}_{low}_{low+1}m" for side in ("front", "rear") for low in range(3)]
    before = dict(
        summary={period: score for period in ("all", "after_15s")},
        floor_totals={region: floor for region in regions},
        contract=dict(patches=[dict(center=[0, 0.3, 0], half_extent=[0.04, 0.06, 0.001])]),
        grids=dict(examples=[dict(evidence_ns=1, file="grid.npz", sha256=sha(original_grid))]),
    )
    (source / "report.json").write_text(json.dumps(before))
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = classes
    path = comparison / "comparison.npz"
    np.savez_compressed(
        path,
        baseline=layers,
        candidate=np.zeros_like(layers),
        origin=[-3, -3],
        resolution=0.1,
        local_from_base=np.eye(4),
    )
    after = dict(
        completed=True,
        exact_baseline=True,
        summary=before["summary"],
        floor_totals=before["floor_totals"],
        input_sha256={str(source / "report.json"): sha(source / "report.json")},
        examples=[dict(evidence_ns=1, elapsed_s=0, file=path.name, sha256=sha(path))],
    )
    (comparison / "report.json").write_text(json.dumps(after))
    return source, comparison, output


def test_comparison_displays_original_and_replay_without_claiming_live_navigation(tmp_path):
    source, comparison, output = fixture(tmp_path)
    render(source, comparison, output)
    text = (output / "false-dropoff-fix.html").read_text()
    assert "Original map" in text and "Replay with fix" in text and "not a new live drive" in text
    assert "50.0%" in text and "60.0%" in text and text.count("<svg") == 2


@pytest.mark.parametrize("fault", ["source", "denominator", "missing", "grid"])
def test_comparison_cannot_change_reference_or_hide_missing_examples(tmp_path, fault):
    source, comparison, output = fixture(tmp_path)
    report = json.loads((comparison / "report.json").read_text())
    if fault == "source":
        report["input_sha256"] = {}
    elif fault == "denominator":
        report["floor_totals"]["front_0_1m"]["samples"] = 101
    elif fault == "missing":
        report["examples"] = []
    else:
        (comparison / "comparison.npz").write_bytes(b"changed")
    (comparison / "report.json").write_text(json.dumps(report))
    with pytest.raises(ValueError):
        render(source, comparison, output)
