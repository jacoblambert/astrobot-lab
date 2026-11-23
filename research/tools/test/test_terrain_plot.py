import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
pytest.importorskip("matplotlib")
from plot_terrain_calibration import evaluation_reference


def test_plot_retains_scored_reference_after_archive_relocation(tmp_path):
    capture = tmp_path / "tilted"
    reference = tmp_path / "original"
    evaluation = tmp_path / "score"
    for directory in (capture, reference, evaluation):
        directory.mkdir()
    payload = b"fixed reference"
    (reference / "collision_tile.npz").write_bytes(payload)
    (evaluation / "manifest.json").write_text(json.dumps(dict(
        reference_capture="/archive/original",
        reference_sha256={"collision_tile.npz": hashlib.sha256(payload).hexdigest()},
    )))
    assert evaluation_reference(capture, evaluation) == reference
    (reference / "collision_tile.npz").write_bytes(b"changed")
    with pytest.raises(ValueError, match="changed after scoring"):
        evaluation_reference(capture, evaluation)


def test_plot_does_not_fall_back_to_new_sensor_reference(tmp_path):
    capture = tmp_path / "tilted"
    capture.mkdir()
    evaluation = tmp_path / "score"
    evaluation.mkdir()
    (evaluation / "manifest.json").write_text(json.dumps(dict(reference_capture="/archive/missing")))
    with pytest.raises(ValueError, match="unavailable"):
        evaluation_reference(capture, evaluation)
