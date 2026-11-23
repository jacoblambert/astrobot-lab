import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import audit_completed_route as runner


@pytest.mark.parametrize("fail", [None, "export", "measure"])
def test_bounded_outputs_keep_evaluators_and_failures(monkeypatch, tmp_path, fail):
    allocated, checked = [], []

    def allocate(root, *, limit):
        assert root == tmp_path and limit == 1024**3
        path = root / str(len(allocated))
        path.mkdir()
        allocated.append(path)
        return path

    def export(recorder, output):
        assert recorder == "recorder" and output == allocated[0]
        if fail == "export":
            raise ValueError("export failure")
        return dict(passed=True)

    def measure(capture, recorder, trajectory, output):
        assert (capture, recorder, trajectory, output) == ("capture", "recorder", *allocated)
        assert json.loads((trajectory / "shadow_trajectory.json").read_text())["passed"]
        if fail == "measure":
            raise ValueError("measure failure")
        return dict(results={}, acceptance=False)

    monkeypatch.setattr(runner, "allocate_run", allocate)
    monkeypatch.setattr(runner, "check_run", checked.append)
    monkeypatch.setattr(runner, "export", export)
    monkeypatch.setattr(runner, "measure", measure)
    if fail:
        with pytest.raises(ValueError, match=fail):
            runner.audit("capture", "recorder", tmp_path)
        assert json.loads((allocated[-1] / "failure.json").read_text())["acceptance"] is False
    else:
        assert runner.audit("capture", "recorder", tmp_path) == tuple(allocated)
        assert checked == allocated
