"""An Isaac shutdown status must never hide a failed/missing fixture result."""

import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_local_isaac_fixture import verify_result


@pytest.mark.parametrize("fault", [None, "zero_exit_failure", "no_report", "false", "qualified", "exit", "two_runs"])
def test_inner_fixture_evidence_required(tmp_path, fault):
    inner = tmp_path / "revision-abc123"
    inner.mkdir()
    log = "Run: /archive/revision-abc123\n"
    report = dict(fixture_passed=True, acceptance=False, rover_qualified=False)
    if fault == "zero_exit_failure":
        (inner / "failure.json").write_text("{}")
    if fault == "false":
        report["fixture_passed"] = False
    if fault == "qualified":
        report["rover_qualified"] = True
    if fault != "no_report":
        (inner / "report.json").write_text(json.dumps(report))
    (inner / "manifest.json").write_text("{}")
    if fault == "two_runs":
        log += log
    if fault:
        with pytest.raises((ValueError, FileNotFoundError)):
            verify_result(tmp_path, log, 1 if fault == "exit" else 0)
    else:
        result = verify_result(tmp_path, log, 0)
        assert result["fixture_passed"] and not result["acceptance"]


def test_inner_run_symlink_escape_rejected(tmp_path):
    archive = tmp_path / "archive"
    archive.mkdir()
    (archive / "revision-abc123").symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError, match="escaped"):
        verify_result(archive, "Run: /archive/revision-abc123\n", 0)
