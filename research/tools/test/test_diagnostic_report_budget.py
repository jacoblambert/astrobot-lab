"""Small reports get stricter quotas, never a reduced archive reserve."""

import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.mark.parametrize("name", ["diagnose_failed_glim", "export_gap_examples", "render_gap_report"])
def test_report_uses_one_gib_quota_without_overriding_reserve_or_external_check(name, tmp_path, monkeypatch):
    module = importlib.import_module(name)
    args = [str(tmp_path)] if name == "render_gap_report" else [str(tmp_path), str(tmp_path)]
    monkeypatch.setattr(sys, "argv", [name, *args])
    monkeypatch.setattr(module, "check_run", lambda path: None)
    called = []

    def allocate(root, **kwargs):
        called.append(kwargs)
        raise ValueError("test allocation boundary")

    monkeypatch.setattr(module, "allocate_run", allocate)
    with pytest.raises(ValueError, match="test allocation boundary"):
        module.main()
    assert called == [dict(limit=1024**3)]


def test_grid_export_quota_failure_preserves_payload_and_marks_failure(tmp_path, monkeypatch):
    import archive_guard
    import export_gap_examples as module

    output = archive_guard.allocate_run(tmp_path, limit=4096, reserve=0, external=False)
    monkeypatch.setattr(sys, "argv", ["export_gap_examples", str(output), str(output)])
    monkeypatch.setattr(module, "allocate_run", lambda root, **kwargs: output)

    def export(*args, **kwargs):
        (output / "oversized_grid_fixture").write_bytes(b"x" * 4096)
        return {}

    monkeypatch.setattr(module, "export", export)
    with pytest.raises(ValueError, match="quota"):
        module.main()
    assert (output / "failure.json").is_file()
    assert not (output / "report.json").exists()
    assert (output / "oversized_grid_fixture").stat().st_size == 4096
