from pathlib import Path
import sys
from collections import namedtuple
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import archive_guard as guard


def test_streamed_size_matches_recursive_file_semantics(tmp_path, monkeypatch):
    (tmp_path / "nested").mkdir()
    (tmp_path / "a").write_bytes(b"abc")
    (tmp_path / "nested/b").write_bytes(b"12345")
    (tmp_path / "file_link").symlink_to(tmp_path / "a")
    (tmp_path / "dir_link").symlink_to(tmp_path / "nested", target_is_directory=True)
    (tmp_path / "broken").symlink_to(tmp_path / "missing")
    expected = sum(p.stat().st_size for p in tmp_path.rglob("*") if p.is_file())
    def forbidden(*args, **kwargs):
        raise AssertionError("quota hot path must not build a recursive Path graph")
    monkeypatch.setattr(Path, "rglob", forbidden)
    assert expected == 11
    assert guard.archive_size(tmp_path) == expected


def test_streamed_size_propagates_scan_failure(tmp_path, monkeypatch):
    def failing(*args, **kwargs):
        raise OSError("archive unavailable")
    monkeypatch.setattr(guard.os, "scandir", failing)
    with pytest.raises(OSError, match="archive unavailable"):
        guard.archive_size(tmp_path)


def test_no_workspace_fallback_or_missing_mount(tmp_path):
    with pytest.raises(ValueError,match='workspace'):
        guard.check_capacity(tmp_path,0,workspace=tmp_path)
    with pytest.raises(FileNotFoundError):
        guard.allocate_run(tmp_path/'missing',limit=100,reserve=0)
    assert list(tmp_path.iterdir())==[]


def test_capacity_failure_preserves_existing_data(tmp_path,monkeypatch):
    Usage=namedtuple('Usage','total used free')
    monkeypatch.setattr(guard.shutil,'disk_usage',lambda _:Usage(100,90,10))
    with pytest.raises(ValueError,match='reserve'):
        guard.allocate_run(tmp_path,limit=20,reserve=10,external=False)
    assert list(tmp_path.iterdir())==[]


def test_unique_runs_and_quota_never_delete(tmp_path):
    a=guard.allocate_run(tmp_path,limit=4096,reserve=0,external=False)
    b=guard.allocate_run(tmp_path,limit=4096,reserve=0,external=False)
    assert a!=b and guard.check_run(a)<4096
    payload=a/'payload'; payload.write_bytes(b'x'*4096)
    with pytest.raises(ValueError,match='quota'):
        guard.check_run(a)
    assert payload.read_bytes()==b'x'*4096


def test_io_error_does_not_create_fallback_or_retry(tmp_path,monkeypatch):
    original=Path.mkdir
    attempts=[]
    def failing(path,*args,**kwargs):
        if path.name.startswith("revision-"):
            attempts.append(path)
            raise OSError(5,"Input/output error")
        return original(path,*args,**kwargs)
    monkeypatch.setattr(Path,"mkdir",failing)
    with pytest.raises(OSError,match="Input/output"):
        guard.allocate_run(tmp_path,limit=4096,reserve=0,external=False)
    assert len(attempts)==1 and list(tmp_path.iterdir())==[]


def test_installed_module_does_not_define_workspace_device(tmp_path, monkeypatch):
    workspace = tmp_path / "workspace"
    (workspace / "astrobot-lab/src/slam_mapping").mkdir(parents=True)
    (workspace / "astrobot-lab/src/slam_mapping/package.xml").write_text("test marker")
    (workspace / "docker-compose.lunaryard.yml").write_text("test marker")
    monkeypatch.setattr(guard, "__file__", str(tmp_path / "archive/install/lib/slam_mapping/archive_guard.py"))
    monkeypatch.setenv("ASTROBOT_WORKSPACE", str(workspace))
    assert guard.workspace_reference() == workspace
    with pytest.raises(ValueError, match="workspace filesystem"):
        guard.check_capacity(tmp_path, 0)  # Still refuses a same-device fallback.


def test_unresolvable_explicit_workspace_is_not_permission_to_skip_check(tmp_path, monkeypatch):
    monkeypatch.setenv("ASTROBOT_WORKSPACE", str(tmp_path / "missing"))
    with pytest.raises(ValueError, match="cannot resolve real workspace"):
        guard.check_capacity(tmp_path, 0)


def test_simulator_partial_workspace_mount_is_recognized(tmp_path, monkeypatch):
    workspace = tmp_path / "omnilrs"
    (workspace / "cfg/environment").mkdir(parents=True)
    (workspace / "run.py").write_text("test marker")
    (workspace / "cfg/environment/lunaryard_40m.yaml").write_text("test marker")
    monkeypatch.setenv("ASTROBOT_WORKSPACE", str(workspace))
    assert guard.workspace_reference() == workspace
    with pytest.raises(ValueError, match="workspace filesystem"):
        guard.check_capacity(tmp_path, 0)


def test_container_root_fallback_is_rejected_even_when_workspace_is_a_bind_mount(tmp_path, monkeypatch):
    from types import SimpleNamespace
    original = Path.stat
    workspace = tmp_path / "bind_workspace"
    def stat(path, *args, **kwargs):
        if path == workspace:
            return SimpleNamespace(st_dev=-12345)
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, "stat", stat)
    # pytest's tmp directory resides on root in our host/container test runner.
    if tmp_path.stat().st_dev != Path('/').stat().st_dev:
        pytest.skip("test temp root is a separate filesystem")
    with pytest.raises(ValueError, match="root filesystem"):
        guard.check_capacity(tmp_path, 0, workspace=workspace)
