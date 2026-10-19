"""Run the actual write-once audit check without importing Isaac/ROS."""

import ast
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


def method():
    path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/natural_holdout_capture.py"
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "check_stationary_audit")
    env = dict(json=json, hashlib=hashlib)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), env)
    return env[node.name]


def test_completed_audit_is_decoded_once_and_exact_bytes_are_sealed():
    payload = json.dumps(dict(passed=True, extrinsic_tf_checked=True, frames=[{"hit": [1.0, 2.0, 3.0]}])).encode()
    calls = []

    class Run:
        def __truediv__(self, name):
            assert name == "sensor_audit.json"
            return SimpleNamespace(read_bytes=lambda: calls.append(name) or payload)

    owner = SimpleNamespace(run=Run(), sensor_audit_sha256=None)
    check = method()
    for _ in range(10000):
        check(owner)
    assert calls == ["sensor_audit.json"]
    assert owner.sensor_audit_sha256 == hashlib.sha256(payload).hexdigest()


@pytest.mark.parametrize("payload", [{}, {"passed": False, "extrinsic_tf_checked": True}, {"passed": True}])
def test_failed_or_unaudited_geometry_never_gets_a_cached_pass(tmp_path, payload):
    (tmp_path / "sensor_audit.json").write_text(json.dumps(payload))
    owner = SimpleNamespace(run=tmp_path, sensor_audit_sha256=None)
    with pytest.raises(ValueError):
        method()(owner)
    assert owner.sensor_audit_sha256 is None
