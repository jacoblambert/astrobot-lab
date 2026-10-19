import importlib.util
from pathlib import Path
from types import SimpleNamespace
import subprocess

import pytest

path = Path(__file__).resolve().parents[2] / "docker/check_cuda_runtime.py"
spec = importlib.util.spec_from_file_location("cuda_runtime", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fake_driver(failure=None, count=1):
    calls = []
    driver = SimpleNamespace()
    for name in ("cuInit", "cuDeviceGetCount", "cuDeviceGet", "cuCtxCreate_v2",
                 "cuMemAlloc_v2", "cuCtxSynchronize", "cuMemFree_v2", "cuCtxDestroy_v2"):
        def invoke(*args, name=name):
            calls.append(name)
            if name == "cuDeviceGetCount":
                args[0]._obj.value = count
            return 999 if name == failure else 0
        setattr(driver, name, invoke)
    return driver, calls


def test_cuda_probe_checks_allocation_and_releases_its_context():
    driver, calls = fake_driver()
    assert module.probe_cuda(lambda _: driver)["passed"]
    assert calls[-3:] == ["cuCtxSynchronize", "cuMemFree_v2", "cuCtxDestroy_v2"]


@pytest.mark.parametrize("failure,cleanup", [
    ("cuInit", []), ("cuCtxCreate_v2", []), ("cuMemAlloc_v2", ["cuCtxDestroy_v2"]),
    ("cuCtxSynchronize", ["cuMemFree_v2", "cuCtxDestroy_v2"]),
    ("cuMemFree_v2", ["cuCtxDestroy_v2"]), ("cuCtxDestroy_v2", []),
])
def test_cuda_errors_fail_closed_and_cleanup_owned_resources(failure, cleanup):
    driver, calls = fake_driver(failure)
    with pytest.raises(RuntimeError, match=failure):
        module.probe_cuda(lambda _: driver)
    assert calls[calls.index(failure) + 1:] == cleanup


def test_zero_devices_cannot_pass():
    driver, calls = fake_driver(count=0)
    with pytest.raises(RuntimeError, match="no accessible"):
        module.probe_cuda(lambda _: driver)
    assert "cuCtxCreate_v2" not in calls


def test_hung_driver_is_bounded():
    def hung(command, **kwargs):
        assert kwargs["timeout"] == 15
        raise subprocess.TimeoutExpired(command, kwargs["timeout"])
    assert not module.bounded_probe(hung)["passed"]


@pytest.mark.parametrize("code,output", [(1, ""), (0, "garbage"), (0, "null"), (0, '{"passed":true}')])
def test_crashed_or_incomplete_worker_is_ineligible(code, output):
    assert not module.bounded_probe(lambda *a, **k: SimpleNamespace(
        returncode=code, stdout=output, stderr=""))["passed"]


def test_launcher_requires_probe_before_starting_isaac():
    launcher = path.with_name("run_omnilrs.sh").read_text()
    assert "set -euo pipefail" in launcher
    assert launcher.index("/isaac-sim/python.sh /workspace/astrobot-lab/docker/check_cuda_runtime.py") < launcher.index(
        "exec /isaac-sim/python.sh /workspace/omnilrs/run.py")
