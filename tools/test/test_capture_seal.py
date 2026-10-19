"""Slow post-motion sealing must not starve real physics/clock callbacks."""

import importlib
import json
import sys
from pathlib import Path
from threading import Event
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_seal import BackgroundSeal


@pytest.fixture
def capture_module(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[2] / "OmniLRS"))
    return importlib.import_module("src.environments_wrappers.ros2.natural_holdout_capture")


def test_slow_seal_allows_actual_capture_ticks_but_no_new_native_acquisition(tmp_path, capture_module):
    entered, release = Event(), Event()
    owner = capture_module.NaturalHoldoutCapture.__new__(capture_module.NaturalHoldoutCapture)
    owner.done, owner.failure = False, None
    sensor_ticks = []
    owner.selected_lidar = SimpleNamespace(tick=lambda stamp, pose: sensor_ticks.append(stamp))
    owner.pose_method = lambda _: (None, None, None, "actual_pose")
    owner.timed_call = lambda name, function, *args: function(*args)
    owner._tick = lambda _: pytest.fail("acquisition/driver resumed while sealing")

    def seal():
        entered.set()
        assert release.wait(5)

    owner.sealing = BackgroundSeal(seal)
    task = owner.sealing
    try:
        assert entered.wait(2)
        for i in range(60):
            owner.tick(10 + i / 60)
        assert len(sensor_ticks) == 60 and not owner.done
    finally:
        release.set()
        task.thread.join(2)
    owner.run = tmp_path
    owner.tick(11)
    assert owner.done and owner.sealing is None and owner.failure is None


@pytest.mark.parametrize("failure", [False, True])
def test_success_marker_waits_for_files_and_receiver_cleanup(tmp_path, capture_module, failure):
    entered, release = Event(), Event()
    cleaned = []
    (tmp_path / "acquisition_finished.json").write_text(json.dumps(dict(commands_stopped=True)))
    driver = SimpleNamespace(stop=lambda: pytest.fail("driver already stopped on physics thread"), reached=[])

    def close():
        entered.set()
        assert release.wait(5)
        if failure:
            raise OSError("fixture disk error")

    native = SimpleNamespace(close=close, frames=[None] * 10)
    task = BackgroundSeal(
        lambda: capture_module.seal_moving_capture(
            tmp_path, {}, 1.0, 2.0, native, driver, announced=True, finalize=lambda: cleaned.append(True)
        )
    )
    try:
        assert entered.wait(2)
        assert not task.poll()
        assert not (tmp_path / "natural_holdout.json").exists()
    finally:
        release.set()
        task.thread.join(2)
    assert cleaned == [True]
    if failure:
        with pytest.raises(RuntimeError, match="sealing failed"):
            task.poll()
        assert not (tmp_path / "natural_holdout.json").exists()
    else:
        assert task.poll()
        assert json.loads((tmp_path / "natural_holdout.json").read_text())["complete"]


def test_existing_failure_cannot_be_overwritten_as_success(tmp_path, capture_module):
    (tmp_path / "failure.json").write_text("{}")
    native = SimpleNamespace(close=lambda: None, frames=[])
    with pytest.raises(ValueError, match="failed capture"):
        capture_module.seal_moving_capture(tmp_path, {}, 1.0, 2.0, native, None, announced=True)
    assert not (tmp_path / "natural_holdout.json").exists()


def test_deadline_is_bounded_without_blocking_or_resetting(monkeypatch):
    import capture_seal

    release = Event()
    now = [10.0]
    monkeypatch.setattr(capture_seal.time, "monotonic", lambda: now[0])
    task = BackgroundSeal(lambda: release.wait(5))
    try:
        assert not task.poll()
        now[0] = 30.01
        with pytest.raises(TimeoutError):
            task.poll()
    finally:
        release.set()
        task.thread.join(2)
    with pytest.raises(TimeoutError):
        task.poll()  # Late completion does not revive the expired deadline.
    with pytest.raises(ValueError):
        BackgroundSeal(lambda: None, deadline_s=21)


def test_native_completion_is_once_and_audit_metadata_freezes_before_worker(tmp_path, capture_module):
    module = importlib.import_module("src.environments_wrappers.ros2.selected_lidar_capture")
    native_module = importlib.import_module("src.environments_wrappers.ros2.terrain_native_capture")
    native = module.SelectedNativeArchive.__new__(module.SelectedNativeArchive)
    messages = []
    native.completion_announced, native.audit_metadata = False, None
    diagnostics = dict(ticks=100, hits=1000, hits_over_1mm=0, max_residual_m=0.0002)
    native.sensor = SimpleNamespace(
        run=tmp_path,
        adapter=SimpleNamespace(
            association_diagnostics=diagnostics, timestamp_policy="fixture", stamp_ns=lambda stamp: round(stamp * 1e9)
        ),
    )
    native.frames = native_module.EncodedFrames()
    native.frames.append(dict(observed_stamp=2.0))
    native.previous, native.limits = (2.0, None), dict(max_native_frames=100)
    native.output = tmp_path / "native_chunks"
    native.native_publisher = True
    native.completion_publisher = SimpleNamespace(publish=lambda message: messages.append(json.loads(message.data)))
    native.completion_message = lambda **kwargs: SimpleNamespace(**kwargs)
    native.packed = SimpleNamespace(close=lambda: None)
    native.announce_complete()
    native.announce_complete()
    diagnostics["ticks"] = 110  # Real post-acquisition scans may continue for GLIM.
    native.close()
    assert messages == [dict(capture=tmp_path.name, sequence=0, stamp_ns=2_000_000_000)]
    report = json.loads((tmp_path / "native_timing_audit.json").read_text())
    assert report["producer_association_diagnostics"]["ticks"] == 100
    assert report["frames"] == [dict(observed_stamp=2.0)]
    with pytest.raises(ValueError, match="already closed"):
        native.observe(3.0, None, None)
