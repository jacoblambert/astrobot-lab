"""Normal SIGINT finishes transactions; unexpected interrupts stay failures."""

import signal
import sys
import json
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from local_terrain.shutdown import DeferredInterrupt
from test_local_terrain import make


def test_sigint_is_deferred_through_callback_and_seal():
    previous = signal.getsignal(signal.SIGINT)
    core = make()
    core.completed = True
    with DeferredInterrupt() as stop:
        signal.raise_signal(signal.SIGINT)
        assert stop.requested
        core.close()  # Reached normally, not via BaseException fault.
        assert core.completed and core.failure is None and core.closed
        signal.raise_signal(signal.SIGINT)  # Repeated shutdown request is harmless.
    assert signal.getsignal(signal.SIGINT) == previous


@pytest.mark.parametrize("error", [KeyboardInterrupt(), ValueError(), ""])
def test_empty_exception_is_sticky_failure_not_success(error):
    core = make()
    core.completed = True
    core.fault(error)
    first = core.failure
    assert isinstance(first, str) and first
    assert core.status()["failure"] is not None and not core.status()["valid"]
    core.fault("later")
    assert core.failure == first
    core.close()
    assert core.failure == first


def test_failed_legacy_seal_cannot_enter_completed_counterfactual(tmp_path):
    from diagnose_costmap_refusal import diagnose

    for name in ("projections.jsonl", "step_trial.jsonl", "contact_manifest.json"):
        (tmp_path / name).write_text("{}")
    (tmp_path / "collection.json").write_text(json.dumps(dict(experiment_completed=True, status=dict(failure=""))))
    (tmp_path / "step_trial_collection.json").write_text(json.dumps(dict(completed=True)))
    with pytest.raises(ValueError, match="completed recording required"):
        diagnose(tmp_path)
