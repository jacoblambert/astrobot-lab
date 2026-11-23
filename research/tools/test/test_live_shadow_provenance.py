"""Pure host source-identity contracts; no archive, ROS or simulator."""
import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from live_terrain_shadow import sha_file
from replay_live_terrain_shadow import (RUNTIME_MODULES, verify_runtime_sources,
                                      validate_collection, validate_terminal_fault)


def manifest():
    return {"source_sha256": {str(Path(__file__).parents[1] / name):
                             sha_file(Path(__file__).parents[1] / name) for name in RUNTIME_MODULES}}


def test_same_runtime_sources_pass():
    verify_runtime_sources(manifest())


@pytest.mark.parametrize("mutation", ["changed", "missing", "ambiguous"])
def test_frozen_files_do_not_allow_a_different_replay_runtime(mutation):
    value = manifest()
    key = next(iter(value["source_sha256"]))
    if mutation == "changed": value["source_sha256"][key] = "0" * 64
    if mutation == "missing": del value["source_sha256"][key]
    if mutation == "ambiguous": value["source_sha256"]["/elsewhere/" + Path(key).name] = value["source_sha256"][key]
    with pytest.raises(ValueError): verify_runtime_sources(value)


def failed_collection():
    return dict(experiment_completed=False, status=dict(completed=False, failure="simulation wall heartbeat lost"))


def test_failed_prefix_is_never_ordinary_replay():
    with pytest.raises(ValueError, match="failed/incomplete"):
        validate_collection(failed_collection())
    validate_collection(failed_collection(), True)
    validate_terminal_fault("simulation wall heartbeat lost", failed_collection(), True, False)


@pytest.mark.parametrize("mutation", ["completed", "core_completed", "other_fault", "missing_fault"])
def test_prefix_diagnosis_does_not_generalize_failed_input_eligibility(mutation):
    value = failed_collection()
    if mutation == "completed": value["experiment_completed"] = True
    if mutation == "core_completed": value["status"]["completed"] = True
    if mutation == "other_fault": value["status"]["failure"] = "native sequence gap"
    if mutation == "missing_fault": value["status"]["failure"] = None
    with pytest.raises(ValueError, match="heartbeat failure"):
        validate_collection(value, True)


@pytest.mark.parametrize("enabled,seen,payload", [
    (False, False, "simulation wall heartbeat lost"),
    (True, True, "simulation wall heartbeat lost"),
    (True, False, "native sequence gap"),
])
def test_terminal_fault_requires_explicit_matching_single_failure(enabled, seen, payload):
    with pytest.raises(ValueError, match="unexpected terminal fault"):
        validate_terminal_fault(payload, failed_collection(), enabled, seen)
