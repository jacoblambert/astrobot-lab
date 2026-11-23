import dataclasses
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.motion_evaluation import MotionCase, MotionDecision, score_motions


def inputs():
    cases = [MotionCase(str(i), "flat" if i < 100 else "turn", str(i // 10), 1000, "s/0/1") for i in range(202)]
    reference = {c.case_id: "blocked" if c.case_id in ("99", "201") else "feasible" for c in cases}
    decisions = [MotionDecision(c.case_id, reference[c.case_id], 800, 900, c.revision, True) for c in cases]
    return cases, reference, decisions


def score(cases, reference, decisions, **kwargs):
    return score_motions(
        cases,
        reference,
        decisions,
        required_families=("flat", "turn"),
        max_age_ns=250,
        source_completed=kwargs.pop("source_completed", True),
        **kwargs
    )


def test_perfect_fixture_is_only_a_metric_pass_not_qualification():
    report = score(*inputs())
    assert report["motion_metrics_pass"]
    assert report["overall"]["safe_motion_availability"] == 1
    assert report["physical_exposures"] == 21
    assert not report["acceptance"] and not report["independent_geometry_validated_by_this_module"]


def test_always_unknown_fails_usefulness():
    cases, reference, decisions = inputs()
    decisions = [dataclasses.replace(d, state="unknown") for d in decisions]
    report = score(cases, reference, decisions)
    assert not report["motion_metrics_pass"]
    assert report["overall"]["unnecessary_stop"] == 200


def test_bad_family_cannot_hide_in_aggregate():
    cases, reference, decisions = inputs()
    decisions[:6] = [dataclasses.replace(d, state="unknown") for d in decisions[:6]]
    report = score(cases, reference, decisions)
    assert report["overall"]["safe_motion_availability"] > 0.95
    assert report["families"]["flat"]["safe_motion_availability"] < 0.95
    assert not report["motion_metrics_pass"]


@pytest.mark.parametrize(
    "change",
    [
        "hazard",
        "missing_decision",
        "missing_reference",
        "unknown_reference",
        "failed_source",
        "stale",
        "future",
        "invalid",
        "revision",
        "no_hazards",
    ],
)
def test_incomplete_or_unsafe_results_never_pass(change):
    cases, reference, decisions = inputs()
    kwargs = {}
    if change == "hazard":
        decisions[99] = dataclasses.replace(decisions[99], state="feasible")
    elif change == "missing_decision":
        decisions.pop()
    elif change == "missing_reference":
        reference.pop("0")
    elif change == "unknown_reference":
        reference["0"] = "unknown"
    elif change == "failed_source":
        kwargs["source_completed"] = False
    elif change == "stale":
        decisions[0] = dataclasses.replace(decisions[0], evidence_ns=700)
    elif change == "future":
        decisions[0] = dataclasses.replace(decisions[0], publication_ns=1001)
    elif change == "invalid":
        decisions[0] = dataclasses.replace(decisions[0], valid=False)
    elif change == "revision":
        decisions[0] = dataclasses.replace(decisions[0], revision="s/1/1")
    else:
        reference = {key: "feasible" for key in reference}
    report = score(cases, reference, decisions, **kwargs)
    assert not report["motion_metrics_pass"]
    assert report["overall"]["cases"] == 202


@pytest.mark.parametrize(
    "change",
    [
        "duplicate_case",
        "duplicate_decision",
        "extra_reference",
        "extra_decision",
        "missing_family",
        "invalid_label",
        "empty",
        "relax_target",
    ],
)
def test_contract_changes_rejected(change):
    cases, reference, decisions = inputs()
    kwargs = {}
    if change == "duplicate_case":
        cases.append(cases[0])
    elif change == "duplicate_decision":
        decisions.append(decisions[0])
    elif change == "extra_reference":
        reference["extra"] = "feasible"
    elif change == "extra_decision":
        decisions.append(dataclasses.replace(decisions[0], case_id="extra"))
    elif change == "missing_family":
        cases = [dataclasses.replace(c, family="flat") for c in cases]
    elif change == "invalid_label":
        reference["0"] = "probably_free"
    elif change == "empty":
        cases, reference, decisions = [], {}, []
    else:
        kwargs["target"] = 0.90
    with pytest.raises(ValueError):
        score(cases, reference, decisions, **kwargs)
