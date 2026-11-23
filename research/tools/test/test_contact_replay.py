import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay_contact_advisory import replay_policy, summarize, without_timing


def test_candidate_changes_only_requested_policy_and_exact_replay_forbids_overrides():
    source = dict(support_core=False, startup_turn_area=True)
    assert replay_policy(source, candidate=True, support_core=True) == dict(support_core=True, startup_turn_area=True)
    assert replay_policy(source, candidate=False) == source
    with pytest.raises(ValueError, match="candidate mode"):
        replay_policy(source, candidate=False, startup_turn_area=False)


def test_timing_is_excluded_from_exact_geometry_comparison_but_not_decisions():
    assert without_timing(dict(state="available", update_ms=2, total_ms=3)) == dict(state="available")
    assert without_timing(dict(state="unknown", update_ms=2, total_ms=3)) != dict(state="available")


def test_current_turn_policy_requires_explicit_candidate_or_recorded_setting():
    assert replay_policy({}, candidate=True, current_turn_area=True)["current_turn_area"] is True
    assert replay_policy({"current_turn_area": True}, candidate=False)["current_turn_area"] is True
    with pytest.raises(ValueError, match="candidate"):
        replay_policy({}, candidate=False, current_turn_area=True)


def test_missing_maps_rejected_and_offline_timings_never_added_to_live_mapper():
    row = dict(
        valid=True,
        query_ns=200_000_000,
        evidence_ns=0,
        history_anchors=1,
        lidar_confirmed_wheels=[False] * 4,
        update_ms=2,
        total_ms=3,
        motions={"stationary": dict(state="available"), "forward_5cm": dict(state="unknown")},
    )
    records = [dict(evidence_ns=0, compute_ms=10), dict(evidence_ns=100_000_000, compute_ms=11)]
    live = summarize([row], records)
    assert live["movement"]["all"]["short_move_or_turn_available"] == 0
    assert live["timing"]["mapper_plus_advisory_ms"]["max"] == 13
    offline = summarize([row], records, offline=True)
    assert "mapper_plus_advisory_ms" not in offline["timing"]
    assert offline["timing_origin"].startswith("offline")
    with pytest.raises(ValueError, match="missing"):
        summarize([], records)
