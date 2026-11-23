import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import assess_step_trial as audit
from test_step_trial import advisory


def fixture(monkeypatch, tmp_path, mutation=None):
    source, capture, output = [tmp_path / name for name in ("mapper", "capture", "audit")]
    for path in (source, capture, output):
        path.mkdir()
    planner = audit.StepTrial("capture")
    contexts = [advisory(1_000_000_000), advisory(2_000_000_000, x=0.29)]
    reference = np.eye(4)
    reference[:2, 3] = [-40, -38]
    for context in contexts:
        context.record["local_from_map"] = reference.copy()
    proposals = [planner.update(a, a.context.query_ns) for a in contexts]
    start = np.eye(4)
    start[:2, 3] = [20, 18]
    request = dict(pattern="terrain_step", capture="capture", home_xyz=[-20, -20, 0], world_from_base=start.tolist())
    rows = []
    for value in proposals:
        gt = np.linalg.inv(reference) @ np.asarray(value["local_from_base"])
        gt[:3, 3] += request["home_xyz"]
        rows.append(
            dict(
                stamp=value["query_ns"] / 1e9,
                proposal=value,
                linear=value["linear"],
                angular=value["angular"],
                world_from_base=gt.tolist(),
            )
        )
    if mutation == "command":
        rows[0]["linear"] = 0.03
    if mutation in ("waiting", "moving_while_waiting", "future_wait"):
        rows[0].update(execution_state="waiting_for_fresh_proposal", proposal_wall_age_s=0.56)
        if mutation in ("waiting", "future_wait"):
            rows[0]["linear"] = 0.0
        if mutation == "future_wait":
            rows[0].update(stamp=0.983333333, proposal_wall_age_s=0.01)
    if mutation == "missing_execution":
        rows = []
    (source / "manifest.json").write_text(json.dumps(dict(request=request)))
    (source / "step_trial.jsonl").write_text("".join(json.dumps(p) + "\n" for p in proposals))
    (source / "step_trial_collection.json").write_text(
        json.dumps(
            dict(
                completed=True,
                proposals=2,
                steps=0,
                outcome="estimated_goal_reached",
                output_sha256=audit.sha(source / "step_trial.jsonl"),
            )
        )
    )
    (capture / "natural_holdout.json").write_text(json.dumps(dict(complete=mutation != "failed_capture")))
    (capture / "motion.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (capture / "step_trial_outcome.json").write_text(json.dumps(dict(outcome="estimated_goal_reached")))

    def replay(source, archive, output, *, observation_callback):
        for a in contexts:
            if mutation == "replay":
                a.context.local_from_base[0, 3] += 0.001
            observation_callback(SimpleNamespace(advisory=a), None, None, dict(query_ns=a.context.query_ns))
        return dict(exact=True)

    monkeypatch.setattr(audit, "evaluate", replay)
    monkeypatch.setattr(audit, "check_run", lambda _: None)
    monkeypatch.setattr(audit, "allocate_run", lambda *args, **kwargs: output)
    monkeypatch.setattr(sys, "argv", ["audit", str(source), "--archive-root", str(tmp_path)])
    return output


def test_audit_separates_actual_progress_from_estimated_completion(monkeypatch, tmp_path):
    output = fixture(monkeypatch, tmp_path)
    audit.main()
    result = json.loads((output / "report.json").read_text())
    assert result["outcome"] == "estimated_goal_reached"
    assert result["gt_forward_progress_m"] == pytest.approx(0.29)
    assert result["consumer_time_xy_error_p50_p95_max_m"] == [0, 0, 0]
    assert result["controller_proposals_compared"] == 2
    assert result["nonzero_command_ticks"] == 1
    assert result["gt_distance_to_fixed_goal_m"] == pytest.approx(0.01)
    assert result["command_seconds"] == dict(forward=1, reverse=0, turn=0, waiting=0, hold=0)
    assert not result["navigation_qualified"]


@pytest.mark.parametrize("mutation", ["waiting", "future_wait"])
def test_zero_command_waits_are_preserved_in_the_denominator(monkeypatch, tmp_path, mutation):
    output = fixture(monkeypatch, tmp_path, mutation)
    audit.main()
    result = json.loads((output / "report.json").read_text())
    assert result["execution_ticks"] == 2 and result["waiting_ticks"] == 1
    assert result["nonzero_command_ticks"] == 0


def test_route_measurements_evaluate_home_and_rock_without_fitted_alignment():
    pose = np.eye(4)
    proposal = dict(
        local_from_map=np.eye(4).tolist(),
        local_from_base=pose.tolist(),
        goals_local_xy=[[2, 0], [0, 0]],
        reached_goals=[dict(index=0), dict(index=1)],
    )
    first = pose.copy()
    first[0, 3] = 1.93
    last = pose.copy()
    last[0, 3] = 0.06
    rows = [
        dict(stamp=1, proposal=dict(proposal, reached_goals=[dict(index=0)]), world_from_base=first.tolist()),
        dict(stamp=2, proposal=proposal, world_from_base=last.tolist()),
    ]
    rock = dict(objects=[dict(points=[[4, -0.2, 0], [4.4, -0.2, 0], [4.4, 0.2, 0.4], [4, 0.2, 0]])])
    result = audit.route_measurements(rows, dict(home_xyz=[0, 0, 0]), rock)
    assert [r["gt_goal_error_m"] for r in result["reached"]] == pytest.approx([0.07, 0.06])
    assert result["minimum_inflated_footprint_to_demo_rock_xy_m"] == pytest.approx(4 - 1.93 - 0.85)


@pytest.mark.parametrize(
    "mutation", ["command", "missing_execution", "failed_capture", "replay", "moving_while_waiting"]
)
def test_failed_or_changed_evidence_cannot_be_scored(monkeypatch, tmp_path, mutation):
    output = fixture(monkeypatch, tmp_path, mutation)
    with pytest.raises(ValueError):
        audit.main()
    assert not (output / "report.json").exists()
