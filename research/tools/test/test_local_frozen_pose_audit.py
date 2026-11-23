from pathlib import Path
import json
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from local_terrain.frozen_pose_audit import exact_native_index, registration_errors
from diagnose_local_frozen_poses import evaluate
from replay_local_terrain import sha


def pose(x=0, yaw=0):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    value[0, 3] = x
    return value


def test_independent_world_gauges_cancel_without_fitting_and_mount_is_retained():
    mount = pose(0.4, 0.05)
    old = np.array([pose(0.1, 0.2), pose(0.2, 0.3)]) @ mount
    current = pose(0.7, 0.4)
    estimated_gauge, truth_gauge = pose(34, 0.7), pose(-7, -1.1)
    result = registration_errors(
        estimated_gauge @ current, estimated_gauge @ old, truth_gauge @ current, truth_gauge @ old
    )
    np.testing.assert_allclose(result, 0, atol=1e-7)


def test_old_sensor_vs_current_base_does_not_confuse_the_lidar_lever_arm():
    truth = np.array([pose(0.4)])
    good = registration_errors(pose(), truth, pose(), truth)
    bad = registration_errors(pose(), np.array([pose()]), pose(), truth)
    assert good[0, 0] == 0
    assert bad[0, 0] == pytest.approx(0.4)


def test_known_translation_and_rotation_bound_contains_transformed_points():
    old, truth = np.array([pose(0.05, 0.1)]), np.array([pose()])
    result = registration_errors(pose(), old, pose(), truth)[0]
    assert result[0] == pytest.approx(0.05) and result[1] == pytest.approx(0.1)
    for radius, column in [(2, 2), (3, 3)]:
        points = np.c_[radius * np.cos(np.arange(100)), radius * np.sin(np.arange(100)), np.zeros(100)]
        transformed = points @ old[0, :3, :3].T + old[0, :3, 3]
        assert np.max(np.linalg.norm(transformed - points, axis=1)) <= result[column]


@pytest.mark.parametrize("stamp", [50, 201, -10])
def test_missing_native_tick_never_interpolates(stamp):
    with pytest.raises(ValueError, match="same-native-tick"):
        exact_native_index(np.array([0, 100, 300]), stamp)


def test_bad_old_transform_rejected():
    bad = np.array([pose()])
    bad[0, 0, 0] = 2
    with pytest.raises(ValueError, match="nonrigid"):
        registration_errors(pose(), bad, pose(), np.array([pose()]))


def audit_inputs(tmp_path):
    local, accuracy, capture = [tmp_path / n for n in ("local", "accuracy", "capture")]
    for directory in (local, accuracy, capture):
        directory.mkdir()
    link = capture / "shadow_link.json"
    link.write_text(json.dumps(dict(request=dict(base_from_lidar=pose(0.4).tolist()))))
    arrays = accuracy / "accuracy_arrays.npz"
    truth = np.array([pose(i * 0.1) for i in range(3)])
    np.savez(arrays, times=np.arange(3) * 0.1, gt_base=truth)
    (accuracy / "accuracy_report.json").write_text(
        json.dumps(
            dict(
                diagnostic_completed=True,
                input_sha256={str(link): sha(link)},
                output_sha256={"accuracy_arrays.npz": sha(arrays)},
            )
        )
    )
    gauge = pose(10, 0.3)
    associations, rows = [], []
    for i in range(3):
        stamp = i * 100_000_000
        associations.append(
            dict(
                sequence=i,
                acquisition_ns=stamp,
                available_ns=stamp + 100_000_000,
                output_ns=stamp + 200_000_000,
                local_from_lidar=(gauge @ truth[i] @ pose(0.4)).tolist(),
            )
        )
        rows.append(
            dict(
                evidence_ns=stamp,
                publication_ns=stamp + 200_000_000,
                integrated_inputs=1,
                associations=associations.copy(),
                local_from_base=(gauge @ truth[i]).tolist(),
            )
        )
    projections = local / "projections.jsonl"
    projections.write_text("".join(json.dumps(r) + "\n" for r in rows))
    (local / "report.json").write_text(
        json.dumps(
            dict(
                diagnostic_completed=True,
                output_sha256=sha(projections),
                observation_policy="frozen_observation",
                map_policy="rolling_incremental",
                input_sha256={str(link): sha(link)},
                window_s=5,
                evidence_lifetime_s=60,
                projections=3,
                source_failed=True,
                bounded_prefix=False,
            )
        )
    )
    return local, accuracy, capture


def test_complete_audit_keeps_failed_source_and_counts_all_age_opportunities(tmp_path):
    report = evaluate(*audit_inputs(tmp_path))
    assert report["projections"] == 3 and report["pose_pairs"] == 6
    assert report["missing_gt_publications"] == 0 and report["source_failed"]
    assert not report["acceptance"] and not report["calibrated_uncertainty"]
    assert report["results"]["0_2s"]["metrics"]["origin_displacement_m"]["maximum"] < 1e-12


@pytest.mark.parametrize("fault", ["future", "changed_old", "dose", "hash"])
def test_audit_rejects_noncausal_changed_or_missing_evidence(tmp_path, fault):
    local, accuracy, capture = audit_inputs(tmp_path)
    projection = local / "projections.jsonl"
    rows = [json.loads(l) for l in projection.read_text().splitlines()]
    if fault == "future":
        rows[-1]["associations"][-1]["output_ns"] += 1
    elif fault == "changed_old":
        rows[-1]["associations"][0]["local_from_lidar"][0][3] += 0.1
    elif fault == "dose":
        rows[-1]["integrated_inputs"] = 0
    else:
        rows[-1]["local_from_base"][0][3] += 0.1
    projection.write_text("".join(json.dumps(r) + "\n" for r in rows))
    if fault != "hash":
        report = json.loads((local / "report.json").read_text())
        report["output_sha256"] = sha(projection)
        (local / "report.json").write_text(json.dumps(report))
    with pytest.raises(ValueError):
        evaluate(local, accuracy, capture)


def test_missing_gt_tick_is_counted_not_interpolated_or_trimmed(tmp_path):
    local, accuracy, capture = audit_inputs(tmp_path)
    path = accuracy / "accuracy_arrays.npz"
    np.savez(path, times=np.array([0.0, 0.10001, 0.2]), gt_base=np.array([pose(i * 0.1) for i in range(3)]))
    report = json.loads((accuracy / "accuracy_report.json").read_text())
    report["output_sha256"][path.name] = sha(path)
    (accuracy / "accuracy_report.json").write_text(json.dumps(report))
    result = evaluate(local, accuracy, capture)
    assert result["missing_gt_publications"] == 2 and result["projections"] == 3
    assert result["pose_pairs"] == 1 and result["acceptance"] is False


def test_nominal_query_grid_is_not_blindly_substituted_for_native_gt_time(tmp_path):
    local, accuracy, capture = audit_inputs(tmp_path)
    path = local / "projections.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    for row in rows:
        row["evidence_ns"] += 1000  # explicit, separate nominal query offset
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    report = json.loads((local / "report.json").read_text())
    report["output_sha256"] = sha(path)
    (local / "report.json").write_text(json.dumps(report))
    result = evaluate(local, accuracy, capture)
    assert result["maximum_query_minus_native_s"] == pytest.approx(1e-6)
    assert result["consumer_query_error_evaluated"] is False
    assert result["pose_pairs"] == 6 and result["missing_gt_publications"] == 0


def test_attribution_identifies_reference_walk_without_changing_first_map_poses(tmp_path):
    local, accuracy, capture = audit_inputs(tmp_path)
    path = local / "projections.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    for i, row in enumerate(rows):
        gauge = pose(i * 0.1)
        row["local_from_map"] = gauge.tolist()
        row["local_from_base"] = (gauge @ pose(i * 0.1)).tolist()
        for observation in row["associations"]:
            j = observation["sequence"]
            map_pose = pose(j * 0.1) @ pose(0.4)
            observation["map_from_lidar"] = map_pose.tolist()
            observation["local_from_lidar"] = (pose(j * 0.1) @ map_pose).tolist()
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    report = json.loads((local / "report.json").read_text())
    report["output_sha256"] = sha(path)
    (local / "report.json").write_text(json.dumps(report))
    original = evaluate(local, accuracy, capture)
    transported = evaluate(local, accuracy, capture, placement="current_gauge_of_first_map")
    assert original["results"]["0_2s"]["metrics"]["origin_displacement_m"]["maximum"] == pytest.approx(0.2)
    assert transported["results"]["0_2s"]["metrics"]["origin_displacement_m"]["maximum"] < 1e-12
    assert original["pose_pairs"] == transported["pose_pairs"] == 6
    assert transported["attribution_only"] and transported["acceptance"] is False
