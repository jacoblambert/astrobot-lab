from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from selected_native_audit import validate_native


def test_origin_counterfactuals_do_not_replace_first_hit_or_modify_evidence():
    from copy import deepcopy
    from selected_native_audit import origin_counterfactuals

    outlier = dict(ray_direction=[1.0, 0.0, 0.0], native_depth_m=0.9, first_hit_residual_m=0.4)
    original = deepcopy(outlier)

    def cast(origin, direction, distance):
        return [
            (0.0, origin, (-1.0, 0.0, 0.0), "/wheel"),
            (0.9 - origin[0], (0.9, 0.0, 0.0), (0.0, 0.0, 1.0), "/terrain"),
        ]

    rows = origin_counterfactuals(outlier, [0.0, 0.0, 0.0], cast)
    assert outlier == original
    assert [r["offset_m"] for r in rows] == [0.0, 0.49, 0.51]
    assert all(r["hits"][0]["collision"] == "/wheel" and r["diagnostic_only"] for r in rows)


def test_loop_duration_extension_requires_explicit_matching_route():
    from selected_lidar_profile import acquisition_limits

    a = fixture()
    base = a["frames"][0]
    a["frames"] = [
        dict(
            base,
            observed_stamp=10 + i / 60,
            world_time=2 + i / 60,
            timeline_time=10 + i / 60,
            producer_sequence=480 + i,
            world_step=120 + i,
        )
        for i in range(12002)
    ]
    with pytest.raises(ValueError, match="duration"):
        validate_native(a)
    with pytest.raises(ValueError, match="bounds"):
        validate_native(a, motion="loop16")
    a["acquisition_limits"] = acquisition_limits("loop16")
    assert validate_native(a, motion="loop16")["chunks"] == 12002
    with pytest.raises(ValueError, match="bounds"):
        validate_native(a, motion="straight_slow")


def fixture():
    return dict(
        profile="selected_generic_v1",
        duplicate_native_reads=0,
        frames=[
            dict(
                observed_stamp=10 + i / 60,
                world_time=2 + i / 60,
                timeline_time=10 + i / 60,
                producer_sequence=i + 480,
                world_step=i + 120,
                samples=10661,
                ray_layout="per_return_generic",
                world_physics_dt=1 / 60,
                world_rendering_dt=1 / 60,
                current_pose=dict(selected=128, missing=0, residuals_m=[0.0001] * 128, nonhorizontal_surface_samples=1),
            )
            for i in range(361)
        ],
    )


def test_whole_motion_native_cadence_and_first_hit_contract():
    report = validate_native(fixture())
    assert report["chunks"] == 361 and report["samples"] == 361 * 128
    assert report["maximum_m"] == 0.0001


def test_missing_reference_hit_is_not_reported_as_a_large_pose_residual():
    audit = fixture()
    surface = audit["frames"][100]["current_pose"]
    surface["missing"] = 1
    surface["residuals_m"].pop()
    with pytest.raises(ValueError, match="tick 100.*missing_hits=1.*successful_residuals=127/128.*not GLIM error"):
        validate_native(audit)


def test_corrected_profile_requires_explicit_matching_protocol():
    audit = fixture()
    audit["profile"] = "selected_optical_v2"
    with pytest.raises(ValueError, match="identity"):
        validate_native(audit)
    assert validate_native(audit, profile="selected_optical_v2")["chunks"] == 361
    audit["frames"][10]["current_pose"]["missing"] = 1
    with pytest.raises(ValueError, match="first-hit"):
        validate_native(audit, profile="selected_optical_v2")


@pytest.mark.parametrize(
    "fault", ["gap", "world_clock", "producer", "physics", "dose", "layout", "surface", "missing", "short"]
)
def test_selected_native_cannot_hide_missing_ticks_or_bad_poses(fault):
    audit = fixture()
    frame = audit["frames"][100]
    if fault == "gap":
        audit["frames"].pop(100)
    elif fault == "world_clock":
        frame["world_time"] += 0.01
    elif fault == "producer":
        frame["producer_sequence"] += 1
    elif fault == "physics":
        frame["world_step"] -= 1
    elif fault == "dose":
        frame["samples"] -= 1
    elif fault == "layout":
        frame["ray_layout"] = "legacy_columns"
    elif fault == "surface":
        frame["current_pose"]["residuals_m"][1] = 0.002
    elif fault == "missing":
        frame["current_pose"]["missing"] = 1
    else:
        audit["frames"] = audit["frames"][:100]
    with pytest.raises(ValueError):
        validate_native(audit)
