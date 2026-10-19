from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assess_ray_terrain import reference_masks, score, grouped_cases, range_regions
from local_terrain.ray_design import TERRAIN_VIEWS_XY, TERRAIN_YAWS_DEG, terrain_view_transform

WHEELS = [[x, y] for x in [-0.256, 0.256] for y in [-0.2854, 0.2854]]


@pytest.mark.parametrize("phase", [0, 0.025, 0.05, 0.075])
def test_range_denominators_are_disjoint_and_unknown_does_not_disappear(phase):
    regions = range_regions(phase)
    bands = [regions[f"range_{i}_{i+1}m"] for i in range(3)]
    assert not ((bands[0] & bands[1]) | (bands[1] & bands[2]) | (bands[0] & bands[2])).any()
    result = score(np.full((60, 60, 6), -1.0), [], phase, WHEELS)
    for name, mask in regions.items():
        assert result["regions"][name]["reference_free_cells"] == mask.sum()
        assert result["regions"][name]["unknown_free_cells"] == mask.sum()
        assert result["regions"][name]["free_fraction"] == 0


@pytest.mark.parametrize("phase", [0, 0.025, 0.05, 0.075])
def test_partial_object_cell_is_never_reference_free(phase):
    box = ([0.013, 0.017, 0], [0.113, 0.117, 0.08])
    free, objects = reference_masks([box], phase)
    assert (~free).sum() >= 4
    np.testing.assert_array_equal(~free, objects[0])
    layers = np.zeros((60, 60, 6))
    result = score(layers, [box], phase, WHEELS)
    assert result["false_free_cells"] == int((~free).sum())
    assert result["obstacle_objects_detected"] == 0


def test_unknown_map_does_not_pass_as_free_or_detected():
    layers = np.full((60, 60, 6), -1.0)
    result = score(layers, [([1, 0, 0], [1.1, 0.1, 0.08])], 0, WHEELS)
    assert result["false_free_cells"] == 0 and result["regions"]["all"]["correctly_free_cells"] == 0
    assert result["obstacle_objects_detected"] == 0


def test_cost20_inferred_or_rough_support_counts_for_availability_and_false_free():
    layers = np.zeros((60, 60, 6))
    layers[:, :, 0] = 20
    box = ([1.01, 0.01, 0], [1.11, 0.11, 0.08])
    result = score(layers, [box], 0, WHEELS)
    assert result["regions"]["all"]["free_fraction"] == 1
    assert result["false_free_cells"] == 4


def test_unknown_class_is_rejected():
    with pytest.raises(ValueError):
        score(np.full((60, 60, 6), 50), [], 0, WHEELS)


def multiview_fixture():
    cases = []
    for layout in ("baseline", "mid_dense_10_to_-60"):
        for scene in ("flat", "boxes"):
            for i, xy in enumerate(TERRAIN_VIEWS_XY):
                pose = np.eye(4)
                pose[:2, 3] = xy
                cases.append(
                    dict(
                        layout=layout,
                        scene=scene,
                        view_index=i,
                        base_translation_xy_m=xy,
                        actual_world_from_lidar=pose,
                        geometry_passed=True,
                        frames=[dict(stamp=i + j / 60) for j in range(12)],
                    )
                )
    return dict(cases=cases), dict(
        protocol="generic_ray_layout_static_multiview_v2_triangle_boxes", base_translations_xy_m=TERRAIN_VIEWS_XY
    )


def test_yaw_protocol_preserves_mount_lever_arm_and_rejects_sensor_center_spin():
    initial = np.eye(4)
    initial[:3, 3] = [0.4, 0, 0.395]
    cases = []
    for scene in ("flat", "boxes"):
        for i, yaw in enumerate(TERRAIN_YAWS_DEG):
            cases.append(
                dict(
                    layout="mid_dense_10_to_-60",
                    scene=scene,
                    view_index=i,
                    base_translation_xy_m=[0, 0],
                    base_yaw_deg=yaw,
                    geometry_passed=True,
                    actual_world_from_lidar=terrain_view_transform(initial, [0, 0], yaw),
                    frames=[dict(stamp=i + j / 60) for j in range(12)],
                )
            )
    manifest = dict(
        protocol="generic_ray_layout_settled_yaw_v1",
        base_yaws_deg=TERRAIN_YAWS_DEG,
        base_translations_xy_m=[[0, 0]] * 24,
    )
    result = grouped_cases(dict(cases=cases), manifest, initial, False, True)
    assert len(result) == 2 and len(result[0]["frames"]) == 288
    assert result[0]["frames"][72]["yaw_deg"] == 90
    cases[6]["actual_world_from_lidar"][:2, 3] = [0.4, 0]
    with pytest.raises(ValueError):
        grouped_cases(dict(cases=cases), manifest, initial, False, True)


def test_multiview_preserves_all_native_stamps_and_poses():
    report, manifest = multiview_fixture()
    groups = grouped_cases(report, manifest, np.eye(4), True)
    assert len(groups) == 4 and all(len(g["frames"]) == 72 for g in groups)
    assert groups[0]["frames"][12]["stamp"] == 1
    np.testing.assert_array_equal(groups[0]["frames"][12]["expected_pose"][:2, 3], [0.1, 0])


@pytest.mark.parametrize("error", ["epoch_reset", "tilt", "missing", "duplicate", "reordered", "protocol", "offset"])
def test_invalid_multiview_fails_closed(error):
    report, manifest = multiview_fixture()
    if error == "epoch_reset":
        report["cases"][1]["frames"][0]["stamp"] = 0
    elif error == "tilt":
        report["cases"][1]["actual_world_from_lidar"][2, 0] = 0.01
    elif error == "missing":
        report["cases"].pop()
    elif error == "duplicate":
        report["cases"][1] = report["cases"][0]
    elif error == "reordered":
        report["cases"][0], report["cases"][1] = report["cases"][1], report["cases"][0]
    elif error == "protocol":
        manifest["protocol"] = "stationary"
    elif error == "offset":
        report["cases"][1]["base_translation_xy_m"] = [0.2, 0]
    with pytest.raises(ValueError):
        grouped_cases(report, manifest, np.eye(4), True)
