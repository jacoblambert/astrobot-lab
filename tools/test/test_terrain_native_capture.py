import importlib.util
from pathlib import Path

import numpy as np
import pytest

path = Path(__file__).resolve().parents[2] / "OmniLRS/src/environments_wrappers/ros2/terrain_native_capture.py"
spec = importlib.util.spec_from_file_location("native_capture", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_native_frame_archive_preserves_every_field_without_retaining_nested_objects():
    import gc
    import io
    import json
    import copy
    records = module.EncodedFrames()
    frame = dict(observed_stamp=1., world_from_lidar=np.eye(4).tolist(),
                 current_pose=dict(residuals_m=[.0001, .0002], selected_indices=[3, 7]),
                 previous_pose=None)
    expected = copy.deepcopy(frame)
    for _ in range(2400):
        records.append(frame)
    frame["current_pose"]["residuals_m"][0] = .99
    assert len(records) == 2400 and records[0] == expected and records[-1] == expected
    assert all(isinstance(r, str) and not gc.is_tracked(r) for r in records.records)
    decoded = records[0]; decoded["observed_stamp"] = -1
    assert records[0] == expected
    stream = io.StringIO()
    records.write_audit(stream, dict(duplicate_native_reads=0, source_sha256="test"))
    audit = json.loads(stream.getvalue())
    assert audit["frames"] == [expected]*2400 and audit["duplicate_native_reads"] == 0


def test_native_frame_archive_rejects_invalid_or_unbounded_evidence():
    records = module.EncodedFrames()
    with pytest.raises(ValueError):
        records.append(dict(value=float("nan")))
    assert len(records) == 0
    records.records = ["{}"]*20000
    with pytest.raises(ValueError, match="capacity"):
        records.append({})


def test_extended_archive_capacity_is_explicit_and_still_bounded():
    records = module.EncodedFrames(capacity=40000)
    records.records = ["{}"] * 20000
    records.append({})
    assert len(records) == 20001
    records.records = ["{}"] * 40000
    with pytest.raises(ValueError, match="capacity"):
        records.append({})
    for capacity in (0, 40001, 1.5):
        with pytest.raises(ValueError, match="capacity"):
            module.EncodedFrames(capacity=capacity)


def test_optional_hit_records_are_streamed_without_retaining_nested_heap_or_changing_audit(tmp_path):
    import sys
    import copy
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from natural_holdout import native_hit_records
    ordinary = dict(selected_indices=[2], residuals_m=[.0001], missing=0)
    original = [dict(point_index=2, first_hit=dict(collision="/rock", position=[1., 2., 3.]), residual_m=.0001)]
    surface = dict(copy.deepcopy(ordinary), hit_records=copy.deepcopy(original))
    path = tmp_path / "chunk_000000.hits.json"
    module.archive_hit_records(surface, path)
    assert "hit_records" not in surface
    assert {k: surface[k] for k in ordinary} == ordinary
    frame = dict(file="chunk_000000.npz", current_pose=surface)
    assert native_hit_records(frame, tmp_path) == original
    with pytest.raises(FileExistsError):
        module.archive_hit_records(dict(ordinary, hit_records=original), path)
    path.write_bytes(path.read_bytes() + b" ")
    with pytest.raises(ValueError, match="changed"):
        native_hit_records(frame, tmp_path)
    surface["hit_records_file"] = "../other.hits.json"
    with pytest.raises(ValueError, match="path"):
        native_hit_records(frame, tmp_path)
    module.archive_hit_records(ordinary, tmp_path / "disabled.json")
    assert not (tmp_path / "disabled.json").exists()


def chunk():
    azimuth, zenith = np.array([0., .1], np.float32), np.array([-.1, .2], np.float32)
    a, z = np.meshgrid(azimuth, zenith, indexing="ij")
    points = 2 * np.column_stack([(np.cos(a) * np.cos(z)).ravel(),
                                  (np.sin(a) * np.cos(z)).ravel(), -np.sin(z).ravel()])
    return [points, np.full(4, 2.), np.full(4, 255), azimuth, zenith, 2, 2]


def test_native_tick_layout_and_bearing_are_checked():
    data = chunk()
    assert module.validate_chunk(*data)[0].shape == (4, 3)
    data[0] = data[0][::-1]
    with pytest.raises(ValueError, match="ordering"):
        module.validate_chunk(*data)


@pytest.mark.parametrize("kind", ["shape", "nan", "range", "dimensions"])
def test_bad_native_data_cannot_become_motion_evidence(kind):
    data = chunk()
    if kind == "shape":
        data[1] = data[1][:-1]
    elif kind == "nan":
        data[0][0, 0] = np.nan
    elif kind == "range":
        data[1][0] = 3.
    else:
        data[-1] = 0
    with pytest.raises(ValueError):
        module.validate_chunk(*data)


def test_pose_counterfactuals_do_not_fit_away_a_wrong_transform():
    points, depths = np.array([[2., 0., 0.]]), np.array([2.])
    def cast(origin, direction, distance):
        hit = (2. - origin[0]) / direction[0]
        return [(hit, (2., 0., 0.), (1., 0., 0.), "/wall")]
    correct = module.surface_audit(points, depths, [0], np.eye(4), cast, .5)
    wrong = np.eye(4)
    wrong[0, 3] = .02
    incorrect = module.surface_audit(points, depths, [0], wrong, cast, .5)
    assert correct["residuals_m"] == [0.] and correct["nonhorizontal_surface_samples"] == 1
    assert incorrect["residuals_m"][0] == pytest.approx(.02)


def test_wheel_initial_overlap_cannot_be_silently_replaced_with_later_terrain():
    depth = 0.9264813661575317
    hits = [(0., (.5, 0., 0.), (-1., 0., 0.), "/Robots/astrobot_0/front_right_wheel_link/collisions"),
            (depth - .5, (depth, 0., 0.), (0., 0., 1.), "/Lunaryard/Terrain/terrain_mesh_0")]
    report = module.surface_audit(np.array([[depth, 0., 0.]]), [depth], [0], np.eye(4),
                                  lambda *args: hits, .5, retain_hits=True)
    assert report["residuals_m"] == pytest.approx([depth - .5])
    assert report["outliers"][0]["first_hits"][0]["distance_m"] == 0
    assert report["hit_records"][0]["first_hit"]["collision"] == hits[0][3]


def test_grazing_outlier_preserves_first_hit_and_later_hit_without_changing_the_gate():
    points=np.array([[2.,0.,0.]],dtype=np.float32)
    hits=[(1.3,(1.8,0.,0.),(0.,1.,0.),"/box_corner"),
          (1.5,(2.,0.,0.),(0.,0.,1.),"/floor")]
    report=module.surface_audit(points,[2.],[0],np.eye(4),lambda *args:hits,.5)
    assert report["residuals_m"]==pytest.approx([.2]), "matching later floor hid the failed first hit"
    assert report["selected_indices"]==[0]
    detail=report["outliers"][0]
    assert detail["point_index"]==0 and detail["world_endpoint"]==[2.,0.,0.]
    assert detail["ray_origin"]==[.5,0.,0.] and detail["direction_norm"]==1.
    assert [hit["collision"] for hit in detail["first_hits"]]==["/box_corner","/floor"]
    assert detail["first_hit_residual_m"]==pytest.approx(.2)
    counterfactual=module.surface_audit(points,[2.],[0],np.eye(4),lambda *args:hits,.5,record_outliers=False)
    assert counterfactual["residuals_m"]==report["residuals_m"] and counterfactual["outliers"]==[]


def test_missing_native_surface_preserves_the_ray_and_cannot_look_like_zero_error():
    report=module.surface_audit(np.array([[2.,0.,0.]],np.float32),[2.],[0],np.eye(4),lambda *args:[],.5)
    assert report["missing"]==1 and report["residuals_m"]==[]
    assert report["outliers"][0]["first_hit_residual_m"] is None
    assert report["outliers"][0]["hit_count"]==0 and report["outliers"][0]["first_hits"]==[]


def test_opt_in_natural_hit_identity_keeps_first_hit_not_favorable_later_hit():
    hits = [(1.3, (1.8, 0., 0.), (0., 1., 0.), "/Rocks/instance_0"),
            (1.5, (2., 0., 0.), (0., 0., 1.), "/floor")]
    kwargs = dict(points=np.array([[2., 0., 0.]], np.float32), depths=[2.], selected=[0],
                  transform=np.eye(4), cast=lambda *args: hits, minimum=.5)
    ordinary = module.surface_audit(**kwargs)
    natural = module.surface_audit(**kwargs, retain_hits=True)
    assert "hit_records" not in ordinary
    assert {k: v for k, v in natural.items() if k != "hit_records"} == ordinary
    assert natural["hit_records"][0]["first_hit"]["collision"] == "/Rocks/instance_0"
    assert natural["hit_records"][0]["residual_m"] == pytest.approx(.2)


def test_exact_native_association_uses_acquisition_not_scan_end_pose():
    point=np.array([[2., .1, -.2]],dtype=np.float32)
    pose=np.eye(4); pose[0,3]=.02
    frame=dict(observed_stamp=1.,world_from_lidar=pose.tolist())
    found=module.associate_returns(point,1.09,[(point,frame)],.2)
    assert found[0][0][0] is frame and found[0][0][1]==0
    assert found[0][0][0]["observed_stamp"]!=1.09


@pytest.mark.parametrize("fault", ["changed_xyz", "excessive", "too_old", "future"])
def test_native_provenance_cannot_guess_or_fit_a_match(fault):
    point=np.array([[2., .1, -.2]],dtype=np.float32)
    frame=dict(observed_stamp=1.,world_from_lidar=np.eye(4).tolist())
    cloud=point.copy()
    if fault=="changed_xyz": cloud[0,0]+=.00001
    elif fault=="excessive": cloud=np.repeat(cloud,33,axis=0)
    elif fault=="too_old": frame["observed_stamp"]=.5
    else: frame["observed_stamp"]=1.2
    with pytest.raises(ValueError,match="candidates|history"):
        module.associate_returns(point,1.09,[(cloud,frame)],.2)


def test_ambiguous_native_times_are_all_retained_not_fitted_or_selected():
    point=np.array([[2.,0.,0.]],dtype=np.float32)
    correct=dict(observed_stamp=1.,world_from_lidar=np.eye(4).tolist())
    pose=np.eye(4);pose[0,3]=.02
    wrong=dict(observed_stamp=1.05,world_from_lidar=pose.tolist())
    groups=module.associate_returns(point,1.09,[(point,correct),(point,wrong)],.2)
    assert len(groups[0])==2
    def cast(origin,direction,distance):
        return [((2-origin[0])/direction[0],(2.,0.,0.),(1.,0.,0.),"/wall")]
    errors=[module.surface_audit(point,[2.],[0],np.asarray(f["world_from_lidar"]),cast,.5)["residuals_m"][0]
            for f,_ in groups[0]]
    assert max(errors)==pytest.approx(.02), "ambiguity hid the incorrect acquisition pose"
