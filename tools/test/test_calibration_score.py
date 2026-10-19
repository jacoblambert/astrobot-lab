import hashlib
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from score_terrain_calibration import validated_scan, reference_labels, validated_route, run, require_single_pose_surface


def test_native_surface_audit_cannot_unlock_undeskewed_ros_replay():
    require_single_pose_surface(dict(passed=True,stationary_only=True))
    with pytest.raises(ValueError,match="not deskew"):
        require_single_pose_surface(dict(passed=True,stationary_only=False,surface_pose_model="native_tick_exact_xyz"))
    with pytest.raises(ValueError,match="not deskew"):
        require_single_pose_surface(dict(passed=True,stationary_only=True,surface_pose_model="native_tick_exact_xyz"))


def test_raw_scan_cannot_be_replaced_after_sensor_audit(tmp_path):
    path = tmp_path/"scan.npz"
    np.savez_compressed(path,points=np.zeros((2,3)),stamp=1.,world_from_lidar=np.eye(4))
    audit = dict(stamp=1.,world_from_lidar=np.eye(4).tolist(),raw_scan_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    assert validated_scan(path,audit,0.)[1] == 1.
    with pytest.raises(ValueError,match="ordering"):
        validated_scan(path,audit,1.)
    audit["world_from_lidar"][0][3] = 1.
    with pytest.raises(ValueError,match="transform"):
        validated_scan(path,audit,0.)
    np.savez_compressed(path,points=np.ones((2,3)),stamp=1.,world_from_lidar=np.eye(4))
    with pytest.raises(ValueError,match="hash"):
        validated_scan(path,audit,0.)


def test_reference_cannot_omit_required_object():
    spec = dict(boxes=[dict(name="box",required=True)])
    tile = dict(object_ids=np.zeros((2,2)),visible=np.ones((2,2),dtype=bool))
    with pytest.raises(ValueError,match="omits required"):
        reference_labels(spec,tile,dict(objects=[]))


@pytest.mark.parametrize("kind",["nan_stamp","scale","reflection","bottom_row","shape"])
def test_hash_match_is_not_a_valid_pose_contract(tmp_path,kind):
    matrix=np.eye(4)
    stamp=1.
    if kind=="nan_stamp": stamp=float("nan")
    elif kind=="scale": matrix[0,0]=1.1
    elif kind=="reflection": matrix[0,0]=-1.
    elif kind=="bottom_row": matrix[3,0]=1.
    elif kind=="shape": matrix=matrix[:3]
    path=tmp_path/"scan.npz"
    np.savez_compressed(path,points=np.zeros((2,3)),stamp=stamp,world_from_lidar=matrix)
    audit=dict(stamp=stamp,world_from_lidar=matrix.tolist(),raw_scan_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    with pytest.raises(ValueError,match="stamp|SE\\(3\\)"):
        validated_scan(path,audit,0.)


@pytest.mark.parametrize("pattern",["straight","diagonal15","spin360","centered","centered_diagonal15","centered_sweep15","wide_sweep15","wide_sweep15_return"])
def test_prescribed_route_cannot_drop_or_change_views(pattern):
    import copy
    heading=float(np.deg2rad(15)) if pattern in ("diagonal15","centered_diagonal15","centered_sweep15") else 0.
    targets=[dict(distance=x/10,heading=heading) for x in [*range(21),*range(19,-1,-1)]]
    if pattern in ("centered","centered_diagonal15"):
        targets=[dict(distance=x/10,heading=heading) for x in [*range(11),*range(9,-11,-1),*range(-9,1)]]
    if pattern=="centered_sweep15":
        targets=[dict(distance=x,heading=heading) for x in (0.,1.,-1.,0.)]
    if heading:
        targets=[dict(distance=0.,heading=0.),*targets,dict(distance=0.,heading=0.)]
    if pattern=="spin360":
        targets=[dict(distance=0.,heading=float(np.deg2rad(d))) for d in range(0,361,15)]
    if pattern in ("wide_sweep15","wide_sweep15_return"):
        targets=[dict(distance=0.,heading=0.),
                 *[dict(distance=x,heading=float(np.deg2rad(15))) for x in (0.,1.2,-1.2,0.)]]
        if pattern=="wide_sweep15_return":
            targets.append(dict(distance=0.,heading=0.))
    route=dict(complete=True,prescribed_pattern=pattern,prescribed_targets=targets,
               views=[dict(target=copy.deepcopy(t)) for t in targets])
    assert len(validated_route(route))==len(targets)
    invalid=copy.deepcopy(route)
    invalid["views"].pop()
    with pytest.raises(ValueError,match="incomplete"):
        validated_route(invalid)
    invalid=copy.deepcopy(route)
    invalid["views"][-1]["target"]["distance"]+=.01
    with pytest.raises(ValueError,match="differs"):
        validated_route(invalid)


@pytest.mark.parametrize("mode,message",[("no_reference","pre-existing reference"),("no_tf","published-TF"),
                                        ("changed_geometry","geometry specification")])
def test_mount_experiment_cannot_change_reference_or_skip_tf(tmp_path,mode,message):
    import json
    from types import SimpleNamespace
    cap=tmp_path/"capture";cap.mkdir()
    reference=tmp_path/"reference";reference.mkdir()
    files={"calibration_checks.json":dict(passed=True),
           "sensor_audit.json":dict(passed=True,stationary_only=True,extrinsic_tf_checked=mode!="no_tf"),
           "calibration_spec.json":dict(center=[0,0],boxes=[]),"summary.json":dict(objects=[]),
           "manifest.json":dict(experimental_lidar_pitch_deg=10,sensor=dict(sensor_attributes=dict(minRange=".5",maxRange="100")))}
    for name,value in files.items():(cap/name).write_text(json.dumps(value))
    np.savez_compressed(cap/"collision_tile.npz",visible=np.zeros((2,2),dtype=bool))
    (reference/"calibration_spec.json").write_text(json.dumps(dict(center=[1,0],boxes=[])))
    args=SimpleNamespace(capture=cap,reference_capture=None if mode=="no_reference" else reference)
    with pytest.raises(ValueError,match=message):run(args,tmp_path/"output")
