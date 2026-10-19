"""Pure audit-control tests; actual PhysX range/pose validation remains a live gate."""
import importlib.util
import json
import sys
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

path = Path(__file__).resolve().parents[2]/"OmniLRS/src/environments_wrappers/ros2/terrain_sensor_audit.py"
spec = importlib.util.spec_from_file_location("terrain_sensor_audit_test",path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_incremental_audit_reserves_same_declared_frame_quota(tmp_path, monkeypatch):
    callbacks = {}
    node = NS(create_subscription=lambda kind, topic, callback, qos: callbacks.update({topic: callback}))
    monkeypatch.setitem(sys.modules, "rclpy", NS(create_node=lambda _: node))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", NS(PointCloud2=object))
    monkeypatch.setitem(sys.modules, "rclpy.qos", NS(qos_profile_sensor_data=object()))
    audit = module.TerrainSensorAudit(tmp_path, frame_limit=2, ray_budget=1)
    receive = callbacks["/astrobot_0/lidar_0/pointcloud/raw"]
    receive("first"); receive("second"); receive("outside-quota")
    assert list(audit.queue) == ["first", "second"]
    audit.queue.popleft()
    audit.pending = object()
    receive("still-outside-quota")
    assert list(audit.queue) == ["second"]
    assert audit.unselected_after_quota == 2 and audit.dropped == 0


def test_pitched_sensor_requires_matching_fresh_tf():
    angle=np.deg2rad(10)
    pitched=module.tf_matrix([.4,0,.25],[0,np.sin(angle/2),0,np.cos(angle/2)])
    assert module.validate_extrinsic_tf([(1.,pitched)],1.01,pitched)["maximum_matrix_error"]==0.
    for samples in ([],[(0.,pitched)],[(1.,np.eye(4))]):
        with pytest.raises(ValueError):module.validate_extrinsic_tf(samples,1.,pitched)
    with pytest.raises(ValueError):module.tf_matrix([0,0,0],[0,0,0,2])


@pytest.mark.parametrize("miss_valid_surface",[False,True])
def test_clamped_ray_diagnostics_never_hide_valid_surface_misses(tmp_path,monkeypatch,miss_valid_surface):
    monkeypatch.setitem(sys.modules,"rclpy",NS(spin_once=lambda *args,**kwargs: None))
    points=np.array([[.5,0,0],[1.,0,0],[2.,0,0]],dtype="<f4")
    message=NS(fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate("xyz")],
               point_step=12,row_step=36,width=3,height=1,data=points.tobytes(),is_bigendian=False,
               header=NS(stamp=NS(sec=1,nanosec=0),frame_id="vlp16"))
    audit=object.__new__(module.TerrainSensorAudit)
    audit.node=NS(destroy_node=lambda: None)
    audit.queue=deque([message])
    audit.poses=deque()
    audit.frames=[]
    audit.boundary=audit.dropped=0
    audit.output=tmp_path
    audit.done=False
    audit.frame_limit=1

    def cast(origin,direction,distance):
        if origin[0] == 0.:
            return [(0.,None,None,"robot")]
        if distance<.11 or (miss_valid_surface and distance<.7):
            return []
        return [(distance-.1,None,None,"surface")]

    audit.observe(1.,dict(world_from_base=np.eye(4),base_from_lidar=np.eye(4),sensor_attributes={"minRange":"0.5"}),cast)
    result=json.loads((tmp_path/"sensor_audit.json").read_text())
    assert result["passed"] is not miss_valid_surface
    assert result["missing_rays"]==1+int(miss_valid_surface)
    assert result["surface_missing_rays"]==int(miss_valid_surface)
    assert result["frames"][0]["missing_details"][0]["surface_eligible"] is False
    assert result["frames"][0]["raw_scan_sha256"]


@pytest.mark.parametrize("fault", ["none", "wrong_second_pose", "missing_source"])
@pytest.mark.parametrize("ray_budget", [None, 1])
def test_native_audit_checks_all_candidates_and_preserves_failed_input(tmp_path,monkeypatch,fault,ray_budget):
    monkeypatch.setitem(sys.modules,"rclpy",NS(spin_once=lambda *args,**kwargs: None))
    points=np.array([[2.,0.,0.]],dtype="<f4")
    message=NS(fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate("xyz")],
               point_step=12,row_step=12,width=1,height=1,data=points.tobytes(),is_bigendian=False,
               header=NS(stamp=NS(sec=1,nanosec=0),frame_id="vlp16"))
    audit=object.__new__(module.TerrainSensorAudit)
    audit.node=NS(destroy_node=lambda: None);audit.queue=deque([message]);audit.poses=deque()
    audit.frames=[];audit.boundary=audit.dropped=0;audit.output=tmp_path;audit.done=False;audit.frame_limit=1
    audit.ray_budget=ray_budget;audit.pending=None
    def associate(cloud,stamp):
        if fault=="missing_source":raise ValueError("missing native source")
        first=dict(file="a.npz",sha256="a",observed_stamp=.9,world_from_lidar=np.eye(4).tolist())
        second=dict(file="b.npz",sha256="b",observed_stamp=1.,world_from_lidar=np.eye(4).tolist())
        if fault=="wrong_second_pose":second["world_from_lidar"][0][3]=.02
        return [[(first,0),(second,0)]]
    audit.native_source=NS(associate=associate)
    calls=[]
    def cast(origin,direction,distance):
        calls.append((origin,direction,distance))
        return [((2.-origin[0])/direction[0],None,None,"wall")]
    pose=dict(world_from_base=np.eye(4),base_from_lidar=np.eye(4),sensor_attributes={"minRange":".5"})
    if fault=="missing_source":
        with pytest.raises(ValueError,match="missing native"):audit.observe(1.,pose,cast)
        assert np.array_equal(np.load(tmp_path/"raw_scan_00.npz")["points"],points)
        assert not (tmp_path/"sensor_audit.json").exists()
        return
    audit.observe(1.,pose,cast)
    if ray_budget is not None:
        assert len(calls)==2 and not audit.done and not audit.frames
        audit.observe(1.033333,pose,cast)
    assert len(calls)==4  # Both acquisition candidates, both origin conventions.
    result=json.loads((tmp_path/"sensor_audit.json").read_text())
    assert result["passed"] is (fault=="none")
    assert not result["stationary_only"] and not result["timing_validated_for_motion"]
    assert result["frames"][0]["ambiguous_source_returns"]==1
    assert len(result["frames"][0]["native_sources"])==2
