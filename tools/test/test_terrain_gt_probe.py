import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

path = Path(__file__).resolve().parents[2]/"OmniLRS/src/environments_wrappers/ros2/terrain_gt_probe.py"
spec = importlib.util.spec_from_file_location("terrain_gt_probe", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def hit(distance):
    return NS(distance=distance, position=(0,0,3-distance), normal=(0,0,1),
              collision="/rock", rigid_body="/rock", protoIndex=7)


class Query:
    def __init__(self, hits):
        self.hits = hits

    def raycast_all(self, origin, direction, distance, report, both_sides=False):
        for item in self.hits:
            try:
                if not report(item):
                    break
            except Exception:
                pass  # Model native callback exception swallowing.


def test_unsorted_hits_and_instance_identity():
    rows = module.raycast_hits(Query([hit(2),hit(1)]), (0,0,3), (0,0,-1), 4)
    assert [r[0] for r in rows] == [1,2]
    assert rows[0][-1] == 7


def test_callback_error_never_becomes_no_hit_ground():
    with pytest.raises(RuntimeError, match="raycast callback failed"):
        module.raycast_hits(Query([NS(distance=1)]), (0,0,3), (0,0,-1), 4)


def test_lidar_audit_explicitly_includes_mesh_backfaces():
    class BackfaceQuery:
        def raycast_all(self,origin,direction,distance,report,both_sides=False):
            if both_sides: report(hit(1.))
    query=BackfaceQuery()
    assert module.raycast_hits(query,(0,0,3),(0,0,-1),4)==[]
    assert len(module.raycast_hits(query,(0,0,3),(0,0,-1),4,both_sides=True))==1


def test_selected_pose_caches_prim_identity_not_world_transform(monkeypatch):
    import sys
    import numpy as np
    base = NS(GetPath=lambda: "/base")
    sensor = NS(GetPath=lambda: "/base/vlp16/SelectedGeneric", GetTypeName=lambda: "Generic")
    world = np.eye(4)
    mount = np.eye(4)
    mount[:3, 3] = [.4, 0., .25]
    def forbidden(*args):
        raise AssertionError("selected per-tick audit must not re-traverse USD prims")
    robot = NS(get_pose=lambda: (world[:3, 3].copy(), [0., 0., 0., 1.]), root_body_id=1,
               dc=NS(get_rigid_body_path=lambda _: "/base"), _find_descendants_by_name=forbidden)
    cache = NS(GetLocalToWorldTransform=lambda p: (world if p is base else world @ mount).T)
    pxr = NS(UsdGeom=NS(XformCache=lambda _: cache), Usd=NS(TimeCode=NS(Default=lambda: None)),
             Gf=NS(Matrix3d=lambda _: np.eye(3), Quatd=lambda *args: None, Vec3d=lambda *args: None))
    monkeypatch.setitem(sys.modules, "pxr", pxr)
    owner = NS(controller=NS(robotManager=NS(robots={"robot": robot})),
               selected_lidar=NS(base_prim=base, prim=sensor, sensor_attributes={"minRange": "0.5"}))
    for x in (0., .1, 2.):
        world[0, 3] = x
        pose = module.TerrainGTProbe._pose(owner)[3]
        np.testing.assert_allclose(pose["world_from_base"], world)
        np.testing.assert_allclose(pose["base_from_lidar"], mount, atol=1e-15)


@pytest.mark.parametrize("kind,expected", [("Mesh",0xFFFFFFFF),("PointInstancer",7)])
def test_undefined_noninstancer_index_not_an_object_identity(kind, expected):
    prim = NS(IsPseudoRoot=lambda: False, GetTypeName=lambda: kind, GetParent=lambda: None)
    stage = NS(GetPrimAtPath=lambda path: prim)
    row = module.raycast_hits(Query([hit(1)]), (0,0,3), (0,0,-1), 4)[0]
    assert module.collision_identity(stage,row) == ("/rock","/rock",expected)
