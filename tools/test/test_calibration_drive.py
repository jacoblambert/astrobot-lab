import importlib.util
from pathlib import Path
import pytest

path=Path(__file__).resolve().parents[2]/"OmniLRS/src/environments_wrappers/ros2/terrain_calibration_drive.py"
spec=importlib.util.spec_from_file_location("terrain_drive",path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_completion_marker_is_fully_serialized_before_exclusive_publication(tmp_path, monkeypatch):
    import json
    import os
    original = os.link
    path = tmp_path / "completion.json"
    def checked_link(source, target):
        assert not path.exists()
        assert json.loads(source.read_text()) == {"complete": True}
        original(source, target)
    monkeypatch.setattr(os, "link", checked_link)
    module.publish_completion_marker(path, {"complete": True})
    with pytest.raises(FileExistsError):
        module.publish_completion_marker(path, {"complete": False})
    assert json.loads(path.read_text()) == {"complete": True}


def test_speed_direction_and_arrival():
    v,w,done=module.drive_command([-20,-20,.15],0.,.1,[-20,-20])
    assert 0<v<=.15 and w==0 and not done
    v,w,done=module.drive_command([-19.9,-20,.15],.01,0.,[-20,-20])
    assert -.15<=v<0 and w<0 and not done
    assert module.drive_command([-20,-20,.15],0.,0.,[-20,-20]) == (0.,0.,True)


def test_centered_route_keeps_distance_views_and_bounded_reverse_motion():
    targets=module.prescribed_targets("centered")
    assert len(targets)==41 and targets[0]==targets[-1]==dict(distance=0.,heading=0.)
    assert min(t["distance"] for t in targets)==-1. and max(t["distance"] for t in targets)==1.
    assert sum(abs(a["distance"]-b["distance"]) for a,b in zip(targets,targets[1:]))==pytest.approx(4.)
    v,w,done=module.drive_command([-.5,0.,.15],0.,-.6,[0.,0.],pattern="centered")
    assert -.15<=v<0 and w==0. and not done
    for x in (-1.081,1.081):
        with pytest.raises(ValueError,match="envelope"):
            module.drive_command([x,0.,.15],0.,0.,[0.,0.],pattern="centered")
    with pytest.raises(ValueError,match="envelope"):
        module.drive_command([-.5,0.,.15],0.,0.,[0.,0.],pattern="straight")


def test_centered_diagonal_retains_budget_and_rotated_tracking_bounds():
    import math
    targets=module.prescribed_targets("centered_diagonal15")
    angle=math.radians(15)
    assert len(targets)==43 and targets[0]==targets[-1]==dict(distance=0.,heading=0.)
    assert all(t["heading"]==angle for t in targets[1:-1])
    assert min(t["distance"] for t in targets)==-1. and max(t["distance"] for t in targets)==1.
    assert sum(abs(a["distance"]-b["distance"]) for a,b in zip(targets,targets[1:]))==pytest.approx(4.)
    p=[-.5*math.cos(angle),-.5*math.sin(angle),.15]
    v,w,done=module.drive_command(p,angle,-.6,[0.,0.],angle,"centered_diagonal15")
    assert -.15<=v<0 and w==0. and not done
    assert module.drive_command([0.,0.,.15],0.,0.,[0.,0.],angle,"centered_diagonal15")== (0.,.15,False)
    for along,across in ((-1.081,0.),(1.081,0.),(0.,.081)):
        p=[along*math.cos(angle)-across*math.sin(angle),along*math.sin(angle)+across*math.cos(angle),.15]
        with pytest.raises(ValueError,match="envelope"):
            module.drive_command(p,angle,0.,[0.,0.],angle,"centered_diagonal15")


def test_native_sweep_preserves_path_and_refuses_stop_scan_only_capture():
    targets=module.prescribed_targets("centered_sweep15")
    assert [t["distance"] for t in targets]==[0.,0.,1.,-1.,0.,0.]
    assert targets[0]==targets[-1]==dict(distance=0.,heading=0.)
    assert sum(abs(a["distance"]-b["distance"]) for a,b in zip(targets,targets[1:]))==4.
    with pytest.raises(ValueError,match="native-tick"):
        module.validate_acquisition("centered_sweep15",False)
    module.validate_acquisition("centered_sweep15",True)
    module.validate_acquisition("centered_diagonal15",False)


@pytest.mark.parametrize("pattern",["centered_sweep15","wide_sweep15","wide_sweep15_return"])
def test_native_sweep_long_leg_keeps_speed_stall_and_clearance_guards(pattern):
    import math
    from types import SimpleNamespace
    angle=math.radians(15)
    driver=object.__new__(module.CalibrationDrive)
    driver.failure=None
    driver.pattern=pattern
    driver.origin=module.np.array([0.,0.])
    driver.node=SimpleNamespace(get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name="driver")],
                                get_name=lambda: "driver")
    boxes,commands=[],[]
    driver.query=SimpleNamespace(overlap_box=lambda *args: boxes.append(args))
    driver.cast=lambda p,*args: [(1.,(p[0],p[1],0.),(0.,0.,1.),"/calibration/floor")]
    driver.target_started=0.
    driver.progress=(5.9,module.np.array([0.,0.,.15]),angle)
    driver.log=[]
    driver._publish=lambda v=0.,w=0.: commands.append((v,w))
    q=[0.,0.,math.sin(angle/2),math.cos(angle/2)]
    assert not driver.move(6.,[0.,0.,.15],q,dict(distance=-1.,heading=angle))
    assert commands[-1][0]==-.15 and boxes[-1][0]==(.85,.7,.48)
    driver.progress=(20.,module.np.array([0.,0.,.15]),angle)
    with pytest.raises(ValueError,match="waypoint timeout"):
        driver.move(20.01,[0.,0.,.15],q,dict(distance=-1.,heading=angle))
    assert commands[-1]==(0.,0.)
    driver.target_started=0.
    driver.progress=(0.,module.np.array([0.,0.,.15]),angle)
    with pytest.raises(ValueError,match="stalled"):
        driver.move(1.6,[0.,0.,.15],q,dict(distance=-1.,heading=angle))
    assert commands[-1]==(0.,0.)


def test_wide_sweep_declares_extra_travel_without_relaxing_tracking_or_native_audit():
    import math
    angle=math.radians(15)
    targets=module.prescribed_targets("wide_sweep15")
    assert [t["distance"] for t in targets]==[0.,0.,1.2,-1.2,0.]
    assert targets[0]==dict(distance=0.,heading=0.)
    assert targets[-1]==dict(distance=0.,heading=angle)
    assert sum(abs(a["distance"]-b["distance"]) for a,b in zip(targets,targets[1:]))==pytest.approx(4.8)
    with pytest.raises(ValueError,match="native-tick"):
        module.validate_acquisition("wide_sweep15",False)
    module.validate_acquisition("wide_sweep15",True)
    for along in (-1.2,1.2):
        p=[along*math.cos(angle),along*math.sin(angle),.15]
        assert module.drive_command(p,angle,0.,[0.,0.],angle,"wide_sweep15")[0]==-math.copysign(.15,along)
    for along,across in ((-1.281,0.),(1.281,0.),(0.,.081)):
        p=[along*math.cos(angle)-across*math.sin(angle),along*math.sin(angle)+across*math.cos(angle),.15]
        with pytest.raises(ValueError,match="envelope"):
            module.drive_command(p,angle,0.,[0.,0.],angle,"wide_sweep15")


def test_wide_return_experiment_adds_only_the_final_bounded_origin_turn():
    import math
    original=module.prescribed_targets("wide_sweep15")
    returned=module.prescribed_targets("wide_sweep15_return")
    assert returned[:-1]==original and returned[-1]==dict(distance=0.,heading=0.)
    assert sum(abs(a["distance"]-b["distance"]) for a,b in zip(returned,returned[1:]))==pytest.approx(4.8)
    with pytest.raises(ValueError,match="native-tick"):
        module.validate_acquisition("wide_sweep15_return",False)
    module.validate_acquisition("wide_sweep15_return",True)
    assert module.drive_command([-.008,.014,.15],math.radians(15),0.,[0.,0.],0.,"wide_sweep15_return")== (0.,-.15,False)
    with pytest.raises(ValueError,match="heading deviated"):
        module.drive_command([.2,.02,.15],math.radians(15),0.,[0.,0.],0.,"wide_sweep15_return")


@pytest.mark.parametrize("position,yaw", [([-20,-19.8,0],0),([-17.8,-20,0],0),([-20,-20,0],.1),
                                          ([float('nan'),-20,0],0)])
def test_unsafe_tracking_fails(position,yaw):
    with pytest.raises(ValueError):
        module.drive_command(position,yaw,1.,[-20,-20])


def test_diagonal_route_and_bounded_origin_rotation():
    import math
    targets=module.prescribed_targets("diagonal15")
    assert len(targets)==43 and targets[0]==targets[-1]==dict(distance=0.,heading=0.)
    heading=math.radians(15)
    assert module.drive_command([-20,-20,.15],0.,0.,[-20,-20],heading)==(0.,.15,False)
    v,w,done=module.drive_command([-19.9937,-19.9924,.15],.109,0.,[-20,-20],heading)
    assert v==0. and w>0. and not done, "skid rotation drift switched into translation"
    p=[-20+math.cos(heading),-20+math.sin(heading),.15]
    v,w,done=module.drive_command(p,heading,2.,[-20,-20],heading)
    assert v==.15 and abs(w)<1e-8 and not done
    with pytest.raises(ValueError):
        module.drive_command(p,0.,2.,[-20,-20],heading)
    with pytest.raises(ValueError):
        module.prescribed_targets("unbounded")


def test_archive_loss_still_stops_and_destroys_driver(tmp_path):
    import threading
    from types import SimpleNamespace
    events=[]
    driver=object.__new__(module.CalibrationDrive)
    driver._publish=lambda: events.append("zero")
    driver.armed=True
    driver.closed=threading.Event()
    driver.thread=SimpleNamespace(join=lambda **kwargs: events.append("join"))
    driver.node=SimpleNamespace(destroy_node=lambda: events.append("destroy"))
    driver.output=tmp_path/"unmounted"
    with pytest.raises(FileNotFoundError):
        driver.close()
    assert events==["zero","join","destroy"] and not driver.armed and driver.closed.is_set()


def test_spin_is_incremental_at_fixed_origin_and_wraps_yaw():
    import math
    targets=module.prescribed_targets("spin360")
    assert len(targets)==25 and targets[-1]==dict(distance=0.,heading=2*math.pi)
    for target in targets[1:]:
        yaw=math.remainder(target["heading"]-math.radians(15),2*math.pi)
        v,w,done=module.drive_command([-.01,.01,.15],yaw,0.,[0.,0.],target["heading"],"spin360")
        assert v==0 and 0<w<=.15 and not done
    assert module.drive_command([0.,0.,.15],0.,0.,[0.,0.],2*math.pi,"spin360")== (0.,0.,True)


@pytest.mark.parametrize("position,target,heading", [([.251,0.,.15],0.,.1),([0.,0.,.15],.1,.1),
                                                   ([0.,0.,.15],0.,1.),([0.,0.,.15],0.,7.)])
def test_spin_refuses_translation_drift_and_large_heading_steps(position,target,heading):
    with pytest.raises(ValueError,match="envelope"):
        module.drive_command(position,0.,target,[0.,0.],heading,"spin360")


def test_spin_clearance_covers_full_drift_envelope_about_original_pose():
    import math
    from types import SimpleNamespace
    boxes, rays, commands = [], [], []
    driver=object.__new__(module.CalibrationDrive)
    driver.failure=None
    driver.pattern="spin360"
    driver.origin=module.np.array([0.,0.])
    driver.node=SimpleNamespace(get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name="driver")],
                                get_name=lambda: "driver")
    driver.query=SimpleNamespace(overlap_box=lambda *args: boxes.append(args))
    def cast(origin, direction, distance):
        rays.append(origin)
        return [(1.,(origin[0],origin[1],0.),(0.,0.,1.),"/calibration/floor")]
    driver.cast=cast
    driver.target_started=None
    driver.progress=None
    driver.log=[]
    driver._publish=lambda v,w: commands.append((v,w))
    assert not driver.move(1.,[.15,.05,.15],[0.,0.,0.,1.],dict(distance=0.,heading=.2))
    radius=math.hypot(.85,.7)+.25
    assert boxes[0][0]==(radius,radius,.48) and boxes[0][1]==(0.,0.,.52)
    assert len(rays)==9 and max(abs(p[0]) for p in rays)==radius
    assert commands==[(0.,.15)]
