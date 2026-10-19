#!/usr/bin/env python3
"""Archive trace-on/off and repeat variation; never select the best replay."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from archive_guard import allocate_run, check_run
from imu_consistency import interpolate_poses
from shadow_pose_replay import load_pose_sensitivity, sha


def motion_bound(times, poses, duration=.1001, radius=5.):
    q = times[times >= times[0]+duration]
    before = interpolate_poses(times, poses, q-duration)
    after = interpolate_poses(times, poses, q)
    displacement = np.linalg.norm(after[:,:3,3]-before[:,:3,3],axis=1)
    angle = (Rotation.from_matrix(before[:,:3,:3]).inv()*Rotation.from_matrix(after[:,:3,:3])).magnitude()
    bound = displacement+2*radius*np.sin(angle/2)
    return dict(interval_s=duration,radius_m=radius,
        translation_p95_max_m=np.percentile(displacement,[95,100]).tolist(),
        angle_p95_max_deg=np.rad2deg(np.percentile(angle,[95,100])).tolist(),
        endpoint_bound_p95_max_m=np.percentile(bound,[95,100]).tolist(),
        interpretation="interval endpoint-motion bound, not measured per-point timing or a registration-error bound")


def compare_poses(left,right):
    distance=np.linalg.norm(left[:,:3,3]-right[:,:3,3],axis=1)
    angles=(Rotation.from_matrix(left[:,:3,:3]).inv()*Rotation.from_matrix(right[:,:3,:3])).magnitude()
    return dict(position_p50_p95_max_m=np.percentile(distance,[50,95,100]).tolist(),
        angle_p50_p95_max_deg=np.rad2deg(np.percentile(angles,[50,95,100])).tolist(),
        equivalent_1um_1urad=bool(distance.max()<=1e-6 and angles.max()<=1e-6))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture",type=Path)
    p.add_argument("baseline",type=Path)
    p.add_argument("traced",type=Path)
    p.add_argument("repeat",type=Path)
    p.add_argument("--archive-root",type=Path,default=Path("/archive"))
    a=p.parse_args();out=allocate_run(a.archive_root);print(f"Run: {out}",flush=True)
    try:
        runs=[a.baseline,a.traced,a.repeat]
        for r in [a.capture,*runs]:check_run(r)
        read=lambda path:json.loads(path.read_text())
        attempts=[read(r/"attempt.json") for r in runs]
        if [x.get("trace_enabled",False) for x in attempts]!=[False,True,False]:raise ValueError("expected off/on/off sequence")
        for key in ("prepared","prepared_sha256","probe_sha256","initial_state_source","mode"):
            if len({str(x[key]) for x in attempts})!=1:raise ValueError("comparison differs in "+key)
        for sub in ("config","loaded_libraries.json"):
            values=[{f.name:read(f) for f in (r/sub).glob("*.json")} if sub=="config" else read(r/sub) for r in runs]
            if not values[0]==values[1]==values[2]:raise ValueError("configuration/loaded libraries differ")
        frames=read(a.capture/"native_timing_audit.json")["frames"]
        seq=[load_pose_sensitivity(r,a.capture,frames,"first_published")[0] for r in runs]
        gt=np.array([f["world_from_lidar"] for f in frames]);times=np.array([f["observed_stamp"] for f in frames])
        result=dict(completed=True,acceptance=False,trace_vs_baseline=compare_poses(seq[0],seq[1]),
            repeat_vs_baseline=compare_poses(seq[0],seq[2]),motion_bound=motion_bound(times,gt),
            selection="All three attempts retained; no best-run selection; variation is not estimator improvement",
            input_sha256={str(r/"shadow_pair_audit.json"):sha(r/"shadow_pair_audit.json") for r in runs})
        with (out/"diagnostic_comparison.json").open("x") as stream:json.dump(result,stream,indent=2,allow_nan=False)
        print(json.dumps(result,indent=2))
    except Exception as exc:
        with (out/"failure.json").open("x") as stream:json.dump(dict(error=str(exc)),stream)
        raise


if __name__=="__main__":main()
