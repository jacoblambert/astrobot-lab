#!/usr/bin/env python3
"""Development-only ideal-flat-plane LiDAR sampling bounds; never mapper input.

Uses recorded settled poses and the native sensor's angular sampling. Ignores
obstacle/body occlusion in the generated rays, so this is an optimistic sensor
screen, NOT obstacle/free-space accuracy or justification for changing labels.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from archive_guard import allocate_run
from score_terrain_calibration import reference_labels,validated_route,validated_scan


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture",type=Path)
    parser.add_argument("--archive-root",type=Path,default=Path("/archive"))
    args=parser.parse_args()
    out=allocate_run(args.archive_root)
    print(f"Run: {out}",flush=True)
    cap=args.capture
    read=lambda name:json.loads((cap/name).read_text())
    spec,manifest=read("calibration_spec.json"),read("manifest.json")
    if (cap/"failure.json").exists():raise ValueError("failed capture")
    attrs=manifest["sensor"]["sensor_attributes"]
    hf,vf,hr,vr=[float(attrs[k]) for k in ["horizontalFov","verticalFov","horizontalResolution","verticalResolution"]]
    az=np.deg2rad(-hf/2+np.arange(int(hf/hr))*hr)
    zen=np.deg2rad(-vf/2+np.arange(int(vf/vr)+1)*vr)
    a,z=np.meshgrid(az,zen,indexing="ij")
    beams=np.column_stack([(np.cos(a)*np.cos(z)).ravel(),(np.sin(a)*np.cos(z)).ravel(),-np.sin(z).ravel()])
    tile=np.load(cap/"collision_tile.npz")
    labels,_,_=reference_labels(spec,tile,read("summary.json"))
    y,x=np.indices(labels.shape)
    mx=np.floor((tile["origin"][0]+(x+.5)*.02-spec["center"][0]+10)/.1).astype(int)
    my=np.floor((tile["origin"][1]+(y+.5)*.02-spec["center"][1]+10)/.1).astype(int)
    flat=labels==0
    poses=[]; previous=-1.; hashes={}
    for view in validated_route(read("multiview.json")):
        folder=cap/view["path"]
        audit=json.loads((folder/"sensor_audit.json").read_text())
        if not audit["passed"]:raise ValueError("failed viewpoint")
        for file,frame in zip(sorted(folder.glob("raw_scan_*.npz")),audit["frames"]):
            _,stamp,pose=validated_scan(file,frame,previous);previous=stamp
            poses.append((stamp,pose))
            hashes[str(file.relative_to(cap))]=frame["raw_scan_sha256"]
    initial_extrinsic=np.array(manifest["sensor"]["base_from_lidar"])
    results=[]
    for degrees in (0,5,10,15):
        c,s=np.cos(np.deg2rad(degrees)),np.sin(np.deg2rad(degrees))
        tilted=initial_extrinsic.copy();tilted[:3,:3]=initial_extrinsic[:3,:3]@np.array([[c,0,s],[0,1,0],[-s,0,c]])
        accum=np.zeros((6,40000))
        for stamp,pose in poses:
            if previous-stamp>30.:continue
            world=pose@np.linalg.inv(initial_extrinsic)@tilted
            direction=beams@world[:3,:3].T
            distance=np.full(len(beams),np.inf)
            np.divide(-world[2,3],direction[:,2],out=distance,where=direction[:,2]<-1e-8)
            good=(distance>float(attrs["minRange"])+1e-5)&(distance<=5.)
            p=world[:3,3]+direction[good]*distance[good,None]-np.r_[spec["center"],0.]
            ix=np.floor((p[:,:2]+10)/.1).astype(int)
            inside=(ix>=0).all(1)&(ix<200).all(1);p=p[inside];ix=ix[inside]
            ids=ix[:,1]*200+ix[:,0]
            for k,weight in enumerate([np.ones(len(p)),p[:,0],p[:,1],p[:,0]**2,p[:,0]*p[:,1],p[:,1]**2]):
                accum[k]+=np.bincount(ids,weights=weight,minlength=40000)
        n=np.maximum(accum[0],1);a=accum[3]/n-(accum[1]/n)**2;b=accum[4]/n-accum[1]*accum[2]/n**2;d=accum[5]/n-(accum[2]/n)**2
        area=((a+d-np.hypot(a-d,2*b))*.5>=1e-6).reshape(200,200)
        hit=(accum[0]>0).reshape(200,200)
        result=dict(pitch_deg=degrees,ideal_hit_coverage=float(hit[my,mx][flat].mean()),
                    ideal_area_support_coverage=float(area[my,mx][flat].mean()))
        results.append(result);print(result,flush=True)
    with (out/"sampling_bounds.json").open("x") as stream:
        json.dump(dict(capture=str(cap),raw_sha256=hashes,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            results=results,active_lifetime_s=30.,no_occlusion=True,mapper_input=False,acceptance=False,
            note=__doc__),stream,indent=2)


if __name__=="__main__":main()
