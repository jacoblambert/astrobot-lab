#!/usr/bin/env python3
"""Distinguish initial gauge/body-origin mismatch from time-varying pose error.

First-sample alignments are DIAGNOSTICS ONLY, never mapper corrections or
replacement acceptance scores. No full-trajectory fitting or point selection.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import allocate_run, check_run
from shadow_pose_replay import load_pose_sensitivity, sha
from revision_evidence import validated_transform


def errors(estimated, truth):
    translation = estimated[:, :3, 3]-truth[:, :3, 3]
    angles = (Rotation.from_matrix(truth[:, :3, :3]).inv()*Rotation.from_matrix(estimated[:, :3, :3])).magnitude()
    position = np.linalg.norm(translation, axis=1)
    return dict(position_m=dict(zip(("initial", "median", "p95", "maximum", "final"),
        map(float, (position[0], np.median(position), np.percentile(position,95), position.max(), position[-1])))),
        rotation_deg=dict(zip(("initial", "median", "p95", "maximum", "final"),
        map(float, np.rad2deg((angles[0],np.median(angles),np.percentile(angles,95),angles.max(),angles[-1]))))),
        initial_translation_xyz_m=translation[0].tolist(), final_translation_xyz_m=translation[-1].tolist())


def diagnose(estimated, truth):
    estimated, truth = np.asarray(estimated), np.asarray(truth)
    if estimated.shape != truth.shape or estimated.ndim != 3 or estimated.shape[1:] != (4,4) or len(estimated)<2:
        raise ValueError("matching nonempty SE(3) sequences required")
    for pose in np.concatenate([estimated, truth]):
        validated_transform(pose)
    world_error = estimated[0]@np.linalg.inv(truth[0])
    body_error = np.linalg.inv(truth[0])@estimated[0]
    return dict(raw=errors(estimated, truth),
        after_initial_world_gauge_only=errors(np.linalg.inv(world_error)@estimated, truth),
        after_initial_body_extrinsic_only=errors(estimated@np.linalg.inv(body_error), truth),
        initial_world_error=world_error.tolist(), initial_body_error=body_error.tolist(),
        fitted_samples=1, full_trajectory_fit=False, corrections_applied_to_mapper=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture",type=Path)
    parser.add_argument("recorder",type=Path)
    parser.add_argument("pairs",type=Path,nargs="+")
    parser.add_argument("--archive-root",type=Path,default=Path("/archive"))
    args=parser.parse_args()
    output=allocate_run(args.archive_root,limit=1024**3)
    print(f"Run: {output}",flush=True)
    try:
        for path in (args.capture,args.recorder,*args.pairs): check_run(path)
        native=json.loads((args.capture/"native_timing_audit.json").read_text())["frames"]
        link=json.loads((args.recorder/"shadow_collection.json").read_text())
        base_lidar=validated_transform(link["request"]["base_from_lidar"])
        base_imu=validated_transform(link["request"]["base_from_imu"])
        truth=np.array([f["world_from_lidar"] for f in native])
        truth[:,:3,3]-=link["home_xyz"]
        frames={"lidar":np.eye(4),"base_link":np.linalg.inv(base_lidar),
                "imu":np.linalg.inv(base_lidar)@base_imu}
        result={}
        for pair in args.pairs:
            report=json.loads((pair/"shadow_pair_audit.json").read_text())
            if str(args.recorder/"shadow_collection.json") not in report["input_sha256"]:
                raise ValueError("pair belongs to another recorder")
            result[pair.name]={}
            for mode in ("first_published","final_revised"):
                estimated,_=load_pose_sensitivity(pair,args.capture,native,mode)
                result[pair.name][mode]={name:diagnose(estimated@offset,truth@offset) for name,offset in frames.items()}
        report=dict(completed=True,acceptance=False,native_chunks=len(native),results=result,
            frame_contract="Same physical origin on both sides; native LiDAR -> base/IMU using recorded audited mounts",
            input_sha256={str(p):sha(p) for p in [args.capture/"native_timing_audit.json",
                args.recorder/"shadow_collection.json",Path(__file__),*[p/"shadow_pair_audit.json" for p in args.pairs]]})
        with (output/"anchor_audit.json").open("x") as stream: json.dump(report,stream,indent=2,allow_nan=False)
        print(json.dumps({run:{mode:origins["base_link"] for mode,origins in values.items()} for run,values in result.items()},indent=2))
    except Exception as exc:
        with (output/"failure.json").open("x") as stream: json.dump(dict(error=str(exc)),stream)
        raise


if __name__=="__main__": main()
