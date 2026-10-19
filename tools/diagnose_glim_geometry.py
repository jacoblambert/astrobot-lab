#!/usr/bin/env python3
"""Archive installed front-end point/normal observability; no GT or estimator tuning."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
from archive_guard import allocate_run, check_run
from shadow_pose_replay import sha


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("run",type=Path)
    p.add_argument("--probe",type=Path,required=True)
    p.add_argument("--archive-root",type=Path,default=Path("/archive"))
    a=p.parse_args(); output=allocate_run(a.archive_root);print(f"Run: {output}",flush=True)
    try:
        check_run(a.run)
        report=json.loads((a.run/"shadow_pair_audit.json").read_text())
        if not report["passed"] or (a.run/"failure.json").exists(): raise ValueError("failed input")
        for path,expected in report["input_sha256"].items():
            if sha(Path(path))!=expected: raise ValueError("changed input provenance")
        attempt=json.loads((a.run/"attempt.json").read_text())
        if attempt["execution_profile"]!="serial": raise ValueError("serial reference required")
        shutil.copy2(a.probe,output/"glim_geometry_probe")
        shutil.copy2(Path(__file__),output/Path(__file__).name)
        shutil.copy2(Path(__file__).with_name("glim_shadow")/"geometry.cpp",output/"geometry.cpp")
        with (output/"probe.log").open("x") as log:
            subprocess.run([str(output/"glim_geometry_probe"),str(a.run/"config"),
                str(Path(attempt["prepared"])/"inputs"),str(output/"geometry.jsonl")],stdout=log,stderr=log,check=True,timeout=180)
        rows=[json.loads(line) for line in (output/"geometry.jsonl").read_text().splitlines()]
        trace=[json.loads(line) for line in (a.run/"estimator_trace.jsonl").read_text().splitlines()]
        actual=[(r["stamp_ns"],r["points"]) for r in rows]
        expected=[(r["stamp_ns"],r["points"]) for r in trace if r["stage"]=="processed_input"]
        if actual!=expected: raise ValueError("geometry input coverage/count mismatch")
        result=dict(completed=True,acceptance=False,frames=len(rows),
            normal_eigenvalues_p0_p50_p100=np.percentile([r["normal_eigenvalues"] for r in rows],[0,50,100],axis=0).tolist(),
            information_eigenvalues_p0_p50_p100=np.percentile([r["point_plane_information_eigenvalues"] for r in rows],[0,50,100],axis=0).tolist(),
            note="Per-point mean unweighted point-to-plane Jacobian information, IMU local chart; not calibrated covariance or correspondence validity",
            input_sha256={str(a.run/"shadow_pair_audit.json"):sha(a.run/"shadow_pair_audit.json")},
            output_sha256={n:sha(output/n) for n in ("glim_geometry_probe","geometry.jsonl","geometry.cpp","probe.log")})
        with (output/"geometry_report.json").open("x") as stream: json.dump(result,stream,indent=2,allow_nan=False)
        print(json.dumps(result,indent=2))
    except Exception as exc:
        with (output/"failure.json").open("x") as stream: json.dump(dict(error=str(exc)),stream)
        raise


if __name__=="__main__":main()
