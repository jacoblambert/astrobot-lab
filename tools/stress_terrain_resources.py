#!/usr/bin/env python3
"""Fixed adversarial resource profile, not physically observed terrain accuracy.

Fill all 200x200 cells, four height layers and 30s of immutable buckets with
48,000 points per 10Hz input. This tests a previously unproven memory/runtime
bound; it does not replace simulator or live latency validation. The separate
dense_flat profile uses the same input count/rate/size, but fills one height
layer with broad XY patches to stress free-surface connectivity and support.
"""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import time

import numpy as np

from archive_guard import allocate_run
from layered_terrain import TerrainMapper


def stress_chunks(profile="layer_capacity"):
    if profile not in ("layer_capacity","dense_flat"):
        raise ValueError("unknown fixed resource profile")
    y,x=np.indices((200,200))
    centers=np.column_stack([(x.ravel()+.5)*.1-10.,(y.ravel()+.5)*.1-10.])
    chunks=[]
    for phase in range(10):
        xy=centers[phase::10]
        if profile=="layer_capacity":
            samples=[(height,dx,dy) for height in (0.,.04,.08,.16)
                     for dx,dy in ((-.01,-.01),(.01,-.01),(0.,.01))]
        else:
            samples=[(0.,dx,dy) for dx in (-.035,0.,.035) for dy in (-.035,-.01,.01,.035)]
        chunks.append(np.vstack([np.column_stack([xy+[dx,dy],np.full(len(xy),height)])
                                 for height,dx,dy in samples]))
    return chunks


def run(library, output, profile="layer_capacity"):
    chunks=stress_chunks(profile)
    timings=[]
    with TerrainMapper(library) as mapper:
        for i in range(320):
            start=time.perf_counter()
            mapper.integrate(chunks[i%10],i/10.,[0.,0.,.5])
            integrated=time.perf_counter()
            layers=mapper.project(i/10.)
            finished=time.perf_counter()
            rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.
            timings.append(dict(stamp=i/10.,integrate_ms=(integrated-start)*1000,
                                project_ms=(finished-integrated)*1000,total_ms=(finished-start)*1000,rss_mib=rss))
            if rss>2048.:
                raise ValueError("stress diagnostic exceeded its 2GiB stop bound")
            if (i+1)%50==0:
                print(f"{i+1}/320 inputs; peak RSS {rss:.1f} MiB",flush=True)
        with (output/"final_layers.npz").open("xb") as stream:
            np.savez_compressed(stream,layers=layers)
    durations=[item["total_ms"] for item in timings]
    p95,p99=map(float,np.percentile(durations,[95,99]))
    result=dict(profile=__doc__,profile_name=profile,frames=timings,points_per_input=48000,inputs=320,
                library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                peak_process_rss_mib=rss,p95_ms=p95,p99_ms=p99,
                memory_passed=rss<=1024.,compute_passed=p95<50. and p99<100.,acceptance=False)
    with (output/"summary.json").open("x") as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ("frames","profile")},indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library",type=Path,required=True)
    parser.add_argument("--archive-root",type=Path,default=Path("/archive"))
    parser.add_argument("--profile",choices=("layer_capacity","dense_flat"),default="layer_capacity")
    args=parser.parse_args()
    output=allocate_run(args.archive_root)
    print(f"Run: {output}",flush=True)
    try:
        run(args.library,output,args.profile)
    except Exception as exc:
        with (output/"failure.json").open("x") as stream:
            json.dump(dict(type=type(exc).__name__,error=str(exc)),stream,indent=2)
        raise


if __name__=="__main__":
    main()
