#!/usr/bin/env python3
"""Render fixed-reference and candidate terrain diagnostics, without rescoring."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from score_terrain_calibration import reference_labels


def evaluation_reference(capture, evaluation):
    """Use the scored reference, including after a Docker-to-host path move."""
    manifest=json.loads((evaluation/"manifest.json").read_text())
    recorded=Path(manifest.get("reference_capture",str(capture)))
    reference=recorded if recorded.is_dir() else capture.parent/recorded.name
    if not reference.is_dir():
        raise ValueError("scored reference capture is unavailable")
    for name,expected in manifest.get("reference_sha256",{}).items():
        if Path(name).name!=name:
            raise ValueError("invalid reference provenance path")
        if hashlib.sha256((reference/name).read_bytes()).hexdigest()!=expected:
            raise ValueError("reference changed after scoring")
    return reference


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture",type=Path)
    parser.add_argument("evaluation",type=Path)
    args=parser.parse_args()
    target=args.evaluation/"terrain_comparison.png"
    if target.exists():
        raise ValueError("figure already exists")
    reference=evaluation_reference(args.capture,args.evaluation)
    manifest=json.loads((args.evaluation/"manifest.json").read_text())
    geometry=manifest.get("terrain_geometry",dict(origin=[-10.,-10.],resolution=.1,width=200,height=200))
    gx,gy=geometry["origin"]
    map_extent=[gx,gx+geometry["width"]*geometry["resolution"],gy,gy+geometry["height"]*geometry["resolution"]]
    spec=json.loads((reference/"calibration_spec.json").read_text())
    summary=json.loads((reference/"summary.json").read_text())
    tile=np.load(reference/"collision_tile.npz")
    labels,_,_=reference_labels(spec,tile,summary)
    origin=tile["origin"]-spec["center"]
    route=None
    if (args.capture/"multiview.json").exists():
        views=json.loads((args.capture/"multiview.json").read_text())["views"]
        route=np.array([v["actual_xyz"][:2] for v in views])-spec["center"]
    size=np.array(labels.shape[::-1])*float(tile["resolution"])
    fig,axes=plt.subplots(1,3,figsize=(15,5),layout="constrained")
    axes[0].imshow(labels+1,origin="lower",extent=[origin[0],origin[0]+size[0],origin[1],origin[1]+size[1]],
                   cmap=ListedColormap(["#dddddd","#ade0aa","#222222","#b54b98"]),vmin=0,vmax=3,interpolation="nearest")
    axes[0].set_title("Independent visible reference\ngreen: flat; black: hazards; purple: hole")
    for ax,name,title in zip(axes[1:],["valid_range_measured","valid_range_inferred"],
                            ["Measured ground only","Measured + bounded inference"]):
        grid=np.load(args.evaluation/f"{name}.npz")["layers"]
        display=np.zeros(grid.shape[:2],dtype=int)
        display[(grid[:,:,0]>=0)&(grid[:,:,0]<100)]=1
        display[grid[:,:,1]==2]=2
        display[grid[:,:,0]==100]=3
        ax.imshow(display,origin="lower",extent=map_extent,
                  cmap=ListedColormap(["#dddddd","#ade0aa","#459860","#222222"]),vmin=0,vmax=3,interpolation="nearest")
        ax.set_title(title+"\ngray: unknown; green: ground; black: obstacle")
    for ax in axes:
        ax.set(xlim=(-5,5),ylim=(-5,5),xlabel="x from initial base (m)",ylabel="y (m)")
        if route is not None:
            ax.plot(route[:,0],route[:,1],"b--",linewidth=1)
        ax.set_aspect("equal")
    fig.savefig(target,dpi=150)
    plt.close(fig)
    print(target)


if __name__=="__main__":
    main()
