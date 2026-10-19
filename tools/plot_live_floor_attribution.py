#!/usr/bin/env python3
"""Plot the last valid paired terrain maps; descriptive, never a coverage gate."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch, Polygon
import numpy as np

from archive_guard import allocate_run, check_run
from replay_local_terrain import sha
from revision_evidence import validated_transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    source = args.source.resolve(strict=True)
    report_path, data_path = source / "report.json", source / "last_valid_layers.npz"
    report = json.loads(report_path.read_text())
    hashes = {str(p): sha(p) for p in (report_path, data_path, Path(__file__))}
    if not report["diagnostic_completed"] or not report["exact_slam_mirror"]:
        raise ValueError("completed exact paired reconstruction required")
    if hashes[str(data_path)] != report["output_sha256"][data_path.name]:
        raise ValueError("spatial layers changed")
    output = allocate_run(source.parent)
    print(f"Run: {output}", flush=True)
    with np.load(data_path, allow_pickle=False) as data:
        origin, resolution = data["origin"], float(data["resolution"])
        transform = validated_transform(data["local_from_map"])
        base = validated_transform(data["local_from_base"])
        points = (data["floor_world_points"] - data["home"]) @ transform[:3, :3].T + transform[:3, 3]
        outline = np.array([[-0.5037, -0.28545, 0], [0.5037, -0.28545, 0], [0.5037, 0.28545, 0], [-0.5037, 0.28545, 0]])
        outline = outline @ base[:3, :3].T + base[:3, 3]
        colors = ["#cbd5e1", "#60a875", "#d0df74", "#dc624e"]
        fig, axes = plt.subplots(1, 2, figsize=(12, 6.3), constrained_layout=True)
        for ax, name, title in zip(
            axes, ("slam", "gt_pose"), ("Recorded live SLAM placements", "Exact GT-pose substitution")
        ):
            layers = data[name]
            cost = layers[:, :, 0]
            display = np.zeros_like(cost, dtype=int)
            for i, value in enumerate((-1, 0, 20, 100)):
                display[cost == value] = i
            ax.imshow(
                display,
                origin="lower",
                interpolation="nearest",
                cmap=ListedColormap(colors),
                norm=BoundaryNorm(np.arange(-0.5, 4.5), 4),
                extent=[
                    origin[0],
                    origin[0] + cost.shape[1] * resolution,
                    origin[1],
                    origin[1] + cost.shape[0] * resolution,
                ],
            )
            ax.scatter(points[:, 0], points[:, 1], s=0.6, c="#0f172a", alpha=0.25)
            ax.add_patch(Polygon(outline[:, :2], fill=False, edgecolor="#111827", linewidth=2))
            ax.arrow(
                base[0, 3],
                base[1, 3],
                base[0, 0] * 0.8,
                base[1, 0] * 0.8,
                width=0.018,
                head_width=0.12,
                length_includes_head=True,
                color="#111827",
            )
            ax.set(title=title, xlabel="Fixed local X (m)", ylabel="Fixed local Y (m)", aspect="equal")
        fig.suptitle(
            "Last valid map: matched grid, returns, ages and projection time\n"
            "Dots: independent floor survey • outline/arrow: estimated chassis/front",
            fontsize=12,
        )
        axes[1].legend(
            handles=[
                Patch(color=c, label=n)
                for c, n in zip(colors, ("Unknown", "Measured support", "Inferred support", "Occupied"))
            ],
            loc="upper right",
            fontsize=8,
        )
        fig.savefig(output / "paired-floor-maps.png", dpi=160)
        plt.close(fig)
    if any(sha(path) != digest for path, digest in hashes.items()):
        raise ValueError("plot input changed")
    check_run(output)
    with (output / "report.json").open("x") as stream:
        json.dump(
            dict(
                completed=True,
                acceptance=False,
                posthoc_visualization_only=True,
                source=str(source),
                input_sha256=hashes,
                output_sha256={"paired-floor-maps.png": sha(output / "paired-floor-maps.png")},
            ),
            stream,
            indent=2,
        )


if __name__ == "__main__":
    main()
