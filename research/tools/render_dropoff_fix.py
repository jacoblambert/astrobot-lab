#!/usr/bin/env python3
"""Render before/after maps from the matched false-dropoff experiment."""

import argparse
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from local_terrain.physx_reference import BodyBox
from local_terrain.support_sweep import swept_polygons
from render_gap_report import grid_svg
from replay_local_terrain import sha


def render(assessment, comparison, output):
    before = json.loads((assessment / "report.json").read_text())
    after = json.loads((comparison / "report.json").read_text())
    if not after["completed"] or not after["exact_baseline"]:
        raise ValueError("completed exact-baseline comparison required")
    key = str(assessment / "report.json")
    if after["input_sha256"].get(key) != sha(assessment / "report.json"):
        raise ValueError("comparison belongs to another assessment")
    hashes = {str(p): sha(p) for p in (assessment / "report.json", comparison / "report.json", Path(__file__))}
    pieces = [
        """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>False dropoff fix</title>
<style>body{font:17px/1.55 system-ui,sans-serif;color:#213047;background:#f4f6f9;margin:auto;max-width:1080px;padding:28px}
h1{line-height:1.15;font-size:36px}table{border-collapse:collapse;width:100%;background:white}td,th{padding:10px;text-align:left;border-bottom:1px solid #d5dce5}
.pair{display:grid;grid-template-columns:1fr 1fr;gap:16px}article{background:white;padding:14px;border-radius:10px}svg{width:100%;overflow:hidden}
.note{padding:18px;background:#e7edf5;border-radius:10px}small{color:#475569}@media(max-width:600px){.pair{grid-template-columns:1fr}}</style>
<h1>False dropoff fix: the same ground, a better decision</h1>
<p>The old rule compared the square root of the sum of squared deviations with a 5 mm limit.
That value grows with the number of returns, even if every individual deviation is tiny.
The new rule checks actual returns against the same limit. It does not use an average or discard outliers.</p>
<p class="note">Same recorded LiDAR, GLIM poses, 10 cm grid, 60 s evidence lifetime,
route, floor samples and motion queries. The left images are reproduced original outputs;
the right images are offline replays with the fix, not a new live drive.</p>
<h2>Useful short motions</h2><table><tr><th>Question</th><th>Before</th><th>With fix</th></tr>"""
    ]
    for period, metric, label in (
        ("all", "progress_available", "Move toward the goal, including startup"),
        ("all", "progress_or_turn", "Move or turn, including startup"),
        ("after_15s", "progress_available", "Move toward the goal, after 15 s"),
        ("after_15s", "progress_or_turn", "Move or turn, after 15 s"),
    ):
        cells = []
        for report in (before, after):
            row = report["summary"][period]
            cells.append(f"{row[metric]}/{row['samples']} ({100*row[metric]/row['samples']:.1f}%)")
        pieces.append(f"<tr><td>{label}</td><td>{cells[0]}</td><td>{cells[1]}</td></tr>")
    pieces.append(
        """</table><p>One fixed sample per second. Swept wheel patches require support;
the body guard must avoid known obstacles. A turn is an opportunity to gather information,
not proof of goal completion. The original drive used GT guards, so this does not prove autonomous startup.</p>
<h2>Recognition of independently surveyed clear ground</h2>
<table><tr><th>Region</th><th>Marked free before</th><th>Marked free with fix</th><th>False occupied with fix</th></tr>"""
    )
    for region in ("front_0_1m", "front_1_2m", "front_2_3m", "rear_0_1m", "rear_1_2m", "rear_2_3m"):
        a, b = before["floor_totals"][region], after["floor_totals"][region]
        if a["samples"] != b["samples"]:
            raise ValueError("changed floor denominator")
        pieces.append(
            f"<tr><td>{region.replace('_', ' ')}</td><td>{100*a['free']/a['samples']:.2f}%</td>"
            f"<td>{100*b['free']/b['samples']:.2f}%</td><td>{100*b['occupied']/b['samples']:.4f}%</td></tr>"
        )
    pieces.append(
        """</table><p>All recorded usable maps, startup included. Unknown/outside stays in the denominator.
This is clear-floor recognition, not missed-obstacle recall. Rock, wall, hole and slope regression fixtures
are a separate check; fresh live hazard-route qualification remains open.</p>
<h2>Fixed-time map comparisons</h2><p>Green: support. Gray: unknown. Red: occupied.
Black: chassis/guard. Purple: current assumed wheel patches. Each square is 10 cm.</p>"""
    )
    patches = [BodyBox(p["center"], p["half_extent"], "same_assumed_patch") for p in before["contract"]["patches"]]
    original_examples = {e["evidence_ns"]: e for e in before["grids"]["examples"]}
    if {e["evidence_ns"] for e in after["examples"]} != set(original_examples):
        raise ValueError("missing fixed display times")
    for example in after["examples"]:
        path = comparison / example["file"]
        if sha(path) != example["sha256"]:
            raise ValueError("changed comparison grid")
        hashes[str(path)] = sha(path)
        old = original_examples[example["evidence_ns"]]
        old_path = assessment / old["file"]
        if sha(old_path) != old["sha256"]:
            raise ValueError("changed original grid")
        hashes[str(old_path)] = sha(old_path)
        pieces.append(f"<h3>+{example['elapsed_s']:.1f} s</h3><div class='pair'>")
        with np.load(path, allow_pickle=False) as data, np.load(old_path, allow_pickle=False) as saved:
            if not np.array_equal(data["baseline"][:, :, 0], saved["classes"]):
                raise ValueError("displayed baseline is not the original recorded grid")
            overlay = []
            for polygon in swept_polygons([data["local_from_base"]], patches):
                xy = (polygon - data["origin"]) / data["resolution"]
                xy[:, 1] = 60 - xy[:, 1]
                coordinates = " ".join(f"{x:.4f},{y:.4f}" for x, y in xy)
                overlay.append(f'<polygon points="{coordinates}" fill="none" stroke="#7125a0" stroke-width=".15"/>')
            for name, label in (("baseline", "Original map"), ("candidate", "Replay with fix")):
                svg = grid_svg(
                    data[name][:, :, 0], data["origin"], data["local_from_base"], float(data["resolution"]), zoom=True
                )
                svg = svg.replace("</svg>", "".join(overlay) + "</svg>")
                pieces.append(f"<article><b>{label}</b>{svg}</article>")
        pieces.append("</div>")
    pieces.append(
        f"<p><small>Comparison: {comparison.name}. Assumed 80×120 mm wheel patches, "
        "1.70×1.40 m guard; no added localization uncertainty or measured braking model. "
        "Offline map availability, not permission to drive.</small></p></html>"
    )
    for path, digest in hashes.items():
        if sha(path) != digest:
            raise ValueError("report input changed")
    (output / "false-dropoff-fix.html").write_text("".join(pieces))
    (output / "manifest.json").write_text(
        json.dumps(
            dict(input_sha256=hashes, html_sha256=sha(output / "false-dropoff-fix.html"), acceptance=False), indent=2
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assessment", type=Path)
    parser.add_argument("comparison", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    for path in (args.assessment, args.comparison):
        check_run(path)
    output = allocate_run(args.archive_root, limit=1024**3)
    render(args.assessment, args.comparison, output)
    check_run(output)
    print(output / "false-dropoff-fix.html")


if __name__ == "__main__":
    main()
