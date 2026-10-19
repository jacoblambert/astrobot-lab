#!/usr/bin/env python3
"""Plain-language, portable HTML from the recorded-map navigation-gap assessment."""

import argparse
import html
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from assess_navigation_gaps import candidate_motions, motion_score
from local_terrain.physx_reference import BodyBox
from local_terrain.support_sweep import swept_polygons
from render_gap_report import grid_svg
from replay_local_terrain import sha


def percent(value, count):
    return f"{100 * value / count:.1f}%" if count else "—"


def render(source, output, explanation=None, grounded=None):
    report = json.loads((source / "report.json").read_text())
    if not report["diagnostic_completed"]:
        raise ValueError("completed diagnostic required")
    hashes = {str(source / "report.json"): sha(source / "report.json"), str(Path(__file__)): sha(Path(__file__))}
    details = []
    if explanation is not None:
        path = explanation / "report.json"
        reasons = json.loads(path.read_text())
        if reasons["input_sha256"].get(str(source / "report.json")) != hashes[str(source / "report.json")]:
            raise ValueError("explanation belongs to another assessment")
        reason_path = explanation / "reasons.jsonl"
        if sha(reason_path) != reasons["output_sha256"]["reasons.jsonl"]:
            raise ValueError("changed rejection explanation")
        hashes.update({str(p): sha(p) for p in (path, reason_path)})
        rows = [json.loads(line) for line in reason_path.open()]
        after = [r for r in rows if r["evidence_ns"] >= rows[0]["evidence_ns"] + 15 * 10**9]
        unknown = sum(r["unknown"] for r in after)
        dropoff = sum(r["unknown_reasons"]["NEGATIVE_EDGE"] for r in after)
        no_returns = sum(r["unknown_reasons"]["NO_RETURNS"] for r in after)
        details.append(f"<p><b>Why current wheel support is missing:</b> After the first 15 s, "
                       f"{dropoff:,}/{unknown:,} ({percent(dropoff, unknown)}) unknown wheel-cell exposures "
                       f"have the possible-dropoff reason; {no_returns:,} have the no-returns reason. "
                       "An exact replay reproduced all original maps before reading these reasons. "
                       "Other unknown cells may have insufficient area or unsupported extent. "
                       "A reason code alone does not identify which input triggered it.</p>")
    if grounded is not None:
        path = grounded / "report.json"
        comparison = json.loads(path.read_text())
        if comparison["input_sha256"].get(str(source / "report.json")) != hashes[str(source / "report.json")]:
            raise ValueError("grounded-start comparison belongs to another assessment")
        hashes[str(path)] = sha(path)
        totals = comparison["summary"]["all"]
        details.append(f"<p>Even an optimistic comparison that ignores unknown cells under the current "
                       f"wheel patches allows a move or turn in only {totals['progress_or_turn']}/{totals['samples']} "
                       f"samples ({percent(totals['progress_or_turn'], totals['samples'])}). "
                       "This assumption was tested offline, "
                       "not enabled: standing on part of a cell does not prove its whole area safe.</p>")
    pieces = ["""<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Do terrain gaps prevent a useful next move?</title><style>
body{font:17px/1.55 system-ui,sans-serif;color:#213047;background:#f4f6f9;margin:auto;max-width:1120px;padding:28px}
h1{line-height:1.15;font-size:36px}h2{margin-top:36px}table{border-collapse:collapse;background:white;width:100%}
td,th{padding:10px;text-align:left;border-bottom:1px solid #d5dce5}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:18px}
article{background:white;padding:16px;border-radius:12px}svg{width:100%;display:block}.note{background:#e7edf5;padding:18px;border-radius:10px}
small{color:#475569}code{overflow-wrap:anywhere}</style>
<h1>Do terrain gaps prevent a useful next move?</h1>
<p>The map is usually clear of false obstacles. But “no obstacle detected” and
“enough ground support for the next move” are different questions.</p>
<div class="note">This is an offline check of a completed live route, using actual GLIM-based maps.
The robot followed a GT-controlled, pre-surveyed clear route. This was not autonomous navigation.
Nothing here changes the mapper or authorizes movement.</div>
<h2>1. Can the map support progress?</h2>
<p>Test 25 cm and 50 cm forward/steering moves and ±30° turns.
Count a move toward the next waypoint only if it reduces distance by at least 5 cm.
Wheel paths must be supported; known obstacles anywhere in the swept body footprint block the move.
Unknown squares between the wheels do not automatically count as missing wheel support.</p>
<table><tr><th>Period</th><th>Samples</th><th>Move toward goal</th><th>Move or turn</th><th>Entire footprint free, toward goal</th></tr>"""]
    for name, label in (("all", "Entire run"), ("first_15s", "First 15 s"),
                        ("after_15s", "After first 15 s"), ("translation", "Translation legs"), ("rotation", "Turning legs")):
        row = report["summary"][name]
        n = row["samples"]
        pieces.append(f"<tr><td>{label}</td><td>{n}</td><td>{percent(row['progress_available'], n)}</td>"
                      f"<td>{percent(row['progress_or_turn'], n)}</td><td>{percent(row['whole_footprint_progress'], n)}</td></tr>")
    pattern = report["contract"].get("route_pattern", report["grids"].get("pattern", "loop16"))
    pieces.append("</table><p>Route: <code>" + html.escape(pattern) + "</code>. "
                  "Candidates do not include reverse. A short out/back return can have a clear reverse "
                  "path without scoring forward progress.</p>" + "".join(details))
    pieces.append("""<p><small>First map in every fixed one-second bin, including startup.
Turning can help observe ground, but an available turn is not proof it will reveal a useful path.
These are geometric map-availability checks, not collision, braking or tire-contact certification.</small></p>
<h2>2. Does the map recognize independently known-clear ground?</h2>
<p>Every recorded valid map is compared with the simulator’s pre-motion floor survey (5 cm spacing).
Unknown and outside-map points remain in the denominator. This measures sampled clear ground, not obstacle recall.</p>
<table><tr><th>Position</th><th>Sample exposures</th><th>Marked free</th><th>Marked occupied</th><th>Unknown</th></tr>""")
    for region, row in report["floor_totals"].items():
        if region == "whole_survey":
            continue
        label = region.replace("_", " ")
        n = row["samples"]
        pieces.append(f"<tr><td>{label}</td><td>{n:,}</td><td>{percent(row['free'], n)}</td>"
                      f"<td>{100 * row['occupied'] / n:.4f}%</td><td>{percent(row['unknown'], n)}</td></tr>")
    pieces.append("""</table><h2>3. Where do the wheel paths meet gaps?</h2>
<p>Each square is 10 cm. Green: measured support. Yellow-green: inferred support.
Gray: unknown. Red: occupied. Blue outlines: the four wheel-patch sweeps for a
50 cm straight move. Black outlines: chassis and the larger acquisition guard.
The pictures use the same fixed display times as the earlier gallery, not selected successes.</p><div class="cards">""")
    patches = [BodyBox(p["center"], p["half_extent"], "declared_assumed_patch") for p in report["contract"]["patches"]]
    body = BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "declared_guard")
    for example in report["grids"]["examples"]:
        path = source / example["file"]
        if sha(path) != example["sha256"]:
            raise ValueError("changed displayed grid")
        hashes[str(path)] = sha(path)
        with np.load(path, allow_pickle=False) as data:
            classes = data["classes"]
            record = {key: data[key] for key in ("origin", "resolution", "local_from_base", "local_from_map")}
            poses = candidate_motions(record["local_from_base"])["move_0.50_+0deg"]
            score = motion_score(classes, record, poses, patches, body, poses[-1][:2, 3])
            svg = grid_svg(classes, record["origin"], record["local_from_base"], float(record["resolution"]), zoom=True)
            overlay = []
            for polygon in swept_polygons(poses, patches):
                xy = (polygon - record["origin"]) / record["resolution"]
                xy[:, 1] = 60 - xy[:, 1]
                coords = " ".join(f"{x:.4f},{y:.4f}" for x, y in xy)
                overlay.append(f'<polygon points="{coords}" fill="none" stroke="#1747b0" stroke-width=".09"/>')
            svg = svg.replace("</svg>", "".join(overlay) + "</svg>")
            pieces.append(f"<article><h3>+{example['elapsed_s']:.1f} s</h3>{svg}<p>50 cm straight: "
                          f"<b>{score['state']}</b>. Unknown wheel-path cells: {score['wheel_support']['unknown']}.</p></article>")
    pieces.append("""</div><h2>What this does and does not tell us</h2>
<p>Do not turn every gray square into an obstacle, or require perfect map-wide coverage.
But a wheel-support gap can stop every candidate even when the actual floor is clear.
That calls for separating already-supported ground, new ground to inspect, and real hazards,
and checking why the dropoff detector rejects surveyed clear terrain.</p>
<p>No returns can lower concern about a large rock only where rays actually tested its possible volume.
It does not rule out small obstacles, holes or unseen terrain. This clear-route recording does not
measure missed obstacles or calibrate a probability that an unknown cell is safe.</p>
<p><small>Assumed 80×120 mm wheel patches at measured wheel centers; 1.70×1.40 m body guard.
No extra localization uncertainty margin. Finite floor samples, not continuous terrain truth.
Startup movement was GT-guarded, so this does not demonstrate self-starting from the initial map.</small></p>""")
    pieces.append(f"<p><small>Source: <code>{html.escape(source.name)}</code> · "
                  f"{report['grids']['recorded_projections']} recorded maps · "
                  f"{len(report['grids']['missing_projection_stamps'])} missing maps.</small></p></html>")
    for path, digest in hashes.items():
        if sha(path) != digest:
            raise ValueError("report source changed")
    (output / "navigation-gaps.html").write_text("".join(pieces))
    (output / "manifest.json").write_text(json.dumps(dict(input_sha256=hashes,
        output_sha256=sha(output / "navigation-gaps.html"), acceptance=False), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    parser.add_argument("--explanation", type=Path)
    parser.add_argument("--grounded", type=Path)
    args = parser.parse_args()
    check_run(args.source)
    output = allocate_run(args.archive_root, limit=1024**3)
    render(args.source, output, args.explanation, args.grounded)
    check_run(output)
    print(output / "navigation-gaps.html")


if __name__ == "__main__":
    main()
