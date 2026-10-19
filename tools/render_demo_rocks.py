#!/usr/bin/env python3
"""Standalone coupled-run report, preserving raw maps and actual GT rock outlines."""

import argparse
import html
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from replay_local_terrain import sha


def diagram(example, record, geometry, home):
    layers = np.asarray(example["classes"])
    height, width = layers.shape
    cells = []
    colors = {-1: "#cbd5e1", 0: "#65b784", 20: "#a1d6af", 100: "#d96363"}
    for y, x in np.ndindex(layers.shape):
        cells.append(
            f'<rect x="{10*x}" y="{10*(height-1-y)}" width="10" height="10" ' f'fill="{colors[int(layers[y, x])]}"/>'
        )
    transform = np.asarray(record["local_from_map"])
    origin, resolution = np.asarray(record["origin"]), record["resolution"]

    def screen(xy):
        p = (xy - origin) / resolution
        return [10 * p[0], 10 * (height - p[1])]

    for index, obj in enumerate(geometry["objects"]):
        points = np.asarray(obj["points"]) - home
        local = points @ transform[:3, :3].T + transform[:3, 3]
        # Draw equator in its authored order; no enlarged matching halo.
        ring = local[1 + 5 * 32 : 1 + 6 * 32]
        outline = " ".join(f"{x:.2f},{y:.2f}" for x, y in map(screen, ring[:, :2]))
        x, y = screen(local[0, :2])
        cells.append(f'<polygon points="{outline}" fill="none" stroke="#171717" stroke-width="2"/>')
        cells.append(f'<text x="{x+9:.2f}" y="{y-8:.2f}" font-size="16" font-weight="bold">{index}</text>')
    base = np.asarray(example["local_from_base"])
    a, b = screen(base[:2, 3]), screen(base[:2, 3] + 0.35 * base[:2, 0])
    cells.append(
        f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" '
        'stroke="#1749b5" stroke-width="5" marker-end="url(#arrow)"/>'
    )
    return (
        f'<svg viewBox="0 0 {width*10} {height*10}" role="img" aria-label="Final raw terrain map">'
        '<defs><marker id="arrow" markerWidth="5" markerHeight="5" refX="4" refY="2.5" orient="auto">'
        '<path d="M0,0 L5,2.5 L0,5" fill="#1749b5"/></marker></defs>' + "".join(cells) + "</svg>"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("mapper", "objects", "observation", "pose"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    for folder in (args.mapper, args.objects, args.observation, args.pose):
        check_run(folder)
    paths = [
        args.objects / "report.json",
        args.observation / "report.json",
        args.pose / "pose_diagnostic.json",
        args.mapper / "manifest.json",
        args.mapper / "projections.jsonl",
        Path(__file__),
    ]
    objects, observation, pose, manifest = [json.loads(p.read_text()) for p in paths[:4]]
    if not all(r["completed"] for r in (objects, observation, pose)):
        raise ValueError("completed assessments required")
    if sha(args.objects / "objects.jsonl") != objects["output_sha256"]["objects.jsonl"]:
        raise ValueError("object evidence changed")
    if sha(args.observation / "observation.jsonl") != observation["output_sha256"]["observation.jsonl"]:
        raise ValueError("observation evidence changed")
    # Both assessments must refer to this exact live mapper, not another run.
    mapper_sha = sha(args.mapper / "manifest.json")
    for report in (objects["exact_replay"], observation["exact_replay"]["exact_raw_map_replay"]):
        if mapper_sha not in report["input_sha256"].values():
            raise ValueError("assessment does not match mapper")
    capture = args.archive_root / manifest["request"]["capture"]
    if sha(capture / "manifest.json") not in pose["input_sha256"].values():
        raise ValueError("pose diagnostic does not match capture")
    geometry_path = capture / "demo_rocks.json"
    if sha(geometry_path) != objects["profile"]["geometry_sha256"]:
        raise ValueError("geometry changed")
    paths.append(geometry_path)
    geometry = json.loads(geometry_path.read_text())
    example = observation["examples"][-1]
    record = next(
        json.loads(line) for line in paths[4].open() if json.loads(line)["publication_ns"] == example["query_ns"]
    )
    svg = diagram(example, record, geometry, np.asarray(manifest["request"]["home_xyz"]))
    accuracy = pose["results"]["first_published"]["overall"]
    rows = "".join(
        f"<tr><td>{html.escape(r['path'].split('/')[-1])}</td>"
        f"<td>{r['sampled_native_hits']}</td><td>{r['maps_detected']}/{objects['maps']}</td>"
        f"<td>{'Occupied' if r['final']['detected'] else 'Not detected'}</td></tr>"
        for r in objects["per_object"]
    )
    bands = observation["final_floor"]
    floors = "".join(
        f"<li>{html.escape(k)}: {v['free']}/{v['samples']} classified free; " f"{v['occupied']} occupied</li>"
        for k, v in bands.items()
    )
    reference_notice = (
        ""
        if pose["native_surface_gate"]["passed"]
        else (
            '<p class="notice"><b>Strict sensor-reference audit failed.</b> '
            "At least one independently recast ray disagreed with the recorded sensor. "
            "The measurements below remain diagnostic, not full terrain/navigation qualification. "
            "Original returns and thresholds are unchanged; see the accompanying status report.</p>"
        )
    )
    movement = observation["exact_replay"]["live"]["movement"]["all"]
    document = f"""<!doctype html><meta charset="utf-8"><title>10 × 15 cm coupled test</title>
<style>body{{font:17px system-ui;max-width:1100px;margin:32px auto;padding:0 20px;color:#183047}}
svg{{max-width:680px;width:100%;border:1px solid #aaa}}table{{border-collapse:collapse}}td,th{{padding:8px 18px;border-bottom:1px solid #ddd;text-align:left}}
.notice{{background:#fff2cb;padding:16px}}h1{{font-size:30px}}</style>
<h1>10 × 15 cm rocks: GLIM + traversability</h1>
<p class="notice">Real simulator LiDAR and live GLIM/terrain. GT-seeded, synthetic RTK-assisted,
GT-controlled acquisition—not autonomous navigation. Eight added test rocks, not all natural obstacles.
Rocks are 1.8–2.65 m from startup; this does not qualify rock detection inside the near-field blind band.</p>
<p><b>{objects['ever_detected']}/8</b> test rocks detected during the run;
<b>{objects['final_detected']}/8</b> occupied in the final map.
GLIM horizontal error: <b>{100*accuracy['horizontal_m']['absolute_p95']:.1f} cm p95</b>,
{100*accuracy['horizontal_m']['absolute_max']:.1f} cm maximum.
Native sensor audit: <b>{'passed' if pose['native_surface_gate']['passed'] else 'failed'}</b>.</p>{reference_notice}
<p>Supported straight exits beyond startup: <b>{observation['decisions_with_supported_exit']}/{observation['decisions']}</b>
decision times. Small moves/turns were available at
<b>{movement['short_move_or_turn_available']}/{movement['decisions']}</b> decision times;
these are not counted as exits.</p>
<h2>Final raw terrain grid</h2><p>10 cm cells; full 6 m map. Green: classified ground; gray: unknown;
red: occupied. Black outlines: actual test-rock geometry (evaluator only); numbers match the table.
Blue: robot heading. No startup assumption or inflated detection halo is painted into the grid.</p>{svg}
<h2>Every test rock retained</h2><table><tr><th>Rock</th><th>Sampled native hits</th><th>Maps detected</th><th>Final</th></tr>{rows}</table>
<p>Zero sampled hits does not prove zero sensor returns. The western rock is behind an existing natural boulder;
it remains in the denominator. Detection requires an occupied cell overlapping the actual footprint.</p>
<h2>Independent clear-floor samples</h2><ul>{floors}</ul>
<p>Source capture: {html.escape(capture.name)}. Unchanged 8 cm obstacle-height threshold.
Raw maps and advisory decisions exact-replayed. No global SLAM correction enabled; no fitted GT alignment.</p>"""
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    (output / "report.html").write_text(document)
    (output / "manifest.json").write_text(
        json.dumps(
            dict(input_sha256={str(p): sha(p) for p in paths}, html_sha256=sha(output / "report.html")), indent=2
        )
    )
    print(output / "report.html")


if __name__ == "__main__":
    main()
