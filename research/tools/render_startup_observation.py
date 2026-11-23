#!/usr/bin/env python3
"""Self-contained raw-grid examples from a completed startup observation assessment."""

import argparse
import html
import json
from pathlib import Path

from archive_guard import allocate_run, check_run
from replay_local_terrain import sha


def render(report):
    cards = []
    for e in report["examples"]:
        b = e["local_from_base"]
        cx, cy, r = b[0][3], b[1][3], e["resolution"]
        cells = []
        for y, row in enumerate(e["classes"]):
            for x, value in enumerate(row):
                px, py = e["origin"][0] + x * r, e["origin"][1] + y * r
                if px + r < cx - 1.5 or px > cx + 1.5 or py + r < cy - 1.5 or py > cy + 1.5:
                    continue
                color = "#cbd5e1" if value == -1 else "#c84a44" if value == 100 else "#6bc4a1"
                cells.append(
                    f'<rect x="{px}" y="{py}" width="{r}" height="{r}" fill="{color}" stroke="white" stroke-width=".002"/>'
                )
        cells.append(
            f'<circle cx="{cx}" cy="{cy}" r=".035" fill="#172554"/><path d="M{cx},{cy} L{cx+.3*b[0][0]},{cy+.3*b[1][0]}" stroke="#172554" stroke-width=".025"/>'
        )
        c = e["floor"]["outside_startup_to_1m"]
        pct = 100 * c["free"] / c["samples"]
        cards.append(
            f'<article><h2>{html.escape(e["label"])} · {e["elapsed_s"]:.1f} s</h2><svg role="img" aria-label="Raw terrain grid" viewBox="{cx-1.5} {-cy-1.5} 3 3"><g transform="scale(1,-1)">{"".join(cells)}</g></svg><p>Support outside startup, to 1 m: <b>{pct:.1f}%</b></p></article>'
        )
    exits = [n.replace("_", " ") for n, d in report["final_decisions"].items() if d["supported_exit"]]
    return f"""<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Startup observation</title><style>
body{{font:16px/1.55 system-ui;color:#14243a;background:#f6f8fb;max-width:1200px;margin:36px auto;padding:0 20px}}
h1{{font-size:30px}}h2{{font-size:17px}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:18px}}
article{{padding:16px;background:white;border:1px solid #dae1e9;border-radius:12px}}svg{{width:100%;background:#cbd5e1}}
.notice{{padding:14px;background:#fff1cb;border-left:4px solid #cc9a23}}p{{max-width:1000px}}
</style><h1>Can observation reveal a way out?</h1>
<p>The initial support assumption is fixed at startup: the physical turning sweep, about <b>61 cm radius</b>.
It never follows the rover and is not painted into these maps. Sensor, GLIM and terrain classifier unchanged.</p>
<p class="notice">GT-controlled acquisition, not terrain-controlled navigation. Exact live-map replay plus offline
motion checks. Clear-floor samples do not measure obstacle recall; no navigation qualification is claimed.</p>
<h2>Result</h2><p><b>{report['decisions_with_supported_exit']} / {report['decisions']}</b> sampled decisions have a supported
straight exit requiring ground beyond both the initial area and current wheel patches.
Final available exits: <b>{html.escape(', '.join(exits) or 'none')}</b>.</p>
<p>Actual net rotation: {report['physical_motion']['net_yaw_degrees']:.1f}°.
Maximum base displacement from startup: {100*report['physical_motion']['max_origin_displacement_m']:.1f} cm.
All {report['maps']} valid maps retained and exact-replayed.</p>
<h2>Raw map examples</h2><p>Green: measured/inferred ground. Gray: unknown. Red: occupied. Blue arrow: rover pose.
Squares are 10 cm; panels span 3 m and track the rover. Examples are the first map after each quarter-turn,
plus the final map. Scores include every decision, not just these examples.</p>
<div class="cards">{''.join(cards)}</div></html>"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assessment", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    check_run(args.assessment)
    source = args.assessment / "report.json"
    report = json.loads(source.read_text())
    if (
        not report["completed"]
        or sha(args.assessment / "observation.jsonl") != report["output_sha256"]["observation.jsonl"]
    ):
        raise ValueError("completed unchanged assessment required")
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    (output / "startup-observation.html").write_text(render(report))
    (output / "manifest.json").write_text(
        json.dumps(
            dict(
                input_sha256={str(source): sha(source), str(Path(__file__)): sha(Path(__file__))},
                html_sha256=sha(output / "startup-observation.html"),
            ),
            indent=2,
        )
    )
    check_run(output)
    print(output / "startup-observation.html")


if __name__ == "__main__":
    main()
