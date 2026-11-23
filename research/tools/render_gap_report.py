#!/usr/bin/env python3
"""Build a portable HTML gallery from immutable, fixed-time recorded grid exports."""

import argparse
import html
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from natural_motion import HALF
from replay_local_terrain import sha
from revision_evidence import validated_transform

COLORS = {-1: "#cbd5e1", 0: "#62af97", 20: "#c3cf76", 100: "#d96350"}
NAMES = {-1: "Unknown", 0: "Measured support", 20: "Inferred support", 100: "Occupied"}


def grid_svg(classes, origin, base, resolution, *, zoom=False):
    classes = np.asarray(classes)
    base = validated_transform(base)
    origin = np.asarray(origin)
    if classes.shape != (60, 60) or not np.isin(classes, list(COLORS)).all() or abs(resolution - 0.1) > 1e-9:
        raise ValueError("expected original 60x60 10cm classes")

    def xy(points):
        result = (np.asarray(points)[:, :2] - origin) / resolution
        result[:, 1] = 60 - result[:, 1]
        return result

    center = xy(base[None, :3, 3])[0]
    view = f"{center[0]-15:.5f} {center[1]-15:.5f} 30 30" if zoom else "-1 -1 62 62"
    pieces = [
        f'<svg viewBox="{view}" role="img" aria-label="Recorded classification grid, 10 centimetre cells">',
        '<rect x="-100" y="-100" width="300" height="300" fill="#f8fafc"/>',
    ]
    for y, row in enumerate(classes):
        start = 0
        while start < 60:
            end = start + 1
            while end < 60 and row[end] == row[start]:
                end += 1
            value = int(row[start])
            pieces.append(
                f'<rect x="{start}" y="{59-y}" width="{end-start}" height="1" fill="{COLORS[value]}"><title>{NAMES[value]}</title></rect>'
            )
            start = end
    lines = []
    for n in range(61):
        lines.append(f"M{n} 0V60 M0 {n}H60")
    pieces.append(
        f'<path d="{" ".join(lines)}" fill="none" stroke="#334155" stroke-opacity=".16" stroke-width=".035"/>'
    )
    pieces.append('<rect x="0" y="0" width="60" height="60" fill="none" stroke="#334155" stroke-width=".12"/>')
    for half, dash, width in [((0.5037, 0.28545), "", 0.18), (HALF, 'stroke-dasharray=".35 .25"', 0.12)]:
        corners = np.array(
            [[-half[0], -half[1], 0], [half[0], -half[1], 0], [half[0], half[1], 0], [-half[0], half[1], 0]]
        )
        transformed = corners @ base[:3, :3].T + base[:3, 3]
        points = " ".join(f"{a:.5f},{b:.5f}" for a, b in xy(transformed))
        pieces.append(f'<polygon points="{points}" fill="none" stroke="#111827" stroke-width="{width}" {dash}/>')
    arrow = xy(np.array([base[:3, 3], base[:3, 3] + base[:3, 0] * 0.85]))
    tip = arrow[1]
    back = tip - (tip - arrow[0]) * 0.25
    normal = np.array([-(tip - arrow[0])[1], (tip - arrow[0])[0]]) * 0.075
    pieces.append(
        f'<path d="M{arrow[0][0]:.5f} {arrow[0][1]:.5f} L{tip[0]:.5f} {tip[1]:.5f}" stroke="#111827" stroke-width=".18"/>'
    )
    pieces.append(
        '<polygon points="'
        + " ".join(f"{a:.5f},{b:.5f}" for a, b in (tip, back + normal, back - normal))
        + '" fill="#111827"/>'
    )
    pieces.append("</svg>")
    return "".join(pieces)


def gap_text(example):
    component = example["base_center_unknown_component"]
    if component is None:
        state = NAMES.get(example["base_center_class"], "Outside grid")
        return f"Base-center cell: {state.lower()}; this does not certify the full footprint."
    boundary = " Connected to the grid edge; not a bounded physical hole." if component["touches_grid_edge"] else ""
    return (
        f'Base-center unknown patch: {component["width_m"]:.1f} × {component["height_m"]:.1f} m bounding box, '
        f'{component["area_m2"]:.2f} m² ({component["cells"]} cells).' + boundary
    )


def pose_reference_notice(pose, report):
    """Do not attach another run's pose score or hide a failed reference audit."""
    key = report["recorder"] + "/collection.json"
    expected = report["input_sha256"].get(key)
    if not expected or pose.get("input_sha256", {}).get(key) != expected:
        raise ValueError("pose report does not match the grid recording")
    gate = pose.get("native_surface_gate")
    if gate is not None and gate.get("passed") is not True:
        return (
            '<p class="notice">Reference-surface audit: <b>FAILED</b>. '
            + html.escape(str(gate.get("failure", "unspecified")))
            + ". Capture completion and pose-only measurements do not clear this terrain-reference gate.</p>"
        )
    return ""


def render(exports, output, pose_reports=()):
    hashes, sections, summary, overview = {}, [], [], []
    poses = list(pose_reports) if pose_reports else [None] * len(exports)
    if len(poses) != len(exports):
        raise ValueError("one optional pose report per grid export")
    for source, pose_path in zip(exports, poses):
        path = source / "report.json"
        report = json.loads(path.read_text())
        if not report["diagnostic_completed"]:
            raise ValueError("incomplete grid export")
        hashes[str(path)] = sha(path)
        state = "Completed capture" if report["source_completed"] else "FAILED CAPTURE — prefix only"
        error_text = ""
        distance_text = "Not scored"
        detailed_failure = report["source_failure"] or report["mapper_failure"] or "None"
        reference_notice = ""
        if pose_path:
            pose = json.loads(pose_path.read_text())
            hashes[str(pose_path)] = sha(pose_path)
            reference_notice = pose_reference_notice(pose, report)
            if "first_published" in pose.get("results", {}):
                metrics = pose["results"]["first_published"]["overall"]["horizontal_m"]
            else:
                metrics = pose.get("results", {}).get("causal", {}).get("horizontal_m")
            if metrics:
                error_text = f'{100*metrics["absolute_p95"]:.2f} / {100*metrics["absolute_max"]:.2f} cm'
            distance = pose.get("path_length_m", pose.get("paired_path_length_m"))
            if distance is not None:
                distance_text = f"{distance:.3f} m" + (" (paired prefix)" if report["failed_prefix_only"] else "")
            if isinstance(pose.get("source_failure"), dict):
                detailed_failure = (
                    f'{detailed_failure}; simulator: {pose["source_failure"].get("error", "unspecified")}'
                )
        summary.append(
            f'<tr><td>{html.escape(report["pattern"])}</td><td>{state}</td><td>{report["projection_span_s"]:.1f} s</td>'
            f'<td>{distance_text}</td><td>{report["recorded_projections"]}/{report["expected_projections"]}</td><td>{error_text or "Not scored"}</td>'
            f'<td>{report["compute_ms"]["p95"]:.2f} ms</td></tr>'
        )
        cards = []
        for example in report["examples"]:
            data_path = source / example["file"]
            if sha(data_path) != example["sha256"]:
                raise ValueError("changed recorded grid")
            hashes[str(data_path)] = example["sha256"]
            with np.load(data_path, allow_pickle=False) as data:
                args = [data["classes"], data["origin"], data["local_from_base"], float(data["resolution"])]
                full, zoom = grid_svg(*args), grid_svg(*args, zoom=True)
            if source == exports[0] and (
                "+15s" in example["labels"] or "+30s" in example["labels"] or "last recorded grid" in example["labels"]
            ):
                overview.append((example, full))
            cards.append(
                f'<article><h3>{html.escape(" / ".join(example["labels"]))} · {example["elapsed_s"]:.1f} s</h3>'
                f'<div class="pair"><figure>{full}<figcaption>Full 6 × 6 m window</figcaption></figure>'
                f"<figure>{zoom}<figcaption>3 × 3 m close-up around estimated base</figcaption></figure></div>"
                f'<p>{gap_text(example)}</p><p class="muted">Whole grid: {example["unknown"]} unknown, '
                f'{example["occupied"]} occupied, {example["measured"]+example["inferred"]} supported / 3,600 cells. '
                f'Unknown in body-relative 3 m region: {example["near_3m_unknown"]}/{example["near_3m_cells"]}.</p></article>'
            )
        failure = html.escape(str(detailed_failure))
        sections.append(
            f'<section><h2>{html.escape(report["pattern"])} <span class="badge">{state}</span></h2>'
            f'{reference_notice}'
            f'<p>Failure: {failure}. Unreached fixed display times: {report["unreached_display_seconds"]} s. '
            f'Missing projected grids: {len(report["missing_projection_stamps"])}.</p>'
            f'<p class="muted">Times are relative to first terrain projection. The final example is the last '
            f"recorded projection before completion invalidation or failure—not the best-looking map. "
            f'Pre-projection messages: {report["preprojection_messages"]}; terminal unknown messages: '
            f'{report["completion_unknown_messages"]}. Recorder: {html.escape(report["recorder"])}</p>'
            f'{"".join(cards)}</section>'
        )
    legend = "".join(
        f'<span><i style="background:{color}"></i>{NAMES[value]}</span>' for value, color in COLORS.items()
    )
    document = """<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Terrain gaps — recorded 2D grids and longer-run test</title><style>
:root{font:16px/1.5 system-ui,sans-serif;color:#1e293b;background:#f1f5f9}body{max-width:1180px;margin:auto;padding:28px}
h1,h2,h3{line-height:1.25}h1{font-size:2.2rem}h2{margin-top:42px}article{background:white;border:1px solid #d7e0e9;border-radius:12px;padding:18px;margin:20px 0}
.pair{display:grid;grid-template-columns:1fr 1fr;gap:20px}figure{margin:0}svg{display:block;width:100%;background:#f8fafc}figcaption{text-align:center;font-size:.85rem;color:#64748b}
.muted{color:#64748b;font-size:.87rem;overflow-wrap:anywhere}.badge{font-size:.8rem;background:#e2e8f0;padding:5px 9px;border-radius:5px}
.legend{display:flex;gap:20px;flex-wrap:wrap}.legend i{display:inline-block;width:16px;height:16px;vertical-align:middle;margin-right:5px}
table{border-collapse:collapse;width:100%;font-size:.88rem}th,td{padding:10px;border-bottom:1px solid #cbd5e1;text-align:left}.scroll{overflow-x:auto}
.notice{background:#fff7ed;border-left:4px solid #ea580c;padding:16px}code{font-size:.85em}a{color:#0369a1}@media(max-width:720px){.pair{grid-template-columns:1fr}body{padding:14px}}
</style><h1>Terrain gaps in actual recorded grids</h1><p>Corrected single-LiDAR mounting · 10 cm cells · native returns + GLIM · advisory terrain only</p>
<div class="notice"><b>Grey means unknown—not an obstacle and not certified free.</b> These are classification grids, before cost inflation.
The solid outline is the calibrated chassis (about 1.01 × 0.57 m), not the full wheel footprint. The dashed 1.70 × 1.40 m rectangle is the acquisition guard envelope.
The arrow points forward. White outside the map is outside the rolling window.</div>
<p>Short tests previously showed startup/return gaps even with exact GT-pose substitution. This gallery includes fixed-time examples to distinguish startup blindness from gaps while moving.
Neither known-cell fraction nor GT-pose agreement establishes obstacle recall or safe wheel support. Failed prefixes are never presented as completed routes.</p>
<p class="muted">Experimental acquisition: physical IMU, one-time GT pose initialization, synthetic RTK-assisted GLIM, and a sole GT-controlled guarded driver. Terrain labels are evaluator-only. This is not autonomous navigation or unassisted SLAM qualification.</p>"""
    document += f'<div class="legend">{legend}</div>'
    if overview:
        width = 400 * len(overview)
        parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} 460" role="img" aria-label="Fixed-time grid sequence from first listed run">',
            f'<rect width="{width}" height="460" fill="white"/>',
        ]
        for i, (example, svg) in enumerate(overview):
            label = f'+{example["elapsed_s"]:.1f} s' + (" (last)" if "last recorded grid" in example["labels"] else "")
            parts.append(
                f'<text x="{400*i+200}" y="25" text-anchor="middle" font-family="sans-serif" font-size="19">{label}</text>'
            )
            parts.append(svg.replace("<svg ", f'<svg x="{400*i+5}" y="40" width="390" height="390" ', 1))
            parts.append(
                f'<text x="{400*i+200}" y="451" text-anchor="middle" font-family="sans-serif" font-size="14">6 × 6 m · 10 cm cells</text>'
            )
        parts.append("</svg>")
        overview_svg = "".join(parts)
        (output / "gap-overview.svg").write_text(overview_svg)
        document += (
            "<h2>At a glance — first listed run</h2><p>Fixed +15 s, +30 s, and last recorded map. Grey near the startup position can remain after an out-and-back even when the center becomes supported mid-route.</p>"
            + overview_svg
        )
    document += f'<h2>Test status</h2><div class="scroll"><table><thead><tr><th>Protocol</th><th>Source</th><th>Map span</th><th>Measured path</th><th>Grids recorded/expected</th><th>First-published XY p95/max</th><th>Compute p95</th></tr></thead><tbody>{"".join(summary)}</tbody></table></div>'
    document += '<p class="muted">Compute target remains &lt;50 ms p95. GUI baselines and the new headless run differ in rendering workload. Pose metrics for failed runs cover only paired recorded samples, not the missing route.</p>'
    document += "".join(sections)
    document += "<footer><p>Fixed display schedule: +0, +5, +15, +30, +60, +120, +180, +240 seconds and last projection. Unreached times are retained in the report. Base-center patch uses 8-connected unknown cells; its dimensions are grid-aligned bounds, not a physical hole measurement. This self-contained file requires no server or network.</p></footer></html>"
    if any(sha(path) != digest for path, digest in hashes.items()):
        raise ValueError("report input changed")
    with (output / "gap-report.html").open("x") as stream:
        stream.write(document)
    return dict(
        completed=True,
        acceptance=False,
        input_sha256=hashes,
        output_sha256={
            name: sha(output / name) for name in ("gap-report.html", "gap-overview.svg") if (output / name).exists()
        },
        source_sha256=sha(Path(__file__)),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("exports", nargs="+", type=Path)
    parser.add_argument("--pose-reports", nargs="*", type=Path)
    args = parser.parse_args()
    exports = [path.resolve(strict=True) for path in args.exports]
    if len({path.parent for path in exports}) != 1:
        raise ValueError("exports must share one external archive")
    output = allocate_run(exports[0].parent, limit=1024**3)
    print(f"Run: {output}", flush=True)
    report = render(exports, output, args.pose_reports)
    check_run(output)
    with (output / "report.json").open("x") as stream:
        json.dump(report, stream, indent=2)
    check_run(output)


if __name__ == "__main__":
    main()
