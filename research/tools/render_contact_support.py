#!/usr/bin/env python3
"""Render support experiments without relabeling their original terrain grids."""

import argparse
import html
import json
from pathlib import Path

from archive_guard import allocate_run, check_run
from replay_local_terrain import sha


def render(reports):
    rows = []
    for run, report in reports:
        uncertainty = report["contract"]["history_xy_erosion_m"] * 100
        geometry = report["contract"].get("positive_support_policy", "full_80x120mm")
        scenario = html.escape(report.get("display_scenario", run))
        for policy, periods in report["summary"].items():
            r = periods["all"]
            rows.append(
                f'<tr><td title="{html.escape(run, quote=True)}">{scenario}</td><td>{uncertainty:g} cm</td>'
                f"<td>{policy}<br>{geometry}</td><td>{r['stationary']}/{r['samples']}</td>"
                f"<td>{r['legacy_progress_or_turn']}/{r['samples']}</td>"
                f"<td>{r['expanded_any_action']}/{r['samples']}</td>"
                f"<td>{str(r['expanded_or_retrace']) + '/' + str(r['samples']) if 'expanded_or_retrace' in r else 'not tested'}</td></tr>"
            )
    payload = json.dumps([dict(run=run, **report) for run, report in reports]).replace("<", "\\u003c")
    return (
        """<!doctype html><meta charset="utf-8"><title>Wheel support: what still blocks movement</title>
<style>
body{font:16px/1.55 system-ui;background:#f6f8fb;color:#17283c;max-width:1200px;margin:32px auto;padding:0 24px}
h1{font-size:30px}h2{font-size:21px}p{max-width:920px}table{border-collapse:collapse;width:100%;font-size:13px}
td,th{padding:8px;text-align:left;border-bottom:1px solid #dbe2eb}td:first-child{font-family:monospace;font-size:11px}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:16px}
article{background:white;padding:16px;border:1px solid #dbe2eb;border-radius:12px}svg{width:100%;background:#eceef2}
.key span{display:inline-block;margin-right:20px}.swatch{display:inline-block;width:14px;height:14px;vertical-align:middle;margin-right:5px!important}
select{padding:8px;max-width:100%}.warning{background:#fff3d5;padding:12px 18px;border-left:4px solid #d88b13}
</style><h1>Wheel support: what still blocks movement</h1>
<p>The original LiDAR map stays unchanged. This experiment adds separate support from a grounded startup footprint,
current wheel contacts and past wheel tracks. The red overlays show the part of a <strong>5 cm forward step</strong>
that still lacks geometric support. These are selected fixed-time examples; the table includes every 1 Hz decision.</p>
<p class="warning">Offline diagnosis, not permission to drive. Recorded motion was driven by the guarded test controller,
not this support policy. Contact sizes and grounded operation are assumptions; historical erosion is a sensitivity setting,
not a calibrated error bound. Any-action availability does not prove progress toward a goal or obstacle avoidance.</p>
<h2>All sampled decisions, including startup</h2><table><thead><tr><th>Experiment</th><th>History erosion</th>
<th>Support policy / region</th><th>Standing supported</th><th>Original goal move / turn</th><th>Any expanded motion</th><th>Or retrace path</th>
</tr></thead><tbody>"""
        + "".join(rows)
        + """</tbody></table>
<h2>Where is the missing support?</h2><select id="run"></select>
<p class="key"><span><i class="swatch" style="background:#d4e7df"></i>LiDAR-supported cell</span>
<span><i class="swatch" style="background:#eceef2"></i>Unknown cell</span>
<span><i class="swatch" style="background:#783533"></i>Occupied cell</span>
<span><i class="swatch" style="background:#71a4ed"></i>Startup + current + remembered support</span>
<span><i class="swatch" style="background:#e84930"></i>Uncovered part of requested wheel sweep</span></p>
<p>Each grid square is 10 cm; each panel spans 1.8 m. Black outlines are the requested positive-support regions
(full patch or central core, as selected). Full wheel/body hazard checks remain in both policies.
Thin gaps have a visible outline, so their drawn width is exaggerated; the reported area is not.
Under-body support is only the one-time startup assumption, not a footprint that follows the robot.</p>
<div class="cards" id="cards"></div><script>
const reports="""
        + payload
        + """;
const select=document.getElementById('run');
for(let i=0;i<reports.length;i++){const o=document.createElement('option');o.value=i;
o.textContent=(reports[i].display_scenario||reports[i].run)+' — '+(reports[i].contract.positive_support_policy||'full_80x120mm')+' — '+(100*reports[i].contract.history_xy_erosion_m)+' cm';select.append(o);}
function paths(polygons,color,opacity,stroke){return polygons.map(p=>{
 const d=[p.outer,...p.holes].map(r=>'M'+r.map(v=>v.join(',')).join('L')+'Z').join('');
 return '<path d="'+d+'" fill="'+color+'" fill-opacity="'+opacity+'" fill-rule="evenodd" stroke="'+stroke+'" stroke-width="0.0025"/>';
}).join('');}
function draw(){const r=reports[+select.value],root=document.getElementById('cards');root.replaceChildren();
 for(const e of r.examples||[]){const card=document.createElement('article'),b=e.local_from_base;
 const cx=b[0][3],cy=b[1][3],res=e.resolution;let content='';
 for(let y=0;y<e.classes.length;y++)for(let x=0;x<e.classes[y].length;x++){
  const v=e.classes[y][x],color=v===100?'#783533':v===-1?'#eceef2':'#d4e7df';
  content+='<rect x="'+(e.origin[0]+x*res)+'" y="'+(e.origin[1]+y*res)+'" width="'+res+'" height="'+res+'" fill="'+color+'" stroke="#d5dbe2" stroke-width="0.0015"/>';
 }
 content+=paths(e.support,'#71a4ed',.45,'none')+paths(e.wheels,'none',0,'#101820')+paths(e.gap,'#e84930',.9,'#e84930');
 content+='<circle cx="'+cx+'" cy="'+cy+'" r=".018" fill="#101820"/><path d="M'+cx+','+cy+'L'+(cx+.2*b[0][0])+','+(cy+.2*b[1][0])+'" stroke="#101820" stroke-width=".009"/>';
 card.innerHTML='<h3>+'+e.elapsed_s+' s · forward 5 cm</h3><svg viewBox="'+(cx-.9)+' '+(-cy-.9)+' 1.8 1.8"><g transform="scale(1,-1)">'+content+'</g></svg><p>Uncovered area: <strong>'+(e.gap_area_m2*10000).toFixed(2)+' cm²</strong></p>';
 root.append(card);
 }
 if(!root.children.length)root.textContent='This earlier experiment did not save geometric examples.';
}
select.onchange=draw;if(reports.length>1)select.value=1;draw();</script>"""
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--archive-root", required=True, type=Path)
    args = parser.parse_args()
    reports, hashes = [], {}
    for run in args.runs:
        check_run(run)
        path = run / "report.json"
        report = json.loads(path.read_text())
        if not report["completed"] or sha(run / "scores.jsonl") != report["output_sha256"]["scores.jsonl"]:
            raise ValueError("complete unchanged support scores required")
        hashes[str(path)] = sha(path)
        baseline_name = next(p for p in report["input_sha256"] if Path(p).name == "report.json")
        baseline = args.archive_root / Path(baseline_name).parent.name / "report.json"
        if sha(baseline) != report["input_sha256"][baseline_name]:
            raise ValueError("changed original assessment")
        hashes[str(baseline)] = sha(baseline)
        original = json.loads(baseline.read_text())
        names = dict(turn15="Turn-only", straight_slow="Straight out/back", combined10="Turn + out/back")
        pattern = original["contract"]["route_pattern"]
        report["display_scenario"] = names.get(pattern, pattern)
        reports.append((run.name, report))
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    (output / "support-gaps.html").write_text(render(reports))
    (output / "manifest.json").write_text(
        json.dumps(
            dict(
                input_sha256=hashes, renderer_sha256=sha(Path(__file__)), html_sha256=sha(output / "support-gaps.html")
            ),
            indent=2,
        )
    )
    print(output / "support-gaps.html")


if __name__ == "__main__":
    main()
