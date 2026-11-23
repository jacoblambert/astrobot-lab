#!/usr/bin/env python3
"""Self-contained grid/path report from completed immutable evaluations."""

import argparse
import base64
import json
from pathlib import Path

import numpy as np

from archive_guard import allocate_run, check_run
from replay_local_terrain import sha


PAGE = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Terrain costs → candidate paths</title><style>
body{font:16px/1.55 system-ui,sans-serif;color:#e5edf4;background:#101820;max-width:1120px;margin:40px auto;padding:0 20px}
h1{line-height:1.2}h2{font-size:21px}section{background:#192631;padding:22px;border-radius:12px;margin:20px 0}
select{background:#243747;color:inherit;border:1px solid #6b7e8d;padding:8px;max-width:100%;margin:4px}
canvas{width:min(100%,520px);height:auto;border:1px solid #708090;image-rendering:pixelated}
.row{display:flex;flex-wrap:wrap;gap:24px}.text{flex:1;min-width:240px}code{overflow-wrap:anywhere}
table{border-collapse:collapse;width:100%;font-size:14px}td,th{text-align:left;padding:8px;border-bottom:1px solid #40505c}
.muted{color:#abc0cf}.warn{color:#ffce8c}p{max-width:900px}pre{white-space:pre-wrap;overflow-wrap:anywhere;font:13px/1.5 monospace}
</style><h1>Terrain costs → candidate paths</h1>
<p>This report checks whether the costmap produces routes and where those routes
intersect independent hazard geometry. No paths here commanded the robot.</p>
<section><h2>What changed</h2><p>Known obstacles stay blocked, with a cost gradient
around them. A single unknown 10 cm cell surrounded by measured, compatible ground
can receive cost 70 instead of blocking the route. The original terrain labels
are untouched. Costs are penalties, not probabilities.</p>
<p class="warn">When included, the additional optimistic control assigns cost 90 to missing
ground even in connected gaps. It is an offline diagnostic, not the live policy.
Explicit terrain conflicts still block.</p>
<p>A green outline, when present, shows the turning-area assumption at the
actual current pose. It supplies ground support without changing raw labels;
known hazards still override it. It is never moved to hypothetical future poses.</p>
<p>In two-horizon results, <code>tentative_route</code> may cross unknown ground.
Only <code>executable_prefix</code> passes the separate local support checks;
its short endpoint is not arrival at the distant goal. A faint line shows the
remaining tentative route, and the gold cross marks the fixed goal.</p>
<p>The planner checks the swept robot body and four wheel patches, not just the
centerline. It uses eight headings and a bounded search, so a failure does not
prove that no other motion exists. Geometry checks assume perfect relative
tracking; actual driving, braking and changing localization remain untested.</p></section>
<section><h2>All-run results</h2><div id="totals"></div></section>
<section><h2>Inspect the grids</h2>
<label>Recording or fixture <select id="dataset"></select></label><br>
<label>Case / time bin <select id="sample"></select></label>
<label>Policy <select id="policy"></select></label>
<label>Goal <select id="goal"></select></label>
<div class="row"><canvas id="grid" width="600" height="600"></canvas>
<div class="text"><p>Each square is 10 × 10 cm; the view is 6 × 6 m.</p>
<p>Grey: unknown / blocking ground support<br>Green → yellow: increasing cost<br>
Red: known occupied<br>Cyan: candidate path and starting body outline<br>
Magenta: independent hazard geometry, when provided by the fixture</p>
<div id="outcome"></div><p class="muted">The outline is the existing body guard,
not the required ground-support area. Ground support is checked under the wheel
patches; startup support is kept separately and is not painted free here.</p></div></div></section>
<section><h2>How to interpret this</h2><p>A returned path is a planning result,
not navigation qualification. A rock-bound intersection is a conservative
possible collision, not exact mesh contact. Missing same-time GT or floor
outside the survey stays unverified. Zero returned paths cannot count as
zero-collision success. Synthetic hidden-rock/hole tests, when included, are
stress cases rather than examples of ordinary route clearance.</p>
<p>Source IDs and hashes are in the accompanying manifest. Every time bin and
goal is available above; examples have not been selected for success.</p></section>
<script>
const data=DATA;
const ds=document.getElementById('dataset'),sm=document.getElementById('sample'),
pl=document.getElementById('policy'),gl=document.getElementById('goal');
function options(el,values){el.replaceChildren(...values.map(v=>new Option(v,v)))}
function chosen(){return data.find(x=>x.name===ds.value)}
function sample(){return chosen().samples[Number(sm.value)]}
function safe(el,text){el.textContent=text}
options(ds,data.map(d=>d.name));
function table(parent,headers,rows){const t=document.createElement('table');
for(const [i,row] of [headers,...rows].entries()){const tr=document.createElement('tr');
for(const value of row){let td=document.createElement(i?'td':'th');safe(td,String(value));tr.append(td)}t.append(tr)}parent.append(t)}
for(const d of data){const parent=document.getElementById('totals'),title=document.createElement('h3');safe(title,d.name);parent.append(title);
let s=d.summary.results;
if(s.contract?.two_horizons){let note=document.createElement('p');safe(note,
'Fixed goals selected after '+s.contract.goal_selection_delay_s+' s of observation. '+s.goal_missing_at_selection+
' of eight target directions/distances had no observed-free goal. Waiting, missing goals and failed searches remain in the totals. '+
'Executable-prefix availability is not arrival at the goal.');parent.append(note);}
if(s.policies)table(parent,['Policy','Available / queries','GT checked','Possible rock overlaps','Outside floor survey','Missing GT'],
Object.entries(s.policies).map(([name,p])=>{let g=p.gt_start_relative_tracking;return [name,p.found+' / '+p.goals,
g.evaluated,g.evaluated?g.possible_rock_intersections:'—',g.evaluated?g.outside_survey:'—',p.found-g.evaluated]}));
else table(parent,['Case','Policy','Route found','Rock / hole overlap'],Object.entries(s).flatMap(([name,ps])=>
Object.entries(ps).map(([policy,p])=>[name.replaceAll('_',' '),policy,p.found?'yes':'no',p.found?(p.rock_hits.length||p.hole_hits.length?'YES':'none'):'not evaluated'])));
if(s.policies)for(const [name,p] of Object.entries(s.policies)){
let heading=document.createElement('p');safe(heading,name+' — distance matters:');parent.append(heading);
table(parent,['Goal distance (m)','Routes / four directions × time bins','Bins with at least one route'],
Object.entries(p.by_distance).map(([distance,count])=>[distance,count+' / '+(4*d.samples.length),
p.bins_with_a_path? p.bins_with_a_path[distance]+' / '+d.samples.length:'not recorded']));}
let note=document.createElement('p');note.className='muted';safe(note,'Source: '+d.summary.source);parent.append(note)}
function load(){options(sm,chosen().samples.map((_,i)=>String(i)));loadSample()}
function loadSample(){options(pl,Object.keys(sample().plans));loadPolicy()}
function loadPolicy(){options(gl,Object.keys(sample().plans[pl.value]));draw()}
function draw(){let s=sample(),p=s.plans[pl.value][gl.value],c=document.getElementById('grid').getContext('2d');
let bytes=Uint8Array.from(atob(s.grids[pl.value]),v=>v.charCodeAt(0)),n=60,scale=100;
for(let y=0;y<n;y++)for(let x=0;x<n;x++){let v=bytes[y*n+x];c.fillStyle=v===255?'#58646e':v===100?'#e84e4e':
`rgb(${Math.round(45+v*2)},${Math.round(165-v*.3)},${Math.round(100-v*.75)})`;c.fillRect(x*10,(n-1-y)*10,10,10)}
function xy(p){return [(p[0]-s.origin[0])*scale,600-(p[1]-s.origin[1])*scale]}
function line(points,color,width=2){if(!points.length)return;c.beginPath();c.strokeStyle=color;c.lineWidth=width;
points.forEach((p,i)=>{let [x,y]=xy(p);i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke()}
for(const r of s.hazards||[])line([[r[0],r[1]],[r[2],r[1]],[r[2],r[3]],[r[0],r[3]],[r[0],r[1]]],'#ff71e8',3);
for(const polygon of s.current_turn_polygons||[])line(polygon,'#7bff89',3);
line((p.tentative_poses||[]).map(t=>[t[0][3],t[1][3]]),'#688d99',1);
line((p.poses||[]).map(t=>[t[0][3],t[1][3]]),'#32e1ff',3);
if(p.goal_local_xy){let [x,y]=p.goal_local_xy;line([[x-.06,y-.06],[x+.06,y+.06]],'#ffd478',3);line([[x-.06,y+.06],[x+.06,y-.06]],'#ffd478',3);}
let t=s.start,corners=[[-.85,-.70],[.85,-.70],[.85,.70],[-.85,.70],[-.85,-.70]];
line(corners.map(([x,y])=>[t[0][3]+t[0][0]*x+t[0][1]*y,t[1][3]+t[1][0]*x+t[1][1]*y]),'#32e1ff');
let out=document.getElementById('outcome');out.replaceChildren();let status=document.createElement('p');
let success=pl.value==='executable_prefix'?'Supported local segment — not goal arrival.':
pl.value==='tentative_route'?'Tentative route — not permission to execute.':'Route found.';
safe(status,(p.found?success:'No route returned: '+p.reason.replaceAll('_',' ')+'.')+' Search expanded '+p.expanded+' states.');out.append(status);
if(p.truth&&p.truth.evaluated){table(out,['Geometry check','Rock overlaps','Surveyed floor'],Object.entries(p.truth.references).map(([k,r])=>
[k==='map_frame'?'Estimated start':'Actual GT start',r.possible_rock_guard_intersections.length,r.surveyed_floor_only?'yes':'NO']));
if(p.truth.missing_current_gt){let x=document.createElement('p');safe(x,'Actual starting pose is unavailable at this timestamp; GT tracking is unverified.');out.append(x)}}
else if(p.found){let x=document.createElement('p');safe(x,'Independent rock overlaps: '+p.rock_guard_intersections.length+
'; wheel overlaps with holes: '+p.hole_wheel_intersections.length+'.');out.append(x)}
let stamp=document.createElement('p');stamp.className='muted';safe(stamp,s.label);out.append(stamp);}
ds.onchange=load;sm.onchange=loadSample;pl.onchange=loadPolicy;gl.onchange=draw;load();
</script></html>"""


def packed(grid):
    if grid.shape != (60, 60):
        raise ValueError("report expects declared 60x60 grids")
    return base64.b64encode(grid.astype(np.int8).tobytes()).decode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=Path)
    parser.add_argument("--replay", type=Path, action="append", default=[])
    parser.add_argument("--archive-root", type=Path, required=True)
    args = parser.parse_args()
    if args.cases is None and not args.replay:
        parser.error("at least one cases or replay evaluation is required")
    hashes, datasets = {}, []
    for source in ([args.cases] if args.cases is not None else []) + args.replay:
        check_run(source)
        report = json.loads((source / "report.json").read_text())
        if not report["completed"] or (source / "failure.json").exists():
            raise ValueError("completed evaluation required")
        expected = report.get("grid_sha256", report.get("output_sha256", {}).get("grids.npz"))
        if sha(source / "grids.npz") != expected:
            raise ValueError("changed evaluated grids")
        for name in ("report.json", "grids.npz"):
            hashes[str(source / name)] = sha(source / name)
        with np.load(source / "grids.npz", allow_pickle=False) as grids:
            samples = []
            if source == args.cases:
                summary = {}
                for case in report["cases"]:
                    plans = {p: {"route": v} for p, v in case["policies"].items()}
                    samples.append(
                        dict(
                            label=case["name"],
                            plans=plans,
                            origin=report["record"]["origin"],
                            start=[[1, 0, 0, -1.4], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
                            grids={p: packed(grids[case["name"] + "_" + p]) for p in plans},
                            hazards=case["rocks"] + case["holes"],
                        )
                    )
                    summary[case["name"]] = {
                        p: dict(
                            found=v["found"],
                            rock_hits=v["rock_guard_intersections"],
                            hole_hits=v["hole_wheel_intersections"],
                        )
                        for p, v in case["policies"].items()
                    }
            else:
                hashes[str(source / "paths.jsonl")] = sha(source / "paths.jsonl")
                if hashes[str(source / "paths.jsonl")] != report["output_sha256"]["paths.jsonl"]:
                    raise ValueError("changed evaluated paths")
                rows = [json.loads(s) for s in (source / "paths.jsonl").read_text().splitlines()]
                first = rows[0]["query_ns"]
                for row in rows:
                    bin_id = (row["query_ns"] - first) // 5_000_000_000
                    strict_key = f"{bin_id}_strict_cost"
                    if strict_key not in grids:
                        if report["gap_cells_min_median_max"][2] != 0:
                            raise ValueError("strict grid must be archived separately when promotions exist")
                        strict_key = f"{bin_id}_cost"
                    samples.append(
                        dict(
                            label=f"bin {bin_id}; query {row['query_ns']} ns",
                            plans=row["plans"],
                            origin=row["origin"],
                            start=row["local_from_base"],
                            current_turn_polygons=row.get("current_turn_polygons", []),
                            grids={
                                p: packed(
                                    grids[
                                        row.get("grid_keys", {}).get(
                                            p,
                                            (
                                                f"{bin_id}_optimistic_cost"
                                                if p == "optimistic_control"
                                                else strict_key if p == "strict" else f"{bin_id}_cost"
                                            ),
                                        )
                                    ]
                                )
                                for p in row["plans"]
                            },
                        )
                    )
                summary = dict(
                    maps=report["maps"],
                    policies=report["policies"],
                    contract=report.get("contract", {}),
                    goal_missing_at_selection=report.get("goal_missing_at_selection"),
                )
            label = (
                "Controlled cases" if source == args.cases else f"Recorded run: {report['maps']} maps · {source.name}"
            )
            if report.get("contract", {}).get("two_horizons"):
                label = f"{report['contract'].get('goal_selection_delay_s', 0):g} s observation before goals · " + label
            datasets.append(dict(name=label, summary=dict(source=source.name, results=summary), samples=samples))
    output = allocate_run(args.archive_root, limit=16 * 1024**2)
    content = json.dumps(datasets, separators=(",", ":")).replace("<", "\\u003c")
    (output / "report.html").write_text(PAGE.replace("const data=DATA;", "const data=" + content + ";"))
    (output / "manifest.json").write_text(
        json.dumps(
            dict(input_sha256=hashes, source_sha256=sha(Path(__file__)), report_sha256=sha(output / "report.html")),
            indent=2,
        )
    )
    check_run(output)
    print(output / "report.html")


if __name__ == "__main__":
    main()
