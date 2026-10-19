#!/usr/bin/env python3
"""Replay a recorded route and count ACTUAL LiDAR returns on the rock the rover wedged on, per approach
distance. Answers: is the rock under-observed (sensor) or well-observed but dropped by the mapper (mapping)?
Usage: analyze_wedge_hits.py <bag_dir>"""
import sys, math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sensor_msgs_py.point_cloud2 as pc2

bag = sys.argv[1]
reader = SequentialReader()
reader.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
msgcls = {name: get_message(t) for name, t in types.items()}

CLOUD='/astrobot_0/slam/aligned_points_corrected'; ODOM='/astrobot_0/slam/odom_corrected'; CMD='/astrobot_0/cmd_vel'
EVENTS='/astrobot_0/exploration/events'
clouds=[]; odoms=[]; cmds=[]; events=[]
import json, re
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == CLOUD:
        m = deserialize_message(data, msgcls[topic])
        pts = np.array([(p[0],p[1],p[2]) for p in pc2.read_points(m,('x','y','z'),skip_nans=True)], dtype=np.float32)
        clouds.append((t*1e-9, pts))
    elif topic == ODOM:
        m = deserialize_message(data, msgcls[topic]); p=m.pose.pose.position
        odoms.append((t*1e-9, p.x, p.y, abs(m.twist.twist.linear.x)))
    elif topic == CMD:
        m = deserialize_message(data, msgcls[topic]); cmds.append((t*1e-9, abs(m.linear.x)))
    elif topic == EVENTS:
        m = deserialize_message(data, msgcls[topic])
        try: events.append((t*1e-9, json.loads(m.data)))
        except Exception: events.append((t*1e-9, {'raw': getattr(m,'data','')}))
print(f"bag: {len(clouds)} clouds, {len(odoms)} odoms, {len(cmds)} cmds, {len(events)} events")
if not clouds or not odoms: sys.exit("missing data")
# show event types present
from collections import Counter
print("event types:", Counter(e.get('event', e.get('type','?')) for _,e in events).most_common(12))

def odom_at(t):
    best=min(odoms, key=lambda o: abs(o[0]-t)); return best
def cmd_at(t):
    if not cmds: return 0.0
    return min(cmds, key=lambda c: abs(c[0]-t))[1]

# 1) find wedge from exploration events: a goal_stalled with reason motion_stall (the real wedge).
def is_stall(e):
    s = json.dumps(e).lower()
    return 'motion_stall' in s
stalls = [(t,e) for (t,e) in events if is_stall(e)]
print(f"motion_stall events: {len(stalls)}")
if stalls:
    wedge_t = stalls[-1][0]                       # analyze the most recent wedge
    ro = odom_at(wedge_t); wx, wy = ro[1], ro[2]
    print(f"  using last stall event @ t={wedge_t:.1f}; event={json.dumps(stalls[-1][1])[:200]}")
else:
    # fallback: cmd>0.08 & odom~0 sustained, but only AFTER exploration is underway (>90s in)
    t0 = clouds[0][0]; wedge_t=None
    for (t,x,y,vlin) in odoms:
        if t - t0 < 90: continue
        if cmd_at(t) > 0.08 and vlin < 0.03:
            window=[o for o in odoms if t <= o[0] <= t+3.0]
            if window and all(w[3] < 0.05 for w in window) and (window[-1][0]-window[0][0]) >= 2.5:
                wedge_t=t; break
    if wedge_t is None: wedge_t=odoms[-1][0]
    ro=odom_at(wedge_t); wx,wy=ro[1],ro[2]
print(f"WEDGE at t={wedge_t:.1f} rover map=({wx:.2f},{wy:.2f})")

# 2) rock location: accumulate cloud points within 1.5m of the wedge rover pos, z above local ground,
#    over the last ~8s before wedge; rock = densest cluster centroid just ahead.
acc=[]
for (t,pts) in clouds:
    if wedge_t-10 <= t <= wedge_t+2 and len(pts):
        d=np.hypot(pts[:,0]-wx, pts[:,1]-wy)
        near=pts[d<1.5]
        if len(near): acc.append(near)
if acc:
    A=np.vstack(acc); ground=np.percentile(A[:,2], 10)
    rock=A[A[:,2] > ground+0.12]
    if len(rock):
        rcx,rcy,rcz=rock[:,0].mean(), rock[:,1].mean(), rock[:,2].max()
        print(f"ROCK center=({rcx:.2f},{rcy:.2f}) top_z={rcz:.2f} (ground~{ground:.2f}, height~{rcz-ground:.2f}m), "
              f"{len(rock)} rock-pts accumulated near wedge")
    else:
        print(f"NO points >0.12m above ground within 1.5m of wedge -> rock is in blind cone at wedge"); rcx,rcy=wx,wy; rcz=ground+0.3; ground=ground
else:
    print("no cloud near wedge"); sys.exit()

# 3) per-frame hits on the rock (xy within 0.45m of rock center, z>ground+0.10), vs rover->rock distance
print(f"\nhits on rock vs approach distance (rock box r=0.45m, z>{ground+0.10:.2f}):")
print(f"{'dist_bin':>10} {'frames':>7} {'median_hits':>12} {'max_hits':>9} {'min_z':>7} {'max_z':>7}")
bins={(0,1):[], (1,2):[], (2,3):[], (3,4):[], (4,5):[], (5,7):[]}
for (t,pts) in clouds:
    if not len(pts): continue
    ro=odom_at(t); rd=math.hypot(ro[1]-rcx, ro[2-1+1]-rcy) if False else math.hypot(ro[1]-rcx, ro[2]-rcy)
    dxy=np.hypot(pts[:,0]-rcx, pts[:,1]-rcy)
    onrock=pts[(dxy<0.45) & (pts[:,2] > ground+0.10)]
    for (lo,hi) in bins:
        if lo <= rd < hi:
            bins[(lo,hi)].append((len(onrock), onrock[:,2] if len(onrock) else np.array([]))); break
for (lo,hi),vals in bins.items():
    if not vals: print(f"{lo}-{hi}m{'':>5} {0:>7}"); continue
    hits=[v[0] for v in vals]; allz=np.concatenate([v[1] for v in vals if len(v[1])]) if any(len(v[1]) for v in vals) else np.array([0])
    print(f"{lo}-{hi}m{'':>5} {len(vals):>7} {int(np.median(hits)):>12} {max(hits):>9} {allz.min():>7.2f} {allz.max():>7.2f}")

# 4) GEOMETRIC classification of the rock cell (replicate mapper height logic on the points): isolates
#    whether the height-above-ground gate would PASS (geometry fine -> failure is occupancy/clearing/seed)
#    or FAIL (ground reference rides up). Accumulate ALL near-rock points over a window, grid 0.1m.
CELL=0.10; RES=0.10
win=[p for (t,p) in clouds if wedge_t-12<=t<=wedge_t+2 and len(p)]
A=np.vstack(win)
near=A[(np.abs(A[:,0]-rcx)<1.5)&(np.abs(A[:,1]-rcy)<1.5)]
cell_pts={}
for (x,y,z) in near:
    cell_pts.setdefault((round(x/CELL),round(y/CELL)),[]).append(z)
def pctl(a,q): a=sorted(a); return a[min(len(a)-1,int(q*len(a)))]
lowz={k:pctl(v,0.10) for k,v in cell_pts.items() if len(v)>=3}
highz={k:pctl(v,0.90) for k,v in cell_pts.items() if len(v)>=3}
import numpy as _np
def classify(rk):
    if rk not in lowz: return None
    dxs=[];dys=[];zs=[]
    for ox in range(-2,3):
        for oy in range(-2,3):
            k=(rk[0]+ox,rk[1]+oy)
            if k in lowz: dxs.append(ox*RES);dys.append(oy*RES);zs.append(lowz[k])
    if len(zs)<3: return None
    M=_np.c_[dxs,dys,_np.ones(len(zs))]; zarr=_np.array(zs)
    coef,_,_,_=_np.linalg.lstsq(M,zarr,rcond=None)
    res=zarr-(M@coef); keep=res<=0.05
    if 3<=keep.sum()<len(zs):
        coef,_,_,_=_np.linalg.lstsq(M[keep],zarr[keep],rcond=None)
    a,b,c=coef; slope=math.hypot(a,b); ss=slope*RES
    h_above=(highz[rk]-c)-0.5*ss; h_intr=(highz[rk]-lowz[rk])-ss
    return max(0.0,h_above,h_intr), c, slope, len(cell_pts[rk])
# scan all cells within 0.5m of rock center
print(f"\nGEOMETRIC classify rock FOOTPRINT (cells within 0.5m of center; gate 0.11, lethal 0.35):")
foot=[(round(x/CELL),round(y/CELL)) for x in _np.arange(rcx-0.5,rcx+0.5,CELL) for y in _np.arange(rcy-0.5,rcy+0.5,CELL)]
foot=sorted(set(foot))
npass=nleth=ntot=nskip=0; maxh=0; best=None
for k in foot:
    if k not in cell_pts: continue
    ntot+=1
    if len(cell_pts[k])<3: nskip+=1; continue
    r=classify(k)
    if r is None: continue
    h,gnd,sl,npts=r
    if h>maxh: maxh=h; best=(k,h,gnd,sl,npts)
    if h>=0.35: nleth+=1
    elif h>=0.11: npass+=1
print(f"  footprint cells with points: {ntot}; cells skipped (<3 pts): {nskip}")
print(f"  cells classified OBSTACLE (>=0.11): {npass}; LETHAL (>=0.35): {nleth}; max height seen: {maxh:.2f}m")
if best:
    k,h,gnd,sl,npts=best
    print(f"  tallest cell {k}: height={h:.2f} (fitted_ground={gnd:.2f} slope={sl:.2f}, {npts} pts)")
print(f"  ==> geometry {'MARKS the rock (so failure is occupancy/clearing/seed)' if (npass+nleth)>0 else 'MISSES the rock -> ground reference rides up / height collapses'}")

# 5) FIX SWEEP (offline, on this rock): try ground-estimation variants, count footprint cells that pass
#    the 0.11 gate. plane-fit radius R (current) vs robust LOW-FLOOR (10th pct of low_z over radius R,
#    slope-detrended). Decides the fix without a sim run.
def ground_planefit(rk, R):
    dxs=[];dys=[];zs=[]
    for ox in range(-R,R+1):
        for oy in range(-R,R+1):
            k=(rk[0]+ox,rk[1]+oy)
            if k in lowz: dxs.append(ox*RES);dys.append(oy*RES);zs.append(lowz[k])
    if len(zs)<3: return None
    M=_np.c_[dxs,dys,_np.ones(len(zs))]; zarr=_np.array(zs)
    coef,_,_,_=_np.linalg.lstsq(M,zarr,rcond=None); res=zarr-(M@coef); keep=res<=0.05
    if 3<=keep.sum()<len(zs): coef,_,_,_=_np.linalg.lstsq(M[keep],zarr[keep],rcond=None)
    a,b,c=coef; return c, math.hypot(a,b)
def ground_lowfloor(rk, R):
    vals=[lowz[(rk[0]+ox,rk[1]+oy)] for ox in range(-R,R+1) for oy in range(-R,R+1) if (rk[0]+ox,rk[1]+oy) in lowz]
    if len(vals)<3: return None
    # slope from a plane fit for detrend; ground = 10th percentile (robust floor under rocks)
    pf=ground_planefit(rk,R); slope=pf[1] if pf else 0.0
    return pctl(vals,0.10), slope
def ground_rgpf(rk, R, seed_frac=0.30, band=0.05, iters=3):
    # matches the C++ R-GPF: seed from lowest seed_frac of window cells, iterate one-sided inlier band.
    dxs=[];dys=[];zs=[]
    for ox in range(-R,R+1):
        for oy in range(-R,R+1):
            k=(rk[0]+ox,rk[1]+oy)
            if k in lowz: dxs.append(ox*RES);dys.append(oy*RES);zs.append(lowz[k])
    n=len(zs)
    if n<3: return None
    dxs=_np.array(dxs);dys=_np.array(dys);zs=_np.array(zs)
    order=_np.argsort(zs); k=max(3,int(round(seed_frac*n)))
    use=_np.zeros(n,bool); use[order[:k]]=True
    a=b=c=0.0; have=False
    def fit(mask):
        if mask.sum()<3: return None
        M=_np.c_[dxs[mask],dys[mask],_np.ones(mask.sum())]
        coef,_,_,_=_np.linalg.lstsq(M,zs[mask],rcond=None); return coef
    for _ in range(iters):
        coef=fit(use)
        if coef is None: break
        a,b,c=coef; have=True
        resid=zs-(a*dxs+b*dys+c); newuse=resid<=band
        if newuse.sum()<3: break
        use=newuse
    if not have: return None
    return c, math.hypot(a,b)
def step_pass(rk, R):
    # step-height: low_z[rk] - min neighbor low_z within R, detrended by local plane slope over R.
    vals=[(ox,oy,lowz[(rk[0]+ox,rk[1]+oy)]) for ox in range(-R,R+1) for oy in range(-R,R+1) if (rk[0]+ox,rk[1]+oy) in lowz and (ox or oy)]
    if not vals or rk not in lowz: return False
    minf=min(v[2] for v in vals)
    pf=ground_planefit(rk,R); slope=pf[1] if pf else 0.0
    step=(lowz[rk]-minf)-slope*(R*RES)
    return step>=0.11
print(f"\nFIX SWEEP on this rock (footprint cells passing 0.11 gate / {ntot} cells with points):")
print(f"  {'R':>3} {'window':>7} {'planefit':>10} {'lowfloor':>10} {'R-GPF':>10} {'step':>10}")
for R in (2,4,6,8):
    pf_pass=lf_pass=rg_pass=st_pass=0
    for k in foot:
        if k not in highz: continue
        for meth in ('pf','lf','rg'):
            g = ground_planefit(k,R) if meth=='pf' else (ground_lowfloor(k,R) if meth=='lf' else ground_rgpf(k,R))
            if g is None: continue
            gnd,sl=g; ss=sl*RES
            h=max(0.0,(highz[k]-gnd)-0.5*ss,(highz[k]-lowz[k])-ss)
            if h>=0.11:
                if meth=='pf': pf_pass+=1
                elif meth=='lf': lf_pass+=1
                else: rg_pass+=1
        if step_pass(k,R): st_pass+=1
    print(f"  {R:>3} {(2*R+1)*RES:>6.1f}m {pf_pass:>10} {lf_pass:>10} {rg_pass:>10} {st_pass:>10}")

# 6) RADIAL GROUND-ELEVATION PROFILE around the rock: is the rock on a real terrain RISE (apron extends
#    far + gradual) or is the "apron" just the rock's own base/skirt (elevation hugs the rock)?
print(f"\nRADIAL GROUND PROFILE around rock ({rcx:.2f},{rcy:.2f}) -- low_z(10th pct)=observed ground level:")
acc3=[]
for (t,pts) in clouds:
    if not len(pts): continue
    d=np.hypot(pts[:,0]-rcx, pts[:,1]-rcy)
    m=d<3.0
    if m.any(): acc3.append(np.column_stack([d[m], pts[m,2]]))
P=np.vstack(acc3)   # (dist, z)
print(f"  {'ring(m)':>10} {'n':>7} {'ground_z(p10)':>14} {'p50':>8} {'high_z(p90)':>12}")
edges=[0.0,0.2,0.4,0.6,0.8,1.0,1.3,1.6,2.0,2.5,3.0]
for i in range(len(edges)-1):
    lo,hi=edges[i],edges[i+1]
    zr=P[(P[:,0]>=lo)&(P[:,0]<hi),1]
    if len(zr)<5: print(f"  {lo:.1f}-{hi:.1f}{'':>3} {len(zr):>7}"); continue
    print(f"  {lo:.1f}-{hi:.1f}{'':>3} {len(zr):>7} {np.percentile(zr,10):>14.3f} {np.percentile(zr,50):>8.3f} {np.percentile(zr,90):>12.3f}")
print("  INTERPRETATION: if ground_z(p10) rises smoothly from the far rings (>~1.5m) toward the rock = a")
print("  REAL terrain rise the rock sits on (the rise itself is the hazard). If far-ring ground_z is flat")
print("  and only the inner rings (<~0.4m) are elevated = the 'apron' is the rock's own skirt (artifact).")
