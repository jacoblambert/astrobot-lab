# Recorded terrain gaps and headless loop testing

Updated October 4, 2026. Diagnostic report; no navigation promotion.

## Latest result — full loop completed

The normal headless **16 m commanded loop completed**, including four corners,
return to the start and recording shutdown. GT measures **17.636 m** of actual
base travel, including turn motion, over **299.13 simulated seconds**. Collection
took **33.8 wall minutes** on this host. All **17,949 native ticks** were integrated;
all **2,991 expected live grids** are recorded. The final completion-only unknown
projection is accounted for separately. No missing acquisition tail or fitted
trajectory alignment was used.

[Open the October 4 complete-loop HTML gallery](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-2ac54b276ccc4bb8a5fa9538a0939edb/gap-report.html),
or [view the three-grid overview PNG](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-8fc099c194834c1d8d311b0fcf2ddf58/gap-overview.png).
The self-contained HTML includes every fixed display time, not selected good
frames. The SVG overview was rasterized and visually inspected; this is not a
full-browser rendering test.

| Complete-route measurement | Result |
| --- | --- |
| First-published GLIM XY error, p95 / maximum / final | **15.22 / 35.70 / 3.85 cm** |
| Revised GLIM XY error, p95 / maximum / final | **12.35 / 16.89 / 4.29 cm** |
| First-published vertical error, p95 / maximum | **3.07 / 3.51 cm** |
| First-published yaw error, p95 / maximum | **0.369 / 0.613 degrees** |
| Terrain computation, median / p95 / p99 | **38.91 / 52.54 / 57.03 ms** |
| Peak mapper RSS | **271.54 MiB** |

No catastrophic drift was observed in this completed run. Both existing overall
pose diagnostic targets still fail; both 1 s relative-pose targets pass. This is
**physical IMU + one-time GT initialization + synthetic RTK-assisted GLIM**, driven
by the declared GT-guarded acquisition controller—not unassisted SLAM or navigation.
All 17,949 poses use matching physical `base_link`, fixed home and exact published
GT snapshots. Transport checks verify **110,509,448 points / 2,993 full windows**,
including acquisition boundary windows; all three mapper payload hashes match.

Nearby gaps, using the same body-relative 3 m region as the existing gallery:

| Fixed time after first map | Unknown cells / nearby cells | Robot-center classification |
| --- | --- | --- |
| +15 s | 195 / 901 | Measured support |
| +60 s | 46 / 930 | Measured support |
| +120 s | 12 / 901 | Measured support |
| +180 s, during a turn | 204 / 904 | Unknown patch, **0.5 × 0.6 m**, 0.26 m² |
| +240 s | 70 / 909 | Measured support |
| Final, +299 s | **28 / 900** | Measured support |

These are class availability, not terrain accuracy, obstacle recall or complete
wheel-support certification. Startup gaps remain; the completed return is much
better supported than the earlier short out/back examples.

### What changed, and what remains

The simulator now loads only the selected world rather than also importing the
unused LargeScale terrain engine. Floor-survey lookup reuses immutable bounds
and preserves the exact nearest-point result. Earlier fixes keep storage checks
from blocking recorder ROS callbacks and receive motion grants independently of
the simulator's diagnostic I/O. Lean startup remains opt-in. No route, sensing,
classifier, GC policy or 500 ms watchdog threshold was relaxed.

The completed run survived two natural full collections (**376 / 338 ms**).
This is one successful normal loop, not an endurance guarantee: the longest
native callback still took **465 ms**, and an exact archive check reached
**401 ms**. The two-files-per-tick layout remains and should be replaced by the
already-planned bounded diagnostic-topic/MCAP path before scaling capture length
substantially. Terrain p95 still exceeds its **50 ms** target.

The strict whole-input surface audit also remains **failed**, for a precisely
identified reference limitation, not GLIM drift: **two of 2,297,472 attempted
same-tick PhysX re-casts returned no hit**. Neither was a large residual. All
2,297,470 successful re-casts were within **0.1283 mm**; p95 was **0.00191 mm**.
Same-tick diagnostic queries shifted along each original ray recovered the
ground within about **1.2 micrometres** of the sensor range. This is consistent
with ray/triangle numerical sensitivity, not proof of its exact cause. The two
misses remain recorded and are not replaced by favorable queries. Improve the
independent reference before claiming terrain-safety qualification; do not retune
GLIM to compensate for these evaluator misses.

Next cycles: (1) bounded evidence-stream migration with loss/shutdown/equivalence
fixtures; (2) exact-output compute optimization and both resource profiles;
(3) independently validated wheel/obstacle scoring through turns/startup/return;
(4) declared 40–60 m and repeated-loop endurance runs. Keep the completed loop
as a regression and all preceding failures visible. Current final regressions:
**495 host tests / 44 isolated ROS tests pass**; the extra report tests prevent
cross-run pose-score substitution and hidden reference failures.

Artifacts: recorder `revision-d4d0f081b64f47caab3831bf2154ab08`; native
`revision-070d4d8a70084dbdb5414ad9dc945b35`; mapper
`revision-0113a56294cf4d8f817fe1404a8c820f`; pose report
`revision-0cf764a43a244f6db46e19181aa02712`. Full history and failed attempts are
in the [implementation ledger](implementation-plan.md).

## Historical October 3 retry — partial route, not qualification

The resumed headless run recorded **12.64 m of GT travel**, including three
corners, before the **30-minute wall-time budget** expired. It did not fail a
heartbeat or sensor-noise check. Its final pose brackets and one grid publication
are missing after shutdown; these remain explicit failures, not trimmed data.
The same-route slow-host execution profile is prepared, but another capture
currently needs more external archive space (111.7 GiB free / 114 GiB required).

[Open the October 3 recorded-grid gallery](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-db2f752c4ea24910a7bd8e522489dd26/gap-report.html).
It retains fixed display times, an unreached +240 s view and the missing final
projection. These are actual class grids, not reconstructed or inflated maps.
At +180 s, the unknown component containing the base center is **0.6 × 0.6 m**;
the last recorded grid has measured support at the center, with **161/900 cells
unknown** in the nearby 3 m region. Unknown is not a physical obstacle or hole.
Neither these fractions nor this gallery establishes obstacle recall.

Failed-prefix GLIM XY error p95/max: **17.60/43.33 cm causal**, **13.89/18.87 cm
revised**, using fixed frames and exact GT snapshots. Ten unpaired terminal
ticks remain reported. Both diagnostic pose targets fail. Recorded mapper
compute p95/p99 is **57.25/63.92 ms**, above the 50 ms p95 target; peak mapper
RSS is **258.3 MiB**. Full-loop, compute and independent terrain qualification
remain open. See the [implementation ledger](implementation-plan.md).

## Historical September 29 results

Evening follow-through: the no-change revision handler is implemented and
crossed **four SUBMAP/GLOBAL batches live**, including 2,227 historical frames,
without interrupting mapping. The next guarded loop reached **13.79 m / 228.40 s
of native acquisition**, then failed the simulator heartbeat guard, coincident
with a 404 ms full Python GC. Causal XY p95/max was **15.59/25.94 cm**, revised
**12.79/15.50 cm**; diagnostic targets still fail. This is not a completed loop.
The mapper's shutdown seal also timed out while hashing 5.84 GB; write-time
hashing now passes a full-size independent byte check and a real ROS shutdown
fixture. Original failed artifacts remain untouched. The gallery below still
shows its original recorded runs, not an export from the unsealed new mapper.
See [implementation ledger](implementation-plan.md) for artifact IDs and limits.

[Open the self-contained HTML grid gallery](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-d16e5df369084834b6cc3e1f91469640/gap-report.html)
or [view the three-grid overview PNG](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-477ff643380b4287bca9b1974a1e01a9/gap-overview.png).
The HTML needs no server/network; the external drive must be mounted to open it.

## How significant are the gaps?

These are actual recorded `/terrain_shadow/classes` grids: **10 cm cells,
6 × 6 m rolling window**, before obstacle cost inflation. Grey is unknown, not
occupied and not certified free. The gallery uses fixed times, not favorable
snapshots, and distinguishes missing publications from absent terrain support.

| Completed run / time after first map | Unknown patch containing the robot center | Interpretation |
| --- | --- | --- |
| Straight out/back, +15 s | 2.1 × 2.0 m bounds; 3.49 m² | Early support gap, larger than isolated cell noise |
| Straight, +30 s | Center supported | Startup shadow is behind the moving robot; not full-footprint certification |
| Straight, final +81.8 s | 1.7 × 1.7 m bounds; 2.41 m² | Substantial remaining gap at the return/start position |
| Combined turn/out/back, +30 s | Center supported | Same distinction between current position and earlier shadow |
| Combined, final +63.1 s | 1.8 × 1.8 m bounds; 2.51 m² | Similar startup/return gap |

The new longer prefix is better locally once moving: at fixed **+60 s**, the
body-relative 3 × 3 m region contains **34 unknown / 900 cells**; at **+120 s**
and the final +130.2 s map, **41 / 899**. The other cells in these examples are
measured support, with none occupied in that region: approximately **95–96%
classified support**, not independently measured terrain accuracy. The robot
center is supported from +15 s onward in the displayed longer-prefix maps.
We did not complete its return to the original startup position.

Bounds describe an eight-connected unknown component, **not a physical hole**.
Very early unknown regions connect to the window boundary and are not bounded
holes. The solid overlay is the calibrated chassis (~1.01 × 0.57 m), not the
whole wheel footprint; the dashed rectangle is the 1.70 × 1.40 m acquisition
guard envelope.

Earlier exact GT-pose counterfactuals retained these gaps: pose correction alone
did not resolve them. This supports investigating visibility and ground-support
evidence near startup, rather than simply retuning SLAM. Obstacle cost tapering
already exists, but deliberately **does not convert unknown into free**.

## Longer test: current result

First headless `loop16` attempt stopped during its first turn: the native sensor
gate rejected `native local XYZ/ray association mismatch`. It archived 2,727
native ticks over 45.433 s, not a completed 16 m route. Failed-prefix GLIM
diagnosis paired 2,719 ticks / 2.262 m of travel; causal XY p95/max were
8.76/12.20 cm, revised 6.50/6.86 cm. Eight missing tail associations remain
counted. This is an acquisition failure, not observed catastrophic SLAM drift.
The failing equality is sensor-local XYZ versus beam direction × range, before
GLIM or GT transforms. It cannot by itself be explained by a map/base-link offset.

The mapper produced 453 receipts; 452 grids reached the recording. Its final
unpublished receipt remains missing. Prefix compute p95 was 44.84 ms versus
the unchanged 50 ms target; an incomplete prefix is not runtime qualification.

Code inspection found the rejected tick was lost before archiving. The diagnostic
repeat adds failure-only preservation of rejected arrays/stamps/poses/hashes and
still aborts. Successful payload equivalence and failure preservation pass unit
tests; the sensor rejection did **not** recur during this repeat, so its cause is
unresolved. No valid input, classifier, mounting, route, evidence lifetime or
guard threshold changed.

**The repeat reached 8.63 m / 130.48 s**, through the first turn and into the
second. It stopped because the terrain consumer rejects non-ODOM revision
messages. The recorder then shut down and the driver correctly stopped on loss
of its recorder. This was not a new command-owner conflict.

- GLIM first-published XY p95/max: **14.96 / 34.95 cm** (live pose targets fail).
  Revised: **9.15 / 11.56 cm** (diagnostic targets pass). Rotation error maximum
  0.583° causal / 0.399° revised. No observed runaway divergence, but transient
  error is worse than in the short baselines. Nine missing tail associations
  remain counted; 7,821 of 7,830 recorded native ticks were paired.
- All **1,303 projection grids** reached the recorder; a repeated final stamp
  carries fault invalidation, not another valid observation. Compute p95/max
  **47.16 / 67.07 ms**; sealed peak mapper RSS **237.22 MiB**. Incomplete run,
  not whole-route/resource qualification. Simulator stopped after sealing.
- Offline inspection of the SUBMAP/GLOBAL batches finds **1,277 / 1,327 frames**,
  no new frame IDs, changed timestamps, extrinsics or deskew flags, and maximum
  translation changes **3.97e-14 / 2.01e-14 m** relative to preceding poses.
  These are bookkeeping callbacks with numerical roundoff, not a meaningful
  loop-closure correction. Disabling optimization does not disable callbacks.
- Correction (September 29 evening): the consumer's actual pose limit was
  **2,048 frames**, not the previously reported 1,024. The first two batches did
  not exceed it. Merely removing the kind check would still not validate whether
  historical terrain evidence remains coherent. No guard was bypassed.

Both attempts are retained. A clean repeat cannot erase the first failure.
Much-longer-route qualification has **not** been established.

## Historical September 29 checklist — superseded, not current tasks

The list below was written before the completed October 4 loop. The harmless
SLAM update handling and full loop are now implemented/tested; millimetre sensor
differences are diagnostic rather than fatal. Do not treat this old list as a
reason to delay terrain/navigation assessment. Current work is measuring whether
map gaps actually block useful short moves; see the implementation ledger.

1. Diagnose the exact rejected native tick if reproduced; keep the 1 mm
   association guard and original sensor bytes. Distinguish simulator return
   inconsistency from frame or adapter mistakes before choosing a fix.
2. Add a **bounded, validated no-change revision path**: preserve revision
   ordering and metadata checks; verify all relevant retained/frozen evidence
   poses against non-ratcheting references before accepting bookkeeping-only
   batches. Changed, missing, epoch-crossing or deskew-invalidating evidence
   must still hold/reset/rebuild. Test batches larger than 1,024 frames, real
   corrections and unknown IDs; do not simply ignore callbacks or rename their
   kinds. Check exact replay equivalence and CPU/memory limits. Then complete
   the coupled 16 m loop, including acquisition/transport audit, causal
   GLIM error, all recorded maps, revision transitions, missing samples and
   runtime/memory. Longer revision/GC limits remain open until exercised.
3. Then declare a 40–60 m surveyed route and a separate repeated-loop endurance
   test. Repeated laps test duration/revisits, not new exploration area. Update
   bounded acquisition/storage contracts explicitly; do not just increase a
   timeout or suppress revision callbacks/watchdogs.
4. Independently evaluate wheel-support and obstacle boundaries through startup,
   travel and return. Unknown coverage and GT pose agreement alone cannot prove
   obstacle recall or safe autonomous motion.

Reproduction commands: [agent runbook](agent-runbook.md#selected-level-lidar--moving-glim-experiment-opt-in).
Artifacts and changing status: [implementation ledger](implementation-plan.md).

Validation: **247 focused/regression tests pass**, plus Python compilation and
shell syntax checks. Actual-grid exports verify full grid transforms and source
hashes. Overview SVG was rasterized and visually inspected. Automated Firefox
HTML screenshot failed because the Snap launcher could not access its temporary
profile; no browser-render pass is claimed. HTML structure/escaping and SVG
class/scale fixtures pass. No algorithm or navigation stack was promoted.
