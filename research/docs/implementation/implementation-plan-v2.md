# Implementation plan v2 — Locally robust terrain estimation

**October 18 handover:** this document preserves the broader design and its
September starting point. Use the [current handover](implementation-plan.md#october-18-handover--start-here)
for implemented state, exact evidence IDs and ordered next steps. Moving
GLIM/terrain integration and v13 committed-route trials now exist, but navigation
is not qualified. Fix partial-cell current-footprint precedence first, then
command-age waits and heading efficiency. Do not restart the old mounting/ray
selection study: the live baseline uses level `selected_optical_v2`, with the
0.45 m optical-origin height, not the earlier generic profile. The checked
dev-container archive bind is stale and must be recreated before capture/replay.
No new implementation or simulation validation occurred in this documentation pass.

Updated: 2026-09-27. Status: opt-in rolling persistent candidate and reference-frame fix implemented; offline runtime passes on the retained recording, full local qualification remains open. The [bounded mounting study](mounting-study.md) now has native Isaac sampling validation; no tested fixed pitch dominates, and startup/self-occlusion remain open. Production mount unchanged.

**Goal:** use estimated poses and LiDAR to reliably assess the rover’s next short motion, with sufficient ground support, obstacle clearance and a stop when evidence is inadequate. First deliver geometric traversability; soil bearing capacity and traction prediction remain outside this milestone.

September 27 correction: [single-LiDAR ray redesign](ray-design-study.md) must use
**0-degree mounting pitch**, not the earlier +5-degree experiment. The explicit
level-mount screen compares the same 11 layouts at the original 32-ring /
1999-column budget and preserves recorded calibration only as provenance.
Both nonuniform +10..-60 variants remain promising in this corrected screen;
the tilted ranking is superseded. Native level validation passes all 22 cases,
with actual USD rotations verified identity; 89 host regressions pass. This is
not live GLIM/terrain or compute qualification.
Optical self-occlusion and native custom-ray adapter contracts precede moving integration.
User selected **mid-dense +10..-60 at a level mount**; stop layout selection.
The frozen development contract is `tools/local_terrain/selected_lidar.json`.
The active task is local/mid-range terrain assessment with this fixed sensor;
GLIM integration follows useful terrain evidence, not further beam-table tuning.
The quick mid-dense far-structure/downsampling screen now passes all 640 declared
cases, with a single-height-band limitation at 40 m. Current native terrain
assessment confirms improved near-ground support, but six settled sensor views
detect 7/8 boxes versus baseline 8/8; the remaining box stays unknown. No false-free
cells in 1,152 maps. This is not moving/SLAM/terrain qualification; see the
[ray-design study](ray-design-study.md) and execution ledger for bounds and failures.

This plan sets the next development priorities. Keep `implementation-plan.md` as the historical experiment ledger. New local experiments do not retroactively qualify failed full-map runs.

## Starting point

- Keep the recovered GLIM configuration and native acquisition-time LiDAR path as the baseline. Current localization is synthetic-RTK-aided with one-time GT initialization; unaided performance is unqualified.
- `combined10` retains 96.6% / 99.4% of GT-pose-mapper free cells within 0–2 / 2–3 m of spawn, falling to 60.6% at 5–10 m. This is final-frame consistency, not terrain truth or moving-rover coverage.
- Remaining mapping failures involve temporal registration scatter, boundary extent and ground selection. The latest live run also fails runtime/heartbeat gates independently of localization accuracy.

## Design decisions

| Keep | Change or investigate | Defer |
| --- | --- | --- |
| Native timestamps, audited extrinsics, independent geometry evaluation | Rolling local map; start near 6 × 6 m at 10 cm resolution | Global map perfection and loop-closure reintegration |
| Separate surface and hazard evidence; unresolved support stays unknown | Explicit relative-pose uncertainty and coherent pose revisions | Further SLAM backend swaps without evidence |
| Wall-sliver, hole, partial-face and expiry regressions | Ground selection anchored to a supported connected surface | Full Nav2/exploration integration |
| Existing recordings and retained failures | Vehicle-pose and short-motion feasibility as primary outputs | Optimizing the last fraction of full-map coverage |

The proposed map size and shorter retention windows are experiment parameters, not accepted operating limits.

## 1. Establish the local operating contract and baseline

- Read actual rover dimensions: wheel size/contact geometry, body clearance, admissible step, pitch and roll. Define the 8 cm hazard requirement together with minimum horizontal extent, hole width/depth and detection distance.
- Freeze speed, sensing arrangement and permitted observation motions. Derive required observed distance from evidence age, stopping performance and uncertainty margin: `d_required > v × total_latency + v² / (2 × stopping_deceleration) + margin`.
- Replay existing recordings and score continuously around the **current rover**, including turns. Use independent simulator geometry for terrain and swept-body/support evaluation. A GT-pose mapper is only a consistency comparator.
- Measure local surface displacement and relative-motion errors over candidate fusion windows, extending existing one-second checks. Verify hazard visibility against actual beam geometry; do not infer sensor specifications from the asset name.

**Exit:** a fixed local evaluation region, hazard family, platform limits and baseline report. Record missing physical parameters as explicit assumptions; continue diagnostics without claiming qualification.

## 2. Make local observations geometrically consistent

- Audit full-scan formation into GLIM: acquisition times, coordinate conventions and motion distortion. Compare verified deskew with the existing input on identical recordings; do not invent per-point times.
- Preserve native observation geometry before pooling. Distinguish within-observation noise/roughness from between-observation registration displacement.
- Prototype bounded reconstruction from retained observations using one coherent set of poses available at publication time. Consume relevant local ODOM revisions; use no future information. A rolling grid alone does not fix pose inconsistency.
- Keep local sensing/action in a continuous reference. Handle global corrections separately; changing a frame name is insufficient.

**Exit:** repeatable local reconstruction through translation and turns, with explicit pose availability and reset behavior. Report remaining geometry error; proceed to uncertainty treatment rather than demanding perfect poses.

## 3. Build and compare the local terrain candidate

- Start with a rolling 6 × 6 m grid at 10 cm resolution. Compare declared short windows, initially 2 and 5 s, against the 30 s baseline. Check near-body blind regions and support loss during stops; select retention from evidence.
- Track one connected supporting surface plus separate obstacle/dropoff evidence. Include connectivity in ground-hypothesis selection so hole bottoms cannot silently replace the driving surface. Preserve lower/upper returns needed to detect hazards.
- Propagate or conservatively bound relative-pose uncertainty in height and XY. Correlated returns must not create artificial confidence. Bound boundary extent using observation geometry and uncertainty; retain protection for partial faces and thin walls.
- Add an offline rover-pose and swept-motion query: wheel support, body clearance, slope/step limits and rejection reason. Retain uncertainty and age alongside terrain geometry.
- Run one established elevation-mapping comparator on the same inputs if integration is modest. Compare safety, usable corridors and compute; do not assume its interpolation or learned traversability output satisfies this rover’s requirements.

**Exit:** select a candidate by independently checked local feasibility and stability. Reject new false-safe regressions. If no candidate improves usefulness safely, investigate the dominant failure and continue with a revised hypothesis.

## 4. Qualify runtime and failure behavior

- Profile the retained natural recording by stage, then test concurrent Isaac/GLIM/mapper load. Attribute simulator GC stalls separately from mapper computation.
- Bound buffers, work and memory. Measure acquisition-to-usable-output age, queue growth and p95/p99 cycle time. Retain the existing 50/100 ms cycle gates for comparison; declare any revised operational budget before qualification.
- Handle missing observations through explicit coverage/freshness degradation where justified. Stop on insufficient local support, stale authoritative output, incoherent transforms or resets. Fix shutdown invalidation and partial-update publication.
- Repeat timing measurements under declared load and report all attempts. Preserve watchdog protection while fixing the scheduling cause.

**Exit:** sustained operation within the declared latency/stopping budget, bounded resource use and verified invalidation/recovery behavior.

## 5. Demonstrate local robustness on held-out cases

Freeze the candidate, parameters and evaluator. Use fresh terrain seeds and routes covering stationary observation, straight travel, turns, slopes, rocks, narrow faces, holes, occlusion and sensor gaps. Keep development fixtures separate.

| Acceptance dimension | Required evidence |
| --- | --- |
| Safety | Zero observed false-safe configurations or swept motions in the declared hazard family; report counts and exposure |
| Usefulness | Predeclared corridor availability and unnecessary-stop targets met; an always-unknown map cannot pass |
| Sensing | Required hazards detected before the stopping boundary; unresolved regions prevent entry |
| Stability | Predeclared limits on free/blocked flicker and registration-induced false hazards met |
| Runtime | End-to-end age and resource budgets met under representative concurrent load |
| Recovery | Faults invalidate affected decisions; motion resumes only after fresh sufficient evidence |

**Exit:** a completed local traversability demonstration using estimated poses. Then test RTK outages/unaided operation and integrate the actual navigation consumer with the qualified footprint/sweep checks.

## Execution discipline

Work through diagnosis → one mechanism-specific change → replay → relevant regressions → held-out validation. Continue until the milestone passes or a concrete blocker requires unavailable data, hardware or a user decision; partial improvement is not completion. Keep exact replay checks for semantics-preserving changes, physically meaningful tolerances for estimator comparisons, and concise experiment records. Do not make unrelated global-map gates prerequisites for local diagnostic progress.

## Approved execution cycles — September 26

The user selected **95% availability of independently verified safe short motions**,
overall and within each declared scenario family, with **zero observed false-safe
motions**. This replaces neither the historical cell metrics nor their frozen
denominators: it is a new vehicle-level qualification contract. Unknown decisions
on reference-safe motions count against availability. Missing reference geometry
remains visible and cannot become a free label. No numerical availability result
alone certifies safety.

| Cycle | Work and improvement loop | Exit / failure branch |
| --- | --- | --- |
| 0: reproducible baseline | Preserve original replay and naive coherent reference; freeze input/source/library/evaluator hashes; maintain this roadmap, ledger and runbook. | Exact baseline replay and explicit candidate status. |
| 1: independent truth | Audit simulator collision/terrain geometry against GT transforms, loaded wheel support and known placements. Build independent pose/sweep labels; measure braking, native-beam hazard visibility and conservative platform limits. | Freeze motion queries, hazard dimensions, stopping margin and latency budget before selection. Missing physical limits block qualification, not offline diagnostics. |
| 2: controlled ablations | Compare observation-pose revision policies with identical current-rover poses, grid, retention and classifier; compare 2/5/30 s, then 6/20 m extents. Keep all body-relative queries, including outside-grid and startup. | Attribute failures to coverage, expiry, registration, boundaries or ground selection before interactions/tuning. Frozen-observation placement is an experimental ablation, not the historical first-publication runtime. |
| 3: frame/observation integrity | Exercise common gauge and relative corrections, gravity, rolling boundaries, missing brackets, late revisions, reset and lost anchors. Audit GLIM full-scan formation; only compare deskew with genuine timing. | Causal coherent geometry with measured residuals and fail-closed unsupported events; change GLIM only if its residual dominates downstream failures. |
| 4: bounded reconstruction | Cache exact transforms by dependencies; rebuild affected old/new spatial tiles plus complete classifier neighborhoods from ordered source observations. Preserve naive differential reference. | Exact outputs for semantics-preserving work and passing offline resource gates; approximate summaries are separate algorithm candidates. |
| 5: support retention | Separate support/hazard provenance and lifetimes; test stops, turns, blind areas, revisit, expiry and occlusion. Compare retention and observation diversity separately. | Improved independent safe-motion availability without false-safe regressions or stale free-space resurrection. Inadequate beams require a separately declared sensing experiment. |
| 6: connected ground + uncertainty | First select observed connected supporting surfaces while retaining alternatives/dropoffs; then bound XY/height registration uncertainty and correlated evidence. | Pass holes, wall slivers, partial faces, phases and rotations, with useful motion improvement. If mechanisms stall, time-box one established mapper comparator to two engineering days on identical inputs. |
| 7: vehicle interface | Offline pose/short-trajectory query returns feasible/blocked/unknown, reasons, support/clearance margins, age, uncertainty and revision. Conservatively bound entire sweeps, not just endpoints. | Agreement with independent analytic/simulator truth; clearance-gradient cost never overrides invalid support or a failed sweep. |
| 8: live runtime | Replay → stationary → straight → turns → bounded mixed route; test concurrent Isaac/GLIM/mapper, native gaps, delayed revisions, resets, overload and shutdown. | Existing 50/100 ms p95/p99 and memory gates, both resource profiles including dense_flat, bounded end-to-end age and safe recovery. All attempts retained; no relaxed watchdog or disabled GC. |
| 9: held-out qualification | Freeze code/config/evaluator/envelope/thresholds and sealed fresh seed/route manifest; cover stationary, straight, turns, slopes, rocks, narrow faces, holes, occlusion and gaps. | Zero observed false-safe, ≥95% motion availability per family and overall, detection/refusal before stopping boundary, runtime/stability/recovery passes. Failed cases become regressions; next qualification uses a fresh sealed batch. |

After v2: controlled RTK degradation/outages and unaided tests; navigation shadow
queries; guarded short closed-loop routes; longer loops/exploration. Global
reintegration is separate; unsupported global changes invalidate local decisions.
Geometric traversability is in scope, physical-rover certification, soil bearing
and traction prediction are not.

Every experiment records hypothesis, one mechanism-specific change, command,
artifact, result, regressions and next decision. After two unsuccessful variants
of one mechanism, review the failure before another parameter adjustment. Prioritize
safety, then valid sensing/frame/timing assumptions, resources, then usefulness.
Archive/GPU failures stop affected workloads, not independent safe fixture work.

### Current cycle status

- Cycle 0: roadmap and reproducible baselines established; original runtime preserved.
- Cycle 1: continuous support-patch geometry and motion-score accounting implemented
  with analytic tests. Active PhysX body-envelope overlap/sweep adapter passes
  six isolated Isaac fixtures, including a thin wall between motion endpoints.
  This is body clearance only; independent wheel support, loaded contact/stopping
  contract and natural collision truth remain open. **Terrain collider uses meshSimplification**: authored USD/DEM is not
  automatically the active physical surface. Do not change simulator geometry for scoring.
  Active body fixtures now pass 13 cases, including four probes of the actual
  chassis collision prim. Found and fixed the USD inspector's unsafe reliance on
  a stale, pre-scaled Cube extent: body y half-width is **0.28545 m**, not 0.16296 m;
  flat loaded bottom is about **0.13615 m**, not 0.23413 m. The asset is unchanged.
  Awake force characterization passes but normal sleeping force reports disappear;
  neither result certifies wheel contact-patch extent or braking.
- Cycle 2: five matched 320-projection development arms completed on the retained
  failed source. Retention dominates availability: 2→5 s adds 10.93/21.04 percentage
  points at 0–2/2–3 m. Frozen observation placement changes only 0.10/0.25 points;
  expanding 6→20 m changes 0.0004/0.79 points. These are not independent motion scores.
- Cycles 3/4: latest-available pose indexing and bounded exact preparation implemented
  opt-in. Reference/prepared 2 s and 5 s pairs match every layer and association in
  all 320 projections. Initial p95 improves 117→92 ms and 288→216 ms, respectively,
  still failing runtime gates. Profiles attribute ~146 ms of a steady 5 s cycle to
  repeated C++ integration. Default remains naive reference; no classifier change.
  Final filtered-only cache revision preserves all 320 results, lowers RSS from
  175 to 143 MiB, and measures 222 ms p95 (not a speedup over the preceding 216 ms).
  Final host regressions: 555 passed; local suite also passes all 139 tests at OMP=4.
  Follow-up opt-in native-patch normal reuse plus exact ordered sparse-neighbor
  traversal lowers 5 s p95 to **179 ms**, with all 320 outputs unchanged; still
  fails 50/100 ms runtime. Both isolated C++ resource profiles pass, but full-window
  reconstruction remains too costly. Expanded host regressions: **591 passed**;
  serial/OpenMP C++ fixtures pass. No default runtime or classifier policy promoted.
  Deferred classification of small inputs further reaches **146 ms p95**, still
  unqualified. Its unconditional version failed both resource profiles; the
  revised large-input direct path restores both passes. **647 host regressions**
  and **187 serial local/differential tests** pass on this revision.
- Cycles 4/5 follow-up: a separately declared persistent frozen-placement algorithm
  integrates each input once. Fixed 20 m and rolling 6 m variants use immutable
  30 s buckets; entering cells are unknown and evicted evidence never resurrects.
  No retrospective pose repair or uncertainty certificate is implied. On the
  320-publication development prefix, rolling matches fixed-grid swept-patch
  counts and reduces instrumented cycles to roughly 15–24 ms. **696 host tests
  pass**. Uninstrumented complete-stream/resource/serial gates are in progress;
  this is not exact equivalence to coherent reconstruction or a promotion.
- Vehicle-level diagnostic: continuous assumed 80x120 mm wheel patches, stationary,
  10 cm forward and ±0.1 rad turns at every publication. Five-second support is
  **0/320** in each family; 30 s is **149/149/146/148 out of 320**, with startup
  retained. Full support first appears about 17 s into the prefix. These are class
  availability figures, not independent safe-motion scores or measured contacts.
  Current age and plane-sigma channels cannot certify dependency-aware support
  freshness or relative-pose uncertainty; do not reuse them as such.
- **Superseding incremental checkpoint:** GT auditing found a moving-reference
  defect in the initial persistent candidates. They are rejected despite passing
  timing. Current incremental code fixes the local/map reference, retains an
  oldest-session-frame witness, and invalidates on witness changes rather than
  moving the robot's frame away from frozen terrain. A non-ratcheting 1e-6
  matrix-entry tolerance admits numeric jitter only. Coherent rebuild is unchanged.
  Exact native GT pairs show 30–60 s origin displacement p95 **36.77→7.93 cm**;
  residuals still need uncertainty treatment. No GT correction enters mapping.
- Current 5 s raw / explicitly declared 60 s evidence replay supports
  **544/544/538/537 of 748** assumed stationary/forward/left/right patch queries
  (**71.79–72.73%**, startup and later failures retained). It is not independent
  safe-motion availability. Uninstrumented final-frame-policy replay measures
  **19.74/22.70 ms p95/p99**, **201.15 MiB**; original heartbeat failure remains.
  Standard 30 s C++ dense/capacity profiles pass, not every 60 s stress workload.
  Startup sensing, later extent support, dependency-aware freshness/uncertainty,
  active terrain/contact truth, consumer-time pose audit, live load and held-out
  gates remain. Do not promote a candidate on timing or class coverage alone.
  Final revision validation: **721 host regressions / 230 serial local tests**;
  both C++ fixture programs pass, and the ordinary C++ library remains byte-identical
  to its baseline. Full qualification is not achieved.
- Cycles 5–9: not qualified or completed. Next work is independent active-collider
  reference plus revision-aware spatial reconstruction with full stencil dependencies,
  then support retention/connected ground, uncertainty, vehicle sweeps and live gates.
  Cache tuning alone is not a route to the 50 ms p95 target. Detailed experiment IDs,
  final cache revision and regression commands are in the execution ledger.

### Retention constraint discovered during execution

Do not select 2/5 seconds merely to reduce compute. The recorded 32 native zenith
angles and +5-degree/off-center mount, combined with measured loaded geometry,
give an optimistic flat-ground direct-visibility bound of **9.47 s front / 14.59 s
rear** at 0.10 m/s. This calculation even ignores discrete-ring gaps and robot
occlusion. Short windows cannot directly supply current wheel-track ground in
this benign straight-motion case. Thirty seconds only passes that necessary
bound, not the availability, uncertainty or compute gates.

Next support work must therefore decouple a short reconstruction/pose-update
window from longer-lived support, with declared uncertainty/expiry, or test a
separately declared sensing change. Stops/turns can make support older still.
Do not solve the blind region by labeling it free, trimming startup exposure,
or quietly changing the frozen lifetime. Retain the short-window implementation
as a differential/ablation reference, not a presumed deployment policy.

## First-slice implementation checkpoint — September 26

- Legacy cleanup: six superseded BEV sweeps are preserved under `tools/legacy/terrain_sweeps/`,
  with original hashes and forwarding entry points. They are not safe current runners:
  some overwrite/delete bags and stop processes. Tests never execute their implementations.
  The root runbook now redirects to `docs/implementation/agent-runbook.md`.
- `tools/local_terrain/` is a separate offline candidate; the original first-publication
  mapper, native transport, source-hash checks and C++ classifier remain unchanged.
  Prototype uses one latest-available ODOM state per projection, shared-anchor gauge
  reconciliation, raw native retention, 60x60 cells, and declared 2/5/30-second windows.
- Initial baseline: 171 existing host contracts passed / 3 library-dependent skips.
  First candidate tests: 22 pass with the frozen real C++ library, including interrupted
  reconstruction invalidation, frame/gauge checks, queue/resource bounds and cleanup.
- Host archive is writable. The earlier read-only observation was the sandbox view,
  not a device fault. No hardware repair, reset or simulator capture was performed.
- Complete recorded-prefix comparisons now cover all 748 projections, with failed-source
  status retained. Original 30 s baseline is exact against every recorded hash/state/input;
  2 s and 5 s local candidates reconstruct causally with the unchanged classifier.

| Diagnostic over recorded prefix | Original 30 s | Local 2 s | Local 5 s |
| --- | --- | --- | --- |
| Mean free-query availability, 0–2 m | 68.55% | 33.05% | 44.76% |
| Mean free-query availability, 2–3 m | 79.86% | 21.89% | 49.50% |
| One-second relative translation p95, same 738 pairs | 4.72 cm | 1.70 cm | 1.70 cm |
| Offline cycle p95 / p99 | 61.32 / 75.58 ms | 117.54 / 134.82 ms | 284.79 / 311.06 ms |

These are availability and geometry diagnostics, not terrain-truth acceptance.
The source route failed; no missing tail is fabricated. Later revised poses help
relative geometry **causally**, but neither short window is a qualified operating
policy: both lose support and fail runtime gates. Profiled rebuilding spends most
time repeating native integration and point transformation, not final projection.

- Final host regressions: **414 passed**, including 80 local/cleanup/evaluator tests.
  Both isolated frozen-core resource profiles pass; the local rebuild wrapper does not.
  USD geometry is extracted, but loaded contacts, braking and safe attitude limits
  remain explicit unknowns. No simulator world or motion was started.
- Keep these tools as a correctness reference. Next compare retention/support and
  incremental per-observation reconstruction without changing safety thresholds,
  then implement connected-ground/uncertainty feasibility. Do not resume full-loop
  qualification as a prerequisite for this local work. Detailed commands, artifacts,
  failures and exact resource caveats are in the historical implementation ledger.

## Literature basis

- [Fankhauser et al., probabilistic elevation mapping](https://www.research-collection.ethz.ch/items/563227f2-bb05-434b-8aef-1b001a9fdebc): local terrain and pose uncertainty.
- [Miki et al., 2022](https://arxiv.org/abs/2204.12876): practical elevation mapping, drift compensation and visibility cleanup.
- [GLIM, 2024](https://arxiv.org/abs/2407.10344): fixed-lag revisions and motion-compensated registration.
- [ACE](https://arxiv.org/abs/1808.00031): wheel-support, attitude and clearance bounds.
- [STEP](https://arxiv.org/abs/2103.02828): vehicle-dependent traversability under uncertainty.
