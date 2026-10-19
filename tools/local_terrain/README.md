# Local terrain v2 — offline diagnostic prototype

This package is opt-in and publishes no ROS messages or motion commands. Its
`valid` means a complete geometric output, never permission to drive. No new
classifier, uncertainty safety certificate, vehicle sweep policy or Nav2
integration is claimed.

The contact-support experiment is also opt-in: `contact_support.py` keeps
startup/current/traversed support separate from LiDAR classes; `contact_query.py`
checks full paths and explicit hazards; `contact_index.py` caches unchanged
geometry without retaining stale corrected poses. `assess_contact_support.py`
replays completed captures and preserves the original queries while adding
separately labeled small-step/recovery metrics. Its `--support-core` policy
requires less positive support and is **not physically qualified**. It must not
be connected to command permission or treated as free-cell painting. See the
[current results](../../docs/implementation/online-terrain-scenarios.md).

Mounting diagnostics are separate from this mapper: `diagnose_local_mounting.py`
compares fixed pitch and virtual yaw using a no-occlusion flat-plane model;
`run_local_isaac_fixture.py mounting --study-run <run>` checks stationary native
LiDAR rays in analytic flat/box scenes. Neither produces mapper evidence or
motion permission. See the [runbook](../../docs/implementation/agent-runbook.md)
and [experiment ledger](../../docs/implementation/implementation-plan.md) for
the fixed denominators, provenance, failures and limits.

`diagnose_local_ray_design.py` separately changes the vertical ring table while
preserving 32 rings, 1999 azimuth columns and the baseline mount. It compares
uniform/asymmetric and nonuniform layouts, including minimum-range losses and
two small-obstacle placement banks. `run_local_isaac_fixture.py ray_design
--study-run <run>` validates these tables with an isolated Generic range sensor;
it does not replace the live LiDAR capture adapter or production sensor asset.

## Provisional operating contract

| Item | Declared diagnostic value / qualification |
| --- | --- |
| Grid and query region | Default 60x60 at 0.10 m; 200x200 is an explicit extent ablation. Origin follows estimated base XY on the local lattice. Both evaluate the same fixed 120x120 body-relative query lattice at 0.05 m over [-3,3)², including outside-map queries. |
| Motion | Existing declared slow-motion envelope: 0.10 m/s and 0.10 rad/s. No new observation motion or route is authorized by these tools. |
| Sensor | Audited native acquisition chunks and recorded mount; no full-scan substitution or invented beam timings. Actual recorded sensor parameters, not `vlp16` naming, define sensing. |
| Windows | 2 and 5 s reconstruction candidates; 30 s remains an explicitly declared comparator, not silently relaxed retention. |
| Positive hazards | 0.08 m relative height. Existing synthetic fixtures span heights 0.05/0.08/0.10/0.14/0.20 m and widths 0.05/0.10/0.20 m. Those tests do not establish real LiDAR detection distance. |
| Negative/partial hazards | Existing 0.2/0.6 m hole-width cases, rotated hole rims, partial faces and thin walls remain regressions. Minimum qualified hole width/depth and visibility distance remain unresolved. |
| Wheel geometry | USD inspection: centers approximately x ±0.256, y ±0.2854, z 0.03282 m in base. Visual wheel diameter about 0.3555 m and width 0.11446 m. Visual bounds are not loaded contact patches. |
| Dynamic limits | Braking deceleration, admissible steps, pitch/roll, traction and soil bearing are unqualified. Existing mapper slope 0.35 is not a certified rover tip limit. |
| Body collision shape | Earlier BBoxCache values are withdrawn: the Cube has a stale, pre-scaled authored extent. Size + transform give x ±0.50370, y ±0.28545, z [-0.00877,0.25873] m in base; active PhysX volume probes confirm the discrepancy. Flat loaded bottom is about 0.13615 m above ground, not the earlier 0.23413 m. This is not a safe-step/attitude limit. |
| Stopping boundary | Require distance greater than v×latency + v²/(2×deceleration) + margin. No numeric safe distance is asserted until deceleration, end-to-end age and margin are fixed. |
| Resources | Retained raw observations ≤256 MiB, bounded queues/pose history, 1 GiB resource gate, 2 GiB diagnostic hard stop; preserve 50/100 ms p95/p99 cycle gates. |

## Reconstruction policy

1. Deliver archived callbacks in actual receipt order. The next received clock
   bounds availability, as in the retained live implementation.
2. Validate and commit an entire ODOM revision batch. Store the latest pose state
   known at that publication, not final poses loaded from the future. Missing
   ancestry, changed extrinsics, resets and global/submap events invalidate.
3. Reconcile gauge corrections through a shared IMU anchor and explicit
   `local_from_map`. Initial local axes retain map's gravity alignment; this is
   not a TF rename or a qualified globally corrected controller frame.
4. Retain native sensor-local XYZ and acquisition times. Interpolate IMU poses
   before multiplying the LiDAR extrinsic. Rebuild all retained observations
   using this one committed state; preserve their acquisition order/lifetime.
5. Build a fresh grid and expose it only after a successful projection. Interrupts
   invalidate rather than leave a partial update valid. Closing always invalidates
   the in-memory view, even after software stream completion.

This is deliberately a correctness-first rebuild, not an optimized incremental
implementation. Every output records revision, transform, window, origin,
associations, age, cost hash and resource/timing observations.

Separate **incremental frozen-placement experiments** now exist; the default
remains `rolling_rebuild`. They are algorithm changes, not exact accelerations:

- `fixed_incremental`: fixed initial 20x20 m grid, 30 s immutable buckets.
- `rolling_incremental`: rolling 6x6 m grid, same persistent 30 s buckets.
  Requires a `--rolling-grid` library. Integer shifts move retained world-coordinate
  evidence; newly entering cells start unknown, without historical backfill.

Both require `frozen_observation`: an observation keeps its first-used local pose,
while the current rover still uses the latest causal pose state. **The local/map
reference is now fixed.** An oldest-session-frame witness is retained; a change
beyond 1e-6 matrix-entry numeric tolerance invalidates the map. Comparison is to
the initial witness, not a succession of small changes. Earlier moving-anchor
incremental artifacts are rejected: they moved the robot's reference away from
frozen terrain, reaching 37 cm p95 old-evidence displacement. The fixed-reference
diagnostic reduces that to about 8 cm p95, without GT input. This residual is not
a certified uncertainty bound. These candidates do not repair old evidence after
relative pose revisions. Native inputs integrate once;
shifts do not refresh stamps, and eviction/re-entry cannot resurrect evidence.
Unsupported revisions/faults/close release the persistent map and invalidate its
output. These candidates still lack qualified relative-pose uncertainty, complete
support freshness, independent motion truth and live/held-out acceptance.
The rolling variant can explicitly separate `--window 5` raw retention from
`--evidence-lifetime 30|60`. Sixty seconds is a different static-scene evidence
contract, not a silent baseline extension or a safety pass. Hazards and support
both retain immutable timestamps; this is not yet separate hazard/support policy.

Exact opt-in C++ optimization experiments are built with `tools/build_local_terrain.py`:
`--patch-normal-cache` avoids repeated identical eigensolves; `--sparse-native-patch`
skips inactive native neighbor slots in the same merge order. `--defer-native`
classifies small independent inputs across four workers while keeping large scans
on their original direct path. No observation, classifier threshold or lifetime
changes. The 5 s / 320-frame development replay falls from 222 to 146 ms p95 with
identical outputs, **still failing runtime**.
Both isolated C++ resource profiles pass; this does not qualify the reconstruction
wrapper. See the implementation ledger for every intermediate run and limitation.

`physx_reference.py` is evaluator-only: it bounds rotating body boxes and calls
active overlap/translation sweeps, preserving self filtering and scene identity.
The isolated Isaac adapter fixture passes, but body-envelope clearance alone is
not wheel support or a safe-motion label. `reference_mesh.py` likewise certifies
only explicit continuous height patches, not a full rover/contact model.

`observability.py` exposes a structural retention limit: at 0.10 m/s on flat
ground, even a continuous unoccluded cone needs 9.47/14.59 seconds of prior travel
to observe the current front/rear wheel-center ground with the recorded mount.
Two/five-second windows are therefore diagnostic references, not viable sole
direct-support lifetimes. The bound is optimistic, not a sensing certificate or
permission to extend evidence lifetime silently. Longer-lived support needs
explicit uncertainty/expiry and efficient updates; actual beam/occlusion tests
remain required.

`support_sweep.py` checks every cell intersected by continuous linear/SLERP patch
envelopes, not just wheel centers or trajectory endpoints. The diagnostic assumes
80x120 mm patches at measured wheel XY; these are NOT measured tire contact areas.
Every startup/outside/unknown query stays counted. On the same 320-publication
development prefix, 5 s yields 0 supported queries; 30 s yields about 46% in each
of stationary, 10 cm forward and ±0.1 rad turn families. Startup dominates: full
support first appears after about 17 seconds. Incremental rolling retains those
counts with much lower compute, but those early variants' reference-frame defect
was subsequently found and fixed. The latest fixed-reference, 60 s evidence / 5 s
raw experiment supports **72–73%** across all 748 recorded publications. Startup
and some later extent-support failures remain. None of these figures is independent
safe-motion availability or a 95% qualification result.

Do not use layer 5 (`age`) to certify support freshness: it is the minimum age
among local layers and omits some neighboring plane/corner dependencies. Layer 3
is plane-fit residual, not bounded relative-pose error. A future certified-support
interface must carry dependency-aware timestamps and uncertainty explicitly.

## Controlled ablations and exact preparation

- `--observation-policy coherent` is the default. `frozen_observation` freezes
  each observation's first-used **local** placement while retaining the identical
  latest-available current-rover pose. It is not the historical first-publication
  runtime and does not mix old map coordinates into a newly corrected frame.
- `--cells 60|200` changes only the local rolling extent; `--window 2|5|30`
  changes retention. Use `compare_local_ablation.py` for one-factor comparisons
  with matching publication times, current poses, source hashes and denominators.
- `--backend reference` remains the naive oracle/default. Opt-in `prepared` builds
  the same publication-time pose index once and caches exact range-filtered native
  coordinates. Its additional numerical cache is bounded to 64 MiB/2048 entries,
  pruned with evidence and cleared on fault/close. A cache miss/eviction recomputes
  from the original returns; no downsampling, crop, tolerance-based pose reuse or
  evidence-lifetime change is introduced. The C++ grid still rebuilds each cycle.
  The cache helper can retain exact transformed arrays for isolated experiments,
  but prepared replay does not: the measured active revisions produced zero exact
  world-array cache hits, so storing those arrays only wasted memory.
- Backend comparisons require identical layers, association records and query
counts. Faster replay alone does not pass concurrent/live latency gates.

## Evaluation separation and limitations

`core.py` has no GT terrain inputs. `evaluation.py` contains separate fixed-query
and independent-reference scoring helpers. Unknown references and outside-grid
queries are counted, never discarded to inflate coverage.

The callback replay currently reports **availability**, not terrain truth:
the existing natural capture's sampled first hits, rock AABBs and finite floor
survey do not certify complete wheel support or swept-body free space. Source
failures stay failed. Body-relative class changes include actual robot motion;
they are not a pure flicker measure. The independent query scorer is not a
footprint/swept-volume safety test.

`diagnose_local_motion.py` separately measures 1/2/5 s relative pose errors from
the independently audited GT/GLIM diagnostic, without fitting or interpolation
across absent endpoint samples. Final-revised results remain retrospective.

## Runners

`reference_mesh.HeightMeshReference.height_patch()` provides an evaluator-only
continuous height interval and slope over a convex contact polygon. Explicit
triangles must cover the entire patch without positive-area overlaps; missing
coverage, multilevel geometry, vertical faces and workload overruns remain
unresolved. This is not a wheel/contact-mechanics or swept-volume oracle.

Important source distinction: OmniLRS currently calls `updateTerrainCollider()`
with **meshSimplification**, and can update rendering without rebuilding its
collider. Authored USD/DEM triangles cannot automatically be called active PhysX
truth. Export the cooked collision surface or validate a declared geometric bound
against active scene queries; preserve source/epoch/approximation identity. Do not
change simulator collision settings to make an evaluator pass.

`motion_evaluation.score_motions()` implements the selected >=95% safe-motion
availability gate per scenario family, zero observed false-safe motions, frozen
case IDs, missing-reference/decision accounting and consumption-time validity.
It still reports `acceptance: false`: independently validated geometry, loaded
vehicle limits, visibility/stopping, runtime, stability and recovery are required.
Fixtures validate accounting, not actual robot terrain performance.

See the [maintained runbook](../../docs/implementation/agent-runbook.md#local-terrain-v2).
All capture/replay outputs must go to the external archive. Python fixture tests
may use temporary test data; that exception never permits archived recordings
or large replay outputs to fall back to the root filesystem.
