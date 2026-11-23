# GT-isolated terrain implementation — 2026-09-11

**Historical subsystem record.** For the October 18 handover and current
SLAM/local-terrain/navigation baseline, read the
[current handover](implementation-plan.md#october-18-handover--start-here).
The old coverage gates and libraries below are not the latest operating setup;
they remain evidence, not current qualification or instructions to revert it.

Ongoing execution is tracked in [implementation plan and progress](implementation-plan.md).
September 14: new GT-free rejection diagnostics and 108 sloped/rotated/shifted
face fixtures are implemented. A signed-wall candidate improved recorded coverage
but **introduced a false-free rotated wall sliver**; it and its speed optimization
were rejected. The ordinary library is restored exactly to `69d182ca...`.
**429 Python tests, 11 CTest programs and ASan/UBSan pass** on that restored path.
The fixtures also expose a pre-existing ground-plane fit that can follow a low
hole wall on a slope. Next work must preserve upward-face boundary extent and
improve ground anchoring; wall sign alone cannot safely remove the guard.
All failed candidates and their apparently favorable course results remain in
the ledger. No terrain model or Nav2 installation is promoted.

September 12: the angular candidate's exact early-rejection optimization preserves
all four original native replay maps and the old settled-spin maps; both resource
profiles pass. Original-sweep coverage is still only **93.65–94.59%**, and older
straight/rotation fixtures are much worse. This is not terrain acceptance.
The user's September 13 recovery passes the CUDA context/allocation probe, and
the six-view terminal-turn capture now completes and passes the native audit.
Its four baseline offsets reach **94.23–94.87%**, still below 95%, with sequence
safety retained. A boundary-witness fallback adds no final reference coverage
over its matched parent; its runtime hook is removed and only its test prototype
remains. A completed same-route **5-degree mount** comparison on retained core
`69d182ca...` reaches **94.526–95.397%** across all four offsets: three pass coverage,
but the 2.5 cm offset fails. All 12 objects, full-sequence safety and sampled compute
pass. The tilt helps, but does not establish robust matching. Fixed-quadrant
diagnostics expose gains behind the robot and losses in front; rejection telemetry
and analytic support/edge reproductions are next. See the execution ledger for
all run IDs and unchanged gates; the simulator is stopped.
The old driver mismatch and later CUDA error 999 are historical; the failed
terminal-turn attempt remains preserved. No agent-initiated
host reset/reboot was performed. See the execution ledger for current results.
The declared wide capture `revision-f3274b3cd28145d4922035b8de90d132` has now
completed and passed the native geometry audit; all four offsets failed coverage
(91.64–92.55%) while retaining sequence safety. The angular-support candidate and
separate terminal-turn experiment are tracked in the execution ledger. Earlier
result tables and failed attempts remain historical evidence.

## Current handoff — read this first

The mapper is still experimental and **not installed in Nav2**. **Robust matching
has NOT passed.** The continuous sweep initially reached 96.11%, but the declared
grid-offset stress exposed measured-ground false-free cells on the hole rim.
Those failures count: the original result is not robust acceptance.

The new whole-cell support rule removes positive/hole false-free errors over all
387 maps at **all four grid offsets**, detects **12/12** required objects, and
falsely occupies **0.44–0.72%** of flat reference. Its safe coverage is now
**93.61–94.55%**, below the unchanged 95% target. Work continues on coverage and
runtime. A separate 15-degree-mount capture completed its route but failed the
native geometry gate on one return and remains **unscored/ineligible**.
The same safe core reaches only **79.35% / 84.22%** on the older 5-degree /
10-degree straight acquisitions. The stronger sweep result must not conceal
that acquisition sensitivity; good matching across the tested routes is not achieved.

### September 11 continuation: changes and blockers

- **Implemented computation savings, not a looser classifier.** The connectivity
  stencil retains exact distance, ordering and intermediate-cell checks but
  computes them once per projection. Clearance DDA is skipped only with no
  occupied columns and the sensor inside the rectangular map; outside origins
  still fail closed. Corner caches retain sufficient actual witnesses instead
  of repeatedly scanning identical certificates. Expiry tracks the earliest
  immutable bucket start, avoiding map scans when no bucket can expire. It uses
  the original `now - first_stamp > lifetime` predicate, including floating-point
  boundaries, not a rounded expiry deadline.
- **Verified exact regression across all four offsets.** All 387 per-frame
  cost/reason records and all six final layers match the original extent-fix core
  at each offset. Coverage remains **93.61–94.55%**, 12/12 required objects,
  zero positive/hole false-free throughout. This is a runtime improvement, not
  an accuracy improvement or a newly passed coverage gate.
- **Both fixed resource profiles pass in the latest run.** Dense ground:
  **42.04/46.95 ms p95/p99, 312.2 MiB**; full layer capacity:
  **44.77/51.04 ms, 994.7 MiB**. Native replay remains marginal: the worst variant
  reaches **51.21 ms p95**. Passing synthetic profiles does not clear every
  observed-map, live ROS, or memory-headroom requirement. Intermediate failures
  remain archived, including a run overlapping host system-update compilation.
- **Implemented fixed-denominator coverage diagnosis.** At the worst 2.5 cm
  offset, flat-reference UNKNOWN includes 1.58% unsupported whole-cell extent,
  1.20% clearance, 1.11% negative-edge guards, 1.07% no returns, 0.46% positive-edge
  guards, and 0.24% other uncertainty/area checks. Another 0.72% is false occupied.
  Both variants have identical free coverage on this reference; enabling
  interpolation alone is not a remedy. The diagnostic verifies reference hashes,
  final cost/reason hashes and recorded metrics; it never feeds GT into mapping.
- **Implemented, but did not run, a wider acquisition.** `wide_sweep15` uses
  4.8 m travel (`0 -> +1.2 -> -1.2 -> 0`) along 15 degrees, five views and no final
  rotation. Native capture is mandatory. The longitudinal envelope is +/-1.28 m;
  speed, lateral/heading tracking, per-step footprint/floor, 20 s leg deadline,
  stall and heartbeat guards are retained. The intended mount is the audited
  10-degree configuration. This is explicitly more travel, not a same-budget
  improvement claim. All four offsets and the original reference must be scored
  with one frozen core.
- **Improved future first-hit failure evidence.** New native audits record a bad
  ray's index, endpoint, direction norm and ordered hit details. They do not
  normalize a different ray, choose a later matching hit, widen the 1 mm gate,
  or rehabilitate the failed 15-degree recording.

**Historical blocker (resolved; see the execution ledger): host GPU state.**
On September 11, `nvidia-smi` fails with a driver/library mismatch: loaded kernel
module **580.173.02**, installed NVML **580.178.04**. The simulator remains stopped;
no driver changes, module reload or reboot was attempted. Restore matching host
GPU components before the next Isaac capture. The external drive is available;
the idle replay container's stale bind mount was refreshed without moving
artifacts onto the root disk.

### Implemented in the September 10 continuation

1. **Correct simulator time before collecting denser evidence.** A second
   `PhysicsContext` reset the substep limit after `World` creation. Combined with
   truncated decimal timestep ratios, this made a configured 10 Hz LiDAR scan at
   19.98 Hz relative to the physics/ROS clock. The ROS2 simulation manager now
   resolves integer-compatible physics/rendering steps before construction and
   reapplies them after physics-scene setup. The actual 60 Hz physics integrator
   is preserved; rendering/sensor ticks are 30 Hz. Fresh recordings measured
   **9.995 Hz**, with matching world/timeline/ROS clock deltas. GLIM/navigation
   behavior under the corrected clock has not yet been revalidated.
2. **Audit and replay native sensor ticks during actual motion.** Each recorded
   native chunk contains one tick's XYZ, native angles/ranges, actual GT sensor
   transform, timestamp and hash. The new replay integrates every chunk and
   projects at 10 Hz. In the completed 10-degree capture, 263,552 sampled returns
   agreed with active PhysX surfaces: maximum error **0.057 mm**, zero misses;
   using the preceding tick's pose instead gave centimetric errors and misses.
   GT geometry stays exclusively in the evaluator. This does **not** unlock
   un-deskewed moving ROS full scans or imply that GLIM pose noise is validated.
3. **Separate ground support from known-obstacle clearance.** The rotation
   recording exposed free floor underneath a suspended obstacle. The mapper now
   withholds free status behind known occupied columns. To avoid forgetting
   useful motion, low-column shadows may use actual ground-observation origins
   retained in the immutable evidence buckets. Both the recorded return and cell
   center must avoid all current occupied columns. Geometry at/above the sensor
   cannot use this historical bypass. This raised moving-native coverage from
   87.59% to 91.77%, without extending the 30 s evidence lifetime. It remains a
   conservative known-geometry guard, **not full 3D body-clearance certification**.
4. **Do not mistake a partially seen wall for a measured low obstacle.** A 6.4 cm
   slice of an 8 cm box was briefly classified as drivable roughness. Separate
   raised layers now require a resolved traversable surface normal before they
   can be treated as sub-threshold terrain. The 8 cm hazard threshold is unchanged.
5. **Check geometry across height layers too.** Two LiDAR rings on a vertical
   box corner can individually resemble shallow ground planes. Their combined
   covariance reveals the upright face. The new joint-layer check removed the
   remaining positive false-free errors in the rotation regression. Rotation
   still gives only **57.63%** final free coverage; it is not the preferred
   acquisition route or an implemented navigation recovery policy.
6. **Tighten evaluation and preserve failure evidence.** Positive-obstacle safety
   is now gated over the whole sequence, not only the final map. Native replays
   save snapshots whenever a safety error reaches a new maximum. No failed run,
   required object, reference cell, or old recording is discarded to improve a
   score. The early doubled-cadence recording is ineligible for moving replay.
7. **Protect partially observed positive-object boundaries.** The 5-degree
   capture exposed a floor-only cell straddling a small box while its adjacent
   face remained unresolved. A nonrecursive one-cell UNKNOWN guard around
   unresolved raised evidence removed the transient eight-pixel false-free
   error. This is not occupied inflation and does not change the height threshold.
   The corrected 5-degree straight replay reached **88.83%**, with all sequence
   safety gates passing, versus 89.08% with that unsafe boundary bug.
8. **Use bounded, compact evidence storage and exact computation savings.**
   Symmetric double-precision covariance, a three-point support certificate,
   shared exact acquisition origins and capped bucket capacity reduced each
   bucket from 280 to 192 bytes. The full-grid stress fell from **1467 to 994 MiB**.
   Independent-cell projection uses at most four OpenMP workers, with an immutable
   occupancy snapshot to avoid clearance read/write races. Plane fitting stops
   only when its inlier set is exactly unchanged. The fixed stress now measures
   **49.16 / 62.77 ms p95/p99**, narrowly within the 50/100 ms budget. An independent
   repeat measured **49.47 / 57.99 ms**, 994.3 MiB. These are controlled process
   benchmarks, not a live ROS resource guarantee.
9. **Test acquisition diversity separately from mapper changes.** The centered
   route `0 -> +1 -> -1 -> 0` keeps the original 4 m travel and 41 views. It
   achieved 92.76%, but remaining flat-reference cells include 2.33% without
   returns and 2.59% withheld by known-obstacle clearance. A separately declared
   43-view centered route at a 15-degree heading reached **93.98%**, 12/12,
   zero sequence positive/hole errors over 789 maps, and 43.73/48.46 ms p95/p99.
   No reference, lifetime, inference bound or safety gate
   is relaxed; these are development experiments, not held-out results.
10. **Remove collector-induced evidence expiry from the next acquisition test.**
    The 43 stops stretch 4 m of travel to about 79 s. Important positive-X
    viewpoints expire before the return from negative X, despite integrating
    every native tick. The separately declared `centered_sweep15` experiment
    keeps the same path, maximum 0.15 m/s speed and 30 s evidence window, with
    only six endpoint/rotation views. Native capture is mandatory; every moving
    chunk and every 10 Hz projected map remain in the audit/evaluation. A 20 s
    deadline accommodates its 2 m leg; per-step footprint, support, tracking,
    stall and heartbeat checks are unchanged. The capture completed in **38 s**;
    its positive endpoint was only 25.6 s old at the final map. It reached
    **96.02% measured / 96.11% inferred**, with all 12 hazards detected and no
    sequence positive/hole false-free cells. It used fewer native chunks (1,158
    versus 2,365), not more total returns. This tests useful evidence density
    during ordinary continuous motion, not an extended evidence lifetime.
11. **Require support across the whole output cell, not only an interior patch.**
    The declared grid shifts exposed a fundamental area-support error: cells
    containing only a small floor strip could straddle the unsupported hole and
    still be marked measured ground. Every free cell now needs support around
    all four corners in eight angular sectors. Actual retained returns or
    bounded observed XY subregions must fit inside the unchanged 30 cm support
    span and agree in height within 2 cm. A four-quadrant prototype failed a
    rotated-hole fixture; eight sectors fixed it. These are bounded local
    support assumptions, **not proof against all unobserved sub-30 cm voids**.
    Compact 5x5 XY-presence and exact corner-sector metadata fit the bucket's
    existing padding; they are not height voxels or occupancy labels. They expire
    and rebuild with their original buckets. The raw XYZ, continuous heights,
    8 cm hazard threshold and 10 cm output cells are unchanged.
12. **Test the new failure mechanism and bound its computation.** Eight analytic
    hole/rim fixtures cover four grid phases and axis-aligned/rotated holes
    without wall returns. All eight fail on the pre-fix core and pass after the
    extent fix. The old center-only gap fixture is now correctly unknown; a
    separate densely bracketed collinear-return fixture still proves that
    supported bounded inference works. Added expiry equivalence verifies old
    spatial certificates do not survive their buckets. Plane-agreement caching
    matches all 387 original per-frame records/final layers on the failing
    2.5 cm recording. Repeated XY-region work is now deduplicated without changing
    first-visit order. Only corners of potentially free cells need evaluation.

### Why the ROS scan-end pose is still not a per-return pose

The first 5-degree capture stopped at a platform-edge return. Its exact native
XYZ was found in a chunk acquired 66.7 ms before the ROS scan stamp. At that
native pose the point lies on the edge; applying the later pose shifts it by
approximately `(4, -30, -8)` micrometres, enough for a grazing scene-query ray to
miss the finite floor. Even a settled skid-steer robot is not mathematically
motionless.

With native recording enabled, the viewpoint auditor now matches selected ROS
returns to their **exact native XYZ provenance**. When repeated XYZ has multiple
possible acquisition ticks, it audits **every candidate**, never fits or chooses
the most favorable pose. All candidates must pass the stricter 1 mm maximum
surface-error check and zero surface misses. Published extrinsic TF checks remain
mandatory. The raw ROS scan is preserved **unchanged and un-deskewed**, the audit
does not claim a unique timestamp for ambiguous returns, and the ordinary
single-pose scorer explicitly rejects this audit mode. A fresh 5-degree capture
completed all 41 views under this contract; its native timing audit also passed.

### Verification and current artifacts

Latest full-suite verification: **291 Python tests, 7 mapper CTest programs, and 4 GLIM ROS
CTest programs passed**. The current frozen C++ terrain core passed ASan/UBSan
(not a thread-race sanitizer). New contracts cover exact expiry boundaries,
same-stamp reintegration after empty projection, outside-origin clearance,
coverage attribution and native first-hit outlier retention. Other contracts include partial faces, cross-layer box corners, retained-view expiry,
unobserved-view rejection, exact/ambiguous native association, failed-input
preservation, and full-sequence positive/dropoff gates.

All `revision-*` paths below are on the external archive (Docker `/archive`).
The old installed ROS overlay is **not** the tested experimental library.

- Fixed-clock, 10-degree native capture: `revision-7197c26bdf5a4e58917f5d38efd26bf1`.
- Initial native replay with current-view-only guard: `revision-b230345729eb4a2880d7dc0bf7adcab9`
  (87.63%, transient two-pixel positive error; retained as a failure).
- Partial-face fix: `revision-67d14b03db8747b9b7c7d3cc20abadfd` (87.59%, sequence safe).
- Retained-view replay: `revision-28511a6d06504a82812dc9c9c6a42ac1` (91.77%, sequence safe;
  47.55 / 49.50 ms p95/p99, 293.96 MiB for the two-variant process, Isaac stopped).
- Earlier frozen library, including joint-layer normal:
  `revision-330aa5caa5b848c695014e90d26475c8/liblayered_terrain.so`, SHA-256
  `908077eb689561eda0a3b9eb8fa23b798a32633bbda1440a44248b0d34163f4a`.
- Latest rotation regression: `revision-e5b0abb2189947a8804309037929f196`
  (57.63%, 12/12, zero positive/hole false-free over all 50 maps).
  Current frozen-core rotation replay `revision-7ff55237a083464b8f57680ae6477c10`
  scores 57.36%, 12/12, zero positive/hole false-free over all 50 maps.
- Doubled-cadence native recording: `revision-c932d87a75b24b3cb2806a342b9dea6a` (ineligible).
- Failed 5-degree attempts: `revision-cc464f02ee2d4911949665df8234dc7e`
  (scan-end pose audit), `revision-b03494b779a740c89d9088b4e4013710`
  (initial unique-match assumption). Neither is resumed or scored.
- Completed 5-degree native capture: `revision-8a2b6fbc2e2444e29bf8b7e577aea898`.
- Corrected 5-degree score: `revision-d6a040d18dd8415484a5cc9b05edba6b`;
  packed-storage replay `revision-983169834dee491e9a1de41afe659183` matches all
  687 per-frame records and all six final layers exactly. Latest frozen-core
  replay `revision-e4d6e3e3640447f9a7c6b163a7a2bc61` also matches exactly.
- Same-current-core 10-degree straight replay:
  `revision-886b76ad962542dea1cf3c84d3ec4095` (**91.52%**, 12/12,
  zero positive/hole false-free throughout). The centered route therefore adds
  1.24 percentage points with the same current core, not 0.99 points versus the
  older 91.77% version that lacked the partial-boundary guard.
- Centered capture: `revision-4223961468ca46e4bc8ea5a99f566685`; packed-serial
  score `revision-c798a161dd4d433f92f7cd48fde34d17` (92.76%, sequence safe).
  Parallel replay `revision-214b38a2cf7040eab2b5f5baa4ab222f` matches exactly.
- Latest frozen library (packed, bounded parallel, exact fit convergence):
  `revision-bf6b32c9b47341f5b727d14288b79310/liblayered_terrain.so`, SHA-256
  `acce0ac618bb860070723295a15aab099a3f632c6a2ef52f147eeff9d48122fe`.
  Its centered regression `revision-ac8cd40af066492986daf063ba71b1ea` completed
  at the same 92.76%, with zero sequence false-free cells, and matched the
  packed-serial baseline exactly over all 687 maps and final six-layer arrays.
- Angled centered capture: `revision-bca4ef862b2f448dbb4c25884eff27f2`;
  score `revision-ebcfd91e47204f6fb7fa10f340fa1b5f` (93.98%, sequence safe).
  2,365 native chunks, 9.995 Hz, 302,720 sampled surface returns, maximum
  0.120 mm error, zero misses. Diagnostic figure is in that score directory.
- Continuous centered sweep: `revision-a0c4486b05854587a2b73a0e2c2bf4f2`;
  default-grid score `revision-89fdbe622ae64abaa555dc447225b442`.
  1,158 chunks, 9.995 Hz, 148,224 audited surface returns, maximum 0.079 mm
  error, zero misses. Measured/inferred p95 **45.66/47.92 ms**, p99
  **50.52/51.64 ms**, two-mapper process RSS **231.0 MiB**, Isaac stopped.
  `terrain_comparison.png` in the score directory shows the fixed reference and
  both final maps. All development geometry and sampled-compute gates passed.
- Full-grid resource failures remain: `revision-a6096b8cce65422eb913acc6acbf6e30`
  (1467 MiB, 128 ms p95), `revision-0e13dbc4305f4914973518361173dd80`
  (994 MiB, 124 ms), `revision-db2c39e66ef84dcb97041b996d078784`
  (994.6 MiB, 61.2 ms). Latest passing stress:
  `revision-0956ed0e8963490da8b5bbd75f84ff6c` (994.1 MiB, 49.16/62.77 ms).
  Independent repeat: `revision-5e988ed06dc4489aab52481e36f85942`
  (994.3 MiB, 49.47/57.99 ms). The stress final six-layer array also exactly
  matches the packed-serial baseline.

### Whole-cell extent correction artifacts (September 10 stage)

- Unsafe grid-phase replays, all preserved: 2.5 cm
  `revision-4b9fcf91363c497693873e44342deb0d`, 5 cm
  `revision-796dc3cb92444046921c8641f541cefc`, 7.5 cm
  `revision-3af2cef494f944f1b04c2c58226bbcaa`.
- Extent-fix frozen core: `revision-f8431642f16740a9bafded577d50f7c2`, library SHA
  `f88afc543fab7261f4afe2fd8250f6f0fc4531ea8c1c3e6755a0f51b26a7dab3`.
  On the same 10-degree sweep, all four offsets have 12/12 recall and zero
  sequence positive/hole false-free cells:

  | XY offset | Safe flat coverage | Score run |
  | --- | --- | --- |
  | 0 cm | 94.47% | `revision-77857e387d924e549f1369ae8f171249` |
  | 2.5 cm | 93.61% | `revision-d807b7ef19e0419393f3c662bcbdfc56` |
  | 5 cm | 94.02% | `revision-5c37fa1eb75e43cba581ae61a22a0648` |
  | 7.5 cm | 94.55% | `revision-542da973345244ec8218d2a93ef0887b` |

- Plane-cache core: `revision-87fab0a14d224e3f9ed7f284c2254cd9`, SHA
  `412e5a7ce2fe751487eee101255fe2bc71980cf6c5ee2140f606c2535f8c8df9`;
  exact 2.5 cm regression `revision-84c210827a1f4f16bf52ce87e92205fb`.
- Latest deduplicated core: `revision-ccfc05c649af4d5091dd2de0f6745646`, SHA
  `5477be264859a2b464416a06632e00c4dfc4430c8278f486a1eba61cae24b272`.
  Full-grid layer-capacity stress `revision-b9a2da8b38444b33acadaaea50b45a11`
  passes at 994.4 MiB, 49.11/52.90 ms p95/p99. The newly added **dense-flat**
  resource profile `revision-ed805deecb55423a938d006e46194c1c` fails compute:
  312.1 MiB, **72.08/78.45 ms**. Both profiles count; the occupied-grid pass
  alone does not establish the ground-support runtime budget.
- 15-degree mount / same continuous route: `revision-1007841f3dae488a9c0f5c1f6df27447`.
  All six views completed, 9.995 Hz cadence, but native chunk 672 has one
  185.25 mm first-hit discrepancy among 148,224 sampled returns. The reconstructed
  earlier hit lies approximately 10 micrometres outside a box corner, while the
  recorded endpoint lies on the floor. This suggests grazing-ray numerical
  ambiguity, **not yet a proven resolution**. The replay correctly rejects it:
  `revision-96152a2a6def4812ae4a8dbd73a7c745`. No return or frame was removed and
  no tolerance was widened to score it.

### September 11 frozen core and results

Current library: `revision-756f0f4292784c20a55244f0ca017402/liblayered_terrain.so`, SHA-256
`9161265ae5fba5d1e95d8e97d68e1042f73d5a50ed44d22dd3d9f74d072dfccb`.
The installed ROS overlay is still not this library. All four new sweep runs
match their September 10 extent-fix counterparts exactly, including 387 per-frame
records per offset and all six final layers. Every run has 12/12 recall and
zero sequence positive/hole false-free cells. Measured/inferred p95 values below
include integration plus projection, not just the C++ projection call.

| XY offset | Flat coverage | p95 measured/inferred (ms) | New score run |
| --- | --- | --- | --- |
| 0 cm | 94.47% | 49.91 / 48.78 | `revision-7acd3bff67c84870bd62f41b8ab91f04` |
| 2.5 cm | 93.61% | 50.16 / 51.11 | `revision-5fbbb0266a784d4bba22a977f0f92094` |
| 5 cm | 94.02% | 48.75 / 51.21 | `revision-97f6a291185541a9bbda1c0066ea9346` |
| 7.5 cm | 94.55% | 46.43 / 48.19 | `revision-e794af9b65aa4d58b4fcbd669739e393` |

The worst-offset score directory contains `coverage_diagnostics.json` and
`terrain_comparison.png`; the figure has been visually inspected. It retains the
original reference and makes no hidden flat-space fill. Earlier-cache exact
comparisons also passed: `revision-6a725a3f4ad64f8ba4641c5ad8d3e98c`,
`revision-9a1bca0108df486883acc9d8561338d8`, and
`revision-82d2e833db0445f5b844fae5865dc55c`.

Resource runs on the current core:

- Dense flat `revision-0a40b79fd74244f9ad9cda5cb9736e72`:
  312.23 MiB, 42.04/46.95 ms p95/p99, both resource gates pass.
- Layer capacity `revision-dd2a32ac675a4301b07d735c02d10b5d`:
  994.68 MiB, 44.77/51.04 ms, both resource gates pass.
- Both final six-layer stress outputs exactly match the previous core's outputs.
  The process is still close to the 1 GiB memory budget at full layer capacity.
- Intermediate stencil/clearance core `revision-f5bc593821a240be84a8dc6d671bf694`
  (SHA `323fb5359152702d3820facb0c5d075173a18ca7985fea78260d1c0eb0e12ca1`):
  dense `revision-55e8c9ad4b01444686fcdbf891c3ff46` failed at 52.94 ms p95;
  capacity `revision-3b601820fa5a40289719fe6c7baaa68e` passed at 48.51 ms.
- Intermediate witness-cache core `revision-b65f03d6afa04925836d5a1a8dc57626`
  (SHA `0a8fadd287761bcaf3b8e75997d31fec8b42c5c68b4282f7be7f783b533a284b`):
  dense `revision-9b1b267df7bd48d99c6344f13a4e450b` failed at 51.40 ms;
  capacity `revision-541515f3d56240f78e98f8df72ffb12a` failed at 57.42 ms.
  Host update compilation was observed during this run series. No failure was
  overwritten, and no best-repeat selection establishes robust timing.

Older acquisition regressions on the current core, canonical grid:

- 5-degree straight: `revision-2cce318dfd594e65a4649bdb816a37ec`,
  **79.328% measured / 79.346% inferred**, versus 88.83% before the extent guard.
- 10-degree straight: `revision-b80449e4c2a942abb0fd4d1f6c11d3c9`,
  **84.200% measured / 84.218% inferred**, versus 91.52% before the extent guard.
- Both have 12/12 recall, 0.50% false occupied, zero positive/hole false-free over
  all 687 output maps each, and pass sampled compute. Their coverage failures
  remain part of the evidence. This is the price of exposing unsupported partial
  cells, not a regression introduced by the exact runtime optimizations.

### Resume sequence and remaining gates

1. Restore matching host NVIDIA kernel/userspace components and confirm
   `nvidia-smi` succeeds. Do not implicitly reboot or alter drivers. Verify the
   external archive and refresh stale idle container binds after reconnects.
2. With no navigation/other command publisher, collect the declared
   `wide_sweep15` native experiment with the 10-degree mount. Use the existing
   guarded capture launch and original reference. The 4.8 m route is new
   development data with a larger dose, not an equal-budget claim. It may fail;
   keep every attempted capture, and do not repeatedly rerun until one passes.
3. Audit cadence, all selected current-pose first hits, exact ROS/native
   associations and published extrinsic TF before scoring. Investigate any
   grazing-ray discrepancy using the new hit metadata; never replace a failed
   first hit with a convenient later floor hit. The earlier 15-degree recording
   remains ineligible.
4. Use the same frozen core for all four **0/2.5/5/7.5 cm** grid offsets, both
   mapper variants, every native input and every 10 Hz safety output. Retain the
   **138,725-cell** reference, all 12 required objects, 95% coverage, zero sequence
   false-free, 30 s evidence lifetime and 30 cm support span. Run reason
   attribution before proposing any classifier change. Changes must also replay
   the older 5/10-degree and rotation regressions; the existing offsets are
   development/regression cases, not untouched holdouts.
5. If wider views still fail, compare acquisition geometry and the conservative
   partial-cell/clearance model explicitly. Do not manufacture freespace through
   GT fill, longer retention, smaller denominators or an unbounded interpolation
   span. An alternative support/clearance representation needs its own corner,
   thin-object, hole and sequence-safety fixtures before replacing this core.
6. Remove the remaining native p95 overrun with measured, semantics-preserving
   work, then repeat both resource profiles and live-cadence checks under recorded
   CPU load. Synthetic passes do not prove live ROS latency or sufficient memory
   headroom. The minimum next compute target is margin below 50 ms on **every**
   native alignment case, not a rounded 50.16 ms pass.
7. Freeze a development candidate before untouched physical alignments, angular
   phases, slopes/noise, natural terrain, rectangular-footprint corridors,
   SLAM-pose replay and live revision/Nav2 installation. Moving full ROS scans
   remain un-deskewed and ineligible. Unknown overhead extent, unobserved dropoffs
   and uncertain support anchors remain fundamental limitations.

## Historical stop-scan handoff — 2026-09-09

The following records earlier stages. Its “current/latest” labels, performance
numbers and in-progress notes describe that date, **not the current handoff above**.

**Implemented and tested as an offline experimental mapper, not promoted to
Nav2. Good drivable-surface matching has NOT yet been achieved.** The 95% free
coverage target and negative-obstacle safety gate remain unchanged. The material
result is a calibrated simulator fixture, a physically driven multi-view capture,
and reproduced/fixed evidence-storage and ground-support bugs—not a finished
navigation system.

### What is now implemented

- The analytic Isaac scene contains 15 boxes (5/8/10/14/20 cm height crossed with
  5/10/20 cm width), a real missing-floor hole, ramp, raised plateau and overhang.
  Active PhysX queries check the known dimensions and swept-route support.
  A 2 cm collision-reference raster and continuous-FOV visibility mask are
  independent of mapper-known cells and actual laser hit locations.
- The opt-in GT driver completed a real 2 m straight out-and-back route, stopping
  every 10 cm: **41 views, 82 settled scans, 65.72 simulated seconds**. It returned
  within approximately 6 mm in X and 3 mm in Y. Another 1,323 continuous raw scans
  and physics-step poses were retained, but **moving scans are not scored**.
  The driver checks sole command ownership, tracking, inflated footprint
  overlap, floor support, timeout/stall and an independent wall-time watchdog.
  There is still no general terrain driver or explicit contact-reporting gate.
- The actual PhysX sensor emits thousands of **minimum-range-clamped returns**
  (about 6,722 per 28k-point scan). They are not terrain surfaces. The scorer keeps
  raw and valid-range-only ablations; it does not use GT to remove points. Valid
  surface range agreement was about **0.035 mm p95**, without fitting a transform.
- Ground is continuous-height geometric evidence in **10 cm XY cells**, not
  10 cm height occupancy voxels. Up to three ground-eligible height clusters and
  a fourth conservative overflow cluster retain extrema, spatial/view support,
  and point covariance. GT terrain labels never enter this core.
- The scorer hashes raw scans, audit associations, library and evaluator;
  preserves each attempt; reports measured-only, bounded inference, raw/valid
  range and single-scan ablations. It now reports p95/p99 timing, process RSS,
  per-frame cost/reason hashes and **worst dropoff false-free count over the
  entire sequence**, not just the final frame.

### Bugs found and fixes

1. **Recent observations were lost when an old window expired.** Evidence now
   uses immutable, at-most-one-second buckets within the 30-second lifetime.
   Recent ground survives old-obstacle expiry; old extrema cannot be refreshed
   merely by later observations. Covariance/extrema are rebuilt from live buckets.
2. **Layer overflow hid known obstacles.** Overflow evidence can still establish
   a positive hazard, but can never become a ground layer. This restored the
   two missing 5 cm-wide, 14/20 cm-high objects in the driven fixture.
3. **Projection could rewind time after destructively expiring evidence.** A
   monotonic evidence-time watermark now rejects both scan and projection rewind.
4. **Shallow vertical hole-wall returns looked like ground by height alone.**
   Resolved steep surface normals and negative residuals create an unknown edge
   guard. Measured ground additionally needs two-dimensional XY support; a line
   of rim hits alone does not certify a ground patch. Single-ring obstacle
   detection remains supported. These changes deliberately reduce free coverage.
5. **One interpolation chord could cross a hole corner.** The newest version
   requires two nonparallel measured-ground brackets, each within the unchanged
   30 cm bound, with consistent height. It never chains inferred endpoints.
   The recorded straight-route replay now has zero free designated-hole cells
   in all 82 frames, for both measured and inferred variants. Two replay runs
   match exactly, including all per-frame records and five final layered arrays.
6. The optional 15-degree diagonal acquisition exposed a driver branch error:
   about 1 cm of skid-turn translation prematurely selected the translation
   controller. The corrected branch retains bounded origin rotation and all
   swept-clearance checks. Archive-loss cleanup now sends zero before attempting
   log writes, destroys the ROS driver even when the log cannot be written, and
   reports capture failure to stderr if the archive is unavailable.

### Results — same straight-route development capture

These are **development results, not independent acceptance**. The visible flat
denominator remains the fixed initial-view reference (138,725 fine cells). Every
one of the 12 required objects remains in the denominator, including zero-hit
objects. No GT mask dilation, lifetime extension or threshold relaxation was used.

| Variant/version | Required objects detected | Flat free | False occupied flat | Final hole cells free | Worst hole cells free over sequence |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original multi-view, valid-range inference | 10/12 | 82.19% | 0.247% | 300 | Not summarized |
| Bucket + overflow fixes, valid-range inference | 12/12 | 90.66% | 0.500% | 300 | Not summarized |
| Area/normal guards, measured-only | 12/12 | 75.49% | 0.500% | 0 | 0 |
| Area/normal guards, one-bracket inference | 12/12 | 82.27% | 0.500% | 0 | 25 |
| Latest two-bracket inference | 12/12 | 80.59% | 0.500% | 0 | 0 |

### Acquisition experiments — unchanged mapper and original reference

The completed diagonal experiment did **not** improve coverage. The 10-degree
mount experiment did improve it materially, but has not passed the 95% gate:

| Acquisition | Views / settled scans | Measured flat free | With bounded inference | Flat false occupied | Worst free hole cells |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original straight, level sensor | 41 / 82 | 75.49% | 80.59% | 0.500% | 0 |
| Diagonal 15-degree route, level sensor | 43 / 86 | 72.12% | 78.34% | 0.500% | 0 |
| Original straight, 10-degree sensor pitch | 41 / 82 | 87.94% | **90.25%** | 0.500% | 0 |

All three detect all 12 required objects in the final map. These rows use the
same frozen library and the original 138,725-cell visible-flat denominator.
The diagonal run peaks at 81.14% before its final turn; evidence expiry during
that turn lowers the final result, but is not the entire coverage deficit.
The tilted sensor's separate initial visibility reference gives 91.14%; that
number does **not** replace the fixed-reference 90.25% comparison.

The mount change is opt-in and live-stage-only: it rotates the `vlp16` parent
frame, leaving the default robot asset unchanged. The initial audit and every
viewpoint verify the published `base_link -> vlp16` transform against actual USD
geometry (within 0.1 s and 1e-5 matrix difference). The scorer refuses a tilted
capture without this audit and a pre-existing identical-geometry reference.
Figures also use the scored reference and verify its hashes.

Tilted straight replay measured 48.62 ms p95 / 59.12 ms p99 for valid-range
inference and 390.5 MiB whole-process peak RSS. This passes sampled offline
compute gates, not the live/dense-memory gates. The next declared experiment is
`spin360` with the same 10-degree pitch: 25 views at 15-degree yaw increments,
zero commanded translation, 25 cm maximum drift and a conservative full-circle
inflated-footprint overlap/floor check covering the complete drift envelope
about the original pose. The initial 4 cm-limit attempt stopped on actual skid
translation at 17.7 degrees of yaw. Inspection confirmed centered wheel geometry
but an off-center base COM (X=-8.748 cm); this does not justify assuming rotation
has zero translation. The new bound is checked geometrically, not fitted into
the point-cloud transforms. It tests angular sampling, not a new
navigation recovery behavior. The evidence lifetime remains 30 seconds.

The higher 90.66% number is unsafe and is **not** the current success claim.
Normal guards alone removed the final hole failures but still had transient
failures; inspecting every frame exposed those. The current measured-support
rule addresses them conservatively. Many residual unknown cells are laser-ring
gaps or inadequately sampled patches, not SLAM pose failures. A diagonal route
is a separate acquisition experiment to add lateral viewpoint coverage, not a
change to the evaluator's denominator.

CPU results depend on contention: the bucket/negative-residual version took
about 35 ms p95 and 36 ms p99 for valid-range inference with Isaac stopped. The
area/normal version measured about 55 ms p95 / 62 ms p99 with Isaac running;
whole scorer peak RSS was about 353 MiB, including five mapper instances and
Python. After reconnecting the archive, two-bracket inference measured
38.89 / 42.67 ms p95 and 46.69 / 47.52 ms p99 in two replays, with about 354 MiB
whole-process peak RSS. **These sampled offline runs meet the timing limits,
but dense worst-case and live-cadence gates are not established.** These are
offline integrate+project timings, not live sensor-to-costmap latency or 10 Hz
publisher validation.

Latest full local verification: **195 Python tests**, **7 mapper CTest programs** and
**4 GLIM ROS CTest programs** passed. The standalone layered-terrain contracts
also passed AddressSanitizer + UndefinedBehaviorSanitizer. New tests cover the
skid-turn branch, archive-loss cleanup/no fallback, minimum-clamped versus valid
surface audit misses, prescribed-route completeness, shallow hole walls,
line-only ground evidence and one-chord corner inference. The tested current
classification baseline used for the acquisition comparison has SHA-256
`3096cefbcd5ad0d7532495facd41578a5954dc15bd8c7927a416820598f3e964`.
The build at `/tmp/repair-build/slam_mapping/liblayered_terrain.so` in
`astrobot-repair-dev` now additionally replaces per-cell plane-fit temporary
vectors with fixed 81-entry stack arrays, removing heap allocations without
changing neighborhood selection or math. Its SHA-256 is
`1a97afa7515f5354115d3cb5e2c9db815739669ed37e95443ee000a6d96d0466`.
The optimized tilted-straight replay matches all 82 baseline frame records and
all five final layer arrays exactly. With Isaac running, p95 was 56.64 ms versus
60.92 ms in the preceding baseline replay: an encouraging sample, **not** a
controlled speedup estimate or a pass of the 50 ms contention budget.
The old installed/frozen ROS overlay must not be mistaken for either library.
The new 12 shifted/rotated, 20/60 cm-wide analytic hole fixtures pass through
bucket expiry; these assume observed walls and do not validate invisible edges.

Dense worst-case memory stress remains pending: bounded bucket count is not by
itself proof of the 1 GiB gate for a grid full of multi-layer returns. Likewise,
bounded interpolation cannot distinguish every narrow real hole from a sampling
gap. The designated calibration-hole result does not establish universal
negative-obstacle safety or calibrated uncertainty under SLAM pose noise.

### Where the evidence is

All paths below are relative to the external archive, normally mounted at
`/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab` (Docker
`/archive`). Straight-capture integrity was checked after reconnecting the drive;
subsequent experiments are listed separately below.

- Straight physical capture: `revision-8b1edba87feb4c60a637249a9b7852d7`.
- Original multi-view score: `revision-b7c79229e04944d0acc666b6542ed768`.
- Bucket/overflow score: `revision-d179c11d5b1f44da93714763303c66b7`.
- Negative-mean, trough, normal and groundlike-normal iterations:
  `revision-4b792cf328ed4652a6a730110aefe21a`,
  `revision-fef6a393663d473fa780dcb994ca1261`,
  `revision-49638c6f6aeb43ddaf592bf51570d70e`,
  `revision-f2b7cb7d9e3a444ea9dbfd358d679945`.
- Area/normal score: `revision-ec06e7f514d0482896a40101b4f3a81e`;
  frozen library: `revision-9ce09a012de54aa9bde043ddc8109973/liblayered_terrain.so`.
- Latest two-bracket frozen library:
  `revision-5689398812cb45daa595dfe9d7ccefac/liblayered_terrain.so`.
  Replays: `revision-d7be742df2914adb943b0a8368deb65c` and
  `revision-675464d372e54e3ca01ef153a40d70fb`. All 132 input hashes were checked
  after reconnecting the drive; all 82 per-frame records and final arrays match.
- Failed diagonal captures: `revision-cee8ea14efa14f038bec31ca7573ef99`
  (rotation/translation branch), `revision-91b84e364011499b9388587f0a563a0b`
  (11 missing audit rays despite approximately 0.019 mm surface p95).
  Neither is eligible for scoring. Diagnostic rerun
  `revision-aacac8245b3a4783ad511ccd8ac10de6` established that all 11 misses were
  valid surface rays on the **back face of the ramp**, not min-clamped returns.
  The sensor uses `eMESH_BOTH_SIDES`, while the Python scene query defaults to
  front faces only. The audit now explicitly matches the sensor flag; the fresh
  43-view diagonal capture passed all surface audits. No pose fitting or range tolerance change was
  used. The old frontface visibility mask is retained as the primary comparison
  target; a second independent `visible_sensor` mask includes both sides and is
  scored separately. Thus the audit correction cannot silently improve the
  reported score by shrinking the old denominator.
- Successful diagonal capture: `revision-bcf3f5a1a1e3459ca48ce725d4ad5174`;
  score: `revision-22eb0393a2d24aa088170be2dc9d33cf`.
- Ideal-plane pitch sampling screen: `revision-c492ebf009e84a29b640ab6d94f04d1d`.
  This ignores occlusion and is not mapper accuracy or acceptance.
- Successful 10-degree tilted straight capture:
  `revision-dcfc046bc93f40dab4aa208e93b42dfb`;
  score and comparison figure: `revision-dc529492433d4ce68163c85f2fafd82c`.
  Baseline repeat: `revision-0bca641f73ac4b9d87e24ed50fc1eeea` (all frame records
  and final arrays identical). Optimized score:
  `revision-2e04c6c5ff81409db766d5f38ac2ef51`; optimized frozen library:
  `revision-6f9276e7e83d4dd2827b4fb670090579/liblayered_terrain.so`.
- Failed initial 10-degree tilted rotation capture:
  `revision-ea2ad6fbc1e948b890ee0dce22bd86ef` (4 cm drift stop after two views;
  ineligible for scoring). A fresh capture uses the full 25 cm drift-envelope
  clearance check; no failed run is resumed or overwritten.
- Fresh rotation capture: `revision-f880a2ace0c64b3c8d770702387457dd`
  (in progress; score only after the complete 25-view audit passes).

The native behavior is explicit in
[Isaac 5.1 LiDAR source](https://raw.githubusercontent.com/isaac-sim/IsaacSim/v5.1.0/source/extensions/isaacsim.sensors.physx/plugins/isaacsim.sensors.physx/lidar/LidarSensor.h);
[PhysX geometry-query documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/GeometryQueries.html)
explains mesh backface handling. The calibration ramp is still a thin mesh,
not a contact-validated solid driving ramp; changing that geometry should be a
separate fixture version, not an unreported scoring change.

Raw flat-ground returns also establish a sampling limitation: with the sensor
about 0.395 m above this platform, rings land at approximately 3.148, 3.854 and
4.963 m horizontal radius. The outer **1.109 m ring gap** cannot be solved merely
by choosing 10 cm map cells. Lateral motion or an independently evaluated mount
tilt may be necessary; changing the classifier alone cannot create observations.

### Immediate continuation — archive restored

The external drive became unavailable during the latest interruption: allocation
returned `EIO`, the host mount disappeared, and `lsblk` showed only the internal
NVMe disk. The simulator container was also no longer present. No root-disk
fallback, mount repair or deletion was attempted. The user has since reconnected
the drive; `/dev/sda1` is mounted at the original path with about 427 GiB free.
The replay container was restarted to refresh its bind mounts. The failed
allocation was not reused, and the newest successful artifacts are listed above.

Next:

1. Archive integrity and repeat replay checks are complete for the straight
   capture. Keep the old installed overlay distinct from the frozen tested core.
2. The two-bracket fix passes the recorded hole sequence, but even the tilted
   straight-route 90.25% coverage still fails the 95% gate. Do not promote or cite the unsafe 90.66% version as
   the current result.
3. The `diagonal15` and tilted straight experiments are complete. Finish the
   declared `spin360` acquisition experiment with the same initial reference;
   valid-surface misses still fail the gate. Keep failed captures. Do not score
   continuous moving scans or silently adjust timing/poses.
4. Compare viewpoint coverage against the straight route and attribute remaining
   unknowns. If adequate views still cannot provide 95% safe coverage, evaluate
   sensor mounting/acquisition and a stronger surface-support model, rather than
   increasing gap fill or changing denominators to pass.
5. Once development gates pass, freeze code/evaluator and collect untouched
   grid-phase, noise/slope and natural-Lunaryard validation scenes. Validate the
   rectangular-footprint corridor, then SLAM-pose replay and live revision/Nav2
   installation. Current synthetic and GT stop-scan results do not substitute
   for those gates.

## Historical first-stage report — 2026-09-08

The following section records the earlier stationary-only stage. Its completion
counts and limitations describe that stage, not the current implementation above.

## What actually changed

There is now a working **raw XYZ → layered terrain → 2D cost** CPU implementation,
an offline GT replay path, regression fixtures, independent scoring contracts,
and a live Isaac collision-reference capture. The new mapper has been exercised
on recorded raw LiDAR. It is **not installed into Nav2** and the overall terrain
acceptance plan is **not complete**. No movement commands were sent in this work.

The current priority supersedes the ordering of the earlier revision-foundation
document: isolate terrain with GT now, while global/submap optimization remains
disabled. A live revision-aware consumer is still required before promoting this
mapper to a correction-capable navigation stack; it does not block offline work.

### Implemented components

| Component | Implementation | Boundary |
| --- | --- | --- |
| Layered CPU mapper | `slam_mapping/{include/slam_mapping,src}/layered_terrain.*` | Separate experimental core; legacy mapper unchanged |
| NumPy adapter | `tools/layered_terrain.py`, C ABI `layered_terrain_c.cpp` | No ROS graph or GT terrain input |
| Raw GT replay | `tools/replay_raw_terrain.py` | Streams raw bag clouds; validates GT and recorded extrinsics; stationary only |
| Fine-reference scorer | `tools/terrain_metrics.py` | Fixed reference denominators; unknown/out-of-map count; no spatial tolerance dilation |
| Isaac geometry probe | `OmniLRS/src/environments_wrappers/ros2/terrain_gt_probe.py` | Opt-in simulation-thread PhysX queries; raw collision reference, not semantic labels |

The mapper uses 10 cm XY cells and **continuous floating-point heights**, not
10 cm vertical occupancy voxels. Each cell retains up to three height layers,
actual return XY, height extrema, spatial support and viewpoint support. A
robust local lower-surface plane separates ground from raised returns. A cluster
on one scan ring can be an obstacle; ring diversity is not required.

Projection distinguishes measured ground, bounded inferred ground, obstacles,
missing ground model, disconnected surfaces, layer-capacity failures and uncertain
raised returns. No occupied-ray carving, GT floor fill, artificial spawn disk or
GT obstacle label enters the classifier. Raw replay currently filters only
nonfinite points; it performs **no decimation or unvalidated robot-body removal**.
Out-of-grid returns are counted explicitly.

Review found and fixed three prototype bugs: refreshed old extrema, duplicate
same-location hits masquerading as a spatial cluster, and lowest-surface seeding
selecting a pit bottom. Tests now exercise them. Ground connectivity instead uses
the nearest observed local surface; this remains a heuristic, not proof of wheel
support.

## Evidence obtained

### Stationary raw replay

Input: archived `revision-3c92fe9c265949b2acf69873f2679ffd/bag`.
All run IDs below are under the external archive configured by
`tools/archive_guard.py`, mounted as `/archive` in the replay container.

- 735 processed scans; one recording-boundary scan explicitly excluded.
- 24,638,666 raw points, zero nonfinite points; 22,257,266 returns within 5 m.
- 352,803 returns outside the fixed 20×20 m output grid, explicitly counted.
- 2,211 recorded `base_link → vlp16` transforms checked. Maximum translation
  variation 0.000000296 m; matrix-rotation variation 0.000000043.
- GT base displacement during the bag: 0.00000270 m. This is stationary evidence,
  **not a moving deskew test**. Raw messages have XYZ only, no ring/time fields.
- End of replay: 1,097 measured-ground cells, 72 obstacle cells, 1,254 disconnected
  cells, 101 missing-plane cells, 38 capacity failures and 61 uncertain raised
  cells. Interpolation adds 426 inferred cells. Counts describe classification,
  not true freespace or obstacle accuracy.
- Measured-only integrate+project p95 **6.48 ms**, p99 **13.13 ms**. With
  interpolation: p95 **7.60 ms**, p99 **14.41 ms**. Whole replay process peak RSS
  **130.3 MiB**, including both mapper variants and Python/bag overhead.
- Repeated while Isaac was starting: p95 9.82 / 11.61 ms respectively; all 735
  per-frame cost/reason hashes matched for both variants. No real-time publisher,
  dense moving-map stress or end-to-end sensor-to-costmap latency was tested.

Artifacts:

- Frozen tested library: `revision-31c741679b584fe38a79647c77c99a6e/liblayered_terrain.so`.
- First replay: `revision-08e4c73758724699a350f717bb9b4c69`.
- Repeat: `revision-c9980a0755074569b8f99f6f6392e68f`.

Each replay contains input/library/source hashes, alignment assumptions, per-frame
hashes and timings, reason counts and both final layered arrays. The old bag did
not include a sealed acceptance manifest; it remains an engineering fixture.

### Live simulator reference capture

Successful raw capture: `revision-38866f5bb134429b8e3d7c4dd749eb77`.
Final-code repeat: `revision-ac3e63337dc4424484404f45f7a04e80` (also 250,000
queries and 11 collider identities). The repeat includes normalization of the
undefined prototype-index field for non-point-instancer colliders.

- Confirmed articulation root `/Robots/astrobot_0/base_link`.
- Actual sensor `/Robots/astrobot_0/base_link/vlp16/Lidar`, type `Lidar` (PhysX),
  extrinsic translation `[0.4, 0, 0.25]` m, identity rotation.
- Captured configured 360° horizontal FOV, 0.18° horizontal resolution,
  40.3° vertical FOV, 1.3° vertical resolution, 10 Hz rotation, 0.5–100 m range.
  Do not infer ring count or full-scan timing from the asset's `vlp16` name.
- Physics-root pose agrees with USD `base_link`; the probe enforces 5 mm / 0.005
  rotation-matrix error bounds throughout capture. The final capture measured
  zero translation discrepancy and maximum rotation-matrix discrepancy 3.0e-9.
- 250,000 downward all-hit queries over a 10×10 m tile at 2 cm XY. Captured
  top/lowest upward-facing heights, top normals, hit counts and collider IDs.
  Terrain, five rock colliders and robot colliders were distinguished by identity.
- Robot identities are provided for exclusion; top surfaces are **not already
  stripped of the robot**. Lowest upward-facing height is not yet a validated
  ground model (for example, buried geometry must be considered).

The first attempt, `revision-2841552ee50a42b2851221a8ce08bdf0`, is explicitly
invalid: `RaycastHit.protoIndex` was initially accessed as `proto_index`, and
PhysX swallowed callback exceptions. The fixed helper propagates callback errors
outside the native call. Both unsorted-hit handling and swallowed errors now have
regression tests. The failed run was preserved, not replaced.

The probe uses the documented active-collider query behavior, explicitly sorting
all-hit results: [NVIDIA scene queries](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.1/extensions/runtime/source/omni.physx/docs/dev_guide/scene_queries.html).
This is collision geometry, not necessarily the exact rendered rock surface.

### Tests

Full sourced-overlay run: **131 Python tests passed**, plus **7 slam_mapping and
4 glim_ros CTest programs passed**. New tests include 60 fixed analytic-return
cases (5 heights × 3 widths × 4 grid phases), single-ring 8 cm hazards, ramp and
gap behavior, disconnected pits/plateaus, duplicate support, expiry, capacity,
PointCloud2 layout/endian handling, fixed scoring denominators and callback errors.

The 60 cases create analytic returns; they do **not** establish that the actual
mechanical LiDAR sees those objects after motion. An initial full-suite command
incorrectly replaced the ROS `PYTHONPATH` and failed collection; the corrected
command preserves it and passes. Runners are documented in `AGENTS.md`.

## Known limitations — do not promote yet

1. **Ground/reference validation is incomplete.** The new tile is not an
   independently visible, labelled drivable/obstacle/dropoff oracle. It was not
   collected in the same session as the archived replay; matching seed alone is
   not enough to claim acceptance against it.
2. **Timing is only stationary-validated.** Raw XYZ has no per-point timestamp.
   Acquisition phase, scan buffering and motion compensation must be established
   from the actual PhysX sensor/ROS node before moving GT replay. The replay
   deliberately rejects moving bags now. Installed `IsaacReadLidarPointCloud`
   node documentation describes a full-scan buffer and scan-completion execution
   output; this is not evidence of an instantaneous/global-shutter scan. Its
   actual graph execution and timestamp wiring still need to be checked.
3. **Inference does not prove support.** Opposing measured planes within 30 cm
   cannot distinguish every sampling gap from a narrow real hole. Inferred cost
   counts as free in the evaluator, so this cannot evade the dropoff gate. Plane
   residual sigma is not a calibrated total uncertainty including pose error.
4. **Evidence storage is conservative, not the final rolling archive.** Three
   fixed height layers expire based on their immutable window-start timestamp.
   Later hits cannot revive old evidence, but recent hits in an expiring window
   are discarded too; the next scan must refresh them. This is bounded memory,
   not an exact 30-second sliding point archive. The map is fixed, not rolling.
5. **Connectivity and support need refinement.** Nearest-local-ground seeding can
   still be ambiguous near unsupported terrain. There is no rectangular-footprint
   usability layer, contact-validated support, safe-motion driver or live Nav2
   revision installation in this implementation. Capacity failures stay unknown.
6. **Only two ablations have run.** Frozen legacy, high-retention legacy and
   no-accumulation layered comparisons on a matched moving fixture remain pending.

## Agreed execution sequence and unchanged acceptance gates

1. **Finish the GT calibration gate.** Create the controlled scene with known
   boxes, ramps, broad raised surfaces, overhangs and holes. Verify collision
   geometry heights to 5 mm, and stationary raw-return-to-surface p95 ≤1 cm within
   5 m without fitting the cloud to GT. Record physics-step pose, raw clouds,
   sensor/TF contract, actual geometry and source/asset/config hashes together.
   Check visible-vs-collision geometry explicitly; ask for visual confirmation if
   those disagree. Do not use the approximate rock-AABB `/gt/map` as the oracle.
2. **Validate timing, then bounded GT-controlled collection.** Establish sensor
   acquisition timing/deskew; only then remove the moving-replay guard. Stationary
   → 2 m translation → clearance-checked ±15° yaw → return. GT feedback at ≤0.15 m/s
   and ≤0.2 rad/s, sole command owner, Nav2/exploration off. Check the rectangular
   swept footprint plus 30 cm margin; stop on stale GT, contact, tracking failure
   or timeout. Reset an unsafe start before recording; do not remove rocks or keep
   pushing when bogged. The old `drive_route.py` is not this guarded collector.
3. **Compare terrain ablations, then fix measured-ground failures.** Use identical
   recorded inputs and independent labels for frozen legacy, high-retention
   legacy, measured-only layered, bounded inference, and multi-view accumulation.
   Attribute each object miss to zero raw hits, filtering, pose/timing, ground
   model, support or projection. Improve uncertainty, bounded sliding evidence,
   negative-obstacle support and footprint usability before increasing gap fill.
   Tilt LiDAR only if raw observation coverage is the bottleneck.
4. **Promote only on fixed gates, then assess SLAM/planning.** Every designated
   calibration hazard ≥8 cm tall and ≥5 cm wide within 5 m must be detected after
   the prescribed motion. Natural scenes require ≥95% object recall, ≤1% false
   occupied flat terrain, ≥95% flat free coverage within independently observable
   GT, no free designated dropoffs and a connected verified corridor. Report
   10/20 m separately. For a 20×20 m map: ≤1 GiB, 10 Hz, p95 <50 ms and p99 <100 ms.
   Replay the same bags with SLAM poses next; quantify the additional error before
   changing GLIM/RTK. Add live revision and costmap-installation barriers before
   promoting to navigation, then test obstacle planning and exploration.

Freeze validation manifests and evaluator before tuning; retain every attempt,
including failed/zero-hit objects. Never choose denominators from candidate-known
cells, silently exclude processing drops, enlarge spatial tolerance to pass, or
reuse acceptance scenes as unrestricted tuning data. GT poses are permitted only
for the labelled isolation experiment; GT terrain must never enter the mapper.
These remain CPU-first experiments; current stationary timing does not justify a
GPU dependency. No safety or mission robustness claim follows from the results above.
