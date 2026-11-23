# Revision-aware terrain implementation

**Historical global-revision workstream.** The
[October 18 handover](implementation-plan.md#october-18-handover--start-here)
describes the current opt-in local mapper and v13 route trials. The latest
recorded GLIM configuration still disables global/submap optimization; accepting
validated no-change bookkeeping updates is not qualification for real global
pose corrections. Do not interpret old milestone status below as the current
local navigation status, or enable optimization over frozen evidence without
the separate correction/rebuild work.

Approved 2026-09-08. Preserve the existing dirty worktree and baseline mapper.

## Ordered milestones

1. Frame/revision contract: full-SE(3) same-time TF, consistent pose/cloud frames,
   immutable GLIM frame/submap revisions, snapshot recovery and contract tests.
2. External immutable scan/revision archive; reconstruction from original local
   evidence; revision-tagged maps and Nav2 installation barrier.
3. Fixed-lag correction tests, then staged submap/global optimization and actual
   loop-correction validation. Never enable optimization over a non-revisable map.
4. Independent simulator geometry labels, evaluator anti-cheating fixtures and
   point retention/unknown-reason audit.
5. Frozen mapper vs CPU layered terrain vs pinned CuPy, identical input/cost policy.
6. Short courses, moving fault/correction tests, 42/411/7 regression soaks and unseen acceptance.

## Contracts

- Raw evidence stays sensor-local, timestamped and immutable. Revisions change
  poses, not observation ages. Re-deskew only when the within-scan trajectory changes.
- Fixed mission/home anchor; one map->odom authority, full SE3. TF is not a
  retroactively corrected historical trajectory store.
- Rebuild old AND new affected footprints and their neighborhood dependencies.
  Mixed map/pose generations cannot authorize motion. Ordinary revisions can build
  in shadow; global changes or invalid local safety evidence require a hold.
- Archive root: `/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab`.
  Container alias `/archive`; 64 GiB/run limit, 50 GiB free-space reserve, no automatic
  deletion or fallback to workspace storage.
- Initial terrain budget: 20x20 m at 10 cm XY, continuous heights, 1 GiB RAM,
  1 GiB GPU allocation, 10 Hz, p95 <50 ms / p99 <100 ms; local correction <1 s.
- Separate GT-isolated, GT-assisted collection, assisted-SLAM and full autonomy scores.
  Fixed independent denominators; unknown/all-blocked maps cannot win. No retry
  replaces a first attempt, no post-hoc alignment or tuning on sealed acceptance.

## Progress

Milestone 1's callback/frame foundation is implemented and stationary-validated.
Milestone 2 has an immutable archive and reconstruction reference, but **not yet
a live revision-aware terrain consumer or Nav2 installation barrier**. Milestones
3–6 have not been promoted. Global/submap optimization remains disabled; the
supervisor now refuses to launch a configuration enabling either optimization
over the legacy consumer. Fixed-lag revisions are exported in shadow and are not
yet retroactively applied to the legacy terrain map.

### Implemented contracts and fixes

- GLIM aligned clouds and pose output use the same full-SE(3) global correction.
  The correction uses the graph's corresponding estimator endpoint, not an older
  live-history pose. Repeated graph callbacks replace visualization submap
  snapshots instead of appending the same submap again.
- Trajectory lookup handles empty, duplicate, reordered and expired history;
  it does not extrapolate an anchor beyond available history. The ROS bridge
  now interpolates full XYZ/quaternions and composes the full IMU/base lever arm.
- Submap consumers copy the original local origin before global handoff. The
  final submap also emits the creation callback. Extension callback cleanup and
  worker-before-extension teardown address dangling callbacks on destruction.
- `SlamRevision` carries session/epoch, parent/revision, frame IDs, measurement
  stamps, extrinsics, submap membership and corrected poses. `GetSlamSnapshot`
  recovers all known frame poses. The adapter owns copies, not mutable GLIM data.
- `SlamObservation` exports each frame's first available local cloud paired to
  its first revision, including initialization frame 0. It is explicitly marked
  **lossy GLIM preprocessing**, not unfiltered raw LiDAR. Raw input, IMU, GT,
  revisions and observations are independently retained in MCAP.
- Exported stamps use GLIM's exact fractional-nanosecond conversion. The first
  live fixture caught a 1 ns rounding-versus-truncation mismatch; association is
  fixed at the producer, not relaxed to nearest-neighbor matching.
- SQLite evidence is append-only and session-isolated. Revision gaps require a
  snapshot; conflicting duplicates and invalid SE(3) are rejected atomically.
  Local points and measurement ages are immutable; ray origins retain extrinsics.
- External archive allocation rejects a missing drive, same-filesystem fallback,
  insufficient reserve and quota exhaustion. The recorder chunks bags and checks
  space periodically. This is a watchdog quota, not a kernel-enforced exact cap;
  bounded in-flight writes can overshoot. No data is automatically deleted.

### Recorded validation, 2026-09-08

- **59 Python tests and 10 C++ test programs passed.** C++ tests exercise actual
  GLIM callbacks, full-SE(3) cloud publications, late-subscriber observations,
  snapshot-service replies and nonuniform submap corrections. Teardown cleanup
  was added after the simulator fixture and is covered by callback-removal tests.
- First fixture: `revision-382311409be44ec786234b9bc29268ca` under the external
  archive root. Its failed audit and partial database remain intact. It exposed
  timestamp mismatch and initialization-evidence gaps. Smoke/snapshot probes were
  started after the bounded launch stopped and failed; neither counts as a pass.
- Second fixture: `revision-3c92fe9c265949b2acf69873f2679ffd`. The new orchestrator
  starts recording/probes before a 60-wall-second stationary Phase 3 launch.
  **497/497 graph frames have immutable local observations; 496/496 published
  corrected cloud pairs agree with their poses** (initial frame 0 has no legacy
  corrected output). Maximum point-coordinate discrepancy: 1.59e-6 m.
- The running snapshot service returned 81 known frames at revision 80. The bag
  retains 496 revisions and 736 raw LiDAR messages. All 1,186 recorded command
  messages are zero; stationary GT displacement is zero.
- All **1,609 recorded map->odom transforms** agree with a causal, fresh,
  full-SE(3), same-measurement-time pose/odometry correction (maximum matrix error
  6.67e-16). The chain is `map -> odom -> base_link`; no `gt_map` TF edge appears.
  TF has no revision field, so this is **not** a unique generation/TF barrier test.
- Independent fixed-home GT diagnostic (no fitted alignment): base-position
  error p95 4.24 cm, maximum 4.47 cm; rotation error maximum 0.00901 rad. This is
  **assisted-SLAM stationary collection**, not pure SLAM accuracy or terrain recall.
- Real-evidence reconstruction check: 20 scans / 232,443 points in a declared
  one-second window, with two independent synthetic SE(3) corrections. Revised
  history and fresh final-pose replay have identical SHA256
  `807467a8d5bfd5292cbf586ea1c93331f4d98dfb0066d9ea0e328607a42f1d1f`.
  1,755 old-only cells disappear; 3,898 new-only cells appear; ages still expire.
  Reference reconstruction took 0.614 s, peak process RSS 160 MiB. This Python
  height-bin oracle is **not** the production mapper or an actual loop closure.
- The legacy terrain map still reports **zero clearance-safe connected cells at
  spawn**. Passing these wiring tests does not resolve terrain support/pathing.

The frozen runtime installs used for the two attempts are retained respectively
in `revision-26272c60bbf34947b5253b0e2bfa9bac/runtime-install` and
`revision-405eda4b90884efdb981a73bccda7504/runtime-install`. These engineering
fixtures did not capture a full in-flight provenance manifest; do not promote
them as sealed acceptance runs. Future evaluation must capture the manifest
before scoring. No simulator goals or moving/loop-closure trials were run.
The stationary orchestrator now starts the existing provenance writer while
the estimator is running and fails the check if collection fails. That hook was
added after the second fixture (syntax-checked, not revalidated in simulation).

### Next implementation gates

**Priority update (2026-09-08):** the user subsequently prioritized GT-isolated
raw-cloud terrain work while global/submap optimization stays disabled. That
offline work now has an implemented CPU mapper and replay results; see
[GT terrain status](gt-terrain-status.md). The live revision/install gates below
remain promotion requirements, not prerequisites for the offline experiment.

1. Build the live disk-backed revision consumer with a bounded tile/scan index,
   archive-health hold, snapshot recovery and epoch/session barriers. Rebuild
   both old and new footprints plus support/interpolation neighborhoods. Match
   the reference oracle before adding terrain heuristics. Preserve the full raw
   mission on disk; bound only active RAM/cache, never silently discard evidence.
2. Add revision-tagged terrain/costmap outputs and explicit Nav2 generation
   acknowledgements. Hold motion across invalid local evidence/global changes;
   do not reopen on stale, mixed, partial or timed-out map installations. Include
   corrections during scan integration, delayed replies, restart and home-anchor
   preservation fixtures. Only this gate can replace the optimization guard.
3. Audit raw point retention, per-point timing/deskew and support observability;
   derive terrain from continuous local heights, not occupied vertical voxel
   survival. Keep 10 cm XY as the initial grid, with 8 cm as a **height hazard**
   requirement, not XY footprint/vertical quantization. Ground, obstacles,
   uncertainty and unknown/dropoff support need separate evidence channels.
4. Create independent simulator geometry labels and controlled small-object,
   flat/noisy/sparse ground, ramp, overhang and dropoff cases. Fix denominators
   independently of each candidate's known/free mask. Freeze evaluator and
   acceptance cases; report all failed first attempts, processing drops and costs.
5. Compare frozen legacy vs layered CPU vs pinned CuPy with the same inputs,
   footprint/cost policy and budgets above. Only then promote a terrain consumer,
   enable submap/global optimization in stages and run actual moving correction,
   obstacle-course and soak gates. Jitter/return-home workarounds are not substitutes
   for supported terrain and feasible paths.
