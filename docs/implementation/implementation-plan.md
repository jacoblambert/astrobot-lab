# Implementation plan and progress

Updated: 2026-10-18 (handover review; latest experiments are October 9).
This is the execution ledger for terrain, localization and
navigation improvements. Update it when an issue is discovered, a fix is made,
or an experiment finishes. Preserve failed attempts and historical evidence.

The [v2 plan](implementation-plan-v2.md) describes the broader direction.
Use the handover below for current priorities; older checkpoints are historical
evidence, not instructions to restart a superseded experiment.

## October 18 handover — start here

### Bottom line

The experimental sensing → SLAM → local terrain → guarded motion pipeline runs
and can reproduce its recorded outputs exactly. It is **not yet ready for reliable
large-scale exploration**. The latest obstacle detour recorded successfully but
did not reach its first goal. The immediate priority is a specific turning-space
boundary bug, followed by excessive turning and command-age waits—not another
sensor redesign or a wholesale SLAM replacement.

This handover changes documentation only. No simulator, replay, build or test
suite was run on October 18. The saved result JSONs were reread, and all **51
workspace runtime-source hashes** referenced by the latest exact-replay report
still match. This does not revalidate external binaries or every untracked file.
The latest revision directories found on the mounted archive are from October 9.

### What is implemented

- **Sensing:** one level-mounted LiDAR, mid-dense +10° to −60° beam layout,
  32 rings × 1,999 columns. The active profile is `selected_optical_v2`, defined
  in `tools/local_terrain/selected_lidar_optical.json`; its base-frame optical
  origin is **(0.4, 0, 0.45) m**. Do not substitute the older generic profile's
  0.25 m height. Native 60 Hz acquisition ticks form six-tick, 10 Hz GLIM windows.
- **Localization:** physical IMU, one-time GT initialization and synthetic RTK
  assistance. GT is not used to steer this costmap trial or label its live map.
  Global mapping is enabled for bookkeeping, but **global and submap optimization
  are disabled** in the recorded configuration. Unaided SLAM and true global
  correction/reintegration are not qualified by these runs.
- **Terrain:** opt-in `live_local_v2`, 6 × 6 m rolling grid, **10 cm XY cells**,
  continuous surface/hazard heights—not 10 cm vertical occupancy voxels.
  Recorded settings: `rolling_incremental`, `frozen_observation`, prepared
  backend, 5 s input window and **60 s retained evidence lifetime**. These two
  lifetimes are different. Native returns use their acquisition-time pose
  associations; the sparse GLIM aligned display cloud is not the terrain input.
- **Motion:** v13 retains a fixed-local-frame route toward the same mission goal.
  Cost changes alone do not trigger a new route. Fresh geometric blockage or a
  failed immediate sweep can invalidate it; a consumed partial route is extended.
  Unknown space is permitted with checks on the immediate 32 cm motion sweep.
  Steering lookahead is 40 cm, forward command 5 cm/s, turn command 0.1 rad/s.
  Turning is yaw-only, with the declared drift bounds; centimeter recentering
  is no longer used. This is a guarded experimental controller, not Nav2.
- **Recording:** native samples are packed without changing their bytes; lease
  reception runs separately from Isaac's Python pauses. After commands and
  acquisition stop, file finalization runs off the physics thread while real
  sensor/clock updates drain the mapper's last samples. The 20 s completion
  deadline remains. Short injected-pause and nominal completion tests passed.

Topic/frame outline: selected timed LiDAR + physical IMU → GLIM poses/revisions;
native returns + causal pose associations → local terrain → `/terrain_shadow/classes`,
`/terrain_shadow/costmap`, contact advisory → `/terrain_trial/proposal` → guarded
driver → `/astrobot_0/cmd_vel`. Preserve each grid's published transform and the
recorded `local_from_map` / `local_from_base`; do not assume grid origin zero or
compare GT and SLAM coordinates without the recorded transforms. GT TF remains
disabled during the declared capture. `/terrain_shadow/planner_costmap` and its
uncertainty mask are a separate opt-in (`--isolated-gap-costs`), not enabled in
the latest baseline. Nav2/exploration integration remains unpromoted.

### Best verified baseline and its limits

The latest completed acquisition is the unchanged `costmap_detour8` layout:
initial-SLAM-frame goal (3.5, −1.5) m, then home; inserted rock 0.4 m high ×
0.6 m wide, 1.8 m ahead. It stopped at **131.9 simulated seconds**, 3.443 m from
the outbound goal. No goal was reached. Recording completed at 134.9 s after
post-roll, taking 862.03 wall seconds.

- Exact replay: **1,159 controller proposals**, **1,160 terrain projections**.
- Transport: **6,950 native ticks / 43,286,273 points** checked, including boundary windows.
- Motion: **17.05 s forward, 67.87 s turning, 21.53 s waiting**, 3.37 s hold.
  GT net displacement 0.681 m; cumulative XY travel 1.674 m.
- Controller computation: median **11.12 ms**, p95 **18.42 ms**. This is recorded
  concurrent timing, not full resource qualification. Mapper + advisory timing
  is separate; do not call the whole pipeline an 11 ms pipeline.
- Causal same-time SLAM horizontal error: median **6.71 cm**, p95 **14.11 cm**,
  maximum **17.41 cm**. Vertical p95 1.35 cm; yaw p95 0.332°. All 6,950 samples
  paired. No catastrophic jump observed, but strict overall diagnostic targets
  still fail; do not describe this as perfect localization or unaided SLAM.
- Last reported regression suite: **626 host / 54 isolated ROS tests passed**
  on October 9. These were not rerun for this documentation-only handover.

All IDs below are directories under the external archive. Keep failures too.

| Evidence | Directory | First files to read |
| --- | --- | --- |
| Latest recorder | `revision-57a87d18ca1a4f699225b6b27e9a7b07` | `collection.json`, `runtime_manifest.json` |
| Native acquisition / executed motion | `revision-5db8f9be8a0f4e4cae92755793b5052e` | `motion.jsonl`, `step_trial_outcome.json`, `natural_holdout.json` |
| Terrain / proposals / refusal grids | `revision-3ac40f8ad9b54eeca76df940250805ca` | `manifest.json`, `step_trial.jsonl`, `terminal_refusal.json`, `terminal_refusal.npz` |
| Exact replay and motion accounting | `revision-2fa08dfa7aa84a27bda582fbb5cfd955` | `report.json` |
| GT/SLAM and transport audit | `revision-fb6a406543ee4e9991e80ed67b708d2a` | `pose_diagnostic.json` |
| Exported trajectory | `revision-a476cec2a4f2489c9cf3391ef4c0b1d7` | `shadow_trajectory.json` |

The earlier v13 recorder `revision-c3d8c7cecc824b49bf78bd0b35354359` is **failed**
because synchronous finalization interrupted the mapper's final updates. It
stopped near occupied rock cells; that is a different refusal from the latest
completed run. Do not merge their diagnoses or score its incomplete recording
as acceptance. The later short completion stress recorder is
`revision-6dfb7e5ff0644f3c8c3a5fb2967f5dee`; its 1,968 ticks and 12.5 million
points passed the separate seal/transport checks.

### Next work, in order

1. **Fix partial-cell turning-space precedence.** In the saved terminal grid,
   an uncertain reason-8 cell starts 50.026 cm from the rover, 5.24 mm inside
   the 50.550 cm wheel-clearance circle. The 61.075 cm current-footprint prior
   applies only to wholly contained squares, so this partial square still
   blocks. No occupied cell is inside the 1.151 m padded body circle. The
   actual oriented wheel sweep has no conflict; the additional circular check
   stops the robot. Reproduce that geometry first. Clip only uncertain geometry
   within the existing prior, retaining the outside portion and every rock
   detection. Do not expand the radius or label an entire partial square free.
   Test grid offsets, rotated poses, tangencies, rocks inside the prior,
   uncertainty just outside it, and attempted entry into narrow corridors.
   Check both route geometry and the immediate sweep. The assumption must stay
   at the current pose, never follow hypothetical path nodes or become map truth.
2. **Validate progress with that one change.** Preserve v13 as the baseline;
   declare a new policy/version before fresh capture. Run focused then broader
   regressions, the same detour, exact replay, independent pose/transport audit,
   and motion/goal accounting. A successful turn alone is not route success.
   Require no new obstacle-clearance violations. Then exercise return and square
   routes before claiming repeatability or starting large exploration trials.
3. **Separate command freshness from slow simulation.** The driver currently
   requires both ≤0.5 s wall receipt age and ≤0.2 s simulation age. Previous
   captures delivered 0.1 simulated-second updates about 0.64 wall seconds apart,
   causing zero-command waits despite a fresh simulated scene. Establish a
   coherent clock/authority contract with explicit publisher-loss and paused-
   simulator tests; do not simply increase or disable the watchdog. Measure
   this independently of the boundary fix.
4. **Improve heading control if it still dominates.** Compare target changes,
   physical yaw/slip and heading error on retained routes. The stop–turn–drive
   controller and repeated clearance-driven replans remain suspects; cheaper-
   route switching is already fixed. Keep the user preference to stay the course.
5. **Only then qualify exploration integration.** Repeat obstacle/return/square
   and longer held-out routes; measure goals reached, collision clearance,
   refusals, waiting/turning time, causal SLAM error, missing data and resources.
   Retain both resource profiles including `dense_flat`, invalidation/recovery
   tests and separation of GT from online decisions. Do not equate exact replay,
   terrain coverage, short-motion availability or a completed bag with exploration
   readiness. Global optimization is a separate later qualification.

### Code entry points

| Area | Files |
| --- | --- |
| Frozen sensor / adapter | `tools/local_terrain/selected_lidar_optical.json`, `tools/selected_lidar_profile.py`, `tools/generic_lidar_adapter.py` |
| Native acquisition / finalization | `OmniLRS/src/environments_wrappers/ros2/{selected_lidar_capture,natural_holdout_capture}.py`, `tools/{native_archive_io,capture_seal,lease_process}.py` |
| Pose associations / local evidence | `tools/live_local_terrain.py`, `tools/local_terrain/{core,pose_lookup,control_context}.py` |
| Terrain classifier | `astrobot-lab/src/slam_mapping/src/layered_terrain.cpp` and matching headers; use the frozen library for baseline comparisons |
| Boundary bug / committed route | `tools/local_terrain/costmap_trial.py`: `current_turn_layers`, `CircularClearance`, `committed_route`, `motion_decision` |
| Actual support/body sweep | `tools/local_terrain/{contact_query,support_sweep,contact_runtime}.py` |
| Route contract / command execution | `tools/costmap_scenarios.py`, `OmniLRS/src/environments_wrappers/ros2/terrain_step_drive.py` |
| Regressions / evaluators | `tools/test/test_{turn_space_policy,route_commitment,route_observation,costmap_scenarios,capture_seal}.py`, `tools/{assess_step_trial,audit_completed_route,diagnose_costmap_refusal}.py` |

### Restart conditions checked October 18

- Archive mounted at `/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab`,
  on `/dev/sdb1`, ext4, approximately **167 GiB available** (`df -h`). Device
  names and free space are observations, not permanent configuration.
- **Stale dev bind:** host `stat -c %d` reports **2065**, running
  `astrobot-selected-dev:/archive` reports **2049**, despite Docker showing the
  expected source path. Recreate that container with the archive overlay after
  verifying no workload owns it, then recheck device identity. Do not capture or
  replay through the stale bind; no environment repair was performed here.
- No simulator or audit/replay container is running. `astrobot-selected-dev` and
  an unrelated PostgreSQL container are running; leave unrelated services alone.
- `nvidia-smi` succeeds (RTX 4070 Laptop GPU, driver 580.178.04, 17 MiB used).
  CUDA context/allocation was **not tested October 18**. Run host and relevant
  container `docker/check_cuda_runtime.py` before Isaac, even if NVML succeeds.
- The repository has extensive staged, unstaged and untracked work, including
  submodules. Root HEAD is `40988b4` (June 1), not a commit containing this
  implementation. Preserve the whole worktree and submodule edits; do not reset,
  clean, or assume handing over the commit alone transfers the current system.
- Old closed MCAP/native files may be `.zst` plus `.cold.json`; use verified
  `cold_archive.py restore` before consuming them. Native TAR restoration is a
  separate byte-checked step. Preserve failed artifacts and the 50 GiB reserve;
  never fall back to workspace storage. No cleanup was done during this review.

Resume commands and pinned dependencies are in the
[handover runbook section](agent-runbook.md#october-18-handover-resume-checklist).

## October 9 — latest experimental checkpoint

**Summary:** route commitment is implemented and its exact live replay passes:
keep a clear path to the same goal, replan only for a blocker or to extend a
consumed local segment. The recording shutdown issue is fixed and passes short
stress plus nominal acquisition. **Navigation efficiency is not solved.** The
latest detour stops after 132 simulated seconds without reaching a goal. Its
specific stop is an uncertain grid-cell edge overriding the existing safe
footprint assumption, not a rock or catastrophic SLAM failure. Controller work
is about 11 ms median; turning and command-age waits dominate elapsed motion
time. Preserve this baseline before changing the footprint-boundary handling.
Simulator and all replay/audit jobs are stopped. Details and earlier attempts
follow; pending statements below are historical checkpoints with results later.

**New task, v13 route commitment:** user rebooted; host CUDA context/allocation
probe now passes. Implement `retain_until_blocked_or_consumed_v1` for the three
multi-goal costmap trials, retaining the same fixed mission goals. Store a
shortened polyline and its next steering endpoint in the fixed local frame;
do not search again merely because another path looks cheaper or new free space
appears. Recheck all remaining segments and the current connector against fresh
occupied/support-conflict geometry. Replan on blockage, extend when a local
partial route is consumed, and start a new route on actual mission-goal arrival.
Rolling-map bounds alone are not obstacles on the tentative retained path;
the unchanged immediate 3-D sweep still requires valid local map coverage.
No classifier, obstacle inflation, turn/slip limit, speed or freshness changes.
The clock-mismatch correction remains a separate experiment, not bundled here.

Initial focused run: **75 pass / one old search-mocking fixture fails** because
an empty *new* search no longer invalidates an already clear retained route.
Update that observation-budget fixture to introduce a real occupied blocker;
keep the original observation timeout assertions. Add route retention, pose
jitter, cost change, new rock/dropoff/support conflict, rolling-edge, immediate
sweep, goal transition and partial-route extension fixtures. **86 focused tests
pass**, including all three ideal closed-loop route/home tests. Broader regression
and live validation pending; these fixtures are not Isaac navigation acceptance.

Final v13 revision: **616 host / 54 isolated ROS tests pass**; formatting,
compilation and root/submodule diff checks pass. A new cheaper-detour comparison
proves fresh search changes sides while the retained clear route does not.
Immediate 3-D sweep rejection now invalidates commitment for the next search;
it never grants motion based on stored geometry alone. A partial-route fixture
initially teleported to its endpoint with a still-active turn anchor and correctly
hit the unchanged drift guard; fix the fixture to start aligned for forward
travel, retaining the guard and adding the explicit forward-command assertion.
Host/container CUDA probes pass after the user's reboot. Prepare a fresh headless
detour using the same v12 sensor/GLIM/terrain/route and unchanged command-age limit.

Pre-capture maintenance completed: lossless, round-trip SHA-256 verified packing
of closed mapper `native.bin` files in `revision-aa85fed72b0e41caaa2b020d3027d291`
and `revision-85c1b956757c411dad70f4be1f8bd89d`; MCAP parts 0–9 in recorder
`revision-bfe2f202920c4949960ba722d35e7aee` and parts 0–8 in
`revision-5d2327fd730844b2822f0ab2805486c0`. Originals remain recoverable using
`cold_archive.py restore` before future replay. No permanent evidence deletion.
Free space **94.30 GiB**. No maintenance overlaps the new capture.

**October 9, 00:01 JST — v13 live test in progress:** recorder
`revision-c3d8c7cecc824b49bf78bd0b35354359` starts before the matching headless
`costmap_detour8` / `single_40x60_detour_v1` world. All host/container CUDA,
archive bind and recorder-first checks pass. Same 600 s simulation / 3,600 s
wall limits; no injected stress. Runtime source frozen through capture and
applicable exact replay. Result pending.

V13 native `revision-00e961a080f2438e907734ec73e8df8b`, mapper
`revision-56e4758f3852441fb2a5460e6509db09`. Interim at 309 simulated seconds:
healthy recording, no reached goal, estimated outbound distance 2.784 m.
Route commitment does retain clear polylines (including a roughly 25 s interval
around 249–274 s), but frequent fresh clearance rejections still cause replans;
at 309 s generation is 125. At 274 s the retained next waypoint was about 1.2 m
away, yet heading corrections continued. Do not claim route retention alone has
solved efficiency. Runtime remains frozen; await terminal recording then exact
replay, motion accounting and clearance-rejection attribution.

**V13 navigation outcome: terrain refusal at 341.200 s**, remaining outbound
distance **2.634 m**, route generation 147. Current circular turning clearance
remains blocked for the original refusal interval. Simulator ends measurement
after the original post-roll at **344.200 s**, with **19,342 native ticks**.
However recording is **incomplete/ineligible**: mapper loses its simulation wall
heartbeat during synchronous finalization (capture callback max **1.689 s**,
tick gap **1.725 s**), with final pose/scan processing still pending. Recorder
wall duration **2,199.87 s**. Simulator stopped; no exact completed-route replay
or navigation pass is claimed. CPU/host CUDA probe still passes. Controller
compute median/p95/max **11.74 / 20.58 / 154.68 ms** over 3,222 proposals; this
is recorded concurrent timing, not a completed navigation/resource qualification.

Locally fixable completion issue: announcing the final native tick before a
blocking seal was insufficient, because the mapper still needs real clock/SLAM
ticks to drain its delayed last pose brackets. Implement a bounded post-motion
sealing worker for selected moving captures. Stop commands and native acquisition
on the physics thread, publish the original completion boundary there, freeze
audit metadata/timing, then continue actual sensor/clock ticks while files and
receiver cleanup finish off-thread. Publish success only after sealing and cleanup;
retain failure markers and the original **20 s** recorder sealing budget. No
synthetic clock, missing-tail trimming or heartbeat extension. New slow-seal,
failure, deadline and no-acquisition-restart fixtures are in `test_capture_seal.py`.
An old cleanup fixture lacked constructor state and could swallow its own
assertion; initialize that state and assert the original fault marker explicitly.
Initial focused sealer run: import-path fixture corrected, then that old fixture
corrected; **79 tests pass** before final frozen timing-snapshot addition.
Final regression and short live completion check pending before repeating detour.

Final sealer revision **626 host / 54 ROS tests pass**. Single completion
publication and frozen producer-audit metadata are tested; resumed sensor ticks
cannot append native acquisition after its declared boundary. Timing is copied
on the physics thread before background cleanup to avoid concurrent mutation.
Original 20 s sealing deadline remains bounded; late/error paths cannot publish
a successful completion marker. Source manifests hash `capture_seal.py`.

Failure-only v13 motion/grid inspection: **53.80 s forward, 193.13 s turning,
65.35 s command waiting, 4.07 s hold**, no reverse. 2,814 proposals retain their
route, 146 invalidate it as blocked, 15 advance within it, one consumes/extends
a partial path; 13 consecutive steering-target jumps exceed 90°. These counts
cover the recorded failed attempt, not a completed-route pass. The terminal
snapshot contains **four occupied cells** within the 1.151 m padded body circle;
nearest distance **1.043 m**. Wheel-support conflicts have minimum distance
0.629 m, outside the 0.506 m wheel bound: **no support-cell blocker** at the stop.
This terminal refusal is rock-clearance margin, not a ground-coverage gap.
No clearance threshold is changed on the strength of this diagnostic.

Post-failure storage maintenance: verified lossless packing of mapper
`revision-56e4758f3852441fb2a5460e6509db09/native.bin` and all 14 MCAP parts in
`revision-c3d8c7cecc824b49bf78bd0b35354359/bag`; full round-trip hashes pass,
original bytes remain recoverable. Free space **83.04 GiB**. Maintenance finished
before the next capture. Host/container CUDA allocation and archive checks pass.
Fresh short completion/GC test recorder: `revision-6dfb7e5ff0644f3c8c3a5fb2967f5dee`,
declared `turn15_gc_stress_v1`, same sensor, terrain library and unchanged limits.
Result pending; runtime frozen during acquisition.

Short async-completion test **passes**: native
`revision-3f5b11a4cd56461a868a4c4ba3445924`, mapper
`revision-3ef5b76c110a4ccebe0e4ab674200910`, **310.80 s wall**, 1,968 native
ticks received/integrated, 329 projections, one completion publication. All
three forced GC events occur (363.90 / 326.85 / 342.02 ms); 110 ms storage
injection retained. Background-sealing timing flag is true. Exact packed-byte
restoration verifies 3,936 logical files; independent mapper seals and ROS/native
transport audit pass (**328 windows, 12,500,296 points**, including boundary
windows). Simulator stopped after recording. This verifies short instrumented
acquisition/finalization, not navigation success or nominal resource acceptance.

Before fresh nominal v13 detour: lossless cold packing of closed MCAP parts in
`revision-d85c01f218354af1907520bf30193c10` (0–8),
`revision-f176f7a28ade4fb9a811a68d9a135378` (0–7) and
`revision-d4d0f081b64f47caab3831bf2154ab08` (0–12). Failed outcomes stay failed;
originals remain recoverable. Wait for all round-trip checks before capture.

All 30 MCAP parts packed/verified successfully; **98.40 GiB free**. Fresh
host/container CUDA checks pass. Nominal v13 retry recorder
`revision-57a87d18ca1a4f699225b6b27e9a7b07`, same `costmap_detour8`,
`single_40x60_detour_v1`, 600 simulated / 3,600 wall seconds. Only async
post-motion finalization differs from the preceding v13 attempt; no navigation,
terrain, speed, slip or clearance changes. Runtime frozen. Result pending.

Nominal retry **recording completed, route refused**: native
`revision-5db8f9be8a0f4e4cae92755793b5052e`, mapper
`revision-3ac40f8ad9b54eeca76df940250805ca`. Controller terminal at **131.9 s**,
`would_lose_turning_clearance`, outbound remaining **3.443 m**, generation 60;
no goal reached. Acquisition finishes at **134.9 s**, **862.03 s wall**,
**6,950 native ticks received/integrated, 1,160 projections**, one completion
publication, no missing required topics or recorder failure. This run confirms
nominal post-motion completion, not navigation success. Stop owned simulator,
then exact controller replay, independent transport/pose and refusal-grid audits.

Exact replay **passes** in `revision-2fa08dfa7aa84a27bda582fbb5cfd955`: all
**1,159 controller proposals** and recorded terrain/contact outputs reproduce.
6,590 execution ticks; **17.05 s forward, 67.87 s turning, 21.53 s waiting,
3.37 s hold**, no reverse. GT travel 1.674 m, net displacement 0.681 m;
no reached goals. Minimum padded footprint clearance to the inserted rock
**34.10 cm** (not a claim about every scene hazard). Controller compute
median/p95/max **11.12 / 18.42 / 47.39 ms**; recorded concurrent timing only.

**Plain-language remaining issue: a partial grid cell can wrongly block turning
inside the assumed-safe footprint.** Unlike the preceding v13 failed recording's
rock-margin stop, this completed retry has **no occupied cell inside the 1.151 m
body circle** (nearest 1.301 m). Instead one UNKNOWN/reason-8 support cell begins
**0.50026 m** from the current center, just inside the **0.50550 m** wheel circle.
It is only partly inside the unchanged **0.61075 m** current-footprint prior, so
`current_turn_layers` leaves the entire square blocking; the outside corner
causes its inside portion to block too. The actual oriented wheel sweep has no
conflict cells; the additional all-heading circular check refuses the turn.
This is a discretization/precedence issue, not evidence of a rock at the stop.
Do not enlarge the prior or mark the whole square free. A separate follow-up
should test clipping only uncertain geometry inside the existing current prior,
retaining its outside portion and every occupied detection. The prior must stay
anchored to the actual query pose, never follow hypothetical future path points.
Recheck all grid phases, actual sweeps, corridor avoidance and live progress.

Independent completed-run audit passes full native/ROS association:
**6,950 ticks, 1,159 windows, 43,286,273 points**, including boundary windows.
Trajectory `revision-a476cec2a4f2489c9cf3391ef4c0b1d7`; pose audit
`revision-fb6a406543ee4e9991e80ed67b708d2a`. First-published same-time SLAM vs GT
horizontal median/p95/max **6.71 / 14.11 / 17.41 cm**, vertical p95/max
**1.35 / 1.77 cm**, yaw p95/max **0.332 / 0.546 degrees**. No missing pose
samples or fitted alignment. Strict overall diagnostic pose targets remain
failed; relative 1 s diagnostic targets pass. These are bounded observed errors,
not a new guaranteed localization bound or terrain/navigation qualification.
No catastrophic SLAM jump explains the cell-boundary refusal. Retrospective
revised poses are separately reported and never substituted for causal inputs.

**Previous v12 checkpoint (October 8):** relaxed drifting-turn control and packed recording are
implemented; **605 host / 54 ROS tests pass**. Packed storage passes short live
stress and byte/seal/transport checks. The new nominal detour runs to **244.5 s**,
surviving the earlier GC/file-scan failure combination, but the **PC sleeps and
CUDA fails on resume**. Kernel Xid 154 reports "Node Reboot Required"; the CUDA
allocation probe now fails with error 999 despite working `nvidia-smi`. Simulator
stopped; no agent reset/reboot or blind retry. No goal reached. Saved motion shows
excessive steering changes and wall-clock command-age waits, so navigation is
not qualified. Offline failed-run diagnosis finds bounded SLAM error (horizontal
p95 **14.70 cm**, max **18.78 cm**, yaw max **0.714°**); seven final pose brackets
are missing. A fresh live test needs host GPU recovery. No watchdog or terrain/obstacle margin was relaxed to hide
these failures. Detailed evidence and failed attempts follow.

**User-authorized drifting-turn trial (v12), in progress:** remove the 6 cm /
2 cm SLAM position-centering controller. Turns command yaw only from the latest
pose; the frozen turn anchor remains diagnostic and bounds cumulative motion,
not a position target. Change the cumulative SLAM/independent-GT turn limit from
25 cm to **75 cm**, explicitly authorized after visual inspection of crater
slip. Legacy `turn15/45/360`, `loop16` and `costmap1/2` keep their own old limits.
No retrospective acceptance of the failed v11 detour.

Every new-route path and immediate command keeps the existing inflated body and
wheel circular clearance plus an additional **5 cm XY drift allowance**. The
driver independently rejects more than 5 cm movement during a turn proposal,
including the transition to its next sequence; a fresh sequence cannot erase
excess motion. No GT steering. Rock vetoes, actual footprint/floor checks,
command freshness, lease, stall, fixed survey bounds, forward horizon/speed,
route goals and 600 s/3,600 s budgets remain. A blocked hypothetical forward
step no longer erases the frozen turn anchor before observation fallback.
The 61 cm current-footprint prior and raw terrain evidence are unchanged. This
is a new control/safety-envelope experiment, not a claimed calibrated slip or
localization-error bound. Existing inflated body margins remain in addition to
the new slip allowance; they are not reduced to make room.

Scoped files: `tools/costmap_scenarios.py`, `tools/local_terrain/costmap_trial.py`,
`OmniLRS/src/environments_wrappers/ros2/terrain_step_drive.py`, and the turn/route
fixtures. Original v11 files are preserved in
`revision-6962a03b760a45cb851e91b7cfd76a8a`. Explicitly update old 25 cm / centering
expectations to the authorized v12 behavior, and add tests for same-proposal and
inter-proposal excessive slip, changed-limit rejection, a rock inside the added
slip margin, and no reverse correction under position jitter. Initial focused
run: 84 pass / one old-limit expectation fails; after updating that declared
expectation and adding fixtures, **89 pass**. Formatting/compilation and both
repository diff checks pass. Broad regression and fresh live detour pending.

Storage maintenance before capture: six closed `native.bin` files packed and
full SHA-256 round trips verified, recovering approximately **22.8 GiB**, free
space **84.59 GiB**. No permanent evidence deletion; restore before replay.
Runs: `revision-0113a56294cf4d8f817fe1404a8c820f`,
`revision-6f4a0744d9da4a85bc83979e1ef15d95`,
`revision-b47a9b95a2da4d9d9dfef8348a5a8a26`,
`revision-d18495f341a34c2db1b37c3d507e6f26`,
`revision-5d48a652c4b046ccb48bf8bbc0ef7d11`,
`revision-cd8d9744bf104965b6300025c304a1d4`. No cleanup/capture overlap or archive
reserve change. The prior GUI recording also completed (641.91 s wall time);
the GUI simulator was already stopped when this task resumed.

V12 final regression: **565 host / 43 ROS tests pass**; 89 focused contracts
pass. Fresh host/container CUDA and archive/bind preflight pass. At **15:29 JST**
start recorder `revision-20786131a0b347eea1f01971eb979d6e` followed by the matching
headless `costmap_detour8` simulator (`single_40x60_detour_v1`, lean experience,
import-stat cache off). Same frozen GLIM/library, selected sensor and wide-goal
route. No capture/cleanup overlap. Runtime source remains frozen through capture
and exact replay. Native `revision-0b54c778539343f0a96f0e0403668bef`, mapper
`revision-1b990d14ffcf42ec831fdb25c4dcc650`.

**V12 live result: incomplete acquisition, not navigation acceptance.** At
314.100 s the simulator lease expired (538.67 ms since last processed grant,
500 ms bound), coincident with a 359.34 ms generation-2 GC pause in native
first-hit auditing. All recorder readiness checks remained healthy. Recording
wall time 2,085.66 s; simulator explicitly stopped. The relaxed turn policy
survived more than 38 cm cumulative physical turn displacement and resumed
forward driving after prolonged observation. No route goal was reached; last
estimated outbound distance remaining 3.049 m. Failed evidence is retained;
completed-route audit/exact replay cannot qualify this incomplete recording.

Recorder filesystem checks reached 232.71 ms work time (240.91 ms total), with
35,546 per-tick native files by this stage. A post-stop five-repeat check of
the same native directory takes 61–65 ms without concurrent capture; this is
not live timing acceptance. The receiver thread shares Isaac's Python GIL,
so simulator GC can still delay it even when DDS/recorder remain healthy.
Next scoped investigation: isolate lease reception from the simulator Python
interpreter, preserving actual receipt timestamps, all identity/freshness
checks, terminal revocation and the 500 ms deadline. First reproduce the
observed timing combination in an isolated ROS fixture; no blind long retry
or heartbeat-timeout increase.

Failed-run motion diagnostic (all 17,592 executed rows, no completion claim):
GT travel 3.816 m, net displacement 1.344 m; 40.00 s forward, 162.37 s yaw,
73.20 s waiting for a fresh proposal, 17.62 s hold, zero reverse. Minimum padded
footprint clearance to the inserted rock 35.02 cm. This measures that one rock,
not every natural obstacle. The route therefore still spends far too much time
turning/waiting; physical slip was not the stopping reason.

Implemented candidate `tools/lease_process.py`: grant reception/validation in a
fresh Python process, bounded local IPC queries, original receipt timestamps and
unchanged 500 ms continuity rule. Child death, hang, wrong publisher, revocation,
clock disagreement and late restart remain terminal. No independent grants,
GT steering, GC suppression or timeout extension. **200 focused host / 50 ROS
tests pass**, including seven new process-isolation tests and the real
180 ms pre-pause + 360 ms GIL-held pause combination. Isaac Python child
startup/no-publisher refusal/clean shutdown probe passes without launching a
world. Native holdout manifests hash the new helper and original lease validator.
Short instrumented simulator test and nominal route retry pending.

Final pre-stress regression: **565 host / 50 ROS pass**, diff/compile checks
pass; host and container CUDA allocation probes pass. At **16:18 JST**, start
recorder `revision-3a0d45d5e63d441996d551b22b1fd2bb`, then matching headless
`turn15` / `turn15_gc_stress_v1` (forced collections at ticks 240/480/720,
110 ms storage injection, no inserted rock, same frozen GLIM/terrain library).
900 s wall limit, original route/guards/reserve. No source edits during capture.

Storage before this retry: eleven additional closed `native.bin` files packed
with full round-trip SHA-256 verification; free space **85.21 GiB**. No permanent
evidence deletion, and the current failed detour is not compressed. Restore cold
files before replay. IDs: `revision-5ea31dff4b0e44649d37fdda39644c19`,
`revision-f3b5f4d770474dfabc0bdfe8cb0a1fb7`,
`revision-2aec77630553438d8668811517295dff`,
`revision-a54551ed2894406f9f0b7d9d3683c811`,
`revision-d91fdd815659455286f973668bc8213b`,
`revision-ba988f41572c40afb3b9482bf43f161e`,
`revision-5738f36de25340bfb078e0d02a04b76c`,
`revision-28e42a35ec4e4913af41c40fbae87763`,
`revision-d9a58ae2713945afa50fa0dc03979afc`,
`revision-7684773b78ae49c68226344c3214c67b`,
`revision-19044e4dede149d5b38412ca90cf0a0b`.

First process-isolation stress attempt **failed** at 22.733 s / 130.60 wall s,
before any forced collection: native `revision-8037f1f9f0ed471487d5b9cb7ee18a65`.
The new synchronous IPC proxy missed that both the simulation thread and the
motion watchdog call `ready()`. Concurrent queries could consume each other's
responses; last actual lease age was only 55 ms. Stop and preserve this failed
run. Fix: serialize complete request/reply pairs with the existing lease lock,
including shutdown. Add three concurrent callers / 360 queries regression.
Also retain the original separate parent proposal callback node/thread for
terrain-controlled routes; only grant reception belongs in the new child.
No heartbeat threshold, stress input or physical limit change. Repeat the same
stress protocol after regressions; no moving-route retry before it passes.

Serialized revision: **51 ROS / 200 focused host pass**, root/submodule diff
checks and fresh host/container CUDA probes pass. Repeat identical stress
recorder `revision-98caf1b9a13042f180d1d7b0162c8b44` at **16:25 JST**; headless
simulator started after recorder readiness. Previous stress failure remains
ineligible. Runtime source frozen while the repeat runs.

**Stress repeat completed** in 302.98 wall s. Native
`revision-3fd3805dabe34a7698bc5340d6c86ad4`, mapper
`revision-f7a451ad9a3c4fbea035703ecebc0ac1`: all **1,965 native ticks integrated**,
328 map projections. All three injections occurred at the original 240/480/720
tick counts; GC pauses **290.61 / 329.37 / 287.32 ms**, maximum native processing
397.00 ms. Recorder storage work max 154.71 ms including the 110 ms injection.
Simulator stopped. Independent `verify_seal` matches all three output hashes;
`selected_native_audit.audit_transport` checks all **1,965 ticks / 328 windows /
12,475,033 points**, including both acquisition boundaries. This proves this
instrumented short run completed without data loss, not long-route qualification
or arbitrary-pause tolerance. Forced pauses are shorter than the failed nominal
detour's 359 ms pause; the isolated 360 ms GIL fixture independently covers that
duration. Resume the unchanged v12 detour with no injected stress.

Additional pre-long-run fixture exposes another IPC edge: a 360 ms GIL pause
*while decoding a reply* makes that reply stale even though the child continues
receiving valid grants. The fixture initially fails at 360.29 ms reply age.
Fix: never authorize from that stale reply; permit at most one fresh query of
the continuously validating child. Child-reported failure is immediately sticky,
the original 500 ms lease deadline stays unchanged, each socket wait remains
200 ms, and the entire read is bounded to 500 ms. Add revocation-during-pause
and two-pause/deadline regressions. This is not backdating a receipt or reviving
an expired lease. Repeat short stress on the final source before long retry.

Final receiver revision: **54 isolated ROS tests pass** (including concurrent
queries, GC during reply decode, revocation and bounded retry failure). Same
stress protocol restarts at **16:35 JST**, recorder
`revision-c1be4f3ceb0e48feafca37c28ec03c4b`, after fresh GPU/archive checks.
Runtime source frozen again. No cleanup or replay beside capture.

Before that launch, eight more closed native streams were losslessly packed;
free space **95.91 GiB**. This includes the now-diagnosed failed v12 mapper;
its original bytes remain recoverable, not erased. Restore before any replay.
IDs: `revision-1b990d14ffcf42ec831fdb25c4dcc650`,
`revision-4dd5fa63d8604534be3f3269d58f069d`,
`revision-236b00c76833406da7767f4b195744d9`,
`revision-64a742ec10ac435fbfd848949e5f3a8b`,
`revision-a7016aa615e54f0489b89e6dcd39a5b0`,
`revision-f1d62591736e43fd9d6f983c412b98b6`,
`revision-e9799ed8aab347f09817ce166dd286cf`,
`revision-7903b7901db04276a507f76dcc38dcae`. Each `.cold.json` records full
original/compressed SHA-256 and verified round-trip checks.

**Final receiver stress completed**, 308.72 wall s. Native
`revision-274c14ff736145ef8d4a9439644db05e`, mapper
`revision-5dea696e91fb4dc99b4ac770a5c5e04a`: **1,952 ticks integrated / 327
projections**, no failure. Three forced GC pauses 299.66 / 315.46 / 290.07 ms;
maximum native processing 379.43 ms, storage work 148.04 ms including injection.
Simulator stopped. Independent final seal/transport verification precedes
nominal v12 retry; the two completed short repeats remain instrumented evidence,
not long-route acceptance. No runtime source change after this stress.

Final stress seal hashes all match; transport independently verifies **1,952
ticks / 326 sensor windows / 12,406,045 points**, including boundary windows.
At **16:42 JST**, fresh nominal v12 `costmap_detour8` recorder
`revision-5d2327fd730844b2822f0ab2805486c0` starts before the matching headless
simulator. Same rock/route, sensor, GLIM, terrain and drift-clearance policy;
new lease receiver, no forced GC or storage injection. Archive has about 94 GiB
free, both CUDA probes pass, no concurrent cleanup/replay. Runtime sources stay
frozen until capture and applicable exact replay finish. Result pending.

Nominal retry native `revision-6024d83afbbd46949fa8f98900cbbcc8`, mapper
`revision-85c1b956757c411dad70f4be1f8bd89d`. Startup passes, proposal reception
works with the preserved parent callback node. At 129.3 s no goal reached;
forward commands are intermittent amid repeated path-heading changes. A single
65.217 s diagnostic sample has GT yaw -77.779°, estimated -77.475°, desired
heading -105.059°; this is consumer-time heading context, not full causal SLAM
accuracy. Do not infer a yaw-estimation failure from slow physical turning.
Need final motion-time accounting, exact maps/proposals and pose/transport audit.

**Nominal retry failed at 232.3667 s**, recorder stopped and simulator stopped.
Failure is now `motion simulation heartbeat expired`, not the grant lease:
last validated grant age **176.28 ms**. Last 35 motion rows were already zero
commands; map clearance flickered blocked/available before the halt. Same-tick
GC **344.29 ms**, native audit/archive **416.42 ms**, followed by the exact
periodic directory quota scan **95.62 ms**: those two stages alone exceed
the unchanged 500 ms driver deadline. A previous 383.54 ms GC at 126.233 s
survived. The separate grant receiver works in this run; it cannot eliminate
physics-thread file/GC work. No completed-route/SLAM acceptance is claimed.

The proposed heartbeat-placement change was **not implemented**: deeper timing
inspection shows it cannot remove the >500 ms actual work. Instead fix growing
live file count. Selected captures now write the *same* per-tick NPZ/first-hit
JSON bytes into one append-only uncompressed TAR, retaining original logical
names, per-file hashes, native timestamps and ROS bytes. Packer retains no
per-member Python index. `native_storage.json` seals its count/size/hash.
`restore_native_archive.py` requires closed native AND recorder records, verifies
every member/hash and capacity before materializing byte-identical files for
unchanged evaluators. It refuses overwrite, symlinks, traversal, bad sizes/counts
and archive/quota failure. Source TAR remains immutable; failed sources remain
failed. `audit_completed_route.py` performs this restoration for packed inputs.
Ordinary/legacy captures retain their existing format. No heartbeat, watchdog,
GC, sensor, terrain or motion threshold changes.

Packed-storage fixtures: initial **42 pass / one fixture-directory collision**;
require the intended `native_chunks` directory and fix isolated fixture layout,
then **43 pass**, including 1,200 ticks / 2,400 identical restored files, bounded
writer state, partial-write rejection and idempotent no-overwrite restoration.
Full failed-run byte/performance comparison and fresh stress/route tests pending.
A first host benchmark command could not resolve the recorded `/archive` alias
and stopped before allocation; rerun in the documented container environment.

All-tick storage comparison **passes**: output
`revision-6411d8a161e4417cad8699267a38d12c`, all **12,630 ticks / 25,260 logical
files** byte-identical under an independent standard TAR reader. Live native
file count changes 25,260 → **1**. Ten paired, alternating-order cached-directory
checks: median **44.56 ms → 0.0156 ms**. All timings retained; this is offline
filesystem evidence, not simulator acceptance. TAR overhead is 19.71 MB on
2.209 GB of unchanged payload. No point removal, relabeling or changed timestamps.
The earlier heartbeat-location hypothesis stays unimplemented; the actual
directory-enumeration cost is what this revision removes from the live loop.

Final packed revision **605 host / 54 ROS tests pass**; compilation, both diff
checks and fresh host/container CUDA probes pass. At **17:37 JST**, matching
`turn15_gc_stress_v1` recorder `revision-7fde65de62db4c82bbbc16c128556d88`
starts before headless Isaac. Same three forced collections and 110 ms recorder
storage delay; only native storage representation changes from the prior final
stress. No source edits during capture. Validate restoration, seal and transport
after stopping Isaac, before another nominal detour.

Additional space recovery before launch: lossless, full-round-trip-verified cold
storage of MCAP parts in four closed recordings (no permanent evidence deletion):
`revision-177c2b2e8a0c436e9d54fd2f358377be` parts 0–8,
`revision-83971b0384884f3ba0ab306d857820c9` parts 0–7,
`revision-20786131a0b347eea1f01971eb979d6e` parts 0–11,
`revision-532e7f5a5fd248a6978224715e865019` parts 0–5.
Free space **96.16 GiB**. Restore these exact parts before any future bag replay;
the most recent failed nominal retry and packed benchmark remain hot. No
compression/replay overlaps this live capture.

**Packed simulator stress completed**, 309.11 wall s. Native
`revision-f4ab35c8d74b47429aff7714b934352e`, mapper
`revision-41dd431de1d1456c868087959d1582d2`: all **1,954 ticks integrated / 327
projections**; original three injection counts retained. GC pauses **353.57 /
351.01 / 364.02 ms**, max native processing 413.60 ms. All **65** native quota
checks total 54.81 ms, maximum **1.748 ms**, versus the previous failed run's
95.62 ms same-tick scan. Recorder max storage work 138.90 ms still includes the
unchanged 110 ms injection. Simulator stopped; verified materialization and
independent seal/transport checks follow. This is instrumented short-run
evidence, not a passed nominal detour or navigation qualification.

Packed stress post-stop verification **passes**: all 3,908 logical files restored
byte-for-byte, all three mapper seals match, and transport checks cover all
**1,954 ticks / 327 windows / 12,415,416 points**, including boundaries. The first
restoration attempt stopped on a root-owned destination directory; after checking
that exact closed `native_chunks` directory, changed only its ownership to
1000:1000 and repeated successfully. Source TAR and recorded bytes unchanged.
No simulator, replay or cleanup remains active before the nominal restart.

At **17:52 JST**, fresh nominal detour recorder
`revision-bfe2f202920c4949960ba722d35e7aee` starts before headless Isaac. Same
v12 route/rock, frozen GLIM/terrain, no injected pauses, packed native storage
and final separate-process lease receiver. Host/container CUDA allocation and
archive bind checks pass; about 94 GiB free. Runtime sources frozen through
capture and applicable exact replay. Result pending.

Packed nominal native `revision-e4df4fb00e8c4a4f95d798cecb8636a4`, mapper
`revision-aa85fed72b0e41caaa2b020d3027d291`. Interim at 155 simulated seconds:
recording remains healthy, no goal reached. Path headings repeatedly switch
sides and forward progress is intermittent. Driver samples also show >500 ms
wall-age waits despite otherwise available checked proposals (76.4 s sample:
614.97 ms receipt age). These are observations, not final timing/pose scores.
No runtime edits or threshold changes during this baseline. Next diagnostic,
if the completed record confirms it: distinguish steering-target churn from
command-delivery cadence before changing terrain classification.

**Packed nominal stops: host sleep / GPU failure, not native-file scanning.**
At 244.533346 s recorder revokes for missing request heartbeat; native failure
is `shadow readiness revoked`, world-step pause **4.703 s**, tick gap **4.794 s**.
Kernel journal records suspend entry **18:17:50**, resume **18:20:31**, NVIDIA
Xid 31 and Xid 154 (**Node Reboot Required**) at **18:20:32**, matching the failure
file's 18:20:32 timestamp. Isaac then logs CUDA 999. After stopping the simulator,
`nvidia-smi` works but `python3 docker/check_cuda_runtime.py` fails `cuInit` with
999. No host reset or further simulator launch. Recorder duration 1,567.78
monotonic wall seconds; 13,436 native ticks retained. Failed/incomplete evidence
is not eligible for completed-route acceptance or exact-controller qualification.

Storage change behaved as intended before the external interruption: native
archive processing max **402.80 ms**, directory quota check max **3.067 ms**
(447 checks), recorder storage work max **45.40 ms**. Native GC pauses **322.24
and 351.53 ms** at 126.717 and 232.933 simulated seconds both survived. This is
measured partial-run evidence, not completed long-run qualification.

All **13,256 executed motion rows** (23.600–244.517 s) show: **38.75 s forward,
0 reverse, 130.18 s turning, 51.73 s waiting, 0.25 s hold**. Every waiting interval
is wall-age-only (>500 ms); none fails the 200 ms simulation-age bound. GT XY
travel **3.715 m**, net displacement **2.815 m**, no goal reached, final outbound
distance **3.361 m**. Across executed nonzero turns: **70 sign reversals**; across
2,237 valid proposals: **17 consecutive available steering-target jumps >90°**.
Minimum padded footprint clearance to the inserted rock **36.44 cm**; scope is
that rock only, not every scene hazard. Motion SHA-256:
`e0fb1b7649e7c6db9bcc65ad05b8d4852ae351f043b4c665988317112fe093c9`.
Computed using unchanged `assess_step_trial.command_times` / `route_measurements`
plus full-row age/sign accounting. No samples removed and no route-success claim.
Candidate next controller investigations: retain a currently clear steering
target to avoid gratuitous side switches, and measure producer cadence before
changing freshness policy. Neither change is implemented or validated yet.

Offline failure-only SLAM diagnostic **completed**, output
`revision-56f791be24d74a9685d33093384373be`, command
`python3 tools/diagnose_failed_glim.py /archive/revision-e4df4fb00e8c4a4f95d798cecb8636a4 /archive/revision-bfe2f202920c4949960ba722d35e7aee --archive-root /archive`
in the documented isolated sourced container (no GPU required). **13,429 / 13,436
ticks** have exact GT snapshots and required pose brackets; final seven lack
brackets and remain explicit failures, not trimmed passing samples. Causal
horizontal median/p95/max **9.42 / 14.70 / 18.78 cm**, vertical p95/max
**2.95 / 3.50 cm**, yaw p95/max **0.441 / 0.714°**; 3-D position RMSE **9.93 cm**.
No fitted alignment or GT steering. Existing strict diagnostic pose targets
remain false; bounded errors on this failed prefix are not full-route SLAM or
terrain acceptance. Revised-history numbers are separately retained in the report.
GT final position is approximately (22.286, 19.658) m: the robot was taking the
north-side detour, not simply stationary at startup, but neither goal was reached.

Final checkpoint: simulator and diagnostic containers stopped; source remains
the tested 605-host / 54-ROS revision (no runtime changes during or after this
capture). Root/submodule `git diff --check` pass. All failed artifacts retained.
Next live test is blocked on host GPU recovery; do not rerun Isaac until the
CUDA context/allocation probe passes. Keep the host awake during capture;
do not mask suspend/GPU failure by increasing heartbeat limits.

**GUI inspection requested:** fresh recorder
`revision-3e1b37f9c2f34a40b80f0abe9142e4f8`, same `turn45_bounded_v1` pure-yaw
protocol and selected optical-v2 sensor, launched at 15:02 JST with
`INSPECT_TERRAIN_V2=true INSPECT_IMPORT_STAT_CACHE=false` and the graphical
inspection overlay. Full GUI experience replaces the lean headless experience;
rendering adds load, so this is a separate visual run, not a timing comparison.
GUI launcher now validates the bounded turn45 profile and corresponding stricter
quota, retaining X11, CUDA, archive, single-recorder and motion checks. **34
inspection tests pass**, shell syntax and `git diff --check` pass. Host/container
CUDA allocation and container X11 probes pass; the 1440×900 Isaac window is
confirmed present. Initialization passed and rotation is confirmed at simulation
stamp 20.917 s: linear command 0, angular command +0.1 rad/s. Native capture
`revision-57f61dce18b64bf7ad5faad9dd755310`. Route outcome remains pending; the
recorder and guarded simulator are left running for the user's visual inspection,
with the declared controller stopping after its route. No live runtime edits.

**Pure-turn reproduction completed:** with no translational commands and no
SLAM centering, the simulated robot moved **12.92 cm** by the +45° endpoint,
**16.06 cm** from the original position at -45°, and **24.09 cm** after returning
to its starting heading (final yaw error -0.626°). These are GT displacements
from the fixed original position, not individual leg lengths. All **3,861**
motion-log samples have zero linear command. The full sequence completed within
the original 25 cm guard; peak measured displacement was 24.09 cm. Simulator
is stopped. This reproduces physical turn translation, not the exact original
47°/25 cm detour event. The acquisition uses the normal spawn, no inserted test
rock and no terrain contact-advisory controller; it is not a flat-ground test
or navigation qualification.

Separately, same-native-timestamp SLAM/GT audit uses all **4,222 samples**:
causal first-published horizontal median/p95/max/final **5.80/12.48/15.06/13.45
cm**, orientation p95/max **0.641/0.750°**. The one-second relative horizontal
p95/max is **7.00/10.25 cm**. Thus centimeter-level recentering is still sensitive
to estimation error, but it did not cause this turn-only physical displacement.
Overall pose targets remain failed; completion is not SLAM acceptance. No fitted
alignment, post-hoc correction or GT interpolation was used. Trajectory export
`revision-5377f4bb61f646d08bb8e418a2e0ea84`; pose/transport audit
`revision-19322c9fd3df48b2b7495bbfa29d486c`. Transport matches all **4,222 ticks /
705 windows / 26,825,784 points**. Original failed v11 detour remains failed.

Command: `tools/run_shadow_capture.py --config
/archive/revision-4c7b70a4acb44a07acb1bca0d63e3692/config --archive-root /archive
--require-physics-imu --natural-motion turn45 --execution-profile
turn45_bounded_v1 --selected-lidar --terrain-v2 --terrain-library
/archive/revision-70647a2b742c446eb61bd3d7141ea011/liblocal_terrain.so
--wall-seconds 900` in the documented sourced dev environment, then
`SELECTED_EXECUTION_PROFILE=turn45_bounded_v1 SELECTED_DEMO_ROCK_PROFILE=
SELECTED_ISAAC_EXPERIENCE=lean_v1 SELECTED_IMPORT_STAT_CACHE=false
bash docker/run_selected_headless.sh start turn45` with normal user/UID/GID
compose variables. Recording wall time 593.86 s. After simulator stop, isolated
`tools/audit_completed_route.py /archive/revision-a9bdc6a7b50645b1af792ecd6fb5b243
/archive/revision-628f0296477349f48c83cf893f01e657 --archive-root /archive` completed.
The runtime source stayed frozen through capture and audit. `git diff --check`
passes. No physics parameter, motion threshold or mapper algorithm was changed.

**Remaining cause isolation:** wheel rates were not published. This run proves
actual simulated displacement under pure-yaw commands, but does not distinguish
asymmetric wheel tracking, friction/contact effects, terrain support or solver
behavior. Next discriminating experiment is wheel-rate instrumentation plus a
separately declared flat-ground comparison; do not diagnose a specific physics
parameter from this trace alone.

**Turn-drift reproduction requested:** run the existing `turn45` pure-yaw
acquisition (+45°, -45°, home) at the normal seed-43 spawn, with unchanged
selected sensor, GLIM configuration, lunar physics and terrain. No commanded
linear motion or SLAM-driven centering; GT only selects the declared yaw
endpoints and enforces the original acquisition guards. This isolates physical
turn motion but is not an exact repeat at the later detour location or a flat
ground comparison. Acceptance for this diagnostic is a reproducible distinction
between GT displacement and SLAM disagreement, not successful navigation.

Added opt-in `turn45_bounded_v1` execution profile: same route hash and 900 s
wall limit, a **stricter 16 GiB/run quota**, unchanged 50 GiB filesystem reserve.
Archive has about 74 GiB free; no deletion or threshold bypass. Headless launcher
now accepts the already-declared turn45 route and validates the profile/route
pair. Optional `/astrobot_0/joint_states` recording is not an estimator input or
readiness requirement; availability remains to be checked. Original runtime
files preserved before edits in `revision-cc457c786b774c35bf0905713edb6b7b`.
Host CUDA context/allocation passes. **185 host / 43 ROS tests pass**; live
outcome pending. Tests cover zero translation, retained 25 cm skid rejection,
unchanged route/hash, strict quota, and recorder/simulator profile matching.
At 13:27 JST recorder `revision-628f0296477349f48c83cf893f01e657` starts before
the selected headless simulator; native `revision-a9bdc6a7b50645b1af792ecd6fb5b243`,
mapper `revision-8c338bd60c0a48d38a9098147e26403f`. Both CUDA probes, archive/bind
preflight and initial GT/GLIM output check pass. At 31.217 simulation seconds,
39.22° yaw has produced **12.44 cm GT displacement** (12.49 cm peak), with
**zero nonzero-linear command samples**. This already reproduces physical turn
motion independently of SLAM centering; completed-route/pose outcome pending.
The joint-state topic has zero publishers, so wheel rates are unavailable in
this recording. Do not infer equal wheel speeds from the commanded pure yaw.

**Latest status (v11):** the navigation policy now assumes the robot's current
61 cm turning footprint is usable unless an occupied cell contradicts it. It
does not relabel the terrain map as measured free. Future motion must preserve
room to turn. If a route or centering correction is blocked, the controller can
try a separately checked observation turn instead of immediately giving up.
Regression: **548 host / 40 ROS tests pass**. Two earlier live detours exposed
controller recovery bugs and did not complete; those results remain below.
The fresh v11 repeat **failed during an observation turn**, not on terrain
uncertainty: physical displacement exceeded the unchanged 25 cm turn guard.
It traveled 2.062 m in total (0.738 m net), reached no mission goals, and used
the footprint prior in 55 motion proposals. Minimum padded-footprint clearance
to the inserted rock was 30.72 cm. Recorder
`revision-99ac5a9b3fae48cca5104445b778fb79`, native
`revision-63ed1212e9994712ac2672eb1f07530e`, mapper
`revision-4dd5fa63d8604534be3f3269d58f069d`. This is incomplete evidence: no
completed-route exact replay or completed-route pose acceptance is claimed.
The owned simulator is stopped; the final focused regression passes **80 tests**.

**What this means:** the requested current-footprint precedence and future
turn-room policy are implemented and regression-tested, and the precedence was
used live. Full-route navigation is still unqualified. The next bottleneck is
centimeter-scale turn centering driven by noisier SLAM positions, compounded by
physical motion during rotation. Do not enlarge the free-space prior, silently
relax the skid guard, feed GT into steering, or claim that more terrain coverage
would solve this control problem. Next control experiment should separate
heading acquisition from precise position holding and explicitly check the
space swept by a realistic drifting turn; any changed motion protocol must be
declared separately from these frozen results.

V11 failed-run diagnosis (read-only motion/proposal logs; not SLAM accuracy
qualification): final turn ID 52 lasted 116.217–136.300 simulation seconds and
rotated 46.81 degrees clockwise. GT XY drift grew to 24.99 cm at the last logged
tick; the guard fired at 136.316674 s. At 122.216674 s the controller estimated
that a **6.37 cm reverse** correction was needed, while the same command-time GT
poses implied a **4.55 cm forward** correction toward the frozen turn start.
The measured relative position discrepancy over this turn was median/p95/max
**6.93/14.55/17.04 cm**. This comparison uses consumer-time poses, includes pose
age, and is not a same-acquisition-time causal SLAM error metric. The prior asset
audit found symmetric wheel centers about base_link XY; a fixed horizontal
wheel-center lever arm is not supported by that audit. Contact-dependent turn
motion remains possible. No fitted alignment or GT control was introduced.

All **1,172 valid proposals** remain archived; 207 used observation recovery,
55 used the current-footprint prior (maximum two overridden cells), and all 55
prior-using proposals offered nonzero motion. Actual command time: forward
21.667 s, reverse 3.083 s, rotation 65.600 s, fresh-proposal waits 23.783 s,
other holds 0.183 s. Controller median/p95/max **42.18/71.00/104.22 ms**.
The 30.72 cm clearance measurement covers all recorded execution ticks against
the inserted rock's XY convex hull, not every natural-scene hazard. Final
focused command: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 uv run
--offline --no-project --with shapely==2.1.2 --with numpy==1.26.4 --with
scipy==1.11.4 --with pytest --with toml==0.10.2 --with pyyaml python -m pytest -q
-p no:cacheprovider tools/test/test_route_observation.py
tools/test/test_turn_space_policy.py tools/test/test_turn_recentering.py
tools/test/test_costmap_scenarios.py tools/test/test_costmap_trial.py`:
**80 passed in 17.47 s**. `git diff --check` passes. Source was unchanged
throughout the v11 capture and this diagnosis; no acceptance runner was weakened
to admit its failed recording.

**New user-directed v9 experiment:** retain turning room instead of threading
non-turnable corridors. At the actual current SLAM pose, the existing nominal
61.08 cm turning footprint takes precedence over uncertain terrain warnings;
occupied rock detections always win. This is an explicitly changed navigation
policy, not improved terrain classification or a reinterpretation of v8 results.
Only fully contained uncertain cells are overridden in a temporary consumer copy;
they remain UNKNOWN, not measured free. Raw map/evidence bytes stay unchanged.
The assumption is never centered on a hypothetical path node or kept as history.
Each proposal records its center, radius and affected cells. The existing raw
contact-advisory/terrain topics remain conservative diagnostics; the route
proposal carries the effective policy, so those displays may disagree locally.

All-heading clearance remains on future paths, and is now checked continuously
over the actual next 32 cm commanded sweep as well: do not drive into a known
corridor where a turn cannot fit. Real rocks, all driver/GT emergency checks,
freshness, skid, floor, lease and archive limits are unchanged. Generic old
`costmap1/2` and positive-support experiments keep their original policies.
Protocol: `costmap_route_trial_v9`. Initial regression **72 tests pass**, including
uncertainty precedence, occupied-cell veto, no historical/hypothetical expansion,
all 73 sampled yaw poses, narrow-corridor refusal, unchanged ideal complete
square/detour/return plants, and turn-centering guards. Initial test collection
needed the repository's fixture import path; fixed without changing policy.
GPU host/container context/allocation checks pass; live result pending.

Saved-map diagnostic `revision-eae3381a674f4177b39890a3ad0d138e` allows a checked
turn at both v8 refusal snapshots; terminal-query median/p95/max
29.83/34.48/51.01 ms. This is a frozen-map counterfactual, not executed motion.
Broad regression: **540 host tests / 40 ROS tests pass**. An additional complete
square fixture injects new uncertainty beneath the current robot throughout
motion; its result is recorded below. No classifier thresholds, sensor settings,
scene layout, goal tolerance or physical guards changed.

The final policy suite passes **13 tests**, including a full ideal square with
new reason-5/6/8 warnings appearing under the current robot throughout the route.
The main suite's 540 tests preceded adding that extra fixture; runtime source
was unchanged. `git diff --check` and compilation pass. At **07:31 JST**, fresh
v9 detour recorder `revision-4fe841bc8ba04269990321f1afa98589` starts before the
matching headless simulator. Same wide-goal layout, selected sensor, frozen
GLIM/library configuration and original GT emergency guards. Source stays frozen
through collection and exact replay. Live outcome pending.
Native run `revision-3981cddc01a94873a152f104212bdbdb`, mapper
`revision-236b00c76833406da7767f4b195744d9`. At 57.93 simulation seconds the
controller is still executing the initial turn/centering maneuver; no terrain
overrides have been needed at that instant. Do not count command availability as
route progress. Live source remains frozen.

**V9 live outcome:** healthy completed recording, but route stops at 126.70 s,
3.315 m short of the outbound goal. Exact replay
`revision-2b05b179971341eeb65e0e9907ace15d` passes all **1,082 maps/proposals**.
Actual travel **1.763 m**, net displacement **0.915 m**, no reached goals;
minimum padded-footprint-to-inserted-rock distance **33.12 cm**. Controller
median/p95/max **49.99/76.34/103.03 ms**, retained without selecting a faster retry.
The current-footprint prior was used in 10 queries, but the terminal map needed
zero overrides: its entire actual turning envelope was already clear. Closest
occupied square: 1.152 m versus the unchanged 1.101 m body circle; nearest
support warning: 0.539 m versus the approximately 0.46 m wheel circle. A safe
leftward grid connection exists, but no route in the current local map gets
closer to the mission goal. The controller incorrectly conflates no goal route
with no possible observation maneuver. This is not a wheel-footprint blockage
or a successful detour. Keep the failure and original v9 evidence.

Four of the 10 prior-using queries issued nonzero commands (maximum nine
overridden cells; 124.30–125.20 s). The footprint precedence was exercised live,
but did not establish a complete route. Pose/transport audit:
`revision-d957a7806c21480d8cd8bf1603022748` (trajectory),
`revision-c97321aa9179462485f39bad032cf424` (diagnostic). Native reference passes
831,104 sampled returns, maximum 0.3355 mm; all 6,493 ticks / 1,083 windows /
40,365,461 points match. Causal horizontal p95/max/final **21.00/25.38/15.67 cm**;
orientation p95/max **0.805/0.954 degrees**. Overall pose target fails; report it
separately from the passed native/transport checks. No fitted alignment.

**V10 scoped follow-up:** after exact v9 replay, add a clockwise observation
turn when goal routing fails but current-footprint motion checks permit it.
Use the same 5-degree checked yaw, SLAM-only turn centering and frozen independent
GT skid anchor. A 90-s simulation-time episode includes all fresh-data waits and
transient path availability; only 30 cm actual estimated translation earns a
new episode. Keep the overall 600-s route limit and all rock/GT/physical guards.
No GT steering or future-node footprint expansion. Proposal explicitly records
the observation mode/start; the driver validates its bounds. Four new observation
tests plus prior focused suites: **77 pass**, including budget exhaustion,
transient-route timer preservation, rock veto, recentering with a fixed anchor,
and resuming the original mission goal after new observations. Full final
regression and a fresh live attempt remain pending.

Saved-state validation exposed a small residual-centering issue: the frozen v9
turn still wanted a 2.69 cm correction, but its full 32 cm translation sweep
could not fit. V10 may defer this correction only while residual drift is below
the unchanged 6 cm correction-start threshold, and only after independently
checking yaw in place. It keeps the original turn/GT skid anchor; no unchecked
translation or smaller sweep is allowed. The frozen-map diagnostic now preserves
the recorded anchor/hysteresis instead of silently resetting it at the query
pose. New tests cover both behaviors. Counterfactual
`revision-f7e040e2957c4497818bc96d78ec210c` permits checked clockwise yaw at both
v9 refusals with the original anchor restored. Terminal query median/p95/max:
13.52/13.87/13.93 ms (offline diagnostic, not live compute qualification).

Additional archive headroom, after simulator and audits stopped: nine closed
September `native.bin` files compressed with full byte-round-trip checks,
**13.73 → 4.01 GiB**, recovering **9.71 GiB**; free space **84.30 GiB**.
No evidence deleted; restore original bytes with `cold_archive.py` before replay.
Runs: `revision-4a4e32c867b7431a8f2a1ba258025433`,
`revision-c668ba6a093249668fea61023ccdf5bd`,
`revision-f0db6c401ce845b7800297622753d1d6`,
`revision-0dd497a6aa6947c585b09d7cd418e6eb`,
`revision-f76e3dd232504365965c8b865997d357`,
`revision-0fa274e900194aa7af5cbbf4c5ae7e6f`,
`revision-675c0781408c4642821bf99be502812c`,
`revision-daee1896dbac4c8c8c00b9af437b04f9`,
`revision-d52e36b1a46546498ea81c23923c4435`. Missing-seal candidates were left
alone. No cleanup overlapped capture/replay; no archive threshold changes.

Final v10 regression: **546 host tests / 40 ROS tests pass**; one subsequently
added saved-anchor regression passes in the six-test observation suite. Runtime
source was unchanged after those final suites. Formatting, compilation and
`git diff --check` pass. Fresh host/container CUDA and archive preflight pass.
At **07:55 JST**, start recorder `revision-daa00134ea6b4c1c861cee8bb4393a0b`
and matching headless v10 detour. Same layout/config/library and no runtime
edits until recording and exact replay finish. Live outcome pending.

**V10 live result:** clean recording; stops at 52.02 s during initial turning,
not a completed route. Native `revision-195866df95314c8e9e1358896c53a7c3`,
mapper `revision-324937e810ce46268e005d28a97d9342`, exact replay
`revision-497f59bfb7f443a4907fab50f5e75169`: all **310 maps/proposals match**.
Actual travel 0.146 m, minimum padded-footprint-to-inserted-rock distance 30.15 cm.
Median/p95/max controller time 40.98/67.33/91.59 ms. Failure is
`would_lose_turning_clearance` on the centering translation, not current yaw.
The observation fallback only handled missing routes, so it never tried yaw in
this case. Anchor-to-current along-heading offset is approximately 8.5 cm.
Do not count this refusal as successful avoidance or successful observation.

**V11 fix after exact replay:** any blocked proposed motion can fall back to a
separately checked clockwise observation turn. If centering translation itself
is blocked, defer it and check pure yaw regardless of the centering hysteresis;
retry correction on later views. No unchecked translation, reduced 32 cm horizon,
changed occupied-cell policy or moved turn anchor. The original independent
25 cm skid guard remains the physical limit. Update the old reverse-hazard
fixture explicitly: reverse remains forbidden, but checked yaw is permitted;
the test independently verifies that yaw against the original unmodified cells.
This is a declared control-policy change, not weakening a collision test.
Budget and full-turn-clearance rules stay as in v10. Final regression and live
validation pending.

V10 pose/native audit: trajectory `revision-e7fe7fe2eaf243fa853d427b3dbf6c80`,
diagnostic `revision-c9d8c35e95b64945acdfd57397fd0c8a`. Native first-hit reference
passes 238,336 samples, maximum 0.2999 mm; all 1,862 ticks / 312 windows /
11,773,752 points match. Causal horizontal p95/max **19.94/22.03 cm**; orientation
p95/max **0.722/0.751 degrees**. Overall pose target remains failed.

V11 final regression: **548 host tests / 40 ROS tests pass**; focused suites
80 pass. Formatting/compilation and `git diff --check` pass. Host/container CUDA
allocation passes. To restore capture headroom, pack the two September 28 MCAP
segments of `revision-1cf991e7efeb4c83b53849bc74e8e103`: original 1,602,372,381
bytes → 567,515,438 bytes, both full round trips verified; original evidence
recoverable. Free space **82.57 GiB**, after all audits ended and before capture.

V11 frozen-map check `revision-7af97613572945da87b6edbebde249b0` permits checked
clockwise yaw at both v10 refusals, with the original centering anchor preserved.
Terminal-query median/p95/max 36.62/37.74/46.74 ms. At **08:06 JST** start fresh
recorder `revision-5ad1e39f041345f4acb72d42938c4134` then the same headless detour.
No capture/replay/cleanup overlap and no runtime edits through exact replay.
Live outcome pending.

**V11 first launch failed before motion:** recorder
`revision-5ad1e39f041345f4acb72d42938c4134`, native
`revision-f7cd80fc3e4d4c858d0c1f9f79050ae8`, mapper
`revision-91b30d7a5d1b4376b9f0c787e005764e`. Failure: GT initialization stationary
window. Read-only failed-bag diagnosis: first cloud 20.916667756 s; preceding
61 GT samples span [19.916667705,20.916667757] s. Maximum speed 0.008631 m/s;
maximum angular rate **0.021393 rad/s**, just above the unchanged 0.020 limit
at 20.666667744 s. XYZ spans [0.1278,0.0343,0.1676] mm. This is tiny uncommanded
startup wobble, not a navigation-policy result. Do not qualify or replay it as
a completed route. Simulator stopped after recorder exit. Keep the original
seed/initialization limits and preserve this failed attempt; one fresh repeat
of the unchanged v11 protocol will test the controller without rewriting the
failed initialization outcome. No hardware repair or repeated blind launch loop.
Recorder-only attempt `revision-f817e874803f4588b0ff85774692c42d` then rejected
a mistyped config-directory ID before subscribing or starting the simulator.
Correct the command to the original verified `revision-4c7b70a4acb44a07acb1bca0d63e3692/config`;
retain this orchestration failure separately from sensor/navigation attempts.
At **08:12 JST**, unchanged v11 fresh repeat starts: recorder
`revision-99ac5a9b3fae48cca5104445b778fb79`, same verified configuration and
matching headless detour. CUDA/archive checks pass. This is a retained repeat,
not a replacement for the failed startup; source remains frozen.

**Bottom line:** cleanup has recovered another **32.62 GiB** this session
(30.85 GiB through reversible compression, 1.77 GiB from explicitly retired
September 9 continuous streams). Recent successful and failed experiments are
retained; **83.21 GiB is now free**. The longest new square travelled **6.56 m**,
but neither it nor the obstacle detours completed its route. Do not describe
these as navigation-ready.

The latest v8 detour now **records and shuts down cleanly**. It gets past the
earlier start-cell rejection, then stops at 148.02 s before the outbound goal.
At its final pose, the actual body/wheel check is clear, but a suspected drop-off
cell is about 43 cm from the robot center. The global planner reserves the whole
roughly 46 cm circle swept by wheels at *every* heading, so it rejects that pose
even though the current wheel patches do not touch the warning. The next planning
experiment should check actual headings and swept wheel patches, not erase
warning cells or shrink safety margins. **All 1,233 maps/proposals replay exactly.**
The native-ray/transport audit passes; causal SLAM horizontal error is **16.76 cm
p95 / 20.14 cm maximum**, not a catastrophic localization failure. The stricter
overall pose target remains failed. Simulator is stopped.

### Earlier October 8 checkpoints (preserved experiment history)

The latest live square drove **6.56 m** and passed the formerly failing corner
turn. It stopped before corner three because of uncertain terrain near the goal;
it did **not** return home. Keep the live-tested v6 clearance/turn-centering
controller. Two offline heading-aware workarounds failed their small closed-loop
test and were removed; source hashes match the recorded v6 again, and all
**512 host tests pass** after restoration (38 ROS tests passed on the unchanged
driver). The next experiment is the declared wide, single-rock detour, to
separate general obstacle navigation from this particular square-corner problem.
Further September MCAP compression completed while the simulator was stopped.
Final regression: **513 host tests / 38 ROS tests pass**, including the detour
layout fixture; `git diff --check` passes. Fresh CUDA/archive checks pass.
At 00:15 JST start the detour recorder `revision-1d6415ec8bff43e9811172c0d46b2a9c`
then matching headless simulator with `single_40x60_detour_v1`. Outcome pending;
runtime source stays frozen through capture and exact replay.

**First detour result: early terrain refusal, plus a shutdown fault.** Native
`revision-03b1aa96517d4874b2ab69e8dcff600b`, mapper
`revision-64a742ec10ac435fbfd848949e5f3a8b`. Controller stops at 90.70 s, still
3.62 m from the outbound goal. Current actual wheel query has no conflict.
Nearest occupied cell center is 1.222 m away; the exact 1.101 m circular body
envelope is clear, but whole-cell padding blocks the starting grid cell. This
is not a completed detour or obstacle-clearance qualification.

During normal launcher SIGINT, Python interrupted a core transaction. Its
`KeyboardInterrupt` string is empty: the mapper fault became `failure=""`,
while a truthiness check incorrectly wrote `experiment_completed=true`.
Recorder correctly rejected this fault. Preserve all files as failed evidence;
do not rescore the saved run as successful. Fix: defer SIGINT until the active
ROS callback finishes, then seal with the ROS context alive; retain the existing
20 s recorder shutdown deadline. Unexpected exceptions now produce a nonempty
sticky fault, and completion explicitly requires `failure is None`. Completed-
recording counterfactuals also reject a legacy empty-string fault.

**V8 candidate:** permit an exactly circle-clear start to leave its current
occupied-padding cell. Endpoint must be padded-free; every other occupied-
padding cell stays blocked, and the full continuous circle plus immediate 3-D
body/wheel sweep still applies. This is not removal of global obstacle padding
(that alternative failed the rock-margin fixture earlier). A saved-map query
permits a checked turn out; this is explicitly a failed-run diagnostic, not
live qualification. Focused controller/shutdown suites: 73 tests pass. Actual
ROS SIGINT fixture passes after fixing its initial use of a paused simulation
clock; use a bounded steady-clock timer. ROS suite: 40 tests pass. Core/rolling
library tests and broad final regression: **180 real-library tests**, **528 host
tests**, **40 ROS tests**, all pass. Initial runs without the explicit rolling
library skipped those cases; the final run supplies both library variables and
has no skips. Selected executable symlink resolves to the edited source;
compilation and `git diff --check` pass.

Third additional cold-storage batch: all 12 September MCAP segments from
`revision-8e36fcafda8e4759a0002f2ffc9fc3ce`,
`revision-5a5ed7ef1a7b47fa951c1148a6af087c`,
`revision-77b353e8129b4a348555d081219853b2`, and
`revision-16dc340d086445eabc07a4794db99f25`. Full byte round trips verified;
original evidence remains recoverable. Free space after packing **82.92 GiB**;
do not reduce the 82 GiB startup requirement or 50 GiB reserve.

At 00:40 JST, fresh v8 detour starts after CUDA/archive checks, same
`wide_goal_south_v2` layout: recorder `revision-532e7f5a5fd248a6978224715e865019`.
Native `revision-52497fb3f913445b8db506472eeabd5d`, mapper
`revision-5738f36de25340bfb078e0d02a04b76c`. Collection completes with no fault;
the deferred-interrupt shutdown fix passes this live test. Route outcome is
`terrain_refused` at 148.0167 s, 3.688 m from the first goal. Simulator stopped
after sealing. No runtime edits during this capture or its replay.

Read-only final-map diagnosis: actual pose XY [-0.1269,-0.8406] m; nearest
occupied square is 1.425 m away, outside the 1.101 m full-body circle. A reason-8
negative-edge square is 0.431 m away and overlaps the all-heading wheel circle;
the recorded actual-footprint query has no conflict. This is a routing restriction,
not proof that the negative-edge label is false. Between 95.90 and 115.88 s the
robot actually rotated 18.79 degrees and translated [-0.0619,-0.2362] m, so it
was not simply oscillating without movement. No GT was used for steering.

**Final v8 audits:** exact controller/map replay
`revision-8a767effde2a4695b866be54f6235fd2` passes all 1,233 proposals. All 7,040
execution ticks retained: 4,885 nonzero-command and 1,867 fresh-data waits. Actual
travel **2.058 m**, net displacement **0.954 m**, zero reached goals. Minimum
XY distance from the padded footprint to the inserted rock is **28.74 cm** across
every executed tick (not a clearance claim for all background rocks). Command
time: forward 24.15 s, reverse 3.58 s, turn 53.68 s, waiting 31.12 s, hold 4.78 s.
Controller median/p95/max **46.46/68.97/93.51 ms**; retain these timings.

Trajectory `revision-e6478e1046d64592874d8e788b9fbdcc`, pose/transport audit
`revision-e508c27383ba46d287236ae9da0aaa9a`: all **7,400 native ticks / 1,235
windows / 46,457,892 points** match, including boundary windows. Native first-hit
reference passes **947,200 samples**, maximum **0.6003 mm**; cadence passes.
Same-stamp GT reference maximum translation difference is 0.1494 micrometers.
Causal horizontal p95/max/final **16.76/20.14/4.72 cm**; orientation p95/max
**0.673/0.825 degrees**. Overall pose target fails; one-second relative target
passes. Final-revised trajectory is separately reported, not substituted for
causal accuracy. This remains a failed route, not navigation qualification.

Commands completed: `assess_step_trial.py MAPPER --archive-root /archive`, then
`audit_completed_route.py NATIVE RECORDER --archive-root /archive`, both in
isolated sourced containers with both archive aliases. Final cleanup/shutdown/
seal regression rerun: **23 tests pass**; the 528-host/180-library/40-ROS final
revision results above also stand. No runtime changes after the live run.

Next useful navigation cycle: reproduce the final clear-footprint/blocked-circle
case in a small motion fixture; require a complete hazard-checked escape or
detour, not just one nonzero command. Evaluate a heading-aware local connector
with the unchanged rock margin and immediate 32 cm sweep. Also retain regressions
for an actual wheel-over-drop-off, blocked turn and impossible route. The earlier
v7 first-command workarounds did not meet this standard and remain removed.
Only after that fixture and existing route plants pass should a fresh live
detour/square be attempted. A new planner is not implemented or qualified here.

Second additional cold-storage batch complete: **16 MCAP files, 13.88 → 4.87
GiB**, recovering **9.01 GiB**; **83.06 GiB free**. Runs:
`revision-6a3e3e9e2192436c8b379917597fa04c`,
`revision-ea53750e680247e4b6e7e5f52328f46d`,
`revision-38b96ba8739f4cfeb84da7d1200f9ec1`,
`revision-b515e5699de54692a635dc2f497a8654`,
`revision-dcd909832dd44ec4905a61a7c31c6749`. All 16 full round trips verified,
compressed payload sizes checked, hot copies absent, original bytes recoverable;
restore before using these September recordings. CUDA allocation passes on host
and in dev. No capture/replay overlapped compression.

**Detour layout corrected before its first live run:** seed-43 USD bounds place
natural rock instance 83 at world XY [24.228,18.194]–[25.065,19.071]. The old
goal (24,18) is only **30 cm from that bounding box**, inside the robot margin.
Do not call that an ordinary open detour. Keep the inserted 40×60 cm rock at
1.8 m ahead, but declare `wide_goal_south_v2`: fixed goal **(3.5,-1.5) m** in
initial planar SLAM frame, then exact home (**7.616 m round trip**). The existing
rectangular survey/position envelope, collision margins and all control guards
stay unchanged. The new goal clears the recorded natural-rock AABB by about
1.84 m, and its straight-line path still crosses the inserted rock's inflated
margin. Added regression checks both facts. This is evaluator-only scenario
design, not GT steering or a passing result; command-time GT geometry veto stays.


### Cleanup completion (October 8)

All five additional cold-storage batches are verified: **79 MCAP files,
54.22 → 23.37 GiB**, recovering **30.85 GiB** without losing sensor bytes.
Together with the explicitly retired September 9 continuous streams (**1.765
GiB**, not recoverable), this session recovered **32.62 GiB**. This excludes the
previous session's 31.76 GiB recovery. Recent experiments consumed some of the
reclaimed space; final available space is **83.21 GiB**, above the unchanged
82 GiB capture-start requirement. No capture or replay overlapped compression.
No October evidence, source, assets, libraries or configurations were deleted.

Fourth batch: **25 September 20–21 bags, 12.07 → 7.00 GiB**. Each run below
has `bag/bag_0.mcap.zst` and its SHA-256/restore metadata. All original and failed
outcomes remain intact. `revision-1008f371b1254dababb9d687740ccc3a` was excluded
because its collection seal is missing; the tool did not weaken that check.

- `revision-56b9a8b2bd654e9ba0a208aa87f66c0c`
- `revision-6cc21be1188642edb2530f48ccc08414`
- `revision-3c29a2e05b9e4933abbd8cc3820d4503`
- `revision-34fdbed4d879407eb395a59080a7747a`
- `revision-6de428b17736459da9bef8ce67d2550d`
- `revision-08dbf22b74b048f3838a14a368843da8`
- `revision-04f22073e2f8424c845b79ad3d328fee`
- `revision-130c42fac14c43df91f7e7c5f9ef0172`
- `revision-39d4a3b8aea34f799a4d674555eea2da`
- `revision-8454800d429d43b19373c17bd31e585c`
- `revision-a1e0eeda0e1c48cb95a064438668946f`
- `revision-d4afb8c2ebfa44428ab6b1ff2322143f`
- `revision-565347844dc7433fb6013c72af381061`
- `revision-e23b3b3b4f3840c49a80b149268aab95`
- `revision-0724ce0d2b21450ba9f9f9f69317cdc2`
- `revision-8cc0350b341d45a192aa5886c76bf2a5`
- `revision-8dc9873b52c8497381048849e9469ade`
- `revision-521eb1b724e84d54826066137b7805d8`
- `revision-864c21ef567c4a0baad935892c9ca361`
- `revision-6a65ec3cefaf4f8899cbe0a288636321`
- `revision-f50e910ab15246b480f701ed69a5338c`
- `revision-6f2b4a51563446a28b93e1855b3eb24c`
- `revision-597c80efcbf0401ca50f8d0e55dc4dba`
- `revision-816d4e50f04d4cb38f162d1bcd9d20ad`
- `revision-7ae79f6b12cd4041bacb14f7bd4045ae`

Fifth batch: **15 September 27–28 bags, 8.66 → 3.55 GiB**. First three runs
below have segments 0 and 1; the others have segment 0. Full round-trip hashes
were checked before removing hot copies; final checks confirm compressed sizes,
metadata, and absence of those hot copies. Restore with `cold_archive.py` before
replay, following the runbook. No evidence is lost through this compression.

- `revision-7d64e4f3f40044adbf01c77ca72e2e63`
- `revision-b3ef0d9beb024eb6a047ce437099a7fb`
- `revision-3885b8425b83408bbdf77c55c7e810bd`
- `revision-8e183b2c89a84ee3a66294d0c243cd12`
- `revision-3855e5257dd6443b918ebac116cf2f6c`
- `revision-5c9d3d6072c94c399c3fc6fa81218c14`
- `revision-d908d0e395e64cc0892f2eefba9d8204`
- `revision-ada8208a15b049b9bd31a01ac377a28d`
- `revision-2eeae3f7866c4eaab928069a84d1742a`
- `revision-086ededb83594b6188ffeb2fc075d9a8`
- `revision-81c6b468b56d4c52aa865fb6d4dc2bf8`
- `revision-ad235e6f6fcc4f79b5280cd19b149a26`

## October 7 — live v4 route validation resumed

User authorizes resuming longer routes and permanent removal of genuinely
obsolete older material. Do not delete merely by age: the September 8 bags
remain referenced by frame/reconstruction fixtures and are retained. Inspect
retired outputs separately; no bulk cleanup during live capture.

Fresh host/container CUDA allocation and external archive preflights pass,
108.71 GiB free. Start the unchanged v4 **8 m square**, recorder
`revision-758de8bdfda54b42bee8823772af19dc`. Same selected optical sensor,
GLIM/RTK configuration, terrain library, fixed goals and safety thresholds;
no source changes during recording. Outcome pending. If complete, audit all
proposals/maps and causal pose/transport, then run the declared obstacle detour.

**V4 square outcome: stopped on side two, not a completed loop.** Native
`revision-2f8606967c274504b3dccbc1caaa70b6`, mapper
`revision-ba988f41572c40afb3b9482bf43f161e`, exact audit
`revision-7487fe2ebe2d4ccd93f3c7df96df9991`. All **1,279 proposals/maps replay
exactly**; collection is healthy. Actual travel **3.884 m**, first corner error
**6.15 cm**. At 142.70 s the planner declares `terrain_refused`, 0.619 m short
of corner two. Controller median/p95/max **8.50/14.42/21.81 ms**. Forward/turn/
freshness-wait/hold times **59.85/35.22/23.78/2.97 s**. No physical guard failed.

Pose/transport audit: trajectory `revision-6b783517192c4cd9a817be1256d1c901`,
diagnostic `revision-44490e0b7a314b9099ee91c31f72fdf9`. All **7,670 ticks /
1,279 windows / 47,492,696 points** match, including boundaries. Native first-hit
reference **passes** (981,760 samples, maximum 0.0616 mm). Causal horizontal
p95/max **11.53/20.90 cm**, orientation p95/max **0.485/0.707 degrees**.
Strict overall pose target still fails; no fitted/final-pose substitution.

**V5 hypothesis/fix:** both saved refusals have clear actual body/wheel areas.
Negative-edge cells are 55–57 cm away, outside the roughly 46 cm wheel envelope,
but conservative whole-grid-cell inflation blocks the *starting cell*. Add an
exact segment-to-closed-cell clearance certificate using the same circular
body/wheel radii. An actually clear start can connect to a neighboring free grid
node without pretending the robot sits at its cell center. Certify connector
edges and shortcut segments; preserve all occupied-cell padding and the exact
immediate 3-D body/wheel sweep. No cell label, hazard threshold, radius, GT
guard or evidence lifetime changes. Protocol is `costmap_route_trial_v5`.

Development checks exposed two integration details: a line on a grid boundary
also touches the adjacent conservative support cell; and replacing occupied-cell
padding with geometry alone made the ideal rock detour stop early. Keep the
original occupied-cell padding, and replace rounding only for support hazards.
Do not weaken the obstacle fixture. Final focused controller suite **79 passes**;
final broader regression **503 host tests and 38 ROS tests pass**. Sealed counterfactual
`revision-7d9c713124fa4a6eab219a8310157983` permits forward at first refusal and
a checked turn at terminal refusal; median/p95 query times **4.27/5.05 ms** and
**6.44/6.60 ms** respectively. Earlier intermediate counterfactual
`revision-1dd9c93c933e4244ae63b5b2ad1ea6a6` is retained. These do not demonstrate
unexecuted motion. Fresh v5 square starts after successful CUDA/archive preflight
at 22:35 JST: recorder `revision-1c548d8c67204051b56428a81f4c6ec5`, native
`revision-694f28ee2f734a0d85231be88b07fa99`. Outcome pending; no runtime source
changes during capture or its exact replay.

**V5 live result: terrain blockage cleared; turn slip stops the square.** Mapper
`revision-cd8d9744bf104965b6300025c304a1d4`. Reached the first two corners and
drove **4.694 m**, then the independent **25 cm rotation-displacement guard**
stopped acquisition at **204.0167 s**. This is a failed recording, not a
completed loop or an eligible exact-replay/pose-acceptance run. Native failure
and all partial streams are retained; simulator was stopped after recorder exit.
The final turn began at 164.2167 s, rotated **78.68 degrees** over 39.78 s, with
26.82 s commanded rotation / 12.98 s fresh-data holds. Base XY moved from
[21.98738, 16.03690] to [21.88415, 16.26459]: **25.0004 cm** at the last logged
tick. SLAM estimated only roughly 11 cm displacement from its turn-start pose.
These are partial-log failure diagnostics, not same-stamp SLAM qualification.

**V6 hypothesis/change:** pure rotation cannot correct the observed turn slip.
Use the frozen SLAM turn-start position to request a separately hazard-checked
straight correction when along-heading drift exceeds 6 cm; finish at 2 cm,
then resume yaw. Forward/reverse 5 cm/s, checked signed 32 cm sweep; no arcs,
GT steering, map relabeling, or changed safety limits. Turn ID/anchor remain
fixed across corrections and fresh-data waits; the driver applies the original
25 cm GT displacement limit throughout the whole maneuver. Original straight
heading/lateral, stall, freshness, footprint and floor checks remain. Protocol
`costmap_route_trial_v6`; this is a changed controller, not a repeat of v5.
Eight focused tests pass, covering signed correction, hysteresis, reverse-path
hazards, metadata rejection, frozen driver guard, and a synthetic slipping turn.
Initial fixture errors (import path, hazard between rather than beneath wheel
tracks, reversed test clock, slip plant not triggering correction) were corrected
without changing the production thresholds. Final broad regression **512 host
tests and 38 ROS tests pass**, including separate reverse-time reporting;
`git diff --check` passes. Host/container CUDA allocation checks pass. Live result
pending.

Fresh v6 square starts at 23:14 JST after recorder readiness and headless
preflight: recorder `revision-9fd01bf5245147e3b862caf0437aaebb`. Runtime source
is frozen during recording and subsequent exact replay; live outcome pending.

**V6 live outcome: 6.557 m driven, stopped 42 cm before corner three.** Native
`revision-e7dde26f45a1476e939ca4baf20b717e`, mapper
`revision-b47a9b95a2da4d9d9dfef8348a5a8a26`, exact replay
`revision-952a69a62b0b4382aae150f0151cb685`: **2,242 proposals/maps match**.
Collection completed normally; route ended `terrain_refused`, not home, at
243.8167 s. First two corner GT errors **12.32 / 5.74 cm**; terminal GT distance
to corner three **31.36 cm** versus estimated 41.53 cm. Turn corrections were
executed, including **0.77 s reverse**, and the formerly failing turn completed
without relaxing its guard. Forward/turn/wait/hold times **96.72 / 65.23 / 46.83 /
8.67 s**. Controller median/p95/max **10.94 / 100.07 / 165.62 ms**; retain these
slower timings. No physical guard failed.

Trajectory `revision-65dacbc55b96475bb5859b170c027473`, pose audit
`revision-37252ae23ea748c5865c5231af602702`: causal horizontal p95/max
**17.28/26.20 cm**, final **8.09 cm**; orientation p95/max **0.534/0.779 degrees**.
All **13,454 native ticks / 2,244 windows / 82,479,795 points** match transport,
including boundaries. GT associations at exact stamps agree within 0.153 µm.
Strict native first-hit reference gate **fails**: one missing PhysX reference
hit at tick 3,758, stamp 85.2333 s (other 127 residuals max 0.0029 mm). Keep
that failure distinct from GLIM error; this run is not sensor-qualified. No
trimming or relaxation of the reference gate.

Terminal current footprint has no occupied/support-conflict overlap. Goal is
inside the circumscribed wheel-circle exclusion around **layer-capacity** warnings
(reason 6, not a measured obstacle or simply a height-span label),
although an exact stationary wheel query at that goal is clear at headings
0/90/180 degrees. This does NOT prove a reachable path: intermediate turns and
translations must also clear the warnings. Next investigate bounded local
heading-aware progress when the all-heading circle route is unavailable;
retain all immediate sweep vetoes and occupied-cell margins. No goal relocation,
relabeling of warnings, or acceptance from endpoint checks alone.

October 8 offline follow-up: a bounded turn/straight candidate advances about
14 cm on the frozen map then stalls; do not launch it as a solution. An eight-
heading, 0.5 m goal-approach candidate has a valid first command (counterfactual
`revision-c934c4a6e74a4c4da9875b7c9d19cde0`), but its small closed-loop warning-
patch fixture still refuses around 33 cm from the goal. Endpoint clearance is
not a robust tracked path; tightening heading tolerances also did not pass.
These unsuccessful unpromoted v7 edits/helpers/tests were removed, restoring
the recorded v6 controller/config bytes. Keep the failure artifacts and this
ledger; no live v7 run occurred. Next compare the declared wide obstacle-detour
scenario with v6 before expanding into a more elaborate pose-space planner or
changing how layer-capacity uncertainty affects navigation. The square remains
incomplete; neither a different route nor an offline candidate erases that result.

Additional cold storage after v5 shutdown: 11 MCAP segments in September 21/27
closed failed runs `revision-8d5f428e340b460c8336665b5929ba0b`,
`revision-0dfc561521e64c9ca3e40875e766e226`, and
`revision-8ac8159f6dd742718aefd00f581308c1`: **10.00 → 4.13 GiB**, recovering
**5.87 GiB**, leaving **91.64 GiB free**. All original bytes round-trip verified;
sidecars and compressed payload sizes checked after completion. Failures and
all original data remain recoverable with `cold_archive.py restore`; restore
before replay. No cleanup overlapped live recording.

**Permanent old-data cleanup (user-authorized):** remove only `continuous_raw`
from September 9 prototype runs `revision-f880a2ace0c64b3c8d770702387457dd`,
`revision-dcfc046bc93f40dab4aa208e93b42dfb`,
`revision-8b1edba87feb4c60a637249a9b7852d7`, and
`revision-bcf3f5a1a1e3459ca48ce725d4ad5174`: **6,494 NPZ files /
1,894,661,449 bytes (1.765 GiB)**. Exact targets were checked for symlinks/file
types before removal. All 40 selected `raw_scan` snapshots, geometry, reports,
manifests and outcomes remain. Each run has a retirement marker: continuous
replay is no longer available and historical results are not requalified.
These four discarded streams cannot be restored from this archive. One old
run directory was root-owned; restore user ownership of that directory only
to write its marker, then use the verified container archive bind for scoped
removal. Nothing current, native-ray-based, or belonging to frame fixtures was
deleted. Cleanup ran after simulator shutdown, not during capture.

## October 7 — archive cleanup complete: 31.76 GiB recovered, captures unblocked

User requests cleanup because the external drive is mostly project data.
Inventory: this archive uses about **419 GiB**, including **218.45 GiB MCAP**,
**110.92 GiB native binary**, **56.01 GiB NPZ**, and about 30.6 GiB JSON/JSONL.
Drive initially has **76.95 GiB** free. Use lossless cold compression of selected
September recordings instead of discarding successful/failed evidence.
All October recordings, current navigation source runs, libraries and configs
remain hot. Cold recordings need explicit restore before replay; metadata,
recorded hashes and outcomes remain intact.

New `tools/cold_archive.py` checks external storage, closed run, explicit regular
file, 50 GiB reserve, unchanged source, full decompressed SHA-256 and durable
replacement before unlinking only the recoverable uncompressed copy. Sidecar
records original/compressed checksums and restore metadata. Seven tests pass:
round trip including failed-run evidence, no overwrite, unsealed/outside/symlink
refusal, late-writer preservation and corruption rejection. Initial host test
exposed Python 3.10's lack of `hashlib.file_digest`; use streamed SHA-256 instead.

Selected recorder runs (MCAP segments only):
`revision-23da6cb44dc94323879532dfba9ee429`,
`revision-14d466ab65f744b3b12e81dfcae6cc77`,
`revision-8fd5089b22d94887a9a2949f3e690885`,
`revision-e941e62597c24a82a0d827483f2ce0f0`,
`revision-f67480e17a8047c69f36f8bcca48c2fa`.
Paired closed mapper `native.bin` files:
`revision-a06f71a221544d29bedcc8a5eff58854`,
`revision-d79bc8bcc8074ebfa71eb0728a0c2f2f`,
`revision-a3f14a6811f546b6a7cdf98128553196`,
`revision-8a15c7cbd1e3499e8716f4bf9f0d7575`.
The other paired mapper has an empty collection seal and is left untouched.
First MCAP verifies **1,075,291,526 → 380,773,253 bytes**. Final batch completes:
**41 files, 48.03 GiB original → 16.26 GiB compressed**, recovering **31.76 GiB**.
Free space rises **76.95 → 108.71 GiB**. Every file passes full decompressed
SHA-256 equality before its uncompressed copy is removed; all four mapper-file
hashes also match their original collection seals. All unique evidence remains
recoverable, including failures. No source code, reports, manifests, original
native acquisition chunks or October data were removed. Old MCAP/native.bin
paths need the documented restore command before replay.

Final tests: **19 cold-storage/archive-guard tests pass** on host Python 3.10;
`git diff --check` passes. Added cases preserve original data on failed round-trip
checks and reject insufficient temporary space before compression starts.
Post-cleanup inventory verifies all 41 cold payloads exist at recorded sizes,
their hot copies are absent, and the next-route **82 GiB archive preflight passes**.
No capture/replay ran alongside compression; no simulator was started this turn.

## October 7 — longer routes: 5.09 m driven; next live tests ready after cleanup

Current result: the 8 m square reached two corners (about 5 cm GT error each)
and stopped on side three because the planner overinflated uncertain terrain.
It did **not** return home. V4 fixes that refusal on both saved maps and passes
the obstacle-margin fixtures, but is **not live-tested**. The previous storage
blocker is resolved by the cleanup above: **108.71 GiB free**, exceeding the
unchanged 82 GiB startup requirement. Next: fresh v4 square, then obstacle detour
and return-home trials as capacity allows.
See the chronological experiments and exact artifact IDs below.

User requests several longer routes, including turns, obstacles and returning
home. The completed 1/2 m trials below are the baseline, not evidence for turns
or obstacle avoidance. New, explicitly separate `costmap_route_trial_v1` scenarios:

- `costmap_return4`: 2 m forward, turn around, return to the original home.
- `costmap_square8`: 2 m-sided clockwise square, 8 m total.
- `costmap_detour8`: goal 4 m forward and return, with one 40 cm-high,
  60 cm-wide rounded rock 1.8 m ahead; ample room on both sides, no tight corridor.

Goals are fixed once in the initial planar SLAM frame, including exact home;
there is no GT steering or repeated goal re-anchoring. Unknown remains finite
cost, not relabeled free. Keep 5 cm/s forward, separate 0.04 rad/s turns,
40 cm lookahead, 32 cm immediate forward / 5-degree turn hazard sweeps.
Allow a rolling-map partial path toward a goal outside its current extent;
only the actual fixed goal satisfies arrival. New scenarios allow full headings
within a frozen rectangular position envelope; retain 25 cm per-turn drift,
straight-line lateral/heading, footprint/floor, stall, lease and freshness guards.
Do not reset a turn reference merely because an old proposal caused a zero wait.
600 simulation-second controller limit; declared 620 s / 38,000 native-tick
recording cap, 3,600 wall seconds. Old trial limits stay unchanged.

Preflight records underlying terrain even beneath rocks for these navigation
scenarios. This does NOT clear rocks for driving: command-time first-hit floor
and full inflated footprint collision checks are unchanged. Reference geometry
stays evaluator-only; the mapper sees sensor returns, not the surveyed labels.

Acceptance: fixtures for ordered fixed goals/home, rolling-grid continuation,
large turns, newly seen obstacles and unchanged failure guards; then run each
scenario with no manual intervention. Report completed legs, actual travel,
GT return-home error, command/wait time, hazard refusals, emergency stops,
SLAM error and all failures separately. Exact controller/map replay and native
transport/pose audit follow completed captures. This is a navigation experiment,
not production/Nav2 promotion or braking-distance certification.

Preflight host NVML/CUDA allocation passes. Archive has approximately 115 GiB
free, barely above the 114 GiB allocation requirement. Existing recordings are
preserved; permission to remove only redundant replay outputs has been asked
asynchronously. Storage may limit how many captures can start.

Before capture: all **234 focused fixtures pass**, including three closed-loop
ideal-motion scenarios. Initial obstacle fixture correctly refused a goal only
1.2 m beyond the rock center: that destination lies inside the existing inflated
body margin. Move the declared goal to 4 m (8 m round trip); keep rock, robot
margin and hazard policy unchanged. This was a test-layout error, not a reason
to reduce clearance. Isaac results remain pending.

These new bounded scenarios use a **stricter 32 GiB per-artifact quota**, rather
than the generic 64 GiB. The 50 GiB free-space reserve, external-drive checks and
quota stop remain unchanged. Recorder/native/mapper and launch preflight use
the same route-specific value; legacy routes keep their original contracts.
This permits starting with 82 GiB free without deleting evidence. It does not
guarantee that all scenarios will fit; an actual quota/reserve breach still stops.

Final pre-capture regression: **443 tests pass** across route/controller,
advisory/runtime, audit, selected sensor, natural motion, timing and launcher
checks (442-suite checkpoint plus the new recorder-budget case). The first CLI
invocation rejected 3,600 seconds before allocating a run: its old default-profile
cap was still 1,800. Fix uses the explicitly declared new route budget; legacy
CLI limits and all live freshness/heartbeat guards are unchanged. Focused budget
tests pass. First live `costmap_return4` recorder:
`revision-f22fc2ed502b40c58e4650e88aa5f49c`. Host/dev CUDA, archive mount and
sole-recorder preflight pass; result pending.

**First return run fails before motion**, native
`revision-8f516e3340974cfba72a9b6d72e8dff2`, mapper
`revision-a354157b03714df48d57377e9c616ee7`: `trial stale/future proposal` at
sim 17.400000907 s, `motion_commands_sent=false`. Recorder/mapper health remained
valid. `/clock` is published during `world.step()` before the simulator capture
callback updates `owner.stamp`; the independent proposal receiver can therefore
see a clock-stamped proposal ahead of its cached capture tick. This is an
execution-order mistake, not evidence of bad SLAM or a dangerous clock tolerance.

Fix receipt/execution separation: validate message identity, sequence, internal
pose/map ages and checked sweep on receipt, then require the original 0..200 ms
proposal age at the actual execution tick. Both future and expired proposals
produce a recorded zero-command wait. Never retimestamp, extrapolate or execute
a future command. Add a callback-order regression and replay accounting for
future-clock waits. Original validator still rejects future execution. Failed
recording remains preserved; next attempt is a fresh run.

All **76 focused controller/driver/audit tests pass** after that fix. Second
return attempt: recorder `revision-86c1bae9df304bb5b8d236b5aae5af5e`, native
`revision-15c397a54dbe4ffb97aa7fe9c106ce16`, mapper
`revision-e9799ed8aab347f09817ce166dd286cf`. Result pending.

**Second return attempt stops after approximately 1.93 m outward travel**, before
the first goal/return leg completes. Recorder reports `capture storage check
exceeded freshness deadline`; the simulator subsequently loses its acquisition
lease at sim 74.850 s. This is not a completed route or terrain collision.
Temporary `no_costmap_route` holds earlier in the run cleared with fresh maps.
The latest command was a small heading correction near the outward goal.

The recorder issued roughly 13,225 grants over the run and repeatedly scanned
7,115 native files. Successful archive checks reached 85 ms; the failing check's
actual worker duration was not logged, so do not claim a proven disk stall or
misuse the simulator's run-wide maximum gap (again a startup event).
Inspection finds the background `StorageCheck` dropped the existing 100 ms
between-check cadence: it could immediately start another full directory scan.
Restore that cadence inside the worker scheduler, preserving a new exact check
for every grant, single-flight operation, and the original 500 ms deadline.
Record worker queue/work/result-consumption timing to distinguish actual slow
filesystem work from executor delay. No cached success, disabled GC, changed
watchdog or continued failed run. Add scheduling/timing regression, then retry.

**49 storage/capture/archive regression tests pass.** Offline exact filesystem
checks on the failed artifacts: recorder median/max 0.265/0.483 ms; native
13.24/25.64 ms (10 checks each). These are idle-host diagnostics, not live
acceptance; they support reducing needless repeated scans rather than assuming
permanent disk failure. Failed-prefix pose diagnostic
`revision-e37b6b05201243e5a663aae95af36adc` pairs 3,535/3,550 ticks, with all
15 missing shutdown-tail associations retained. Recorded path is 1.985 m;
whole failed route remains unqualified. Orientation maximum is 0.543 degrees.
Fresh third return recorder `revision-8e38cad7439144b9875870a6c65a463b` starts
after host/dev CUDA and unchanged archive/sole-recorder checks pass. Archive
110 GiB free, nothing deleted. Result pending.

Third attempt native `revision-0df6388cadbf40a18f1dd328ff808890`, mapper
`revision-d9a58ae2713945afa50fa0dc03979afc`. At sim 81.67 s the first goal is
reached and home becomes active; actual position is about 1.936 m ahead of
startup. It passes the previous storage-failure point with the restored cadence.
This is a checkpoint, not return-home acceptance; turn/return still running.

**Third return attempt fails at sim 113.533 s during the turn**, heading about
28 degrees from deployment. No collision/terrain refusal; the outward goal was
reached. Storage fix is exercised: 5,759 checks, max worker 95.79 ms, queue
11.10 ms, result wait 19.94 ms; no storage deadline failure. Simulator lease
age reaches 562.95 ms. This time diagnostics locate a **353.66 ms generation-2
GC inside the grant receiver's publisher-info validation**, at the exact failure
tick; native callback totals 423.51 ms. Do not attribute this one to disk scans.

Further code audit: receipt time was obtained AFTER publisher validation/JSON
decode. A grant entering its callback on time can therefore be rejected against
the previous lease after validation triggers GC. New `receive_checked` freezes
the actual callback-entry time, serializes validation/commit with lease readers,
and checks actual completion freshness as well. It cannot revive an already
expired lease, refresh from a duplicate, backdate a late arrival or accept >500 ms
validation. Proposal wall ages likewise begin at callback entry, not completion.
No GC suppression/freezing or deadline change. **73 focused host tests pass**;
new real-DDS fixed-GIL-pause regression and existing scheduling/dead-sender tests
are running before a small Isaac stress capture. Preserve this failed recording.

The broader ROS scheduler regression rejects the added 100 ms archive-check
pause: only 7 fresh results versus the existing minimum 10 in 1.5 seconds under
110 ms synthetic I/O. That pause also reduces lease timing margin. **Withdraw
the pause**, retain the new queue/work/result timing, and preserve the failed
regression result. Do not lower the fixture requirement. The new real-DDS
validation-pause test reproduces the old receive-timestamp failure and passes
with callback-entry timing; full ROS suite is being rerun after this withdrawal.

For the existing short `turn15_gc_stress_v1` diagnostic, declare the same
stricter 32 GiB artifact quota (50 GiB reserve unchanged) on recorder, simulator
and mapper; default legacy routes remain 64 GiB. Forced ticks 240/480/720,
110 ms storage work, all sensor bytes, route and safety thresholds remain the
same. This explicit archive-contract change permits the short stress check
with 103 GiB currently free. It is not nominal navigation acceptance.

After withdrawing the scheduling pause, **38/38 isolated ROS tests pass**,
including real DDS, fixed 350 ms GIL-held validation (old receipt policy fails,
new policy passes), dead sender, late revival, callback scheduling and storage
mutation. 143 other focused host cases pass; the stress-budget fixture initially
omitted the newly declared quota and is updated to assert 32 GiB explicitly,
with forced ticks/I/O/route checks unchanged. That focused case then passes.
Starting the short Isaac stress diagnostic before resuming longer navigation.

**Short Isaac stress completes**: recorder
`revision-3cc9d1124d14406ca6a18ff136ea3ad7`, native
`revision-ee2627d3e2774c388a42b34050a39c2e`, mapper
`revision-420f8c3cdca84a3da0c0d64b323de9bd`. All three forced ticks are present;
full GC pauses **366.17 / 366.19 / 356.36 ms**. Native callback max 431.58 ms;
storage worker max 161.41 ms including the injected 110 ms. No expiry.
Collection completes in 364.73 wall seconds, 1,960/1,960 native ticks integrated,
328 map projections, mapper peak RSS 204.32 MiB. Independent seal verifies all
three payload hashes. Simulator stopped. This exercises timing under deliberate
load; it is not nominal timing or route qualification.

Next navigation revision is explicitly `costmap_route_trial_v2`: same fixed
goals, forward speed, grid policy, all physical/lease guards, 5-degree turn sweep
and 600 s deadline; turn command cap increases **0.04 → 0.10 rad/s**. The failed
return traces show only about 0.008 rad/s actual yaw at the low command, making
full turns needlessly slow; the existing bounded turn protocol already uses
0.10. Old single-goal trials retain their 0.04 cap. This is a declared controller
change, not relabeling earlier failures or altering SLAM/terrain parameters.
Run the route/guard regression suite, then resume nominal navigation.

Stress transport audit matches **1,960 native ticks / 327 windows /
12,449,684 points**, including boundary windows. First ad-hoc transport command
replaced rather than extended sourced PYTHONPATH and could not import rosbag;
corrected environment completes the audit, without changing data.
Final nominal-v2 preflight: **471 host regressions pass**. Next nominal scenario
is the 8 m square, to exercise several corners and exact-home return rather than
spending another long run solely on a 180-degree turn. No forced GC/storage load.
Archive is about 100 GiB free; no artifacts have been removed.

Nominal square-v2 recorder `revision-90ebfc6fb0dd4741b298b013af0d7104`,
native `revision-e6996a3b359242419111eab210c6b26f`. Fresh CUDA/archive checks
and initialization pass; run in progress.

Square checkpoint at sim 119.30 s: first goal and first right turn complete,
driving toward goal 2. Turn displacement stayed below 25 cm (about 16–18 cm
near completion). Current command is forward despite 47 unconfirmed wheel-sweep
cells; explicit hazards remain vetoes. Mapper
`revision-28e42a35ec4e4913af41c40fbae87763`. This demonstrates a real
turn-to-new-ground transition, not yet full square/home acceptance.

Square-v2 then fails at **122.70 simulation seconds** with a false stall: most
of the previous interval was deliberately commanded zero while the route
flickered between available and blocked. The old stall timer counted this
waiting time. It now counts only time with a nonzero command; the original
1.5 s / 5 mm-or-rotation threshold and frozen turn/skid reference remain.
Recorder health passes; normal full GC of **376 ms** survived the receipt fix.
This capture remains a failure, not a completed square.

The first saved route-refusal grid also exposes a planner geometry mismatch:
negative-edge cells 73 cm laterally from the robot were inflated by the full
1.10 m body radius. Missing ground is a wheel-support hazard, not an elevated
body collision. Route revision **v3** uses the circumscribed wheel extent for
negative edges only; occupied cells and other conflicts keep full-body inflation.
The exact immediate wheel/body sweep and GT collision veto are unchanged.
Fixtures cover a lateral edge, an edge under wheels, a rock at the same lateral
position, and a genuine stall after 1.5 s of commanded motion. This explains
the saved early refusal; the final turn's full reason grid has not been replayed,
so do not claim that every later refusal has the same cause. Next: fresh square
with these two scoped fixes, then obstacle detour and home-return trials.

Final v3 host regression: **473 tests pass**. Fresh host/container CUDA and
external archive checks pass. Square recorder
`revision-f176f7a28ade4fb9a811a68d9a135378` starts with 92 GiB archive free;
no artifacts removed, no safety/resource thresholds relaxed.

**Square-v3 stops safely on side three, not at home.** Native
`revision-4323129c731a425982d51e5f3159ecaf`, mapper
`revision-5ea31dff4b0e44649d37fdda39644c19`, exact audit
`revision-a618a9ee4672437190d11b89931aded4`. Collection completes without
lease/storage/physical-guard failure; controller terminates `terrain_refused`
at 194.10 s. **1,807 proposals and every map replay exactly**. Actual travel
**5.091 m**; first two corner errors **5.02 / 4.90 cm**. No home-return claim.
Command times: forward 75.50 s, turn 56.20 s, freshness waits 38.30 s,
other hold 4.62 s. Controller median/p95/max **8.93 / 86.47 / 147.72 ms**.
Consumer-time position error p95/max **18.10 / 43.77 cm**, including pose age;
same-time pose/transport audit is still pending.

Terminal snapshot has no actual body or wheel conflict. The planner rejects its
start because uncertain layer/positive-edge cells about **1.11 m** away overlap
the circumscribed circular inflation. This circle is sufficient clearance for
every heading, not necessary clearance for the current rectangular footprint.
Next scoped candidate: make the outer clearance band a cost, not an unconditional
route veto, while retaining conservative hard centers and the unchanged exact
near-horizon footprint sweep. Validate the frozen refusal grid and adversarial
obstacles before another run. Current archive free space is **78 GiB**, below
the **82 GiB** new-capture requirement. No source/failed artifacts deleted.

Add `audit_completed_route.py` as a bounded wrapper around the unchanged
trajectory and pose/transport evaluators: two 1 GiB diagnostic artifacts with
the original 50 GiB reserve, not the 64 GiB sensor-recording default. Eleven
wrapper/pose tests pass, including retained export/measurement failures.

**Completed square pose/transport audit:** trajectory
`revision-f3f21f02e9f9454eb3f8ee84ff81afa3`, diagnostic
`revision-3f787c86aa1643dca5a0cc7bd90e2dd8`. All **10,838 ticks / 1,807 windows /
66,612,391 points** match transport, including boundaries. Exact-stamp GT pose
agreement is below 0.14 micrometers / 0.38 microradians. Causal horizontal
median/p95/max **8.25 / 17.47 / 43.77 cm**; final error 15.04 cm. Orientation
p95/max **0.495 / 0.733 degrees**. Retrospective corrected poses are better
(horizontal p95/max 13.21/15.05 cm), but were not available causally and are not
substituted. This uses the existing GT-seeded, synthetic-RTK-assisted setup,
not unaided GLIM. Overall diagnostic pose targets remain **failed**.

Native PhysX first-hit reference gate also **fails**: tick 4,595 at 93.0667 s
has one missing reference hit out of 128 selected returns; the other 127 agree
within 2.4 micrometers. Preserve this result and the 1 mm threshold. It is not
SLAM error and did not cause the controller stop. No full terrain/sensor
qualification claim follows from this navigation experiment.

**Planner v4, offline-tested only:** an initial inscribed-body-radius candidate
completed the ideal obstacle route but violated the existing conservative rock
clearance fixture (1.129 m versus >1.401 m center clearance). Reject that change;
do not weaken the fixture. Keep the original full-body circle around occupied
cells. Instead, apply wheel-extent clearance to *all explicit unknown support
conflicts*, not just negative edges, matching `PreparedMotion` semantics.
Those cells remain warnings; actual wheel intersection still vetoes motion.
Occupied cells retain the same body margin and exact body sweep. No terrain
labels, map thresholds, immediate swept geometry or GT guard are changed.

Sealed-map counterfactual `revision-eccfa673bd2c41a0945f3b738cc5a95e` checks
both refusal snapshots against their original map/proposal hashes. V3 finds
no route in either. V4 proposes **0.05 m/s forward** at the first refusal and
a **-0.10 rad/s checked turn** at the terminal refusal. Frozen-query timings
median/p95/max: first **5.50/8.59/16.32 ms**, terminal **51.35/62.11/66.81 ms**.
These are 20-repeat diagnostic timings, not live compute acceptance. A valid
first command does not establish the unexecuted rest of the square.

Diagnostic development: the first host invocation lacked the container's
`/archive` alias and allocated nothing. Container diagnostic
`revision-5056cd910b544317b5592bd2cb4d3bcd` initially compared the current
consumer pose against the map-evidence pose and correctly stopped on a mismatch.
Fix checks grid/frame against sealed projection and consumer pose against the
separately sealed proposal, exactly (no tolerance/fitting); regression rejects
both pose substitution and changed grid bytes. Failure artifact is retained.

Live v4 square, obstacle detour and return-home tests remain **NOT RUN**.
Read-only next-capture preflight reports **76.95 GiB available / 82 GiB required**;
this is now the external blocker. Simulator and replay containers are stopped.
No cleanup performed. Need additional external archive space or explicit
direction on removable artifacts; retaining 25–40 GiB additional room would
allow more useful attempts than merely crossing the startup threshold.

Final verification: **491 broad host tests pass**, then **37 focused tests pass**
on the final diagnostic revision (including the new sealed-consumer-pose case);
**38 isolated ROS tests pass** on domain 179. Commands: host pytest with pinned
NumPy/SciPy/Shapely via the runbook environment; `test_costmap_scenarios.py`,
costmap/contact/controller/route/archive/timing suites, plus
`test_audit_completed_route.py` / `test_measure_exported_glim.py`; isolated
`test_lease_callbacks_ros.py`, `test_capture_polling_ros.py`,
`test_shadow_capture_ros.py`. Exact live-v3 audit command was
`assess_step_trial.py /archive/revision-5ea31dff4b0e44649d37fdda39644c19
--archive-root /archive` before changing the controller to v4. No live-v4,
full-square, obstacle-detour or home-return acceptance is claimed.

## October 6 — planner costs and path evaluation

### Unknown-permitted costmap navigation (1 m and 2 m normal-terrain trials verified)

**Result:** continuous costmap-driven motion works on the tested normal terrain.
The 1 m trial travelled 0.97 m; the final 2 m trial travelled 1.92 m. Both stopped
at their estimated goal tolerance and passed exact controller/map replay. Known
hazard reactions have fixture coverage; live obstacle avoidance is still next.

User explicitly requests driving through unobserved cells with fresh hazard
checks, not requiring every wheel cell green. This changes the **experimental
navigation policy**, not raw terrain labels or the original contact/step trial.
The offline polygon-heavy planner is not a live navigation acceptance gate.

New opt-in `costmap1` / `costmap2`: choose once a measured/inferred-free cell
within 30 cm of a nominal 1 / 2 m forward destination after three seconds of
observation. Unknown carries cost 30 on a 2-D grid; known occupied and explicit
ground conflicts block routes. Inflate with the unchanged body's circumscribed
radius and cell-area margin; 8-connected A* prohibits corner cutting. Track a
visible route segment with 40 cm lookahead, 5 cm/s translation and separate
0.04 rad/s turns with heading hysteresis. No 5 cm temporary arrival targets.
Each command checks a fresh 30 cm translation plus the original 2 cm experimental
reserve, or the original 5-degree rotation sweep. Full body obstacles, wheel
conflicts and outside-grid remain vetoes; absence of positive ground support
is explicitly permitted. This is a risk policy, not proof of free ground.

Original capture lease, sole publisher, native timing, 0.5 s wall / 0.2 s
simulation proposal age, 0.5 s map / 0.2 s pose age, GT emergency footprint/floor,
heading/lateral/skid/stall guards remain. Survey and maximum displacement are
declared separately for the new longer route, never changed in `terrain_step`.
GT remains one-time initialization plus evaluator/emergency veto, not steering.
Goal tolerance is 10 cm; time limit 120 simulation seconds, total capture budget
1,800 wall seconds. First test is normal seed-43 terrain, no inserted obstacles;
it tests progress, not detection of an approaching obstacle. Retain all attempts.

Acceptance for this cycle: controller regression fixtures, then a completed
guarded 1 m run with independently measured progress and exact map/controller
replay. If successful, repeat at 2 m. Report actual goal error, unknown exposure,
turn/wait/forward time and controller computation separately. Hazard-insertion
fixtures test immediate withdrawal; unseen-obstacle live response remains a
separate test, not implied by a clear-terrain arrival.

First launch: recorder `revision-e88a189162024a4998dcfce0f4336a7f`, native
`revision-b12ebf9d44204720910d6751906c6993`, fails before motion because the new
validator module imports mapper-only Shapely in Isaac. Move that import into
the mapper-side update method; keep the validator dependency-light. Preserve
this failure and start a fresh capture after regression checks.
Dependency regression now covers validator import without Shapely; **142 focused
tests pass** including original controller/driver and new route/unknown/hazard
withdrawal/pose-noise/frame/freshness cases. Second recorder:
`revision-97f07be837fb4d1796173acdca4ca06a`; unchanged declared `costmap1` protocol.

**1 m live trial completes and exact replay passes.** Native
`revision-3cd6e3c05ca24381bbdb4525c5e412c5`, mapper
`revision-36247dd797da4f47b0473c058c4fbf1e`, exact controller/map audit
`revision-f22dd323f73042ca82f8061e766113ca`. All 373 proposals and raw maps
reproduce; outcome `estimated_goal_reached`. GT displacement **96.92 cm**, GT
distance to the fixed goal **9.50 cm**, estimated remaining **9.57 cm**. Commands:
16.17 s forward, 7.87 s turning, 7.05 s fresh-proposal waits, 0.18 s other hold.
No terrain refusal or emergency guard trip. Of 373 maps, 218 have at least one
fully LiDAR-confirmed wheel patch; 310 moving proposals still include unknown
wheel cells, permitted explicitly rather than relabeled free. Live controller
(2-D search plus hazard sweep) median/p95/max **6.96/12.49/15.93 ms**. These
instrumented timings do not replace full resource qualification.

Native/transport/pose diagnostic `revision-ef205cf8b2bc4e7b87611557bb4a6b69`
passes surface association and all 2,237 native ticks / 374 windows /
14,045,740 points. Trajectory export `revision-b9ff40fbb5ee400899296624119fe2d7`.
First-published same-time GLIM horizontal p95/max **6.59/8.24 cm**; the report's
stricter diagnostic pose target remains false, not silently promoted to pass.
No fitted alignment or retrospective poses used by the controller. **366 host
regressions pass** on this revision. Simulator stopped before replay.

Before the 2 m extension, inspection finds a planner-only boundary mistake:
inflating the edge of a 6 m map as though it were a wall can reject a 2 m goal
despite no actual obstacle. Treat outside-map footprint area as unobserved for
tentative routing; keep route centers in the map. The independent immediate
body/wheel sweep must still stay inside the available map. Add both regression
cases. This changes route-edge handling, not the local hazard/stop envelope;
the completed 1 m result above retains its original hashes and audit.
Both boundary regression cases pass. The 2 m extension is running with recorder
`revision-e02f5a0408e64d09b640cff76ebac2b6`, native
`revision-8380bf8d3f2e4251b0c6b86506e0cfcc`, mapper
`revision-f21ce1c4e702475ba27589b5495f0b2c`. Same scene, sensor, controller speed,
local hazard envelope and original guards; only declared goal/survey length and
the documented tentative-route boundary correction differ from the 1 m run.

**First 2 m attempt fails on the mapper heartbeat, not navigation refusal.** At
sim 58.15 s mapper revokes output and the driver stops. The initial diagnosis
mistook the run-wide **1.269 s** maximum tick gap for a pause at the failure;
the timing follow-up below corrects that attribution. Last commanded GT position is about
**1.60 m** ahead of startup; estimated goal remaining **33.7 cm**, still requesting
forward motion. This is failed-run diagnosis, not completed-route scoring.
Controller median/p95/max **6.82/11.25/13.25 ms** across 386 proposals. No in-motion
GC pause above 50 ms; max native processing **91.4 ms**. Similar to the preserved
earlier 4 m capture, existing diagnostics cannot place the rest of the pause.
Stop the owned simulator; retain all evidence. Do not silently retry or enlarge
the 500 ms heartbeat.

Next hypothesis-driven run adds bounded passive stage timing around world step,
selected sensor read/adaptation, GT publishing, native audit/archive, quota check,
guarded command execution and loop pacing. Counts/maxima and the twelve worst
intervals per stage go into the existing final acquisition timing JSON, not
per-tick files. Source is hashed. Physics, sensor bytes, GC and guard thresholds
are unchanged. Re-run `costmap2` to either finish or locate the stalled stage.

**Timing run exposes our own callback bottleneck, not a simulator stall.**
Recorder `revision-37a7de7c3d8a4656a323eb40068d8f25`, native
`revision-78abdac6642349e594978de0d0c59a58`, mapper
`revision-c365872e3dbd4266b13dbff87d4bb976` fails at sim 48.97 s. Its run-wide
1.160 s gap belongs to pre-request setup: startup capture tick 1.134 s at sim
10.97 s. During motion, world-step max is 64.2 ms (including startup), native
processing max 97.1 ms, driver max 17.1 ms, archive check max 15.7 ms. No long
in-motion GC pause. The final mapper clock callback spends **410 ms rebuilding
contact support history / 444 ms total advisory work**, on 194 history anchors.
Its next event is its own heartbeat fault **513 ms** after that clock receipt.
The preceding failed 2 m run similarly ends with **435 ms** advisory update and
230 anchors. Terrain core max in the instrumented run is 62.8 ms. Thus the
old diagnostic support-history work blocks the same executor that receives
clock messages. Do not infer simulator hangs from a run-wide maximum again.

Scoped fix: new unknown-permitted trials use `HazardAdvisory`, preserving
current-context frame/session/revision/pose/map-age checks, full body/wheel
hazard checks and observed-wheel reporting, but **not building the unused
historical support unions**. Their commands already did not consume that history;
this does not relax the new navigation policy or any timeout. Legacy positive-
support trials retain `ContactAdvisory`. Manifest explicitly identifies the
hazard-only mode; exact replay restores it. Regression monkeypatches forbid
history construction in this mode and verify stale-context invalidation.
All 80 focused advisory/controller/replay tests pass before the next run.
Fresh same-protocol 2 m run after this fix: recorder
`revision-4adc4584372d4152bc89f759564964bf`, native
`revision-67bbaed1a3aa486ea998c91eff057c32`, mapper
`revision-7903b7901db04276a507f76dcc38dcae`. This run **completes**; neither old
failed run is continued or relabeled successful.

Exact controller/map replay `revision-34ab154b97e14f8eb93c001d1d315dd7` compares
all **510 proposals**, 2,696 execution ticks, and every raw map; outcome
`estimated_goal_reached`. GT net/forward/cumulative distance **1.924/1.924/1.943 m**.
Estimated distance remaining **9.64 cm**. Actual distance to the selected fixed
goal is **0.34 cm in this particular run**, not a repeatable precision claim:
consumer-time pose error p95/max is **10.98/14.14 cm**, and the controller stops
on estimated, not GT, arrival. Selection allows a 30 cm offset from the nominal
2 m destination; do not present displacement as exactly 2.000 m.

Executed command time: **30.92 s forward, 4.72 s turning, 9.10 s waiting, 0.18 s
other hold**, with no terrain refusal or emergency veto. Controller planning
plus hazard check median/p95/max **6.31/10.98/18.50 ms**. Hazard-advisory update
median about **4.9 ms**, max **42.7 ms**; total including the five once-per-second
inspection queries max **57.9 ms**, versus the failed 444 ms historical-support
update. Existing proposal-age limits still cause occasional waits; no stale
proposal is reused as fresh. No simulator or mapper heartbeat fault this run.

Raw terrain remains unchanged: **361/510** map updates contain at least one
fully classified wheel patch. Of **447 moving proposals**, **227** have no
unknown cells in the complete checked wheel sweep and **220** explicitly permit
unknown cells. Thus the robot actually transitions from startup missing-ground
coverage into classified support; this is not solely a moving safe-area prior.
Both successful runs use one-time GT initialization and synthetic RTK assistance;
ongoing GT steering is absent and the independent emergency clearance veto stays.

**380 final host regressions pass**, including both navigation controllers,
unknown/known-hazard transitions, closed-cell route visibility, map-edge handling,
frame/freshness failures, no-history hazard runtime, exact-audit fixtures,
natural-motion/lease guards and bounded timing. Final simulator is stopped.
Final trajectory export `revision-9cf6e03c6dad4a85bb0832288df28059`; native/pose
audit `revision-75f558dc0024496e8f476eb1771eadf1` completes. Surface/reference
association and transport checks pass across **3,056 native ticks, 510 windows,
19,121,337 points**. Same-time first-published GLIM horizontal p95/max/final
**11.17/14.43/10.89 cm**, yaw p95/max **0.366/0.520 degrees**. No fitted alignment
or future-revision use. Stricter diagnostic pose targets remain false; this is
successful short navigation with bounded observed error, not perfect SLAM.
Black passes for all twelve new/affected formatted Python files; root/submodule
whitespace checks pass. Both completed trials retain their source/config/input
hashes; failed launch and both failed 2 m attempts remain archived. No capture,
replay or simulator process is left running.

Next navigation test: an ordinary open detour around a detectable rock, then
several successive goals and a longer route. Measure detection lead distance,
GT clearance, unexpected stops and actual arrivals. Do not substitute a narrow
contrived rock corridor or demand every unknown cell be filled. Hole detection,
obstacle-response latency and stopping allowance still need dedicated moving
tests; clear-ground arrivals alone are not exploration safety qualification.

### Size the navigation bottleneck (recorded diagnosis complete; controller changes not yet made)

User asks whether this will make steady exploration progress or repeatedly snag.
Do not change the controller, terrain policy, freshness limits or scene during
diagnosis. Use the completed normal-terrain trial, its exact execution audit
`revision-29700cc569574c1b99cfc431808d7cb3`, and final two-horizon assessment
`revision-f8fda644b5944ab59469b59b06265ece`. Measure command time (not just
proposal counts), denied forward versus turn time, current-pose discrepancy,
and local-prefix dependence on observed versus separately credited support.
On the same ten frozen planning snapshots, diagnose the first locally rejected
route edge and whether 1 cm subdivision would recover appreciable extra range.
This is diagnostic subdivision, not a changed live clearance rule or claim of
longer executed motion. Profile the first forward 2 m success and first forward
2 m search-limit failure; profile timings are not compute acceptance. Also time
the existing fresh 7 cm sweep query separately from full route planning.
Preserve every original case, goal and outcome. No new motion commanded.

Result: `revision-3b9e594c5c63449289bef22801f03026` completes exact replay of
all 483 maps/advisory outputs and checks all ten planning snapshots. New tool:
`tools/assess_navigation_bottleneck.py`. This measures the existing 30 cm trial;
it does **not** claim the new long-goal planner has driven the robot.

**The robot mostly corrected its heading instead of driving.** Of 42.30 seconds
between the first and last execution ticks (2,539 ticks, no invented terminal
interval):

| Executed command state | Seconds | Share |
| --- | ---: | ---: |
| Turning | 22.72 | 53.7% |
| Waiting for a fresh proposal | 10.82 | 25.6% |
| Driving forward | 8.07 | 19.1% |
| Other zero-command hold | 0.70 | 1.7% |

Actual forward progress was 20.48 cm; the trial timed out. Terrain refused
**none of the 483 proposals**. The old controller uses 5 cm temporary targets,
a 2 cm arrival tolerance and stops translation above about 2.9 degrees heading
error. Previously audited one-second relative horizontal pose error p95 was
5.45 cm: comparable to an entire temporary target. Even modest position error
can therefore make the apparent direction to such a nearby target swing widely.
As an independent symptom, 20.25 of the 22.72 turning seconds commanded the
opposite turn sign to the same fixed goal's bearing evaluated from GT (excluding
GT heading errors below 0.05 rad). This is not large yaw drift, a recommendation
to steer with GT, or proof of the effect of each command on skid dynamics.
It supports testing continuous tracking of a longer route instead of repeatedly
trying to achieve tiny pose targets.

**Freshness waits are a separate scheduling issue, not terrain refusals.** All
649 waiting ticks exceeded the existing 0.5 s wall-age limit; their proposal
simulation ages were only 0.05–0.10 s, below the 0.2 s simulation-age limit.
The recorded simulator tick gap never exceeded 16.67 ms of simulation time.
Wall ages during waits reached 1.06 s. The live code publishes proposals with
map updates, so slowed simulation/host scheduling can leave a recent-in-sim-time
proposal too old in wall time. These logs do not separate computation, callback
queueing and delivery delay; timestamp those stages in the next run. Only
**2.92 s** of these waits were denying a forward proposal; 7.63 s were denying
turns. Removing all waits alone would not solve the excess turning. Do not
relax the watchdog or repeatedly republish an old decision as a fresh check.

**The short available paths do not yet demonstrate sensor-supported travel.**
Across all 44 returned prefixes, zero swept wheel area overlaps raw measured
or inferred-free cells. They rely on separately credited startup/current support,
including the agreed turning-area assumption. Independently loading the hashed
archived grids and sampling wheel-patch interiors at every returned pose also
finds all 90,024 samples unknown. This cross-check is not a replacement for the
continuous sweep. Every first rejected extension fails for missing new ground,
not an obstacle or drop-off. Subdividing just that rejected translation into
1 cm pieces recovers a median **7.5 cm** (range 0–10 cm), showing some of the
short horizon is coarse 10 cm path discretization. This is diagnostic only;
it is not additional observed ground or a changed live policy. The recording
does not travel far enough to establish the transition onto previously seen
ground. Do not infer a five-second forgetting problem: raw buffering is five
seconds, but this run's persistent terrain evidence lifetime is **60 seconds**.

**Separate route planning from the fast motion check.** Ten isolated existing
7 cm motion queries take median **4.04 ms**, p95 **4.45 ms**, max **4.60 ms**.
That excludes map/support-index updates and is not live compute qualification.
The original unprofiled route-plus-prefix p95 is **507 ms**, max **863 ms**.
Profiles of the first forward 2 m success and first forward 2 m search-limit
failure reproduce the original paths exactly and show substantial swept-footprint
geometry cost. They also reveal 480 / 2,810 temporary-file creations taking
35 / 178 ms of 220 / 1,613 ms instrumented route time. A separate one-hull probe
confirms the installed SciPy `ConvexHull` creates a temporary file internally;
these are not thousands of retained ROS capture files. Repeated hull construction
and template reuse are optimization candidates, but file creation alone accounts
for only about 11–16% of these two profiles. Do not call profile timing a gate
or assume removing this overhead solves the controller behavior.

Next implementation/test cycle, in priority order:

1. Add a separate opt-in long-goal controller: fixed 1 m observed-free goal,
   continuous route tracking with lookahead, locally checked commands, then the
   same declared protocol at 2 m. Do not reuse 5 cm position-arrival targets as
   the tracking objective. Test pose perturbations at the measured error scale,
   hazard withdrawal, goal overshoot and stale/missing inputs before motion.
   Keep the original 30 cm protocol and its failed result unchanged.
2. Put bounded route search outside the mapper/control callback. Reuse a route
   only while its frame/revision remains valid; recheck the next motion against
   current pose/map/support. Record map production, local-check completion,
   proposal reception and execution times. Preserve the actual freshness limits,
   stopping reserve, full body/wheel hazard veto and sole command ownership.
3. In fresh guarded runs on the same normal terrain, measure goal arrival,
   actual distance, forward/turn/wait time, command refusals, tracking error and
   how much future wheel ground was seen **before entering the near blind area**.
   Separate measured support, contact history and the moving turning-area prior.
   Repeated availability from a recentered prior is not evidence of improved
   sensing. GT stays evaluator-only except declared initialization/emergency
   guards. Do not demand that every wheel cell be directly visible at all times.
4. Advance to longer routes only after actual 1 m and 2 m arrivals with no
   collision/reference hazard intersections, guard violations or missing evidence,
   and transparent accounting of assumed support. Repeat on held-out headings
   and normal-terrain seeds; retain all attempts. If progress still stalls,
   distinguish unsupported new ground from tracking/scheduling before changing
   terrain rules. Optimize repeated geometry if measured deadlines still require
   it, with exact-output and applicable resource regressions.

No live controller, sensor, SLAM, terrain thresholds or freshness rules changed
in this diagnosis. No new simulator motion was run.
Validation: **177 host tests pass** across the new diagnostic, horizon/cost
planners, local costmap, swept support, contact advisory/index/runtime/replay,
step controller/audit and cost-report contracts. Command uses the runbook's
pinned NumPy 1.26.4 / SciPy 1.11.4 / Shapely 2.1.2 host environment; test files
are `test_{navigation_bottleneck,horizon_planner,cost_planner,local_costmap,
local_support_sweep,contact_advisory,contact_index,contact_runtime,
contact_support,contact_replay,step_trial,assess_step_trial,navigation_cost,
cost_report}.py`. These tests and the completed recorded replay are not a new
moving-robot acceptance result.

### Two-horizon route intent and executable prefix (offline screen complete; live execution not tested)

User requests a 1–2 m goal on observed free terrain, a tentative route through
free or unknown cells, and locally supported motion along that route. This is
not permission to drive the entire unknown route. Implement a separate opt-in
planner: missing-evidence unknown has cost 90 for route intent only; occupied
and explicit edge/ground conflicts remain vetoes. The local prefix uses the
unchanged isolated-gap costs, full wheel/body sweep and actual-pose support
index. Never move the assumed turning disk onto hypothetical future nodes.
Use a maximum 0.5 m checked prefix, preserve a 2 cm experimental stopping
reserve inside it, and report usable translation separately from available
in-place turns. These are diagnostic parameters, not a braking certificate.

First test: exact-replay completed normal-terrain mapper
`revision-d8d341965f80425a8cfdd2fbc950c391`. On the first map choose once the
nearest measured/inferred-free cell within 30 cm of each nominal 1 m and 2 m
cardinal goal. No GT or route success enters selection; missing goals remain
counted. Hold goals fixed in the recorded local/map reference. At every fixed
five-second bin compare the old full-route local-support requirement, tentative
route, and locally checked executable prefix on identical maps. Evaluate full
route and prefix separately against independent rocks/floor and same-time GT;
retain missing GT, failures, every bin, and source hashes. A returned prefix
does not establish reaching the long goal. Test unknown barriers, occupied and
drop-off overrides, unknown goal rejection, stopping reserve, future-support
leakage and map/frame boundaries before recorded replay. Only after this screen
should an explicitly guarded live long-goal controller be considered; the old
30 cm protocol and its failures remain unchanged.

Initial analytic tests pass (19 including existing planner contracts). First
recorded attempt `revision-2dc0faf5bd8e45a09825e4b2cf2051ea` stopped before
any scored maps because adding an endpoint predicate to the shared planner
changed a source hash frozen by the live recording. Restored that shared file
exactly; the new wrapper instead selects a known-free lattice endpoint inside
the unchanged 7.5 cm goal tolerance. This keeps the requested goal fixed and
the old runtime/replay untouched. No replay guard was bypassed or weakened.

Immediate-first-map screen completes: `revision-2a2d09f8d7d54d61b632ca4918ca0255`,
483 exact maps, ten bins, 80 goal/time combinations. Only two initial 1 m goals
have observed-free destinations; all six missing goals remain counted. The two
goals have tentative routes **and usable local prefixes in all ten bins**;
the former full-route local-support requirement finds none. All 18 GT-matched
prefixes and full routes avoid reference rocks and remain on surveyed floor;
two per stage lack same-time GT. This is counterfactual planning, not execution.

Failure-driven next variant: the first map contains only 132 free cells; at
five seconds it has 775, with observed free ground near front/left/right 2 m
targets. Declare `--goal-selection-delay-s 5`: select once after five seconds
of recorded observation, then hold goals fixed. Keep the startup bin as
`awaiting_initial_observation`, all 80 combinations, the same map/pose sequence,
thresholds and evaluation. This is a separately reported observation-start
protocol, not deletion of the first experiment's missing goals.

Five-second observation-start result `revision-dc6414e2253e4aa9a837908088b4a405`:
six initially free fixed goals (front/left/right, 1 and 2 m); both rear goals
unavailable and retained. Of all 80 goal/time slots, eight are observation wait,
18 have absent rear goals, two lose destination free classification later,
eight exhaust the original 4,000-state search and **44 return a route plus a
usable local prefix**. The 1 m goals succeed in all 27 post-start combinations;
2 m succeeds in 17/27. At least one 2 m option exists in every post-start bin.
All 44 prefixes have same-time GT and pass independent rock/floor checks. Full
routes have zero rock-bound intersections, but 17 extend outside the floor
survey: unverified, never full-route safety acceptance. Local usable distance
is 8–22.14 cm including the retained reserve. No path in this experiment drove
the simulator or established reaching a goal.

Inspecting the eight search-limit failures shows **five have no permissible
arrival footprint in any of the eight planner headings**, despite a free center
cell; three have a valid endpoint but no route found within budget. Add a cheap
necessary endpoint-footprint check in the new wrapper only. It must preserve
every previously found route/prefix exactly and report those five as unavailable
goal footprints rather than wasting the search budget. The remaining three
bounded-search failures are retained, not renamed as terrain impossibility.

Final replay `revision-f8fda644b5944ab59469b59b06265ece` completes all 483 raw
maps and exact original advisory outputs. The endpoint precheck preserves **all
88 returned full-route/prefix paths, every fixed goal, every query stamp and
all their independent truth results exactly**. Only the five identified
failures change from search exhaustion to unavailable goal footprint. Other
failures remain: three bounded searches, two later unknown goal cells, both
unobserved rear goals and the initial observation wait.

For the six selected front/side goals after observation:

| Goal distance | Route plus usable prefix / selected goal-time pairs |
| --- | ---: |
| 1 m | 27 / 27 |
| 2 m | 17 / 27 |

The forward 2 m goal works in 8/9 samples. Across all requested slots, it is
still **44/80**, including missing destinations and startup wait. All **44/44**
prefixes reduce direct distance to the same fixed goal by **7.55–18.90 cm**;
usable path lengths min/median/max are **8.00/10.07/22.14 cm** after the 2 cm
reserve. This is not a new 10 cm navigation goal: the prefix belongs to a fixed
1–2 m route and must be reconsidered on each fresh map. Local coverage continues
to rely on the explicit current turning-area assumption, not only observed
ground. No isolated-gap cells qualify on this recording.

Final candidate's offline route p95 is **323 ms**; route plus prefix p95
**507 ms**, max **863 ms** across all requested slots. This is not compute or
10 Hz controller qualification. The full route search must not be run in the
mapper/control callback; a live integration needs a separate bounded worker
and a fresh local swept-motion check before every command. The old tiny-target
controller is not modified or claimed fixed by these offline results. Actual
1/2 m goal arrival, stopping behavior and changing sensor coverage remain the
next **live** experiment, with its own declared route/driver envelope; the
existing 30 cm experiment must not silently be extended.

Interactive comparison of immediate vs five-second goal selection:
`revision-39ac611e7ef849a089e965a75b041de7/report.html`. Gold cross = fixed goal;
faint route = tentative future motion; bright prefix = locally supported part.
All 480 dataset/time/policy/goal selections pass a JavaScript/DOM smoke test
(not visual-browser inspection). Reports distinguish full routes extending
outside surveyed floor from the 44 independently checked local prefixes.
**166 affected host tests pass**, including original cost/contact/sweep/trial
contracts, fixed-goal selection, observed endpoints, hidden unknown strips,
stopping reserve, current-assumption isolation, hazard withdrawal, footprint
precheck and equivalence. Black and root/submodule whitespace checks pass.
Existing shared planner source is restored byte-for-byte (`cc8b6862...`).
No ROS live node, sensor, GLIM settings, native capture or command policy changed
this cycle; no fresh moving-robot test is claimed. All experiment containers
finished and the simulator remains stopped.

### Normal-terrain current-turn-area trial (short trial verified; longer capture incomplete)

User clarified that tight rock corridors are not the navigation requirement;
unknown cells within rock-avoidance margins need not be cleared. The previous
runtime only credited a startup turning disk and current wheel patches, not
the agreed turning area at the current robot pose. Add opt-in
`--contact-current-turn-area`: the same 61.07 cm nominal turning footprint
is placed once per actual causal consumer pose. It is never centered on a
hypothetical future pose, written into raw terrain labels or saved as observed
history. Known occupied and explicit ground/drop-off conflicts still veto.

First normal-terrain experiment: repeat the original `terrain_step` 30 cm
goal on natural seed-43 terrain, no added demo rocks, unchanged mount/SLAM,
full wheel/body guards, 2 cm/s commands, stopping allowance and acquisition
lease/GT emergency checks. Only the current-turn assumption changes. Compare
terrain refusals, actual GT progress, estimated completion and tracking behavior
against the earlier 22.8 cm baseline. Goal progress, no hazard veto and exact
replay are separate results; completion alone cannot qualify navigation.
If controller jitter still dominates, diagnose it separately before a longer
normal-terrain route. Old adapter sources archived before modification:
`revision-f94c3cabf24342d58c7d74a78487f4fc`,
`revision-21542e071535446daed197424458fa2d`,
`revision-d0b41ae87db04d11bcd7d68d09d482bb`.

Pre-run verification: 140 focused host tests and 31 sourced ROS tests pass;
rebuilt the mapper/launch packages. Regression fixtures cover actual-pose
movement of the assumption, no future-path expansion, no assumed history,
occupied/drop-off override, stale invalidation and recorded replay policy.
Previous trial replays exactly with the new option disabled:
`revision-12f891602d4a4f29be3d555296292c62`, all 483 controller proposals and
2,534 execution ticks preserved. Host/dev CUDA allocation and archive-bind
preflight passed. New recorder: `revision-53060e754aec41259259abf824992371`.

The new trial completed collection with **483/483 terrain decisions available**
(baseline: 91 refusals). Native `revision-5f30a81b9e3e4216b014c62c6e9dae1c`,
mapper `revision-d8d341965f80425a8cfdd2fbc950c391`. It still reaches the 45 s
controller limit rather than the goal: 311 proposal updates turn, 129 advance,
43 stop. A read-only consumer-pose audit shows yaw error below roughly 0.4° at
sampled five-second checkpoints but sideways translation discrepancies up to
about 9 cm. The point-target controller steers toward that displacement noise;
this is not a terrain refusal. Exact full-run audit follows; do not claim goal
success from the improved terrain availability.

Next declared normal-terrain diagnostic: existing `straight_slow` 2 m out +
2 m back route, no inserted rocks, with current-turn-area advisory and the same
sensor/SLAM. This is **GT-guided guarded acquisition**, not terrain-controlled
navigation. It separates sustained-motion map/path availability from the known
short-goal controller mismatch. Score every recorded map and fixed five-second
planning bin; add 0.1/0.2 m goals while retaining all original 0.3/0.5/1 m goals
in each cardinal direction (`assess_cost_paths.py --include-short-goals`).
Occupied/drop-off checks and GT-only evaluation remain unchanged. Do not
present this as fixing the controller or as autonomous exploration.

First trial exact audit `revision-29700cc569574c1b99cfc431808d7cb3`: all
483 proposals and 2,539 execution ticks replay/check exactly; actual forward
progress **20.48 cm**, cumulative XY travel 23.92 cm, estimated goal still
17.20 cm away. Zero terrain refusals. Current assumption fixes the terrain
restriction but does not improve goal tracking by itself. Consumer-time XY
error p50/p95/max 10.28/14.31/16.05 cm includes pose age; it is not same-time
SLAM error. Original baseline had 22.80 cm forward progress, so do not claim
an overall progress improvement from the changed terrain assumption alone.

Completed normal-terrain path assessment: `revision-21ec586f21224d3aa051a96a8ba2b321`.
All **483 maps** replay exactly, with ten fixed five-second planning samples.
Both strict and isolated-gap cost policies return the same results:

| Goal distance | Paths found / 40 direction-time pairs | Samples with any path / 10 |
| --- | ---: | ---: |
| 10 cm | 40 | 10 |
| 20 cm | 18 | 9 |
| 30 cm | 0 | 0 |
| 50 cm | 0 | 0 |
| 1 m | 0 | 0 |

All ten samples have a 10 cm route in the forward test direction, as well as
the other three cardinal directions. No isolated-gap cells qualified; the
current turning-area support, not hole filling, makes these short paths possible.
Of the 58 returned paths per policy, **54 have a same-time GT start**; all 54
avoid independent rock bounds and remain within surveyed floor. Four paths
remain GT-unverified, not successes. This checks perfect relative path tracking,
not actual execution of those planned paths. A 10 cm goal has the unchanged
7.5 cm endpoint tolerance; reported distance is the requested goal distance.
The longer-path failures are not proof that incremental navigation is blocked:
the planner deliberately does not move the assumed support disk into future
poses. Equally, repeated one-step availability does not prove sustained progress.

Normal-terrain-only interactive report:
`revision-ac40b9d352384612a67c73f238d10a41/report.html`. It shows every sampled
grid/goal, a green outline for the current support assumption, separate strict
cost grids and per-distance results. All 400 selector combinations passed a
JavaScript/DOM smoke check (not a visual browser inspection). Cost construction
p95 **0.63 ms**; bounded Python search p95 **515/546 ms** for the two policies.
These are offline timings. The search is not a qualified 10 Hz live planner.
The support overlay is consumed by this planner/advisory, not painted into raw
terrain or automatically understood by an unmodified Nav2 costmap consumer.

Independent completed-trial pose/sensor audit:
`revision-2c4ba89de2b544489e0bbef2572efc16`, trajectory export
`revision-a2ca19c2a0df44ebbc29a44932a1b8e9`. All **2,899 native ticks**, 484
transport windows and 18,334,037 delivered points checked; native first-hit
reference gate passes (371,072 samples, maximum 0.046 mm). Same-time published
GT agrees with native GT to 0.00003 mm; frame comparison is physical base_link,
fixed home, no fitted alignment. First-published horizontal pose p95/max/final
is **14.14/16.02/7.68 cm**, yaw p95/max **0.353/0.567°**. One-second relative
horizontal error p95 is **5.45 cm**, versus the controller's 5 cm step targets
and 2 cm goal tolerance. This substantiates the tracking-scale mismatch;
absolute diagnostic pose targets still fail. Retrospective revised results
are recorded separately and are not substituted for causal control inputs.

The declared 4 m out-and-back attempt is **incomplete**:
recorder `revision-502d4d43c2054ecfa1d584e6df2b8ff7`, native
`revision-dfecef428be645298f5a5c4dca6fc271`, mapper
`revision-2d2307efa9304f94b9c484be52541811`. The robot reached the outbound
2 m point and started returning; its last command pose is 1.808 m ahead of
startup. A **1.222 s simulator tick gap** caused the unchanged 500 ms heartbeat
guard to revoke readiness at sim 59.17 s. Recorder stopped at 465.81 wall s;
the owned simulator was then stopped. Preserve all failed evidence; do not
score this prefix as completed-route terrain or navigation acceptance.
Unlike earlier GC failures, this run records no >50 ms GC pause during motion
(only the 361 ms pre-request collection). Native processing max was 99 ms.
Those records do not locate the remaining pause; kernel logs show no GPU/OOM
failure in the failure interval. No timeout, GC policy or sensor input changed,
and no blind repeat was launched. A future long-run diagnostic needs timing
around simulator stepping and its callbacks to distinguish rendering/physics
delay from host scheduling or uninstrumented work.

**Practical result:** on ordinary terrain the current-pose turning assumption
removes the short-motion terrain veto. The next navigation task is stable
incremental goal tracking, not squeezing through tight rock gaps or clearing
every unknown square. Retain the failed 30 cm goal result while replacing the
controller's noise-sensitive tiny point targets; test actual progress and rock
clearance separately. The longer-run simulator pause remains a separate limit.

Final revision checks: **147 host tests, 31 isolated ROS tests pass**; Black
checks and repository/submodule whitespace checks pass. Exact replay after the
comment/documentation cleanup: `revision-39bcfbf8132745c1a14c44fa00f79bef`,
all 483 raw maps and advisory outputs unchanged. Sources before that comment
cleanup are archived in `revision-6904a5077fd74a66a3284273e8ac3bc6` and
`revision-d1dbe016085847b792613b3d7fa4b089`. The 49 once-per-second advisory
batches all allow standing, forward/reverse 5 cm and left/right 5°; the separate
trial checked its requested action on all 483 updates. No full wheel patch is
LiDAR-confirmed in this short run, so do not attribute the near-field result to
new measured support. Live instrumented mapper p95 is **71.44 ms** and combined
mapper/advisory p95 **116.76 ms**; compute qualification remains open. There
were 649 zero-command stale-proposal waits among 2,539 execution ticks, also
counted in the trial audit. The simulator and diagnostic replay containers are
stopped. No artifacts were deleted and no normal navigation launch was changed.

**Earlier offline control (before the current-turn trial):** treating unknown ground as an absolute barrier is preventing
short routes. An offline high-cost-unknown control finds **all 184 tested
30 cm routes** across two recordings. Of those, 172 have a same-time GT starting
pose; all 172 stay inside the surveyed floor and avoid the independent rock
bounds. The other 12 remain unverified. This is promising for short-step,
stop-and-observe navigation—not evidence that arbitrary unknown terrain is safe.
At 50 cm and 1 m, some paths overlap a rock's conservative boundary. No new
policy in that offline comparison commanded the robot; Nav2 execution remains unqualified.

User requested a real costmap/planner implementation and obstacle checks, not
another binary support query. Separate candidate planner costs from untouched
terrain labels: free/inferred 0/20, isolated low-confidence unknown 70 (a penalty,
not a probability), occupied 100, other unknown -1. Promote only a single <=10 cm
cell surrounded by eight **measured** free neighbours, with a compatible local
plane (<=2 cm residual, <=20° slope), no explicit edge/clearance conflict and
more than 20 cm from occupied/conflicting cells. One pass on original data;
startup assumptions, inferred ground and promoted cells cannot seed more filling.

Implement bounded eight-heading cost-aware search with forward/reverse steps
and in-place turns. Check the entire continuous wheel/body sweep; retain existing
body guard and full wheel geometry. Startup/history support remains separate
exact geometry, never written free into the map. Publish opt-in candidate cost
and uncertainty-mask topics; do not silently replace the existing costmap or
connect an unqualified candidate to command execution.

Validation order: synthetic benign gap / seen obstacle / multi-cell hole /
hidden single-cell hazard fixtures, then matched strict/candidate planning on
completed native+GLIM recordings. Preserve fixed goals, maps, denominators and
GT separation. Score swept paths against independent scene rock geometry and
survey coverage, including current localization error; outside surveyed terrain
is **unverified**, not safe. Include a hidden-hazard positive control so the
test cannot "prove safety" simply by assuming unknown cells are empty. Assess
CPU/search limits and report navigation failure rather than retrying until a
favourable path appears. No sensor/mount/classifier change in this candidate.

Initial fixtures: a single soft gap connects a corridor rejected by strict
unknown handling; a hidden 6 cm hazard in that same gap is correctly exposed
as a potential collision by independent geometry. Ordinary Euclidean and
weighted-Euclidean search exhausted 4,000 expansions around a seen rock; add
obstacle-aware holonomic cost-to-go guidance without weakening any collision
check or increasing the search budget. Those intermediate failures are retained
here; no real-scene safety or live driving claim yet.

Implemented `local_terrain/navigation_cost.py`, `cost_planner.py`, and the
evaluator-only `planner_evaluation.py`; runners are `assess_cost_paths.py` and
`run_navigation_cost_cases.py`. Live publication is opt-in with
`--isolated-gap-costs` (`isolated_gap_costs:=true`):
`/terrain_shadow/planner_costmap` and `/terrain_shadow/uncertainty_mask`.
Original classes/costmap and command policy are unchanged. This is an
experimental footprint-aware planner, **not a connected Nav2 controller**.
Its 45° headings are coarse: an unavailable initial turn can hide routes that
a finer-heading planner could find. Failure is not proof that movement is
physically impossible.

Implementation/test fixes: footprint-aware cost-to-go now finds the seen-rock
detour within the unchanged 4,000-expansion budget. Touching only the boundary
of an unobserved cell no longer requires nonexistent positive-area support;
occupied/conflicting boundary checks remain. ROS wire tests account explicitly
for the `float32` map resolution rather than expecting Python `float64` bitwise
equality. A first analytic runner (`revision-482fb54ffbd84fe593309c12b150ab36`)
failed by applying a **single-motion** evaluator budget to a complete turning
route. Evaluate every bounded edge and union the results, without skipping
edges or increasing the individual motion budget. Failure evidence retained.

Controlled maps, final runner `revision-6a4e3489931e4d63ab1673227ef9000d`:
both policies find clear-floor routes, avoid a seen rock and the declared
10 cm-high/15 cm-wide rock footprint, avoid a known 20 cm hole, and reject a
solid wall. Only the soft policy crosses the isolated benign gap. In that same
gap it also crosses a deliberately hidden 6 cm rock footprint and a 10 cm hole.
Those are intentional risk counterexamples, not cases excluded from scoring.
These are analytic maps and swept geometry, not actual sensor observations or
executed collisions. No blanket safety claim.

First strict/soft replay pair (`revision-9fe1bd24776a4486a5e705614dd230e3`,
`revision-bf6b5ede120b4c4388a0019e26849298`) found **zero eligible isolated
cells** across 483 short-drive and 1,777 full-turn maps. Both policies found
0/120 and 0/432 tested routes to 0.3/0.5/1 m goals. Full-turn failures include
240 initial-heading-connector rejections, so do not attribute every failure
to terrain. Original maps/advisories replay exactly. Final-revision repeats
are being recorded separately; no earlier output is overwritten.

Next diagnostic: preserve strict/isolated results and add an explicitly
**optimistic, offline-only control** that assigns high cost to missing-ground
cells, including connected gaps. Keep occupied and explicit terrain-conflict
cells blocking. Score resulting paths against independent scene geometry and
same-time GT pose. This separates “unknown policy blocks progress” from
“obstacles block the route”; it is not permission to drive through blind areas.
Do not promote this control into the live policy. Existing hazard/compute gates
remain unchanged.

### Completed path comparison and resource checks

Final strict/isolated repeats: `revision-2794f099768243668e14bd78bc627ee9`
(short drive), `revision-9b0183ef0d1c41248740436f896d4e3a` (full turn). Both
retain zero promoted cells and zero returned routes. Added optimistic-control
runs preserve those same results and the same goals:

| Recording / run | Strict or isolated-gap routes | High-cost missing-ground routes | Same-time GT checks |
| --- | ---: | ---: | --- |
| Short drive, `revision-fe010af5ed994d53829e5d6b57f1aced` | 0/120 | 111/120 | 99 checked; no rock-bound overlap or survey exit; 12 missing GT |
| Full turn + test rocks, `revision-383591e4e8784ba0959c3879ce216a6e` | 0/432 | 310/432 | 290 checked; 15 possible rock overlaps, 2 survey exits; 20 missing GT |

Full-turn breakdown: 30 cm **144/144 found**, 136 GT-checked, zero overlaps;
50 cm **143/144 found**, 136 GT-checked, 14 overlaps; 1 m **23/144 found**,
18 GT-checked, one overlap and two survey exits. All 15 possible collisions
involve the same natural rock (`medium_rocks/.../instance_38`), using the existing
1.7 × 1.4 m body guard and projected USD bounds. These are **possible envelope
collisions, not observed crashes or exact mesh contacts**. The 14 half-metre
overlaps are the map-local 180° goal; do not average them away. Nominal-map
checks are also retained separately (short drive: one overlap; full turn: 16
overlaps and six survey exits). No fitted alignment is used to reconcile them.
The full-turn source retains its historical native first-hit discrepancy and
is diagnostic, not newly hazard-qualified.

The single-cell rule has no real-recording effect because unknown cells form
connected gaps, not isolated holes with eight measured-free neighbours. The
optimistic control makes **all** missing-ground cells finite-cost, including
blind regions, while retaining explicit conflicts/occupied cells. It is a
diagnostic upper bound on progress—not the proposed deployment policy.

Cost performance: the initial implementation needlessly fitted planes where
no eligible cell existed. Both fixed 200×200, 320-input resource profiles failed
the 50 ms p95 gate: `revision-38f0374f41dd490eabdd16f24dfe52c7` layer_capacity
58.41 ms, 1,042.31 MiB; `revision-1d4ca4451d2049b390e8e8865299547a` dense_flat
55.40 ms, 359.66 MiB. Preserve these failures. Skip the plane fit only when
the unchanged eligibility mask is empty; no threshold or label changes.
Final profiles: `revision-657169651b8d4e0f90428cf562e634fc` layer_capacity
**48.61 / 55.72 ms p95/p99**, **1,039.49 MiB** (compute passes, 1 GiB memory
gate still fails); `revision-844fe1146bba4a07a310e3954073bef6` dense_flat
**48.92 / 53.03 ms**, **356.86 MiB** (both pass). These measure the frozen
mapper plus cost layer, **not GLIM/ROS/planner end-to-end**. The Python reference
planner can take roughly 1.2 s on a cold query and is not a 10 Hz controller.
The large replay process peaks near 1.9 GiB; do not label it live memory acceptance.

Final exact-equivalence checks: `revision-7a66647719bc4b1196c4be5ca408484c`
(483 maps) and `revision-47ef7a1738bf4a22a5674e5ea0099097` (1,777 maps).
Every cost/mask array matches the pre-optimization path evaluation, and the
original terrain/advisory replays remain exact. Thus the speed fix does not
change any planner input behind the reported path results. Final validation:
**151 host tests, 31 sourced ROS tests passed**; rebuilt `slam_mapping` and
`astrobot_launch`; Python compilation, targeted Black check and both worktree
diff checks passed. Commands and dependencies are in the runbook's October 6
planner-cost section. No simulator/GPU workload or motion was needed this turn.

Interactive report: external archive
`revision-c4e8f5aca2be4159aea3eb03a43a4875/report.html`. It includes every
controlled case and all 46 recorded time bins, policies and goals, with
cost grids, starting footprint, paths and hazard outcomes. Source/grid/path
hashes accompany it. Open locally in a browser; no web service is required.

Next implementation direction: retain the costmap/planner separation, bound
how much unobserved ground a short path crosses, preserve known-rock clearance,
and re-observe after a short move. Merely filling isolated cells is insufficient;
blindly making all unknown ground drivable is too permissive. Test that bounded
policy against the saved rock-overlap cases before connecting commands, and
retain the earlier short-step pose/heading tracking issue as a separate task.

## October 6 — bounded terrain-controlled step trial

**Completed result:** the robot moved **22.8 cm forward** under live estimated-pose
and terrain control, then stopped at the declared 45 s controller limit. It did
not reach the 30 cm estimated goal. This is not “no movement possible”: a small
move/turn was available at **49/49** sampled advisory times; forward 5 cm at
**39/49**, reverse at 49/49 and each turn at 48/49. These are local options,
not a demonstrated connected route out of startup support.

The user's suspected small-gap problem is real. The first refusal lacked only
**0.245 cm²** of wheel-path support, with no occupied/conflicting cell. Across
483 controller updates, 91 refused the proposed sweep for unconfirmed ground;
missing area ranged from 0.000349 to 8.343 cm². No refusal remained continuous
for the 2 s terminal threshold. No new LiDAR-confirmed historical wheel support
was added: history stayed at its single startup anchor. The completed run did
not demonstrate extending support onto independently observed near-field ground.

It also exposed a controller mismatch: centimetre-scale step completion and a
roughly 3° heading threshold are too sensitive to the current estimated pose.
Execution-time horizontal discrepancy against GT was **7.45 / 12.64 / 15.57 cm
p50 / p95 / maximum**, including 0.217 / 0.250 / 0.267 s pose age. This is not
a same-timestamp SLAM accuracy metric. At the terminal command the estimator
still reported 23.05 cm to its fixed 30 cm goal, despite 22.8 cm actual progress.
Do not claim the raw step counter's zero completed steps means zero movement.
Of 2,534 execution ticks: 563 translated, 988 rotated, 525 waited for a fresh
proposal and 458 otherwise commanded zero. This controller needs improvement;
the result does not establish navigation impossibility or a need to relocate
the LiDAR.

Completed seventh attempt: recorder `revision-7e85b7425e0d47309f7265dbf714a87b`,
native `revision-fb0bc0c5066d407dbd104036d532da75`, mapper
`revision-3fc871732e5344be83239e21d026db98`. Audit
`revision-7f57ca66dcd9471ca0f231831bb52c9f`: **484 map projections and all
483 advisory/controller updates replay exactly**; all 2,534 recorded execution
ticks match checked proposals or explicit zero-command waits. Actual net
displacement 22.85 cm; cumulative XY path 25.94 cm. No GT command selection.
The six earlier failed attempts below remain failed. No mount, beam, classifier,
full-patch support, startup radius or evidence-lifetime changes were made.

Validation on the final runtime revision: **226 host tests and 71 sourced ROS
tests passed**; Python compile, shell syntax and both worktree diff checks pass.
Commands: the runbook's `terrain_step` recorder/launcher and
`tools/assess_step_trial.py`; pytest covered step trial, audit, contact runtime,
advisory/index/support/sweep, natural motion/loop, selected sensor/native audit,
contact replay and isolated ROS capture contracts. The simulator is stopped.
This is a completed diagnostic, not goal success, hazard/first-hit qualification
or compute acceptance. Instrumented mapper-plus-existing-advisory processing
was 99.2/120.6 ms p95/p99, excluding the new trial query, callback queues and
transport; both resource profiles were not rerun. Do not present those numbers
as total-controller timing or a passed resource gate.

**Next targeted work, not another identical sweep:** (1) use a suitable local
motion estimate and heading hysteresis for short-step tracking, keeping the
map/odom transform explicit and measuring GT progress separately; (2) evaluate
physically meaningful wheel support rather than requiring every tiny fragment
to be confirmed, against real rock/hole cases as well as benign gaps; (3) audit
why actual traversed contacts never become retained support, without copying GT
terrain labels into the planner; (4) repeat the same bounded trial with one
declared change at a time, then test a genuine exit and longer route. Attachment
modeling remains a separate known issue, not a demonstrated sensing remedy.

### Protocol and integration history

User agreed to try the actual closed loop while expecting the known support
gap may recur. Do not repeat a long coverage sweep. Implement a separate opt-in
`terrain_step` experiment: fixed 30 cm goal relative to the first causal GLIM
base pose, 5 cm increments at 2 cm/s, 0.5 s stop/observe intervals, estimated-pose
heading corrections at 0.04 rad/s. Query the remaining step plus a declared
2 cm experimental stop/latency allowance against unchanged full-patch support.
This allowance is not physically calibrated and this is not navigation qualification.

First persistent terrain refusal (2 s) ends the trial with a separate refused
outcome, not a successful goal claim. Save first/terminal refused grid and exact
unconfirmed wheel polygons. Maximum controller duration 45 simulated seconds;
unchanged recorder/lease/watchdog, archive, footprint/floor and original
heading/lateral/skid/stall guards remain. Sole command publisher remains the
guarded simulator driver. GT is preflight, independent emergency veto and
evaluation only; never passed into the goal/step planner. One-time GT GLIM
initialization and synthetic RTK assistance remain explicitly declared.

No mount, beam table, classifier, support patch, startup area or hazard scope
changes in this trial. No added test-rock profile: natural seed-43 rocks remain;
this is a short local-progress test, not another obstacle-recall qualification.
Preserved original advisory runtime source in
`revision-3c5c03b1339d4bf9ad3b6a6d3a8d34ec` before edits. Old advisory outputs
remain unchanged when the new route is not selected. Initial focused host
contracts: 137 passed. Host and dev CUDA allocation checks passed; external
archive has 142 GiB free. Live trial not yet run at this checkpoint.

First attempt stopped **before nonzero commands**: recorder
`revision-37de6ea8a6cf44baa4bce9dd287b3da8`, native
`revision-f188f6f9389c493a91a130ae18666a73`, mapper
`revision-3a2c6b0898024a2580647e8ca80adccf`. New driver expected the constructor
name `terrain_local_v2`, but ROS launch remaps it to `terrain_shadow_mapper`.
Fix the exact allowed publisher name, retaining sole-publisher enforcement;
add callback and launch-contract regressions. Preserve failed evidence and
restart fresh. Pre-attempt final host suite 210 passed; sourced ROS suite 55
passed. Rebuilt `slam_mapping` to install the new Python module; unchanged
frozen classifier library is still used. No sensing or terrain policy changed.

Second attempt: recorder `revision-c8189988e1cb43d491514627bb625473`,
native `revision-4726909c66894342aad5286cdb3ae378`, mapper
`revision-cd4a84da073d4c06b4067e733db09b77`. Failed during startup: a 1.4 cm
backward estimated-pose shift increased the retained step's remaining distance,
so the planner requested an 8.4 cm sweep while the consumer correctly limited
it to 5 cm plus 2 cm allowance. Cap each newly checked forward increment at
5 cm, retaining the fixed target and unchanged terrain checks. Added the
observed backward-estimate regression. Preserve this failed acquisition; it is
not evidence of a terrain refusal.

Third attempt: recorder `revision-0d5e49e8efb64ff883f3226fcd849b90`,
native `revision-6d1b56a2288a4eb28386585ccbe6dc1f`, mapper
`revision-ba07d0f7eb2e492abafa4b42aa4fafb0`. No motion sent; proposal heartbeat
expired at the end of pre-roll, despite available terrain proposals. New trial
subscription was drained only on the physics thread, unlike the existing
independently serviced acquisition grants. Move proposal reception onto that
receiver executor, with locking around proposal state and execution; it cannot
issue nonzero commands. Preserve 0.5 s heartbeat and 200 ms simulated freshness
limits. Add the executor-wiring regression and include actual age in future
failure messages. This remains a failed integration attempt, not terrain data.

Fourth attempt: recorder `revision-91d5e66e251447afaf2f9d215d743ba1`,
native `revision-5d4cd75b641e4215990ebaf17bf64cc1`, mapper
`revision-c3b991d2d56e41a484af60cf90535d21`. Stopped before movement when
estimated-pose noise put the retained 5 cm target more than 2 cm off-axis.
That target is not a safety corridor: correct recovery is zero command,
0.5 s observation and a newly checked step toward the unchanged 30 cm goal.
Implement that recovery, log replan counts, and test that it does not count as
progress or reuse the old command. Actual GT lateral/heading emergency limits
remain unchanged. Invalid proposal errors now retain the mapper's reason.

Fifth attempt: recorder `revision-1939c4c3a7b24d8587de4121a3ad1d12`,
native `revision-4c7b9c4563f04168a36a89b7f4c8f17b`, mapper
`revision-656af4fa0a0a430faee284e6acb98034`. Five execution ticks, then a
0.560 s proposal gap. Independent reception did not eliminate producer delays.
Keep stale-command rejection unchanged, but implement zero-command waiting
instead of aborting acquisition on missing/old proposals. Only a newly received,
fully checked proposal can resume. Record all waiting ticks and audit that none
commands movement. Original simulator/lease heartbeat failures still abort;
invalid proposals and frame/identity faults still abort. No clock retimestamping
or watchdog widening. The failed fifth attempt remains failed, not rehabilitated.

Sixth attempt: recorder `revision-70a268d6d38641de8ea6161531589ac6`,
native `revision-8140f6d943ef4a699e8cbbfa5d1bc5c3`, mapper
`revision-ab562947c82b4ab792c00836c8b710e9`. Actual forward motion reached
about 11 cm before the first terrain refusal. Saving its diagnostic failed:
Shapely's `mapping` is in `shapely.geometry`, not the package root. Correct the
import and test actual JSON/NPZ geometry serialization for both first and terminal
refusal. This partial moving run remains failed, not scored as acceptance.

## October 6 — mounting realism and what “no exit” actually established

User correctly challenged the unmounted forward-offset proposal and the apparent
loss of earlier coverage. Layout experiments are permitted, but positions must
have a physically plausible attachment. Withdraw the 12 cm forward offset as a
recommended next mounting: the ray-only screen did not establish attachment.
Do not exclude holes or change the startup/support rules in response to this.

**Historical comparison:** September 27's 100% 0–1 m yaw coverage came from
24 sensor-only placements with **no rover body**, optical offset [0.4,0,0.25].
Later actual-collider auditing found the old origin inside the chassis and
0.5 m ray starts bypassing body occlusion. The corrected profile moved z to
0.45 and cast rays from zero, keeping the 0.5 m measurement cutoff separately.
Current ~99% ground support at 1–1.3 m is not inconsistent with missing support
near the body. Neither metric is the new continuous full-wheel-path criterion.

**New attachment defect found, not yet fixed:** USD-only audit
`revision-31ddad6fb31e49aa80d91df09c20b414` examines transformed mesh vertices,
not authored extent metadata. The live setup translates the entire `vlp16`
parent, including its support mesh. Authored support spans z=0.06..0.21 m;
the selected raised profile moves it to z=0.26..0.41 m. XY-overlapping body/plate
vertices reach at most z=0.224557 m, leaving **at least 35.44 mm of separation**.
The support was not extended when the optical origin was raised. This is a
modeling omission, independent of the previously passing optical collision test.
Also, this support mesh has no collision API. No new asset/profile was applied.
Further mounting work must model the attachment and its occlusion, not just move
a TF or sensor origin. Static mesh bounds do not certify structural strength.

**Correct interpretation of prior motion results:** the 2,848 turn/drive paths
were evaluated against one map per query; they assumed no new observations
during the path and used 0.5/1 m drives. This did not test an active sequence
of small movements with new observations and replanning. “No route is possible”
was too strong; only the declared paths failed. The original 5 cm move / 5°
turn checks already allowed a small action at 174/178 full-turn decision times.

New idealized active-observation screen `revision-878e7c00a6e642a384afaf63c8e3d57d`:
unchanged mount/beams, analytic flat ground + chassis, exact poses, same frozen
C++ classifier, 60 s evidence, fixed 61 cm startup polygon with 2 cm erosion,
original full wheel/body sweeps. Reverse 5/10/15 cm, return to center, turn 15°,
repeat through 345°. Every move is checked against existing support before the
next scan is supplied. **All 213 segments / 214 observation poses are admitted**,
but no straight 10–100 cm candidate becomes an exit under the existing rule.
This reproduces a support-classification bottleneck without SLAM noise; it does
not prove no possible observation strategy works. The final 10 cm forward and
reverse steps are allowed. The 20 cm forward step lacks only 9.08 cm² of the
required support area; longer forward paths encounter larger unconfirmed strips.

Limits: one ideal instantaneous full scan per endpoint, 321.61 s modeled time,
no native Isaac returns, wheel/mast occlusion, skid, pose errors or hazard recall.
No GT ground labels enter the classifier: analytic geometry generates its points.
This is a separately declared diagnostic, not a successful physical maneuver or
permission to bridge unknown ground. Retain the original negative results.

Next direction: repair/model a grounded mount in a separate candidate; audit
the specific missing support fragments and test causal incremental observation
on real native data. Keep “small observation move available” separate from
“connected exit available.” Do not jump to another unsupported mounting or claim
SLAM is the cause based on these results. Use physically plausible mounting and
actual wheel/support geometry when deciding whether the all-area support rule
is excessively restrictive. Current live behavior and sensor remain unchanged.
Validation: **76 focused tests passed**, including a fixture proving every
declared observation segment fits the fixed, eroded startup support even with
the rest of the grid unknown. New USD attachment and analytic observation tools
are opt-in diagnostics. No live capture or resource qualification was run.

## October 5 — toward exploration-ready local motion

User requested an integration-ready path toward exploration and longer runs.
Baseline: completed 10x15 full turn, bounded SLAM error, 98.73% outer ground
support but 0/178 connected exits; raw evidence lasts 60 s during a 177.7 s turn.
First falsifiable hypothesis: retention, rather than missing observations alone,
breaks the connection. Test an explicitly optimistic causal union of previously
free cells as an upper-bound diagnostic before implementing a support-memory
policy. Never substitute this optimistic union for terrain truth or motion
authority. Preserve original raw-map and advisory replay exactly.

Then implement only a mechanism justified by that result, test hazard/fault/frame
regressions and frozen-recording availability, followed by live short-horizon
movement and bounded resource tests. Exploration wiring stays non-authoritative
until local support and stop behavior are demonstrated. Keep holes/drop-offs in
scope unless the user explicitly chooses a rock-only continuous-ground demo.
No larger startup circle, no GT terrain labels in the estimator, no silent
sensor-audit relaxation. The prior single recast discrepancy remains separate
from the practical near-field support problem.

Retention falsified as a sufficient fix: `revision-f54e8f460fe8497b9396265eac3e6ab7`
exact-replays all 1,777 raw/advisory maps. Optimistic causal past-ground union
adds 95 final support cells but still gives **0/178 exits**, unchanged. Ten
host tests cover non-mutation, hazard veto/deletion, rolling eviction and
frame/epoch/grid-phase/clock refusal. Diagnostic only: no age/uncertainty bounds,
therefore not a deployable memory policy. Do not spend a cycle promoting it.

Next hypothesis: whole-cell reduction discards usable partial support near the
startup boundary. Predeclare four 10 cm grid phases: (0,0), (5,0), (0,5), (5,5) cm.
Same native inputs, causal frozen poses, classifier/library and 60 s lifetime.
Union only approved ground polygons; every phase's occupied/edge evidence vetoes
the candidate sweep. Start with the completed/native-audit-passing 20 cm reverse
recording, then test the full turn. This is a compute-heavy diagnostic first,
not a newly promoted mapper or a relaxed hole/obstacle definition.

**Four-grid phase result:** `revision-b5e6afd41440474f98896d2097cde73d`
reproduces all 253 original maps/advisories of the reverse-20 cm recording.
Baseline **0/26 exits**, four-phase support **0/26 exits**. Three extra grids
cost 88.83/98.20 ms p95/p99, on top of baseline work (offline instrumented
diagnostic, not compute qualification). Nine focused tests pass, including
partial-cell support and cross-phase hazard vetoes. Do not promote this added
cost; the planned full-turn version is deferred after the short test failed.

**October 6 follow-up:** preserve the selected sensor and support rules. An
explicit analytic screen varies only the lowest seven beams, to -65/-70/-75/-80
degrees, retaining the upper 25, the level mount and total ray count. Extra
downward rays hit the chassis; none narrows the parked 73.5 cm blind radius.
This rules out simple beam steepening, not all possible mounting changes.

Also test a limitation of the original motion assessment: add 16 fixed
turn-then-drive paths (eight headings, 0.5/1 m travel), checking the entire turn
and drive against the same current map and support. No future scans are assumed;
keep the original 20 straight paths and every decision time separately.
Reverse-20 cm result `revision-3da52d7757ca4b12a56471a9d0e5a7da`:
**0/26 straight exits; 0/26 turn-then-drive exits**, all 416 added paths retained.
Full-turn result `revision-58aba6d6ea654a65a78b2650590e6e6f`:
**0/178 straight exits; 0/178 turn-then-drive exits**, all 2,848 added paths
retained. All 1,777 original raw/advisory maps reproduce exactly. This is a
fixed set of additional paths, not proof that every possible maneuver fails.
No commands or production changes were made. A turn within startup is still
not an exit.

Analytic sensor screen: original mount-only run
`revision-6b5f2d7b36154994bfe52ff537292f15`; extended, separately declared
hypothetical forward-placement screen `revision-790b7513e03d4f48bd20ebcec06a43e5`.
Moving optical x from 0.40 to 0.52 m with the upper 25 beams unchanged and the
lowest seven spanning -35..-80 degrees gives a **51.08 cm** minimum ground
radius and 1,150 ground returns inside the startup circle, versus 73.49 cm
and zero in the selected setup. Same 63,968 rays and 0.5 m range cutoff;
chassis is intersected first. Wheels/mast/terrain relief are excluded, so this
is a promising hypothesis, not native acceptance or a hardware recommendation
validated for deployment. No asset or selected profile changed.

**Next action needs an operating choice:** test that forward mounting while
keeping holes/drop-offs in scope, or explicitly choose a continuous-ground
rock-only first demo and evaluate a separate clearance policy. Asked the user;
strict scope and fixed sensor remain unchanged pending the answer. Do not
quietly turn unknown support into free space or resume layout selection as if
the original freeze never existed. Recommended next experiment: the forward
mount, retaining the hazard scope. No host/GPU/archive blocker was encountered;
Isaac stayed off for these offline comparisons.

Plain-language integration sequence and current topic/code entry points:
[Getting to exploration](exploration-readiness.md). Final focused regression:
**287 tests passed** (contact/runtime/index/support, continuous sweeps, all new
diagnostics, navigation-gap scoring, natural routes, rocks, costmap, startup,
current-pose context, pose lookup and selected-sensor contracts). Black checks
on all nine added files, Python compilation and root/OmniLRS diff checks pass.
Both phase and retention candidates remain diagnostics only; no new live or
compute qualification is claimed. Commands are in the runbook.

## October 5 — selected 10 x 15 cm coupled rerun

User selected **10 cm exposed height x 15 cm width**, not the proposed 12 cm
height. Implementing the opt-in `rounded_10x15_v1` test-rock profile, keeping the
current natural rocks, terrain, LiDAR, GLIM configuration and 8 cm classifier
threshold unchanged. Eight closed rounded triangle meshes sit around the
startup area (axis offsets 1.8 m, diagonal offsets +/-1.8 m in each axis),
outside all existing guarded observation-route survey regions. Each vertex is
placed relative to independently raycast underlying terrain; the apex rises
10 cm, horizontal diameter is 15 cm. Native PhysX apex/shoulder checks must pass
before sensor audit and capture. Exact mesh geometry and profile hashes are
archived, evaluator-only; no GT rock map feeds SLAM or terrain.

This is a dimension-controlled coupled fixture, not an assertion that scaling
every natural rock produces those exposed dimensions. No random-rock scale
settings change. The initial 1.7 m layout failed a host test for the existing
reverse-route survey margin; moved the declared test objects to 1.8 m before
any capture, preserving all route/clearance thresholds.

Test sequence: host mesh/route contracts; short `turn15` GLIM + native terrain
capture; diagnose/fix scoped integration faults, then full `turn360`. Assess
first-published GLIM accuracy, exact map/advisory replay, all eight objects,
independent clear-floor samples, supported exits and recorded processing time.
Any missed object remains reported, rather than requiring perfect scores or
silently changing size. Native sensor evidence and the actual classifier are
required; the October 5 analytic visibility fractions are not acceptance.

**First startup failed before motion:** native `revision-b36bced40c4d4f07856aded7a3100e19`,
recorder `revision-48424d0b9e98453d8bf5d419d72fd6a7`. A declared position at
(-1.8, 0) overlaps natural boulder `instance_38` (world bounds x17.726–18.566,
y17.532–18.354). No SLAM/mapper was started and no commands sent. Preserve the
failure; the recorder was interrupted after simulator shutdown. Declare profile
`rounded_10x15_v2`, moving only that test object to (-2.65, 0), outside the
recorded natural boulder bounds. Other seven positions and all dimensions unchanged.
Added explicit failed-position/hit diagnostics. Seven mesh/score/profile tests
pass; earlier focused route/capture suite passed 110 tests. CUDA allocation and
external archive checks passed before both launches. No natural rock removed.

Second startup: recorder `revision-52bead8a405945c09aa014497c48e702`, native
`revision-8bed2bb937d64d97a6aae0032d01560c`, mapper
`revision-c0e8e509f04242b8a1066ac0b96fcf6e`. All 40 independent PhysX
apex/shoulder checks passed (maximum residual 1.20e-8 m); original clearance
survey passed. Coupled GLIM/terrain `turn15` capture running. Current focused
regression set: 325 passed; formatting and both repository diff checks passed.

**Short coupled run completed:** 1,962 native ticks, 326 valid maps, 33 advisory
decision bins, 328.23 s wall. Exact object/map replay
`revision-a548e992d2d84969a8cc8c818105a2d0`: **7/8 final and ever-detected**.
All seven detected objects have sampled native hits (124–416 each). West rock_4
has zero sampled hits and remains unknown, behind the existing western boulder;
occlusion is the leading explanation, not a reason to remove it from the count.
No false-free apex among final eight object queries. These test rocks are
1.8–2.65 m from startup, not a qualification of the earlier 0.65–1.1 m blind band.

Observation assessment `revision-2db6a3509516449e9b039d0c28ac3c95`: **0/33 supported
straight exits**, despite short moves/turns being available at all 33 decisions.
Final sampled floor support: startup 0/472; startup boundary–1 m 172/783 (21.97%);
1–1.3 m 671/865 (77.57%); zero false-occupied floor samples. No classifier or
ray-clearance support rule was changed, so this run does not fix the near-field
gap. Recorded mapper/advisory total p95/p99 94.69/115.71 ms, RSS 210.91 MiB;
instrumented timings, not compute qualification.

Pose/native audit `revision-9712eaf5ec39443aa1989fd0b2fdf448` (trajectory
`revision-a51d7fa2d63344109bdaba9b4e890707`): native gate passed, 251,136 sampled
hits, maximum geometry residual 0.0508 mm; all 12,467,456 transported points and
1,962 GT pairs verified. First-published GLIM horizontal p95/max **7.72/9.53 cm**,
height p95 4.44 mm, yaw p95 0.316 degrees. No catastrophe; original pitch target
remains failed (p95 0.389 degrees), without threshold changes or fitted alignment.
HTML `revision-15c6f673de504e48ac97113d86233208/startup-observation.html`.
Proceeding with the full turn, same profile and unchanged SLAM/terrain settings.

Full-turn recorder `revision-2c3cd48e3cdb4f10a108f71faf50c039`, native
`revision-916bda3c56ee4d43aec10f3315064a01` started. All test-rock geometry
checks and original motion-clearance preflight passed again. Added a standalone
coupled-result renderer: short-run report
`revision-feb907461614413098c2acf9186db976/report.html`, showing full raw grid,
actual test-rock outlines, every object and GLIM/ground-support metrics.

**First full turn stopped near 293 degrees:** recorder reported
`physics_imu_ready` false after 996.57 s wall / 8,393 native ticks. Mapper
`revision-a54551ed2894406f9f0b7d9d3683c811` and capture remain failed/ineligible.
Other readiness checks passed; simulator then stopped on the expired recorder
lease. Both IMU topics recorded 9,322 messages. Investigating recorded sample
pairing and arrival timing before selecting a fix; this is not evidence of a
GLIM tracking failure. Preserve this attempt and rerun from a fresh capture
after any scoped fix and regression tests. No freshness threshold changed.

Cause identified: the simulator recorded a **424.39 ms generation-2 collection
at the exact failing tick**, inside the native surface audit. Surrounding work
took the IMU receipt gap over 500 ms. Last 120 IMU/header and diagnostic stamps
pair exactly with no cadence gaps. Archive checks were already asynchronous;
their elapsed timing is not evidence that they blocked ROS callbacks. These
launches accidentally used `experience=default, import_stat_cache=true`, not
the previously validated lean/cache-disabled configuration. Correct the explicit
launch environment to `SELECTED_ISAAC_EXPERIENCE=lean_v1
SELECTED_IMPORT_STAT_CACHE=false`; preserve both attempted startup configurations
and all failed evidence. Sensor/SLAM/terrain/route/GC/watchdog settings unchanged.
The lean retry is a declared startup change, not an interchangeable repeat.

Fresh full-turn retry: recorder `revision-83971b0384884f3ba0ab306d857820c9`,
native `revision-2281895a7ee545aaaefcb08377d01f0c`. CUDA allocation and archive
preflight passed; native manifest confirms lean/cache-disabled startup. Geometry,
clearance and sensor checks passed, motion underway. Results pending.

**Lean full turn completed:** mapper `revision-f3b5f4d770474dfabc0bdfe8cb0a1fb7`;
10,663/10,663 native chunks integrated, 1,778 projections, 1,243.26 s wall.
All final recorder readiness checks passed; mapper completed and closed without
failure. Ordinary in-motion generation-2 GC cost 356.66 ms; maximum native
callback 412.09 ms. This run exercised GC successfully but does not qualify
arbitrary long-duration execution. Simulator stopped for offline scoring;
exact replay, object/floor/exit assessment and GT pose error pending.

Full-turn exact replay and terrain scoring completed: object assessment
`revision-93274ae6d7e44554ab2c48ee77e50946`, observation assessment
`revision-84c0290e86b348b3a58017c127c1c1d2`. All 1,777 valid maps and original
advisory decisions reproduce exactly; 1,778 projections / 10,663 chunks verified.
**8/8 test rocks detected during the run, 7/8 in the final map**. West rock_4
has 144 sampled native hits, detected in 1,073 maps, last detection at 159.0 s;
final footprint unknown, not free. Last sampled hit at 97.900005 s is consistent
with later evidence expiry behind the natural boulder, not proof of the exact
last full-cloud hit. Every rock's apex has zero free classifications across all
1,777 maps (queried from `objects.jsonl`; no inflated association region).

Actual turn 359.818 degrees, maximum displacement 18.127 cm. Final independent
floor support: startup 0/472; boundary–1 m **262/783 (33.46%)**; 1–1.3 m
**854/865 (98.73%)**. Zero occupied floor samples in all maps, retaining every
sample/unknown. **0/178 supported straight exits**; small moves/turns available
174/178, standing 178/178. Inner unknowns: 244 no-return, 260 unsupported-extent,
17 insufficient-area samples. This remains a near-field support/connectivity
problem, not evidence that all directions contain obstacles. No ray-clearance
inference or larger safe radius implemented by changing rock size.

Live instrumented mapper p95/p99 65.11/70.58 ms; mapper plus advisory
100.65/121.06 ms, max 141.31 ms, RSS 279.75 MiB. Not both-profile resource
qualification. Final focused regression suite **319 passed** with runtime-matched
NumPy 1.26.4 / SciPy 1.11.4, Shapely 2.1.2 and toml 0.10.2. First host collection
lacked toml; unpinned dependency retry selected SciPy 1.18.1 and failed the
existing paired-IMU Euler-array fixture (318 passed / 1 failed). No runtime or
assertion change: reran with the actual container's scientific-library versions.
Black, Python compilation, root and OmniLRS diff checks passed. GT trajectory
export `revision-b65bf429d3ac449fbfcffab19becaf57` completed; pose audit pending.

**Full pose/native audit completed:** `revision-005af64f6c6649ba99cc05c920f8e44f`.
All 67,602,670 transported points / 10,663 GT pairs verified. First-published
horizontal p95/max **13.59/18.96 cm**, vertical p95 1.54 cm, yaw p95 0.318 degrees.
No catastrophic tracking loss; older strict pitch target remains failed
(p95 0.495 degrees). Same physical base_link, fixed home, no fitted alignment.

**Strict reference audit failed on one of 1,364,864 sampled rays**, tick 3383,
76.366670649 s, point 283. Native ground distance 2.1773138 m versus independent
PhysX rock_0 edge hit 2.1287622 m, residual 4.855 cm. Same saved ray shifted
0.49 m along its line hits ground; 0.51 m hits rock. New all-outlier diagnostic
`tools/diagnose_demo_rock_rays.py`, result
`revision-31aa72637f4b421e8fcafe98d503b46c`, finds no intersection with any
float32-authored demo mesh using float64 math for all three saved rays. This
supports a numerical/reference edge case, not a demonstrated pose-frame bug.
It does not reproduce PhysX cooking or qualify all-scene geometry. The 1 mm
gate remains failed, every return/map retained; no raw-data repair or threshold
change. Object/support figures above are diagnostics, not full terrain acceptance.
This rare recast discrepancy does not explain the persistent inner unknown band.

Final full-grid report `revision-912257912eaf4198a86cd9e5f0954612/report.html`;
quarter-turn gap gallery `revision-698081e1f9c346f69ff6e0ce0afbf959/startup-observation.html`.
Both display qualification limits. Recorder/transport ROS regressions: **40 passed**
on isolated domain 179. Diagnostic mesh tests first exposed a 1e-16 arithmetic
assertion discrepancy; the new math fixture now uses 1e-14 tolerance (not a
change to any sensor/terrain acceptance threshold). Final host suite: **327 passed**.
No exploration promotion. Next algorithm work: bounded near-field support,
frozen-map exit/false-free comparison, then actual blind-band hazard and
terrain-guided motion tests—not another blind SLAM parameter sweep.

## October 5 — practical near-field hazard visibility

User approved measuring practical visibility rather than requiring every possible
adversarial placement to be observable. A slightly larger minimum demo rock is
allowed as a separately declared environment change; do not relabel original
8 cm performance or change historical thresholds.

Implemented `tools/assess_nearfield_hazards.py`: 1,728 fixed placements spanning
regular and shifted bearings/radii, box and rounded rocks (8–20 cm high,
10–30 cm wide), and pits (8–40 cm deep, 10–60 cm wide). Preserves selected
32 x 1,999 ray geometry, chassis-first occlusion and the existing 0.5 m software
range cutoff. Pits return from walls/bottom where physically intersected; far
ground remains visible when a ray passes over a hole. Missing expected returns
are recorded separately and never used as proof of free ground.

Four view sets: parked, reverse 20 cm, 24 yaw views, and yaw plus reverse peeks
at each heading. The combined set is a hypothetical visibility experiment, not
a validated physical maneuver. Report individual cases and fractions with
1/3/10 changed returns in a view; no worst-case-only pass/fail and no claim that
placement fractions are deployment probabilities. Hazards overlapping the
startup assumption remain in the raw results but are separate from external
hazards. Different widths have different eligible placements: size comparisons
must additionally use common placements, not silently compare denominators.

Initial 16 geometry/regression tests passed, including ray cropping equivalence,
box/rounded shape differences, a hole that is invisible beneath a passing ray,
pit bottom/wall hits, and a too-close obstacle suppressing expected returns.
External archive/device/capacity checked; no Isaac or GPU workload required for
this analytic screen. Run `revision-0a7afddee096451a8d2d2ad2c79ab626` in progress.
No live mapper, obstacle threshold, sensor configuration or rock scale changed.
The current `lunaryard_40m.yaml` scale range is 8–11; it is not a physical exposed
height guarantee. Earlier comments also record narrower corridors when raising
the minimum, so future demo changes must check navigable space as well as recall.

**Completed geometry screen:** `revision-0a7afddee096451a8d2d2ad2c79ab626`,
1,728 placements / 1,188 wholly outside the startup circle, 79.77 s single-worker
wall time. Ideal rotation exposes every eligible box/rounded rock in both banks,
including 8 cm high x 10 cm wide. In the shifted bank it misses 8/44 placements
of 10 cm-wide pits (both 8 and 20 cm depths); wider eligible pits are observable.
The hypothetical rotate-plus-reverse-peeks view set exposes all eligible hazards.
These are first-hit differences from flat ground, NOT terrain recall or safe
motion. A geometrically visible rounded 8 cm rock need not produce points above
the classifier's 8 cm height threshold.

**Recorded-path counterfactual:** `revision-2e8424e1a3484f70bbd6b40be4079071`
uses start/end and the first 15-degree crossings of completed native turn
`revision-5670c52bfd8942f9b9b6f4cabf4c1edc`. Reuses its XY/yaw only on ideal flat
ground, not actual Z/tilt, surfaces, native returns or SLAM noise. The outward
turning drift matters: rounded 8 x 10 cm rocks give >=3 changed returns in
32/36 regular and 34/48 shifted eligible placements. At 8 x 20 cm this becomes
36/36 in each bank. Recorded-path pits: 10 cm wide 25/36 and 26/44;
20 cm wide 31/36 and 33/36; 40 cm wide 24/24 each. Increasing rock size does not
resolve small-hole uncertainty. Source hash and all missed placements retained.

**Separate post-baseline size sweep:** `revision-d07b97008ea945cf97105b8437e29d91`,
1,536 rounded-rock placements, 8/10/12/16 cm heights x 10/12/15/20 cm widths.
On the same 72 locations eligible for every tested size, 8 x 10 cm yields
65/72 useful signals; 10 x 15 cm and 12 x 15 cm both yield 72/72. Minimum
best-view changed-return counts are respectively 0, 3 and 29. Thus **12 cm exposed
height x 15 cm width is a promising demo candidate**, not a certified minimum or
a changed world setting. The original 8 x 10 cm counts on all 96 positions
(including prior-overlap cases) reproduce the earlier comparison exactly.
Narrower rocks can legally sit closer to the startup circle, so full eligible
denominators differ by width; the report preserves both full and matched scores.

Final verification: **435 focused tests passed**, including 19 new tests;
Black check, Python compile and `git diff --check` passed. HTML size-sweep report
visually inspected. Both scripts, original inputs/config hashes, all per-view
counts and failed/missed cases archived. First host recorded-path invocation
failed before output allocation on missing `/archive`; resolved by the documented
container aliases, without altering the archive guard. No Isaac launch, GT-fed
mapper, safe-area inflation, sensor/rock config mutation or classifier change.

Practical next cycle (not demanding worst-case perfection): put the baseline
8 x 10 cm and candidate 12 x 15 cm exposed shapes through the actual classifier,
including rounded/embedded rocks and sampled pose errors, and retain all misses.
Then test observed ray-clearance as a separate input to motion support, preserving
the distinction from wheel-support evidence. If the candidate remains effective,
create a named demo world profile, measure actual exposed mesh dimensions and
check passage width before adopting a minimum scale. Keep small-hole limits
explicit rather than treating far-ground visibility as proof that no hole exists.
Plain-language results: [near-field hazard tests](nearfield-hazard-status.md).

## October 4 — startup turning area and deliberate observation

**Latest outcome:** implemented the fixed 61 cm startup turning area and tested
a full turn, a 15 cm reverse/return, and a 20 cm reverse/return. The full turn
observed almost all surveyed clear floor 1–1.3 m away, but none of these protocols
produced a supported straight exit beyond the startup assumption. This is a
near-field sensing/support problem, not a successful navigation qualification.
See [plain-language results](startup-observation-status.md). All failed attempts
below remain part of the record; later completed results supersede the earlier
"running" checkpoints.

User clarified the deployment assumption: safe ground within the robot's own
in-place turning sweep, not merely the parked rectangle and not an arbitrary
large cleared pad. Implementing an opt-in, fixed startup circle using the
physical hull/wheel extents (half-size 0.5037 by 0.3454 m; about 0.611 m radius).
It is pose-anchored, never follows the robot, has no added skid allowance, and
cannot override measured obstacles or edges. Raw terrain remains unchanged.

Hypothesis: the earlier +/-15-degree capture and whole-wheel support check did
not adequately test active observation. Declare a separate `turn360` acquisition:
eight consecutive 45-degree arcs, original 0.1 rad/s cap, fixed-origin feedback
up to 0.03 m/s to reduce skid, original 25 cm skid limit and all lease/floor/
stall guards. This remains the existing GT-controlled acquisition protocol,
not autonomous terrain-controlled driving. Actual skid beyond the ideal swept
circle must be reported, not silently included in the startup assumption.

Test order: geometry/hazard/route regressions; fresh headless full-turn capture
with the turning-area advisory; exact replay; then score raw classified support
and proposed exits BEYOND the startup area. Turning availability alone cannot
establish useful progress. Preserve all failed attempts, original maps and
timings. Core support stays off to isolate the startup-area/observation change.
Host and selected-dev CUDA allocation checks passed; simulator initially stopped.
Previous live/runtime sources archived before editing at
`revision-ae5582e4e3ab46b3abdb6bf5943fdbf3` and
`revision-963a1c1ed3974595b5f05e118fb75c68`.

Implementation and first **138 focused regressions passed**. The initial polygon
prototype exceeded the existing 32-vertex spatial-index budget; corrected to an
inscribed 16-sided circle (at most 1.174 cm inside the true circle), which also
fits the budget after projecting the tilted contact-height band. No index budget
or safety threshold was relaxed. Both ROS packages rebuilt successfully.
Fresh recorder `revision-a9a2bf89989b40c3b0e96670a4d93b63`, native capture
`revision-065fda203ae6497395d6d969b04124e5` now running. Added a separate offline
assessment of fixed 10–100 cm forward/reverse paths and independent surveyed
floor samples; it reports exits requiring wheel support outside the initial
area separately from movement confined to that area. No terrain classifier or
sensor changes in this experiment.

**Attempt 1 stopped at 91.2 degrees:** physical drift crossed the unchanged
25 cm origin bound (recorder above; mapper `revision-a6530897f9cd4683a7b8cf733cbb6d71`).
Keep it failed/ineligible for completed-route scores. Live small motions were
available at all first 27 decisions, but this is only a failed-prefix diagnostic.
Its driver source is preserved at `observation-driver-v1-QKoihKqP/natural_motion.py`,
SHA-256 `c999a0b925bd7bfa4c8df02aa50d7d5cc54c19245ba5c9ad745a6b487da47021`.

Failure analysis: measured motion has substantial lateral turning slip, and
weak response to simultaneous linear corrections. Least-squares diagnostic on
turning samples gives body velocity approximately
`vx = 0.158*v_command - 0.085*yaw_rate`,
`vy = -0.080*v_command + 0.162*yaw_rate` (actual measured yaw rate).
This is a local acquisition-controller diagnostic, not a calibrated vehicle model.
Revised, separately hashed `turn360` protocol prepositions forward 15 cm, turns
in eight arcs with a 15 cm nominal observation orbit and bounded slip compensation,
then returns to the original origin. Linear corrections remain <=3 cm/s; original
25 cm guard, startup circle, timing/watchdogs and terrain classifier unchanged.
The identified-model regression predicts a bounded full turn; **106 route/observation
tests pass**. Next: fresh physical validation, not promotion from the model.

**Attempt 2 also failed the physical skid bound, at about 164 degrees.**
Recorder `revision-a6466a948a244db6809c338485604834`, native
`revision-de44def2ad314e46ae806490543e9702`, mapper
`revision-a7016aa615e54f0489b89e6dcd39a5b0`. The simple slip model was not accurate
enough; its passing regression did not establish physical control. Preserved
driver at `observation-driver-v2-UeZaBxUh/natural_motion.py`, SHA-256
`505f7b4103d356be0e83d8ff2e02c2f7524b4077962935a585c2997897b5aa64`.

Revised hypothesis for v3: stop yaw and recenter by a short straight move when
radius exceeds 18 cm and the heading can reduce that error, then resume the
same arc. Do not rely on simultaneous translation during skid to recenter.
The original 25 cm stop limit remains. **144 focused tests pass**, including
the second failed pose as a correction/stop regression. The new ten-leg
observation protocol declares its own 420 s / 26,000-frame storage/audit bounds
(ten unchanged 40 s leg caps plus settles); old route bounds unchanged.
No failed capture is rescored under the revised protocol.

Third fresh run is in progress: recorder `revision-eba82b70217844e58285386ed8436b77`,
native `revision-5670c52bfd8942f9b9b6f4cabf4c1edc`, mapper/advisory
`revision-2aec77630553438d8668811517295dff`. Selected-profile/native-audit/route
regression set: **126 passed** after declaring the new acquisition bounds.
`assess_startup_observation.py` and `render_startup_observation.py` are ready;
their fixed exit paths exclude support supplied merely by current wheel
occupancy. The report preserves first-quarter-turn examples and the final map.
Actual live validation and exit/coverage scores remain pending completion.

**Full turn completed and exact replay passed:** 10,597 native chunks,
1,766 valid maps, 177 decision bins; 359.82 degrees net yaw, maximum displacement
18.09 cm, cumulative motion 2.58 m. Assessment
`revision-f2780c4cd4bc4a18b62007fc4e76c96a`. **No supported exit at any of the
177 decisions.** Final independent floor support: initial circle 0/472 samples;
circle boundary to 1 m 300/783 (**38.31%**); 1–1.3 m 852/865 (**98.50%**).
Zero false-occupied samples. Near-field unknowns are missing returns / unsupported
extent, not false dropoffs. The circle is useful as a deployment assumption but
rotation along this outward orbit does not connect it to observed ground.

GLIM diagnostic `revision-eb74981e672c4fb0a8a1d6a0738ec7b4`, trajectory
`revision-7b37e4a99b5e4eefafdc6c68991d85f6`: all 10,597 GT pairs and 67,181,803
transported points checked. **Strict native geometry gate passed**, maximum
residual 0.234 mm over 1,356,416 sampled hits. Causal SLAM XY p95/max
13.58/18.57 cm; height p95 1.66 cm, yaw p95 0.390 degrees. No catastrophic
failure, but the original 10 cm XY / 0.3-degree roll/pitch diagnostic targets
are not met. This is not full terrain/navigation qualification. Final focused
suite at this checkpoint: **360 passed**, no skips.

Next failure-informed experiment: `peek_reverse`, reverse 15 cm, dwell 3 s,
return, using the SAME fixed startup turning area. The front-offset sensor's
steepest forward returns start beyond the parked front edge; moving backward
can overlap those observations with the assumed startup support. An outward
observation orbit moves the visible front annulus farther away instead. This
is a sensing-geometry hypothesis to verify, not permission to interpolate the
unknown band or enlarge the prior. Fixed forward/reverse exit bank and floor
denominators remain; this is a separate acquisition protocol, not a repeat or
replacement of the full-turn outcome.

**15 cm reverse experiment completed, but still no exit:** recorder
`revision-d9ac276727fa467598d0d4aae03ef52c`, native
`revision-4017efea2c2c45e78c9471fab5c02781`, mapper
`revision-81a48ab2e4094910bedcccf62f1983a9`. Exact map/advisory replay
`revision-0cd39ecd4f0144de932e845359edde8b`: 204 maps, **0/21** exit decisions.
Actual reverse displacement was **13.11 cm**, because the inherited 2 cm route
arrival tolerance is coarse relative to the requested small observation move.
GLIM/transport diagnostic `revision-02a1855bd03c41a1b0909551838d2fdd`: causal XY
p95 9.01 cm, vertical p95 2.51 mm, yaw p95 0.268 degrees. All 1,229 native ticks
and 7,792,916 transported points checked. Strict native geometry audit failed
one missing PhysX reference hit at tick 889 (other 127 within 2.75 micrometres),
so retain this as a coverage/integration diagnostic, not terrain qualification.

New independently declared `peek_reverse20`: 20 cm reverse, 5 mm arrival
tolerance, same 3 s dwell and <=3 cm/s, then return. Previous routes/tolerances
unchanged; no guard or terrain rule relaxed. Focused routes/profile/geometry
suite: **133 passed**. Recorder `revision-ff65b706376c419cbdda8db13d5461f8`,
native `revision-15156ce46c834bc6bed8b849a3f6c982`, mapper
`revision-5266638257a84e179cf8f60a239e12c3`.
Correction: `revision-2494f60a6f754e8fac7b951377f9f349` is the archived simulator
experience configuration, not the mapper output.

**20 cm reverse completed:** actual displacement 19.59 cm, 1,522 native chunks,
253 valid maps, 26 decision bins. Exact raw-map/advisory replay passed in
`revision-4a7053f9455d41129fa93279d83d2529`; **0/26 supported straight exits**.
Final floor support: startup 5/472, startup boundary–1 m 206/783 (26.31%),
1–1.3 m 577/865 (66.71%). Zero occupied samples in these surveyed bands.
The final map allows 10 and 20 cm forward moves confined to the startup
assumption; the 30 cm candidate reaches new ground and remains unknown.
This is not "no movement allowed": it is no tested connection to new ground.
Forward paths of 40–100 cm share about 0.02125 m² of unsupported wheel-sweep
area; a small physical area can still interrupt every tested route.

Recorded mapper/advisory combined processing p95/p99: 91.09/114.03 ms,
peak mapper RSS 205.16 MiB. Instrumented measurements, not both required resource
profiles or compute qualification. Simulator stopped after completed sealing.
Trajectory `revision-c42c1bf956a3461c860dffe2209e85f8`; pose/native diagnostic
`revision-e3dbca144d96412dadf4c78c8e1d8199` completed. Native geometry **passed**:
194,816 sampled hits, maximum residual 0.05436 mm; all 1,522 GT pairs and
9,658,547 transported points verified. First-published GLIM XY p95/max
6.70/10.21 cm, height p95 3.29 mm, yaw p95 0.317 degrees. Original diagnostic
pose target still fails (roll p95 0.314 degrees); no fitted alignment or target
relaxation. These results do not establish that pose uncertainty is irrelevant
at a 10 cm cell boundary, only that there is no gross frame mismatch.
HTML: `revision-166748cca0a847c788f92a07e84a45e8/startup-observation.html`.

Final code verification: **367 focused regressions passed**, no skips; Black
check passed on the seven new/changed observation/contact modules and tests;
Python compile, headless runner `bash -n`, and `git diff --check` passed.
Tests cover startup geometry, hazard veto, immutable maps, candidate-policy
replay, failed-turn corrections, reverse-route bounds, fixed denominators and
occlusion-before-range-filtering. No classifier, sensor or safety threshold was
changed to improve the reported scores.

Independent ideal visibility screen `revision-1b2ef36c5bb9448484a274cf6431a77f`
uses all 63,968 selected rays, flat ground and the calibrated chassis box, with
occlusion BEFORE minimum-range filtering. The parked mount sees no ground inside
the 61 cm circle: nearest return radius **73.49 cm**. Reverse 15/20 cm moves
that minimum to **58.68/53.76 cm**. A hypothetical lateral sensor offset of
32 cm reaches 27.70 cm, but excludes wheels/mast/relief and is **not a native
validation or sensor change**. This exposes the geometric blind region; it does
not justify filling it with free cells. A sufficiently overlapping observation
still must pass the actual whole-cell classifier and unchanged motion queries.

Next improvement cycle: use the saved 20 cm replay to locate the unsupported
wheel-sweep fragments and distinguish absent returns from measured ground
discarded by whole-cell classification. Test any partial-cell support model
against the existing hole/edge/raised-block fixtures before a fresh live run.
If observations are genuinely absent, compare a chassis-aware sensing/maneuver
change separately; do not simply mark the gap free or enlarge the assumed area.
The three observation protocols are different sensing doses, not interchangeable
repeats, and straight-path failure is not proof that every curved strategy fails.

Full-turn HTML: `revision-6665b7c06f2d44e9aae9d9782c719ae5/startup-observation.html`
(Chrome screenshot visually inspected); reverse-15 HTML:
`revision-41a0be10c79c4666abb21f6de07a41e9/startup-observation.html`.

## October 4 continuation — qualify the support decision before driving

Goal: reliable terrain-controlled progress toward goals, then exploration. A
retreat being available is not goal completion. Existing recordings and scores
remain unchanged.

Current experiment: place the real loaded rover on a fixed set of isolated
surfaces: flat control, narrow supporting rails, offset rails, one missing wheel
support, a front-axle drop, and an 8 cm raised block. Record all settling poses
and actual wheel forces. This tests the physical assumptions behind the smaller
support region; it is not a LiDAR/GLIM or driving test. Keep unstable outcomes in
the results rather than treating them as missing trials. No commands or ROS
mission stack run beside this fixture. Then implement the current-pose advisory
query and measure a single online workload before closed-loop integration.

Host archive and CUDA context/allocation preflight passed at this continuation.
No navigation policy has been promoted.

**Physical result:** all six cases completed, 480 ticks each. Inner run
`revision-435b09ef32fe47abb1e9d20726a73211`, launcher
`revision-7c3433ddf235453b9a3d1905ab0795b0`. Flat ground and both rail widths
(60 mm centred, 30 mm offset) settled at the normal 14.49 cm base height with
all four wheels loaded. With the front-left wheel over a 30 cm hole, the rover
remained level but that wheel had **zero support for all 60 settled samples**.
The 8 cm block lifted the opposite front wheel (zero support for all 60 samples)
and tilted the rover about 9 degrees. The front-axle drop tilted it 35.85 degrees.
This rules out treating stable body attitude as proof that every wheel is on
ground. The narrow-rail outcomes support studying partial support, not a blanket
free rectangle or permission to drive over unseen hazards. These are stationary
mechanics results, not terrain recall or closed-loop acceptance.

Next implementation: distinguish individual supported wheels from the legacy
all-wheels-grounded assumption when retaining tracks. Current-pose query context
is implemented separately from the old map pose; age/generation/frame tests pass.
One initial unit assertion encountered 1.4e-16 floating-point roundoff and now
uses the same 1e-14 algebraic tolerance as its adjacent transform assertion.
The first host age-audit invocation found the source archive's `/archive` alias
unavailable; no source was changed. Run that tool in the existing correctly
mounted dev container, not by rewriting archive contracts.

**Current-pose audit completed:** all 1,774 valid maps across the three captures
have a same-generation, receipt-causal current pose within the 200 ms contract.
Newest GLIM poses are about 183.33 ms old. Their maximum difference from the
grid's 200 ms-old pose is **0.71 / 1.72 / 2.25 mm** (turn / straight / combined),
with maximum rotation difference **0.073 / 0.099 / 0.120 degrees**. This timing
difference is not the coverage bottleneck; stale-pose motion still belongs in a
controller's clearance allowance. Audit runs `revision-737fc6498a794203b38f81d707eed01e`,
`revision-88b03d73b8e74d56952b3866386c981c`,
`revision-5e0ebaa266e143aa8ac2f1a5edf4a1bd`.

**Implemented:** per-wheel history masks; current-pose context with age/frame/
generation checks; a shared advisory that distinguishes present wheel occupancy
from ground remembered for later. Its default only retains LiDAR-confirmed
wheel patches, while the old all-wheels-grounded assumption remains explicit.
Raw terrain is never rewritten. The opt-in live output is
`/terrain_shadow/contact_advisory`: five small candidate checks at 1 Hz, plus
per-map status/age/timing. This is an inspection output, not a command-rate
controller or navigation permission. The capture runner now requires its
successful archive seal when `--contact-advisory` is selected.

**Runtime optimization verified:** union swept geometry once per query and clip
cells in one vectorized operation. All **179 original decision bins**, every
candidate/recovery result and all original map bytes still match. New replay
runs turn `revision-5e0e036f1cce48298fa3caf053ba1cb8`, straight
`revision-1c5125dcfecd4d9ab61d49220d66b935`, combined
`revision-de3965ee9e1041c682a870c53537bc91`; exact comparison reports
`revision-cc2cb64a167b4ffb8be63bc59b973cde`,
`revision-dbf5b6aa03ed4cbc9888d9243e2c6b1b`,
`revision-53dd270011a0460196197321732814b7`. Batch p95 improved by 15–20%.
Single-candidate synthetic benchmark `revision-51d41a099257493c8d5db3cf6ac2d95e`:
full-patch p95 **3.53–4.98 ms**, core **7.10–8.17 ms**, 70.85 MiB process RSS.
All 480 measured timings and 48 warmups retained. This excludes pose/history
updates and the mapper; it is not coupled compute qualification.

The prior live ROS entrypoint is preserved at
`revision-02da6dcf6934416b9a378258bbf84665`, indexed by its original SHA-256.
Replay may verify that archived entrypoint because it does not execute the ROS
node. **No mapper/reconstruction dependency gets this exception**; those hashes
and every original output still must match. This permits new opt-in ROS outputs
without making old captures unreplayable.

Next declared run: fresh headless `turn15`, fixed sensing/GLIM/terrain library,
existing sole GT acquisition driver and guards, contact advisory enabled. It
tests actual integration and timing, not terrain-controlled goal completion.
`slam_mapping` and `astrobot_launch` rebuilt successfully; focused runtime/seal
tests pass. Shapely 2.1.2 is staged separately on the archive, with native hashes
recorded in the advisory manifest; the mission ROS environment is unchanged.

**Fresh live turn completed, but the conservative movement policy failed.**
Recorder `revision-9376ff2c50194bc19326cc215d94ab05`, native
`revision-41c463b63ea84bbfa02dd4be0e337bcd`, mapper/advisory
`revision-e8fad238e4e94f37be0869782e55cc74`. All 1,937 native chunks and required
topics recorded; 322 valid maps/advisory updates. Exact raw-map AND advisory
replay passed (`revision-13d4ac5bf18846ffac5c77809cc38446`). Standing was available
at **33/33** decisions, but a 5 cm move or 5 degree turn was available at **0/33**.
Not one wheel patch passed whole-patch LiDAR confirmation, so history retained
only the startup prior. This rule is too restrictive for the sensing blind spot;
a successful recording must not be mistaken for usable navigation.

Live instrumented p95: mapper **56.73 ms**, advisory updates **16.43 ms**,
mapper+advisory **91.61 ms** (p99 **118.60 ms**), 211.45 MiB peak process RSS.
The combined workload does **not** meet the current timing target. A future
controller should check one proposed motion rather than five inspection
candidates synchronously, and the intended combined workload still needs both
resource gates. No timing threshold or watchdog was changed.

Fresh GLIM pose diagnostic `revision-de22fbf19302429aaecbffc56b907313`:
causal XY p95/max **7.28 / 11.42 cm**, vertical p95 **0.382 cm**, yaw p95
**0.329 degrees**, all 1,937 native GT pairs and 12,299,039 transported points
verified. The separate strict PhysX first-hit check missed **one reference hit
at tick 338** (the other sampled residuals there were at most 3.58 micrometres).
Keep that failure: this recording is useful for pose/integration/failure
diagnostics, **not terrain safety qualification**. It is not a large SLAM error.
Trajectory export `revision-975991eec2b64cdfa8a03decf4ae0a5d`.

Locally fixable issue found and regression-tested: the adapter unnecessarily
eroded the startup prior by the historical 2 cm margin even when prior and query
used the **same pose timestamp**. Relative pose-placement error is zero in that
case; historical samples still retain the original erosion. Added an exact
first-step regression. The smaller support region is now a separate opt-in
`--contact-support-core` experiment, not a default change. A paired diagnostic
on the just-recorded failure is running before choosing the next fresh capture.
Old adapter sources are preserved at `revision-d753f7abfdd446af842bdbd6837c569d`
and `revision-9fce50deebc6484ca9feca1279fd6366`; old raw map replay may verify
these **non-mapper** sources by hash. Changed advisory replay must be explicitly
labelled `--candidate`; exact comparison still rejects changed decisions.

Final pre-capture focused regression set: **231 passed**, no skips. Simulator
stopped after the sealed run; no capture artifacts removed.

Paired smaller-core + startup-placement fix diagnostic
`revision-8eb076f1f9654026be83d1c050657c64` retained all 322 maps and all 33
decision times. It improved small-motion availability from **0/33 to 3/33**,
but still **0/18 after 15 seconds**. Whole-patch LiDAR confirmation still created
no historical wheel tracks. This is not a sufficient improvement and will not
be promoted. The initial version of that report added recorded mapper timings
to offline advisory timings in its candidate table; that mixed total is **not
a measured runtime**. The reporting tool now omits mixed totals and labels
offline timings explicitly; the original artifact is preserved.

The limiting question is now what is known at deployment: a fixed cleared
staging area around the robot, or only its actual footprint. The user has been
asked this operating-contract question while investigation continues. Do not
silently enlarge the startup prior, infer all four contacts from stable attitude,
or count a GT-driven route as terrain-controlled progress.

Final isolated comparisons on the same failed-movement recording:
startup-placement fix alone **1/33** (`revision-f66bc59c77c54f138f8939087116ca70`);
fix plus smaller support region **3/33**
(`revision-71f336c6fbdc4656a15228b0d01f36c6`). Both remain **0/18 after 15 s**.
All 322 raw maps and pose/time exposures match; all source versions are archived
and offline timings are now correctly separated from live timings. The fix is
verified, but it is not a solution to the overall support deadlock.

Final focused regression: **233 passed**, no skips; `git diff --check` passed.
No C++ classifier or sensing geometry changed in this continuation. The new
smaller-core live flag is implemented but has **not** been run in a fresh live
capture; the conservative live run and its offline candidates remain distinct.
The simulator is stopped, dev container idle, and generated physical-fixture
artifacts are user-owned. No artifacts were deleted.

## October 4 — wheel support and local-navigation implementation

**In progress: support memory and offline comparisons implemented; live control
not enabled.** [Plain-language results](online-terrain-scenarios.md#update-support-memory-implementation).
The next milestone is useful, terrain-controlled local movement,
not perfect visibility in every cell. Keep the current sensor, GLIM configuration
and LiDAR classifier fixed while addressing the missing support under the robot.

Implementation order and improvement cycles:

1. Add separate support from a normally grounded deployment footprint, current
   wheel contacts and previously occupied wheel patches. Never paint unknown
   cells free wholesale, move the startup prior with the robot, or override
   an observed obstacle/edge. Keep original LiDAR maps unchanged.
2. Replay the completed turn, straight and combined recordings. Preserve the
   original 1 Hz decisions and original motion scores; separately test shorter
   forward/reverse steps and small turns. Compare current contacts alone,
   startup support and remembered tracks. Report failures as well as gains.
3. Tie historical patches to acquisition-time poses; reproject with the pose
   history available at each decision, withhold missing history, and bound memory.
   No short time expiry. Test corrections, resets, uncertainty, sparse contacts,
   grid edges and explicit hazards. Start with declared 0/1/2 cm erosion
   sensitivity, not a claim that those values bound real drift.
4. Add opt-in live advisory support/query output. Verify it agrees with replay,
   keeps raw terrain unchanged, and meets bounded CPU/memory requirements.
5. Validate loaded contact geometry, relative pose uncertainty, stopping distance
   and independent obstacle truth before enabling terrain-controlled commands.
   Integrate the shared query into forward, reverse and turning checks. Then run
   short closed-loop goals, followed by 10–20 m waypoint/revisit missions.

Acceptance remains per-scenario useful-motion availability of at least 95%,
zero observed false-safe motions against independent truth, and eventually at
least 95% goal completion under frozen deadlines. Support replay alone cannot
establish these claims. Guarded GT-driven recordings are diagnostic input, not
autonomous navigation. Current contact and startup support explicitly assume
normal grounded deployment; they are not inferred from command requests.

Initial implementation: `tools/local_terrain/contact_support.py` stores bounded
pose-anchored patches and checks geometric coverage inside each cell;
`tools/assess_contact_support.py` adds the matched causal replay experiment.
Initial focused tests: **47 passed** (`test_contact_support.py`,
`test_navigation_gaps.py`, `test_local_support_sweep.py`). Live integration,
resource qualification and closed-loop navigation are not yet implemented.
Next experiment: turn-only replay first, then straight and combined recordings.

Implementation experiments so far:

- `revision-aac0ba8e936a40b3a7f22373750e6aa2`: failed new replay check. The new
  scorer used nominal base/LiDAR mounting instead of the mapper's measured
  IMU/base chain (2.01 micrometres difference). Fixed the scorer, not the SLAM
  estimate or tolerance. It now also uses the mapper's own revision receipts,
  not the independent bag subscriber's callback order.
- `revision-f0dbb7dba2f34e2da13fc6ab40011910`: stopped the slow prototype after
  query cost grew from 1.95 to 22.25 seconds in its first four decision bins.
  Repeated polygon subtraction fragmented overlapping tracks. Preserved this
  incomplete run; replaced that hot path with per-cell double-precision GEOS
  unions, retaining the independent clipping implementation as a test oracle.
  No grid snapping, free-cell painting or hole filling.
- `revision-c8beb3e9a4ac4d76bc72091f4f81b64e`: all 321 valid turn maps reproduced
  exactly. Current contacts restore stationary support at **33/33** checks;
  the first support candidate still allows **0/33** moving actions. Query cost
  is around 2 seconds for three policies and 21 candidates, still not live-ready.
  Inspection found another scorer geometry bug: startup support was projected
  as a zero-thickness plane, while wheel queries include their 2 mm height band.
  A tilted body therefore produced tiny unsupported edge strips even for a
  straight step inside the startup footprint. A new fixture reproduced this;
  use the same declared contact-height band in both projections. No XY halo.

Next: rerun that geometry fix on all three recordings, explicitly compare
0/1/2 cm historical-position uncertainty, and investigate any substantive gaps
before live command integration. These numbers are diagnostic, not qualification.

**Matched replay results:** the geometric fix allows 5/10 cm straight steps at
the initial deployment. Current support is available at **179/179** stationary
queries across the three routes. With 2 cm historical erosion, the complete
support policy permits an expanded action at **40/82** straight decisions,
**22/64** combined decisions and **1/33** turn-only decisions. Original
goal-move-or-turn counts are **33/82**, **19/64**, **0/33** respectively. These
new action counts include reverse/smaller steps, so do not compare them directly
to the old forward/30-degree-turn metric. Recorded LiDAR maps remain exact.

Runs: straight `revision-f538a89d4a46485a80a1a74290a1d073`, combined
`revision-bbd1e77b300f4caa8afcd6de9f33744d`, turn 2 cm
`revision-13484d231fa84f2b9f0666aa61576323`; turn 0 cm sensitivity
`revision-84cb2e826d8e4cf8a83bfc3b7ed4dbf0`. Counts include every original 1 Hz
bin; replay also validates all **819 + 634 + 321** live valid maps and their
separate terminal invalidations. The prepared-query implementation keeps the
geometry policy but reuses each path across policies. Query-batch p95 is
**1.62 / 1.41 / 0.91 s** respectively (three policies × 21 candidates, excluding
historical pose placement). Much faster than the first prototype, but still
not live compute acceptance. No live command policy has been enabled.

Added area diagnostics and fixed-time geometry examples without changing
decisions. Final turn replays: 2 cm `revision-8f898427b5e74c1c974ebd0adfbb185c`,
1 cm `revision-cbc48c7d150d4d99a1465b0774799cbe`, 0 cm
`revision-aab0a71566ef4eab89bec44a81bd55d0`. Counts reproduce the earlier runs:
**1/33**, **1/33**, **4/33** expanded actions, respectively. The zero-erosion
case is an optimistic sensitivity, not permission to ignore localization error.
At +1 s its best rejected step lacks only **2.51 cm²** of support; at +15 s
the best rejected step lacks **7.20 cm²**. The 2 cm erosion itself removes
40 cm² from a 5 cm step near startup. This exposes a policy problem, not just
map decay: demanding complete area coverage of an assumed contact patch is
fragile at its edges. Tiny missing strips and large unseen gaps need separate
treatment; neither is automatically an obstacle.

Grid/patch gallery: `revision-84eb598891d141dc9f509038d552a6d8/support-gaps.html`.
It overlays untouched LiDAR cells, separate support and the missing area of a
5 cm forward step at 0/5/15/30 s. Erosion variants are selectable.
The original acquisition driver followed a route independent of this policy;
its later rejected poses are not evidence that a support-controlled robot
could reach them or that a different startup action would fail.

Static geometry cross-check (read-only USD, no Isaac world): the four wheel
collision meshes span y = +/-0.342630 m; each is **114.46 mm wide**. The old
assumed support patch is **120 mm wide**, slightly wider than the wheel itself.
Do not silently shrink the frozen baseline patch or call the new startup
rectangle an independently qualified physical contact area. Loaded contact
extent and a practical partial-support criterion remain unresolved. The current
startup rectangle is explicitly chassis plus assumed patch extents, not the
inflated 1.70 × 1.40 m acquisition guard.

Next scoped experiment: extend the existing isolated, zero-command loaded-rover
fixture to record actual wheel/floor contact manifold points, forces and
separations. Same lunar gravity, original asset, 480 ticks, awake protocol,
settling/safety checks and 240 s outer limit. Keep all contact frames in one
JSONL file, not one file per tick. New buffer-decoder tests reject truncated,
overlapping, saturated and nonfinite active contacts; unused buffer entries are
not treated as contacts. **16 contact/fixture tests pass.** The point manifold
does not by itself prove the entire rectangle between points is supporting
ground, and will not be used to silently narrow the frozen motion metric.
The idle selected development container was stopped for the isolated fixture;
no mission or robot commands were running.

**Loaded contact experiment passed:** inner
`revision-b8359b50f2d04e6abf2a77d4113fdcbf`, launcher
`revision-c7ebed8a015a4e58858b5ef8c0ae0aa0`. Host and image CUDA allocation checks
pass; 480 ticks retained, all four wheels have contact samples on every one of
the final 60 ticks. Per-wheel manifold spans approximately **47.6 × 82.5 mm**;
loads and settled height exactly match the earlier awake fixture. One 2.3 MiB
contact file. No driving; manifold extent is not a minimum safe support area.
The solver contacts extend slightly beyond the raw mesh bounds, so the static
114.46 mm mesh width must not be treated as an exact cooked-contact bound.
The idle development container was restarted afterwards; simulator remains off.

Next declared ablation: **40 × 60 mm central positive-support region**, strictly
inside the old 80 × 120 mm patch. Keep the original full wheel/body sweep for
occupied, edge and height-conflict vetoes, and keep support evidence geometry,
2 cm historical erosion, poses, maps, sampling and original motion metrics
unchanged. This tests whether the full-area support requirement is the bottleneck;
it is a new, unqualified policy, not a faster equivalent or a certified contact
model. First turn-only, then the other two routes if promising. Do not enable
commands or treat this as evidence of missed-obstacle/dropoff safety.

Core result on turn-only: `revision-19b47a6a05d94f5097332f7259b0c0f9`, **11/33**
expanded actions versus full-patch **1/33**, with the same 2 cm erosion. Helpful,
but insufficient, and the original larger actions remain 0/33. Regression suite
now **160 passed** including full-wheel hazard vetoes outside the smaller core.
Next hypothesis: the fixed pure-yaw/straight candidates do not follow the path
actually traversed during skid-steer turns. Add a **separate recovery metric**
that checks reversing the recent estimated path (net at least 5 cm or 5 degrees).
Keep every old candidate/count unchanged. Never join missing poses or call
withdrawal goal progress; geometry availability is not controller feasibility.

Recovery diagnostic on turn-only: `revision-ba2df6bcd87041f3ac0fb4de487565b0`,
**25/33** supported withdrawals; combined with ordinary smaller moves/turns,
**29/33** decisions have an option (after 15 s: **17/18**). Three remaining
withdrawal gaps are under 0.4 cm²; one early sample lacks enough traversed path.
Tightening the conservative rotation envelopes to 0.005 rad segments of the
same linear/SLERP path did **not** change those counts. That disproves envelope
padding as the sole explanation; no numerical gap tolerance was increased.

Full three-scenario core + recovery results (2 cm erosion):

| Scenario | Smaller move/turn | Supported withdrawal | Either option | Original larger goal move/turn |
| --- | ---: | ---: | ---: | ---: |
| Straight out/back | 74/82 | 77/82 | **81/82** | 38/82 |
| Turn + out/back | 46/64 | 57/64 | **61/64** | 21/64 |
| Turn-only | 11/33 | 25/33 | **29/33** | 0/33 |

Runs: turn `revision-4ba5f6424c7c4666a1fcd06cfc6ba0bc`, straight
`revision-2ec0c874487546828e6228355a3ab9db`, combined
`revision-3af4fc46150747329c7aa6997c446d76`. These are class/support-geometry
results, **not** independently safe-motion or goal-completion scores. The new
core deliberately requires less positive support; full known-hazard vetoes
remain, but unseen hazards and controller tracking still need testing. Do not
promote a core or withdrawal path merely because this table looks better.

Observed resource issue: an offline 512-anchor profile spent 0.845 s re-placing
unchanged patches and 0.661 s rebuilding all per-cell unions. The straight
diagnostic also reached about 1.2 GiB container memory while retaining every
past version of each pose. Implemented bounded geometry caching, incremental
cell unions with atomic replacement on corrections, and pose-history compaction
that retains the latest pose of every historical frame. A paired checker now
requires identical decisions, bins, support counts and original map replay;
only geometric area rounding at 1e-12 m² is allowed. Next: replay all three and
verify equivalence before crediting speed/memory improvements. Runtime metrics
remain offline diagnostics, not live compute qualification.

**Optimization verified on all three recordings.** Final replay runs: turn
`revision-a85be0edebdc49beb07ce0795a24a7b2`, straight
`revision-bd3f2c7995774e1e9ef22d571add96b1`, combined
`revision-5c9dbd5ce7ab4e12a2d4dbade8781ecd`. Paired comparisons
`revision-becba5a9b45d41aab18e215e5a8fd76f`,
`revision-c9a1ac31f216411faefa52875a9a1534`,
`revision-bdca29d0016645bb8a0b7f115a440637` match **all 179 decisions**, every
candidate/recovery result, retained-support counts and original map replay.
Query-batch p95 before → after: turn **1.485 → 1.194 s**, straight
**2.040 → 0.974 s**, combined **1.893 → 1.124 s**. These are entire offline
three-policy comparison batches, not one controller query or live latency.
No instrumented timing is counted as compute acceptance.
Final whole-process diagnostic peak RSS: **191.9 MiB turn, 225.2 MiB straight,
214.2 MiB combined**. This is not a coupled live-stack or adversarial resource
profile measurement. The final HTML was opened in headless Chrome and visually
checked, including its scenario labels, legend, holes and 10 cm grid overlays.

Final relevant regression command (Shapely-enabled host environment):
`python -m pytest -q -p no:cacheprovider tools/test/test_contact_support.py
tools/test/test_contact_support_assessment.py tools/test/test_contact_index.py
tools/test/test_contact_support_comparison.py tools/test/test_contact_support_report.py
tools/test/test_contact_manifold.py tools/test/test_navigation_gaps.py
tools/test/test_local_support_sweep.py tools/test/test_local_isaac_fixture.py
tools/test/test_local_vehicle_geometry.py tools/test/test_local_physx_reference.py
tools/test/test_local_costmap.py tools/test/test_local_pose_lookup.py
tools/test/test_local_motion_evaluation.py tools/test/test_local_bookkeeping.py`:
**175 passed, no skips**. Renderer-only test also passes after the final display
label change. Exact commands/environment setup are in the runbook.

Final grid gallery:
`revision-00cbe80725ad48339d4a29e7e084a6c7/support-gaps.html`, including full-patch
reference and optimized core-policy examples for all three scenarios. Earlier
figures/runs remain preserved. No captures were deleted, no production ROS
topics or command owners changed, and the simulator is stopped.

Remaining implementation/validation, in order:

1. **Support rule:** test the smaller required region against controlled rocks,
   dropoffs and partial wheel support. Known-hazard veto fixtures pass, but
   unseen-hazard safety is not established by benign-route replays. The
   turn-only family still misses 95% even on the broader move-or-recovery metric.
2. **Live advisory:** evaluate at the robot pose when a command is consumed,
   not simply the recorded map's roughly 200 ms-old reference pose. Account for
   actual relative pose/tracking error and measured stopping distance. Publish
   support and decisions separately; keep raw terrain unchanged.
3. **Runtime:** benchmark the intended online query workload and both resource
   profiles with the coupled mapper. The prototype has bounded storage (4,096
   contact samples and the existing 8,192-frame ledger), not unbounded mission
   history. Capacity eviction can remove old support; it is not a time-decay rule.
4. **Closed-loop goals:** short startup/obstacle scenarios first, with sole
   command ownership and evaluator-only terrain GT, followed by waypoint/revisit
   missions. Withdrawals must demonstrably help observation/progress rather
   than inflate an availability statistic. No Nav2 or exploration promotion yet.

## October 4 — fresh online scenarios with the false-dropoff fix

**Completed:** three online scenarios, one recording race fixed, near-wheel
visibility still limits useful motion. [Readable report](online-terrain-scenarios.md).

Initial experiment declaration: run `straight_slow` (2 m out/back), `combined10` (15-degree
turn plus out/back), then `loop16` if the short runs complete. Use the tested
`revision-70647a2b742c446eb61bd3d7141ea011` library, frozen GLIM config
`revision-4c7b70a4acb44a07acb1bca0d63e3692/config`, selected optical v2,
physical IMU, GT initialization and synthetic RTK. Headless `lean_v1`, import
stat cache off, ordinary GC and all motion/lease/watchdog checks remain enabled.
The longer loop uses the declared `loop16_slow_host_v1` execution profile.

Check complete capture and recorded live maps, actual GLIM/GT error, sampled
clear-floor recognition, short-move/turn availability, startup and missing
outputs. Preserve all failures. Floor recognition is not obstacle recall;
guarded GT-driven motion is not autonomous navigation. Compare the loop using
the existing fixed-time queries and denominators. Results and the evidence-based
change from a long loop to a turn-only diagnostic follow below.

First straight attempt: recorder `revision-32d9dc402e254a69908e72f43b483738`,
native `revision-0c57c0b52fe14e03a5a2ab11b7919d63`, mapper
`revision-19044e4dede149d5b38412ca90cf0a0b`. The physical route completed, but
the recorder failed at shutdown with an empty/partial JSON read. The finish
barrier had already been read; `natural_holdout.json` was being created directly
while the recorder polled it. This attempt stays failed/ineligible. Simulator
log preserved in `revision-71607ef0533f48dc8defcea185734a13`; simulator stopped.

Fix: moving completion now uses the existing exclusive atomic marker helper,
after command shutdown and native sealing. No exception suppression, changed
deadline or changed sensor/terrain logic. New tests read during serialization,
check failed sealing cannot publish success, and preserve exclusive creation.
Natural/driver/recorder-guard regressions: **144 passed**. The scorer now handles
short route distance/heading goals and their archived survey, while preserving
loop goals exactly; motion candidates still omit reverse (explicit report limit).
Scorer/export/floor/support/route/reassessment regressions: **120 passed**.
These small host tests overlapped part of the failed first capture; its timing
is not isolated compute evidence. Restart the same straight scenario fresh.

**Straight repeat completed and sealed:** recorder
`revision-f3860dc7476648d2b24f947986de6be6`, native
`revision-67a57265a79742948c0237f244e1f5ed`, mapper
`revision-7684773b78ae49c68226344c3214c67b`; simulator log
`revision-ad9fb4c57ecc46afa6e6d1a8a215e601`. 613.72 wall seconds, 81.90 seconds
of native evidence, 3.952 m actual travel. All **4,915** native ticks integrated,
**819/819** live class maps recorded; terminal invalidation separate. Mapper
p95/p99 **50.79/57.18 ms**, peak **226.59 MiB**. Atomic shutdown fix verified
live; isolated ROS recorder regressions also **29 passed**.

Assessment `revision-9856036df5d14960bfc266d797b66e95`: front 1–2 m surveyed
floor **98.18%** free, front 0–1 m **74.08%**, rear 0–1 m **42.97%**; **zero**
false-occupied surveyed-floor samples. Post-startup move-or-turn only **32/67
(47.8%)**; all-time **32/82**. Refusal stretches +0–22 s and +55–81 s. Reverse
motions are not candidates, but stationary support is also absent in many
samples, so this is not solely a forward-goal scoring artifact.

Exact replay attribution `revision-2b2f4387161f4d308b477641440c2851` matches all
maps: unknown under-wheel exposures are **4,769 no returns**, **821 unsupported
extent**, **106 insufficient area**, **zero negative-edge warnings**. The fix
holds; straight-only visibility/support is still a limitation. Do not fill these
cells as free. Test changed viewing angles next, before escalating route length.

Trajectory `revision-f7d137b8a8ae496285b9973af5ba4d3a`; pose diagnostic
`revision-ac4b5102a84c4ffea470b32637200033`: first-published horizontal p95/max
**9.56/14.05 cm**, vertical p95 **1.16 cm**, yaw p95 **0.314 degrees**. All
30,788,124 timed-window points verify against native inputs. The separate strict
PhysX recast gate fails on one missing reference hit at tick 3,787 (successful
hits there differ by at most 2.63 micrometres). Preserve that failure; it does
not explain the measured map gaps or authorize calling this full qualification.

Experiment choice after the straight result: run `combined10` as planned, then
the existing `turn15` route (+15°, −15°, return to initial heading), rather than
immediately extending distance. This isolates whether a small observation turn
can establish near-wheel support. The straight run did not meet useful-motion
coverage, so a long-loop repeat is deferred, not counted as passed. All three
routes keep the same mapper/configuration; no classification changes this cycle.

**Turn + out/back completed:** recorder `revision-5fc129e618184220b3f7b341625b1e9b`,
native `revision-c067b3c7c9e649b389f4fdd95a4723fe`, mapper
`revision-f1d62591736e43fd9d6f983c412b98b6`, log
`revision-0395ba5656b14387ae7d04765fc5a3e7`. 513.30 wall seconds, 63.40 native
seconds, 4.096 m actual travel. **3,805/3,805** inputs integrated and **634/634**
live maps recorded. p95/p99 **52.90/58.87 ms**, peak **220.09 MiB**.
Assessment `revision-39bfda5344294c56a4a7ae0239a595b4`: post-startup move-or-turn
**19/49 (38.8%)**, all-time **19/64**. Front 1–2 m clear-floor recognition
**98.00%**, front 0–1 m **64.20%**, rear 0–1 m **36.46%**. No false-occupied
survey samples. Refusals +0–19 s and +39–63 s; a small initial turn did not
solve support coverage. These routes have distinct surveys/doses, not a paired
algorithm comparison or evidence that turning makes the mapper worse.

Attribution `revision-7b56a1dd2b7d49a2abbf5310d0072c94` matches all maps:
**5,858 no returns**, **295 unsupported extent**, **82 insufficient area**,
**zero negative-edge warnings** under current wheels. Trajectory
`revision-8531771242b1472ab3d8ca71699fabc3`; pose diagnostic
`revision-e52c6641650f46acafbb4685900fae66`: horizontal p95/max **12.25/14.69 cm**,
vertical p95 **1.07 cm**, yaw p95 **0.345 degrees**. Every native tick and all
24,088,536 transported points checked; strict PhysX reference check **passes**.
Next: complete the declared turn-only sweep, then summarize limits and priorities.

**Turn-only sweep completed:** recorder `revision-a346de9413814ae8a49ed46b8dcba49c`,
native `revision-675bb023c989428ab9aaae49bc49238e`, mapper
`revision-3f6e38493f724cd78751561541487082`, log
`revision-911c78d7744047ccb2f80105c772f64a`. 303.60 wall seconds, 32.17 native
seconds, 0.229 m cumulative translation from skid. All **1,931/1,931** inputs
integrated; **321/321** live maps recorded. p95/p99 **60.21/64.67 ms**, peak
**202.68 MiB**. These are concurrent live mapper timings, not full qualification.
Assessment `revision-3f9240f897d541bfb374b7fbf242a82b`: **0/33** all-time and
**0/18** post-startup move-or-turn availability. Front 1–2 m floor recognition
**92.12%**; front/rear 0–1 m **22.64/3.89%**. Zero false-occupied floor samples.
Its 2–3 m survey bands have only 16/142 sample exposures; do not interpret those
percentages as broad long-range coverage. HTML now displays denominators.

Attribution `revision-d08bed0f7721495fbc94af61fdb94718` matches all maps:
all **5,322** unknown under-wheel exposures are **no returns**, no negative-edge
warnings. Trajectory `revision-e11eaa7abb6241d6b6314135fc044021`; pose diagnostic
`revision-1d7fd05aaf5e4ad1b486dc1f3ada0c24`: horizontal p95/max **9.16/12.64 cm**,
vertical p95 **0.47 cm**, yaw p95 **0.321 degrees**. Native reference passes;
all 12,259,340 timed-window points checked against native data.

Counterfactuals, not changed motion permission: ignoring only current unknown
wheel cells still gives **0/33** actions on turn-only
(`revision-83974103c1f94792aab258cda12cd4db`), and only **35/82** instead of
32/82 on straight (`revision-f7d855862d5243d9a4fdd249167c552c`). New ground in
the swept path remains unseen. A small observation turn or current-contact
assumption alone is not a demonstrated solution.

Cycle outcome: three completed online scenarios, no missing live maps, bounded
SLAM error, no recurrence of the false-dropoff cause. **Short-route startup/
near-wheel movement availability remains insufficient**. No obstacle recall,
autonomous navigation or new long-loop qualification. Final focused host tests:
**207 passed**; isolated domain-179 ROS recorder tests **29 passed**. Simulator
stopped; all attempts retained, nothing deleted. See the plain-language
[online report and next cycle](online-terrain-scenarios.md).

Final fixed-time HTML galleries: straight `revision-3fb424165af846f0b74281757563ea26`,
turn-and-drive `revision-992fb14b6b964e16a1556449247acfce`, turn-only
`revision-9aaf2c46099c42f58fbdc3bd4ef30dc5`. Earlier report renders remain
preserved. HTML now names the actual route/reverse limitation, reports observed
rejection counts rather than assuming a dropoff cause, and shows floor sample
denominators. Final host regression rerun after these edits: **207 passed**.

## October 4 — fix false dropoff refusals, then repeat the same assessment

Acceptance for this cycle: reproduce the cause on the completed loop, improve
the unchanged short-motion/floor scores, preserve real-hole/partial-face/small
obstacle regressions, and run both existing resource profiles. Do not change
SLAM poses, evidence lifetime, route, ray layout, terrain thresholds or scoring
denominators to obtain an improvement. A candidate remains opt-in until tested.

Added `reassess_local_terrain.py`: replay the recorded input placements and
projection schedule with a byte-exact baseline mirror, then score a separately
built library with the same floor samples and 300 motion queries. Diagnostic
mode also attributes rejected wheel cells to low returns, pooled normals or
native boundary evidence. The first diagnostic build
`revision-d4e5a3b8c0194493a65766cb57e4000f` failed because deferred classification
does not support the diagnostic ABI. Preserved that attempt; the build wrapper
now rejects that combination before allocation. Immediate-classification
diagnostic build `revision-3ddfdc9f9b694546a541d177ca362bc7` succeeds. Its full
replay must match every baseline output before any attribution is trusted.
Initial build-wrapper and navigation-query regression: **18 passed**.

Attribution `revision-d0913b7bc11248d899838ac1b184387f` matches **2,992/2,992**
baseline maps and the original motion/floor scores. Of the 2,756 rejected wheel
cell exposures, **2,750 are native boundary warnings, six pooled-normal warnings,
zero low-return warnings**. Found a density-dependent test: the square root of
the *sum* of squared native residuals was treated as an observed >5 mm geometric
deviation. That upper bound can grow with return count even when every actual
deviation is small.

Opt-in `--pointwise-boundary` keeps the same fitted slope cap and 5 mm threshold,
but, when that upper bound is inconclusive, checks every actual return against
the fitted native surface. No RMS averaging or outlier removal. Current-input
point-to-layer associations preserve actual clustering, including overflow;
only resulting evidence flags enter the existing immutable buckets. Deferred
and eager paths share the same decision. No raw history is added to retention.
First candidate compilation `revision-c306b4bbe8b14a5685cdd5631ea277e6` failed
on a misplaced scratch initialization; corrected before any test/replay.
Build `revision-70647a2b742c446eb61bd3d7141ea011` succeeds, both C++ contract
programs pass, and **17 new density/outlier/expiry fixtures pass**. Candidate
ordinary/local/rolling/floor regressions: **304 passed**. Full replay is underway.

First partial-face suite invocation used the modern baseline where one historical
control expected the pre-two-path false-normal bug: **604 passed, one baseline
control failed** (not a candidate false-free failure). Re-running the unchanged
suite with its documented legacy baseline; keep the failed invocation visible.

**Completed:** first candidate replay `revision-10124f9f4c3242ecb295686136fb6a1b`
raises after-startup move-or-turn availability from **202/285 to 283/285
(70.9% → 99.3%)**, and forward 1–2 m clear-floor recognition from **94.82% to
99.31%**. Final diagnostic comparison `revision-cf3189e2c434446e8aa4857bd937547a`
matches the original baseline and verifies immediate/instrumented versus
deferred/uninstrumented candidate byte equality on **all 2,992 projections**.
Native-boundary refusals under current wheels drop **2,750 → 0**; six pooled-normal
cell exposures remain. The long +152–212 s refusal stretch disappears. Remaining
post-startup sampled refusals: +214 s (two unsupported-extent cells), +245 s
(one pooled-normal cell). Startup remains unsupported through sampled +13 s.
No threshold, pose, frame, ray, retention or evaluation-denominator changes.

Final verification: **605 legacy-parent hazard tests**, **304 candidate terrain/
local/rolling/calibration tests**, **17 new pointwise fixtures**, **158 eager/
deferred/diagnostic comparisons**, **50 serial/OpenMP comparisons**, **50 unchanged-
mode comparisons**, **145 evaluator/report/query tests** all pass (groups overlap;
do not sum them). Both C++ contract programs pass in parallel and serial builds.
The failed initial historical-baseline control and compilation attempts stay
documented above. No simulator was started and no captures were deleted.

Both unchanged resource profiles pass: layer-capacity run
`revision-db1f90a7bc034c2e9f4261ea044ea0e0` has **43.26/56.67 ms p95/p99,
1,016.05 MiB peak**; dense-flat `revision-f3fdc3dcaa3a41e8a3d13109b02a7fe8`
has **37.31/43.43 ms, 472.69 MiB**. Capacity memory remains close to its limit.
The tested advisory candidate is
`revision-70647a2b742c446eb61bd3d7141ea011/liblocal_terrain.so` (`e452b2b6...`).
It is explicit opt-in, not installed in Nav2. New inspection instructions select
it for the next guarded capture; **no new live result is claimed**.

[Plain-language fix and reassessment](false-dropoff-fix.md) includes all fixed-time
before/after grids. HTML: `revision-803fe3e1a6f74edb95f4562ce6d4f9c8`;
inspected +180 s preview: `revision-381dd459574e4d1f93665f6d3040e3cd`.
Next is fresh guarded live validation and hazard-route/startup assessment, not
globally filling unknown cells or optimizing the remaining isolated refusals
at the expense of real dropoff protection.

## October 4 — do the gaps actually prevent a useful next move?

Current experiment, declared before scoring: reuse the completed October 4 loop;
no simulator restart, new driving, classifier change or relaxed input gate.
`tools/assess_navigation_gaps.py` reads actual recorded class maps. Score every
map against the independent 5 cm clear-floor survey, split front/rear and
0–1 / 1–2 / 2–3 m distance bands. Unknown and outside samples remain counted.
This measures recognition of sampled clear ground, not rock/hole recall.

On the first map in each fixed one-second bin, test 25/50 cm straight moves and
two-segment steering moves ending at ±30°, plus ±30° observation turns.
Require at least 5 cm progress toward the next route translation waypoint for
a move to count as progress. Turning is reported separately: it may gather
information but is not proof of either visibility gain or goal completion.
Sweep the full assumed 80×120 mm patches at measured wheel locations. Compare
that with requiring the entire 1.70×1.40 m acquisition footprint to be free.
Unknown between wheels is not automatically missing wheel support; occupied
cells anywhere in that footprint still block. A separate “no known obstacle”
score is an optimistic comparison, never motion permission. No localization
margin, measured braking, physical tire contact extent or obstacle truth is
claimed. Startup and turns remain visible; 1 Hz motion queries cannot establish
that every intervening map permits motion. Initial focused regression: 43 pass.

Hypothesis: map-wide unknown percentages overstate navigation impact, but gaps
under wheel sweeps can still prevent starting or turning. Measure this before
changing the classifier. Results and plain-language next steps follow below.

First completed assessment: `revision-282097f0d80a4d98aa4275dd31f7d395`,
82.31 s offline, all **2,991/2,991 recorded maps** and 300 fixed-time motion
samples. Short goal progress: **187/300**; progress or turn: **202/300**.
Requiring every body-footprint cell to be free reduces progress to **67/300**.
All tested candidate motions are clear of *known* occupied cells. Of 98 samples
with no supported move/turn, **96 already lack support under current wheel
patches**, with median four unknown cells there. The longest sampled no-action
stretch is +152 to +212 s. These are map refusals, not observed physical traps.
Known-clear floor in front is marked free **89.87% / 94.82% / 91.34%** at
0–1 / 1–2 / 2–3 m; false occupied is zero within 2 m and 46 sample-exposures
in the front 2–3 m band (0.00095%). No missed-obstacle score is available from
this clear-route floor survey.

Observed-failure follow-ups: exact replay to identify why the *current* wheel
cells are unknown, and a separately labeled optimistic comparison that ignores
unknown cells touched by the current patches but still rejects unknown new
ground and every occupied cell. This comparison does not implement a policy:
being supported on part of a 10 cm cell does not prove its whole area safe.
Its purpose is to measure whether validated current-support memory would help.

Follow-ups completed: exact replay `revision-2af143de7f784820a3b9b57f69006c5d`
matches all **2,992 projections / 17,949 native chunks**, and matches the original
wheel-query counts at all 300 sampled times. After 15 s, **2,653/2,665 unknown
current-wheel cell exposures (99.55%)** carry the possible-dropoff rejection;
12 lack full-cell extent and **zero** have no returns. The current-support
comparison `revision-666b24331b2047aab1fe73a080886683` improves move-or-turn
availability only from **202 to 213/300**. Revised conclusion: support memory
alone is insufficient; diagnose the specific false edge trigger before changing
retention, sensor rays, or filling gaps. No classifier/safety-policy change made.

[Plain-language findings and next cycle](navigation-gap-assessment.md), including
the HTML gallery and swept wheel-path examples. Report artifact
`revision-97f67794578d4475b6f2f7ecda899d1d`; inspected preview
`revision-91840edd103646ed82afcf740aed4c3a`. Earlier renderer-only preview
`revision-b30cbe9110e344bea9810fc67efc653a` lacked viewport clipping and is not
used; source grids and HTML were unaffected. Final affected regression:
**132 passed**, including start/interior obstacles, wheel-vs-body gaps, outside
cells, goal direction, frame translation, reason preservation and report checks.
The simulator remains stopped. No bags/artifacts deleted. Next implementation
target: identify low-return vs fitted-normal vs native-boundary rejection in a
retained +152–212 s case, preserve it as a fixture, and improve that rule while
retaining hole/partial-face protections. Fresh hazard-route recall and autonomous
startup/progress remain separate tests, not established by this clear-route run.

Plain-English current status: **the normal 16 m loop now completes and seals**;
full-route pose and grid diagnostics are finished. Earlier long runs stopped
because simulator garbage collection and delayed ROS servicing together exceeded
the unchanged 500 ms heartbeat limit. Lean simulator startup reduces retained
objects, while recorder storage checks and simulator grant reception now run
without blocking their ROS servicing. These are scheduling changes, not weaker
safety limits or terrain-classification changes. Lazy environment imports and
exact survey lookup subsequently reduced simulator-critical work. The final
pre-loop regression has **493 host / 44 isolated ROS tests passing** (final
diagnostic/report revision: **495 / 44**). Recorder
`revision-d4d0f081b64f47caab3831bf2154ab08` completes in **2,027.64 wall s**, with
**17,949/17,949 native ticks integrated and 2,992 projections**. Capture completion
is not yet independent terrain-safety, compute or navigation qualification.
First-published GLIM XY p95/max/final is **15.22 / 35.70 / 3.85 cm**, without
fitted alignment; no catastrophic drift observed. Final nearby unknown count is
**28/900 cells**, but turn/startup gaps remain. Compute p95 is **52.54 ms**, above
the 50 ms target. The strict reference audit fails on **two missing PhysX re-cast
hits out of 2,297,472 attempts**, not a large pose or range residual. See the
[complete-loop report and HTML gallery](long-run-gap-report.md).
Archive capacity is no longer the blocker; no files were deleted. Earlier failed
attempts and the agent's missed overnight follow-through remain documented below.

Historical result (September 29 evening): bounded verified-no-change SUBMAP/GLOBAL
handling is implemented. Candidate replay preserves **1,303/1,303** maps from
the originally blocked loop and **633/633** maps from a completed short control;
273 affected tests and both core resource profiles pass. A fresh live run hit an
independent native sensor XYZ/range discrepancy (one 12.94 m return, 1.045 mm
residual versus the unchanged 1 mm limit). A new guarded run includes same-tick
failure-only PhysX diagnostics crossed **four bookkeeping batches**, including a
2,227-frame GLOBAL batch, and continued mapping. It reached **13.79 m / 228.40 s
of native acquisition** before a simulator heartbeat failure coincident with a
404 ms full GC. Causal XY p95/max is **15.59/25.94 cm**; revised **12.79/15.50 cm**.
Both diagnostic pose targets fail; no observed runaway in this incomplete prefix.
Mapper shutdown also exceeded five seconds while re-reading 5.84 GB for hashing,
leaving an empty seal. Streaming write-time hashing fixes that scaling bug in
the latest source; exact independent read-back on all 5.84 GB passes, writer
close/hash takes **0.047 ms**. A subsequent viewport-disabled moving attempt
closed its failed mapper archive with all three independent disk hashes matching
(including 1.72 GB native data). That attempt stopped on another native XYZ/ray
mismatch before the previous stall point, so viewport stall mitigation remains
unproven. Follow-up isolated tests reproduce the PhysX discrepancy exactly;
centering and collider simplification changes do not fix it. An independent
NPZ writer correction eliminates per-tick disk read-backs with 100/100 real
ticks byte-identical; final affected regression is **286 passed**. The strict
sensor-consistency contract has now been changed with explicit user direction:
millimetre XYZ/range differences are diagnostics, not fatal errors; only gross
discrepancies above 5 cm fail this check. Native values remain unchanged.
See the [capture I/O audit](capture-io-audit.md).
Displayed maps show ~95–96% measured support after travel, but short-run
return/startup gaps remain ~1.7–1.8 m across. No completed long loop, independent
terrain-safety qualification or promotion. Next: reduce acquisition/GC and
archive-check overhead without relaxing guards, validate the new writer live,
complete the coupled loop, resolve sensor discrepancy, assess pose errors, startup support and
independent wheel/obstacle scoring. See the
[small report and grid gallery](long-run-gap-report.md).

### Latest verified checkpoint — not full qualification

October 4 — extended attempt `revision-55b34c12ada74d869471c67946cc7e2a`
failed after **1,938.23 wall s** (3,600 s allowance). Native
`revision-f6d51c114a4b4fc08eb5a57098d05019` records the same-stamp GC and motion
heartbeat failure; last commanded pose is sim 258.26668 s, leg 6. Recorder also
observed stale clock/request/IMU; last GLIM status and mapper status were healthy
(14,159 ticks / 2,357 projections), not evidence of completed mapping or pose
accuracy. Recorder maximum readiness check was 130.19 ms. Failed artifacts are
preserved. New hypothesis: remaining simulator-wide retained containers make
full-GC traversal too expensive. Added an opt-in, **stationary, pre-request-only**
heap ownership profile; it deliberately terminates before any motion grant and
cannot qualify a route. No GC disabling/freezing, threshold or watchdog change.
Next: identify retained storage, regression-test an evidence-equivalent fix, then
repeat the complete coupled capture with active follow-through.

Stationary diagnostic `revision-762140599486402ca8b8b0e99275ff39` completed
before request publication, with no motion records. Startup full GC **494.45 ms**;
**829,705 tracked objects**, including **10,318 retained frames**. Top frame
locations are Kit import-stat caching and Python import machinery. The installed
Isaac 5.1 `omni/ext/_impl/stat_cache.py` caches exception objects and re-raises
them, retaining their tracebacks. No remaining multi-million-element Python
terrain-index list was found. Log preserved in user-owned
`revision-bdc0fd4aeef04b0e97a706ecc6120bca` after a root-owned native-directory
log destination rejected the watcher write; simulator explicitly stopped.
Next experiment: opt-in `OMNILRS_IMPORT_STAT_CACHE=false` forwards Kit's
`/app/extensions/pathStatCacheEnabled=false` setting at startup. Default unchanged;
no monkey-patching vendor code, GC suppression, physics or sensing change. Added
three explicit pre-authority collection timings to the diagnostic. No claim yet
that reducing cached import frames solves the measured runtime pause.

Cache-disabled stationary comparison `revision-2c68231354924391bebce040112dfa24`
(log `revision-8401da2a7ff8455395f459f6130307e6`) has **811,394 tracked objects**,
**zero retained frames**, but three full collections still cost **465.26 / 473.06 /
480.82 ms**. Scene, clearance and full stationary sensor-audit JSON match the
baseline exactly. This is a verified retention reduction, **not a stall fix**;
no moving retry justified by this result alone. A second, separate experiment
selects a lean Isaac experience (`OMNILRS_SELECTED_EXPERIENCE=lean_v1`). The builder
merges and preserves upstream base/Python settings, resolves app-directory tokens
to their original location, records source/generated hashes, and removes only
unneeded direct extension dependencies. Default startup remains unchanged. The
required sensor/ROS dependencies and inherited settings must pass stationary
checks before motion. Tests cover exact config round-trip and refusal of changed
upstream dependency contracts.

Lean-experience stationary diagnostic `revision-172bfba79d1242128ece7697baa3a847`
(log `revision-d60d7eb8f3ff451384bcf6c32ef18094`) completes with **748,361 tracked
objects**, zero retained frames, and three full collections at **431.06 / 417.99 /
416.31 ms**. Baseline scene, clearance, stationary sensor audit and graph contract
all compare exactly equal. This reduces GC cost but leaves a tight live margin;
it is not a demonstrated stall fix. Startup and generated experience provenance
are recorded in each new native manifest. Both startup switches remain opt-in;
default and GUI behavior are unchanged. Final focused pre-retry regressions:
**173 host tests passed**, plus host/dev CUDA allocation and archive checks.
Fresh full-loop trial starting October 4 06:49 JST with
`SELECTED_ISAAC_EXPERIENCE=lean_v1 SELECTED_IMPORT_STAT_CACHE=false` and matching
`loop16_slow_host_v1` recorder/simulator budget. No classifier/library, route,
acquisition dose, GC thresholds or lease/watchdog change. Results pending; active
follow-through is required before any success claim or further moving retry.
Recorder `revision-177c2b2e8a0c436e9d54fd2f358377be`; native
`revision-9babd8d50f5f4f5a85e36cc50d884c3f`. Simulator launched at 06:50 JST,
exact container `2520883619a7a3c3359a917528c0fa4f34bb004e8fae30a163e0225b164ffdf6`.
Watcher now guarantees simulator stop even if preserving its log fails.

Lean moving result: **failed**, recorder `revision-177c2b2e8a0c436e9d54fd2f358377be`
after **1,456.41 wall s**, native at **235.18335 sim s** on the return leg. Mapper
`revision-5d48a652c4b046ccb48bf8bbc0ef7d11`. In-motion full GC **382.91 ms**,
native callback **441.15 ms**. Recorder clock/IMU/pose/map checks remained healthy,
but the 5 Hz request heartbeat expired; simulator lease age was **575.88 ms**.
Thus lean startup reduces measured GC cost but does **not** fix the coupled
deadline. Failed evidence/logs retained, simulator stopped automatically.

Next scoped scheduling revision: exact recorder filesystem/config checks run in
one bounded worker while ROS callbacks continue. Every readiness/grant consumes
a new, identity-matched result with an unchanged 500 ms freshness bound; no
cached permission, unbounded queue or late-lease revival. Pending/error/mismatched
checks cannot grant. Worker shutdown cannot block revocation/failure sealing.
Simulator motion requests publish once per actual physics tick rather than the
old 200 ms wall throttle; no independent timer can claim stalled physics is alive.
Added single-flight, error/identity/reuse/deadline and actual ROS storage/DDS
fixtures. A declared `turn15_gc_stress_v1` small-scale test forces full collections
after native ticks 240/480/720 and adds 110 ms to each actual recorder storage
check, modelling measured long-archive cost. This instrumented profile requires
matching recorder/simulator declarations and is explicitly rejected by nominal
holdout qualification. Route/sensing/GC thresholds/lease rules remain unchanged.
Stress result pending; do not start another long retry until this is inspected.
Pre-stress revision passes **138 host / 40 isolated ROS tests**. Short stress
recorder `revision-b74b0f03e96342c9ab892a7b18fba62a` started October 4 07:26 JST,
with host/dev CUDA and archive preflight passed. Result pending; exact simulator
container `54ef3b44c171b83c62ea51d8be66351f69577e853ea1e90e3d47da5f0489e68e` has
a seal-triggered log/stop watcher with unconditional shutdown cleanup.

First stress outcome: recorder `revision-b74b0f03e96342c9ab892a7b18fba62a`, native
`revision-323492292ca34428af4c162d4c1f352e`, mapper
`revision-8a0b313b86bb48e7bf1fa091637e4289`: **failed at the third injection**,
26.16667 sim s / 181.64 wall s. All recorder health checks stayed true. Forced
collections were **365.82 / 360.33 / 393.51 ms**; maximum native callback
**441.98 ms**, but simulator's last processed lease aged **552.48 ms**. First
two stress events survived. This verifies the recorder scheduling improvement
under the declared load, but exposes the remaining simulator-side grant-service
delay; not a passed stress route. Evidence retained and simulator stopped.
Next scoped fix: grant reception uses a dedicated ROS node/executor thread.
Only grant validation runs there; TF/PhysX/world access stays on the simulation
thread. `ShadowLease` terminal expiry/identity rules are unchanged. Added real
DDS regression with no main-thread spinning, dead-publisher expiry and refusal
of late revival, plus callback-failure/thread-shutdown tests. Repeat the same
three-injection stress protocol before a nominal long retry.
Grant-receiver revision passes **104 focused host / 42 isolated ROS tests**.
Second stress recorder `revision-e984898dd7214da0966957ab703bb973` started at
07:33 JST after fresh GPU/archive preflight. The stopped task simulator was
restarted, not a second simultaneous world; its container ID is unchanged from
the first stress attempt. Watcher log preservation is now limited to this
recorder's start time so historical container logs do not mix into the new report.

Second stress result: **collection completed** in **304.76 wall s**. Native
`revision-1160059c745443b88c6adc0a97bef118`, mapper
`revision-01bc741ecf074728a85a6b2eba1a4fc5`: all **1,970 ticks integrated / 329
projections**, completed/closed with no mapper failure. All three injections
recorded at ticks 240/480/720; forced GC **343.91 / 334.78 / 349.33 ms**; maximum
native callback **430.32 ms**. Recorder storage-work max **164.66 ms**, readiness
max **15.30 ms**, all final health checks true. GC pauses were shorter than the
failed stress run, so this live comparison alone cannot isolate the scheduling
effect. Added a separate-process DDS sender plus fixed **400 ms GIL-held pause**
and 40 ms simulated I/O fixture: old main-thread-only lease handling expires;
dedicated reception survives. Dead sender and late-revival checks remain strict.
Final ROS suite now **44 passed**. Stress evidence stays explicitly instrumented,
not nominal timing/terrain qualification. Independent seal/transport verification
and a fresh nominal long-loop retry follow.
Independent stress-seal verification matches all three mapper file hashes;
native/ROS transport verifies **1,970 ticks / 329 windows / 12,507,468 points**,
including both boundary windows. A first ad-hoc verification command lacked
`PYTHONPATH=tools`; corrected runner completed successfully without changing data.
Final pre-long-retry suite: **487 host tests passed** with the frozen real terrain
library, **44 ROS tests passed**. Host/dev CUDA, archive and diff checks pass.
Next nominal loop has **no forced GC or injected storage delay**; it retains lean
startup, cache-disabled imports, concurrent exact storage checks, tick-driven
requests and independent grant reception. Same 16 m route and 3,600 s envelope.
Fresh nominal recorder `revision-4cec4c6ee2234027a422f7a45e9d68e3` started
October 4 07:41 JST; exact simulator container
`00f4337ec4faedb932fbf61cec0ae1b6f0c1232547f5048e662d826546925bea`, with automatic
log/stop cleanup and active monitoring. Result pending.
Native `revision-7fa84675b3b54291a5aa14d0c75032eb`; first-leg movement confirmed.
Mapper `revision-d91fdd815659455286f973668bc8213b`. At 07:47 JST the first
straight is complete and the first corner is underway; this is progress only,
not route or accuracy acceptance. Runtime sources remain frozen during capture.

Nominal outcome: **failed after 1,021.31 wall s**, at **152.38334 sim s** during
the second corner. Simulator's motion-step watchdog expired; recorder readiness
checks all remained true, and the simulator grant was only **14.19 ms old**.
An ordinary full GC at that tick cost **384.98 ms**, with native capture taking
**463.05 ms** before the motion step. No forced GC was present. This isolates a
remaining simulator-critical-path delay: successful grant reception alone cannot
keep the motion update timely. The simulator stopped automatically and evidence
is preserved. The normal long loop is **not passed**. Next: measure and reduce
synchronous native evidence/guard work, test exact evidence/geometry equivalence,
then repeat the short stress protocol before another long retry. Do not change
the 500 ms watchdog or count the healthy mapper prefix as completed acceptance.

Follow-up: cached immutable survey bounds plus the exact split-XY squared-distance
argmin preserve **4,491/4,491** sampled real-survey query results. Offline cost
per nine queries decreases **3.98 → 0.277 ms**; too small to claim a stall fix.
The completed floor survey's heights/normals now use immutable NumPy storage,
not thousands of nested Python lists. All nine live raycasts, collision queries,
tie ordering and safety thresholds remain. Separate source inspection found
the ROS world factory eagerly imports LargeScale/Numba/clipmap code even when
running Lunaryard. Changed it to load only the selected manager; explicit custom
manager registration remains supported. **104 focused host tests pass**.
Stationary same-scene heap comparison started October 4 08:04 JST, container
`def7f6c599d40cbfd4ce581ce3012581c952e8a4345e7b2a1ba479644adee276`, after fresh
CUDA/archive preflight. This diagnostic cannot request a motion lease. Compare
scene/sensor/clearance/graph artifacts before considering any moving retry.
Stationary result: native `revision-66fc337e89eb48ba9eb32273c8129dc6`, logs
`revision-f94b9ca16f4147d3ae424fd9bbd63f54`. Tracked objects decrease
**748,361 → 674,796**; full collections **335.33 / 361.34 / 347.21 ms**, versus
the previous lean comparison's **431.06 / 417.99 / 416.31 ms**. All four checked
scene, clearance, sensor-audit and graph-contract JSON artifacts compare exactly
equal. Diagnostic stopped before any lease/motion, simulator shut down. This is
a useful heap reduction, not long-run acceptance. Repeat the unchanged short
three-GC stress route next; retain every attempt and then test normal acquisition.
Final pre-stress regression: **493 host / 44 isolated ROS tests passed**. Fresh
stress recorder `revision-bd02889e10e84c8a8aa0b0d296bba553`, simulator container
`5a1da377bb30140cdb15cf54befaa8eb004812becaf6f27d1a342aab612db639`, started at
08:07 JST with an active seal-triggered log/stop watcher. Same three forced
collections and 110 ms storage workload; result pending.
Stress outcome: **completed**, **324.01 wall s**. Native
`revision-a67c2eaaf1e34482b7ef2819151ea01b`, mapper
`revision-c8c48b8a07d647aab5dd3f2abcca3a64`: **1,944/1,944 ticks integrated,
325 projections**. Full collections **337.50 / 347.61 / 354.61 ms** at the exact
declared ticks; maximum native callback **419.54 ms**. All final health checks
true, mapper completed/closed with no failure. Independent disk verification
matches all three payload hashes; transport matches **1,944 ticks / 325 windows /
12,347,689 points**, including boundary windows. Not nominal compute or terrain
qualification. Next: fresh uninstrumented loop16 on this frozen revision.
Nominal recorder `revision-d4d0f081b64f47caab3831bf2154ab08` started October 4
08:14 JST after fresh GPU/archive preflight. Exact simulator container
`8c5aead0c682cfe077dac206481131dfdbd9dcf02d44383792c9d1b9f6771615`; active watcher
preserves logs/stops the simulator at sealing, with a bounded fallback stop.
No forced collections/storage delay. Same route and frozen terrain core.
Result pending; runtime sources are frozen until capture finishes.
Nominal outcome: **completed and sealed**, **2,027.64 wall s**. Native
`revision-070d4d8a70084dbdb5414ad9dc945b35`, mapper
`revision-0113a56294cf4d8f817fe1404a8c820f`: all **17,949 ticks received/integrated,
2,992 projections**, no mapper failure, completed/closed. Final recorder checks
all true. Simulator stopped automatically after logs were saved. Peak mapper RSS
**271.54 MiB**. Next, in progress: independent disk hashes, complete trajectory
export, whole-input audit, fixed-frame GT pose comparison and recorded grid gaps.
Independent mapper hashes all match. Nominal run has no forced GC; it survives
two natural full collections at **142.95 / 271.17 sim s**, lasting **376.46 /
338.13 ms**. Maximum native callback **465.07 ms**: margin remains tight, so one
completion is not an endurance guarantee. Recorder storage-check max **400.74 ms**
is another scaling concern; main readiness max **23.12 ms**. Trajectory export
`revision-c7399d2da70c447796598329ad22a522` passes with **3,018 revisions / 3,017
frames**. Grid export `revision-2e0489322a09412e818b7c460805bffb` has **2,991/2,991
expected live grids**, no missing views; the terminal projection is deliberately
published as unknown on completion, separately verified, not dropped missing data.
Full-run compute median/p95/p99/max **38.91 / 52.54 / 57.03 / 75.72 ms**; p95
still fails the 50 ms compute target. Initial whole-input audit
`revision-cf2555b453ab4159983b6cf636c44a99` fails before scoring because the frozen
candidate records the host archive path, absent inside the dev container. Keep
the failed audit. Retrying offline with the same dev image and a second read-only
bind of the same external device at its original host path; no archive contract,
guard or capture modification. Pose diagnostic continues independently.
The read-only-alias attempt `revision-c746c8211a3943758bbeaadb96c86163` is also
retained as failed: `check_run` requires write access even for input roots.
Use a normal bind of the already-authorized archive, verify identical device
IDs for both aliases, and keep evaluator inputs immutable. No guard changes.
Whole-input retry `revision-6057fc12bef540979600814c00043aa6` reaches the native
reference gate and fails on two missing same-tick first hits (ticks **5,594 /
6,999**, sim **111.43334 / 134.85001 s**). Across **2,297,472 attempted rays**, all
**2,297,470 successful hits** are within **0.12827 mm**; p95 **0.001907 mm**.
Same-tick along-ray origin diagnostics recover both terrain hits to approximately
1.2 micrometres. Consistent with query numerics, not a demonstrated GLIM/frame
fault; the failed gate remains failed and counterfactuals do not replace evidence.

Completed-route pose diagnostic `revision-0cf764a43a244f6db46e19181aa02712`:
**17,949/17,949** exact GT pairs, **299.133 s / 17.636 m actual base travel**.
First-published XY median/p95/max/final **6.05 / 15.22 / 35.70 / 3.85 cm**;
revised **5.21 / 12.35 / 16.89 / 4.29 cm**. Causal vertical p95/max
**3.07 / 3.51 cm**, yaw **0.369 / 0.613 degrees**. Both overall diagnostic targets
fail; both 1 s relative-pose targets pass. No observed catastrophic drift on this
completed GT-seeded, synthetic-RTK-assisted route; not unaided SLAM qualification.
Transport verifies **17,949 ticks / 2,993 windows / 110,509,448 points**, including
both boundary windows. GT/native base reference maximum discrepancy is **0.157
micrometres**, a wiring cross-check, not the GLIM error. No fitted alignment,
missing-tail trimming, terrain-truth inference or successful strict-audit claim.

Gallery `revision-2ac54b276ccc4bb8a5fa9538a0939edb/gap-report.html` includes all
fixed-time grid examples and paired pose score. Overview PNG
`revision-8fc099c194834c1d8d311b0fcf2ddf58/gap-overview.png` visually inspected.
At +180 s, center unknown component is **0.5 × 0.6 m / 0.26 m²**; final center is
measured support, with **28/900 nearby cells unknown**. This does not certify
wheel support or obstacle recall. CairoSVG was unavailable; installed system
GI/Rsvg+Cairo rendered the SVG successfully, without installing packages. No
full-browser screenshot claimed.

Post-capture diagnostic-only fixes: native gate errors now distinguish missing
PhysX hits from residual size and identify the exact tick; no criterion changed.
HTML refuses pose scores whose recording hash differs from the grid source and
visibly retains failed surface-audit status. Final host suite **495 passed**;
ROS suite **44 passed**. No runtime classifier or terrain library changes after
capture, no C++ rebuild or new core-resource qualification. Simulator stopped.
Next bounded cycles: stream evaluator diagnostics through MCAP with exact
round-trip/loss/shutdown gates; recover compute margin with exact-output tests;
validate independent wheel/obstacle truth; then declare 40–60 m/endurance routes.

October 3, 23:03 JST — storage recovery and fresh extended-budget attempt:
host and dev CUDA context/allocation checks, archive device/bind/capacity checks,
and **99 focused host regressions** pass (`test_natural_motion.py`,
`test_capture_polling.py`, `test_inspection_config.py`). Recorder
`revision-55b34c12ada74d869471c67946cc7e2a` uses
`--execution-profile loop16_slow_host_v1 --wall-seconds 3600`; headless startup
uses matching `SELECTED_EXECUTION_PROFILE=loop16_slow_host_v1`. Hypothesis:
the unchanged loop can finish with sufficient overall wall time. No motion,
native-dose, freshness or geometry thresholds change. Result pending; success
requires complete route and sealed outputs, followed by whole-input audits.
The older capacity blocker below is resolved. The agent deleted no files.
Startup verification: native `revision-f6d51c114a4b4fc08eb5a57098d05019`,
mapper `revision-6f4a0744d9da4a85bc83979e1ef15d95`. Native data and terrain
projections are being archived. At sim **28.48 s**, the controller is on leg 0
with forward command **0.1 m/s** and approximately **0.34 m** base displacement
from that leg's start. This establishes resumed acquisition, not route completion
or map accuracy. A host watcher waits for `collection.json`, saves the simulator
log into the recorder archive, and stops only this attempt's exact simulator
container ID (`633a03e9747b2d8aac238f368ed954e9e2fd55bdc410b20e7ced26d68dfda54a`).
The watcher stops before printing, avoiding the previous broken-output cleanup
hazard. Next: inspect the sealed outcome before any retry; on success export
trajectory, audit native/recorded inputs and measure pose/grid gaps. On failure
retain this run and use explicitly failed-prefix diagnostics.

October 3 full-loop retry outcome: recorder
`revision-d85c01f218354af1907520bf30193c10` stopped on **shadow collection
deadline exceeded** after **1,801.785 wall s**, at **237.07 sim s** with three
corners complete and the return leg underway. Native
`revision-8b54a249c5164a18b73734ccccc17512`; mapper
`revision-d18495f341a34c2db1b37c3d507e6f26`. Recorder's last mapper status was
healthy, with 13,118 received ticks / 2,184 projections. Readiness max **118.64 ms**;
no recorder heartbeat failure. The earlier attempt failed at 249.53 sim s, so this
shorter prefix does not prove full long-duration robustness. Native callback max
**92.01 ms**; only logged full GC was **437 ms** during stationary preflight.
Simulator's subsequent missing-recorder message is secondary to recorder expiry.
Failure evidence and the simulator log are preserved; simulator is stopped.
The mapper's final failed seal contains 13,120 ticks / 2,185 projections and
the expected terminal missing-pose-bracket failure after shutdown. Independent
disk hashes match all three files, including 5,594,892,800 native bytes.

Observed issue: the 1,800 s overall wall budget is insufficient for this
hardware's sub-real-time simulation. Implemented explicit opt-in
`loop16_slow_host_v1` execution profile, bounded at **3,600 wall seconds**.
This is a resource-budget change, not a latency/compute acceptance improvement.
Default routes/hashes and historical captures retain their original budgets.
Route length, commands, native dose/duration bounds, leg deadlines, half-second
heartbeat/lease guards, geometry checks, pose brackets and storage reserves are
unchanged. New requests archive the execution declaration; the recorder rejects
simulator/profile mismatch before granting motion. Regression covers legacy
requests, exact matching, wrong-route use, unknown/tampered budgets and real ROS
request handling. The extended profile is **not live-validated** yet.

Concrete external blocker: capacity preflight reports **111.712 GiB free**,
**114 GiB required**, a **2.288 GiB deficit** before another capture. Requested
user assistance freeing backed-up external storage; no archive cleanup or
workspace fallback. Derived failed-pose/grid/HTML reports now use a stricter
**1 GiB output quota**, preserving the same external-device and 50 GiB reserve
checks, with final quota checks. Capture budgets/preflight are untouched.
Report allocation and overflow regressions pass; this allows small offline
diagnostics without pretending there is room for another raw capture.

Failed-prefix pose diagnostic `revision-7b24ea6ee0a04c159e087b5c0fa7f2d9`:
13,110 / 13,120 ticks paired; all 10 missing terminal associations retained.
Executed native duration **218.65 s**, paired GT path **12.64 m**. Causal XY
p95/max **17.60 / 43.33 cm**; revised **13.89 / 18.87 cm**. Causal vertical
p95/max **3.34 / 3.73 cm**; causal yaw p95/max **0.371 / 0.618 degrees**.
Both diagnostic pose targets fail. No fitted alignment, dropped denominator,
terminal extrapolation or completed-route/terrain claim.
Recorded-grid export `revision-2e3c244985854facbc83d6398de8c7f3`: 2,184 / 2,185
projections recorded; the missing final projection and unreached +240 s view
remain explicit. Compute median/p95/p99 **43.91 / 57.25 / 63.92 ms**; p95 exceeds
the 50 ms target. Peak mapper RSS **258.30 MiB**. At +180 s the unknown component
containing the base center is 0.6 x 0.6 m; last recorded center is measured
support, with 161/900 near-region cells unknown. No terrain truth/recall was
evaluated. New self-contained gallery:
`revision-db2f752c4ea24910a7bd8e522489dd26/gap-report.html`; linked in
[the gap report](long-run-gap-report.md). Source runs remain failed.

Final candidate verification: **445 host tests passed** (frozen real terrain
library, motion/lease/budget/transport/bookkeeping/archive/grid contracts),
**36 ROS tests passed** on isolated domain 179 (recorder, selected transport,
polling load and matched/mismatched execution profiles). Commands use
`python3 -m pytest -q -p no:cacheprovider`; new runners are in the runbook.
Python/shell syntax and extended compose configuration validate. No capture
or simulator is running. Full-loop validation of the extended profile still
requires external storage capacity; no failed attempt is promoted.

October 3 resumed after the host crash: no capture was left running. Unrestricted
checks confirm NVIDIA visibility and CUDA context/allocation pass; the external
archive is now mounted read/write on `/dev/sda1` with 129 GiB available. No host
repair or remount was performed by the agent. Restarting only the task-owned dev
container, checking its archive bind, and rerunning scheduling/native regressions
before a fresh declared loop16. Hypothesis: reserving callback-processing time
after readiness checks removes the recorder starvation seen September 30.
Acceptance for this step: complete the unchanged route and seal all native,
mapper and recorder outputs, then audit transport, pose accuracy and map gaps.
No failed prefix substitutes for a completed loop; no safety thresholds change.
Restart regressions: **234 host tests / 32 isolated ROS tests passed**. Host and
dev CUDA allocation, capacity and device/bind checks pass. Fresh recorder
`revision-d85c01f218354af1907520bf30193c10` started at 21:36 JST, followed by
the selected headless loop16 simulator. Source/config/library inputs frozen for
this attempt. Result pending; old failed runs remain unchanged.
Native `revision-8b54a249c5164a18b73734ccccc17512`; mapper
`revision-d18495f341a34c2db1b37c3d507e6f26`. The agent-server restart during
startup did not restart the capture: existing recorder/simulator processes and
first-leg motion were confirmed before continuing monitoring.

September 30 live noise verification and recorder scheduling correction:
recorder `revision-14d466ab65f744b3b12e81dfcae6cc77`, native
`revision-b7edc52b297043bb8a803135f46f92da`, mapper
`revision-a06f71a221544d29bedcc8a5eff58854` reached **249.53 sim s**, third
corner completed / return leg underway, before recorder
`request_heartbeat` failure after **1,661.54 wall s**. It did **not** hit the
1,800 s overall budget. Native diagnostics counted **129,385,458 hits**, two
above 1 mm, maximum **1.149 mm**, with native points unchanged and no sensor
rejection. The corrected timestamps also crossed the exact former 158.05 s
failure. Mapper received 13,511 ticks and produced 2,250 projections; its
three sealed file hashes independently match disk. Capture remains failed and
ineligible for completed-route acceptance.

The latest stop is not established as another simulator GC pause: the only
logged full GC is 398 ms during stationary preflight. Native callbacks max
93 ms. Recorder readiness checks reached **118 ms**, with 5,641 callbacks
above the 50 ms diagnostic level (all stages combined); final readiness checks
are typically 80–110 ms. The collector schedules them every 100 ms **from
their start**, so a slow check immediately triggers another after only one ROS
callback. The final recorded request is about 524 ms old when failure is
declared, while clock messages continue. The simulator's later missing-recorder
message is secondary. Its all-run 1.758 s maximum tick gap lacks a timestamp,
so it cannot by itself explain this terminal failure.

Fix: `capture_polling.ReadinessCadence` reserves 100 ms of callback-processing
opportunity **after** each expensive check finishes. Every actual readiness
check still performs the same exact archive/config/ownership validation; no
caching, stale grant, heartbeat relabeling or timeout relaxation. Source hash
is included in recorder provenance. A deterministic 110 ms-check load model
and an actual isolated ROS fixture with 32 topics reproduce starvation under
the old scheduling and pass under the candidate. Missing heartbeat still fails
the unchanged 0.5 s limit. This addresses scheduling; it does not remove the
unnecessary per-tick file layout. Full-loop validation of this last fix remains
pending; do not claim the viewport change or scheduler solved all long-run stalls.
Final-revision verification: **361 host tests passed** with the frozen real
terrain library, including capture polling/lease guards; **32 sourced ROS tests
passed** on domain 179 (`test_shadow_capture_ros.py`,
`test_selected_transport_ros.py`, `test_capture_polling_ros.py`). Commands use
`python3 -m pytest -q -p no:cacheprovider`; focused reproducible scheduling
commands are in the runbook. Python compilation and root/submodule
`git diff --check` passed. The failed simulator was stopped after preserving its
log and archives; no acquisition remains running.

September 30 timestamp correction: noise-policy trial recorder
`revision-8fd5089b22d94887a9a2949f3e690885`, native
`revision-15f85f441dbf4a2a87ca0322d958be6c`, mapper
`revision-d79bc8bcc8074ebfa71eb0728a0c2f2f` stopped during the second turn.
The mapper rejected native sequence 8475 at **158050008243 ns**, bounded by
the next received clock **158050008242 ns**. The original acquisition double
is **158.05000824294984 s**. Native encoding used `round(t*1e9)` whereas Isaac
5.1's ROS clock uses `int64(t*1e9)`. This is conversion disagreement, not a
late sensor or measurable physical motion error. Recorder shutdown then caused
the secondary simulator message about a missing recorder; that was not the
root cause. The failed mapper sealed 8,475 received ticks / 1,410 projections.

Fix: explicitly version selected live acquisition as `isaac_clock_ns_v2` and
use the clock's conversion for native headers, six-tick GLIM window headers,
per-point offsets and completion. Preserve native acquisition doubles and XYZ.
Archive the policy and conversion source hash; transport auditing uses each
capture's declared policy. Old captures retain their legacy nearest-ns encoding.
No arrival-time substitution, interpolation, deadline change or temporal
guard relaxation. Regression reproduces the exact 1 ns failure under the old
encoding and passes the unchanged scheduler/core with the new encoding; checks
40,001 simulator ticks, invalid stamps, exact XYZ, both window policies and
real ROS/MCAP transport including corrupt/missing windows. **327 host tests +
8 ROS tests pass**. Native association stats in this failed trial: 81,742,015
hits, zero above 1 mm, maximum 0.329 mm. Thus it crossed the previous failure
times but did not itself exercise >1 mm noise; the saved real failure ticks do.
No full-loop or terrain/SLAM qualification is claimed.

Fresh timestamp-policy retry: recorder
`revision-14d466ab65f744b3b12e81dfcae6cc77`, started at 00:13 JST after
host/dev CUDA allocations and archive checks passed. Same loop16, terrain
library, GLIM settings, viewport setting and motion safeguards. Runtime source
is frozen until this attempt ends; result pending.
Native: `revision-b7edc52b297043bb8a803135f46f92da`; mapper:
`revision-a06f71a221544d29bedcc8a5eff58854`. Loaded sensor manifest confirms
clock policy v2 and the diagnostic-only 1 mm policy; stationary sensor/TF audit
passed. At 00:26 JST / 125.7 sim s, both earlier noise-failure times have been
crossed and the first 6 m leg is nearing completion. This is progress, not a
completed-route acceptance result.

September 29 noise-handling correction: user explicitly directs that tiny
simulator noise must not stop the experiment. `NativeWindow` now retains raw
points and counts XYZ/range discrepancies above 1 mm without failing. A **5 cm
gross-corruption** check remains; this is not a claim of 5 cm sensor accuracy.
Sequence, timing, beam order, array shape/finite values, chassis filtering and
all motion/lease/GT alignment guards are unchanged. This deliberately replaces
the old association acceptance contract; failed historical runs remain failed.
Policy v2 is archived in `selected_sensor.json` and the native timing audit,
alongside bounded counts/maxima. No per-point logging or additional raycasts.

Regression cases at 1.045, 1.484, 2 and 10 mm pass while proving emitted XYZ
bytes remain the original measurements. Gross 10 cm corruption, missing/reordered
beams and timing faults still invalidate the stream. Both real saved rejected
ticks pass the current adapter unchanged: `revision-647cff30199a4ad8b75fb6c283c2b77b`
and `revision-b3ef63d81280437898a80f301a6ae432`. These are isolated fragment tests,
not promotion of failed captures. Fresh guarded loop16 follows regression and
GPU/archive preflight, with viewport disabling and the byte-identical NPZ writer
enabled. No need to rebuild PhysX to eliminate millimetre-level noise.
Final affected tests: **290 passed**; root/submodule diff checks pass. Fresh
recorder `revision-8fd5089b22d94887a9a2949f3e690885` started after host/dev CUDA
allocation and external archive checks passed, followed by the selected headless
loop16 simulator at 23:45 JST. Runtime inputs frozen for this attempt; result pending.
Native acquisition: `revision-15f85f441dbf4a2a87ca0322d958be6c`.
At 00:00 JST, stamp 132.9 s: first 2 m leg, first turn and 6 m leg complete;
second turn underway without sensor/heartbeat failure. Both prior noise-failure
times (83.95 / 117.83 s) have been crossed. Completion remains pending.

September 29 continuation: the preceding stop was a premature handoff, not an
external blocker. Investigating the native ray discrepancy before resuming the
declared loop. The saved beam/normal dot product is **3.735e-5**, only **0.00214°**
from tangent; the XYZ-direction recast hits a different facet at 0.14213°.
Isaac's tagged GenericSensor source independently uses PhysX distance and hit
position. Hypothesis: ill-conditioned grazing ray/triangle arithmetic, rather
than a GT/GLIM transform mismatch. Added opt-in `native_precision` isolated
fixture: regenerate seed-43 terrain, retain the exact rejected rays plus a fixed
perturbation grid, compare original, centered-coordinate and unsimplified
colliders. No live native data/thresholds changed. Thirteen focused fixture and
rejection tests pass. First diagnostic launch
`revision-fde05066a55744d69d6a08095510f251`; idle dev/inspection containers stopped
for isolation, host/device preflight passed.

Result: inner `revision-f369a1c1a2da4387b564f2b99e319676` reproduces the original
beam and XYZ-direction recasts **exactly** without ROS, GLIM or rover motion.
Original and unsimplified collider max residual **1.745298 mm**; coordinate
centering **1.744446 mm**. Both attempted remedies are rejected, not promoted.
Installed runtime is **Isaac 5.1**, not the previously inspected 5.0 source;
[5.1 GenericSensor source](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.sensors.physx/plugins/isaacsim.sensors.physx/generic/GenericSensor.h)
confirms independent PhysX distance and position fields and two-sided rays.
CPU float64 local-triangle diagnostic `revision-7db0e84fa4b04c5c87b7a23c2d0f6d2a`
finds **1.658704 mm range error / 1.385442 mm position error** for the recreated
beam. Local reference is not all-scene terrain truth. Captured triangle/ray
regression added; later fixture revision explicitly refuses a regenerated
baseline that does not reproduce the archived query within 1 micrometre.
This establishes an upstream near-tangent numerical issue, not a frame fix to
make in GLIM. A strict 1 mm contract versus a separately bounded noise-tolerant
sensor contract is now an explicit user choice; no gate weakened or raw point
repaired while awaiting that decision. Final-revision isolated repeat
`revision-cfc85e290768489b8834a9ebfa90faa5` (inner
`revision-96d1838a8a8d4ab9964ba50dbb64343e`) completed, verifies exact baseline
reproduction and returns the same case report SHA-256 as the first run. All
three cases retained; this does not resume the failed motion capture.

Independent I/O fix implemented: `native_archive_io.write_native_npz` encodes a
bounded record in memory, hashes those exact bytes and writes once exclusively;
selected capture no longer reads each newly written NPZ back. Provenance includes
the helper. Original format, raw arrays, timestamps and independent disk-hash
checks remain. **100/100 real ticks have byte-identical legacy/candidate output**
with independent disk verification (`revision-44ee3902714e4e52bc699bc5914182b6`,
actual Isaac NumPy environment). Alternating-order offline p50/p95/max:
legacy **4.864/5.540/13.677 ms**, candidate **4.718/5.573/11.929 ms**. No meaningful
p95 speedup demonstrated; benefit is eliminating redundant reads, not a claimed
stall cure. Focused geometry, archive, adapter, rejection and fixture tests:
**34 passed**, subsequently expanded with short-write/close-error fixtures.
Final combined affected suite: **286 passed**; Python compilation and both
root/submodule diff checks pass. Idle dev and inspection containers restored;
simulator remains stopped. Diagnostic artifacts made UID/GID 1000-owned.
ROS/MCAP diagnostic migration remains separate unfinished work. There is no
successful moving validation of the latest NPZ writer yet; only exact real-data
round-trip verification. No pose/terrain/route qualification claimed.

September 29 viewport trial: user selects disabling unused headless viewport
updates first; if stalls persist, audit the synchronous diagnostic I/O and use
recorded diagnostic topics/chunked evidence rather than two files per native tick.
Added explicit `OMNILRS_DISABLE_VIEWPORT_UPDATES=true` only to selected-headless
compose, forwarded to typed `RendererConf` / `SimulationApp`. GUI/default remains
false; non-headless requests are rejected. Startup checks the effective active
viewport state and emits `VIEWPORT_UPDATE_AUDIT`. Physics/render-step cadence,
ray design, CPU limits, route and every watchdog/geometry threshold are unchanged.
27 focused configuration/profile tests pass. The previously implemented streamed
mapper seal remains enabled; comparison to the earlier failed run is therefore a
same-route feasibility test with that declared writer change, not a pristine
one-variable full-stack timing A/B. No runtime file-format rewrite in this trial.
Recorder `revision-8e36fcafda8e4759a0002f2ffc9fc3ce`, native
`revision-f5c9bff6f9a8413bb5f04c07b5f8221a`, started 23:00 JST after archive
and host/dev CUDA allocation checks passed. Actual startup confirms
`active_viewport=True updates_enabled=False`; `viewport_experiment.json` records
the effective setting and source hashes. Runtime inputs stayed frozen until seal.

Trial result: **failed**, at stamp **83.950004 s**, 4,040 archived native ticks,
on `native local XYZ/ray association mismatch`, not a heartbeat timeout. One of
9,906 hits at 11.909 m has **1.484 mm** inconsistency; independent same-tick beam
recast has **1.745 mm** PhysX hit-position/distance inconsistency. This supports
an upstream raycast-numerics hypothesis, not permission to relax the 1 mm guard.
Rejected arrays/recasts remain preserved; diagnosis
`revision-559d2edffb0443da960dd8ed4930d34c`. No repeat launch or data repair.
Maximum native callback **82 ms**, recorder readiness **40 ms**. The only logged
full GC was **473 ms during stationary preflight**. Run ends too early to judge
whether viewport disabling fixes the earlier 249.6 s heartbeat failure.
Mapper `revision-f76e3dd232504365965c8b865997d357` closed with its terminal
`missing causal bracket: extrapolation forbidden` failure and **672 projections**.
Independent reads verify all three output hashes, including **1,722,817,600**
native bytes; streamed shutdown hashing now has failed-moving-run validation,
not successful-route acceptance. Simulator log preserved and owned container
stopped. [I/O audit and topic migration gates](capture-io-audit.md) identify
duplicate native serialization, per-tick NPZ read-back and growing archive walks;
file count is **not** established as the cause of GC. Diagnostic-topic migration
is proposed, not implemented, and must retain all raw fields and evaluator data.
Final regression: **274 passed** across local terrain/costmap, selected sensor,
native adapter/audit/rejection, chassis, Isaac fixture, inspection configuration,
optical origin, output seal, floor sampling/attribution, gap rendering and
bookkeeping suites using the frozen `revision-9eadef1010bf433583ce71a351502ab4`
library. `bash -n` on both affected launch scripts and root/submodule
`git diff --check` pass. Exact completed-run replay was not attempted on this
failed source; its eligibility gate remains intact.

September 29 evening follow-through: user asks to implement the integration fix
and continue testing without another handoff. `AGENTS.md` now explicitly treats
locally fixable integration failures as diagnose/fix/regression/fresh-run cycles,
not external blockers. Safety stops and artifact preservation remain mandatory.
Frozen GLIM `config_global_mapping.json` and `config_sub_mapping.json` both have
`enable_optimization=false`; no optimization setting is changed.

Candidate: opt-in live v2 `allow_bookkeeping=True` keeps canonical ODOM frame
poses/immutable stamps in a bounded 8,192-frame ledger, separate from the short
interpolation window. SUBMAP/GLOBAL deltas must match every referenced canonical
pose and extrinsic within **1e-10 matrix-entry rounding tolerance**, with known
unique IDs, unchanged stamps, no deskew invalidation and contiguous session/epoch
revision chain. Validate the whole batch before advancing the chain with no pose,
availability, evidence-age or terrain-geometry change. Bookkeeping never updates
canonical references (no cumulative tolerance drift). Real changes/snapshots/
unknown frames still fail closed; no loop-closure reconstruction is claimed.
Ordinary offline/default core remains ODOM-only. Source/config changes are
declared; classifier/library, sensor, route and all acquisition guards unchanged.

Correction to the preceding report: the actual frozen live `max_frames` was
**2,048**, not 1,024. Thus its initial 1,277/1,327-frame batches would not breach
that bound; the type rejection was the observed stop. Separate bookkeeping
capacity avoids resurrecting historical frames into the recent-pose window as
later batches grow. The new bound itself remains explicit and fail-closed.

Focused core/ledger/sealing tests with the actual frozen library: **163 pass**.
Without the library the initial smoke run was 60 pass / 97 skip, not acceptance.
Changed-runtime replay comparison is explicit `--bookkeeping-candidate`; ordinary
exact replay retains its original strict source/eligibility checks. Candidate
must match every recorded layer/association/state field (except wall timing),
and may diagnose only the specifically archived non-ODOM-kind failure. It never
qualifies missing route time or changes a failed source to completed. Comparing
the 1,303-map failed loop and a completed shorter control before fresh capture.

Pre-capture validation completed: explicit candidate replay
`revision-2dbbb8c521c04bfb9ab8f83299f17c1b` matches **all 1,303 recorded
projections/associations/state fields exactly** and accepts both original
bookkeeping batches, with 1,327 canonical frames. It processes the one native
chunk queued behind the original failure (7,828 versus original 7,827 processed);
no missing source data is fabricated. Failed source stays ineligible. Completed
combined control `revision-d0ff08a7bdb6485ba9e2d62998b31873`: **633/633 exact**,
3,791 chunks, no bookkeeping. No classifier/library changes.
Both uninstrumented C++ resource profiles pass: `layer_capacity`
`revision-c3bce6af4e6e4a3ba06fe753742922b3` p95/p99 46.967/58.075 ms,
1,014.30 MiB peak; `dense_flat` `revision-ea04c73d82f94bd9a98bf9af960f4c82`
49.014/55.787 ms, 331.47 MiB. These are separate fixed synthetic core profiles,
not concurrent live acceptance; layer-capacity memory margin remains narrow.
Archive device and host/dev CUDA context/allocation preflights pass this session.

Upper-capacity bookkeeping microbenchmark
`revision-34dc936b55a6487fba3ea77e8fdd3a07`: all 32 × 8,192-frame callbacks
retained, p95/max 59.299/81.914 ms; 347.64 MiB process peak, 2 MiB canonical
pose arrays, interpolation timeline stays at 52 frames. It excludes transport/
deserialization and concurrent Isaac, so it is not live heartbeat qualification.
Fresh guarded headless loop (subsequently failed): recorder
`revision-f67480e17a8047c69f36f8bcca48c2fa`, native
`revision-393ec39c3a8443d6960e1720b0a8a765`, mapper
`revision-8a15c7cbd1e3499e8716f4bf9f0d7575`. Installed wrapper resolves to the
edited source; manifest explicitly confirms `allow_bookkeeping=true` and 8,192
canonical capacity. Exact same optical v2 / frozen library / GLIM config / route;
recorder and route wall bound 1,800 s, normal 500 ms heartbeat guards unchanged.
Do not modify frozen runtime inputs during capture. Preserve logs and stop the
owned simulator once the recorder seals, then audit and continue from its result.

This fresh run failed at **117.833339 s**, before the bookkeeping boundary, on
`native local XYZ/ray association mismatch`; simulator stopped and log preserved.
Rejected tick 6,955 is now preserved with its original arrays/hash. Diagnostic
`revision-0aca7411d7c443358b2018bf9e1e4a0c`: **one of 8,785 hits** exceeds the
unchanged 1 mm guard: index 6,652 at 12.935455 m, residual **1.044963 mm**.
Median/p95/p99 hit residuals are 0.809/3.018/4.869 micrometres. Float64 trig
still gives 1.044816 mm, so NumPy float32 trig alone does not explain it.
The previous 20 accepted ticks contain no residual over 0.1 mm. Failed source
remains ineligible; this is not evidence of a live bookkeeping fix yet.

Isaac's tagged 5.0 `GenericSensor.h` reads linear depth from PhysX hit.distance
and sensor XYZ from transformed hit.position; the two are not constructed from
one another. A PhysX numerical inconsistency is a hypothesis, not a conclusion.
Added failure-only same-tick beam/XYZ-direction recasts, including each physics
hit's own position-versus-distance residual. Commands stop before diagnostic
casts; original failure is re-raised, raw arrays are never changed and no failed
tick is admitted. At most 20 bad rays / two directions each; no successful-path
cast overhead. Next fresh same-route run obtains this missing causal evidence
or proceeds through the original bookkeeping boundary. No threshold relaxation.
Final affected regression suite with frozen library: **270 passed** (9.18 s).
Fresh diagnostic-enabled recorder `revision-23da6cb44dc94323879532dfba9ee429`,
native `revision-acebc91ed3a249b3b5f0806c1fb9a5ef` started at 21:20 JST;
same declared loop, GPU context/allocation and external archive preflights pass.
The owned cleanup watcher preserves logs and stops the simulator after sealing.
At 21:34 JST the mapper (`revision-4e3b324715db4c9abcb62a988a3b0301`)
has received both SUBMAP and GLOBAL records and continues projecting beyond
stamp 151 s while the guarded robot turns at corner two. This is the first live
crossing of the original integration barrier, not yet a sealed whole-loop result.

Final fresh-run result: **failed**, simulation stamp 249.600013 s, during the
return leg. Recorder sealed its MCAP; simulator stopped and log retained.
Native timing records a **404.179 ms generation-2 GC pause** in `surface_audit`
at the failure, max native callback 482.461 ms. The 1.770 s `max_tick_wall_gap`
is a whole-run maximum (including startup), NOT an isolated fault-tick duration.
Do not call that a measured 1.77 s moving pause. The unchanged 500 ms watchdog
stopped motion; no host repair or GC suppression is justified by this evidence.

Failed-prefix pose diagnostic `revision-1e96a75aaf034d1e8e39967e37dcc662`:
13,694/13,705 native ticks paired, **11 missing tail brackets retained**;
228.400 s native duration, 13.786 m GT path including turn displacement.
Causal XY p95/max **15.587/25.938 cm**, revised **12.791/15.496 cm**;
rotation max 0.719/0.454 degrees. Both existing diagnostic pose targets fail.
The native sensor association mismatch did not recur; no recast evidence was
produced. This repeat is not a cure for that intermittent failure.

The longer run exposes a second locally fixable integration bug: mapper
`finish()` re-reads `native.bin` (5,844,360,200 bytes) to compute the seal, but
launch escalates SIGINT to SIGTERM after five seconds. `collection.json` was
opened before hashing and is empty. Ordinary sealed revision diagnosis correctly
refuses it; exact full-run replay and completed-grid scoring remain ineligible.
An explicitly **unsealed failed-source diagnostic**, never a repaired seal,
`revision-5e42cf0a6dbd4d99810f6476c3f95d39`, snapshots the unchanged event hash
and observes 2,311 contiguous revisions, 2,308 frame IDs, four non-ODOM batches:
1,056/1,106/1,121/2,227 frames. All IDs/stamps/extrinsics/deskew flags unchanged,
maximum translation difference 2.85e-14 m. Mapping continued after both batch
pairs; the 2,227-frame batch exceeds the old 2,048 recent-window capacity and
demonstrates why separate bounded canonical history is useful.

Implemented `OutputDigest` for all three live evidence streams: hash the exact
successfully written bytes, check size at close, obtain final digests without
bulk shutdown I/O. Added helper to opt-in package install and source provenance.
Independent `verify_seal` still re-reads all disk payloads; same-length edits
remain detectable. No shutdown grace period, guard or evidence threshold change.
Large-file fixture `revision-bb4a27295ca34cbbb5467626aba808b9`: **all 5.84 GB
match** between source/read-back/streamed digest, writer seal **0.046946 ms**,
independent two-file readback 6.303 s. This is a writer scaling check, not full
ROS shutdown or live acceptance; original empty seal stays untouched.
Latest revision: **273 affected tests pass**, opt-in `slam_mapping` rebuild passes.
Real ROS no-input SIGINT fixture on loopback/domain 179 also passes the actual
five-second launch escalation budget and independent disk hash checks:
`revision-5f15f38b63024f34a6dd7fab3978ebf8`. It seals `closed=true`, zero
samples, `experiment_completed=false`; never a moving/native acceptance claim.
After final formatting, combined ROS-shutdown/seal regressions pass **10/10**;
fresh fixture `revision-dd032c54a19f490993b8cb526ce4b7ed`. Both repository
`git diff --check` checks pass; simulator remains stopped.

Resource diagnosis `revision-cb9890895c5b4aeb97a4a55461b72490`: 27,425 native
artifact files; ten exact quota walks all retained, **46.398–48.655 ms** with
simulator stopped. Live recorder readiness was frequently 60–100 ms late in the
run. Repeated synchronous whole-directory scans are a separate scaling concern;
no quota/capacity/device check was removed or cached away. A further identical
loop without a timing hypothesis/fix is not selected.

September 29 headless loop attempt: recorder
`revision-16dc340d086445eabc07a4794db99f25`, native
`revision-88b3fa8464fa411691266f3ee020c279`, mapper
`revision-daee1896dbac4c8c8c00b9af437b04f9`. Archive/CUDA allocation and 17,072
clearance samples pass. **Capture fails** during first turn at stamp 64.450 s:
`native local XYZ/ray association mismatch`; 2,727 archived native ticks,
45.433 s native duration. Failed-prefix pose diagnostic
`revision-c2fb5de4e87d4b1b8ccbc52858742b79`: 2,719 paired ticks / 2.262 m
path, eight missing tail brackets retained; causal XY p95/max 8.763/12.202 cm,
revised 6.497/6.855 cm. No observed catastrophic divergence in this prefix;
not a completed 16 m loop or whole-route stability evidence. Mapper has 453
projection receipts, 452 recorded grids, one missing final publication; compute
p95 44.840 ms on this incomplete run only. Simulator stopped after recorder
sealed; failure and simulator log preserved.

Code inspection found rejected sensor arrays were lost before native archiving.
Add failure-only `native_rejection.guarded_push`: preserve exact offending
arrays, acquisition stamp/pose and hashes in separate non-eligible artifacts,
then re-raise the unchanged fail-sticky adapter exception. No repaired/dropped
tick, threshold or successful-path payload change. Focused tests: 35 pass
(adapter/rejection/headless helper). One diagnostic headless repeat is justified
to obtain those missing failure bytes, not to select a lucky successful repeat.
Longer-than-16 m route remains conditional on a completed coupled gate.

Diagnostic repeat recorder `revision-e941e62597c24a82a0d827483f2ce0f0`, native
`revision-dcdd8abf2dcf4ece9b2c49ef134d7dea`, mapper
`revision-a3f14a6811f546b6a7cdf98128553196` **fails** with that
failure-preservation-only change: terrain `only contiguous ODOM deltas are
supported`. Recorder shutdown precedes native `holdout requires exactly one
identified recorder`; that driver message is a consequence, not root cause.
Sensor mismatch did not recur; no rejected tick was captured this time and the
first failure remains unresolved. Both simulator runs are stopped and preserved.

Repeat pose diagnostic `revision-88df108e841f49529c6655affc5dfce2`: 7,821/7,830
native ticks paired, nine missing tail brackets retained, 8.6293 m / 130.483 s.
Causal XY p95/max 14.962/34.946 cm (**fails**), revised 9.148/11.558 cm (passes
diagnostic targets); causal/revised rotation max 0.583/0.399 degrees. The worst
causal position sample is at stamp 87.7167 s on the long translating leg, not at
the stopping callback. No whole-route/no-catastrophe claim follows from a prefix.
1,303/1,303 projected grids recorded; one repeated final stamp is fault
invalidation. Compute median/p95/p99/max 34.479/47.162/52.871/67.068 ms; sealed
peak mapper RSS 237.219 MiB (last recorder status had only 234.297 MiB).

Revision diagnostic `revision-fd7f7cc89bba4531ac1d571836c6a6ea` compares every
received non-ODOM frame to the preceding full pose state. SUBMAP revision 1327
has 1,277 frames; GLOBAL revision 1328 has 1,327. No new IDs, changed stamps,
extrinsic delta or deskew flags; maximum translation deltas 3.973e-14 / 2.012e-14 m.
Optimization-disabled still emits callbacks. This is **offline diagnosis**, not
a bounded live proof. The earlier 1,024-frame limit assertion was incorrect:
the frozen manifest specifies 2,048 (see correction above). Required next implementation is a
bounded verified no-change path that advances the chain without inventing poses
or ignoring changes to retained/frozen evidence. Real changes still require
hold/reset/rebuild; cover duplicate/missing IDs, real corrections, epoch changes,
non-ratcheting references and large batches, then exact replay/resource tests.
No third blind retry or longer unguarded route was launched.

Gap gallery measurements from actual MCAP grids: straight +15 s center component
2.1 × 2.0 m / 3.49 m², final +81.8 s 1.7 × 1.7 m / 2.41 m²; combined final
+63.1 s 1.8 × 1.8 m / 2.51 m². Center is supported at +30 s in both runs,
not proof that the whole footprint is safe. New exporter preserves every missing
projection, excludes completion invalidation, and shows the actual last recorded
grid even when a later receipt is unpublished. Class SVGs preserve 10 cm cells,
full 6 m window, chassis/guard overlays and unknown versus occupied; no smoothing
or inflated costs are substituted. See [small report](long-run-gap-report.md).

Final gallery exports (straight / combined / first failed loop / repeat):
`revision-3e47f0eca5ca4c1fa0e2e9dfc9f37ac0`,
`revision-0b9955f30bbb4a09b4f165f3a790100c`,
`revision-a0e935d72ec44c9eb50c76763b24512c`,
`revision-14b23be07fdb4a08a21e11d2dd72203d`.
Older exports remain preserved; final exporter explicitly distinguishes the
last receipt from the last grid actually recorded. Repeat +60 s: 34/900 unknown
nearby; +120 s and final +130.2 s: 41/899 unknown. Remaining nearby cells are
measured support, zero occupied; not independent floor accuracy/obstacle recall.
Portable HTML/SVG `revision-d16e5df369084834b6cc3e1f91469640`; visually inspected
overview PNG `revision-477ff643380b4287bca9b1974a1e01a9`. Browser screenshot
attempt `revision-701f28ee700448d78e9f000e2e886a42` failed (Snap temporary-profile
visibility); preserve it, no browser pass or host repair claimed. SVG rendering
and HTML escaping/structure fixtures pass. Final affected regression: 247 tests
pass, Python compilation/shell syntax/diff whitespace checks pass. These tool
checks are not additional simulator/terrain/navigation acceptance.

September 28 late-session follow-up: user requests gap-grid demos and much longer
headless testing. No interrupted capture remains active. New headless-only
selected overlay/helper retains corrected sensing, native terrain, archive/CUDA,
sole-recorder and motion guards; GUI/X11 alone is removed. First attempt is the
existing 16 m coupled loop, unchanged classifier/config/lifetime/thresholds. A
longer repeated circuit is conditional on this first gate completing; existing
non-ODOM and simulator-GC stops remain active. Failure is diagnostic evidence,
not permission to ignore revisions, change watchdogs or claim a completed loop.
Prepare a small HTML report with fixed-time full grids and near-robot close-ups,
10 cm cell scale and chassis overlay; distinguish unknown from occupied and
whole-grid availability from independently verified terrain/motion safety.

| Gate / mechanism | Current evidence |
| --- | --- |
| Rolling persistent runtime | Fixed-reference, 5 s raw / explicit 60 s evidence: 748-record replay at **19.74/22.70 ms p95/p99**, **201.15 MiB**. Offline thresholds pass; source heartbeat failure remains. |
| Frame consistency | Fixed local/map reference plus non-ratcheting oldest-frame witness replaces flawed moving-anchor persistence. Native GT relative displacement for 30–60 s-old observations falls **36.77→7.93 cm p95**; not a calibrated bound. |
| Assumed swept wheel-patch coverage | **71.79–72.73%** across all 748 queries per family. Startup and later unsupported extent remain counted. NOT independent safe-motion scoring. |
| Independent truth / sensing / live gates | Body overlap/sweep fixtures verified; continuous natural wheel support, operating limits, consumer-time pose error, startup sensing, held-out safety and concurrent live runtime remain open. |
| Promotion | **None.** Earlier moving-anchor incremental variants are rejected. Default coherent reference and production mapper remain unchanged. |
| Optical-origin correction / live v2 | `selected_optical_v2`: level 0.45 m mounting, zero-origin collision queries, software 0.5 m minimum. Full-beam isolated geometry and moving sensor audit pass. Combined turn produces 328 v2 maps from 1,963 ticks; sealed exact replay passes. GUI compute p95 55.09 ms fails the 50 ms target; terrain truth/long-route qualification still open. |
| Corrected-mount translating v2 | Straight out/back completes **3.951 m / 81.883 s**, 4,914 ticks / 820 maps, exact replay and whole-input audit pass. First-published XY p95/max **11.21/14.75 cm**; GUI compute p95 **50.053 ms** fails. Independent sampled clear-floor support **61.67%**, zero occupied samples; substantial unknown support remains. Not continuous terrain truth or navigation qualification. |
| Selected single-LiDAR setup | **Level mid-dense +10..-60**, locked development contract. Isolated yaw fixture: 48/48 geometry pass; flat support 100% within 2 m, 97.20–98.08% at 2–3 m, 8/8 boxes, no false-free in 2,304 maps. Physical-rover straight capture completes, but one wheel-overlap reference ray fails the strict geometry audit. |
| New-ray GLIM input | Physical IMU/GT-start/RTK-assisted straight and ±15-degree routes complete with exact transport/GT matches. First-published XY p95 **11.315 / 7.642 cm**, final-revised **8.542 / 7.712 cm**. First-published pitch targets fail on both; straight XY also fails. Turn whole-input audit passes; straight self-geometry audit fails. No production promotion. |
| Longer moving-GLIM campaign | Four preserved loop attempts; longest usable prefix **13.646 m / 228 s**, three 90° turns. First-published XY p95/max **11.552/19.285 cm**, revised **8.378/11.005 cm**, first-published rotation max **0.646°**. No observed catastrophic pose divergence; full loop remains incomplete. Turn control and large revision-batch processing improved; **410 ms Isaac full-GC pause** still breaks the unchanged heartbeat contract. |

## September 27 — selected geometry and local/mid-range terrain gate

### September 28 — corrected-mount translating validation (completed diagnostic stage)

- User approved the next guarded tests. Freeze `selected_optical_v2`, GLIM
  config `revision-4c7b70a4acb44a07acb1bca0d63e3692`, terrain library
  `revision-9eadef1010bf433583ce71a351502ab4` and all existing safety/accuracy/
  runtime limits. First `straight_slow` (2 m out / 2 m back, 0.05 m/s), then
  `combined10` only after examining the straight result. The GUI helper now
  explicitly allows these already-declared protocols; recorder ownership,
  CUDA/archive/X11 checks and motion guards are unchanged.
- Acceptance for this diagnostic stage: completed guarded capture, sealed live
  mapper, exact causal replay, whole-input geometry/timing/transport audit and
  base-frame first-published versus final-revised pose metrics. Report all
  projections and compute failures, startup unknown and missing evidence.
  The straight route is expected to exceed the unchanged 60 s evidence life.
- Independent terrain scoring remains a separate gate. Existing natural-scene
  AABBs and a GT-pose reconstruction are not a complete terrain oracle. The
  existing active-PhysX clearance samples can support a **sampled floor-only
  diagnostic**, not continuous wheel support, obstacle recall or the 95% safe
  motion claim. Do not turn unlabelled cells into free truth or omit them.
- No 16 m loop or navigation promotion while the non-ODOM revision barrier,
  long-run simulator GC stalls and independent terrain truth remain unresolved.
- Launcher regressions: **17 pass**. Host/dev CUDA allocation and same-device
  external bind pass. Fresh GUI straight recorder
  `revision-6a3e3e9e2192436c8b379917597fa04c`, native
  `revision-fbb3be1ddae6425498a6b2821d518b3c`, mapper
  `revision-4a4e32c867b7431a8f2a1ba258025433`. Initial live GT/GLIM base
  check passes (1.225 mm, 0.00781 degrees); motion currently in progress.
  Read-only probe `revision-233ffe4340ec4cdfae792acfc3bad59b` passes ten exact
  class/taper pairs. A bounded watcher will archive the GUI log and stop only
  this exact simulator instance after recorder completion/failure.
- Added `score_live_floor_samples.py`, evaluator-only and completed-source-only:
  exact receipt-order replay before each score; full world/home/map/local frame
  handling; every original PhysX survey sample in every projection, outside and
  unknown retained. Three initial frame/denominator tests pass after correcting
  the standalone test's import path. This does not create continuous terrain
  truth or establish obstacle recall; no mapper inputs/classifier changes.
- Sample diagnostic regressions now **10 pass**: denominator/order and hashes,
  failed/incomplete/mismatched reference refusal, SE(3)/home offset, outside and
  startup unknown within projections, expiry-period accounting, and completed
  publication invalidation. The first denominator-test expectation incorrectly
  assumed the full survey fit the grid; the fixture now asserts that outside
  samples stay unknown, without changing the evaluator. Pre-projection startup
  publications are explicitly outside this diagnostic's scope, not silently
  counted as successful availability. No safe-motion score is claimed.
- Straight capture **completed and sealed**, all 4,914 native acquisitions
  integrated in 820 projections; 81.9 s projection span crosses the unchanged
  evidence lifetime. Peak mapper RSS **224.23 MiB**. All-projection compute
  median/p95/p99/max **37.289/50.053/57.101/83.060 ms**: retain the slight
  p95 failure, do not round it into a pass. No runtime generation-2 GC pause;
  the 500 ms collection was the explicit pre-request stationary checkpoint.
  This does not resolve the longer-loop simulator GC problem. Owned Isaac
  instance stopped after log archival. Trajectory export
  `revision-26d8a115658740b6bb1c0ec88e353413` passes (845 revisions / 846 frames).
- First sampled-floor CLI invocation on host refused the capture's recorded
  `/archive` contract before allocating an output. Correct invocation uses the
  documented isolated network-none dev container with both original aliases of
  the **same external device**; no contract rewriting or checks weakened.
  Score/exact replay `revision-15a82ecce8914cbc93c9680c60c3d56e` is in progress.
- Straight whole-input audit **passes**, `revision-77b070cd1d8c4041970912e51386553a`:
  628,992 sampled first hits, worst **0.0966 mm**; **30,779,822** exact wire
  points including boundary windows; all 4,914 native GT snapshots agree.
  Base-frame pose diagnostic `revision-80b4ba51fa054f5da4de36d4ecf782ec`:
  travelled **3.9509 m**, first-published XY p95/max **11.213/14.754 cm**,
  final-revised **7.410/8.668 cm**. First-published R/P/Y p95
  **0.131/0.276/0.299 degrees**. First-published overall target still fails on
  XY, revised passes; no catastrophic divergence observed and no fitted alignment.
- Floor diagnostic/exact replay **completes**: all 820 layer/association records
  match. **3,154 fixed PhysX floor points × 820 maps = 2,586,280 queries**;
  supported **61.669%**, occupied **0**, unknown **991,346** (including 10,222
  outside-grid queries). Last valid projection supports **62.968%**. There are
  219 projections beyond 60 s. These are sampled-floor availability numbers,
  not false-safe/obstacle recall/95% motion qualification. Unknown remains a
  significant issue in this route's floor envelope; not falsely called occupied.
- Next experiment proceeds because guarded acquisition, exact replay and whole-
  input integrity pass and pose remains bounded: existing `combined10`, unchanged
  sensing/classifier/limits. It tests combined viewing angles and translation,
  not a retry to replace the straight result. Its independently declared survey
  has a different physical extent, so aggregate percentages are **not** a
  like-for-like improvement comparison. Spatial/GT-pose attribution follows if
  unknown support persists. No long-loop/navigation promotion.
- Combined GUI recorder `revision-ea53750e680247e4b6e7e5f52328f46d`, native
  `revision-a47284b3740f4117afbba52ec6950c41`, mapper
  `revision-0fa274e900194aa7af5cbbf4c5ae7e6f` now running, after renewed GPU/
  archive/X11 checks. Exact-instance cleanup watcher is armed. No estimator,
  classifier, sensor or safety changes.
- Implemented offline `attribute_live_floor_support.py`: same raw input and
  integration/projection sequence, same SLAM-chosen rolling origins/reference,
  same lifetime; substitute only exact-tick native GT acquisition poses.
  A mandatory recorded-SLAM mirror must reproduce **every original layer hash**.
  Compare both branches against the same original PhysX floor samples; preserve
  unknown reason counts and final valid spatial layers. This is attribution,
  never GT terrain truth, runtime qualification or a live algorithm change.
  Two initial exact-time/frame/reason-count tests pass; actual execution waits
  until the concurrent live capture finishes to avoid adding replay CPU load.
- Combined acquisition **completed and sealed**: all **3,791** native ticks
  integrated, **633** maps / **63.2 s** projection span. Compute median/p95/p99/
  max **38.065/50.316/58.831/62.685 ms**, **218.16 MiB**: p95 still fails,
  including all samples. No runtime full-GC pause; pre-request checkpoint
  was 505 ms. Owned Isaac stopped, GUI log retained. Trajectory export
  `revision-98e53ffce44e411ab67f1f01424598bb` passes (656 revisions / 657 frames).
- Combined exact replay/floor score `revision-fed066d7d1a8400c826595b1123d28fc`
  passes all 633 layer/association comparisons. **6,204** surveyed floor points
  × 633 maps: support **71.074%**, occupied **0**, unknown **1,135,948** including
  **151,932** outside-grid queries. Last valid support **73.179%**. The route's
  survey is larger/different than straight; do not subtract percentages and
  call it a controlled improvement. There are 32 projections beyond 60 s.
- Relevant final host suite at this checkpoint: **229 pass**, including actual
  archived C++ library fixtures (no skipped local-core tests), launcher cases,
  frame/denominator/expiry/completion checks and new attribution helper tests.
  Completed-source GLIM and whole-input combined audits, then paired GT-pose
  attribution remain in progress. Runtime/sensor sources were not edited during
  either live capture.
- Combined whole-input audit **passes**, `revision-db4d5ca98442419fa461c6d004a0fe88`:
  **485,248** first-hit samples, worst **0.1383 mm**; **23,996,287** wire points,
  all 3,791 exact native GT ticks agree. Base-frame pose diagnostic
  `revision-cf0f8ae2063048c2a280863ba8900634`: **4.0945 m / 63.1667 s**;
  first-published XY p95/max **7.868/12.556 cm**, revised **5.455/6.594 cm**.
  First-published R/P/Y p95 **0.115/0.130/0.348 degrees**, all existing overall
  pose targets pass (yaw has its own threshold). No catastrophic failure.
- Straight paired attribution **passes**, `revision-f29710ccc47b42f7b179c9c1419bee28`:
  all **820** original SLAM layer hashes reproduce exactly. Holding every input,
  grid origin, evidence age and projection time fixed, exact GT poses yield
  **57.829%** sampled-floor support versus **61.669%** with recorded SLAM.
  Both have zero occupied classifications on these floor samples. Of SLAM's
  991,346 unknown queries, **657,818 NO_RETURNS** and **265,786
  UNSUPPORTED_EXTENT** dominate; only **34,796 NEGATIVE_EDGE**. GT has MORE
  no-return queries, not a recovery to ideal-fixture coverage. This implicates
  missing/insufficient retained support rather than GLIM error as the main
  issue for this diagnostic. It is not evidence that less accurate poses are
  better: pose spread can fill unsupported cells and cannot count as a safety
  improvement. No classifier thresholds changed. Combined attribution pending.
- Combined paired attribution **passes**, `revision-e49b7106e9854ef88d4232bc204d803b`:
  **633** exact SLAM mirrors; support **71.074% SLAM / 68.896% GT pose**.
  SLAM unknown reasons: **577,740 NO_RETURNS**, **346,436 UNSUPPORTED_EXTENT**,
  **151,932 outside grid**, **38,964 NEGATIVE_EDGE**; neither branch marks a
  surveyed floor sample occupied. Exact GT does not eliminate the gaps on either
  route. Both source tests and postprocessors are now complete; no new capture
  failure occurred in this stage.
- Saved last-valid spatial plots, generated by `plot_live_floor_attribution.py`:
  straight `revision-7f7098514e644c8b9cbb0f1af59e7153/paired-floor-maps.png`;
  combined `revision-0d06122d17a84927a74986db54ee408d/paired-floor-maps.png`.
  Grey areas cluster around the initial/return chassis region in both SLAM and
  GT-pose branches. **Post-hoc location diagnostic only**, using the already
  declared 1.35114 m rotation-guard radius around the initial base: straight's
  last SLAM map has **964/1,168 unknown samples inside** that circle; combined
  has **1,004/1,664 inside**, plus **528 outside the rolling grid**. Denominators
  and primary scores remain unchanged; these partitions are not a new gate.
  Thus the low full-survey average is not proof of widespread false terrain
  obstacles, nor does a cropped outside-shadow percentage qualify navigation.
- **Next improvement cycle, based on observed failures:**
  1. Attribute the startup-region missing evidence to optical body occlusion,
     minimum-range gaps, rolling eviction and expiry separately, on these frozen
     inputs. Measure support on predeclared short swept wheel/body queries,
     including startup, rather than mistaking a full-route floor average for
     driving availability.
  2. If the missing evidence is unobservable at startup, test a separately
     declared guarded observation maneuver with independent clearance/contact
     evidence. Never make unknown cells free from robot occupancy or GT labels,
     and do not use SLAM jitter as synthetic sensing coverage.
  3. Finish continuous wheel-support/body-clearance truth and obstacle/hole
     safety scoring; these floor-only tests do not establish obstacle recall
     or zero false-safe behavior. Preserve the 95% per-family motion target.
  4. Restore compute margin with exact-output optimizations; both new GUI p95s
     fail narrowly. Re-run both fixed resource profiles including `dense_flat`
     and then concurrent live timing before promotion.
  5. Resolve bounded SUBMAP/GLOBAL revision handling and long-run simulator GC
     before the 16 m coupled route. No controller/Nav2 promotion in this stage.
- Final revision checks: relevant **229-test host suite passes again** with the
  actual frozen C++ library; both paired attribution executions and both spatial
  plot executions pass. Python compilation, GUI-helper shell syntax, root and
  OmniLRS whitespace checks pass. No runtime rebuild needed (runtime sources
  unchanged); no fresh dense_flat/resource or Nav2 qualification claimed.
  Isaac is stopped; selected dev and read-only Foxglove bridge remain available.

### Optical-origin collision audit and v2 costmap integration (in progress)

- Hypothesis: the embedded optical origin and native 0.5 m raycast start can
  bypass chassis occlusion. Added `probe_lidar_optical_origin.py`, exposed as
  `python3 tools/run_local_isaac_fixture.py optical`. It copies active collision
  geometry with full base-relative transforms, checks origin overlap, and
  compares all 63,968 beam identities against independent zero-origin first hits.
  Original / 0.45 m / 0.65 m level mounts and 0.5 m / zero raycast minima are
  separate cases; the measurement minimum remains 0.5 m. No moving qualification.
- First launcher attempt `revision-06b66c31047a4e95bf713abf7ab77785` correctly
  refused while the owned idle inspection containers were still stopping.
  After stop completed, isolated attempt
  `revision-76347cde032644239809764f30346cd9` passed host/image CUDA preflight;
  results pending. No mission motion or host hardware repair.
- Added independently tested advisory cost conversion (`local_terrain/costmap.py`):
  measured occupied stays 100, unknown stays -1, distance taper only raises
  known supported cells, and the full map/local grid-corner transform is
  preserved. **18 focused tests pass** (costmap + launcher evidence contracts).
  This is not yet a live publisher or a navigation qualification. Native-v2
  transport and publication-causal GLIM association must precede live use.
- Optical audit completed: inner `revision-12cdcdee05de482eb3436aa5d3248113`.
  All five active colliders copied (chassis + four wheel meshes); 127,932 native
  samples per case, covering all 63,968 directions. Original origin overlaps
  chassis. Raised 0.45/0.65 m mounts with 0.5 m ray start still bypass **8,490/96**
  body hits. Both raised mounts with zero-origin rays have **zero mask mismatches,
  zero bypasses**, worst distance differences **5.72/7.63 micrometres**.
  This confirms a simulator sensing error, not just random GLIM thinning.
- Implemented separate opt-in `selected_optical_v2`: level [0.4,0,0.45] mount,
  unchanged mid-dense beams/rate, zero raycast minimum, 0.5 m software measurement
  minimum. Original selected JSON remains unchanged. Corrected profile excludes
  calibrated chassis hits (actual primitive box + 1 mm numeric margin), never
  sees through them; all raw returns are still archived. Moving startup now
  checks optical-origin overlap and first hits from zero, plus existing TF checks.
  Initial config preparation `revision-fe892b0fd64546c786f179c5f93d61ca` preceded
  chassis-filter declaration and is superseded, not overwritten. Final new frozen
  GLIM extrinsic/profile config: `revision-4c7b70a4acb44a07acb1bca0d63e3692`.
- Implemented opt-in `live_local_terrain.py` plus selected per-tick transport,
  recorder `--terrain-v2`, and launch wiring. Uses the existing rolling v2
  library `8a01220c...`, prepared/frozen observations, fixed reference, 5 s raw /
  60 s evidence; no classifier or safety threshold retuning. Publishes native
  classes and separate 1 m advisory cost taper in map frame; unknown on failure,
  stale clock, completion or shutdown. Actual callback order and raw payloads
  are archived. Global/submap changes still fail closed, not silently ignored.
  `slam_mapping` and `astrobot_launch` rebuild succeeded; installed imports pass.
  End-to-end live validation pending. This is not production/Nav2 promotion.
- First combined GUI run: recorder `revision-7d64e4f3f40044adbf01c77ca72e2e63`,
  native `revision-753d2f8be7194e1f9da47c0bde59301c`, mapper
  `revision-7f4e232b4833474a838a5100e0422d95`. Live startup origin/TF and sampled
  zero-origin first hits pass; guarded turn completes with 1,929 ticks, 322
  projections and all live readiness guards. Read-only topic probe
  `revision-0b28d13879a74f8a916c07ed4d7eebe3` verifies ten exact class/taper pairs.
  Example: 1,629 known / 3,600 cells, 27 occupied; this is availability, NOT
  terrain accuracy. Concurrent GUI compute is slower (intermediate p95 ~59 ms),
  not a passing resource result.
- **First mapper sealing failed:** default ROS SIGINT invalidates publishers
  before `finally`; final unknown publication raised `RCLError` and prevented
  `collection.json`. Completed live status alone had allowed the recorder to
  report success. Preserved unsealed archive, original runtime source and GUI
  log; it is not completed replay evidence. Fix: Python-managed SIGINT keeps
  ROS alive for final unknown publication/sealing, plus recorder requires a
  closed successful mapper seal with matching counts and every payload hash.
  Six seal regressions and eight strict optical-result regressions pass.
  A fresh combined capture is required; no retrospective seal is fabricated.
- Isolated real ROS SIGINT test passes (domain 179, no commands/clock/sensor
  data); archive `revision-8dbb925d7b4c4c75bb5336ad95e0cd07` is closed with zero
  inputs and correctly **not** complete. Added and executed durable ROS fixture
  `test_live_local_shutdown_ros.py`: **1 passes**, with its own fresh archive.
  Fresh corrected combined run: recorder
  `revision-1cf991e7efeb4c83b53849bc74e8e103`, native
  `revision-b0475470000c40128c84dc7c0c13578f`, v2 mapper
  `revision-d52e36b1a46546498ea81c23923c4435`. Same mount/rays/algorithm/guards;
  only shutdown handling and sealed-archive verification changed. **Completed
  and sealed successfully**, 1,963 native ticks / 32.700 s / 328 projections,
  every tick integrated. Startup and completion are unknown, not stale valid maps.
- Exact sealed receipt-order replay `revision-4ad584267cba46ea956a3975d8e8a202`
  reproduces **every layer, pose association and non-timing projection field**.
  Live topic check `revision-fd28626380584da38dbbc579d50a4e78` passes ten class/
  taper pairs; example near run end: 2,395 known cells / 3,600, 37 occupied,
  225 supported cells with raised proximity cost. Not terrain-truth coverage.
- Whole moving acquisition audit passes, `revision-26be5de2dea44205ab531284770f52e8`:
  **251,264** sampled zero-origin first hits, worst residual **0.203 mm**;
  exact native→GLIM wire match for **12,469,745 points**, all boundary windows
  included; GT agrees on all 1,963 exact acquisition ticks. The first offline
  audit `revision-0a0fa38ca5e245568482517ece73f2bb` stopped on a missing host-path
  archive alias for the host-created frozen config. Re-ran in an isolated
  network-none dev container binding the SAME external device at both `/archive`
  and its recorded host path; no contract rewrite or device-check bypass.
  Postprocessors now require an explicit matching corrected profile rather
  than hardcoding v1; all original timing/first-hit thresholds remain unchanged.
- GLIM base-frame accuracy (`revision-9c014233eade4c178757b2d5354a7827`, exported
  poses `revision-da62f4e1e67948b7ab27647746f15789`): first-published horizontal
  **8.463 cm p95 / 11.507 cm max**; final-revised **5.539 / 6.341 cm**.
  First-published roll/pitch/yaw p95 **0.189/0.334/0.382 degrees**; pitch still
  fails the unchanged 0.3-degree target. Revised overall pose targets pass,
  but are not live guarantees. No alignment/time fitting. GT initialization,
  physical IMU and synthetic RTK assistance remain declared experimental inputs.
- Concurrent GUI compute, ALL projections: first/unsealed mapper p95/p99
  **58.252/66.764 ms**; sealed repeat **55.094/60.794 ms**, **202.46 MiB** peak RSS.
  Retain both results; neither meets the existing p95<50 ms resource target.
  No classifier change, new dense_flat/resource qualification or navigation
  promotion is claimed. No runtime full GC in the sealed short run; 395 ms
  collection occurs at the explicit pre-request heap checkpoint. This does not
  resolve the prior long-run GC failure.
- Final relevant host suite: **213 pass**, including real frozen C++ library
  cases (no skipped local-core tests). Actual ROS shutdown fixture: **1 pass**.
  Package build, Python compilation and root/submodule diff checks pass.
  Isaac is stopped, bridge/dev remain available. New run instructions and
  topic/frame meanings are in [live inspection](live-inspection.md).
- Next gates: corrected-geometry independent terrain truth/coverage; longer
  translating combined run only after bounded SUBMAP/GLOBAL revision handling;
  GUI/runtime compute and both resource profiles; long-run simulator GC.
  The previous no-self-occlusion ideal-fixture percentages are not transferred
  to this physically corrected sensor as a qualification claim.

### GUI / read-only live inspection preparation

- User-requested live Foxglove loop, September 27 23:14 JST: found dev and
  bridge containers stopped (exit 137, not reported OOM) and no simulator or
  recorder running. Restarted only the selected dev and inspection bridge;
  archive device/bind, host/dev CUDA, container X11 and sole-command preflights
  pass. Started fresh `loop16` recorder
  `revision-b3ef0d9beb024eb6a047ce437099a7fb`, then GUI Isaac. Visible window
  confirmed; native capture `revision-7b4844a6d43545b190b13e53ea8d67d7`.
  Host Foxglove probe receives all five declared streams; first translation
  is underway at simulation stamp 20.18 s, commanded 0.10 m/s. No
  estimator/route/GC/watchdog changes. A bounded cleanup watcher will preserve `simulator-gui-loop.log`
  and stop only this exact simulator container when recording completes or
  fails; Foxglove remains available. **Failed safely at stamp 110.133339 s**:
  `motion simulation heartbeat expired`, lease wall age 509.68 ms. Runtime
  generation-2 GC is 447.28 ms, native callback max 503.33 ms. Cleanup archived
  the log and stopped the exact simulator successfully. Not a completed loop;
  no accuracy/terrain qualification derived from this failed recording.

- User's `docs/photos/2026-09-27_23-26.png` and `23-27*.png` prompted a read-only
  sparsification / rear-visibility diagnostic, not estimator or mount changes.
  Frozen run preprocessing uses random-grid sampling, 0.25 m voxels and 20,000
  target points; cropbox and outlier removal disabled. Matching stamp 20.016668:
  selected native window **39,498** points, local/aligned GLIM outputs **11,194**.
  Random-grid equalizes samples across voxels, not ring azimuth; close dense
  rings lose a larger share. Current native-return terrain fixtures bypass this
  GLIM downsampler; the future live terrain adapter must do so too.
- Rear diagnostic on the first six native ticks (~one revolution), sector
  cos(azimuth)<-0.97: five rings -51.67/-47.5/-43.33/-39.17/-35 degrees each retain
  156 valid returns. Rings -30 through 0 degrees retain none. In the checked
  intermediate-angle band, raw depth is exactly 0.5 m with hit intensity 255,
  removed by the existing minimum-range gate, before GLIM. First 60 ticks'
  sampled first-hit records identify those steep rear returns as terrain.
  This is a limited failed-prefix diagnostic, not complete visibility scoring.
- **Mount/collider concern requiring an optical audit:** current sensor origin
  [0.4,0,0.25] in base lies inside the previously active-PhysX-validated chassis
  cube x +/-0.50370, y +/-0.28545, z [-0.00877,0.25873]. Asset SHA256 still
  `357bd6cd9531cceb7971059c96d8a61052165eea7f8f6ff9bf28f35a5de1ca67`, matching
  this run. At 0.5 m along a rear -30-degree ray the position is [-0.033,0,0]
  (inside body), whereas -35 degrees gives [-0.010,0,-0.037] (below body).
  This explains why minimum-range/self-intersection semantics can produce
  near under-body rings followed by a gap. Do not treat apparent under-body
  visibility as physically valid merely because endpoint/offset-first-hit
  terrain agreement passes. Next diagnostic should cast from the actual
  optical origin against the complete body before considering a mount change.

- User-reported launcher portability fix: host `rg` was absent, and its failure
  was incorrectly presented as a missing recorder. Reproduced with an empty
  executable search path and mocked preflight/Docker commands (no GPU, ROS,
  archive writes or motion). Recorder counting now uses Bash builtins with
  exact node-name matching; zero/duplicates still fail closed. ROS discovery
  failures get their own error. Extra arguments (e.g. an accidentally pasted
  `docker logs` command) are rejected before preflight. Six new regressions
  fail on the old launcher and pass after the fix. Final focused inspection,
  WebSocket, CUDA, revision, selected-profile, loop and lease suite: **109 pass**
  using the same pytest command listed below; shell syntax and diff checks pass.
  No live capture
  restarted or recorder replaced for this launcher-only fix.

- Brief simulator review confirms the last long-route stop was generation-2
  Python GC: **409.83 ms**, only 240 objects collected, inside the unchanged
  native first-hit audit; total native callback 455.74 ms. The stack identifies
  the allocation that triggered collection, not proof that raycasting itself
  is the source of the large retained heap. No safe quick fix is verified;
  GC, acquisition dose and 500 ms watchdog remain unchanged. Full-loop
  completion remains open; do not relabel this as a GLIM catastrophe.
- Added opt-in `docker-compose.inspect.yml`, bounded read-only Foxglove config,
  `docker/run_selected_gui.sh`, and [live inspection guide](live-inspection.md).
  The bridge is separate from the existing dev container and binds only host
  `127.0.0.1:8766`; unrelated owner of 8765 is untouched. No navigation, browser
  command publishing, parameter writes, service calls or asset access enabled.
  All current selected-LiDAR/GLIM/IMU/GT/status topics are available on demand.
- Important integration gap exposed: the selected-sensor recorder explicitly
  requires pose-only acquisition. Current terrain-v2 is offline; the older
  live shadow grid is not v2. No legacy grid is launched as a substitute, and
  an absent costmap is documented as absent, never inferred free space.
- First GUI-intended `turn15`: recorder
  `revision-5c9d3d6072c94c399c3fc6fa81218c14`, native
  `revision-ff6036c30ade4747ad15b7888db81d2f`. Collection completes with all
  required topics, 349 revisions and healthy epoch 1, but **GUI fails**: host
  `xdpyinfo` passed while container X authorization failed and Isaac silently
  continued without a window. Preserved `simulator-gui-attempt.log` alongside
  this recording. This is not a passing GUI run or an audited accuracy result.
- Fix: read-only current-session Xauthority bind plus `XAUTHLOCALHOSTNAME`;
  add Xlib authorization probe inside the container before startup, and in the
  generic GUI launcher before world creation. No broad `xhost` permission.
  Fresh host/dev CUDA and disposable-container Xlib preflights pass. New actual
  **Isaac Sim Python 5.1.0 X11 window** observed. Fresh corrected-GUI recorder
  `revision-d908d0e395e64cc0892f2eefba9d8204`, native
  `revision-d60f36e780f049a7bbeb3fa661cf222b`: **collection completed** in
  394.86 wall seconds, no missing required topics, 351 revisions, healthy
  epoch 1. Declared ±15°/return sequence finishes; no longer route attempted.
  Preserved `isaac-gui.png`, `simulator-gui.log` and `inspection-smoke.json`
  (probe results, source hashes and actual loopback port bindings) in recorder
  archive. Simulator stopped after completion; inspection bridge remains up.
- Foxglove bridge 3.2.6 uses `foxglove.sdk.v1`; a legacy-only custom handshake
  misleadingly returned HTTP 400. New bounded, read-only
  `tools/probe_inspection_bridge.py` offers both protocols. Host handshake
  passes; advertised capabilities are only `connectionGraph` plus SDK `time`.
  Actual host-side messages received on selected input, corrected GLIM cloud,
  GT base pose, localization status and capture request. This tests transport,
  not a full Foxglove client layout or metric pose/terrain accuracy.
- Final focused inspection/X11/WebSocket, CUDA, revision-gate, sensor-contract,
  loop and lease regressions: **103 pass**. Shell syntax, composed config and
  root/OmniLRS diff checks pass. GUI smoke's maximum native callback is
  **73.12 ms**; only logged >50 ms GC is **434.90 ms before the motion lease**.
  This confirms full-GC cost is still substantial, not solved. No classifier,
  estimator tuning, sensor geometry or movement safety threshold changed.
  Long `loop16` launch is prepared, not started or qualified by these checks.
  Reproduce checks with `bash docker/run_selected_gui.sh preflight loop16`,
  `python3 tools/probe_inspection_bridge.py [--handshake-only]` and
  `python3 -m pytest -q -p no:cacheprovider tools/test/test_inspection_config.py
  tools/test/test_inspection_probe.py tools/test/test_cuda_runtime.py
  tools/test/test_live_revision_gate.py tools/test/test_selected_lidar_profile.py
  tools/test/test_natural_loop.py tools/test/test_shadow_capture_gate.py`.

### Longer-motion stability campaign — user-approved next stage

- User accepts the small bounded short-route errors as a reason to proceed,
  not to retune against those examples. Freeze estimator/sensor settings and
  exercise the existing `loop16` protocol: 16 m rectangle, four 90-degree turns,
  0.10 m/s / 0.10 rad/s maxima, fixed post-turn connectors, seed 43, physical IMU,
  GT-controlled acquisition/one-time GT seed and synthetic RTK. No mapper/Nav2.
- Keep original accuracy gates reported separately. Add descriptive stability
  checks fixed **before** acquisition: >0.5 m horizontal or >5-degree geodesic
  error sustained for 1 s, >0.5 m / >5-degree step in GT-relative error, all pose
  resets/clock gaps/missing brackets and health/route failures. Report whole-run
  maxima and errors over time, not merely p95 or endpoint closure. These are
  catastrophic-failure screens, not certified navigation-clearance bounds.
- Found short-test plumbing incompatible with a long loop: native evaluation
  capped all motion at 200 s and archive storage at 20,000 ticks. Opt-in selected
  `loop16` now declares 660 s / 40,000 ticks (existing leg deadlines total 640 s,
  plus settles/pre/postroll). Limits are in request/native metadata and checked
  against the explicit route; short-test limits stay unchanged. No live safety
  threshold, speed, leg deadline or 1,800 s wall deadline changed.
- Wheel discrepancy gets **diagnostic-only** additional casts at sensor origin,
  0.49 m and 0.51 m for every detected outlier. Original 0.5 m first-hit samples,
  residuals, selection and strict failed status remain untouched. Preserve all
  self hits; do not pick the later matching terrain hit. Added source hashes and
  regression cases for these counterfactuals and explicit bounded loop storage.
- Preflight: host/dev/Isaac CUDA context+allocation pass; external device
  `/dev/sda1` and `/archive` bind verified, >300 GiB free. Affected route/native/
  profile/pose tests **122 pass**. Recorder
  `revision-77b353e8129b4a348555d081219853b2`, native capture
  `revision-79b7dc3c5edb47f6b59047739eda8e15`: first translation completes and
  first 90-degree turn is underway. Outcome pending.
- Added `assess_glim_stability.py` with fixed-frame whole-run extrema, temporal
  quarters, sustained excursions and both adjacent-tick and **100 ms scan-scale**
  GT-relative error steps. The latter prevents 60 Hz interpolation from hiding a
  10 Hz jump. Six fixtures cover drift, short transient jumps, rotations, gaps,
  perfect moving trajectories and interpolation dilution. Only completed,
  hash-verified pose diagnostics can produce its descriptive stability report.
- Live checkpoint: first 90-degree turn and post-turn connector succeed; at
  simulation stamp 97.72 s the rover is ~2.68 m into the 6 m leg, beyond the
  historical loop's 97.10 s heartbeat failure. Base elevation has risen from
  ~0.12 m to ~0.46 m. No current fault; this is progress, not completion or an
  accuracy claim. Keep the full recorded route denominator and postroll.
- **Failed safely at the second corner**, stamp 152.1167 s: center distance from
  the nominal waypoint reaches **0.250058 m**, exceeding the unchanged 0.25 m
  skid envelope. 8,143 native ticks retained over 135.70 s. Recorder reports
  `simulator capture failed`; its last localization/clock/request/IMU/driver
  readiness checks are all healthy. This is an acquisition-control failure,
  not evidence that GLIM diverged. Preserve the failed prefix and unexecuted
  route; full-loop accuracy/stability remains unqualified.
- Two native first-hit outliers reproduce the wheel-origin discrepancy.
  At 44.4333 s the 0.49 m origin is inside the front-left wheel; 0.51 m is
  outside. The independent terrain hit at **0.6263429 m** agrees with native
  range **0.6263434 m**. The second case behaves identically at ~0.878523 m.
  This localizes the discrepancy to initial-overlap/min-range query semantics;
  it does not authorize replacing the original first hit or altering its gate.
- Changed acquisition turn policy (new route hash), **not SLAM**:
  `bounded_corner_feedback_v1`, position gain 0.6/s and max linear correction
  0.03 m/s along current heading while rotating. Target, survey, 0.25 m radius,
  angular speed, arrival/settle/stall/heartbeat/leg deadlines all unchanged.
  Already-outside poses still stop before any corrective command. Regression
  retains the exact recorded escape and shows corrective velocity opposes drift;
  deterministic disturbed-turn fixture stays below 0.15 m (not real dynamics
  qualification). Recorded turn commands must reproduce the declared policy.
- Added `diagnose_failed_glim.py`, separate from completed-route exporters:
  keeps every recorded native index, lists missing GT/pose tail associations,
  forbids extrapolation and fitted alignment, and always preserves failed status.
  Its diagnostic cannot feed completed-run stability or terrain qualification.
- Failed-prefix diagnostic `revision-143b42e3e0bc4ac889718258d5b5da6a` pairs
  8,131/8,143 ticks over **8.649 m**; final 12 ticks lack pose brackets and remain
  explicit missing evidence. First-published XY p95/max **7.761/11.560 cm**,
  orientation max **0.629 degrees**; revised XY p95/max **6.710/8.798 cm**.
  No large pose error in the usable prefix, but no completed-route conclusion.
  Combined route/native/failed-prefix regressions **95 pass**. Fresh capture of
  the same loop with changed turn policy follows successful CUDA preflight.
- Corrected-turn attempt: recorder `revision-5a5ed7ef1a7b47fa951c1148a6af087c`,
  simulator `revision-c9dc81e2a89c4b0fb1173521e1496c75`. Fresh initialization,
  source/route hashes and evidence; same frozen estimator/sensor configuration.
  Outcome pending. No failed file overwritten or safety envelope enlarged.
- Corrected-turn attempt also **fails safely**, at 159.2833 s in the second
  corner (~84 degrees through the turn), on the same 25 cm radial guard.
  Recorder health remains normal, epoch 1. Diagnostic
  `revision-e464324521254731900c707ef28b3342` retains 8,542 ticks over 142.35 s;
  8,531 pair successfully and the 11-tick shutdown tail remains missing.
  Paired travel **8.572 m**; first-published XY p95/max **8.706/12.187 cm**,
  geodesic maximum **0.614 degrees**; revised **7.513/11.100 cm**, maximum
  **0.417 degrees**. Neither failed run establishes full-loop stability.
- Distinct controller hypothesis from both archived turn traces: observed yaw
  rate is ~0.05 rad/s for a 0.10 rad/s command. Effective lateral-velocity/yaw-rate
  ratios differ substantially by corner (~0.02 vs 0.20 m in the corrected run),
  so this is not evidence for a single wrong static drive-frame offset. Late
  along-heading feedback cannot remove a now-perpendicular position error.
  Supersede reactive-only control with `anticipatory_corner_feedback_v2`: its
  fixed command reference is 15 cm ahead along the **declared entry heading**.
  This reserves room early in the turn; it does not move the nominal waypoint,
  arrival heading, surveyed region or 25 cm guard. Same 3 cm/s correction cap.
  Four explicit body-twist fixtures spanning measured side-slip/bias stay below
  22 cm; they are not Isaac dynamics or localization acceptance. Full ideal
  route now exercises actual frozen post-turn connectors, with the existing
  2 cm translation / 25 cm turn arrival contracts. **86 route tests pass**.
  Another live trial must validate this hypothesis; do not infer success from
  the simplified fixtures or hide either previous failure.
- Final pre-capture regressions: **136 host / 25 sourced isolated ROS tests
  pass**, root and OmniLRS diff checks pass. Fresh host/dev/Isaac CUDA allocation
  and external archive checks pass. Anticipatory trial: recorder
  `revision-0dfc561521e64c9ca3e40875e766e226`, native capture
  `revision-9d75d324ca174fdc992fd68b2d44912c`; outcome pending. Same frozen
  estimator and single-LiDAR profile, independently hashed new controller route.
- Anticipatory live checkpoint: both first and second turns plus their frozen
  connectors complete. At 188.35 s the rover is on the far 2 m leg, beyond both
  previous failures. Second-turn exit is about 11 cm from its nominal corner.
  This validates progress of the control hypothesis only; whole-route accuracy
  and stability still await capture completion and an independent pose audit.
- Third attempt **fails during turn 3**, not on skid: recorder observes stale
  request/clock/physical-IMU heartbeats while pose health and epoch 1 remain good;
  producer subsequently rejects its revoked recorder lease at **223.3167 s**.
  **11,176 native ticks / 186.25 s** retained. Failed-prefix diagnostic
  `revision-72104b579c18414b8defbdc2f00736ff` pairs 11,164 ticks / **11.128 m**;
  12 unpaired tail ticks include four missing GT snapshots. A final revision has
  no following clock and is explicitly excluded from causal pose evidence.
  First-published XY p95/max **15.551/20.164 cm**, geodesic max **0.630 degrees**;
  revised **13.760/16.009 cm**, max **0.451 degrees**. Thus no observed large
  pose divergence in the usable prefix, but the prior ~10 cm error is not a
  demonstrated hard bound, and original accuracy gates still fail.
- Timing investigation: no runtime full-GC pause in the simulator log; native
  callback max **79.52 ms**. Ten read-only finished-archive quota checks take
  **38.44–68.21 ms** (diagnostic, not live timing). Terminal MCAP event is
  revision **1885**, **SUBMAP**, containing **1,835 poses** rather than ordinary
  49-pose ODOM updates. Its receipt precedes lease revocation by **385.60 ms**.
  **Optimization disabled does not mean submap assignment/revisions disabled.**
  The recorder was running an offline all-history SE(3) validator synchronously
  on this batch. This is a concrete callback-latency hazard; the existing
  heartbeat failure remains real and must not be waived as benign replay noise.
- Added recorder-only `LiveRevisionGate`: vectorized SE(3) checks at identical
  tolerances; retain immutable frame timestamps and canonical revision digests,
  but no historical pose heap or pose-query API. MCAP and offline `PoseTimeline`
  remain unchanged. Atomic failure, snapshot completeness, duplicates, clocks,
  extrinsics and reset barriers have parity fixtures, including 1,835-pose batches.
- Actual-input processing diagnostic `revision-9b6bfaadce11409d8990e5311ddf1af5`:
  all **1,885** revisions yield identical validation-state hashes. Large-batch
  conversion+validation **259.69→78.79 ms**, validation alone **199.44→19.27 ms**;
  callback p99 **14.40→2.48 ms**. Old live recorder retained **93,023 pose
  versions**; new gate retains zero poses and all **1,885 frame identities**.
  Both use normal enabled GC. This sequential offline processing diagnostic
  excludes disk reads and does not establish live heartbeat acceptance.
  Added bounded per-stage callback/readiness telemetry and recorder source hashes
  to subsequent collections; no safety deadline changed.
- Recorder fix regressions: **151 host / 26 isolated ROS tests pass**; both
  worktree diff checks pass. Fresh CUDA/archive preflight passes. Fourth capture
  keeps the anticipatory controller and frozen estimator/sensor unchanged:
  recorder `revision-8d5f428e340b460c8336665b5929ba0b`, native
  `revision-35d9b7335a244711bab7daaef9dbd54f`. This is live validation of the
  recorder processing change; result pending, previous heartbeat failure retained.
- Fourth live checkpoint: third 90-degree turn and connector complete; at
  230.7833 s the rover is on the 6 m return leg, beyond the previous submap-batch
  heartbeat failure. No live acceptance yet; sealed revision/processing records
  must still verify which large batches were handled and whole-route pose error.
- **Fourth attempt stops on the return leg**, stamp **250.8500 s**, after three
  complete 90-degree turns. Native archive retains **13,681 ticks / 228.000 s**.
  Diagnostic `revision-feec9e9bd7db414bbbea9401257e7aa0` pairs **13,673 ticks /
  13.646 m**; all recorded ticks have exact GT, eight shutdown-tail ticks lack
  pose brackets and remain explicitly missing. No fitted alignment, retiming,
  extrapolation or acceptance promotion. First-published XY p95/max
  **11.552/19.285 cm**, Z p95/max **1.739/2.275 cm**, rotation max **0.646°**;
  revised XY **8.378/11.005 cm**, rotation max **0.436°**. First-published
  temporal-quarter XY p95 is **5.341 / 9.369 / 11.532 / 13.049 cm**; final error
  is **7.303 cm**. Report this growth/variation, not a universal 10 cm bound.
- Controller evidence: maximum nominal-corner radii across the three completed
  turns are **14.42 / 15.65 / 15.13 cm**. The same 25 cm guard, fixed connectors,
  speed/deadline/floor checks stayed active. This is guarded GT acquisition,
  not a qualified autonomous navigation policy.
- Recorder fix has live evidence: **2,302 ODOM + one SUBMAP + one GLOBAL**
  revisions recorded. SUBMAP revision 1623 carries **1,573 poses**, GLOBAL 1624
  carries **1,623 poses**. Largest revision callback **70.91 ms**, readiness
  check **113.04 ms**. Per-frame placement changes at those two batch events
  are at most **5.22e-14 m / 1.26e-12 degrees**: bookkeeping updates in this
  frozen configuration, not meaningful loop-closure corrections or frame jumps.
  All archived recorder dependency hashes still match the tested source.
- **Remaining failure is simulator GC**, independently instrumented: generation-2
  collection **409.83 ms** at 250.8333 s, `native_capture` → `surface_audit` →
  PhysX raycast callback. Whole native callback reaches **455.74 ms**; surrounding
  simulator work then expires the unchanged clock/IMU/lease heartbeat. Only 240
  objects are collected, consistent with an expensive scan of the large retained
  Isaac Python heap rather than a large amount of useful garbage reclamation.
  Recorder pose health remains good, epoch 1; recorder revocation produces the
  secondary simulator message about the missing identified recorder. The
  preflight-inclusive 1.646 s max tick gap is **not** this runtime GC duration.
- Final relevant regressions: **151 host + 26 isolated ROS pass**. Additional
  revision/lease/offline-reference regressions **110 pass, two skip** because no
  frozen C++ terrain library was supplied; those skipped terrain-core checks
  are not counted as acceptance. No terrain/GLIM C++ algorithm changed. All four
  failed captures, their denominators and missing tails remain preserved. Isaac
  is stopped; no fifth blind retry and no watchdog/GC/surface-gate relaxation.

#### Campaign conclusion and next improvement cycle

The longer tests exposed control and recorder-latency failures before any large
SLAM divergence. This is positive but partial localization evidence, not a
completed-loop or navigation qualification. One-time GT initialization and noisy
synthetic RTK remain part of every run; unaided/drift/dropout robustness is untested.

1. Keep the current estimator, sensor, controller and reference thresholds fixed.
   Profile the simulator's retained Python heap and audit allocation path against
   the **recorded 410 ms GC failure**. Reducing full-GC frequency alone is not a
   bounded-latency solution. Prefer exact batch/native query handling and bounded
   data ownership; if needed, separate diagnostic work from the physics/control
   process without dropping acquisition evidence, live guards or same-state GT.
   Do not disable/freeze GC, hide contention, or increase lease deadlines to pass.
2. Require small-scale output-equivalence and sustained normal-GC timing tests
   before another full acquisition. Keep the same sampled rays/hit identities,
   native stamps, source hashes and archive contract. Explicitly test large
   SUBMAP/GLOBAL batches as well as normal ODOM updates.
3. Then repeat the **entire** loop, retaining failures. A completed recording must
   pass exact native/ROS transport and GT/pose association before the predeclared
   whole-run catastrophe screen. Keep the independent wheel-origin surface
   discrepancy failed until first-hit semantics are properly resolved.
4. After a completed-loop baseline, declare held-out reverse/curved routes and
   seeds, followed by explicit RTK dropout/noise and consumer-time terrain tests.
   Do not turn this single-seed, RTK-assisted partial route into a general bound.

Executed validation commands (ROS commands inside the sourced dev overlay;
capture additionally requires the documented recorder-first Isaac startup):

```bash
python3 -m pytest -q -p no:cacheprovider \
  tools/test/test_live_revision_gate.py tools/test/test_natural_motion.py \
  tools/test/test_natural_loop.py tools/test/test_selected_lidar_profile.py \
  tools/test/test_selected_native_audit.py tools/test/test_terrain_native_capture.py \
  tools/test/test_measure_exported_glim.py tools/test/test_glim_stability.py \
  tools/test/test_failed_glim.py
# 151 passed
ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q -p no:cacheprovider \
  tools/test/test_selected_transport_ros.py tools/test/test_shadow_capture_ros.py
# 26 passed
python3 -m pytest -q -p no:cacheprovider \
  tools/test/test_live_revision_gate.py tools/test/test_revision_evidence.py \
  tools/test/test_shadow_pose_replay.py tools/test/test_shadow_capture_gate.py \
  tools/test/test_shadow_terrain.py tools/test/test_natural_holdout.py
# 110 passed, 2 terrain-library-dependent checks skipped
python3 tools/run_shadow_capture.py \
  --config /archive/revision-1545bcc16e884638ae30b137cacb17ea/config \
  --archive-root /archive --require-physics-imu --natural-motion loop16 \
  --selected-lidar --wall-seconds 1800
python3 tools/profile_revision_validation.py \
  /archive/revision-0dfc561521e64c9ca3e40875e766e226
python3 tools/diagnose_failed_glim.py \
  /archive/revision-35d9b7335a244711bab7daaef9dbd54f \
  /archive/revision-8d5f428e340b460c8336665b5929ba0b
git diff --check
git -C OmniLRS diff --check
```

### Moving-GLIM integration in progress

- User accepts the bounded terrain result as sufficient to proceed. Freeze it;
  do not block localization testing on full navigation qualification. First run:
  selected level sensor, seed 43, existing `straight_slow` 2 m out/return route,
  physical IMU, one-time live GT initialization and unchanged synthetic RTK
  assistance. No terrain mapper/Nav2 or additional command publisher. Compare
  complete same-time base-frame trajectories without fitted alignment.
- Code inspection: legacy collector hardcodes 5-degree pitch and uniform
  column/ring arrays; recorder requires legacy raw/filtered clouds. Add an
  explicitly declared selected profile, exclusive Generic sensor replacement,
  six-tick timed cloud topic and per-return archive. Render every 60 Hz physical
  tick for this profile only. Keep old protocols and all motion/heartbeat guards.
- New stationary readiness audits 12 native ticks × 128 first hits plus measured
  TF; all moving ticks retain 128 current/previous-pose first-hit diagnostics.
  Producer runs before initialization and through recorder postroll; native
  measurement sealing does not stop the final estimator pose bracket. Actual
  live timing, geometry and GLIM accuracy remain **unrun**, not inferred from
  static adapter tests. Host NVIDIA/CUDA allocation and archive preflight pass.
- Initial environment recovery: host port 8765 belongs to an unrelated process;
  use no-published-ports `compose run --no-deps --name astrobot-selected-dev`
  instead of disturbing it. Fresh container needs `astrobot_interfaces` and the
  launch/supervisor/GLIM overlay rebuilt. Cached GLIM CMake selection initially
  chose old image headers; explicitly select workspace `install/glim/share/glim`
  (not its generated `cmake` subfolder). Final GLIM ROS build and all four CTest
  frame/revision tests pass. 24 ROS/profile contracts and 97 host contracts pass.
- Historical archive's old Linux device number changed 2065→2049 after reconnect.
  Do not resume/rewrite that run's device lease. `prepare_selected_glim.py` makes
  a **read-only historical config import** on the verified external root and
  allocates a fresh device-bound output; preserves original contract/config
  hashes. New profile: `revision-1545bcc16e884638ae30b137cacb17ea`, based on
  completed recorder `revision-08dbf22b74b048f3838a14a368843da8`.
- First selected-profile recorder `revision-ada8208a15b049b9bd31a01ac377a28d`
  started before Isaac, without mapper. CUDA context/allocation passes inside
  dev and Isaac startup. Outcome pending; this is a new sensor/runtime experiment,
  not claimed to be an exact historical-binary A/B replay.
- First live attempt fails **before any motion grant**: recorder reports
  `restart_timeout`; simulator `revision-32cf45cc42b343a591c535fe3783e874` passes
  12-tick stationary first-hit/TF and floor checks, but GLIM receives corrupt
  point times. MCAP inspection establishes the cause: NumPy 1.26/Isaac structured
  `concatenate` packs the declared **32-byte** point into **30 bytes**, whereas
  host NumPy 1.21 preserves padding. ROS advertised 32-byte stride over a 30-byte
  payload. The prior host-built C++ input probe could not expose this producer-side
  version difference. Fix by allocating the explicit wire dtype and zeroing
  padding, then copying structured records; reject payload/stride mismatch before
  ROS publication. No timestamps, observations or physics are adjusted.
- Also disconnect the legacy OmniGraph execution edge explicitly: deactivating
  its USD graph alone left compiled callbacks querying the removed old sensor.
  Preserve the failed recording and repeat only after host and container NumPy
  regressions pass. This is an adapter/runtime bug, not measured GLIM drift.
- Second recorder `revision-ad235e6f6fcc4f79b5280cd19b149a26`, constructor-only
  scene `revision-09e5b05835f44b2685513d5972c640ce`: explicit graph disconnect
  used relative strings not resolved by this existing-graph API. Replaced them
  with validated absolute Attribute objects; constructor errors now write a
  failure marker before propagating. No motion or GLIM accuracy from that attempt.
- Third recorder `revision-81c6b468b56d4c52aa865fb6d4dc2bf8`, simulator
  `revision-7f3f750bf4ad44bca6fd57b7c22694b1`: corrected wire stream works; initial
  GLIM/GT error **0.264 mm / 0.00171 degrees**, live TF/IMU readiness passes.
  **Fails stationary pre-roll, no nonzero commands**, on 500 ms clock heartbeat.
  Gen-2 GC **404 ms** plus native callback work reaches **447 ms** before other
  simulator work. Do not relax watchdog or claim moving accuracy.
- Next distinct resource hypothesis: pose-only GLIM does not consume the
  approximate `/gt/map` raster or resource maps. Disable those explicitly for
  this isolated acquisition; keep **every physics-tick GT pose**, physical IMU,
  selected returns, all 128 first-hit samples/tick and all safety checks.
  Default missions still publish the raster. Record the ablated workload in
  request/manifest/timing; not concurrent full-stack resource acceptance.
  Also reuse the same-tick audited pose instead of traversing the robot twice,
  precompute fixed mount geometry, and avoid building discarded previous-pose
  hit dictionaries. Evidence, residuals and current first-hit identities unchanged.
- Resource ablation recorder `revision-086ededb83594b6188ffeb2fc075d9a8`, simulator
  `revision-7933a00013b44c0cacd41f4f7b762b5d`, still **fails before motion**:
  434 ms full GC plus surrounding work expires the 500 ms lease. Removing
  unused raster/resource work is not a GC fix; keep this failed result explicit.
  Native callback max is 61.6 ms. Next code-level cause: `_pose` traverses *all*
  robot descendants to find a static `base_link` prim on every 60 Hz tick.
  Cache **prim identity and static sensor metadata only**, never pose matrices;
  current USD/physics transforms and mount checks remain per tick. Regression
  exercises moving transforms while forbidding traversal. Add bounded GC stack
  provenance to locate any remaining full-collection trigger.
- Found GLIM's shutdown default `/tmp/dump`; for GT-seeded shadow acquisition,
  supervisor now explicitly directs it into the recorder's current externally
  leased run/epoch. Latest failed dump copied into its recorder artifact; prior
  default scratch dumps were not used as evidence. No estimator parameter change.
- Cached-prim attempt: recorder `revision-2eeae3f7866c4eaab928069a84d1742a`,
  simulator `revision-ba553e02a900408cba1beed464eaff34`, again stops before nonzero
  commands. New stack traces identify the actual repeated full-GC trigger:
  `json.loads(sensor_audit.json)` in the acquisition loop, **every 60 Hz tick**.
  The completed selected audit is ~954 kB with nested first-hit records; rebuilding
  it repeatedly triggers 369–400 ms full-heap scans, mostly collecting zero objects.
  Prim caching alone was not the fix. Decode/validate this write-once artifact
  once, retain its SHA-256 in the simulator manifest, and require exact hash
  agreement in the final evaluator. Regression proves one read across 10,000
  calls and rejects failed/missing TF audits. No GC disabling, threshold change,
  sample reduction, pose caching or watchdog relaxation.
- One-time-audit run now reaches **actual guarded motion**: recorder
  `revision-3855e5257dd6443b918ebac116cf2f6c`, simulator
  `revision-52959d63312c4e90a4042806bc9d4453`. Same selected sensor/config and
  declared raster/resource ablation. Full out/return, native timing/GT matching,
  transport integrity and moving pose accuracy remain pending until completion.
- This run stops at 1.98 m outbound, before the return: **incomplete**, never
  accuracy acceptance. Stack trace identifies full GC (397 ms, zero collected)
  inside `check_run`'s recursive `Path.rglob` of the growing native archive.
  Replace only that walk with streaming `os.scandir`: exact same file-byte quota,
  device/reserve checks and symlink semantics; no cached size or relaxed lease.
  Add equivalence, no-rglob and I/O-failure regressions; include archive guard
  source hash in new manifests. Repeat only after tests and preflight pass.
- Archive tests: 10 pass. Exact count on the failed 445,112,663-byte run agrees;
  host diagnostic walk 32 ms→7 ms (not live timing acceptance). New recorder
  `revision-3885b8425b83408bbdf77c55c7e810bd`, simulator
  `revision-2686b61d2e7b435cbf34e06435c4a8e9`, same route/sensor/config/guards.
- Failure-only geometry inspection of `52959...`: one of 2,400 ticks fails the
  strict first-hit check. At 42.950002240 s / point 8779 the reference cast starts
  inside `/Robots/astrobot_0/front_right_wheel_link/collisions`, reports a zero
  distance hit, then terrain at the native range. All other samples in that tick
  agree closely. This is an unresolved self-geometry/query discrepancy, **not
  a 42.6 cm GLIM pose error**. Do not discard the hit or relax the 1 mm gate.
  [PhysX's documented initial-overlap semantics](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/GeometryQueries.html)
  support this diagnosis, but do not establish the Generic sensor's filtering.
- Added separate `measure_exported_glim.py`: completed recordings only, exact
  GT/native snapshots, frozen exported revisions, exact timed-window transport,
  all-tick pose brackets, fixed home and physical base lever arms. Reports native
  surface failure explicitly; cannot produce terrain qualification or a reusable
  qualified-pair artifact. Four fixtures retain a real 3 cm offset and reject
  wrong mounts, missing GT ticks and missing final GLIM brackets. This separates
  pose measurement from sensor-reference qualification without weakening either.
- Straight route **completes**: 81.883 s measurement, 3.951 m actual travel,
  4,914 native ticks. Export `revision-d94557b325d64f8fb1fa813f8a8df305`, diagnostic
  `revision-a9a0e664df6841eda5a98f6d58ac8ffd`. Exact native→ROS bytes pass for
  819 windows / 32,955,978 valid points, all boundary tails included. Published
  GT matches all ticks (max position discrepancy 1.30e-8 m); no fitted alignment.
  First-published p95 XY/Z **11.315/1.538 cm**, roll/pitch/yaw
  **0.155/0.312/0.367 degrees**. Final-revised p95 XY/Z **8.542/1.236 cm**,
  angles **0.119/0.233/0.213 degrees**. First-published XY/pitch narrowly fail
  unchanged 10 cm/0.3-degree targets; retrospective passes are not live passes.
  First pose availability delay median/p95 **167/217 ms**; typical final revision
  is 48 estimator updates later. This distinction matters for terrain consumers.
- Whole sensor qualification still **fails**, one of 628,992 sampled rays:
  87.933337919 s / point 4757, initial overlap with the **front-left wheel**,
  later terrain agrees within 1 micrometre. No ray removed. Audit
  `revision-7964b56da46d4f208414dcf9829bdada` preserves that failure. An earlier
  audit `revision-f052831d6726415cbb875427ca20d0d1` exposed a remaining hardcoded
  legacy topic in runtime config validation; add explicit selected-profile wiring
  and regression (legacy default still rejects it). 23 replay-contract tests pass.
- No full GC during the measurement after streamed quota fix; the retained
  399 ms collection occurs only at the explicit **pre-request** heap checkpoint.
  Overall max-tick metric includes preflight and is not live latency acceptance.
  Host affected suite **202 pass**, ROS/NumPy1.26 affected suite **60 pass**.
- Next bounded baseline: unchanged estimator/sensor/config on existing `turn15`
  (+15, -15, return heading) route, physical IMU/GT-start/RTK assistance and all
  clearance/skid/heartbeat guards. Distinct motion protocol, not a favorable
  straight-route retry and not a threshold change. Rotation baseline pending.
- `turn15` **completes**, with all motion guards intact: recorder
  `revision-8e183b2c89a84ee3a66294d0c243cd12`, simulator
  `revision-9501bb24402a4d02945ee52a0de5f82f`, export
  `revision-fe5fe644bffe4469b9e45a972bec21bf`. Whole audit
  `revision-a392412007da460f809106ae8b40f6be` **passes**: 1,927 native ticks /
  32.100 s, 246,656 first hits (max 0.0422 mm), 322 windows / 12,700,286 points
  matched exactly, and all exact GT snapshots/config/plugin/motion contracts.
  This rotation result does not erase straight-route self-geometry failure.
- Rotation accuracy `revision-0d1c9f446c6544e39528230e5c6f2bd1`: first-published
  p95 XY/Z **7.642/0.543 cm**, RPY **0.195/0.345/0.299 degrees**; final-revised
  **7.712/0.420 cm**, RPY **0.194/0.242/0.152 degrees**. First-published pitch
  exceeds the unchanged 0.3-degree target; revised passes. No catastrophic drift,
  estimator reset or starting-frame mismatch was observed in these two bounded
  assisted protocols. They are not unaided SLAM, long-route or Nav2 acceptance.
- Final affected host suite **229 passes**, sourced ROS suite **60 passes**,
  four GLIM ROS CTests pass; diff/selected-new-file formatting checks pass.
  Isaac stopped after each capture; no collector or navigation command owner
  remains active. Default terrain and estimator tuning unchanged.
- Investigate the live/revised gap on frozen recordings, not by retuning against
  the route: `diagnose_glim_revision_delay.py` evaluates fixed **0.3/0.5/1.0 s**
  receipt deadlines after acquisition. Queries use only revisions received by
  that deadline; a missing association or post-roll tail invalidates the whole
  aggregate rather than trimming samples. Three regression fixtures pass.
  This is offline acquisition-pose sensitivity, **not** current robot pose,
  navigation latency, runtime acceptance or permission to use final revisions
  early. Both recordings' fixed-deadline results are pending.
- Fixed-deadline diagnostics finish with **every tick represented at every
  deadline**, no missing brackets/tails. Straight
  `revision-c6c38e479a7248aa9efa09bdd579981d`: 1.0 s p95 XY **9.899 cm**,
  pitch **0.284 degrees**, diagnostic pose targets pass (surface failure remains).
  Turn `revision-17b6044a7a0f45fe81be0f7895194490`: 1.0 s XY **7.731 cm**,
  pitch **0.314 degrees**, still fails pitch. Short delays are not a general
  fix; **no delay added** to the estimator/consumer and no target relaxed.

### Moving-GLIM checkpoint and next improvement cycles

This stage establishes two reproducible moving baselines with the selected
sensor and fixes real integration defects (wire packing, legacy sensor execution,
repeated stationary-audit decoding, allocation-heavy quota scanning and external
GLIM dump placement). It does **not** establish full moving terrain/navigation
qualification or solve every existing pose target. Terrain algorithm and GLIM
factor settings were not tuned in this stage.

1. **Resolve sensor/reference self-geometry discrepancy before straight terrain
   replay.** Add independent native-vs-query diagnostics for wheel initial
   overlap, including actual ray origins/filter policies and moving collision
   transforms. Do not select a later matching hit or drop the failing ray. The
   strict failed straight run remains failed even if a later protocol is fixed.
2. **Attribute first-published pitch/XY residuals.** Use these frozen native
   inputs and measurement/receipt times to distinguish IMU prediction, scan
   matching and fixed-lag updates. Preserve current first/revised/deadline
   baselines; avoid changing mounting or adding blanket latency to chase a score.
   Explicitly support selected timed-input replay before using the old legacy
   topic replay runner. Any trace-instrumented timing is diagnostic only.
3. **Then evaluate a single justified estimator/consumer change**, against both
   complete protocols with unchanged targets (XY p95 10 cm, Z 2 cm, roll/pitch
   0.3 degrees, yaw 1 degree). Keep first-available and revised results separate;
   terrain evidence must follow revisions without granting navigation authority.
4. **Progress to combined motion / longer loop**, followed by estimated-pose
   terrain replay and concurrent shadow integration, only with explicit remaining
   limitations and unchanged motion guards. These two short routes do not qualify
   long-term drift, loop closure, unaided localization or full-stack throughput.

Validation commands executed: final `python3 -m pytest` affected host suite
(232 passes, including revision-deadline fixtures); sourced isolated
ROS domain 179 affected suite (60 passes); `ctest --output-on-failure` in the
built `glim_ros` directory (4 passes). Both captures use `run_shadow_capture.py`,
`export_shadow_trajectory.py`, and `audit_natural_holdout.py --selected-lidar`;
strict audit outcomes remain separate from `measure_exported_glim.py` diagnostics.
`diagnose_glim_accuracy.py` ran on the qualified turn input. Exact runner setup
and the added diagnostic commands are in the runbook. Isaac is stopped; no
recording, GLIM worker or command publisher remains active in the task container.

### Earlier selected-geometry terrain assessment

- User selected the level mid-dense setup; stop beam-layout selection. Freeze
  `tools/local_terrain/selected_lidar.json`: one sensor, identity mount rotation,
  translation [0.4,0,0.25] m, 32 rings (+10..0 / -2..-30 / -35..-60 with 5/20/7
  rings), original 1999 columns, native angular increment/ranges and 10 Hz.
  Capture and assessment now reject drift from that contract. Loaded base height
  is still measured, not replaced by the mount translation. This selects the
  development setup; it does not silently replace the incompatible production
  uniform-LiDAR adapter or promote terrain/navigation authority.
- First assess all existing validated native views with **additive** 0–1 / 1–2 /
  2–3 m base-origin center-selected bands and forward 90-degree sectors. Keep old
  grid/regions, all ticks/phases, whole-cell obstacle overlap and all unknowns.
  Report availability, false occupation and unknown reasons per range; no changed
  denominator may masquerade as improvement. Existing eight-box test cannot
  qualify slopes/holes/self-occlusion or all hazards throughout those bands.
- GLIM integration is conditional on useful terrain evidence, not the previous
  distant-feature screen. The current generic native ray fixture is sound, but
  the live collector assumes uniform column×ring arrays and cannot be reused
  unchanged. No guessed per-ray timing or ROS full-scan substitution is allowed.
- Additive range assessment `revision-a3ce89c7a7c247b3b6fd21e868a83c16` preserves
  the original six-view results: flat mid-dense full-azimuth free coverage is
  **91.59–93.67 / 94.36–96.70 / 30.24–31.71%** at 0–1 / 1–2 / 2–3 m.
  Forward 2–3 m is 78.99–83.67%. Mid-range support remains a real gap, primarily
  no-return/insufficient-area/unsupported-extent cells, not false-free output.
- Next fixed acquisition hypothesis: the off-center origin's yaw sweep fills
  stationary ring gaps. Selected sensor only, same flat/eight-box geometry,
  24 settled base yaws **0..345 degrees in 15-degree steps**, no base translation,
  same 6 warmup + 12 recorded ticks per view, original stamps continuous per scene.
  This is explicitly increased dose and synthetic sensor placement, NOT robot
  rotation, live acquisition or optical self-occlusion. Transform must rotate
  both sensor orientation and 0.4 m lever arm; tests reject spinning about the
  sensor itself. Keep triangle colliders, 2 mm geometry gates, mapper/library,
  all phases/regions and classifier thresholds. Score all startup publications.
  Development target: >=95% flat-ground support per full-azimuth 1–2 and 2–3 m
  band at every phase, zero false-free cells throughout, and all eight boxes
  occupied after observation. This additional geometry screen does not replace
  the existing independent >=95% safe-motion availability qualification contract.
- Native yaw launcher `revision-b96d82789b754a518b3ff7da1a6ac70e`, inner
  `revision-79dabea8584946f48f04bf07fdc5ce44`: **48/48 cases pass**. Terrain
  `revision-58aacf438269495e948e2efd2d04c9e2` passes the declared flat/hazard screen:
  **100%** supported flat cells at 0–1 and 1–2 m; **97.20–98.08%** at 2–3 m;
  **8/8** boxes at every phase, **zero false-free in 2,304 maps**. With boxes,
  free-reference support is **91.63–95.78%** at 1–2 m and **93.24–93.91%** at
  2–3 m; maximum four false-occupied ground cells, no flat-scene false occupation.
  Report obstacle-boundary losses rather than exclude a halo from denominators.
  Existing library analytic contracts also pass **108/108**, without skips.
- Proceed to opt-in GLIM **input-contract integration**, not motion acceptance.
  Source audit finds the legacy configuration sets `global_shutter_lidar=true`:
  GLIM then zeros all point times. A six-native-tick window must instead retain
  actual acquisition tick offsets and use an explicit non-global-shutter profile.
  Build/test a bounded nonuniform-ray adapter and check it through the installed
  ROS converter, TimeKeeper and preprocessor. No GT poses/labels or synthetic
  IMU enters this bridge. Settled yaw recordings cannot establish moving GLIM
  accuracy because they have no synchronized physical IMU or continuous motion.
- Implemented bounded `generic_lidar_adapter.py`: six complete native ticks per
  cloud, original sensor-local XYZ, native ring/beam IDs and tick offsets in
  seconds; native stamps are rounded only to ROS integer nanoseconds. Raw archive
  remains unchanged; invalid/missed returns are excluded from GLIM geometry using
  the declared range/intensity rule, never GT. Missing/reordered/repeated beams,
  sequence/time gaps and partial final windows invalidate the stream. No live
  publication or synthetic IMU. Ten focused host contracts pass.
- First installed-GLIM input probe `revision-dc49779786a8411d91d99a6643e29a01`
  failed compilation because the standalone Jazzy include list omitted
  `rosidl_typesupport_interface`. Preserve it; add that single-prefix header path
  and repeat in a fresh archive. No mission environment/setup changes.
- Second compile-only failure `revision-e29a88f2c2f04a498b9200073d95c2d3`
  exposes the generated message's `rcutils` include dependency; add its Jazzy
  include path too. No point data was processed in either failed build.
- Input-contract run `revision-c6e002567ee94a9b9ed3c4a3f15ad3c7` passes **96/96
  windows**, **4,512,944 valid input returns**, using the installed GLIM ROS
  converter, TimeKeeper and preprocessor (15,554–15,712 output points/window).
  Sensor-local coordinates and native tick times are unchanged by conversion;
  preprocessing retains acquisition extent. Global-shutter negative control
  correctly rejects the erased timing. All source/config/image/library/input
  hashes and both failed builds retained. Add a stream seal so finish forbids
  post-completion input and empty captures; rerun the final revision.
- Remaining integration gates: actual rover self-occlusion; instrumented native
  Generic sensor running alongside physical IMU with full acquisition-pose audit;
  updated level-mount LiDAR/IMU/base TF audit; stationary then guarded continuous
  motion GLIM accuracy/terrain comparison; both resource profiles and concurrent
  age/watchdog gates. Settled sensor teleports cannot stand in for this evidence.
  Legacy natural capture still deliberately selects its historical 5-degree
  uniform sensor; do not launch it as if it were the selected new setup. Its
  eventual new-profile branch must preserve the old protocol and command guards.
- Final sealed-adapter run `revision-e7822661f63d48ac896866a872d8cb09` repeats all
  96 windows successfully; all input payload hashes and GLIM probe records are
  identical to the first successful input-contract run. Final six-view terrain
  replay `revision-32e1f82a8d0448548a413b34b160321e` preserves **all 1,152 maps,
  all six layers and every original region metric/denominator** exactly; new range
  diagnostics are additive. No classifier or library changes in this cycle.
- Final verification: **130 focused Python + 108 library-backed calibration tests**
  pass without skips; archived `test_layered_terrain` and
  `test_observation_moments` pass. Black and `git diff --check` pass. Task containers
  stopped. Commands: guarded `ray_terrain_yaw`, `assess_ray_terrain.py
  --observation-yaw` and `--multiview` with frozen library/`OMP_NUM_THREADS=4`,
  `check_generic_glim_input.py <native-yaw-run>`, focused pytest and
  `TERRAIN_LIBRARY=<frozen-library> pytest tools/test/test_layered_terrain_calibration.py`.
  This completes the bounded geometry/terrain assessment and first GLIM input
  integration step; moving-SLAM, live resource and independent safe-motion gates
  listed above remain unrun, not accepted.

## September 27 — quick far-structure check, then new-geometry terrain

- User approved a bounded distant-structure measurement rather than extended
  SLAM qualification; proceed to terrain assessment if useful structure remains.
  Keep **0-degree mounting**. Compare original ring pattern and mid-dense +10..-60.
- Freeze independent analytic scenes: 3x3x2 m structured boxes at 10/20/30/40 m,
  eight bearings and two horizontal angular phases (0 and half-column), one
  structure plus ground per scene. Native ray model was separately Isaac-validated;
  this screen itself is analytic/no-noise, not live sensing or moving SLAM.
- Use installed `gtsam_points::randomgrid_sampling`, exactly the GLIM code path:
  0.25 m resolution, target 20,000 / input-count rate, two workers, then strict
  0.5–100 m range filtering. The configured 0.1 rate is overridden by the positive
  target. Fixed seeds **7/42/179/411/177**, all retained. No GT labels enter sampling.
- Development go/no-go only: every tall-structure scene/seed must retain >=8
  object returns, >=4 distinct 25 cm occupied object voxels and >=1 m XY extent.
  Also report raw/post count, height span and baseline comparison. These thresholds
  only reject vanished/unusable structure; they do not certify SLAM observability.
- If passed, assess the current C++ terrain mapper using the already archived
  real native **level** baseline and mid-dense flat/eight-box cases. Preserve all
  12 native ticks, timestamps, raw returns, four grid phases and safety masks.
  This is stationary GT-isolated geometry assessment, not moving acceptance.
- Far-structure run `revision-6197671236ec47bbb2ec6446884ce46f` passes the declared
  sampling go/no-go in **640 scene/seed combinations** (64 geometries x 2 layouts
  x 5 seeds). Mid-dense retains **23–34** object returns at 40 m (baseline 46–68),
  at least 12 object voxels, all seeds. At 40 m mid-dense's height span is zero:
  one horizontal band survives, versus baseline ~0.87–0.94 m vertical span.
  Keep that limitation explicit; this is sufficient to proceed to terrain, not
  full registration observability or moving GLIM accuracy. Installed sampler and
  library/image/config/source hashes and all failures/seeds are retained.
- Started terrain replay on actual native level fixtures using unchanged current
  C++ library `8a01220c...`. First evaluator run
  `revision-2bab210fc9694e5c9a13f092baae4c7b` incorrectly counted only cost 0 as
  traversable, omitting cost 20 (inferred/rough supported). **Do not use its
  availability or false-free metrics as acceptance.** Corrected evaluator to the
  existing consumer convention `{0,20}`, added explicit class/false-free tests,
  and rerun every native tick/phase; no mapper behavior or threshold change.
- Corrected stationary assessment `revision-40571439e1fb405c96c8c6cc3b36f219`:
  **zero false-free cells in all 192 publications**; final whole-grid supported
  free ground is 8.28–9.81% mid-dense versus 0.28–2.31% baseline on flat ground.
  Both layouts detect 6/8 boxes at every grid phase. Two rear-diagonal 8 cm boxes
  remain unknown, with only ~4–5 cm low-face height observed; do not lower the
  obstacle threshold to manufacture recall. One false-occupied free-ground cell
  occurs in mid-dense boxes at phase 0.05. Dominant gaps are missing returns and
  insufficient within-cell support area, not evidence of globally wrong frames.
- Next bounded hypothesis: spatial diversity fills stationary rings and exposes
  box tops. Freeze six **sensor-only settled views**, XY offsets (0,0), (0.1,0),
  (0.2,0), (0.2,0.1), (0.1,0.1), (0,0.1) m. Same baseline/mid-dense layouts,
  flat/eight-box scenes, native rays and 6 warmup + 12 recorded ticks per view.
  This changes acquisition dose to six views, explicitly separate from stationary
  results. Keep native clock continuous within each layout/scene; never retime
  old reset epochs. Replay all 72 ticks/four phases using unchanged C++ thresholds,
  GT masks and grid. No rover, no motion safety/SLAM/runtime acceptance claim.
- First six-view launcher `revision-26afdc43e22d4b758a6e9462beb34822`, inner
  `revision-5d53db8ff84c4eec965dd24afb73d111`, **rejected / not terrain-scored**.
  All beam identities/doses pass, 23/24 cases pass geometry. Last mid-dense box
  view has one grazing beam repeated twice: PhysX returns (1.20000007,
  -0.05000488, 0.07990383), 4.88 micrometres outside the declared box side.
  Analytic geometry predicts ground instead, giving a 22.14 cm range difference.
  This is a collider-edge discrepancy, not a reason to relax the 2 mm range gate,
  drop the ray, retime data or alter the sensor route.
- Revised hypothesis: PhysX convex-cube boundary tolerance causes the grazing hit.
  Repeat the full declared six-view protocol with explicit closed triangle-mesh
  box surfaces at the **same bounds**, all other geometry/rays/dose/thresholds
  unchanged. New native protocol v2 records the collider representation. Preserve
  the first failure; strict original geometry gates must still pass before scoring.
- Triangle-box native launcher `revision-83ffa967cb8a4716a48908e628aa0538`, inner
  `revision-959b889fa1d346e7a3c720cda16beebc`: **24/24 pass**, 3,070,368 samples,
  equal dose, all beam identities, zero mask discrepancies, maximum depth/XYZ
  error 2.01/2.07 micrometres. Resolves the grazing discrepancy without changing
  any view, ray, box bound, warmup/dose or tolerance. Warmup SDK sensor-registration
  warnings remain archived; this is not a runtime benchmark.
- Six-view terrain assessment `revision-f949513c4bc1477487bfd185e23636ba`:
  **zero false-free cells across all 1,152 publications**. Final flat-grid free
  coverage: mid-dense **46.53–47.97%**, baseline **42.50–43.72%**. Fixed forward
  wheel strips (0–0.25 / 0.25–0.5 / 0.5–1 m ahead of original axle) reach
  **41.7–81.3 / 56.3–83.3 / 100%** for mid-dense on flat ground; baseline is zero
  in all three strips. These remain fixed-reference diagnostics, not swept-wheel
  safety or whole-map 95% acceptance.
- Obstacle tradeoff is real: baseline detects **8/8**, mid-dense **7/8**, at every
  phase after all views (both were 6/8 stationary). Remaining mid-dense box #3 at
  rear diagonal ~2.25 m still exposes only <=5.93 cm of its 8 cm height and stays
  unknown, never free. Mid-dense boxed far strip is 87.5–100%, not uniformly 100%.
  Maximum false-occupied free-ground cells is two in any publication; final flat
  scenes have zero. Do not claim all-obstacle detection or promote the layout.
- Assessment outcome: useful far structure survives GLIM sampling, so terrain is
  again the active bottleneck. Keep mid-dense as the near-ground candidate, but
  measure shallow-ring small-object gaps and real rover self-occlusion next;
  compare `far_dense` under a declared equal-dose protocol before choosing the
  final beam table. Validate a guarded observation maneuver only after the actual
  platform clearance/support contract and custom native adapter pass. Do not
  loosen ground-support/obstacle guards to obtain coverage. No production change.
- Final-revision regression: stationary re-evaluation
  `revision-4d07bd181c1a465da7ea2d1cc62e3fa6` exactly preserves all **192 maps,
  all six layers and scores** from corrected stationary evidence. Focused suite
  **107 passed**, no skips; Black and `git diff --check` pass. Isaac fixture
  containers have stopped. Neither actual moving GLIM error nor concurrent
  sensor/mapper compute acceptance was measured in this bounded task.
  Executed runners: `measure_ray_features.py <level-screen>`, guarded
  `run_local_isaac_fixture.py ray_terrain_views --study-run <level-screen>`,
  `OMP_NUM_THREADS=4 assess_ray_terrain.py <native-run> [--multiview] --library
  <frozen-library>`; exact usage and prerequisites are in the runbook.

## September 27 — correction: level mount for ray redesign (verified)

- User clarified the intended sensor pitch is **0 degrees**, with downward
  coverage supplied by the beam table. Previous ray-design screens incorrectly
  retained +5 degrees; they remain archived as historical tilted experiments,
  not results for the requested level setup. Withdraw their candidate ranking
  for the level-mount decision until recomputed.
- Corrected the screen to construct an explicit identity-rotation experimental
  mount while retaining the recorded calibration unchanged for provenance.
  Preserve translation, all eleven 32-ring tables, horizontal columns/ranges,
  both obstacle banks, all grid phases and regions. Run all candidates and the
  original symmetric **ring-pattern control at 0 degrees**, not a tilted control.
- Added level-frame regressions: reject inherited 5-degree pitch, roll and changed
  translation; every azimuth must have the requested base-relative elevation.
  The native fixture verifies and archives the actual USD transform per case.
  New v2 level manifests fail closed if a recorded tilted mount leaks back in.
- Corrected screen `revision-4fa87065a0014efda74d266bc46b946a`: all eleven ray
  tables and both 128-placement banks exactly match the prior experiment; only
  experimental mounting rotation changes. Level symmetric control hits **116/120**
  placements; both nonuniform mid-dense and far-dense +10..-60 hit **127/128**
  (original/shifted, each denominator 128). These remain any-ray counts, not
  detector recall. Level mid-dense first/middle/far bands are **50–100 / 88–100 /
  100%** versus **0/0/0%** level symmetric control; do not substitute these for
  whole-cell support or safe-motion availability. The historical tilted results
  are not overwritten or used as a level-mount control.
- Focused contracts **89 passed**, including six new level-frame cases. Native
  level launcher `revision-ab46b21ad95e46b6b902d8a08599c192` runs all 22 unchanged
  layout/scene cases, with host/image CUDA preflight and actual USD mount audit.
- Native inner `revision-7fe679f34b61447bbf4b22e794ead00c` passes **22/22 cases**,
  **2,814,504 samples**, equal per-case dose, all 63,968 beam identities per case,
  zero hit-mask discrepancies and **2.01/2.07 µm** maximum depth/XYZ error.
  **Every actual USD mounting rotation is exactly identity**, and every native
  flat-scene region/phase metric matches the corrected screen. Final formatter
  and whitespace checks pass; fixture container stopped. No live deployment or
  GLIM/terrain qualification claim. Current ray-design default is level, not 5°.

## September 27 — historical +5-degree ray-layout experiment

- User requested one LiDAR, unchanged ray count, and redesigned vertical spread.
  Interpret same FOV as unchanged **360-degree horizontal coverage**; explicit
  vertical-span changes are experimental. Keep the measured +5-degree mount,
  32 rings, 1999 columns, 10 Hz nominal rotation, 0.5/100 m ranges unchanged.
  Elevations below are **sensor-local, positive up**; fixed mounting adds
  azimuth-dependent body-frame inclination. No production/GLIM edits.
- Predeclare baseline plus uniform +0..-40.3 (same span control), +0..-45,
  +10..-45, +0..-60, +10..-60, +0..-75, +10..-75; two nonuniform +10..-60
  designs (mid-range-dense and steep-dense). Every design has exactly 32 unique
  elevations. Preserve all old strip/sector/grid-phase denominators and the
  eight 10x10x8 cm boxes. Supplement with individually scored 8 cm boxes at
  eight ranges 0.75/1/1.25/1.5/2/2.5/3/4 m and sixteen azimuths (128 cases),
  not just the favorable eight-object scene. No selection based on held-out data.
- Compare stationary geometry and the same 25-view ideal yaw diagnostic. Report
  below-minimum-range ground rays, usable returns, all sector/strip coverage,
  per-range hazard misses and high/near-horizontal ray allocation. Hit counts
  are not classifier recall, whole-cell support or GLIM accuracy.
- Validate ray arithmetic with host regressions and an isolated Isaac PhysX
  custom-ray fixture. The legacy symmetric/uniform LiDAR settings alone cannot
  express arbitrary asymmetric/nonuniform patterns; preserve this distinction
  rather than claiming a pitch adjustment implements the requested ring layout.
- Initial screen `revision-23f8bae3cdc844dd95ca9d6a6e1cad3d` completes all ten
  layouts at **63,968 rays/revolution**. Baseline hits 116/128 independent box
  placements; same-span downward control 126, 0..-45 124, +10..-45 125,
  mid-dense +10..-60 123. These are any-ray sampling counts, not detector recall.
  At 4 m the baseline hits 16/16, while those alternatives hit 14/12/14/12:
  near-field gains must not conceal reduced distant small-object sampling.
- Steep 75-degree layouts waste **27.4–30.9%** of the unchanged ray budget below
  the 0.5 m minimum range. The 45-degree layouts waste none in the ideal flat
  fixture. More useful rays can still increase mapper load: +10..-45 supplies
  **46,664** ground returns within 5 m versus **24,998** baseline. Equal ray count
  is not a runtime pass. No classifier, SLAM, runtime or moving acceptance claimed.
- Focused host contracts **80 passed**. Native explicit-ray attempt launched as
  `revision-fbd08efc890047d79ac0efb61a849622`; host/image CUDA and archive preflight
  pass. Uses the installed SDK Generic range sensor with explicit 32-ring tables;
  fixed nominal 639,680 samples/s, twelve recorded 60 Hz ticks after six warmup
  ticks per layout/scene. Must verify returned native angles and equal actual
  dose; no association by fitted geometry or substitution of invented timestamps.
- Explicit-ray attempt failed the **angle identity gate** at the first layout
  switch, after both baseline scenes passed. Inner run
  `revision-043dc315303b414bb0c3706590634eb6` is retained, not accepted. Hypothesis:
  Generic sensor queued ray tables survive `world.reset()`. Next fixture creates
  a fresh uniquely named sensor per case (only one active), rather than increasing
  warmup or discarding mismatched data. Save raw samples before validation and
  verify every column/ring identity is represented. No angular tolerance changes.
- Mechanism-directed second screen, declared before execution: distant misses
  suggest too few shallow-down rings, not a need for still steeper endpoints.
  Add **far_dense_10_to_-60**: 3 rays at +10/+5/0, 18 from -2..-20,
  6 from -22..-37, 5 from -41..-60 (32 total). Keep all ten earlier layouts.
  Score all eleven additionally on a fixed shifted diagnostic bank: each original
  radius +0.10 m and bearing +11.25 degrees (128 more placements). This is a
  development robustness check, not sealed held-out or moving qualification.
- Final screen `revision-b478574938d84961be08040ce8e8e5a8` preserves every metric
  of all ten original layouts, adds the declared eleventh and shifted bank.
  `far_dense_10_to_-60` yields **126/128 original, 128/128 shifted** box placements
  with rays (baseline **116/118**). First/middle/far axle-band hit-cell coverage
  becomes **50–88% / 88–100% / 100%**, versus **0 / 0 / 25–33%** baseline.
  It still misses two original 4 m placements that baseline hits. This is NOT
  detection/terrain recall, an actual yaw maneuver or safe-motion availability.
- Corrected explicit-ray launcher `revision-1fdd058181984e99b548e8db4853d308`,
  inner `revision-cae5f66863554d13babf917efaf32529`, passes **22/22 cases**,
  **264 chunks / 2,814,504 native samples**, with equal **127,932 samples/case**
  and all **63,968 unique beam identities** observed in every case. Zero <=5 m
  expected/measured hit-mask discrepancies; max depth/XYZ residuals **1.81/2.48 µm**
  against unchanged 2 mm gates. All stationary phase/region metrics exactly match
  the screen. Fresh sensor identity fixes the rejected stale-queue attempt.
  SDK `need more data` warnings remain archived; measured gates pass, but this
  is not runtime acceptance or proof that a live adapter can ignore queue state.
- Final focused suite **83 passed, no skips**; formatter and whitespace checks
  pass. All isolated containers stopped. Leading development candidate is
  nonuniform +10..-60, with uniform +10..-45 as simpler comparator. No production
  changes. [ray-design-study.md](ray-design-study.md) contains all configurations,
  counts, misses, reproduction commands and integration gates. Next: actual-rover
  optical coverage and opt-in per-ray native adapter before moving GLIM/terrain
  and both resource/concurrent-load gates. Retain the trusted-startup requirement;
  no sensor pattern turns unobserved ground into free space.

## September 27 — bounded mounting / startup visibility study

- Approved scope: compare practical mounting changes before choosing a guarded
  startup observation policy. No production asset, mapper, safety threshold or
  evidence lifetime changes; no robot motion authorized by this diagnostic.
- Hypothesis: downward pitch reduces near-field blindness but trades away other
  directions, and off-center yaw need not supply all wheel-track ground. Earlier
  full-map tilt experiments exist; they do not answer the new short-motion query.
- Predeclared screen: absolute sensor pitches **5 (current), 10, 15, 20, 30 deg**,
  unchanged 0.4 m forward / 0.25 m high base offset, recorded native elevation
  angles/range limits, measured flat loaded height. Compare stationary versus
  virtual full-circle yaw at 15-degree increments (25 views, not a drivable
  protocol). Use fixed 10 cm grid phases 0/2.5/5/7.5 cm, wheel-track ground bands
  0–0.25 / 0.25–0.5 / 0.5–1 m ahead of the front axle and four 0.75–3 m sectors.
  Count every declared cell; a hit is not full-cell support or terrain truth.
- Validation order: analytic geometry/unit fixtures; archived discrete-ring
  screen; isolated native Isaac flat-plane and 8 cm-high box sampling checks.
  The isolated sensor check has no rover/SLAM/commands and cannot certify physical
  mounting clearance or actual self-occlusion. No full-circle maneuver is executed.
- Host preflight: external `/dev/sda1` mount present, NVML and actual CUDA allocation
  pass; no running mission/Isaac container. Image preflight remains mandatory.
- First analytic fixture disproved a test assumption (26 passed, 1 failed): even
  +30-degree forward/down tilt cannot expose initial wheel-center ground during
  pure yaw. The wheel radius (~0.383 m) is smaller than the sensor offset (0.4 m),
  so those targets stay behind the sensor in body coordinates. Corrected the
  test expectation and, **before any screen/simulator results**, extended the
  declared candidate set with **-10/-20/-30 degrees** (rearward/down) to test the
  opposing tradeoff. The original positive-pitch candidates remain counted.
- Screen `revision-5d5fcf467be04a45a92da50174e4e340` completes all 16
  pitch/acquisition combinations and four fixed grid phases. **28 analytic tests
  pass**; combined observability/launcher/body/sweep tests **65 pass**. Host Black
  is absent (not a test failure); a cached formatter is being located.
- Ideal results: every stationary pitch has **0%** hit-cell coverage in the first
  0–0.25 m front-wheel band. +30 degrees raises the 0.5–1 m band from baseline
  **25–33% to 100%**, but rear-sector ground falls to **0%** and three of eight
  stationary 8 cm boxes have no rays. -10 degrees plus 25 virtual yaw views hits
  **100% of all three band cell sets**, versus baseline **0%, 0%, 54–67%**;
  however stationary -10 degrees misses the directly-forward 1.25 m box and all
  three front bands. These are no-occlusion hit counts, NOT support/recall/safety.
  Therefore no tested fixed pitch is an unqualified replacement for the baseline.
- Isolated native fixture launched as `revision-5577e47557b9444b8a22b52780d2eed4`
  (inner `revision-ebc68268e9b44dbb8341e25937fd9bbb`), with archive and host/image
  CUDA preflight, a 240 s wall deadline, actual 10 Hz native chunks and no rover.
  Each pitch uses separate flat and fixed-box scenes, six warmup/six recorded
  ticks; 2 mm maximum ray-depth/XYZ error and complete azimuth are geometry gates.
  Stationary native acquisition dose differs from the ideal 25-view screen and
  is reported separately. It does not validate virtual yaw or self-occlusion.
- First native attempt stopped on a **diagnostic serialization bug**: NumPy's
  boolean azimuth-completeness result is not JSON serializable. Preserved all six
  baseline flat chunks and the partial report; launcher correctly rejected Isaac's
  zero exit status. Those chunks show zero hit-mask discrepancies and micrometre
  ray residuals, but the incomplete run is **not accepted**. Converted the result
  to a Python boolean through a tested pure helper; no geometry tolerance changed.
- Final source screen `revision-0ac13d780e334b889880f89fdae0d5bd` exactly reproduces
  every first-screen metric. Corrected native launcher
  `revision-1131448c4ad94eb2a0427045389d90ca`, inner
  `revision-b1f689b1c49c4510954a51f110e60a38`, passes **16/16 geometry cases**:
  **96 chunks / 2,045,952 rays**, zero expected/measured hit-mask discrepancies,
  maximum depth/XYZ errors **5.28/5.44 micrometres**, below unchanged 2 mm gates.
  All stationary phase/region metrics match the analytic screen exactly; hazard
  counts match at the declared native two-revolution dose. This validates the
  ray/sampling model, not classifier detection, self-occlusion or robot motion.
- Final focused suite: **71 passed, no skips**; five-file cached Black 24.10.0
  check and `git diff --check` pass. Fixture container is stopped. No production
  changes or navigation promotion. See [mounting-study.md](mounting-study.md) for
  the comparison table and bounded next experiment. Important interpretation:
  the first 25 cm strip ahead of the front axle is mostly beneath the current
  chassis, so its zero direct returns alone cannot prove an unsafe startup.
  Actual optical self-occlusion and an independently established startup-support
  contract are the next prerequisites; no blind spin/jitter was implemented.

## September 26 — exact integration optimization and active collision reference

- Follow-up hypothesis: point-query free-cell availability can conceal unusable
  wheel tracks. Added opt-in `support_sweep.py`: full linear/SLERP motion is
  conservatively enclosed, every intersected cell is checked, boundary/outside
  cells remain counted, and work is bounded. **17 host fixtures pass**, covering
  interior obstacles/holes, rotations/tilt/off-center patches and grid boundaries.
  This reports **class availability only**, not independently safe motion.
- Predeclared diagnostic: same 320-publication failed source, 60x60 grid, causal
  coherent poses and final hybrid library; compare 5 vs 30 seconds. Four assumed
  **80x120 mm** ground patches use measured loaded wheel XY and flat base height;
  these are NOT measured tire contacts. Queries at every publication: stationary,
  forward 0.10 m, and yaw ±0.10 rad. Preserve startup/all queries and failed-source
  status. No classifier/lifetime change to existing runs; this is a new explicit
  retention comparison. Observer instrumentation disqualifies timing acceptance.
- Metadata audit: current age is the minimum age among local layers and does not
  carry all neighboring plane/corner-support dependencies; plane sigma is not a
  relative-pose error bound. Neither can certify long-lived support freshness or
  uncertainty. The diagnostic explicitly withholds both certificates. Next support
  policy needs separate immutable provenance/expiry, not a fresh-looking old cell.
- Final pre-observer broad regression (including corrected Cube inspector):
  **653 tests passed, no skips**. New sweep/comparison fixtures: **42 passed**.
- Swept-support diagnostics finished: 5 s `revision-6f23b19ca558443ead6028cf952c2733`
  has **0/320 supported** in each of four query families; every queried patch
  cell is `NO_RETURNS`, not an obstacle or outside the map. All 320 layer hashes,
  poses, associations, origins, timestamps and cell-availability records exactly
  match the earlier final-hybrid replay. Same physical path covers about 2.19 m.
  Thirty seconds `revision-d71ec7c4e7fd45c5bec85a5116aaa4e0` supports
  **149/320 stationary, 149/320 forward, 146/320 left, 148/320 right**. Startup
  remains counted. Late cycles exceed 1 second; no timing or terrain-truth pass.
- Next explicitly different algorithm experiment: `fixed_incremental`, frozen
  first-used **local** observation placements, fixed initial 20x20 m grid and
  persistent immutable 30 s C++ buckets. It integrates each native input once;
  current rover pose still uses the identical causal/latest revision path.
  This changes placement, extent/grid motion and bucket-expiry behavior relative
  to coherent reconstruction, so it is **not a one-factor or exact-equivalence
  claim**. Fixed boundaries stay unknown, old poses are not retroactively repaired,
  and no uncertainty/freshness certificate is implied. No default changes.
  Host fixtures exercise single integration, fixed origin, old-pose updates,
  interruption/epoch/clock invalidation and exact direct-stream expiry behavior.
- Corrected static USD inspection archived at
  `revision-0268c63257694667965551f1560100b7/vehicle.json`. Preceding attempt
  `revision-6dcbe997f3af46d1be3eb68e3d070486` failed because mounting the script
  as `/inspect.py` shadowed Python's standard-library `inspect`; stderr/output
  preserved. Renamed the container script only; no asset or geometry change.
- Fixed incremental support replay `revision-d0e50e4dc2624aeca97e4664d42c7926`
  retains **149/149/146/148 supported queries out of 320**, identical counts to
  the 30 s coherent control (not identical maps). Late instrumented cycles are
  roughly 60–85 ms instead of >1 s. This still does not establish the 50 ms gate.
- Next declared candidate: opt-in `TERRAIN_ROLLING_GRID` lattice translation,
  6x6 m persistent frozen-placement map. Move retained cells in world coordinates,
  flush pending native votes before changing indices, preserve immutable bucket
  stamps, and initialize entering cells unknown. No historical backfill or pose
  reintegration. Eviction/re-entry and expiry require explicit regressions. Build
  `revision-9eadef1010bf433583ce71a351502ab4` (`8a01220c...`) passes both C++
  programs. First Python shift test caught its own `3*.1` versus `.3` projection
  timestamp regression; changed the test query to `.4`, preserving the production
  monotonic-time check. This is an algorithm candidate, not exact reconstruction.
- Rolling incremental support replay `revision-88e9ee60da6e4eaf9dd306057c2d7e0b`
  retains the fixed-grid arm's **149/149/146/148** supported query counts and
  identical per-family free-patch cell totals in this prefix. Instrumented cycles
  are roughly **15–24 ms**, versus fixed-grid 60–85 ms; an uninstrumented complete
  failed-source stream follows before timing claims. New entering cells remain
  unknown and do not receive old raw-window backfill. **696 host regressions
  passed, no skips** with the rolling library (including direct-stream expiry,
  pending-vote relocation, no resurrection, old native exact-differential tests).
  Ten rolling-specific tests also pass at OMP=4. Body/surface truth, startup,
  uncertainty, live load and held-out gates still prevent promotion.
- Complete **uninstrumented** rolling-incremental replay
  `revision-c74cad5028b04f8faac93bcbe6e40f18`: all **748** publications,
  **25.03/34.88 ms p95/p99**, median **19.97 ms**, peak RSS **419.61 MiB**.
  This passes the offline cycle/memory thresholds on this recording. It is NOT
  a live or completed-mission pass: the original `simulation wall heartbeat lost`
  terminal failure is preserved, and final output is invalidated/closed.
  Both resource profiles and serial support are being checked separately.
  First host resource invocation omitted its required `--archive-root` and stopped
  before allocation because `/archive` is container-only; rerun uses the verified
  external host mount, never a workspace fallback.
- Rolling build's isolated resource gates pass: dense-flat
  `revision-6049103a0e7f48aca7ece7aab521e3fc`, **39.45/52.67 ms**, **327.77 MiB**;
  capacity `revision-7ce4fe53c4ab4c198cb9c1584dd01d2a`, **44.67/48.22 ms**,
  **1010.72 MiB**. Serial build `revision-fa9362ee3a024ef18f16bfd6cc0f9fb1`
  (`0979cf9f...`) passes both C++ programs and **208 local/rolling/incremental/
  exact-differential tests**. Two additional native tests cross the 2048-input and
  64 MiB deferred-work bounds and match the frozen eager reference exactly.
- Complete support diagnostic `revision-cbd1fad8609946a58dfe607a45b4837e` exposes
  a later failure, not just startup: **217/217/211/214 supported out of 748**
  (29.01/29.01/28.21/28.61%). Stationary support appears at stamp 39.3 s, is lost
  at 58.1 s during the slow turn, and returns at 94.1 s on the next straight leg.
  Failures are mostly `NO_RETURNS` with some `UNSUPPORTED_EXTENT`; no queries are
  outside the grid. All startup/turn/failure exposure remains counted. This is
  class availability, not terrain-truth safety. No 95% claim is justified.
- Next bounded retention experiment, declared before replay: separate the raw
  reconstruction buffer (5 s) from persistent immutable evidence (30 s), requiring
  identical layers/current poses to the previous 30/30 s stream. Then change ONLY
  evidence lifetime to **60 s**, retaining the same 5 s raw buffer and all queries.
  This explicitly changes the terrain-evidence contract and retains both hazards
  and support, not just free cells. It is an opt-in static-scene retention diagnostic,
  not validated long-term uncertainty, separate hazard/support policy or promotion.
  Startup sensing still requires a separately declared intervention; user preference
  requested while independent software validation continues.
- Raw/evidence decoupling succeeds: complete uninstrumented 5 s raw / 30 s
  persistent replay `revision-439d7949d1164c05ad2b8c91b3f48244` matches all **748**
  layer hashes, rover/map transforms, origins, timestamps, query counts and newly
  integrated observation placements of the preceding 30/30 s stream. Peak RSS
  falls **419.61→169.54 MiB**; p95/p99 **21.26/23.15 ms**. This comparison does not
  require old raw-association lists to remain identical: only their storage window
  changed, not the persistent map's input stream or evidence lifetime.
- Explicit 60 s evidence / 5 s raw support diagnostic
  `revision-e4d22ca6396b4491b950938f83a7240c`: **577/577/574/576 of 748** queries
  supported (**77.14/77.14/76.74/77.01%**). The later turn-support loss disappears;
  startup unknowns remain counted. This supports the expiry hypothesis, not a
  safety certificate or the 95% gate. No GT labels entered the candidate and no
  frozen baseline lifetime changed. **704 broad regressions passed, no skips**.
- Before treating retained old geometry as trustworthy, added a GT-only audit of
  frozen native-LiDAR placements relative to the **current physical base**, with
  exact native-tick correspondences and the audited mount. It bins registration
  errors by observation age, preserves missing-GT counts, and performs no alignment,
  fitted correction or extrapolation. Seven analytic tests pass, including separate
  world gauges and a sensor/base lever-arm mismatch. It audits retained-window pose
  opportunities, not actual surviving cell dependencies or calibrated uncertainty.
- Initial GT audit `revision-9728147c326445b1887acf25ea941366` found **zero exact
  consumer-time GT matches**: nominal 100 ms mapper query stamps differ from native
  physics stamps by accumulated simulator clock rounding. No tolerance expansion
  or fitted retimestamping was used. A separately declared exact-native endpoint
  audit uses the latest assimilated native base at that publication (up to **33.33 ms
  before the map query**); consumer-time error remains unevaluated. Fourteen tests
  cover gauge/extrinsics, provenance, missing GT, causality and query/native offset.
- **Frame defect discovered; earlier incremental variants are rejected.** Exact
  native audit `revision-20a6e8b8cf0c436dac2416e7ad11c469` covers all 748 outputs /
  **806,700** retained-window pose pairs with no missing GT. Frozen-local placement
  error grows to **36.77 cm p95** at ages 30–60 s. Attribution-only
  `revision-c9acf5ed04294e2eb9ef11c19fda2d04` uses the SAME first map poses transported
  through the current gauge: **7.93 cm p95** on exactly the same pairs. It is not
  a GT correction or implemented terrain repair. The persistent map was freezing
  local evidence while the recent-anchor policy continually moved the robot's
  reference. Its favorable coverage/runtime cannot establish geometric validity.
- Fix: incremental candidates now keep `local_from_map` fixed and retain the
  oldest session frame as a reference witness. A witness change beyond **1e-6
  matrix-entry numeric tolerance** invalidates/releases the map; the comparison
  is always against the INITIAL witness, so small changes cannot ratchet into a
  gauge shift. Unsupported global events still invalidate. Current/recent GLIM
  pose updates do not move the reference. Coherent-rebuild default is unchanged.
  New fixtures reproduce recent-anchor frame walking and witness-change failure;
  fresh full replay/GT/coverage/runtime checks follow. Old artifacts stay retained.
- Fixed-reference 60 s / 5 s raw support replay
  `revision-3921fc26dd4647bba19c6e5c50050774` completes all 748 recorded outputs:
  **544/544/538/537 supported** (72.73/72.73/71.93/71.79%). These replace neither
  old counts nor their failed frame status; fixing geometry reduces apparent
  usefulness. Startup is 173 stationary queries; another 31 stationary queries
  fail extent support at stamps 88.1–90.6 and 91.1–91.5 s. No obstacle/outside-grid
  rejections. Underlying `map_from_base` differs from the preceding run by at most
  **2.14e-14 per matrix entry**: the same estimator trajectory and motion exposure
  were used, without GT corrections. Local lattice phase changes with the frame fix.
- Actual fixed-reference native GT audit
  `revision-9cd2820cfef34cbdbe8fb01673c0a212` covers the same **806,700** pairs and
  zero missing GT publications. Ages 30–60 s now measure **7.93 cm p95 / 13.35 cm
  maximum** sensor-origin displacement, confirming the attribution prediction.
  At 3 m return range, translation+rotation bound is **9.50 cm p95 / 15.10 cm max**.
  These are measured diagnostics, NOT uncertainty guarantees or current-consumer
  pose error. Fifteen audit fixtures pass; no fitted alignment/retimestamping.
- Final fixed-reference **uninstrumented** 60 s evidence / 5 s raw replay
  `revision-bffd07694b0040c588ab8ec32e6cdf17`: **748** publications,
  **19.74/22.70 ms p95/p99**, median **18.12 ms**, **201.15 MiB** peak RSS.
  Same original heartbeat failure and final invalidation are preserved. This is
  the current runtime checkpoint, not the earlier flawed-reference timings and
  not live/full qualification. Standard core resource profiles remain at their
  frozen 30 s workload; no unrun 60 s saturation/resource pass is asserted.
- Final revision validation: **721 host/regression tests, no skips**, using the
  rolling OpenMP library, frozen eager reference, and all local support/GT audit
  fixtures. **230 serial local/rolling/incremental/native/audit tests** pass with
  `revision-fa9362ee3a024ef18f16bfd6cc0f9fb1`. Both C++ programs pass in serial and
  OpenMP builds. Fresh ordinary build `revision-85370e6737de4a04854fae90d7f7b7fa`
  has SHA `d91c139b...`, **byte-identical to the pre-optimization ordinary baseline**.
  No default terrain policy, production launch, asset, GT labels, safety threshold
  or baseline artifact was changed. `git diff --check` passes.
- Remaining decision/gates: choose a separately declared near-field sensing study
  or guarded startup observation protocol; preserve startup exposure and never
  seed unseen terrain free. Independently validate contact/terrain truth and
  stopping limits; add dependency-aware support age and relative-pose uncertainty;
  address the remaining extent failures; then test the fixed-reference candidate
  under concurrent live load and a frozen held-out batch. The original failed
  source, rejected moving-anchor variants and all failed fixtures remain archived.
  **Full qualification is not achieved and no candidate is promoted.**

- Hypothesis: repeated eigensolves of the identical per-observation support patch
  are avoidable without changing geometry. Added an **opt-in**
  `TERRAIN_PATCH_NORMAL_CACHE` path; ordinary builds retain the prior path.
  The cache lives only within one integration call, not across pose revisions.
  `tools/build_local_terrain.py` freezes sources, compiler commands and hashes in
  a fresh archive run. Replay accepts a validated candidate library only for the
  experimental local policy; frozen original replay cannot substitute a library.
- Baseline build `revision-f863cf06dd3a4a67a652e94c3dcda504`; optimized OpenMP
  build `revision-d905d580d1a1459089703f428917efd9` (SHA `480452ef...`);
  serial build `revision-27f6f199a06c4000b29fa7b0c732a85d` (`f002e139...`).
  Observation-moment and layered-terrain C++ tests pass; layered tests pass with
  both one/four OpenMP workers and without OpenMP. Both resource profiles are
  being checked separately; this is not yet compute acceptance.
- Same 5 s / 320-projection prepared replay:
  `revision-dafd6f2bd13c45968c65a6d232619039`, p95/p99 **202.83/227.25 ms**,
  peak RSS **144.38 MiB**, versus prior **222.31/249.75 ms**.
  Exact comparison `revision-14be5ca2e7734a65a96da31ea6d95866` matches every
  layer hash, association, rover transform, origin and availability count.
  **Still a runtime failure**, on the same failed recording prefix. Tests and
  inspection overlapped portions of this replay; no live/uncontended claim.
- Independent truth hypothesis: active scene queries avoid mistaking authored
  terrain for the cooked collider. Added evaluator-only conservative rotating-box
  envelopes, initial overlaps and continuous translation sweeps. Off-center body
  lever arms, exact self-root filtering, query budgets, callback failure and stable
  scene epoch/tick are checked. Clear means **body envelope clear only**, never
  wheel support or a feasible motion. Twelve analytic adapter tests pass.
- GPU preflight passed on host and in `isaac-sim-omnilrs:latest`, including actual
  CUDA context/allocation. Isolated Isaac fixture (no ROS, rover or commands)
  passed **6/6 active PhysX cases**: clear, thin obstacle between endpoints,
  initial overlap, floor retained, self excluded, other robot retained.
  Inner results `revision-e5beaf3d35b24c25b82b6bba8b9ccf89`; launch/log/exit
  `revision-acfb65eb11884e5892369d5106b78953`. Container exited normally (0).
  This validates the adapter, not natural terrain, loaded contact or braking.
- Remaining gates unchanged: exact rebuild still exceeds runtime; independent
  continuous wheel support, loaded operating limits, vehicle feasibility,
  retention/connected ground, live concurrent load and held-out motion scores
  remain open. No candidate promoted and no historical failure discarded.
- Isolated normal-cache core resource gates passed, OMP=4 with Isaac stopped:
  capacity `revision-c4aa7f1cb205464386a6b8bbdbfb62a6`, **35.76/49.49 ms**,
  **1010.81 MiB** (close to 1024 MiB gate); dense-flat
  `revision-e2051de307114e7e836d4a46da1dc99d`, **47.12/56.16 ms**, **328.02 MiB**.
  These do not qualify the rolling reconstruction wrapper.
- Next experiment, declared before replay: sparse native patches contain few
  active layers but visit all 324 slots per center. An opt-in sorted active-layer
  index skips stale/empty slots while retaining the exact dy/dx/layer merge order.
  It never caches across revisions. Compare 320 projections against the preceding
  normal-cache run, then both resource profiles and the serial path if exact.
- Sparse-neighbor build `revision-32c2361aedc14b8e81ee913894951123`
  (`c875440b...`) replay `revision-d6ceb5f7f702411dab7f646076922f1e`:
  **179.43/201.94 ms p95/p99**, **139.01 MiB**. Exact comparison
  `revision-ac6530ed4f434ccabe7355c641db705f` verifies all 320 outputs against
  the preceding normal-cache version. The 50/100 ms local gate still fails.
  Both isolated core resource profiles pass: dense-flat
  `revision-6ade0d1843fd45adbf8a8d9520208dbc` **32.46/39.55 ms**, **327.78 MiB**;
  capacity `revision-7a8b7a7ee05f4ab09ccf5e0dc4e96698`
  **46.19/54.09 ms**, **1010.52 MiB**. Capacity timing is slower than the earlier
  normal-cache-only run; retain both results, not just the favorable dense result.
- Added exact differential native-patch tests across 1/7/60/80-cell grids, three
  phases, sparse multilayer/overflow observations and expiry. Complete focused/
  regression command below, adding build, PhysX and native-patch tests with the
  candidate and dense reference libraries, passes **591 tests, no skips**.
  Sparse serial build `revision-4eeec13c6afa49f78acc4373a0f3d30c`
  (`79ea0cd5...`) passes both C++ programs. No geometry thresholds changed.
- Cycle 1 next: isolated actual-asset **zero-command settling** fixture on a
  known floor. Explicit lunar gravity, 60 Hz physics, disabled command/sensor
  graphs, zero wheel targets, 5 cm XY drift guard, 480-step / 60 s run bound.
  Keeps all startup rows, examines predeclared final second for settled base
  speed and four upward wheel contacts. This is not a brake test, contact-patch
  extent certificate, natural-terrain truth or mission qualification. Host and
  image CUDA context/allocation preflight passed before its launch.
- Loaded-contact first attempt **failed before physics**, preserved at
  `revision-cc85d1ffd6fe4d72b7958050af06f05a`, launch
  `revision-847819cb7bb6470ab932fe2600ff78dd`: deactivating an OmniGraph parent
  invalidated already-collected descendant prim handles. Fix: finish physical
  prim inspection before deactivating graph/sensor subtrees. Also observed that
  Isaac shutdown can return process exit 0 after an exception: fixture success
  requires the inner report and no failure marker, never the exit code alone.
  No contact/settling result is claimed from this failed attempt.
- Final sparse-path compatibility checks: **195 tests pass at OMP=4** (local,
  exact native patches, build/comparison and PhysX contracts); **151 local/native
  patch tests pass with the serial library**, plus both serial C++ programs.
  Launcher contracts add **8 passing tests**, explicitly including exception
  markers with exit 0, missing results, duplicate runs and archive path escape.
  Python formatter is available through cached
  `uv tool run --offline --from black==24.10.0 black`; host `python3 -m black`
  is unavailable. No dependency installation or host-library replacement needed.
- Corrected contact run `revision-5bc16e8bfee6404a8275a3de0d91aa8e` (launcher
  `revision-8cd1a3d000804b749f6096a937570326`) completed all 480 steps but
  **failed the declared steady-force gate**: forces existed at steps 32–48 and
  were zero thereafter; base settled at **0.144923 m**, wheels at **0.177743 m**,
  with zero final velocity. Do not turn those zero forces into “no support” or
  silently pick earlier nonzero samples to pass. Installed Isaac source confirms
  sleeping bodies do not issue contact reports. RigidPrim contact instrumentation
  also silently sets body sleep thresholds and disables stabilization by default.
- Next declared characterization is separate: preserve stabilization explicitly,
  record sleeping states, and **explicitly wake the base each tick** (`contact_awake`)
  to measure settled loads. It is not default mission physics, runtime acceptance
  or a cure for motion uncertainty; the unmodified-force failure remains counted.
  Missing remote visual MDL materials produced retained warnings but did not
  prevent the 480 physics steps. No collider/material substitution was made.
- Awake characterization passed all 480 steps:
  `revision-3ba6642d9fa64823b47392e8dc757cad` (launch
  `revision-3cc232ea9c0341d5b24a10ed59127c14`). Final-second wheel loads
  **14.02/12.93/20.66/23.83 N**, base height **0.1449231 m**, maximum base speed
  **0.000284 m/s**, all five queried bodies awake. This supports the missing-report
  explanation and agrees with the earlier settled transforms; it does **not**
  qualify normal sleeping-physics contact reporting, support extent or braking.
- Fresh ordinary (no optimization flags) build
  `revision-f504c0d5221546c7b1929d666ba6f986` is byte-for-byte identical to the
  pre-change host library (`d91c139b...`). Both optimizations remain opt-in.
- Expanded active-body fixture adds a non-cubic 90-degree orientation check and
  an interior rotation collision; axis-aligned cubes alone could not detect a
  quaternion-order mistake. Running all nine cases on the new launcher, with
  local image content ID pinned and frozen outer launcher-source hashes.
- Expanded body fixture **9/9 passed** at
  `revision-bfca04519f9f40b6bd1bad79be4bbf6a`, verified launcher
  `revision-e1fa6c2042d14ac98057841f2e53cf59`. No fixture container remains running.
- Next reconstruction mechanism (declared before replay): native scans are too
  sparse individually to use the existing intra-scan OpenMP threshold. Preserve
  ordered point-to-layer integration, snapshot each input's exact native moments,
  and classify these independent inputs across at most four workers before
  projection. Commit only byte-sized native votes to their **original buckets**;
  flush before expiry, bound snapshots to 64 MiB / 2048 inputs, and retain serial
  support. This is opt-in `--defer-native`; no fuzzy transforms or shifted poses.
  The ordinary path remains unchanged. Required tests include multi-input batches,
  expiry, exact replay, both resource profiles and serial/OpenMP equivalence.
- Deferred build `revision-55c0c5c135084fe9a9b16b4536a18ed9` (`f8fdec30...`)
  passes both C++ programs and **163 local/differential tests at OMP=4**.
  Its 5 s / 320-frame replay `revision-fd68ef4c9810456094fbf47446a5ed23`
  measures **153.66/170.48 ms p95/p99**, median **131.57 ms**, **145.18 MiB**.
  This is another improvement, not a 50/100 ms pass. Full equivalence/resource
  and serial checks follow; all preceding attempts remain part of the record.
- Exact deferred comparison `revision-a6e9f3fcdb60419082c8b25a737271be` passes
  all 320 outputs. However, its dense-flat resource run
  `revision-0466777a605f407a99afb03cd1ff5af4` **fails p95 at 50.99 ms**
  (p99 57.54 ms, 329.26 MiB). Do not promote or replace this attempt with a retry.
  Mechanism-specific revision: large scans already meet the existing 1024-layer
  intra-scan worker threshold, so use their original eager path. Only small
  native inputs need cross-input classification. Expiry/order/bucket guards stay
  unchanged. Expanded exact tests include both sparse and dense input doses.
- A separate exact preparation change reuses a SLERP interpolator for repeated
  acquisitions in the **same adjacent pose pair and publication**. Pose histories,
  fractions, gap checks and extrapolation rejection remain unchanged; no cache
  survives a publication revision. A dedicated fixture checks pair/revision
  invalidation. It will be measured separately from the C++ scheduling change.
- Unconditional-deferred capacity profile also **fails**:
  `revision-1c40282e28b64d79b204398ff400dbf3`, **56.77/67.44 ms**,
  **1013.48 MiB**. This reinforces keeping the direct path for large inputs.
  The per-publication SLERP fixture passes all **10** pose-lookup tests; its
  independent replay uses this same frozen C++ build before the eager-large
  replacement is measured, avoiding attribution to two simultaneous changes.
- Isolated SLERP reuse replay `revision-696b6e90f9bd4535b71bdd2cd926dece`
  remains exact in all 320 records (`revision-440e2952de1c4af7850f919b7f04dd49`).
  Median 131.57→127.96 ms, p95 **153.15 ms**, p99 **172.90 ms**, RSS **141.08 MiB**.
  The p95 difference is negligible and p99 worsens: no robust speedup claim.
  Retained bounded reuse reduces repeated interpolation setup without altering
  poses; it is not a route to qualification by itself. Eager-large/deferred-small
  OpenMP and serial builds are next, followed by exact/resource regression gates.
- Eager-large/deferred-small builds: OpenMP
  `revision-cc0680eb868143b68c40b8d9ff2ec0fc` (`b0cf0a35...`), serial
  `revision-bf6668b86867407aa30aa3ffcda9e8ac` (`49e7d821...`). Both C++ programs
  pass in both builds; **187** sparse/dense/batched local tests pass at OMP=4.
  Replay `revision-d2e55ff7b87f4510a40c80cd2eef05a0` remains exact across 320
  outputs (`revision-4f8fb8879d8649cc9f59a6dba1bcfd35`): **146.21/163.27 ms**,
  **143.89 MiB**. Still fails the local runtime gate.
- Both isolated resource gates are restored on the changed schedule: dense-flat
  `revision-adacf4558b02446585ccc8af7e5cb569`, **32.62/44.55 ms**, **327.76 MiB**;
  capacity `revision-b86b4154742d4ac1a86758d669ecccf8`, **39.29/48.54 ms**,
  **1010.78 MiB**. The unconditional-deferred failures remain archived.
- **New fundamental retention constraint:** `diagnose_local_observability.py`
  uses the recorded mount/native zenith angles and independently measured flat
  loaded geometry. On level ground at 0.10 m/s, even an optimistic continuous
  angular cone (no discrete-ring gaps or occlusion) needs **9.47 s** of prior
  forward travel to observe current front-wheel ground and **14.59 s** for the
  rear wheels. Artifact `revision-1bc5985a802e4aebbb0b46c1935e35b8`; **11 analytic
  tests pass**, including closed-form tilted/offset cones and range restrictions.
  This is a **counterfactual necessary direct-visibility bound**, not measured
  natural-terrain recall or actual beam coverage. All failed-source status stays.
  Nevertheless 2/5 s direct-evidence windows cannot supply those wheel-track
  observations in this benign case; more classifier tuning cannot fix that.
  Thirty seconds only passes this necessary bound, not safety or performance.
  Prioritize separately retained support with explicit uncertainty/expiry and
  efficient updates, or a separately declared sensing change. Do not silently
  extend the frozen lifetime or bridge unobserved support with free labels.
- **Vehicle-envelope audit found an unsafe metadata assumption.** The chassis
  collision prim is a Cube of size 1 with scale [1.0074, 0.5709, 0.2675], but its
  authored `extent` is already scaled. USD BBoxCache trusts that cached extent;
  applying the prim transform again yielded the earlier y +/-0.16296 and
  z [0.08920,0.16076] figures. USD's geometry-plugin extent is correctly +/-0.5.
  Primitive parameters instead imply y +/-0.28545 and z [-0.00877,0.25873].
  The previous bounds/ideal clearance are **untrusted**, not conservative.
  The asset itself is unchanged. Fixing `inspect_vehicle.py` to derive Cube bounds
  from size and full affine transform, and label generic authored bounds unverified.
  Six host geometry tests pass; four active PhysX volume probes of the copied
  actual collision prim are running before validating the corrected dimensions.
- **Active PhysX confirms the chassis-bound bug.** All 13 fixture cases pass at
  `revision-9e10d66a52c4455380dfbba8b4dfd444` (pinned-image launcher
  `revision-21a23c6da65e425fb0512348d7e9dc6c`). A tiny probe at body y=0.25 m and
  another at z=0.04 m intersect the collider despite lying outside the old
  authored-extent envelope; probes beyond the size-derived bounds remain clear.
  Correct chassis bounds in base: x +/-0.50370, y +/-0.28545,
  z [-0.00877,0.25873] m. Combined with measured flat loaded height, bottom
  clearance is **0.13615 m**, not the earlier **0.23413 m**. This does not establish
  a step/roll/pitch limit. No USD asset, deployed footprint or navigation policy
  was changed; previous inspection artifacts remain historical failed evidence.
- Final hybrid revision regression so far: **647 host tests, no skips**, using
  the OpenMP library at one worker; **187 local/differential tests pass with the
  serial library**. The subsequent geometry fix adds six passing host fixtures
  and the 13-case active PhysX run. Full qualification remains open.

## September 26 — approved v2 cycles, controlled ablations and evaluator foundation

- Cycle 0: saved the approved ten-cycle roadmap in v2. User-selected target is
  95% independently verified safe **motion** availability, overall and per family,
  plus zero observed false-safe motions. This is not the old cell-coverage metric.
- Baseline before new code: **80 local tests passed** with the frozen C++ library.
  No frozen runtime module, classifier threshold or historical evidence was changed.
- Cycle 2 hypothesis: old comparisons confounded observation-pose policy, lifetime
  and grid extent. Added opt-in `--observation-policy coherent|frozen_observation`
  and `--cells 60|200`. Both arms use the same current-rover pose and revision
  reconciliation. Frozen observations retain their first-used *local* placement,
  not stale coordinates in a revised map frame; this is not the original live baseline.
- Added matched comparison with source/input/library hashes, publication/pose/dose
  and query-denominator checks. It refuses multiple changed factors, missing tails,
  altered retained sources and instrumented timings. Initial tests: **89 local +
  17 comparison contracts passed**. The original 30-second replay path is preserved.
- Declared five-arm development matrix **before execution**: 320 projections each,
  coherent 2/5/30 seconds on 60 cells, frozen-observation 2 seconds on 60 cells,
  coherent 2 seconds on 200 cells. Same failed September 22 source/capture/library;
  startup and outside queries stay counted. All remain bounded, failed-source,
  non-authoritative diagnostics. OMP=4; host tests/inspection and comparator work
  overlap portions of this series, so timings are not uncontended/live qualification.
- Completed arms so far: coherent 2 s
  `revision-eed614c82105499c964b8e6a3170e68c` p95/p99 **119.92/137.18 ms**;
  frozen 2 s `revision-2fbeaebdd11d4292bd67fba39dbb2ad9` **109.04/126.30 ms**;
  coherent 5 s `revision-d2becfb938e241c794608bfdafc189db` **289.89/309.14 ms**;
  coherent 2 s/200 cells `revision-1e6684b8f4e14a19bbc3e642fe4d41d2`
  **277.57/303.14 ms**. All miss the 50/100 ms gates.
- Matched policy comparison `revision-a666bffd3fca49c285018fca62581f11`:
  coherent vs frozen availability **35.247% vs 35.344%** at 0–2 m and
  **28.047% vs 28.295%** at 2–3 m. Policy alone does not explain the large loss.
- Matched retention comparison `revision-1cb5fba50078486ca7f0b96faaf033e9`:
  2→5 s gives **35.247→46.178%** / **28.047→49.089%** availability. Unlike the
  earlier confounded comparison, this directly establishes retention sensitivity
  on this development prefix; it does not establish independent terrain safety.
- Matched extent comparison `revision-05eb6ef6725341a9986a40c191710837`:
  6→20 m gives **35.247→35.248%** / **28.047→28.841%** availability. Enlarging
  the map adds little near-body availability while roughly doubling cycle time.
- Cycle 1 foundations: evaluator-only continuous triangle support patches and
  motion-score accounting. Patch tests refuse gaps, overlapping/multilevel surfaces,
  vertical faces and workload overruns; no raster interpolation supplies missing truth.
  Motion metrics count missing/unknown decisions, require each family to meet 95%,
  reject stale/future/wrong-revision feasible outputs, retain failed-source status and
  distinguish correlated frame counts from physical exposures. **22 geometry + 21
  motion-accounting tests pass**; neither module is a complete wheel/body oracle.
- Concrete truth gap: the natural recording has sampled first hits, rock AABBs and
  a finite floor tile, not a full active collision mesh or loaded contact/braking
  contract. Do not use it to claim the new motion gate. Active-scene export and
  reference validation remain required; no world or robot motion started this turn.
- Environment: external archive resolves to writable `/dev/sda1`; no mission
  containers running. `nvidia-smi` and host `python3 docker/check_cuda_runtime.py`
  pass (context/allocation checked). Container/Isaac preflight and live acceptance
  remain unrun; no driver repair/reset was attempted.
- Cycle 4 preparation: standalone exact publication pose index and bounded native
  range/transform cache have **9 + 7 focused tests passing**. They are not yet
  connected to the mapper while the frozen ablation matrix is running. Next finish
  the matrix, connect as an opt-in backend, compare every output against the naive
  reference, and measure remaining integration cost before changing mechanisms.
- Five-arm matrix finished: coherent 30 s/60 cells
  `revision-1584988199974e51a8211e40586a15b8` reaches **67.572% / 68.552%**
  availability but **1651.41/1697.93 ms p95/p99**, 303.73 MiB RSS. Matched 2→30 s
  comparison `revision-e004f4d1ba494062a6aa71f22b57d293` keeps all 320 projections.
  No short/long retention window is promoted from these diagnostic counts.
- Connected the prepared backend opt-in, preserving default naive reference and
  C++ integration order. Additional real face/hole cases exercise both backends;
  full focused/regression suite after initial connection: **553 passed**, no skips.
- Declared exact backend matrix, 320 projections per arm: 2 s reference
  `revision-dc8e225436d34080a77664f243c17311` **117.24/143.82 ms p95/p99** vs prepared
  `revision-cdf79b77a0394186bf71d43e18213cc9` **92.40/113.99 ms**;
  5 s reference `revision-e0b1f887e83d49cda877049425eae939` **287.94/321.89 ms** vs
  prepared `revision-b48b7031cb244b9d90bcfa00a302f351` **215.74/243.92 ms**.
  Exact comparisons `revision-91b32e985f8a4280b5d7330219c1c570` and
  `revision-c11b0dca13b74d18af6fb1fe2dff1cf6` match **all 320 layer hashes,
  associations, poses, origins and availability values per pair**. Both remain
  runtime failures; no retry replaces a prior result.
- Prepared 5 s profile `revision-f63bd111f0924012a7c767949b2b2cf9`: all 80
  instrumented map/association records exactly match its uninstrumented prefix.
  Median over final 30 frames: association **23.45 ms**, transform **20.81 ms**,
  integration **146.47 ms**, projection **11.39 ms**. This is attribution, not compute
  acceptance. Repeated C++ integration remains the dominant cost.
- Observed mechanism limit: both prepared arms had **zero exact transformed-world
  cache hits** (18,610/44,275 transforms), because causal revisions kept changing
  placements. Range filtering ran only 959 times per arm. Do not introduce fuzzy
  pose reuse to claim a hit. Follow-up retains only filtered native coordinates by
  default, halving cache payload; optional exact world caching remains unit tested.
  Final memory/equivalence replay is recorded below when complete.
- Original baseline rechecked with the expanded runner:
  `revision-cd76be62edc3496aa489944a1e7d371f` matches all **748 original hashes,
  states and associations**, preserves terminal `simulation wall heartbeat lost`,
  p95/p99 **66.29/77.21 ms**, 197.93 MiB RSS. It is not a completed mission or live
  compute pass. No GLIM/native runtime source or C++ classifier was modified.
- Independent truth audit: `TerrainManager.updateTerrainCollider()` uses
  `meshSimplification`; `update(update_collider=False)` updates rendering without
  rebuilding the collider. This is a source distinction, not evidence of a measured
  terrain discrepancy in this run. Raw DEM triangles cannot be promoted to active
  physical truth. Inspected installed 107.3.26 Python stubs in the Isaac image:
  collision representation retrieval exposed there is convex, not a ready-made
  triangle-surface exporter. Next reference work must use validated active scene
  queries or a verified cooked-mesh extraction, preserving approximation/epoch.
  Read-only transient containers inspected SDK files only; no Isaac world started.
- Final cache revision: `revision-5ca18db493f94cdf8a700fbf63814ac4`, 5 s/320
  projections, retains filtered native coordinates only. Peak RSS **143.03 MiB**
  vs 175.02 MiB previously; p95/p99 **222.31/249.75 ms**, slightly slower than the
  preceding 215.74/243.92 ms attempt. Retained for verified memory reduction and
  exact outputs, **not** reported as another speedup or compute pass. Exact
  implementation comparison `revision-381482b184ce4e2bb909e830528f9046` verifies
  all 320 layers, associations, origins, poses and availability values unchanged.
- Final source validation: **555 passed, no skips** using OMP=1, frozen library and
  the 19-file regression command below. Black 24.10.0 check passes on all 13 changed
  Python modules/tests; `git diff --check` passes. Legacy destructive sweep bodies
  are tested only with mocked forwarding, never executed.
  Final local suite with **OMP=4: 139 passed**, including both backends' real C++
  face/hole fixtures. This is parallel-library regression, not live timing acceptance.
- Remaining implementation, not claimed complete: obtain independently validated
  active collider/support truth and measured operating limits; replace repeated
  integration with revision-aware spatial reconstruction (old/new footprints plus
  full 9x9 classifier context); then connected-ground selection, uncertainty,
  vehicle sweeps and held-out/live qualification. Do not optimize away evidence
  history to meet timing, use authored triangles as cooked truth, or substitute
  the passing host fixtures for simulator acceptance. Both old isolated C++ resource
  profiles remain historical; they do not qualify this new wrapper.

Final broad host regression command (repository root):

```bash
OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=astrobot-lab/src/astrobot_core \
TERRAIN_LIBRARY=/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-ecd489e9a270438eaf5bc646e8ed6010/libresolved_boundary.so \
python3 -m pytest -q -p no:cacheprovider \
  tools/test/test_local_terrain.py tools/test/test_local_ablation.py \
  tools/test/test_local_motion_evaluation.py tools/test/test_local_reference_mesh.py \
  tools/test/test_local_pose_lookup.py tools/test/test_local_return_cache.py \
  tools/test/test_shadow_terrain.py tools/test/test_shadow_inputs.py \
  tools/test/test_live_terrain_shadow.py tools/test/test_live_shadow_provenance.py \
  tools/test/test_natural_motion.py tools/test/test_natural_loop.py \
  tools/test/test_live_glim_accuracy.py tools/test/test_glim_accuracy.py \
  tools/test/test_layered_terrain_calibration.py tools/test/test_relative_terrain_surface.py \
  tools/test/test_archive_guard.py tools/test/test_system_overview.py \
  tools/test/test_terrain_algorithm_overview.py
```

## September 26 — local v2 implementation (in progress)

- Hypothesis: coherent short-window reconstruction can reduce registration scatter
  without changing the terrain classifier. Implemented a separate, non-authoritative
  local package and archived callback replay entry point; no changes to frozen runtime
  source modules or C++ classifier. Tests and replay results will be recorded here.
- Cleanup: relocated six old sweep implementations verbatim with hashes and CLI
  forwarders; moved the runbook to its documented path, retaining a root redirect.
  Preserved all existing failed artifacts and unrelated worktree changes.
- Baseline: 171 host tests passed, 3 skipped without a library. New local/cleanup
  suite passes 22 tests with the archived resolved-boundary library.
- Archive mount was verified outside the sandbox: writable host mount; no hardware
  fault or repair required. All archived experiments continue using the external drive.
- USD inspection failures preserved: plain Python lacked pxr
  (`revision-075b42ba259a4b3dbdc422cfaa5912ac`); bundled BBoxCache required explicit
  positional arguments (`revision-0037eb9b0d4047e5a8d34d1d98cb2735`). Fixed extraction
  uses extension paths and all positional arguments, without launching Isaac.
  Collision-inclusive result: `revision-f17bb2415b974e2d858af699ca349b16`;
  asset SHA256 `357bd6cd9531cceb7971059c96d8a61052165eea7f8f6ff9bf28f35a5de1ca67`.
  Wheel centers x +/-0.256, y +/-0.2854, z 0.03282 m; collision width ~0.11446 m,
  diameter ~0.3555 m. Body collision AABB x +/-0.50743, y +/-0.16296,
  z [0.08920,0.16076] m. Static geometry is not a qualified dynamic limit.
- Relative-motion diagnostic `revision-ff2e5ccffa464e8c92e6e31ad0ba7007`:
  first-published translation p95 at 1/2/5 s **4.87/5.72/6.88 cm**; final-revised
  retrospective **1.64/2.67/4.98 cm**. Missing endpoints and source failure remain
  explicit. Final-revised poses never become causal mapper input.
- Bounded 80-projection local 2 s replay `revision-08b178203b9c400097b4918c80c605ad`:
  **125.48/155.56 ms p95/p99**, 139.77 MiB RSS; not a complete-prefix or compute pass.
  Complete local 5 s replay `revision-1e92282f0c6c49a9b5c804635223c70c` processes
  **748 projections**, preserves the source heartbeat failure, and measures
  **284.79/311.06 ms p95/p99**, 140.72 MiB RSS. Neither is terrain-truth acceptance.
- Original 30 s baseline `revision-9ad2c58b63734a658bd5e6b4c1364668` matches **all
  748 recorded six-layer hashes, states and pose associations**. Offline p95/p99
  **61.32/75.58 ms** is not live acceptance; brief test/inspection activity overlapped
  this diagnostic. No best-repeat selection or relaxed timing gate.
- Fixed rover-relative query availability, mean over all 748 projections: baseline
  **68.55% / 79.86%** free at 0–2 / 2–3 m; coherent 5 s **44.76% / 49.50%**.
  Startup, unknown and outside-map queries stay counted. This is availability,
  not recall; shorter retention loses useful support despite improved geometry.
- Coherent causal pose evaluation `revision-70403c484bbd46eb91d67d0bcb5a349c`:
  **738 identical one-second pairs**, ten warmup frames, no missing matched endpoints.
  Translation p95 **1.70 cm coherent vs 4.72 cm first-publication**; rotation p95
  **0.176° vs 0.385°**. Uses actual publication-time associations, not future revisions.
- Profile `revision-62106c087dc74474bb5d005722b64c6f` exactly matches all 80 initial
  uninstrumented map hashes. Steady-prefix median stages: association **10.33 ms**,
  transformation **30.99 ms**, integration **57.62 ms**, projection **10.84 ms**.
  Repeated transformation/integration dominates; instrumented timing is not acceptance.
  Initial executed sources were hash-verified and retained in
  `revision-d631e2c39a8944f6a625018de4b90b9b`; later runners freeze dependencies at startup.
- All **74 local/cleanup contracts pass** with the real C++ library, including
  partial faces/holes across four phases, three rotations and two visible heights.
- Full 2 s diagnostic `revision-de25dbe85fd24a1291fc75531e32a87f` completed all
  **748 projections**, retaining the original heartbeat failure: **117.54/134.82 ms
  p95/p99**, 144.86 MiB RSS; both latency gates fail. Mean body-query free availability
  **33.05% / 21.89%** at 0–2 / 2–3 m. Coherent pose check
  `revision-69a84d45e697431f8573334b8864a4ec` gives the same 738 matched one-second
  geometry results as 5 s (1.70 vs 4.72 cm p95); shorter retention does not improve
  this pose metric, and loses still more observed support.
- Both frozen C++ resource profiles rerun, all attempts retained:
  capacity `revision-28354f7e06904057bf82e8b48db7677e` **1009.48 MiB /
  44.34 ms p95 / 56.40 ms p99**; dense-flat
  `revision-0af7342bbb6c43f7823a2e3a79fe1818` **326.18 MiB /
  37.72 ms p95 / 44.08 ms p99**. These pass isolated *core* gates, not the new
  repeated-reconstruction wrapper or concurrent Isaac qualification. Ancillary
  read-only diagnostics/tool setup occurred during the profiles; no rerun was
  selected to replace failures or claim an idle/concurrent-runtime benchmark.
- Final Python regression command: `OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
  PYTHONPATH=astrobot-lab/src/astrobot_core TERRAIN_LIBRARY=<frozen-resolved-library>
  python3 -m pytest -p no:cacheprovider` with local, shadow, shadow-inputs, live-shadow,
  provenance, natural-motion/loop, live/GLIM-accuracy, layered-calibration,
  relative-surface, archive and both HTML suites: **414 passed**, no skips.
  Includes 80 new local/cleanup/evaluator tests. `git diff --check` passes.
- Applied Black 24.10.0 (120 columns) only to eight new Python modules/tests;
  all ASTs are identical before/after formatting. Verified pre-format source
  snapshot `revision-e7a20b6c8ddc4e54a63f85c27fe15749` preserves reproduction of
  the executed diagnostics; newer replay runs already retain their own sources.
- Archived C++ executable cannot run directly on this host (`GLIBCXX_3.4.32`
  unavailable); no system libraries were replaced. Built the unchanged candidate
  sources locally with C++17/Eigen and matching observation/two-path/resolved-boundary
  macros. Layered contracts pass with OpenMP at 1 and 4 workers and in a separate
  non-OpenMP build. These are host geometry contracts, not live ROS validation.
- **First-slice decision:** retain the separately tested causal reconstruction and
  evaluators as diagnostics; do not promote a 2/5 s operating window. Better causal
  relative geometry is verified, but coverage/usefulness and rebuild latency regress.
  Next mechanism-specific work is retention/support and incremental observation
  reconstruction, followed by connected-ground/uncertainty feasibility—not a wider
  loop or another backend swap. Independent complete terrain/swept-body reference,
  braking and safe attitude limits are still missing for qualification. No Isaac
  world, movement, hardware reset or new navigation authority was started.

Visual entry point: [System field guide](system-overview.html): current architecture,
topic flow, frames, native timing/revisions, terrain classification, critical code,
guarded motion, evidence limits and tiered test/reproduction commands. The older
[interactive algorithm fixture](terrain-algorithm-overview.html) remains a historical
September 14 snapshot. Both are self-contained and need no ROS or Isaac to read.

## Latest checkpoint — September 22

### System documentation — current architecture and test field guide

- Added `system-overview.html`, a repository-grounded, self-contained guide with
  frame/timing/route schematics, searchable topic table, source-code links and
  copyable test commands. Explicitly separates the advisory native terrain path
  from the older Nav2 BEV path and separates GT terrain labels from permitted
  GT initialization, synthetic RTK aiding and guarded acquisition driving.
- Records the latest v2 loop's failed-prefix accuracy and runtime failures,
  incomplete full-loop gate, first-publication/global-revision limits, fixed-grid
  extent and outstanding shutdown invalidation gap. Historical fixture results
  remain dated and are not presented as current natural-scene acceptance.
- Runbook separates offline/read-only checks from opt-in simulator motion;
  archive/CUDA/sole-command-owner constraints and frozen configuration/library
  identities remain explicit. No simulator, capture or navigation launch was
  performed for this documentation task.
- Validation: **24 documentation contracts pass**, including the preserved
  historical fixture; **122 combined documentation/route/accuracy tests pass**.
  Tests cover local links/anchors, source-backed topics/reasons/frame and failure
  distinctions, accessible control metadata and shell syntax without executing
  runbook commands. Runner documented in `AGENTS.md`.
- Headless Chrome smoke checks pass at **1440x1000 desktop / 390x844 mobile**:
  no page-level horizontal overflow, topic search/reset/no-results, disclosure
  toggling, clipboard copy and selection fallback, no JavaScript exceptions and
  no external page requests. Desktop/frame/mobile screenshots visually checked.
  Documentation-only validation is not new terrain, SLAM or navigation acceptance.

### Loop driver correction — transition verified live; rerun stopped on heartbeat

- User requested fixing the driver's perfect-pivot assumption and repeating the
  loop. `loop16` now declares `frozen_post_turn_connection_v2`: each translation
  freezes its **actual first GT pose** as the connection start; the next nominal
  waypoint is unchanged. Endpoint steering closes the position error rather
  than merely matching the nominal rectangle heading. The route hash changes
  explicitly; this is not a claim of identical controller conditions to v1.
- Retained 8 cm cross-track bound about that fixed connection, 0.15 rad heading
  limit, 25 cm allowed starting corner offset, original surveyed center envelope,
  live floor/footprint checks and all speed, watchdog and timing limits. The
  entire rectangle plus existing envelope was already independently surveyed;
  no extra clearance is inferred from SLAM or GT terrain labels in the mapper.
- Recorded starts are immutable throughout each leg and retained in every
  command/reached record. Completion audit rejects fabricated or changing
  anchors and uses those same connections. **79 route tests pass**, including
  the old failure, corrected connector convergence, unchanged cross-track
  rejection and retained survey envelope. GPU/CUDA and archive checks pass.
- The prior physical-asset audit found nominal wheel centers symmetric about
  base_link XY (x +/-0.256 m, y +/-0.2854 m). This rules out a fixed horizontal
  wheel-center/base_link lever arm, not terrain/contact-dependent turning-center
  motion. Do not relabel all observed displacement as proven traction slip.
- Rerun started: simulator `revision-379884211d744a1790860f9969775530`, recorder
  `revision-38b96ba8739f4cfeb84da7d1200f9ec1`, mapper
  `revision-f0db6c401ce845b7800297622753d1d6`. Same 17,072-point clearance
  survey passes; the first leg is underway. GLIM/terrain library unchanged.
- **Driver correction verified in the live rerun**: first 2 m leg and 90° turn
  completed; the next connection started at world XY **(21.80260, 18.12814) m**
  and advanced **1.991 m over 18.50 s**. Maximum measured cross-track error about
  the frozen connection was **1.98 mm**, below the unchanged 8 cm bound. The
  recorded anchors for both translation legs pass the immutability/first-pose
  audit. The old nominal-line transition rejection did not recur.
- Rerun nevertheless **FAILED** at stamp **97.1000 s** on the separate
  `motion simulation heartbeat expired` condition. Recorded generation-2 GC
  pause **0.43733 s** coincides with the fault; lease wall age **0.58120 s**.
  Maximum native callback **0.16360 s**. The **1.71195 s** maximum tick gap is
  whole-run telemetry (including preflight), not an isolated measurement of the
  terminal stall. Permissions revoked and mapper invalidated; simulator stopped.
  No watchdog, acquisition deadline, clearance or compute gate was relaxed.
- Accuracy report **`revision-76d17e74292e428dac1ac36daa1a5549`**: recorded
  prefix **74.97 s / 4.25 m actual path length**, 2,250 native chunks, 748 map
  projections. All **2,243 integrated pose associations** match the live map
  inputs; no unprojected shutdown integration tail. Three final native poses
  lack GLIM brackets and remain unscored, not extrapolated. First-published
  p95: **6.32 cm horizontal / 5.58 mm height / 0.456° geodesic rotation**.
  Retrospective final-revised p95: **6.84 cm / 3.96 mm / 0.355°**. Fixed-frame
  GT/native checks pass; estimator remained RTK-aided with one-time GT init.
- Live mapper-cycle **80.47 / 106.25 / 109.78 ms median/p95/p99**, failing both
  latency gates. **98 route/accuracy contracts pass** and both diff checks pass.
  The requested assumption fix and rerun are complete, but the **full 16 m loop
  is not qualified or completed**. Next blocker is simulator/runtime scheduling
  and workload attribution, not another widening of driver tolerances or GLIM
  retuning. All old/new failed attempts remain archived.

## Previous checkpoint — September 21

### Longer-loop experiment — attempted; corner-to-straight transition failed safely

- User requested a much longer route with a loop. Added distinct `loop16`:
  initial-planar-base waypoints `(0,0) -> (2,0) -> (2,-6) -> (0,-6) -> (0,0)`
  metres, four bounded clockwise 90° turns, final original heading. **16 m**
  translation at **0.10 m/s**, turns capped at **0.10 rad/s**. No autonomous
  obstacle avoidance; simulator GT controls acquisition, not estimator inputs.
- Saved seed-43 USD rock bounds show no overlap with the complete inflated
  rectangular survey region. This is only a planning screen: actual first-hit
  PhysX ground, relative surface and swept-volume checks must pass before grant.
  No rock removal, teleport, geometry-label input or silent route substitution.
- Preserved 0.5 s wall watchdog, 1.5 s stall checks, 8 cm lateral and 0.15 rad
  heading bounds, 25 cm turn skid limit, footprint inflation and floor gates.
  Separate route dose has declared 85 s translation / 75 s rotation leg bounds
  and 1,800 s experiment wall cap, accounting for 6 m legs and 90° turns; old
  route deadlines/hashes remain unchanged. Longer cap is not a heartbeat waiver.
- Driver logs the actual arrived segment explicitly so retrospective checks do
  not apply the next leg's heading to a settled corner transition. **75 host
  contracts pass**, including old routes, all loop segments, corner skid bounds,
  complete rotation-footprint coverage and a closed ideal-controller loop.
  CUDA context/allocation and external-drive checks pass. Live attempt pending;
  the prior repeatable heartbeat/compute failures remain unresolved.
- Live attempt: simulator `revision-8cb2193d3ba44bdaa5e589b871881abd`, recorder
  `revision-dcd909832dd44ec4905a61a7c31c6749`, mapper
  `revision-675c0781408c4642821bf99be502812c`. Full preflight **passes**:
  17,072 samples, maximum local discontinuity **6.49 mm**, detrended roughness
  **15.83 mm**, maximum patch gradient **0.2148**, no inflated-volume rock
  overlap. The first 2 m leg completes and the first 90° turn is in progress.
  No completed-loop or timing/accuracy acceptance yet.
- Attempt **FAILED** at **79.7000 s**, before the first 6 m leg:
  `loop translation exceeded heading/lateral/longitudinal envelope`. Turn endpoint
  in the initial planar frame was **(1.79952, +0.13582) m**, heading
  **-89.4646°**. Relative to the nominal `(2,0)` corner, skid was **24.22 cm**
  total and **20.05 cm lateral** to the next leg. Thus the turn was legal under
  its 25 cm envelope, but the next leg's unchanged 8 cm cross-track guard refused
  motion. This is an acquisition-driver transition failure, not an obstacle or
  a GLIM-pose safety trigger. Added the exact retained endpoint as a regression:
  turn arrival must pass and the following straight command must remain rejected.
- The mapper retained **572 projections / 1,716 integrated chunks** before the
  collector stopped the experiment. No runtime generation-2 GC pause was logged;
  the timing summary's 1.759 s maximum tick gap includes pre-motion work and must
  not be mistaken for the cause of this stop. Simulator/collector are stopped.
  The 16 m loop was **not completed**; failed artifacts remain ineligible.
- Follow-up needed before another loop attempt: an explicit, bounded corner
  rejoin maneuver (or tested continuous-curvature path) whose full footprint lies
  inside independently checked free ground. Do not simply widen the straight
  guard, redefine the failed run as complete, or attribute commanded-path error
  from skid to GLIM. Runtime heartbeat/compute issues remain separate open gates.
- Accuracy diagnostic `revision-6d89cd6adaca416da18ec24ce0366952` stopped on a
  **1 ns** float-round versus ROS-truncate endpoint difference. Replaced generic
  GT interpolation with the existing `validate_gt_snapshots` same-tick contract:
  at most **2 ns representation difference**, exact published pose selected,
  no interpolation/extrapolation, gaps or missing tail snapshots accepted.
  Added a regression that accepts this numerical case but rejects a missing
  microsecond/tick. This changes the evaluator only, not estimator timestamps.
- Diagnostic `revision-7b6193ff946045f181ff2059f6bce693` then correctly exposed
  a shutdown-tail mismatch: after the collector's last received status, the node
  archived **574 maps / 1,722 projected input associations**, then SIGINT
  interrupted the next callback after **three more integrations** but before
  projection. Its final counter is 1,725, not 1,722. The diagnostic now reports
  this failed/incomplete-run gap explicitly; all projected associations must
  match, and any such gap still rejects completed-run qualification. No tail
  poses/maps are fabricated. The advisory node's final `valid:true` with
  `completed:false` also exposes a shutdown-status cleanup gap to fix before
  downstream integration; no navigation authority existed in this experiment.
- Final diagnostic **`revision-ad87923e56e34373a900f56b51dee1f6`** succeeds as
  a failed-prefix analysis, not route acceptance. **57.50 s / 2.25 m actual
  travel**, including the turn's skid. All 1,726 native stamps have exact GT
  snapshots and bounded GLIM associations; all **1,722 projected pose inputs**
  match their live records. Three unprojected shutdown integrations remain
  explicitly unverified as map output. First-published GLIM p95 errors:
  **6.08 cm horizontal / 8.53 mm height / 0.470° geodesic rotation**. Final-revised
  retrospective values: **5.33 cm / 6.03 mm / 0.315°**. Thus no catastrophic
  GLIM failure was observed through the first 90° turn; path-following failure
  was physical skid versus the route driver's nominal segment, not pose drift.
  Mapper cycle **71.99 / 88.02 / 94.69 ms median/p95/p99** still fails compute.
  Final route/accuracy regressions: **95 passed**; both worktree diff checks pass.

### Requested reproduction — heartbeat failure reproduced; GLIM accuracy measured

- User requested a repeat and GLIM-vs-GT accuracy, not a mapper tuning change.
  Repeating the identical seed-43 `straight_slow` live-shadow experiment with the
  same frozen library/configuration, guarded driver and watchdogs. This is an
  explicitly requested repeat, never replacement of the prior failed attempt.
  NVIDIA/CUDA context-allocation and external-device mount checks pass.
- Repeat capture `revision-ded594b82bc147ff9751d20953753b62`, recorder
  `revision-b515e5699de54692a635dc2f497a8654`, mapper
  `revision-0dd497a6aa6947c585b09d7cd418e6eb`. Initial measurement-time IMU pose
  check passes: **0.349 mm / 0.00224 degrees**, using the declared one-time GT
  initialization. This is startup accuracy, not route accuracy.
- Added `tools/measure_live_glim_accuracy.py`: diagnostic-only evaluation of
  completed or failed prefixes, fixed world-to-map home, same physical base_link,
  no fitted alignment or time shift. Independently checks published GT against
  native Isaac sensor poses and mount TF; verifies every actually integrated
  pose association, and separates causal first-publication from retrospective
  final revisions. Missing associations and source failures stay explicit.
  **15 tests pass** across new frame/receipt/barrier and existing accuracy
  contracts. No workload profiling/replay is run concurrently with the capture.
- Repeat **FAILED** at stamp **92.6000 s** after **748 projections / 2,243
  integrated chunks**: simulator tick gap **1.16009 s**, coincident generation-2
  GC pause **0.42150 s**, native callback maximum **0.11405 s**. Collector revoked
  permission on stale request heartbeat; mapper subsequently latched simulation
  wall-heartbeat loss. Simulator stopped; original and repeated failures retained.
- First diagnostic attempts (`revision-e77a4b37c7ba43bca970445231e3f9ef` and
  `revision-e2e57873c6924ad6a96d3647c7885af4`) rejected GT before computing
  errors: new evaluator incorrectly expected a `map` header. Recorded messages
  and the existing `glim_gt_start` contract confirm simulator world is `gt_map`.
  Corrected the evaluator to require **gt_map**, retaining the explicit fixed
  home translation into GLIM map and independent native-GT cross-check. Added a
  regression that rejects `map`, `odom` and `base_link`; no estimator/frame wiring
  or captured evidence was changed. Failed diagnostic attempts remain archived.
- Diagnostic `revision-0689d279f3e041deb6059a4b045bdb87` then rejected a
  sub-nanosecond native-float/ROS-integer endpoint mismatch (0.365 ns), not a
  missing GT tick. Normalize native acquisition stamps to the **same integer
  nanoseconds used in transport**, without fitting a time offset or enabling
  extrapolation. Added an endpoint regression that still rejects a real 1 ns
  extrapolation. **17 accuracy/frame/timing contracts pass**.
- Completed diagnostic reports: original
  `revision-5c8f498165ec409593a1bbed82c63377`, repeat
  `revision-c06d71c211d24d5595ab6c3d5ea7d714`. Both contain per-chunk poses,
  arrays and `accuracy.png`. Every actually integrated association matches the
  live record exactly (**2,331 / 2,243**). Published GT and independently captured
  native/TF-derived base pose agree below **1e-9 m / 4e-6 degrees**; no alignment
  was fitted. Native cadence/current-pose geometry audits pass for these prefixes.

| Fixed-frame base_link diagnostic | Original | Repeat |
| --- | ---: | ---: |
| Recorded acquisition duration / GT travel | 77.97 s / 3.95 m | 74.97 s / 3.85 m |
| First-published horizontal p95 / maximum | 6.66 / 10.79 cm | 7.13 / 9.68 cm |
| First-published absolute height p95 | 8.06 mm | 7.25 mm |
| First-published roll / pitch / yaw absolute p95 | 0.112 / 0.230 / 0.351° | 0.113 / 0.236 / 0.330° |
| First-published geodesic rotation p95 | 0.389° | 0.367° |
| First-published 3D position RMSE | 3.99 cm | 4.25 cm |
| Final-revised horizontal p95 | 7.23 cm | 6.45 cm |
| Final-revised absolute height p95 | 8.29 mm | 5.80 mm |
| Final-revised geodesic rotation p95 | 0.267° | 0.232° |
| Unassociated native tail samples (no extrapolation) | 1 of 2,340 | 3 of 2,250 |
| Live mapper-cycle p95 / p99 | 90.92 / 95.89 ms | 90.70 / 98.16 ms |

These are the **current RTK-position/heading-aided GLIM configuration**, physics
IMU and one-time GT initialization—not unaided LiDAR SLAM. First-published scores
use the unchanged 200 ms availability deadline. Final-revised scores are
retrospective and were not the terrain mapper's input; smoothing is not uniformly
better in horizontal absolute error. All missing associations are at the
interrupted tails, none among actually integrated poses. First/last-three-second
breakdowns and full sequences are retained in the reports.

Conclusion: the scheduling/heartbeat failure is reproducible; catastrophic GLIM
pose loss is **not observed in either short recorded prefix**. Centimetre-scale
translation and sub-degree orientation remain relevant to boundary sensitivity,
but do not explain a wall-clock heartbeat stall. Both captures remain failed,
the 50 ms cycle-p95 gate fails in both, and longer/more diverse route robustness
is unproven. Next work remains timing-stage/stall attribution, not arbitrary
GLIM retuning. Simulator/collector are stopped; no broad campaign was started.

### Live shadow integration — implemented; physical qualification failed timing/heartbeat

User approved broad shadow testing before autonomous integration. Deliver an
advisory ROS mapper, not a replacement navigation source. Preserve the frozen
compact-fit library, acquisition-time native evidence, first-publication pose
policy, 10 cm cells, 30 s lifetime and existing gates. Then qualify seed-43
routes before the declared seeds 44–47 × four routes × two repeats campaign.
Physical integration attempts are recorded below; the wider campaign gates remain
**NOT RUN**. Do not count synthetic ROS transport tests as simulator acceptance.

- Found a live integration bug before capture: projecting at current time makes
  subsequently arriving measurement-time evidence older than the core's monotonic
  evidence clock. Implemented a **200 ms delayed acquisition watermark**, with
  10 Hz projection; output header carries evidence time, never a falsely fresh
  publication stamp. Pose availability is conservatively bounded by the next
  received `/clock`, and each chunk retains its original 200 ms pose deadline.
- Added standard `PointCloud2` native transport, independent of Isaac custom
  message builds: exact XYZ/depth/intensity plus atomic capture UUID/sequence,
  no GT poses or terrain labels. Opt-in `OMNILRS_TERRAIN_NATIVE_PUBLISH=true`
  currently requires the guarded natural-capture node; default capture unchanged.
- Added bounded receipt/acquisition queues, first-published pose-window pruning,
  explicit completion, and latched failure/unknown-map publication on gaps,
  duplicates, deadline loss, resets, extrinsic changes or unqualified non-ODOM
  revisions. Normal local ODOM revisions never reposition integrated evidence.
  Non-ODOM events currently fail closed even if they might be harmless submap
  bookkeeping; revision-aware reconstruction is not claimed.
- Added opt-in ROS node, separate `/terrain_shadow/costmap` and status topics,
  archival callback order/raw bytes/projection hashes, and exact event replay.
  Requires explicit frozen library, map origin and sensor range bounds. No
  `cmd_vel`, map TF or active-navigation topic publisher is added.
- Initial host contracts: **25 passed, 1 optional native-library test skipped**.
  Actual ROS transport/replay, deployment build, physical capture readiness,
  concurrent GLIM timing and the broader campaign remain pending. Host NVIDIA
  visibility and bounded CUDA context/allocation probes both pass this turn.
- Build/install gate `revision-9de6cfe95d3246aead03867784a55009`: opt-in mapper
  package and launch overlay build successfully; **10/10 CTests** pass. Real-core
  host suite **26/26** passes; existing native/timing/shadow/natural contracts
  **130 passed, 2 optional skips**. Actual ROS/DDS tests pass for a nonempty map,
  exact six-projection callback-order replay, and unknown-map invalidation on
  clock reset or a missing chunk. Extended ROS/MCAP recorder suite: **24 passed**.
- Guarded collector now accepts `--terrain-library`, freezes it, derives map
  origin/range only from declared initialization/sensor metadata, checks mapper
  identity/heartbeat/failure, and requires complete native-prefix integration.
  Simulator request must explicitly declare native publication before any grant.
  Native/status/map topics are archived but remain optional for historical runs.
- First physical seed-43 `straight_slow` integration attempt started after the
  above gates, simulator `revision-7ff3e0dc5f8e4aa49cd3f6f4d3c559ab`.
  No broad-campaign run is authorized by these software results. A shell setup
  attempt initially used a nonexistent root `local_setup.bash` for the standalone
  CMake install; corrected the isolated AMENT prefix before the collector started.
  No grant/motion came from that failed setup attempt. Existing GLIM and mapper
  library parameters remain frozen.
- First physical attempt **failed before any motion grant**: the installed
  archive guard treated its own external-drive install prefix as the workspace,
  so the mapper correctly refused what it mistakenly classified as a fallback.
  Recorder `revision-34fdbed4d879407eb395a59080a7747a` retained the failure and
  false mapper-heartbeat gate. Interrupted the collector cleanly and stopped the
  simulator; no arbitrary pose/terrain tuning followed.
- Fix: resolve the actual repository/deployment workspace using repository
  markers or explicit `ASTROBOT_WORKSPACE`; still reject same-device, missing,
  unknown and insufficient-capacity archive destinations. Added installed-prefix
  and unknown-workspace contracts. Launch now shuts the experiment down when
  the advisory mapper process exits rather than leaving GLIM alive waiting for
  a mapper heartbeat. Rebuild `revision-9fb0c283273047b99be061705a90fcd6`
  tests the **installed executable**, not only `python tools/...`; no overwrite
  of the failed install/run.
- Installed tests then pass **30/30**. Second physical startup attempt, recorder
  `revision-7ae79f6b12cd4041bacb14f7bd4045ae`, exposes the other deployment layout:
  Isaac mounts only OmniLRS and tools, so full-repository-only marker discovery
  fails before a simulator capture directory or native measurements are created.
  Docker automatically restarted once; stopped that restart and the collector,
  preserving the full simulator log under `revision-9fb0c283273047b99be061705a90fcd6`.
  This is a workspace-resolution failure, **not CUDA recovery or terrain evidence**.
- Added simulator-submodule markers and an independent container-root-filesystem
  rejection (a root fallback can have a different device ID from the host-bound
  workspace). **8 archive contracts pass**. A lightweight preflight inside the
  actual Isaac image with actual mounts resolves `/workspace/omnilrs` and accepts
  `/archive`; no world construction was needed for this check. Final isolated
  install is `revision-b178b18ddea54befa50bf8e8164ccd25`.
- Final installed ROS/recorder/archive/provenance test run: **36 passed**.
  The opt-in archive compose override now sets simulator `restart: "no"`:
  one declared experiment cannot silently turn into a second world after a
  crash. Normal non-archive compose behavior is unchanged. Third physical
  startup attempt uses that policy and the mount-preflighted installation.
- Live attempt reached actual guarded motion: simulator
  `revision-6141060d69be4cf697d0a2a2c8b46813`, recorder
  `revision-8ac8159f6dd742718aefd00f581308c1`, advisory mapper
  `revision-c668ba6a093249668fea61023ccdf5bd`. **777** projections / **2,331**
  integrated native chunks passed causal/sequence checks before a terminal
  simulator heartbeat failure near the end of the route. The usual pending queue
  was roughly six chunks (~1.3 MiB). **FAILED**, not completed acceptance.
- Final concurrent mapper cycle timings: **74.86 ms median / 90.92 ms p95 /
  95.89 ms p99**, exceeding the unchanged 50 ms p95 gate despite earlier isolated
  stress passes. These are mapper-cycle measurements, not end-to-end ROS latency.
  No timing retry, input decimation or threshold change was used. Wider seeds and
  repeated-route campaign remain gated on profiling this realistic workload and
  verifying complete live replay/native provenance.
- Simulator stopped safely at stamp **96.2667 s**: maximum tick wall gap
  **1.21766 s**, recorded generation-2 GC pause **0.48811 s** at the same stamp,
  maximum native callback duration **0.11919 s**. The GC pause is correlated, not
  proof of the entire stall's cause. Collector revoked motion permission on
  clock/request/IMU staleness; mapper latched `simulation wall heartbeat lost`
  and invalidated its map. Simulator and collector are stopped. All three failed
  artifacts remain intact and ineligible for completed-run acceptance.
- Added explicit `replay_live_terrain_shadow.py --diagnose-heartbeat-prefix`:
  accepts only this specific archived terminal failure, verifies all recorded
  prefix maps/associations and terminal status, and preserves the failed label.
  Ordinary replay still rejects failed/incomplete captures. Wall-watchdog failure
  is replayed as a recorded event, not claimed to be reproduced offline. Reported
  offline timings are diagnostic only. **45 host contracts passed, 1 optional
  real-library test skipped** across provenance/live-core/archive tests.
- Exact live replay also verifies the code executing replay against the recorded
  runtime modules, not only that archived input files are unchanged. **4** pure
  provenance contracts cover matching, missing, changed and ambiguous sources.
- Failed-prefix diagnostic `revision-c76f8bb5f8ba44bda2b90ab29f337400` verifies
  **all 777 six-field projection hashes, associations, counts and terminal
  invalidation exactly** with the unchanged frozen library. It explicitly reports
  `source_experiment_completed:false`; every acceptance field remains false.
  Offline diagnostic cycle timings **57.18 / 75.04 / 82.18 ms p50/p95/p99**
  remain above the p95 budget. This suggests concurrent GLIM/Isaac load is not the
  sole problem; the realistic mixed terrain workload needs stage profiling too.
  This diagnostic is not an idle-resource acceptance benchmark or a replacement
  for the failed live attempt.
- Final replay-change regression: **15 passed** (three real ROS/DDS
  healthy/reset/missing-chunk cases plus twelve provenance/diagnostic-eligibility
  contracts). Healthy completed replay still passes; fault cases remain rejected
  by ordinary replay. Root and simulator-submodule `git diff --check` pass.

#### Next gated actions (not yet executed)

1. Profile the exact retained natural prefix by stage with a separately frozen,
   instrumented build; require identical maps/associations. Distinguish terrain
   classification, evidence integration and Python/ROS overhead before changing
   the algorithm. No decimation, weaker support or longer pose deadlines.
2. Attribute simulator heartbeat stalls using phase/GC timing. The observed
   0.488 s collection alone does not establish the full 1.218 s gap's cause.
   Retain automatic GC and watchdogs until a bounded, tested fix is justified.
3. After a substantive fix, rerun the declared seed-43 route with unchanged
   safety/compute gates, exact completed live replay and native sensor audit.
   Then qualify the remaining three development routes and fault conditions.
4. Only after those gates pass, start the 32-attempt seed-44–47 campaign and
   endurance run. Navigation integration remains a later gate: the mapper is
   advisory only, analytic coverage remains below 95%, revised-pose conflicts
   and original Nav2 primitive safety failures remain unresolved.

### Continuing iteration — compact evidence and bounded boundary analysis

**Current result:** both fixed resource profiles now pass: capacity **1,013.2
MiB / 48.85 ms p95 / 54.29 ms p99**, dense-flat **330.1 MiB / 47.70 ms p95 /
51.99 ms p99**. The compact-fit cache preserves full replay outputs exactly.
Margins remain small; these are isolated resource runs, not live scheduling proof.
Classification
refinement recovers analytic measured coverage **93.52% → 94.18%** while retaining
12/12 observed objects and zero course sequence hazards. Still below 95%; still
opt-in. The three revised-GLIM conflicts are traced to centimetre-scale lateral
displacement, not corrected or hidden. A separate Nav2 primitive interior-check
failure is reproduced; full-area and conservatively bounded swept-motion vetoes
are tested offline, **not installed**. The full Nav2 test suite still reports the
original safety failure rather than suppressing it.

User requested continued iterative GLIM/traversability improvement. Preserve the
qualified GLIM configuration and all recorded failures; do not conflate GLIM
pose sensitivity with terrain truth. First isolate the resource failure without
changing classification: replace full runtime observation counters with a
one-byte sufficient statistic (saturating two ground votes, context veto, native
boundary veto), retained and merged in the same immutable buckets. Full counts
remain diagnostic-only. Reuse identical ordered native neighborhood merges
within one input/cell; do not regroup points or cache across timestamps.
Frozen candidate: `revision-756613974bbf4f799bf271ae4483ae2b`.
Require exhaustive compact-state merge contracts, all terrain fixtures, exact
frozen analytic/natural output comparison, then both standalone resource profiles.
No shorter lifetime, point skipping, relaxed gates or production promotion.

- Compact candidate passes **581 + 128 terrain fixtures**, full direct C++
  mapper contracts, and exhaustive **65,536 event sequences × 9 bucket splits**
  for sufficient-statistic equivalence. Analytic replay
  `revision-e5f957414f204b859b577ab9deef4c54` matches all **439 frames / two
  variants / final six arrays** exactly. Natural replay
  `revision-b2022742bdcb46c0ac8f5014bf266722` matches all **633 frames / three
  branches**, final arrays and input/pose provenance exactly. Replays ran
  concurrently for correctness only; their timing is not resource acceptance.
- Added exact raw-index conflict tracing, not closest-point fitting or a new
  recall denominator. `revision-b25c76840e214642a5f955f448ac7b6b` verifies all
  **1,895 chunks**, preserves **1,986 point correspondences** for the three
  final revised conflicts, and both measurement-time transforms. First-published
  has no final occupied→free conflict. Four host contracts pass. This is an
  immutable diagnostic fixture; no GT information enters mapping.
- Host `nvidia-smi` and bounded CUDA context/allocation probe pass. No new
  simulator or motion capture has been started during this iteration.
- Compact-state resource runs: `revision-8f549e49d0d8442d833de0861ea5990f`
  capacity **1,013.4 MiB / 57.33 ms p95 / 68.53 ms p99**;
  `revision-b21008c41621493c8dbea27edb5cf294` dense-flat **330.0 MiB /
  57.20 ms p95 / 61.54 ms p99**. Memory passes (capacity has little headroom),
  both p95 gates fail. Preserve these results; no timing retry is a pass.
  Projection dominates dense-flat latency. Next representation-only change:
  separate native-input moments from retained cell/layer storage, so projection
  and expiry do not scan around unused native matrices. Same values/order,
  immutable buckets and classifier; require exact replays again.
- Separated-native-state candidate frozen as
  `revision-70f4ce5f1c704b26a007b1eea209e9dc`; 581 boundary and 128 broad tests
  pass. Full exact-output replays running before resource measurement.
- Boundary attribution finds **42 of 48 native-warning source cells already
  classified occupied** on the frozen worst-offset analytic final map; only
  six remain partial/unknown. Next isolated classifier hypothesis: native
  ambiguity should not add a second unknown-space halo from an already occupied
  source. Keep the source occupied and retain **all existing** below-plane,
  pooled-normal, positive-edge, support and body-clearance guards. Unresolved
  native faces keep the full original halo. This is not a general halo removal,
  changed occupied threshold or navigation inflation substitute; require every
  partial-face and full-course sequence safety gate before considering it.
- Separated-state exact comparisons pass: analytic
  `revision-bae38166094a4960885c519e58427721` (**439 frames / two variants**),
  natural `revision-b771c7f9a65b4455b2f4bd96d04463f0` (**633 frames / three
  modes**), including full arrays/provenance. Resource profiles run separately.
- Resolved-source-only halo candidate frozen as
  `revision-aaf0158eb4484edfac81f98f7a73dbe1` (explicit
  `TERRAIN_RESOLVED_BOUNDARY`, no production target change): **581 original +
  24 new mixed resolved/partial-face sequences + 128 broad terrain tests pass**.
  The added fixture verifies that detecting one tall end cannot clear the rest
  of a partially observed wall across rotations, grid offsets and rigid jitter.
- Resolved-source candidate also passes **108 diagnostic/mirrored boundary
  fixtures and full C++ contracts**. Worst-offset physical native replay starts
  only after those gates; no promotion inferred from synthetic results.
- Separated-native-state resources: `revision-a05efe5867ad4a2db8e7d389b1eae668`
  capacity **1,012.5 MiB / 58.21 ms p95 / 69.39 ms p99**;
  `revision-b31f461031eb4409a45f23025508e8db` dense-flat **330.5 MiB /
  50.029 ms p95 / 56.67 ms p99**. Both p95 tests **fail**, including the narrow
  0.029 ms miss; do not round or retry it into a pass. Dense-flat cache behavior
  improved, capacity expiry/classification remains too expensive.
- Resolved-source halo physical replay
  `revision-ee65802443b1476bae919f2f30819929`: **94.183% measured / 94.256%
  inferred** (up from 93.518% / 93.590%), **12/12 obstacles**, **zero sequence
  positive/dropoff false-free**. Still fails 95% and remains below the original
  retained mapper's 94.454% / 94.526% analytic coverage. Retain as an opt-in
  improvement over the two-path prototype, not production acceptance.
- Next compute candidate `revision-3de9a5b2273340e2b297a3492d570557` parallelizes
  only independent bucket expiry, same-input patch construction and per-layer
  native classification, max four workers. All ordered merges inside a cell
  stay serial/exact; earliest expiry uses a minimum reduction. Diagnostic and
  retained cells never race with patch construction. Worker-count fixtures,
  complete exact-output replays and both resource profiles are required again.
- Resolved-source natural replay `revision-5b8b1ca07f034dc08d9097f3be268939`
  completes all **1,895 chunks / 633 projections**: final free GT/first/revised
  **15,936 / 13,404 / 15,451**. The first-published gain over original retained
  mapper is **2,519 cells (+23.14%)**, but incremental gain over the two-path
  parent is only five cells. The same three revised occupied→free conflicts
  remain. This boundary refinement helps the analytic course more than the
  natural recording; do not present it as a GLIM accuracy improvement.
- Exact-index conflict tracing shows **184** recent returns at least 8 cm above
  the GT-pose mapper's final estimated ground in the three conflict cells; **all
  move from column 182 to 183** under revised poses. X displacement is
  **1.94–5.77 cm (median 3.25 cm)**. This subset is explanatory only: the plane
  is mapper-estimated, not independent terrain GT, and the 30 s window is not
  asserted to reconstruct immutable bucket membership. No pose fit/correction
  or height/denominator change is introduced. Addressing registration margins
  is more justified by this evidence than arbitrarily retuning GLIM calibration.
- Parallel parent analytic `revision-25b35b2d81b54150a027e47a36779a90` and
  natural `revision-606bcd79f197487b8969e4f2f7f61011` pass exact complete-output
  comparisons to the pre-optimization prototype. Resolved-source parallel
  analytic `revision-ab4fd472ca4c4f12976ec7b6144c3dfd` also exactly matches its
  serial parent, all 439 frames/both variants/final arrays. CMake builds default,
  two-path and resolved-source targets; **all four corresponding CTests pass**.
- Parallel resolved-source resource runs:
  `revision-d2484491d6b043068ec2df5a6d624afe` capacity **1,012.6 MiB / 48.45 ms
  p95 / 53.84 ms p99**, passes all declared resource gates (little headroom).
  `revision-c062d87df7f046969ff764024cfee158` dense-flat **329.9 MiB / 52.20 ms
  p95 / 57.98 ms p99**, fails p95. Keep both; partial resource success is not
  promotion. Capacity improved from the original 1,206 MiB / 67.27 ms p95.
- Installed Nav2 primitive check `revision-36f272153c6b4e4faa88aeeb34fc5a2b`
  still **fails** with this mapper: **3/1,932** tested padded footprints accepted
  despite continuous-wall intersection, including **2 body collisions**.
  All three have perimeter cost **253**, below the primitive's lethal-254 veto.
  A center/perimeter-only test can miss a wall wholly inside the noncircular
  footprint; high soft inflation does not prove collision rejection. Preserve
  this result and unchanged primitive output. Add a separate **offline-only**
  convex footprint/cell-overlap veto for lethal/unknown interior cells, then
  independently test usefulness and continuous-wall safety; no controller or
  installed costmap policy is changed by this diagnostic prototype.
- Offline area-veto archive `revision-1dffada8f40f4e61a812943184c4c867` retains
  the original primitive failure (**3 padded / 2 body intersections**) and
  reports the separate candidate: **0 intersections / 1,143 accepted poses**
  versus primitive 1,153 accepted, over all **1,932** fixed queries. The runner
  still exits nonzero for the original failure; no relabeling of acceptance.
  Six targeted tests pass, including interior unknown/lethal cells, out-of-map
  footprint, soft-cost traversability and independent continuous wall geometry.
  The two original retained/rejected-core wall checks were explicitly deselected
  in that targeted run; their primitive failure is independently archived above.
  This prototype checks convex rectangular footprint/cell area overlap; it is
  **not installed**, not a swept-motion check and not controller/mission proof.

#### Profile-guided cache and swept-footprint continuation

Continuation in progress: host kernel-matched `perf` is unavailable. Added
compile-time-only `TERRAIN_PROFILE` projection-stage instrumentation (no normal
build output/overhead), frozen in `revision-6ea2661b0ea84fdc9ebfd0f99d30c458`.
Its diagnostic dense-flat run `revision-fd268406d21e42bc962bc3fce795e9e7`
retains all 320 inputs; **not a resource acceptance attempt**. Mean stage times:
ground classification **19.04 ms**, corner certificates **3.83 ms**, connectivity
**2.44 ms**, setup **1.25 ms**, support cache **1.11 ms**. The next isolated
representation change caches the exact selected lowest points in a compact
per-projection array, only in the two-path experiment. It preserves ordered fit
samples, thresholds, output classes and evidence lifetime. Frozen output equality
and both independent resource profiles remain required before claiming a gain.

- Compact-fit candidate frozen in `revision-ecd489e9a270438eaf5bc646e8ed6010`:
  **605 boundary + 128 broad terrain contracts pass**. Full analytic and natural
  same-input replays are running (correctness only, concurrent timings excluded).
- Offline swept-area veto now bounds continuous linear-XY/unwrapped-yaw motion
  using midpoint footprints expanded by the maximum intervening point motion.
  Rotation winding, unknown cells, out-of-map geometry and a finite work budget
  are preserved; no live planner/controller change. First extended test run:
  **57 passed, 2 failed, 2 original primitive checks deselected**. Both failures
  were the independent oracle rejecting an incorrect test assumption: a fixed
  first-quadrant obstacle lies in the counterclockwise but not clockwise
  quarter-turn corner arc. Mirror the obstacle for clockwise fixtures, retaining
  both original failure output and the same oracle/acceptance requirements.
- Corrected offline suite: **59 passed, 2 original primitive checks deselected**;
  log `revision-ecd489e9a270438eaf5bc646e8ed6010/nav2_tests_mirrored.txt`.
  Includes 53 new grid-phase/rotation/sweep contracts. Sweeps are qualified only
  for declared linear XY and unwrapped-yaw interpolation, with a conservative
  enclosing margin and fail-closed 4,096-interval work cap. No curved-path,
  traction, dynamic-obstacle or live scheduling claim follows.
- Compact-fit C++ suites **4/4**, diagnostic fixtures **108/108** pass. Analytic
  `revision-479c896e0d5c4d91b2e10aaeafc40952` exactly matches every one of
  **439 frames / two variants / final six arrays**. Natural
  `revision-5aedfb157f484c6cbd06572271ca3d28` exactly matches **633 frames /
  three branches**, final arrays, pose sensitivity and input provenance. Coverage
  and the three revised-pose conflicts are therefore unchanged, not improved.
  Standalone capacity and dense-flat resource runs follow sequentially, with no
  replay/compiler/test workload running alongside them.
- Actual installed MPPI `critics/cost_critic.hpp` inspection confirms its inline
  `inCollision` uses `footprintCostAtPose`, treats cost **253 as non-collision
  when `consider_footprint` is true**, and treats **255 as non-collision when
  tracking unknown**. Current YAML enables footprint checking and unknown-space
  tracking. This strengthens the need for a consumer-level test; it does **not**
  prove the full optimizer emits a colliding command (scoring, trajectory
  selection and other controller checks are not exercised by the primitive
  probe). Smac2D's `allow_unknown: false` does not establish a controller-wide
  unknown-footprint veto. No installed code or navigation setting was modified.
- Uninstrumented compact-fit resource results (one attempt each, all 320 inputs):
  - Capacity `revision-63323e67a5694517bbc34956b55a5d5f`: **1,013.15 MiB /
    48.8498 ms p95 / 54.2936 ms p99**, all declared resource gates pass.
  - Dense-flat `revision-c0618ccd039a4cb8819509a219f985d3`: **330.07 MiB /
    47.7011 ms p95 / 51.9850 ms p99**, all declared resource gates pass.
  - Previous 52.20 ms dense-flat failure remains retained. No unchanged-code
    retry was substituted. Both native replays were completed before these
    sequential resource attempts; concurrent correctness timing is not used.
    Instrumented dense-flat final six fields also exactly match the frozen
    parent; its timing remains diagnostic only.
- Full extended Nav2 suite (no deselection): **59 passed, 1 failed, 1 skipped**.
  Failure is the unchanged original primitive accepting **three** wall-intersecting
  padded footprints. Skip is the optional rejected-library comparison, not the
  candidate. The 53 new static/swept and six previous area/primitive contracts
  pass. Log: `revision-ecd489e9a270438eaf5bc646e8ed6010/nav2_full_tests.txt`.
  Probe/test/adapter sources, actual inspected MPPI header and current YAML are
  frozen under that build's `nav2-sweep-source/`. The candidate gate did not
  override, xfail or relabel the original failure.
- **34** additional host provenance/replay/coverage/conflict contracts pass;
  `git diff --check` passes. No simulator, movement command, driver reset,
  production mapper switch or GLIM parameter change occurred.

#### Current next steps

1. Resource gates now pass for the opt-in compact-fit candidate, but capacity
   memory headroom is only about 11 MiB and p95 headroom about 1.15 ms. Sparse
   native work-state allocation remains an exact-output optimization opportunity;
   live co-scheduling with GLIM and navigation is a separate unpassed gate.
   Do not turn isolated profile success into a production promotion.
2. Retain the resolved-boundary experiment separately. Coverage is 94.18%, not
   95%, and the original retained core still wins the analytic coverage metric.
   Attribute its remaining support/clearance losses; do not remove unresolved
   face guards to recover the last fraction of a percent.
3. Evaluate pose-uncertainty treatment with the frozen three-cell correspondence
   fixture, then separate natural motion data. The 1.94–5.77 cm displacement is
   real, not a frame-origin blunder; avoid pretending zero-error mapping follows
   from acceptable global pose percentiles. GLIM settings were **not retuned**
   this iteration. Independent natural terrain geometry and live-causal mapping
   qualification remain open.
4. Grid-phase/rotation and bounded straight-segment swept fixtures now pass.
   Build an actual MPPI/Smac shadow-consumer test with frozen maps and trajectories,
   including unknown interiors and the three retained wall failures. Measure
   CPU cost and preserve the original primitive result beside the candidate veto.
   Curved segments need a proved enclosure; piecewise endpoints alone are not
   sufficient. No mission launch or command publication has changed here.

### Two-path native surface / boundary prototype — promising, not promoted

**Decision:** retain as an opt-in experiment, leave production unchanged.
It improves recorded first-published GLIM free coverage by **23.1%** and passes
the synthetic safety fixtures plus the tested physical course's sequence safety.
The original prototype failed independent **flat coverage and resource gates**;
the latest representation-only work above clears the fixed resource profiles,
but flat coverage and three revised-pose obstacle conflicts remain unresolved.
General robustness is **not proven**.

User approved the next classifier experiment. Opt-in `layered_terrain_two_path`
uses the same native-only 9x9 context and 5 mm conservative center-return bound.
Repeated contextual ground evidence may suppress a pooled-normal trigger, but
native returns that cannot fit a slope-bounded local surface independently leave
a boundary warning (the earlier contextual-offset version is rejected below).
The warning is retained in the immutable time bucket and
applies the existing one-cell unknown halo; later ground votes cannot clear it.
It is **not** an occupied label or a lower physical obstacle threshold. Existing
below-plane, raised-object and full-cell support checks remain unchanged.
No installed mapper or SLAM settings change. Frozen first prototype:
`revision-ca76dfc568d44b4c940aa8c5347c8d3a`.

Before any recording replay, test all 288 partial-face geometries with both
stationary repetition and the previously declared rigid registration sequence,
at every frame, plus expiry and globally shifted flat-plane counterexamples.
Both absolute safety and individual-probe nonregression are required. Flat
coverage loss from an overly conservative residual warning also counts against
the candidate; no parameter sweep or denominator reduction is pre-authorized.

- First prototype: **581 new gates pass**, including all 40 pre-existing
  shallow-face failures and the repeated-face regressions. Broader slope/object/
  support suite: **121 pass / 7 fail**. Four failures are unknown-reason changes;
  three are real over-rejection (resolved 5 cm top and two historical-clearance
  fixtures). Retain this failed prototype and its XML; it is not promoted.
- Targeted refinement, not threshold tuning: distinguish constant offset from
  within-observation residual spread for an independently resolved ground-like
  top. The same 5 mm bound remains; a sub-8 cm measured top is not automatically
  a partial wall. Apply native ambiguity separately after existing positive/
  negative guards, preserving their reason codes while forbidding interpolation
  through the new boundary. Re-run every fixture, not just the failing subset.
- Refined frozen candidate `revision-ddb63145ea9f48b8839ebca67d6ad572` passes
  **all 581 new gates and all 128 existing slope/object/support/clearance tests**.
  This includes the 40 baseline shallow-face cases, all three prior repeated-
  face regressions, and the resolved 5 cm top. XML files are retained in the
  build run. Real-cloud `combined10` replay is next; analytic success is not
  natural-terrain or live-navigation acceptance.
- Direct C++ runner initially exited 139: its compilation omitted the candidate's
  public `TERRAIN_OBSERVATIONS` / `TERRAIN_TWO_PATH` definitions, so its stack
  class layout did not match the shared library. Rebuilding that runner with the
  exact definitions (as the CMake target exports) is required; retained failing
  executable is not overwritten. Python C-API tests allocate inside the library
  and are not affected. This is a runner ABI error, not evidence of an algorithm
  crash on the already-passing C-API fixtures.
- ABI-correct C++ runner exposes an additional reason-precedence bug: native
  ambiguity overwrote `LAYER_CAPACITY` with `NEGATIVE_EDGE` (both unknown).
  Tightened its application to potentially free/interpolatable cells only;
  existing blocking reasons remain intact. Candidate normal/diagnostic pairs
  pass all **108 mirrored-boundary/expiry tests** before this reason-only fix.
  A new frozen build and full C++ rerun are required; no failed attempt discarded.
- Real `combined10` replay **rejects the contextual-offset warning**:
  `revision-7e5f8ea1136d438da9cdbb4a5fa0e627`, all 1,895 chunks retained. Final
  free GT-pose/first/revised = **12,722 / 4,790 / 12,209**, versus baseline
  **15,938 / 10,885 / 13,489**. First-published lost-support attribution includes
  5,815 disconnected cells. Reporting only reduced negative-edge counts or
  revised-map agreement would conceal this severe coverage regression.
- Next isolated refinement: native boundary uses within-observation residual
  after a fitted height gradient capped at the unchanged **0.35 slope limit**,
  rather than offset from the neighborhood plane. Keep the **5 mm residual
  bound**, all-context support requirement, one-cell safety margin and immutable
  lifetime. Constant offsets and locally sloping/curved terrain are not steps;
  vertical/mixed face returns remain unexplained by a bounded gradient. A line
  fitting this model still cannot itself certify ground. New frozen run:
  `revision-ce438ed657014376849fa772e3b2558e`; full fixture gates must pass again.
- Bounded-gradient candidate passes **581 new safety gates / 128 existing
  terrain tests**, plus the full ABI-correct C++ mapper and observation-moment
  contracts. The reason-precedence fix passes. Frozen full `combined10` replay
  is running; these synthetic results do not establish recorded coverage,
  independent physical-geometry safety, or computational acceptance.
- Added opt-in CMake `SLAM_MAPPING_TEST_TWO_PATH=ON` (with `BUILD_TESTING=ON`)
  to link the full C++ mapper contract runner against the experimental target,
  inheriting its public class-layout definitions. This prevents recurrence of
  the manual runner ABI mismatch without enabling the experiment by default.
- Bounded-gradient `combined10` completes as
  `revision-25c2ef3fa91e4c398ef266485c130001`: all **1,895 chunks / 633
  projections**, final free GT-pose/first/revised **15,935 / 13,399 / 15,431**.
  Baseline is **15,938 / 10,885 / 13,489**: first-published gains 2,514 free
  cells (+23.1%) without the previous GT-pose coverage collapse. Candidate's
  GT-pose-free→unknown counts first/revised are 2,724 / 812 (baseline 5,104 /
  2,603). This is pose sensitivity, **not independent terrain recall**.
  Final candidate GT-pose-occupied→free is first **0**, revised **3** (baseline
  both 0): retain this warning, not just the favorable coverage counts. Normal/
  diagnostic library pair passes all **108 boundary/expiry tests**. Next gate:
  independent analytic native course at its frozen worst 2.5 cm grid offset;
  no promotion based on the natural comparison alone.
- Independent 5° analytic native course, worst **2.5 cm phase**:
  `revision-cdf911de9d724139884ae9f0a47337bf`, all **1,314 chunks / 439
  projections**. **12/12 required objects**, **zero positive/dropoff false-free
  throughout the sequence**, unchanged 0.7165% flat false occupied. Coverage
  falls to **93.518% measured / 93.590% inferred**, versus baseline
  **94.454% / 94.526%**. Both fail the unchanged 95% coverage gate. Sampled p95
  also fails; concurrent CMake configuration occurred during this replay, so
  retain that failure and do not use its timing to claim isolated throughput.
  The candidate remains **unpromoted**. Run both standalone resource profiles
  separately from other project workloads; attribute additional unknown ground
  before any further classifier change.
- CMake option verification hit environment issues: unsourced overlay lacked
  `astrobot_interfaces`; `/tmp/repair-install` supplies it but mixes exported
  Jazzy typesupport with Space ROS `std_msgs`, so generation fails. Both build
  directories remain archived. The direct, ABI-correct C++ runner passes; full
  package CMake/CTest verification remains separate and unclaimed.
- Resolved the **runner environment**, not production, using one ROS prefix:
  `AMENT_PREFIX_PATH=/tmp/repair-install/astrobot_interfaces:/opt/ros/jazzy`
  and the same `CMAKE_PREFIX_PATH`, new `cmake-jazzy-build` directory.
  Optional candidate target builds and its **CTest passes**. Default launch/
  overlay setup is unchanged. Removed a diagnostic-only unused-variable warning.
- Standalone resource results (both retained, no timing retries):
  `revision-26e079a62c954086acfc686f85438958` **layer_capacity**:
  **1,206.1 MiB / 67.27 ms p95 / 81.74 ms p99**, fails 1 GiB and 50 ms p95.
  `revision-c886c9cd393b472cb5855243b17eff65` **dense_flat**:
  **382.8 MiB / 57.49 ms p95 / 62.13 ms p99**, fails p95. No concurrent
  project build/replay ran during these two stress workloads. These results
  disqualify production use independently of the coverage failure.
- Fixed-reference coverage attribution is archived with the analytic replay.
  Fine-flat `NEGATIVE_EDGE` count rises **1,680 → 3,664**; unsupported extent
  falls **2,100 → 1,536**, clearance occlusion **1,342 → 1,220**. Other final
  groups are unchanged: net measured-free loss **1,298 / 138,725** reference
  cells. This is an edge-guard tradeoff, not missing returns or a changed mask.
  Added explicit diagnostic bit-8 `native_boundary` decoding and a regression
  contract; unrelated/unknown bits still fail closed. **21 evaluator/diagnostic
  tests pass** (in addition to the 817 mapper/safety tests above).
- The three final revised natural conflicts are adjacent map cells at
  **x=48.25 m, y=42.75/42.85/42.95 m**. GT-pose obstacle heights are
  **9.10/9.77/8.70 cm**; revised-pose heights are **0.95/0.89/1.36 cm**.
  Baseline revised maps marked these unknown (normal edge), candidate marks
  them measured ground. Neither candidate branch changes any **same-pose
  baseline occupied** cell to free at the final frame. This suggests spatially
  displaced evidence, **not** an excuse to ignore the cross-pose conflicts;
  physical natural-terrain GT and pose-uncertainty margins remain unresolved.
- Instrumented analytic replay `revision-c013c9d24b2442339307ba79f502e1c7`
  exactly matches the candidate's **439 per-frame records in both variants
  and all six final arrays**. Its diagnostic attribution confirms **1,984
  fine-flat cells** are rejected by **native-boundary alone**, 965 more by
  native plus pooled-normal causes. Instrumented timings are excluded.
  Updated natural telemetry too: native warnings must not be silently grouped
  as unattributed or eligible for normal-only recovery. Final diagnostic-tool
  run: **24 pass / 5 optional native/candidate skips**, with 192 rejected-old-
  candidate face tests explicitly deselected (current replacement's 576 face
  sequence cases already pass). No skips are counted as acceptance.

#### Plan recorded at the previous checkpoint (execution updates above)

1. Preserve the current candidate and every failed version. Extract the
   native-boundary-only calibration locations and the three revised natural
   conflicts as fixed regression fixtures, retaining their original raw input,
   transforms, timestamps and whole reference denominator. Use independent
   PhysX natural geometry before interpreting cross-pose disagreements as
   obstacle recall. Do not substitute GT pose correction for a mapper fix.
2. Test spatially bounded boundary evidence: retain the observed face location
   and uncertainty rather than treating every residual warning as an isotropic
   3x3-cell exclusion. Ground context must never certify clearance through a
   partial face; no global halo removal or residual-threshold sweep. Require
   the complete 581 safety fixtures, 128 broad terrain tests, 108 mirrored
   boundary tests and C++ contracts before replay. Preserve the existing
   sub-8 cm partial-face protections and immutable expiry.
3. Separately optimize representation/computation **without changing maps**:
   transient same-input moments, compact immutable classification state, and
   reuse same-cell native neighborhood moments instead of rebuilding per layer.
   Exact per-frame replay equivalence precedes both resource profiles; do not
   trade shorter lifetime, skipped input or fewer layers for apparent speed.
4. Only after the worst analytic offset recovers coverage without sequence
   safety regression, run all four declared offsets, the separate `combined15`
   recording and live-cadence/pose-uncertainty gates. These are still development
   gates, not held-out natural-terrain or navigation acceptance. The other
   offsets and `combined15` were **not rerun for this failing candidate**.

### Observation-aware terrain evidence — diagnostic complete, classifier unpromoted

**Outcome:** within-native-observation statistics and spatial-context diagnostics
are implemented and verified. The cell-local classifier fails repeated-face
safety and is removed. A larger native-only neighborhood provides substantially
more usable orientation evidence, but **does not yet change the drivable map**.
The default mapper, GLIM, grid, lifetime and safety thresholds remain unchanged.

User approved trying within-observation geometry after the two rejected pooled-
normal relaxations. Added opt-in, non-installed `layered_terrain_observations`
instrumentation: per-native-input Welford scatter and normal classes, immutable
bucket expiry, within/between scatter attribution. A native input is one rigid
measurement, **not the un-deskewed ROS full scan**. Default target has no added
state or changed classification. Frozen source/diagnostic build:
`revision-b95e65d7706642a39c706c460c9fec51`.

Predeclared separate candidate: only at least two resolved ground-normal
observations, with **no** rank-deficient, nonplanar or steep observations in the
retained layer, may contradict its pooled steep-normal trigger. Actual below-
plane depth, raised objects, support/extent and evidence lifetime remain fixed.
Sparse ring lines cannot qualify. This is a hypothesis, not a promoted fix;
first require diagnostic exact-output equivalence, decomposition/expiry tests,
then check how often the needed evidence exists in frozen `combined10`.
No new captures, GT correction, estimator tuning or safety-threshold changes.

- Cell-local diagnostic `revision-2e94e30805f94507a255da890cc52e84` exactly matches
  all **633 baseline projections / 1,895 chunks**. New observation/expiry and
  existing instrumentation/replay checks: **120 pass / 2 candidate-only skips**;
  with the candidate loaded, all six observation contracts pass. Final first-
  published source observations: **6,622 rank deficient / 4 nonplanar / 367
  ground normals / 200 steep normals**. Only five pooled-wall layers have all-
  ground observations; removing their triggers could address at most **one**
  final lost cell (30 at peak). Revised: zero final eligible cells (13 peak).
  Therefore this strict **10 cm cell-local** candidate cannot materially solve
  the recorded failure. This is evidence scarcity, not a reason to call sparse
  line returns ground. The instrumented scatter decomposition matches independent
  raw-point calculations, and all evidence expires without rejuvenation.
- Initial partial-face invocation accidentally selected the rejected candidate's
  old `registration_thickness` positive test through `-k thick`: **41 fail / 547
  pass**, comprising the 40 known absolute-safety failures plus that incompatible
  expectation that a single ring should override the normal guard. It remains
  archived as `partial_face_gates.xml`; the observation candidate correctly keeps
  that ring unresolved, explicitly asserted in its own new contract. All 288
  per-probe no-regression tests passed. Repeated stationary and jittered face
  sequences are added because a >=2-observation rule cannot be meaningfully
  challenged by single-input tests alone.
- Next diagnostic, predeclared before replay: reuse the bounded **9x9-cell**
  neighborhood for a plane from the **same native input only**, no temporal
  pooling, keeping the 10 cm output grid. Only rank-deficient observations may
  acquire contextual orientation; nonplanar/steep evidence is not overwritten.
  A conservative scatter bound must place every center-observation return within
  5 mm of the contextual plane (the existing fit's minimum sigma). No outlier
  dropping, changed ground band or classifier use of this context. New frozen
  diagnostic: `revision-309115d787ef46ab960d1df26d27ea80`; test/replay pending.
- Repeated-face test is decisive: **195 pass / 3 fail** for the cell-local
  candidate. All three new false-free cases are 2 cm visible positive faces,
  rotated 0.17 rad, under the declared millimetre/centimetre rigid registration
  perturbations. Single-input nonregression was insufficient. **Reject this
  classifier too**; archive `revision-b95e65d7706642a39c706c460c9fec51/repeated_faces.xml`.
  Ground normals of observed patches do not prove that adjacent cell extent is
  clear of a partly observed face. No natural candidate score or resource
  acceptance will be used to override this failure. The neighborhood experiment
  remains diagnostics only; its purpose is to measure geometric evidence
  availability, not authorize removing an obstacle/boundary guard.
- Found and fixed an evaluator provenance weakness: source hashes were collected
  only at replay completion, so an edit during execution could misattribute the
  loaded code. Startup hashes are now frozen and rechecked before completion;
  a source-mutation regression joins the nine replay contracts. I edited an
  explanatory output field during the first neighborhood replay, exposing this
  problem. That run (`revision-baf49c3ae2e1469aa5a82fc82b4cdc50`) is explicitly
  marked failed/ineligible and retained, despite matching map outputs. A fresh
  guarded replay is required; no favorable diagnostic counts are promoted from
  the ineligible attempt. Final source/evaluator snapshot and builds:
  `revision-36f7d08aea1648c883d860d0e283a825`. Rejected classifier hooks are
  absent; only opt-in diagnostics remain.
- Minimal repeated-face reproducer archived at
  `revision-2b45a65fdc444956844fa66ea31ecf5e`: after the second rigid input, three
  new unsafe fine probes in one output cell; after the third, 15 in five cells.
  Responsible pooled-normal sources each have two/three individually resolved
  ground observations and no unresolved/steep ones. Thus the new criterion is
  actually exercised, not failing because of a counter bug. A plane-normal
  certificate alone cannot remove the extent/registration safety margin around
  a partly observed face. Raw returns, both map sequences, source telemetry and
  fixed probe cells are retained; GT probes never enter either mapper.
- Final default/diagnostic regression: **281 pass / 194 candidate-only skips /
  four existing strict xfails**. Both direct C++ runners pass (full retained
  mapper contracts and new observation-moment contracts). This is not a green
  candidate safety suite: its three repeated-face regressions remain failures.
- Guarded neighborhood replay **passes exact equivalence and provenance**:
  `revision-c262893da62f4e63a09afd0a80d41631`, all 1,895 native chunks / 633
  projections. Final first-published contextual source observations:
  **2,423 unresolved / 4 nonplanar / 4,566 ground / 200 steep**; 990 pooled-wall
  layers have repeated ground context without competing observation classes.
  **2,740 of the 3,953 lost cells** have only those contextual sources and no
  direct below-plane trigger. Revised: **1,722 of 2,027**. These are potential
  attribution targets, **not recovered cells or GT terrain recall**. All six
  map outputs remain identical to baseline. Between-observation scatter includes
  sampling geometry as well as pose effects and is not a calibrated uncertainty.

**Next implementation decision:** retain 10 cm output cells, but decouple their
size from normal-estimation scale. Use native-only local surface context to
estimate relative heights, while retaining a separate per-observation edge/
partial-face evidence path. Never let a plane fit or a ground vote clear that
edge evidence or certify an adjacent cell's full extent. Preserve registration
uncertainty in boundary clearance rather than asking pooled normals to encode
both terrain shape and temporal misalignment. First reproduce the three new
repeated-face counterexamples and 40 baseline shallow-face failures under that
two-path model; then re-evaluate the same frozen motion fixtures, independent
analytic/native safety cases, four grid phases and both resource profiles.
The 9x9 diagnostic is bounded but has no live compute acceptance. No additional
candidate/holdout/resource qualification was run after the safety rejection.
All runs started in this continuation are finished; simulator/navigation stayed
off. Root and OmniLRS whitespace checks pass. No robustness acceptance claimed.

### Traversability robustness — active continuation

**Current decision:** diagnostic infrastructure and stronger regression fixtures
are implemented; **neither classifier candidate is safe and neither is retained
in the runtime source**. Default mapper and GLIM settings remain unchanged.
Do not read the candidate experiments below as deployed improvements.

User accepted the controlled-motion SLAM results as sufficient to prioritize
traversability. Estimator settings and all three completed motion recordings
remain frozen. No new simulator acquisition or navigation is needed for the first
diagnostic/candidate stage. Extended natural replay with explicit instrumentation
mode: partition propagated `NEGATIVE_EDGE` losses into below-plane, pooled-normal,
or both. Instrumented runs must reproduce **every six-layer projection hash**,
input hash, grid and schedule from a frozen uninstrumented reference; timings
remain excluded from acceptance. Eight replay contracts pass. Diagnostic library
and source snapshot: `revision-2e4ce3a0f481443486f3b0436ed89103`; source-level
attribution and boundary regression completed before the candidate replay.

Diagnostic replay `revision-541746cd527746df84de8e3a60b87fbb` reproduces every
baseline projection exactly. All **108 boundary/expiry contracts pass** against
the frozen library. Final `combined10` first-published negative-edge losses:
**3,215 normal-only / 291 below-plane-only / 447 both**. Revised: **1,946 / 19 /
62**. Main source is pooled covariance, not direct below-ground evidence.

First isolated candidate: `layered_terrain_resolved_normal`, explicitly excluded
from default build/install. It additionally requires the smallest covariance
eigenvalue <=0.1 times the second before a PCA direction can assert a wall.
This is a predeclared normal-resolution test, not a wider ground/obstacle band.
Below-plane, raised-obstacle, connectivity and full-cell support guards remain
unchanged. Added sparse-ring, centimetre registration-thickness fixtures across
four grid phases and three rotations. Candidate is **not promoted**; boundary,
small-object and frozen moving replay gates must decide whether it is useful.

- Candidate frozen source/libraries: `revision-508c4b550db04f4882dd77866490bc22`.
  **128 existing slope/small-object/support tests, 108 mirrored-boundary/expiry
  diagnostic tests, 12 jittered-ring tests and 72 newly added rough-wall/hole
  tests pass**. The latter perturb real face returns by 3/8/15 mm, across four
  phases and three rotations, to challenge the same volumetric-covariance case
  on real hazards. These are analytic contracts, not general safety proof.
  First new-fixture attempt failed on projection timestamp rounding
  (`1 + 7*.1 > 1.7`); fixture now uses the exact last integrated timestamp.
  No mapper time guard or geometry threshold changed. Frozen `combined10`
  replay is next; independent natural terrain truth remains unavailable.
- **First normal-confidence candidate rejected.** Its `combined10` replay
  (`revision-bb2c259d437f432d9bbc09da46dfbd25`) integrates all 1,895 chunks and
  increases final first-published free cells from 10,885 to 11,458; revised
  13,489 to 14,153. Its own GT-pose map also changes (15,938 to 15,971), so these
  are not fixed-truth recall gains. Expanded visible-face tests (2/4/6/10 cm,
  same phases/rotations/noise) yield **45 failures / 255 passes**, including new
  false-free hazards and pre-existing baseline shallow-face misses. Increased
  coverage cannot override these failures. The rejected runtime macro/target
  was removed; frozen source and binary remain archived for reproduction.
- Second isolated candidate `layered_terrain_local_normal_uncertainty`: a poorly
  resolved steep normal withholds its own cell as `UNRESOLVED_NORMAL` and cannot
  supply ground-support witnesses; only the extra one-cell wall expansion is
  withheld. Actual below-plane, resolved-wall and raised-obstacle guards remain.
  This is a new experiment, not a fix claimed safe. Frozen source/build:
  `revision-543b3beb14614a42870f3f1c68f9aae3`. The new partial-face fixture also
  compares baseline and candidate independently so baseline failures do not hide
  additional candidate regressions. Production remains unchanged.
- Full 288-case partial-face audits (four visible heights, three thicknesses,
  three rotations, four phases, positive/negative faces) are saved beside each
  frozen candidate as `partial_face_audit.json`, with fixture/library hashes.
  Baseline fails **40** cases. Candidate 1 fails **45**, introducing newly free
  hazard probes in **16** cases; candidate 2 fails **40**, but introduces newly
  free probes in **10** cases. The equal aggregate failure count hides real
  regressions. **Candidate 2 is also rejected; its runtime macro/target removed.**
  Added a separate per-probe no-regression gate; pre-existing baseline failures
  remain explicit failing safety tests, not blanket xfails or altered labels.
  Rejected-candidate reason 13 is recognized by diagnostic readers solely to
  preserve readability of archived experiments; the retained mapper emits 0–12.
- Candidate 2's already-started `combined10` replay completed and is retained as
  a **rejected experiment**, `revision-fbf1ad0606ad4fe0879f674bc4e8043d`: final
  free cells 15,944 GT-pose / 11,369 first-published / 13,990 revised. No failed
  hazard gate is overridden by these counts. No further speed/holdout/compute
  qualification was attempted for either unsafe candidate.
- Final retained-core check: **275 tests pass, four pre-existing strict xfails**
  (time-varying +/-5 cm registration interpreted as obstacles). Default C++ source
  and header match the pre-experiment diagnostic snapshot byte-for-byte. Root
  and OmniLRS `git diff --check` pass. New absolute/per-probe partial-face gates
  intentionally remain failing when run against these rejected candidates;
  the normal host suite skips native candidates unless their libraries are set.
  No simulator restart, robot movement, mapper deployment or SLAM change occurred.
- Final separated-gate run against candidate 2: **538 pass / 50 fail** (40
  absolute-safety failures plus 10 per-probe regressions). Archived XML:
  `revision-543b3beb14614a42870f3f1c68f9aae3/partial_face_individual_gates.xml`.
  The final fixture source is saved as `normal_confidence_gate.py` alongside
  both candidate builds. These failures are the rejection evidence, not a green
  regression suite. All work started in this continuation has stopped cleanly.

**Next implementation route, revised by these counterexamples:**

1. Retain per-native-observation (or short rigid submap) surface statistics before
   cross-time accumulation. Separate within-observation scatter from between-pose
   displacement; a pooled eigenvector alone cannot distinguish registration
   thickness from a rough, partly observed face. Bound additional state by the
   existing expiry and memory cap. Start with an offline diagnostic, not a new
   authoritative free-space rule.
2. Require positive traversable-surface evidence before recovering a cell or
   removing boundary support exclusion. Ambiguous/single-ring returns cannot
   prove the visible return is the obstacle top. The 2 cm visible-face failures
   expose this pre-existing limitation; they do not justify lowering the 8 cm
   physical obstacle requirement or claiming detection of an unseen object.
3. Test the new 288-case hazard family plus jittered rings, inclined 8 cm objects,
   holes, expiry and worker determinism. Gate both absolute safety and individual
   new false-free probes; never compare only failed-case totals. Then replay
   `combined10` and the separate `combined15` stress with one frozen candidate,
   followed by all four original analytic-native GT offsets and both resource
   profiles. These latter promotion gates were **not run for rejected versions**.
4. Add the fixed dense natural PhysX reference before calling any natural map
   result recall/coverage acceptance. Existing GT-pose comparisons measure pose
   sensitivity only. No estimator retuning, radius cropping, ground-band widening,
   fixture dropping, or safety-gate relaxation was used in these experiments.

### Controlled natural motion — execution started

**Current result:** relative-surface acquisition checks and a bounded turn-control
fix are implemented. Full natural `turn15`, `combined10` and `combined15` fixtures
pass native/GT frame audits and the declared GLIM pose targets (6.05–7.03 cm XY /
8.75–10.05 mm Z p95). `combined15` also passes the full 200 ms pose-availability
diagnostic. `turn45` fails its skid envelope; slow straight runs fail the wall-
heartbeat requirement. Natural-cloud replay exposes far-field negative-edge
over-rejection despite passing poses. Natural terrain recall and general motion
robustness are **not proven**. The
chronological entries below retain superseded hypotheses and every failed attempt.

**Current continuation:** user authorized separating local surface discontinuity
from absolute incline. Acquisition policy `relative_surface_v2` uses both endpoint
terrain tangents to check each 5 cm edge (2 cm residual limit), a 9x9 local plane
for detrended roughness (2 cm maximum), and the retained mapper's separate 0.35
incline cap. That cap is an experimental criterion, not vehicle tip/traction
certification. Dynamic support checks also compensate for the sampled tangent.
Slope policy is hashed into the route; prior failed runs remain failed. No SLAM
parameters, mapper thresholds, recall denominators or watchdogs were changed.
Read-only diagnosis of the prior 3,154-point tile: maximum relative jump 5.03 mm,
detrended roughness 12.79 mm; this motivates a fresh capture, not retroactive
acceptance. Added smooth incline, inclined +/-8 cm step, roughness, excessive
incline, missing-normal and reordered-grid contracts. Moving SLAM and natural
terrain acceptance remain open.

- Relative-surface implementation/regression: **183 sourced-overlay tests pass**.
  New live straight run: simulator `revision-12c059f2e9584a20af58c08df9d8db6d`,
  recorder `revision-3c29a2e05b9e4933abbd8cc3820d4503`. Full floor and rock-volume
  clearance pass. Actual frame-zero GT comparison passes: **0.349 mm translation,
  0.00224 degrees rotation**. Route is running; no full-sequence accuracy or
  traversability pass is claimed from the initial state alone. Offline audit
  rechecks the saved clearance tile and allows only 1e-9 numerical differences
  across Isaac/ROS LAPACK builds, not changed safety thresholds.
- That first moving run **failed near the return endpoint** (about 12 cm short
  of origin): a generation-2 GC pause of **704.7 ms** during native diagnostics
  exceeded the unchanged 500 ms motion heartbeat. Independent recorder clock,
  request and IMU freshness guards also failed; localization health remained
  healthy at the stop. Both failed runs remain ineligible for acceptance.
  Source inspection found that, despite streaming optional hit identities, the
  native recorder still retained every decoded pose/residual/index object graph.
  Changed long-term frame retention to immutable JSON strings and streaming final
  output; only the existing bounded 16-frame association window stays decoded.
  GC stays enabled and thresholds unchanged. New contracts cover 2,400 retained
  records, exact decoded identity, mutation isolation, untracked strings and the
  20,000-frame cap. This is a targeted heap-retention fix, not yet a live timing pass.
- Encoded-frame retry: simulator `revision-6340935b70df4ae1afc1e138b043d77b`,
  recorder `revision-6cc21be1188642edb2530f48ccc08414`. **Still failed**: 783.1 ms
  generation-2 collection near the return endpoint. Encoding preserves evidence
  and reduces tracked retention but is **not sufficient** to eliminate the pause.
  Next isolated change: explicit normal generation-2 collection while stationary,
  after setup and before request/GLIM/motion authority. Hypothesis: startup's
  pending long-lived graph still triggers the later full collection; this has not
  yet been demonstrated. The checkpoint records GC stats/time and refuses any
  active lease, prior movement, started capture or disabled GC. Runtime GC remains
  enabled with its unchanged thresholds; no stop or acceptance gate is relaxed.
- Frozen mapper check: **20 relative-terrain fixtures pass** (five inclines up to
  slope 0.30, four grid phases, each with and without a true 8 cm object). Smooth
  inclined support stays free and the object stays occupied. No C++ mapper edit
  or safety-threshold change was needed for these analytic cases. Combined
  mapper/acquisition/native contracts: **83 passed**. This does not close the
  existing cross-time registration or natural-terrain recall gaps.
- Startup-checkpoint attempt also **failed**: simulator
  `revision-8599d6f0be7c410fbab2b186dcf3a5f9`, recorder
  `revision-6de428b17736459da9bef8ce67d2550d`. Initial 647 ms collection occurred
  before authority, but a later 839 ms generation-2 pause still stopped motion.
  Neither heap change establishes long-route timing robustness. Next: inspect
  retained simulator object types before authority and run the separately
  predeclared `turn15` fixture for actual motion/SLAM evidence. This reorders
  diagnostic work; it does **not** replace or pass the failed straight fixture.
- The `turn15` attempt (simulator `revision-ba484248ad1f41658bdc9439432d4a03`,
  recorder `revision-565347844dc7433fb6013c72af381061`) failed **before motion**
  in the new heap profiler: an Isaac extension type exposes non-string
  `__module__` metadata. Made diagnostic type names explicitly string-safe and
  added that regression. No rotation evidence or acceptance is claimed.
- Stronger source-level GC finding: `TerrainManager.buildGrid` retains
  **23,976,006 Python integer indices** for the 40 m / 2 cm terrain, in one
  GC-tracked list. Converted the completed buffer to `np.int64`, exactly the
  dtype previously created in `renderMesh`. No vertex, UV, index value, ordering,
  terrain seed or collider setting changes. Added actual-method contracts for
  four rectangular grids, exact topology/vertices/UVs and untracked storage;
  the simulator manifest now includes terrain-manager source provenance.
  Live pause reduction and renewed long-route qualification are still pending.
- Numeric mesh-buffer run: simulator `revision-a5383e69adfe4eae8e1dcf00242c208b`,
  recorder `revision-56b9a8b2bd654e9ba0a208aa87f66c0c`. Exact mesh-storage/acquisition
  contracts **67 passed**. Preflight GC fell to 448 ms; in-motion GC was 461 ms,
  but surrounding work still exceeded the unchanged 500 ms heartbeat and the
  route **failed** near origin. This reduces a confirmed GC traversal burden but
  does not prove long-duration timing robustness. Stopped repeating the slow
  route with marginal timing changes. Proceeding with the original short rotation
  experiments to measure motion-induced SLAM error; they cannot substitute for
  long-duration qualification. All four moving straight attempts remain failed.
- Corrected `turn15` attempt: simulator `revision-2fed8145ed2e4d578f6f43593dd19f14`,
  recorder `revision-8dc9873b52c8497381048849e9469ade`. **Failed on the independent
  stall guard**, not GC: at the negative endpoint, proportional angular command
  decayed to 0.0127 rad/s just outside the unchanged 0.012 rad arrival tolerance.
  Added a hashed, rotation-only 0.04 rad/s minimum drive command, still bounded by
  the original 0.10 rad/s cap; command is exactly zero inside tolerance. No stall,
  arrival, clearance, lease or footprint limit is relaxed. Signed endpoint and
  lossy-actuator contracts cover this acquisition-controller fix; live validation
  remains pending. This failed recording is not moving-SLAM acceptance.
- Bounded-turn retry **completed acquisition**: simulator
  `revision-ac4e32e206f14159a575fbd0ef1e78ec`, recorder
  `revision-d4afb8c2ebfa44428ab6b1ff2322143f`, trajectory export
  `revision-1d62f1bb280847899398a5429ca2cf79`. All three targets and postroll
  completed; 970 native chunks over 32.3 s. Turn/mesh host contracts **59 pass**.
  Export passes identity/receipt checks; full geometry/pose qualification is next.
  The simulator and estimator are stopped before offline scoring. A completed
  short rotation does not close the failed straight-route timing gate.
- `turn15` full audit **passes** (`revision-ac6fc339993944bf893ddbeae283ede9`):
  all 970 ticks, 124,160 first-hit samples, zero missing, maximum residual
  **0.1304 mm**; exact ROS-GT/native pose agreement and 243 frozen dependencies.
  Fixed-frame accuracy (`revision-4468cc4f3a4441a2817ca2a381eb1732`) passes
  unchanged p95 targets overall, in each observed phase, and first/last 3 s.
  First-published base error: **6.647 cm XY / 1.005 cm Z / 0.257, 0.137,
  0.353 degrees roll, pitch, yaw**. Max XY is 8.428 cm; max roll 0.316 degrees
  (the declared roll gate is p95, not max). Final revised XY p95 is 7.355 cm,
  so revision is not uniformly better in every component. No fitted alignment.
  Surface diagnostic `revision-23025e54061740258fdc882cbe1d5b2e`: ground-normal
  p95 **1.035 cm**, max 3.404 cm; three observed large rocks have placement p95
  **7.326 / 6.863 / 6.831 cm**. These are observed placement, not detection recall.
- Strict 200 ms deadline diagnostic initially failed
  (`revision-ce43275798d84fb0ab388d61de0d0188`). Added full-denominator missed-
  deadline telemetry, with **10 contracts passing** and no deadline tolerance.
  New diagnostic `revision-50f25dc1738c4f52aec1618ac5646356` finds exactly **1/970**
  unproven samples: the last tick's next-clock upper bound exceeds 200 ms by
  **10 ns**. This is not evidence of a meaningful physical latency regression;
  it also is not strict deadline proof. Neither sample trimming nor rounding
  is used to turn the result into a pass. The original failure is preserved.
  Next acquisition: separately declared `turn45`, unchanged estimator/config.
- Added natural-cloud mapper isolation tool `tools/replay_natural_terrain.py`:
  every audited native chunk, identical filtering/grid/30 s lifetime/10 Hz
  projections, GT-pose versus first-published/final-revised pose branches.
  Saves final six-layer maps, all projected class counts/hashes and the complete
  4x4 class-transition matrix (including unknown and cost-20 support). **Six host contracts pass**,
  including exact input reuse and failed/changed-input rejection. This diagnostic
  can attribute pose-induced map changes but cannot measure natural recall:
  the GT-pose mapper is not a terrain oracle. Dense independent natural-terrain
  labels and full-route/causal acceptance remain required.
- `turn45` capture **failed safely**: simulator
  `revision-1cef6d4c606c4df4a279fdfbd6eed22d`, recorder
  `revision-04f22073e2f8424c845b79ad3d328fee`. During the final return turn,
  skid drift crossed the original **25 cm** envelope at sim stamp 82.733 s.
  This is a driver/traction envelope failure, not a measured GLIM divergence.
  No in-motion generation-2 GC was recorded. Do not enlarge clearance or score
  a convenient prefix as a completed rotation. Both processes were stopped.
- Initial natural mapper replay `revision-5e23253fb08f4ac99ca7be01f775c214`
  failed before integration: USD range metadata is string-valued. Explicitly
  convert and validate numeric limits; regression now uses actual string form.
  Failed attempt retained, same immutable `turn15` inputs used for the retry.
- Replay retry `revision-bd0ff77902a644078f0089b04ad113b3` exposed an omitted
  output class in the new diagnostic: retained terrain uses cost 20 as well as
  0 for supported ground. Corrected the reader to the existing mapper/scorer
  contract `[-1, 0, 20, 100]`, retaining both free classes separately in the
  full transition matrix. Added exhaustive class-pair accounting. Mapper output
  and thresholds are unchanged; failed diagnostic artifacts remain preserved.
- Natural mapper isolation completed: `revision-90f8ad61fa504e49bc66a2b0b9d1b5ed`,
  frozen mapper `revision-bf1fde8ac66a45979c61a8404232e8f0/liblayered_terrain.so`.
  All 970 chunks enter all three branches. Final counts over the fixed 40,000
  cells: GT-pose **11,585 free / 281 occupied**, first-published GLIM **11,262 /
  488**, final-revised GLIM **10,530 / 335**; remainder unknown. Of GT-pose-free
  cells, first-published changes **1,492 to unknown and 2 to occupied**; revised
  changes **1,249 to unknown and 1 to occupied**. Most additional occupied cells
  were unknown under GT-pose mapping, not certified free ground. No final
  GT-pose-occupied cell becomes free in either branch, but this is **not obstacle
  recall or a sequence-safety pass**. Improved angular smoothness does not
  automatically improve supported-space coverage. Natural oracle/causal gates
  remain open. Starting the separately declared `combined10` acquisition.
- Full-sequence read-only check of that same isolation report: **324 projections**,
  zero GT-pose-occupied to SLAM-pose-free changes in either branch; maximum
  GT-pose-free to occupied changes **4 first-published / 1 revised**. These are
  mapper-to-mapper consistency counts, never terrain safety labels. Sourced
  acquisition/seed/ROS supervisor/native/accuracy/replay regression **199 passed**;
  `git diff --check` passes. `combined10` is acquiring with unchanged frozen SLAM.
- `combined10` **completed acquisition**: simulator
  `revision-620cf652bf494d1285851f018f7d1be0`, recorder
  `revision-08dbf22b74b048f3838a14a368843da8`, trajectory export
  `revision-97b833069f1c455ca6fea5f29e7cc8be`. Both 2 m translation legs,
  origin turns and postroll completed within unchanged guards. Export passes;
  geometry/pose/surface qualification follows with simulator stopped.
- `combined10` audit **passes** (`revision-16ce4fd61f34474095571b1fbc3bd6da`):
  1,895 native ticks, **242,560** first-hit samples, zero missing, maximum
  **0.1500 mm** residual; exact ROS-GT/native snapshots and frozen dependencies.
  Accuracy `revision-42dce4c033064a28843f0d08b33d0209` passes overall, every phase,
  and first/last 3 s for both pose modes. First-published p95: **6.045 cm XY /
  8.746 mm Z / 0.126, 0.147, 0.369 degrees roll, pitch, yaw**; maximum XY
  8.582 cm. Observed surfaces `revision-e70abb6d419e4526852f912847ef6c8c`
  include five large rocks; whole observed-set normal-error p95 is 9.79 mm.
- Deadline attempt `revision-f3aee405744a45f2b1fc9908c49fcf47` retains one
  10 ns upper-bound excess plus two **diagnostic horizon errors**: last revision
  receipt was mistaken for recording end, although /clock continued. Export now
  retains the independently checked last clock; deadline queries may use it only
  through hashed export provenance. Known poses need no later revision to remain
  available. No extrapolation, deadline rounding or sample removal. **12 deadline/
  exporter contracts pass**. Existing artifacts are not rewritten/requalified.
- `combined10` terrain sensitivity `revision-06e66ff9f7b44dc99344a29f2a30c80b`
  exposes a **real remaining mapping limitation despite passing pose p95**.
  Final free cells: **15,938 GT-pose / 10,885 first-published / 13,489 revised**.
  First-published sends **5,104 GT-pose-free cells to unknown**, of which **3,953**
  carry `NEGATIVE_EDGE`; 99 become occupied. Revised reduces these to 2,603
  unknown / 2,027 negative-edge / 26 occupied. Existing 3 cm below-plane and
  pooled-normal wall triggers plus one-cell dilation warrant source-level audit;
  do not remove them because they also guard genuine holes/walls.
  Fixed spawn-distance attribution is explanatory, **not a new acceptance mask**:
  retained GT-pose-free counts are 652/675 within 2 m, 1,188/1,195 at 2–3 m,
  3,085/3,298 at 3–5 m, 5,465/9,024 at 5–10 m, 345/1,746 beyond 10 m.
  Loss is predominantly far-field; do not trim those cells to claim a pass.
  A predeclared local-map range/uncertainty policy is a potential future design,
  not retroactive qualification. Actual natural terrain recall is still unscored.
- `combined15` **completed acquisition**: simulator
  `revision-ca8d5ce1d3c44dc1b03b6abfe0d6a4c8`, recorder
  `revision-130c42fac14c43df91f7e7c5f9ef0172`, export
  `revision-2d9d4bc4a3ab4cd7b3fda8c1fc83efb3`. Faster 0.15 m/s out-and-back
  and both origin turns pass the unchanged acquisition guards. Fresh export
  includes the audited recording clock tail; full qualification is next.
- `combined15` audit **passes** (`revision-930d21f0626e43659d4f73aab4fdb2eb`):
  1,586 ticks, **203,008** first-hit samples, zero missing, maximum **0.1152 mm**
  residual. Accuracy `revision-56e489b9b41641e99f906a434848ce2e` passes overall,
  every motion phase and first/last 3 s for both pose modes. First-published p95:
  **7.028 cm XY / 9.101 mm Z / 0.181, 0.191, 0.392 degrees roll, pitch, yaw**.
  Maximum XY is **10.423 cm**, max pitch **0.362 degrees**; these do not violate
  the declared p95 gates but must not be hidden by reporting only percentiles.
  No sustained >10 cm episode. Surface report
  `revision-719b86df604d448dbac2d5303b989d60` includes five observed large rocks;
  whole observed-set normal-error p95 **1.256 cm**, max **9.832 cm**.
  Full 200 ms deadline report **passes** on this fixture
  (`revision-7cb8a09faba24e9db07b4ebcb147025d`): all 1,586 associations, no dropped
  samples/tolerance/extrapolation; fixed-deadline poses also pass accuracy.
  This is clock-bounded offline availability, not a deployed terrain scheduler.
  Final regression: **222 sourced-overlay tests pass**, including 20 frozen-C++
  slope/object cases and the new horizon/metadata/class-attribution regressions.
- `combined15` terrain replay `revision-b1c4ef7ff69e4d40a9d4c4be00d1275f`
  **does not establish traversability robustness**. All 1,586 chunks / 530
  projections retained. Final free counts: **15,867 GT-pose / 10,332 first-
  published / 12,025 revised**. Of GT-pose-free cells, first-published changes
  **5,306 to unknown / 380 to occupied**, revised **3,851 / 117**. Negative-edge
  reasons explain 3,461 / 2,799 of those unknown cells, respectively. Full-sequence
  peak GT-pose-free to occupied disagreement is much worse than the final frame:
  **1,884 first-published / 364 revised** cells. Peak GT-pose-occupied to free
  is **0 first-published / 3 revised**. These comparisons are not independent
  collision truth, but forbid treating pose-target passes or revised-map
  smoothness as terrain safety proof. Original native inputs and every projected
  map hash are retained for exact reconstruction. No mapper candidate promoted.
  All acquisition/estimator/replay processes are stopped; unrelated dev containers
  remain running. Root and OmniLRS `git diff --check` pass. The next gates below
  remain **unimplemented/open**, not completed by this checkpoint.

### Next implementation gates after these controlled-motion results

1. Freeze the completed motion fixtures and estimator unchanged. Use the
   `combined10` far-field losses to create bounded regression reproducers;
   distinguish below-plane depth from pooled-normal triggers with the existing
   diagnostic build. Require exact uninstrumented/instrumented six-output replay
   agreement before attributing a failure to a particular trigger.
2. Test a **registration-aware boundary model**, not a wider global height band:
   investigate normals/support calculated within individual observations before
   pooling pose-jittered returns, and distinguish temporal pose uncertainty from
   independently observed physical height discontinuities. Keep unresolved cells
   unknown. Require the original inclined 8 cm objects, shallow/rotated hole and
   wall-sliver fixtures, all four grid phases, expiry, worker determinism and
   unchanged full-sequence native safety gates. No candidate is promoted yet.
3. Add an independent dense, fixed natural PhysX reference for actual terrain
   scoring. Current first-hit identities and safe-route samples establish wiring
   and observed placement, **not full-area drivable-ground/obstacle labels**.
   Report the unchanged full region; any future near-field mapping design must
   be predeclared and evaluated on a new reference/holdout, not obtained by
   deleting the far-field failures from these results.
4. Re-establish long-duration acquisition timing without suppressing GC or
   weakening the 500 ms stop. Address simulator heap/critical-path work, then
   retry the original slow route. Large-turn control must remain inside an
   independently surveyed envelope; the failed +/-45-degree skid run cannot
   justify silently enlarging that envelope.
5. Only after those gates: bounded causal mapper integration/revision handling,
   resource profiles, inflation-aware planning, repeat motion/seed holdouts, and
   recovery tests. Current passes do not certify closed-loop navigation or
   catastrophic-SLAM recovery. Do not claim the user’s robustness objective done.

User approved proceeding with one-time GT initialization; microscopic replay
differences and normal-start estimation are **not blockers** for this stage.
Input identity, timing, frame and safety gates remain mandatory. Frozen GLIM,
RTK noise, 5-degree mount and pose-accuracy thresholds stay unchanged.

Execution order: (1) opt-in live GT seed + first-output verification; (2) dense
natural-ground/swept-footprint clearance; (3) separate slow straight, ±15-degree,
±45-degree, and combined 0.10/0.15 m/s acquisitions; (4) full-sequence pose,
surface and availability checks; (5) terrain evaluation only after moving pose
qualification. No unguarded manual/reverse movement or calibration-floor assumption.
Preflight: NVML and CUDA allocation pass; external archive mounted; no simulator
or navigation stack active. Implementation and unit contracts are in progress.

- Implemented experiment-only `shadow_seed_path`: one immutable GT world-base to
  fixed-map IMU conversion, forbidden outside shadow mode and forbidden on restart.
  Recorder verifies frame zero against measurement-time GT (preceding stationary
  second, unchanged 1 cm / 0.1-degree first-output gate) **before any motion grant**.
- Added separately declared natural routes, 5 cm collision-floor survey over the
  whole inflated turn/travel envelope, full-region rock overlap checks, per-tick
  footprint/support checks, signed bounded commands, 1.5 s stall detection and
  independent 0.5 s heartbeat/lease stop. Floor survey is finite sampling, not a
  continuous mesh-support proof; no calibration-floor Z assumption or GT terrain
  labels enter SLAM. Straight route is 2 m out/back at 0.05 m/s.
- Host adjacent contracts: **66 passed**. Rebuilt experiment overlay including
  the new core seed module; sourced ROS/pure seed/driver/lease/supervisor contracts:
  **77 passed**. Production defaults remain unchanged.
- Starting the first slow straight acquisition with the frozen CPU/absolute
  plugins and unchanged RTK settings. Motion/terrain acceptance remains pending.
- First motion attempt **stopped before issuing any command or starting GLIM**:
  simulator `revision-45cf6038fe664ea4adcba244181af652`, recorder
  `revision-597c80efcbf0401ca50f8d0e55dc4dba`. The broad sweep included ground
  steeper than the declared 0.15 slope. Stopped recording after that terminal
  preflight failure; artifacts remain ineligible. The initial survey incorrectly
  required a full-spin envelope along a straight-only route. Corrected its bounds
  to contain every admitted heading/lateral/endpoint footprint (exhaustive sampled
  corner contracts plus conservative angular padding), without relaxing slope,
  footprint or control limits. Added complete failed-floor coordinates/normals
  and a retained clearance tile so further route failures are diagnosable.
- Second attempt also **stopped before any motion or estimator launch**:
  simulator `revision-6ace4b3b01fa4f208056026b3c328924`, recorder
  `revision-f50e910ab15246b480f701ed69a5338c`. Full corrected sweep has 3,154
  samples; 121 exceed the newly introduced 0.15 acquisition slope limit. Local
  slopes are 6.91 degrees median, 8.45 p95, 10.17 maximum. Some rejected samples
  are actually inside the starting inflated footprint, so this is no longer just
  an oversized survey. All sampled first hits are terrain; full-volume overlap
  validation was not reached. **No moving SLAM result exists yet.**
- The 0.15 guard was newly chosen for this harness, not inherited from vehicle
  validation. The retained terrain algorithm uses `max_slope=0.35` (~19.3 degrees).
  Asked whether to use that existing terrain slope criterion with the other
  guards unchanged, or retain 0.15 and select a flatter test location. Do not
  silently relax the acquisition criterion or present this as a GLIM failure.
  Simulator stopped; failed recordings sealed. While awaiting that choice,
  continuing offline guard/audit regression work. Steps 3–5 remain gated.
- Final sourced-overlay regression: **170 passed** (natural acquisition/geometry,
  GT initialization, ROS recorder/lease, supervisor, paired replay, accuracy,
  surface and deadline contracts). Added malformed-pose/NaN-floor rejection,
  route/safety-parameter hashing, seed/runtime/clearance/target/command audit
  mutation tests, and offline first-frame verification against the bag. Accuracy
  reports now include first/last 3 s; deadline diagnostics accept qualified live
  exported trajectories as well as replay-local trajectories. These are harness
  contracts, not live moving-pose qualification. `git diff --check` passes.
  No acquisition, estimator or simulator process is left running. Step 1 is
  implemented/tested but still needs live verification; step 2 has measured the
  clearance blocker; steps 3–5 have not passed or been waived.

### Generalization stage — independent natural scene [stationary GT-start passes; motion pending]

**New result:** the frozen estimator passes the unchanged pose targets on a
separate seed-43 natural-rock scene with a one-time exact GT start. First-published
base-pose p95: **3.91 cm XY, 4.06 mm Z, 0.025/0.059/0.342 degrees roll/pitch/yaw**.
Two independently identified ~0.8–1.1 m rocks have **4.56/4.58 cm** observed-point
placement p95 (773/508 sampled returns); ground-normal error p95 is **3.64 mm**.
These are stationary, observed-surface results—not moving terrain recall or mission
acceptance. Natural live normal-start absolute accuracy still fails. No estimator
parameter was tuned on the new scene. Both replay repetitions pass accuracy;
they agree numerically but **are not bit-identical** (details below).

Implemented this stage: guarded natural-scene acquisition, exact ROS-GT/physics
cross-checking, loaded-plugin/config attestation, streamed first-hit evidence to
avoid GC stalls, and observed-rock surface diagnostics. Chronological evidence
and preserved failures follow.

- Frozen the passing curvature/target/association candidate; no additional tuning
  on the analytic course. Host NVML and CUDA context/allocation preflights pass;
  external `/dev/sdb1` archive is mounted. No production stack is running.
- Implemented a separate **seed-43, 30 s stationary natural-terrain acquisition**,
  preserving the development fixture's 5-degree LiDAR mount and fixed map origin.
  It never teleports, constructs calibration geometry or publishes commands.
  Every command publisher is forbidden. It requires stationary USD/physics/ROS
  sensor-TF agreement before recording native chunks and a live recorder lease.
  Motion >5 mm / 0.1 degree, stale leases, archive faults or missing native ticks
  fail closed. This cannot establish moving-route robustness by itself.
- Recorder opt-in `--natural-stationary --require-physics-imu` retains all existing
  clock/physics-IMU/health/extrinsic barriers, but expects **zero** drivers and a
  separate stationary completion artifact, not a fabricated calibration route.
  Live capture uses the supervisor's existing normal initialization (which
  overrides the copied initial pose); exact GT-start qualification will be a
  separate held-out replay. No live result will be mislabeled GT-start.
- Initial host acquisition/lease contracts: **37 passed**. Actual ROS recorder
  contracts and the first physical acquisition are next. No natural-terrain or
  general robustness pass is claimed yet.
- First attempt preserved: simulator `revision-185ce1723ce54ec7af434cdd4799af82`,
  recorder `revision-e23b3b3b4f3840c49a80b149268aab95`. **Ineligible**: the
  environment's default is `rocks_settings.enable: False`, and no explicit
  rocks-enabled override was supplied. Added both an explicit flag requirement
  and actual USD rock-instance inventory; an empty rock scene cannot pass.
- That recording also failed physics-IMU freshness during native archive sealing.
  The new stationary collector announced completion **after** its synchronous
  JSON flush, unlike the existing guarded collector. Moved the finish barrier
  before sealing, keeping the same live watchdog and bounded sealing deadline.
  The failed attempt is not repaired or reused for acceptance. Initial live
  innovations were also larger away from the old fixed RTK origin; no estimator
  weights or accuracy gates were changed in response.
- New/adjacent host contracts: **71 passed**; actual ROS recorder/lease tests:
  **51 passed** before the first capture. The corrected acquisition will explicitly
  enable natural rocks with the same predeclared seed 43, mount and frozen core.
- Second attempt preserved: simulator `revision-01aa1e7737fa438faa56bde1688d9a5d`,
  recorder `revision-6a65ec3cefaf4f8899cbe0a288636321`. **Ineligible**: simulator
  grant heartbeat expired at 28.300 s; the recorder subsequently lost requests.
  This is not an estimator-accuracy result. Partial diagnostic: 110 actual rock
  instances, 365 consecutive native chunks, 46,720 current-pose first-hit checks,
  no missing hits, maximum 0.0697 mm (unchanged 1 mm limit). Recorded grants had
  no receipt gap above 125 ms; current evidence does not explain subscriber-side
  expiry. Isolated the collector from the shared default ROS executor, bounded
  callback draining to eight callbacks/tick, and added failure phase/lease-age/
  tick-gap diagnostics. **The 0.5 s lease was not relaxed.** A new diagnostic
  capture will determine whether this is sufficient; no root cause is claimed.
- Added actual GLIM loaded-library fingerprints to runtime manifests and a
  holdout comparator that permits only the declared initial-pose difference.
  Changed weights, initial velocity, missing/wrong plugins fail qualification.
  Native first-hit identities are optional evaluator-only diagnostics; the
  ordinary collector output remains unchanged when not requested.
- Third attempt: simulator `revision-68ddd0cf910a4baf9576d03ee27a28c0`, recorder
  `revision-521eb1b724e84d54826066137b7805d8`. **Ineligible**: request/clock
  heartbeat expired; simulator diagnostics show a **925 ms tick gap**. Dedicated
  callback draining is not sufficient. Frozen plugins are now captured in the
  runtime manifest. Next diagnostic removes only unused whole-scene GT grid
  rasterization from the opt-in natural acquisition; GT pose publication and
  all first-hit collision checks remain. Added bounded GC-pause/native-step
  telemetry. This isolates acquisition overhead, not full-stack latency.
- Current contracts: **82 host tests passed** (natural capture, manifest, lease,
  native capture and paired replay). No failed capture is being scored or used
  to tune estimator parameters.
- Fourth attempt: simulator `revision-a3fe7bb73a004bcd9a4c5fcb6c359ac3`, recorder
  `revision-864c21ef567c4a0baad935892c9ca361`. **Ineligible**, but timing instrumentation
  identifies the long pause: **816 ms generation-2 Python GC**, 895 ms tick gap;
  ordinary native-capture work max 79 ms. Disabling unused GT-grid rasterization
  did not resolve it and that change was **removed**. The new optional diagnostic
  retained 128 deeply nested first-hit records/tick on Isaac's long-lived heap.
  It now streams exact identities to exclusive-create, hashed per-chunk sidecars,
  retaining only path/hash metadata; GC stays enabled. Readers verify hashes and
  original selected indices. Next capture restores the original GT grid publisher.
- Added an observed-natural-surface diagnostic: fixed recorded first-hit identities,
  full-SE(3) point displacement and normal error, with no fitting/reassociation.
  Large-object eligibility is predeclared as all three world-AABB dimensions >=30 cm
  and >=20 audited sampled hits; this is not height-above-ground or obstacle recall.
  Partial failed input reveals two in-range ~0.8–1.1 m rocks but supplies **no
  accepted accuracy score**. Current focused host suite: **92 passed**; sourced ROS
  recorder/natural/provenance suite: **35 passed** before the sidecar change.
- Fifth acquisition **completed**: simulator `revision-8528ff66d19641cea1914e0e063db9ef`,
  recorder `revision-8cc0350b341d45a192aa5886c76bf2a5`. Streaming identities removes
  the observed pause without disabling GC or the GT grid publisher: maximum tick
  gap **222 ms**, native-capture step max **102 ms**, no GC pause >50 ms recorded.
  All recorder heartbeat/physics-IMU/pose-health gates stayed true, no commands,
  full 30 s acquisition. This establishes capture completion, **not yet accuracy**;
  geometry/provenance qualification and exact-GT-start replay follow.
  Broader host checks: **154 passed, 2 skipped** (optional terrain-library cases);
  focused sourced-ROS suite: **79 passed**.
- Fifth-run qualification **passes** (`revision-70d35bf502d249f8abf2dc90501ee5c0`):
  901 chunks, 115,328 audited first hits, zero missing, maximum 0.0697 mm;
  USD/TF/GLIM mounts and actual loaded frozen CPU/absolute plugins match.
  Export `revision-36e492afc59d4135b139dd160c6afd6b`; live accuracy diagnostic
  `revision-8657088ba4d9428387d1be2c3b9ca0c0`: base XY p95 **4.37 cm**, Z **27.21 cm**,
  roll/pitch/yaw **0.042/0.128/0.317 degrees**. Relative 1 s error passes, but the
  fixed vertical offset fails absolute accuracy. Normal startup is not promoted.
- Exact-GT-start attempt `revision-237dd06211f046e782b5e218f6a436c6` was correctly
  **rejected before estimator execution**: the preceding GT pose window contains
  a 100 ms gap (11.933→12.033 s), exceeding the unchanged 75 ms reference limit.
  Prepared data `revision-dbe23988a7af4eeab4c8918307334228` remains retained; no
  reference interpolation, missing-sample fabrication or gate relaxation.
  GT pose and expensive map rasterization share a timer executor. The natural
  capture now opt-in publishes each immutable physics pose snapshot immediately
  after capture on the simulation thread, with the pose timer disabled (never
  two publishers). Default runtime behavior remains unchanged. This also changes
  synthetic RTK sampling cadence, so the next recording is a new declared fixture,
  not a same-input A/B against the fifth recording. Method contracts cover each
  tick, duplicate suppression, exact measurement stamps and default timer mode.
- Read-only normal-start attribution on the fifth recording: the first published
  frame at **14.100000735 s** has an exact ROS GT timestamp. Applying the audited
  base→IMU mount and fixed home gives true IMU Z **0.38325709 m**, while the
  installed normal-start seed was **0.10998449 m**: **−27.327 cm already at seed**.
  Its initial pose damping is `1e10`. This accounts for the persistent vertical
  datum error; it is not evidence of a 27 cm mount mismatch or accumulating drift.
  RTK's unchanged range-dependent noise is much larger at this natural spawn
  (~55 m from the fixed beacon), and normal initialization averages just one
  second without propagating seed covariance. Do not tune this away using GT.
  A common vertical datum offset is also not equivalent to local terrain-shape
  error, since robot and observed surfaces share that frame.
- Sixth capture completed with tick-driven GT: simulator
  `revision-02b613306146474598b56a40614f0a3d`, recorder
  `revision-0724ce0d2b21450ba9f9f9f69317cdc2`. New qualification additionally
  requires an exact ROS GT counterpart for **every** native tick (2 ns conversion
  tolerance, no interpolation), <=1 mm translation / 0.0001 rad rotation
  against the independently audited physics pose. Old reports remain unchanged;
  the fifth capture's earlier qualification did not assert this stronger gate.
- Sixth full qualification **passes** (`revision-2a311d2f21fb4e488673de37196fe7bc`):
  901/901 exact GT/native counterparts; maximum position difference **4.84 nm**,
  rotation **3.65e-8 rad** (same physics reference, not an independent physical
  accuracy claim). First-hit collision gate still passes at 0.0697 mm maximum.
  GC telemetry again has no >50 ms pause; largest tick gap **234 ms** with the GT
  grid enabled. Export `revision-764b07c68b7e47f3b877a9c805e332ef`, prepared input
  `revision-eac260b8d90d42b6bbbab30b28d409d2`.
- Frozen GT-start replay **passes** (`revision-b6af0bb7fccc4db7ae1532b50bf6e94c`),
  accuracy `revision-32742879a8164c54a685cd878404513a`, observed surfaces
  `revision-8b77ba893dd94bbf9ccd98f9ed623dbd`. All 361 filtered scans, 2,280 physics
  IMU measurements and 1,071 accepted absolute fixes were dispatched. Exact seed
  window has 31 GT samples, max speed 0.292 mm/s, max angular speed 0.000222 rad/s.
  Initial-pose, overall, 1 s relative and available stationary-phase targets pass;
  there are **no turn/translation phases** in this recording. First-published
  all-surface displacement p95 **4.23 cm**, max **7.11 cm**; ground-normal max
  **1.51 cm**. All original sampled returns count; no fitted alignment or dropped
  outliers. Final-revised poses are diagnostic only.
- Same sixth recording's live normal-start diagnostic
  `revision-760371cd66a04750ba2c47051462941b` fails absolute XY/Z (p95 **15.35/30.96 cm**)
  while relative 1 s targets pass. Live-vs-replay is not a strict initialization-only
  A/B: live startup/availability differ from the direct-callback replay. The exact
  fifth-run seed comparison above independently establishes its initial datum error.
- Replay preparation now explicitly supports qualified physics-IMU recordings;
  inherited CPU/closed-window candidate settings cannot silently be reported as
  the default baseline. The caller must declare frozen plugins/policies. Full
  focused sourced-ROS contracts: **89 passed**; latest broader host suite before
  that extra inherited-candidate guard: **164 passed, 2 optional-library skips**.
- Repeat `revision-660232f7af3f4d1aadad824e3bacc9a9` also passes the unchanged
  accuracy targets (`revision-a879b226e99046bca9b31732ba05b433`). Comparison
  `revision-17cd11e9ee914981b129d5210eefc727`: all 361 processed clouds/neighbors/
  point-time hashes and 357 associated GPS factors match; buffered IMU count/latest
  stamp differs at **18 frames**. Maximum pose-matrix element difference is
  **7.9e-9**. Thus numerical accuracy repeats, but strict bitwise repeatability
  **does not pass on this fixture**. No repeat was selected as a winner, and no
  tolerance was relaxed. Input dispatch counts match in both runs. Callback
  availability also differs, so this is not a live timing acceptance result.
- Added shared-core attestation as well as candidate-plugin attestation: the
  recorded live process matches **243** frozen loaded libraries, not just the two
  candidate `.so` files. Only absent standalone bag-reader dependencies are allowed.
  A changed/missing shared core fails qualification even with identical plugins.
  Strengthened re-audit `revision-46faa20d2ceb42adaaf784d39dc79f20` passes; the
  earlier audit artifacts remain unchanged.
  Latest broader host contracts: **166 passed, 2 optional terrain-library skips**.
  Final focused sourced-ROS suite: **91 passed** (overlaps the host suite;
  do not add the counts). Root/submodule whitespace checks pass. Simulator and
  this turn's recorder/estimator/replay processes are stopped; no motion commands
  or production localization parameter changes were issued.

To reproduce the natural GT-start replay in the sourced ROS dev environment
with `/archive` mounted (allocates a fresh run; no simulator or navigation needed):

```bash
python3 tools/run_paired_glim_replay.py run \
  /archive/revision-eac260b8d90d42b6bbbab30b28d409d2 \
  --mode physics --initial-state-source gt-start --execution-profile serial --trace \
  --cpu-plugin /archive/revision-009c6a6d47ab4b85b81bb6cc7aabcf44/librevision_target_cpu.so \
  --target-policy smoothed-window --scan-information registration-mean \
  --absolute-plugin /archive/revision-009c6a6d47ab4b85b81bb6cc7aabcf44/libclosed_window_abs.so \
  --association-policy closed-window \
  --probe /archive/revision-057e1a1445f74c94bd9e4e03c9ee527a/glim_shadow_replay
```

Use `tools/diagnose_glim_accuracy.py` and `tools/diagnose_natural_surfaces.py`
with capture `/archive/revision-02b613306146474598b56a40614f0a3d` and the newly
printed replay run. The first command also archives `accuracy.png`.

#### Next bounded steps (no promotion yet)

1. Retain the explicit repeatability limitation: inspect the 18 IMU-buffer timing
   differences before claiming exact deterministic replay. Keep this scene/config
   frozen; any future tuning on it makes it development evidence. Do not confuse
   tiny numerical pose variation with passing the exact-input/trace gate.
2. Add a separate guarded **moving** natural-rock route with independent swept
   footprint/floor clearance and the same native/GT/clock audits. Do not reuse an
   analytic-platform driver that assumes its known floor or authorize unguarded
   reverse/rotation. Qualify GT-start first; measure turn/translation errors.
3. Address normal initialization as an **uncertain global datum**, not a hidden
   mounting offset: propagate measured RTK/gravity uncertainty, inspect the strong
   initial gauge anchor, and separate local terrain-shape accuracy from global
   altitude. Test synthetic seed/noise/correlation sweeps before any physical
   candidate. No GT-derived offset correction or indefinitely averaged samples
   masquerading as independent information.
4. After moving pose qualification, rerun the unchanged terrain/support/safety
   and resource gates with causal pose availability/revisions. Neither the 95%
   terrain coverage target nor normal-start/navigation acceptance is achieved here.

### Current outcome in brief

**GT-start assisted GLIM now passes every frozen pose-accuracy target on the
recorded calibration course, with bit-identical trajectories in two repeats.**
This is not yet natural-terrain, normal-start, live latency or mission acceptance.

| First-published base-pose error, p95 | Starting baseline | Recovery candidate | Frozen limit |
| --- | ---: | ---: | ---: |
| Horizontal position | 71.54 cm | 3.92 cm | 10 cm |
| Vertical position | 12.70 cm | 1.29 cm | 2 cm |
| Roll | 2.433 degrees | 0.140 degrees | 0.3 degrees |
| Pitch | 2.575 degrees | 0.179 degrees | 0.3 degrees |
| Yaw | 0.772 degrees | 0.350 degrees | 1 degree |

Implemented, **opt-in / not installed into mission launch**:

1. Rebuild the bounded CPU scan target from fused poses, retaining immutable
   sensor-frame samples instead of accumulating them at stale pre-fusion poses.
2. Use the already-recorded physics-tick IMU stream to retain acceleration
   impulses lost by the legacy render-rate stream. Gravity/mount/GT seed audits
   pass; simply raising frequency is not the proposed mechanism.
3. Close the existing RTK time-association window before selecting its nearest
   accepted fix. Serial preprocessing removes another source of repeat variation.
4. Preserve mean converged GICP curvature in the unary scan constraint instead
   of replacing all six axes with equal precision. No GT-derived weight tuning.

The unchanged terrain mapper reaches **93.11% free-ground coverage**, detects
**12/12 test objects**, and has **zero false-free hazards over the sequence**.
It still fails 95% coverage and 1% false-occupied limits (measured **1.055%**).
No denominator, cell size, height threshold or acceptance limit changed.

Remaining promotion gates: independent natural-rock geometry
holdout (this course cannot validate large rocks); opt-in live ROS/physics-IMU
clock and callback-latency verification; causal terrain revision handling;
unchanged multi-phase terrain/safety/resource gates. Normal-start qualification
remains deliberately deferred per the user's GT-start-first decision. The recovery
experiments above were offline; the subsequent natural acquisitions are recorded
separately above. No production parameter change or navigation command was issued.

#### Reproduce the retained recovery experiment

In the sourced ROS dev container with the external drive mounted at `/archive`:

```bash
python3 tools/run_paired_glim_replay.py run \
  /archive/revision-a928088680b0424eb4fd70c43cb4c3af \
  --mode physics --initial-state-source gt-start --execution-profile serial --trace \
  --cpu-plugin /archive/revision-009c6a6d47ab4b85b81bb6cc7aabcf44/librevision_target_cpu.so \
  --target-policy smoothed-window --scan-information registration-mean \
  --absolute-plugin /archive/revision-93fa94399540406790f212461481c5e8/libclosed_window_abs.so \
  --association-policy closed-window \
  --probe /archive/revision-057e1a1445f74c94bd9e4e03c9ee527a/glim_shadow_replay
```

The runner allocates a new immutable attempt. It uses the archived assisted
sensor recording and exact initial GT pose; it does not launch Isaac or move the
robot. Its `passed` field qualifies replay wiring/provenance, **not accuracy**:
score the printed run with `tools/diagnose_glim_accuracy.py <native-capture> <run>`.
Use native capture `revision-49ade291e2f5423caeb35310aa88ba68` from this fixture.

#### Next execution order — no additional tuning on this course

1. Freeze the passing candidate and its current settings. Collect an independent,
   GT/frame-audited natural-rock fixture, stationary first and then a separately
   collision-cleared bounded route. Do not reuse the analytic platform as a
   holdout or treat its unobserved plateau as a large-rock pass.
2. Run the same initial/moving pose gates and independently selected large-object
   surface-displacement gate. Preserve failed captures and every repeat. Only
   revisit the algorithm if this independent evidence exposes a mechanism.
3. Qualify live physics-IMU/RTK receipt timing, output TF and correction events,
   callbacks under load, and bounded target memory. Replay CPU/RSS is insufficient.
4. Integrate corrected evidence into the experimental terrain consumer with real
   arrival/revision timing, then run all fixed grid phases, support/boundary safety
   and both compute profiles. Keep the 95%/1%/zero-false-free gates unchanged.
5. Promote an opt-in live profile only after those gates. Normal initialization
   and navigation are subsequent gates; no unguarded movement or automatic
   replacement of the production configuration is implied by this checkpoint.

### Approved GLIM accuracy recovery plan [implementation in progress]

#### Continued execution — repeatability and pose recovery [fixture gate passed]

The entries below are an append-only experiment history; earlier “running” and
“pending” notes are superseded by the subsequent completion entries.

- Added opt-in processed-cloud fingerprints (XYZ order, neighborhood indices,
  point times/count), consumed IMU horizon, and an archived trace comparator.
  These are read-only diagnostics, not estimator inputs or accuracy fixes.
  Comparator/parser contracts: **13 passed** on host. Baseline frozen-input
  trace/repeat is underway; production remains unchanged.
- Source review confirms the CPU target map is populated before fusion and is
  not revised by subsequent fixed-lag corrections. The relative scan factor
  composes through the last fused pose, while the absolute prior retains the
  target-map pose. This is an outstanding consistency/feedback hypothesis,
  not yet proof that changing coordinates or weights will solve accuracy.
- Frozen repeat `revision-e7e3236faec4481abca6461ca9c96799` versus
  `revision-303f38109721400faa6898f0f8cb40b1`, comparison
  `revision-6b264d0f00de4140a6d1c926c9488eed`: **377/513 processed clouds and
  neighbor arrays differ**, beginning at the first scan. Point counts/times
  and all consumed IMU horizons agree. Only two of 510 GPS measurements differ;
  first optimized-pose divergence precedes those differences. This establishes
  preprocessing nondeterminism, not merely trace overhead. Version-matched
  [upstream sampler source](https://github.com/koide3/gtsam_points/blob/v1.2.1/src/gtsam_points/types/point_cloud_cpu_funcs.cpp)
  uses thread-local RNGs with dynamically scheduled sampling blocks. A declared
  `--execution-profile serial` ablation is running; input geometry, noise and
  factor weights are unchanged. No lowest-error attempt is selected.
- Added an **opt-in, not promoted** `revise_target_poses` CPU candidate: retain
  each accepted sensor-frame sample once, rebuild a bounded target from fused
  frame poses after smoothing, and express the next prediction in that same
  chart. At most `lru_thresh` accepted observations are retained; this changes
  target retention semantics and requires resource testing. It is built as a
  separately named standalone plugin, never overwriting installed GLIM. Default
  behavior remains frozen-target. Build and controlled baseline/candidate tests
  are pending; no accuracy benefit claimed.
- Serial repeat pair `revision-5b58e48f4c1a4778ab30871603c86818` /
  `revision-ce96e4a0fe954213ad588026c6d6354b`, comparison
  `revision-1853be9c2cb04539a59d795f47e58a56`: **all 513 processed clouds,
  neighborhoods, point times and IMU horizons now match**. However, 101 GPS
  associations differ; the first different factor and optimized pose are both
  frame 9. Serial processing alone does not fix runtime association timing or
  accuracy (LiDAR position p95 70.33 / 73.08 cm).
- Implemented `--absolute-schedule measurement` strictly for offline mechanism
  tests: supply the original recorded fixes with stamps <= the current scan
  immediately before the real absolute-factor callback, in a deterministic
  snapshot. No fix values/covariances or GT are changed; remaining tail fixes
  are dispatched after estimator drain and all original counts are checked.
  **This changes receipt availability and cannot establish live causal accuracy.**
  Defaults and production are unchanged. Repeatability qualification is next.
- Candidate reconstruction C++ contracts **pass**: no pre-fusion insertion,
  translation+rotation reprojection, unchanged sensor evidence, idempotence,
  observation-count expiry, survival after smoother release, and rejecting an
  unbounded horizon. Initial build needed explicit CMake reconfiguration to
  discover the new opt-in target; no installed libraries were replaced.
- Deterministic frozen-target control/repeat:
  `revision-51bc5bb1c85649e9ba929139701a4569` /
  `revision-52a2f17d674245279a05a5bc18b79f8e`. Serial processing plus the declared
  offline RTK schedule gives **bit-identical optimized poses on all 512 updates**
  and identical 1,417 native pose errors. Both retain all 513 scans/1,595 IMUs/
  1,480 fixes. LiDAR position p95 is 68.50 cm in both. Base accuracy report
  `revision-71758b0cfe1645b1acfcc83e2da2b977`: horizontal/vertical p95
  **68.84 / 12.46 cm**, roll/pitch/yaw **2.771 / 2.222 / 0.607 degrees**.
  This is reproducibility recovery, **not estimator accuracy recovery**.
  Standalone RTK schedule contracts preserve all 1,480 CDR messages exactly.
  The same-plugin, same-input target-reconstruction trial is now running.
- Target-reconstruction run `revision-a0c88e6cad7d47c3aa5f1a6fd43724cf` completed:
  LiDAR position p95 **68.50 -> 29.59 cm first-published**, **63.84 -> 15.83 cm
  revised**. All inputs are retained; only target policy changes relative to the
  deterministic control. This supports the local-map feedback mechanism, but
  height/tilt remain unacceptable (LiDAR Z p95 10.23 cm; world-roll residual
  p95 ~4.03 degrees). **Not promoted.** A single observed process RSS was
  ~689 MiB during replay; this is not a resource acceptance measurement.
- Next declared ablation keeps the same candidate and replaces only legacy IMU
  with the existing physics-tick stream from the same bag. GT-start validation
  now permits either IMU-centered backend input; LiDAR-only remains excluded.
  This follows the measured acceleration-aliasing evidence, not an assertion
  that GLIM inherently requires 60 Hz. Normal starts/live receipt timing and
  natural-rock holdout remain mandatory downstream gates.
- Physics-input + revised-target result `revision-057e1a1445f74c94bd9e4e03c9ee527a`:
  first/revised LiDAR position p95 **2.863 / 1.637 cm**. Base accuracy report
  `revision-9cffa03d91744d2792b769530525e163`: horizontal **2.755 cm**, vertical
  **1.480 cm**, roll/pitch/yaw **0.319 / 0.340 / 0.325 degrees** p95. The large
  position failure is substantially corrected on this input, but both tilt
  targets still narrowly fail. Do not relax the 0.3-degree threshold. Recorded-
  receipt repeats and unchanged-core terrain sensitivity are running.
- Independent installed-front-end geometry diagnostic
  `revision-186e9062481143518e58bf8e0cf2a765` covers all 513 scans. Median normal
  scatter eigenvalues are **0.0164 / 0.0249 / 0.9584**: strongly planar geometry.
  Mean point-to-plane Jacobian information eigenvalues span **0.0162–33.916**,
  unlike the current equal six-axis scan precision. This is geometric support,
  not calibrated covariance or verified correspondences. No GT enters it.
  A geometry-shaped factor candidate will be tested with unchanged scalar scan
  weights (normalize by return count, numerical eigenvalue floor only), plus
  analytic plane/nullspace, rotation, duplication and invalid-input contracts.
  This is a separately declared model change, not a parameter sweep or a ground-
  truth plane supplied to SLAM. Neither it nor the revised-target policy is
  promoted until the unchanged gates and recorded-arrival tests pass.
- Unchanged mapper, first-published candidate-pose sensitivity
  `revision-b2896d242e744c92b8bb218254838db2`: **51.08% flat free**, **1.54%
  false occupied**, **47.39% unknown**, **12/12 objects detected**, zero positive
  or dropoff false-free anywhere in the sequence. Same 1,417 native chunks,
  138,725-cell flat denominator, original reference and frozen `69d182ca...` core.
  This is a large recovery from the earlier 0–2.54% first-published coverage,
  but still fails coverage/false-occupied gates. It is pose-only sensitivity,
  not live integration or acceptance; no mapper threshold has been relaxed.
- Revised-pose sensitivity `revision-361d40c80e76457d8968d2f719100cdb` reaches
  **93.18% flat free / 0.802% false occupied / 6.018% unknown**, 12/12 objects
  and zero sequence hazard false-free. It still misses the frozen 95% coverage
  gate and uses future revised poses, so it is not a causal operating policy.
- Restoring original RTK receipt timing with the same revised-target/physics
  candidate gives LiDAR first-position p95 **5.771 / 5.187 cm** across both
  attempts `revision-d83e1a6c5af34d6ba0b0679daaf033b8` /
  `revision-b6f13ffa5b99451eac1e3d7e848a966b`. Position recovery persists, but
  tilt worsens (~0.8-degree world-roll p95), so accuracy is not yet passed.
  Comparison `revision-c39b5f4dbb3e424b93f41ef532655a9c` again has identical
  processed clouds but 98 different GPS associations; initial IMU horizon also
  differs once. Both attempts count; deterministic scheduling is not silently
  substituted for the real receipt-path result.
- Geometry-shaped-factor candidate `revision-fe03451a8df3469cad5a55e050c3749f`
  **does not pass**. Same-plugin isotropic control
  `revision-62e520b389b748178fdb34c0f5374fae` reproduces the prior physics/target
  trajectory bit-for-bit (`revision-318dbdb32b86441b961bdc871267348b` comparison).
  Geometry candidate base p95 is **3.945 cm XY / 1.790 cm Z / 0.303, 0.392,
  0.346 degrees roll,pitch,yaw**, report `revision-464007b8cd6548a6a4fd1e6945e5e64c`.
  Analytic contracts pass, but anisotropy alone does not fix tilt and is **not
  selected for the next trial**. Existing scalar precisions/IMU noise remain
  untouched in the retained isotropic candidate.
- Implemented a separate, opt-in **causal closed-window absolute association**:
  wait for the complete existing +/-50 ms matching window before choosing a
  fix, require strictly increasing validated source stamps, and still add only
  one fix per surviving pose. The supervisor already rejects non-increasing
  accepted fixes. This avoids irreversibly choosing a worse older fix just
  before a nearer one arrives. It delays fusion; it does not change stamps,
  covariance, fix values or receipt order. Default association remains unchanged.
  Host and standalone C++ fixtures pass nearest-match, no-premature-consumption,
  no-reuse, outage and marginalization checks. Two recorded-arrival trials with
  the retained isotropic/physics/revised-target candidate are running.
- Host NVIDIA and actual CUDA context/allocation preflights pass on September 21.
  No simulator or motion publisher was started for these tests.
- Closed-window recorded-arrival repeats complete:
  `revision-93fa94399540406790f212461481c5e8` /
  `revision-f899a4caa0a042389d45e3ae913afc67`. Comparison
  `revision-79ffe83692434b0f9726706c499fac81` confirms **all 512 optimized poses
  bit-identical**, all 513 processed clouds and all 509 associated fixes equal.
  Two callback IMU horizons differ without changing predictions or poses.
  Base report `revision-5ca8969e8a284cd89b38f22e18449b6b`: first-published
  horizontal/vertical p95 **2.969 / 1.493 cm**, roll/pitch/yaw
  **0.346 / 0.285 / 0.335 degrees**. Only roll fails the frozen pose targets.
  This uses the original receipt order, not the noncausal schedule, but remains
  offline callback replay rather than live ROS latency qualification.
- Next declared architectural ablation: installed tightly-coupled GPU odometry,
  keeping the exact GT seed, physics IMU, sensor/noise/lag settings, serial
  preprocessing and closed-window recorded RTK policy. GPU registration and
  keyframe settings come from the original frozen GPU config; CPU target/scan
  priors are not used. CLI rejects combining CPU-only candidates with this
  backend. Parser explicitly omits the nonexistent CPU scan-prior stage rather
  than fabricating a measurement. Config/parser tests pass; run
  `revision-a47d347a0efd442690124587db624399` is in progress. This is a backend
  experiment, not a controlled IMU-only comparison or a production promotion.
- GPU trial completed, **not selected**. Base report
  `revision-15ebf97ccff9476b8f6badc7f951e14d`: XY/Z p95 **29.55 / 0.112 cm**,
  roll/pitch/yaw **0.076 / 0.048 / 7.282 degrees**. Accurate plane alignment
  does not establish horizontal/yaw observability; both those targets fail.
  All original inputs and 509 absolute associations remain present. This
  contrasts with the retained CPU candidate, rather than establishing a frame
  offset or proving IMU integration alone causes the residual tilt.
- Declared next CPU ablation, `--scan-information registration-mean`: retain
  the converged GICP unary registration Hessian, normalized by source count,
  instead of its isotropic absolute scan precision. Keep relative scan factors,
  sensor noise, rejected-match policy, target reconstruction and causal absolute
  association unchanged. Unlike the earlier normal-only shape, this includes
  actual correspondences and covariance regularization. It is still only a local
  target approximation, **not calibrated covariance or a wrong-match detector**;
  no GT-dependent eigenvalue cutoff or fitted multiplier is introduced. Positive-
  semidefinite, finite-input and duplicate-count contracts precede replay.
- Verification checkpoint: **151 focused ROS-overlay/Python contracts pass**
  (GT start, frame/surface accuracy, trace comparison, replay preparation, IMU
  timing/consistency, pose sensitivity, shadow input/capture/pair gates).
  Mean-curvature and target C++ builds pass; the target executable initially
  received no required config argument and exited 2, then passed with the frozen
  run config. No estimator result was produced or discarded by that invocation.
  Same-build isotropic control `revision-883a58969e7240d696aae511ae84ddaa` and
  curvature candidate are running sequentially; no concurrent simulator.
- Provenance clarification: copied `harness_sources` are a working-tree snapshot,
  **not an attestation of the source used to build a reused frozen executable**.
  Executed probe/plugins and actually loaded libraries have separate retained
  hashes; matched experiments explicitly reuse the same executable. New attempt
  metadata states this distinction. Older artifacts are not rewritten.
- Same-build isotropic control matches the earlier closed-window trajectory
  exactly: comparison `revision-24606e9981e0465fa7edb00dbf61ec9c`, every pose,
  processed cloud and GPS factor equal (one harmless callback-horizon difference).
- **First full pose-target pass:** mean-registration-curvature run
  `revision-009c6a6d47ab4b85b81bb6cc7aabcf44`, base report
  `revision-45488aaed4ba462d8b0be91142c5486c`, first-published XY/Z p95
  **3.919 / 1.290 cm**, roll/pitch/yaw **0.140 / 0.179 / 0.350 degrees**.
  Initial-output, overall, 1 s relative-motion, stationary, translation and turn
  pose targets all pass. All 513 clouds, 3,190 physics IMUs and 1,480 recorded
  fixes are retained. No fitted trajectory, ongoing direct GT, altered terrain
  classifier or gate relaxation. Frozen-binary repeat
  `revision-1ba60296bbf54064b5e6d71359bc6530` and unchanged-core first-published
  terrain sensitivity `revision-57192091de504d028ccf77b840aed973` are in progress.
  This is **development-fixture pose recovery**, not natural-rock or live-nav
  acceptance; do not install the candidate into mission launch yet.
- Independent delay hypothesis **fails**: `revision-f047210cc67c42c1a6c7890e33887408`
  queries the earlier isotropic run at exactly acquisition+200 ms, using only
  already-arrived revisions. All 1,417 acquisitions remain included; roll p95 is
  **0.343 degrees**, still above 0.3. No longer delay is selected to make it pass.
  Added future-revision exclusion, missing bracket/tail, order and no-trimming
  contracts (**6 passed**). No delayed-publication policy was installed.
- First-published curvature-candidate terrain sensitivity completes:
  `revision-57192091de504d028ccf77b840aed973`, **93.114% flat free / 1.055%
  false occupied / 5.830% unknown**, **12/12 objects** and **zero sequence
  positive/dropoff false-free**. Unchanged core/reference/grid/1,417 native chunks
  and 138,725 fine flat cells. This is a substantial recovery from 51.08% with
  the earlier first-published isotropic candidate, but **coverage and false-
  occupied gates still fail**. Pose-only sensitivity cannot pass live compute
  or revision-handling gates; no costmap promotion follows from pose recovery.
- Frozen curvature plugin SHA-256:
  `4ccee2909b6d1c20acfdf1294c1da321532b967861efb69b799a1c9cc5745bc6`.
  The initial curvature run used **732.8 MiB peak child RSS**, **44.34 user +
  3.13 system CPU seconds** for the 51.23 s measurement span. These are process
  diagnostics, not an idle-host, long-run or live callback-latency guarantee.
  Defaults/mission launch remain untouched; both failed geometry/GPU candidates
  and the failed delay hypothesis remain archived rather than selected away.
- Frozen candidate repeat completes: `revision-1ba60296bbf54064b5e6d71359bc6530`.
  Comparison `revision-f9765cd0b0f84c6997870a1a8e213ca3` confirms **all 512
  optimized poses and 513 predictions bit-identical**, all processed clouds and
  509 absolute associations equal. One consumed-IMU callback horizon differs
  without changing any pose. Both complete 1,417-native-pose error sequences
  agree. This closes the declared GT-start development-fixture accuracy gate,
  not the independent holdout/live qualification gates.
- Final-revised terrain sensitivity `revision-8ce4ffc4bc9c4d77b6685073771aa78e`:
  **93.549% flat free / 0.891% false occupied / 5.560% unknown**, 12/12 objects,
  zero sequence positive/dropoff false-free. Coverage still fails. These final
  poses use future corrections and cannot be substituted for live availability.
  Improved pose accuracy therefore does not eliminate the remaining terrain
  support/boundary problem; no further classifier tuning was done in this task.
- Final verification: **157 focused Python/ROS contracts pass**, plus standalone
  target-reconstruction, scan-curvature, closed-window association and exact-CDR
  scheduling C++ contracts. Repository/GLIM/GLIM-ROS whitespace checks pass.
  All experimental processes completed; no simulator, command publisher or
  production estimator was started or reconfigured.
- Independent surface displacement report
  `revision-6617c1e1951a4243970a75111675c253`: all **15 observed small-object**
  surfaces have first-published displacement p95 **3.44–4.23 cm** (revised
  **1.64–2.01 cm**). The 12 objects required by the terrain-height gate remain a
  separate denominator. Across 10,324,031 flat-ground returns, absolute Z p95 is
  **1.327 cm first / 0.932 cm revised**. There are **zero eligible observed large
  objects**; the plateau has no returns. Natural-rock validation remains explicitly
  NOT_EVALUATED, not a vacuous pass.

User selected **exact GT start first**, deferring normal-start qualification.
Execute in order: (1) loaded-binary/gravity/frame/historical audit; (2) one-time
GT initial state at the first estimator cloud; (3) per-stage/time error attribution;
(4) large-rock surface displacement independently of ground classification;
(5) one-mechanism fixes with frozen replay, then natural-rock holdout and terrain
safety regressions. CPU, legacy IMU, recorded accepted RTK, current mount, disabled
global optimization and terrain core remain fixed. Scan distortion is a hypothesis,
not the assumed cause. No live navigation or parameter sweep is authorized by a
failed diagnostic. Preserve every attempt on the external archive.

Frozen diagnostic targets: initial emitted pose <=1 cm / 0.1 degree; moving
horizontal/vertical position p95 <=10/2 cm; yaw p95 <=1 degree; roll and pitch
p95 <=0.3 degree each. Large objects are GT-selected before scoring (>=30 cm high
and across, >=20 GT-visible returns), with per-object surface displacement p95
<=10 cm. These targets do not replace terrain safety/coverage or navigation gates.

- Implemented opt-in replay `--initial-state-source gt-start`: measurement-time
  GT interpolation, preceding 1 s stationarity, resolved base/IMU/LiDAR composition,
  zero stationary velocity and unchanged bias/RTK policy. Rejects missing brackets,
  gaps, reversing GT stamps and nonstationarity. Archives seed provenance and gates
  the actual first emitted pose; no trajectory alignment or ongoing direct GT.
- Initial host verification: **24 GT-start and replay contracts pass**. Frozen
  recording preflight and installed-runtime diagnostics are in progress. No
  estimator accuracy improvement or completion of this plan is claimed yet.

#### GT-start baseline and error attribution [completed diagnostics; accuracy fails]

- First-cloud seed is at **20.200001053 s**, using an exact GT timestamp and
  30 preceding stationary samples. Maximum measured stationary speed/angular
  speed is **0.000115 m/s / 0.000075 rad/s**. GT is explicitly in `gt_map`;
  `map_from_world` translates by `[20,20,0]`. The helper initially asserted
  `map` and failed its read-only preflight; corrected to the actual published
  frame before any replay. This was a new helper assertion, not a discovered
  GLIM frame error. Initial emitted error is **2.57e-11 m / 3.77e-9 degrees**.
- Uninstrumented baseline: `revision-e21f970cf269421db45af4565dc0c022`.
  Exactly 513 clouds, 1,595 legacy IMU, 1,480 recorded accepted absolute fixes
  and 1,595 clocks; all 1,417 native acquisitions qualified. No direct GT after
  initialization, no trajectory alignment, no new physical capture.
- Actual loaded-library probe passes: configured and GTSAM gravity are
  **1.62 / [0,0,-1.62] m/s²**. Integrating 60 stationary lunar samples over 1 s
  produces only **5.55e-16 m / 6.66e-16 m/s** error. `/proc/self/maps` confirms
  `/tmp/repair-install/glim/lib/libglim.so` (`7ed14238...`) and CPU plugin
  `libodometry_estimation_cpu.so` (`552731e7...`). Actual dlopen plugin hashes
  are archived, closing the previous ldd-only provenance gap.
- At the **base origin**, baseline first-published horizontal/vertical p95 is
  **71.54 / 12.70 cm**; roll/pitch/yaw p95 **2.433 / 2.575 / 0.772 degrees**.
  The initial gate passes but moving accuracy fails. Seed accuracy does not
  resolve the developing error. Figure, arrays, 1 s relative-motion and
  stationary/translation/turn summaries:
  `revision-2e46dd6d7f7d4c5587a4b452213cf301`.
- Read-only callbacks added in the standalone probe (no core/library edits):
  IMU-predicted pose, scan prior and relative-factor measurements, pre-smoother
  current pose, optimized pose, actual factor sigmas/GPS measurements and keys.
  Trace run `revision-c52f6a0208b945ffa8aee3864a9b49dd`; stage report/figure
  `revision-70ffd8ddb937438a8b01f64092576ff9`. All 472 estimator frames inside
  the native acquisition interval have all four stage records.

| Stage in traced run | Horizontal p95 | Vertical p95 | Roll / pitch / yaw p95 |
| --- | ---: | ---: | --- |
| IMU prediction | 68.09 cm | 13.84 cm | 2.396 / 2.307 / 0.790 deg |
| Scan-match target prior | 111.01 cm | 0.170 cm | 0.076 / 0.051 / 18.614 deg |
| Pre-smoother current pose | 67.29 cm | 12.57 cm | 2.392 / 2.277 / 1.560 deg |
| Optimized current pose | 67.29 cm | 12.57 cm | 2.372 / 2.267 / 0.771 deg |

- These measurements expose **different failing directions**: scan matching
  preserves the floor's height/normal but poorly tracks XY/yaw; the fused
  trajectory gains XY/yaw information but has substantial height/tilt error.
  This is consistent with weak planar-scene registration plus fusion conflict,
  not proof of the precise root cause. Correspondence/Hessian diagnostics,
  matched absolute-factor timestamps and local-map consistency remain to audit.
  Do not change scan weights solely from this table.
- Actual scan-prior sigma is **0.057735** in all six coordinates (precision 300),
  confirming the current split-weight implementation rather than stale notes
  about an immutable 1,000 prior. All **510** position and heading constraints
  enter; no scan rejection occurs (all observed scan-prior sigmas unchanged).
- Repeatability is a newly measured limitation, not a successful tuning result:
  uninstrumented repeat `revision-6624123e880d4b44926e9b919365658b` has the same
  inputs/configuration/binary/library hashes but differs from the baseline by
  **27.93 cm p95 / 34.21 cm max** at the same LiDAR acquisition poses. Trace-on
  versus baseline differs **4.69 / 7.01 cm**. Both fail the strict 1 um /
  1 urad diagnostic equivalence check. Comparison artifact:
  `revision-ca1b517598884b309c290ffed87285ca`. No run is selected as a pass.
  Trace timing is excluded, and stage findings apply to their own run; no claim
  that instrumentation preserves the exact trajectory is made. Input delivery
  to asynchronous workers, factor association and parallel preprocessing need
  repeatability attribution before claiming small A/B improvements.
- The scan-match target column describes the **absolute prior actually applied
  to the optimized pose key**, not a separately GT-qualified raw registration
  trajectory. CPU `create_factors` composes its relative match through the last
  optimized pose but inserts the absolute prior in the retained target chart.
  That chart's consistency after fused/RTK corrections needs explicit examination;
  the trace alone cannot distinguish poor correspondence geometry from target-
  versus-optimized-frame drift. This remains a first-class frame audit, not an
  assumption that feature-poor geometry explains everything.
- Measured native **LiDAR-origin** endpoint motion over 100.1 ms, at radius 5 m,
  gives a **4.33 cm p95 / 5.32 cm max** displacement bound. (The earlier read-only
  base-origin estimate was 4.07/4.92 cm; final archive uses the correct off-center
  LiDAR origin.) This is not a per-point timing audit or a bound on amplified
  registration error, but does not support attributing 70 cm drift directly to
  motion distortion. A checked max-range-sentinel hypothesis is also unsupported:
  initial filtered clouds have maximum range ~57.9 m, below the actual 100 m
  sensor limit; no range-filter change was made.
- Separate surface diagnostic: `revision-28f60efd25a64b2aa127799a43d81a3b`.
  Identical GT-selected box top/side returns are displaced under estimated poses;
  no obstacle-recall denominator or fitted transform is used. Ground-only
  absolute Z p95 is **16.36 cm first-published / 10.81 cm revised**, independently
  of the terrain classifier. **No eligible large-object observation exists**:
  all 15 boxes are 5–20 cm high/across and the 30 cm x 1 m plateau has zero
  selected surface returns. This recording cannot establish performance on
  natural large rocks, either positively or negatively.

#### Earlier success evidence reconciled, not discarded

- `rosbags/glim_vs_rko_replay_20260607_143914/metrics.txt` retains a genuinely
  better GLIM result: **17.04 cm RMSE / 28.22 cm p95**, with a fitted yaw of
  **-0.208421 rad**. The windowed report
  `rosbags/window_metrics_20260607_rko_glim/summary.txt` reports GLIM p95
  **9.09 / 12.26 / 11.31 cm** on its three separately aligned windows. These
  are useful recovery references, not directly equivalent fixed-frame scores.
- June 18 sample-return gates for seeds 42/411/7 are preserved and pass, with
  home distances **0.786 / 0.706 / 0.707 m**. These demonstrate mission progress,
  not centimetric surface alignment. Conversely the June 9 `rtk_ab_confirm_btw`
  record has **1.68 m first-aligned p95** despite no corruption. Historical
  performance was not uniformly poor or uniformly centimetric.
- Source commit `8495fec` contains the lunar-gravity fix and the loaded runtime
  now passes its physical test. Later scan-gating/split-weight/RTK-association
  changes and the difference between natural rock scenes and the current tiny-
  object flat platform remain relevant. Do not replace verified gravity code or
  blindly restore obsolete mounts, Earth-scale acceleration or stale-fix reuse.

Verification: **122 focused contracts pass; four existing strict temporal terrain
xfails remain**. Standalone C++ rebuild passes after correcting two diagnostic
compile errors (GTSAM params accessor is a pointer; factor graph needs its complete
header). Root, OmniLRS, GLIM and GLIM ROS whitespace checks pass. No installed
estimator, production launch, terrain threshold or driver configuration changed.

Next gate: explain replay variability and the scan-match XY/yaw failure using
actual retained inputs and feature/correspondence evidence. Recover a provenance-
qualified historical natural-rock fixture or collect a separately declared,
guarded feature-rich fixture; the current small-object platform cannot substitute
for the requested large-rock test. Only then select a mechanism-specific fix,
repeat fixed-input accuracy, and proceed to the untouched natural-rock holdout
and terrain safety gates. **The overall accuracy-recovery plan remains open.**

### Previous center/initialization diagnosis

**New center/initialization audit:** comparing the same physical `base_link`
origin on both sides, the paired first-published trajectories begin only
**3.35/3.37 mm** and **0.133/0.153 degrees** from GT (legacy/physics-rate).
Their route position p95 is nevertheless **74.25/72.39 cm**. Removing only
the first-sample world gauge gives **74.02/72.15 cm**; removing only the
first-sample body extrinsic gives **73.98/72.12 cm**. A fixed origin mismatch
does not explain these recordings. This does not rule out estimator-internal
timing/extrinsic errors. All 1,417 native samples are included, with no
full-trajectory fit and no correction supplied to the mapper. LiDAR and IMU
origins are also reported separately in
`revision-0b0307b33ef54136a788973f8eb04c9d/anchor_audit.json`.

60 Hz is the simulator's existing physics rate, not a demonstrated requirement
to use an IMU. A separately labeled **LiDAR-only backend ablation is complete**:
the installed CT-GICP backend explicitly requires no IMU and starts at the LiDAR
origin with identity pose. The harness dispatches only the unchanged recorded
clouds and clock, disables RTK and local/global mapping, and checks the identity
export extrinsic. It retains the original rigid-scan approximation because these
ROS clouds have no measured per-point times. Initial-pose GT alignment is
reported only as an evaluation diagnostic, not fed to GLIM or silently admitted
as a standard terrain score. Production configuration and mapper are unchanged.

#### September 21 — center audit and no-IMU trial results

- Added `tools/audit_glim_anchor.py` plus synthetic fixed-world-offset,
  fixed-body-offset, zero-start/developing-drift and identical-origin tests.
  Both sides use the independently audited native LiDAR pose and recorded mount
  to evaluate the same LiDAR, base or IMU origin. The initial sample means the
  first native acquisition in the scored route, not necessarily GLIM's first
  input scan. The existing paired-input qualification/hashes remain required.
- Added the explicit `lidar_only` mode to the **standalone** replay harness.
  It uses the existing CT-GICP parameters without tuning, preserves cloud CDR,
  preprocessing, receipt pace/order and physical mount, and disables absolute
  assistance and local/global mapping. The CT state is LiDAR-centered with an
  identity exported `imu_from_lidar`; the wire field name `map_from_imu` must
  not cause a second physical IMU transform. No production package was rebuilt
  or installed; only the standalone replay executable changed.
- Code inspection confirms the global-shutter preprocessor zeros all point
  times and the CT factor guards its time-normalization denominator. Thus this
  is a valid **rigid-scan** experiment, not a measured-time CT deskew test. The
  startup warning about generated pseudo times is retained; those times are
  subsequently zeroed, not treated as measured acquisitions.
- Archive/GPU/CUDA preflight passed. Run
  `revision-aaf4b1a0252445da86cc9b67cd6f6d0f` consumed exactly **513 clouds and
  1,595 clock messages**, **zero IMU and zero absolute-pose messages**, and
  bracketed every one of the **1,417 native acquisitions**. Raw trajectories,
  source/config snapshots, installed CT plugin hash, dispatch counts and log
  are retained. No simulator or command publisher was started.
- At `base_link`, the raw initial **57.15 cm / 5.001-degree** offset is expected:
  CT initializes an identity LiDAR-local world, unlike the home-anchored assisted
  backend. We explicitly remove only this initial **world gauge** for the
  diagnostic comparison. Result: position median/p95/max/final
  **73.04/115.38/121.28/77.55 cm**; orientation median/p95/max/final
  **14.409/18.536/19.569/1.708 degrees**. Initial error becomes numerical zero,
  but subsequent error remains. First-published and revised results coincide.
  This rules out an initial gauge explanation for this trial, not all possible
  calibration/timing problems in the input pipeline.
- The no-IMU run changes backend **and removes RTK**, so it is not a controlled
  proof that IMU helps or is indispensable. It establishes that the available
  untuned no-IMU/rigid-scan alternative is not an immediate fix on this fixture.
  Its output is intentionally `lidar_only_audit.json`, not an accepted
  `shadow_pair_audit.json`: downstream terrain scoring cannot silently consume
  its GT-initial-aligned diagnostic as an ordinary home-anchored estimate.
- Verification: **91 tests pass, four pre-existing strict terrain-registration
  xfails remain** across anchor/dispatch, shadow pair/pose/timeline and temporal
  terrain contracts in the sourced ROS environment. Standalone C++ rebuild and
  root `git diff --check` pass. No terrain threshold or evidence lifetime changed.

Next bounded steps (no blind rate/threshold tuning):

1. Qualify the **actual ROS full-scan input** to GLIM against native acquisition
   identities: coordinate frame, scan header versus acquisition interval,
   ordered points and rolling-scan motion. Native ground-truth ray correctness
   alone does not establish this full-scan deskew contract. Use rigid/stationary
   and moving fixtures to separate scan formation from registration.
2. Once that contract is explicit, compare a qualified LiDAR-only baseline and
   LiDAR-inertial baseline with declared initialization and assistance policies.
   A fair IMU ablation cannot also silently change the RTK constraints or apply
   a trajectory-wide fit. Retain initial and time-varying errors separately.
3. Investigate residual estimator/absolute-factor calibration and weighting on
   fixed inputs, then repeat frozen terrain safety/coverage evaluation. Do not
   infer that the IMU must run at 60 Hz, or that small raw sensor noise guarantees
   accurate scan registration. Pose-uncertain terrain accumulation remains open.

### Previous September 20 checkpoint

**Completed:** a guarded moving Isaac/native-LiDAR/assisted-GLIM recording,
live USD/TF/GLIM extrinsic qualification, native timing audit, arrival-aware pose
export, and fixed-input terrain sensitivity replays. The simulator is stopped;
the experimental mapper is still outside Nav2 and its core remains `69d182ca...`.

**Main finding:** this is not just the old boundary case. On the same 1,419 native
chunks and original fixed GT reference (single phase-0 development fixture):

| Poses used for terrain replay | Final flat ground free | Flat ground wrongly occupied |
| --- | --- | --- |
| Measurement-time GT | 95.172% | 0.500% |
| GLIM first published | 0% | 95.317% |
| GLIM final corrected | 0% | 94.759% |

The two estimated-pose runs are **offline pose-only sensitivity**, not causal
map-scheduling or live revision-rebuild tests. All raw evidence, timestamps,
projection ticks, frozen core, GT denominator and gates stay fixed. They expose
a large SLAM/terrain-model compatibility failure: accumulated ground evidence is
classified as obstacle under measured pose errors. The subsequent code audit
isolates a mechanism: the lowest-layer ground fit plus peak-height obstacle test
interprets temporally misregistered floor returns as distinct surfaces. This is
not proof that GLIM itself, rather than its sensor inputs, is the root cause.
GT baseline retains 12/12 objects and zero sequence hazard false-free;
the all-occupied estimated maps also retain recall but are unusable, not passes.

First-published GLIM position/orientation p95 is **24.36 cm / 2.72 degrees**;
final corrections improve it to **22.26 cm / 2.05 degrees**. No alignment was
fitted and no native chunk was excluded. Supervisor `HEALTHY` is therefore not
a certificate of terrain-mapping accuracy. IMU/scan timing, fusion behavior and
pose-uncertain terrain accumulation take priority over further boundary tuning.

Implemented integration fixes: opt-in localization-only mode with no navigation
authority; recorder-before-motion lease; fresh archive/GPU preflight; precise
settled IMU calibration; bounded all-ray viewpoint audits; atomic two-phase
completion and a final pose-bracketing tail. Four failed integration attempts
remain archived. Detailed run IDs, test results and next steps follow below.

Final verification: **175 focused Python/ROS/C++-binding contracts pass**, with
no skips in that run; root and OmniLRS `git diff --check` pass. The C++ mapper was
not changed. No simulator, estimator or motion process remains running; dev
containers remain available. Existing unrelated worktree edits are preserved.

**Subsequent audit/implementation:** isolated the mapper's temporal-registration
failure and added explicit failing robustness fixtures. The IMU orientation lag
does NOT imply the gyro shares that lag; a standalone Isaac component fixture
shows current-state gyro and a resettable sensor clock. Implemented a separate,
opt-in physics-rate IMU stream, preserving normal GLIM inputs. A fresh stationary
real-robot check passes 600 contiguous 60 Hz samples alongside 300 legacy 30 Hz
samples. At that checkpoint, moving IMU accuracy and terrain recovery were not
yet demonstrated.
Detailed artifacts, the preserved failed check and next decision gate are below.

**Newest result:** the guarded moving paired-IMU experiment is now complete.
On identical GT windows, the physics-rate stream reduces acceleration residual
p95 from **0.3740 to 0.02811 m/s²** and gyro-increment residual p95 from
**0.02652 to 0.01717 rad/s**. Every legacy acquisition has identical acceleration
and gyro in the physics stream (allowing only the documented 1 ns serialization
difference). This supports testing the higher-rate input in GLIM; it does **not**
yet establish localization or terrain recovery. Production inputs/core unchanged;
simulator and estimator are stopped. Details and exact artifacts follow.

**Subsequent same-bag estimator/terrain A/B: the 60 Hz IMU does not fix terrain.**
It lowers first-published vertical pose p95 from 10.20 to 1.81 cm, but remaining
horizontal/attitude error still breaks accumulated mapping. Flat-free coverage
is 95.042% with GT, 0% with legacy GLIM, and only 2.542% with first-published
60 Hz GLIM poses. Revised 60 Hz poses reach 37.548%, but fail obstacle-location
recall and create positive-obstacle/dropoff false-free cells. **No promotion.**
The complete controlled comparison, safety failures and next checks are below.

### September 14 documentation — offline algorithm walkthrough

- Added an eight-stage pipeline explanation, frame/integration boundaries,
  continuous-height versus XY-resolution distinctions, and output reason meanings.
- Embedded the unchanged retained 5-degree/2.5-cm-offset comparison PNG from
  `revision-2de805151f14489991d5ff646864f2a0`. No archive dependency when viewing.
- Generated the interactive wall comparison directly from frozen retained `69d182ca...`
  and rejected `7a13403f...` libraries, verifying both SHA-256 values. All three
  fixture slopes reproduce zero versus one false-free wall probe; the page embeds
  actual cropped cost/reason arrays and evaluator-only geometry, not a JS mapper.
- Included local-viewing, small-fixture, native-replay and rejection-diagnostic
  commands with archive/environment requirements. No simulator was started and no
  algorithm or acceptance threshold changed for this documentation task.
- Validation: three host-only HTML contracts and headless Chrome desktop/mobile
  checks (all slope selections, return-overlay toggle, image load, no horizontal
  overflow, no JS errors or external requests). Cached Playwright initially lacked
  dependencies under host Python 3.10; checks used its existing Python 3.12 packages.
  The first static check assumed 50 wall positions; the actual frozen NumPy fixture
  contains 51 including its near-endpoint, so the assertion was corrected without
  altering the evidence. Runner is documented in `AGENTS.md`.

## Moving GLIM shadow evaluation — September 14 [in progress]

### September 20 continuation [in progress]

#### Same-recording IMU → GLIM → terrain A/B [completed; terrain gates fail]

- GPU/CUDA and external archive preflight pass; no new Isaac capture needed.
- Located exact supervisor-modified initialization and accepted-absolute topic
  configuration embedded in the original runtime manifest. Restore those bytes,
  not the pre-launch defaults or a fresh fitted/GT-derived initialization.
- Implementing a standalone, non-installed replay runner that dispatches original
  CDR messages directly to the existing installed GLIM/absolute-input callbacks,
  at the same recorded receipt pace/order for both arms. This avoids DDS input
  loss and provides callback counts. It does not modify the estimator or publish
  GT, motion commands or newly generated RTK. Output revision availability is
  offline bookkeeping, explicitly not live timing/compute acceptance.
- Both arms will use one prepared post-teleport input bag, identical filtered
  clouds and recorded accepted assistance, changing only the IMU stream. Native
  GT remains evaluator-only; the assisted-estimator limitation still applies.
  Final terrain results are recorded below.
- Preparation `revision-a928088680b0424eb4fd70c43cb4c3af` freezes 513 clouds,
  1,480 accepted absolute measurements, 1,595 legacy and 3,190 physics IMU
  samples, and 1,595 clocks following the first post-calibration request. All
  CDR payloads and original receipt order/timestamps are retained and hashed.
- The first standalone build failed on mixed Space ROS/Jazzy type-support
  exports. A fresh explicit Jazzy + existing GLIM/interfaces overlay build
  succeeds. No installed estimator library or launch setup changed.
- Legacy attempt `revision-4c622ca9f9ad4b1a9dde7a9debc4c288` ran first;
  the probe executable and harness sources are archived with dependency hashes.
  Fourteen preparation/comparison contracts now cover exact runtime bytes,
  callback counts, the input allowlist, and rejecting extra changes to binary,
  noise, initialization or source provenance.
- Legacy arm completes with all callbacks counted and all 1,417 native pose
  brackets valid. First-published position/rotation p95 **74.48 cm / 3.799°**;
  final revised **64.21 cm / 2.560°**. This is an offline control, not an exact
  reproduction of live scheduling (initial warm-up dispatch and final drain
  differ). The preserved postroll produces expected time-keeper warnings once
  IMU continues beyond the last cloud; no IMU/cloud input was rejected. The
  physics arm is next with exactly the same prepared bag and probe binary.
- Physics arm `revision-7417b6d2bb6a4365a85409f95540d576` completes: all 3,190
  selected IMU callbacks, 513 clouds, 1,480 assistance callbacks and all 1,417
  native brackets pass. Both arms add exactly 510 position/yaw assistance factors
  (same 152 no-match events). First-published vertical absolute p95 falls from
  **10.20 to 1.81 cm**, rotation norm p95 **3.799 to 2.924°**. However, horizontal
  p95 remains **74.25 versus 71.45 cm**, and position median worsens **9.02 to
  31.79 cm**. Final revised vertical p95 is **7.66 versus 1.37 cm**, rotation p95
  **2.560 versus 1.614°**, position p95 **64.21 versus 64.51 cm**. This is a
  partial improvement, not a localization pass. Frozen terrain scoring follows.
- A/B comparability audit `revision-5333171193464ae3a7a7b95d79e1965e` passes:
  identical prepared inputs, probe/dependency hashes and every estimator setting
  other than `imu_topic`. No noise, initial pose or assistance changes.
- Same-capture GT terrain control `revision-6f3b5457bafd419083bc1fcbe336cb0f`
  reaches **95.042% flat free / 0.500% false occupied**, 12/12 objects, zero
  sequence positive/dropoff false-free. Both measured/inferred variants pass
  the development geometry and sampled compute gates. This is phase 0 on the
  analytic development course only. Four estimated-pose terrain runs follow
  in fixed order: legacy first-published, legacy revised, physics first-published,
  physics revised. Core/reference/grid/evidence and safety gates stay unchanged.
- Legacy first-published terrain `revision-a53defe84ff54dbfa7888b6e6607fc32`:
  **0% flat free / 93.716% false occupied**, 12/12 objects and zero sequence
  positive/dropoff false-free. Coverage and false-occupied gates fail. The
  legacy revised-pose terrain run is `revision-474251d651944217bd0a746a5208cca8`.
  Estimated-pose compute is intentionally excluded from acceptance.
- Legacy revised finishes at **0% flat free / 94.689% false occupied**, 12/12
  required objects, zero sequence positive/dropoff false-free.
- Physics first-published `revision-f1853dc91143474b82755de6ff671a73` reaches
  only **2.542% flat free / 86.976% false occupied**, **10/12** required objects
  detected at the fixed reference locations (IDs 4 and 9 absent). Zero sequence
  positive/dropoff false-free, but coverage, false-occupied and recall fail.
  Higher-rate IMU is **not a terrain fix** on this controlled replay. Physics
  revised run is `revision-05c2acb3c4504e058d28e0e37974aa5f`.
- Physics revised completes at **37.548% flat free / 9.567% false occupied /
  52.884% unknown**. Required-location recall is **0/12**; final/worst sequence
  positive false-free is **3.874%** and dropoff false-free **300 fine reference
  cells** (2 cm reference, not 300 mapper cells). Both measured/inferred variants
  fail recall, coverage, false-occupied and positive/dropoff sequence safety.
  This is NOT an improvement eligible for promotion. A visually less occupied
  map can conceal displaced hazards. Missing GT-location overlap does not prove
  no obstacle evidence exists elsewhere in the estimated map; persistent
  horizontal errors require separate relative-versus-absolute diagnosis.

Final phase-0 comparison (both measured and inferred variants agree on these
metrics; every row uses all 1,417 native chunks and the unchanged original GT):

| Pose source | Flat free | Flat false occupied | Required objects at GT locations | Worst positive false-free | Worst dropoff free (2 cm reference cells) |
| --- | --- | --- | --- | --- | --- |
| GT | 95.042% | 0.500% | 12/12 | 0% | 0 |
| Legacy first-published | 0% | 93.716% | 12/12 | 0% | 0 |
| Legacy revised | 0% | 94.689% | 12/12 | 0% | 0 |
| Physics first-published | 2.542% | 86.976% | 10/12 | 0% | 0 |
| Physics revised | 37.548% | 9.567% | 0/12 | 3.874% | 300 |

Visual/reference verification: exclusive-create `terrain_comparison.png` files
were generated and inspected in the GT and physics-revised terrain runs. The
revised map contains recognizable **displaced** obstacle groups, not an absence
of all obstacle detections. Its 0/12 is strictly required-location overlap, not
an assertion that LiDAR never saw any rocks. The original fixed reference and
home are retained, with no best-fit translation, tolerance dilation or mask
change. Coverage diagnosis in the physics-revised run attributes 17.150% of all
flat reference cells to `negative_edge` unknown and 14.335% to `positive_edge`
unknown; only 0.251% lacks returns. Thus most remaining missing coverage is not
simply insufficient LiDAR sampling. These GT-labelled fractions are diagnosis
only and never mapper input.

Decision and next bounded steps:

1. Retain the physics stream as an **opt-in input-quality candidate**, not a
   production terrain fix. No tuning, new capture, fitted alignment or repeat
   selection was used to improve this failed result. Both completed A/B arms
   and all five terrain results remain archived; these are development data.
2. Audit the actual ROS-full-scan/GLIM timing and deskew contract next. Native
   chunk GT alignment is validated, but the estimator still receives rolling
   scans as global-shutter clouds without point acquisition times. Quantify
   that discrepancy on this recording before changing SLAM settings.
3. Separately diagnose the persistent XY/tilt errors and absolute-assistance
   residuals on these frozen trajectories. Add a **parallel, evaluator-only**
   robot-relative check using measured transforms (not a fitted alignment) to
   separate common pose displacement from locally warped hazard geometry.
   Keep fixed-world recall and sequence safety gates; never relabel this run
   as accepted by switching the scoring frame or relaxing the denominator.
4. Then test registration-aware, temporally coherent terrain evidence against
   the four existing varying-registration failures and the unchanged 8 cm
   object/whole-cell support fixtures. Do not solve false obstacles by merely
   allowing stronger height variation: the revised-pose safety failure shows
   why lowering occupied evidence can expose genuinely unsafe free cells.

Validation: **248 focused tests passed, four known strict registration xfails,
no skips**, plus the standalone compiled replay executed both full recordings
with exact callback counts, continuous revision chains and all native brackets.
These tests do not turn failed terrain gates into acceptance. Simulator stayed
off throughout this A/B; both estimator processes and all terrain replays ended.
Production configuration, installed estimator binaries and mapper core unchanged.

#### Paired moving IMU comparison [completed diagnostic; estimator A/B next]

- Continuing the declared input-quality gate, not changing terrain thresholds or
  promoting the physics stream into GLIM. NVIDIA/CUDA context/allocation and the
  external archive preflight pass; simulator was stopped before preparation.
- Added an equal-window 30/60 Hz comparison: identical bounded GT acceleration
  windows and rotation-increment endpoints, base-pose interpolation before the
  IMU lever arm, no fitted time shift, no residual-based exclusions. Missing
  samples, clock resets, unpaired diagnostics and invalid sensor phase fail the
  audit. GT remains evaluator-only; its 30 Hz sampling limits this diagnostic.
- Added `run_shadow_capture.py --require-physics-imu`: require one contiguous
  second of exactly paired 60 Hz samples/clock diagnostics before granting the
  existing motion lease, then fail closed on stream loss/reset. Historical
  captures remain valid without this opt-in. Frozen GLIM input stays legacy.
- New contracts in `tools/test/test_paired_imu.py` cover equal windows, a known
  decimation alias, missing ticks, exact clock pairing and readiness faults.
  Added rotated mount/lever-arm and actual ROS message/MCAP readiness contracts.
  Final focused regression result is recorded below.
- Started recorder `revision-a1e0eeda0e1c48cb95a064438668946f` before simulator
  capture `revision-49ade291e2f5423caeb35310aa88ba68`, with the declared 5-degree
  `wide_sweep15_return` route and required physics-IMU readiness. The frozen GLIM
  configuration still subscribes to `imu`. All six views and bounded acquisition
  completed; no readiness loss, command-ownership fault or publisher reset.

Completed evidence:

- **Native audit:** all 1,417 chunks hash-match; observed scan rate 9.995 Hz,
  no duplicate reads, no missing current-pose ray hits, maximum current-pose
  geometric residual **0.0544 mm**. This is the existing analytic development
  course, not natural-terrain or held-out acceptance.
- **Trajectory export:** `revision-f3a9013443094c85918ba279ae7461a6`, 496 revisions,
  497 frames, one session/epoch. Pair audit
  `revision-8d34a0daf0264874b8b5050261500e7f` passes every chunk and capture link.
  Legacy-input GLIM first-published position/orientation p95 is **68.37 cm /
  4.897 degrees**, revised **59.30 cm / 2.775 degrees**. This separate acquisition
  is worse than the prior one; do not attribute that to the unused physics
  stream or substitute runs to claim improvement. Same-bag A/B is necessary.
- **Initial paired audit:** `revision-81333b5367054f4bb3c86d019dfdabc7`. All 2,833
  physics samples have exact clock-diagnostic partners and contiguous 60 Hz
  cadence. All 1,417 legacy samples are contiguous 30 Hz. Use 1,405 identical
  0.4 s acceleration windows and 1,415 identical gyro intervals. Twelve boundary
  samples lack full acceleration windows, and one legacy gyro interval lacks a
  strictly common endpoint bracket; neither is excluded by residual magnitude.
- **Expanded serialization diagnostic:** `revision-e3a8bdfb86ff4e54af945c766819ab87`.
  Initial exact timestamp comparison covered 710/1,417 legacy samples. The other
  707 differ by **one nanosecond** (Isaac graph conversion versus Python rounding),
  not a physics tick. Added a fixed, tested 1 ns coincidence-only diagnostic:
  every legacy acceleration and gyro vector equals its physics-rate counterpart
  exactly. This does not retimestamp data, change integration windows or apply a
  fitted delay. The first report remains archived; every kinematic residual is
  unchanged in the expanded report.

| Identical-window residual (norm p95) | Legacy 30 Hz | Physics 60 Hz |
| --- | --- | --- |
| Acceleration, m/s² | 0.374001 | 0.0281125 |
| Gyro increment / interval, rad/s | 0.0265242 | 0.0171704 |

The acceleration discrepancy falls **13.3×**; gyro discrepancy falls **35.3%**.
This is strong evidence for loss/aliasing from playback-rate decimation in this
fixture, not a whole-message timestamp shift or axis mismatch. The reference is
still sampled GT and finite-window integration, not calibrated sensor metrology.
No terrain replay was repeated for this capture, and no new terrain pass claimed.
The estimator remains explicitly **synthetic-RTK-assisted**: GT supplies the
declared noisy simulated RTK sensor, while exact GT poses and terrain labels are
evaluation-only. Audit `gt_used_by_estimator=false` refers to direct exact-pose
input, not the absence of that already-declared assistance.

Next bounded implementation:

1. Implement a new isolated, external-archive, same-bag GLIM replay harness.
   Do **not** use `rtk_replay_ab.sh` as-is: it regenerates RTK from GT, uses an old
   home/mount and fitted trajectory scoring, writes to the workspace, and has
   broad process cleanup. Those are inappropriate for this experiment.
2. Freeze one post-teleport warm-up/acquisition/postroll interval and all recorded
   filtered clouds, accepted absolute measurements, configuration and ordering.
   Replay legacy versus physics IMU with only that declared input choice changed;
   no GT publication, newly generated RTK, refitted initialization or timestamp
   shift. Record estimator revisions, received-input counts and timing; require
   every native tick to remain bracketed. Retain failed attempts and fixed order.
3. Compare both pose streams to the same qualified native GT, using fixed home
   and mount. Keep first-published and revised outputs separate. Only then repeat
   the unchanged terrain sensitivity replay, including sequence safety gates.
4. Full-scan rolling acquisition/deskew and registration-uncertain terrain fusion
   remain separate open contracts. A higher IMU rate alone does not close them.

All new archive outputs are host-user-owned. Simulator, driver and estimator are
stopped; no hardware reset, production input switch, mapper edit or gate relaxation.
Final sourced-overlay regression: **234 passed, four known strict registration
xfails, no skips**. Includes actual ROS/MCAP callbacks, paired IMU arithmetic and
clock/serialization guards, shadow/native replay provenance, driver/sensor checks,
localization-supervisor contracts and frozen-core registration cases. Root and
OmniLRS `git diff --check` pass. The four xfails remain an open safety/usability
problem, not acceptance.

#### Frame/registration audit follow-up [in progress]

After the user challenged the 95%-to-zero result, stopped acquisition/tuning and
audited the existing recording. All 497 published GLIM LiDAR poses match the
revision pose chain to floating-point precision. Independent ROS GT base poses
plus the LiDAR mount reproduce all 1,419 native GT poses (maximum 0.51 mm /
0.045 degrees). All recorded revisions are odometry updates, not global graph
corrections. The estimated-pose map itself has no free cells before GT scoring.

Actual floor returns in cell (2.05, 2.05) span approximately -12 to +9 cm under
first-published poses, versus zero under GT; the mapper fits ground at -5.4 cm
and reports a 14.7 cm obstacle. In the final 30 s, 96% / 92% of sampled flat-return
cells span more than 8 cm for first-published / revised poses. These explanatory
subsets do not replace the fixed scoring denominator or enter mapper inputs.
A frozen-core analytic control reproduces the mechanism: constant +5 cm offset
leaves 100/100 interior cells free, whereas scans at -5/0/+5 cm make all 100
occupied. Plane-fit inlier sigma does not include registration uncertainty.

Execution order now:

1. Make frame and IMU/timing diagnostics reproducible on the existing bag;
   distinguish input-wiring defects from estimator behavior, without fitted
   alignment, selecting favorable delays, or changing the recording.
2. Add permanent constant-versus-varying registration regression fixtures,
   preserving the known failure explicitly rather than calling it acceptance.
3. Only then evaluate temporally coherent local surface estimation/fusion;
   keep the 8 cm obstacle requirement, original reference and safety gates.
4. New captures, tuning and promotion remain deferred until the above evidence
   supports a specific change. Production mapper/configuration unchanged.

Implemented follow-up (same recording, no new simulation):

- `tools/test/test_terrain_registration.py`: 20 passing fixed-gauge and real
  8 cm object contracts across four grid phases; four strict **xfails** preserve
  the unresolved varying-registration false-obstacle case. Unknown is acceptable
  for that diagnostic, but real obstacles must still be occupied. These failures
  are not terrain acceptance and no mapper constants were changed.
- `tools/imu_consistency.py` and `tools/audit_shadow_imu.py`: hash-linked,
  external-archive-only bag evaluator. Full sensor lever arms, bounded SE(3)
  interpolation, gyro rotation increments, and triangular acceleration integrals
  against GT position second differences. No fitted alignment, delay, axis,
  gravity or scale; GT never enters GLIM. Twelve analytic math/clock contracts
  pass. Acceleration windows omit only the explicitly counted boundary samples
  lacking a full +/-0.2 s window, not high-error intervals.
- First archived diagnostic `revision-ba0c6b1dab5f4ffaaeb8e9f9b4eb529b`:
  all 1,419 IMU samples are 30 Hz while physics is 60 Hz. Gyro-increment residual
  p95 is 0.02818 rad/s; acceleration consistency residual p95 is 0.4088 m/s^2.
  The latter is a finite-cadence kinematic diagnostic, not a calibrated noise
  estimate or proof of one cause. Independent GT and revision/published-pose
  checks reproduce the previous read-only audit. Filtered GLIM clouds contain
  XYZ only; global-shutter mode is still enabled despite accumulated scans.
- Source inspection of the robot USD (standalone USD Python, no simulator/GPU)
  identifies a timing concern: `ROS_Camera/isaac_read_imu_node` is executed by
  `OnPlaybackTick`, but the publisher timestamp connects to
  `IsaacReadSimulationTime.outputs:simulationTime`, **not** the reader's
  `outputs:sensorTime`. Sensor period is -1 (physics cadence); installed reader
  schema defaults `useLatestData=false`. Do not blindly substitute sensorTime:
  its relation to monotonic ROS time across World.reset must first be measured.
  Added three explicit header/previous-physics/previous-render orientation
  hypotheses to the audit, reporting all of them without selecting/applying a
  fitted offset. The initial artifact remains unchanged.

Further isolation changes the interpretation of that timing concern:

- Expanded bag audit `revision-de70c0fbe52b4551bb4075b23de5d9d2` reports
  orientation-error p95 0.002637 rad at header time, 0.001322 rad one physics
  tick earlier, and 2.36e-7 rad one render tick earlier. **Do not infer that
  acceleration and gyro share the orientation's lag.** GLIM ignores the IMU
  orientation field.
- Added `tools/isaac_imu_clock_probe.py`, an isolated rotating-cube component
  fixture, no robot/ROS/estimator, 90 render ticks on either side of World.reset.
  Host and same-container CUDA probes pass. Retained all three instrumentation
  versions: `revision-8a67658a591a45b99f8581a84c8b37b2` (render observations),
  `revision-b8c5824b708e4ecda97680d7f6f943d3` (adds every physics callback),
  `revision-829d4f0b82c444b9a2f018a1cc67d97e` (adds actual body velocities).
  The first version's unused graph clock output remains zero and is NOT clock
  evidence; subsequent versions remove it. Direct core monotonic time is
  recorded in all versions. No version is terrain/SLAM acceptance.
- All 360 final-fixture physics samples match actual rigid-body angular velocity
  to 1.2e-16 rad/s, even though sensor orientation lags and sensorTime labels the
  preceding physics interval. World.reset resets sensor time but not monotonic
  time. This **rules out blindly shifting the whole IMU message by 33 ms or
  copying sensorTime into a ROS header** as a justified fix. The recorded bag's
  acceleration residual still needs full-rate moving evidence to distinguish
  decimation, sensor/rigid-body behavior and estimator error.
- Implemented opt-in `OMNILRS_PHYSICS_IMU=true` shadow publisher on
  `/astrobot_0/imu_physics` plus `/clock` diagnostics (under that topic). It reads
  each physics callback, uses the independently checked current monotonic clock,
  checks the version-scoped sensor-clock phase, and terminates its stream on a
  stale/skipped/invalid timing contract. Orientation is explicitly unavailable
  (`orientation_covariance[0]=-1`). Legacy IMU, GLIM configuration and normal
  launches remain unchanged. Recorder includes these topics optionally.
- Twelve host clock/payload/fault/capture-cadence contracts pass. A bounded
  stationary real-robot wiring check is now in progress, with no navigation,
  estimator or command publisher. This checks 60 Hz delivery alongside the
  legacy 30 Hz stream, not moving accuracy. New tool
  `tools/check_physics_imu_capture.py` records all small JSONL samples exclusively
  on the external archive. Terrain algorithm changes remain deferred.

First stationary attempt `revision-d21470704cee49e9b51c349188319a6d` remains
**failed**: the new stream delivers 601 consecutive 60 Hz samples with no clock
fault, but the evaluator incorrectly includes a legacy startup-reset gap
(3.4333 -> 3.5000 s) before the new publisher starts (3.4833 s). Fixed the checker
to declare the intersection of stream-availability intervals, retain every
startup sample, report exclusions and the full-record maximum gap, and still
reject any gap inside the common interval. Added a regression for this specific
startup ordering. No sensor thresholds changed; thirteen host publisher/checker
contracts now pass. The failed recording is not relabeled; a fresh stationary
check is in progress. The combined sourced suite before this last fixture had
163 passes and four known registration xfails.

Fresh stationary check `revision-7d8227bf80894cd5ae2058a49eb0990b` **passes its
limited wiring gate**: common interval 3.483333515–13.466667369 s, 600 physics
samples / 300 legacy samples, maximum cadence error 1.34e-9 / 2.67e-9 s,
600 clock diagnostics and no command publisher. All 400 legacy / 601 physics
recorded messages remain in JSONL; the 100 startup legacy samples and one trailing
physics sample outside the fixed common interval are reported, not deleted.
Orientation covariance is explicitly unavailable on the new stream. GLIM has
not been switched. The failed prior capture remains ineligible.

Added a runtime-callback regression showing why full-rate delivery matters:
alternating physics-step accelerations integrate to zero when both steps are
retained, while render-rate decimation aliases them into a nonzero velocity.
This is an analytic mechanism test, not proof of the moving bag's error source.
Latest focused sourced run: **165 passed, four strict known registration xfails**.

Final expanded verification adds the existing sensor-audit, calibration-driver,
GT-probe and localization-supervisor suites: **221 passed, four strict known
registration xfails**, no skips. Root and OmniLRS `git diff --check` pass.
The stationary simulator is stopped; no estimator, navigation or motion process
was started for this check. The three component-probe runs have been made
host-user-owned; other new archive runs were created as the host UID/GID.
No terrain library, obstacle threshold, production GLIM input or recorded
historical fixture was changed. Physics-rate IMU remains explicitly opt-in.

Next decision gate (no terrain threshold changes):

1. One guarded moving paired capture on the already declared route, collecting
   legacy and physics-rate IMU simultaneously; keep GLIM on legacy input for
   this first comparison. Recorder now includes both optional new topics.
2. Extend the kinematic evaluator to compare both acceleration/gyro streams over
   identical GT-bounded windows, without using the unavailable new orientation
   field. Require complete physics cadence and unchanged source/extrinsic checks.
3. Only if measured consistency supports it, replay the same recorded LiDAR with
   legacy versus physics IMU into GLIM using frozen settings and no fitted GT
   correction. Keep all attempts. Full-scan rolling acquisition/deskew remains
   a separate open input contract; changing IMU rate does not solve it.
4. Then evaluate coherent-local terrain evidence against the retained core,
   fixed reference, original 8 cm requirement and all sequence safety gates.
   The four registration xfails must not be hidden or counted as acceptance.

Rechecked runtime after interruption: archive now mounted as `/dev/sdb1`, NVIDIA
580.178.04 visible, CUDA context/allocation probe passes. Simulator is stopped.
The September 19 Docker permission request was interrupted; no capture started.

New integration issue: disabling the legacy mapper is insufficient for a shadow
launch. The localization supervisor waits for `reset_map`, then for map readiness
and Nav2 clear-costmap services. Implement an explicit opt-in shadow mode that
can initialize/validate poses without those services but **never grants navigation
motion permission**. Production behavior must stay unchanged. Any shadow fault
must terminate that estimator session, not silently restart during acquisition.

Implemented opt-in supervisor/launch mode (19 ROS callback contracts pass), and
capture-specific dual-clock readiness leases (43 host lease/driver/GT contracts
pass). Navigation motion permission remains false even when shadow poses are
healthy. Native acquisition waits for the grant; grant expiry, reset or identity
change terminates acquisition without weakening driver clearance checks.

Runtime issue found before capture: the host drive is accessible but the old
`astrobot-revision-dev-v2` bind mount does not expose the prepared configuration
after reconnect. Do not reuse it or fall back to root disk. A fresh archive-bound
dev container is required; existing artifacts and old containers stay intact.

Fresh `astrobot-shadow-dev` verifies the prepared config hash and external-drive
capacity; three changed launch/supervisor packages build successfully over the
frozen revision-capable overlay. Added the coupled recorder with synchronous
MCAP writes, frozen run-local config, command-owner checks, and live USD/TF/graph
extrinsic agreement before grants. Estimator startup waits for the simulator's
post-teleport/static-calibration request. Late heartbeats cannot revive an expired
lease. Initial sourced verification: **89 tests pass**, including actual MCAP
finalization, supervisor behavior, geometry/lease and C++ reconstruction contracts.
No mapper thresholds changed. Moving capture and scoring are still pending.

Expanded focused suite: **116 passed**. First live attempt is preserved as
recorder `revision-1008f371b1254dababb9d687740ccc3a` / simulator
`revision-057434b27ed94263905fed71c124ced8`, with **no grant or commanded motion**.
Two integration defects: the new dev container omitted NVIDIA runtime (selected
GLIM modules report insufficient driver), and readiness wrongly required a
`/tf_static` message although Isaac publishes the sensor chain on `/tf` only.
Require live transform agreement while retaining optional static-TF recording;
add same-container CUDA preflight before launching GLIM. SIGINT also exposed a
double ROS shutdown exception that prevented the final summary; use idempotent
shutdown and retain an explicit orchestration-failure artifact for this attempt.
No host driver changes or resets were performed; the simulator was stopped.

The replacement `astrobot-shadow-gpu-dev` passes the same-container CUDA
context/allocation probe and rebuilds all three changed packages. Focused
verification now totals **118 tests** (117-test run plus the added passing
terminal-fault contract). Second collection started at
`revision-816d4e50f04d4cb38f162d1bcd9d20ad`; outcome pending. The recorder now
retains missing-topic/status diagnostics and stops the estimator while still
recording trailing clocks, rather than risking an unbounded final revision.

Second attempt failed **before any grant**, correctly: stationary live TF agrees
with USD within 1.2 micrometres, but the millimetre-rounded IMU origin differs by
**0.547 mm** (rotation residual only 6.6e-8 rad). Sensor and revision checks both
reject it under the unchanged 1e-4 matrix tolerance. Its paired simulator run is
`revision-ddd7ee425b2e423bab71e7af75d2647e`; neither is eligible for moving scoring.
Read-only authored-USD inspection also confirms that the D455 fixed joint is
resolved by physics: do not use its pre-physics authored world pose as calibration.

Recorded the precise settled USD/TF IMU origin with evidence IDs in the audit
utility and opt-in shadow RTK lever arm; production launches are unchanged.
Prepared config `revision-2bb48a78e85f47afa51119bdd59ac766/candidate-config`
changes only `T_lidar_imu` (sensor SHA `b9d90e910a7ad5fa6607982a3108e3f1d6e088670b5a280119ec736ff1611e12`).
The successful stationary sensor/TF prefix is calibration evidence only, not a
promotion of the incomplete acquisition. A fresh live check remains mandatory.
Archived mount-audit tolerance now matches the strict live matrix gate; it is
not loosened to accommodate the old rounded value.

Archive portability issue also exposed: historical `check_run` contracts retain
the old device number after `/dev/sda1` becomes `/dev/sdb1`; audit attempt
`revision-946f1397c34e4acdbfd294eefdc3f493` fails with `archive device changed`.
Historical contracts were not rewritten and active-run device checks remain
strict. The new calibration preparation uses the current stationary prefix;
safe read-only validation of relocated historical contracts remains open.

Precise-mount focused suite: **119 passed**. Third recorder
`revision-6f2b4a51563446a28b93e1855b3eb24c`, simulator
`revision-9164a9a435c94f26abd6192710ff8c88`, is in progress after fresh host and
container CUDA checks. Added paired-pose qualification with nine passing host
contracts: full native sequence, fixed home, first-publication availability,
retrospective corrections, no extrapolation, and strict bidirectional archive
identity/config/driver checks. This diagnostic precedes terrain replay; it does
not claim causal-map latency or noisy-terrain acceptance.

Third attempt passes the strict live mount checks (worst USD/TF matrix error
8.8e-7, config/graph 6.6e-8), grants recording, then stops after view 1/6 with
**zero nonzero commands**. Bag trace isolates the failure: the synchronous
1,000-ray evaluator stalls a stationary simulation step; the request heartbeat
ages to 0.567 s while the clock age is 0.442 s and the localization pose remains
current/healthy. This is capture scheduling, not demonstrated SLAM drift.
Both failed runs remain ineligible; the simulator is stopped.

Implemented shadow-only incremental viewpoint auditing: 100 ray pairs per held
tick, every original selected ray and ambiguous native acquisition candidate
retained. The declared two-scan sample is reserved before processing; later
unselected scans are explicitly counted, not silently treated as selected drops.
The driver still checks settling each tick. No freshness/watchdog, range-error,
route deadline or coverage gate is relaxed. This scheduling changes stationary
acquisition duration and must be recorded as such, not presented as an identical
old acquisition dose. Added bounded-work/missing-candidate/frame-quota contracts
and named readiness-failure diagnostics; next fresh capture is pending.

Bounded-audit focused verification: **138 passed**, followed by one additional
passing fixed-home cross-link contract (139 in the combined set). Fourth recorder
`revision-39d4a3b8aea34f799a4d674555eea2da` / simulator
`revision-3e6f76ab49114559a6500284a1de867a` has passed live qualification and
reached view 4/6 with assisted GLIM healthy, including both translation endpoints.
Collection completion, native audit and paired scoring are still pending.

Fourth acquisition completes **all six views**, with 1,414 native chunks. The
independent native audit passes cadence (9.995 Hz), zero missing rays, 180,992
current-pose checks and **0.0281 mm maximum residual**; previous-pose association
fails badly, as expected. However, the paired recorder fails at shutdown:
`sole_driver` becomes false between driver destruction and completion-file
publication. This collection remains failed/ineligible; no success flag was
rewritten based on the otherwise complete course.

Fixed terminal protocol: publish an atomic, exclusive acquisition-finished
marker after zero command and all six views, then seal driver/native files before
the final completion marker. Recorder stops granting motion during sealing,
rejects any subsequent nonzero command, and waits a bounded postroll for a pose
bracket beyond the final native tick before estimator shutdown. This adds no
native evidence and avoids extrapolating the route's tail. Regression contracts
cover normal publisher teardown, incomplete markers and exclusive publication.

All **142 focused contracts pass** after terminal-protocol changes. Fifth
recorder `revision-8454800d429d43b19373c17bd31e585c` has started after fresh
host/container CUDA checks, with the same precise config, route and safety gates.
No failed collection or score has been relabeled as passing.

Fifth collection **completes and finalizes successfully**: simulator
`revision-1802f888579a499a9fb6002fc44c1250`, 1,419 native chunks, 497 revisions /
498 graph frames. Native audit: 9.995 Hz, no skipped/duplicate ticks or missing
rays, 181,632 current-pose tests, 0.0283 mm worst ray residual. Trajectory export
`revision-b7c8d4391af047f39c27a595cb26da03` passes all next-clock arrival bounds;
the simulator is now stopped.

Full-sequence pose association passes without dropping chunks or fitting an
alignment. Latest hashed pose bundle: `revision-51521972d2e1442fb6d33295eaa8bf67`
(earlier explanatory outputs `revision-d00c1a752de042b7bd3011d9f7f3fe99` and
`revision-82af1837337c4007970fd0ed8a5964ff` remain preserved). The accuracy result
is concerning despite the supervisor's healthy state:

| Native-pose mode | Position p95 / max | Orientation p95 / max | Availability |
| --- | --- | --- | --- |
| First published | 24.36 / 31.60 cm | 2.72 / 3.11 degrees | p95 133 ms, max 167 ms |
| Final corrected | 22.26 / 25.46 cm | 2.05 / 2.31 degrees | retrospective, roughly 4.9 s |

First-published vertical error p95 is 12.71 cm and horizontal error p95 24.08 cm;
world-axis rotation-vector p95 components are approximately 2.64/1.43/0.72 degrees.
These are fixed-anchor errors, not proof of a closed-loop collision, and a healthy
status is not a small-obstacle mapping accuracy certificate. Fixed-lag corrections
matter even with global/submap optimization disabled.

GT-only replay on the same new native sequence passes this **single phase-0
development fixture**: 95.172% flat free coverage, 12/12 objects, zero sequence
positive/dropoff false-free, sampled compute gates pass
(`revision-10c3e512d1134229b57283fba33d42d1`). Added an explicit offline pose-only
sensitivity adapter for first-published/final-corrected estimates. It preserves
all input points/timestamps/projection ticks/reference/gates and excludes live
causal/compute acceptance; delay scheduling and revision rebuild are not modeled.
GT regression after the adapter (`revision-a8c464af4a354b82952a7a03d4304a35`)
matches **all 474 per-frame records and final six-layer arrays exactly**. Paired
estimated-pose replays are running; no SLAM or terrain tuning has been applied.

Estimated-pose replays complete and **both fail terrain coverage**:
first published `revision-4abaa441179d4d25a7a57e7243c87b58`, final corrected
`revision-daf0f463d1514f2e9a78e23c97efdc8d`. Final free-ground coverage is zero;
false occupied is 95.317% / 94.759%. Fixed-reference `coverage_diagnostics.json`
in each run confirms the occupied cells carry reason `obstacle`, rather than a
whole-cell-support rejection. Peak free coverage was only 1.041% / 4.687%, and
all remaining free cells disappear by simulation time 30.9 / 29.0 s. No thresholds,
evidence lifetime, reference mask or point filters were changed to improve this.
These are unusable maps despite 12/12 recall and zero hazard false-free: counting
everything occupied trivially preserves recall. Estimated-pose timing remains
excluded from compute acceptance. The initial GT run's inferred p95 is 47.31 ms;
its exact regression repeat is not a substitute best-of timing result.

### Next bounded implementation steps

1. Freeze this coupled fixture and the three pose/map results. Add reduced
   reproductions of flat-ground-to-obstacle contamination using measured pose
   perturbations, retaining the existing 8 cm object and hole safety cases.
2. Audit recorded IMU axes, units, measurement stamps and gravity against GT
   derivatives, then quantify un-deskewed rotating-scan distortion. The configured
   lunar gravity is 1.62; an upstream generic acceleration warning alone is not
   evidence of a gravity-config fault. Do not guess a GLIM tuning fix from it.
3. Separate pose-error propagation from terrain-model sensitivity: diagnose
   registration residuals, correlated height spread and ground-layer selection.
   Compare local/submap evidence and uncertainty-aware accumulation with the
   frozen long-window baseline, never blindly relax the obstacle threshold.
4. Test fixed-lag correction consumption explicitly. Final corrected poses still
   fail this fixture; a revision-aware map alone is not demonstrated sufficient.
   Preserve causal receipt times and bounded rebuild/installation requirements.
5. Only after the baseline failure is understood, proceed with declared upstream
   LiDAR noise, RTK outages, LiDAR/IMU-only, sloped and natural held-out routes.
   No new physical acquisition dose or favorable retry replaces these failures.

The approved next stage freezes conservative core `69d182ca...` and moves to
GT/causal-GLIM/revision-corrected comparisons; exact 95% coverage on every old
offset is not a prerequisite. Current assisted GLIM is the primary baseline.
Scripted acquisition remains the only motion owner; experimental terrain must
not publish to production costmap or command topics. C3 modeling work is
deferred unless broader tests show a material navigation limitation.

Execution order:

1. Implement acquisition/arrival-time, SE(3), session/epoch and revision association
   contracts; native chunks only, no single-pose moving ROS scans.
2. Add reproducible pre-estimator radial-noise/dropout fixtures, reconstruction
   reference and paired error reporting. Do not claim a noise experiment unless
   the same perturbed evidence actually enters both GLIM and terrain mapping.
3. Audit tilted USD/TF/GLIM extrinsics and current GLIM scan timing. Couple guarded
   native acquisition to a revision/raw/IMU/RTK recorder before any moving capture.
4. Collect the existing 5-degree/six-view 4.8 m route with assisted GLIM; compare
   identical evidence under GT, causal estimates and revisions. Then perform the
   declared outage/noise variants and guarded sloped/feature-poor routes.
5. Test installed Nav2 inflation/footprint behavior in an isolated harness, retain
   raw-cell and robot-level safety separately, and update the HTML report.

Preflight: external `/dev/sda1` archive mounted; `nvidia-smi` reports 580.178.04
and the CUDA context/allocation probe passes. Simulator and navigation are not
running. No new capture has started. Initial finding: receipt order/time must
be retained separately from measurement time; final graph poses cannot stand in
for causal GLIM output.

Implemented so far:

- `tools/shadow_terrain.py`: atomic revision timeline; first-published versus
  revised trajectories at a causal receipt cutoff; 0.2 s maximum interpolation
  gap; session/reset barriers; IMU-first interpolation then LiDAR lever arm;
  deterministic per-chunk radial-noise/dropout; fixed-anchor SE(3) error and a
  full-prefix C++ reconstruction oracle. Cropping raw input at 30 s is **not** a
  valid shortcut for immutable bucket boundaries; bounded revision rebuild is
  still pending, not silently approximated.
- `tools/export_shadow_trajectory.py`: fresh-run MCAP export with source hashes
  and next-/clock receipt bounds. Historical stationary fixture
  `revision-3c92fe9c265949b2acf69873f2679ffd` passes: 496 revisions / 497 frames,
  output `revision-8185df997c00463aafe2bb6494c53707`. This is not moving evidence.
- `tools/audit_shadow_inputs.py` found that the production GLIM extrinsic is
  untilted, whereas the proposed capture has 5-degree pitch: **0.0872665 rad and
  0.0119106 m mismatch**. Failure preserved in
  `revision-4525b8d8a71e46588128d847ef2ac500`. The opt-in prepared configuration in
  `revision-58ec915d60bf45029827d0c75fb709d5/candidate-config` changes only
  `sensors.T_lidar_imu`; production remains unchanged. Candidate sensor-config
  SHA-256: `3e3328df8b00a8742f5a213f2c451180fad8d816e488677d1e76be1f7a3d8432`.
  Preparation preserves the failed baseline verdict; live IMU TF remains required.
- Source audit confirms `global_shutter_lidar=true` zeros every preprocessed
  point time. Missing incoming point times otherwise receive synthetic ordering
  times from GLIM. Neither behavior validates actual rotating-scan acquisition
  timing; keep this unchanged and labeled in the baseline, not as valid deskew.
- Candidate mount verification passes in `revision-b07a21a9367448a6a3794e34856d5231`
  (zero translation error, rotation residual 6.94e-18 rad). This verifies the
  prepared configuration against archived geometry, not new live TF.
- Installed-Nav2 inflation/footprint harness now builds against the consistent
  Jazzy 1.3.11 prefix. It invokes actual `InflationLayer` and perimeter collision
  primitives; the StaticLayer's cost translation is reproduced in the harness,
  not an end-to-end StaticLayer/planner/controller launch. No map/cmd publishers.
  The soft gradient, lethal center and unknown-preservation checks pass.
- The stronger wall check **fails**, including for the retained core:
  `revision-3e424dfa23784c57a6981fdeb3bf5661` retains layered/inflated arrays and all
  1,932 pose queries. 1,153 are accepted by the tested center/perimeter rule;
  three intersect the continuous wall with the padded footprint, two with the
  configured unpadded XY body rectangle. Center costs are 181/221/245 and edge
  cost 253: expensive, not hard vetoes. The completely missed-wall control
  admits 442 body intersections, confirming inflation cannot recover absent
  hazards. This is **not a demonstrated closed-loop collision**, and does not
  make this minor fixture a prerequisite for moving-SLAM diagnosis. Both frozen
  cores' zero-intersection assertions are explicitly xfailed, not reported passed.
- Harness environmental failures remain archived: host-UID ROS log permission
  abort in `revision-59dd15763ec94f65800de6b32e2eb708`, then stale container DDS
  interface in `revision-fbd0528d3ab849a19b9d913640c99668`. Fixed only the probe
  child environment (no external log files, explicit loopback DDS domain 179);
  no production DDS settings or ownership were changed. Mixed-prefix CMake
  failure remains in `/tmp/nav2-shadow-build`; successful isolated build is
  `/tmp/nav2-shadow-jazzy-build` in both dev containers.

Verification at this checkpoint: **50 passed, 2 expected safety failures**, no
skips in the sourced archive-container run over the three new test modules plus
revision/archive/HTML contracts. Includes real C++ reconstruction, nonuniform
corrections and a fixture proving naive 30-second input cropping rejuvenates an
expired bucket. Test runners and standalone Nav2 build are in `AGENTS.md`.

**Not completed:** acquisition-time/deskew proof for current GLIM full scans;
live-causal/revision-aware native terrain scoring (the September 20 offline
pose-only sensitivity comparison is complete); noise injected into actual GLIM;
RTK-outage captures; sloped/natural routes; bounded live revision consumption.
The new perturbation functions are tested utilities, not a completed noisy-SLAM
experiment. Coupled native/GLIM collection, readiness gating and live 5-degree
USD/IMU/TF checking now pass. The simulator is stopped; the core is unchanged.

The earlier recorder/home/mount prerequisite is complete: the opt-in launch
declares home (-20,-20), never fits it to GT, and preserves bidirectional archive
links. The next implementation steps are the bounded diagnosis list above.
The broader plan remains in progress; this failed estimated-pose terrain result
does not establish moving robustness.

## Current outcome

**Good drivable-surface matching and autonomous-navigation readiness have not
been achieved under estimated poses.** September 20 paired evidence now takes
priority: the new route reaches 95.172% coverage with GT poses but zero final
free coverage with first-published or final-corrected GLIM poses, using identical
raw inputs and fixed scoring. The experimental mapper remains outside Nav2. The previous
driver mismatch and subsequent CUDA error 999 are historical: after the user's
September 13 recovery, the host **CUDA context/allocation probe passes** on
driver 580.178.04. No agent-initiated host reset/reboot was performed. The archive
is mounted. **The signed-wall candidate is rejected:** despite passing recorded
sequence safety, a new rotated-wall fixture exposes false-free cells. Its runtime
hook is removed; the retained baseline is `69d182ca...` (5-degree inferred coverage
94.526–95.397%). Diagnostics and stronger regression fixtures remain. Simulator stopped.

### September 14 continuation — diagnostic progress; unsafe candidate rejected

Implemented GT-free rejection telemetry and 108 flat/upward/downward face cases
across slopes, rotations and grid phases. The signed-wall candidate and its exact
speed optimization (`7a13403f...`) passed all recorded sequence-safety regressions
and both resource profiles, but **failed a stronger geometric wall probe**. It is
archived in `revision-c1f43b957ca64674bd40db79b0c5a847`, not retained or installed.
The failure/counterfactual is recorded in `revision-8f92c7fe09b54c0bbce69c4b60aa352f`.
The runtime is restored to the earlier conservative boundary guard. Its sign
diagnosis remains imperfect, but removing it without an upward-face extent guard
is unsafe. New evidence also exposes pre-existing sloped ground-plane selection
errors. These are the next modeling problems, not justification to relax safety.

| Subsystem | Implemented / validated evidence | Remaining work |
| --- | --- | --- |
| Localization safety | Supervisor, epoch reset, command gating, corrected pose/cloud frames; stationary recovery tests | Moving faults, outages and spontaneous SLAM failure rates |
| Revisions | Export/snapshot interfaces, immutable observations, archive, reconstruction reference | Live terrain consumer and coherent Nav2 installation barrier |
| Terrain | Continuous-height layers, partial-face and whole-cell support checks, bounded inference, clearance, immutable expiry | Coverage, acquisition sensitivity, native latency and real terrain |
| Evaluation | Independent GT geometry, native-tick audits, fixed denominators, sequence safety, immutable failed captures | Untouched physical/noisy/natural-terrain acceptance |
| Navigation | Global home goal, stale-action protection, bounded recovery and reachability | Shared footprint/support geometry and actual course validation |

Global mapping is enabled, but submap/global **optimization** stays disabled.
Fixed-lag corrections still matter. `SlamObservation` is lossy GLIM-preprocessed
evidence, not unfiltered native LiDAR. Moving full ROS scans remain un-deskewed
and ineligible for single-pose terrain replay.

### Historical acquisition baseline

Archive root: `/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab`
(Docker `/archive`). Historical whole-cell acquisition-comparison library:
`revision-756f0f4292784c20a55244f0ca017402/liblayered_terrain.so`, SHA-256
`9161265ae5fba5d1e95d8e97d68e1042f73d5a50ed44d22dd3d9f74d072dfccb`.
The existing ROS install is not this experimental library.

- Continuous 10-degree-mount sweep: 93.61–94.55% flat coverage across all four
  offsets, 12/12 required objects, zero sequence positive/hole false-free cells.
- Older 5/10-degree straight acquisitions: 79.35% / 84.22% coverage, all objects
  detected and sequence safety retained. Acquisition sensitivity remains severe.
- Dense-ground stress: 42.04/46.95 ms p95/p99, 312.2 MiB. Layer-capacity stress:
  44.77/51.04 ms, 994.7 MiB. Some native replays reach 51.21 ms p95 and fail.
- Baseline suite: 291 Python tests, 11 C++ tests, ASan/UBSan passed. Current
  route/preflight additions expand this to **308 Python tests**, all passing;
  the unchanged C++ build still passes all 11 programs.
- The older 96.11% result is not robust acceptance: shifted grids exposed hole-rim
  false-free cells. Exact runtime optimizations preserve the corrected maps;
  they do not improve their classification accuracy.

Full evidence and run IDs: [terrain status](gt-terrain-status.md),
[revision foundation](revision-aware-terrain-status.md), and
[autonomy repair](autonomy-repair-status.md).

### Latest implementation status — September 13

- **Implemented:** angular-support candidate and regression fixtures; guarded
  terminal-turn acquisition and independent route validation; bounded CUDA
  launch preflight; exact redundant-angular-check optimization. No live Nav2
  installation or new SLAM/revision consumer is claimed.
- **Retained experimental baseline:** frozen optimized angular core
  `revision-bf1fde8ac66a45979c61a8404232e8f0` (`69d182ca...`), with source snapshots
  and compiler flags. This is distinct from the retained `9161265a...` acquisition
  baseline and from the installed ROS mapper. It is not a promoted terrain model.
- **Rejected candidate:** boundary-witness fallback `8a38fdb9...`, frozen in
  `revision-fd55da08f58f492481b03e39947784ec`. It adds no final fixed-reference
  coverage and fails original-route latency. Its runtime hook has been removed;
  a test-only prototype and all archived evidence remain. The retained mapper
  path is unchanged; no terrain model has been promoted.
- **Accuracy remains blocked:** original sweep 93.65–94.59% free, older straight
  routes 79.50/84.43%, old settled rotation 33.61%; target remains 95%. All scored
  required-object and positive/hole safety checks pass, but that does not establish
  safety between old settled scans, on held-out terrain, or under SLAM error.
- **Runtime:** optimized original sweep passes all four sampled offsets,
  worst p95 **49.06 ms**, and spin p95 **43.48 ms**. Exact original-sweep/spin
  comparisons preserve all maps. Resource/sanitizer checks are recorded below;
  sub-millisecond native margin is not robust live timing headroom.
- **External blocker resolved:** host CUDA returned error 999 at the earlier
  handoff. The user's September 13 recovery passes the host and Isaac probes;
  the completed terminal-turn capture also passes its geometry audit. The failed
  pre-recovery attempt remains preserved and ineligible.
- **B2 is now completed:** baseline whole-cell mapper reaches **94.23–94.87%**
  after the final turn; every offset still fails 95%. The first 1,152 native chunks
  and poses match the old wide capture exactly. No further route tuning is selected.
- **Mount comparison completed:** retained `69d182ca...` reaches
  **94.526–95.397%** with the 5-degree mount on B2, versus **94.335–94.924%**
  at 10 degrees. The 2.5 cm offset still fails coverage. All eight variants retain
  12/12 objects, zero sequence positive/hole false-free and sampled compute passes
  (worst p95 **46.21 ms**, p99 **52.76 ms**, process RSS **234.75 MiB**).
  No mapper or sensor configuration is promoted from this development comparison.
- **Next implementation:** instrument whole-cell support and edge-guard rejection
  causes without changing decisions, then construct a minimal geometric fixture
  for the dominant failure before changing the classifier. No additional pitch,
  route, evidence-lifetime or grid-phase tuning is selected.

## Non-negotiable evaluation contract

- 10 cm output XY cells and continuous heights; 8 cm is a height hazard threshold,
  not an XY footprint or vertical voxel size. Retain all 12 required objects.
- Original 138,725-cell flat reference; unknown/outside-map cells stay in the
  denominator. At least 95% flat free, at most 1% false occupied, zero designated
  positive/hole false-free over the entire sequence, not just the final map.
- 30 s immutable evidence lifetime; support spans at most 30 cm with existing
  height-agreement checks. No GT fill, inferred endpoint chaining, air-ray ground,
  arbitrary free spawn disk, post-hoc pose alignment, or rejuvenated observations.
- 20 x 20 m, 10 Hz, process RAM at most 1 GiB, p95 below 50 ms and p99 below 100 ms.
  Both synthetic resource profiles and observed-map/live timing must pass.
- GT terrain stays evaluator-only. GT pose use is explicitly labeled isolation
  or guarded collection, never represented as SLAM accuracy.
- Preserve every attempt, freeze source/config/library/evaluator provenance,
  record load and dropped inputs, and do not select best retries or grid phases.
  Existing offsets and any failure used for tuning are development, not holdout.
- Archive failure is a hard stop: 64 GiB/run and 50 GiB reserve, no root-disk
  fallback or automatic deletion. Preserve the dirty worktree and submodules.

## Ordered milestones and exit gates

### A — Restore and validate the experimental runtime [host CUDA restored September 13]

Start only task-owned dev containers; verify archive bindings, frozen library,
ROS setup and relevant contracts. Before capture, verify Isaac GPU access,
corrected clock cadence, TF ownership and no other command publisher. Do not
enable navigation or modify host drivers.

Exit: reproducible runtime checks pass and a uniquely owned guarded collector
can start without changing the experiment definition.

### B — Complete the declared acquisition comparison [completed; coverage failed]

Collect native-only `wide_sweep15` with the audited 10-degree mount: rotate from
yaw 0 to 15 degrees at the origin, travel `0 -> +1.2 -> -1.2 -> 0`, stop at the
travel heading (five views). Explicitly report 4.8 m versus the earlier 4 m dose.
Keep the 0.15 m/s speed, +/-1.28 m longitudinal / 0.08 m lateral envelope, 20 s
longest-leg deadline, per-step footprint/floor checks, stall and heartbeat guards.

Audit before mapping; preserve and investigate any failed first-hit ray using
the new ordered hit metadata. A matching later floor hit does not rescue it.
The historical failed 15-degree-mount recording remains ineligible.

Score the frozen core, measured/inferred variants, every native input and every
10 Hz output at offsets 0/0.025/0.05/0.075 m. Use the original reference and
coverage attribution. Keep failed attempts; no rerunning until a favorable score.

Exit: an audited capture and complete four-offset comparison, whether it passes
or fails. Capture completion alone does not establish terrain acceptance.

### B2 — Isolate the terminal-turn acquisition effect [completed; coverage failed]

`wide_sweep15_return` preserves the five wide-route targets and adds only a
bounded origin turn to yaw 0. Six views, 4.8 m translation, same mount/reference,
30 s evidence lifetime and all motion guards. Score all four offsets using
baseline `9161265a...`, not the angular candidate, to isolate the acquisition
effect. The first attempt failed before measurement; no conclusion about the
turn follows from it. Do not extend expiry or choose a passing intermediate map.

### C — Improve terrain evidence and runtime separately [angular-support experiment in progress]

Attribute missing coverage to observations, extent, clearance and boundary
uncertainty. Reproduce each targeted failure in an analytic fixture before a
classifier change. Keep mapper changes and acquisition changes in separate
comparisons. If wider views fail, evaluate an alternative support/clearance
representation explicitly, with thin-object, rotated-hole and partial-cell tests;
do not silently weaken the current guard to reach 95%.

Profile native integration, classification, connectivity, extent and clearance.
Require exact replay equivalence for semantics-preserving optimizations; rerun
both resource profiles. Replay older straight/rotation and all offset fixtures
after classifier changes. Report coverage failures as well as safety successes.

Exit: all declared development geometry gates pass and every native alignment
case has timing margin below 50 ms, with adequate process-memory headroom.

#### C1 — Matched mount geometry comparison [completed; one offset fails coverage]

Use the same six-view `wide_sweep15_return` route, frozen `69d182ca...` core,
original reference, seed, speed, evidence lifetime and all four grid phases.
Change only the declared LiDAR mount from 10 to 5 degrees; audit the actual
geometry/TF and every native acquisition before scoring. Report actual route
duration, chunk count and poses, since identical commands need not yield an
identical physical trajectory. Compare to the `baseline_*` variants of the four
matched-parent B2 runs in the ledger, not their rejected `8a38fdb9...` variants.
Attribute any gain/loss using the same fixed-axis coverage quadrants. A mount
change is useful only if safety and compute survive as well as coverage; neither
one passing offset nor this development fixture establishes generalization.

| XY grid phase | 10-degree inferred free | 5-degree measured / inferred free | 5-degree coverage |
| --- | --- | --- | --- |
| 0 cm | 94.855% | 95.354% / 95.397% | Pass |
| 2.5 cm | 94.335% | 94.454% / 94.526% | **Fail** |
| 5 cm | 94.588% | 95.023% / 95.046% | Pass |
| 7.5 cm | 94.924% | 95.108% / 95.182% | Pass |

All four runs contain 1,314 native chunks and 439 scored maps per variant. The
mount changes sensor poses; prescribed targets, settled base positions and view
timestamps match exactly. These are development observations, not four independent
physical trials or exact replay equivalence. All original gates remain unchanged.

#### C2 — Diagnose certificate/edge loss before another classifier [diagnostics verified; signed-only candidate rejected]

The failed 2.5 cm offset has 1.550% unsupported extent, 1.211% negative-edge,
0.967% clearance, 0.626% no returns, 0.291% positive-edge, 0.079% unresolved raised
and 0.034% insufficient area, plus 0.717% false occupied. The mount reduces
no-return loss but does not reduce extent loss; bounded inference contributes
only 100 fine cells (0.072 percentage points) at this offset.

1. Add opt-in, GT-free diagnostic data for each rejected corner: no adjacent
   ground anchor, too few retained witnesses, excessive angular gap, excessive
   span/region uncertainty, or disagreement with the anchor plane. Separately
   identify actual below-plane troughs versus vertical-normal-only edge triggers.
   Keep evidence retention and the six ordinary output layers unchanged.
2. Verify diagnostic collection preserves **every** output map on the frozen
   fixtures. Diagnostic builds/timings are not production performance evidence;
   default-path regressions and both resource profiles still gate runtime edits.
3. Use the data to build a minimal analytic case, including rotated/shifted
   hazards and identical flat controls. Inspect whether positive-obstacle side
   returns are causing a negative-edge exclusion on otherwise supported adjacent
   ground. This is a hypothesis from code and reason maps, **not a confirmed bug**;
   removing the halo without proving equivalent hole/partial-cell safety is forbidden.
4. Only then evaluate one support/edge representation change against the unchanged
   full sequences and both mounts. If conservative XY regions discard essential
   geometry, compare bounded exact-witness or explicit local support-boundary
   representations with a memory budget, rather than another angular-mask shortcut.

The 5-degree figure remains in its worst-offset score run. Fixed quadrant counts
show a net gain of 521 fine free cells in negative-X regions, offset by a loss of
256 in positive-X regions (net 265). This supports a directional tradeoff, not a
claim of uniformly better observations or solved mapping. Do not continue mount
search just to pass the remaining development grid.

#### C3 — Separate ground anchoring from boundary extent [deferred during shadow diagnosis]

The C2 experiment establishes two distinct requirements:

- **A face's sign does not certify neighboring cell clearance.** A rotated
  upward wall crosses a tiny cell-corner sliver between actual sampled returns.
  Corner ground-support witnesses still surround that cell, so the signed-only
  change calls it free. The old boundary guard prevents this. Reproduce this
  case at all three slopes and retain its continuous geometric probes. Any
  replacement must bound the observed face's extent through adjacent cells using
  actual evidence and explicit uncertainty; relabeling a normal or removing its
  halo is insufficient. Test gaps between separate obstacles as controls so a
  new boundary representation does not simply join every nearby object.
- **The lowest nearby layer is not always the connected driving surface.** On
  +0.15 slope, the existing fit can follow the low hole wall (roughly -0.045 m)
  instead of the ground near +0.05 m. Both parent and candidate exhibit this;
  the hole remains non-free, but estimator-relative edge flags become misleading.
  Add ground-plane error/anchor diagnostics against analytic truth, then compare
  connection to independently supported ground patches rather than blindly
  selecting the lowest local returns. Keep GT out of the mapper and preserve
  reset/expiry behavior. Do not use a fitted ground sign as absolute truth.

Retain `69d182ca...` until a replacement passes these stronger safety fixtures as
well as the frozen native sequences. Near-95% coverage is useful; the extra
0.5 percentage points from an unsafe boundary shortcut are not an improvement.

### D — Generalization and live frame/revision correctness [shadow foundations in progress]

Freeze a candidate before new physical alignments, angular phases, slopes, noise
and natural terrain. Validate moving raw-cloud timing/deskew before accepting
ROS full scans. Preserve sensor-local immutable observations, measurement times,
extrinsics and session/epoch/revision identity.

Implement a bounded live revision consumer with snapshot recovery and archive
health holds. Rebuild both old and new affected footprints and dependencies
without refreshing evidence age. Require explicit costmap-generation
acknowledgement; stale/mixed/partial installs cannot authorize motion. Preserve
one map-to-odom authority and the home anchor. Enable global/submap optimization
only after correction and installation barriers pass, not before.

Exit: held-out terrain gates, moving association tests, revision-gap/reset tests,
fresh-versus-revised reconstruction equality and coherent live installation pass.

### E — Localization and navigation validation [pending]

Test moving absolute-sensor outages, delayed/invalid updates, estimator faults,
pose corrections and restart while preserving home/mission state. Separate
intentional fault containment from spontaneous SLAM failure rates.

Share oriented robot geometry across support, reachability and Nav2; distinguish
wheel-track support from body clearance. Test blocked starts, narrow passages,
concave obstacles, reverse recovery, rejected/cancelled actions and blocked home.
Retain one global home goal and one recovery owner; no breadcrumb or unknown-space
shortcut to conceal a perception/planning/control failure.

Exit: repeatable short-course success, then seeds 42/411/7 for 20 simulation minutes
each with all first attempts counted, followed by separate untouched acceptance.

## Interfaces and regression coverage

Keep current terrain outputs: cost, reason, ground height, uncertainty, obstacle
height and evidence age. No API change is needed for the next capture/replay.
Live integration later requires generation-tagged terrain outputs and installation
acknowledgements in addition to existing revision/snapshot interfaces.

Required regressions include hole rims and rotated boundaries, thin objects,
partial faces and cross-layer normals, overhead clearance, exact/fractional expiry,
same-stamp reintegration, outside-origin clearance, worker determinism, audit
first-hit retention, revision gaps and stale action callbacks. Runner details
are maintained in the root AGENTS.md.

## Execution ledger

### 2026-09-11 — Resume after host GPU repair

- Host `nvidia-smi` succeeds: RTX 4070 Laptop GPU, 580.178.04. External device
  `/dev/sda1` is mounted at the archive root. Task dev/simulator containers were
  stopped after reboot; unrelated postgres is left alone.
- Verified the frozen library SHA and historical six acquisition/offset scores
  during the survey. No historical failed artifact was changed.
- Created this plan/progress file before execution. Next: start task-owned dev
  environments, rerun contracts, then attempt the declared wide capture.
- Started `astrobot-repair-dev` and `astrobot-revision-dev-v2`; archive bindings
  and frozen library SHA match, 420 GiB free. Reran **291 Python tests and 11 C++
  programs**, all passing after the reboot. No navigation stack is running.
- Requested only the simulator with native `wide_sweep15`, 10-degree mount,
  seed 42, cameras/GT TF disabled. Capture/audit outcome is pending; mapper
  and evaluator thresholds are unchanged.
- Isaac starts with GPU access and resolves 60 Hz physics / 30 Hz rendering.
  A diagnostic ROS query failed because the dev container inherited the old
  `br-f221da6afe07` DDS interface after reboot. The collector is simulator-local;
  this query failure is not a capture success/failure verdict. Discover the
  current simulator bridge and override only diagnostic process DDS settings.
- The current simulator network resolves to bridge `br-c15627b1b7fd`. A per-exec
  CycloneDDS override removes the invalid-interface error; the first query still
  finds no command topic while the world is loading. Rediscover this bridge on
  future host/network recreation rather than persisting this name in config.
- Native Linux `perf` is unavailable for the running kernel (missing matching
  linux-tools). Do not install/change host packages for this experiment; use
  bounded application instrumentation if stage profiling is needed. This does
  not block acquisition or ordinary measured latency tests.

### 2026-09-11 — Resume after interruption, wide capture completed

- The original attempt `revision-f3274b3cd28145d4922035b8de90d132` completed all
  five views during the interruption; no replacement capture was started.
  The driver returned near its origin (final world X -20.008 m). Native audit
  and all four replay scores are still pending; completion is not acceptance.
- The drive was reconnected again (`/dev/sdb1`); the dev container retained a
  stale archive bind and returned I/O errors. Host archive is mounted and the
  completed run exists. Stop the completed simulator and refresh the task-owned
  replay container before reading/scoring; do not fall back to the root disk.
- Simulator stopped and archive bind refreshed. The completed wide capture's
  **1,152 native chunks** pass the hash/cadence audit: **9.995 Hz**, matching
  world/timeline clock deltas, **147,456** sampled first-hit rays, **0 missing**,
  maximum current-pose discrepancy **0.0229 mm**, 7,294 nonhorizontal samples,
  no duplicate reads/outlier frames. Previous-pose counterfactual has centimetric
  errors and 196 misses; no transform fitting or ray substitution was used.
- Views span **37.8 simulated seconds**; the positive endpoint is 24.1 s old at
  the final view, inside the unchanged 30 s lifetime. Final position is about
  1.6 cm from the original XY origin, ending at the declared 15-degree heading.
- Started the four offsets sequentially with frozen core `9161265a...`, original
  reference, and both mapper variants. Isaac is stopped for replay timing.
- First two wide-route scores **fail coverage**: offset 0 is 92.215% and offset
  2.5 cm is 91.622% measured / 91.640% inferred, below the prior 4 m sweep.
  Both retain 12/12 objects, zero sequence positive/hole false-free and pass
  sampled compute. Runs: `revision-26b62205cd424b5f87f0c972302f3d75` and
  `revision-ef0c529e590e478db72856da0ab874ef`. The remaining offsets still count;
  the larger acquisition dose is not being promoted as an improvement.
- All four wide-route offsets finished, **385 maps per variant each**, all 12/12
  and zero sequence positive/hole false-free. Inferred coverage at 0/2.5/5/7.5 cm
  is **92.215/91.640/91.975/92.554%**. Remaining runs:
  `revision-760833de8be84022960468a9cb328856` and
  `revision-9683068e12454a7aaea6ddbd23779d61`. All sampled compute gates pass.
- Worst-offset attribution: measured variant has 2.30% no returns and 2.62%
  unsupported extent; inferred variant has 1.88% no returns and 3.18% unsupported
  extent. Inference mostly moves unknown cells between reasons, not into free.
  Wider motion is therefore not the selected remedy; no further route tuning yet.
- Implemented a **candidate**, not promoted, rotation-invariant angular support
  check after the existing fixed-sector certificate. Six evenly spaced returns
  can have safe 60-degree gaps while occupying only six of eight fixed sectors.
  The candidate requires every circular gap strictly below 90 degrees and all
  pairwise spans within 30 cm; observed-region angular uncertainty is included.
  Height agreement, immutable expiry, hazard checks and evaluator remain unchanged.
  This tests an axis-dependent conservatism, not proof against every small unseen
  void. Analytic wedge/radius/rotation and all existing hole fixtures must pass
  before freezing and replaying the candidate. Capture baseline remains intact.
- Candidate passed all **291 Python tests**, including all 108 terrain cases,
  and **7 mapper CTest programs** including the new angular fixtures. Frozen
  candidate: `revision-69845623856f4726af6399a43c682a2f/liblayered_terrain.so`, SHA
  `cc172f65523b21596409143aaace5fdea067d4fd13ebec4869402b283b067625`.
  All four native wide-route replays are running; no passing accuracy or compute
  claim is made before those results. The previous baseline binary is unchanged.
- Angular candidate completed all four wide-route offsets: inferred coverage
  **92.251/91.730/92.030/92.572%**, only **0.018–0.090 percentage points** above
  baseline. All sequence positive/hole safety and sampled-compute gates pass.
  Runs (0/2.5/5/7.5 cm): `revision-13dcd937d2aa4480b14c89297f9103f8`,
  `revision-fbb28478c9ae41aa9f28ba752e2007bf`,
  `revision-5fc58c6ba38c4a49a6c37367ff7f1a19`,
  `revision-4fe9a4d43c9a45469844d168cd4388c3`.
  ASan/UBSan also passed. This small gain is not a breakthrough or promotion.
- **Acquisition attribution correction:** comparing the existing causal per-view
  records at 2.5 cm, the old 4 m route returns at heading 15 degrees with **90.235%**
  coverage, then its final in-place turn to heading 0 yields **93.614%**. The
  wide route's 15-degree return has **91.640%**. Thus translation and final
  heading/turn were confounded in the initial route comparison. Larger motion
  alone cannot explain the final-score difference; these are development data.
- Next declared acquisition experiment: `wide_sweep15_return`, identical to
  `wide_sweep15` through all five targets, plus the existing bounded origin turn
  back to yaw 0 (six views, still 4.8 m translation). Keep the baseline `9161265a...`
  core for all four scores to separate acquisition from the angular candidate.
  The expected endpoint age is near 30 s, so actual expiry remains active; no
  extension or stopping on a passing intermediate score is allowed. This is a
  new specified terminal-turn experiment, not a retry of the failed wide route.
- Original 4 m sweep regressions for the angular candidate are now running at all
  four offsets before any candidate promotion. No navigation is enabled.
- Original 4 m angular-candidate regressions completed: inferred coverage at
  0/2.5/5/7.5 cm is **94.586/93.650/94.039/94.563%**. All 12 objects, full-sequence
  positive/hole safety and sampled compute pass, but every coverage gate fails.
  Runs: `revision-49796ffb7c74423285eee3f60b71d94f`,
  `revision-bd2937b448874f39b89bb41ec7249083`,
  `revision-a7b422cb7d0a4e5aa2335071046f3da2`,
  `revision-5cd751c4e709464f8a544e30ef4a7132`.
- Implemented `wide_sweep15_return` in both the guarded driver and independent
  scorer, with prefix/translation-budget/native-only/off-origin-turn contracts.
  The targeted driver/scorer suite passed 40 cases. Before capture, rerun the
  full suite and confirm GPU/archive health. No classifier promotion is implied.
- Full verification after the new route: **294 Python tests and all 11 C++
  programs pass**. GPU and mounted `/dev/sdb1` archive remain healthy. Candidate
  dense-ground stress passes at **44.83/52.61 ms p95/p99, 311.9 MiB**, run
  `revision-a70f84d9d746404d892a88eef0946b26`. Its first seconds overlapped the
  short contract suite, so it is not an idle-only benchmark. No favorable retry
  replaces it; layer-capacity stress follows sequentially before Isaac starts.
- Candidate layer-capacity stress also passes: **46.29/54.54 ms p95/p99,
  994.1 MiB**, `revision-b9fcded1d27a49cbbe9f7aa006f6f2c2`. Memory margin remains
  only about 30 MiB under the process limit; this is not a live ROS memory bound.
- Started the separately declared `wide_sweep15_return` capture after all timing
  work completed: seed 42, 10-degree mount, native capture, cameras and GT TF off.
  The baseline binary hash was rechecked. The driver remains the sole movement
  owner; completion, native audit and all four baseline scores are pending.
- **Terminal-turn capture failed before measurement**, run
  `revision-d248daea5c394cabbc64f5bf0a4155b0`: invalid TF pose, zero collision rays.
  Isaac startup reports CUDA error 999 and cannot create any GPU device. Stopped
  the owned simulator and preserved a startup-log excerpt alongside the failed
  run (the repeated GPU errors exceeded the tool-output limit; not a full log).
  Do not score this input or treat the turn hypothesis as tested.
- Read-only diagnosis: host and dev-container `cuInit(0)` both return **999**,
  while host/simulator `nvidia-smi` still report driver 580.178.04. Kernel log at
  **18:17:15 JST** records Xid 31 then Xid 154 with **Node Reboot Required**;
  suspend/resume also occurred. This establishes a host CUDA failure, not its
  underlying cause. Driver reload/reset/reboot was not attempted.
- Fixed the insufficient launch preflight: new `docker/check_cuda_runtime.py`
  checks driver initialization, visible devices, creation of an owned context,
  a four-byte allocation, synchronization and cleanup, in a 15-second bounded
  subprocess. The normal Isaac launcher requires it before world construction.
  It never repairs drivers or resets hardware; success is not sensor acceptance.
  **14 new mocked contracts pass**, shell syntax passes, and the actual host
  probe fails closed with error 999. Further physical captures require host
  recovery; continue CPU-only validation without replaying this failed attempt.
- Post-preflight full suite: **308 Python tests pass**. Root and OmniLRS diff
  whitespace checks pass. CPU-only angular-candidate regressions continue on
  older fixed 5/10-degree straight recordings; no new physical data is implied.
- Older 5-degree straight replay completed: **79.495%** inferred free (79.477%
  measured), 12/12 objects, zero sequence positive/hole false-free and sampled
  compute pass, `revision-ee1ec10a52db4efa984cc0d49acbdf5e`. The small increase
  from 79.35% does not resolve acquisition sensitivity or pass coverage.
- Older 10-degree straight replay also fails coverage: **84.434%** inferred
  (84.416% measured), 12/12 objects, zero sequence positive/hole false-free and
  sampled compute pass, `revision-d4cff6b1a4bd41f58de222592cdc5cb4`. Both straight
  runs integrate all **2,059 chunks / 687 maps per variant**. Started the older
  spin fixture's settled-scan regression separately; its old native timing is
  not eligible for native moving replay, and it cannot establish live safety.
- Spin settled-scan candidate regression completed, run
  `revision-34651fa669354b389d7bd01cd043f517`: range-valid measured/inferred free
  **30.373/33.609%**, all 12 objects, zero positive/hole false-free over the
  scored settled scans. Inferred **50.40 ms p95 fails** the unchanged compute
  gate; measured 43.22 ms passes. This is not a native moving-sequence test.
  The old 57.36% spin result used an earlier support rule and is not a matched
  angular-candidate baseline. Replay the frozen whole-cell baseline to isolate
  this candidate's effect rather than attributing the entire difference to it.

### 2026-09-12 — Resume with CPU-only validation

- At 05:21 JST the bounded host CUDA probe still fails at `cuInit` with error
  999. Simulator remains stopped, external `/dev/sdb1` is mounted, and CPU dev
  containers are available. No new capture or host repair was attempted.
- Started the matched spin replay using frozen baseline `9161265a...` with the
  same old settled scans, fixed reference and current evaluator. Preserve its
  result even if it fails; this is a baseline comparison, not a timing retry.
- Implemented an exact early rejection for angular support: two neighboring
  empty 45-degree sectors necessarily leave a >=90-degree gap, and a failed
  all-eight-sector pairwise-span check cannot pass the identical check again.
  Skip those redundant calculations only; keep actual witnesses, order, height,
  expiry and thresholds unchanged. Added exhaustive 256-mask / 256 within-sector
  boundary-placement contracts. Build, full suite and frozen exact replays are
  required before claiming this optimization preserves behavior or helps timing.
- Matched whole-cell baseline spin score is **28.892/31.480%** measured/inferred
  free, `revision-c4fb772df28a478dba59192b183b70a2`; all safety gates pass and
  inferred p95 is 46.20 ms. Thus the angular candidate adds **2.129 percentage
  points** on this sparse settled fixture, but remains far below 95% and had a
  50.40 ms p95 failure. Neither result validates rotation as a recovery policy.
- Early-rejection optimization passed **308 Python tests and 7 mapper CTest
  programs**, including the exhaustive mask contracts. Frozen binary and exact
  source snapshots: `revision-bf1fde8ac66a45979c61a8404232e8f0`, SHA-256
  `69d182cadf025fa8f8c11934566fad0322f4d4d7c8f72f1f79faa14b6b34f9ff`.
  Sequential spin and four-offset original native replays are running against
  the frozen pre-optimization angular candidate; geometry must match exactly.
- Exact comparison passes for the spin fixture (**50 frames, five variants**)
  and original-sweep offset 0 (**387 frames, two variants**), including all
  per-frame records and final six-channel arrays. Optimized runs:
  `revision-91d70782d9094f73920266f53b8f5279` and
  `revision-48ed833fa8444cd2a860ed3eb0672841`.
  Spin range-valid inferred p95 is **43.48 ms** versus the prior failed
  50.40 ms run. This is observed runtime improvement with identical maps,
  not a new coverage gain or proof of live timing; remaining offsets and both
  resource profiles still count before accepting the optimization.
- Original-sweep **2.5 cm and 5 cm** optimized replays also match exactly, all
  387 records and final arrays in each variant. Runs:
  `revision-c222f5a25abc4701b180fe402026c774` and
  `revision-80138aab5bdf4be0b1e0b98f0e62c475`. Safety and sampled compute pass,
  coverage is unchanged and still fails. Final 7.5 cm replay is running.
- Final **7.5 cm** comparison passes exactly as well:
  `revision-bd9f2fe403da4075bde7c6577381ca62`. Thus all four original native
  offsets preserve 387 records per variant and every final layer; all sampled
  timing gates pass, worst p95 **49.06 ms**, worst p99 **53.20 ms**. Earlier
  failures remain archived. This does not establish universal live timing margin.
- Both optimized resource profiles pass on their first declared runs:
  dense ground **41.87/46.61 ms p95/p99, 312.3 MiB**,
  `revision-ab6c68d70bb64269a7c3304c05271c8d`; layer capacity
  **44.25/49.52 ms, 994.7 MiB**, `revision-dcb0be431ded482c9ef1cf06d4c81eb6`.
  They ran after the native batch, before sanitizer compilation, with Isaac
  stopped. The small memory margin remains a live-integration concern.
- Optimized standalone **ASan/UBSan contracts pass**. The subsequent GLIM CTest
  invocation mistakenly omitted the ROS/repair overlay setup, yielding a missing
  generated-interface library, an old GLIM symbol mismatch and a stale-endpoint
  test failure. Preserved `unsourced_glim_test_failure.log` in the frozen run.
  Correct the test environment using `/etc/ros_setup.sh` and the repair overlay
  before rerunning; do not change GLIM code or call these failures resolved yet.
- With the required ROS/repair overlay sourced, **all four GLIM tests and all
  seven mapper tests pass**. No GLIM source change was needed. Final verification
  for this increment: **308 Python tests, 11 CTest programs, ASan/UBSan**, exact
  spin + four-offset native comparisons, and both resource profiles passed.
  The unsourced failure remains archived as an invocation/environment failure.
- The exact early rejection is retained in the experimental source. The terrain
  model remains unpromoted because coverage/generalization and live gates fail
  or remain untested. No navigation stack, simulator, or motion command remains
  running from this work.

### Next execution steps

1. Host owner resolves the CUDA fault; require the bounded probe to pass on the
   host and in the Isaac environment. Do not equate successful NVML with CUDA.
2. Complete B2 using the unchanged acquisition baseline, preserving the first
   failed attempt. Audit all native geometry and score every declared offset.
3. Remaining CPU-only work: exact optimized-core comparisons on wider/older
   straight recordings, detailed stage profiling, and live-memory budgeting.
   These can proceed without a new physical capture; they do not establish 95%
   coverage or authorize integration into navigation.
4. Use the fixed-reference acquisition/unknown-reason evidence to choose the
   next terrain representation change. Keep each classifier change separate
   from route changes; rerun all geometry, lifetime and resource gates. Milestones
   D/E remain gated on robust terrain and coherent frame/revision installation.

### 2026-09-13 — Resume after user CUDA recovery

- At 16:57 JST the host CUDA probe passes device/context/allocation checks;
  `nvidia-smi` reports driver 580.178.04. Archive is mounted on `/dev/sda1` after
  reboot. The earlier sandboxed probe returned code 100 and could not access
  Docker; unrestricted verification passes. Do not misdiagnose sandbox device
  isolation as another host CUDA failure.
- Starting only the two task-owned dev containers, verifying the baseline hash
  and archive binding, and rerunning contracts before the declared B2 attempt.
  The previous failed capture remains untouched; unrelated postgres is untouched.
- Archive binding and frozen baseline SHA match, 420 GiB free; all **308 Python
  tests pass** after restart. Started the guarded `wide_sweep15_return` capture:
  seed 42, 10-degree mount, native capture, cameras and GT TF off, no navigation
  stack. Its launcher must pass the CUDA context/allocation preflight before
  Isaac starts. Capture completion, native audit and all four scores are pending.
- Isaac's own CUDA preflight passes and the simulator starts with 60 Hz physics /
  30 Hz rendering. New run: `revision-923d8f696d2f444589065ff06d0e13b3`.
  The pre-world ROS query found no command topic; the collector independently
  requires sole command ownership before motion. GT TF is confirmed disabled.
- Before motion, ROS reports **zero command publishers and one robot command
  subscriber**. Stationary collision capture completes and the six-view driver
  begins. This intermediate completion is not the route/audit acceptance gate;
  do not stop early or choose an intermediate favorable map.
- **All six prescribed views complete**, final world X about -20.004 m. Stopping
  the owned simulator before audit/replay timing. This completes collection only;
  no eligibility or terrain-accuracy claim is made until the audit and scoring.
- Native hash/cadence/pose audit passes: **1,314 chunks, 9.995 Hz**, matching
  world/timeline deltas; **168,192 sampled first-hit rays**, 8,248 nonhorizontal,
  zero missing/duplicate reads, max current-pose discrepancy **0.0229 mm**.
  The previous-pose counterfactual has 240 misses and centimetric p95 errors.
  All six views complete; final XY is about 1.77 cm from the original origin.
- Final timestamp **63.600 s**, positive endpoint **34.100 s**: endpoint age
  **29.500 s**. Immutable bucket expiry remains active with no extension.
  Started all four baseline offsets sequentially, with Isaac stopped, the
  original reference and unchanged measured/inferred variants and gates.
- The original wide capture's **entire 1,152-chunk prefix matches exactly** in
  both native-file hashes and recorded poses. The new 162 chunks cover the
  added final turn/settling; this is a clean acquisition continuation, not a
  claim that two independently varying trajectories isolated the turn effect.
- All four baseline scores complete, **439 maps per variant**: inferred free at
  0/2.5/5/7.5 cm is **94.801/94.227/94.552/94.870%**. All 12 objects, zero
  sequence positive/hole false-free and sampled compute pass; every coverage
  gate fails. Runs: `revision-2b33a80ba65445e792105297aa66f168`,
  `revision-27afa2717e2b4b42bdc0510d13ae2830`,
  `revision-133803d0d4d4465fb3d9073ef0e3f839`,
  `revision-f8f3e1be349544b3995880f7796f8f0c`.
- Worst-offset attribution retains all 138,725 flat cells: inferred unknown
  includes **1.602% unsupported extent, 0.942% no returns, 1.000% negative-edge,
  0.975% clearance, 0.405% positive-edge**. The final turn helps, but is not a
  complete remedy; do not tune another route just to cross 95%.
- New narrowly scoped candidate: retain otherwise valid witnesses on/crossing
  fixed angular-sector boundaries in a supplemental pool. The old certificate
  is unchanged and tried first; the fallback requires the same actual-bearing
  gaps, conservative region radii, pairwise 30 cm span and height checks.
  No mask alone certifies support. A six-spoke reproducer shows the old prefilter
  discarding the two X-axis witnesses; added missing-wedge, uncertainty-region
  and excessive-span tests. Full geometry/safety tests precede frozen replay.
- Boundary-witness candidate passes **308 Python tests and all 7 mapper CTest
  programs**, including hole-rim/rotation/expiry/worker contracts and the new
  prefilter reproducer. Frozen with exact source snapshots:
  `revision-fd55da08f58f492481b03e39947784ec/liblayered_terrain.so`, SHA-256
  `8a38fdb9eb9a8b9947df2a8ad4e89c3651d823085bb060f01fbb4cb632f2a22a`.
  Started all four B2 offsets with both this candidate and its actual parent
  `69d182ca...` in each run. This separates the boundary fallback from the earlier
  angular change; all full-sequence safety and compute results still count.
- Boundary candidate completes all four B2 offsets: final inferred free is
  **94.855/94.335/94.588/94.924%**, **identical to its matched parent** at every
  offset. Some maps/reasons outside that aggregate differ, so this is not exact
  replay equivalence. All 12 objects and sequence safety pass; candidate sampled
  p95 peaks at **49.82 ms**, versus 41.70–45.70 ms across parent variants.
  Runs: `revision-04502801ba0a4f8a8beb3a4f83dd5fea`,
  `revision-a90a4b1b7c194b598f6f3ce02a59b9f2`,
  `revision-c2409a2990ab40c1af4ea11dd75e064e`,
  `revision-bd8d49e453e84c528a53aa82366a0637`.
  This does not justify promotion; finish original-route and resource regressions.
- The original-sweep offset-0 boundary candidate also adds no final reference
  coverage and **fails measured p95** (`revision-e1775f6e1de349c885ede321df9c75bd`).
  Decision: reject its runtime hook for now; retain the archived source/binary
  and a test-only reproducer. Restore the proven `69d182ca...` production path
  after the fixed remaining regressions finish. No favorable retry replaces it.
- All four original-sweep boundary-candidate offsets retain sequence safety and
  unchanged final reference coverage. Remaining runs:
  `revision-6812f8e01265433ba25e28d3c0afadaf` (2.5 cm),
  `revision-a4af5c7cf8424f6d9766b86013185974` (5 cm),
  `revision-dbd5d4d11b85420e9969512676d53dc9` (7.5 cm).
  Native latency failures confirm that this candidate is not an improvement.
- Candidate stress passes but does not rescue its native failures: dense ground
  **47.36/55.53 ms p95/p99, 312.2 MiB**, `revision-1ed25728de3d48b0bd672614c88b2729`;
  layer capacity **46.52/50.51 ms, 994.5 MiB**, `revision-3969235e92ba4748bd97e5270deda3d7`.
- Removed only the rejected runtime hook and moved its witness-pool prototype to
  `test/angular_boundary_fixture.hpp`. The reproducible boundary cases remain
  tested; they do not enable the rejected behavior. Rebuilding and verifying the
  retained mapper path now, after all benchmark runs completed.
- Restored runtime rebuild is **byte-for-byte the retained `69d182ca...` binary**;
  **308 Python tests and all 11 CTest programs pass**. The boundary prototype
  remains test-only. A fixed-reference figure is saved with baseline B2's worst
  offset (`revision-27afa2717e2b4b42bdc0510d13ae2830/terrain_comparison.png`).
- The figure shows direction-dependent unknown regions; measured and inferred
  outputs barely differ. Actual sensor metadata is **40.3-degree vertical FOV,
  1.3-degree vertical resolution, 0.5 m minimum range**, regardless of the
  `vlp16` frame/asset name. Do not reason from that name as a 16-ring specification.
- Next predeclared-family comparison: **5-degree mount on the exact B2 six-view
  route**, one post-preflight capture, same 4.8 m motion, baseline `69d182ca...`,
  original reference and all four offsets. A shallower tilt may reduce the rear
  blind region while retaining angular diversity; this is a hypothesis, not an
  accuracy claim. Do not change lifetime, denominator, speed or classifier to
  obtain a pass. Compare against the matched 10-degree parent results, not the
  older straight-route 5-degree capture or a different mapper.
- Resumed at 22:27 JST: host NVML and CUDA context/allocation pass, external
  `/dev/sda1` archive binding is healthy with 419 GiB available, and frozen core
  SHA is exactly `69d182ca...`. Started the declared 5-degree B2 capture with
  cameras/GT TF disabled, seed 42 and no navigation stack. Capture, native audit
  and all four scores are pending; no result from the previous mount is reused.
- New physical run: `revision-80479640be564a4d9ac489dfed0e86cb`; Isaac CUDA
  preflight and normal world startup pass, GT TF is disabled. Stationary geometry
  collection is in progress; this is not yet a completed route.
- Implemented evaluator-only fixed-platform-quadrant coverage attribution in
  `tools/diagnose_terrain_coverage.py`. All quadrants retain the original global
  flat denominator, including unknown/outside-map cells; map heading and candidate
  visibility cannot redefine regions. Five new regression cases cover all four
  grid phases, axis centers and empty quadrants; **10 diagnostic tests pass**.
  This adds diagnosis only, not a classifier, scorer or acceptance change.
- The 5-degree capture completes all six views without a drive safety stop.
  Stopping the owned simulator before native audit and replay timing; capture
  completion alone does not certify terrain accuracy or timing eligibility.
- Native hash/cadence/pose audit passes: **1,314 chunks, 9.995 Hz**, matching
  world/timeline deltas, **168,192 sampled first-hit rays**, 7,428 nonhorizontal,
  zero missing/duplicates, maximum current-pose discrepancy **0.0502 mm**.
  All six view timestamps match the 10-degree route; final stamp is 63.600 s,
  so the positive endpoint is still 29.500 s old. The previous-pose counterfactual
  fails (232 misses, p95 2.34 cm). Isaac is stopped before compute measurements.
- **313 Python tests and all 11 CTest programs pass** after the diagnostic
  addition. Started the four 5-degree native replays sequentially, without other
  task CPU workloads, using frozen `69d182ca...` and the original reference.
- The 5-degree offset-0 replay passes development geometry and sampled compute:
  **95.354% measured / 95.397% inferred free**, 12/12 objects, zero sequence
  positive/hole false-free (`revision-fe9148f5b4f14c34996b103eaee588a4`).
  The other three offsets remain pending; this partial result is not acceptance.
  All six recorded settled base positions, timestamps and targets exactly match
  the 10-degree B2 recording (sensor poses intentionally differ with the mount).
- The 2.5 cm offset fails coverage at **94.454% measured / 94.526% inferred**
  (`revision-2de805151f14489991d5ff646864f2a0`), while the 5 cm offset passes
  at **95.023% / 95.046%** (`revision-a80a8527c5b64adda1ed74a86a7c2df7`).
  Both retain all 12 objects, zero sequence positive/hole false-free and sampled
  compute passes. The final offset is pending; the observed failure already
  rules out claiming robust coverage from this mount change alone.
- Final 7.5 cm offset reaches **95.108% measured / 95.182% inferred**
  (`revision-406238e2e4604a91933e7f3aeb544b48`). All four runs preserve sequence
  safety and pass sampled compute; worst p95/p99 **46.20/52.76 ms**, peak process
  **234.742 MiB**. This completes C1, but the 2.5 cm coverage failure counts.
- Created exclusive fixed-reference diagnostics for the matched worst-offset
  10-degree parent (`revision-a90a4b1b7c194b598f6f3ce02a59b9f2`, `baseline_inferred`)
  and 5-degree run (`revision-2de805151f14489991d5ff646864f2a0`). The latter also
  contains `terrain_comparison.png`, visually inspected against the original
  reference. Negative-X free cells gain 521 while positive-X loses 256; no-return
  loss falls from 0.942% to 0.626%, but extent loss rises from 1.494% to 1.550%
  and negative-edge loss from 1.000% to 1.211%. Do not infer a uniformly improved
  mapper from the aggregate coverage gain.
- C2 is now the next implementation step: opt-in rejection telemetry and minimal
  geometric reproductions before classifier changes. The retained core remains
  `69d182ca...`; no live Nav2 install or additional pitch/route experiment started.
  All captures, failed gates, frozen binaries and historical diagnostics remain
  preserved. Simulator stopped; dev containers remain available.
- Continuing at the user's request: implemented a separate, non-installed
  `layered_terrain_diagnostics` target (explicit build only). It records GT-free
  corner rejection flags/counters and distinguishes below-plane edge evidence
  from vertical-normal-only triggers, including coexistence with raised obstacles.
  Preprocessor isolation removes telemetry from the default library. The native
  replayer requires explicit `--diagnostics` for these libraries, retains normal
  per-frame maps separately and excludes instrumented compute acceptance.
  Analytic equivalence/edge/expiry and rejection-flag fixtures are being verified
  before frozen replay; no classification rule or threshold has changed.
- Diagnostic contracts pass: **323 Python tests and all 11 CTest programs**.
  Default rebuild SHA remains exactly `69d182ca...`. One initial CMake invocation
  regenerated targets but its already-running Make process could not see the new
  target; the subsequent explicit build succeeds. Fixed an enum/integer warning
  before freezing. Diagnostic binary/source snapshot:
  `revision-8b46e0e7666040cd80f9f32b93013686`, SHA `fba947c5...`.
  Starting exact native verification on the 5-degree capture, worst offset first.
- Worst-offset diagnostic replay (`revision-f6acbae4d4e740d5a1e8b9ed7aa67f90`)
  matches all **439 per-frame records and final six-layer arrays** exactly.
  GT-isolated attribution: 1,378 of 1,680 negative-edge flat cells have only
  vertical-normal triggers, with 1,044 also receiving a trigger from a known
  raised obstacle. Only 302 have actual below-plane triggers. Extent rejection
  is dominated by angular gaps and span, not missing anchors or plane mismatch.
  These counts select the next minimal fixture: mirrored positive and negative
  vertical faces on the same flat ground. A normal alone has no up/down sign;
  test signed below-plane evidence before treating a face as a dropoff.
- Analytic reproduction confirmed: a positive 10 cm wall with occluded ground
  in its own coarse cells creates **six wall-normal-only dropoff sources and 16
  negative-edge cells** in the unchanged mapper. Dense floor artificially mixed
  through that wall hid the issue; the fixture now preserves this occlusion.
  A mirrored negative wall retains actual below-plane triggers.
- Implemented a narrowly scoped signed-wall candidate: a vertical normal only
  triggers the dropoff guard if its trough is below the fitted plane beyond
  existing two-sigma uncertainty plus a 1 micrometre numerical tolerance.
  The independent deep-below-plane guard, raised/partial-face classification,
  whole-cell support, obstacle threshold and all evidence retention are unchanged.
  Positive/upward, shallow negative, uncertainty and numerical-boundary fixtures
  are added. Candidate is in analytic verification, not yet promoted.
- All four diagnostic offsets match their frozen parent **exactly**, 439 frames
  per variant plus final arrays: 0=`revision-f052f386e3634e8ba1ec44140448ceff`,
  2.5=`revision-f6acbae4d4e740d5a1e8b9ed7aa67f90`,
  5=`revision-4c6b80ec06ad4aaa935c8ad184343b9e`,
  7.5=`revision-351a5c9463ed4aa0bb37ce7d5a69f7cc`.
- Signed-wall candidate passes **324 Python tests and seven mapper CTest
  programs**. Frozen uninstrumented binary `revision-6a5de93ae8964d9eb6bfe1cf4a8d59b6`,
  SHA `ff89a7e0...`, with source snapshots. Starting all four 5-degree B2 offsets,
  then matched 10-degree and original-route/resource regressions if sequence
  safety survives. Diagnostic timing is not used as compute acceptance.
- First signed-wall replay (2.5 cm, `revision-4421680de16341c984bf06bf426add44`)
  reaches **94.948% measured / 95.020% inferred**, all 12 objects and zero
  sequence positive/hole false-free. Inferred p95 fails; this attempt counts.
  Offset 0 (`revision-ce58ee1dd08148959f5efcded20f917b`) reaches
  **95.762% / 95.805%**, safety and compute pass. Remaining offsets are running.
  A semantics-preserving follow-up is available: test the signed-height predicate
  before the normal eigensolve, since nonnegative troughs cannot trigger this
  branch. Verify exact maps against the current frozen candidate before using
  that optimization's timings; no favorable retry replaces the failed run.
- All four signed-wall offsets retain 12/12 objects and zero sequence positive/
  hole false-free. Inferred coverage is **95.020–95.805%**; measured coverage
  still fails at 2.5 cm (94.948%). Remaining runs:
  5=`revision-2b2e6ec94fc6461a9286118510fb636c`,
  7.5=`revision-363405f8436e4d2fa37c5e720fa28e43`.
  The first inferred p95 failure is **51.702 ms**, not a robust runtime pass.
- Moved the identical signed-height predicate ahead of the normal eigensolve.
  This changes computation order only for branches already unable to trigger;
  exact comparison against `ff89a7e0...` is required. Expanded mirrored-face
  diagnostic fixtures to all four grid phases and three rotations before replay.
- Expanded signed-face/diagnostic suite passes: **357 Python tests, 11 CTest
  programs**. Optimized candidate frozen with exact source at
  `revision-c1f43b957ca64674bd40db79b0c5a847`, SHA `7a13403f...`.
  Starting all four 5-degree exact comparisons and both resource profiles,
  sequentially with the simulator stopped and no parallel task CPU workload.

### 2026-09-14 — Resume signed-wall verification

- Interrupted batch finished normally. All four optimized 5-degree replays match
  every one of the 439 per-frame records per variant and all final layers:
  2.5=`revision-b90d818ce0d6493c97c5986c9f0207ef`,
  0=`revision-5811158dcf8945debf093371cfdabd12`,
  5=`revision-4506605b978040019e38b2b957a5c93d`,
  7.5=`revision-aeb0f8b2753b48c9a58ad7959f8f82df`.
  All sampled compute gates pass, worst p95/p99 **44.473/50.539 ms**. The previous
  51.702 ms failure remains archived; this is a changed, verified computation
  path rather than a selected timing retry.
- Resource profiles pass: dense ground `revision-ce16be8805d244b1ba49b20238393bb8`,
  **36.760/41.676 ms p95/p99, 312.50 MiB**; capacity
  `revision-5c09dcc2172749ecb804ca95522b9119`, **43.403/48.819 ms, 994.70 MiB**.
  These are synthetic resource gates, not terrain/live-navigation acceptance.
- At 07:50 JST, archive `/dev/sda1` and both task dev containers are healthy;
  simulator remains stopped. Rechecking tests and frozen SHA before the declared
  10-degree B2 and original-route regressions. No new physical capture selected.
- Recheck: **357 tests pass**, frozen source build still `7a13403f...`.
  The first 10-degree B2 results retain all objects and sequence safety and pass
  compute: offset 0 is **95.291%** (`revision-97356597a99145fbb65088f0f6f2ed26`),
  2.5 cm is **94.783%**, still a coverage failure
  (`revision-b86c2f474d2549e88440400fb59982b4`). No threshold or acquisition
  change follows from that failure. Remaining fixed regressions are running.
- All four 10-degree B2 offsets complete: inferred coverage at 0/2.5/5/7.5 cm
  is **95.291/94.783/95.413/95.459%**, all required objects and sequence safety
  retained. Remaining runs: 5=`revision-5724e67294f944d79bfe967fa5cc3597`,
  7.5=`revision-cc436f17f0bf43a2887a62a866e0c375`. Original-route regressions
  are running. Extended analytic mirrored-face cases to slopes -0.15/0/+0.15
  in addition to their existing grid/rotation stress; testing follows the timing
  batch, without changing the frozen candidate.
- Original 4 m sweep completes all offsets: **94.778/94.188/94.759/94.955%**
  inferred coverage, all 12 objects, zero sequence positive/hole false-free;
  every sampled compute gate passes (worst p95 **48.829 ms**).
  Runs: `revision-f99902e2bb194a0da5d0b2457d314d24`,
  `revision-08230b4abd5a4672b200c9cc33ac1714`,
  `revision-e87c50cb0cc54d2eb1c0392b7b6f4c1a`,
  `revision-0b8497aaf2f84f49967e831d4cd87824`.
  All coverage failures remain counted; the boundary fix is not a new
  observation source and does not extend the 30 s evidence lifetime.
- The older stop-heavy 5-degree recording reaches **79.943%** inferred coverage
  (`revision-17ded23334684abaad493d3f7472c694`). It remains a coverage failure,
  as before. The remaining 10-degree straight/settled-spin regressions are running.
- Remaining recorded regressions complete: 10-degree straight **84.864%**
  (`revision-34f96ff1b8314f9d8855424842cf6a7b`); old settled spin **34.114%**
  valid-range inferred (`revision-5bc0d3bf0a6e4a768cf20bf41af24b95`). All required
  objects and recorded sequence safety pass, as do sampled compute gates.
  Old spin remains settled-only, never a moving-native timing validation.
- **Expanded suite exposed a blocker:** 11 failures / 418 passes. Three are
  genuine new wall false-free regressions at rotation 0.17 rad, phase 7.5 cm,
  slopes -0.15/0/+0.15. A continuous face crosses cell (20,27) between sampled
  returns; the signed candidate calls it measured free. Frozen-parent replay
  of the identical synthetic inputs has zero false-free probes at every slope.
- The other eight failures were over-specific test assertions that every slope
  must produce a below-estimated-plane diagnostic bit. The parent also fits the
  low wall as ground in those cases and labels the rim occupied instead. Corrected
  that diagnostic expectation to its original level-plane fixture, retaining
  **every sloped geometric hole/wall non-free assertion**. No safety probe or
  coverage denominator was removed. Ground-plane selection remains an open issue.
- Rejected the signed-only runtime change and its optimization; moved its
  prototype to `test/terrain_boundary_fixture.hpp`. The original conservative
  wall guard is restored, with opt-in diagnostics still isolated from the
  ordinary library. Failure analysis is archived in
  `revision-8f92c7fe09b54c0bbce69c4b60aa352f`. Rebuild, expanded tests and
  sanitizer verification are in progress; no model promotion or Nav2 install.
- Rollback verified: the ordinary binary is **byte-for-byte `69d182ca...`**;
  **429 Python tests and all 11 CTest programs pass**, including all 108
  slope/rotation/phase face cases and their independent wall/hole safety probes.
  Direct C++ ASan/UBSan with leak detection also passes. The earlier sanitizer
  command was gated behind the failed expanded tests and therefore did not run;
  this completed sanitizer run is on the restored mapper, not the rejected one.
- Archived the final geometric reproducer, rejected test-only sign prototype
  and diagnostic tool alongside the failure analysis. No failed capture, score,
  timing attempt or candidate binary was overwritten. Retained implementation
  changes are opt-in diagnostics, fail-closed attribution/replay checks and
  stronger regression fixtures. C3 (boundary extent plus ground anchoring) is
  next; no replacement classifier or extra physical experiment started.
