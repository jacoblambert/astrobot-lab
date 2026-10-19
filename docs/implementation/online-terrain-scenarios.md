# Online terrain checks — October 4, 2026

**Latest:** [fresh live integration result](#latest-live-result-startupsupport-rule-still-blocks-progress).
The live advisory is implemented and exactly replayed, but its conservative
support rule still blocks useful movement. The earlier offline results below
used a different history assumption and are not a navigation pass.

## Update: support-memory implementation

The replay experiments now distinguish **ground we have seen**, **the place the
robot was safely deployed**, and **ground its wheels have already occupied**.
Current wheel support no longer disappears just because LiDAR cannot see under
the robot. The original LiDAR maps and the earlier results below are unchanged.

Implemented and tested:

- A one-time startup footprint that stays at its original location; it does not
  follow the robot and keep declaring new terrain safe.
- Wheel-track memory without a short time expiry. Tracks follow revised pose
  estimates; missing pose history removes their support until it is available.
- Smaller forward/reverse moves, smaller turns, and a separate check for
  withdrawing along the recent estimated path rather than assuming perfect
  turning in place.
- Full checks for known obstacles and edge/height warnings, including outside
  any smaller positive-support region.

**Standing support now passes all 179 sampled poses** across the three recordings.
That alone does not fix movement: requiring every part of an assumed 80 × 120 mm
wheel patch to be covered still rejects many short steps. Some rejected steps
are missing only a few square centimetres at an edge.

We also ran a new stationary Isaac contact test. All four wheels stayed loaded;
their reported contact points span about **48 × 83 mm**. Those points are not
proof of a filled rectangular contact area. They do show why the old larger
rectangle should not be treated as a physically established requirement.

An explicitly experimental **40 × 60 mm central support region**, with the full
wheel/body hazard checks retained, gives the following offline results with
remembered tracks and a 2 cm historical-position margin:

| Scenario | Some smaller move or turn | Or a withdrawal along the recent path |
| --- | ---: | ---: |
| Straight out/back | 74/82 (90.2%) | 81/82 (98.8%) |
| Turn + out/back | 46/64 (71.9%) | 61/64 (95.3%) |
| Turn-only | 11/33 (33.3%) | 29/33 (87.9%) |

These include startup and keep every original one-second decision. A withdrawal
is **not forward progress or goal completion**. The test driver took a route
independent of this policy; these are not trajectories driven by the new logic.
The smaller support region is not yet qualified against unseen rocks/dropoffs.
The 2 cm margin is a sensitivity setting, not a measured error guarantee.

The next real bottleneck is therefore the **motion-support rule and recovery
behavior**, not another generic relaxation of the LiDAR classifier. Before
enabling commands, test the smaller support rule against controlled rocks,
holes and partial wheel support; calibrate relative pose/tracking margins; then
test a closed-loop startup move and short obstacle goals. Online advisory output,
controller integration, stopping validation and full resource gates are still
unfinished. No navigation safety or 95% goal-completion claim is made.

The [implementation ledger](implementation-plan.md) records source versions,
failed attempts, exact replays, contact fixture and optimization checks. The
[runbook](agent-runbook.md#contact-support-experiments-october-4) gives commands.

[Open the interactive support-gap examples](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-00cbe80725ad48339d4a29e7e084a6c7/support-gaps.html).
Select a scenario to see unchanged 10 cm LiDAR cells, separate wheel/startup
support and the remaining uncovered part of a 5 cm step. The final code passed
175 relevant regression tests and reproduced every decision after caching and
memory optimizations. The stationary physical fixture also passed; no new
terrain-controlled driving run has been performed.

## What these tests answer

Does the false-dropoff fix work when Isaac, GLIM and the terrain mapper run
together? Does the resulting map leave room for a useful next move?

These are headless, guarded test-driver runs, not autonomous navigation. The
driver uses simulator pose and a pre-surveyed clear route. GLIM uses the physical
IMU, a one-time GT initialization and synthetic RTK. The terrain mapper does not
receive GT terrain labels. All scenarios use the same selected optical v2 LiDAR
and the same pointwise-boundary library (`revision-70647a2b742c446eb61bd3d7141ea011`).

## Results

| Scenario | Recorded live maps | Clear floor recognized, 1–2 m ahead | Available forward move or turn, after first 15 s | GLIM horizontal error, p95 / worst |
| --- | ---: | ---: | ---: | ---: |
| 2 m out/back, 0.05 m/s | 819/819 | 98.18% | 32/67 (47.8%) | 9.56 / 14.05 cm |
| 15° turn + 2 m out/back, 0.10 m/s | 634/634 | 98.00% | 19/49 (38.8%) | 12.25 / 14.69 cm |
| +15°, −15°, return to starting heading | 321/321 | 92.12% | 0/18 (0%) | 9.16 / 12.64 cm |

All three scenarios have **zero false-occupied samples on the surveyed
clear ground**. This is not a measurement of missed rocks or dropoffs. Their
surveys and motion histories differ, so these are scenario results, not a paired
comparison claiming the turn degraded the algorithm.

Pose errors use the first-published acquisition pose, converted to the same
physical `base_link` as GT, with the fixed home anchor and no fitted alignment.

Every valid recorded map is scored against the independent 5 cm ground survey.
Motion checks use the first map in each fixed one-second bin, including startup.
They sweep assumed 80×120 mm patches at measured wheel locations, plus a
1.70×1.40 m body guard against occupied cells. Candidates are 25/50 cm forward or
steering moves and ±30° turns. **Reverse is not tested**: return legs may have a
clear reverse path without scoring forward progress. These checks also lack
braking and localization margins, so they are not permission to drive.

## What remains problematic

The old false-dropoff warning does **not** appear under the current wheels in
any of these runs. Exact replay reproduces every map before attributing its
unknown cells. The main remaining cause is **no LiDAR returns**, followed by
insufficient coverage of a cell—not falsely detected obstacles.

Coverage close to the robot is much weaker than coverage ahead. In the straight
run, clear-floor recognition is 74.1% within 1 m in front and 43.0% within 1 m
behind. In the turn-and-drive run it is 64.2% and 36.5%. Some gaps lie under the
current wheel patches, so rejecting every move that touches any unknown square
creates a startup/support problem. A small initial turn did not solve it.

The earlier long-loop replay's 99.3% post-startup availability therefore does
not generalize to these short routes. More varied observation history matters.
The turn-only scenario confirms that a small sweep is insufficient: all 33
sampled checks lack support, with no-return cells as the sole under-wheel cause.
Even an optimistic diagnostic ignoring unknown cells already under the wheels
still permits 0/33 actions; newly swept ground is also unseen. On the straight
run, that assumption improves availability only from 32/82 to 35/82.
No unknown cells have been relabeled free to make these numbers improve.

## Reliability and runtime

An initial straight attempt finished driving but failed recording shutdown:
the recorder saw the completion filename before its JSON contents were fully
written. Moving completion now uses the existing atomic-file publication helper.
Regression tests reproduce a read during writing and a failed native-data seal.
All three subsequent runs completed and sealed successfully.
The failed attempt remains archived and is excluded from acceptance results.

Mapper p95 / p99 update times: **50.79 / 57.18 ms** straight, **52.90 / 58.87 ms**
turn-and-drive, **60.21 / 64.67 ms** turn-only; peak mapper memory
**226.59 / 220.09 / 202.68 MiB** respectively. These
are live mapper timings, not full-stack real-time or autonomous-navigation
qualification. No missing live map projections in any of the three runs.

The independent native-ray reference test passes for turn-and-drive and turn-only. The straight
run has one missing PhysX reference hit, kept as a failed strict check; all its
other checks at that tick are within 2.63 micrometres. Original inputs are
unchanged. Every timed-window point and exact GT/native pose association was
checked separately in all three runs. That small reference-query failure
does not explain the large near-wheel blind area.

## Next improvement cycle

1. Keep this terrain classifier unchanged as the reference. The false-dropoff
   fix works; more classifier relaxation cannot manufacture unseen ground.
2. Separate **already supported wheel contact** from **new ground a move would
   enter**. Test any contact-based assumption conservatively, without declaring
   entire unknown cells safe. The optimistic result above shows that this alone
   will not solve the problem.
3. Compare a larger guarded observation maneuver with a short translation that
   reveals the blind area. Retain unknown cells and use the existing independent
   ground/obstacle checks; measure when a safe next motion actually becomes
   available. A larger turn is a hypothesis, not a demonstrated solution.
4. Add reverse and smaller-step candidates as **separate new metrics**, while
   preserving these original scores. Then test actual terrain-controlled local
   planning on clear and obstacle routes before claiming navigation readiness.

The longer-loop repeat was deferred in favor of the turn-only diagnostic because
the short tests exposed this more immediate limitation. No new long-loop or
obstacle-avoidance acceptance is claimed. The simulator is stopped; no captures
were deleted. Final focused host regression: **207 passed**; isolated ROS
recorder regression: **29 passed**.

## Evidence and reproduction

[Implementation ledger](implementation-plan.md) records every capture, scorer,
pose diagnostic, exact replay and failed attempt. The [runbook](agent-runbook.md)
and [live inspection guide](live-inspection.md) contain the recorder-first
commands. Start the recorder with `--natural-motion straight_slow`, `combined10`
or `turn15`, then use the matching headless launcher route. Keep the selected
library, frozen GLIM configuration, archive bind and guards unchanged.

After stopping Isaac, run `assess_navigation_gaps.py`,
`export_shadow_trajectory.py`, `measure_exported_glim.py` and
`explain_navigation_gaps.py` on the completed capture. `render_navigation_gaps.py`
creates a portable HTML gallery of fixed-time, actual recorded grids.

Fixed-time grid galleries (10 cm squares, unknown remains gray):

- [Straight out/back](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-3fb424165af846f0b74281757563ea26/navigation-gaps.html).
- [Turn + out/back](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-992fb14b6b964e16a1556449247acfce/navigation-gaps.html).
- [Turn-only sweep](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-9aaf2c46099c42f58fbdc3bd4ef30dc5/navigation-gaps.html).
## Latest live result: startup/support rule still blocks progress

October 4 continuation. The new advisory ran alongside the real LiDAR, GLIM and
terrain mapper in a fresh headless turn test. It sent **no motion commands**;
the existing guarded acquisition driver followed its declared route.

| Check | Result |
| --- | --- |
| Capture/integration | Completed: 1,937 native chunks, 322 valid maps/advisory updates |
| Live/replay agreement | All map outputs and advisory decisions reproduced exactly |
| GLIM error, 95th percentile | 7.3 cm horizontal, 3.8 mm vertical, 0.33° yaw |
| Standing allowed | 33/33 decisions |
| 5 cm move or 5° turn allowed, conservative rule | 0/33 decisions |
| Smaller support-region + startup fix, offline candidate | 3/33 decisions; still 0/18 after 15 seconds |

**This is not exploration-ready.** Zero allowed moves does not mean the whole
terrain map is unknown. It means the query insists on support in the small area
near the wheels, where the rover blocks the sensor's view. Requiring each entire
wheel patch to be LiDAR-confirmed also prevented this test from retaining any
wheel-track history.

Two useful fixes are implemented: motion queries use the newest available
same-frame robot pose, and the initial footprint is no longer shrunk for
historical pose uncertainty when it uses the exact same pose as the query.
Current unknown wheel locations do not prevent standing, and explicit hazards
still override remembered support. The historical 2 cm margin remains a
sensitivity assumption, not a calibrated error bound.

The physical rover tests explain why history needs care: a rover can stay level
with one wheel over a hole. That location must not automatically become a free
terrain cell for later approaches. Narrow supporting strips did hold the rover;
requiring every square millimetre of a rectangular wheel patch is also too strict.

The remaining operating question is whether startup has a deliberately cleared
staging area or only the actual robot footprint is known safe. These require
different startup policies. No larger free area has been silently inserted.
The next navigation gate is actual terrain-controlled goal progress, not merely
finding a possible retreat on a GT-driven recording.

Runtime is also unfinished: the combined live mapper/advisory measured 91.6 ms
p95 and 118.6 ms p99, above the current target. A single query is only 3–8 ms p95
in a separate synthetic benchmark, but that is not a combined runtime pass.
The new ROS inspection topic is `/terrain_shadow/contact_advisory`; existing
raw class and costmap topics are unchanged.

One sampled PhysX reference ray had no matching hit. All sensor bytes and GT
pose pairs were checked, so the pose and integration diagnostics remain useful,
but this run is **not terrain safety qualification**. The failed reference check
and every movement refusal remain recorded in the implementation ledger.

Evidence: mapper `revision-e8fad238e4e94f37be0869782e55cc74`, exact replay
`revision-13d4ac5bf18846ffac5c77809cc38446`, pose diagnostic
`revision-de22fbf19302429aaecbffc56b907313`. Full commands, changes and artifact
links are in [the implementation plan](implementation-plan.md) and
[runbook](agent-runbook.md#contact-support-experiments-october-4).
