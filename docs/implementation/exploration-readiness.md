# Getting to exploration

Updated October 6, 2026. See the [execution ledger](implementation-plan.md)
for immutable run IDs and the [runbook](agent-runbook.md#exploration-entry-diagnostics-october-6)
for commands.

**Correction after the mounting review:** the proposed 12 cm forward shift was
only a ray-geometry screen, not a mountable design, and is withdrawn as the
recommended next change. The existing raised model also needs its support
extended down to the body (a 35.4 mm visible gap was found). Layout changes are
authorized only with a plausible attachment and an occlusion audit. Earlier
“no exit” results do not prove that every move-observe-replan strategy fails.

## Bottom line

The current stack produces useful estimated-pose terrain maps and a cost
gradient around obstacles. It is ready for **inspection and planner shadow
tests**, not yet for terrain-controlled exploration. The immediate blocker is
getting from the startup area onto observed ground. A larger exploration run
driven by GT would not demonstrate that this problem is solved.

The latest 10 × 15 cm rock experiment saw all eight test rocks at some point
during a full turn. Seven remained detected at the end; the western rock was
occluded and its old evidence expired. Ground coverage was 98.73% at 1–1.3 m
from startup but only 33.46% between the startup boundary and 1 m. These are
separate denominators, not an overall 98.73% terrain-accuracy claim.

SLAM horizontal error was 13.59 cm at p95 and 18.96 cm maximum on that turn,
with GT initialization and synthetic RTK assistance. No large tracking failure
was observed. The run still has one strict native-reference ray discrepancy;
it remains diagnostic evidence, not a fully qualified terrain trial.

## Current trial

The opt-in `terrain_step` live trial completed: **22.8 cm actual forward progress**
toward a 30 cm estimated goal, then the 45 s controller deadline. It did not
reach the goal. A small move/turn was available at **49/49** sampled decisions,
and forward 5 cm at **39/49**. Thus the current result is not “nowhere to move.”

The first rejected sweep lacked only **0.245 cm²** of support; all 91 refused
controller updates were due to unconfirmed ground, not an occupied cell.
But the controller also spent more ticks turning than translating. GT comparison
found 12.6 cm p95 horizontal discrepancy at command consumption, including pose
age. Five-centimetre progress targets and a ~3° heading threshold are too
sensitive to that estimate. Zero completed internal steps does not mean zero
actual movement. No new LiDAR-confirmed wheel-history anchor was added.

All **484 maps and 483 controller updates replay exactly**. The final code passes
226 host and 71 sourced ROS tests. Six failed integration attempts are preserved
in the ledger, not counted as successes. No sensor, mounting, classifier or
support rule changed. GT selected no commands; it remained an independent
emergency veto and evaluation source (GT GLIM seeding / synthetic RTK assistance
remain declared). The simulator is stopped.

Next work should separate **short-step control stability** from **overly strict
wheel-support requirements**, and check why traversed contacts never extend
retained support. Test those changes against both benign gaps and actual hazards,
then repeat this same bounded trial. This result does not justify relocating
the sensor, prove an exit, or qualify exploration. See the runbook's
[bounded trial](agent-runbook.md#bounded-terrain-controlled-trial-october-6).

## Earlier offline work

Added reproducible **offline diagnostics and regression tests**, not a new live
mapper or permission to drive through unknown ground:

- Remembering all previously classified ground still gives no straight exit
  on the full turn. Longer memory alone will not fix this.
- Combining supported portions from four shifted grids still gives no exit
  on the reverse-20 cm test, and adds about 89 ms p95 processing. Not promoted.
- Added full turn-then-drive queries, keeping the original straight-path
  results separate. These test the turn itself, not just the destination:
  **0/26 exit opportunities** after reverse/return and **0/178** during the
  full-turn recording. All 416 + 2,848 added paths remain in the reports.
- Screened steeper beams while preserving chassis occlusion. With the current
  mount, they hit the robot and do not close the blind strip.

For those offline comparisons the selected mount, beam table, GLIM settings,
classifier, startup assumption and live movement rules were unchanged. Existing
completed recordings supplied their replay inputs; they were not live trials.
All original maps/advisory outputs replay exactly. **287 focused tests pass**;
this does not replace native sensing, live driving or compute qualification.

## Corrected next investigation

The startup assumption extends about **61 cm** from the initial center. On
flat ground the current sensor first sees ground at about **73.5 cm**. Rocks
can be seen before their surrounding ground is visible, but a clear ray above
the ground cannot prove that a hole is absent underneath it.

Earlier 100% near-ground yaw coverage was a **sensor-only** test without the
rover body. Later we corrected rays seeing through the chassis and raised the
optical origin from 25 to 45 cm. Good outer-map coverage and a fully confirmed
wheel path are also different measurements. The old and current percentages
must not be presented as a like-for-like loss caused by SLAM.

The frozen-map turn/drive tests do not test gathering new information while
moving. A new idealized flat-ground test does: reverse in 5 cm steps to 15 cm,
observe, return to center, turn 15°, repeat. It admits all 213 motion segments
under the fixed startup support, but still finds no complete straight exit.
Perfect poses are used, so this reproduces a problem without SLAM error.
At the final pose a 10 cm step is allowed; a 20 cm step lacks 9.08 cm² of
required support. This is **not** a live result: no skid, wheel/mast occlusion
or hazards, one ideal scan per endpoint, and the original 60 s evidence expiry.

Keep the existing hole scope. The new live trial above takes priority for
control/support diagnosis. Separately model a real attachment at the current
location, including its occlusion; do not assume relocation is needed. The user authorizes physically
plausible layout changes, not floating coordinates. The 12 cm forward shift
is withdrawn as a recommended design; its archived ray-only result remains
historical. Do not enlarge startup support or paint unknown ground free.

## Integration sequence after a usable local connection

1. **Demonstrate a usable exit.** Validate the chosen sensing/policy change on
   fixed near-field hazard placements and a fresh coupled startup run. Report
   missed hazards and unknown terrain, not just rock recall or average coverage.
2. **Connect a planner in shadow mode.** Propose short trajectories toward a
   longer goal. Query the existing current-pose wheel/body sweep, recording
   available/blocked/unknown and why. Keep observation turns separate from
   progress. Do not send velocity commands yet.
3. **Test a short closed-loop route.** Use estimated poses and terrain for
   decisions; GT is evaluation-only. Measure actual tracking and stopping
   margins, then test stale maps, missing poses, resets and shutdown. An
   unavailable response must stop motion, not trigger an unknown-space bypass.
4. **Qualify the combined workload and new scenarios.** Keep the existing
   50/100 ms p95/p99 comparisons and both resource profiles. Then use frozen,
   held-out terrain cases for the declared hazard family and the existing
   ≥95% independently safe-motion availability target.
5. **Increase route length, then enable exploration.** First reproduce the
   16 m loop under terrain-controlled motion; then predeclare a longer route.
   Track distance/progress, interventions, missed hazards, false stops, SLAM
   error and memory/latency over time. GT-driven acquisition is a separate test.

## Existing integration points

| Component | Code / output | Important limit |
| --- | --- | --- |
| Native returns and causal mapping | `tools/live_local_terrain.py` | Uses native acquisition-time evidence, not sparse GLIM display clouds. |
| Terrain classes | `/terrain_shadow/classes` | Unknown is distinct from free; six-channel replay retains rejection reasons. |
| Obstacle cost gradient | `tools/local_terrain/costmap.py`, `/terrain_shadow/costmap` | Display/advisory output; inflation does not establish wheel support. |
| Current-pose short-motion query | `tools/local_terrain/contact_advisory.py` | Full wheel/body sweep; no command authority or calibrated stopping certificate. |
| Live inspection results | `tools/local_terrain/contact_runtime.py`, `/terrain_shadow/contact_advisory` | Fixed small candidates at 1 Hz, not a controller-rate service. |
| Existing exploration manager | `astrobot-lab/src/exploration_manager/exploration_manager/exploration_manager_node.py` | Legacy unknown-space/bypass behavior needs a dedicated integration review before connecting commands. |

The raw map's age channel is not the oldest age of all evidence supporting a
cell. Do not use it alone as a controller freshness certificate. Map/pose
generation, receipt time and actual stopping distance remain part of the
integration work, not something the visible costmap already guarantees.
