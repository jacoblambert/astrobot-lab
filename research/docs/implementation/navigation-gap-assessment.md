# Do terrain gaps prevent a useful next move?

**Follow-up:** the density-dependent false-dropoff bug is now fixed in the tested
candidate. After-startup move-or-turn availability improved **70.9% → 99.3%** on
the same recording. See [the fix and before/after reassessment](false-dropoff-fix.md).
The original measurements below are preserved as the baseline, not current
candidate performance.

October 4, 2026. **Many gaps do not matter, but we cannot yet say that the map
always permits useful progress. The main remaining problem is the mapper's
possible-dropoff rule rejecting surveyed clear terrain, not missing nearby rays.**

[Interactive HTML report with wheel-path overlays](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-97f67794578d4475b6f2f7ecda899d1d/navigation-gaps.html)
· [Three example grids](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-91840edd103646ed82afcf740aed4c3a/wheel-paths.png).

## What was measured

We reused the completed 16 m loop: all **2,991 recorded usable class maps**, with
no missing maps. Independent simulator floor samples were compared with every
map. At 300 fixed one-second intervals, we tested 25 cm and 50 cm straight or
steering moves, and 30° turns in either direction. Startup stays in the results.

A move counts as progress only if it gets at least 5 cm closer to the next route
waypoint. Every cell touched by the swept wheel patches must have support.
Known obstacles anywhere in the swept body guard block the move. Unknown cells
between the wheels do not automatically mean missing wheel support. A turn is
reported as an observation opportunity, not progress or guaranteed new visibility.

These are offline questions asked of recorded maps, **not executed navigation
decisions**. The original driver used GT and a pre-surveyed clear route. Later
good maps therefore do not prove the robot could autonomously get past startup.

## Does the map recognize clear ground?

The reference is the independent pre-motion 5 cm floor survey, placed using the
declared world/map transform—not fitted to the point cloud or GLIM trajectory.
Percentages below include startup and unknown samples.

| Surveyed ground | Marked free | Marked occupied | Unknown |
| --- | ---: | ---: | ---: |
| Front, 0–1 m | 89.87% | 0% | 10.13% |
| Front, 1–2 m | 94.82% | 0% | 5.18% |
| Front, 2–3 m | 91.34% | 0.00095% | 8.66% |
| Rear, 0–1 m | 88.84% | 0% | 11.16% |
| Rear, 1–2 m | 84.96% | 0% | 15.04% |
| Rear, 2–3 m | 83.09% | 0% | 16.91% |

So false occupied labels are negligible on this sampled clear route. The
near-field problem is withholding support, not inventing solid obstacles.
These are repeated sample/map exposures, not independent statistical trials.
Only surveyed ground is covered; this is **not missed-obstacle accuracy**.
Finite samples also do not prove that every part of a cell is clear.

## Do the gaps stop the next move?

| Map requirement | Successful sampled decisions |
| --- | ---: |
| Entire swept body footprint free, move toward goal | 67/300 (22.3%) |
| Wheel support plus no known body obstacle, move toward goal | 187/300 (62.3%) |
| Same wheel/body check, move **or turn** | 202/300 (67.3%) |
| Move or turn after the first 15 s | 202/285 (70.9%) |

Requiring the whole footprint to be green is clearly overrestrictive. Choosing
short steps also helps: straight 25 cm motions have support in **195/300**
samples, versus **148/300** for straight 50 cm motions, irrespective of goal
direction. But there are still meaningful interruptions under the wheel-support
test. The longest sequence of failed sampled decisions runs from **+152 to
+212 seconds**, spanning movement and a turn. This was not a physical trap: the
GT-controlled robot continued through it.

No tested motion was rejected for an occupied cell. Of the 98 decisions with no
supported move or turn, **96 already have unknown cells under the current wheel
patches**, typically four cells. Since every motion includes its starting
position, those cells reject all candidates immediately.

An intentionally optimistic follow-up ignored unknown cells touched by the
current wheel patches, while still rejecting occupied cells and unknown new
ground. Availability rose only to **213/300 (71.0%)**. Therefore support memory
under the current wheels alone will not solve this. That comparison is not a
safe policy: contact with part of a 10 cm cell does not validate its whole area.

## Why are those cells unknown?

An exact replay reproduced all **2,992 original mapper outputs**, including the
terminal invalidation, before reading the classifier's reasons. After the first
15 seconds, among unknown cells touched by current wheel patches:

- **2,653/2,665 (99.55%)** are rejected by the possible-dropoff rule.
- **12/2,665** lack enough support for the full cell extent.
- **Zero** have the no-returns reason.

This identifies which rule is blocking motion, not yet which input made that
rule fire. In `layered_terrain.cpp`, low returns or a steep fitted surface can
trigger the rule; native boundary ambiguity can also trigger it. The result
spreads to neighboring cells to protect against partially observed hole edges.
That protection is useful for real holes, but its clear-ground rejections are
now a specific, measurable target. Possible causes to distinguish are local
surface modeling, within-scan boundary interpretation, and accumulated SLAM
registration scatter. Do not assume ordinary incline alone is the cause.

## What about “we would have seen a big rock”?

That is a useful distinction **where rays actually examined the relevant
space**. We should separately represent ground support, evidence against a
large obstacle, and genuinely unseen terrain. But a clear ray above the ground
does not rule out a small rock or a hole, and no-hit probability needs a tested
visibility/detection model. This recording does not calibrate that probability.
Also, the main post-startup rejection found here is not “too few hits”: it is
the classifier interpreting available evidence as a possible edge.

## Recommended next improvement cycle

1. Reproduce one cell from the +152–212 s interval. Separate low-return, fitted
   normal and native-boundary triggers; inspect its actual returns and local
   ground model. Turn that exact case into a regression fixture.
2. Fix the responsible trigger on the same recording. Keep real hole rims,
   partial walls, small obstacles and slopes in the regression suite. Do not
   erase unknowns or disable dropoff protection globally.
3. Rerun this unchanged short-motion evaluation. Look for improved useful-motion
   availability and shorter refusal stretches, not merely greener pictures.
   Then test fresh obstacle/negative-obstacle routes and a guarded startup turn.

No mapper, cost policy, driving guard or SLAM parameter changed in this work.
The four-wheel patches remain an **80×120 mm assumption** at measured wheel
locations, and the body check uses the existing 1.70×1.40 m acquisition guard.
No localization margin or measured stopping model was added, so these results
are not navigation safety qualification. Motion availability was sampled at
1 Hz; intervening maps were still counted for floor classification, not motion.

## Evidence and reproduction

Assessment: `revision-282097f0d80a4d98aa4275dd31f7d395` (82.31 s offline).
Exact rejection explanation: `revision-2af143de7f784820a3b9b57f69006c5d`.
Optimistic grounded-start comparison: `revision-666b24331b2047aab1fe73a080886683`.
Reports retain source hashes, per-time scores, all denominators and assumptions.
The independent floor survey is separate from the two missing ray re-casts in
the earlier sensor-reference audit; that older audit remains failed, unchanged.

Tools: `assess_navigation_gaps.py`, `explain_navigation_gaps.py`,
`assess_grounded_start.py`, `render_navigation_gaps.py` under `tools/`.
Commands and prerequisites are in the [runbook](agent-runbook.md).
Final affected host regression: **132 tests passed**. No simulator was started;
no capture files were deleted. Full HTML browser rendering was not tested;
the wheel-overlay SVGs were rasterized and visually inspected.
