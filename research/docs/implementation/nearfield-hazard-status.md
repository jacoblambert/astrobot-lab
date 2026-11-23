# Near-field hazard tests — October 5

Follow-on: the user selected **10 x 15 cm**. The
[live GLIM/traversability rerun](demo10x15-status.md) now records the dimensioned
simulator tests. The results below remain the earlier analytic visibility study,
not evidence of live near-field support.

**The geometry idea is useful. Small increases in rock size also help, but we
have tested visibility so far—not the terrain classifier's final decisions.**

We tested 1,728 box/rounded-rock/pit placements with the selected LiDAR beams and
chassis occlusion, then repeated them using the recorded turn's XY/yaw path.
Finally, we ran a separate 1,536-placement rounded-rock size sweep.

## What matters for the demo

An ideal in-place rotation sees every eligible rock tested, even at 8 cm high
and 10 cm wide. The recorded turning path moves outward, which hides some of
the smallest nearby rocks. It is therefore important to test the maneuver we
can actually execute, not just a perfect rotation about the center.

For a fair size comparison, these results use the **same 72 locations** across
both placement banks and all sizes:

| Rounded rock, exposed height x width | Locations with at least 3 changed LiDAR returns in one view | Fewest changed returns in the best view |
| --- | --- | --- |
| 8 x 10 cm | 65/72 | 0 |
| 10 x 15 cm | 72/72 | 3 |
| 12 x 15 cm | 72/72 | 29 |

**12 cm high x 15 cm wide is a sensible candidate for the next demo test.**
The extra signal is useful margin; 10 x 15 cm barely produces three returns in
one placement. This is not a requirement to detect every hypothetical rock.
We will report ordinary-placement misses rather than treating one contrived
failure as a reason to stop the project.

We did not change the world configuration. Its scale range is currently 8–11,
which does not translate directly into exposed dimensions. Mesh shape, rotation
and burial in the terrain matter. Larger rocks can also narrow passages, so a
demo profile must check both visibility and remaining navigable space.

## Holes are a separate limitation

In the recorded-path model, 10 cm-wide pits were observable in 25/36 regular
and 26/44 shifted eligible locations. For 20 cm-wide pits this rose to 31/36
and 33/36; 40 cm-wide pits were observable at all 24 locations in each bank.
These fractions use different eligible locations and are not encounter probabilities.

Some beams pass over a small hole and still hit ground beyond it. Other beams
hit the pit's wall or bottom, so a hole does not necessarily mean missing returns.
The model handles both cases. Increasing rock size does not eliminate this
small-hole limitation; it should remain an explicit demo limitation.

## What these tests establish—and what they don't

- Correct first-hit geometry, including chassis occlusion and software range
  filtering. A rejected close rock return never reveals the ground behind it.
- Fractions over fixed regular and shifted placement banks. Every miss remains
  in the output. Cases intersecting the assumed clear startup area are retained
  separately, not counted as valid external hazards.
- Observable differences from flat ground: a return changes by over 5 mm or
  appears where none existed. Reports show thresholds of 1, 3 and 10 returns;
  missing expected returns are a separate count.
- **Not yet measured:** actual mesh/native-LiDAR behavior, noise, slopes,
  roll/pitch/Z motion, GLIM uncertainty, classifier recall, or autonomous progress.
  A ray hitting an 8 cm rounded rock may hit below its top, so visibility alone
  does not prove that the classifier will flag it.

The next step is to test baseline and candidate rock sizes through the actual
classifier, then use ray-clearance evidence to help bridge the short unknown
strips. A good sensor signal should not be lost merely because a whole 10 cm
cell lacks direct ground hits. Ground support and obstacle clearance must still
be represented separately.

## Reproduce and inspect

New tools: `tools/assess_nearfield_hazards.py` and
`tools/assess_recorded_hazard_views.py`; tests:
`tools/test/test_nearfield_hazards.py`. Commands and archive alias requirements
are in the [runbook](agent-runbook.md#startup-observation-extension).

All outputs are under the external archive root
`/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab`:

- Ideal views: `revision-0a7afddee096451a8d2d2ad2c79ab626/report.html`.
- Recorded-path comparison: `revision-2e8424e1a3484f70bbd6b40be4079071/report.html`.
- Size sweep: `revision-d07b97008ea945cf97105b8437e29d91/report.html`.

Each contains JSON results, per-placement/per-view counts and source hashes.
The size-sweep baseline reproduces the earlier recorded-path counts exactly.
**435 regression tests passed**, including 19 new geometry/reporting tests.
