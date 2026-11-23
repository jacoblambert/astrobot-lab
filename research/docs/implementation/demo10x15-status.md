# 10 × 15 cm rocks: live GLIM and traversability

Updated October 5, 2026. Both short and full-turn captures and their assessments
are complete. **Rock detection looks useful; the inner ground-support gap still
prevents a supported exit.** This is not full navigation qualification.

We added eight rounded test rocks to the existing natural world. They are
**10 cm above their local terrain and 15 cm wide**. The natural rocks, selected
LiDAR, GLIM configuration and **8 cm obstacle detection threshold** are unchanged.
This is a named opt-in fixture, not a new minimum scale for all natural assets.

## Results

| Measurement | Short ±15° observation | Full turn |
| --- | --- | --- |
| Capture | Completed | Completed, 359.8° turn |
| Test rocks detected during run / in final map | 7/8 / 7/8 | 8/8 / 7/8 |
| GLIM horizontal error, p95 / maximum | 7.72 / 9.53 cm | 13.59 / 18.96 cm |
| Native sensor geometry audit | Passed | Failed on 1 of 1,364,864 sampled rays |
| Final clear-ground support, 1–1.3 m | 671/865 = 77.57% | 854/865 = 98.73% |
| Final clear-ground support, startup boundary–1 m | 172/783 = 21.97% | 262/783 = 33.46% |
| False occupied surveyed ground samples | 0 | 0, across all maps |
| Small move or turn available | 33/33 decisions | 174/178 decisions |
| Supported straight exits beyond startup | 0/33 decisions | 0/178 decisions |

The short run detected every test rock with sampled native hits. The remaining
western rock had no sampled hits and stayed unknown; an existing natural boulder
lies between it and the robot. It remains in the denominator. Zero sampled hits
does not prove that the full sensor produced no returns.

During the full turn, the western rock received 144 sampled hits and was detected
in 1,073 of 1,777 maps. It became unknown again later, consistent with old evidence
expiring after it was obscured. It was **not marked free**. None of the eight
rock-apex cells was marked free in any full-turn map. These are eight controlled
objects, not a claim of perfect recall for every natural rock or placement.

**Rock detection and ground support are different results.** Small moves and turns
were available at all 33 short-run decisions. None of the tested straight paths
connected the starting assumption to newly supported ground. Changing the test
rock size does not by itself add ray-clearance reasoning to the terrain classifier.
The full turn has the same issue: within the inner band, 244 surveyed samples
had no returns, 260 had inadequate ground extent, and 17 had insufficient area.
Unknown here means missing support evidence, not a detected obstacle.

The rocks are 1.8–2.65 m from startup, outside the existing guarded motion region.
This tests their detection through real LiDAR/GLIM, **not detection in the
0.65–1.1 m blind band**, and not autonomous exploration. Motion used the declared
GT acquisition driver; GLIM used one-time GT initialization and synthetic RTK.
No fitted alignment was used. Full-turn vertical error p95 was 1.54 cm and yaw
error p95 0.318°. The older strict pitch diagnostic target still fails
(pitch error p95 0.495°); no threshold was changed. This is a local observation
test, not a long-distance route or unassisted-SLAM qualification.

## The one ray disagreement

At 76.37 s, the native sensor returned ground at 2.1773 m, while an independent
PhysX recast returned the edge of rock_0 at 2.1288 m: a 4.86 cm difference.
Moving the recast origin 49 cm along the same line returned ground instead;
moving it 51 cm returned the rock again. A float64 intersection check against
the actual float32-authored demo meshes finds no rock intersection for any of
these three rays. This supports a simulator intersection/reference edge case,
not a GLIM frame error; it is not a reproduction of PhysX's internal mesh cooking.

The strict 1 mm audit remains **failed**. We did not remove the ray, alter native
measurements, change thresholds, or rerun until a lucky pass. That limits formal
qualification but does not explain the widespread near-field unknown band.
All 67,602,670 transported points and 10,663 GT pose pairs were verified separately.

## What was implemented

- `tools/demo_rock_profile.py`: closed rounded meshes, terrain-relative sizing,
  explicit profile and independent simulator collision checks before capture.
- `OMNILRS_DEMO_ROCK_PROFILE` / `SELECTED_DEMO_ROCK_PROFILE`: opt-in simulator
  wiring. Empty/default leaves the original world unchanged.
- `tools/assess_demo_rocks.py`: exact live-map replay, per-object detection using
  actual footprint overlap, all maps retained, sampled native-hit counts.
- `tools/render_demo_rocks.py`: standalone report with the raw grid, GT outlines,
  pose errors and ground-support results.
- `tools/diagnose_demo_rock_rays.py`: preserves every saved recast disagreement
  and checks authored demo-mesh intersections without repairing measurements.
- `tools/test/test_demo_rock_profile.py`: dimensions, closed/outward mesh,
  route separation, profile validation, evidence hashes and scoring regressions.

An initial startup failed before movement because a test position overlapped an
existing boulder. Profile v2 moved that test object outward; no natural rock was
removed and no motion guard was loosened. Both the failure and subsequent runs
are preserved in the [implementation ledger](implementation-plan.md).

The first full turn stopped when an Isaac garbage-collection pause pushed the
IMU heartbeat beyond 500 ms. That launch had omitted our previously validated
lean/cache-disabled Isaac settings. Restoring those explicit startup settings
allowed the full turn to finish, including an ordinary in-motion collection.
No timestamp correction, watchdog relaxation or sensor/SLAM change was needed.

## Next step

Keep 10 × 15 cm as the declared demo test size. Focus the next implementation
on **connecting the known starting area to observed ground**, using bounded
near-field ray-clearance evidence rather than declaring a larger circle free.
First compare candidates on these frozen maps: supported exits must improve
without turning the known rocks into free space. Then test rocks and holes
inside the actual blind band and run new online, terrain-guided movement tests.
The current rocks at 1.8–2.65 m do not validate that near-field rule.

Full-turn mapper plus advisory processing: p95 100.65 ms, p99 121.06 ms;
peak mapper RSS 279.75 MiB. These are instrumented live timings, not completed
qualification under both resource profiles.

Final regressions: **327 host tests and 40 isolated ROS tests passed**. The
implementation ledger and runbook retain the commands, dependency versions,
failed attempts and exact evidence IDs.

## Inspect or repeat

Under the external archive root:

- Full-turn report: `revision-912257912eaf4198a86cd9e5f0954612/report.html`.
- Quarter-turn gap examples: `revision-698081e1f9c346f69ff6e0ce0afbf959/startup-observation.html`.
- Short-run report: `revision-feb907461614413098c2acf9186db976/report.html`.

The full report shows the 6 m raw grid, numbered test-rock outlines and all eight
object results. Green means classified ground, gray unknown, red occupied.

The [runbook](agent-runbook.md#startup-observation-extension) contains recorder-first
startup, the `rounded_10x15_v2` profile and assessment commands. These tools do not
enable Nav2 or grant motion permission.
