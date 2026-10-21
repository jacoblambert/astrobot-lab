# Decision note — SLAM + terrain robustness for Nav2 (2026-10-20)

Question: is GLIM+RTK localization feeding the `live_local_v2` layered terrain good enough to act
on, so Nav2 integration can resume? Criteria: act on gaps (C1), tolerate 10–20 cm SLAM error
(C2), never diverge catastrophically (C3), never miss a large obstacle (C4).

This revision replaces the Oct 19 note. Scoring now uses the costmap Nav2 would actually see, the
navigation-mode supervisor runs in the C3 loop, and real GLIM error is fed into terrain. How to
re-run everything: `docs/implementation/agent-runbook.md`, section "October 20".

**Verdict: integrate, with the publisher/config conditions below.** Each condition is
confirmed by a measured failure. The supervisor fix is done and C3 now passes (update at the
end of the Oct 20 session, after your answers on RTK noise, turning clearance and the capture
guard).

| | Verdict | Key number (scorecard) | Caveat |
|---|---|---|---|
| C1 gaps | **Pass, with the bounded + shadow policy** | 100 % of frames have a Nav2-acceptable motion on recorded and replayed GLIM poses; 0 sweeps over ≥ 0.15 m hazards; hidden-hazard rate 0 | shipped `GapPolicy` (isolated cells only): 60 % on the detour |
| C2 SLAM error | **Pass** | open-floor phantom ≤ 0.28 % of observed cells; phantom ahead ≤ 3.5 % of frames at 15 cm / 1° | knee at ~0.6° time-varying tilt, 20 cm horizontal, 2–3 cm z |
| C3 divergence | **Pass** (RTK σ 0.02 + 0.001·d, per-axis innovation gate) | supervised nominal p95 4.5 cm / max 9 cm; published error while permitted ≤ 31 cm (the 0.3 m outlier); a 1 m outlier burst is rejected and holds motion at 2.0 s | was 0.58–0.89 m while permitted at the old 0.6 m RTK noise; bare-GLIM 1 m outlier still pulls 1.1 m (no gate without the supervisor) |
| C4 large obstacles | **Pass on what exists; sample gate fails** | 0 unmarked frames within 2 m for ≥ 0.20 m objects, all 46 jobs; 0 footprint false-safe | only 2 distinct large rocks, closest 1.26 m, 0 danger-range approaches |

Scorecards (archive):
- terrain full: `revision-4b085de1` (46 jobs, 385 s re-score from cache, 1,242 s cold);
- terrain quick: `revision-5fb0bd36` (63 s);
- C3: `revision-1cc4aa3f` (12 jobs, 847 s, PASS). Before: `revision-5fd76bbd` (old RTK noise),
  `revision-fd6ec0c2` (low noise, old gates: outlier1 101 cm while permitted).

## What the new loop found

1. **The shipped unknown policy is the C1 blocker under real Nav2 semantics.** Configured costmap
   = `navigation_costs` → StaticLayer (non-trinary) → InflationLayer → padded footprint, with
   Smac2D `allow_unknown: false` and MPPI `track_unknown_space: true`. The Python contract
   matches installed Nav2 inflation and swept-area results exactly: 3 random grids, 180 motions
   (`tools/test/test_nav2_contract.py`, container).

   Detour `slam`, frames with an acceptable motion:

   | policy | frames with a motion |
   |---|---|
   | strict | 0 % |
   | shipped isolated-cell `GapPolicy` | 60 % |
   | `mode="bounded"` | 100 % |

2. **The publisher must clear the rover's own footprint.** The rover stands in its own LiDAR blind
   disc: unknown, more than 0.5 m from measured ground. A NO_INFORMATION start cell fails
   Smac2D. Unknown under the unpadded body is published free; occupied stays occupied
   (`nav2_contract.clear_footprint`).
3. **An obstacle's unknown flank must be published lethal.** MPPI with `track_unknown_space`
   does not veto NO_INFORMATION under the footprint. The 0.42 m rock `instance_38` has unknown
   flank cells (negative edge, positive edge, unsupported extent) one cell from its marked core.
   With plain `bounded`, 275 accepted sweeps on realistic poses (164 with GT poses) cross it.
   Publishing unknown within 0.20 m of an occupied cell as 100 (`occupied_shadow_lethal_m`)
   gives **0**, for a 4 % drop in available primitives. This is the gated policy
   `nav2_bounded_shadow`.
4. **Stock Nav2 footprint checks miss interior cells.** On the rotated-wall fixture with the v2
   library (`revision-bda8469b`), installed centre + perimeter checks accept 3 poses whose padded
   footprint crosses the wall (2 body collisions). The full-area check accepts 0. Nav2 needs a
   full-footprint collision check (a custom critic or the area check), or inflation that keeps
   the centre check ahead of thin obstacles.
5. **Supervisor (as found at the old RTK noise; fixed, see the update below).**
   - **Holds work.** It drops `motion_permitted` 1.99–2.01 s into every RTK outage. It
     restarts and permits motion again 2.6–105 s after RTK returns; restart needs the rover
     stationary, and the recorded rover keeps moving.
   - **Init comes from noisy RTK.** It initialises GLIM from 1 s of RTK. This capture's
     synthetic RTK has σ = 0.05 + 0.01 × the 55 m distance to the home beacon: 0.6 m horizontal,
     1.2 m vertical.
   - **Result.** z bias is 23–62 cm. Corner transients reach 0.58–0.89 m while permitted;
     bare GLIM from the GT seed reaches 0.32 m at the same corner.
   - **Gates can't catch it.** Its disagreement fault, max(0.5 m, 6σ) = 8.9 m, and its RTK jump
     gate, 6σ, are both inert at this σ. The 0.3 m and 1 m outliers pass straight through.
     GLIM absorbs them, so they add at most about 20 cm.
6. **GLIM latency has margin at real time.** At sim rate 1, p95 is 0.18 s and max 0.23 s,
   against the supervisor's 0.5 s `pose_stale` limit. At rate 2 the limit trips, so supervised
   replays must run at rate 1.
7. **Real GLIM error is tolerated by the terrain.** The loop16 terrain replayed with C3 GLIM
   poses gives open-floor phantom ≤ 0.28 % and 100 % of frames with a motion. That covers bare
   nominal, a 120 s outage, supervised nominal, and a supervised 1 m outlier. The bare-nominal
   coupling matches `slam` within 2 % of headings.
8. **Turning clearance: escape-permitted (your call: skid risk accepted).** A motion is allowed
   when it never comes closer to any conflict than the start already is
   (`CircularClearance.permits`), so an in-place rotation is no longer vetoed by a cell already
   inside the clearance circle. The old veto test is now
   `test_rock_in_additional_slip_envelope_permits_rotation_but_not_approach`; the driver's
   0.05 m drift allowance and 0.75 m turn-translation limit still bound a skid.

   **Update.**
   - You confirmed the 0.6 m noise is not a hardware model. synthetic_rtk now defaults to
     σ = 0.02 + 0.001·d (7.5 cm horizontal, 15 cm vertical at 55 m). C3 replays the same
     recorded drive by rescaling each fix's own noise sample (`--rtk-sigma`).
   - At that noise the init bias is gone: supervised nominal error is 4.5 cm p95. But GLIM
     now follows RTK, so the 1 m outlier reached 101 cm while permitted. The old gates never
     fired, because z variance widened a 3-D trace σ and the jump gate allowed 2 m/s.
   - Per-axis gates alone were not enough. The fix-to-fix gate rejected 92 burst fixes, then
     one noisy fix slipped inside the threshold and the rest of the burst chained after it.
   - The supervisor now gates each fix against GLIM's own position at the fix stamp, with a
     0.1 m GLIM allowance plus 0.5 m/s over any uncovered time. Fix-to-fix comparison
     remains only when GLIM is not running. A persistent outlier then trips the 2 s outage
     fault: motion is held, and the published error is 10 cm.
   - Replay artifact: "permitted again" after an outage swings between 3 and 160 s. Restart
     needs 1 s stationary, and the recorded drive's corner settles are 1.00–1.05 s, so it
     comes down to timing. In a mission the held rover stands still.

## Decision

Integrate v2 as the Nav2 costmap source, with these changes in this order:

1. **Publisher (`tools/live_local_terrain.py`): publisher side done; Nav2 wiring next.**
   - Done: `planner_gap_policy:=nav2_bounded_shadow` (recorder `--planner-gap-policy
     nav2_bounded_shadow`) publishes `/terrain_shadow/planner_costmap` through
     `nav2_contract.planner_costs`: `navigation_costs` under
     `PUBLISHED_POLICIES["nav2_bounded_shadow"]`, then `clear_footprint` at the record pose.
     The audit scores through the same function; on the 232 cached frames of the last quick
     run the old and new paths give identical grids (2,784 of 2,784). The manifest records
     the policy and footprint.
   - Next: run the publisher in the Nav2 launch. The local costmap's StaticLayer reads the
     window. The global costmap reads a mosaic of the windows (item 4: the window alone limits
     global goals to ~2.5 m). Both replace the legacy `slam/bev_costmap`. Add full-footprint
     collision checking to the controller, then re-run `quick` and `full`.
2. **Supervisor init and gates: done** (RTK noise lowered, per-axis innovation gate, C3 PASS).
   Before real hardware, set GLIM_ALLOWANCE and MAX_SPEED from the receiver's actual
   behaviour.
3. **Stress drives (C4 sample, speed-realistic C2/C3): approach_field and transit50 captured
   and passing.** The capture guard surveys a 1.35 m corridor about the route
   instead of its bounding box, and exempts only declared demo rocks from the preflight survey
   (the per-step footprint guard is unchanged). Two routes on seed 43, both clearing every
   rock box by ≥ 1.43 m:
   - `transit50`: a 50.5 m seven-leg loop at 0.18 m/s, every in-place turn ≤ 105°;
   - `approach_field`: 9.5 m out past 0.12 / 0.20 / 0.30 / 0.40 m demo rocks whose edges are
     0.95 m from the centreline (inside the 1 m C4 danger range), then 9.5 m in reverse on the
     same lane: 6 danger approaches to ≥ 0.20 m rocks.

   The live attempts (all preserved on the archive, all stopped safely by their guards):
   - `6bf40592`, out-and-back with a 180° turn: skid left the 25 cm bound at 141°. The base
     turns about a point 9–13 cm behind `base_link` (fit to loop16 and this log), and past
     ~120° the rest of that orbit is sideways, which a skid-steer cannot correct. Fix: the
     waypoint loops cap in-place turns at 120°, approach_field reverses home, and a v3 turn
     lead (loop16 keeps v2). A kinematic model gives ≤ 0.17 m on every turn.
   - `569ba2de`, reverse return: lateral tracking ≤ 3.6 cm over 9.5 m, but steering at the
     endpoint bearing with 3 cm lateral and 1.5 cm to go swung the heading to 8.6°, past the
     0.15 rad bound. Fix: v3 legs hold the leg heading for the last 0.30 m and arrive on the
     along-leg distance.
   - `d60b6395`: the recorder's 0.5 s wall-clock request heartbeat lapsed at sim 134 s, at a
     0.37 s generation-2 GC pause in the sim. Every capture, loop16 included, has that pause
     there (tick gaps 1.1–1.6 s), so this one is intermittent.

   - `ae9ba662`: GLIM exited at startup. The archive cleanup had removed the GLIM plugins the
     frozen config loads, along with the config, the terrain library and every Oct 19 suite
     fixture's capture data (runbook: "Archive cleanup (Oct 20)"). Rebuilt: config `ed871eab`,
     plugins `9f0c2307`, library `06a82bed` (reproduces all 1,163 recorded projections of
     `4ac1810d` exactly).

   **approach_field completed** (5th attempt): capture `2379e8d0`, recorder `377f2609`,
   terrain `507b46e8`, published with `nav2_bounded_shadow` live. GLIM first-published
   horizontal p95 / max 4.4 / 7.5 cm, vertical p95 1.6 cm (pose audit `4046718a`). Suite
   `approach_field` (run `42505fb0`): **C1, C2, C4 PASS**. All three ≥ 0.20 m rocks entered the
   1 m danger range on both passes, 0 unmarked danger frames, 0 accepted sweeps over a ≥ 0.20 m
   hazard under `nav2_bounded_shadow` on every pose profile. The old `isolated` policy accepted
   22 such sweeps on `slam`, 47 on `gt` and 0–40 under the pose-error profiles: close approaches are where the
   bounded+shadow policy earns its place.

   **transit50 completed** (4th attempt, Oct 21): capture `91cbf324`, recorder `ed0eb26e`,
   terrain `215e1aaf`, all 32,331 native chunks integrated. Earlier attempts stopped on sim
   capture guards, which you ruled convenience-only for sim (runbook, "New stress drives"):
   the 2 cm corridor floor-roughness limit (2.14 cm peak; now recorded, not gated), a
   40,000-frame archive cap below transit50's declared 55,000, and 0.5 s lease heartbeats
   that the recorder's own growing GLIM-revision decode starved at 510 sim-s (now 2 s).
   - GLIM first-published horizontal p95 / max 4.3 / 7.4 cm, vertical p95 1.5 cm over 50 m at
     0.18 m/s (pose audit `8e58f69b`).
   - Suite `transit50` (run `be5f3188`): **C1, C2 PASS**. 4/4 ≥ 0.20 m rocks marked within
     2 m on every profile; 0 accepted sweeps over a ≥ 0.20 m hazard under
     `nav2_bounded_shadow` on every profile (`isolated`: 112 on `gt`, 10 on `slam`). Open-floor
     phantom 0.04 % on `slam`; the tilt knee `r0.6` reaches 3.4 % (5.0 % combined), as on the
     short drives. C4's sample check fails by design: this route keeps ≥ 1.4 m from rocks.
   - Suite `c3_transit` (run `bcb36c2e`): **C3 PASS**. GLIM-only nominal 4.3 / 7.3 cm; outage120
     peaks at 19 cm; a 1 m outlier pulls bare GLIM 1.09 m (no gate). Supervised: motion held
     ≤ 2.01 s after every outage; worst published error while permitted 26 cm (`outlier0.3`,
     below the gate); `outlier1` is caught (13.7 cm). Bare jobs must run at rate 1 on long
     drives: at rate 2 GLIM lagged 53–60 s (run `8ddc7c18`, discarded).
   - Suites rebuilt on the recaptured fixtures (Oct 21): `quick` on detour8 `833118eb`, `c3` on
     loop16 recorder `2623e294` (run `1ec29770`, **C3 PASS**, all jobs at rate 1), and `full`
     on detour8 / loop16 / transit50 / approach_field plus four real-GLIM coupling profiles
     (run `c14503b1`): **C1, C2, C4 PASS**, 0 accepted sweeps over a ≥ 0.20 m hazard under
     `nav2_bounded_shadow` anywhere. Coupling excludes supervised runs that restart GLIM (the
     replay has no map reset); sup `outlier1` restarts on this recorder (44.8 % phantom in run
     `d9b3afff`), so `outlier0.3` (27.7 cm published error, no restart) replaces it. square8 and
     turn360/turn15 are not yet recaptured.
   - Re-permission after an outage again swings run to run (`outage30`: 322 s vs 28 s). This
     is the replay artifact noted under item 2's gates (restart needs 1 s stationary; recorded
     corner settles are 1.00–1.05 s), not a supervisor defect.
4. **Nav2 offline replay (Oct 22): the recorded costmap passes the real planner; the global
   costmap needs a mosaic.** `tools/nav2_replay_check.py` feeds the recorded
   `/terrain_shadow/planner_costmap` (published live under `nav2_bounded_shadow`) into a real
   `planner_server`, with SmacPlanner2D, StaticLayer and InflationLayer from
   `nav2_phase3_bev.yaml`. A second instance plans on a GT rock-core grid (≥ 0.10 m) on the same
   lattice. Each shadow path is moved into GT by `gt_base · slam_base⁻¹`, and the body
   footprint is swept along it, with in-place turns at the start and at kinks, against the GT
   rock cores. Goals whose GT body plus padding touches a rock are skipped (no mission would
   send them).
   - Local window, 16 goals at 2.4 m every 2 s (5 s on transit50):

     | Drive | Run | Goals | Found / GT-feasible | Transit contacts ≥ 0.10 m |
     |---|---|---|---|---|
     | loop16 | `9d0f37f4` | 2,182 | 98.5 % | 0 |
     | detour8 | `55ad6227` | 1,791 | 97.5 % | 0 |
     | transit50 | `df2f687b` | 1,570 | 99.2 % | 0 |

     Every miss is Nav2 error 208 (no valid path through still-unknown cells), and no shadow
     path exists where GT has none.
   - Arrival contacts: 2–3 per drive on the shadow map and 7–10 on GT, all in the last 0.65 m.
     Smac2D plans for the rover's centre and stops within its 0.35 m tolerance, and the body's
     0.495 m front overhang exceeds the 0.435 m inscribed radius. The controller (MPPI
     `consider_footprint`) and the goal checker own this; the mission should also check the
     footprint at the goal.
   - **Global extent.** StaticLayer resizes a non-rolling costmap to each map it receives (the
     recorded master costmap is 60 × 60). Pointing the global costmap at the 6 m window would
     make every goal past ~2.5 m unplannable.
     `local_terrain/global_stitch.py::GlobalStitch` mosaics the windows on their own lattice;
     the latest observed value wins, and unknown never erases.
   - Planned on that mosaic to home and to points 5/10/20/40 m back along the route
     (`--global-map`): loop16 `c50bc10d`, detour8 `81646ef5` and transit50 `3947fd31` found
     181 of 181 goals, up to 13 m away, with path length ≈ GT, and 0 contacts in transit or at
     arrival.
   - Integration plan:
     - the global costmap's StaticLayer reads a published mosaic (`allow_unknown` true for
       exploration goals);
     - the local costmap's StaticLayer reads the live window;
     - both replace `slam/bev_costmap`.
   - Not covered: the controller, and closed-loop effects such as the costmap lagging the
     rover.

Then re-run the 42/411/7 gates with Nav2.

**Watch:** tilt/z margin (C2). If nominal-speed driving pushes the tilt deviation past ~0.5°
p95, make the height tolerance pose-covariance-aware.

## Stop carrying (after agreement)

- The legacy `probabilistic_voxel_mapper` path, once v2 feeds Nav2; the gated R-GPF and
  ground-floor options; and the 5-library `#if` matrix in `layered_terrain.cpp`.
- `exploration_manager` dead params, breadcrumb recording, `_line_of_sight_clear`, and the
  unreachable map→odom watchdog.
- The 17 unreferenced `tools/` scripts, the orphan `install/` packages, and the stale
  fallback-smoother comments.
- The old C3 runs that each copied a 3.4 GB input bag (Oct 19, 9 runs, ~31 GB on the archive).
  Shared bases replace them. Delete only with your OK.
- `CLAUDE.md`: rewrite to the current stack.
