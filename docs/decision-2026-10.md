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
5. **v2 Nav2 stack built, closed loop passing T1–T3 (Oct 22, seed 43).**
   `phase3_nav`/`phase4_exploration slam_stack:=v2 nav2_params:=…/nav2_v2.yaml`.
   - **Sim:** the nav stream (`OMNILRS_SELECTED_NAV`, `docker-compose.selected-nav.yml`)
     publishes the same selected-LiDAR points and native chunks without the capture harness,
     at 0.4–0.47× real time.
   - **Map:** the publisher's `deployment:=nav` serves `slam/reset_map` and the mosaic. The
     supervisor's map readiness reads the v2 window. The legacy BEV mapper is not launched.
   - **T1 static:**
     - supervisor HEALTHY `validated`;
     - `map→odom→base_link` only, no `gt_map`;
     - window lag p50 / p95 0.217 / 0.233 s;
     - the global costmap is the 600 × 600 mosaic, the local costmap the live window.
   - **T2, 10 scripted 3–8 m goals** (`tools/v2_nav_trial.py`, trial `a8d8191a`):
     - 9 succeeded;
     - 0 GT body contacts with ≥ 0.10 m rocks;
     - SLAM error p50 / p95 / max 2.4 / 6.1 / 12.9 cm.
     - The miss was a publisher fault, `measurement after arrival: clock domains disagree`.
       Under nav load its `/clock` queue lagged, so a GLIM revision got a stale receipt bound.
       The supervisor held (`map_stale`), reset, restarted GLIM and validated again, and the
       next goal succeeded.
     - Fix: `ReceiptScheduler` holds such a revision until a covering clock. That is still a
       causal bound, and streams that never faulted replay unchanged. Grids are now published
       as int8 arrays; the 360k-cell mosaic was converted to a list every second.
   - **T3, RTK 10 s outage every 150 sim-s, 8 goals** (trial `bf7e4dc4`):
     - the outage hit mid-goal; the supervisor held motion at +1.98 s and the rover was still
       by +2.5 s;
     - the goal was cancelled, then `reset_map`, a GLIM restart, HEALTHY at +17.2 s;
     - the mission resumed;
     - 7 of 8 succeeded, 0 contacts, SLAM p95 / max 9.5 / 22 cm;
     - no `map_stale` with the fix.
   - **Bookkeeping budget.** GLIM's bookkeeping revisions carry the whole trajectory, and the
     8,192-frame limit would end a mission at ~14 min. Nav raises it to 32,768. Accept is linear:
     25 ms at 4,842 frames, 167 ms at 32,768.
   - **Exploration on v2: two fixes from the first seed-42 gate attempts.** Both attempts
     failed `coverage<0.7` with `no_candidates` and 0 contacts.
     - First attempt (`9ab3da64`): the manager returned home at once. Its reachable set (cells
       with a 0.75 m disk of known free, cost ≤ 35) was empty. A stationary rover's front LiDAR
       never sees the ground beside the body: 127 unknown and 93 bounded-unknown cells within
       ±0.8 m. The same happens after every supervisor map reset.
     - Fix 1: an opt-in reveal goal (`reveal_distance_m`, 1.5 m on v2; 0 keeps legacy
       unchanged). With nothing reachable, it sends a short goal ahead, then left, right, back;
       4 attempts, then `no_candidates` as before.
     - Second attempt (`fc3bb15b`): all four reveals succeeded and reachability stayed 0.
       Two causes:
       - the cost mosaic's latest-wins let the current blind ring's policy cells (70)
         overwrite ground observed on approach;
       - v2's 1 m display taper on observed cells (the legacy map has none) exceeds the
         manager's free threshold near rocks.
     - Fix 2: the mosaic keeps observations; policy cells fill only cells nothing has observed.
       The manager reads a separate class mosaic, `/terrain_shadow/exploration_map`
       (0 free / 100 occupied / −1 unknown, observations only), with the same semantics as the
       legacy BEV map. The cost mosaic remains the Nav2 global input.
     - Third attempt (`59cb3dc5`): explored after one reveal.
       - 38 goals, 36 succeeded; the manager ended `coverage_goal_met` (online 0.712).
       - Home within 0.76 m; 0 GT rock contacts; no localization holds.
       - Gate: **FAIL by 0.008**, GT coverage 0.692 < 0.70. The manager stops at its own
         0.70 estimate, which reads above GT here; this is the boundary that left the legacy
         stack at 2/3.
       - T4 data: the publisher stayed valid for the whole 636 sim-s, and RSS rose
         188 → 292 MiB.
       - Re-scored (Oct 27): the miss was the eval's reference, not the robot. The eval
         centred its coverage grid on the nominal home (20, 18); the rover started at GT
         (19.958, 17.976), and the manager centres on its own start. Anchored at the GT start,
         the same track covers 37/52 = 0.712, equal to the manager's figure (the 4.8 cm shift
         drops one corner-clipped cell). `phase4_exploration_eval.py --home-from-gt-start`
         (used by `v2_gate_run.sh`) anchors coverage and home distance there and still reports
         the nominal-home figures for comparison with legacy gates.
     - Seed 411 explore, first attempt (`79ee30dc`, Oct 27): `no_candidates` after 4 reveal
       goals, coverage 0.058, 0 contacts. The rover sat in an unknown patch the size of its
       body: the front LiDAR never sees the spawn footprint, so no start cell was 0.75 m-clear
       and reachable stayed 0 (stall snapshots: 3,410 free cells mapped, 0 reachable).
       Fix: the exploration mosaic takes unobserved cells under the unpadded body as free
       (the same `clear_footprint` the planner window uses); observed cells still win.
       Offline on those snapshots, reachable rises 0 → 192 and 0 → 734 cells.
     - Oct 27 setup note: the archive USB disk dropped on Oct 23 and re-mounted on Oct 26;
       the dev container kept the dead bind (EIO) until it was restarted.
     - Seed 411 explore, second attempt (`9d5cd98c`, body fix in): explored north and east
       (26 goals, GT coverage 0.50, 0 contacts), then the publisher faulted on a native
       chunk discontinuity at 0.36x sim speed. The supervisor held motion (`map_stale`),
       restarted GLIM and reset the map, which wiped the mosaic; the rover re-revealed beside a
       rock, two reveals aborted at once and the 6.4 m return failed. Fixes:
       - nav deployment tolerates a lossy stream (`LocalConfig.tolerate_stream_gaps`): a
         dropped or deadline-late chunk is skipped and counted (`skipped_sequences`,
         `late_chunks` in status); stale input pauses projection instead of faulting, and
         the supervisor's 1 s `map_stale` stays the outage gate. Identity changes and order
         regressions still fault; captures stay strict;
       - the chunk subscription queue is 64 deep (16 dropped after ~0.27 sim-s of stall).
     - Seed 7 explore, first attempt (`e2aa8d4b`): three problems.
       - World: the spawn sits over rock `instance_82` (0.12 m), so every seed-7 run starts in
         GT contact. `score_gt_contacts.py` now reports rocks under the body at the first
         sample as `spawn_rock_contacts`, outside `contacts_by_height`.
       - Eval: `reference_invalid` from a `future_stamp` on the first GT pose, received
         before the eval's first `/clock` (time 0). GT is now skipped until the clock runs.
         Bring-up also outlasted the manager's 35 s start delay, so the eval's first GT pose
         was already 0.35 m from spawn; `v2_gate_run.sh` now starts the eval before the
         mission.
       - Mission: no start cell had a clear 0.75 m disk (small unseen patches beside the
         rover), so it kept issuing blind reveals; one ended with the padded footprint on a
         rock's lethal cells, where Smac, MPPI and backup all refuse to move. Fix:
         `reachable_seed_distance_m` (v2 gates: 1.0) seeds reachability from clear cells
         reachable through known-free cells within that distance when the rover's own cell
         lacks clearance. The walk cannot cross obstacles or unknown cells.
     - Seed 411 explore, third attempt (`44ac0ac3`, Oct 28): after 3 reveals the seeded
       reachable set was a small pocket with no mission candidates. Reveals required
       reachable == 0, so the manager ended `no_candidates` at coverage 0.04 (0 contacts).
       Reveals now fire whenever there are no candidates (still 4 per stuck episode; the count
       resets only when candidates appear).
     - Seed 7 explore, second attempt (`40a2a712`): GT coverage 0.75 and 0 avoidable contacts
       (2,532 samples on the spawn rock reported apart), but the return failed. Home is the
       spawn, on top of `instance_82`; once that rock is mapped, Smac finds no valid path to the
       exact home pose and the mission fails on the first attempt. Fixes (v2 only):
       `home_approach_radius_m` (0.5) returns to the nearest known-free cell with 0.5 m
       obstacle clearance when home is closer than that to a mapped obstacle; `nav2_v2.yaml`
       `xy_goal_tolerance` 0.65 → 0.35 so that a return success still lands within the
       manager's 0.9 m `home_reached_radius_m` (≤ 0.85 m).
     - Seed 411 explore, fourth attempt (`a789ef5b`): 8 of 16 explore goals failed with the
       0.35 tolerance on every goal (the 0.65 attempt failed 3 of 26), and at 835 sim-s the
       publisher hit its 2 GiB archive quota: each `projections.jsonl` record carried the
       whole 60 s association list (~270 KB). Map reset was then refused, the restart budget
       ran out and the mission failed `localization_fault` (coverage 0.29, 0 contacts). Fixes:
       - without the raw archive, projection records keep `association_count` instead of the
         list (nothing can be replayed without `native.bin`), cutting JSON work on the
         callback thread too;
       - two goal checkers: `general_goal_checker` 0.65 (legacy) for every goal and
         `precise_goal_checker` 0.35 for `return_home` only, through BTs that name the
         checker (`behavior_trees/navigate_to_pose_v2*.xml`, `navigate_through_poses_v2.xml`)
         and the manager's `return_behavior_tree`. The first try copied Jazzy's stock BTs, but
         phase3 overrides the BT with `navigate_to_pose_lunar_recovery.xml`, so every goal
         failed at once (`FollowPath` with no checker; run `f9a1e6ec`). The lunar BT now names
         `general_goal_checker` (legacy configs' only checker) and
         `navigate_to_pose_lunar_recovery_precise.xml` is the return tree.
     - Seed 7 explore, third attempt (interim config, stopped at ~63 min): a `map_stale` hold
       mid-mission. The supervisor's `_map` cleared the last fresh map time on any rejected
       message, so one map older than 0.75 sim-s on arrival (a publisher load spike) read as
       stale and restarted GLIM. On the restart the publisher faulted its freshly reset core
       with `multiple v2 input authorities` (old and new GLIM both briefly in discovery), so
       the rebuild hit `restart_timeout`. Fixes: `_map` keeps the newest valid map stamp (a
       rejected message only clears `map_ready`); in nav a second revisions publisher is
       tolerated for 10 s (`DUPLICATE_AUTHORITY_GRACE_S`), as only the reset epoch is
       accepted, and still faults if it persists.
     - Seed 411 explore, final config (`9eee12ee`): clean mission (39 goals, 3 failed,
       `coverage_goal_met`, home 0.23 m, 0 contacts, no faults), but GT coverage 0.692 vs the
       manager's 0.712: again one corner-clipped cell (36 vs 37 of 52) at the stop, now with
       both grids anchored at the spawn. The online count marked a cell on any pose inside
       it, so SLAM poses a few cm from GT decide edge cells. Fix: `mission_visit_margin_m`
       (v2 gates 0.15 m, above SLAM p95 6-10 cm) counts a cell only when the pose is that far
       inside it, so the online estimate reads at or below GT; the 0.70 stop is unchanged.
     - Seed 42 explore, final config with the visit margin (`b6f88195`): **PASS**. GT coverage
       0.769 (online 0.712, now below GT as intended), home 0.43 m, 0 contacts.
     - Seed 411 explore, same config (`3b95aef9`): **safety failure**, 701 GT contact samples
       with `instance_0` (0.34 m). After the explore goals around (34, 37) failed, reveal goal 24
       (north) started with the rover facing south; MPPI (`vx_min` -0.05) reversed while
       turning and backed 0.4 m into the rock, then every recovery reported `Collision Ahead`
       and the return failed. The rock was in the global mosaic (lethal since t = 142 s) but
       not in MPPI's local costmap: that was the live v2 window, which drops an obstacle 60 s
       after its last observation, and the front LiDAR never sees behind the rover, so the rock
       had decayed to freed-unknown (cost 70). Fix: in nav the published
       `/terrain_shadow/planner_costmap` is the mosaic cropped to the window
       (`GlobalStitch.window`): current observations as before, earlier observations
       remembered, policy only where nothing was ever observed. (Merging the mosaic as a second
       Nav2 StaticLayer was rejected: in a rolling window its `use_maximum` merge is, as far as I
       recall the Jazzy source, a plain max in which unknown (255) beats lethal.)
     - Seed 7 explore, final4 attempt (`6bb47eb5`): aborted by a bash syntax error. I rewrote
       `v2_gate_run.sh` in place while it was running (bash reads scripts incrementally); scripts
       are now replaced atomically.
     - Seed 411 explore with the mosaic-backed local window (`9da61594`): **PASS**. GT
       coverage 0.788 (online 0.712), home 0.31 m, 0 contacts, 980 sim-s.
     - Seed 7 explore, final config (`a0cb8c50`): **PASS**. GT coverage 0.731 (online 0.712),
       home 0.69 m via the home approach (goal moved 0.36 m off the spawn rock), 0 avoidable
       contacts (2,041 samples on the spawn rock reported apart).
     - **Gate summary (final code, v2 stack, legacy mission parameters): 6/6 PASS.**

       | Gate | Run | Date | Result | Key numbers |
       |---|---|---|---|---|
       | 42 explore | `6df0bdb9` | Oct 29 | PASS | GT cov 0.769 (online 0.712), home 0.55 m |
       | 411 explore | `9da61594` | Oct 28 | PASS | GT cov 0.788, home 0.31 m |
       | 7 explore | `a0cb8c50` | Oct 28 | PASS | GT cov 0.731, home 0.69 m |
       | 42 sample_return | `554286f9` | Oct 29 | PASS | sample 0.758, home 0.44 m |
       | 411 sample_return | `2f9cfff7` | Oct 29 | PASS | sample 0.756, home 0.38 m |
       | 7 sample_return | `f1dab517` | Oct 29 | PASS | sample 0.777, home 0.79 m (2,629 spawn-rock samples apart) |

       All six with 0 avoidable GT contacts at >= 0.10 m, one code version (the Oct 29 runs
       differ only by the nav-run archive reserve below, which does not touch behaviour).
       Sample values just above 0.75 are the stop rule, not a margin: the manager returns home
       at the first sample >= `sample_success_threshold` 0.75. Superseded earlier passes on
       interim configs: 42 explore `b6f88195`, sample_return `a2a9bc08`, `050fad92`, `4c1f4219`.
     - Archive reserve (Oct 29): nav runs write MBs (no raw archive), but the default 50 GiB
       free-space reserve refused every allocation once other data on the disk left 41 GiB.
       The three nav-only allocations (gate run, sim nav stream, nav publisher) now keep a
       20 GiB reserve; capture paths keep the default.
     - Gates after these fixes are re-run on the final code (`tools/v2_gate_run.sh`, same
       mission parameters); earlier attempts above are kept as failed artifacts.
   - Not yet run: the T2 audit-suite replay (needs `terrain_archive_raw:=true`); T4/T5 below.

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
