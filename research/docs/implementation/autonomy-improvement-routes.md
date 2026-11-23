# Next improvement routes and gates

These are follow-on experiments, not claims that the present system is mission-ready.
See [implementation status](autonomy-repair-status.md) for completed changes and actual evidence.

The approved revision-aware SLAM/terrain execution sequence and latest frame,
archive and simulator evidence are tracked in
[revision-aware terrain status](revision-aware-terrain-status.md). Its revision
consumer/installation barriers precede optimization or terrain promotion below.

## 1. Establish supported terrain before autonomous motion

The stationary seed-42 fixture exposes sparse ground samples and a roughly meter-scale blind region near the chassis. Strictly requiring a fully observed 0.75 m-radius disk produces zero candidate centers. The old air-ray/free-space and nearest-start behavior could conceal this condition. Increasing the coverage metric or bypassing planning does not add sensor evidence.

Work in this order:

1. Stationary repeatability is complete for both baseline and full-SE(3) GT-isolated replay (184/184 matching frame hashes). Add representative approach/turn fixtures and GT obstacle labels/recall scoring before tuning thresholds.
2. Separate measured ground support, inferred ground support, obstacle evidence, and unknown terrain in diagnostics. Report support as a function of distance/bearing from the rover and report obstacle recall separately. Do not use overall known-cell count as the support metric.
3. Try bounded interpolation only between consistent measured ground samples: small spatial gaps, bounded plane residual/slope uncertainty, explicit evidence age, and no interpolation across height discontinuities. Compare 5/8/10/14/20/25 cm objects, sparse/noisy flat terrain, ramps, dropoffs, and wide obstacles. The large chassis blind region must remain unknown unless another justified observation covers it.
4. Evaluate near-field depth fusion or a sensor mounting/coverage change for the large blind region. A contact-supported starting footprint is a separate assumption requiring wheel-contact evidence and a collision-checked observed egress path; do not mark an arbitrary radius free.
5. Replace the circular candidate filter with geometry shared with Nav2. Distinguish whole-body obstacle clearance from wheel-track ground support. Verify oriented narrow corridors, boundary cells, blocked starts, and disconnected free regions without snapping through obstacles.
6. Validate the implemented, currently disabled obstacle-only near-field memory on approach/turn fixtures and at live projection cadence. Core ray-over-rock, expiry, reset, and evidence-age fixtures pass. Require no fabricated ground support and no evidence-age renewal from publishing or ground-only hits. Keep the old unconditional occupied-voxel latch disabled.

Promotion gate: a measured/supported spawn-to-open-ground route, no falsely free unknown/dropoff fixture, positive soft cost at 5 cm, lethal confirmed obstacles at >=8 cm, and repeatable results at multiple cloud replay rates. Then run the short obstacle course three times before any soak test.

## 2. Reduce SLAM failures, not just their consequences

The implemented supervisor is fault containment, not an accuracy improvement claim. Stationary child-exit and absolute-sensor-outage recovery worked; moving recovery and terrain degeneracy remain unvalidated.

1. Add moving replay injections: NaN and finite pose jumps, delayed/reordered/duplicate positioning, bad covariance, heading loss, gravity disagreement, mapper stalls/capacity exhaustion, and estimator exceptions. Assert stop timing, no rejected clouds crossing the map epoch, bounded attempts, and preserved world/home/mission history.
2. Compare covariance-aware assisted GLIM on identical bags with short and long outages. Report spontaneous estimator faults separately from deliberately injected failures. Include position/heading error, accepted/rejected factors, lag, reset count, and time without localization—not merely final mission success.
3. Instrument factor residuals/conditioning and alignment quality before changing precision values. Sweep a small bounded set of LiDAR/IMU/absolute-factor weights; avoid selecting a seed or retry that happens to survive.
   Include pitched/rolled motion: the current heading prior uses a weakly constrained Pose3 tangent-space rotation, which is only an approximation to a pure global-yaw measurement away from level attitude. Validate its residual/Jacobian or replace it with a dedicated heading factor before interpreting heading-weight sweeps on slopes.
4. If the estimator remains fragile, compare a simpler assisted architecture offline: continuous local odometry with an independently estimated global position/heading correction, rather than feeding every absolute update into the same scan-matching smoother. Test discontinuity handling and outages explicitly. This is a candidate experiment, not a validated replacement.

Promotion gate: finite continuous local motion, bounded error through the specified outage, no spontaneous reset on the short courses, and no lost map/mission epoch synchronization. Only then claim baseline SLAM stability.

## 3. Planning and control around obstacles

Keep one global home goal and one owner of local backup/replanning. Preserve rejected-action and away-from-home success fixtures. Do not restore breadcrumb execution or precheck bypass as the default escape mechanism.

1. Complete shared footprint/support semantics and expose why a candidate/path was rejected.
2. Test straight clearance, narrow passages, concave obstacles, reverse escape, blocked home, rejected/cancelled actions, and backup that temporarily increases goal distance.
3. Compare current 2D Smac + MPPI against an orientation-aware planner only if fixtures show footprint/heading-feasibility failures. Compare controller alternatives only when the global path is demonstrably feasible; otherwise controller tuning confounds mapping/planning faults.
4. Profile planning/controller timing, path length actually driven, minimum clearance, progress, and recovery time in simulation seconds. Do not interpret a changing remaining plan length as executed-path efficiency.

Promotion gate: real home completion, no colliding/unknown-space shortcut, bounded cancellation/recovery, and repeatable course completion.

## 4. Performance and final evaluation

- Stationary integration was approximately 22–30 ms at ~100k voxels; do not extrapolate this to a 6M-voxel mission.
- Profile integration, decay, projection, Python cloud relay, reachability, planning, and control separately. The full-voxel decay/projection scans remain candidates for lazy timestamp decay, active spatial tiles, or a rolling local map plus a coarser persistent global map.
- Capacity exhaustion now causes a hold rather than silently admitting incomplete maps. A bounded-memory admission/eviction policy still needs its own endpoint-retention and replay tests.
- Require manifests with resolved binaries/configuration, every attempt's result, input validity, received/processed/dropped frame counts, and runtime timing.
- After the short-course gates pass, run seeds 42, 411, and 7 for 20 simulation minutes each. All first attempts count. A successful retry or successful automatic restart must not erase a spontaneous failure.
