# astrobot_sim

Simulation-only nodes driven by ground truth. None of them belongs in a hardware deployment.

| Node | Role |
|---|---|
| `synthetic_rtk` | RTK / surface-beacon stand-in: noisy absolute position + heading fixes from `/gt/base_link_pose` |
| `gt_diagnostics` | Ground-truth navigation diagnostics for the Ops console (`diagnostics`, JSON) |

Both express ground truth in the RTK-anchored map frame, map = world - home, with `home_xy`
(-20, -20) from `astrobot_bringup/config/sim.yaml`.

## synthetic_rtk

On the Moon there is no GNSS: the absolute source is modelled as a surveyed beacon or lander base
station ("RTK from home"). The node publishes `/rtk/pose` (`PoseWithCovarianceStamped`, map frame,
never a TF) at the ground-truth rate:

- position of the antenna (`antenna_offset_xyz`, at the IMU so the fix matches GLIM's IMU pose)
  with baseline-scaled noise `sigma(d) = sigma0 + sigma_per_m * d` (0.02 m + 1 mm/m; vertical
  x `sigma_z_scale`);
- heading in the IMU frame (`imu_yaw_offset` pi) with `sigma_yaw` (a GNSS-compass model);
- fault injection: `dropout_prob`, periodic occlusion (`occlusion_period_s`,
  `occlusion_duration_s`), latency (`latency_s`, `latency_jitter_s`).

On hardware, replace it with the beacon driver publishing the same contract. Downstream, the
localization supervisor gates each fix (per-axis innovation against SLAM) and republishes the
accepted ones on `localization/absolute_pose`, which the GLIM extension
`libastrobot_abs_constraint_module.so` (astrobot_glim_extensions) adds to the odometry smoother:
a position-only `GPSFactor` and a yaw-only prior on the live pose key, matched by timestamp.
The fault-injection run in `docs/running.md` exercises the outage path.

### How strongly RTK constrains GLIM

`abs_constraint` in `astrobot_bringup/config/glim/config_ros.json` sets `gate_dist: 0` and
`huber_k: 0`: a fix is never gated or down-weighted inside GLIM (the supervisor does the gating).
That does not make RTK dominate a frame: with `sigma_floor` 0.05 m its information is about 400
against 1000 for GLIM's scan-match prior. RTK bounds drift because it is an absolute prior on every
pose (the scan prior only ties each pose to a local map that drifts with the rover), and in a
feature-poor divergence it keeps the pose near truth, so scan matching recovers instead of running
away. To make RTK stronger per frame, lower the scan prior's precision, not the gate.

## gt_diagnostics

Every `period_sec` it compares the SLAM pose (converted from GLIM's IMU frame to base_link) with
ground truth, checks the Nav2 plan and the selected goal against the GT obstacle map and the Nav2
costmaps, tracks Nav2 progress, and publishes a JSON record with a classification (`primary`,
`reasons`, `confidence`), e.g. `slam_pose_error_drifting`,
`planned_path_intersects_gt_obstacle_corridor`, `nav2_feedback_no_distance_progress`. The console
shows the classification and the goal-distance series. It never feeds back into the stack.
