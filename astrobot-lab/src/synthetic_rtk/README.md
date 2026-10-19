# RTK / absolute-position fusion for GLIM

Fuses an **absolute position + heading** source into GLIM's LiDAR-IMU odometry to bound drift and
arrest catastrophic divergence in feature-poor lunar terrain (GLIM loop closure is disabled, so there
is otherwise no global anchor). On the Moon there is no GNSS — this is framed as a **surveyed surface
beacon / lander base station** ("RTK from home"), credible mission hardware (LunaNet / surface beacons),
not literal GPS satellites.

**On by default** in `phase2_slam` / `phase3_nav` / `phase4_exploration` launches.

## Architecture (additive — GLIM core is stock, no fork edits)

```
/gt/base_link_pose ──> synthetic_rtk ──> /rtk/pose ──> abs_constraint_module (in GLIM) ──> GLIM smoother
   (sim ground truth)   (this pkg)      (PoseWithCov)   (glim_ros2 fork, additive)        GPSFactor + yaw prior
```

- **`synthetic_rtk`** (this package): the *sim* RTK source. Subscribes `/gt/base_link_pose`, applies an
  antenna lever-arm, baseline-scaled noise `σ(d)=σ0+k·d`, optional dropout/latency, and publishes
  `/rtk/pose` (`PoseWithCovarianceStamped`, **home-relative world frame**, position + heading with
  covariance). Republishes at **30 Hz with current-time stamps**. Publishes a measurement topic only —
  **never a TF**. *On real hardware, replace this node with the real RTK/beacon driver publishing the
  same `/rtk/pose` contract; nothing downstream changes.*
- **`abs_constraint_module`** (`glim_ros2/src/glim_ros/`, an `ExtensionModuleROS2`, additive): subscribes
  `/rtk/pose`; on `OdometryEstimationCallbacks::on_smoother_update` it appends to the live pose key
  `X(current)`, matched by timestamp: a position-only `gtsam::GPSFactor` + a yaw-only `PriorFactor`
  (heading). Loaded via `config_ros.json` `extension_modules: [..., "libabs_constraint_module.so"]`.
- **Frame pinning**: `config_odometry_cpu.json` `init_T_world_imu` pins GLIM's map frame to the rover
  spawn so the home-relative `/rtk/pose` is a *direct* world prior (base_link at map origin;
  `(0,0,1,0)` = yaw π for the base→IMU mount).

## "Full authority" — precise meaning (this trips people up)

`gate_dist=0` + `huber_k=0`. This **only** means the RTK factor is **never gated out and never
robust-down-weighted** — it always enters the graph at its configured covariance. It does **NOT** mean
RTK outweighs the scan match. The relative weight is set by the noise models, untouched by the gate/Huber:

| factor | σ | information (Precision) |
|---|---|---|
| RTK position | ≈0.05 m (near home) | ≈ **400** |
| scan-match prior | hardcoded `Isotropic::Precision(6, 1e3)` | **1000** |

So **per frame the scan prior is ~2.5× stronger than RTK** — RTK does not dominate locally. RTK bounds
drift because:
1. it is an **absolute anchor on every pose**, and those priors **accumulate** to pin global position,
   while the scan prior is effectively **relative** (it anchors each pose to the GICP match against the
   *local map, which drifts with the robot* — it pins local shape, not global position); and
2. in catastrophic divergence it **breaks the feedback loop** — keeping the pose near truth keeps the
   local map near truth, so GICP keeps matching and stops running away. It is not a per-frame tug-of-war.

To make RTK *dominate per-frame*, you would lower the scan prior's `Precision(6, 1e3)`
(the GICP-overconfidence lever), **not** the gate/Huber.

## Validated (offline replay + live long drive)

Fixture: `rosbags/seed411_sparse_explore` (a real catastrophic feature-poor GICP divergence, reproducible
in replay). A/B via `tools/rtk_replay_ab.sh`:

- **Catastrophic divergence**: ISOTROPIC no-RTK = **9480 m** runaway → ISO + RTK (full authority) = **~1 m**. Arrested.
- **Long autonomous drive** (~18 min, normal rocks, RTK continuous): GT-SLAM error **bounded ~0.3 m** the
  whole session, vs **2.5 m unbounded** when RTK had dropped out of the graph.
- **CPU contention disproven** as the cause: the same bag replayed with no contention diverged *worse* →
  the failure is feature-poor GICP geometry, not contention.
- Honest-covariance (data-driven GICP Hessian) was explored and **reverted** — it helps without RTK
  (9480→480 m) but is unnecessary once RTK is fused, and made the well-conditioned case slightly worse.

## The "RTK not getting into the graph" fix

RTK was intermittently starving out of the graph (`no_match` climbing, buffer draining → drift grew):
the timestamp association (`assoc_tolerance=0.05 s`) was too tight as SLAM-frame vs RTK stamps drift
apart. **Fix**: `assoc_tolerance` → **0.2 s** (primary) + synthetic_rtk **30 Hz republish** with
current-time stamps (`publish_rate_hz`, secondary). After: `no_match=0`, continuous fusion.

## Config knobs

`config_ros.json` → `abs_constraint`:
`enable` (true), `rtk_topic` (/rtk/pose), `assoc_tolerance` (0.2), `gate_dist` (0), `huber_k` (0),
`sigma_floor` (0.05), `use_msg_covariance` (true), `use_yaw` (true), `yaw_huber_k` (0),
`yaw_sigma_floor` (0.01), `yaw_cov_threshold` (1.0).

`config_odometry_cpu.json` → `init_T_world_imu` (frame pinning).

`synthetic_rtk` (set via `phase2_slam` launch args `rtk_sigma0` / `rtk_sigma_per_m` / `rtk_dropout_prob`
/ `rtk_latency_s`, plus node params): `home_xy`, `antenna_offset_xyz`, `sigma0`, `sigma_per_m`,
`sigma_yaw`, `dropout_prob`, `latency_s`, `publish_rate_hz` (30).

**Disable RTK**: `start_synthetic_rtk:=false` (launch) or `abs_constraint.enable=false` (config).

## Production caveats

- `gate_dist=0` / `huber_k=0` give **no protection against a genuinely bad RTK fix**. For real hardware,
  add a **kinematic (RTK-to-RTK) plausibility gate** (reject a fix that implies impossible motion vs the
  previous fix) before trusting RTK blindly — do not ship gate/Huber off against a real receiver.
- `synthetic_rtk` uses ground truth as the *sim* RTK stand-in; swap for the real beacon driver on hardware.
- The teleop_click `_slam_diverged` watchdog reads the `map→odom` offset — only meaningful from a **fresh
  sim+GLIM start** (a mid-session GLIM restart leaves stale odom → false "diverged").
- RTK currently fuses at the **odometry** level (per-frame). Fusing it additionally as a low-rate
  **global-mapping anchor** (and/or multi-robot range/relative constraints) is the natural extension —
  the `abs_constraint` module is structured for pluggable providers.

## Testing

```bash
# Offline A/B (no sim): replay raw LiDAR+IMU+GT through GLIM + synthetic_rtk, score vs GT.
tools/rtk_replay_ab.sh /workspace/astrobot-lab/rosbags/seed411_sparse_explore <config_path> <name>
# Toggle RTK with abs_constraint.enable; the fixture diverges deterministically with RTK off.
```
