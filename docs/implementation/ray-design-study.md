# Single-LiDAR ray design — September 27, 2026

## Selected setup: local/mid-range assessment

The user selected **mid-dense +10..−60 with 0° mounting pitch**. The checked
development contract is [selected_lidar.json](../../tools/local_terrain/selected_lidar.json).
Keep the original [0.4, 0, 0.25] m mounting offset, 32 rings, 1999 columns, 10 Hz
and 0.5–100 m range. No further beam-layout selection is part of this stage.
This does not silently reconfigure the old uniform-LiDAR mission adapter.

Native Isaac data now passes a local/mid-range development screen using the
unchanged terrain mapper. Range bands are relative to the original base origin;
unknown cells remain in the denominator, with all four grid phases retained.

| Flat-ground supported free cells | Six small XY viewpoints | 24 yaw viewpoints |
| --- | --- | --- |
| 0–1 m, all bearings | 91.59–93.67% | 100% |
| 1–2 m, all bearings | 94.36–96.70% | 100% |
| 2–3 m, all bearings | 30.24–31.71% | 97.20–98.08% |
| Eight-box occupied detections | 7/8 | 8/8 |

The yaw acquisition uses 24 fixed **sensor-only** placements (0..345° by 15°),
rotating the mounting lever arm as well as orientation. It is increased spatial
diversity and acquisition dose, not a classifier improvement or a physical robot
spin. No rover body, slopes, holes, continuous motion or SLAM error is present.

Across **2,304 yaw-sequence maps**, no hazard-overlapping cell is free. With boxes,
ground support is lower: **91.63–95.78% at 1–2 m**, **93.24–93.91% at 2–3 m**.
Those conservative boundary losses stay counted. At most four free-ground cells
are falsely occupied; the flat scenes have none. The >=95% flat 1–2/2–3 m screen
and eight-box checks pass, but this does not establish >=95% independent safe-motion
availability. Actual self-occlusion and the safe observation maneuver remain open.

Evidence: native `revision-79dabea8584946f48f04bf07fdc5ce44` (48/48 geometry cases);
terrain `revision-58aacf438269495e948e2efd2d04c9e2`. The implementation ledger
tracks the subsequent GLIM input-contract integration and retained build failures.

### First GLIM integration step

[generic_lidar_adapter.py](../../tools/generic_lidar_adapter.py) now assembles six
complete native ticks into a timed sensor-local cloud, preserving tick timestamps
and beam identities. Missing/repeated beams, clock/sequence gaps, partial windows
and post-finish input fail closed. Raw terrain input remains separate and unchanged.

The existing GLIM global-shutter setting would collapse all point times. The
opt-in copied profile disables it; no production configuration is edited.
Installed ROS conversion, TimeKeeper and preprocessing pass **96/96 windows**
(4,512,944 valid returns), with exact sensor-local coordinate/time preservation at
conversion and preserved acquisition extent through preprocessing. The old-shutter
negative control rejects collapsed times as expected. Final evidence:
`revision-e7822661f63d48ac896866a872d8cb09`.

This is **input integration, not odometry accuracy**: the probe does not run IMU
fusion, ROS publication or motion estimation. Next live stage requires the actual
rover's self-occlusion and level sensor/IMU/base TF audit, synchronized physical
IMU/native acquisition, then stationary and guarded moving GLIM/terrain checks.
Do not use the historical five-degree uniform-sensor collector as the new profile.

## Correction: the requested mount is level (0°)

The first experiment below incorrectly retained the old +5° mounting pitch.
Those results are historical, **not the requested level-mount comparison**.
The current screen explicitly sets sensor rotation to identity: 0° pitch, roll
and yaw, keeping the original translation and the same eleven ray tables.
Beam elevations now match base-relative elevations at every azimuth.
Recorded calibration stays unchanged for provenance; it is not silently reused
as the experimental mount. All denominators, ranges and obstacle banks stay fixed.

Level-mount screen: `revision-4fa87065a0014efda74d266bc46b946a`. The symmetric
ring-pattern control is also evaluated at **0°**, not at its old 5° pose.

| Layout at 0° mounting pitch | First 0–0.25 m band | 0.25–0.5 m band | 0.5–1 m band | Original / shifted placements with rays, each /128 |
| --- | --- | --- | --- | --- |
| Symmetric original ring pattern | 0% | 0% | 0% | 116 / 120 |
| Uniform 0..-40.3 | 0–6% | 33–50% | 96–100% | 127 / 124 |
| Uniform 0..-45 | 0–50% | 62–83% | 100% | 125 / 126 |
| Uniform +10..-45 | 0–50% | 62–83% | 100% | 121 / 122 |
| Uniform 0..-60 | 50–81% | 88–100% | 100% | 117 / 116 |
| Uniform +10..-60 | 50–88% | 88–100% | 100% | 111 / 112 |
| Uniform 0..-75 | 50–88% | 88–100% | 100% | 115 / 118 |
| Uniform +10..-75 | 50–81% | 88–100% | 100% | 104 / 106 |
| Mid-dense +10..-60 | 50–100% | 88–100% | 100% | 127 / 128 |
| Steep-dense +10..-60 | 50–88% | 88–100% | 100% | 122 / 122 |
| Shallow-down-dense +10..-60 (`far_dense`) | 50–88% | 88–100% | 100% | 127 / 128 |

These are geometric hit-cell/any-ray counts, not drivable coverage or detector
recall. Both nonuniform mid-dense and shallow-down-dense layouts are promising
at zero pitch; the earlier tilted ranking must not select the level candidate.
The ray count remains 32×1999 and both nonuniform candidates lose 6.25% of rays
below minimum range on the ideal flat plane. All self-occlusion, SLAM, runtime
and moving-terrain limitations below still apply.

Corrected native validation passes **22/22 cases**, **2,814,504 samples** with
equal dose, all intended beam identities, and zero hit-mask discrepancies.
Every actual USD sensor mounting rotation is **exactly identity**. All native
flat-scene metrics match the level screen. Maximum depth/XYZ errors are
**2.01/2.07 µm**, below unchanged 2 mm gates. Final host suite: **89 passed**;
formatter and whitespace checks pass.
Launcher: `revision-ab46b21ad95e46b6b902d8a08599c192`;
inner: `revision-7fe679f34b61447bbf4b22e794ead00c`.

The current reproduction commands below now run this **level** protocol, not
the old +5° protocol. Historical runs retain their original archived sources.

## Quick far-structure check and actual terrain classification

The selected development candidate is now **mid-dense +10..−60 at 0° mount**:
five rings +10..0, twenty rings −2..−30, seven rings −35..−60. Production sensor
configuration has not been changed. The custom Generic sensor adapter remains
an isolated experiment, not the current mission native transport.

The installed GLIM random-grid sampler retains useful distant structure with this
layout. Its **20,000-point target overrides the configured 0.1 sampling rate**;
25 cm random-grid sampling does not automatically discard nine of ten sparse
distant returns. Analytical 3×3×2 m boxes, eight bearings, two angular phases and
five fixed seeds all pass the predeclared retention/extent screen (640 cases).

| Range | Mid-dense object points after sampling | Original level pattern |
| --- | --- | --- |
| 10 m | 630–783 | 880–1,291 |
| 20 m | 98–163 | 245–345 |
| 30 m | 64–92 | 96–153 |
| 40 m | 23–34 | 46–68 |

At 40 m the candidate has at least twelve occupied 25 cm object voxels but only
one horizontal height band. It sees large distant structure, with less vertical
constraint than the original pattern. This is a no-noise geometry/downsampling
screen, **not a measured GLIM trajectory error or full registration test**.
Evidence: `revision-6197671236ec47bbb2ec6446884ce46f`.

Replaying actual native Isaac rays through the unchanged C++ terrain classifier
exposes the difference between *hitting a cell* and *supporting its ground area*:
stationary mid-dense covers 8.28–9.81% of the entire 6×6 m flat grid as traversable,
versus 0.28–2.31% for the original level pattern. These are short stationary
two-revolution acquisitions, not comparable to the earlier long-route ~95% result.
Most missing support is either no returns or a thin ring with insufficient area.
The stationary candidate is much better locally than that whole-grid number:
the fixed forward wheel strips at 0–0.25 / 0.25–0.5 / 0.5–1 m ahead of the axle
have **0–25% / 37.5–66.7% / 79.2–91.7%** supported free cells across grid phases.
The near strip still includes chassis-covered regions in the real rover; this
no-rover fixture cannot establish their actual optical visibility or safe motion.

Both patterns classify six of eight 10×10×8 cm boxes as occupied at all four grid
phases. Two rear-diagonal boxes expose only ~4–5 cm of their lower faces and remain
unknown. **No hazard-overlapping cell becomes traversable in any of 192 maps**;
one free-ground cell is falsely occupied for mid-dense at phase 0.05. The safety
guard is behaving conservatively; lowering the obstacle threshold would not solve
the missing ground support. Corrected evidence:
`revision-40571439e1fb405c96c8c6cc3b36f219`. Earlier evaluator run
`revision-2bab210fc9694e5c9a13f092baae4c7b` omitted traversable cost 20 and is rejected.

Six settled sensor-only views within a 20×10 cm XY rectangle now confirm that
spatial diversity helps, with unchanged mapper thresholds. On flat ground:

| Fixed-reference region | Original level pattern, six views | Mid-dense, six views |
| --- | --- | --- |
| Entire 6×6 m grid | 42.50–43.72% | 46.53–47.97% |
| First 0–0.25 m forward wheel strip | 0% | 41.7–81.3% |
| 0.25–0.5 m forward wheel strip | 0% | 56.3–83.3% |
| 0.5–1 m forward wheel strip | 0% | 100% |
| Eight-box scene: occupied objects | 8/8 | 7/8 |

No false-free cells occur across **1,152 publications**. The remaining mid-dense
box exposes at most 5.93 cm height, so remains unknown. The original shallow-ring
pattern has better small-object detection here despite its poor near-ground
coverage: mid-dense is a tradeoff, not a universally superior layout. With boxes,
the candidate's far forward strip is 87.5–100%; at most two ground cells are falsely
occupied. Coverage still depends on grid phase. These results use the original
fixed reference, not the final sensor-relative footprint or swept-motion scoring.

This is not a robot maneuver or safe-motion authorization. The first native run
failed on a PhysX convex-box grazing hit 4.88 µm outside the box. It remains
unscored. A fresh full run with triangle-mesh boxes at the same bounds passes
all original strict geometry gates, 24/24 cases, without deleting any return or
changing views/thresholds. See the ledger for the retained failure.
Native evidence: `revision-959b889fa1d346e7a3c720cda16beebc`;
terrain: `revision-f949513c4bc1477487bfd185e23636ba`.

The later user selection above supersedes further layout comparisons. Preserve
conservative unknown handling. Live custom-ray adapter, actual self-occlusion and
guarded acquisition checks remain before moving SLAM/terrain qualification.

## Historical +5° result and recommendation (superseded for level mounting)

**Redistributing the existing rays is substantially more promising than merely
tilting the old symmetric pattern.** The leading development candidate is
`far_dense_10_to_-60`: 32 nonuniform elevations from +10° to -60°, with more
shallow-down rings. Keep `uniform_10_to_-45` as a simpler comparator and the old
pattern as the frozen control. These are experimental designs, not deployed
sensor settings or a terrain/SLAM qualification.

One LiDAR remains active. The mount stays at its measured +5° pitch and
[0.4, 0, 0.25] m base offset. Every design retains **32 rings × 1999 columns =
63,968 rays/revolution**, 360° horizontal coverage, 10 Hz nominal rotation and
0.5/100 m range limits. “Same FOV” is interpreted as unchanged horizontal FOV;
vertical endpoints/span are explicitly changed. The 0°..-40.3° control also
preserves the original vertical span.

All elevations here are **sensor-local, positive upward**. The unchanged tilted
mount makes body-frame elevation azimuth-dependent (approximately ±5° further
variation). It is not equivalent to pitching a symmetric scanner down: the new
low rings look down around the whole azimuth, not only ahead of the robot.

## Comparison

Stationary flat-plane percentages count grid cells with **at least one return**,
not full-cell support or drivable terrain. Ranges span four fixed 10 cm grid
phases. Band distances are ahead of the front axle, not the front bumper.

The obstacle columns count placements receiving at least one ray, **not classifier
detections**. Each bank has 128 independent 10×10×8 cm boxes: eight ranges × sixteen
bearings. The shifted bank adds 0.10 m to every range and 11.25° to every bearing.
These are development checks, not sealed held-out acceptance.

| Layout (degrees) | First 0–0.25 m band | 0.25–0.5 m band | 0.5–1 m band | Original / shifted boxes hit, each /128 | Rays below min range |
| --- | --- | --- | --- | --- | --- |
| Original ≈+20..-20 | 0% | 0% | 25–33% | 116 / 118 | 0% |
| Uniform 0..-40.3 | 0–12% | 62–83% | 100% | 126 / 128 | 0% |
| Uniform 0..-45 | 17–62% | 88–100% | 100% | 124 / 126 | 0% |
| Uniform +10..-45 | 17–62% | 88–100% | 100% | 125 / 124 | 0% |
| Uniform 0..-60 | 50–88% | 88–100% | 100% | 119 / 122 | 13.95% |
| Uniform +10..-60 | 50–88% | 88–100% | 100% | 120 / 112 | 12.24% |
| Uniform 0..-75 | 50–100% | 88–100% | 100% | 115 / 118 | 30.92% |
| Uniform +10..-75 | 50–100% | 88–100% | 100% | 114 / 104 | 27.42% |
| Mid-dense +10..-60 | 50–81% | 88–100% | 100% | 123 / 128 | 7.22% |
| Steep-dense +10..-60 | 50–88% | 88–100% | 100% | 117 / 112 | 14.28% |
| **Shallow-down-dense +10..-60** (`far_dense`) | **50–88%** | **88–100%** | **100%** | **126 / 128** | **6.47%** |

All designs hit all eight boxes in the earlier fixed eight-object fixture. The
larger banks expose misses that this easier fixture conceals. For example, the
leading candidate misses 2/16 placements at 4 m in the original bank, where the
baseline hits 16/16; at 4.1 m shifted it hits 16/16 versus baseline 14/16. Gains
are therefore not uniform across range/phase, and there is no “all obstacles
detected” qualification claim.

## Why the leading layout helps

Its fixed 32 elevations are generated as:

- 3 rings: +10°, +5°, 0°;
- 18 evenly spaced rings from -2° to -20°;
- 6 evenly spaced rings from -22° to -37°;
- 5 evenly spaced rings from -41° to -60°.

The dense shallow-down segment improves distant small-object sampling compared
with uniformly spreading all rings over 70–85°. A small steep segment supplies
near-ground opportunities. Eleven rings lie within ±10° of the sensor horizon,
versus sixteen in the original design. This is a feature-allocation proxy only:
**GLIM accuracy with the new distribution has not been measured**.

At the measured 0.394923 m sensor height, the unchanged 0.5 m minimum range leaves
an approximately **0.307 m-radius ground disk** around the sensor unobservable.
Going steeper does not eliminate that disk; many -60°/-75° rays hit ground before
the minimum range. The first front-axle band is also mostly beneath the current
chassis. Its missing returns alone do not establish unsafe startup, and the
no-rover fixtures do not establish visibility through actual chassis/tires.

The same 25-view **virtual** yaw union covers every declared band cell for all
new designs, versus incomplete baseline coverage. No robot yaw was executed;
this does not establish full support or authorize an observation maneuver.

## What was implemented and verified

- [ray_design.py](../../tools/local_terrain/ray_design.py): deterministic layouts,
  fixed-budget sampling, independent obstacle banks, and strict native ray IDs.
- [diagnose_local_ray_design.py](../../tools/diagnose_local_ray_design.py): fresh
  external-archive runs, immutable input/source hashes, all candidates and misses.
- [probe_local_ray_design.py](../../tools/probe_local_ray_design.py): isolated
  Isaac **Generic range sensor** using explicit azimuth/zenith tables, actual
  returned native samples, fixed scene/mount and equal acquisition dose.
- Tests cover fixed count/endpoints, minimum-range loss, off-center yaw geometry,
  changed obstacle placements, float32 angle identities and stale-table rejection.

Final native validation: **22/22 cases passed**, **264 recorded chunks**,
**2,814,504 samples**. Each case records exactly **127,932 samples** and observes
all **63,968 unique beam identities**. Nominal rate is 639,680/s; the SDK delivers
10,661 samples per measured 60 Hz tick, so actual dose is recorded rather than
rounded to “two complete revolutions.” No hit-mask discrepancies; maximum
depth/XYZ errors **1.81/2.48 µm**, below unchanged 2 mm limits. Every stationary
region/phase metric exactly matches the analytic screen.

The native validation uses the fixed eight-box scene, not a claim of physically
capturing all 256 independent bank placements. Those banks remain analytic
geometry screens, supported by the independently checked ray/box model.

Initial sensor-reuse attempt failed when the next layout returned foreign beam
angles after `world.reset()`. It is retained and rejected. Fresh, uniquely named
sensors per case solve the queued-pattern contamination; only one USD sensor is
active. Raw measured data is now saved before association validation. SDK “need
more data” warnings remain in the log; all measured identity/dose/geometry gates
pass, but this instrumented fixture is not a sensor/runtime benchmark.

## Evidence and reproduction

External archive runs:

- First ten-layout screen: `revision-23f8bae3cdc844dd95ca9d6a6e1cad3d`.
- Final eleven-layout/two-bank screen: `revision-b478574938d84961be08040ce8e8e5a8`.
  Every original ten-layout metric is unchanged; the additional candidate/bank
  were declared before the second screen.
- Failed sensor-reuse launcher: `revision-fbd08efc890047d79ac0efb61a849622`,
  inner `revision-043dc315303b414bb0c3706590634eb6`.
- Successful isolated-sensor launcher: `revision-1fdd058181984e99b548e8db4853d308`,
  inner `revision-cae5f66863554d13babf917efaf32529`.

Commands (host; archive path arguments must resolve on the external drive):

```bash
python3 tools/diagnose_local_ray_design.py <mount-study-run> --diagnose-failed-prefix
python3 tools/run_local_isaac_fixture.py ray_design --study-run <ray-screen-run>
python3 -m pytest -q -p no:cacheprovider tools/test/test_local_ray_design.py tools/test/test_local_mounting.py tools/test/test_local_observability.py tools/test/test_local_isaac_fixture.py tools/test/test_local_support_sweep.py tools/test/test_local_physx_reference.py
```

Final focused suite: **83 passed, no skips**. Formatter and whitespace checks pass.
No production assets, GLIM configuration, navigation, classifier thresholds or
evidence lifetimes changed.

## Next integration gates

1. Check the selected rays against actual rover optical self-occlusion and terrain
   slopes/attitude; preserve the near-field minimum-range gap as unknown.
2. Add an opt-in live custom-ray capture adapter. The existing LiDAR adapter
   expects separate column/ring vectors; Generic returns per-ray angle arrays and
   partial columns. Preserve native times and identities—do not stitch/retimestamp
   them into invented full scans. Invalidate old evidence on layout/epoch changes.
3. Compare baseline, uniform +10..-45, and `far_dense` on the same guarded moving
   protocol, measuring GLIM accuracy and causal terrain/swept-motion safety.
4. Run both resource profiles and concurrent live-load checks. The leading layout
   gives **47,613** ideal ground returns within 5 m versus **24,998** baseline:
   unchanged emitted ray count does **not** mean unchanged mapper computation.
   Promotion still requires the existing safety, availability and timing gates.
