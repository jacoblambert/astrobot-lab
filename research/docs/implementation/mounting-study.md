# LiDAR mounting study — September 27, 2026

## Decision

**Keep the current production mount for now.** The tested fixed pitches trade
near-field ground sampling against visibility elsewhere; none dominates the
current +5-degree arrangement. A pure startup spin with the current mount does
not solve the innermost ground blind region in the ideal model. Do not implement
a blind spin/jitter recovery from these results.

This is a completed bounded sampling study, **not rover or terrain qualification**.
We changed diagnostic code only: no production asset, TF, GLIM, mapper thresholds,
evidence lifetime or command controller changed.

## What was tested

- Recorded mount offset: 0.4 m forward and 0.25 m above base; measured level-ground
  base height 0.144923 m. Native 32 elevations span approximately ±20.15 degrees,
  minimum range 0.5 m, full revolution 1999 columns. Asset naming is not the spec.
- Absolute pitch candidates: +5/+10/+15/+20/+30/-10/-20/-30 degrees. Positive means
  front beams tilt down; negative means rear beams tilt down.
- Ideal no-occlusion flat-plane screen: stationary and 25 virtual views at yaw
  0:15:360 degrees, same number of views in both arms. This moves the off-center
  sensor around the base, not just its viewing direction. **No robot was spun.**
- Four fixed 10 cm grid phases: 0/2.5/5/7.5 cm. Wheel-track strips are 12 cm wide,
  using measured left/right wheel centers, at 0–0.25 / 0.25–0.5 / 0.5–1 m ahead
  of the front axle. Every cell with positive-area strip overlap is counted.
  Front/left/rear/right sectors separately cover center radii 0.75–3 m.
- Native Isaac check: stationary sensor, no rover/ROS/SLAM, analytic floor and
  eight 10×10×8 cm boxes at alternating radii 1.25/2.25 m and 45-degree bearings.
  Six warmup and six recorded native ticks per pitch/scene, at the actual 10 Hz
  sensor rotation rate. No synthetic scans were substituted into a mapper.

The boxes are a development fixture, not a complete minimum-hazard specification.
One or more rays hitting a box does not mean the traversability classifier detects it.

## Stationary results confirmed by native Isaac

Percentages are **cells with any ground return**, spanning all four grid phases.
They are not full-cell support, safe-motion availability, or terrain accuracy.

| Pitch | 0–0.25 m strip | 0.25–0.5 m strip | 0.5–1 m strip | Boxes receiving rays / 8 |
| --- | --- | --- | --- | --- |
| +5° current | 0% | 0% | 25–33% | 8 |
| +10° | 0% | 0% | 50–58% | 7 |
| +15° | 0% | 0% | 67–83% | 7 |
| +20° | 0% | 0–25% | 83–100% | 5 |
| +30° | 0% | 50–67% | 100% | 5 |
| -10° | 0% | 0% | 0% | 7 |
| -20° | 0% | 0% | 0% | 7 |
| -30° | 0% | 0% | 0% | 5 |

For example, +10 degrees already loses the directly-rear 1.25 m box; -10 degrees
loses the directly-forward box. Stronger forward tilt loses more rear coverage.
No candidate eliminates the near-body sampling gap while retaining every fixture
hazard at the stationary pose.

**Interpret the first strip carefully:** the front axle is at x≈0.256 m and the
body front at x≈0.504 m. Its first 25 cm are mostly beneath the present chassis.
Unobserved ground there is not equivalent to an unseen obstacle corridor ahead
of the bumper, nor proof that a supported rover cannot start. Initial support
and newly entered ground must be evaluated separately.

## Virtual yaw result and its limit

With the current +5-degree mount, full virtual yaw still supplies **0%, 0%,
54–67%** of the three strip cell sets. At -10 degrees, the ideal yaw union supplies
**100% in all three strips**. This is a large change in sampling opportunity,
not an achieved drivable map or a verified startup maneuver.

The direction matters: wheel centers lie approximately 0.383 m from the base
origin, inside the sensor's 0.4 m orbit. Their initial ground locations remain
behind the sensor during rotation. Stronger front-down pitch therefore does not
solve those locations; rear-down pitch can bring them into the angular cone.

Neither the screen nor the native fixture includes rover optical self-occlusion.
Actual chassis/tires, discrete returns, support extent, SLAM errors, skid drift and
age/uncertainty still need evaluation. A current-body collision proxy must not be
mistaken for a validated optical model. A single fixed tilt is not yet justified.

## Validation and evidence

- Final ideal run: `revision-0ac13d780e334b889880f89fdae0d5bd`; all metrics exactly
  match first screen `revision-5d5fcf467be04a45a92da50174e4e340` after the reporting
  fix/formatting. Source recording's failed status remains explicit.
- Native launcher: `revision-1131448c4ad94eb2a0427045389d90ca`;
  inner run: `revision-b1f689b1c49c4510954a51f110e60a38`.
  **16/16 geometry cases pass**, 96 chunks / **2,045,952 rays**, zero hit-mask
  discrepancies, maximum depth/XYZ residual **5.28/5.44 µm** (2 mm gates).
  All stationary grid metrics match the ideal screen exactly; all box counts
  match its predictions at the separately declared two-revolution native dose.
- Initial native attempt `revision-5577e47557b9444b8a22b52780d2eed4`, inner
  `revision-ebc68268e9b44dbb8341e25937fd9bbb`, failed JSON serialization of a NumPy
  boolean. It is retained and ineligible; conversion and regression tests fix it.
- **71 focused host tests pass**, formatter check passes. No runtime/GLIM/terrain
  or moving-rover acceptance is claimed by these diagnostic tests.

Artifacts reside under the external archive listed in the runbook. The exact
commands and prerequisites are in [agent-runbook.md](agent-runbook.md).

## Next bounded experiment

1. Define the independently established starting support/clear area and the
   newly entered wheel/body corridor. Never substitute GT free labels into the
   deployed mapper or infer adjacent safety from the robot merely standing still.
2. Check actual rover self-occlusion and mounting feasibility for the baseline
   and rear-down diagnostic candidate before paying for another moving SLAM run.
   Preserve stationary forward/rear hazard checks; do not optimize only the yaw union.
3. Only then compare a guarded observation route from that verified starting
   region. Pure yaw with the baseline is insufficient in this model; tiny jitter
   must not be assumed sufficient either. The existing forward-history bound is
   approximately 0.95/1.46 m for front/rear wheel-center ground at baseline.
4. Score the causal traversability map and swept short motions, including startup,
   all stops and missing evidence. Retain the existing zero-false-safe and 95%
   reference-safe-motion availability gates; none was passed by this study.
