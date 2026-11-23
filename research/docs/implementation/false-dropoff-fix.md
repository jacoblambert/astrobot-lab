# False dropoff fix — October 4, 2026

**Implemented and verified on the completed loop. After startup, a useful short
move or turn is available in 99.3% of sampled decisions, up from 70.9%.**

This is the long-loop **replay** result, not a guarantee for every route. The
subsequent [three online scenarios](online-terrain-scenarios.md) preserve the
dropoff fix but expose substantial near-wheel blind areas on short routes.

[Before/after HTML and all fixed-time grids](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-803fe3e1a6f74edb95f4562ce6d4f9c8/false-dropoff-fix.html)
· [Example at +180 seconds](/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab/revision-381dd459574e4d1f93665f6d3040e3cd/dropoff-before-after.png).

## What was wrong

The single-scan boundary test compared `sqrt(sum(residual²))` with 5 mm.
That is an upper bound on an individual return's deviation, not an observed
discontinuity. It grows with point count. Many tiny deviations could therefore
trigger a warning even though no individual point was that far from the surface.
The warning then withheld support from neighboring cells for the retained
evidence lifetime. More nearby returns could make the map *less* useful.

The diagnostic replay reproduced every original map before inspecting this.
Of 2,756 rejected current-wheel cell exposures, 2,750 came from this native
boundary path, six from the separate cross-scan surface-normal rule, and none
from a low return below the fitted ground plane.

## What changed

The experimental pointwise-boundary path keeps the same slope-limited surface
fit and **the same 5 mm threshold**. When the aggregate bound cannot establish
that the input is clear, it checks each actual return. A warning requires an
actual excessive deviation. This is not RMS averaging, percentile clipping,
outlier removal or raising the obstacle threshold.

Each return stays associated with the exact height layer chosen during its
original integration. Only the current native input needs that scratch storage;
no new raw-cloud history is retained. Evidence expiry, neighboring-cell protection,
8 cm obstacle classification, grid size, ray layout, GLIM poses and map retention
are unchanged. Real boundary warnings are not erased by later smooth scans.

Critical code: `layered_terrain.cpp` in the `TERRAIN_POINTWISE_BOUNDARY` integration
block; the fitted direction is in `observation_moments.hpp`. Both immediate and
deferred classification paths use the same pointwise decision. The build option
is `tools/build_local_terrain.py --pointwise-boundary`; normal builds remain
unchanged. The tested library is available for the next opt-in advisory capture.

## Same-data reassessment

All 17,949 native inputs and 2,992 mapper outputs were replayed with the original
recorded placements and timing. A baseline copy matched the original output
hash at every projection. The instrumented immediate candidate also matched
the uninstrumented deferred candidate **byte for byte on all 2,992 outputs**.
The 2,991 usable maps, independent floor samples and 300 fixed-time motion
queries are the same as in the original assessment.

| Measurement | Before | With fix |
| --- | ---: | ---: |
| Move toward goal, including startup | 187/300 (62.3%) | 272/300 (90.7%) |
| Move or turn, including startup | 202/300 (67.3%) | 284/300 (94.7%) |
| Move toward goal, after first 15 s | 187/285 (65.6%) | 271/285 (95.1%) |
| Move or turn, after first 15 s | 202/285 (70.9%) | 283/285 (99.3%) |
| Surveyed clear ground marked free, front 0–1 m | 89.87% | 97.74% |
| Surveyed clear ground marked free, front 1–2 m | 94.82% | 99.31% |
| Surveyed clear ground marked free, front 2–3 m | 91.34% | 91.42% |

The +152 to +212 second refusal sequence is gone. All 136 sampled turning-leg
decisions now have a supported move/turn. False occupied sample counts are
unchanged: zero within 2 m in front and 46 repeated sample exposures at 2–3 m.

The two post-startup sampled refusals are now separate issues:

- **+214 s:** two wheel cells lack support for their full cell extent.
- **+245 s:** one wheel cell is withheld by the cross-scan fitted-normal rule.

Across all maps, the candidate has **zero native-boundary rejections under the
current wheel patches**, versus 2,750 before. Six pooled-normal cell exposures
remain. This is not a claim that every native boundary everywhere disappeared
or should disappear.

## Tests and runtime

- **17 new fixtures pass:** benign roughness at multiple point densities and
  grid phases; single ±8 mm / ±20 mm deviations among many ground returns;
  boundary evidence survives later smooth input and expires normally.
- **605 existing two-path tests pass**, including holes, thin/partial faces,
  registration perturbations and immutable expiry, with the documented legacy
  baseline. A first invocation used the wrong baseline for one historical
  control: 604 passed, one control failed. That attempt remains in the ledger.
- **304 candidate terrain/local/rolling/calibration tests pass**, including
  8 cm objects on slopes, hole-rim support and 1/2/4-worker checks.
- **158 immediate/deferred and diagnostic equivalence tests pass**;
  **50 serial/OpenMP equivalence tests pass**. Both C++ contract programs pass
  for the parallel and serial candidates.
- **50 unchanged-mode equivalence tests pass** against the original library.
  **145 evaluator/report/query tests pass** on the final Python revision.

Both unchanged resource profiles pass, with four workers:

| Profile | p95 | p99 | Peak process memory |
| --- | ---: | ---: | ---: |
| Layer capacity | 43.26 ms | 56.67 ms | 1,016.05 MiB |
| Dense flat | 37.31 ms | 43.43 ms | 472.69 MiB |

The capacity profile is close to the 1 GiB limit; do not assume headroom for
larger retention or additional layers. These are separate fixed-load tests,
not timing under concurrent live Isaac/GLIM. Instrumented replay timing is not
used for compute acceptance.

## What remains

The first 14 sampled seconds still cannot support a move/turn from map evidence
alone. The original driver used GT clearance to start, so later good maps do
not prove autonomous startup. The two isolated later refusals remain visible;
no additional rule was weakened to force a perfect score.

Next: a fresh guarded capture with this library, then fresh routes containing
real obstacles and holes and a declared startup observation maneuver. This work
establishes a large improvement on identical recorded LiDAR/SLAM inputs and
nonregression on the existing analytic hazards. It is not new live navigation,
independent obstacle recall on the natural scene, or full safety qualification.
GT remains evaluator-only; no mapper or driving labels were supplied from GT.

## Reproducible artifacts

- Tested uninstrumented library: `revision-70647a2b742c446eb61bd3d7141ea011/liblocal_terrain.so`,
  SHA `e452b2b61e8d177642d9f4260851d3b3047fb6c019193f3fbe0b7d1d5b9b4df8`.
- Serial library: `revision-1168b36eb64c4bf690a68a8e587709f0`.
- Immediate diagnostic library: `revision-8ece4fc0f04f415f8b603e1dadfb2972`.
- Original-rule attribution: `revision-d0913b7bc11248d899838ac1b184387f`.
- First candidate reassessment: `revision-10124f9f4c3242ecb295686136fb6a1b`.
  Its uninstrumented cause counters are placeholders, not measured zeros.
- Final matched diagnostic comparison and grids: `revision-cf3189e2c434446e8aa4857bd937547a`.
- Resource runs: `revision-db1f90a7bc034c2e9f4261ea044ea0e0` and
  `revision-f3fdc3dcaa3a41e8a3d13109b02a7fe8`.

Commands are in the [runbook](agent-runbook.md). No simulator was started and
no captures were deleted. The HTML's before/after grids were rasterized and
visually checked; a full browser rendering was not exercised.
