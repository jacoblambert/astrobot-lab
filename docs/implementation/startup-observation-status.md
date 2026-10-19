# Startup observation: what we learned

Updated October 4, 2026.

We implemented the requested safe starting area: a fixed circle around the
initial robot center, approximately **61 cm radius**, based on the robot's
physical turning sweep. It does not grow or follow the robot. Measured obstacles
still override it. The raw terrain map is not painted free.

## Actual simulator results

| Observation maneuver | Actual movement | Tested straight exits into newly observed ground |
| --- | --- | --- |
| Full turn, with short repositioning moves | 359.8°; center stayed within 18.1 cm of startup | 0 of 177 decision times |
| Reverse 15 cm, then return | 13.1 cm backward | 0 of 21 decision times |
| Reverse 20 cm, then return | 19.6 cm backward | 0 of 26 decision times |

At each decision time we checked forward and reverse paths of 10–100 cm.
These were offline checks against exact replays of the live maps, not autonomous
driving. The simulator acquisition driver used GT and a separately surveyed
clear patch. GLIM used a GT startup seed and synthetic RTK assistance. Two
earlier turn attempts exceeded the existing drift limit and remain failed.

**Zero exits does not mean zero movement.** After the 20 cm test, the map allowed
10 and 20 cm forward moves within the starting assumption. A 30 cm move reached
ground that was still unconfirmed. We have not tested every possible curved or
multi-stage observation maneuver.

## Where the gap is

After the full turn, **98.5% of the surveyed clear-ground samples 1–1.3 m from
startup were classified free**. Between the 61 cm starting circle and 1 m,
only **38.3%** were classified free. No sampled points in these bands were
classified occupied. The problem here is unknown ground, not false dropoffs.

A flat-ground model including chassis occlusion explains part of this: the
current mount's nearest visible ground is about **73.5 cm from the center**.
Many steep rays hit the robot itself. Turning in place does not automatically
fill that inner blind region. Reversing brings some visible ground into the
starting area, but partly observed 10 cm cells can still remain unknown.
This model is a diagnosis aid, not a substitute for native sensor validation.

The 20 cm test did gain a little classified ground inside the starting circle,
but the tested forward wheel paths still crossed a small unsupported region
(about 0.021 m² for the 40–100 cm candidates). We need to inspect that region,
not indiscriminately fill the much larger raw-map hole.

## What to change next

1. Locate those unsupported wheel-path fragments in the saved replay. Separate
   missing measurements from valid measurements lost when reducing evidence to
   whole 10 cm cells.
2. If the measurements exist, test whether support can be represented inside a
   cell without declaring its unseen portion free. Keep hole, edge and raised
   obstacle tests unchanged.
3. If the measurements are absent, test a different observation trajectory or
   chassis-aware sensor placement. Do not enlarge the initial safe area to hide
   the problem.
4. Require a fresh run that actually offers supported progress beyond startup,
   then evaluate terrain-controlled driving and compute limits.

**Verification:** 367 regression tests passed. All three completed runs reproduce
their original maps and advisory decisions exactly. The full-turn native-ray
audit passed; the 15 cm run had one missing simulator reference hit and remains
diagnostic-only. The 20 cm native audit also passed, with maximum sampled
geometry error 0.055 mm. First-published GLIM horizontal error in that run was
6.7 cm at the 95th percentile (10.2 cm maximum), so pose uncertainty can still
matter at 10 cm cell boundaries. Resource and exploration qualification remain open.

## Grid examples

The archive root is
`/media/jacoblambert/6c8aadd0-9332-4357-aea4-437b6aadef261/astrobot-lab`.
Open either standalone HTML file in a browser:

- Full turn: `revision-6665b7c06f2d44e9aae9d9782c719ae5/startup-observation.html`.
- Reverse 20 cm: `revision-166748cca0a847c788f92a07e84a45e8/startup-observation.html`.

Green is classified ground, gray is unknown, red is occupied. The starting
assumption is deliberately not painted into these raw grids. Detailed artifact
IDs, failed attempts and commands are in the [implementation ledger](implementation-plan.md)
and [runbook](agent-runbook.md#startup-observation-extension).
