# Capture I/O audit

Updated: 2026-10-04. Scope: selected optical v2, coupled GLIM/terrain capture.
This is a source audit and an implementation proposal, not a storage-performance
qualification. Existing captures and their acceptance rules remain immutable.

October 4 status: the extended-budget October 3 loop failed on a measured
491.93 ms full-GC pause at 258.283 sim s, not a storage-capacity or overall-budget
failure. Current heap diagnosis found cached Kit import exceptions retaining
10,318 Python frames. Disabling that optional cache removes the frames but still
costs 465–481 ms per full collection. A lean, opt-in Isaac experience reduces
stationary full collections to 416–431 ms while exactly preserving the checked
scene, clearance, sensor audit and graph contract. Subsequent moving trials still
failed. Concurrent exact storage checks and dedicated grant reception pass a
short stress capture, but a nominal loop's 385 ms GC plus synchronous simulator
work still expired the motion-update watchdog. Lazy loading only the selected
environment and compact floor-survey storage reduce tracked objects by another
73,565; a fresh stationary comparison measures 335–361 ms collections with all
four geometry/scene artifacts exactly equal. Short moving stress is being
retested; long-run robustness is not established. No GC policy or watchdog
changes. The diagnostic-topic migration below remains unimplemented and separate
from these startup-runtime experiments; see the implementation ledger for IDs.

## Finding

The two-files-per-tick layout is unnecessary for the live estimator. Native
returns already travel on `/terrain_shadow/native_chunks` and are recorded in
MCAP. The simulator also writes an NPZ and a PhysX-hit JSON for every 60 Hz tick
**before publishing that topic**. At 13,705 ticks this means 27,410 per-tick files,
plus metadata. A separate diagnostic topic is the appropriate replacement for
the evaluator-only information that the current native topic omits.

File count is not evidence that filesystem I/O caused garbage collection.
The earlier failed loop measured a 404 ms generation-2 GC inside the surface
audit; compression, serialization, raycasts and filesystem work are separate
costs sharing the same simulator callback and deadline. None has yet been
isolated as the sole cause of the heartbeat stop.

## Current synchronous work

| Owner / code | Work | Why it exists / problem |
| --- | --- | --- |
| `SelectedNativeArchive.observe`, `OmniLRS/src/environments_wrappers/ros2/selected_lidar_capture.py` | Current-pose and previous-pose surface raycasts; hit-record JSON; compressed NPZ with in-memory SHA-256; encode frame metadata; publish native cloud | The NPZ read-back is fixed, but thousands of separate synchronous writes are still unnecessary for the live estimator. |
| `tools/native_chunk_transport.py` | Encode XYZ, range, intensity, sequence and capture UUID into PointCloud2 | Already carries the estimator's native returns without GT. Does not carry beam angles, SDK producer sequence, original floating-point stamp or the PhysX audit. |
| `Capture.record`, `tools/run_shadow_capture.py` | Synchronous serialization and MCAP writes, with 1 GiB bag splitting | Already records native clouds, GT, GLIM, IMU, clock and terrain outputs. Moving simulator writes here still requires bounded buffering and explicit completeness checks. |
| `Capture.check_ready`, same file | Hash frozen config; inspect command ownership; check recorder and simulator archives | Archive checks recursively stat the growing file tree. Exact device, reserve, quota and mutation checks must remain; reducing file count removes the growth without weakening them. |
| `tools/live_local_terrain.py` | Archive received native bytes plus receipt-order events and projections | The third native copy supports exact causal replay. Keep it initially; removing it requires proving reconstruction of the mapper's actual receipt order, not just bag timestamp sorting. |
| `tools/local_terrain_seal.py` | Hash output as written; close without bulk re-read | Implemented. Independent consumers still read and verify disk contents. Eliminates the previously observed multi-GB shutdown read. |

The old 27,425-file archive's exact quota walk took 46–49 ms in ten offline
measurements. The recorder previously measured 60–100 ms readiness callbacks.
Those are diagnostic costs, not an isolated proof of the simulator stall.

## Viewport trial result

Selected headless compose now disables unused viewport updates. GUI defaults,
60 Hz physics/render stepping, sensor geometry, route and safety thresholds are
unchanged. Runtime confirmed `active_viewport=True updates_enabled=False`.

- Recorder: `revision-8e36fcafda8e4759a0002f2ffc9fc3ce`.
- Native: `revision-f5c9bff6f9a8413bb5f04c07b5f8221a`.
- Mapper: `revision-f76e3dd232504365965c8b865997d357`.
- Trial stopped at simulation time 83.950004 s, after 4,040 archived native
  ticks, on **native XYZ/ray association mismatch**, not a heartbeat timeout.
  This is earlier than the previous stall and is inconclusive about its fix.
- One of 9,906 hits had 1.484 mm residual at 11.909 m; the limit remains 1 mm.
  Same-tick independent beam recast had a 1.745 mm PhysX position/distance
  inconsistency. This supports an upstream raycast-numerics hypothesis, but
  does not establish a safe error bound or justify dropping/repairing the hit.
  Diagnostic: `revision-559d2edffb0443da960dd8ed4930d34c`.
- Maximum native callback: 82 ms. A 473 ms full GC occurred during the
  stationary preflight heap checkpoint; no moving full-GC pause was logged
  before the separate failure. Recorder maximum readiness: 40 ms, with zero
  callbacks above its 50 ms diagnostic threshold. The shorter duration makes
  these numbers unsuitable for a claimed stall-rate improvement.
- Mapper closed with a nonempty failed-run seal: 1,722,817,600 native bytes,
  19,005,729 event bytes and 172,869,687 projection bytes all match independent
  disk SHA-256 reads. Its missing terminal pose bracket remains a failure;
  storage integrity is not completed-run acceptance.

## Replacement design and gates

1. Define a versioned **evaluator-only acquisition diagnostic** topic, emitted
   by the producer that can observe same-tick SDK/PhysX state. An external node
   can record it, but cannot reconstruct same-tick raycasts after the fact.
   Include capture UUID, capture sequence, SDK sequence, integer acquisition
   stamp, original observed stamp, world/timeline/step times, beam angles,
   current/previous-pose audit results and hit records. GT transforms and labels
   stay outside estimator inputs. Publish failure/GC/timing diagnostics with
   the same identity, not one file per event.
2. Extend the existing MCAP recorder to record that topic. Join clouds and
   diagnostics by **identity and sequence**, not arrival order across topics.
   Declare message-size/queue limits and fail explicitly on missing, duplicate,
   reordered, foreign, truncated or overflowing evidence. Reliable DDS alone
   is not a completeness guarantee. Completion requires recorded final counts
   and last sequence, terminal pose brackets, drained writer and verified seal.
3. Add an explicit format-version reader for evaluators. Keep legacy NPZ captures
   readable and immutable; do not rewrite their UUIDs or mark failed runs valid.
   Start with an offline bag round-trip fixture comparing every native array,
   timestamp, transform and audit record, including misses and rejected data.
   Only then opt a fresh live run into the new format and remove its per-tick
   NPZ/JSON output. Temporary dual recording is an equivalence test, not a
   performance measurement.
4. Keep exact quota/device/reserve checks over the small set of MCAP chunks.
   Preserve frozen config checks and all lease/watchdog thresholds. Instrument
   raycasts, serialization, compression, writes, quota checks and GC separately
   with bounded diagnostics. Do not disable GC or add unbounded async queues.
5. Test disk-full/write failure, drive loss, recorder death, DDS backpressure,
   lost final diagnostic and interrupted shutdown. Require explicit failure
   with no falsely complete seal. Benchmark short and long archives without
   concurrent replay/load; report every attempt. Finally repeat the declared
   route and its whole-input/pose/terrain gates with source/config hashes.

Follow-through: selected native NPZ writing now hashes the bounded serialized
buffer, then writes it once with exclusive creation. It no longer reads that
file back in the producer callback. `native_archive_io.py` rejects object arrays,
oversized records, overwrites and write failures. The simulator records its
source hash. A fixed 100-tick real-data comparison in the actual Isaac NumPy
environment verifies identical legacy/candidate NPZ bytes and independent disk
hashes (`revision-44ee3902714e4e52bc699bc5914182b6`). Median writer time changes
4.864 → 4.718 ms, p95 5.540 → 5.573 ms; **not a demonstrated p95 speedup or stall
fix**. File count is unchanged by this intermediate correction.

No diagnostic-topic migration has been implemented yet. With explicit user
direction, small XYZ/range differences are now diagnostic-only; a 5 cm gross
corruption check remains, without changing native points. The next loop passed
the previous noise-failure times but stopped on a separate 1 ns acquisition/clock
conversion disagreement. The producer conversion is now corrected to match
Isaac's clock. See the implementation ledger for the preserved failed trial
and fresh retry. Neither result yet establishes a long-run stall fix. Storage
migration and its offline fault fixtures remain independent follow-up work.

The timestamp-policy retry reached 249.53 sim s and tolerated two >1 mm native
discrepancies. It then exposed recorder scheduling starvation: exact growing-
archive readiness checks cost 80–110 ms near the end, but the main loop schedules
them every 100 ms from their start, often leaving only one ROS callback between
checks. `capture_polling.ReadinessCadence` now reserves a 100 ms callback interval
after each check finishes. No archive check is cached or weakened, and all
heartbeat/lease thresholds remain unchanged. Deterministic and real 32-topic ROS
fixtures reproduce the old failure and pass with this policy. Full-route retry
is pending. No moving full GC was logged in that attempt; do not attribute its
recorder heartbeat failure to GC from the untimed all-run maximum tick gap.
