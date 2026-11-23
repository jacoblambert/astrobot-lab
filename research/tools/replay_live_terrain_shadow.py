#!/usr/bin/env python3
"""Exact replay of completed advisory ROS shadow callback order, not GT recall."""
import argparse
import hashlib
import json
import numpy as np
from pathlib import Path
from types import SimpleNamespace as NS

from archive_guard import allocate_run, check_run
from layered_terrain import TerrainMapper
from live_terrain_shadow import sha_file
from native_chunk_transport import FIELDS, MAX_POINTS, decode
from terrain_shadow_core import ShadowCore
from terrain_shadow_runtime import ReceiptScheduler

RUNTIME_MODULES = ("terrain_shadow_core.py", "terrain_shadow_runtime.py", "native_chunk_transport.py",
                   "layered_terrain.py", "shadow_terrain.py", "raw_return_filter.py", "revision_evidence.py")


def verify_runtime_sources(manifest):
    """Checking archived files alone does not identify the code running replay."""
    recorded = {}
    for path, value in manifest["source_sha256"].items():
        name = Path(path).name
        if name in recorded:
            raise ValueError("ambiguous runtime source provenance")
        recorded[name] = value
    for name in RUNTIME_MODULES:
        if recorded.get(name) != sha_file(Path(__file__).with_name(name)):
            raise ValueError("replay runtime differs from recorded source: " + name)


def validate_collection(collection, diagnose_heartbeat_prefix=False):
    if diagnose_heartbeat_prefix:
        if (collection["experiment_completed"] or collection["status"]["completed"] or
                collection["status"]["failure"] != "simulation wall heartbeat lost"):
            raise ValueError("prefix diagnosis requires an archived simulation heartbeat failure")
    elif not collection["experiment_completed"] or collection["status"]["failure"]:
        raise ValueError("failed/incomplete live shadow run is ineligible")


def validate_terminal_fault(payload, collection, enabled, seen):
    if not enabled or seen or payload != "simulation wall heartbeat lost" or payload != collection["status"]["failure"]:
        raise ValueError("unexpected terminal fault in diagnostic prefix")


def replay(source, *, diagnose_heartbeat_prefix=False):
    source = Path(source)
    check_run(source)
    manifest = json.loads((source / "manifest.json").read_text())
    collection = json.loads((source / "collection.json").read_text())
    validate_collection(collection, diagnose_heartbeat_prefix)
    hashes = {**manifest["source_sha256"], manifest["library"]: manifest["library_sha256"]}
    for name, expected in collection["output_sha256"].items():
        if name not in ("events.jsonl", "native.bin", "projections.jsonl") or sha_file(source / name) != expected:
            raise ValueError("live output hash mismatch")
    if set(collection["output_sha256"]) != {"events.jsonl", "native.bin", "projections.jsonl"}:
        raise ValueError("incomplete live output provenance")
    for path, expected in hashes.items():
        if sha_file(path) != expected:
            raise ValueError("live source/library changed: " + path)
    verify_runtime_sources(manifest)
    core = ShadowCore(lambda: TerrainMapper(manifest["library"], **manifest["geometry"]),
                      minimum=manifest["minimum"], maximum=manifest["maximum"])
    scheduler = ReceiptScheduler(core)
    projections = 0
    terminal_fault = False
    live_ms, replay_ms = [], []
    try:
        with (source / "native.bin").open("rb") as raw, (source / "events.jsonl").open() as events, \
                (source / "projections.jsonl").open() as maps:
            for line in events:
                if terminal_fault:
                    raise ValueError("events follow terminal fault")
                event = json.loads(line)
                kind, payload = event["kind"], event["payload"]
                if kind == "chunk":
                    size = payload["size"]
                    if raw.tell() != payload["offset"] or not 0 < size <= 40 * MAX_POINTS or size % 40:
                        raise ValueError("raw chunk extent mismatch")
                    data = raw.read(size)
                    if len(data) != size or hashlib.sha256(data).hexdigest() != payload["sha256"]:
                        raise ValueError("raw chunk hash mismatch")
                    sec, nanosec = divmod(payload["stamp_ns"], 10**9)
                    message = NS(header=NS(frame_id="vlp16", stamp=NS(sec=sec, nanosec=nanosec)),
                        height=1, width=size//40, point_step=40, row_step=size, is_dense=True, is_bigendian=False,
                        fields=[NS(name=n, offset=o, datatype=t, count=c) for n, o, t, c in FIELDS], data=data)
                    chunk = decode(message)
                    if any(chunk[k] != payload[k] for k in ("sequence", "capture", "stamp_ns")):
                        raise ValueError("raw chunk metadata mismatch")
                    scheduler.receive(kind, chunk, size)
                elif kind in ("revision", "complete"):
                    scheduler.receive(kind, payload, len(json.dumps(payload)))
                elif kind == "clock":
                    result = scheduler.clock(payload)
                    if result is not None:
                        expected = json.loads(next(maps))
                        status = core.status()
                        # Timing is deliberately excluded. Every six-field map
                        # hash, identity, watermark, count and association agrees.
                        for key, value in expected["status"].items():
                            if key != "compute_ms" and status[key] != value:
                                raise ValueError("live/replay status differs: " + key)
                        if expected["associations"] != core.last_associations:
                            raise ValueError("live/replay association differs")
                        live_ms.append(expected["status"]["compute_ms"])
                        replay_ms.append(status["compute_ms"])
                        projections += 1
                elif kind == "fault":
                    # A wall watchdog cannot be reproduced by an offline clock.
                    # Verify the entire causal prefix, then replay its *recorded*
                    # terminal invalidation. This never qualifies the failed run.
                    validate_terminal_fault(payload, collection, diagnose_heartbeat_prefix, terminal_fault)
                    core.fault(payload)
                    scheduler.pending.clear()
                    scheduler.bytes = 0
                    terminal_fault = True
                else:
                    raise ValueError("fault/unknown live event cannot pass replay")
            if (raw.read(1) or next(maps, None) is not None or projections == 0 or
                    (not terminal_fault if diagnose_heartbeat_prefix else not core.completed)):
                raise ValueError("incomplete replay prefix/tail")
            for key, value in collection["status"].items():
                if key != "compute_ms" and core.status()[key] != value:
                    raise ValueError("live/replay terminal status differs: " + key)
        for path, expected in hashes.items():
            if sha_file(path) != expected:
                raise ValueError("replay source changed during evaluation")
        verify_runtime_sources(manifest)
        def percentiles(values):
            return dict(zip(("p50", "p95", "p99"), np.percentile(values, [50, 95, 99]).tolist()))
        return dict(exact_match=True, projections=projections, chunks=core.integrated,
                    diagnostic_failed_prefix=diagnose_heartbeat_prefix,
                    source_experiment_completed=collection["experiment_completed"],
                    source_failure=collection["status"]["failure"],
                    compute_ms=dict(live=percentiles(live_ms), offline_diagnostic=percentiles(replay_ms)),
                    source=str(source), input_sha256={str(source / k): v for k, v in collection["output_sha256"].items()},
                    terrain_acceptance=False, live_timing_acceptance=False, navigation_acceptance=False)
    finally:
        core.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--diagnose-heartbeat-prefix", action="store_true",
                        help="diagnose an archived heartbeat-failed prefix; NEVER a completed-run qualification")
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = replay(args.source, diagnose_heartbeat_prefix=args.diagnose_heartbeat_prefix)
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc)), stream, indent=2)
        raise
    with (output / "live_replay.json").open("x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
