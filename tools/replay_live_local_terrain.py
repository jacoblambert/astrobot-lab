#!/usr/bin/env python3
"""Exact v2 receipt-order replay. Completed source only, no GT terrain inputs."""

import argparse
import copy
import json
from pathlib import Path

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.core import LocalConfig, LocalReconstructor
from local_terrain.rolling_mapper import RollingTerrainMapper
from local_terrain.self_filter import chassis_returns
from replay_local_terrain import decoded_chunk, relocate, sha
from selected_lidar_profile import profile_path, selected_setup
from terrain_shadow_runtime import ReceiptScheduler


def replay(source, archive, *, projection_observer=None, bookkeeping_candidate=False):
    source = Path(source).resolve(strict=True)
    archive = Path(archive).resolve(strict=True)
    if source.parent != archive:
        raise ValueError("source must be directly on external archive")
    manifest = json.loads((source / "manifest.json").read_text())
    collection = json.loads((source / "collection.json").read_text())
    healthy = collection["experiment_completed"] and not collection["status"]["failure"]
    if (
        manifest["policy"] != "live_local_v2"
        or not healthy
        and not (
            bookkeeping_candidate and collection["status"]["failure"] == "only contiguous ODOM deltas are supported"
        )
    ):
        raise ValueError("completed healthy v2 source required")
    hashes = {source / name: digest for name, digest in collection["output_sha256"].items()}
    hashes.update({source / name: sha(source / name) for name in ("manifest.json", "collection.json")})
    changed_runtime = {}
    historical_live = {}
    for name, digest in manifest["input_sha256"].items():
        path = relocate(name, archive)
        if str(path).startswith("/workspace/astrobot-lab/"):
            path = Path(__file__).resolve().parents[1] / Path(path).relative_to("/workspace/astrobot-lab")
        if (
            bookkeeping_candidate
            and path
            in (
                Path(__file__).with_name("live_local_terrain.py"),
                Path(__file__).parent / "local_terrain/core.py",
            )
            and sha(path) != digest
        ):
            changed_runtime[str(path)] = dict(recorded=digest, candidate=sha(path))
            hashes[path] = sha(path)
        elif (
            path
            in (
                Path(__file__).with_name("live_local_terrain.py"),
                Path(__file__).parent / "local_terrain/contact_advisory.py",
                Path(__file__).parent / "local_terrain/contact_runtime.py",
            )
            and sha(path) != digest
        ):
            # The ROS entrypoint is not executed by this replay. Preserve and
            # verify its recorded source while allowing opt-in output changes.
            # No reconstruction dependency receives this exception.
            from terrain_entrypoint_archive import historical_entrypoint

            historical_live[path] = historical_entrypoint(path, digest, archive)
            hashes[path] = digest
        else:
            hashes[path] = digest
    if bookkeeping_candidate:
        for path in (Path(__file__), Path(__file__).parent / "local_terrain/bookkeeping.py"):
            hashes[path] = sha(path)
    for path, expected in hashes.items():
        if sha(historical_live.get(path, path)) != expected:
            raise ValueError("source/runtime input changed: " + str(path))
    library = relocate(manifest["library"], archive)
    config_values = dict(manifest["config"])
    if bookkeeping_candidate:
        config_values["allow_bookkeeping"] = True
    config = LocalConfig(**config_values)
    setup, _ = selected_setup(profile_path(manifest["request"]["lidar_profile"]["name"]))
    core = LocalReconstructor(
        lambda origin, lifetime: RollingTerrainMapper(
            library, width=config.cells, height=config.cells, origin=origin, lifetime=lifetime
        ),
        manifest["request"]["base_from_imu"],
        config,
    )
    scheduler = ReceiptScheduler(core)
    frames = 0
    archived_fault = None
    try:
        with (source / "native.bin").open("rb") as raw, (source / "projections.jsonl").open() as projections:
            for line in (source / "events.jsonl").open():
                event = json.loads(line)
                kind, payload = event["kind"], event["payload"]
                if archived_fault is not None:
                    raise ValueError("unexpected events after archived terminal fault")
                if kind == "chunk":
                    chunk = decoded_chunk(payload, raw)
                    if chunk["capture"] != manifest["request"]["capture"]:
                        raise ValueError("foreign native source")
                    points = chunk["points"].copy()
                    points[chassis_returns(points, setup) | (chunk["intensities"] == 0)] = 0
                    chunk["points"] = points
                    scheduler.receive(kind, chunk, payload["size"])
                elif kind in ("revision", "complete"):
                    scheduler.receive(kind, payload, len(json.dumps(payload)))
                elif kind == "clock":
                    if scheduler.clock(payload) is not None:
                        expected = json.loads(next(projections))
                        actual = core.last_record.copy()
                        # Wall timing is not deterministic and never used to hide
                        # geometry/association/state differences.
                        expected.pop("compute_ms")
                        actual.pop("compute_ms")
                        if actual != expected:
                            raise ValueError("live/replay projection mismatch at " + str(frames))
                        if projection_observer is not None:
                            # Evaluator receives copies, only AFTER exact validation.
                            # Never let labels or diagnostics modify live reconstruction.
                            projection_observer(
                                core.last_result.copy(),
                                copy.deepcopy(core.last_record),
                                published_valid=not core.completed,
                            )
                        frames += 1
                elif (
                    bookkeeping_candidate
                    and not healthy
                    and kind == "fault"
                    and payload == collection["status"]["failure"]
                ):
                    archived_fault = payload
                else:
                    raise ValueError("unexpected/fault event in healthy v2 source")
            if projections.read() or raw.read(1) or (healthy and not core.completed):
                raise ValueError("missing completion or unconsumed source tail")
        if (healthy and core.received != collection["status"]["received"]) or frames != collection["status"][
            "projected"
        ]:
            raise ValueError("live/replay completeness mismatch")
        if not healthy and (archived_fault is None or core.failure or not core.bookkeeping.accepted):
            raise ValueError("candidate did not resolve recorded bookkeeping barrier")
        for path, expected in hashes.items():
            if sha(historical_live.get(path, path)) != expected:
                raise ValueError("input changed during replay: " + str(path))
        result = dict(
            passed=True,
            projections=frames,
            native_chunks=core.received,
            exact_layers_and_associations=True,
            acceptance=False,
            input_sha256={str(k): v for k, v in hashes.items()},
        )
        if bookkeeping_candidate:
            result.update(
                candidate_diagnostic=True,
                source_completed=bool(healthy),
                archived_fault=archived_fault,
                changed_runtime=changed_runtime,
                bookkeeping_batches=core.bookkeeping.accepted,
                canonical_frames=len(core.bookkeeping.frames),
                original_received=collection["status"]["received"],
                note="All recorded projections must remain exact. Failed source stays failed; only the recorded barrier is tested, not the missing route.",
            )
        return result
    finally:
        core.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument(
        "--bookkeeping-candidate",
        action="store_true",
        help="Explicit changed-runtime comparison; permits ONLY the recorded ODOM-kind failure, never capture acceptance",
    )
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = replay(args.source, args.archive_root, bookkeeping_candidate=args.bookkeeping_candidate)
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(result, stream, indent=2)
        print(json.dumps({k: v for k, v in result.items() if k != "input_sha256"}), flush=True)
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
