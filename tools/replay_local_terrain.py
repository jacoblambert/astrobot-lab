#!/usr/bin/env python3
"""Opt-in v2 local reconstruction from archived live callback order (no ROS).

Outputs are diagnostics, never route or terrain-truth acceptance. Failed sources
require an explicit flag; bounded development prefixes require an explicit count.
Input hashes are checked before and after processing. No frozen runtime edits.
"""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import shutil
from types import SimpleNamespace as NS

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from build_local_terrain import validate_build
from layered_terrain import TerrainMapper
from local_terrain.core import LocalConfig, LocalReconstructor
from local_terrain.evaluation import availability, query_grid
from local_terrain.rolling_mapper import RollingTerrainMapper
from native_chunk_transport import FIELDS, MAX_POINTS, decode
from terrain_shadow_runtime import ReceiptScheduler
from terrain_shadow_core import ShadowCore


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def relocate(path, archive):
    path = Path(path)
    if path.is_relative_to("/archive"):
        return archive / path.relative_to("/archive")
    return path


def decoded_chunk(payload, raw):
    size = payload["size"]
    if raw.tell() != payload["offset"] or not 0 < size <= MAX_POINTS * 40 or size % 40:
        raise ValueError("raw native extent mismatch")
    data = raw.read(size)
    if len(data) != size or hashlib.sha256(data).hexdigest() != payload["sha256"]:
        raise ValueError("raw native payload changed")
    sec, nsec = divmod(payload["stamp_ns"], 10**9)
    message = NS(
        header=NS(frame_id="vlp16", stamp=NS(sec=sec, nanosec=nsec)),
        height=1,
        width=size // 40,
        point_step=40,
        row_step=size,
        is_bigendian=False,
        is_dense=True,
        data=data,
        fields=[NS(name=n, offset=o, datatype=t, count=c) for n, o, t, c in FIELDS],
    )
    chunk = decode(message)
    if any(chunk[k] != payload[k] for k in ("capture", "sequence", "stamp_ns")):
        raise ValueError("native identity mismatch")
    return chunk


def replay(
    source,
    capture,
    archive,
    output,
    window,
    allow_failed=False,
    max_projections=None,
    policy="local",
    profile=False,
    cells=60,
    observation_policy="coherent",
    backend="reference",
    candidate_library=None,
    projection_observer=None,
    map_policy="rolling_rebuild",
    evidence_lifetime=None,
):
    source, capture, archive = map(lambda p: Path(p).resolve(strict=True), (source, capture, archive))
    if not source.is_relative_to(archive) or not capture.is_relative_to(archive):
        raise ValueError("source and capture must be on declared external archive")
    manifest = json.loads((source / "manifest.json").read_text())
    collection = json.loads((source / "collection.json").read_text())
    source_failed = not collection["experiment_completed"] or bool(collection["status"]["failure"])
    if source_failed and not allow_failed:
        raise ValueError("failed source requires --diagnose-failed-prefix")
    link = json.loads((capture / "shadow_link.json").read_text())
    if link["request"]["capture"] != collection["status"]["capture"] or capture.name != collection["status"]["capture"]:
        raise ValueError("capture/mapper identity mismatch")
    library = relocate(manifest["library"], archive)
    hashes = {source / n: sha(source / n) for n in ("manifest.json", "collection.json")}
    hashes[capture / "shadow_link.json"] = sha(capture / "shadow_link.json")
    for path, expected in {**manifest["source_sha256"], manifest["library"]: manifest["library_sha256"]}.items():
        resolved = relocate(path, archive)
        if sha(resolved) != expected:
            raise ValueError("recorded runtime/library changed: " + str(resolved))
        hashes[resolved] = expected
    if set(collection["output_sha256"]) != {"events.jsonl", "native.bin", "projections.jsonl"}:
        raise ValueError("missing original output provenance")
    for name, expected in collection["output_sha256"].items():
        if sha(source / name) != expected:
            raise ValueError("recorded stream changed: " + name)
        hashes[source / name] = expected
    candidate_hashes = {}
    if candidate_library is not None:
        if policy != "local":
            raise ValueError("candidate library cannot replace frozen baseline")
        candidate_hashes = validate_build(candidate_library, archive)
        library = Path(candidate_library).resolve(strict=True)
    if policy not in ("local", "first_published") or policy == "first_published" and (window != 30 or profile):
        raise ValueError("baseline requires uninstrumented original 30-second configuration")
    if policy == "first_published" and (
        cells != 60
        or observation_policy != "coherent"
        or backend != "reference"
        or map_policy != "rolling_rebuild"
        or evidence_lifetime is not None
    ):
        raise ValueError("local ablation controls cannot change the frozen baseline")
    config = LocalConfig(
        window_s=window,
        minimum=manifest["minimum"],
        maximum=manifest["maximum"],
        profile=profile,
        cells=cells,
        observation_policy=observation_policy,
        backend=backend,
        map_policy=map_policy,
        evidence_lifetime_s=evidence_lifetime,
    )
    if policy == "local":
        mapper_type = RollingTerrainMapper if map_policy == "rolling_incremental" else TerrainMapper
        core = LocalReconstructor(
            lambda origin, lifetime: mapper_type(library, width=cells, height=cells, origin=origin, lifetime=lifetime),
            link["request"]["base_from_imu"],
            config,
        )
    else:
        # The running baseline's sources must match their recorded identities.
        for name in (
            "terrain_shadow_core.py",
            "terrain_shadow_runtime.py",
            "shadow_terrain.py",
            "layered_terrain.py",
            "raw_return_filter.py",
            "revision_evidence.py",
        ):
            expected = [digest for path, digest in manifest["source_sha256"].items() if Path(path).name == name]
            if expected != [sha(Path(__file__).with_name(name))]:
                raise ValueError("baseline runtime changed: " + name)
        core = ShadowCore(
            lambda: TerrainMapper(library, **manifest["geometry"]),
            minimum=manifest["minimum"],
            maximum=manifest["maximum"],
        )
    scheduler = ReceiptScheduler(core)
    coords = np.arange(-2.975, 3.0, 0.05)
    x, y = np.meshgrid(coords, coords)
    queries = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    radius = np.linalg.norm(queries[:, :2], axis=1)
    timings, frames, previous, changed, comparisons = [], 0, None, 0, 0
    terminal, limited = None, False
    with (output / "projections.jsonl").open("x") as saved, (source / "native.bin").open("rb") as raw, (
        source / "projections.jsonl"
    ).open() as originals:
        for line in (source / "events.jsonl").open():
            event = json.loads(line)
            kind, payload = event["kind"], event["payload"]
            if terminal is not None:
                raise ValueError("events after terminal fault")
            if kind == "chunk":
                scheduler.receive(kind, decoded_chunk(payload, raw), payload["size"])
            elif kind in ("revision", "complete"):
                scheduler.receive(kind, payload, len(json.dumps(payload)))
            elif kind == "clock":
                result = scheduler.clock(payload)
                if result is not None:
                    if policy == "local":
                        record = core.last_record.copy()
                    else:
                        expected = json.loads(next(originals))
                        status = core.status()
                        if any(
                            status[key] != value for key, value in expected["status"].items() if key != "compute_ms"
                        ):
                            raise ValueError("baseline differs from recorded projection")
                        if core.last_associations != expected["associations"]:
                            raise ValueError("baseline pose associations changed")
                        association = core.timeline.associate(core.watermark_ns, payload, mode="causal")
                        base = (
                            association.map_from_lidar
                            @ np.linalg.inv(core.timeline.extrinsic)
                            @ np.linalg.inv(link["request"]["base_from_imu"])
                        )
                        record = dict(
                            policy="frozen_first_published_30s",
                            evidence_ns=core.watermark_ns,
                            publication_ns=payload,
                            origin=manifest["geometry"]["origin"],
                            local_from_base=base.tolist(),
                            local_from_map=np.eye(4).tolist(),
                            associations=core.last_associations,
                            layers_sha256=status["layers_sha256"],
                            compute_ms=core.last_compute_ms,
                            acceptance=False,
                            navigation_authority=False,
                        )
                    costs, outside = query_grid(result, record["origin"], record["local_from_base"], queries)
                    record["availability"] = {
                        name: availability(costs[mask], outside[mask])
                        for name, mask in (
                            ("square_6m", np.ones(len(queries), bool)),
                            ("radius_0_2m", radius < 2),
                            ("radius_2_3m", (radius >= 2) & (radius < 3)),
                        )
                    }
                    # Body-fixed query changes include true terrain motion;
                    # never label this raw transition rate pure map flicker.
                    if previous is not None:
                        changed += int((previous != costs).sum())
                        comparisons += len(costs)
                    previous = costs.copy()
                    if projection_observer is not None:
                        # Diagnostic sink only, outside core computation. Pass
                        # owned values so it cannot alter the mapper/replay state.
                        projection_observer(result.copy(), json.loads(json.dumps(record, allow_nan=False)))
                    saved.write(json.dumps(record, allow_nan=False) + "\n")
                    timings.append(record["compute_ms"])
                    frames += 1
                    if frames % 20 == 0:
                        check_run(output)
                        print(f"{frames} projections; last {timings[-1]:.1f} ms", flush=True)
                    if resource.getrusage(resource.RUSAGE_SELF).ru_maxrss > 2 * 1024**2:
                        raise ValueError("2 GiB diagnostic memory stop")
                    if max_projections is not None and frames >= max_projections:
                        limited = True
                        break
            elif kind == "fault":
                if not allow_failed or payload != collection["status"]["failure"]:
                    raise ValueError("undeclared source terminal fault")
                terminal = payload
                core.fault(payload)
            else:
                raise ValueError("unsupported event: " + kind)
        if not limited and raw.read(1):
            raise ValueError("unconsumed native tail")
    if not limited:
        if source_failed and terminal != collection["status"]["failure"]:
            raise ValueError("missing source failure event")
        if not source_failed and not core.completed:
            raise ValueError("source ended without local completion")
    for path, expected in hashes.items():
        if sha(path) != expected:
            raise ValueError("input changed during replay")
    if any(sha(path) != expected for path, expected in candidate_hashes.items()):
        raise ValueError("candidate library/build changed during replay")
    if not timings:
        raise ValueError("no local projections")
    report = dict(
        diagnostic_completed=True,
        acceptance=False,
        navigation_authority=False,
        policy=policy,
        observation_policy=observation_policy if policy == "local" else "frozen_original_runtime",
        backend=backend if policy == "local" else "frozen_original_runtime",
        grid_cells=cells if policy == "local" else manifest["geometry"]["width"],
        grid_motion=(
            ("fixed_initial" if map_policy == "fixed_incremental" else "rolling")
            if policy == "local"
            else "fixed_original"
        ),
        map_policy=map_policy if policy == "local" else "frozen_original_runtime",
        bucket_expiry_policy=(
            "persistent_immutable_buckets" if map_policy != "rolling_rebuild" else "reconstructed_window"
        ),
        instrumented=profile or projection_observer is not None,
        projection_observer=projection_observer is not None,
        source_failed=source_failed,
        terminal_fault=terminal,
        bounded_prefix=limited,
        requested_max_projections=max_projections,
        window_s=window,
        evidence_lifetime_s=evidence_lifetime or window,
        projections=frames,
        compute_ms=dict(zip(("median", "p95", "p99"), map(float, np.percentile(timings, [50, 95, 99])))),
        peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
        body_query_class_change_fraction=changed / max(1, comparisons),
        change_limit="includes actual motion; not pure registration flicker",
        terrain_truth_evaluated=False,
        input_sha256={str(p): v for p, v in hashes.items()},
        library_sha256=sha(library),
        original_library_sha256=manifest["library_sha256"],
        candidate_build_sha256=candidate_hashes,
        output_sha256=sha(output / "projections.jsonl"),
    )
    core.close()
    report["closed_status"] = core.status()
    if policy == "first_published":
        # Do not conceal the frozen baseline's known shutdown-status semantics.
        report["closed_status_is_frozen_baseline"] = True
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--window", type=int, choices=(2, 5, 30), required=True)
    parser.add_argument("--diagnose-failed-prefix", action="store_true")
    parser.add_argument("--max-projections", type=int)
    parser.add_argument("--policy", choices=("local", "first_published"), default="local")
    parser.add_argument("--profile", action="store_true", help="stage telemetry, never compute acceptance")
    parser.add_argument(
        "--evidence-lifetime",
        type=int,
        choices=(30, 60),
        help="separate persistent evidence lifetime; rolling_incremental only, never a silent extension",
    )
    parser.add_argument(
        "--map-policy",
        choices=("rolling_rebuild", "fixed_incremental", "rolling_incremental"),
        default="rolling_rebuild",
        help="incremental is a separate frozen-pose algorithm, NOT an exact rebuild optimization",
    )
    parser.add_argument(
        "--candidate-library", type=Path, help="opt-in archived build_local_terrain.py output; local only"
    )
    parser.add_argument(
        "--backend",
        choices=("reference", "prepared"),
        default="reference",
        help="opt-in exact pose index and bounded return cache; no terrain-policy change",
    )
    parser.add_argument(
        "--cells", type=int, choices=(60, 200), default=60, help="local rolling grid only; queries unchanged"
    )
    parser.add_argument(
        "--observation-policy",
        choices=("coherent", "frozen_observation"),
        default="coherent",
        help="freeze first-used local observation placements or apply causal revisions",
    )
    args = parser.parse_args()
    if args.max_projections is not None and args.max_projections <= 0:
        parser.error("max-projections must be positive")
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    sources = [
        Path(__file__),
        *[
            Path(__file__).with_name("local_terrain") / name
            for name in (
                "__init__.py",
                "core.py",
                "evaluation.py",
                "pose_lookup.py",
                "return_cache.py",
                "rolling_mapper.py",
            )
        ],
    ]
    sources += [
        Path(__file__).with_name(name)
        for name in (
            "shadow_terrain.py",
            "revision_evidence.py",
            "raw_return_filter.py",
            "native_chunk_transport.py",
            "terrain_shadow_runtime.py",
            "terrain_shadow_core.py",
            "layered_terrain.py",
            "archive_guard.py",
            "build_local_terrain.py",
        )
    ]
    provenance = {str(path.resolve()): sha(path) for path in sources}
    for path in sources:
        relative = path.resolve().relative_to(Path(__file__).resolve().parent)
        target = output / "source" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    with (output / "manifest.json").open("x") as stream:
        json.dump(
            dict(
                arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                source_sha256=provenance,
            ),
            stream,
            indent=2,
        )
    try:
        result = replay(
            args.source,
            args.capture,
            args.archive_root,
            output,
            args.window,
            args.diagnose_failed_prefix,
            args.max_projections,
            args.policy,
            args.profile,
            args.cells,
            args.observation_policy,
            args.backend,
            args.candidate_library,
            map_policy=args.map_policy,
            evidence_lifetime=args.evidence_lifetime,
        )
        if any(sha(path) != expected for path, expected in provenance.items()):
            raise ValueError("candidate sources changed during replay")
        with (output / "report.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps({k: v for k, v in result.items() if k != "input_sha256"}, indent=2))
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
