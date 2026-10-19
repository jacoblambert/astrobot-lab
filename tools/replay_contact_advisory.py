#!/usr/bin/env python3
"""Exact live-advisory replay and plain movement/runtime accounting. No commands."""

import argparse
import json
import shutil
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from assess_contact_support import mapper_revisions
from local_terrain.contact_runtime import ContactRuntime, verify_contact_seal
from replay_live_local_terrain import replay
from replay_local_terrain import sha


def without_timing(row):
    return {k: v for k, v in row.items() if k not in ("update_ms", "total_ms")}


def replay_policy(contact, *, candidate, support_core=None, startup_turn_area=None, current_turn_area=None):
    """Unspecified candidate settings retain the source, avoiding hidden ablations."""
    overrides = dict(support_core=support_core, startup_turn_area=startup_turn_area)
    if "current_turn_area" in contact or current_turn_area is not None:
        overrides["current_turn_area"] = current_turn_area
    if any(v is not None and type(v) is not bool for v in overrides.values()):
        raise ValueError("explicit boolean candidate policy required")
    if not candidate and any(v is not None for v in overrides.values()):
        raise ValueError("policy overrides require candidate mode")
    return {key: bool(contact.get(key, False)) if value is None else value for key, value in overrides.items()}


def summarize(rows, records, *, offline=False):
    valid = [r for r in rows if r["valid"]]
    batches = [r for r in valid if r["motions"] is not None]
    if len(valid) != len(records) - 1 or not batches:
        raise ValueError("missing live advisory update or query batch")
    first = valid[0]["query_ns"]
    counts = {}
    for name, selected in (
        ("all", batches),
        ("after_15s", [r for r in batches if r["query_ns"] - first >= 15 * 10**9]),
    ):
        counts[name] = dict(
            decisions=len(selected),
            standing_available=sum(r["motions"]["stationary"]["state"] == "available" for r in selected),
            short_move_or_turn_available=sum(
                any(v["state"] == "available" for n, v in r["motions"].items() if n != "stationary") for r in selected
            ),
            per_candidate={
                n: sum(r["motions"][n]["state"] == "available" for r in selected) for n in batches[0]["motions"]
            },
        )
    mapper = {r["evidence_ns"]: r for r in records}
    distributions = {
        "mapper_ms": [mapper[r["evidence_ns"]]["compute_ms"] for r in valid],
        "advisory_update_ms": [r["update_ms"] for r in valid],
        "advisory_total_ms": [r["total_ms"] for r in valid],
        "mapper_plus_advisory_ms": [mapper[r["evidence_ns"]]["compute_ms"] + r["total_ms"] for r in valid],
    }
    if offline:
        # Do not add a recorded mapper interval to a new offline query interval
        # and present the sum as a measured coupled workload.
        distributions.pop("mapper_plus_advisory_ms")
        distributions["recorded_mapper_reference_ms"] = distributions.pop("mapper_ms")
    return dict(
        valid_maps=len(valid),
        movement=counts,
        largest_history_anchors=max(r["history_anchors"] for r in valid),
        maps_with_any_lidar_confirmed_wheel=sum(any(r["lidar_confirmed_wheels"]) for r in valid),
        timing={
            name: dict(zip(("p50", "p95", "p99", "max"), map(float, np.percentile(values, [50, 95, 99, 100]))))
            for name, values in distributions.items()
        },
        timing_origin="offline_advisory_with_recorded_mapper_reference" if offline else "live_instrumented",
        note=(
            "Offline advisory timings cannot establish live improvement. No mixed live/offline total is reported."
            if offline
            else "Live instrumented processing intervals only; excludes callback queueing and is not both resource profiles. Availability is not independent safety or goal completion."
        ),
    )


def evaluate(
    source,
    archive,
    output,
    *,
    candidate=False,
    support_core=None,
    startup_turn_area=None,
    current_turn_area=None,
    observation_callback=None,
):
    source = source.resolve(strict=True)
    seal = verify_contact_seal(archive, dict(archive=str(source)))
    manifest = json.loads((source / "manifest.json").read_text())
    contact = json.loads((source / "contact_manifest.json").read_text())
    paths = [
        source / name
        for name in (
            "contact_manifest.json",
            "contact_collection.json",
            "contact_advisory.jsonl",
            "events.jsonl",
            "projections.jsonl",
        )
    ]
    hashes = {str(p): sha(p) for p in paths}
    root = Path(__file__).parent
    sources = [Path(__file__), root / "replay_live_local_terrain.py", root / "terrain_entrypoint_archive.py"]
    sources += [
        root / "local_terrain" / name
        for name in (
            "contact_runtime.py",
            "contact_advisory.py",
            "contact_support.py",
            "contact_query.py",
            "contact_index.py",
            "control_context.py",
            "pose_lookup.py",
            "support_sweep.py",
            "physx_reference.py",
        )
    ]
    for path in sources:
        hashes[str(path)] = sha(path)
        target = output / "source" / path.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    rows = [json.loads(line) for line in (source / "contact_advisory.jsonl").open()]
    expected = iter([r for r in rows if r["valid"]])
    records = [json.loads(line) for line in (source / "projections.jsonl").open()]
    if len([r for r in rows if r["valid"]]) != seal["frames"]:
        raise ValueError("advisory seal/update denominator disagreement")
    emitted = []
    node = SimpleNamespace(
        run=output,
        hashes={},
        core=SimpleNamespace(
            config=SimpleNamespace(**manifest["config"]),
            base_from_imu=np.asarray(manifest["request"]["base_from_imu"]),
            completed=True,
            failure=None,
            last_clock_ns=-1,
        ),
        create_publisher=lambda *args: SimpleNamespace(publish=lambda m: emitted.append(json.loads(m.data))),
        get_logger=lambda: SimpleNamespace(error=lambda text: print(text, flush=True)),
    )
    runtime = ContactRuntime(
        node,
        deployment_confirmed=contact["deployment_confirmed"],
        hazard_only=contact.get("hazard_only", False),
        **replay_policy(
            contact,
            candidate=candidate,
            support_core=support_core,
            startup_turn_area=startup_turn_area,
            current_turn_area=current_turn_area,
        ),
    )
    hashes.update(node.hashes)
    revisions = mapper_revisions(source / "events.jsonl")
    pending = next(revisions, None)
    compared = 0
    changes = []

    def observe(layers, record, *, published_valid):
        nonlocal pending, compared
        if not published_valid:
            return
        # Preserve EACH original callback clock fence, including clocks without
        # map output, rather than assigning every revision to the next map.
        while pending is not None and pending["arrival_ns"] <= record["publication_ns"]:
            stamp = pending["arrival_ns"]
            runtime.receive({k: v for k, v in pending.items() if k != "arrival_ns"})
            pending = next(revisions, None)
            if pending is None or pending["arrival_ns"] != stamp:
                runtime.clock(stamp, None, None)
        runtime.clock(record["publication_ns"], layers, record)
        original = next(expected)
        if runtime.failure or not emitted:
            raise ValueError("live/replay contact decision mismatch at map " + str(compared))
        actual = emitted[-1]
        if candidate:
            for field in ("query_ns", "pose_ns", "evidence_ns", "session", "epoch", "revision"):
                if actual[field] != original[field]:
                    raise ValueError("candidate changed pose/time exposure")
            if (actual["motions"] is None) != (original["motions"] is None):
                raise ValueError("candidate changed query denominator")
            if actual["motions"] is not None:
                for name, decision in actual["motions"].items():
                    before = original["motions"][name]["state"]
                    if before != decision["state"]:
                        changes.append(
                            dict(
                                query_ns=record["publication_ns"],
                                candidate=name,
                                before=before,
                                after=decision["state"],
                            )
                        )
        elif without_timing(actual) != without_timing(original):
            raise ValueError("live/replay contact decision mismatch at map " + str(compared))
        compared += 1
        if observation_callback is not None:
            observation_callback(runtime, layers, record, actual)

    try:
        exact = replay(source, archive, projection_observer=observe)
        if next(expected, None) is not None or compared != seal["frames"]:
            raise ValueError("missing contact replay tail")
    finally:
        runtime.finish()
    if any(sha(p) != h for p, h in hashes.items()):
        raise ValueError("advisory source changed during replay")
    return dict(
        completed=True,
        exact_advisory_decisions=not candidate,
        changed_policy_diagnostic=candidate,
        state_changes=changes,
        compared_maps=compared,
        exact_raw_map_replay=exact,
        live=summarize(rows, records),
        candidate=summarize(emitted, records, offline=True) if candidate else None,
        input_sha256=hashes,
        acceptance=False,
        navigation_authority=False,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument(
        "--candidate",
        action="store_true",
        help="Explicit changed-advisory comparison; original raw maps and exposures remain exact",
    )
    parser.add_argument(
        "--support-core",
        action="store_true",
        default=None,
        help="Candidate-only smaller positive-support region, not qualification",
    )
    parser.add_argument(
        "--startup-turn-area", action="store_true", default=None, help="Candidate-only fixed startup turning circle"
    )
    parser.add_argument(
        "--current-turn-area",
        action="store_true",
        default=None,
        help="Candidate-only turning assumption at the actual current pose",
    )
    args = parser.parse_args()
    if args.support_core and not args.candidate:
        parser.error("--support-core requires --candidate")
    if args.startup_turn_area and not args.candidate:
        parser.error("--startup-turn-area requires --candidate")
    if args.current_turn_area and not args.candidate:
        parser.error("--current-turn-area requires --candidate")
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        report = evaluate(
            args.source,
            args.archive_root,
            output,
            candidate=args.candidate,
            support_core=args.support_core,
            startup_turn_area=args.startup_turn_area,
            current_turn_area=args.current_turn_area,
        )
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps({k: report[k] for k in ("live", "candidate", "state_changes")}, indent=2), flush=True)
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
