#!/usr/bin/env python3
"""Matched one-factor local diagnostics; never terrain or runtime qualification."""

import argparse
from itertools import zip_longest
import json
from pathlib import Path

from archive_guard import ARCHIVE_ROOT, allocate_run
from build_local_terrain import validate_build
from replay_local_terrain import sha

FACTORS = ("window_s", "grid_cells", "observation_policy", "backend")


def compare(left, right, *, exact=False):
    runs = [Path(left).resolve(strict=True), Path(right).resolve(strict=True)]
    hashes, reports, manifests = {}, [], []
    for run in runs:
        for name in ("manifest.json", "report.json", "projections.jsonl"):
            hashes[str(run / name)] = sha(run / name)
        report = json.loads((run / "report.json").read_text())
        manifest = json.loads((run / "manifest.json").read_text())
        if not report["diagnostic_completed"] or report["output_sha256"] != hashes[str(run / "projections.jsonl")]:
            raise ValueError("incomplete or modified diagnostic")
        if (
            report["policy"] != "local"
            or report["grid_motion"] != "rolling"
            or report.get("map_policy", "rolling_rebuild") != "rolling_rebuild"
        ):
            raise ValueError("matched ablations require the same local reconstruction path")
        if report["instrumented"]:
            raise ValueError("compare uninstrumented arms; profiling is a separate experiment")
        # Verify retained dependency bytes, not today's possibly changed workspace.
        for path, digest in manifest["source_sha256"].items():
            parts = Path(path).parts
            relative = Path(*parts[parts.index("tools") + 1 :])
            retained = run / "source" / relative
            if sha(retained) != digest:
                raise ValueError("retained candidate dependency changed")
            hashes[str(retained)] = digest
        reports.append(report)
        manifests.append(manifest)
        for path, expected in report.get("candidate_build_sha256", {}).items():
            if sha(path) != expected:
                raise ValueError("retained candidate build changed")
            hashes[path] = expected
        build_hashes = report.get("candidate_build_sha256", {})
        if build_hashes:
            libraries = [
                Path(p) for p, h in build_hashes.items() if Path(p).suffix == ".so" and h == report["library_sha256"]
            ]
            if len(libraries) != 1:
                raise ValueError("candidate build does not identify the replay library")
            library = libraries[0]
            verified = validate_build(library, library.parent.parent)
            if verified != build_hashes:
                raise ValueError("candidate build provenance is incomplete")
    a, b = reports
    for key in ("source_sha256",):
        if not exact and manifests[0][key] != manifests[1][key]:
            raise ValueError("candidate sources differ between arms")
    for key in (
        "input_sha256",
        "source_failed",
        "terminal_fault",
        "bounded_prefix",
        "requested_max_projections",
        "projections",
    ):
        if a[key] != b[key]:
            raise ValueError("unmatched replay contract: " + key)
    if a["library_sha256"] != b["library_sha256"]:
        if not exact or not (a.get("candidate_build_sha256") or b.get("candidate_build_sha256")):
            raise ValueError("different libraries require explicit exact comparison and retained build provenance")
    # Pre-backend ablation runs are the explicit naive reference implementation.
    factors = [key for key in FACTORS if a.get(key, "reference") != b.get(key, "reference")]
    if exact:
        if set(factors) - {"backend"}:
            raise ValueError("exact implementation comparison cannot change algorithm factors")
        factor = factors[0] if factors else "implementation"
    else:
        if len(factors) != 1:
            raise ValueError("comparison requires exactly one changed factor")
        factor = factors[0]
    summaries = [{}, {}]
    count, equal_layers = 0, 0
    with (runs[0] / "projections.jsonl").open() as left_stream, (runs[1] / "projections.jsonl").open() as right_stream:
        for lines in zip_longest(left_stream, right_stream):
            if None in lines:
                raise ValueError("missing projection tail")
            rows = [json.loads(line) for line in lines]
            for key in (
                "evidence_ns",
                "publication_ns",
                "session",
                "epoch",
                "revision",
                "local_from_base",
                "local_from_map",
            ):
                if rows[0][key] != rows[1][key]:
                    raise ValueError("unmatched publication/current-rover state: " + key)
            if factor != "window_s":
                selected = [[(p["sequence"], p["acquisition_ns"]) for p in row["associations"]] for row in rows]
                if selected[0] != selected[1]:
                    raise ValueError("observation dose changed")
            if rows[0]["availability"].keys() != rows[1]["availability"].keys():
                raise ValueError("query regions changed")
            for name in rows[0]["availability"]:
                if rows[0]["availability"][name]["queries"] != rows[1]["availability"][name]["queries"]:
                    raise ValueError("query denominator changed")
                for summary, row in zip(summaries, rows):
                    values = row["availability"][name]
                    totals = summary.setdefault(
                        name, {key: 0 for key in ("queries", "free", "occupied", "unknown", "outside")}
                    )
                    for key in totals:
                        totals[key] += values[key]
            equal_layers += rows[0]["layers_sha256"] == rows[1]["layers_sha256"]
            if factor == "backend" or exact:
                for key in ("layers_sha256", "associations", "availability", "origin"):
                    if rows[0][key] != rows[1][key]:
                        raise ValueError("exact backend equivalence failed: " + key)
            count += 1
    if count != a["projections"] or count == 0:
        raise ValueError("projection exposure count mismatch")
    for summary in summaries:
        for values in summary.values():
            values["free_fraction"] = values["free"] / values["queries"]
    if any(sha(path) != digest for path, digest in hashes.items()):
        raise ValueError("comparison inputs changed")
    return dict(
        acceptance=False,
        terrain_truth_evaluated=False,
        runtime_qualified=False,
        factor=factor,
        exact_implementation_comparison=exact,
        values=(
            [str(run) for run in runs]
            if factor == "implementation"
            else [a.get(factor, "reference"), b.get(factor, "reference")]
        ),
        projections=count,
        identical_layers=equal_layers,
        source_failed=a["source_failed"],
        bounded_prefix=a["bounded_prefix"],
        availability=summaries,
        free_fraction_delta={
            name: summaries[1][name]["free_fraction"] - summaries[0][name]["free_fraction"] for name in summaries[0]
        },
        compute_ms=[r["compute_ms"] for r in reports],
        peak_rss_mib=[r["peak_rss_mib"] for r in reports],
        input_sha256=hashes,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("left", type=Path)
    parser.add_argument("right", type=Path)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument(
        "--exact",
        action="store_true",
        help="allow different retained implementation sources but require exact outputs and unchanged policy",
    )
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = compare(args.left, args.right, exact=args.exact)
        result["source_sha256"] = {str(Path(__file__).resolve()): sha(Path(__file__))}
        with (output / "comparison.json").open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps({k: v for k, v in result.items() if k != "input_sha256"}, indent=2))
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise
