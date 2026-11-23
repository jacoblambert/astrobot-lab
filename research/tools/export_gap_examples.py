#!/usr/bin/env python3
"""Export fixed-time ACTUAL recorded class grids; failed sources stay diagnostic.

No reconstructed labels or interpolated maps. Validate each saved grid's full
map pose against the matching immutable projection receipt. Preserve missing
grids, unreached display times, pre-projection unknown and terminal invalidation.
Display selection is fixed, never selected for favorable coverage.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import label

from archive_guard import allocate_run, check_run
from export_shadow_trajectory import pose_matrix, stamp_ns
from local_terrain.costmap import map_from_grid
from replay_local_terrain import sha
from revision_evidence import validated_transform

DISPLAY_SECONDS = (0, 5, 15, 30, 60, 120, 180, 240)


def choose_examples(stamps):
    stamps = sorted(stamps)
    if not stamps or len(set(stamps)) != len(stamps):
        raise ValueError("nonempty unique grid stamps required")
    chosen, missing = {}, []
    for seconds in DISPLAY_SECONDS:
        index = np.searchsorted(stamps, stamps[0] + seconds * 10**9)
        if index == len(stamps):
            missing.append(seconds)
        else:
            chosen.setdefault(stamps[index], []).append(f"+{seconds}s")
    chosen.setdefault(stamps[-1], []).append("last projection receipt")
    return chosen, missing


def gap_summary(classes, origin, local_from_base, resolution):
    classes = np.asarray(classes)
    pose = validated_transform(local_from_base)
    if classes.ndim != 2 or not np.isin(classes, [-1, 0, 20, 100]).all() or resolution <= 0:
        raise ValueError("invalid class grid")
    unknown = classes == -1
    iy, ix = np.indices(classes.shape)
    centers = np.column_stack(
        ((ix.ravel() + 0.5) * resolution + origin[0], (iy.ravel() + 0.5) * resolution + origin[1], np.zeros(ix.size))
    )
    relative = (centers - pose[:3, 3]) @ pose[:3, :3]
    near = (np.abs(relative[:, :2]) <= 1.5).all(axis=1).reshape(classes.shape)
    ij = np.floor((pose[:2, 3] - origin) / resolution).astype(int)
    component = None
    if 0 <= ij[0] < classes.shape[1] and 0 <= ij[1] < classes.shape[0] and unknown[ij[1], ij[0]]:
        components, _ = label(unknown, structure=np.ones((3, 3)))
        ys, xs = np.nonzero(components == components[ij[1], ij[0]])
        component = dict(
            cells=len(xs),
            area_m2=len(xs) * resolution**2,
            width_m=(xs.max() - xs.min() + 1) * resolution,
            height_m=(ys.max() - ys.min() + 1) * resolution,
            grid_bbox=[int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)],
            touches_grid_edge=bool(
                xs.min() == 0 or ys.min() == 0 or xs.max() == classes.shape[1] - 1 or ys.max() == classes.shape[0] - 1
            ),
        )
    return dict(
        cells=classes.size,
        unknown=int(unknown.sum()),
        occupied=int((classes == 100).sum()),
        measured=int((classes == 0).sum()),
        inferred=int((classes == 20).sum()),
        near_3m_cells=int(near.sum()),
        near_3m_unknown=int((unknown & near).sum()),
        base_center_unknown_component=component,
        terrain_truth_evaluated=False,
        base_center_class=(
            int(classes[ij[1], ij[0]]) if 0 <= ij[0] < classes.shape[1] and 0 <= ij[1] < classes.shape[0] else None
        ),
    )


def export(recorder, mapper, output, *, allow_failed=False, grid_observer=None):
    collection = json.loads((recorder / "collection.json").read_text())
    sealed = json.loads((mapper / "collection.json").read_text())
    manifest = json.loads((mapper / "manifest.json").read_text())
    failed = not collection["collection_completed"] or not sealed["experiment_completed"]
    if failed and not allow_failed:
        raise ValueError("failed source requires explicit --diagnose-failed-prefix")
    recorded_status = collection["terrain_shadow_status"]
    if (
        Path(recorded_status["archive"]).name != mapper.name
        or recorded_status["capture"] != manifest["request"]["capture"]
        or sealed["status"]["capture"] != recorded_status["capture"]
    ):
        raise ValueError("recorder/mapper identity mismatch")
    paths = [
        recorder / "collection.json",
        recorder / "bag/metadata.yaml",
        mapper / "collection.json",
        mapper / "manifest.json",
        mapper / "projections.jsonl",
        Path(__file__),
        Path(__file__).with_name("local_terrain") / "costmap.py",
        Path(__file__).with_name("export_shadow_trajectory.py"),
    ]
    paths.extend(sorted((recorder / "bag").glob("*.mcap")))
    hashes = {str(p): sha(p) for p in paths}
    if hashes[str(mapper / "projections.jsonl")] != sealed["output_sha256"]["projections.jsonl"]:
        raise ValueError("projection receipts changed")
    records = {}
    compute_ms = []
    for line in (mapper / "projections.jsonl").open():
        record = json.loads(line)
        stamp = record["evidence_ns"]
        if stamp in records:
            raise ValueError("duplicate projection receipt")
        records[stamp] = {
            key: record[key]
            for key in ("evidence_ns", "publication_ns", "origin", "resolution", "local_from_map", "local_from_base")
        }
        compute_ms.append(record["compute_ms"])
    terminal = sealed["status"]["evidence_ns"] if sealed["status"]["completed"] else None
    expected = sorted(set(records) - {terminal})
    choices, unreached = choose_examples(expected)
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import OccupancyGrid

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(recorder / "bag"), storage_id="mcap"), ConverterOptions("", ""))
    reader.set_filter(StorageFilter(topics=["/terrain_shadow/classes"]))
    seen, examples = set(), []
    preprojection = duplicates = invalidations = 0
    last_recorded = None

    def save_example(stamp, classes, receipt, labels):
        record = records[stamp]
        summary = gap_summary(classes, np.asarray(record["origin"]), record["local_from_base"], record["resolution"])
        name = f"grid-{stamp}.npz"
        np.savez_compressed(
            output / name,
            classes=classes,
            origin=record["origin"],
            local_from_base=record["local_from_base"],
            local_from_map=record["local_from_map"],
            resolution=record["resolution"],
        )
        return dict(
            labels=list(labels),
            elapsed_s=(stamp - expected[0]) / 1e9,
            evidence_ns=stamp,
            publication_ns=record["publication_ns"],
            bag_receipt_ns=receipt,
            file=name,
            sha256=sha(output / name),
            **summary,
        )

    while reader.has_next():
        _, raw, receipt = reader.read_next()
        message = deserialize_message(raw, OccupancyGrid)
        stamp = stamp_ns(message.header.stamp)
        if stamp not in records:
            preprojection += 1
            continue
        if stamp == terminal:
            if any(value != -1 for value in message.data):
                raise ValueError("completion published a stale valid grid")
            invalidations += 1
            continue
        if stamp in seen:
            duplicates += 1
            continue  # First publication at this projection; later fault/shutdown stays in MCAP.
        record = records[stamp]
        if (
            message.header.frame_id != "map"
            or message.info.width != 60
            or message.info.height != 60
            or abs(message.info.resolution - record["resolution"]) > 1e-7
            or not np.allclose(pose_matrix(message.info.origin), map_from_grid(record), atol=1e-7, rtol=0)
        ):
            raise ValueError("recorded grid frame/geometry mismatch")
        seen.add(stamp)
        classes = np.asarray(message.data, dtype=np.int8).reshape(60, 60)
        if not np.isin(classes, [-1, 0, 20, 100]).all():
            raise ValueError("invalid class in recorded grid")
        if grid_observer is not None:
            grid_observer(classes.copy(), record)
        if last_recorded is None or stamp > last_recorded[0]:
            last_recorded = (stamp, classes, receipt)
        if stamp not in choices:
            continue
        examples.append(save_example(stamp, classes, receipt, choices[stamp]))
    if last_recorded is not None:
        matches = [e for e in examples if e["evidence_ns"] == last_recorded[0]]
        if matches:
            matches[0]["labels"].append("last recorded grid")
        else:
            examples.append(save_example(*last_recorded, ["last recorded grid; later projection receipt missing"]))
    missing = sorted(set(expected) - seen)
    if any(sha(path) != digest for path, digest in hashes.items()):
        raise ValueError("input changed during grid export")
    return dict(
        diagnostic_completed=True,
        acceptance=False,
        source_completed=not failed,
        failed_prefix_only=failed,
        source_failure=collection["failure"],
        mapper_failure=sealed["status"]["failure"],
        recorder=str(recorder),
        mapper=str(mapper),
        pattern=collection["natural_motion"],
        expected_projections=len(expected),
        recorded_projections=len(seen),
        missing_projection_stamps=missing,
        missing_selected_projection_stamps=sorted(set(choices) - seen),
        unreached_display_seconds=unreached,
        preprojection_messages=preprojection,
        repeated_stamp_messages=duplicates,
        completion_unknown_messages=invalidations,
        projection_span_s=(max(records) - min(records)) / 1e9,
        compute_ms=dict(zip(("median", "p95", "p99", "max"), map(float, np.percentile(compute_ms, [50, 95, 99, 100])))),
        peak_mapper_rss_mib=recorded_status["peak_process_rss_mib"],
        examples=examples,
        input_sha256=hashes,
        note="Actual recorded class grids, not inflated costs. No obstacle truth/recall or navigation qualification. Missing and unexecuted evidence remains visible.",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recorder", type=Path)
    parser.add_argument("mapper", type=Path)
    parser.add_argument("--diagnose-failed-prefix", action="store_true")
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    for path in (args.recorder, args.mapper):
        check_run(path)
    # Derived grid examples do not need a raw-capture-sized quota.
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        report = export(args.recorder, args.mapper, output, allow_failed=args.diagnose_failed_prefix)
        check_run(output)
        with (output / "report.json").open("x") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
        check_run(output)
        print(json.dumps({k: v for k, v in report.items() if k not in ("input_sha256", "examples")}, indent=2))
    except BaseException as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), acceptance=False), stream)
        raise


if __name__ == "__main__":
    main()
