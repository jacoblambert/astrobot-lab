#!/usr/bin/env python3
"""Matched, receipt-causal support-memory experiment on completed live inputs.

Never changes original maps/scores. Grounded deployment/traversal is an explicit
assumption of these guarded recordings, not a GT label supplied to the policy.
XY uncertainty is a declared sensitivity parameter, not a calibrated guarantee.
"""

import argparse
import json
from pathlib import Path
import time
import resource
import shutil

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import allocate_run, check_run
from assess_navigation_gaps import candidate_motions, motion_score
from export_shadow_trajectory import apply_record
from local_terrain.contact_support import ContactMemory, SupportIndex, convex, patch_polygons, supported_motion
from local_terrain.bookkeeping import BookkeepingLedger
from local_terrain.physx_reference import BodyBox
from local_terrain.pose_lookup import PublicationPoseLookup
from replay_live_local_terrain import replay
from replay_local_terrain import sha
from revision_evidence import validated_transform
from shadow_terrain import PoseTimeline


def example_geometry(record, layers, polygons, poses, patches):
    """Small fixed-time visualization payload; no changed decisions or labels."""
    from shapely import Polygon, box, difference, intersection, union_all
    from local_terrain.support_sweep import swept_polygons

    wheel = union_all([Polygon(p) for p in swept_polygons(poses, patches)], grid_size=0)
    support = union_all([Polygon(p) for p, _ in polygons], grid_size=0)
    unknown_parts = []
    for y, x in np.argwhere(layers[:, :, 0] == -1):
        low = np.asarray(record["origin"]) + np.array([x, y]) * record["resolution"]
        cell = box(*low, *(low + record["resolution"]))
        part = intersection(wheel, cell, grid_size=0)
        if not part.is_empty:
            unknown_parts.append(part)
    gap = difference(union_all(unknown_parts, grid_size=0), support, grid_size=0)

    def coordinates(shape):
        if shape.is_empty:
            return []
        if shape.geom_type == "Polygon":
            return [
                dict(
                    outer=list(map(list, shape.exterior.coords)),
                    holes=[list(map(list, r.coords)) for r in shape.interiors],
                )
            ]
        return [part for g in getattr(shape, "geoms", []) for part in coordinates(g)]

    return dict(
        classes=layers[:, :, 0].astype(int).tolist(),
        origin=record["origin"],
        resolution=record["resolution"],
        local_from_base=record["local_from_base"],
        support=coordinates(support),
        wheels=coordinates(wheel),
        gap=coordinates(gap),
        gap_area_m2=float(gap.area),
        note="Geometric unobserved area only; explicit terrain conflicts remain separate vetoes.",
    )


def extended_motions(start):
    motions = candidate_motions(start)
    for distance in (0.05, 0.10, 0.25, 0.50):
        for sign in (-1, 1):
            delta = np.eye(4)
            delta[0, 3] = sign * distance
            motions[f"{'reverse' if sign < 0 else 'forward'}_{distance:.2f}"] = [start, start @ delta]
    for degrees in (-15, -5, 5, 15):
        delta = np.eye(4)
        delta[:3, :3] = Rotation.from_euler("z", degrees, degrees=True).as_matrix()
        motions[f"turn_{degrees:+d}deg"] = [start, start @ delta]
    return motions


def retraced_motion(start, current_ns, anchors, resolve):
    """Separate recovery diagnostic: reverse the recent estimated actual path.

    Count only a net 5 cm withdrawal or 5 degree heading change, not pose jitter.
    Never connect missing history or extrapolate a skid-free rotation. This
    checks geometry only: a tracking controller still needs validation.
    """
    result, previous = [start], current_ns
    start_yaw = np.arctan2(start[1, 0], start[0, 0])
    for anchor in list(reversed(anchors))[:100]:
        stamp = anchor.stamp_ns
        if stamp >= current_ns:
            continue
        if previous - stamp > 200_000_000:
            return None
        resolved = resolve(stamp)
        if resolved is None:
            return None
        pose = resolved[0]
        result.append(pose)
        previous = stamp
        angle = np.arctan2(pose[1, 0], pose[0, 0]) - start_yaw
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        if np.linalg.norm(pose[:2, 3] - start[:2, 3]) >= 0.05 or abs(angle) >= np.deg2rad(5):
            return result
    return None


def mapper_revisions(path):
    """Use the mapper's receipts, not another ROS subscriber's callback order."""
    pending = []
    with path.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event["kind"] == "revision":
                pending.append(event["payload"])
            elif event["kind"] == "clock":
                for row in pending:
                    yield dict(row, arrival_ns=event["payload"])
                pending.clear()
    if pending:
        raise ValueError("unbounded revision receipt tail")


def apply_mapper_revision(timeline, bookkeeping, row):
    if row["kind"] in (2, 3):
        payload = {k: v for k, v in row.items() if k != "arrival_ns"}
        bookkeeping.accept(payload, row["arrival_ns"], timeline)
    else:
        if row["kind"] != 1 or row["snapshot"]:
            raise ValueError("unsupported mapper revision")
        apply_record(timeline, row)
        bookkeeping.commit_odom([timeline.frames[f["frame_id"]][-1][2] for f in row["frames"]])


def compact_pose_history(timeline, max_frames):
    """After assimilating a publication, only current/future queries are allowed.

    Retain the latest pose for EVERY historical frame, not every past version of
    each pose. Measurement history remains available for old contact anchors.
    """
    if len(timeline.frames) > max_frames:
        raise ValueError("support pose history budget exceeded")
    for frame, history in timeline.frames.items():
        timeline.frames[frame] = history[-1:]
    timeline.events = dict(list(timeline.events.items())[-128:])


def evaluate(assessment, trajectory, archive, output, uncertainty, support_core=False):
    import shapely
    from local_terrain.contact_query import PreparedMotion, evaluate_support_core, refine_trajectory
    from local_terrain.contact_index import IncrementalSupportIndex

    report = json.loads((assessment / "report.json").read_text())
    if not report["diagnostic_completed"] or report["grids"]["missing_projection_stamps"]:
        raise ValueError("complete baseline map assessment required")
    source = archive / Path(report["grids"]["mapper"]).name
    manifest = json.loads((source / "manifest.json").read_text())
    exported = json.loads((trajectory / "shadow_trajectory.json").read_text())
    if not exported["passed"] or Path(exported["source"]).name != Path(report["grids"]["recorder"]).name:
        raise ValueError("trajectory belongs to another recording")
    recorder = archive / Path(exported["source"]).name
    for name, digest in exported["input_sha256"].items():
        if Path(name).is_absolute() or ".." in Path(name).parts or sha(recorder / name) != digest:
            raise ValueError("exported recording input changed")
    paths = [
        assessment / "report.json",
        assessment / "motion_scores.jsonl",
        trajectory / "trajectory.jsonl",
        trajectory / "shadow_trajectory.json",
        source / "manifest.json",
        Path(__file__),
        Path(__file__).parent / "local_terrain/contact_support.py",
        Path(__file__).parent / "local_terrain/contact_query.py",
        Path(__file__).parent / "local_terrain/contact_index.py",
        Path(__file__).parent / "local_terrain/support_sweep.py",
        Path(__file__).parent / "local_terrain/pose_lookup.py",
        source / "events.jsonl",
    ]
    hashes = {str(p): sha(p) for p in paths}
    tools_root = Path(__file__).resolve().parent
    for path in paths:
        if path.resolve().is_relative_to(tools_root):
            target = output / "source" / path.resolve().relative_to(tools_root)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
    geometry_dir = Path(shapely.__file__).parent
    for path in [*geometry_dir.glob("*.so"), *(geometry_dir.parent / "shapely.libs").glob("*.so*")]:
        hashes[str(path)] = sha(path)
    if (
        sha(paths[1]) != report["output_sha256"]["motion_scores.jsonl"]
        or sha(paths[2]) != exported["trajectory_sha256"]
    ):
        raise ValueError("changed scores or trajectory")
    baseline = {r["evidence_ns"]: r for r in map(json.loads, paths[1].read_text().splitlines())}
    revisions = mapper_revisions(source / "events.jsonl")
    pending = next(revisions, None)
    timeline = PoseTimeline(exported["session"], exported["epoch"])
    bookkeeping = BookkeepingLedger(manifest["config"]["max_bookkeeping_frames"])
    patches = [BodyBox(p["center"], p["half_extent"], "baseline_assumed_wheel") for p in report["contract"]["patches"]]
    query_patches = (
        [BodyBox(p.center, (0.02, 0.03, 0.001), "experimental_support_core_40x60mm") for p in patches]
        if support_core
        else patches
    )
    body = BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "unchanged_baseline_body_guard")
    # Audited chassis XY plus the declared wheel-patch extents, not the inflated
    # acquisition guard. Keep this declared assumption visible in every report.
    half_x = max(0.50370, max(abs(p.center[0]) + p.half_extent[0] for p in patches))
    half_y = max(0.28545, max(abs(p.center[1]) + p.half_extent[1] for p in patches))
    footprint = np.array([[-half_x, -half_y], [half_x, -half_y], [half_x, half_y], [-half_x, half_y]])
    memory = ContactMemory(patches, footprint)
    imu_from_base = np.linalg.inv(validated_transform(manifest["request"]["base_from_imu"]))
    rows, examples, first, compared = [], [], None, 0
    indices = {name: IncrementalSupportIndex() for name in ("current", "startup", "tracks")}
    contract = dict(
        policy="contact_support_v1_diagnostic",
        grounded_assumption=True,
        startup_half_extent_xy_m=[half_x, half_y],
        history_xy_erosion_m=uncertainty,
        positive_support_policy="core_40x60mm" if support_core else "full_80x120mm",
        full_wheel_hazard_veto=True,
        support_core_physically_qualified=False,
        recovery_diagnostic="reverse recent actual estimated path; net >=5cm or >=5deg; not goal progress",
        recovery_core_envelope_angle_step_rad=0.005,
        uncertainty_calibrated=False,
        history_capacity=4096,
        history_time_expiry=False,
        input="recorded GLIM poses and original terrain; no GT support labels",
        query_sampling="unchanged baseline 1 Hz bins; contact samples every valid map",
        geometry_backend="incremental_geos_double_precision_union",
        shapely_version=shapely.__version__,
        geos_version=shapely.geos_version_string,
        pose_receipt_source="mapper events",
        future_pose_revisions=False,
        navigation_authority=False,
        acceptance=False,
    )
    (output / "manifest.json").write_text(json.dumps(dict(contract=contract, input_sha256=hashes), indent=2))
    with (output / "scores.jsonl").open("x") as sink:

        def observe(layers, record, *, published_valid):
            nonlocal pending, first, compared
            if not published_valid:
                return
            stamp, publication = record["evidence_ns"], record["publication_ns"]
            while pending is not None and pending["arrival_ns"] <= publication:
                apply_mapper_revision(timeline, bookkeeping, pending)
                pending = next(revisions, None)
            compact_pose_history(timeline, manifest["config"]["max_bookkeeping_frames"])
            if memory.identity is None:
                first = stamp
                memory.start(timeline.session, timeline.epoch, stamp, grounded=True)
            else:
                memory.observe(timeline.session, timeline.epoch, stamp, grounded=True)
            if stamp not in baseline:
                return
            lookup = PublicationPoseLookup(timeline, publication)
            support_started = time.monotonic()
            local_from_map = validated_transform(record["local_from_map"])

            def resolve(t):
                try:
                    pose = (
                        local_from_map
                        @ lookup.associate(t).map_from_lidar
                        @ np.linalg.inv(timeline.extrinsic)
                        @ imu_from_base
                    )
                except ValueError:
                    return None
                return pose, (0.0 if t == stamp else uncertainty)

            current = resolve(stamp)
            if current is None or not np.allclose(current[0], record["local_from_base"], rtol=0, atol=1e-8):
                raise ValueError("current replay/trajectory frame disagreement")
            placed, missing = memory.placed(timeline.session, timeline.epoch, publication, resolve)
            current_polygons = [(p, "current") for p in patch_polygons(current[0], patches)]
            variants = dict(
                current=current_polygons,
                startup=current_polygons + [(p, s) for p, s in placed if s == "startup"],
                tracks=current_polygons + placed,
            )
            motions = extended_motions(current[0])
            retrace = retraced_motion(current[0], stamp, memory.anchors, resolve)
            old = baseline[stamp]
            # Original classes/body/wheel queries remain exact; only support
            # policy is varied below. Goal-directed flags stay frozen.
            for name, poses in candidate_motions(current[0]).items():
                score = motion_score(layers[:, :, 0], record, poses, patches, body, np.zeros(2))
                for field in ("state", "wheel_support", "body_projection"):
                    if score[field] != old["motions"][name][field]:
                        raise ValueError("baseline query changed")
            compared += 1
            started = time.monotonic()
            prepared = {n: PreparedMotion(layers, record, poses, patches, body) for n, poses in motions.items()}
            core_queries = (
                {n: PreparedMotion(layers, record, poses, query_patches, body) for n, poses in motions.items()}
                if support_core
                else prepared
            )
            retrace_full = PreparedMotion(layers, record, retrace, patches, body) if retrace is not None else None
            retrace_core = (
                PreparedMotion(layers, record, refine_trajectory(retrace), query_patches, body)
                if retrace is not None and support_core
                else retrace_full
            )
            scores = {}
            recovery = {}
            for name, polygons in variants.items():
                index = indices[name].update(polygons, record["origin"], record["resolution"])
                scores[name] = {
                    n: (evaluate_support_core(query, core_queries[n], index) if support_core else query.evaluate(index))
                    for n, query in prepared.items()
                }
                recovery[name] = (
                    dict(state="unknown", reason="insufficient_recent_path")
                    if retrace_full is None
                    else (
                        evaluate_support_core(retrace_full, retrace_core, index)
                        if support_core
                        else retrace_full.evaluate(index)
                    )
                )
            row = dict(
                evidence_ns=stamp,
                elapsed_s=(stamp - first) / 1e9,
                history_anchors=len(memory.anchors),
                missing_pose_anchors=missing,
                diagnostic_query_wall_s=time.monotonic() - started,
                diagnostic_support_total_wall_s=time.monotonic() - support_started,
                index_rebuilt_cells={n: i.rebuilt_cells for n, i in indices.items()},
                variants=scores,
                retrace=recovery,
            )
            if row["elapsed_s"] in (0.0, 5.0, 15.0, 30.0):
                example = example_geometry(record, layers, variants["tracks"], motions["forward_0.05"], query_patches)
                examples.append(dict(elapsed_s=row["elapsed_s"], motion="forward_0.05", **example))
            rows.append(row)
            sink.write(json.dumps(row) + "\n")
            if len(rows) % 10 == 0:
                print(f"Scored {len(rows)} samples; +{row['elapsed_s']:.1f}s", flush=True)

        exact = replay(source, archive, projection_observer=observe)
    if compared != len(baseline):
        raise ValueError("missing baseline decision bins")
    summary = {}
    for variant in ("current", "startup", "tracks"):
        summary[variant] = {}
        for period, selected in (("all", rows), ("after_15s", [r for r in rows if r["elapsed_s"] >= 15])):
            counts = dict(
                samples=len(selected),
                stationary=0,
                legacy_progress_or_turn=0,
                expanded_any_action=0,
                retrace_available=0,
                expanded_or_retrace=0,
            )
            for row in selected:
                scores = row["variants"][variant]
                old = baseline[row["evidence_ns"]]["motions"]
                counts["stationary"] += scores["stationary"]["state"] == "available"
                counts["legacy_progress_or_turn"] += any(
                    scores[n]["state"] == "available" and (n.startswith("turn") or s["toward_goal"])
                    for n, s in old.items()
                    if n != "stationary"
                )
                counts["expanded_any_action"] += any(
                    s["state"] == "available" for n, s in scores.items() if n != "stationary"
                )
                recover = row["retrace"][variant]["state"] == "available"
                counts["retrace_available"] += recover
                counts["expanded_or_retrace"] += recover or any(
                    s["state"] == "available" for n, s in scores.items() if n != "stationary"
                )
            summary[variant][period] = counts
    if any(sha(p) != digest for p, digest in hashes.items()):
        raise ValueError("experiment input changed")
    return dict(
        completed=True,
        acceptance=False,
        contract=contract,
        summary=summary,
        peak_diagnostic_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
        examples=examples,
        exact_replay=exact,
        input_sha256=hashes,
        output_sha256={"scores.jsonl": sha(output / "scores.jsonl")},
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assessment", type=Path)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--archive-root", type=Path, required=True)
    parser.add_argument("--history-xy-erosion", type=float, choices=(0.0, 0.01, 0.02), default=0.02)
    parser.add_argument(
        "--support-core",
        action="store_true",
        help="Explicit unqualified 40x60mm positive-support ablation; full hazard checks retained",
    )
    args = parser.parse_args()
    for path in (args.assessment, args.trajectory):
        check_run(path)
    output = allocate_run(args.archive_root, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        report = evaluate(
            args.assessment,
            args.trajectory,
            args.archive_root,
            output,
            args.history_xy_erosion,
            support_core=args.support_core,
        )
        check_run(output)
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report["summary"], indent=2), flush=True)
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=str(exc), acceptance=False)))
        raise


if __name__ == "__main__":
    main()
