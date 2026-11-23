#!/usr/bin/env python3
"""GT-isolated native tick replay after independent cadence/pose audit.

This does NOT unlock the moving ROS full-scan replay. Native tick XYZ comes from
one sensor pose; ROS full scans concatenate several such chunks without deskew.
GT terrain is evaluation-only, and no cloud fitting or timestamp adjustment is
performed. Every chunk is integrated; projection/evaluation run at 10 Hz.
"""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import time

import numpy as np

from archive_guard import allocate_run
from audit_native_terrain import summarize_frames
from layered_terrain import TerrainMapper
from raw_return_filter import valid_range_mask
from score_terrain_calibration import reference_labels, validated_route, validated_scan
from terrain_metrics import calibration_gates, score_terrain


def grid_geometry(phase=0.):
    """Declared alignment stress only; never change cells, resolution or GT."""
    if phase not in (0., .025, .05, .075):
        raise ValueError("grid phase must be one of the four declared offsets")
    return dict(origin=[-10.+phase,-10.+phase],resolution=.1,width=200,height=200)


def validate_diagnostic_mode(builds, requested):
    if not builds or any(builds) != requested or (requested and not all(builds)):
        raise ValueError("diagnostic libraries require --diagnostics for every variant; timings are excluded")


def validate_native_source(audit):
    report = summarize_frames(audit["frames"])
    current = report["current_pose"]
    if (not report["cadence_passed"] or abs(report["timeline_time_delta_ratio"] - 1.) > .005
            or abs(report["world_time_delta_ratio"] - 1.) > .005 or audit["duplicate_native_reads"]
            or current["missing"] or current["maximum_m"] > .001 or current["samples"] < 10000
            or current["nonhorizontal_surface_samples"] < 100):
        raise ValueError("native cadence/current-pose geometry gate failed")
    stamps = np.array([f["observed_stamp"] for f in audit["frames"]])
    intervals = np.array([f["world_rendering_dt"] for f in audit["frames"][1:]])
    if not np.allclose(np.diff(stamps), intervals, rtol=.005, atol=1e-6):
        raise ValueError("native capture skipped render ticks or changed clock cadence")
    poses = np.array([f["world_from_lidar"] for f in audit["frames"]])
    if poses.shape != (len(audit["frames"]), 4, 4) or not np.isfinite(poses).all():
        raise ValueError("invalid native pose sequence")
    translation = np.linalg.norm(poses[:, :3, 3] - poses[0, :3, 3], axis=1).max()
    rotation = np.linalg.norm(poses[:, :3, :3] - poses[0, :3, :3], axis=(1, 2)).max()
    if translation < .1 and rotation < .1:
        raise ValueError("stationary evidence cannot validate native motion association")
    return report


def run(args, output):
    cap, ref = args.capture, args.reference_capture
    grid=grid_geometry(getattr(args,"grid_phase",0.))
    for folder in (cap, ref):
        if (folder / "failure.json").exists():
            raise ValueError("failed capture/reference is ineligible")
    read = lambda folder, name: json.loads((folder / name).read_text())
    audit = read(cap, "native_timing_audit.json")
    timing = validate_native_source(audit)
    pose_folder = getattr(args, "shadow_pose_audit", None)
    pose_mode = getattr(args, "shadow_pose_mode", None)
    poses, pose_provenance = None, None
    if bool(pose_folder) != bool(pose_mode):
        raise ValueError("pose sensitivity requires both audit and explicit mode")
    if pose_folder:
        from shadow_pose_replay import load_pose_sensitivity
        poses, pose_provenance = load_pose_sensitivity(pose_folder, cap, audit["frames"], pose_mode)
    route = read(cap, "multiview.json")
    views = validated_route(route)
    if (audit["frames"][0]["observed_stamp"] > views[0]["stamp"]
            or audit["frames"][-1]["observed_stamp"] < views[-1]["stamp"] - 1e-6):
        raise ValueError("native capture does not cover the entire prescribed route")
    for folder in [cap, *(cap / v["path"] for v in views)]:
        sensor = read(folder, "sensor_audit.json")
        if not sensor["passed"] or not sensor.get("extrinsic_tf_checked", False):
            raise ValueError("settled surface/extrinsic-TF checks did not pass")
    spec = read(cap, "calibration_spec.json")
    if spec != read(ref, "calibration_spec.json"):
        raise ValueError("reference geometry differs")
    for folder in (cap, ref):
        if not read(folder, "calibration_checks.json")["passed"]:
            raise ValueError("collision calibration failed")
    tile = np.load(ref / "collision_tile.npz")
    labels, ids, names = reference_labels(spec, tile, read(ref, "summary.json"))
    files = sorted((cap / "native_chunks").glob("chunk_*.npz"))
    if len(files) != len(audit["frames"]) or any(f.name != a["file"] for f, a in zip(files, audit["frames"])):
        raise ValueError("native chunk count or ordering differs from audit")
    manifest = read(cap, "manifest.json")
    attributes = manifest["sensor"]["sensor_attributes"]
    minimum, maximum = float(attributes["minRange"]), float(attributes["maxRange"])
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    anchor = np.r_[spec["center"], 0.]
    if poses is not None and not np.array_equal(anchor, [-20., -20., 0.]):
        raise ValueError("pose sensitivity fixed home differs from calibration reference")
    variants = {"valid_range_measured": (args.library, False), "valid_range_inferred": (args.library, True)}
    if args.baseline_library:
        variants.update(baseline_measured=(args.baseline_library, False), baseline_inferred=(args.baseline_library, True))
    input_files = [cap / n for n in ("native_timing_audit.json", "manifest.json", "multiview.json", "drive_log.json",
                                    "sensor_audit.json", "calibration_checks.json", "calibration_spec.json")]
    input_files += [cap / v["path"] / "sensor_audit.json" for v in views]
    provenance = dict(source="native_tick_GT_isolation", capture=str(cap), reference_capture=str(ref),
        input_sha256={str(f.relative_to(cap)): sha(f) for f in input_files},
        reference_sha256={n: sha(ref / n) for n in ("collision_tile.npz", "summary.json", "calibration_spec.json",
                                                   "calibration_checks.json", "manifest.json")},
        library_sha256={n: sha(lib) for n, (lib, _) in variants.items()}, evaluator_sha256=sha(Path(__file__)),
        metrics_sha256=sha(Path(__file__).with_name("terrain_metrics.py")),
        filter_sha256=sha(Path(__file__).with_name("raw_return_filter.py")),
        timing_auditor_sha256=sha(Path(__file__).with_name("audit_native_terrain.py")),
        scan_validator_sha256=sha(Path(__file__).with_name("score_terrain_calibration.py")),
        variants=list(variants), multiview=True, native_chunks=True, ros_full_scan_motion_allowed=False,
        gt_terrain_passed_to_mapper=False, development_fixture=True, timing=timing,
        visibility_denominator="fixed original reference", projection_hz=10., terrain_geometry=grid,
        alignment_stress_only=getattr(args,"grid_phase",0.)!=0.)
    if poses is not None:
        provenance.update(source="native_tick_offline_pose_sensitivity", pose_sensitivity=pose_provenance)
    mappers = {n: TerrainMapper(lib, origin=grid["origin"], interpolate=interpolate) for n, (lib, interpolate) in variants.items()}
    frames, previous, projected = [], -1., -1.
    batch = {n: 0. for n in variants}
    latency = {n: [] for n in variants}
    chunk_counts = []
    failure_snapshots = []
    worst = {n: dict(dropoff_free_cells=0, obstacle_false_free_fraction=0.) for n in variants}
    collect_diagnostics = bool(getattr(args, "diagnostics", False))
    diagnostic_frames = []
    try:
        diagnostic_builds = [hasattr(mapper.lib, "terrain_diagnostics") for mapper in mappers.values()]
        validate_diagnostic_mode(diagnostic_builds, collect_diagnostics)
        provenance["diagnostic_collection"] = collect_diagnostics
        for i, (file, frame) in enumerate(zip(files, audit["frames"])):
            association = dict(raw_scan_sha256=frame["sha256"], stamp=frame["observed_stamp"],
                               world_from_lidar=frame["world_from_lidar"])
            points, stamp, transform = validated_scan(file, association, previous, stamp_key="observed_stamp")
            previous = stamp
            provenance["input_sha256"][str(file.relative_to(cap))] = frame["sha256"]
            keep, counts = valid_range_mask(points, minimum, maximum)
            chunk_counts.append(dict(stamp=stamp, **counts))
            transform[:3, 3] -= anchor
            if poses is not None:
                transform = poses[i]  # Already in the declared home-relative map; never fit to GT.
            points = points[keep] @ transform[:3, :3].T + transform[:3, 3]
            project = projected < 0 or stamp - projected >= .1 - 1e-6 or i == len(files) - 1
            record = dict(stamp=stamp, through_native_chunk=i)
            for name, mapper in mappers.items():
                started = time.perf_counter()
                mapper.integrate(points, stamp, transform[:3, 3], np.arctan2(transform[1, 0], transform[0, 0]))
                if project:
                    layers = mapper.project(stamp)
                batch[name] += (time.perf_counter() - started) * 1000
                if project:
                    latency[name].append(batch[name])
                    batch[name] = 0.
                    record[name] = score_terrain(layers[:, :, 0], map_origin=grid["origin"], map_resolution=grid["resolution"],
                        labels=labels, object_ids=ids, gt_origin=tile["origin"] - anchor[:2],
                        gt_resolution=float(tile["resolution"]))
                    record[name]["cost_reason_sha256"] = hashlib.sha256(layers[:, :, :2].tobytes()).hexdigest()
                    if collect_diagnostics:
                        cells, corners = mapper.diagnostics()
                        diagnostic_frames.append(dict(stamp=stamp, variant=name,
                            corner_flags={str(flag): int(np.count_nonzero(corners[:, :, 0] & flag))
                                          for flag in (1, 2, 4, 8, 16, 32, 64)},
                            source_edges={str(bits): int(np.count_nonzero(cells[:, :, 0] == bits))
                                          for bits in range(1, 8)}))
                        if i == len(files) - 1:
                            with (output / f"diagnostics_{name}.npz").open("xb") as stream:
                                np.savez_compressed(stream, cells=cells, corners=corners)
                            provenance.setdefault("diagnostic_sha256", {})[f"diagnostics_{name}.npz"] = sha(
                                output / f"diagnostics_{name}.npz")
                    worsened = [key for key, value in worst[name].items() if record[name][key] > value]
                    if worsened:
                        snapshot = f"failure_{name}_chunk_{i:06}.npz"
                        with (output / snapshot).open("xb") as stream:
                            np.savez_compressed(stream, layers=layers)
                        failure_snapshots.append(dict(file=snapshot, variant=name, stamp=stamp,
                                                      through_native_chunk=i, worsened=worsened, sha256=sha(output / snapshot)))
                        for key in worsened:
                            worst[name][key] = record[name][key]
                    if i == len(files) - 1:
                        np.savez_compressed(output / f"{name}.npz", layers=layers)
            if project:
                projected = stamp
                frames.append(record)
            if (i + 1) % 300 == 0:
                print(f"Integrated {i + 1}/{len(files)} chunks; projected {len(frames)} maps per variant", flush=True)
        rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.
        results = dict(frames=frames, chunk_counts=chunk_counts, native_chunks_integrated=len(files),
                       peak_process_rss_mib=rss, latency_ms=latency, object_names=names, failure_snapshots=failure_snapshots,
                       latency_p95_ms={n: float(np.percentile(v, 95)) for n, v in latency.items()},
                       latency_p99_ms={n: float(np.percentile(v, 99)) for n, v in latency.items()},
                       note="Native-tick GT isolation, not ROS full-scan deskew or navigation acceptance. Timing sums integration plus projection per output batch.")
        results["sequence_max_dropoff_free_cells"] = {n: max(f[n]["dropoff_free_cells"] for f in frames) for n in variants}
        results["sequence_max_positive_false_free_fraction"] = {n: max(f[n]["obstacle_false_free_fraction"] for f in frames) for n in variants}
        results["development_gates"] = {n: calibration_gates([f[n] for f in frames], required_objects=len(names),
            p95_ms=results["latency_p95_ms"][n], p99_ms=results["latency_p99_ms"][n], peak_rss_mib=rss) for n in variants}
        if poses is not None:
            results["note"] = pose_provenance["interpretation"]
            for gates in results["development_gates"].values():
                gates.update(sampled_compute_passed=False, offline_pose_timing_excluded=True)
        if collect_diagnostics:
            for gates in results["development_gates"].values():
                gates["sampled_compute_passed"] = False
                gates["diagnostic_timing_excluded"] = True
            with (output / "terrain_diagnostics.json").open("x") as stream:
                json.dump(dict(schema=1, gt_input=False, classification_changed=False,
                               timing_acceptance=False, frames=diagnostic_frames), stream, indent=2)
        for name, data in (("manifest.json", provenance), ("summary.json", results)):
            with (output / name).open("x") as stream:
                json.dump(data, stream, indent=2)
        print(json.dumps(dict(final=frames[-1], gates=results["development_gates"],
                              sequence_max_positive_false_free_fraction=results["sequence_max_positive_false_free_fraction"],
                              sequence_max_dropoff_free_cells=results["sequence_max_dropoff_free_cells"]), indent=2))
    finally:
        for mapper in mappers.values():
            mapper.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--reference-capture", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--baseline-library", type=Path)
    parser.add_argument("--shadow-pose-audit", type=Path, help="Qualified frozen native/GLIM pose-pair audit")
    parser.add_argument("--shadow-pose-mode", choices=("first_published", "final_revised"),
                        help="Offline pose-only sensitivity, not a live causal/revision mapper")
    parser.add_argument("--diagnostics", action="store_true",
                        help="Require diagnostic-only libraries; save rejection data and exclude compute acceptance")
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--grid-phase",type=float,choices=(0.,.025,.05,.075),default=0.,
                        help="Declared XY grid-alignment stress; same physical inputs, reference and safety gates")
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        run(args, output)
    except Exception as exc:
        with (output / "failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), type=type(exc).__name__), stream, indent=2)
        raise


if __name__ == "__main__":
    main()
