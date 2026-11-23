#!/usr/bin/env python3
"""Score stationary calibration raw scans; never pass GT terrain to the mapper."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import resource
import numpy as np

from archive_guard import allocate_run
from layered_terrain import TerrainMapper
from terrain_metrics import score_terrain, calibration_gates
from raw_return_filter import valid_range_mask


def require_single_pose_surface(audit):
    if (not audit.get("passed") or not audit.get("stationary_only")
            or audit.get("surface_pose_model", "scan_end_pose") != "scan_end_pose"):
        raise ValueError("raw ROS scan lacks a passed single-pose stationary surface audit; native audit is not deskew")


def validated_scan(file, audit_frame, previous, stamp_key="stamp"):
    if hashlib.sha256(file.read_bytes()).hexdigest() != audit_frame.get("raw_scan_sha256"):
        raise ValueError("raw scan missing audit hash or changed after sensor audit")
    with np.load(file) as scan:
        points, stamp, transform = scan["points"].copy(), float(scan[stamp_key]), scan["world_from_lidar"].copy()
    if (points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(stamp) or stamp<0
            or not np.isfinite(audit_frame["stamp"])):
        raise ValueError("invalid raw point shape or measurement stamp")
    if (transform.shape!=(4,4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3],[0,0,0,1],atol=1e-9,rtol=0)
            or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-5,rtol=0)
            or abs(np.linalg.det(transform[:3,:3])-1.)>1e-5):
        raise ValueError("audited transform is not a finite rigid SE(3) pose")
    if stamp <= previous or abs(stamp-audit_frame["stamp"])>1e-9:
        raise ValueError("scan timestamp ordering/audit association failed")
    if not np.array_equal(transform,np.asarray(audit_frame["world_from_lidar"])):
        raise ValueError("scan transform differs from audited physics pose")
    return points,stamp,transform


def validated_route(multiview):
    pattern = multiview.get("prescribed_pattern", "straight")
    distances = [x/10 for x in range(21)]+[x/10 for x in range(19,-1,-1)]
    if pattern not in ("straight", "diagonal15", "spin360", "centered", "centered_diagonal15", "centered_sweep15", "wide_sweep15", "wide_sweep15_return"):
        raise ValueError("unknown prescribed route")
    if pattern in ("centered", "centered_diagonal15"):
        distances=[x/10 for x in range(11)]+[x/10 for x in range(9,-11,-1)]+[x/10 for x in range(-9,1)]
    if pattern=="centered_sweep15":
        distances=[0.,1.,-1.,0.]
    heading = float(np.deg2rad(15)) if pattern in ("diagonal15", "centered_diagonal15", "centered_sweep15") else 0.
    expected = [dict(distance=x,heading=heading) for x in distances]
    if heading:
        expected = [dict(distance=0.,heading=0.),*expected,dict(distance=0.,heading=0.)]
    if pattern=="spin360":
        expected=[dict(distance=0.,heading=float(np.deg2rad(degrees))) for degrees in range(0,361,15)]
    if pattern in ("wide_sweep15","wide_sweep15_return"):
        expected=[dict(distance=0.,heading=0.),
                  *[dict(distance=x,heading=float(np.deg2rad(15))) for x in (0.,1.2,-1.2,0.)]]
        if pattern=="wide_sweep15_return":
            expected.append(dict(distance=0.,heading=0.))
    if not multiview["complete"] or len(multiview["views"]) != len(expected):
        raise ValueError("incomplete prescribed multi-view route")
    if "prescribed_targets" in multiview and multiview["prescribed_targets"] != expected:
        raise ValueError("route targets changed from frozen acquisition pattern")
    for view,target in zip(multiview["views"],expected):
        actual = view.get("target",dict(distance=view.get("target_x"),heading=0.))
        if actual != target:
            raise ValueError("view target differs from prescribed route")
    return multiview["views"]


def reference_labels(spec, tile, summary):
    labels = np.full(tile["object_ids"].shape, -1, dtype=np.int8)
    ids = np.zeros_like(labels, dtype=np.int32)
    required = {b["name"]: i+1 for i,b in enumerate(spec["boxes"]) if b["required"]}
    names = {}
    for obj in summary["objects"]:
        name = obj["collision"].split("/")[-1]
        mask = tile["object_ids"] == obj["id"]
        if name.startswith("floor_"):
            labels[mask & tile["visible"]] = 0
        elif name in required:
            labels[mask] = 1
            ids[mask] = required[name]
            names[str(required[name])] = name
        elif name in ("plateau", "overhang"):
            labels[mask] = 1
    # Do not let missing/occluded/zero-ray reference objects shrink recall's
    # denominator. Every required analytic object must have raster support.
    if len(names) != len(required):
        raise ValueError("reference tile omits required objects; cannot score a reduced set")
    y,x = np.indices(labels.shape)
    wx = tile["origin"][0]+(x+.5)*float(tile["resolution"])
    wy = tile["origin"][1]+(y+.5)*float(tile["resolution"])
    hx,hy = np.asarray(spec["center"])+spec["hole"]["center"]
    hole = (abs(wx-hx)<spec["hole"]["width"]/2) & (abs(wy-hy)<spec["hole"]["width"]/2)
    labels[hole] = 2
    return labels,ids,names


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--multiview", action="store_true", help="Score all prescribed settled views after physical drive")
    parser.add_argument("--reference-capture",type=Path,help="Keep a previous identical-geometry reference fixed for a sensor experiment")
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        run(args, output)
    except Exception as exc:
        with (output/"failure.json").open("x") as stream:
            json.dump(dict(error=str(exc), type=type(exc).__name__), stream, indent=2)
        raise


def run(args, output):
    capture = args.capture
    if (capture/"failure.json").exists():
        raise ValueError("failed capture is ineligible")
    read = lambda name: json.loads((capture/name).read_text())
    geometry, sensor = read("calibration_checks.json"), read("sensor_audit.json")
    if not geometry["passed"] or not sensor["passed"] or not sensor["stationary_only"]:
        raise ValueError("stationary geometry/sensor calibration gate not passed")
    require_single_pose_surface(sensor)
    spec, summary = read("calibration_spec.json"), read("summary.json")
    capture_manifest=read("manifest.json")
    if capture_manifest.get("experimental_lidar_pitch_deg",0):
        if args.reference_capture is None:raise ValueError("mount experiment requires the pre-existing reference capture")
        if not sensor.get("extrinsic_tf_checked",False):raise ValueError("tilted LiDAR has no published-TF audit")
    sensor_attributes = capture_manifest["sensor"]["sensor_attributes"]
    minimum, maximum = float(sensor_attributes["minRange"]),float(sensor_attributes["maxRange"])
    captured_tile = np.load(capture/"collision_tile.npz")
    tile=captured_tile
    reference_files=[]
    reference=args.reference_capture or capture
    reference_summary=summary
    if args.reference_capture is not None:
        if (reference/"failure.json").exists():raise ValueError("failed reference capture")
        if json.loads((reference/"calibration_spec.json").read_text())!=spec:
            raise ValueError("reference geometry specification differs from capture")
        if not json.loads((reference/"calibration_checks.json").read_text())["passed"]:
            raise ValueError("reference collision geometry not validated")
        tile=np.load(reference/"collision_tile.npz")
        reference_summary=json.loads((reference/"summary.json").read_text())
        reference_files=[reference/n for n in ["calibration_spec.json","calibration_checks.json","collision_tile.npz","summary.json","manifest.json"]]
    labels,ids,names = reference_labels(spec,tile,reference_summary)
    sensor_labels=None
    if "visible_sensor" in captured_tile:
        sensor_tile=dict(captured_tile)
        sensor_tile["visible"]=captured_tile["visible_sensor"]
        sensor_labels,sensor_ids,_=reference_labels(spec,sensor_tile,summary)
    files = sorted(capture.glob("raw_scan_*.npz"))
    if len(files) != 10 or len(sensor["frames"]) != 10:
        raise ValueError("incomplete fixed 10-frame fixture")
    audit_frames = sensor["frames"]
    extra_provenance = []
    if args.multiview:
        multiview = read("multiview.json")
        views = validated_route(multiview)
        files,audit_frames = [],[]
        extra_provenance = [capture/"multiview.json",capture/"drive_log.json"]
        for i,view in enumerate(views):
            if view["path"] != f"view_{i:02}":
                raise ValueError("unexpected viewpoint path/order")
            folder = capture/view["path"]
            audit = json.loads((folder/"sensor_audit.json").read_text())
            require_single_pose_surface(audit)
            scans = sorted(folder.glob("raw_scan_*.npz"))
            if not audit["passed"] or len(scans)!=2 or len(audit["frames"])!=2:
                raise ValueError("viewpoint calibration failed or incomplete")
            if capture_manifest.get("experimental_lidar_pitch_deg",0) and not audit.get("extrinsic_tf_checked",False):
                raise ValueError("tilted viewpoint lacks published sensor TF validation")
            files.extend(scans)
            audit_frames.extend(audit["frames"])
            extra_provenance.append(folder/"sensor_audit.json")
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    provenance_files = [*files, *extra_provenance, *(capture/n for n in ["calibration_spec.json","calibration_checks.json",
                       "sensor_audit.json","collision_tile.npz","summary.json","manifest.json","graph_contract.json"])]
    with (output/"manifest.json").open("x") as stream:
        json.dump(dict(capture=str(capture), input_sha256={str(p.relative_to(capture)):sha(p) for p in provenance_files},
                       reference_capture=str(reference),reference_sha256={p.name:sha(p) for p in reference_files},
                       library_sha256=sha(args.library), evaluator_sha256=sha(Path(__file__)),
                       metrics_sha256=sha(Path(__file__).with_name("terrain_metrics.py")),
                       validity_filter_sha256=sha(Path(__file__).with_name("raw_return_filter.py")),
                       stationary_only=True, development_fixture=True, gt_terrain_passed_to_mapper=False,
                       multiview=args.multiview, visibility_denominator="fixed initial-view continuous-FOV reference",
                       sensor_visibility_reported=sensor_labels is not None,
                       variants=["measured_accumulated","inferred_accumulated","measured_single_scan",
                                 "valid_range_measured","valid_range_inferred"]), stream, indent=2)
    origin = np.array([-10.,-10.])
    anchor = np.r_[spec["center"],0.]
    mappers = {n:TerrainMapper(args.library, interpolate=interp) for n,interp in
               [("measured_accumulated",False),("inferred_accumulated",True),
                ("valid_range_measured",False),("valid_range_inferred",True)]}
    latency = {n:[] for n in [*mappers,"measured_single_scan"]}
    frames, raw_counts, previous = [], {b["name"]:0 for b in spec["boxes"]}, -1.
    try:
        for file, audit_frame in zip(files,audit_frames):
            points,stamp,transform = validated_scan(file,audit_frame,previous)
            previous = stamp
            valid_ranges, validity_counts = valid_range_mask(points,minimum,maximum)
            transform[:3,3] -= anchor
            valid = np.isfinite(points).all(axis=1)
            valid_ranges = valid_ranges[valid]
            points = points[valid]@transform[:3,:3].T+transform[:3,3]
            for box in spec["boxes"]:
                bounds = np.all(abs(points[:,:2]-box["center"])<=box["width"]/2+.001, axis=1)
                raw_counts[box["name"]] += int((bounds & (points[:,2]>.03) & (points[:,2]<=box["height"]+.005)).sum())
            frame = dict(stamp=stamp, raw_count=len(valid), invalid_count=int((~valid).sum()), validity_counts=validity_counts)
            single = TerrainMapper(args.library, interpolate=False)
            try:
                for name, mapper in {**mappers,"measured_single_scan":single}.items():
                    started = time.perf_counter()
                    mapper.integrate(points[valid_ranges] if name.startswith("valid_range_") else points,
                                     stamp,transform[:3,3],np.arctan2(transform[1,0],transform[0,0]))
                    layers = mapper.project(stamp)
                    latency[name].append((time.perf_counter()-started)*1000)
                    metrics = score_terrain(layers[:,:,0], map_origin=origin, map_resolution=.1,
                        labels=labels, object_ids=ids, gt_origin=tile["origin"]-anchor[:2], gt_resolution=float(tile["resolution"]))
                    frame[name] = metrics
                    if sensor_labels is not None:
                        frame[name]["sensor_matched_visibility"] = score_terrain(layers[:,:,0], map_origin=origin,
                            map_resolution=.1, labels=sensor_labels,object_ids=sensor_ids,
                            gt_origin=captured_tile["origin"]-anchor[:2],gt_resolution=float(captured_tile["resolution"]))
                    frame[name]["cost_reason_sha256"] = hashlib.sha256(layers[:,:,:2].tobytes()).hexdigest()
                    if file == files[-1]:
                        np.savez_compressed(output/f"{name}.npz",layers=layers)
            finally:
                single.close()
            frames.append(frame)
        results = dict(frames=frames, object_names=names, raw_object_return_counts=raw_counts,
            sequence_max_dropoff_free_cells={k:max(f[k]["dropoff_free_cells"] for f in frames) for k in latency},
            latency_p95_ms={k:float(np.percentile(v,95)) for k,v in latency.items()},
            latency_p99_ms={k:float(np.percentile(v,99)) for k,v in latency.items()},
            latency_ms=latency,
            peak_process_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,
            memory_scope="Entire Linux scorer process, four accumulated mappers plus one single-scan mapper and Python arrays",
            note="Development stop-scan diagnostic; moving scans and mission safety are not validated. Zero-return required objects remain misses.")
        results["development_gates"]={name:calibration_gates([f[name] for f in frames],
            required_objects=len(names),p95_ms=results["latency_p95_ms"][name],
            p99_ms=results["latency_p99_ms"][name],peak_rss_mib=results["peak_process_rss_mib"]) for name in latency}
        if sensor_labels is not None:
            results["sensor_visibility_development_gates"]={name:calibration_gates(
                [f[name]["sensor_matched_visibility"] for f in frames],required_objects=len(names),
                p95_ms=results["latency_p95_ms"][name],p99_ms=results["latency_p99_ms"][name],
                peak_rss_mib=results["peak_process_rss_mib"]) for name in latency}
        with (output/"summary.json").open("x") as stream:
            json.dump(results,stream,indent=2)
        print(json.dumps(dict(final=frames[-1],raw_object_return_counts=raw_counts,
                              latency_p95_ms=results["latency_p95_ms"], latency_p99_ms=results["latency_p99_ms"],
                              peak_process_rss_mib=results["peak_process_rss_mib"]),indent=2))
        print("Sequence worst dropoff false-free:",results["sequence_max_dropoff_free_cells"])
    finally:
        for mapper in mappers.values():
            mapper.close()


if __name__ == "__main__":
    main()
