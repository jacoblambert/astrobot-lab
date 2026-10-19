#!/usr/bin/env python3
"""Fixed-frame pose errors and read-only estimator-stage attribution. No fitted alignment."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from archive_guard import allocate_run, check_run
from imu_consistency import interpolate_poses
from shadow_pose_replay import load_pose_sensitivity, sha


def residuals(estimated, truth):
    xyz = estimated[:, :3, 3] - truth[:, :3, 3]
    angles = (Rotation.from_matrix(estimated[:, :3, :3]).as_euler("xyz") -
              Rotation.from_matrix(truth[:, :3, :3]).as_euler("xyz") + np.pi) % (2*np.pi) - np.pi
    return dict(position_xyz_m=xyz, horizontal_m=np.linalg.norm(xyz[:, :2], axis=1),
                vertical_m=np.abs(xyz[:, 2]), rpy_deg=np.rad2deg(angles))


def summary(estimated, truth):
    r = residuals(estimated, truth)
    result = dict(samples=len(estimated))
    for name, values in r.items():
        result[name] = dict(median=np.median(values, axis=0).tolist(),
            absolute_p95=np.percentile(abs(values), 95, axis=0).tolist(),
            absolute_max=np.max(abs(values), axis=0).tolist(), final=values[-1].tolist())
    result["targets_pass"] = bool(result["horizontal_m"]["absolute_p95"] <= .1 and
        result["vertical_m"]["absolute_p95"] <= .02 and
        np.all(np.asarray(result["rpy_deg"]["absolute_p95"]) <= [.3, .3, 1.]))
    return result


def first_sustained(times, errors, threshold=.1, duration=1.):
    start = None
    for t, error in zip(times, errors):
        if error > threshold:
            if start is None: start = t
            if t-start >= duration: return float(start)
        else:
            start = None
    return None


def relative_summary(times, estimated, truth):
    valid = times >= times[0]+1.
    e0 = interpolate_poses(times, estimated, times[valid]-1.)
    g0 = interpolate_poses(times, truth, times[valid]-1.)
    return summary(np.linalg.inv(e0)@estimated[valid], np.linalg.inv(g0)@truth[valid])


def trace_stages(records):
    stages = {name: {} for name in ("imu_prediction", "scan_match_target", "pre_smoother", "optimized")}
    gps, scan_sigmas, shaped_sigmas = [], [], []
    for row in records:
        stage = row["stage"]
        if stage == "processed_input":
            continue
        if stage != "pre_smoother":
            if row["frame_id"] in stages[stage]: raise ValueError("duplicate stage/frame")
            stages[stage][row["frame_id"]] = row
            continue
        for p in row["poses"]:
            stages[stage][p["frame_id"]] = p
        for f in row["factors"]:
            if "gps_position" in f: gps.append(f)
            if "prior_pose" not in f or len(f.get("sigmas", [])) != 6: continue
            sigmas = np.array(f["sigmas"])
            # The declared absolute extension constructs a yaw-only prior with
            # these five exact kBig sigmas. Do not mistake a geometry-shaped
            # scan factor for heading merely because it is anisotropic.
            if np.allclose(sigmas[[0, 1, 3, 4, 5]], 1000., rtol=0, atol=1e-12): continue
            if len(f["keys"]) != 1: raise ValueError("scan prior key contract")
            key = int(f["keys"][0])
            if key >> 56 != ord("x"): raise ValueError("scan prior is not a pose key")
            frame = key & ((1 << 56)-1)
            if frame not in stages[stage]: raise ValueError("scan prior has no matching current pose")
            stages["scan_match_target"][frame] = dict(stages[stage][frame], map_from_imu=f["prior_pose"])
            if np.allclose(sigmas, sigmas[0], rtol=0, atol=1e-12): scan_sigmas.append(sigmas[0])
            else: shaped_sigmas.append(sigmas)
    result = dict(gps_factors=len(gps), scan_prior_sigmas=sorted(set(scan_sigmas)))
    if shaped_sigmas:
        result["anisotropic_scan_priors"] = len(shaped_sigmas)
        result["anisotropic_reported_sigmas_min_max"] = [np.min(shaped_sigmas, axis=0).tolist(), np.max(shaped_sigmas, axis=0).tolist()]
    return stages, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("pair", type=Path)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        for p in (args.capture, args.pair): check_run(p)
        native = json.loads((args.capture/"native_timing_audit.json").read_text())["frames"]
        report = json.loads((args.pair/"shadow_pair_audit.json").read_text())
        recorder_paths = [Path(p) for p in report["input_sha256"] if p.endswith("/shadow_collection.json")]
        if len(recorder_paths) != 1: raise ValueError("ambiguous mount provenance")
        mount = json.loads(recorder_paths[0].read_text())["request"]
        lidar_base = np.linalg.inv(np.array(mount["base_from_lidar"]))
        imu_base = np.linalg.inv(np.array(mount["base_from_imu"]))
        truth = np.array([f["world_from_lidar"] for f in native])
        truth[:, :3, 3] -= report["fixed_home"]
        truth = truth@lidar_base
        times = np.array([f["observed_stamp"] for f in native])
        speed = np.r_[0., np.linalg.norm(np.diff(truth[:, :3, 3], axis=0), axis=1)/np.diff(times)]
        rot = Rotation.from_matrix(truth[:, :3, :3])
        omega = np.r_[0., (rot[:-1].inv()*rot[1:]).magnitude()/np.diff(times)]
        phases = np.where(omega > .03, "turn", np.where(speed > .02, "translation", "stationary"))
        results = {}
        arrays = {"times": times, "gt_base": truth, "phase": phases}
        for mode in ("first_published", "final_revised"):
            estimated, _ = load_pose_sensitivity(args.pair, args.capture, native, mode)
            estimated = estimated@lidar_base
            r = residuals(estimated, truth)
            results[mode] = dict(overall=summary(estimated, truth), relative_1s=relative_summary(times, estimated, truth),
                phases={str(p): summary(estimated[phases==p], truth[phases==p]) for p in np.unique(phases)},
                first_3s=summary(estimated[times <= times[0]+3.], truth[times <= times[0]+3.]),
                last_3s=summary(estimated[times >= times[-1]-3.], truth[times >= times[-1]-3.]),
                first_sustained_horizontal_over_10cm=first_sustained(times, r["horizontal_m"]))
            arrays[mode] = estimated
        trace = args.pair/"estimator_trace.jsonl"
        if trace.exists():
            if report["input_sha256"].get(str(trace)) != sha(trace): raise ValueError("unqualified trace")
            stages, factors = trace_stages([json.loads(line) for line in trace.read_text().splitlines()])
            results["trace_factors"] = factors
            results["stages"] = {}
            stage_ids = None
            for stage, poses in stages.items():
                if stage == "scan_match_target" and report.get("backend", "cpu") == "gpu":
                    if poses: raise ValueError("GPU backend unexpectedly emitted CPU scan pose priors")
                    continue  # GPU registration is inside the smoother, not a separate pose-prior stage.
                rows = [p for _, p in sorted(poses.items()) if times[0] <= p["stamp_ns"]/1e9 <= times[-1]]
                ids = [p["frame_id"] for p in rows]
                if not rows or (stage_ids is not None and ids != stage_ids): raise ValueError("incomplete stage coverage")
                stage_ids = ids
                stamps = np.array([p["stamp_ns"]/1e9 for p in rows])
                gt = interpolate_poses(times, truth, stamps)
                est = np.array([p["map_from_imu"] for p in rows])@imu_base
                results["stages"][stage] = summary(est, gt)
                arrays["stage_"+stage] = est
                arrays["stage_times"] = stamps
                arrays["stage_gt"] = gt
        with (output/"accuracy_arrays.npz").open("xb") as stream: np.savez_compressed(stream, **arrays)
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        for mode in ("first_published", "final_revised"):
            r = residuals(arrays[mode], truth)
            axes[0].plot(times-times[0], r["horizontal_m"], label=mode)
            axes[1].plot(times-times[0], r["position_xyz_m"][:, 2], label=mode)
            axes[2].plot(times-times[0], r["rpy_deg"][:, 2], label=mode)
        for ax, label in zip(axes, ("Horizontal error (m)", "Signed Z error (m)", "Yaw error (deg)")):
            ax.set_ylabel(label); ax.grid(); ax.legend()
        axes[-1].set_xlabel("Seconds since first native acquisition")
        fig.tight_layout(); fig.savefig(output/"accuracy.png"); plt.close(fig)
        result = dict(completed=True, acceptance=False, frame="same physical base_link; fixed home; no fitted alignment",
            phase_definition="GT angular speed >0.03 rad/s: turn; otherwise speed >0.02 m/s: translation; else stationary",
            results=results, input_sha256={str(args.pair/"shadow_pair_audit.json"): sha(args.pair/"shadow_pair_audit.json")},
            output_sha256={name: sha(output/name) for name in ("accuracy_arrays.npz", "accuracy.png")})
        with (output/"accuracy_report.json").open("x") as stream: json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(results, indent=2))
    except Exception as exc:
        with (output/"failure.json").open("x") as stream: json.dump(dict(error=str(exc)), stream)
        raise


if __name__ == "__main__": main()
