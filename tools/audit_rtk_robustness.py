#!/usr/bin/env python3
"""C3 audit: GLIM + absolute-pose (RTK) robustness on a recorded route. Container only.

Replays a completed recorder's selected LiDAR + physical IMU + /clock with the exact
embedded launch-time GLIM config. Default: the navigation-mode localization_supervisor
owns GLIM (as in a Nav2 mission) and the raw /rtk/pose stream is edited, so the
supervisor's RTK jump gate, outage fault and restart path are exercised. A mock
navigation side answers slam/reset_map and the costmap clears and publishes a free
slam/bev_costmap at the published pose. --no-supervisor runs a bare glim_rosnode on the
recorded (supervisor-output) absolute poses instead.
Scores GLIM pose_corrected against recorded GT and the supervisor status timeline.
Not deterministic (ROS timing), not an acceptance gate; run `nominal` twice to see
run-to-run spread.

Profiles (comma-joined terms):
  nominal                 recorded absolute poses unchanged
  outage<s>               drop absolute poses for <s> sim-seconds starting at --outage-at
  noise<cm>               add N(0, <cm>) per horizontal axis to every absolute pose
  outlier<m>              add +<m> m in x to absolute poses for 3 s starting at --outage-at

--make-base --rtk-sigma S0,K rescales every recorded fix's own noise sample to the
synthetic_rtk model sigma(d) = S0 + K*d (z: 2x), so one recorded drive replays at another
RTK noise level with the same noise realization (see `rescale_fix`).

  tools/audit_rtk_robustness.py /archive/<recorder> --make-base            # once per recorder/mode
  tools/audit_rtk_robustness.py /archive/<recorder> --base /archive/<base run> --profile outage30 \
      --base-from-imu-source /archive/<terrain>/manifest.json
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from archive_guard import allocate_run, check_run  # noqa: E402
from run_paired_glim_replay import ABSOLUTE, runtime_configs  # noqa: E402
from selected_lidar_profile import TOPIC as POINTS  # noqa: E402

PREFIX = "/astrobot_0/"
IMU = PREFIX + "imu_physics"
GT = "/gt/base_link_pose"
LIVE = PREFIX + "slam_raw/pose_corrected"
INPUTS = ("/clock", POINTS, IMU, ABSOLUTE)
RTK = "/rtk/pose"
SUPERVISED_INPUTS = ("/clock", POINTS, IMU, PREFIX + "imu", PREFIX + "odom", RTK)
RTK_LEVER = np.array([0.264, 0.017, 0.262])  # synthetic_rtk antenna_offset_xyz (base frame)
RTK_SIGMA_Z_SCALE = 2.0  # synthetic_rtk sigma_z_scale
STATES = ("STARTING", "HEALTHY", "DEGRADED", "STOPPING", "REINITIALIZING", "REBUILDING", "FAULTED")


def parse(profile):
    out = dict(outage=0.0, noise=0.0, outlier=0.0)
    for term in profile.split(","):
        if term == "nominal":
            continue
        for key in out:
            if term.startswith(key):
                out[key] = float(term[len(key):])
                break
        else:
            raise ValueError("unknown profile term " + term)
    return out


def stamp_s(header_stamp):
    return header_stamp.sec + header_stamp.nanosec * 1e-9


def pose_matrix(p):
    from scipy.spatial.transform import Rotation

    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def fix_noise(fixes, truths):
    """Recover the per-fix noise of synthetic_rtk fixes: fix = (truth - home) + n, z unshifted.

    fixes, truths: (N, 3). Returns (noise (N, 3), offset = -home with z 0). The constant
    offset is the noise-free median, rounded to the cm (synthetic_rtk homes are cm values).
    """
    residual = np.asarray(fixes, float) - np.asarray(truths, float)
    median = np.median(residual[:, :2], 0)
    offset = np.append(np.round(median, 2), 0.0)
    if np.abs(median - offset[:2]).max() > 0.004:
        raise ValueError(f"fix offset {median} is not a cm home position")
    return residual - offset, offset


def rescale_fix(noise, cov, d, sigma0, sigma_per_m):
    """One fix's noise and 6x6 row-major covariance rescaled to sigma(d) = sigma0 + sigma_per_m * d."""
    sigma = sigma0 + sigma_per_m * d
    old = np.sqrt([cov[0], cov[7], cov[14]])
    new = np.array([sigma, sigma, sigma * RTK_SIGMA_Z_SCALE])
    cov = list(cov)
    cov[0], cov[7], cov[14] = (new ** 2).tolist()
    return np.asarray(noise) * new / old, cov


def rescale_stream(keep, edited_topic, cls, gt_rows, rtk_sigma):
    """Rewrite the edited stream's fixes at the rtk_sigma noise level; needs a GT pose per fix stamp."""
    from rclpy.serialization import deserialize_message, serialize_message

    gt = {round(t * 1e9): T for t, T in gt_rows}
    idx = [i for i, b in enumerate(keep) if b[0] == edited_topic]
    msgs = [deserialize_message(keep[i][1], cls) for i in idx]
    truths = []
    for i, m in zip(idx, msgs):
        T = gt.get(round(keep[i][3] * 1e9))
        if T is None:
            raise ValueError("fix without a same-stamp GT pose: not a synthetic_rtk stream")
        truths.append(T[:3, 3] + T[:3, :3] @ RTK_LEVER)
    fixes = [[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in msgs]
    noise, offset = fix_noise(fixes, truths)
    for i, m, n, truth in zip(idx, msgs, noise, truths):
        rel = truth + offset
        n, m.pose.covariance = rescale_fix(n, m.pose.covariance, float(np.hypot(rel[0], rel[1])), *rtk_sigma)
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = (rel + n).tolist()
        keep[i] = (keep[i][0], serialize_message(m), keep[i][2], keep[i][3])
    return dict(sigma0=rtk_sigma[0], sigma_per_m=rtk_sigma[1], offset=offset.tolist(), fixes=len(idx),
                old_noise_std=np.std(noise, 0).round(3).tolist())


def make_base(recorder, base, supervised, timing, rtk_sigma=None):
    """One shared input per recorder: every replayed topic except the edited one, plus its raw stream.

    timing="sim" maps each receipt onto the recorded /clock (interpolated at its receipt), so
    playback paces at sim speed with the live arrival order; "receipt" keeps live wall
    receipts (the live host ran the sim at ~0.15x real time, so playback is ~6x slower).
    """
    from rclpy.serialization import deserialize_message
    from rosbag2_py import (ConverterOptions, SequentialReader, SequentialWriter, StorageFilter, StorageOptions,
                            TopicMetadata)
    from rosidl_runtime_py.utilities import get_message

    inputs, edited_topic = (SUPERVISED_INPUTS, RTK) if supervised else (INPUTS, ABSOLUTE)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(recorder / "bag"), storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(StorageFilter(topics=[*inputs, GT, LIVE]))
    gt_cls = get_message(types[GT])
    clock_cls = get_message(types["/clock"])
    stamped = {t: get_message(types[t]) for t in inputs if t != "/clock"}
    gt_rows, live_rows, clock_rows, buffered = [], [], [], []
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        if topic in (GT, LIVE):
            m = deserialize_message(data, gt_cls)
            (gt_rows if topic == GT else live_rows).append((stamp_s(m.header.stamp), pose_matrix(m.pose)))
            continue
        if topic == "/clock":
            clock_rows.append((receipt, deserialize_message(data, clock_cls).clock))
            t = stamp_s(clock_rows[-1][1])
        else:
            t = stamp_s(deserialize_message(data, stamped[topic]).header.stamp)
        buffered.append((topic, data, receipt, t))
    # The supervisor launched the live estimator only once the rover had settled and
    # was stationary. Feed sensors from 1 s before the first live pose, never the drop.
    # Supervised: 3 s, so the supervisor itself finds the >= 1 s stationary reference.
    sensor_start = live_rows[0][0] - (3.0 if supervised else 1.0)
    receipts = np.array([r for r, _ in clock_rows], dtype=np.int64)
    sims = np.array([stamp_s(c) for _, c in clock_rows])
    keep = [b for b in buffered if b[3] >= sensor_start]
    if timing == "sim":
        order = np.argsort(receipts, kind="stable")
        retimed = np.interp([b[2] for b in keep], receipts[order], sims[order])
        keep = [(topic, data, int(round(t * 1e9)), stamp) for (topic, data, _, stamp), t in zip(keep, retimed)]
        keep.sort(key=lambda b: b[2])
    elif timing != "receipt":
        raise ValueError("timing must be sim or receipt")
    rescaled = rtk_sigma and rescale_stream(keep, edited_topic, get_message(types[edited_topic]), gt_rows, rtk_sigma)
    for name, topics in (("base_bag", [t for t in inputs if t != edited_topic]), ("edit_source", [edited_topic])):
        writer = SequentialWriter()
        writer.open(StorageOptions(uri=str(base / name), storage_id="mcap"), ConverterOptions("", ""))
        for i, topic in enumerate(topics):
            writer.create_topic(TopicMetadata(id=i, name=topic, type=types[topic], serialization_format="cdr"))
        for topic, data, receipt, _ in keep:
            if topic in topics:
                writer.write(topic, data, receipt)
        del writer
    np.savez_compressed(base / "reference.npz",
                        gt_t=np.array([r[0] for r in gt_rows]), gt=np.array([r[1] for r in gt_rows]),
                        live_t=np.array([r[0] for r in live_rows]), live=np.array([r[1] for r in live_rows]))
    abs_stamps = [b[3] for b in keep if b[0] == edited_topic]
    meta = dict(recorder=str(recorder), supervised=supervised, timing=timing, edited_topic=edited_topic,
                edited_type=types[edited_topic], sensor_start=sensor_start, absolute_span=[abs_stamps[0], abs_stamps[-1]],
                skipped_before_start=len(buffered) - len(keep), absolute_messages=len(abs_stamps),
                gt_messages=len(gt_rows), live_messages=len(live_rows), rtk_sigma=rescaled or None)
    (base / "base.json").write_text(json.dumps(meta, indent=2))
    return meta


def prepare(base, output, edit, outage_at_frac, seed):
    """Write this profile's edited absolute stream as a separate small bag next to the shared base."""
    from rclpy.serialization import deserialize_message, serialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, SequentialWriter, StorageOptions, TopicMetadata
    from rosidl_runtime_py.utilities import get_message

    meta = json.loads((base / "base.json").read_text())
    t0, t1 = meta["absolute_span"]
    start = t0 + outage_at_frac * (t1 - t0)
    window = dict(start=start, end=start + (edit["outage"] or (3.0 if edit["outlier"] else 0.0)))
    rng = np.random.default_rng(seed)
    abs_cls = get_message(meta["edited_type"])
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(base / "edit_source"), storage_id="mcap"), ConverterOptions("", ""))
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=str(output / "edited_bag"), storage_id="mcap"), ConverterOptions("", ""))
    writer.create_topic(TopicMetadata(id=0, name=meta["edited_topic"], type=meta["edited_type"],
                                      serialization_format="cdr"))
    dropped = edited = 0
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        if any(edit.values()):
            m = deserialize_message(data, abs_cls)
            t = stamp_s(m.header.stamp)
            inside = window["start"] <= t < window["end"]
            if edit["outage"] and inside:
                dropped += 1
                continue
            if edit["noise"]:
                m.pose.pose.position.x += rng.normal(0, edit["noise"] / 100)
                m.pose.pose.position.y += rng.normal(0, edit["noise"] / 100)
                edited += 1
            if edit["outlier"] and inside:
                m.pose.pose.position.x += edit["outlier"]
                edited += 1
            data = serialize_message(m)
        writer.write(topic, data, receipt)
    del writer
    return dict(meta, window=window, dropped=dropped, edited=edited)


def run_replay(recorder, base, output, rate, domain, supervised):
    """Play base + edited bags into GLIM (bare, or owned by the supervisor) and record the outputs."""
    manifest = json.loads((recorder / "runtime_manifest.json").read_text())
    configs = runtime_configs(manifest, expected_imu="imu_physics", selected_lidar=True)
    config = output / "config"
    config.mkdir()
    for name, content in configs.items():
        (config / name).write_text(content)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    if supervised:
        # The supervisor writes its own epoch configs (stationary RTK + gravity init, RTK topic).
        command = ["ros2", "run", "localization_supervisor", "localization_supervisor", "--ros-args",
                   "-r", "__ns:=/astrobot_0", "-p", "use_sim_time:=true", "-p", f"config_path:={config}"]
    else:
        command = ["ros2", "run", "glim_ros", "glim_rosnode", "--ros-args", "-r", "__node:=slam_raw",
                   "-r", "__ns:=/astrobot_0", "-p", "use_sim_time:=true", "-p", f"config_path:={config}",
                   "-p", f"dump_path:={output / 'glim_dump'}"]
    child = subprocess.Popen(command, env=env, stdout=(output / "glim.log").open("w"), stderr=subprocess.STDOUT,
                             start_new_session=True)
    rec = dict(raw=[], latency=[], published=[], status=[], resets=[], clears=0)
    done, ready = threading.Event(), threading.Event()

    def listen():
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile

        os.environ["ROS_DOMAIN_ID"] = str(domain)
        rclpy.init()
        node = rclpy.create_node("rtk_audit_listener", parameter_overrides=[Parameter("use_sim_time", value=True)])
        def raw(m):
            rec["raw"].append((stamp_s(m.header.stamp), pose_matrix(m.pose)))
            rec["latency"].append(node.get_clock().now().nanoseconds * 1e-9 - rec["raw"][-1][0])  # sim seconds

        node.create_subscription(PoseStamped, LIVE, raw, QoSProfile(depth=1000))
        if supervised:
            mock_navigation(node, rec)
        ready.set()
        while not done.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()

    listener = threading.Thread(target=listen, daemon=True)
    listener.start()
    ready.wait(30)
    time.sleep(8)  # node start-up (GLIM config load / supervisor discovery) before the first message
    began = time.time()
    play = subprocess.run(["ros2", "bag", "play", "-i", str(base / "base_bag"), "-i", str(output / "edited_bag"),
                           "--rate", str(rate)],
                          env=env, stdout=(output / "play.log").open("w"), stderr=subprocess.STDOUT)
    play_wall = time.time() - began
    # GLIM may lag the bag (bare, rate 2: several sim-seconds); drain until poses stop arriving
    count, quiet_since, deadline = -1, time.time(), time.time() + 120
    while time.time() < deadline and time.time() - quiet_since < 5:
        if len(rec["raw"]) != count:
            count, quiet_since = len(rec["raw"]), time.time()
        time.sleep(0.5)
    os.killpg(child.pid, signal.SIGINT)
    try:
        child.wait(timeout=120)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGKILL)
    done.set()
    listener.join(timeout=5)
    return rec, play.returncode, child.returncode, play_wall


def mock_navigation(node, rec):
    """The navigation side the supervisor waits on: map reset, costmap clears and a fresh BEV map.

    The BEV map is free space around the latest published pose: map *readiness*, never terrain.
    """
    from astrobot_interfaces.msg import LocalizationStatus
    from astrobot_interfaces.srv import ResetMap
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.srv import ClearEntireCostmap
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile

    ns = PREFIX
    last = {}

    def reset(request, response):
        rec["resets"].append(dict(epoch=int(request.epoch), t=node.get_clock().now().nanoseconds * 1e-9))
        response.success, response.epoch = True, request.epoch
        return response

    def clear(request, response):
        rec["clears"] += 1
        return response

    node.create_service(ResetMap, ns + "slam/reset_map", reset)
    for name in ("local_costmap/clear_entirely_local_costmap", "global_costmap/clear_entirely_global_costmap"):
        node.create_service(ClearEntireCostmap, ns + name, clear)
    node.create_subscription(LocalizationStatus, ns + "localization/status", lambda m: rec["status"].append(
        (stamp_s(m.header.stamp), int(m.state), int(m.epoch), bool(m.motion_permitted), m.reason)), QoSProfile(depth=1000))

    def published(m):
        last["pose"] = m
        rec["published"].append((stamp_s(m.header.stamp), pose_matrix(m.pose)))

    node.create_subscription(PoseStamped, ns + "slam/pose_corrected", published, QoSProfile(depth=1000))
    grid_pub = node.create_publisher(OccupancyGrid, ns + "slam/bev_costmap", 2)

    def publish_map():
        if "pose" not in last:
            return
        p = last["pose"].pose.position
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.info.resolution, msg.info.width, msg.info.height = 0.1, 40, 40
        msg.info.origin.position.x, msg.info.origin.position.y = p.x - 2.0, p.y - 2.0
        msg.data = [0] * 1600
        grid_pub.publish(msg)

    node.create_timer(0.2, publish_map)  # sim-time timer: 5 Hz in sim seconds at any rate


def score(poses, reference, base_from_imu, home_xyz, window):
    gt_t, gt = reference["gt_t"], reference["gt"]
    map_from_world = np.eye(4)
    map_from_world[:3, 3] = -np.asarray(home_xyz)

    def errors(rows):
        out = []
        for t, map_from_imu in rows:
            j = int(np.clip(np.searchsorted(gt_t, t), 1, len(gt_t) - 1))
            j = j if abs(gt_t[j] - t) < abs(gt_t[j - 1] - t) else j - 1
            if abs(gt_t[j] - t) > 0.02:
                continue
            est = map_from_imu @ np.linalg.inv(base_from_imu)
            truth = map_from_world @ gt[j]
            d = est[:3, 3] - truth[:3, 3]
            yaw = np.arctan2(est[1, 0], est[0, 0]) - np.arctan2(truth[1, 0], truth[0, 0])
            out.append((t, float(np.hypot(d[0], d[1])), float(abs(d[2])), float(np.degrees(abs(np.angle(np.exp(1j * yaw)))))))
        return np.array(out)

    def stats(e):
        if not len(e):
            return None
        return dict(samples=len(e), h_p50_cm=round(100 * float(np.median(e[:, 1])), 2),
                    h_p95_cm=round(100 * float(np.percentile(e[:, 1], 95)), 2), h_max_cm=round(100 * float(e[:, 1].max()), 2),
                    z_p95_cm=round(100 * float(np.percentile(e[:, 2], 95)), 2),
                    yaw_p95_deg=round(float(np.percentile(e[:, 3], 95)), 3))

    e = errors(poses)
    live = errors(list(zip(reference["live_t"], reference["live"])))
    result = dict(replay=stats(e), live_recorded=stats(live))
    if len(e) and window["end"] > window["start"]:
        during = e[(e[:, 0] >= window["start"]) & (e[:, 0] < window["end"])]
        after = e[e[:, 0] >= window["end"]]
        before = e[e[:, 0] < window["start"]]
        result.update(during=stats(during), after=stats(after), before=stats(before))
        if len(before) and len(after):
            limit = max(0.05, 2 * float(np.percentile(before[:, 1], 95)))
            bad = np.flatnonzero(after[:, 1] > limit)
            result["recovery_s"] = 0.0 if not len(bad) else round(float(after[bad[-1], 0] - window["end"]), 2)
            result["recovery_limit_cm"] = round(100 * limit, 2)
    if len(e):
        result["diverged_over_2m"] = bool(e[:, 1].max() > 2.0)
        result["coverage"] = dict(first=float(e[0, 0]), last=float(e[-1, 0]))
    return result, e


def supervision(rec, errors_published, window):
    """Status timeline: when motion was held / permitted, faults, restarts, error while permitted."""
    rows = sorted(rec["status"])
    if not rows:
        return dict(status_messages=0)
    t = np.array([r[0] for r in rows])
    permitted = np.array([r[3] for r in rows])
    out = dict(status_messages=len(rows), final_state=STATES[rows[-1][1]], final_reason=rows[-1][4],
               epochs=max(r[2] for r in rows), map_resets=len(rec["resets"]), costmap_clears=rec["clears"])
    first = np.flatnonzero(permitted)
    out["first_permitted_s"] = float(t[first[0]]) if len(first) else None
    faults, previous = [], None
    for ts, state, _, _, reason in rows:
        if STATES[state] in ("STOPPING", "FAULTED") and previous not in ("STOPPING", "FAULTED"):
            faults.append(dict(t=round(ts, 2), state=STATES[state], reason=reason))
        previous = STATES[state]
    out["faults"] = faults
    if len(first):
        span = t[-1] - t[first[0]]
        live = t >= t[first[0]]
        dt = np.diff(t[live], append=t[-1])
        out["permitted_fraction_after_first"] = round(float((dt * permitted[live]).sum() / span), 4) if span > 0 else None
    if window["end"] > window["start"]:
        held = np.flatnonzero((t >= window["start"]) & ~permitted)
        out["motion_held_after_s"] = round(float(t[held[0]] - window["start"]), 2) if len(held) else None
        if len(held):
            again = np.flatnonzero((t > t[held[0]]) & permitted)
            out["motion_permitted_again_after_window_s"] = (round(float(t[again[0]] - window["end"]), 2)
                                                           if len(again) else None)
    if len(errors_published) and len(first):
        # published (supervisor-accepted) pose error while motion was permitted
        idx = np.clip(np.searchsorted(t, errors_published[:, 0]) - 1, 0, len(t) - 1)
        ok = permitted[idx]
        out["published_h_max_cm_while_permitted"] = (round(100 * float(errors_published[ok, 1].max()), 2)
                                                     if ok.any() else None)
    return out


def latency(values):
    """GLIM pose_corrected receipt minus stamp, in sim seconds (supervisor pose_stale limit: 0.5 s)."""
    v = np.asarray(values)
    if not len(v):
        return None
    return dict(p50=round(float(np.median(v)), 3), p95=round(float(np.percentile(v, 95)), 3),
                p99=round(float(np.percentile(v, 99)), 3), max=round(float(v.max()), 3),
                over_0_5s=int((v > 0.5).sum()))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recorder", type=Path)
    parser.add_argument("--profile", default="nominal")
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    parser.add_argument("--rate", type=float, default=1.0, help="playback rate (of sim time with --timing sim)")
    parser.add_argument("--timing", choices=("sim", "receipt"), default="sim")
    parser.add_argument("--no-supervisor", action="store_true", help="bare GLIM on the recorded supervisor output")
    parser.add_argument("--base", type=Path, help="shared input from a prior --make-base run (else built here)")
    parser.add_argument("--make-base", action="store_true", help="only build the shared input for later --base")
    parser.add_argument("--rtk-sigma", type=lambda v: tuple(float(x) for x in v.split(",")),
                        help="with --make-base: S0,K -> fixes rescaled to sigma(d) = S0 + K*d")
    parser.add_argument("--domain", type=int, default=181)
    parser.add_argument("--outage-at", type=float, default=0.35, help="edit window start, fraction of the absolute-pose span")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--base-from-imu-source", type=Path,
                        help="live_local_v2 terrain manifest.json for this capture (base_from_imu, home_xyz)")
    args = parser.parse_args()
    recorder = args.recorder.resolve(strict=True)
    supervised = not args.no_supervisor
    output = allocate_run(args.archive_root, limit=8 * 1024**3)
    print(f"Run: {output}", flush=True)
    try:
        if args.make_base or args.base is None:
            base = output if args.make_base else output / "base"
            base.mkdir(exist_ok=True)
            make_base(recorder, base, supervised, args.timing, args.rtk_sigma)
            if args.make_base:
                check_run(output)
                print((base / "base.json").read_text())
                return
        else:
            base = args.base.resolve(strict=True)
        meta = json.loads((base / "base.json").read_text())
        if (meta["recorder"], meta["supervised"]) != (str(recorder), supervised):
            raise ValueError("base was built for another recorder or supervision mode")
        if args.base_from_imu_source is None:
            raise ValueError("--base-from-imu-source required to score")
        request = json.loads(args.base_from_imu_source.read_text())["request"]
        edit = parse(args.profile)
        prepared = prepare(base, output, edit, args.outage_at, args.seed)
        rec, play_rc, child_rc, play_wall = run_replay(recorder, base, output, args.rate, args.domain, supervised)
        base_from_imu = np.asarray(request["base_from_imu"])
        with np.load(base / "reference.npz") as ref:
            result, e = score(rec["raw"], ref, base_from_imu, request["home_xyz"], prepared["window"])
            published = None
            if supervised:
                published, ep = score(rec["published"], ref, base_from_imu, request["home_xyz"], prepared["window"])
                result["published"] = dict(replay=published["replay"], during=published.get("during"),
                                           after=published.get("after"))
                result["supervision"] = supervision(rec, ep, prepared["window"])
        np.save(output / "errors.npy", e)
        if supervised:  # supervisor timeline: (sim t, state, epoch, motion_permitted, reason)
            (output / "status.json").write_text(json.dumps(
                [[t, STATES[state], epoch, bool(permitted), reason] for t, state, epoch, permitted, reason in rec["status"]]))
        np.savez_compressed(output / "poses.npz", t=np.array([r[0] for r in rec["raw"]]),
                            map_from_imu=np.array([r[1] for r in rec["raw"]]).reshape(-1, 4, 4))
        report = dict(tool="audit_rtk_robustness", recorder=str(recorder), profile=args.profile, edit=edit,
                      rate=args.rate, timing=meta["timing"], supervised=supervised, base=str(base),
                      prepared=prepared, glim_poses=len(rec["raw"]), play_rc=play_rc, child_rc=child_rc,
                      play_wall_s=round(play_wall, 1), acceptance=False,
                      pose_latency_s=latency(rec["latency"]), **result)
        (output / "report.json").write_text(json.dumps(report, indent=2))
        check_run(output)
        print(json.dumps(report, indent=1))
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), profile=args.profile)))
        raise


if __name__ == "__main__":
    main()
