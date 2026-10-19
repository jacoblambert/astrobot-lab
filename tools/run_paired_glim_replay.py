#!/usr/bin/env python3
"""Frozen same-bag IMU A/B; no simulator, GT publication or production edits.

Preparation retains recorded CDR/receipt order after the first post-calibration
shadow request. Runs dispatch to installed GLIM callbacks, preserving counts.
Pose qualification is offline only, compatible with native terrain sensitivity.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
import resource
from pathlib import Path
import shutil
import subprocess

from archive_guard import allocate_run, check_run
from shadow_pose_replay import sha, load_pose_sensitivity

PREFIX = "/astrobot_0/"
IMUS = {"legacy": PREFIX+"imu", "physics": PREFIX+"imu_physics"}
POINTS = PREFIX+"lidar_0/pointcloud/filtered"
ABSOLUTE = PREFIX+"localization/absolute_pose"
TYPES = {"/clock": "rosgraph_msgs/msg/Clock", POINTS: "sensor_msgs/msg/PointCloud2",
         ABSOLUTE: "geometry_msgs/msg/PoseWithCovarianceStamped", **{v: "sensor_msgs/msg/Imu" for v in IMUS.values()}}


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def runtime_configs(manifest, expected_imu="imu", *, selected_lidar=False):
    processes = [p for p in manifest["processes"] if p.get("executable", {}).get("resolved_path", "").endswith("/glim_rosnode")]
    if len(processes) != 1:
        raise ValueError("one captured estimator process required")
    result = {}
    for f in processes[0]["runtime_files"]:
        content = f["content"]
        if hashlib.sha256(content.encode()).hexdigest() != f["sha256"]:
            raise ValueError("embedded runtime configuration hash mismatch")
        name = Path(f["path"]).name
        if name in result or not name.endswith(".json"):
            raise ValueError("ambiguous runtime configuration")
        result[name] = content
    selected = json.loads(result["config.json"])["global"]
    if any(selected["config_"+s] not in result for s in ("ros", "sensors", "odometry", "sub_mapping", "global_mapping")):
        raise ValueError("runtime configuration incomplete")
    ros = json.loads(result[selected["config_ros"]])
    if expected_imu not in ("imu", "imu_physics"):
        raise ValueError("undeclared captured IMU topic")
    points_topic = "lidar_0/pointcloud/filtered"
    if selected_lidar:
        from selected_lidar_profile import TOPIC
        if expected_imu != "imu_physics":
            raise ValueError("selected LiDAR requires declared physical IMU")
        points_topic = TOPIC
    if (ros["glim_ros"]["imu_topic"] != expected_imu or ros["glim_ros"]["points_topic"] != points_topic
            or ros["abs_constraint"]["rtk_topic"] != ABSOLUTE or not ros["abs_constraint"]["enable"]
            or ros["glim_ros"]["imu_time_offset"] != 0 or ros["glim_ros"]["points_time_offset"] != 0):
        raise ValueError("unexpected recorded input wiring")
    return result


def validate_dispatch(expected, dispatch, mode):
    if mode == "lidar_only":
        wanted = {k: v for k, v in expected.items() if k in (POINTS, "/clock")}
    elif mode in IMUS:
        wanted = {k: v for k, v in expected.items() if k != IMUS["physics" if mode == "legacy" else "legacy"]}
    else:
        raise ValueError("undeclared replay mode")
    if not dispatch.get("completed") or dispatch.get("mode") != mode or dispatch["counts"] != wanted:
        raise ValueError("input callback count mismatch")


def execution_configs(configs, profile):
    """Declared threading ablation; preserve all sensor, factor and geometry settings."""
    result = json.loads(json.dumps(configs))
    if profile == "recorded":
        return result
    if profile != "serial":
        raise ValueError("undeclared execution profile")
    selected = result["config.json"]["global"]
    result[selected["config_preprocess"]]["preprocess"]["num_threads"] = 1
    result[selected["config_odometry"]]["odometry_estimation"].update(num_threads=1, num_smoother_update_threads=1)
    return result


def absolute_candidate_config(config, plugin, policy):
    result = json.loads(json.dumps(config))
    modules = result["glim_ros"]["extension_modules"]
    candidates = [m for m in modules if Path(m).name in ("libabs_constraint_module.so", "libclosed_window_abs.so")]
    if len(candidates) != 1 or policy not in ("immediate", "closed-window"):
        raise ValueError("ambiguous absolute module/policy")
    result["glim_ros"]["extension_modules"] = [str(plugin) if m == candidates[0] else m for m in modules]
    result["abs_constraint"]["require_closed_window"] = policy == "closed-window"
    return result


def validate_inherited_candidates(configs, mode, backend, cpu_plugin, absolute_plugin):
    """A prepared candidate must not masquerade as the default/baseline arm."""
    if mode == "lidar_only":
        return  # Explicit CT ablation removes these modules and absolute factors.
    selected = configs["config.json"]["global"]
    odom = configs[selected["config_odometry"]]["odometry_estimation"]
    absolute = configs[selected["config_ros"]]["abs_constraint"]
    if backend == "cpu" and not cpu_plugin and any(odom.get(k, False) for k in
            ("revise_target_poses", "use_scan_geometry_information", "use_registration_mean_information")):
        raise ValueError("recorded CPU candidate requires explicit plugin and candidate policy flags")
    if absolute.get("enable") and absolute.get("require_closed_window") and not absolute_plugin:
        raise ValueError("recorded absolute candidate requires explicit plugin and association policy")


def gpu_configs(configs):
    """Backend ablation: frozen GPU registration settings, unchanged IMU/seed policy."""
    result = json.loads(json.dumps(configs))
    selected = result["config.json"]["global"]
    base = result[selected["config_odometry"]]["odometry_estimation"]
    gpu = result["config_odometry_gpu.json"]["odometry_estimation"]
    keys = ("so_name", "voxel_resolution", "voxel_resolution_max", "voxel_resolution_dmin",
            "voxel_resolution_dmax", "voxelmap_levels", "voxelmap_scaling_factor", "full_connection_window_size",
            "keyframe_update_strategy", "max_num_keyframes", "keyframe_min_overlap", "keyframe_max_overlap",
            "keyframe_delta_trans", "keyframe_delta_rot", "keyframe_entropy_thresh")
    if gpu["so_name"] != "libodometry_estimation_gpu.so": raise ValueError("unexpected GPU backend")
    candidate = dict(base, **{k: gpu[k] for k in keys})
    selected["config_odometry"] = "config_odometry_gpu_trial.json"
    result["config_odometry_gpu_trial.json"] = {"odometry_estimation": candidate}
    return result


def lidar_only_configs(configs, odometry):
    """Explicit backend ablation, NOT an IMU-topic-only controlled A/B.

    Retains point preprocessing and physical mount; CT states are LiDAR-centered
    with identity export extrinsic. No absolute prior or mapping extension may
    interpret those states as the physical IMU center.
    """
    result = json.loads(json.dumps(configs))
    selected = result["config.json"]["global"]
    sensors = result[selected["config_sensors"]]["sensors"]
    if sensors.get("global_shutter_lidar") is not True:
        raise ValueError("this trial requires the recorded rigid-scan approximation")
    ros = result[selected["config_ros"]]
    ros["glim_ros"].update(enable_local_mapping=False, enable_global_mapping=False,
        extension_modules=["librevision_module.so"], imu_frame_id="vlp16",
        imu_topic="disabled_lidar_only_imu", publish_imu2lidar=False, publish_odometry_tf=False)
    ros["abs_constraint"]["enable"] = False
    if odometry["odometry_estimation"]["so_name"] != "libodometry_estimation_ct.so":
        raise ValueError("wrong LiDAR-only backend")
    selected["config_odometry"] = "config_odometry_ct_trial.json"
    result["config_odometry_ct_trial.json"] = json.loads(json.dumps(odometry))
    return result


def prepare(args, output):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata, StorageFilter
    from rosidl_runtime_py.utilities import get_message
    from export_shadow_trajectory import stamp_ns

    capture, recorder, pair = args.capture, args.recorder, args.pair
    for folder in (capture, recorder, pair):
        check_run(folder)
    native = read(capture / "native_timing_audit.json")
    load_pose_sensitivity(pair, capture, native["frames"], "first_published")
    qualified = read(pair / "shadow_pair_audit.json")
    if str(recorder / "shadow_collection.json") not in qualified["input_sha256"]:
        raise ValueError("recorder not in qualified pair")
    upstream = {p: sha(Path(p)) for p in qualified["input_sha256"]}
    exported = [Path(p) for p in upstream if p.endswith("/shadow_trajectory.json")]
    if len(exported) != 1 or Path(read(exported[0])["source"]) != recorder:
        raise ValueError("missing recording provenance")
    for name, expected in read(exported[0])["input_sha256"].items():
        if Path(name).is_absolute() or ".." in Path(name).parts or sha(recorder/name) != expected:
            raise ValueError("recorded source changed")
        upstream[str(recorder/name)] = expected
    manifest = recorder/"runtime_manifest.json"
    natural = qualified.get("experiment") in ("natural_stationary_holdout", "natural_motion_gt_start")
    configs = runtime_configs(read(manifest), expected_imu="imu_physics" if natural else "imu")
    config = output/"config"
    config.mkdir()
    for name, content in configs.items():
        with (config/name).open("x") as stream:
            stream.write(content)
    upstream[str(manifest)] = sha(manifest)
    upstream[str(pair/"shadow_pair_audit.json")] = sha(pair/"shadow_pair_audit.json")
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(recorder/"bag"), storage_id="mcap"), ConverterOptions("", ""))
    reader.set_filter(StorageFilter(topics=[*TYPES, "/terrain_shadow/request"]))
    actual_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if any(actual_types.get(k) != v for k, v in TYPES.items()):
        raise ValueError("input bag type mismatch")
    classes = {t: get_message(kind) for t, kind in TYPES.items()}
    string_type = get_message("std_msgs/msg/String")
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=str(output/"inputs"), storage_id="mcap"), ConverterOptions("", ""))
    for i, (topic, kind) in enumerate(TYPES.items()):
        writer.create_topic(TopicMetadata(id=i, name=topic, type=kind, serialization_format="cdr"))
    counts, stamps, digests = Counter(), {t: [] for t in TYPES}, {t: hashlib.sha256() for t in TYPES}
    begin = previous = None
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        if topic == "/terrain_shadow/request":
            request = json.loads(deserialize_message(data, string_type).data)
            if request["capture"] != capture.name:
                raise ValueError("mixed capture request")
            if begin is None:
                begin = dict(receipt_ns=receipt, request=request)
            continue
        if begin is None:
            continue
        if previous is not None and receipt < previous:
            raise ValueError("source receipt ordering reversed")
        previous = receipt
        msg = deserialize_message(data, classes[topic])
        stamp = stamp_ns(msg.clock if topic == "/clock" else msg.header.stamp)
        if stamps[topic] and stamp <= stamps[topic][-1]:
            raise ValueError("duplicate/reversing source measurement")
        stamps[topic].append(stamp)
        writer.write(topic, data, receipt)
        counts[topic] += 1
        digests[topic].update(receipt.to_bytes(8,"little",signed=True)+bytes(data))
    del writer
    if not begin or any(counts[t] < 10 for t in TYPES):
        raise ValueError("missing post-calibration inputs")
    if stamps[POINTS][0] > round(native["frames"][0]["observed_stamp"]*1e9) or stamps[POINTS][-1] < round(native["frames"][-1]["observed_stamp"]*1e9):
        raise ValueError("clouds cannot bracket complete native route")
    result = dict(prepared=True, capture=str(capture), recorder=str(recorder), original_pair=str(pair),
        input_sha256=upstream, config_sha256={p.name: sha(p) for p in config.glob("*.json")},
        bag_sha256={p.name: sha(p) for p in (output/"inputs").iterdir() if p.is_file()},
        counts=dict(counts), per_topic_cdr_receipt_sha256={t: h.hexdigest() for t,h in digests.items()},
        measurement_bounds_ns={t: [v[0], v[-1]] for t,v in stamps.items()}, start=begin,
        source_selection="all permitted recorded inputs following first post-calibration shadow request; original order/CDR",
        direct_gt_published=False, assistance="unchanged recorded accepted synthetic RTK", fixed_home=[-20.,-20.,0.])
    write(output/"prepared.json", result)
    return result


def run_arm(args, output):
    from audit_shadow_pair import compare_pose_sequence
    from export_shadow_trajectory import apply_record
    from shadow_terrain import PoseTimeline
    check_run(args.prepared)
    prepared = read(args.prepared/"prepared.json")
    if not prepared.get("prepared") or (args.prepared/"failure.json").exists():
        raise ValueError("failed preparation")
    for path, expected in prepared["input_sha256"].items():
        if sha(Path(path)) != expected:
            raise ValueError("upstream evidence changed")
    for name, expected in prepared["bag_sha256"].items():
        if sha(args.prepared/"inputs"/name) != expected:
            raise ValueError("prepared inputs changed")
    for name, expected in prepared["config_sha256"].items():
        if sha(args.prepared/"config"/name) != expected:
            raise ValueError("prepared configuration changed")
    from ament_index_python.packages import get_package_prefix
    glim_library = Path(get_package_prefix("glim_ros"))/"lib/libglim_ros.so"
    symbols = subprocess.run(["nm", "-D", "-C", str(glim_library)], check=True, capture_output=True, text=True).stdout
    if "GlimROS::image_callback" not in symbols:
        raise ValueError("standalone probe requires the qualified CV-enabled GlimROS ABI")
    shutil.copy2(args.probe, output/"glim_shadow_replay")
    sources = output/"harness_sources"
    sources.mkdir()
    for source in (Path(__file__), Path(__file__).with_name("glim_shadow")/"replay.cpp",
                   Path(__file__).with_name("glim_shadow")/"CMakeLists.txt",
                   Path(__file__).with_name("glim_shadow")/"absolute_schedule.hpp",
                   Path(__file__).with_name("glim_shadow")/"diagnostics.hpp"):
        shutil.copy2(source, sources/source.name)
    dependencies = subprocess.run(["ldd", str(output/"glim_shadow_replay")], check=True,
                                  capture_output=True, text=True).stdout
    library_hashes = {word: sha(Path(word)) for word in dependencies.split() if word.startswith("/") and Path(word).is_file()}
    for name in ("libabs_constraint_module.so", "librevision_module.so", "librviz_viewer.so"):
        path = glim_library.parent/name
        library_hashes[str(path)] = sha(path)
    shutil.copytree(args.prepared/"config", output/"config")
    validate_inherited_candidates({p.name: read(p) for p in (output/"config").glob("*.json")},
        args.mode, getattr(args, "backend", "cpu"), getattr(args, "cpu_plugin", None),
        getattr(args, "absolute_plugin", None))
    ros_path = output/"config/config_ros.json"
    ros = read(ros_path)
    ros["glim_ros"]["imu_topic"] = "imu" if args.mode == "legacy" else "imu_physics"
    # Exclusive input copy; this is the one declared A/B change.
    with ros_path.open("w") as stream:
        json.dump(ros, stream, indent=2)
    seed = None
    initial_state_source = getattr(args, "initial_state_source", "recorded")
    if initial_state_source == "gt-start":
        from glim_gt_start import initial_state, recorded_gt
        if args.mode not in IMUS:
            raise ValueError("GT-start requires an IMU-centered backend and unchanged RTK")
        times, poses = recorded_gt(prepared["recorder"])
        link = read(Path(prepared["recorder"])/"shadow_collection.json")
        seed = initial_state(times, poses, prepared["measurement_bounds_ns"][POINTS][0], prepared["fixed_home"],
                             link["request"]["base_from_imu"], link["request"]["base_from_lidar"])
        selected = read(output/"config/config.json")["global"]
        path = output/"config"/selected["config_odometry"]
        odometry = read(path)
        odometry["odometry_estimation"].update({k: seed[k] for k in ("init_T_world_imu", "init_v_world_imu")})
        with path.open("w") as stream:
            json.dump(odometry, stream, indent=2)
        write(output/"gt_initial_state.json", seed)
        shutil.copy2(Path(__file__).with_name("glim_gt_start.py"), sources/"glim_gt_start.py")
    if args.mode == "lidar_only":
        odometry_source = Path(__file__).resolve().parents[1]/"astrobot-lab/src/glim_slam_launch/config/glim_astrobot_lidar_only/config_odometry_ct.json"
        shutil.copy2(odometry_source, sources/odometry_source.name)
        configs = lidar_only_configs({p.name: read(p) for p in (output/"config").glob("*.json")}, read(odometry_source))
        for name, content in configs.items():
            with (output/"config"/name).open("w") as stream:
                json.dump(content, stream, indent=2)
        plugin = Path(get_package_prefix("glim"))/"lib/libodometry_estimation_ct.so"
        library_hashes[str(plugin)] = sha(plugin)
    execution_profile = getattr(args, "execution_profile", "recorded")
    if execution_profile != "recorded":
        configs = execution_configs({p.name: read(p) for p in (output/"config").glob("*.json")}, execution_profile)
        for name, content in configs.items():
            with (output/"config"/name).open("w") as stream:
                json.dump(content, stream, indent=2)
    target_policy = getattr(args, "target_policy", "frozen")
    scan_information = getattr(args, "scan_information", "isotropic")
    cpu_plugin = getattr(args, "cpu_plugin", None)
    backend = getattr(args, "backend", "cpu")
    if backend == "gpu":
        if cpu_plugin or target_policy != "frozen" or scan_information != "isotropic" or args.mode not in IMUS:
            raise ValueError("GPU backend cannot silently combine CPU/LiDAR-only ablations")
        configs = gpu_configs({p.name: read(p) for p in (output/"config").glob("*.json")})
        for name, content in configs.items():
            with (output/"config"/name).open("w") as stream: json.dump(content, stream, indent=2)
        plugin = Path(get_package_prefix("glim"))/"lib/libodometry_estimation_gpu.so"
        library_hashes[str(plugin)] = sha(plugin)
    if (target_policy != "frozen" or scan_information != "isotropic") and cpu_plugin is None:
        raise ValueError("target/information candidate requires an explicit frozen candidate plugin")
    if cpu_plugin is not None:
        if args.mode not in IMUS or initial_state_source != "gt-start" or execution_profile != "serial":
            raise ValueError("CPU candidate requires the declared serial assisted GT-start experiment")
        plugin = output/"librevision_target_cpu.so"
        shutil.copy2(cpu_plugin, plugin)
        library_hashes[str(plugin)] = sha(plugin)
        selected = read(output/"config/config.json")["global"]
        path = output/"config"/selected["config_odometry"]
        odometry = read(path)
        odometry["odometry_estimation"].update(so_name=str(plugin), revise_target_poses=target_policy == "smoothed-window")
        odometry["odometry_estimation"]["use_scan_geometry_information"] = scan_information == "geometry"
        odometry["odometry_estimation"]["use_registration_mean_information"] = scan_information == "registration-mean"
        with path.open("w") as stream: json.dump(odometry, stream, indent=2)
        source_root = Path(__file__).resolve().parents[1]/"astrobot-lab/src/glim"
        for name in ("src/glim/odometry/odometry_estimation_cpu.cpp",
                     "src/glim/odometry/odometry_estimation_cpu_create.cpp",
                     "include/glim/odometry/scan_geometry_information.hpp",
                     "include/glim/odometry/odometry_estimation_cpu.hpp"):
            shutil.copy2(source_root/name, sources/Path(name).name)
    env = dict(os.environ, ROS_DOMAIN_ID="179", ROS_LOCALHOST_ONLY="1",
               CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>',
               ROS_LOG_DIR=str(output/"ros-log"))
    env["GLIM_SHADOW_TRACE"] = "1" if getattr(args, "trace", False) else "0"
    absolute_schedule = getattr(args, "absolute_schedule", "recorded")
    if absolute_schedule != "recorded" and (args.mode not in IMUS or execution_profile != "serial" or seed is None):
        raise ValueError("scheduled absolute diagnostic requires serial assisted GT-start")
    env["GLIM_SHADOW_ABSOLUTE_SCHEDULE"] = absolute_schedule
    association_policy = getattr(args, "association_policy", "immediate")
    absolute_plugin = getattr(args, "absolute_plugin", None)
    if association_policy != "immediate" and absolute_plugin is None:
        raise ValueError("closed-window policy requires explicit candidate plugin")
    if absolute_plugin:
        if absolute_schedule != "recorded" or args.mode not in IMUS:
            raise ValueError("absolute candidate must use original receipt delivery and assisted inputs")
        plugin = output/"libclosed_window_abs.so"
        shutil.copy2(absolute_plugin, plugin)
        library_hashes[str(plugin)] = sha(plugin)
        ros = absolute_candidate_config(read(ros_path), plugin, association_policy)
        with ros_path.open("w") as stream: json.dump(ros, stream, indent=2)
        source_root = Path(__file__).resolve().parents[1]/"astrobot-lab/src/glim_ros2"
        for name in ("src/glim_ros/abs_constraint_module.cpp", "include/glim_ros/abs_constraint_module.hpp",
                     "include/glim_ros/absolute_association.hpp"):
            shutil.copy2(source_root/name, sources/Path(name).name)
    command = [str(output/"glim_shadow_replay"), str(args.prepared/"inputs"), str(output), args.mode, "--ros-args",
               "-r", "__node:=slam_raw", "-r", "__ns:=/astrobot_0", "-p", "use_sim_time:=false",
               "-p", "config_path:="+str(output/"config")]
    write(output/"attempt.json", dict(command=command, mode=args.mode, probe_sha256=sha(output/"glim_shadow_replay"),
        library_sha256=library_hashes,
        initial_state_source=initial_state_source,
        backend=backend,
        execution_profile=execution_profile,
        target_policy=target_policy,
        scan_information=scan_information,
        cpu_plugin_sha256=sha(output/"librevision_target_cpu.so") if cpu_plugin else None,
        absolute_schedule=absolute_schedule,
        association_policy=association_policy,
        trace_enabled=getattr(args, "trace", False),
        source_snapshot_scope="working-tree context, not a build attestation; executed binaries are copied and hashed separately",
        prepared=str(args.prepared), prepared_sha256=sha(args.prepared/"prepared.json"),
        offline_only=True, note="same recorded receipt pace; direct callbacks; no online latency claim"))
    with (output/"replay.log").open("x") as log:
        usage_before = resource.getrusage(resource.RUSAGE_CHILDREN)
        process = subprocess.Popen(command, env=env, stdout=log, stderr=log, start_new_session=True)
        try:
            code = process.wait(timeout=600)
        finally:
            if process.poll() is None:
                import signal
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            usage_after = resource.getrusage(resource.RUSAGE_CHILDREN)
            write(output/"process_resources.json", dict(
                children_peak_rss_kib=usage_after.ru_maxrss,
                replay_user_seconds=usage_after.ru_utime-usage_before.ru_utime,
                replay_system_seconds=usage_after.ru_stime-usage_before.ru_stime,
                note="Peak RSS upper bound includes earlier small nm/ldd children; CPU covers replay child. Not an idle-host timing or live acceptance gate."))
    if code:
        raise ValueError(f"GLIM replay failed with exit {code}; retained replay.log")
    # Capture actually mapped libraries, including dlopen plugins omitted by ldd.
    mapped = set()
    for line in (output/"loaded_maps.txt").read_text().splitlines():
        fields = line.split()
        if len(fields) >= 6 and fields[-1].startswith("/") and ".so" in fields[-1]:
            mapped.add(fields[-1])
    write(output/"loaded_libraries.json", {p: sha(Path(p)) for p in sorted(mapped)})
    dispatch = read(output/"dispatch.json")
    validate_dispatch(prepared["counts"], dispatch, args.mode)
    log = (output/"replay.log").read_text()
    for phrase in ("skip an invalid", "smoother corruption", "IndeterminantLinearSystem", "failed to extract points"):
        if phrase in log:
            raise ValueError("estimator rejected input/corrupted state: "+phrase)
    timeline = None
    for line in (output/"trajectory.jsonl").read_text().splitlines():
        record = json.loads(line)
        if timeline is None:
            timeline = PoseTimeline(record["session"], record["epoch"])
        apply_record(timeline, record)
    if timeline is None:
        raise ValueError("no replay trajectory")
    if seed is not None:
        from glim_gt_start import verify_first_output
        first = verify_first_output([json.loads(line) for line in (output/"trajectory.jsonl").read_text().splitlines()], seed)
        write(output/"initial_output_check.json", first)
        if not first["passed"]:
            raise ValueError("GT-start first-output accuracy gate failed; moving results not qualified")
    capture = Path(prepared["capture"])
    native = read(capture/"native_timing_audit.json")
    if args.mode == "lidar_only":
        import numpy as np
        from audit_glim_anchor import diagnose
        # The exported field is named map_from_imu for wire compatibility, but
        # CT declares FrameID::LIDAR and identity T_lidar_imu. Never apply the
        # physical IMU mount to this state a second time.
        raw_records = [json.loads(line) for line in (output/"trajectory.jsonl").read_text().splitlines()]
        if any(not np.allclose(f["imu_from_lidar"], np.eye(4), rtol=0, atol=1e-10)
               for record in raw_records for f in record["frames"]):
            raise ValueError("LiDAR-only export is not LiDAR-centered")
        truth = np.array([f["world_from_lidar"] for f in native["frames"]])
        truth[:, :3, 3] -= prepared["fixed_home"]
        link = read(Path(prepared["recorder"])/"shadow_collection.json")
        lidar_from_base = np.linalg.inv(np.asarray(link["request"]["base_from_lidar"]))
        results = {}
        for mode in ("causal", "revised"):
            estimated = np.array([timeline.associate(round(f["observed_stamp"]*1e9),
                timeline.last_arrival_ns, mode=mode).map_from_lidar for f in native["frames"]])
            results[mode] = {"lidar": diagnose(estimated, truth),
                             "base_link": diagnose(estimated@lidar_from_base, truth@lidar_from_base)}
        result = dict(completed=True, acceptance=False, experiment="lidar_only_rigid_scan_backend_ablation",
            chunks=len(native["frames"]), dispatched_counts=dispatch["counts"], results=results,
            gt_in_estimator=False, rtk_in_estimator=False, imu_in_estimator=False,
            frame_contract="CT state is LiDAR, exported identity extrinsic; physical base mount applied to BOTH trajectories",
            alignment="diagnostic initial native sample only; not a full-trajectory fit or a mapper correction",
            scan_timing="unchanged global_shutter approximation, no invented point times; moving deskew NOT qualified",
            input_sha256={str(args.prepared/"prepared.json"): sha(args.prepared/"prepared.json"),
                **{str(output/name): sha(output/name) for name in ("attempt.json", "dispatch.json", "trajectory.jsonl", "replay.log")},
                **{str(p): sha(p) for p in (output/"config").glob("*.json")}})
        write(output/"lidar_only_audit.json", result)
        return result
    rows, stats = compare_pose_sequence(timeline, native["frames"], prepared["fixed_home"])
    with (output/"paired_poses.jsonl").open("x") as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False)+"\n")
    provenance = {str(capture/"native_timing_audit.json"): sha(capture/"native_timing_audit.json"),
                  str(args.prepared/"prepared.json"): sha(args.prepared/"prepared.json"),
                  **prepared["input_sha256"]}
    for name in ("dispatch.json", "attempt.json", "trajectory.jsonl", "replay.log", "glim_shadow_replay",
                 "runtime_gravity.json", "loaded_maps.txt", "loaded_libraries.json", "process_resources.json"):
        provenance[str(output/name)] = sha(output/name)
    if cpu_plugin:
        provenance[str(output/"librevision_target_cpu.so")] = sha(output/"librevision_target_cpu.so")
    if absolute_plugin:
        provenance[str(output/"libclosed_window_abs.so")] = sha(output/"libclosed_window_abs.so")
    if getattr(args, "trace", False):
        provenance[str(output/"estimator_trace.jsonl")] = sha(output/"estimator_trace.jsonl")
    if seed is not None:
        for name in ("gt_initial_state.json", "initial_output_check.json"):
            provenance[str(output/name)] = sha(output/name)
    for path in (output/"config").glob("*.json"):
        provenance[str(path)] = sha(path)
    result = dict(passed=True, chunks=len(rows), errors=stats, session=timeline.session, epoch=timeline.epoch,
        fixed_home=prepared["fixed_home"], paired_poses_sha256=sha(output/"paired_poses.jsonl"), input_sha256=provenance,
        experiment="gt_start_accuracy" if seed is not None else "same_recorded_inputs_IMU_AB",
        initial_state_source=initial_state_source, gt_initialization_only=seed is not None,
        backend=backend,
        execution_profile=execution_profile,
        target_policy=target_policy,
        scan_information=scan_information,
        absolute_schedule=absolute_schedule,
        association_policy=association_policy,
        mode=args.mode, dispatched_counts=dispatch["counts"],
        causal_map_projection_evaluated=False, availability_timing_validated=False, terrain_validation="NOT_RUN",
        navigation_acceptance=False, gt_published=False)
    write(output/"shadow_pair_audit.json", result)
    return result


def validate_ab_settings(left, right, left_configs, right_configs):
    if (left["mode"], right["mode"]) != ("legacy", "physics"):
        raise ValueError("declared order must be legacy then physics")
    if left.get("initial_state_source", "recorded") != "recorded" or right.get("initial_state_source", "recorded") != "recorded":
        raise ValueError("IMU A/B cannot include GT initialization")
    for key in ("prepared", "prepared_sha256", "probe_sha256", "library_sha256"):
        if left[key] != right[key]:
            raise ValueError("A/B changed more than IMU input: "+key)
    a, b = json.loads(json.dumps(left_configs)), json.loads(json.dumps(right_configs))
    if a["config_ros.json"]["glim_ros"]["imu_topic"] != "imu" or b["config_ros.json"]["glim_ros"]["imu_topic"] != "imu_physics":
        raise ValueError("wrong estimator IMU selection")
    b["config_ros.json"]["glim_ros"]["imu_topic"] = "imu"
    if a != b:
        raise ValueError("A/B configuration differs beyond the declared IMU topic")


def compare(args, output):
    runs = (args.legacy, args.physics)
    attempts = [read(p/"attempt.json") for p in runs]
    configs = [{p.name: read(p) for p in (run/"config").glob("*.json")} for run in runs]
    validate_ab_settings(*attempts, *configs)
    reports = []
    for run in runs:
        check_run(run)
        report = read(run/"shadow_pair_audit.json")
        if not report.get("passed") or (run/"failure.json").exists():
            raise ValueError("failed replay cannot enter A/B")
        for path, expected in report["input_sha256"].items():
            if sha(Path(path)) != expected:
                raise ValueError("replay provenance changed")
        if sha(run/"paired_poses.jsonl") != report["paired_poses_sha256"]:
            raise ValueError("replay poses changed")
        reports.append(report)
    if reports[0]["chunks"] != reports[1]["chunks"] or reports[0]["fixed_home"] != reports[1]["fixed_home"]:
        raise ValueError("A/B evaluation differs")
    result = dict(comparable=True, only_estimator_config_difference="imu_topic", chunks=reports[0]["chunks"],
                  errors={r["mode"]: r["errors"] for r in reports},
                  input_sha256={str(r/"shadow_pair_audit.json"): sha(r/"shadow_pair_audit.json") for r in runs},
                  terrain_validation="NOT_RUN", causal_timing_acceptance=False)
    write(output/"comparison.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=Path("/archive"))
    sub = parser.add_subparsers(dest="action", required=True)
    prep = sub.add_parser("prepare")
    for name in ("capture", "recorder", "pair"):
        prep.add_argument(name, type=Path)
    arm = sub.add_parser("run")
    arm.add_argument("prepared", type=Path)
    arm.add_argument("--mode", choices=[*IMUS, "lidar_only"], required=True)
    arm.add_argument("--backend", choices=["cpu", "gpu"], default="cpu")
    arm.add_argument("--initial-state-source", choices=["recorded", "gt-start"], default="recorded")
    arm.add_argument("--trace", action="store_true", help="read-only GLIM stage/factor diagnostics; excludes timing acceptance")
    arm.add_argument("--execution-profile", choices=["recorded", "serial"], default="recorded")
    arm.add_argument("--cpu-plugin", type=Path, help="standalone non-installed CPU candidate; frozen into each attempt")
    arm.add_argument("--target-policy", choices=["frozen", "smoothed-window"], default="frozen")
    arm.add_argument("--scan-information", choices=["isotropic", "geometry", "registration-mean"], default="isotropic")
    arm.add_argument("--absolute-schedule", choices=["recorded", "measurement"], default="recorded",
                     help="measurement-time snapshots are explicitly NON-CAUSAL offline diagnostics")
    arm.add_argument("--absolute-plugin", type=Path)
    arm.add_argument("--association-policy", choices=["immediate", "closed-window"], default="immediate")
    arm.add_argument("--probe", type=Path, required=True)
    ab = sub.add_parser("compare")
    ab.add_argument("legacy", type=Path)
    ab.add_argument("physics", type=Path)
    args = parser.parse_args()
    output = allocate_run(args.archive_root)
    print(f"Run: {output}", flush=True)
    try:
        result = {"prepare": prepare, "run": run_arm, "compare": compare}[args.action](args, output)
        print(json.dumps({k:v for k,v in result.items() if k not in ("input_sha256", "config_sha256", "start")}, indent=2))
    except Exception as exc:
        write(output/"failure.json", dict(error=str(exc), type=type(exc).__name__))
        raise


if __name__ == "__main__":
    main()
