"""Whole-route native evidence checks for the explicit per-return sensor layout."""

import numpy as np


def origin_counterfactuals(outlier, sensor_origin, cast):
    """Diagnostic-only queries; never replace a failed first-hit reference.

    Same direction/end extent, origins before/after the frozen 0.5 m start.
    Record all first hits, even self hits or mismatches, without choosing one.
    """
    direction = np.asarray(outlier["ray_direction"])
    result = []
    for offset in (0.0, 0.49, 0.51):
        origin = np.asarray(sensor_origin) + offset * direction
        distance = float(outlier["native_depth_m"]) + 0.1 - offset
        hits = cast(tuple(origin), tuple(direction), distance)
        result.append(
            dict(
                offset_m=offset,
                ray_origin=origin.tolist(),
                ray_direction=direction.tolist(),
                hits=[
                    dict(
                        distance_from_origin_m=float(h[0]),
                        distance_from_sensor_m=float(h[0]) + offset,
                        position=list(map(float, h[1])),
                        normal=list(map(float, h[2])),
                        collision=str(h[3]),
                    )
                    for h in hits[:8]
                ],
                hit_count=len(hits),
                diagnostic_only=True,
            )
        )
    return result


def audit_transport(capture, recorder):
    """Reproduce each native acquisition's exact bytes in recorded ROS windows.

    Window boundaries may straddle measurement start/end; every measurement tick
    still needs its recorded containing window. No trimming a missing tail.
    """
    import hashlib
    import json
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from generic_lidar_adapter import DTYPE, FIELDS, NativeWindow
    from selected_lidar_profile import TOPIC, profile_path
    from local_terrain.self_filter import chassis_returns

    sensor = json.loads((capture / "selected_sensor.json").read_text())
    profile = sensor["profile"]
    frames = json.loads((capture / "native_timing_audit.json").read_text())["frames"]
    windows = [json.loads(line) for line in (capture / "selected_windows.jsonl").read_text().splitlines()]
    wanted = {f["producer_sequence"] for f in frames}
    metadata = {w["stamp_ns"]: w for w in windows if wanted.intersection(w["sequences"])}
    if len(metadata) < 2:
        raise ValueError("missing selected window provenance")
    by_sequence = {f["producer_sequence"]: f for f in frames}
    if len(by_sequence) != len(frames):
        raise ValueError("duplicate archived native sequence")
    adapter = NativeWindow(
        profile_path(profile["name"]), timestamp_policy=sensor.get("timestamp_policy", "nearest_ns_v1")
    )
    checked = set()
    points_checked = 0
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(recorder / "bag"), storage_id="mcap"), ConverterOptions("", ""))
    observed = set()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != TOPIC:
            continue
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp not in metadata:
            continue  # Explicitly outside the native measurement interval.
        spec = metadata[stamp]
        if (
            stamp in observed
            or msg.header.frame_id != "vlp16"
            or msg.height != 1
            or msg.is_bigendian
            or msg.width != spec["points"]
            or msg.point_step != 32
            or msg.row_step != msg.width * 32
            or len(msg.data) != msg.row_step
            or tuple((f.name, f.offset, f.datatype, f.count) for f in msg.fields) != FIELDS
            or hashlib.sha256(bytes(msg.data)).hexdigest() != spec["sha256"]
        ):
            raise ValueError("recorded selected window bytes/schema/identity differ")
        observed.add(stamp)
        rows = np.frombuffer(msg.data, dtype=DTYPE)
        for sequence, acquired in zip(spec["sequences"], spec["native_stamps"]):
            if sequence in wanted:
                if sequence in checked:
                    raise ValueError("duplicate native sequence in recorded windows")
                ns = adapter.stamp_ns(acquired)
                tick_rows = rows[rows["time"] == (ns - stamp) / 1e9]
                frame = by_sequence[sequence]
                path = capture / "native_chunks" / frame["file"]
                if path.name != frame["file"] or hashlib.sha256(path.read_bytes()).hexdigest() != frame["sha256"]:
                    raise ValueError("native source changed or escaped archive")
                with np.load(path) as raw:
                    acquired = float(raw["observed_stamp"])
                    if acquired != frame["observed_stamp"] or not np.array_equal(
                        raw["world_from_lidar"], frame["world_from_lidar"]
                    ):
                        raise ValueError("native array pose/stamp differs from audit")
                    adapter.push(
                        len(checked),
                        acquired,
                        *(raw[k] for k in ("points", "depths", "intensities", "azimuth", "zenith"))
                    )
                    valid = (raw["intensities"] > 0) & (raw["depths"] > 0.50001) & (raw["depths"] < 100.0)
                    valid &= ~chassis_returns(raw["points"], adapter.setup)
                    xyz = np.column_stack([tick_rows[k] for k in ("x", "y", "z")])
                    from local_terrain.ray_design import native_pattern_indices

                    ids = native_pattern_indices(raw["azimuth"], raw["zenith"], adapter.azimuth, adapter.elevations)[
                        valid
                    ]
                    if (
                        ns != adapter.stamp_ns(acquired)
                        or not np.array_equal(xyz, raw["points"][valid].astype(np.float32))
                        or not np.array_equal(tick_rows["intensity"], raw["intensities"][valid])
                        or not np.array_equal(tick_rows["beam_id"], ids)
                        or not np.array_equal(tick_rows["ring"], ids % 32)
                    ):
                        raise ValueError("native returns/tick association changed in ROS transport")
                    points_checked += len(tick_rows)
                checked.add(sequence)
    if observed != set(metadata) or checked != wanted:
        raise ValueError("recording lost selected windows or native acquisition tail")
    # This assembler's arbitrary archive start is not a producer-window boundary.
    # All native ticks were checked and each has an actual full recorded window;
    # don't use finish() to pretend these arbitrarily grouped ticks were emitted.
    return dict(
        native_ticks=len(frames),
        recorded_windows=len(observed),
        points_checked=points_checked,
        all_measurement_ticks_including_boundary_windows_verified=True,
    )


def validate_native(audit, *, motion="straight_slow", profile="selected_generic_v1"):
    from selected_lidar_profile import EXTENDED_PATTERNS, PROFILES, acquisition_limits
    from costmap_scenarios import SCENARIOS

    limits = acquisition_limits(motion)
    if (
        motion in EXTENDED_PATTERNS
        and audit.get("acquisition_limits") != limits
        or "acquisition_limits" in audit
        and audit["acquisition_limits"] != limits
    ):
        raise ValueError("selected acquisition bounds differ from declared route")
    frames = audit["frames"]
    if (
        profile not in PROFILES
        or audit.get("profile") != profile
        or audit.get("duplicate_native_reads") != 0
        or len(frames) < 2
    ):
        raise ValueError("invalid selected native audit identity/count")
    stamps = np.array([f["observed_stamp"] for f in frames])
    if not np.isfinite(stamps).all() or not np.allclose(np.diff(stamps), 1 / 60, atol=1e-7, rtol=0):
        raise ValueError("selected native acquisition gap/reset")
    if not 6 <= stamps[-1] - stamps[0] <= limits["max_duration_s"] or len(frames) > limits["max_native_frames"]:
        raise ValueError("selected moving duration outside frozen motion gate")
    for key in ("world_time", "timeline_time"):
        clocks = np.array([f[key] for f in frames])
        if not np.isfinite(clocks).all() or not np.allclose(np.diff(clocks), np.diff(stamps), atol=1e-7, rtol=0):
            raise ValueError("selected acquisition clock disagreement")
    if not np.all(np.diff([f["producer_sequence"] for f in frames]) == 1) or not np.all(
        np.diff([f["world_step"] for f in frames]) == 1
    ):
        raise ValueError("selected native producer/physics skipped steps")
    for index, frame in enumerate(frames):
        if (
            frame.get("ray_layout") != "per_return_generic"
            or frame.get("samples") != 10661
            or any(abs(frame[k] - 1 / 60) > 1e-9 for k in ("world_physics_dt", "world_rendering_dt"))
        ):
            raise ValueError("selected native dose/layout changed")
        surface = frame["current_pose"]
        errors = np.asarray(surface["residuals_m"])
        if (
            surface["selected"] != 128
            or surface["missing"]
            or len(errors) != 128
            or not np.isfinite(errors).all()
            or np.max(errors) > 0.001
        ):
            maximum = float(np.max(errors)) if len(errors) else None
            raise ValueError(
                f"selected first-hit reference gate failed at tick {index}, stamp {frame['observed_stamp']:.9f}: "
                f"selected={surface['selected']}/128, missing_hits={surface['missing']}, "
                f"successful_residuals={len(errors)}/128, max_residual_m={maximum}; "
                "requires all 128 finite first hits within 0.001 m; this is a PhysX reference check, not GLIM error"
            )
    nonhorizontal = sum(f["current_pose"]["nonhorizontal_surface_samples"] for f in frames)
    if nonhorizontal < 100:
        raise ValueError("insufficient nonhorizontal geometry for native timing check")
    residuals = np.array([e for f in frames for e in f["current_pose"]["residuals_m"]])
    return dict(
        chunks=len(frames),
        duration_s=float(stamps[-1] - stamps[0]),
        samples=len(residuals),
        maximum_m=float(residuals.max()),
        p95_m=float(np.percentile(residuals, 95)),
        nonhorizontal_surface_samples=nonhorizontal,
        cadence_passed=True,
        source="per-return explicit rays; every native physics tick",
        navigation_acceptance=False,
    )
