#!/usr/bin/env python3
"""Parity harness: Python live_local_v2 nav semantics vs the C++ astrobot_terrain pipeline.

  generate <run_dir>                     synthetic drive in the raw archive layout (events.jsonl + native.bin)
  reference <run_dir> <out_dir> <lib>    replay through the Python reference (nav deployment of
                                         live_local_terrain.py) with a nav-flag terrain library
  robot <path>                           robot.json for terrain_replay (same geometry as the reference)
  compare <a_dir> <b_dir>                compare published windows and final mosaics

Run the C++ side with: terrain_replay <run_dir> <out_dir> <robot.json>. An archived nav run with
archive_raw:=true (events.jsonl + native.bin) replays the same way.
"""

import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from local_terrain.nav2_contract import Footprint, Nav2Config, clear_footprint, grid_pose, planner_costs  # noqa: E402
from local_terrain.navigation_cost import PUBLISHED_POLICIES  # noqa: E402
from local_terrain.global_stitch import GlobalStitch  # noqa: E402
from local_terrain.costmap import map_from_grid  # noqa: E402
from native_chunk_transport import DTYPE  # noqa: E402

REQUEST = Path(__file__).resolve().parents[2] / "research/legacy/astrobot_launch/config/v2_nav_request.json"
PROFILE = Path(__file__).resolve().parent / "local_terrain/selected_lidar_optical.json"
CAPTURE = "revision-" + "5a" * 16


def geometry():
    request = json.loads(REQUEST.read_text())
    setup = json.loads(PROFILE.read_text())
    return np.asarray(request["base_from_imu"]), setup


def robot(path):
    base_from_imu, setup = geometry()
    nav2 = Nav2Config.from_yaml()
    Path(path).write_text(json.dumps(dict(
        base_from_imu=base_from_imu.ravel().tolist(),
        base_from_lidar_translation=setup["base_from_lidar_translation_m"],
        chassis_aabb=[*setup["chassis_exclusion_base_aabb_m"][0], *setup["chassis_exclusion_base_aabb_m"][1]],
        footprint_half_length=nav2.half_length, footprint_half_width=nav2.half_width,
        footprint_padding=nav2.padding, mosaic_half_extent_m=30.0), indent=2))


# ---------------------------------------------------------------- synthetic drive
ROCKS = np.array([[1.6, 0.9, 0.25], [2.4, -1.1, 0.35], [3.5, 0.4, 0.18], [0.8, -1.6, 0.3], [4.6, -0.6, 0.22],
                  [-0.9, 1.2, 0.28], [2.9, 1.9, 0.4], [5.4, 1.1, 0.3]])


def ground(x, y):
    return 0.03 * np.sin(0.4 * x) + 0.02 * np.cos(0.3 * y)


def hits(origin, dirs):
    """First intersection of rays with the undulating ground or hemispherical rocks."""
    # Ground: fixed-point iteration on z = ground(x, y) (gentle slopes converge).
    down = dirs[:, 2] < -1e-3
    t = np.full(len(dirs), np.inf)
    tg = np.where(down, (origin[2] - 0.0) / -np.where(down, dirs[:, 2], -1), np.inf)
    for _ in range(6):
        p = origin + dirs * tg[:, None]
        tg = np.where(down, (origin[2] - ground(p[:, 0], p[:, 1])) / -np.where(down, dirs[:, 2], -1), np.inf)
    t = np.minimum(t, tg)
    for cx, cy, r in ROCKS:
        centre = np.array([cx, cy, ground(cx, cy)])
        oc = origin - centre
        b = dirs @ oc
        c = oc @ oc - r * r
        disc = b * b - c
        ok = disc >= 0
        ts = np.where(ok, -b - np.sqrt(np.where(ok, disc, 0)), np.inf)
        z = origin[2] + dirs[:, 2] * ts
        ts = np.where((ts > 0) & (z >= centre[2]), ts, np.inf)
        t = np.minimum(t, ts)
    return t


def rotz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    out = np.eye(4)
    out[:2, :2] = [[c, -s], [s, c]]
    return out


def base_pose(t):
    yaw = 0.15 * np.sin(0.3 * t)
    pose = rotz(yaw)
    x, y = 0.2513 * t + 0.0137, 0.4 * np.sin(0.2 * t) + 0.0071
    pose[:3, 3] = [x, y, ground(x, y)]  # off the 0.1 m lattice: no exact floor() ties
    return pose


def generate(run, duration=10.0, column_stride=4):
    run = Path(run)
    run.mkdir(parents=True, exist_ok=False)
    base_from_imu, setup = geometry()
    base_from_lidar = np.eye(4)
    base_from_lidar[:3, 3] = setup["base_from_lidar_translation_m"]
    imu_from_lidar = np.linalg.inv(base_from_imu) @ base_from_lidar
    elevations = np.deg2rad(np.concatenate([np.linspace(a, b, n) for a, b, n in setup["elevation_bands_deg"]]))
    columns = setup["horizontal_columns"]
    azimuths = np.arange(0, columns, column_stride) * 2 * np.pi / columns
    rng = np.random.default_rng(3)
    events, offset = [], 0
    raw = (run / "native.bin").open("wb")
    tick = 1 / 60
    frames, revision, frame_id = [], 0, 0
    pending = []  # (publish_time, record)

    def emit(kind, payload):
        events.append(dict(kind=kind, payload=payload, receipt_wall_ns=len(events)))

    def record(kind, rows):
        nonlocal revision
        revision += 1
        return dict(session="synthetic", epoch=1, revision=revision, parent=revision - 1, snapshot=False,
                    frames=rows, kind=kind)

    def row(fid, stamp, pose):
        return dict(frame_id=fid, stamp_ns=stamp, map_from_imu=pose.tolist(), imu_from_lidar=imu_from_lidar.tolist(),
                    deskew_recomputed=False)

    emit("reset", dict(epoch=1, minimum_ns=0))
    for k in range(int(duration * 60)):
        now = 1.0 + k * tick
        stamp = int(now * 1e9)
        # SLAM: one frame per 0.1 s, published 50 ms later; the previous frame is re-estimated
        # by 0.1 mm (smoother update); a no-op SUBMAP bookkeeping revision every second.
        if k % 6 == 0:
            pose = base_pose(now) @ base_from_imu
            current = row(frame_id, stamp, pose)
            rows = [current]
            if frames:
                fid, fstamp, fpose = frames[-1]
                if fid != 0:  # the oldest frame is the fixed-map reference: never re-estimated
                    fpose = fpose.copy()
                    fpose[0, 3] += 1e-4
                    frames[-1] = (fid, fstamp, fpose)
                    rows.insert(0, row(fid, fstamp, fpose))
            frames.append((frame_id, stamp, pose))
            frame_id += 1
            pending.append((now + 0.05, ("odom", rows)))
            if k % 60 == 30 and len(frames) > 3:
                pending.append((now + 0.05, ("submap", [row(f, s, p) for f, s, p in frames[-4:-1]])))
        for due, item in list(pending):
            if due <= now + 1e-9:
                pending.remove((due, item))
                kind, rows = item
                emit("revision", record(1 if kind == "odom" else 2, rows))
        # LiDAR: one sixth of a revolution per physics tick, sensor-frame returns.
        sector = azimuths[(azimuths >= (k % 6) * np.pi / 3) & (azimuths < ((k % 6) + 1) * np.pi / 3)]
        el, az = np.meshgrid(elevations, sector, indexing="ij")
        local = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1).reshape(-1, 3)
        map_from_lidar = base_pose(now) @ base_from_lidar
        dirs = local @ map_from_lidar[:3, :3].T
        t = hits(map_from_lidar[:3, 3], dirs)
        keep = np.isfinite(t) & (t > 0.5) & (t < 100)
        ranges = t[keep] + rng.normal(0, 0.004, keep.sum())
        points = (local[keep] * ranges[:, None]).astype(np.float32)
        data = np.zeros(len(points), dtype=DTYPE)
        for i, name in enumerate("xyz"):
            data[name] = points[:, i]
        data["depth"] = ranges
        intensity = rng.integers(1, 256, len(points)).astype(np.uint8)
        intensity[rng.random(len(points)) < 0.01] = 0
        data["intensity"], data["sequence"] = intensity, k
        data["capture_uuid"] = np.frombuffer(bytes.fromhex(CAPTURE[9:]), dtype=np.uint8)
        payload = data.tobytes()
        raw.write(payload)
        emit("chunk", dict(offset=offset, size=len(payload), stamp_ns=stamp, sequence=k, capture=CAPTURE,
                           sha256=hashlib.sha256(payload).hexdigest()))
        offset += len(payload)
        emit("clock", stamp)
    raw.close()
    with (run / "events.jsonl").open("w") as stream:
        for event in events:
            stream.write(json.dumps(event) + "\n")
    print(f"{run}: {len(events)} events, native {offset / 1e6:.1f} MB")


# ---------------------------------------------------------------- Python reference (nav semantics)
def reference(run, out, library):
    from local_terrain.core import LocalConfig, LocalReconstructor
    from local_terrain.rolling_mapper import RollingTerrainMapper
    from local_terrain.self_filter import chassis_returns
    from terrain_shadow_runtime import ReceiptScheduler

    run, out = Path(run), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    base_from_imu, setup = geometry()
    config = LocalConfig(window_s=5, evidence_lifetime_s=60, observation_policy="frozen_observation",
                         backend="prepared", map_policy="rolling_incremental", allow_bookkeeping=True,
                         max_bookkeeping_frames=32768, tolerate_stream_gaps=True)
    policy = PUBLISHED_POLICIES["nav2_bounded_shadow"]
    footprint = Footprint((60, 60), (0.0, 0.0), 0.1, Nav2Config.from_yaml())

    def make():
        core = LocalReconstructor(lambda origin, lifetime: RollingTerrainMapper(
            library, width=60, height=60, origin=origin, lifetime=lifetime), base_from_imu.tolist(), config)
        return core, ReceiptScheduler(core)

    core, scheduler = make()
    state = dict(epoch=None, minimum=None, armed=None, capture=None, mosaic=None, explore=None)
    windows = (out / "windows.bin").open("wb")
    index = (out / "windows.jsonl").open("w")
    published = 0

    def guarded(function):
        if core.failure:
            return None
        try:
            return function()
        except Exception as exc:
            core.fault(exc)
            scheduler.pending.clear()
            scheduler.bytes = 0
            return None

    raw = (run / "native.bin").open("rb")
    for line in (run / "events.jsonl").open():
        event = json.loads(line)
        kind, payload = event["kind"], event["payload"]
        if kind == "reset":
            core.close()
            core, scheduler = make()
            state.update(epoch=payload["epoch"], minimum=payload["minimum_ns"], armed=None, mosaic=None, explore=None)
        elif kind == "revision":
            def action(record=payload):
                if state["epoch"] is None or record["epoch"] != state["epoch"]:
                    return
                if state["armed"] is None:
                    if record["kind"] != 1 or not record["frames"]:
                        return
                    state["armed"] = max(state["minimum"], min(f["stamp_ns"] for f in record["frames"]))
                scheduler.receive("revision", record, len(json.dumps(record)))
            guarded(action)
        elif kind == "chunk":
            raw.seek(payload["offset"])
            rows = np.frombuffer(raw.read(payload["size"]), dtype=DTYPE)

            def action(payload=payload, rows=rows):
                if state["capture"] is None:
                    state["capture"] = payload["capture"]
                if payload["capture"] != state["capture"]:
                    raise ValueError("foreign native capture")
                if state["armed"] is None or payload["stamp_ns"] < state["armed"]:
                    return
                if core.capture is None and payload["sequence"] > 0:
                    core.resume_after(payload["capture"], payload["sequence"] - 1)
                points = np.column_stack([rows[n] for n in "xyz"]).copy()
                rejected = chassis_returns(points, setup) | (rows["intensity"] == 0)
                points[rejected] = 0
                chunk = dict(capture=payload["capture"], sequence=payload["sequence"], stamp_ns=payload["stamp_ns"],
                             points=points)
                scheduler.receive("chunk", chunk, payload["size"])
            guarded(action)
        elif kind == "clock":
            layers = guarded(lambda stamp=payload: scheduler.clock(stamp))
            if layers is None:
                continue
            record = core.last_record
            classes = layers[:, :, 0].astype(np.int8)
            grid = map_from_grid(record)
            planner, _ = planner_costs(layers, record, policy, footprint)
            origin = grid[:2, 3]
            if state["mosaic"] is None:
                centre = origin + 0.5 * np.array(planner.shape[::-1]) * 0.1
                lo, hi = centre - 30.0, centre + 30.0
                state["mosaic"], state["explore"] = GlobalStitch(lo, hi, origin, 0.1), GlobalStitch(lo, hi, origin, 0.1)
            state["mosaic"].update(planner, origin, classes != -1)
            body = clear_footprint(np.full(classes.shape, -1, np.int8), footprint, grid_pose(record)) == 0
            explore = np.where(classes == 20, 0, classes).astype(np.int8)
            explore[body & (classes == -1)] = 0
            state["explore"].update(explore, origin, classes != -1)
            window = state["mosaic"].window(origin, planner.shape)
            windows.write(np.ascontiguousarray(window, dtype=np.int8).tobytes())
            index.write(json.dumps(dict(stamp_ns=core.watermark_ns, origin=origin.tolist(),
                                        compute_ms=record["compute_ms"])) + "\n")
            published += 1
    windows.close()
    index.close()
    summary = dict(published=published, failure=core.failure, received=core.received, projected=core.projected,
                   skipped_sequences=core.skipped_sequences, late_chunks=core.late_chunks)
    for name in ("mosaic", "explore"):
        mosaic = state[name]
        if mosaic is None:
            continue
        label = "cost" if name == "mosaic" else "explore"
        (out / f"{label}_mosaic.bin").write_bytes(mosaic.grid.tobytes())
        summary[f"{label}_mosaic"] = dict(origin=mosaic.origin.tolist(), width=mosaic.grid.shape[1],
                                          height=mosaic.grid.shape[0])
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))


# ---------------------------------------------------------------- comparison
def compare(a, b):
    a, b = Path(a), Path(b)
    sa, sb = (json.loads((d / "summary.json").read_text()) for d in (a, b))
    wa = np.frombuffer((a / "windows.bin").read_bytes(), np.int8).reshape(-1, 60, 60)
    wb = np.frombuffer((b / "windows.bin").read_bytes(), np.int8).reshape(-1, 60, 60)
    ia = [json.loads(line) for line in (a / "windows.jsonl").open()]
    ib = [json.loads(line) for line in (b / "windows.jsonl").open()]
    report = dict(published=(len(wa), len(wb)), failure=(sa.get("failure"), sb.get("failure") or None))
    n = min(len(wa), len(wb))
    report["stamps_equal"] = [x["stamp_ns"] for x in ia[:n]] == [x["stamp_ns"] for x in ib[:n]]
    report["origin_max_diff_m"] = float(max((np.abs(np.subtract(x["origin"], y["origin"])).max()
                                             for x, y in zip(ia[:n], ib[:n])), default=0.0))
    diff = (wa[:n] != wb[:n])
    report["window_cells_differing"] = int(diff.sum())
    report["windows_differing"] = int(diff.any(axis=(1, 2)).sum())
    report["window_cells_total"] = int(diff.size)
    for name in ("cost", "explore"):
        if f"{name}_mosaic" not in sa or f"{name}_mosaic" not in sb:
            report[f"{name}_mosaic"] = "missing"
            continue
        ma = np.frombuffer((a / f"{name}_mosaic.bin").read_bytes(), np.int8)
        mb = np.frombuffer((b / f"{name}_mosaic.bin").read_bytes(), np.int8)
        report[f"{name}_mosaic_cells_differing"] = int((ma != mb).sum()) if ma.shape == mb.shape else "shape"
        report[f"{name}_mosaic_origin_diff_m"] = float(np.abs(np.subtract(sa[f"{name}_mosaic"]["origin"],
                                                                          sb[f"{name}_mosaic"]["origin"])).max())
    ca = [x["compute_ms"] for x in ia]
    cb = [x["compute_ms"] for x in ib]
    if ca and cb:
        report["compute_ms_p95"] = (float(np.percentile(ca, 95)), float(np.percentile(cb, 95)))
    classes = np.unique(wa, return_counts=True)
    report["window_values_a"] = {int(k): int(v) for k, v in zip(*classes)}
    print(json.dumps(report, indent=2))
    exact = (report["published"][0] == report["published"][1] and report["stamps_equal"]
             and report["window_cells_differing"] == 0
             and all(report.get(f"{m}_mosaic_cells_differing") == 0 for m in ("cost", "explore")))
    return 0 if exact else 1


if __name__ == "__main__":
    command, *args = sys.argv[1:]
    if command == "generate":
        generate(*args)
    elif command == "reference":
        reference(*args)
    elif command == "robot":
        robot(*args)
    elif command == "compare":
        sys.exit(compare(*args))
    else:
        raise SystemExit(__doc__)
