#!/usr/bin/env python3
"""SLAM+terrain robustness audit (C1 gaps, C2 pose error, C4 large obstacles).

Replays a completed live_local_v2 terrain source with a chosen pose profile and
scores every projection against independent simulator geometry. Evaluator only:
GT never enters reconstruction except as the explicit `gt*` pose profiles.

Pose profiles (applied to every GLIM revision frame before reconstruction):
  slam                 recorded GLIM poses (as flown)
  gt                   GT base poses (map frame = world - home_xyz)
  gt+h<cm>[,z<cm>][,y<deg>][,r<deg>][,tau<s>][,seed<n>]   GT plus a smooth (OU) error
                       process with the given p95 horizontal / vertical / yaw /
                       roll-and-pitch (body frame) magnitudes
  slam+...             the same error added on top of the recorded SLAM poses
  glim:<run>[+...]     poses of an audit_rtk_robustness.py replay (poses.npz) of this capture's
                       own recorder, e.g. under an RTK outage: real GLIM error, not synthetic

Rover-relative scoring: GT is projected into each grid through
local_from_base(estimated) @ inv(world_from_base(GT)) at the evidence time, i.e.
"is the hazard where the rover believes it is relative to itself". Not an
acceptance gate; layers are not bit-certified on the host (see
replay_live_local_terrain.py for exact certification).

Action availability is scored twice: the original straight-corridor proxy
(`POLICIES`) and the Nav2 contract (`NAV2_POLICIES`): the costmap
`navigation_costs` would publish, through Nav2 static/inflation semantics, swept
by the padded footprint (local_terrain/nav2_contract.py).

Suites, scorecards and the replay-once/score-many layer cache:
terrain_audit_suite.py.
"""

import argparse
import hashlib
import inspect
import json
import re
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import shapely
from scipy.ndimage import distance_transform_edt
from shapely import MultiPoint, Point, affinity, box, prepare

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.core import LocalConfig, LocalReconstructor
from local_terrain.nav2_contract import Footprint, Nav2Config, accept, grid_pose, master_costs, planner_costs, sweeps
from local_terrain.navigation_cost import PUBLISHED_POLICIES, GapPolicy
from local_terrain.rolling_mapper import RollingTerrainMapper
from local_terrain.self_filter import chassis_returns
from replay_local_terrain import decoded_chunk, relocate
from selected_lidar_profile import profile_path, selected_setup
from terrain_shadow_runtime import ReceiptScheduler

HAZARD_HEIGHTS = (0.10, 0.15, 0.20, 0.30)  # m above local ground
LARGE = 0.20
NEAR_M = 1.5  # near-field radius for gap / action metrics
SELF_M = 0.6  # rover half-diagonal; cells inside are under the rover
DANGER_M = 1.0  # hazard this close (footprint edge to rover centre) must be marked
CORRIDOR = dict(start=0.5, end=1.5, half_width=0.4, headings=16)
PHANTOM_CLEAR_M = 0.25
WARMUP_S = 5.0
POLICIES = ("strict", "bounded", "bounded_wide", "optimistic")
NAV2_POLICIES = {
    "nav2_strict": None,  # navigation_costs(enabled=False): unknown stays NO_INFORMATION
    "nav2_isolated": GapPolicy(),  # shipped default
    "nav2_bounded": GapPolicy(mode="bounded", hazard_clearance_m=0.30),
    # unknown flank of an obstacle published lethal (MPPI tolerates unknown under the footprint)
    "nav2_bounded_shadow": PUBLISHED_POLICIES["nav2_bounded_shadow"],
}
NAV2_LENGTH_M = 0.9  # translation after the in-place turn; front edge reaches ~1.5 m
UNSAFE_HEIGHT = 0.15
P95 = 2.4477  # p95 of a 2-D isotropic Gaussian norm in units of per-axis sigma


# ----------------------------------------------------------------------------- pose profiles
def parse_profile(text):
    base, _, spec = text.partition("+")
    if base not in ("slam", "gt") and not re.fullmatch(r"glim:revision-[0-9a-f]{4,32}", base):
        raise ValueError("profile must start with slam, gt or glim:<revision>")
    noise = dict(h=0.0, z=0.0, y=0.0, r=0.0, tau=5.0, seed=0)
    for item in filter(None, spec.split(",")):
        m = re.fullmatch(r"(h|z|y|r|tau|seed)([0-9.]+)", item)
        if not m:
            raise ValueError("bad noise term " + item)
        noise[m[1]] = float(m[2])
    return base, noise


class OUError:
    """Seeded Ornstein-Uhlenbeck error on a 10 ms lattice; deterministic in stamp."""

    def __init__(self, h_p95_cm, z_p95_cm, yaw_p95_deg, tilt_p95_deg, tau, seed, t0, t1):
        rng = np.random.default_rng(int(seed))
        self.dt, self.t0 = 0.01, t0 - 1.0
        n = int((t1 + 1.0 - self.t0) / self.dt) + 2
        sig = np.array([h_p95_cm / 100 / P95] * 2 + [z_p95_cm / 100 / 1.96, np.deg2rad(yaw_p95_deg) / 1.96]
                       + [np.deg2rad(tilt_p95_deg) / 1.96] * 2)
        a = np.exp(-self.dt / tau)
        x = np.zeros((n, 6))
        x[0] = rng.normal(0, sig)
        drive = rng.normal(0, 1, (n, 6)) * sig * np.sqrt(1 - a * a)
        for i in range(1, n):
            x[i] = a * x[i - 1] + drive[i]
        self.x = x

    def __call__(self, stamp_s):
        f = np.clip((stamp_s - self.t0) / self.dt, 0, len(self.x) - 1.001)
        i = int(f)
        return self.x[i] + (f - i) * (self.x[i + 1] - self.x[i])


def perturb(T, e):
    """Estimated = true with translation offset, world yaw and body roll/pitch about the body origin."""
    out = T.copy()
    tilt = Rotation.from_euler("xy", e[4:6]).as_matrix()
    out[:3, :3] = Rotation.from_euler("z", e[3]).as_matrix() @ T[:3, :3] @ tilt
    out[:3, 3] += [e[0], e[1], e[2]]
    return out


class GTTrajectory:
    def __init__(self, times, poses):
        self.t = np.asarray(times)
        self.p = np.asarray(poses)
        self.slerp = Slerp(self.t, Rotation.from_matrix(self.p[:, :3, :3]))

    def __call__(self, stamp_s):
        s = float(np.clip(stamp_s, self.t[0], self.t[-1]))
        out = np.eye(4)
        out[:3, :3] = self.slerp([s]).as_matrix()[0]
        for k in range(3):
            out[k, 3] = np.interp(s, self.t, self.p[:, k, 3])
        return out


# ----------------------------------------------------------------------------- GT scene
class Scene:
    """Independent geometry in the map frame (world - home_xyz)."""

    def __init__(self, capture, home_xyz):
        shift = -np.asarray(home_xyz, dtype=float)
        scene = json.loads((capture / "natural_scene.json").read_text())
        with np.load(capture / "clearance_tile.npz", allow_pickle=False) as data:
            tile_xy, tile_z = data["xy"] + shift[:2], data["z"] + shift[2]
            corridor = data["mask"] if "mask" in data else None
        if corridor is None:
            self.survey = MultiPoint(tile_xy).convex_hull
            prepare(self.survey)
            self.tree = None
        else:  # route-corridor survey (waypoint loops): only the surveyed cells, not their hull
            from scipy.spatial import cKDTree

            tile_xy, tile_z = tile_xy[corridor], tile_z[corridor]
            self.tree = cKDTree(tile_xy)
        objects = []
        for r in scene["objects"]:
            lo, hi = np.asarray(r["minimum"]) + shift, np.asarray(r["maximum"]) + shift
            poly = box(lo[0], lo[1], hi[0], hi[1])
            centre = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2])
            ring = np.linalg.norm(tile_xy - centre, axis=1)
            near = (ring > 0.5 * max(hi[0] - lo[0], hi[1] - lo[1])) & (ring < 1.0)
            ground = float(np.median(tile_z[near])) if near.sum() >= 5 else float(tile_z[np.argmin(ring)])
            # USD bounds are a conservative box; the inscribed ellipse is the likely rock footprint.
            core = affinity.scale(Point(*centre).buffer(1.0, 32), (hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2)
            objects.append(dict(name=r["path"].rsplit("/", 2)[-2] + "/" + r["path"].rsplit("/", 1)[-1],
                                poly=poly, core=eroded(core), height=float(hi[2] - ground),
                                in_survey=bool(near.sum() >= 5)))
        self.tile_xy = tile_xy
        demo = capture / "demo_rocks.json"
        if demo.exists():
            for r in json.loads(demo.read_text())["objects"]:
                pts = np.asarray(r["points"])[:, :2] + shift[:2]
                hull = MultiPoint(pts).convex_hull
                objects.append(dict(name=r["path"], poly=hull, core=hull,
                                    height=float(r["height_above_local_ground_m"]), in_survey=True))
        for o in objects:
            prepare(o["poly"])
            prepare(o["core"])
        self.objects = objects

    def in_survey(self, xy):
        """Inside the GT clearance survey (rock-free, validated floor)."""
        if self.tree is None:
            return shapely.contains_xy(self.survey, xy[:, 0], xy[:, 1])
        return self.tree.query(xy, distance_upper_bound=0.05 * 0.75)[0] < np.inf


def eroded(core, margin=0.05):
    """Hazard label for an approximate (ellipse) footprint: half a cell inside its edge.

    A cell centre within half a cell of an approximate outline is not evidence of
    a hazard (e.g. measured ground at an ellipse tip). Demo rocks are exact hulls
    and are not eroded. A footprint narrower than the margin keeps its outline.
    """
    inner = core.buffer(-margin)
    return core if inner.is_empty else inner


# ----------------------------------------------------------------------------- replay
def replay(source, archive, profile, on_projection):
    base, noise = parse_profile(profile)
    manifest = json.loads((source / "manifest.json").read_text())
    if manifest["policy"] != "live_local_v2":
        raise ValueError("live_local_v2 source required")
    request = manifest["request"]
    capture = archive / request["capture"]
    library = relocate(manifest["library"], archive)
    config = LocalConfig(**manifest["config"])
    setup, _ = selected_setup(profile_path(request["lidar_profile"]["name"]))
    base_from_imu = np.asarray(request["base_from_imu"])
    pose_dir = find_pose_arrays(archive, request["capture"])
    with np.load(pose_dir, allow_pickle=False) as data:
        gt = GTTrajectory(data["times"], data["gt_base"])
    glim = glim_trajectory(archive, capture, base[5:]) if base.startswith("glim:") else None
    err = None
    if any(noise[k] for k in ("h", "z", "y", "r")):
        err = OUError(noise["h"], noise["z"], noise["y"], noise["r"], noise["tau"], noise["seed"],
                      gt.t[0] - 5, gt.t[-1] + 5)

    def pose_hook(frame):
        stamp = frame["stamp_ns"] / 1e9
        if base == "gt":
            body = gt(stamp)
        elif glim is not None:
            body = glim(stamp) @ np.linalg.inv(base_from_imu)
        else:
            body = np.asarray(frame["map_from_imu"]) @ np.linalg.inv(base_from_imu)
        if err is not None:
            body = perturb(body, err(stamp))
        return (body @ base_from_imu).tolist()

    core = LocalReconstructor(
        lambda origin, lifetime: RollingTerrainMapper(
            library, width=config.cells, height=config.cells, origin=origin, lifetime=lifetime
        ),
        request["base_from_imu"],
        config,
    )
    scheduler = ReceiptScheduler(core)
    identity = base == "slam" and err is None
    projections = 0
    mismatches = []
    try:
        with (source / "native.bin").open("rb") as raw, (source / "projections.jsonl").open() as recorded:
            for line in (source / "events.jsonl").open():
                event = json.loads(line)
                kind, payload = event["kind"], event["payload"]
                if kind == "chunk":
                    chunk = decoded_chunk(payload, raw)
                    points = chunk["points"].copy()
                    points[chassis_returns(points, setup) | (chunk["intensities"] == 0)] = 0
                    chunk["points"] = points
                    scheduler.receive(kind, chunk, payload["size"])
                elif kind == "revision":
                    # ODOM (1) and bookkeeping (2/3) frames carry the same canonical poses;
                    # edit both identically so the bookkeeping no-change contract still holds.
                    if payload.get("frames") and not identity:
                        payload = dict(payload, frames=[dict(f, map_from_imu=pose_hook(f)) for f in payload["frames"]])
                    scheduler.receive(kind, payload, len(line))
                elif kind == "complete":
                    scheduler.receive(kind, payload, len(line))
                elif kind == "clock":
                    if scheduler.clock(payload) is None:
                        continue
                    record = core.last_record
                    expected = json.loads(next(recorded))
                    if identity and expected["layers_sha256"] != record["layers_sha256"]:
                        # Host numpy differs from the capture container in final float bits;
                        # the deviation accumulates. Exact certification: replay_live_local_terrain.py.
                        mismatches.append(projections)
                    on_projection(core.last_result, record, gt(record["evidence_ns"] / 1e9))
                    projections += 1
                else:
                    raise ValueError("unexpected event " + kind)
    finally:
        core.close()
    return projections, dict(first=mismatches[0] if mismatches else None, count=len(mismatches)) if identity else None


def glim_trajectory(archive, capture, prefix):
    """map_from_imu of a C3 GLIM replay; refuses a replay of any other drive than this capture's."""
    matches = sorted(archive.glob(prefix + "*"))
    if len(matches) != 1:
        raise ValueError(f"{prefix}: {len(matches)} archive matches")
    report = json.loads((matches[0] / "report.json").read_text())
    recorder = Path(report["recorder"]).name
    link = capture / "shadow_link.json"
    if not link.exists() or recorder not in link.read_text():
        raise ValueError(f"{matches[0].name} replays {recorder}, not this capture's recorder")
    with np.load(matches[0] / "poses.npz", allow_pickle=False) as data:
        t, poses = data["t"], data["map_from_imu"]
    # GLIM may republish a revised frame: keep the last pose per stamp, in stamp order
    _, last = np.unique(t[::-1], return_index=True)
    keep = np.sort(len(t) - 1 - last)
    order = np.argsort(t[keep], kind="stable")
    return GTTrajectory(t[keep][order], poses[keep][order])


def find_pose_arrays(archive, capture):
    """The GT/SLAM audit run whose pose_arrays.npz covers this capture."""
    for d in sorted(archive.glob("revision-*/pose_diagnostic.json")):
        text = d.read_text()
        if capture in text and (d.parent / "pose_arrays.npz").exists():
            return d.parent / "pose_arrays.npz"
    raise ValueError("no pose_arrays.npz audit found for " + capture)


# ----------------------------------------------------------------------------- scoring
class Scorer:
    def __init__(self, scene, cells, resolution, nav2=None):
        self.scene = scene
        self.nav2 = nav2 or Nav2Config.from_yaml()
        self.footprint = Footprint((cells, cells), (0.0, 0.0), resolution, self.nav2)  # grid-relative poses
        n = cells
        ij = np.stack(np.meshgrid(np.arange(n), np.arange(n), indexing="xy"), -1).reshape(-1, 2)
        self.ij = ij  # (col, row)
        self.res = resolution
        self.rows = []
        self.t0 = None
        self.objects = {o["name"]: dict(height=o["height"], in_survey=o["in_survey"], min_dist=np.inf,
                                        first_mark_dist=None, danger_frames=0, danger_unmarked=0,
                                        near_frames=0, near_unmarked=0, max_unmarked_dist=None)
                        for o in scene.objects}

    def __call__(self, layers, record, gt_base):
        cls = layers[:, :, 0]
        t = record["evidence_ns"] / 1e9
        self.t0 = t if self.t0 is None else self.t0
        warm = t - self.t0 >= WARMUP_S
        reason = layers[:, :, 1].astype(int)
        origin = np.asarray(record["origin"])
        local_from_base = np.asarray(record["local_from_base"])
        # cell centres (local) -> GT map via rover-relative alignment
        local_xy = origin + (self.ij + 0.5) * self.res
        map_from_local = gt_base @ np.linalg.inv(local_from_base)
        homog = np.c_[local_xy, np.full(len(local_xy), local_from_base[2, 3]), np.ones(len(local_xy))]
        map_xy = (homog @ map_from_local.T)[:, :2]
        flat_cls = cls.reshape(-1)  # row-major == self.ij order (col, row)
        flat_reason = reason.reshape(-1)
        rover_local = local_from_base[:2, 3]
        rel = local_xy - rover_local
        dist = np.linalg.norm(rel, axis=1)
        heading = np.arctan2(local_from_base[1, 0], local_from_base[0, 0])

        # GT per-cell labels: max hazard height covering the cell centre, and clearance from any object
        hazard = np.zeros(len(map_xy))  # likely footprint (ellipse / demo hull)
        hazard_box = np.zeros(len(map_xy))  # conservative USD bounds
        object_gap = np.full(len(map_xy), np.inf)  # distance to nearest object bounds
        hidden = np.zeros(len(map_xy), bool)  # cells of large objects with no occupied cell anywhere on them
        clear = np.ones(len(map_xy), bool)
        rover_gt = gt_base[:2, 3]
        for o in self.scene.objects:
            b = o["poly"].bounds
            cand = (map_xy[:, 0] > b[0] - 0.4) & (map_xy[:, 0] < b[2] + 0.4) & (map_xy[:, 1] > b[1] - 0.4) & (
                map_xy[:, 1] < b[3] + 0.4)
            stat = self.objects[o["name"]]
            d_obj = o["poly"].distance(Point(*rover_gt))
            stat["min_dist"] = min(stat["min_dist"], d_obj)
            if not cand.any():
                continue
            idx = np.flatnonzero(cand)
            pts = shapely.points(map_xy[idx])
            d = shapely.distance(o["poly"], pts)
            inside = idx[d <= 0]
            core_in = idx[shapely.distance(o["core"], pts) <= 0]
            clear[idx[d < PHANTOM_CLEAR_M]] = False
            object_gap[idx] = np.minimum(object_gap[idx], d)
            hazard_box[inside] = np.maximum(hazard_box[inside], o["height"])
            hazard[core_in] = np.maximum(hazard[core_in], o["height"])
            # object marked if any occupied cell within its footprint dilated by one cell
            marked = bool((flat_cls[idx[d <= self.res]] == 100).any())
            if not marked and o["height"] >= LARGE:
                hidden[core_in] = True
            if marked and stat["first_mark_dist"] is None:
                stat["first_mark_dist"] = d_obj
            if not warm:
                continue
            if d_obj <= DANGER_M:
                stat["danger_frames"] += 1
                stat["danger_unmarked"] += int(not marked)
            if d_obj <= 2.0:
                stat["near_frames"] += 1
                stat["near_unmarked"] += int(not marked)
                if not marked:
                    stat["max_unmarked_dist"] = max(stat["max_unmarked_dist"] or 0.0, float(d_obj))
        in_survey = self.scene.in_survey(map_xy)

        occ, unk, free = flat_cls == 100, flat_cls == -1, np.isin(flat_cls, (0, 20))
        flat_ref = clear & in_survey
        near = (dist <= NEAR_M) & (dist > SELF_M)
        corridor_rel = rel @ np.array([[np.cos(-heading), np.sin(-heading)], [-np.sin(-heading), np.cos(-heading)]]).T
        row = dict(
            t=t,
            warm=bool(warm),
            flat_ref=int(flat_ref.sum()),
            flat_observed=int((flat_ref & ~unk).sum()),
            flat_free=int((flat_ref & free).sum()),
            flat_phantom=int((flat_ref & occ).sum()),
            open_floor_ref=int((flat_ref & (object_gap > 0.6)).sum()),
            open_floor_observed=int((flat_ref & (object_gap > 0.6) & ~unk).sum()),
            open_floor_phantom=int((flat_ref & (object_gap > 0.6) & occ).sum()),
            phantom_open_near=int((flat_ref & (object_gap > 0.6) & occ & (dist <= NEAR_M)).sum()),
            phantom_near=int((flat_ref & occ & (dist <= NEAR_M)).sum()),
            phantom_ahead=int((flat_ref & occ & (corridor_rel[:, 0] > 0) & (corridor_rel[:, 0] < 1.5)
                               & (np.abs(corridor_rel[:, 1]) < CORRIDOR["half_width"])).sum()),
            near_cells=int(near.sum()),
            near_unknown=int((near & unk).sum()),
            near_unknown_hazard=int((near & unk & (hazard >= LARGE)).sum()),
            near_hidden_large=int((near & hidden).sum()),
            near_free_hazard=int((near & free & (hazard >= LARGE)).sum()),
            unknown_reasons={int(k): int(v) for k, v in zip(*np.unique(flat_reason[near & unk], return_counts=True))},
        )
        occ_dist = distance_transform_edt(cls != 100) * self.res if (cls == 100).any() else np.full(cls.shape, np.inf)
        row["actions"] = self.actions(rel, heading, flat_cls, flat_reason, hazard, hazard_box, occ_dist.reshape(-1))
        pose = grid_pose(record)
        geometry = sweeps(self.footprint, pose, length=NAV2_LENGTH_M)
        row["nav2"] = {name: self.nav2_actions(layers, record, geometry, policy, unk, hazard, hazard_box)
                       for name, policy in NAV2_POLICIES.items()}
        self.rows.append(row)

    def nav2_actions(self, layers, record, geometry, policy, unk, hazard, hazard_box):
        """Footprint-swept primitives on the costmap Nav2 would see under this gap policy.

        The grid is exactly what tools/live_local_terrain.py publishes (nav2_contract.planner_costs).
        """
        grid, _ = planner_costs(layers, record, policy, self.footprint, resolution=self.res)
        prims, start = accept(master_costs(grid, self.res, self.nav2), geometry, self.nav2)
        out = dict(available=0, available_area=0, controller_only=0, gt_unsafe=0, gt_unsafe_large=0, gt_possible=0,
                   crossed_unknown=0, crossed_unknown_hazard=0, start_lethal=int(start["start_lethal"]))
        crossed = np.zeros(len(unk), bool)
        for p in prims:
            out["available_area"] += int(p["area"])
            out["controller_only"] += int(p["controller"] and not p["planner"])
            if not (p["planner"] and p["controller"]):
                continue
            swept = p["swept"]
            out["available"] += 1
            out["gt_unsafe"] += int((hazard[swept] >= UNSAFE_HEIGHT).any())
            out["gt_unsafe_large"] += int((hazard[swept] >= LARGE).any())
            out["gt_possible"] += int((hazard_box[swept] >= UNSAFE_HEIGHT).any())
            crossed |= swept & unk
        out["crossed_unknown"] = int(crossed.sum())
        out["crossed_unknown_hazard"] = int((crossed & (hazard >= UNSAFE_HEIGHT)).sum())
        return out

    def actions(self, rel, heading, cls, reason, hazard, hazard_box, occ_dist):
        """Straight-corridor action proxy under three unknown-cell policies."""
        occ = cls == 100
        unk = cls == -1
        plain_missing = unk & np.isin(reason, (0, 4, 9, 12))
        # hazard shadow: unknown next to an occupied cell (0.3 m nominal, 0.6 m wide variant)
        shadow, wide_shadow = unk & (occ_dist <= 0.3), unk & (occ_dist <= 0.6)
        out = {p: dict(available=0, gt_unsafe=0, gt_possible=0) for p in POLICIES}
        for a in np.linspace(-np.pi, np.pi, CORRIDOR["headings"], endpoint=False):
            c, s = np.cos(heading + a), np.sin(heading + a)
            along = rel[:, 0] * c + rel[:, 1] * s
            across = -rel[:, 0] * s + rel[:, 1] * c
            inside = (along >= CORRIDOR["start"]) & (along <= CORRIDOR["end"]) & (np.abs(across) <= CORRIDOR["half_width"])
            if occ[inside].any():
                continue
            gt_unsafe = bool((hazard[inside] >= 0.15).any())
            gt_possible = bool((hazard_box[inside] >= 0.15).any())
            passes = dict(
                strict=not unk[inside].any(),
                bounded=not (unk[inside] & ~(plain_missing[inside] & ~shadow[inside])).any()
                and unk[inside].mean() <= 0.5,
                bounded_wide=not (unk[inside] & ~(plain_missing[inside] & ~wide_shadow[inside])).any()
                and unk[inside].mean() <= 0.5,
                optimistic=True,
            )
            for p, ok in passes.items():
                if ok:
                    out[p]["available"] += 1
                    out[p]["gt_unsafe"] += int(gt_unsafe)
                    out[p]["gt_possible"] += int(gt_possible)
        return out

    def summary(self):
        startup = [r for r in self.rows if not r["warm"]]
        rows = [r for r in self.rows if r["warm"]]

        def tot(key):
            return int(sum(r[key] for r in rows))

        policies = {}
        for p in POLICIES:
            avail = [r["actions"][p]["available"] for r in rows]
            policies[p] = dict(
                frames_with_action=float(np.mean([a > 0 for a in avail])),
                mean_available_headings=float(np.mean(avail)),
                chosen_unsafe_corridors=int(sum(r["actions"][p]["gt_unsafe"] for r in rows)),
                chosen_possible_unsafe_corridors=int(sum(r["actions"][p]["gt_possible"] for r in rows)),
                startup_frames_with_action=float(np.mean([r["actions"][p]["available"] > 0 for r in startup]))
                if startup else None,
                available_corridors=int(sum(avail)),
            )
        for p in NAV2_POLICIES:
            avail = [r["nav2"][p]["available"] for r in rows]
            crossed = sum(r["nav2"][p]["crossed_unknown"] for r in rows)
            crossed_hazard = sum(r["nav2"][p]["crossed_unknown_hazard"] for r in rows)
            policies[p] = dict(
                frames_with_action=float(np.mean([a > 0 for a in avail])),
                mean_available_headings=float(np.mean(avail)),
                frames_with_area_action=float(np.mean([r["nav2"][p]["available_area"] > 0 for r in rows])),
                frames_start_lethal=float(np.mean([r["nav2"][p]["start_lethal"] > 0 for r in rows])),
                controller_only_primitives=int(sum(r["nav2"][p]["controller_only"] for r in rows)),
                chosen_unsafe_corridors=int(sum(r["nav2"][p]["gt_unsafe"] for r in rows)),
                chosen_unsafe_large=int(sum(r["nav2"][p]["gt_unsafe_large"] for r in rows)),
                chosen_possible_unsafe_corridors=int(sum(r["nav2"][p]["gt_possible"] for r in rows)),
                crossed_unknown_cells=int(crossed),
                crossed_unknown_hazard_cells=int(crossed_hazard),
                hidden_hazard_rate=crossed_hazard / crossed if crossed else 0.0,
                startup_frames_with_action=float(np.mean([r["nav2"][p]["available"] > 0 for r in startup]))
                if startup else None,
                available_corridors=int(sum(avail)),
            )
        objects = {}
        for name, s in self.objects.items():
            if s["min_dist"] > 3.0:
                continue
            objects[name] = dict(
                height_m=round(s["height"], 3), in_survey=s["in_survey"], min_dist_m=round(float(s["min_dist"]), 3),
                first_mark_dist_m=None if s["first_mark_dist"] is None else round(float(s["first_mark_dist"]), 3),
                danger_frames=s["danger_frames"], danger_unmarked=s["danger_unmarked"],
                within_2m_frames=s["near_frames"], within_2m_unmarked=s["near_unmarked"],
                max_unmarked_within_2m=s["max_unmarked_dist"],
            )
        recall = {}
        for h in HAZARD_HEIGHTS:
            rel = [o for o in objects.values() if o["height_m"] >= h and o["min_dist_m"] <= 2.0]
            recall[f">={h:.2f}m"] = dict(
                objects=len(rel),
                ever_marked=sum(o["first_mark_dist_m"] is not None for o in rel),
                within_2m_frames=sum(o["within_2m_frames"] for o in rel),
                within_2m_unmarked_frames=sum(o["within_2m_unmarked"] for o in rel),
                entered_danger=sum(o["danger_frames"] > 0 for o in rel),
                danger_unmarked_frames=sum(o["danger_unmarked"] for o in rel),
                # 0.0 = an object that came within 2 m and was never marked
                min_first_mark_dist_m=min((o["first_mark_dist_m"] or 0.0 for o in rel), default=None),
            )
        return dict(
            frames=len(rows),
            startup_frames=len(startup),
            c2_flat=dict(
                reference_cells=tot("flat_ref"),
                observed_fraction=tot("flat_observed") / max(1, tot("flat_ref")),
                free_of_observed=tot("flat_free") / max(1, tot("flat_observed")),
                phantom_of_observed=tot("flat_phantom") / max(1, tot("flat_observed")),
                open_floor_observed_fraction=tot("open_floor_observed") / max(1, tot("open_floor_ref")),
                open_floor_phantom_of_observed=tot("open_floor_phantom") / max(1, tot("open_floor_observed")),
                frames_with_open_floor_phantom_near=float(np.mean([r["phantom_open_near"] > 0 for r in rows])),
                frames_with_phantom_ahead=float(np.mean([r["phantom_ahead"] > 0 for r in rows])),
                frames_with_phantom_near=float(np.mean([r["phantom_near"] > 0 for r in rows])),
            ),
            c1_gaps=dict(
                near_unknown_fraction=tot("near_unknown") / max(1, tot("near_cells")),
                near_unknown_large_hazard_cells=tot("near_unknown_hazard"),
                near_cells_of_unmarked_large_objects=tot("near_hidden_large"),
                near_free_large_hazard_cells=tot("near_free_hazard"),
                policies=policies,
            ),
            c4_recall=recall,
            objects=objects,
        )


class _Stop(Exception):
    pass


def replay_key(source, profile, every, limit):
    """Cache identity: inputs plus the source of every function that shapes the frames."""
    code = "".join(inspect.getsource(f) for f in (parse_profile, OUError, perturb, GTTrajectory, replay))
    return dict(source=str(source), profile=profile, every=every, limit=limit,
                code_sha256=hashlib.sha256(code.encode()).hexdigest())


class LayerCache:
    """Scored frames of one replay: exact classes/reasons, float64 terrain channels."""

    def __init__(self):
        self.frames = []

    def add(self, layers, record, gt_base):
        self.frames.append((layers, record, gt_base))

    def save(self, path, key, extra):
        layers = np.stack([f[0] for f in self.frames]) if self.frames else np.zeros((0, 1, 1, 6))
        np.savez_compressed(
            path, labels=layers[..., :2].astype(np.int16), terrain=layers[..., 2:].astype(np.float64),
            origin=np.array([f[1]["origin"] for f in self.frames], float).reshape(-1, 2),
            local_from_base=np.array([f[1]["local_from_base"] for f in self.frames], float).reshape(-1, 4, 4),
            evidence_ns=np.array([f[1]["evidence_ns"] for f in self.frames], np.int64),
            gt_base=np.array([f[2] for f in self.frames], float).reshape(-1, 4, 4),
            meta=json.dumps(dict(key=key, **extra)))

    @staticmethod
    def load(path, key):
        with np.load(path, allow_pickle=False) as data:
            meta = json.loads(str(data["meta"]))
            if meta["key"] != key:
                return None
            layers = np.concatenate([data["labels"].astype(np.float64), data["terrain"]], axis=-1)
            frames = [(layers[i], dict(origin=data["origin"][i].tolist(),
                                       local_from_base=data["local_from_base"][i].tolist(),
                                       evidence_ns=int(data["evidence_ns"][i])), data["gt_base"][i])
                      for i in range(len(layers))]
        return frames, meta


def run(source, profile, archive, output, *, every=1, limit=None, cache_from=None, save_layers=False):
    """Replay (or reuse cached frames), score and write report.json / frames.jsonl into `output`."""
    source, archive, output = Path(source), Path(archive), Path(output)
    manifest = json.loads((source / "manifest.json").read_text())
    capture = archive / manifest["request"]["capture"]
    scene = Scene(capture, manifest["request"]["home_xyz"])
    scorer = Scorer(scene, manifest["config"]["cells"], manifest["config"]["resolution"])
    key = replay_key(source, profile, every, limit)
    began = time.time()
    cached = LayerCache.load(cache_from, key) if cache_from and Path(cache_from).exists() else None
    cache = LayerCache() if save_layers else None
    if cached is not None:
        frames, meta = cached
        for layers, record, gt_base in frames:
            scorer(layers, record, gt_base)
            if cache is not None:
                cache.add(layers, record, gt_base)
        projections, layer_mismatch = meta["projections"], meta["recorded_layer_mismatch"]
    else:
        count = [0]

        def observe(layers, record, gt_base):
            if limit is not None and count[0] >= limit:
                raise _Stop
            if count[0] % every == 0:
                # the reconstructor reuses its buffers; keep an independent copy of every scored frame
                layers = np.array(layers, dtype=np.float64)
                scorer(layers, record, gt_base)
                if cache is not None:
                    cache.add(layers, record, gt_base)
            count[0] += 1

        layer_mismatch = None
        try:
            projections, layer_mismatch = replay(source, archive, profile, observe)
        except _Stop:
            projections = count[0]
    report = dict(
        tool="audit_terrain_robustness", source=str(source), profile=profile, every=every, limit=limit,
        projections=projections, recorded_layer_mismatch=layer_mismatch, layers_cached=cached is not None,
        wall_s=round(time.time() - began, 1), acceptance=False,
        constants=dict(hazard_heights=HAZARD_HEIGHTS, large=LARGE, near_m=NEAR_M, self_m=SELF_M,
                       danger_m=DANGER_M, corridor=CORRIDOR, phantom_clear_m=PHANTOM_CLEAR_M,
                       unsafe_height=UNSAFE_HEIGHT, nav2=scorer.nav2.__dict__, nav2_length_m=NAV2_LENGTH_M,
                       nav2_policies={k: v and v.__dict__ for k, v in NAV2_POLICIES.items()}),
        **scorer.summary(),
    )
    (output / "report.json").write_text(json.dumps(report, indent=2))
    with (output / "frames.jsonl").open("x") as sink:
        for r in scorer.rows:
            sink.write(json.dumps(r) + "\n")
    if cache is not None:
        cache.save(output / "layers.npz", key, dict(projections=projections, recorded_layer_mismatch=layer_mismatch))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", type=Path, help="live_local_v2 terrain directory on the archive")
    parser.add_argument("--profile", default="slam")
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    parser.add_argument("--every", type=int, default=1, help="score every Nth projection (replay is always full)")
    parser.add_argument("--limit", type=int, default=None, help="smoke test: stop after N projections")
    args = parser.parse_args()
    source, archive = args.source.resolve(strict=True), args.archive_root.resolve(strict=True)
    output = allocate_run(archive, limit=1024**3)
    print(f"Run: {output}", flush=True)
    try:
        report = run(source, args.profile, archive, output, every=args.every, limit=args.limit)
        check_run(output)
        print(json.dumps({k: v for k, v in report.items() if k != "objects"}, indent=1))
    except BaseException as exc:
        (output / "failure.json").write_text(json.dumps(dict(error=repr(exc), profile=args.profile)))
        raise


if __name__ == "__main__":
    main()
