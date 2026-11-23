#!/usr/bin/env python3
# Copyright 2026 Jacob Lambert
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""GT body contacts with rock cores along a recorded GT trajectory (gate runs, post hoc).

Reads a TUM trajectory in the simulator world frame (mission_eval --record-dir writes
gt_base.tum) and the sim run's evaluator geometry (natural_scene.json and the terrain-only
clearance_tile.npz). Scores the unpadded body at every sample. No ROS; numpy only.

Rocks the body already overlaps at the first sample (the eval starts before the mission, so that
is the spawn pose: seed 7 spawns over a 0.12 m rock) are world content the rover cannot avoid.
Their contacts are reported as spawn_rock_contacts and left out of contacts_by_height.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

HAZARD_HEIGHTS = (0.10, 0.15, 0.20, 0.30)  # m above local ground
LARGE = 0.20
BODY = (0.495, 0.335)  # half length / width, no padding: physical contact
CORE_MARGIN = 0.05  # an approximate (ellipse) footprint is eroded by half a cell


def ellipse(cx, cy, ax, ay, segments=128):
    """Convex polygon (counter-clockwise) approximating the ellipse with semi-axes ax, ay."""
    t = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    return np.column_stack([cx + ax * np.cos(t), cy + ay * np.sin(t)])


def erode(polygon, margin):
    """Inward offset of a convex counter-clockwise polygon; the polygon itself if it vanishes."""
    p = np.asarray(polygon, float)
    edges = np.roll(p, -1, axis=0) - p
    normals = np.column_stack([edges[:, 1], -edges[:, 0]]) / np.linalg.norm(edges, axis=1)[:, None]  # outward
    offsets = np.einsum("ij,ij->i", normals, p) - margin
    out = []
    for i in range(len(p)):
        j = (i + 1) % len(p)
        a = np.array([normals[i], normals[j]])
        if abs(np.linalg.det(a)) < 1e-12:
            continue
        out.append(np.linalg.solve(a, [offsets[i], offsets[j]]))
    out = np.asarray(out)
    # A footprint narrower than twice the margin vanishes: keep its outline (shapely semantics).
    # Valid erosion: every offset vertex lies inside every shifted edge.
    if len(out) < 3 or np.any(out @ normals.T - offsets > 1e-9):
        return p
    area = 0.5 * np.sum(out[:, 0] * np.roll(out[:, 1], -1) - np.roll(out[:, 0], -1) * out[:, 1])
    return out if area > 1e-9 else p


def body(x, y, yaw, half=BODY):
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(half[0], half[1]), (-half[0], half[1]), (-half[0], -half[1]), (half[0], -half[1])]
    return np.array([(x + c * u - s * v, y + s * u + c * v) for u, v in corners])


def intersects(a, b):
    """Separating-axis test for two convex polygons (touching counts as intersecting)."""
    for poly in (a, b):
        edges = np.roll(poly, -1, axis=0) - poly
        for nx, ny in np.column_stack([-edges[:, 1], edges[:, 0]]):
            pa, pb = a @ (nx, ny), b @ (nx, ny)
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


class Scene:
    """Rock cores and heights in a frame shifted by -home_xyz (world frame for home 0)."""

    def __init__(self, run, home_xyz=(0.0, 0.0, 0.0)):
        run = Path(run)
        shift = -np.asarray(home_xyz, dtype=float)
        scene = json.loads((run / "natural_scene.json").read_text())
        with np.load(run / "clearance_tile.npz", allow_pickle=False) as data:
            tile_xy, tile_z = data["xy"] + shift[:2], data["z"] + shift[2]
            if "mask" in data:
                tile_xy, tile_z = tile_xy[data["mask"]], tile_z[data["mask"]]
        self.tile_xy = tile_xy
        self.objects = []
        for r in scene["objects"]:
            lo, hi = np.asarray(r["minimum"]) + shift, np.asarray(r["maximum"]) + shift
            centre = (lo[:2] + hi[:2]) / 2
            ring = np.linalg.norm(tile_xy - centre, axis=1)
            near = (ring > 0.5 * max(hi[0] - lo[0], hi[1] - lo[1])) & (ring < 1.0)
            ground = float(np.median(tile_z[near])) if near.sum() >= 5 else float(tile_z[np.argmin(ring)])
            # USD bounds are a conservative box; the inscribed ellipse is the likely rock footprint.
            core = erode(ellipse(centre[0], centre[1], (hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2), CORE_MARGIN)
            self.objects.append(dict(name=r["path"].rsplit("/", 2)[-2] + "/" + r["path"].rsplit("/", 1)[-1],
                                     core=core, height=float(hi[2] - ground)))


def contacts(scene, tum):
    """Samples whose body touches a rock core >= the lowest hazard height: (t, x, y, height, name)."""
    rocks = [o for o in scene.objects if o["height"] >= HAZARD_HEIGHTS[0]]
    centres = np.array([o["core"].mean(axis=0) for o in rocks]) if rocks else np.zeros((0, 2))
    radii = np.array([np.linalg.norm(o["core"] - o["core"].mean(axis=0), axis=1).max() for o in rocks])
    reach = math.hypot(*BODY)
    found = []
    for t, x, y, _, qx, qy, qz, qw in tum:
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        rect = body(x, y, yaw)
        near = np.flatnonzero(np.hypot(centres[:, 0] - x, centres[:, 1] - y) <= reach + radii) if rocks else []
        hits = [i for i in near if intersects(rect, rocks[i]["core"])]
        if hits:
            tallest = max(hits, key=lambda i: rocks[i]["height"])
            found.append((float(t), float(x), float(y), rocks[tallest]["height"], rocks[tallest]["name"]))
    return found


def summarize(found, samples, spawn=()):
    """found: contacts() rows; spawn: names of rocks under the body at the first sample."""
    avoidable = [c for c in found if c[4] not in spawn]
    return dict(samples=samples, contact_samples=len(avoidable),
                contacts_by_height={f"ge_{h:.2f}": sum(c[3] >= h for c in avoidable) for h in HAZARD_HEIGHTS},
                large_contact=any(c[3] >= LARGE for c in avoidable),
                rocks_touched=sorted({c[4] for c in avoidable}),
                first_contact=avoidable[0] if avoidable else None,
                spawn_rocks=sorted(spawn), spawn_rock_contacts=len(found) - len(avoidable))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tum", type=Path, help="GT base trajectory (TUM, world frame)")
    parser.add_argument("--scene", type=Path, required=True, help="sim run directory with natural_scene.json")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    scene = Scene(args.scene)
    tum = np.loadtxt(args.tum, ndmin=2)
    found = contacts(scene, tum)
    spawn = {c[4] for c in contacts(scene, tum[:1])}
    text = json.dumps(dict(scene=str(args.scene), **summarize(found, len(tum), spawn)), indent=2)
    if args.output:
        args.output.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
