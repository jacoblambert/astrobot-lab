#!/usr/bin/env python3
"""Deterministic, chassis-aware hazard visibility screen; not navigation acceptance.

Exact analytic first hits on a plane, boxes, ellipsoids and rectangular pits.
Reported placement fractions are not real-world probabilities or classifier recall.
No sensor/world configuration or map is changed. Combined views are hypothetical,
not a physically validated controller. Missing returns are cues, not free space.
"""

import argparse
import hashlib
import json
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

from archive_guard import ARCHIVE_ROOT, allocate_run, check_run
from local_terrain.mounting_study import beam_directions, first_intersections, valid_returns
from local_terrain.ray_design import selected_setup

STARTUP_RADIUS = float(np.hypot(0.5037, 0.3454))
SIZES = ((0.08, 0.10), (0.08, 0.20), (0.12, 0.20), (0.16, 0.20), (0.16, 0.30), (0.20, 0.30))
PITS = ((0.08, 0.10), (0.20, 0.10), (0.08, 0.20), (0.20, 0.20), (0.20, 0.40), (0.40, 0.60))


def box_interval(origin, directions, low, high):
    """Slab entry/exit with parallel rays and inside origins handled explicitly."""
    low, high = np.asarray(low), np.asarray(high)
    if np.any(high <= low):
        raise ValueError("positive box extents required")
    near, far = np.full(len(directions), -np.inf), np.full(len(directions), np.inf)
    possible = np.ones(len(directions), bool)
    for axis in range(3):
        moving = directions[:, axis] != 0
        a, b = np.full(len(directions), -np.inf), np.full(len(directions), np.inf)
        np.divide(low[axis] - origin[axis], directions[:, axis], out=a, where=moving)
        np.divide(high[axis] - origin[axis], directions[:, axis], out=b, where=moving)
        possible &= moving | ((low[axis] <= origin[axis]) & (origin[axis] <= high[axis]))
        near, far = np.maximum(near, np.minimum(a, b)), np.minimum(far, np.maximum(a, b))
    possible &= far >= np.maximum(near, 0)
    return np.where(possible, np.maximum(near, 0), np.inf), np.where(possible, far, np.inf)


def ellipsoid_hit(origin, directions, center, radii):
    radii = np.asarray(radii)
    if np.any(radii <= 0):
        raise ValueError("positive ellipsoid radii required")
    p, d = (origin - center) / radii, directions / radii
    a, b, c = np.sum(d * d, axis=1), d @ p, float(p @ p - 1)
    disc = b * b - a * c
    root = np.sqrt(np.maximum(disc, 0))
    near, far = (-b - root) / a, (-b + root) / a
    return np.where((disc >= 0) & (far >= 0), np.maximum(near, 0), np.inf)


def scene_depth(origin, directions, baseline, labels, case):
    """Modify only the hazard's first intersection, preserving chassis occlusion."""
    x, y, width, height = (case[k] for k in ("x", "y", "width", "height"))
    low, high = np.array([x - width / 2, y - width / 2, 0.0]), np.array([x + width / 2, y + width / 2, height])
    if case["kind"] == "pit":
        # Only rays whose ordinary plane hit falls in the opening enter the pit.
        # Their first exit from its volume is the floor or an inner wall.
        plane = origin + directions * np.where(np.isfinite(baseline), baseline, 0)[:, None]
        opening = (labels == 0) & np.all((plane[:, :2] > low[:2]) & (plane[:, :2] < high[:2]), axis=1)
        _, exit_depth = box_interval(origin, directions, [*low[:2], -height], [*high[:2], 0])
        return np.where(opening, exit_depth, baseline), opening
    if case["kind"] == "box":
        hit, _ = box_interval(origin, directions, low, high)
    elif case["kind"] == "rounded":
        hit = ellipsoid_hit(origin, directions, np.array([x, y, height / 2]), [width / 2, width / 2, height / 2])
    else:
        raise ValueError("unknown hazard shape")
    hit_first = hit < baseline - 1e-9
    return np.minimum(hit, baseline), hit_first


def make_sensor():
    path = Path(__file__).parent / "local_terrain/selected_lidar_optical.json"
    setup, elevations = selected_setup(path)
    azimuth = setup["horizontal_start_rad"] + np.arange(setup["horizontal_columns"]) * np.radians(
        setup["horizontal_resolution_deg"]
    )
    directions = beam_directions(azimuth, -np.radians(elevations))
    origin = np.asarray(setup["base_from_lidar_translation_m"]) + [0, 0, 0.1449231063]
    chassis = np.asarray(setup["chassis_exclusion_base_aabb_m"]) + [0, 0, 0.1449231063]
    depth, labels = first_intersections(origin, directions, [chassis])
    return setup, origin, directions, depth, labels, np.repeat(azimuth, len(elevations))


def placements():
    """Frozen regular and shifted banks. Shifts are declared before scoring."""
    result = []
    for bank, shift in (("regular", 0), ("shifted", 1)):
        for radius in (0.65, 0.75, 0.90, 1.10):
            for bearing in range(0, 360, 30):
                angle, r = np.radians(bearing + shift * 15), radius + shift * 0.025
                x, y = r * np.cos(angle), r * np.sin(angle)
                for kind, sizes in (("box", SIZES), ("rounded", SIZES), ("pit", PITS)):
                    for height, width in sizes:
                        # Exact planar footprint distance: circle for ellipsoid,
                        # axis-aligned square for box/pit. Report prior conflicts.
                        nearest = (
                            r - width / 2
                            if kind == "rounded"
                            else np.linalg.norm(np.maximum(np.abs([x, y]) - width / 2, 0))
                        )
                        result.append(
                            dict(
                                id=len(result),
                                bank=bank,
                                kind=kind,
                                radius=r,
                                bearing=bearing + shift * 15,
                                x=float(x),
                                y=float(y),
                                height=height,
                                width=width,
                                outside_startup=bool(nearest >= STARTUP_RADIUS),
                            )
                        )
    return result


def view_counts(sensor, case, yaw_degrees, reverse=0.0, *, crop=True):
    setup, origin, directions, depth, labels, azimuth = sensor
    c, s = np.cos(np.radians(yaw_degrees)), np.sin(np.radians(yaw_degrees))
    # Hazard is world-axis-aligned. Transform rays into world coordinates;
    # rotate chassis and sensor together, never rotate just the LiDAR origin.
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    translation = np.array([-reverse * c, -reverse * s, 0])
    world_origin = rotation @ origin + translation
    delta = np.array([case["x"], case["y"]]) - world_origin[:2]
    rho, radius = np.linalg.norm(delta), case["width"] / np.sqrt(2)
    mask = np.ones(len(depth), bool)
    if crop and rho > radius:
        angle = np.arctan2(delta[1], delta[0]) - np.radians(yaw_degrees)
        wrapped = (azimuth - angle + np.pi) % (2 * np.pi) - np.pi
        mask = np.abs(wrapped) <= np.arcsin(radius / rho) + 1e-12
    rays, before, classes = directions[mask] @ rotation.T, depth[mask], labels[mask]
    after, on_hazard = scene_depth(world_origin, rays, before, classes, case)
    old_valid = valid_returns(before, setup["min_range_m"], setup["max_range_m"])
    new_valid = valid_returns(after, setup["min_range_m"], setup["max_range_m"])
    both = old_valid & new_valid
    change = np.zeros(len(after), bool)
    change[both] = np.abs(after[both] - before[both]) > 0.005
    observed = on_hazard & new_valid
    hit_z = world_origin[2] + rays[:, 2] * np.where(np.isfinite(after), after, 0)
    return dict(
        surface_returns=int(observed.sum()),
        raised_8cm_returns=int((observed & (hit_z >= 0.08 - 1e-9)).sum()) if case["kind"] != "pit" else 0,
        changed_returns=int((change | (~old_valid & new_valid)).sum()),
        missing_expected_returns=int((old_valid & ~new_valid).sum()),
    )


def summarize(rows):
    groups = defaultdict(list)
    for row in rows:
        if row["outside_startup"]:
            for protocol, metrics in row["protocols"].items():
                groups[(row["bank"], row["kind"], row["height"], row["width"], protocol)].append(metrics)
    output = []
    for (bank, kind, height, width, protocol), values in sorted(groups.items()):
        counts = {str(n): sum(v["max_changed_returns"] >= n for v in values) for n in (1, 3, 10)}
        output.append(
            dict(
                bank=bank,
                kind=kind,
                height=height,
                width=width,
                protocol=protocol,
                placements=len(values),
                visible_placements_by_min_changed_returns=counts,
                no_observable_change=sum(
                    v["max_changed_returns"] == 0 and v["max_missing_expected_returns"] == 0 for v in values
                ),
                min_best_view_changed_returns=min(v["max_changed_returns"] for v in values),
            )
        )
    return output


def render(summary):
    rows = []
    for r in summary:
        if r["bank"] == "shifted":
            n = r["placements"]
            hits = r["visible_placements_by_min_changed_returns"]
            rows.append(
                f"<tr><td>{r['kind']}</td><td>{100*r['height']:g} × {100*r['width']:g}</td>"
                f"<td>{r['protocol']}</td><td>{n}</td><td>{hits['1']}/{n}</td>"
                f"<td>{hits['3']}/{n}</td><td>{hits['10']}/{n}</td><td>{r['no_observable_change']}</td></tr>"
            )
    return (
        """<!doctype html><meta charset="utf-8"><title>Near-field hazard visibility</title>
<style>body{font:16px system-ui;max-width:1200px;margin:32px auto;padding:0 20px;color:#183047}
table{border-collapse:collapse;width:100%}th,td{padding:8px;text-align:left;border-bottom:1px solid #ddd}
th{position:sticky;top:0;background:#e5edf5}p{max-width:950px}.note{background:#fff2cb;padding:16px}</style>
<h1>Can a hazard hide in the near-field gap?</h1>
<p class="note">Analytic sensing test, not simulator or navigation qualification. Counts are fractions of a
declared placement bank, not probabilities of encountering hazards. No world settings were changed.</p>
<p>Selected 32 × 1999 rays; level front-offset mount; chassis blocks rays before range filtering.
Flat ground, box and rounded rocks, rectangular pits with real bottom/side-wall returns.
Parked; reverse 20 cm; 24 yaw views; or yaw views plus a 20 cm reverse peek at each heading.
The last protocol is hypothetical and has not been physically executed.</p>
<p>Shown: shifted bank (15° bearing and 2.5 cm radius shift), hazards entirely outside the fixed 61 cm
startup area. Regular bank, all individual placements, startup-overlap cases and exact poses are preserved
in JSON. A changed return differs from flat ground by more than 5 mm or appears where none existed.
Columns require 1, 3 or 10 changed returns in at least one view. This is an observable cue, not a
successful terrain-classifier detection. Missing expected returns are recorded separately, never called free.</p>
<p>Excludes real mesh shapes, wheels/mast, slopes, noise, pose error and physical skid.
Larger rocks may also close navigable corridors. Mesh scale is not exposed height.</p>
<table><thead><tr><th>Shape</th><th>Height/depth × width (cm)</th><th>Views</th><th>Cases</th>
<th>≥1 cue</th><th>≥3 cues</th><th>≥10 cues</th><th>No change at all</th></tr></thead><tbody>"""
        + "".join(rows)
        + "</tbody></table>"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-root", type=Path, default=ARCHIVE_ROOT)
    args = parser.parse_args()
    output = allocate_run(args.archive_root, limit=128 * 1024**2)
    print(f"Run: {output}", flush=True)
    start = time.monotonic()
    sensor, rows = make_sensor(), []
    paths = [
        Path(__file__),
        Path(__file__).parent / "local_terrain/selected_lidar_optical.json",
        Path(__file__).parent / "local_terrain/mounting_study.py",
        Path(__file__).parent / "local_terrain/ray_design.py",
        Path(__file__).parents[1] / "OmniLRS/cfg/environment/lunaryard_40m.yaml",
    ]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    with (output / "placements.jsonl").open("x") as stream:
        for case in placements():
            views = [
                dict(yaw=yaw, reverse=reverse, **view_counts(sensor, case, yaw, reverse))
                for reverse in (0, 0.2)
                for yaw in range(0, 360, 15)
            ]
            selections = dict(parked=views[:1], reverse20=[views[24]], turn360=views[:24], turn_plus_peeks=views)
            protocols = {
                name: {
                    f"max_{key}": max(v[key] for v in subset)
                    for key in ("surface_returns", "raised_8cm_returns", "changed_returns", "missing_expected_returns")
                }
                for name, subset in selections.items()
            }
            row = dict(case, protocols=protocols, views=views)
            stream.write(json.dumps(row) + "\n")
            rows.append(row)
            if len(rows) % 96 == 0:
                check_run(output)
                print(f"Scored {len(rows)} / 1728 placements", flush=True)
    summary = summarize(rows)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != hashes[str(p)] for p in paths):
        raise ValueError("inputs changed during screen")
    report = dict(
        completed=True,
        acceptance=False,
        description=__doc__,
        placements=len(rows),
        outside_startup=sum(r["outside_startup"] for r in rows),
        summary=summary,
        wall_seconds=time.monotonic() - start,
        input_sha256=hashes,
        limits=[
            "no noise, pose error, wheels/mast or terrain relief",
            "not classifier recall",
            "placement fraction is not real-world probability",
            "no motion authority",
        ],
        output_sha256={"placements.jsonl": hashlib.sha256((output / "placements.jsonl").read_bytes()).hexdigest()},
    )
    (output / "report.json").write_text(json.dumps(report, indent=2))
    (output / "report.html").write_text(render(summary))
    check_run(output)
    print(json.dumps({k: report[k] for k in ("placements", "outside_startup", "wall_seconds")}))


if __name__ == "__main__":
    main()
