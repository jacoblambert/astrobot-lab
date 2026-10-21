"""Opt-in, dimensioned live test rocks. Geometry is evaluator-only, never mapper input."""

import hashlib
import json
from pathlib import Path

import numpy as np

PROFILE = "rounded_10x15_v2"
DETOUR_PROFILE = "single_40x60_detour_v1"
HEIGHT = 0.10
WIDTH = 0.15
OFFSETS = ((1.8, 0), (1.8, 1.8), (0, 1.8), (-1.8, 1.8), (-2.65, 0), (-1.8, -1.8), (0, -1.8), (1.8, -1.8))
# C4 close-approach field (2026-10-20): four rocks beside the approach_field out-and-back,
# alternating sides, 2 m apart along the leg. Edge 0.95 m from the route centreline: clear
# of the per-step guard footprint (0.70 m) plus admitted tracking (0.08 m lateral, 0.15 rad
# heading -> 0.82 m), so 0.61 m from the rover body, and inside the 1 m C4 danger range.
APPROACH_PROFILE = "approach_field_v1"
APPROACH_EDGE_M = 0.95
APPROACH_ROCKS = ((1.5, 1, 0.12, 0.18), (3.5, -1, 0.20, 0.30), (5.5, 1, 0.30, 0.45), (7.5, -1, 0.40, 0.60))


def approach_layout():
    """(along, lateral) offsets in the route-leg frame, plus (height, width), per rock."""
    return [((along, side * (APPROACH_EDGE_M + width / 2)), height, width)
            for along, side, height, width in APPROACH_ROCKS]


def validate_profile(name, pattern):
    if name == APPROACH_PROFILE or pattern == "approach_field":
        if (name, pattern) != (APPROACH_PROFILE, "approach_field"):
            raise ValueError("approach field rocks and route are declared together")
        return name
    if name == DETOUR_PROFILE:
        if pattern != "costmap_detour8":
            raise ValueError("single detour rock requires its declared navigation scenario")
        return name
    if pattern == "costmap_detour8":
        raise ValueError("detour scenario requires the declared single rock")
    if name not in ("", PROFILE):
        raise ValueError("unknown demo rock profile")
    if name and pattern not in ("turn15", "turn360", "peek_reverse20"):
        raise ValueError("dimensioned rocks only declared for startup observation routes")
    return name


def rounded_mesh(height=HEIGHT, width=WIDTH, rings=12, sectors=32):
    """Closed ellipsoid, bottom z=0, exact equatorial width and apex height."""
    if height <= 0 or width <= 0 or rings < 4 or rings % 2 or sectors < 8 or sectors % 4:
        raise ValueError("positive dimensions and even rings / quarter-aligned sectors required")
    points = [[0, 0, height]]
    for i in range(1, rings):
        a = np.pi * i / rings
        for j in range(sectors):
            b = 2 * np.pi * j / sectors
            points.append(
                [width / 2 * np.sin(a) * np.cos(b), width / 2 * np.sin(a) * np.sin(b), height / 2 * (1 + np.cos(a))]
            )
    bottom = len(points)
    points.append([0, 0, 0])
    triangles = []
    for j in range(sectors):
        k = (j + 1) % sectors
        triangles.append([0, 1 + j, 1 + k])
        for i in range(rings - 2):
            a, b = 1 + i * sectors, 1 + (i + 1) * sectors
            triangles.extend([[a + j, b + j, b + k], [a + j, b + k, a + k]])
        a = 1 + (rings - 2) * sectors
        triangles.append([a + j, bottom, a + k])
    return np.asarray(points), np.asarray(triangles, dtype=int)


def create(stage, cast, world_from_base, run, *, profile=PROFILE):
    """Before acquisition: drape mesh base height onto measured local terrain."""
    from pxr import Gf, UsdGeom, UsdPhysics
    from natural_motion import ground_sample

    pose = np.asarray(world_from_base)
    if profile not in (PROFILE, DETOUR_PROFILE, APPROACH_PROFILE):
        raise ValueError("unknown rock geometry")
    angle = np.arctan2(pose[1, 0], pose[0, 0])
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    if profile == APPROACH_PROFILE:
        from natural_motion import route

        leg = route("approach_field")["targets"][1]  # first translate: the out leg
        h = leg[2]
        along = np.array([[np.cos(h), -np.sin(h)], [np.sin(h), np.cos(h)]])
        rocks = [(along @ offset, height, width) for offset, height, width in approach_layout()]
    else:
        height, width = (0.4, 0.6) if profile == DETOUR_PROFILE else (HEIGHT, WIDTH)
        rocks = [(offset, height, width) for offset in (((1.8, 0),) if profile == DETOUR_PROFILE else OFFSETS)]
    prefix = {PROFILE: "DemoRocks10x15", DETOUR_PROFILE: "DemoDetourRock", APPROACH_PROFILE: "DemoApproachRocks"}[profile]
    objects = []
    for index, (offset, height, width) in enumerate(rocks):
        local, triangles = rounded_mesh(height, width)
        xy = pose[:2, 3] + offset
        if profile != PROFILE:
            xy = pose[:2, 3] + rotation @ offset
        ground = []
        for point in local:
            location = xy + point[:2]
            hits = cast((float(location[0]), float(location[1]), float(pose[2, 3] + 2)), (0.0, 0.0, -1.0), 4.0)
            try:
                ground.append(float(ground_sample(hits)))
            except ValueError as exc:
                (Path(run) / "demo_rocks_failure.json").write_text(
                    json.dumps(
                        dict(
                            profile=profile,
                            object=index,
                            xy=location.tolist(),
                            hits=hits,
                            error=str(exc),
                            motion_commands_sent=False,
                        ),
                        indent=2,
                    )
                )
                raise
        points = local + np.r_[xy, 0]
        points[:, 2] += ground
        path = f"/{prefix}/rock_{index}"
        if stage.GetPrimAtPath(path).IsValid():
            raise ValueError("demo rock already exists; cannot replace a scene")
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in points])
        mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
        mesh.CreateFaceVertexIndicesAttr(triangles.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.32, 0.29)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
        objects.append(
            dict(
                path=path,
                center_xy=xy.tolist(),
                points=points.tolist(),
                underlying_ground_z=ground,
                triangles=triangles.tolist(),
                height_above_local_ground_m=height,
                horizontal_diameter_m=width,
            )
        )
    data = dict(
        profile=profile,
        objects=objects,
        initial_world_from_base=pose.tolist(),
        natural_rocks_unchanged=True,
        terrain_labels_in_estimator=False,
        surface="closed rounded triangle mesh; vertex elevations follow underlying terrain",
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    )
    (Path(run) / "demo_rocks_created.json").write_text(json.dumps(data, indent=2))
    return data


def verify(data, cast, run):
    """Independent PhysX apex/width witness checks after collider cooking."""
    checks = []
    for obj in data["objects"]:
        points = np.asarray(obj["points"])
        # Upper shoulder witnesses, not tangent silhouette rays. The exact
        # horizontal extent is independently recorded in the authored vertices.
        probes = [0, *[1 + 3 * 32 + j for j in (0, 8, 16, 24)]]
        for index in probes:
            p = points[index]
            hits = [
                h
                for h in cast((float(p[0]), float(p[1]), float(p[2] + 1)), (0.0, 0.0, -1.0), 2.0)
                if not h[3].startswith("/Robots/")
            ]
            first = hits[0] if hits else None
            passed = first is not None and first[3] == obj["path"] and abs(first[1][2] - p[2]) <= 0.002
            checks.append(
                dict(path=obj["path"], vertex=index, expected_z=float(p[2]), actual=first, passed=bool(passed))
            )
    report = dict(
        data, verified=all(r["passed"] for r in checks), checks=checks, acceptance=False, geometry_tolerance_m=0.002
    )
    path = Path(run) / "demo_rocks.json"
    path.write_text(json.dumps(report, indent=2))
    if not report["verified"]:
        raise ValueError("dimensioned demo rock PhysX verification failed")
    return dict(
        profile=data["profile"],
        geometry_file=path.name,
        geometry_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        objects=len(data["objects"]),
    )
