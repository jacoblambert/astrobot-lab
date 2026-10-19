"""Read USD geometry only; does not construct an Isaac world or use a GPU.

Run using the existing Isaac image's Python (pxr required). JSON on stdout;
redirect only into a newly allocated external archive run. No physical limit
is inferred from mesh extent or from a collision bounding box.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def cube_bounds(size, base_from_prim):
    """Derive a Cube's geometry from size, never an authored/stale USD extent.

    The affine transform intentionally includes scale, unlike a rigid TF pose.
    This is primitive geometry, not arbitrary cooked-mesh/skin-margin truth.
    """
    transform = np.asarray(base_from_prim, dtype=float)
    if (
        not np.isfinite(size)
        or size <= 0
        or transform.shape != (4, 4)
        or not np.isfinite(transform).all()
        or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
        or abs(np.linalg.det(transform[:3, :3])) < 1e-12
    ):
        raise ValueError("positive primitive size and nonsingular affine transform required")
    half = float(size) / 2
    corners = np.array([[x, y, z] for x in (-half, half) for y in (-half, half) for z in (-half, half)])
    points = corners @ transform[:3, :3].T + transform[:3, 3]
    if not np.isfinite(points).all():
        raise ValueError("primitive bound overflow")
    return dict(minimum=points.min(axis=0).tolist(), maximum=points.max(axis=0).tolist())


def inspect(asset):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(asset))
    if not stage or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9:
        raise ValueError("expected USD stage in metres")
    bases = [prim for prim in stage.Traverse() if prim.GetName() == "base_link"]
    if len(bases) != 1:
        raise ValueError("ambiguous physical base_link")
    base = bases[0]
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    base_inverse = cache.GetLocalToWorldTransform(base).GetInverse()
    bounds = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide],
        False,
        True,
    )
    records = []
    for prim in stage.Traverse():
        wheel = "wheel" in prim.GetName().lower()
        sensor = prim.GetName() in ("Imu_Sensor", "vlp16")
        collision = prim.HasAPI(UsdPhysics.CollisionAPI)
        if not (wheel or sensor or collision):
            continue
        relative = cache.GetLocalToWorldTransform(prim) * base_inverse
        item = dict(
            path=str(prim.GetPath()),
            type=prim.GetTypeName(),
            wheel_name=wheel,
            collision_api=collision,
            base_from_prim=[[float(relative[j][i]) for j in range(4)] for i in range(4)],
        )
        if prim.IsA(UsdPhysics.Joint):
            item["joint_local_pos0"] = list(map(float, UsdPhysics.Joint(prim).GetLocalPos0Attr().Get()))
            item["note"] = "Joint prim transform is not joint attachment position; use joint_local_pos0."
        if prim.IsA(UsdGeom.Imageable):
            box = bounds.ComputeWorldBound(prim).ComputeAlignedBox()
            if not box.IsEmpty():
                low, high = box.GetMin(), box.GetMax()
                corners = [
                    base_inverse.Transform(Gf.Vec3d(x, y, z))
                    for x in (low[0], high[0])
                    for y in (low[1], high[1])
                    for z in (low[2], high[2])
                ]
                # BBoxCache trusts authored extents. Those can be stale and
                # are not guaranteed conservative for the actual collider.
                item["authored_base_aabb_unverified"] = dict(
                    minimum=[min(p[i] for p in corners) for i in range(3)],
                    maximum=[max(p[i] for p in corners) for i in range(3)],
                )
        if prim.IsA(UsdGeom.Cube):
            cube = UsdGeom.Cube(prim)
            item["cube_size"] = float(cube.GetSizeAttr().Get())
            item["primitive_base_aabb"] = cube_bounds(item["cube_size"], item["base_from_prim"])
            authored = np.asarray(cube.GetExtentAttr().Get(), dtype=float)
            expected = np.full((2, 3), item["cube_size"] / 2) * [[-1], [1]]
            item["authored_extent_matches_primitive"] = bool(np.allclose(authored, expected, atol=1e-7, rtol=0))
            item["bounds_source"] = "Cube size and complete affine transform; authored extent ignored"
        records.append(item)
    layers = {
        layer.realPath: hashlib.sha256(Path(layer.realPath).read_bytes()).hexdigest()
        for layer in stage.GetUsedLayers()
        if layer.realPath and Path(layer.realPath).is_file()
    }
    return dict(
        asset=str(asset),
        asset_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),
        layer_sha256=layers,
        base_path=str(base.GetPath()),
        records=records,
        unresolved=[
            "dynamic wheel contact patches",
            "braking deceleration",
            "admissible step",
            "pitch/roll safety limits",
            "traction and soil bearing",
        ],
        qualification=False,
        note="Static primitive bounds only; authored/hierarchy extents are unverified, not conservative collision truth.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("asset", type=Path)
    args = parser.parse_args()
    print(json.dumps(inspect(args.asset), indent=2, allow_nan=False))
