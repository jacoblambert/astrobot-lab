"""Continuous swept-patch CLASS AVAILABILITY, not a vehicle safety certificate.

The input patches are a declared geometric experiment, not inferred tire contact
mechanics. Rotating envelopes cover the entire linear/SLERP trajectory. Every
touched cell, including grid-edge/outside and unknown cells, counts. No sampling
of only wheel centers/endpoints, smoothing, or free-space filling is performed.

The current six mapper channels do not expose support-provenance age or bounded
relative-pose uncertainty. In particular channel 5 is NOT the oldest evidence
used by neighboring corner certificates. Never turn this diagnostic into motion
authority using that age or the plane residual in channel 3.
"""

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from local_terrain.physx_reference import envelope_segments


def swept_polygons(poses, patches, *, max_segments=512):
    """Conservative XY projections of complete swept 3-D patch boxes."""
    for segment in envelope_segments(poses, patches, max_queries=max_segments * 2):
        half = segment["half_extent"]
        corners = np.array(
            [[x, y, z] for x in (-half[0], half[0]) for y in (-half[1], half[1]) for z in (-half[2], half[2])]
        )
        world = Rotation.from_quat(segment["quaternion"]).apply(corners) + segment["position"]
        points = np.vstack([world[:, :2], (world + segment["displacement"])[:, :2]])
        yield points[ConvexHull(points).vertices]


def touched_cells(polygons, origin, resolution, *, max_cell_tests=32768):
    """Closed-cell polygon intersection using separating axes (no raster gaps).

    Boundary-only contacts count on both sides. The floating-point outward pad
    is numerical conservatism, NOT a localization-uncertainty allowance.
    Returns signed grid indices so callers cannot silently crop missing support.
    """
    origin = np.asarray(origin, dtype=float)
    if origin.shape != (2,) or not np.isfinite(origin).all() or not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("finite grid origin and positive resolution required")
    if type(max_cell_tests) is not int or not 1 <= max_cell_tests <= 1_000_000:
        raise ValueError("bounded swept-cell workload required")
    work, result = 0, set()
    for polygon in polygons:
        polygon = (np.asarray(polygon, dtype=float) - origin) / resolution
        if polygon.ndim != 2 or polygon.shape[1] != 2 or not 3 <= len(polygon) <= 32:
            raise ValueError("bounded convex swept polygon required")
        if not np.isfinite(polygon).all() or np.abs(polygon).max() > 1e9:
            raise ValueError("invalid or excessive grid coordinates")
        # Own hull construction here as well: arbitrary point ordering cannot
        # trick SAT into missing interior cells. Degenerate hulls fail closed.
        polygon = polygon[ConvexHull(polygon).vertices]
        pad = 32 * np.finfo(float).eps * max(1.0, float(np.abs(polygon).max()))
        low = np.floor(polygon.min(axis=0) - pad).astype(np.int64)
        high = np.floor(polygon.max(axis=0) + pad).astype(np.int64)
        count = int(high[0] - low[0] + 1) * int(high[1] - low[1] + 1)
        work += count
        if work > max_cell_tests:
            raise ValueError("swept-cell work budget exceeded")
        x, y = np.meshgrid(np.arange(low[0], high[0] + 1), np.arange(low[1], high[1] + 1))
        indices = np.c_[x.ravel(), y.ravel()]
        centers = indices + 0.5
        edges = np.roll(polygon, -1, axis=0) - polygon
        axes = np.vstack([np.eye(2), np.c_[-edges[:, 1], edges[:, 0]]])
        axes /= np.linalg.norm(axes, axis=1)[:, None]
        vertices = polygon @ axes.T
        projected = centers @ axes.T
        radius = 0.5 * np.abs(axes).sum(axis=1)
        hit = np.all(
            (projected + radius >= vertices.min(axis=0) - pad) & (projected - radius <= vertices.max(axis=0) + pad),
            axis=1,
        )
        result.update(map(tuple, indices[hit]))
    if not result:
        raise ValueError("empty swept support request")
    return np.asarray(sorted(result), dtype=np.int64), work


def support_availability(layers, origin, resolution, poses, patches, *, max_cell_tests=32768):
    """Check full swept patch coverage; supported != feasible or independent GT."""
    layers = np.asarray(layers)
    if layers.ndim != 3 or layers.shape[2] != 6 or not all(layers.shape[:2]):
        raise ValueError("six-channel terrain projection required")
    if not np.isin(layers[:, :, 0], [-1, 0, 20, 100]).all():
        raise ValueError("invalid terrain classes")
    indices, work = touched_cells(swept_polygons(poses, patches), origin, resolution, max_cell_tests=max_cell_tests)
    x, y = indices.T
    inside = (x >= 0) & (y >= 0) & (x < layers.shape[1]) & (y < layers.shape[0])
    cost = np.full(len(indices), -1.0, dtype=float)
    cost[inside] = layers[y[inside], x[inside], 0]
    free = np.isin(cost, [0, 20])
    blocked = cost == 100
    reasons = layers[y[inside & ~free], x[inside & ~free], 1]
    if not np.isfinite(reasons).all() or not np.equal(reasons, np.floor(reasons)).all():
        raise ValueError("invalid rejection reason")
    values, counts = np.unique(reasons.astype(int), return_counts=True)
    return dict(
        support_class_state="blocked" if blocked.any() else "supported" if free.all() else "unknown",
        cells=len(indices),
        free_cells=int(free.sum()),
        occupied_cells=int(blocked.sum()),
        unknown_cells=int((cost == -1).sum()),
        outside_cells=int((~inside).sum()),
        rejection_reasons={str(k): int(v) for k, v in zip(values, counts)},
        cell_tests=work,
        acceptance=False,
        navigation_authority=False,
        terrain_truth_evaluated=False,
        body_clearance_evaluated=False,
        support_freshness_certified=False,
        relative_pose_uncertainty_evaluated=False,
        scope="continuous assumed patch envelopes; map-class availability only, not motion feasibility",
    )


def diagnostic_motions(local_from_base):
    """Frozen 0.10 m / +/-0.10 rad diagnostic queries; includes stationary/startup."""
    from revision_evidence import validated_transform

    start = validated_transform(local_from_base).copy()
    motions = {"stationary": [start]}
    for name, distance, yaw in (("forward_10cm", 0.1, 0), ("left_0.1rad", 0, 0.1), ("right_0.1rad", 0, -0.1)):
        delta = np.eye(4)
        delta[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
        delta[0, 3] = distance
        motions[name] = [start, start @ delta]
    return motions
