"""Float64 evaluator-only triangle reference; never repairs sensor measurements."""

import numpy as np


def intersections(origin, direction, triangles, max_distance=100.0):
    """Two-sided ray intersections, no triangle expansion or fitted alignment."""
    origin, direction, triangles = (np.asarray(a, dtype=np.float64) for a in (origin, direction, triangles))
    if (
        origin.shape != (3,)
        or direction.shape != (3,)
        or triangles.ndim != 3
        or triangles.shape[1:] != (3, 3)
        or not all(np.isfinite(a).all() for a in (origin, direction, triangles))
        or abs(np.linalg.norm(direction) - 1.0) > 1e-7
        or not np.isfinite(max_distance)
        or max_distance <= 0
    ):
        raise ValueError("invalid diagnostic ray/triangles")
    e1, e2 = triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    p = np.cross(direction, e2)
    det = np.einsum("ij,ij->i", e1, p)
    tvec = origin - triangles[:, 0]
    q = np.cross(tvec, e1)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.einsum("ij,ij->i", tvec, p) / det
        v = q @ direction / det
        distance = np.einsum("ij,ij->i", e2, q) / det
    valid = (det != 0) & (u >= 0) & (v >= 0) & (u + v <= 1) & (distance >= 0) & (distance <= max_distance)
    ids = np.flatnonzero(valid)
    ids = ids[np.argsort(distance[ids], kind="stable")]
    return [
        dict(
            triangle=int(i),
            distance=float(distance[i]),
            u=float(u[i]),
            v=float(v[i]),
            position=(origin + direction * distance[i]).tolist(),
        )
        for i in ids
    ]
