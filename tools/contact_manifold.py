"""Decode bounded PhysX contact buffers; contact samples are not filled terrain."""

import numpy as np


def decode_contacts(data, *, capacity):
    if data is None or len(data) != 6:
        raise ValueError("missing detailed contact buffers")
    forces, points, normals, separations, counts, starts = map(np.asarray, data)
    if counts.shape != starts.shape or counts.ndim != 2 or not counts.size:
        raise ValueError("invalid contact pair layout")
    if (
        not np.issubdtype(counts.dtype, np.integer)
        or not np.issubdtype(starts.dtype, np.integer)
        or np.any(counts < 0)
        or np.any(starts < 0)
        or np.any(starts + counts > capacity)
    ):
        raise ValueError("invalid contact pair ranges")
    if (
        points.shape != (capacity, 3)
        or normals.shape != (capacity, 3)
        or forces.shape != (capacity, 1)
        or separations.shape != (capacity, 1)
    ):
        raise ValueError("unexpected contact buffer capacity")
    indices = np.concatenate([np.arange(s, s + c) for s, c in zip(starts.flat, counts.flat)]).astype(int)
    if len(indices) >= capacity or len(set(indices)) != len(indices):
        raise ValueError("saturated or overlapping contact buffers")
    arrays = [forces[indices, 0], points[indices], normals[indices], separations[indices, 0]]
    if not all(np.isfinite(a).all() for a in arrays):
        raise ValueError("nonfinite active contact")
    return dict(
        normal_force_n=arrays[0].tolist(),
        points_world=arrays[1].tolist(),
        normals_world=arrays[2].tolist(),
        separation_m=arrays[3].tolist(),
        pair_counts=counts.tolist(),
        pair_starts=starts.tolist(),
    )


def summarize_contacts(rows):
    from scipy.spatial.transform import Rotation

    results = []
    for wheel in range(4):
        placed, spans, missing = [], [], 0
        for row in rows:
            data = row["floor_contacts"][wheel + 1]
            points = np.asarray(data["points_world"], dtype=float).reshape(-1, 3)
            if not len(points):
                missing += 1
                continue
            q = row["orientations_wxyz"][0]
            local = Rotation.from_quat([*q[1:], q[0]]).inv().apply(points - row["positions"][0])
            placed.append(local)
            spans.append(np.ptp(local, axis=0))
        points = np.concatenate(placed) if placed else np.empty((0, 3))
        results.append(
            dict(
                wheel=wheel,
                frames=len(rows),
                missing_contact_frames=missing,
                points=len(points),
                minimum_base_m=points.min(axis=0).tolist() if len(points) else None,
                maximum_base_m=points.max(axis=0).tolist() if len(points) else None,
                maximum_single_frame_span_m=np.max(spans, axis=0).tolist() if spans else None,
            )
        )
    return dict(
        wheels=results,
        filled_contact_area_certified=False,
        note="Solver manifold points only; neither their hull nor their bounds prove filled support area.",
    )
