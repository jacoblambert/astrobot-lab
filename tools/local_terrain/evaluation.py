"""Evaluator-only metrics. These functions never provide mapper input labels."""

import numpy as np

from revision_evidence import validated_transform


def query_grid(layers, origin, local_from_base, query_base, resolution=0.1):
    """Fixed body-relative queries; outside-grid is unknown, never omitted."""
    points = np.asarray(query_base, dtype=float)
    pose = validated_transform(local_from_base)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("invalid fixed query points")
    world = points @ pose[:3, :3].T + pose[:3, 3]
    ij = np.floor((world[:, :2] - np.asarray(origin)) / resolution).astype(int)
    good = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < layers.shape[1]) & (ij[:, 1] < layers.shape[0])
    costs = np.full(len(points), -1.0, dtype=float)
    costs[good] = layers[ij[good, 1], ij[good, 0], 0]
    return costs, ~good


def availability(costs, outside):
    """Map output availability only, explicitly not independent terrain safety."""
    costs, outside = np.asarray(costs), np.asarray(outside)
    if costs.shape != outside.shape or costs.ndim != 1 or not np.isin(costs, [-1, 0, 20, 100]).all():
        raise ValueError("invalid query classes")
    return dict(
        queries=len(costs),
        free=int(np.isin(costs, [0, 20]).sum()),
        occupied=int((costs == 100).sum()),
        unknown=int((costs == -1).sum()),
        outside=int(outside.sum()),
        terrain_truth_evaluated=False,
    )


def independent_score(costs, truth):
    """Unknown truth remains in exposure totals; it never becomes free truth."""
    costs, truth = np.asarray(costs), np.asarray(truth)
    if costs.shape != truth.shape or costs.ndim != 1 or not np.isin(truth, [-1, 0, 100]).all():
        raise ValueError("invalid independent reference")
    if not np.isin(costs, [-1, 0, 20, 100]).all():
        raise ValueError("invalid candidate classes")
    free = np.isin(costs, [0, 20])
    return dict(
        queries=len(costs),
        reference_missing=int((truth == -1).sum()),
        known_free=int((truth == 0).sum()),
        known_hazard=int((truth == 100).sum()),
        false_safe=int((free & (truth == 100)).sum()),
        unnecessary_blocked=int((~free & (truth == 0)).sum()),
    )


def relative_error(estimated_start, estimated_end, truth_start, truth_end):
    """Relative SE(3) discrepancy, not an optimized alignment."""
    a, b, c, d = map(validated_transform, (estimated_start, estimated_end, truth_start, truth_end))
    estimated, truth = np.linalg.inv(a) @ b, np.linalg.inv(c) @ d
    error = np.linalg.inv(truth) @ estimated
    return dict(
        translation_m=float(np.linalg.norm(error[:3, 3])),
        rotation_rad=float(np.arccos(np.clip((np.trace(error[:3, :3]) - 1) / 2, -1, 1))),
    )


def stopping_distance(speed, latency, deceleration, margin):
    if (
        not np.isfinite([speed, latency, deceleration, margin]).all()
        or min(speed, latency, margin) < 0
        or deceleration <= 0
    ):
        raise ValueError("measured positive braking deceleration required")
    return speed * latency + speed**2 / (2 * deceleration) + margin
