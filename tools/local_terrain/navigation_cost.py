"""Experimental planner costs, NOT rewritten terrain labels or probabilities.

Only a single unknown cell enclosed by eight measured-ground neighbours may
receive a finite uncertainty penalty. All tests use the ORIGINAL classes in
one pass: assumptions, inferred cells and previous promotions cannot bootstrap
an expanding free region. Explicit edge/clearance conflicts remain unknown.
"""

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt

from local_terrain.costmap import inflate


@dataclass(frozen=True)
class GapPolicy:
    cost: int = 70
    max_cell_m: float = 0.100000001
    max_plane_residual_m: float = 0.02
    max_slope_deg: float = 20.0
    hazard_clearance_m: float = 0.20

    def __post_init__(self):
        if type(self.cost) is not int or not 20 < self.cost < 100:
            raise ValueError("uncertainty cost must lie between inferred ground and occupied")
        values = (self.max_cell_m, self.max_plane_residual_m, self.max_slope_deg, self.hazard_clearance_m)
        if not np.isfinite(values).all() or min(values) <= 0 or self.max_slope_deg >= 90:
            raise ValueError("positive finite bounded gap policy required")


def navigation_costs(layers, *, resolution=0.1, enabled=True, policy=GapPolicy()):
    """Return cost [-1,0..100] and a distinct low-confidence mask.

    These integer costs are NOT calibrated obstacle probabilities. Channel 5
    is deliberately not used to assert evidence freshness. No GT input.
    """
    layers = np.asarray(layers)
    if layers.ndim != 3 or layers.shape[2] != 6 or type(enabled) is not bool:
        raise ValueError("six-channel terrain and explicit policy toggle required")
    classes, reasons, ground = (layers[:, :, i] for i in (0, 1, 2))
    costs = inflate(classes, resolution=resolution)
    uncertain = np.zeros(classes.shape, dtype=bool)
    if not enabled or min(classes.shape) < 3 or resolution > policy.max_cell_m:
        return costs, uncertain
    # Missing observations / insufficient support only. In particular clearance,
    # positive/negative edges and ground-fit conflicts cannot be promoted.
    eligible = (classes[1:-1, 1:-1] == -1) & np.isin(reasons[1:-1, 1:-1], (0, 4, 9, 12))
    if not eligible.any():
        return costs, uncertain
    neighbours, offsets = [], []
    h, w = classes.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if not (dx or dy):
                continue
            sl = (slice(1 + dy, h - 1 + dy), slice(1 + dx, w - 1 + dx))
            eligible &= classes[sl] == 0  # Measured ground only, not class 20.
            neighbours.append(ground[sl])
            offsets.append([dx * resolution, dy * resolution, 1.0])
    # No candidate means the plane fit cannot affect any output. Avoid its
    # full-grid temporaries on dense maps and connected visibility gaps.
    if not eligible.any():
        return costs, uncertain
    heights = np.stack(neighbours, axis=-1)
    eligible &= np.isfinite(heights).all(axis=-1)
    design = np.asarray(offsets)
    coefficients = np.where(np.isfinite(heights), heights, 0) @ np.linalg.pinv(design).T
    residual = np.max(np.abs(heights - coefficients @ design.T), axis=-1)
    slope = np.rad2deg(np.arctan(np.linalg.norm(coefficients[:, :, :2], axis=-1)))
    eligible &= (residual <= policy.max_plane_residual_m) & (slope <= policy.max_slope_deg)
    conflicts = (classes == 100) | ((classes == -1) & ~np.isin(reasons, (0, 4, 9, 12)))
    if conflicts.any():
        distance = distance_transform_edt(~conflicts) * resolution
        eligible &= distance[1:-1, 1:-1] > policy.hazard_clearance_m
    uncertain[1:-1, 1:-1] = eligible
    costs[uncertain] = policy.cost
    if (classes == 100).any():
        distance = distance_transform_edt(classes != 100) * resolution
        taper = np.ceil(99 * np.maximum(0, 1 - distance)).astype(np.int8)
        costs[uncertain] = np.maximum(costs[uncertain], taper[uncertain])
    return costs, uncertain
