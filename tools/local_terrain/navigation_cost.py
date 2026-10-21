"""Experimental planner costs, NOT rewritten terrain labels or probabilities.

mode="isolated" (default): only a single unknown cell enclosed by eight
measured-ground neighbours may receive a finite uncertainty penalty.
mode="bounded" (opt-in): any plain-missing unknown cell within
max_support_gap_m of measured ground and farther than hazard_clearance_m from
every conflict may receive it; the interior of a large gap stays unknown.
All tests use the ORIGINAL classes in one pass: assumptions, inferred cells and
previous promotions cannot bootstrap an expanding free region. Explicit
edge/clearance conflicts remain unknown in both modes.

occupied_shadow_lethal_m > 0 (opt-in, either mode): unknown cells within that
distance of an occupied cell are published as occupied (100). Nav2 MPPI with
track_unknown_space does not veto NO_INFORMATION under the footprint, so an
obstacle's unmeasured flank must be lethal in the published costmap itself.
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
    mode: str = "isolated"
    max_support_gap_m: float = 0.5
    occupied_shadow_lethal_m: float = 0.0

    def __post_init__(self):
        if type(self.cost) is not int or not 20 < self.cost < 100:
            raise ValueError("uncertainty cost must lie between inferred ground and occupied")
        if self.mode not in ("isolated", "bounded"):
            raise ValueError("gap policy mode must be isolated or bounded")
        values = (self.max_cell_m, self.max_plane_residual_m, self.max_slope_deg, self.hazard_clearance_m,
                  self.max_support_gap_m)
        if not np.isfinite(values).all() or min(values) <= 0 or self.max_slope_deg >= 90:
            raise ValueError("positive finite bounded gap policy required")
        if not np.isfinite(self.occupied_shadow_lethal_m) or self.occupied_shadow_lethal_m < 0:
            raise ValueError("occupied shadow distance must be finite and non-negative")


# Planner-costmap policies the live publisher accepts by name (tools/live_local_terrain.py
# planner_gap_policy). "nav2_bounded_shadow" is the policy the C1 suite gates on
# (docs/decision-2026-10.md): bounded unknown, plus unknown within 0.20 m of an occupied
# cell published lethal, since MPPI tolerates unknown under the footprint.
PUBLISHED_POLICIES = {
    "isolated": GapPolicy(),
    "nav2_bounded_shadow": GapPolicy(mode="bounded", hazard_clearance_m=0.30, occupied_shadow_lethal_m=0.20),
}


def navigation_costs(layers, *, resolution=0.1, enabled=True, policy=GapPolicy()):
    """Return cost [-1,0..100] and a distinct low-confidence mask.

    These integer costs are NOT calibrated obstacle probabilities. Channel 5
    is deliberately not used to assert evidence freshness. No GT input.
    """
    costs, uncertain = _gap_costs(layers, resolution, enabled, policy)
    if policy.occupied_shadow_lethal_m > 0:
        classes = np.asarray(layers)[:, :, 0]
        if (classes == 100).any():
            near = distance_transform_edt(classes != 100) * resolution <= policy.occupied_shadow_lethal_m + 1e-9
            shadow = near & (classes == -1)
            costs[shadow] = 100
            uncertain &= ~shadow
    return costs, uncertain


def _gap_costs(layers, resolution, enabled, policy):
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
    if policy.mode == "bounded":
        return _bounded(classes, reasons, costs, resolution, policy)
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
    return _penalize(classes, costs, uncertain, resolution, policy)


def _bounded(classes, reasons, costs, resolution, policy):
    plain = np.isin(reasons, (0, 4, 9, 12))
    eligible = (classes == -1) & plain
    measured = classes == 0  # not class 20: inference cannot extend the support
    if not eligible.any() or not measured.any():
        return costs, np.zeros(classes.shape, dtype=bool)
    eligible &= distance_transform_edt(~measured) * resolution <= policy.max_support_gap_m
    conflicts = (classes == 100) | ((classes == -1) & ~plain)
    if conflicts.any():
        eligible &= distance_transform_edt(~conflicts) * resolution > policy.hazard_clearance_m
    return _penalize(classes, costs, eligible, resolution, policy)


def _penalize(classes, costs, uncertain, resolution, policy):
    costs[uncertain] = policy.cost
    if (classes == 100).any():
        distance = distance_transform_edt(classes != 100) * resolution
        taper = np.ceil(99 * np.maximum(0, 1 - distance)).astype(np.int8)
        costs[uncertain] = np.maximum(costs[uncertain], taper[uncertain])
    return costs, uncertain
