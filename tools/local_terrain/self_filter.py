"""Calibration-only chassis return exclusion. Never cast through the body.

The bounds come from the asset's active Cube size/affine transform plus 1 mm
numeric margin, not terrain labels or authored USD extent. Native evidence stays
unchanged; rejecting a self hit does not recover anything behind that hit.
"""

import numpy as np


def chassis_returns(points, setup):
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("finite sensor-local XYZ required")
    bounds = setup.get("chassis_exclusion_base_aabb_m")
    if bounds is None:
        return np.zeros(len(points), dtype=bool)
    bounds = np.asarray(bounds, float)
    if bounds.shape != (2, 3) or not np.isfinite(bounds).all() or np.any(bounds[0] >= bounds[1]):
        raise ValueError("invalid chassis exclusion bounds")
    if setup["mount_rpy_deg"] != [0, 0, 0]:
        raise ValueError("chassis filter requires explicitly level sensor calibration")
    base = points + np.asarray(setup["base_from_lidar_translation_m"])
    return np.all((base >= bounds[0]) & (base <= bounds[1]), axis=1)
