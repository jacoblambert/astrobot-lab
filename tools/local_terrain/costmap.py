"""Advisory display costs from v2 classes, never a navigation safety certificate.

Unknown stays unknown, occupied stays 100. A finite distance taper only raises
known traversable costs; it does not invent support behind an occluder. Original
classes must also be published so inflation cannot masquerade as classification.
"""

import numpy as np
from scipy.ndimage import distance_transform_edt

from revision_evidence import validated_transform


def inflate(classes, *, resolution=0.1, radius=1.0):
    classes = np.asarray(classes)
    if classes.ndim != 2 or not classes.size or not np.isin(classes, [-1, 0, 20, 100]).all():
        raise ValueError("expected v2 unknown/measured/inferred/occupied classes")
    if not np.isfinite([resolution, radius]).all() or resolution <= 0 or radius <= 0:
        raise ValueError("positive finite resolution and inflation radius required")
    result = classes.astype(np.int8, copy=True)
    occupied = classes == 100
    if occupied.any():
        distance = distance_transform_edt(~occupied) * resolution
        # 100 is exclusively measured occupied, not an inflated obstacle label.
        taper = np.ceil(99 * np.maximum(0, 1 - distance / radius)).astype(np.int8)
        known_free = (classes == 0) | (classes == 20)
        result[known_free] = np.maximum(result[known_free], taper[known_free])
    return result


def map_from_grid(record):
    """Transform the grid CORNER and plane, not its center or the rover pose."""
    origin = np.asarray(record["origin"], dtype=float)
    if origin.shape != (2,) or not np.isfinite(origin).all():
        raise ValueError("finite 2D grid origin required")
    local_from_grid = np.eye(4)
    local_from_grid[:2, 3] = origin
    return np.linalg.inv(validated_transform(record["local_from_map"])) @ local_from_grid
