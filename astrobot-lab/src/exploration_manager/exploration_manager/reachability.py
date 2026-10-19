"""Conservative, orientation-independent candidate filtering; Nav2 owns exact paths."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, label


def clearance_components(known_free, resolution, radius):
    """Label 4-connected centers whose circular footprint has observed support.

    Padding treats map boundaries as unsupported. Unlike snapping to a nearby
    free cell, component lookup cannot jump through a wall or an unknown strip.
    The circle is conservative for the rectangular rover; an empty component
    means hold/replan, not permission to bypass Nav2 collision checks.
    """
    if not math.isfinite(resolution) or resolution <= 0 or not math.isfinite(radius) or radius < 0:
        raise ValueError("invalid clearance geometry")
    known_free = np.asarray(known_free, dtype=bool)
    distance = distance_transform_edt(np.pad(known_free, 1, constant_values=False))[1:-1, 1:-1]
    safe = known_free & (distance * resolution > radius)
    return label(safe)[0]


def component_at(labels, start):
    if start is None or not (0 <= start[0] < labels.shape[0] and 0 <= start[1] < labels.shape[1]):
        return np.zeros(labels.shape, dtype=bool)
    value = labels[start]
    return labels == value if value else np.zeros(labels.shape, dtype=bool)
