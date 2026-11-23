"""Conservative, orientation-independent candidate filtering; Nav2 owns exact paths."""
import math
from collections import deque

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


def component_near(labels, known_free, start, max_steps):
    """component_at, or when the start cell lacks clearance (a rover beside a rock or an unseen
    patch), the components of clear cells reachable from it through known-free cells within
    max_steps 4-connected steps. The walk never crosses an obstacle or unknown cell, so it
    cannot jump a wall; Nav2 still owns the exact path out."""
    found = component_at(labels, start)
    if found.any() or max_steps <= 0 or start is None:
        return found
    known_free = np.asarray(known_free, dtype=bool)
    if not (0 <= start[0] < labels.shape[0] and 0 <= start[1] < labels.shape[1]) or not known_free[start]:
        return found
    seen, queue, values = {tuple(start)}, deque([(tuple(start), 0)]), set()
    while queue:
        (row, col), steps = queue.popleft()
        if labels[row, col]:
            values.add(int(labels[row, col]))
            continue
        if steps == max_steps:
            continue
        for nr, nc in ((row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)):
            if (0 <= nr < labels.shape[0] and 0 <= nc < labels.shape[1] and known_free[nr, nc]
                    and (nr, nc) not in seen):
                seen.add((nr, nc))
                queue.append(((nr, nc), steps + 1))
    return np.isin(labels, sorted(values)) if values else found
