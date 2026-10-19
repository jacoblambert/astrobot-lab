"""Optimistic flat-ground visibility bound, not sensing/terrain acceptance.

Replace discrete beams by a continuous elevation interval and full azimuth.
Ignore occlusion/noise. This can establish that a retention window is too short
for direct observation, but cannot certify that any actual scan observes ground.
Prior poses differ by straight forward translation only; mount and attitude are
fixed. Native Isaac zenith is positive DOWN, unlike mathematical elevation.
"""

import numpy as np

from revision_evidence import validated_transform


def _roots(a, b, c):
    # Exact zero degree reduction; do not erase small nonzero coefficients.
    coefficients = [float(a), float(b), float(c)]
    while coefficients and coefficients[0] == 0:
        coefficients.pop(0)
    return [float(v.real) for v in np.roots(coefficients) if abs(v.imag) < 1e-9]


def minimum_forward_history(base_from_lidar, base_height, ground_xy, zenith, *, minimum=0.5, maximum=100.0):
    """Infimum prior forward travel needed to place this flat-ground XY in cone.

    Returns distance, not a selected retention policy. None means no interval of
    prior travel is visible within the declared range. A zero result still does
    not establish angular sampling, footprint support or a real return.
    """
    mount = validated_transform(base_from_lidar)
    xy, angles = np.asarray(ground_xy, dtype=float), np.asarray(zenith, dtype=float)
    if xy.shape != (2,) or not np.isfinite(xy).all():
        raise ValueError("finite ground query XY required")
    if angles.ndim != 1 or not len(angles) or not np.isfinite(angles).all() or np.any(np.abs(angles) >= np.pi / 2):
        raise ValueError("native zenith angles in (-pi/2,pi/2) required")
    if not np.isfinite([base_height, minimum, maximum]).all() or base_height <= 0 or not 0 <= minimum < maximum:
        raise ValueError("positive loaded height and valid range interval required")
    point = np.array([*xy, -base_height])
    q = mount[:3, :3].T @ (point - mount[:3, 3])
    direction = mount[:3, :3].T[:, 0]
    lo, hi = float(angles.min()), float(angles.max())
    limit = maximum + np.linalg.norm(q)

    def visible(distance):
        v = q + distance * direction
        radius = np.linalg.norm(v)
        angle = np.arctan2(-v[2], np.linalg.norm(v[:2]))
        return minimum <= radius <= maximum and lo <= angle <= hi

    if visible(0):
        return 0.0
    cuts = [0.0, float(limit)]
    for radius in (minimum, maximum):
        cuts.extend(_roots(direction @ direction, 2 * (q @ direction), q @ q - radius**2))
    for angle in (lo, hi):
        t2 = np.tan(angle) ** 2
        cuts.extend(
            _roots(
                direction[2] ** 2 - t2 * (direction[:2] @ direction[:2]),
                2 * (q[2] * direction[2] - t2 * (q[:2] @ direction[:2])),
                q[2] ** 2 - t2 * (q[:2] @ q[:2]),
            )
        )
    if direction[2] != 0:
        cuts.append(float(-q[2] / direction[2]))
    cuts = sorted({value for value in cuts if np.isfinite(value) and 0 <= value <= limit})
    for start, end in zip(cuts, cuts[1:]):
        if visible((start + end) / 2):
            return start
    return None
