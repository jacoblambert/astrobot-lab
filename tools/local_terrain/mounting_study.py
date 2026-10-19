"""Counterfactual sampling diagnostics, never terrain labels or motion permission.

Flat plane, fixed attitude, no self-occlusion/noise. Grid results count cells with
at least one ideal return, NOT full-cell support. Virtual yaw is not a controller.
"""

import numpy as np

from revision_evidence import validated_transform


PITCHES = (5, 10, 15, 20, 30, -10, -20, -30)
PHASES = (0.0, 0.025, 0.05, 0.075)


def sensor_mount(pitch, offset=(0.4, 0.0, 0.25)):
    if not np.isfinite(pitch) or np.asarray(offset).shape != (3,) or not np.isfinite(offset).all():
        raise ValueError("finite pitch and XYZ offset required")
    angle = np.deg2rad(pitch)
    c, s = np.cos(angle), np.sin(angle)
    mount = np.eye(4)
    mount[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    mount[:3, 3] = offset
    return mount


def beam_directions(azimuth, zenith):
    a, z = [np.asarray(value, dtype=float) for value in (azimuth, zenith)]
    if any(v.ndim != 1 or not len(v) or not np.isfinite(v).all() for v in (a, z)):
        raise ValueError("finite nonempty angle vectors required")
    a, z = np.meshgrid(a, z, indexing="ij")
    return np.stack([np.cos(a) * np.cos(z), np.sin(a) * np.cos(z), -np.sin(z)], axis=-1).reshape(-1, 3)


def world_rays(mount, base_height, azimuth, zenith, yaw=0.0):
    mount = validated_transform(mount)
    if not np.isfinite([base_height, yaw]).all() or base_height <= 0:
        raise ValueError("positive loaded height and finite yaw required")
    c, s = np.cos(yaw), np.sin(yaw)
    rotate = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    origin = rotate @ mount[:3, 3] + [0, 0, base_height]
    directions = beam_directions(azimuth, zenith) @ mount[:3, :3].T @ rotate.T
    return origin, directions


def first_intersections(origin, directions, boxes=()):
    """Exact flat z=0 plane and closed axis-aligned box first intersections.

    Range filtering is deliberately separate: an object inside minimum range
    still occludes farther objects in this physical-ray model. An origin inside
    a box hits at zero, unlike some engine ray-query conventions.
    """
    origin, directions = np.asarray(origin, float), np.asarray(directions, float)
    if (
        origin.shape != (3,)
        or directions.ndim != 2
        or directions.shape[1] != 3
        or not np.isfinite(origin).all()
        or not np.isfinite(directions).all()
        or not np.allclose(np.linalg.norm(directions, axis=1), 1, atol=1e-9, rtol=0)
        or origin[2] <= 0
    ):
        raise ValueError("finite origin above ground and unit ray directions required")
    depth = np.full(len(directions), np.inf)
    np.divide(-origin[2], directions[:, 2], out=depth, where=directions[:, 2] < 0)
    labels = np.where(np.isfinite(depth), 0, -1)
    for index, (low, high) in enumerate(boxes, 1):
        low, high = np.asarray(low, float), np.asarray(high, float)
        if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or np.any(high <= low):
            raise ValueError("finite positive-volume box required")
        near, far = np.full(len(depth), -np.inf), np.full(len(depth), np.inf)
        possible = np.ones(len(depth), bool)
        for axis in range(3):
            moving = directions[:, axis] != 0
            a, b = np.full(len(depth), -np.inf), np.full(len(depth), np.inf)
            np.divide(low[axis] - origin[axis], directions[:, axis], out=a, where=moving)
            np.divide(high[axis] - origin[axis], directions[:, axis], out=b, where=moving)
            possible &= moving | ((low[axis] <= origin[axis]) & (origin[axis] <= high[axis]))
            near = np.maximum(near, np.minimum(a, b))
            far = np.minimum(far, np.maximum(a, b))
        distance = np.maximum(near, 0)
        nearer = possible & (far >= distance) & (distance < depth)
        depth[nearer], labels[nearer] = distance[nearer], index
    return depth, labels


def valid_returns(depth, minimum, maximum):
    if not np.isfinite([minimum, maximum]).all() or not 0 <= minimum < maximum:
        raise ValueError("valid sensor range required")
    return np.isfinite(depth) & (depth > minimum + 1e-5) & (depth <= maximum)


def azimuth_coverage(angles, resolution_degrees):
    """Bound the largest angular gap, accounting for the native seam gap.

    This is azimuth coverage only, not temporal completeness or support. Return
    Python scalars so a valid fixture cannot lose its report to NumPy JSON types.
    """
    angles = np.asarray(angles, float)
    if (
        angles.ndim != 1
        or not len(angles)
        or not np.isfinite(angles).all()
        or not np.isfinite(resolution_degrees)
        or not 0 < resolution_degrees < 180
    ):
        raise ValueError("finite nonempty azimuth and positive angular resolution required")
    angles = np.sort(np.mod(angles, 2 * np.pi))
    gap = float(np.max(np.diff(np.r_[angles, angles[0] + 2 * np.pi])))
    return dict(largest_azimuth_gap_rad=gap, complete_azimuth=bool(gap < 2.1 * np.deg2rad(resolution_degrees)))


def hazard_boxes():
    """Eight separate-bearing 10x10x8 cm boxes, fixed development scene.

    Alternating 1.25/2.25 m base-centered ranges. This is not a frozen minimum
    hazard specification or evidence that a single return suffices to detect it.
    """
    boxes = []
    for index in range(8):
        angle = index * np.pi / 4
        radius = 1.25 if index % 2 == 0 else 2.25
        x, y = radius * np.cos(angle), radius * np.sin(angle)
        boxes.append(([x - 0.05, y - 0.05, 0], [x + 0.05, y + 0.05, 0.08]))
    return boxes


def region_masks(wheel_xy, phase):
    """Fixed 6 m / 10 cm grid; patch-band cells with positive-area overlap.

    Sector denominators use cell centers; bands use rectangular overlap, not
    fitted coverage. All candidates share every denominator within each phase.
    """
    wheels = np.asarray(wheel_xy, float)
    if wheels.shape != (4, 2) or not np.isfinite(wheels).all() or phase not in PHASES:
        raise ValueError("four wheel XY locations and a declared phase required")
    yy, xx = np.indices((60, 60))
    lower_x, lower_y = -3 + phase + xx * 0.1, -3 + phase + yy * 0.1
    x, y = lower_x + 0.05, lower_y + 0.05
    radius = np.hypot(x, y)
    annulus = (radius >= 0.75) & (radius <= 3)
    masks = dict(
        front=annulus & (x >= np.abs(y)),
        rear=annulus & (-x >= np.abs(y)),
        left=annulus & (y > np.abs(x)),
        right=annulus & (-y > np.abs(x)),
    )
    front_x = wheels[:, 0].max()
    for lo, hi in ((0.0, 0.25), (0.25, 0.5), (0.5, 1.0)):
        mask = np.zeros((60, 60), bool)
        for side in (-1, 1):
            center_y = wheels[wheels[:, 1] * side > 0, 1].mean()
            mask |= (
                (lower_x < front_x + hi)
                & (lower_x + 0.1 > front_x + lo)
                & (lower_y < center_y + 0.06)
                & (lower_y + 0.1 > center_y - 0.06)
            )
        masks[f"wheel_band_{lo:g}_{hi:g}m"] = mask
    return masks


def sampling_metrics(points, wheel_xy, phase):
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("finite XYZ returns required")
    indices = np.floor((points[:, :2] + 3 - phase) / 0.1).astype(int)
    inside = ((indices >= 0) & (indices < 60)).all(axis=1)
    indices = indices[inside]
    hit = np.zeros((60, 60), bool)
    hit[indices[:, 1], indices[:, 0]] = True
    return {
        name: dict(cells=int(mask.sum()), hit_cells=int((mask & hit).sum()), fraction=float(hit[mask].mean()))
        for name, mask in region_masks(wheel_xy, phase).items()
    }


def sampled_cone_visibility(mount, height, wheel_xy, zenith, yaws, minimum, maximum):
    """Optimistic cone at sampled yaw poses; fixed INITIAL ground targets.

    A positive result is not a discrete beam return. A negative sampled result
    is not an exact impossibility proof over the continuous yaw interval.
    """
    mount = validated_transform(mount)
    targets = np.column_stack([wheel_xy, np.full(len(wheel_xy), -height)])
    seen = np.zeros(len(targets), bool)
    for yaw in yaws:
        c, s = np.cos(yaw), np.sin(yaw)
        inverse_yaw = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        local = (targets @ inverse_yaw.T - mount[:3, 3]) @ mount[:3, :3]
        angle = np.arctan2(-local[:, 2], np.linalg.norm(local[:, :2], axis=1))
        seen |= (
            valid_returns(np.linalg.norm(local, axis=1), minimum, maximum)
            & (angle >= min(zenith))
            & (angle <= max(zenith))
        )
    return seen.tolist()
