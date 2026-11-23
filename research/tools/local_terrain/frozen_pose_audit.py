"""GT-only relative registration audit; never mapper input or uncertainty proof."""

import numpy as np

from revision_evidence import validated_transform


def registration_errors(current_estimated, old_estimated, current_truth, old_truth):
    """Old sensor poses relative to CURRENT physical base, with no fitted gauge.

    Each estimate/truth pair can use unrelated world gauges; relative transforms
    cancel them independently. Old poses here are lidar, current poses are base.
    Translation is sensor-origin displacement; range bounds include rotation.
    """
    a, b = map(validated_transform, (current_estimated, current_truth))
    c, d = [np.asarray(value, dtype=float) for value in (old_estimated, old_truth)]
    if c.shape != d.shape or c.ndim != 3 or c.shape[1:] != (4, 4) or not len(c):
        raise ValueError("matching nonempty old sensor transforms required")
    for poses in (c, d):
        if not np.isfinite(poses).all() or not np.allclose(poses[:, 3], [0, 0, 0, 1], atol=1e-9, rtol=0):
            raise ValueError("invalid old sensor transforms")
        rotations = poses[:, :3, :3]
        if not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-6, rtol=0) or not np.allclose(
            np.linalg.det(rotations), 1, atol=1e-6, rtol=0
        ):
            raise ValueError("nonrigid old sensor transform")
    estimated = np.linalg.inv(a)[None] @ c
    truth = np.linalg.inv(b)[None] @ d
    translation = np.linalg.norm(estimated[:, :3, 3] - truth[:, :3, 3], axis=1)
    relative = truth[:, :3, :3].transpose(0, 2, 1) @ estimated[:, :3, :3]
    angle = np.arccos(np.clip((np.trace(relative, axis1=1, axis2=2) - 1) / 2, -1, 1))
    return np.c_[translation, angle, translation + 4 * np.sin(angle / 2), translation + 6 * np.sin(angle / 2)]


def exact_native_index(times_ns, stamp_ns):
    index = np.searchsorted(times_ns, stamp_ns)
    matches = [i for i in (index - 1, index) if 0 <= i < len(times_ns) and abs(int(times_ns[i]) - stamp_ns) <= 2]
    if len(matches) != 1:
        raise ValueError("missing/ambiguous same-native-tick GT pose; interpolation forbidden")
    return matches[0]
