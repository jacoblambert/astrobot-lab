"""Sensor-range validity only; no terrain labels, height or robot footprint crop."""
import numpy as np


def valid_range_mask(points, minimum, maximum, tolerance=1e-5):
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("expected Nx3 sensor-frame XYZ")
    if (not np.isfinite([minimum,maximum,tolerance]).all() or not 0 <= tolerance < minimum < maximum
            or maximum-minimum<=2*tolerance):
        raise ValueError("invalid sensor range limits")
    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points,axis=1)
    near = finite & (ranges <= minimum+tolerance)
    far = finite & (ranges >= maximum-tolerance)
    keep = finite & ~near & ~far
    return keep, dict(input_points=len(points), nonfinite=int((~finite).sum()),
                      near_limit=int(near.sum()), far_limit=int(far.sum()), retained=int(keep.sum()),
                      range_tolerance_m=tolerance)
