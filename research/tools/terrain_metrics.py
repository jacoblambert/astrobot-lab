"""Independent fine-grid reference scoring; never used by the terrain mapper.

Labels: -1 outside independently observable reference, 0 drivable surface,
1 positive obstacle, 2 unsupported/dropoff. Object IDs >0 designate REQUIRED
hazards; include zero-return objects. There is no candidate-derived mask or
spatial dilation. Coarse predicted cells are sampled at fine reference centers.
"""
import numpy as np


def score_terrain(cost, *, map_origin, map_resolution, labels, object_ids, gt_origin, gt_resolution):
    cost, labels, object_ids = np.asarray(cost), np.asarray(labels), np.asarray(object_ids)
    if cost.ndim != 2 or labels.ndim != 2 or labels.shape != object_ids.shape:
        raise ValueError("incompatible grid dimensions")
    if not np.isin(cost, [-1, 0, 20, 100]).all() or not np.isin(labels, [-1, 0, 1, 2]).all():
        raise ValueError("unsupported cost/reference label")
    if not np.isfinite(object_ids).all() or (object_ids < 0).any() or (object_ids != object_ids.astype(int)).any():
        raise ValueError("object IDs must be nonnegative integers")
    if ((object_ids > 0) & (labels != 1)).any():
        raise ValueError("required object labels must be observable obstacles")
    for resolution in (map_resolution, gt_resolution):
        if not np.isfinite(resolution) or resolution <= 0:
            raise ValueError("invalid resolution")
    for origin in (map_origin, gt_origin):
        if np.shape(origin) != (2,) or not np.isfinite(origin).all():
            raise ValueError("invalid origin")
    y, x = np.indices(labels.shape)
    mx = np.floor((gt_origin[0]+(x+.5)*gt_resolution-map_origin[0])/map_resolution).astype(int)
    my = np.floor((gt_origin[1]+(y+.5)*gt_resolution-map_origin[1])/map_resolution).astype(int)
    inside = (mx >= 0) & (my >= 0) & (mx < cost.shape[1]) & (my < cost.shape[0])
    predicted = np.full(labels.shape, -1)
    predicted[inside] = cost[my[inside], mx[inside]]
    free, occupied = (predicted >= 0) & (predicted < 100), predicted == 100
    flat, hazard, dropoff = labels == 0, labels == 1, labels == 2
    objects = {str(int(i)): bool(occupied[object_ids == i].any()) for i in np.unique(object_ids) if i > 0}

    def fraction(mask, event):
        return float(np.count_nonzero(mask & event)/np.count_nonzero(mask)) if mask.any() else None

    return dict(required_objects=objects, object_count=len(objects), detected_objects=sum(objects.values()),
                object_recall=sum(objects.values())/len(objects) if objects else None,
                hazard_cells=int(hazard.sum()), dropoff_cells=int(dropoff.sum()),
                flat_cells=int(flat.sum()), flat_free_fraction=fraction(flat, free),
                flat_false_occupied_fraction=fraction(flat, occupied),
                flat_unknown_fraction=fraction(flat, predicted == -1),
                obstacle_false_free_fraction=fraction(hazard, free), dropoff_free_cells=int((dropoff & free).sum()),
                observable_outside_prediction=int(((labels >= 0) & ~inside).sum()),
                note="Fine GT centers sample exact coarse cells; no tolerance dilation; unknown remains in denominators.")


def calibration_gates(sequence, *, required_objects, p95_ms, p99_ms, peak_rss_mib):
    """Fixed development gates, NOT held-out or navigation acceptance.

    Missing denominators, missing metrics, NaN and empty sequences fail closed.
    Unknown remains a coverage failure; positive and dropoff safety are checked
    at every output frame, including before all required objects are detected.
    """
    final=sequence[-1] if sequence else {}
    finite=lambda value: value is not None and bool(np.isfinite(value))
    fraction=lambda name,limit,lower=False: (finite(final.get(name)) and
        (final[name]>=limit if lower else final[name]<=limit))
    geometry=dict(
        reference_present=bool(sequence and required_objects>0 and final.get("flat_cells",0)>0
                               and all(f.get("dropoff_cells",0)>0 for f in sequence)),
        required_recall=bool(required_objects>0 and final.get("object_count")==required_objects
                             and final.get("detected_objects")==required_objects),
        flat_coverage=fraction("flat_free_fraction",.95,True),
        flat_false_occupied=fraction("flat_false_occupied_fraction",.01),
        final_positive_false_free=fraction("obstacle_false_free_fraction",0.),
        sequence_positive_safe=bool(sequence and all(f.get("obstacle_false_free_fraction")==0. for f in sequence)),
        sequence_dropoff_safe=bool(sequence and all(f.get("dropoff_free_cells")==0 for f in sequence)))
    compute=dict(p95=bool(finite(p95_ms) and 0<=p95_ms<50.),
                 p99=bool(finite(p99_ms) and 0<=p99_ms<100.),
                 process_memory=bool(finite(peak_rss_mib) and 0<peak_rss_mib<=1024.))
    return dict(geometry=geometry,sampled_compute=compute,geometry_passed=all(geometry.values()),
                sampled_compute_passed=all(compute.values()),promotion_allowed=False,
                note="Development replay gates only. Held-out terrain, footprint corridor, SLAM and live-cadence gates remain separate.")
