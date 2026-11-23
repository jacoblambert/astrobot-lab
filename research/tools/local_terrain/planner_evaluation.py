"""Evaluator-only swept-path checks. Never imported by cost generation/search.

Rock XY bounds are conservative possible-collision tests, not exact mesh contact.
A separate GT-start evaluation applies the commanded relative path at the true
current pose; it is hypothetical perfect tracking, NOT fitted SLAM alignment.
Survey coverage is sampled floor evidence, not proof about unsampled small holes.
"""

import numpy as np
from shapely import Polygon, union_all

from local_terrain.support_sweep import swept_polygons
from revision_evidence import validated_transform


def path_sweep(poses, patches):
    """Evaluate a bounded route edge-by-edge, retaining each motion's budget.

    The active-query limit belongs to one motion, not an entire multi-turn
    route. Union each edge before combining, to bound intermediate polygons.
    """
    if not 1 <= len(poses) <= 30002:
        raise ValueError("bounded nonempty planner path required")
    edges = zip(poses, poses[1:]) if len(poses) > 1 else [(poses[0], poses[0])]
    return union_all([union_all([Polygon(p) for p in swept_polygons(edge, patches)]) for edge in edges])


def evaluate_path(poses, record, home_xyz, rocks, survey_region, patches, body, *, gt_start=None):
    if not poses:
        return dict(evaluated=False, reason="no_path")
    world_from_local = np.linalg.inv(validated_transform(record["local_from_map"]))
    world_from_local[:3, 3] += np.asarray(home_xyz)
    nominal = [world_from_local @ validated_transform(p) for p in poses]
    paths = {"map_frame": nominal}
    if gt_start is not None:
        transform = validated_transform(gt_start) @ np.linalg.inv(nominal[0])
        paths["gt_start_relative_tracking"] = [transform @ p for p in nominal]
    results = {}
    for label, path in paths.items():
        body_sweep = path_sweep(path, [body])
        wheel_sweep = path_sweep(path, patches)
        hits = [name for name, polygon in rocks if body_sweep.intersects(polygon)]
        uncovered = None if survey_region is None else float(wheel_sweep.difference(survey_region).area)
        results[label] = dict(
            possible_rock_guard_intersections=hits,
            wheel_area_outside_survey_m2=uncovered,
            surveyed_floor_only=uncovered is not None and uncovered <= 1e-12,
            sampled_clear_and_no_rock_guard_intersection=not hits and uncovered is not None and uncovered <= 1e-12,
        )
    return dict(
        evaluated=True,
        missing_current_gt=gt_start is None,
        references=results,
        actual_tracking_tested=False,
        navigation_qualified=False,
    )
