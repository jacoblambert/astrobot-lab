"""Prepared support queries; same geometric policy, no terrain mutation.

GEOS unions remove repeated clipping of overlapping rotational envelopes. The
reference in contact_support remains an independent differential test oracle.
"""

import numpy as np
from shapely import Polygon, box, intersection, union_all, difference

from local_terrain.support_sweep import swept_polygons, touched_cells


def refine_trajectory(poses, *, angle_step=0.005, max_segments=128):
    """Tighter outer envelopes of the SAME linear/SLERP path; no added evidence."""
    from scipy.spatial.transform import Rotation, Slerp
    from revision_evidence import validated_transform

    if not np.isfinite(angle_step) or not 0 < angle_step <= 0.05 or not 1 <= max_segments <= 128:
        raise ValueError("bounded trajectory refinement required")
    poses = [validated_transform(p) for p in poses]
    if not poses or len(poses) > max_segments + 1:
        raise ValueError("trajectory refinement budget")
    result = [poses[0]]
    for first, last in zip(poses, poses[1:]):
        rotations = Rotation.from_matrix(np.array([first[:3, :3], last[:3, :3]]))
        count = max(1, int(np.ceil((rotations[0].inv() * rotations[1]).magnitude() / angle_step)))
        if len(result) + count > max_segments + 1:
            raise ValueError("trajectory refinement budget")
        fractions = np.arange(1, count + 1) / count
        orientations = Slerp([0, 1], rotations)(fractions).as_matrix()
        for fraction, rotation in zip(fractions, orientations):
            pose = np.eye(4)
            pose[:3, :3] = rotation
            pose[:3, 3] = (1 - fraction) * first[:3, 3] + fraction * last[:3, 3]
            result.append(pose)
    return result


def evaluate_support_core(full, core, index):
    """Experimental partial-support policy; full wheel/body hazard veto remains.

    This is an explicit different policy, not an optimization or equivalence
    claim. Unknown-only outer strips may be omitted from positive support needs;
    no occupied cell or classified edge/height conflict is omitted.
    """
    if full.terminal is not None:
        return full.terminal.copy()
    if full.conflicts:
        return dict(state="unknown", reason="full_wheel_terrain_conflict", terrain_conflict_cells=len(full.conflicts))
    return core.evaluate(index)


class PreparedMotion:
    def __init__(self, layers, record, poses, patches, body):
        layers = np.asarray(layers)
        if (
            layers.ndim != 3
            or layers.shape[2] != 6
            or not all(layers.shape[:2])
            or not np.isin(layers[:, :, 0], [-1, 0, 20, 100]).all()
        ):
            raise ValueError("valid six-channel terrain required")
        self.origin = np.asarray(record["origin"], dtype=float)
        self.resolution = record["resolution"]
        self.terminal = None
        self.parts = {}
        self.conflicts = set()
        height, width = layers.shape[:2]
        body_cells, _ = touched_cells(swept_polygons(poses, [body]), self.origin, self.resolution)
        x, y = body_cells.T
        inside = (x >= 0) & (y >= 0) & (x < width) & (y < height)
        if np.any(layers[y[inside], x[inside], 0] == 100):
            self.terminal = dict(state="blocked", reason="body_obstacle")
            return
        if not inside.all():
            self.terminal = dict(state="unknown", reason="outside_grid")
            return
        polygons = list(swept_polygons(poses, patches))
        cells, _ = touched_cells(polygons, self.origin, self.resolution)
        x, y = cells.T
        inside = (x >= 0) & (y >= 0) & (x < width) & (y < height)
        if np.any(layers[y[inside], x[inside], 0] == 100):
            self.terminal = dict(state="blocked", reason="wheel_obstacle")
            return
        if not inside.all():
            self.terminal = dict(state="unknown", reason="outside_grid")
            return
        cost, reason = layers[y, x, :2].T
        missing = cost == -1
        lack_of_evidence = np.isin(reason, (0, 4, 9, 12))
        self.conflicts = set(map(tuple, cells[missing & ~lack_of_evidence]))
        unknown = cells[missing & lack_of_evidence]
        if len(unknown):
            # Union once for the whole query, then clip in a vectorized GEOS
            # operation. Intersection distributes over union; no snapping,
            # filling of holes, thresholds or swept geometry have changed.
            wheel_union = union_all([Polygon(p) for p in polygons], grid_size=0)
            low = self.origin + unknown * self.resolution
            high = low + self.resolution
            squares = box(low[:, 0], low[:, 1], high[:, 0], high[:, 1])
            parts = intersection(wheel_union, squares, grid_size=0)
            self.parts = dict(zip(map(tuple, unknown), parts))

    def evaluate(self, index):
        if (
            index.backend != "geos"
            or not np.array_equal(index.origin, self.origin)
            or index.resolution != self.resolution
        ):
            raise ValueError("prepared support query/index mismatch")
        if self.terminal is not None:
            return self.terminal.copy()
        unresolved, credited = set(self.conflicts), set()
        unknown_area = 0.0
        for ij, part in self.parts.items():
            cover = index.unions.get(ij)
            uncovered = part.area if cover is None else difference(part, cover, grid_size=0).area
            if uncovered > 1e-12:
                unresolved.add(ij)
                unknown_area += uncovered
            else:
                credited.add(ij)
        return dict(
            state="unknown" if unresolved else "available",
            reason="new_ground" if unresolved else "supported",
            unknown_cells=len(unresolved),
            support_credited_cells=len(credited - unresolved),
            new_ground_area_m2=unknown_area,
            terrain_conflict_cells=len(self.conflicts),
        )
