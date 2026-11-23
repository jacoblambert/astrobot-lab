"""Incremental support unions, with full rebuild on grid-phase changes.

GEOS geometry is immutable. Each update commits a complete new index only after
all bounds and geometry operations succeed. Geometry keys are exact, not rounded
poses: a correction removes the old polygons rather than leaving a free trail.
"""

from collections import defaultdict

import numpy as np
from shapely import Polygon, union_all

from local_terrain.contact_support import SupportIndex, convex
from local_terrain.support_sweep import touched_cells


class IncrementalSupportIndex:
    backend = "geos"

    uncovered = SupportIndex.uncovered

    def __init__(self, *, max_entries=131072, max_polygons=16389):
        self.max_entries, self.max_polygons = max_entries, max_polygons
        self.reference_origin = None
        self.resolution = None
        self.geometry = {}
        self.members = {}
        self.global_unions = {}
        self.unions = {}
        self.origin = None
        self.entries = 0
        self.rebuilt_cells = 0

    def update(self, polygons, origin, resolution):
        origin = np.asarray(origin, dtype=float)
        if origin.shape != (2,) or not np.isfinite(origin).all() or not np.isfinite(resolution) or resolution <= 0:
            raise ValueError("valid support grid required")
        if len(polygons) > self.max_polygons:
            raise ValueError("support polygon budget exceeded")
        reference = self.reference_origin
        if reference is None or resolution != self.resolution:
            reference = origin.copy()
        shift = np.rint((origin - reference) / resolution).astype(int)
        if not np.allclose(origin - reference, shift * resolution, rtol=0, atol=1e-12):
            reference, shift = origin.copy(), np.zeros(2, dtype=int)
        rebuild = (
            self.reference_origin is None
            or resolution != self.resolution
            or not np.array_equal(reference, self.reference_origin)
        )
        old = {} if rebuild else self.geometry
        requested = {}
        for p, source in polygons:
            p = np.asarray(p, dtype=float)
            if p.ndim != 2 or p.shape[1] != 2 or not 3 <= len(p) <= 32 or not np.isfinite(p).all():
                raise ValueError("invalid bounded support polygon")
            requested[(p.shape, p.tobytes(), source)] = p
        removed = old.keys() - requested.keys()
        added = requested.keys() - old.keys()
        geometry = {key: value for key, value in old.items() if key not in removed}
        members = {} if rebuild else self.members.copy()
        dirty = set()

        def edit(ij):
            if ij not in dirty:
                members[ij] = set(members.get(ij, ()))
                dirty.add(ij)
            return members[ij]

        for key in removed:
            for ij in old[key][1]:
                edit(ij).remove(key)
        for key in added:
            p = convex(requested[key])
            cells, _ = touched_cells([p], reference, resolution)
            indices = tuple(map(tuple, cells))
            geometry[key] = (Polygon(p), indices)
            for ij in indices:
                edit(ij).add(key)
        entries = sum(map(len, members.values()))
        if entries > self.max_entries:
            raise ValueError("support spatial index budget exceeded")
        unions = {} if rebuild else self.global_unions.copy()
        for ij in dirty:
            if members[ij]:
                # Sort exact keys for deterministic union inputs; set iteration
                # order must not change numerical results between processes.
                unions[ij] = union_all([geometry[key][0] for key in sorted(members[ij])], grid_size=0)
            else:
                members.pop(ij)
                unions.pop(ij, None)
        projected = {(int(ij[0] - shift[0]), int(ij[1] - shift[1])): g for ij, g in unions.items()}
        self.geometry, self.members, self.global_unions = geometry, members, unions
        self.reference_origin, self.resolution, self.origin = reference.copy(), resolution, origin.copy()
        self.entries, self.rebuilt_cells, self.unions = entries, len(dirty), projected
        return self
