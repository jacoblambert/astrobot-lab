"""Separate, pose-anchored support evidence; never edits LiDAR classes.

The startup footprint and grounded wheel patches are explicit operating
assumptions, not contact-force measurements. Historical patches need current
causal poses and an externally supplied relative-position uncertainty radius.
No commanded-motion interpolation, time-based expiry, or automatic re-seeding.
"""

from collections import defaultdict
from dataclasses import dataclass

import numpy as np
from scipy.spatial import ConvexHull

from local_terrain.support_sweep import swept_polygons, touched_cells
from revision_evidence import validated_transform
from shadow_terrain import integer


def area(p):
    if len(p) < 3:
        return 0.0
    q = np.asarray(p) - p[0]
    return abs(float(np.sum(q[:, 0] * np.roll(q[:, 1], -1) - q[:, 1] * np.roll(q[:, 0], -1))) / 2)


def halfplane(p, a, b, offset=0.0):
    """Clip CCW polygon to left half-plane, optionally eroded in metres."""
    if len(p) == 0:
        return np.empty((0, 2))
    e = b - a
    d = e[0] * (p[:, 1] - a[1]) - e[1] * (p[:, 0] - a[0]) - offset * np.linalg.norm(e)
    result = []
    for i, end in enumerate(p):
        j = (i - 1) % len(p)
        if (d[j] >= 0) != (d[i] >= 0):
            result.append(p[j] + (end - p[j]) * (d[j] / (d[j] - d[i])))
        if d[i] >= 0:
            result.append(end)
    return np.asarray(result, dtype=float).reshape(-1, 2)


def convex(p):
    p = np.asarray(p, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all():
        raise ValueError("finite nondegenerate support polygon required")
    return p[ConvexHull(p).vertices]


def erode(p, radius):
    if not np.isfinite(radius) or radius < 0:
        raise ValueError("finite nonnegative uncertainty required")
    result = p.copy()
    for a, b in zip(p, np.roll(p, -1, axis=0)):
        result = halfplane(result, a, b, radius)
    return result


def subtract(subject, cover):
    """Convex pieces of subject outside cover; no filled corners or hull union."""
    result, remainder = [], subject
    for a, b in zip(cover, np.roll(cover, -1, axis=0)):
        outside = halfplane(remainder, b, a)
        if area(outside) > 1e-12:  # Numerical area tolerance, NOT a geometric gap allowance.
            result.append(outside)
        remainder = halfplane(remainder, a, b)
        if not len(remainder):
            break
    return result


def patch_polygons(pose, patches):
    # Zero-displacement envelope is exact, unlike an outer swept envelope that
    # must never be used as evidence that the rover actually touched an area.
    return list(swept_polygons([validated_transform(pose)], patches))


@dataclass(frozen=True)
class SupportAnchor:
    stamp_ns: int
    polygons: tuple
    source: str
    patch_indices: tuple | None = None


class ContactMemory:
    def __init__(self, patches, footprint, *, capacity=4096):
        if type(capacity) is not int or capacity < 1 or capacity > 16384:
            raise ValueError("bounded contact history required")
        self.boxes = tuple(patches)
        self.patches = patch_polygons(np.eye(4), self.boxes)
        self.floor_z = float(np.mean([p.center[2] for p in self.boxes]))
        self.contact_z_bounds = (
            min(p.center[2] - p.half_extent[2] for p in self.boxes),
            max(p.center[2] + p.half_extent[2] for p in self.boxes),
        )
        self.footprint = convex(footprint)
        self.footprint.flags.writeable = False
        self.capacity = capacity
        self.identity = None
        self.anchors = []
        self.last_stamp = None
        self.evicted = 0
        self.geometry_cache = {}

    def start(self, session, epoch, stamp_ns, *, grounded):
        if self.identity is not None:
            raise ValueError("startup prior cannot be reseeded")
        if not isinstance(session, str) or not session or grounded is not True:
            raise ValueError("explicit grounded startup required")
        identity = (session, integer(epoch, "epoch"))
        stamp_ns = integer(stamp_ns, "contact stamp")
        self.identity = identity
        self.anchors = [SupportAnchor(stamp_ns, (self.footprint.copy(),), "startup")]
        self.last_stamp = stamp_ns

    def observe(self, session, epoch, stamp_ns, *, grounded, supported_wheels=None):
        """Retain only explicitly supported wheels when a mask is supplied.

        None preserves the legacy, declared ALL-wheels-grounded experiment.
        A stable body attitude is not a valid way to infer this mask: a rover
        can remain level with one wheel hanging over a hole. Callers must name
        their evidence source; simulator force truth is evaluation-only unless
        the experiment explicitly declares a contact-sensor input.
        """
        if (session, epoch) != self.identity:
            raise ValueError("contact session/epoch mismatch; reset required")
        stamp_ns = integer(stamp_ns, "contact stamp")
        if stamp_ns <= self.last_stamp:
            raise ValueError("contact stamps must advance")
        selected = None
        if supported_wheels is not None:
            mask = np.asarray(supported_wheels)
            if mask.shape != (len(self.boxes),) or mask.dtype.kind != "b":
                raise ValueError("one explicit boolean support flag per wheel required")
            if grounded is not True and mask.any():
                raise ValueError("supported wheel conflicts with ungrounded observation")
            selected = tuple(map(int, np.flatnonzero(mask)))
        self.last_stamp = stamp_ns
        if grounded is not True or selected == ():
            return
        patches = self.patches if selected is None else [self.patches[i] for i in selected]
        self.anchors.append(SupportAnchor(stamp_ns, tuple(p.copy() for p in patches), "traversed", selected))
        if len(self.anchors) > self.capacity:
            # Eviction cannot resurrect support. The startup record also obeys
            # the bound; it is not silently recreated at the current location.
            expired = self.anchors.pop(0)
            self.geometry_cache.pop(expired.stamp_ns, None)
            self.evicted += 1

    def reset(self):
        self.identity = None
        self.anchors.clear()
        self.last_stamp = None
        self.evicted = 0
        self.geometry_cache.clear()

    def placed(self, session, epoch, publication_ns, resolve):
        """resolve(stamp) -> (local_from_base, XY displacement bound metres) or None.

        Re-evaluate all anchors against ONE publication's pose state. Missing
        anchors are withheld, not left at their previous map positions. Caller
        supplies receipt-causal poses; this module has no GT or pose extrapolator.
        The erosion bound must include displacement at EVERY patch point from
        translation AND rotation, not only base-origin translation error.
        """
        if (session, epoch) != self.identity:
            raise ValueError("contact session/epoch mismatch")
        publication_ns = integer(publication_ns, "publication")
        result, withheld = [], 0
        for anchor in self.anchors:
            if anchor.stamp_ns > publication_ns:
                raise ValueError("future contact evidence")
            resolved = resolve(anchor.stamp_ns)
            if resolved is None:
                withheld += 1
                continue
            pose, uncertainty = resolved
            pose = validated_transform(pose)
            cached = self.geometry_cache.get(anchor.stamp_ns)
            if cached is not None and uncertainty == cached[1] and np.array_equal(pose, cached[0]):
                result.extend(cached[2])
                continue
            if anchor.source == "startup":
                xyz = np.vstack([np.c_[self.footprint, np.full(len(self.footprint), z)] for z in self.contact_z_bounds])
                footprint = (xyz @ pose[:3, :3].T + pose[:3, 3])[:, :2]
                # The deployed footprint includes the actual projected wheels.
                # Use the same contact-height band as the query patches. A
                # zero-thickness plane creates artificial slivers on a tilted
                # body, even for a straight motion inside the deployed footprint.
                polygons = [convex(np.vstack([footprint, *patch_polygons(pose, self.boxes)]))]
            else:
                boxes = self.boxes if anchor.patch_indices is None else [self.boxes[i] for i in anchor.patch_indices]
                polygons = patch_polygons(pose, boxes)
            projected = []
            for placed in polygons:
                placed = erode(placed, uncertainty)
                if area(placed) > 1e-12:
                    placed.flags.writeable = False
                    projected.append((placed, anchor.source))
            self.geometry_cache[anchor.stamp_ns] = (pose.copy(), uncertainty, projected)
            result.extend(projected)
        return result, withheld


class SupportIndex:
    """Per-publication spatial index; polygon coverage, not touched-cell filling."""

    def __init__(self, polygons, origin, resolution, *, max_entries=131072, backend="reference"):
        if backend not in ("reference", "geos"):
            raise ValueError("unknown support geometry backend")
        self.backend = backend
        self.origin = np.asarray(origin, dtype=float)
        self.resolution = resolution
        self.cells = defaultdict(list)
        entries = 0
        for polygon, source in polygons:
            polygon = convex(polygon)
            indices, _ = touched_cells([polygon], origin, resolution)
            entries += len(indices)
            if entries > max_entries:
                raise ValueError("support spatial index budget exceeded")
            for ij in indices:
                self.cells[tuple(ij)].append((polygon, source))
        self.entries = entries
        self.unions = {}
        if backend == "geos":
            from shapely import Polygon, union_all

            # Exact double precision union: no snapping, buffering or hull
            # filling. A cell's many overlapping contacts are merged once.
            self.unions = {
                ij: union_all([Polygon(p) for p, _ in values], grid_size=0) for ij, values in self.cells.items()
            }

    def uncovered(self, polygon, ij, *, max_fragments=4096):
        low = self.origin + np.asarray(ij) * self.resolution
        high = low + self.resolution
        square = np.array([low, [high[0], low[1]], high, [low[0], high[1]]])
        part = polygon
        for a, b in zip(square, np.roll(square, -1, axis=0)):
            part = halfplane(part, a, b)
        if area(part) <= 1e-12:
            return 0.0
        if self.backend == "geos":
            from shapely import Polygon, difference

            cover = self.unions.get(tuple(ij))
            return area(part) if cover is None else float(difference(Polygon(part), cover, grid_size=0).area)
        fragments = [part]
        for cover, _ in reversed(self.cells.get(tuple(ij), [])):
            fragments = [piece for fragment in fragments for piece in subtract(fragment, cover)]
            if len(fragments) > max_fragments:
                raise ValueError("support polygon work budget exceeded")
            if not fragments:
                return 0.0
        return sum(area(p) for p in fragments)


def supported_motion(layers, record, poses, patches, body, index):
    """Support-aware geometric query; unchanged LiDAR hazards/body checks win."""
    layers = np.asarray(layers)
    if layers.ndim != 3 or layers.shape[2] != 6 or not np.isin(layers[:, :, 0], [-1, 0, 20, 100]).all():
        raise ValueError("valid six-channel terrain required")
    if not np.array_equal(index.origin, record["origin"]) or index.resolution != record["resolution"]:
        raise ValueError("support/grid frame mismatch")
    body_cells, _ = touched_cells(swept_polygons(poses, [body]), record["origin"], record["resolution"])
    height, width = layers.shape[:2]
    outside = False
    for x, y in body_cells:
        if not (0 <= x < width and 0 <= y < height):
            outside = True
        elif layers[y, x, 0] == 100:
            return dict(state="blocked", reason="body_obstacle")
    if outside:
        return dict(state="unknown", reason="outside_grid")
    unresolved, credited = set(), set()
    for polygon in swept_polygons(poses, patches):
        cells, _ = touched_cells([polygon], record["origin"], record["resolution"])
        for x, y in cells:
            if not (0 <= x < width and 0 <= y < height):
                return dict(state="unknown", reason="outside_grid")
            cost, reason = layers[y, x, :2]
            if cost == 100:
                return dict(state="blocked", reason="wheel_obstacle")
            if cost != -1:
                continue
            # Only lack-of-evidence reasons can be supplied by contact. A
            # negative/positive edge, raised surface or conflicting layer wins.
            if reason not in (0, 4, 9, 12):
                unresolved.add((x, y))
            elif index.uncovered(polygon, (x, y)) > 1e-12:
                unresolved.add((x, y))
            else:
                credited.add((x, y))
    return dict(
        state="unknown" if unresolved else "available",
        reason="new_ground" if unresolved else "supported",
        unknown_cells=len(unresolved),
        support_credited_cells=len(credited - unresolved),
    )
