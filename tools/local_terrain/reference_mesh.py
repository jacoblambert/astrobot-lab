"""Evaluator-only continuous support patches from explicit collision triangles.

No raster interpolation, gap filling or mapper labels. This answers a geometric
height-patch question, not wheel mechanics, body collision or traversability.
An active-simulator collision export is still required for real-scene truth.
"""

import hashlib

import numpy as np

from revision_evidence import validated_transform


def signed_area(polygon):
    if len(polygon) < 3:
        return 0.0
    # Translate before summing to avoid cancellation at large world origins.
    p = np.asarray(polygon) - polygon[0]
    return float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1)) / 2)


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def clip_convex(subject, clip):
    """Intersect two CCW convex polygons without outward tolerance expansion."""
    points = list(subject)
    for a, b in zip(clip, np.roll(clip, -1, axis=0)):
        previous = points
        points = []
        if not previous:
            break
        start = previous[-1]
        d_start = cross(b - a, start - a)
        for end in previous:
            d_end = cross(b - a, end - a)
            if (d_start >= 0) != (d_end >= 0):
                points.append(start + (end - start) * (d_start / (d_start - d_end)))
            if d_end >= 0:
                points.append(end)
            start, d_start = end, d_end
    return np.asarray(points, dtype=float).reshape(-1, 2)


class HeightMeshReference:
    def __init__(self, vertices, triangles, reference_from_mesh, *, max_patch_triangles=256):
        vertices, triangles = np.asarray(vertices), np.asarray(triangles)
        transform = validated_transform(reference_from_mesh)
        if vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices) or not np.isfinite(vertices).all():
            raise ValueError("finite collision vertices required")
        if (
            triangles.ndim != 2
            or triangles.shape[1] != 3
            or not len(triangles)
            or triangles.dtype.kind not in "iu"
            or np.any(triangles < 0)
            or np.any(triangles >= len(vertices))
        ):
            raise ValueError("explicit valid triangle indices required; do not invent topology")
        if type(max_patch_triangles) is not int or not 1 <= max_patch_triangles <= 4096:
            raise ValueError("invalid bounded patch workload")
        with np.errstate(over="ignore", invalid="ignore"):
            transformed = vertices.astype(float) @ transform[:3, :3].T + transform[:3, 3]
            self.faces = transformed[triangles].copy()
            self.normals = np.cross(self.faces[:, 1] - self.faces[:, 0], self.faces[:, 2] - self.faces[:, 0])
            magnitudes = np.linalg.norm(self.normals, axis=1)
        if not np.isfinite(self.faces).all() or not np.isfinite(magnitudes).all() or np.any(magnitudes <= 1e-12):
            raise ValueError("degenerate collision triangle")
        self.low, self.high = self.faces[:, :, :2].min(axis=1), self.faces[:, :, :2].max(axis=1)
        self.limit = max_patch_triangles
        self.geometry_sha256 = hashlib.sha256(self.faces.tobytes()).hexdigest()
        for array in (self.faces, self.normals, self.low, self.high):
            array.flags.writeable = False

    def height_patch(self, polygon):
        polygon = np.array(polygon, dtype=float, copy=True)
        if polygon.ndim != 2 or polygon.shape[1] != 2 or not 3 <= len(polygon) <= 32 or not np.isfinite(polygon).all():
            raise ValueError("finite convex contact polygon required")
        if signed_area(polygon) < 0:
            polygon = polygon[::-1]
        area = signed_area(polygon)
        # Every vertex must lie on the inward side of every oriented edge.
        if (
            not np.isfinite(area)
            or area <= 1e-10
            or any(cross(b - a, p - a) < -1e-12 for a, b in zip(polygon, np.roll(polygon, -1, axis=0)) for p in polygon)
        ):
            raise ValueError("nondegenerate convex contact polygon required")
        lower, upper = polygon.min(axis=0), polygon.max(axis=0)
        indices = np.flatnonzero(np.all(self.high >= lower, axis=1) & np.all(self.low <= upper, axis=1))
        report = dict(
            complete=False,
            acceptance=False,
            navigation_authority=False,
            geometry_sha256=self.geometry_sha256,
            query_area_m2=area,
            covered_area_m2=0.0,
            candidates=len(indices),
            reason=None,
            minimum_height_m=None,
            maximum_height_m=None,
            maximum_slope=None,
            scope="continuous height patch only; not full vehicle/sweep truth",
        )
        if len(indices) > self.limit:
            report["reason"] = "reference_work_budget"
            return report
        clipped, heights, slopes = [], [], []
        covered = 0.0
        for index in indices:
            face, normal = self.faces[index], self.normals[index]
            if abs(normal[2]) <= 1e-12:
                # A vertical face's height cannot be represented by this oracle.
                # Conservatively refuse a query near its projected AABB.
                report["reason"] = "vertical_face_requires_volume_reference"
                return report
            footprint = face[:, :2]
            if signed_area(footprint) < 0:
                footprint = footprint[::-1]
            patch = clip_convex(footprint, polygon)
            patch_area = abs(signed_area(patch))
            if patch_area <= 1e-14:
                continue
            for old in clipped:
                if np.any(old.max(axis=0) < patch.min(axis=0)) or np.any(patch.max(axis=0) < old.min(axis=0)):
                    continue
                if abs(signed_area(clip_convex(patch, old))) > 1e-12:
                    report["reason"] = "overlapping_or_multilevel_reference"
                    return report
            clipped.append(patch)
            covered += patch_area
            gradient = -normal[:2] / normal[2]
            heights.extend(face[0, 2] + (patch - face[0, :2]) @ gradient)
            slopes.append(float(np.linalg.norm(gradient)))
        report["covered_area_m2"] = covered
        if not heights or abs(area - covered) > max(1e-12, area * 1e-10):
            report["reason"] = "uncovered_reference_area"
            return report
        report.update(
            complete=True,
            reason="complete_single_surface",
            minimum_height_m=float(min(heights)),
            maximum_height_m=float(max(heights)),
            maximum_slope=max(slopes),
        )
        return report
