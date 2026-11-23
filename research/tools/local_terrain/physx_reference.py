"""Evaluator-only continuous body-envelope queries against an active PhysX scene.

Uses existing box overlap/sweep APIs, not authored or reconstructed terrain.
Linear translation + shortest SLERP is the declared segment model. Rotation is
enclosed conservatively; a hit is only a potential collision of the supplied
bounding box, not proof that the actual vehicle collides. Wheel support, loading,
stopping and traversability remain separate. Run on the simulation thread while
the scene token is stable; never issue these queries from a ROS callback.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from revision_evidence import validated_transform


@dataclass(frozen=True)
class BodyBox:
    center: tuple
    half_extent: tuple
    source_identity: str

    def __post_init__(self):
        if (
            np.asarray(self.center).shape != (3,)
            or np.asarray(self.half_extent).shape != (3,)
            or not np.isfinite([self.center, self.half_extent]).all()
            or np.any(np.asarray(self.half_extent) <= 0)
            or not isinstance(self.source_identity, str)
            or not self.source_identity
        ):
            raise ValueError("finite positive body envelope and geometry identity required")
        object.__setattr__(self, "center", tuple(map(float, self.center)))
        object.__setattr__(self, "half_extent", tuple(map(float, self.half_extent)))


def envelope_segments(poses, boxes, *, angle_step=0.05, max_queries=512):
    """Yield swept fixed-orientation boxes enclosing the FULL rotating motion.

    At a segment's midpoint orientation, every body point deviates at most
    2*r*sin(angle/4) from its frozen-orientation point. Include the box's offset
    from base in r. Isotropic extent inflation encloses this deviation ball.
    """
    poses = [validated_transform(p).copy() for p in poses]
    boxes = tuple(boxes)
    if not poses or not boxes or len(poses) > 256 or len(boxes) > 32:
        raise ValueError("empty or excessive reference trajectory/envelopes")
    if not np.isfinite(angle_step) or not 0 < angle_step <= 0.1 or type(max_queries) is not int or max_queries < 1:
        raise ValueError("invalid bounded reference work contract")
    pairs = list(zip(poses, poses[1:])) or [(poses[0], poses[0])]
    work, plan = 0, []
    for first, last in pairs:
        rotations = Rotation.from_matrix(np.array([first[:3, :3], last[:3, :3]]))
        angle = (rotations[0].inv() * rotations[1]).magnitude()
        count = max(1, math.ceil(angle / angle_step))
        work += 2 * count * len(boxes)  # Initial overlap plus sweep; static also bounded.
        if work > max_queries:
            raise ValueError("active reference query budget exceeded")
        plan.append((first, last, rotations, angle, count))
    for first, last, rotations, angle, count in plan:
        orientations = Slerp([0, 1], rotations)((np.arange(count) + 0.5) / count)
        for i, orientation in enumerate(orientations):
            start = first[:3, 3] + (last[:3, 3] - first[:3, 3]) * (i / count)
            end = first[:3, 3] + (last[:3, 3] - first[:3, 3]) * ((i + 1) / count)
            for box in boxes:
                radius = np.linalg.norm(box.center) + np.linalg.norm(box.half_extent)
                inflation = float(2 * radius * math.sin(angle / (4 * count)))
                yield dict(
                    position=start + orientation.apply(box.center),
                    half_extent=np.asarray(box.half_extent) + inflation,
                    quaternion=orientation.as_quat(),
                    displacement=end - start,
                    inflation_m=inflation,
                    source_identity=box.source_identity,
                )


def body_clearance(query, poses, boxes, *, scene_token, robot_root, max_queries=512):
    """A clear result is about supplied body envelopes ONLY; no drive authority."""
    if not isinstance(robot_root, str) or not robot_root.startswith("/") or robot_root.endswith("/"):
        raise ValueError("exact robot prim root required for self exclusion")
    before = scene_token()
    if not isinstance(before, tuple) or len(before) != 2 or not isinstance(before[0], str) or not before[0]:
        raise ValueError("scene token must identify immutable collider epoch and physics tick")
    if type(before[1]) is not int or before[1] < 0:
        raise ValueError("invalid reference physics tick")
    # Materialize and validate the bounded work before invoking any scene query.
    segments = list(envelope_segments(poses, boxes, max_queries=max_queries))
    hits, errors, performed = set(), [], 0

    def report(hit):
        try:
            path = hit.collision
            if not isinstance(path, str) or not path.startswith("/"):
                raise ValueError("invalid active collider identity")
            if path != robot_root and not path.startswith(robot_root + "/"):
                hits.add(path)  # Never exclude terrain or other robots.
            return True
        except Exception as exc:
            errors.append(str(exc))
            return False

    for segment in segments:
        half, position, quaternion = [tuple(map(float, segment[k])) for k in ("half_extent", "position", "quaternion")]
        query.overlap_box(half, position, quaternion, report, False)
        performed += 1
        distance = float(np.linalg.norm(segment["displacement"]))
        if distance > 0:
            direction = tuple(map(float, segment["displacement"] / distance))
            query.sweep_box_all(half, position, quaternion, direction, distance, report, True)
            performed += 1
        if errors:
            raise ValueError("active collision callback failed: " + errors[0])
    if scene_token() != before:
        raise ValueError("scene/physics tick changed during reference query")
    return dict(
        body_envelope_clear=not hits,
        possible_collisions=sorted(hits),
        query_count=performed,
        maximum_rotation_inflation_m=max(s["inflation_m"] for s in segments),
        geometry_sources=sorted({s["source_identity"] for s in segments}),
        scene_epoch=before[0],
        physics_tick=before[1],
        acceptance=False,
        navigation_authority=False,
        wheel_support_evaluated=False,
        interpolation="linear_translation_shortest_slerp",
        limitation="Supplied bounding boxes only; a hit is not an exact-body collision certificate.",
    )
