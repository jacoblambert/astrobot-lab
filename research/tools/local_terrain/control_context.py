"""Receipt-causal current-pose context for advisory motion queries.

A map's evidence pose is NOT the pose at which a new command is consumed.
Use the newest actually available estimator sample, report its age, and never
extrapolate it to the current clock. This module does not certify stopping or
tracking error and has no command authority.
"""

from dataclasses import dataclass

import numpy as np

from local_terrain.pose_lookup import PublicationPoseLookup
from revision_evidence import validated_transform
from shadow_terrain import integer


@dataclass(frozen=True)
class CurrentPoseContext:
    query_ns: int
    pose_ns: int
    evidence_ns: int
    publication_ns: int
    local_from_base: np.ndarray
    session: str
    epoch: int
    revision: int

    @property
    def pose_age_s(self):
        return (self.query_ns - self.pose_ns) / 1e9

    @property
    def terrain_age_s(self):
        return (self.query_ns - self.evidence_ns) / 1e9

    def poses(self, base_relative_poses):
        """Anchor a candidate to the current sample, never the older grid pose."""
        deltas = [validated_transform(p) for p in base_relative_poses]
        if not 1 <= len(deltas) <= 129 or not np.array_equal(deltas[0], np.eye(4)):
            raise ValueError("bounded candidate must start at current base identity")
        return [self.local_from_base @ delta for delta in deltas]


def current_context(
    timeline, record, base_from_imu, query_ns, *, max_pose_age_ns=200_000_000, max_map_age_ns=500_000_000
):
    """Require one coherent generation and bounded measured ages.

    The age limits are advisory operating contracts, NOT calibrated error
    bounds. A controller must additionally enclose latency, stopping and actual
    tracking error. A later revision requires a new coherent map/context.
    """
    query = integer(query_ns, "query timestamp")
    evidence = integer(record["evidence_ns"], "evidence timestamp")
    publication = integer(record["publication_ns"], "map publication")
    for value in (max_pose_age_ns, max_map_age_ns):
        if type(value) is not int or value <= 0:
            raise ValueError("positive bounded age contract required")
    identity = (record["session"], record["epoch"], record["revision"])
    if identity != (timeline.session, timeline.epoch, timeline.revision):
        raise ValueError("map/pose generation mismatch")
    if not 0 <= evidence <= publication <= query:
        raise ValueError("map is from the future or has invalid timestamps")
    if query - evidence > max_map_age_ns:
        raise ValueError("stale terrain snapshot")
    lookup = PublicationPoseLookup(timeline, query)
    rows = [row for row in lookup.rows if row[2].stamp_ns <= query]
    if not rows:
        raise ValueError("no causal current pose")
    stamp = rows[-1][2].stamp_ns
    if stamp < evidence or query - stamp > max_pose_age_ns:
        raise ValueError("stale current pose")
    # Same exact IMU/base chain as the mapper. Never substitute nominal LiDAR
    # mounting constants or a retrospective/fitted map-to-GT alignment.
    pose = (
        validated_transform(record["local_from_map"])
        @ lookup.associate(stamp).map_from_lidar
        @ np.linalg.inv(validated_transform(timeline.extrinsic))
        @ np.linalg.inv(validated_transform(base_from_imu))
    )
    pose.flags.writeable = False
    return CurrentPoseContext(query, stamp, evidence, publication, pose, *identity)


def displacement_bound(*, radius_m, translation_error_m, angle_error_rad, speed_m_s, yaw_rate_rad_s, age_s):
    """Enclose displacement at every body point; no claim inputs are calibrated.

    Caller must supply independently justified nonnegative bounds. The returned
    radius accounts for stale-pose motion, translation error and angular error;
    braking distance and command-tracking error must be added by a controller.
    """
    values = [radius_m, translation_error_m, angle_error_rad, speed_m_s, yaw_rate_rad_s, age_s]
    if not np.isfinite(values).all() or min(values) < 0:
        raise ValueError("finite nonnegative motion/error bounds required")
    angle = min(np.pi, angle_error_rad + yaw_rate_rad_s * age_s)
    return float(translation_error_m + speed_m_s * age_s + 2 * radius_m * np.sin(angle / 2))
