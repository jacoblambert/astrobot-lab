"""One-publication index for the same latest-available PoseTimeline semantics.

The reference timeline sorts/filter-scans every frame for every observation.
Preparing its immutable rows once changes neither interpolation nor availability.
Keep the ordinary timeline implementation as the differential oracle.
"""

import bisect

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from shadow_terrain import Association, immutable_pose, integer


class PublicationPoseLookup:
    def __init__(self, timeline, output_ns):
        self.output_ns = integer(output_ns, "publication bound")
        self.session, self.epoch, self.max_gap_ns = timeline.session, timeline.epoch, timeline.max_gap_ns
        rows = []
        for history in timeline.frames.values():
            known = [row for row in history if row[0] <= self.output_ns]
            if known:
                rows.append(known[-1])
        self.rows = sorted(rows, key=lambda row: row[2].stamp_ns)
        self.times = [row[2].stamp_ns for row in self.rows]
        self.interpolators = {}  # bounded by adjacent row pairs in this publication only

    def associate(self, acquisition_ns):
        stamp = integer(acquisition_ns, "acquisition")
        if stamp > self.output_ns:
            raise ValueError("acquisition after publication")
        index = bisect.bisect_left(self.times, stamp)
        if index < len(self.times) and self.times[index] == stamp:
            before = after = self.rows[index]
            pose = before[2].map_from_imu.copy()
        else:
            if index == 0 or index == len(self.times):
                raise ValueError("missing causal bracket: extrapolation forbidden")
            before, after = self.rows[index - 1], self.rows[index]
            gap = after[2].stamp_ns - before[2].stamp_ns
            if gap > self.max_gap_ns:
                raise ValueError("pose interpolation gap exceeds 0.2-second contract")
            fraction = (stamp - before[2].stamp_ns) / gap
            pose = np.eye(4)
            pose[:3, 3] = (1 - fraction) * before[2].map_from_imu[:3, 3] + fraction * after[2].map_from_imu[:3, 3]
            interpolator = self.interpolators.get(index)
            if interpolator is None:
                interpolator = Slerp(
                    [0.0, 1.0],
                    Rotation.from_matrix(np.array([before[2].map_from_imu[:3, :3], after[2].map_from_imu[:3, :3]])),
                )
                self.interpolators[index] = interpolator
            pose[:3, :3] = interpolator([fraction]).as_matrix()[0]
        return Association(
            self.session,
            self.epoch,
            stamp,
            max(before[0], after[0]),
            self.output_ns,
            "revised",
            (before[2].frame_id, after[2].frame_id),
            (before[1], after[1]),
            (before[2].stamp_ns, after[2].stamp_ns),
            immutable_pose(pose @ before[2].imu_from_lidar),
        )
