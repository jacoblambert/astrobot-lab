"""Fresh hazard checks for unknown-permitted trials; no support-history unions.

Missing ground remains unknown. This policy does not certify wheel support and
must never be substituted for the original positive-support advisory silently.
"""

import numpy as np

from local_terrain.contact_advisory import ContactAdvisory, observed_wheels
from local_terrain.contact_query import PreparedMotion
from local_terrain.control_context import current_context


class HazardAdvisory(ContactAdvisory):
    def update(self, layers, record, timeline, base_from_imu, query_ns, *, grounded, deployment_confirmed=False):
        self.context = self.layers = self.record = None
        if query_ns < self.last_query_ns or grounded is not True:
            raise ValueError("invalid current hazard context")
        self.last_query_ns = query_ns
        layers = np.asarray(layers)
        if layers.ndim != 3 or layers.shape[2] != 6 or not all(layers.shape[:2]):
            raise ValueError("complete six-channel terrain required")
        if not np.isin(layers[:, :, 0], (-1, 0, 20, 100)).all():
            raise ValueError("invalid terrain classes")
        context = current_context(timeline, record, base_from_imu, query_ns)
        if self.memory.identity is None:
            if deployment_confirmed is not True:
                raise ValueError("one explicitly confirmed clear deployment required")
            self.memory.start(context.session, context.epoch, context.pose_ns, grounded=True)
        elif self.memory.identity != (context.session, context.epoch):
            raise ValueError("new session/epoch requires a new deployment")
        if context.pose_ns < self.memory.last_stamp:
            raise ValueError("current pose timestamp regressed")
        self.memory.last_stamp = context.pose_ns
        # For reporting only. Neither these flags nor historical wheel support
        # enter command decisions under the explicit unknown-permitted policy.
        self.supported_wheels = observed_wheels(layers, record, context.local_from_base, self.patches)
        self.layers = layers.copy()
        self.layers.flags.writeable = False
        self.record = dict(record, origin=np.asarray(record["origin"]).copy())
        self.context = context
        return self.status()

    def status(self):
        value = super().status()
        value.update(
            history_mode="disabled_unknown_permitted",
            history_anchors=0,
            positive_support_policy="not_required",
            current_turn_area_assumed=False,
        )
        return value

    def query(self, base_relative_poses, *, query_ns):
        if self.context is None or query_ns != self.context.query_ns:
            raise ValueError("fresh current hazard context required")
        sweep = PreparedMotion(
            self.layers, self.record, self.context.poses(base_relative_poses), self.patches, self.body
        )
        return sweep.terminal or (
            dict(state="blocked", reason="wheel_terrain_conflict")
            if sweep.conflicts
            else dict(state="available", reason="hazards_clear_unknown_allowed", unknown_wheel_cells=len(sweep.parts))
        )
