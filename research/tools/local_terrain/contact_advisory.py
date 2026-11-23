"""Bounded current-pose support advisory. Never publishes commands or edits maps.

This is the shared query path for replay and future live integration. The default
history only retains wheel patches already classified as supported by LiDAR.
Unknown current wheel locations are a separate in-place operating assumption,
not a reason to write free terrain into historical tracks. Optional assumed
grounded history and the smaller core remain explicitly experimental policies.
The optional current turning footprint also supplies support only at the actual
query pose; it is neither retained as history nor moved along proposed paths.
"""

import numpy as np

from local_terrain.contact_index import IncrementalSupportIndex
from local_terrain.contact_query import PreparedMotion, evaluate_support_core
from local_terrain.contact_support import ContactMemory, convex, patch_polygons
from local_terrain.control_context import current_context
from local_terrain.physx_reference import BodyBox
from local_terrain.pose_lookup import PublicationPoseLookup
from local_terrain.support_sweep import touched_cells
from revision_evidence import validated_transform


def observed_wheels(layers, record, pose, patches):
    """Whole declared patch has LiDAR support; does not infer a contact force."""
    result = []
    height, width = layers.shape[:2]
    for polygon in patch_polygons(pose, patches):
        cells, _ = touched_cells([polygon], record["origin"], record["resolution"])
        x, y = cells.T
        inside = (x >= 0) & (y >= 0) & (x < width) & (y < height)
        result.append(bool(inside.all() and np.isin(layers[y[inside], x[inside], 0], (0, 20)).all()))
    return result


class ContactAdvisory:
    def __init__(
        self,
        patches,
        footprint,
        body,
        *,
        history_erosion_m=0.02,
        history_mode="lidar_confirmed",
        support_core=False,
        current_turn_footprint=None
    ):
        if history_mode not in ("lidar_confirmed", "assumed_grounded"):
            raise ValueError("explicit contact history policy required")
        if not np.isfinite(history_erosion_m) or history_erosion_m < 0:
            raise ValueError("finite nonnegative history erosion required")
        if type(support_core) is not bool:
            raise ValueError("explicit support-core policy required")
        self.memory = ContactMemory(patches, footprint)
        self.current_turn_footprint = None if current_turn_footprint is None else convex(current_turn_footprint)
        self.patches, self.body = tuple(patches), body
        self.query_patches = (
            tuple(BodyBox(p.center, (0.02, 0.03, 0.001), "experimental_core") for p in patches)
            if support_core
            else self.patches
        )
        self.support_core = support_core
        self.history_mode = history_mode
        self.history_erosion_m = history_erosion_m
        self.index = IncrementalSupportIndex()
        self.context = None
        self.layers = self.record = None
        self.last_query_ns = -1
        self.missing_anchors = 0
        self.supported_wheels = []

    def update(self, layers, record, timeline, base_from_imu, query_ns, *, grounded, deployment_confirmed=False):
        # Invalidate FIRST: any failure must remove the previous available answer.
        self.context = self.layers = self.record = None
        if query_ns < self.last_query_ns:
            raise ValueError("advisory clock regressed")
        self.last_query_ns = query_ns
        if grounded is not True:
            raise ValueError("current grounded operating assumption not available")
        layers = np.asarray(layers)
        if (
            layers.ndim != 3
            or layers.shape[2] != 6
            or not all(layers.shape[:2])
            or not np.isin(layers[:, :, 0], (-1, 0, 20, 100)).all()
        ):
            raise ValueError("complete six-channel terrain required")
        context = current_context(timeline, record, base_from_imu, query_ns)
        if self.memory.identity is None:
            if deployment_confirmed is not True:
                raise ValueError("one explicitly confirmed clear deployment required")
            self.memory.start(context.session, context.epoch, context.pose_ns, grounded=True)
        elif self.memory.identity != (context.session, context.epoch):
            raise ValueError("new session/epoch requires a new deployment; no automatic reseed")
        confirmed = observed_wheels(layers, record, context.local_from_base, self.patches)
        if context.pose_ns > self.memory.last_stamp:
            self.memory.observe(
                context.session,
                context.epoch,
                context.pose_ns,
                grounded=True,
                supported_wheels=confirmed if self.history_mode == "lidar_confirmed" else None,
            )
        elif context.pose_ns < self.memory.last_stamp:
            raise ValueError("current pose timestamp regressed")
        lookup = PublicationPoseLookup(timeline, query_ns)
        local_from_map = validated_transform(record["local_from_map"])
        lidar_from_base = np.linalg.inv(validated_transform(timeline.extrinsic)) @ np.linalg.inv(
            validated_transform(base_from_imu)
        )

        def resolve(stamp):
            try:
                pose = local_from_map @ lookup.associate(stamp).map_from_lidar @ lidar_from_base
            except ValueError:
                return None
            # At the exact same anchor timestamp, relative placement error is
            # zero: query and prior use the same estimator pose. This does NOT
            # remove historical erosion once the rover/sample has advanced.
            return pose, (0.0 if stamp == context.pose_ns else self.history_erosion_m)

        polygons, missing = self.memory.placed(context.session, context.epoch, query_ns, resolve)
        # Current unknown-under-wheel coverage is local to this query. It is
        # never appended to history unless the chosen evidence policy allows it.
        polygons += [(p, "current_operating_assumption") for p in patch_polygons(context.local_from_base, self.patches)]
        if self.current_turn_footprint is not None:
            # One assumption at the ACTUAL consumer pose, shared by all candidate
            # queries. Never re-center on a hypothetical search node or retain
            # this disk as observed historical ground. Hazard vetoes stay in query.
            xyz = np.vstack(
                [
                    np.c_[self.current_turn_footprint, np.full(len(self.current_turn_footprint), z)]
                    for z in self.memory.contact_z_bounds
                ]
            )
            pose = context.local_from_base
            polygon = convex((xyz @ pose[:3, :3].T + pose[:3, 3])[:, :2])
            polygons.append((polygon, "current_turn_operating_assumption"))
        self.index.update(polygons, record["origin"], record["resolution"])
        self.layers = layers.copy()
        self.layers.flags.writeable = False
        self.record = dict(record, origin=np.asarray(record["origin"]).copy())
        self.missing_anchors, self.supported_wheels = missing, confirmed
        self.context = context
        return self.status()

    def status(self):
        status = dict(
            valid=self.context is not None,
            query_ns=self.context.query_ns if self.context else None,
            pose_ns=self.context.pose_ns if self.context else None,
            pose_age_s=self.context.pose_age_s if self.context else None,
            terrain_age_s=self.context.terrain_age_s if self.context else None,
            history_mode=self.history_mode,
            history_anchors=len(self.memory.anchors),
            withheld_history_anchors=self.missing_anchors,
            lidar_confirmed_wheels=list(self.supported_wheels),
            positive_support_policy="core_40x60mm" if self.support_core else "full_80x120mm",
            uncertainty_calibrated=False,
            tracking_and_stopping_evaluated=False,
            acceptance=False,
            navigation_authority=False,
        )
        if self.current_turn_footprint is not None:
            status["current_turn_area_assumed"] = True
        return status

    def query(self, base_relative_poses, *, query_ns):
        if self.context is None or query_ns != self.context.query_ns:
            raise ValueError("update advisory at the current consumer clock before querying")
        poses = self.context.poses(base_relative_poses)
        full = PreparedMotion(self.layers, self.record, poses, self.patches, self.body)
        result = (
            evaluate_support_core(
                full, PreparedMotion(self.layers, self.record, poses, self.query_patches, self.body), self.index
            )
            if self.support_core
            else full.evaluate(self.index)
        )
        return dict(result, **self.status())
