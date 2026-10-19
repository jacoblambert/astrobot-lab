"""Opt-in 30 cm estimated-pose goal trial, never general navigation authority.

Commands are proposals to the independently guarded simulator driver. The GT
survey/footprint monitor can veto execution but cannot choose a goal or command.
No support rule is changed. Stop at the first refusal lasting two seconds.
"""

import math

import numpy as np

from revision_evidence import validated_transform


PROTOCOL = "terrain_step_trial_v1"
TOPIC = "/terrain_trial/proposal"


class StepTrial:
    def __init__(self, capture):
        self.capture = capture
        self.goal = self.origin = self.target = None
        self.identity = self.reference = None
        self.first_ns = self.last_ns = None
        self.wait_until = 0
        self.refused_since = None
        self.terminal = None
        self.sequence = 0
        self.steps = 0
        self.replans = 0

    def update(self, advisory, stamp):
        context = advisory.context
        if context is None or context.query_ns != stamp:
            raise ValueError("trial requires fresh current-pose advisory")
        current = context.local_from_base
        identity = (context.session, context.epoch)
        reference = np.asarray(advisory.record["local_from_map"])
        if self.goal is None:
            self.origin = current.copy()
            self.goal = current[:2, 3] + 0.30 * current[:2, 0]
            self.identity, self.reference = identity, reference.copy()
            self.first_ns = stamp
        if identity != self.identity or not np.array_equal(reference, self.reference):
            raise ValueError("trial frame/session changed")
        if self.last_ns is not None and stamp <= self.last_ns:
            raise ValueError("trial clock did not advance")
        self.last_ns = stamp
        distance = float(np.linalg.norm(self.goal - current[:2, 3]))
        if np.linalg.norm(current[:2, 3] - self.origin[:2, 3]) > 0.50:
            raise ValueError("trial estimated-position envelope")
        linear = angular = 0.0
        relative = [np.eye(4)]
        decision = dict(state="available", reason="observing")
        if not self.terminal and stamp - self.first_ns > 45_000_000_000:
            self.terminal = "time_limit"
        if not self.terminal and distance <= 0.02:
            self.terminal = "estimated_goal_reached"
        if not self.terminal and stamp >= self.wait_until:
            delta = self.goal - current[:2, 3]
            error = math.atan2(delta[1], delta[0]) - math.atan2(current[1, 0], current[0, 0])
            error = (error + math.pi) % (2 * math.pi) - math.pi
            end = np.eye(4)
            if abs(error) > 0.05:
                angle = math.copysign(math.radians(5), error)
                end[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
                angular = math.copysign(0.04, error)
                self.target = None
            else:
                if self.target is None:
                    self.target = current[:2, 3] + min(0.05, distance) * current[:2, 0]
                remaining = float((self.target - current[:2, 3]) @ current[:2, 0])
                lateral = abs(float((self.target - current[:2, 3]) @ current[:2, 1]))
                if lateral > 0.02:
                    # A short-lived step target is not a fixed route corridor.
                    # Stop when it is no longer straight ahead, then choose a
                    # fresh bounded step toward the SAME fixed goal. Never
                    # continue the old command or count this as progress.
                    self.target = None
                    self.wait_until = stamp + 500_000_000
                    self.replans += 1
                    decision = dict(state="available", reason="stopped_to_replan_step")
                elif remaining <= 0.005:
                    self.steps += 1
                    self.target = None
                    self.wait_until = stamp + 500_000_000
                else:
                    # Enclose the rest of the 5 cm step plus a separately
                    # declared 2 cm experimental stopping/latency allowance.
                    # Not a measured physical stopping certificate.
                    # Estimator motion can increase distance to the retained
                    # target. Recheck only the bounded next increment; never
                    # enlarge the execution envelope to chase that target.
                    end[0, 3] = min(remaining, 0.05) + 0.02
                    linear = 0.02
            if linear or angular:
                relative.append(end)
                decision = advisory.query(relative, query_ns=stamp)
                if decision["state"] != "available":
                    linear = angular = 0.0
                    if self.refused_since is None:
                        self.refused_since = stamp
                    if stamp - self.refused_since >= 2_000_000_000:
                        self.terminal = "terrain_refused"
                else:
                    self.refused_since = None
        if self.terminal:
            linear = angular = 0.0
        self.sequence += 1
        return dict(
            schema=1,
            protocol=PROTOCOL,
            capture=self.capture,
            sequence=self.sequence,
            session=context.session,
            epoch=context.epoch,
            revision=context.revision,
            query_ns=stamp,
            pose_ns=context.pose_ns,
            evidence_ns=context.evidence_ns,
            valid=True,
            linear=linear,
            angular=angular,
            terminal=self.terminal,
            decision=decision,
            relative_poses=[p.tolist() for p in relative],
            local_from_base=current.tolist(),
            local_from_map=reference.tolist(),
            goal_local_xy=self.goal.tolist(),
            remaining_goal_m=distance,
            completed_steps=self.steps,
            step_replans=self.replans,
            navigation_authority=False,
            experiment_execution=True,
            stopping_allowance_m=0.02,
            stopping_allowance_calibrated=False,
        )


def validate_proposal(value, capture, identity, now_ns, *, previous_sequence=0):
    if not isinstance(value, dict) or value.get("schema") != 1 or value.get("protocol") != PROTOCOL:
        raise ValueError("invalid trial proposal")
    if value.get("capture") != capture or (value.get("session"), value.get("epoch")) != tuple(identity[:2]):
        raise ValueError("trial capture/session mismatch")
    if value.get("valid") is not True:
        raise ValueError("terrain trial invalidated")
    sequence, stamp = value["sequence"], value["query_ns"]
    if type(sequence) is not int or sequence <= previous_sequence or type(stamp) is not int:
        raise ValueError("trial proposal sequence/timestamp")
    if not 0 <= now_ns - stamp <= 200_000_000:
        raise ValueError("trial proposal stale/future")
    if not 0 <= stamp - value["pose_ns"] <= 200_000_000 or not 0 <= stamp - value["evidence_ns"] <= 500_000_000:
        raise ValueError("trial proposal stale evidence/pose")
    linear, angular = value["linear"], value["angular"]
    if not np.isfinite([linear, angular]).all() or linear not in (0.0, 0.02) or angular not in (-0.04, 0.0, 0.04):
        raise ValueError("trial command bounds")
    if linear and angular:
        raise ValueError("trial cannot execute an unchecked arc")
    if (linear or angular) and (value["terminal"] or value["decision"]["state"] != "available"):
        raise ValueError("trial motion without available terrain")
    if value["terminal"] not in (None, "terrain_refused", "estimated_goal_reached", "time_limit"):
        raise ValueError("undeclared trial completion")
    validated_transform(value["local_from_base"])
    validated_transform(value["local_from_map"])
    if len(value["relative_poses"]) not in (1, 2):
        raise ValueError("trial path budget")
    for pose in value["relative_poses"]:
        validated_transform(pose)
    if not np.array_equal(value["relative_poses"][0], np.eye(4)):
        raise ValueError("trial path does not start at current base")
    if linear or angular:
        if len(value["relative_poses"]) != 2:
            raise ValueError("moving trial requires a checked sweep")
        end = np.asarray(value["relative_poses"][-1])
        if linear and (
            not np.array_equal(end[:3, :3], np.eye(3))
            or not 0.02 <= end[0, 3] <= 0.071
            or not np.array_equal(end[1:3, 3], [0.0, 0.0])
        ):
            raise ValueError("translation differs from checked sweep")
        if angular:
            angle = math.copysign(math.radians(5), angular)
            expected = np.eye(4)
            expected[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
            if not np.array_equal(end, expected):
                raise ValueError("rotation differs from checked sweep")
    return linear, angular
