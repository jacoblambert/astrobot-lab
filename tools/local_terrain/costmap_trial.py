"""Opt-in unknown-permitted 2-D navigation experiment, not production authority.

No GT inputs. Fixed observed-free destination, 40 cm tracking lookahead and
fresh 30 cm hazard sweep. Missing ground is a declared risk, not measured free.
The independent simulator driver retains clearance, lease and stop guards.
"""

import heapq
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from costmap_scenarios import SCENARIOS, PROTOCOL as ROUTE_PROTOCOL, check_position, scenario_spec
from revision_evidence import validated_transform

PROTOCOL = "costmap_unknown_trial_v1"
TOPIC = "/terrain_trial/proposal"


def current_turn_layers(layers, record, current, radius):
    """Navigation-only current-footprint prior; never measured free terrain.

    Only fully contained uncertain cells receive the user-declared precedence.
    Occupied detections always win. Anchor once at the actual query pose, NOT
    at hypothetical path nodes. Requiring turning clearance over the next
    command prevents deliberately entering known narrow/unsupported passages.
    Raw layers, heights, evidence ages and classifications are not mutated.
    """
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("positive finite turning assumption radius required")
    center = np.asarray(current)[:2, 3]
    res, origin = record["resolution"], np.asarray(record["origin"])
    cells = np.argwhere((layers[:, :, 0] == -1) & ~np.isin(layers[:, :, 1], (0, 4, 9, 12)))[:, ::-1]
    low = origin + cells * res
    farthest = np.maximum(np.abs(low - center), np.abs(low + res - center))
    cells = cells[np.linalg.norm(farthest, axis=1) <= radius]
    effective = layers.copy()
    effective[cells[:, 1], cells[:, 0], 1] = 0  # Still UNKNOWN, allowed by declared navigation policy.
    return effective, dict(
        policy="current_footprint_over_uncertainty_occupied_always_veto",
        radius_m=radius,
        center_xy=center.tolist(),
        grid_origin_xy=origin.tolist(),
        grid_resolution_m=res,
        overridden_cells_xy=cells.tolist(),
        terrain_evidence_changed=False,
    )


class CircularClearance:
    """Exact planar segment-to-cell clearance, with unchanged inflated radii.

    Used only to connect an actually clear start out of a conservatively
    blocked grid cell. No cell evidence is cleared or made traversable.
    """

    def __init__(self, layers, record, body_radius, wheel_radius):
        from shapely import box

        if not 0 < wheel_radius <= body_radius:
            raise ValueError("positive ordered clearance radii required")
        self.groups = []
        classes, reasons = layers[:, :, 0], layers[:, :, 1]
        occupied = classes == 100
        self.body_blocked = (
            distance_transform_edt(~occupied) * record["resolution"]
            <= body_radius + math.sqrt(2) * record["resolution"]
            if occupied.any()
            else np.zeros_like(occupied)
        )
        for mask, radius in (
            (classes == 100, body_radius),
            ((classes == -1) & ~np.isin(reasons, (0, 4, 9, 12)), wheel_radius),
        ):
            cells = np.argwhere(mask)[:, ::-1]
            low = np.asarray(record["origin"]) + cells * record["resolution"]
            high = low + record["resolution"]
            self.groups.append((box(low[:, 0], low[:, 1], high[:, 0], high[:, 1]), radius))

    def __call__(self, start, end):
        from shapely import distance, LineString, Point

        line = Point(start) if np.array_equal(start, end) else LineString([start, end])
        return all(not np.any(distance(line, cells) <= radius + 1e-10) for cells, radius in self.groups)

    def permits(self, start, end):
        """Clear, or an escape that never comes closer to any conflict than the start already is.

        The radii are circles about the rover centre, so an in-place rotation (start == end)
        never reduces clearance and is permitted even with a conflict already inside a radius.
        Accepted risk (user decision 2026-10-20): a skid turn can still translate the rover by
        the command drift allowance; the driver's drift and turn-translation limits bound it.
        """
        from shapely import distance, LineString, Point

        if self(start, end):
            return True
        line = Point(start) if np.array_equal(start, end) else LineString([start, end])
        here = Point(start)
        return all(
            not len(cells) or float(np.min(distance(line, cells))) >= float(np.min(distance(here, cells))) - 1e-10
            for cells, _ in self.groups
        )


def routing_grid(
    layers,
    record,
    radius,
    *,
    wheel_radius=None,
    separate_support_conflicts=False,
):
    """Shared, unchanged costs/padding for search and retained-route checks."""
    classes, reasons = layers[:, :, 0], layers[:, :, 1]
    hazard = (classes == 100) | ((classes == -1) & ~np.isin(reasons, (0, 4, 9, 12)))
    dropoff = (classes == -1) & (reasons == 8) if wheel_radius is not None else np.zeros_like(hazard)
    if separate_support_conflicts:
        if wheel_radius is None or not 0 < wheel_radius <= radius:
            raise ValueError("footprint route needs valid body/wheel bounds")
        dropoff = (classes == -1) & hazard
    hazard = hazard & ~dropoff
    res = record["resolution"]
    # Cell-area allowance: EDT measures center distances, not square boundaries.
    # The map edge is lack of observation, not an obstacle in a tentative
    # route. Keep centers inside the map, but do not inflate its boundary.
    # The separate immediate body/wheel sweep still rejects leaving the map.
    distance = distance_transform_edt(~hazard) * res if hazard.any() else np.full(hazard.shape, np.inf)
    blocked = distance <= radius + math.sqrt(2) * res
    costs = np.where(classes == -1, 30.0, np.maximum(classes, 0))
    costs += 40 * np.maximum(0, 1 - np.maximum(0, distance - radius) / 0.5)
    if dropoff.any():
        # Support conflicts use the same wheel/body distinction as PreparedMotion.
        # Circumscribed wheel extent covers every heading; the immediate sweep
        # still tests the actual patches against every explicit conflict.
        drop_distance = distance_transform_edt(~dropoff) * res
        blocked |= drop_distance <= wheel_radius + math.sqrt(2) * res
        costs += 40 * np.maximum(0, 1 - np.maximum(0, drop_distance - wheel_radius) / 0.5)
    return blocked, costs


def grid_route(
    layers,
    record,
    start_xy,
    goal_xy,
    radius,
    *,
    partial=False,
    wheel_radius=None,
    separate_support_conflicts=False,
    start_clearance=None,
):
    """Tentative center route; only the exact immediate sweep grants motion."""
    blocked, costs = routing_grid(
        layers,
        record,
        radius,
        wheel_radius=wheel_radius,
        separate_support_conflicts=separate_support_conflicts,
    )
    classes = layers[:, :, 0]
    res = record["resolution"]
    origin = np.asarray(record["origin"])
    cell = lambda xy: tuple(np.floor((np.asarray(xy) - origin) / res).astype(int))
    start, goal = cell(start_xy), cell(goal_xy)
    h, w = classes.shape

    def allowed(p):
        x, y = p
        return 0 <= x < w and 0 <= y < h and not blocked[y, x]

    exact_start = False
    if (
        start_clearance is not None
        and 0 <= start[0] < w
        and 0 <= start[1] < h
        and blocked[start[1], start[0]]
        and start_clearance(start_xy, start_xy)
    ):
        blocked = blocked.copy()
        blocked[start[1], start[0]] = False
        exact_start = True
    if not allowed(start) or not partial and not allowed(goal):
        return [], blocked
    queue, scores, parent = [(0.0, 0.0, start)], {start: 0.0}, {}
    best, best_distance = start, math.dist(start, goal)

    def reconstruct(p, complete):
        path = [p]
        while p != start:
            p = parent[p]
            path.append(p)
        cells = path[::-1][1:] if exact_start else path[::-1]
        xy = [origin + (np.asarray(p) + 0.5) * res for p in cells]
        return [np.asarray(start_xy), *xy, *([np.asarray(goal_xy)] if complete else [])], blocked

    while queue:
        _, cost, p = heapq.heappop(queue)
        if cost != scores[p]:
            continue
        if math.dist(p, goal) < best_distance:
            best, best_distance = p, math.dist(p, goal)
        if p == goal:
            if exact_start and p == start and not start_clearance(start_xy, goal_xy):
                return [], blocked
            return reconstruct(p, True)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            q = (p[0] + dx, p[1] + dy)
            if not allowed(q) or not allowed((p[0] + dx, p[1])) or not allowed((p[0], p[1] + dy)):
                continue
            if exact_start and p == start and not start_clearance(start_xy, origin + (np.asarray(q) + 0.5) * res):
                continue
            value = cost + math.hypot(dx, dy) * (1 + costs[q[1], q[0]] / 100)
            if value < scores.get(q, float("inf")):
                scores[q], parent[q] = value, p
                heapq.heappush(queue, (value + math.hypot(q[0] - goal[0], q[1] - goal[1]), value, q))
    # A rolling map need not contain the mission goal. Route toward the closest
    # reachable cell, but never call this temporary frontier a reached goal.
    if partial and best != start and best_distance < math.dist(start, goal) - 1:
        return reconstruct(best, False)
    return [], blocked


def visible(a, b, blocked, record, *, clearance=None, allow_outside=False):
    """Closed-square segment test, including corner contacts; no sampled gaps."""
    if clearance is not None and not clearance(a, b):
        return False
    origin, res = np.asarray(record["origin"]), record["resolution"]
    a, b = (np.asarray(a) - origin) / res, (np.asarray(b) - origin) / res
    low, high = np.floor(np.minimum(a, b) - 1e-10).astype(int), np.floor(np.maximum(a, b) + 1e-10).astype(int)
    if not allow_outside and ((low < 0).any() or high[0] >= blocked.shape[1] or high[1] >= blocked.shape[0]):
        return False
    if clearance is not None:
        # Exact swept circles replace support-cell rounding only. Retain the
        # original occupied-cell padding as well, including tracking headroom.
        blocked = clearance.body_blocked
        start = np.floor(a).astype(int)
        end = np.floor(b).astype(int)
        h, w = blocked.shape
        if 0 <= start[0] < w and 0 <= start[1] < h and blocked[start[1], start[0]]:
            # The robot can already sit inside conservative cell padding while
            # its exact circular envelope is clear. Permit only a connector
            # OUT to a padded-free cell; every other padded cell stays blocked.
            # clearance(a,b) above still checks the complete original radius.
            if not (0 <= end[0] < w and 0 <= end[1] < h) or blocked[end[1], end[0]]:
                return False
            blocked = blocked.copy()
            blocked[start[1], start[0]] = False
    # A retained tentative route may extend beyond the rolling map. Check all
    # currently observed hazards, never treat missing map as a new obstacle.
    # Immediate motion still requires the original in-map PreparedMotion sweep.
    low = np.maximum(low, 0)
    high = np.minimum(high, np.array(blocked.shape[::-1]) - 1)
    if (low > high).any():
        return True
    for y, x in np.argwhere(blocked[low[1] : high[1] + 1, low[0] : high[0] + 1]):
        lower = np.array([x, y]) + low
        t0, t1 = 0.0, 1.0
        for axis in (0, 1):
            d = b[axis] - a[axis]
            if abs(d) < 1e-12:
                if not lower[axis] - 1e-10 <= a[axis] <= lower[axis] + 1 + 1e-10:
                    t0, t1 = 1, 0
                    break
            else:
                first, last = sorted(((lower[axis] - a[axis]) / d, (lower[axis] + 1 - a[axis]) / d))
                t0, t1 = max(t0, first), min(t1, last)
        if t0 <= t1 + 1e-10:
            return False
    return True


class CostmapTrial:
    def __init__(self, capture, goal_distance=1.0, *, scenario=None):
        if scenario is not None and scenario not in SCENARIOS:
            raise ValueError("undeclared navigation scenario")
        if scenario is None and goal_distance not in (1.0, 2.0):
            raise ValueError("declared 1/2 m trial only")
        self.scenario = scenario
        self.spec = None if scenario is None else scenario_spec(scenario)
        self.goals, self.goal_index, self.reached = [], 0, []
        self.capture = capture
        self.goal_distance = self.spec["goal_distance_m"] if self.spec else goal_distance
        self.goal = self.origin = self.identity = self.reference = None
        self.first_ns = self.last_ns = self.refused_since = None
        self.sequence = self.steps = 0
        self.terminal = None
        self.turning = False
        self.turn_origin = None
        self.turn_id = 0
        self.recentering = False
        self.observation_started_ns = None
        self.observation_origin = None
        self.route_points = []
        self.route_goal = None
        self.route_generation = 0

    def committed_route(self, layers, record, current, body_radius, wheel_radius, clearance):
        """Keep the same polyline/steering endpoint until blocked or consumed.

        Costs becoming cheaper elsewhere do not trigger a search. Every remaining
        segment, including the connector from the actual pose, is rechecked.
        Points live in the fixed local frame, not rolling-grid cell coordinates.
        """
        blocked, _ = routing_grid(
            layers, record, body_radius, wheel_radius=wheel_radius, separate_support_conflicts=True
        )
        check = lambda a, b: visible(a, b, blocked, record, clearance=clearance, allow_outside=True)
        action = "retained" if self.route_points else "searching"
        if self.route_goal is None or not np.array_equal(self.route_goal, self.goal):
            self.route_points = []
            self.route_goal = self.goal.copy()
            action = "goal_changed"
        # Advance along the SAME route. Never steer back to an already reached
        # grid center; validate the new connector before cutting a corner.
        while self.route_points and np.linalg.norm(current - self.route_points[0]) <= 0.1:
            if len(self.route_points) > 1 and not check(current, self.route_points[1]):
                break
            self.route_points.pop(0)
            action = "advanced" if self.route_points else "local_route_consumed"
        remaining = [current, *self.route_points]
        if self.route_points and not all(check(a, b) for a, b in zip(remaining, remaining[1:])):
            self.route_points = []
            action = "blocked"
        if not self.route_points:
            raw, blocked = grid_route(
                layers,
                record,
                current,
                self.goal,
                body_radius,
                partial=True,
                wheel_radius=wheel_radius,
                separate_support_conflicts=True,
                start_clearance=clearance,
            )
            # Freeze the shortcut geometry once, rather than choosing a new
            # farthest-visible point (and possibly a different side) every tick.
            points = []
            index = 0
            while index + 1 < len(raw):
                following = next(
                    (
                        j
                        for j in range(len(raw) - 1, index, -1)
                        if visible(raw[index], raw[j], blocked, record, clearance=clearance)
                    ),
                    None,
                )
                if following is None:
                    points = []
                    break
                points.append(np.asarray(raw[following]).copy())
                index = following
            if points:
                self.route_points = points
                self.route_generation += 1
            elif action == "retained":
                action = "searching"
        return ([current, *self.route_points] if self.route_points else []), blocked, action

    def turn_command(self, current, error):
        """Track heading from the actual pose, not a centimeter-scale pivot."""
        end = np.eye(4)
        if self.spec and self.turn_origin is None:
            self.turn_origin = current[:2, 3].copy()
            self.turn_id += 1
        self.recentering = False
        angle = math.copysign(math.radians(5), error)
        end[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        linear, angular = 0.0, math.copysign(self.spec["angular_speed"] if self.spec else 0.04, error)
        return end, linear, angular

    def update(self, advisory, stamp):
        # Simulator-side validation must not load mapper-only GEOS/Shapely.
        from local_terrain.contact_query import PreparedMotion

        context = advisory.context
        if context is None or context.query_ns != stamp:
            raise ValueError("trial requires fresh current pose")
        current = context.local_from_base
        if self.observation_origin is not None and np.linalg.norm(current[:2, 3] - self.observation_origin) >= 0.3:
            self.observation_started_ns = self.observation_origin = None
        reference = np.asarray(advisory.record["local_from_map"])
        identity = (context.session, context.epoch)
        if self.first_ns is None:
            self.first_ns, self.origin = stamp, current.copy()
            self.reference, self.identity = reference.copy(), identity
        if identity != self.identity or not np.array_equal(reference, self.reference):
            raise ValueError("trial frame/session changed")
        if self.last_ns is not None and stamp <= self.last_ns:
            raise ValueError("trial clock did not advance")
        self.last_ns = stamp
        if self.spec:
            check_position(self.origin, current, self.spec["position_bounds_initial_xy"])
        elif np.linalg.norm(current[:2, 3] - self.origin[:2, 3]) > self.goal_distance + 0.5:
            raise ValueError("trial estimated-position envelope")
        layers, record = advisory.layers, advisory.record
        turn_assumption = None
        if self.spec:
            layers, turn_assumption = current_turn_layers(
                layers, record, current, self.spec["current_turn_assumption_radius_m"]
            )
        if self.goal is None and stamp - self.first_ns >= 3_000_000_000:
            if self.spec:
                yaw = math.atan2(self.origin[1, 0], self.origin[0, 0])
                rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
                self.goals = [self.origin[:2, 3] + rotation @ p for p in self.spec["goals_initial_xy"]]
                self.goal = self.goals[0]
            else:
                nominal = self.origin[:2, 3] + self.goal_distance * self.origin[:2, 0]
                yx = np.argwhere(np.isin(layers[:, :, 0], (0, 20)))
                xy = np.asarray(record["origin"]) + (yx[:, ::-1] + 0.5) * record["resolution"]
                distances = np.linalg.norm(xy - nominal, axis=1)
                if len(distances) and distances.min() <= 0.3:
                    self.goal = xy[int(np.argmin(distances))]
                else:
                    self.terminal = "no_observed_goal"
        remaining = None if self.goal is None else float(np.linalg.norm(self.goal - current[:2, 3]))
        time_limit = self.spec["maximum_trial_sim_s"] if self.spec else 120
        if not self.terminal and stamp - self.first_ns > time_limit * 1e9:
            self.terminal = "time_limit"
        if not self.terminal and remaining is not None and remaining <= 0.10:
            if self.spec:
                self.reached.append(dict(index=self.goal_index, query_ns=stamp, remaining_m=remaining))
                if self.goal_index + 1 < len(self.goals):
                    self.goal_index += 1
                    self.goal = self.goals[self.goal_index]
                    remaining = float(np.linalg.norm(self.goal - current[:2, 3]))
                    self.turning, self.refused_since = False, None
                    self.turn_origin, self.recentering = None, False
                else:
                    self.terminal = "estimated_goal_reached"
            else:
                self.terminal = "estimated_goal_reached"
        linear = angular = 0.0
        relative, path = [np.eye(4)], []
        decision = dict(state="available", reason="observing")
        target = None
        observing_for_route = False
        route_action = "idle"
        if self.goal is not None and not self.terminal:
            body_radius = math.hypot(*advisory.body.half_extent[:2])
            wheel_radius = max(np.linalg.norm(p.center[:2]) + math.hypot(*p.half_extent[:2]) for p in advisory.patches)
            if self.spec:
                # Full circles enclose any heading plus bounded XY slip during
                # one fresh command. Keep the pre-existing inflated body guard;
                # this allowance is additional, not a smaller obstacle margin.
                body_radius += self.spec["turn_command_drift_allowance_m"]
                wheel_radius += self.spec["turn_command_drift_allowance_m"]
            clearance = CircularClearance(layers, record, body_radius, wheel_radius) if self.spec else None

            def motion_decision(relative):
                if (
                    self.spec
                    and relative[-1][0, 3] == 0
                    and self.turn_origin is not None
                    and np.linalg.norm(current[:2, 3] - self.turn_origin) > self.spec["turn_translation_limit_m"]
                ):
                    return dict(state="blocked", reason="turn_translation_limit")
                sweep = PreparedMotion(layers, record, context.poses(relative), advisory.patches, advisory.body)
                checked = sweep.terminal or (
                    dict(state="blocked", reason="wheel_terrain_conflict")
                    if sweep.conflicts
                    else dict(
                        state="available", reason="hazards_clear_unknown_allowed", unknown_wheel_cells=len(sweep.parts)
                    )
                )
                if self.spec and checked["state"] == "available":
                    if not clearance.permits(current[:2, 3], (current @ relative[-1])[:2, 3]):
                        return dict(state="blocked", reason="would_lose_turning_clearance")
                return checked

            if self.spec:
                path, blocked, route_action = self.committed_route(
                    layers, record, current[:2, 3], body_radius, wheel_radius, clearance
                )
            else:
                path, blocked = grid_route(
                    layers,
                    record,
                    current[:2, 3],
                    self.goal,
                    body_radius,
                    partial=self.spec is not None,
                    wheel_radius=wheel_radius if self.spec else None,
                    separate_support_conflicts=self.spec is not None,
                    start_clearance=clearance,
                )
            if not path:
                decision = dict(state="blocked", reason="no_costmap_route")
            else:
                # Shortcut only across the same inflated grid. Target is for
                # steering, never a stop/arrival waypoint or motion authority.
                far = (
                    path[1]
                    if self.spec
                    else next(
                        (
                            p
                            for p in reversed(path[1:])
                            if visible(current[:2, 3], p, blocked, record, clearance=clearance)
                        ),
                        None,
                    )
                )
                if far is None:
                    decision = dict(state="blocked", reason="no_visible_route_segment")
                else:
                    delta = far - current[:2, 3]
                    target = current[:2, 3] + delta * min(1.0, 0.4 / max(np.linalg.norm(delta), 1e-12))
                    error = math.atan2(delta[1], delta[0]) - math.atan2(current[1, 0], current[0, 0])
                    error = (error + math.pi) % (2 * math.pi) - math.pi
                    self.turning = abs(error) > (0.06 if self.turning else 0.12)
                    end = np.eye(4)
                    if self.turning:
                        end, linear, angular = self.turn_command(current, error)
                    else:
                        end[0, 3], linear = 0.32, 0.05
                    relative.append(end)
                    decision = motion_decision(relative)
                    if self.spec and decision["state"] != "available":
                        # A clear 2-D route is not proof that the actual 3-D
                        # command fits. Do not repeatedly recommit a route whose
                        # immediate command has failed the fresh sweep.
                        self.route_points = []
                        route_action = "immediate_sweep_blocked"
                    if linear > 0 and decision["state"] == "available":
                        self.turn_origin, self.recentering = None, False
            if self.spec and decision["state"] != "available":
                if self.observation_started_ns is None:
                    self.observation_started_ns = stamp
                    self.observation_origin = current[:2, 3].copy()
                if stamp - self.observation_started_ns < self.spec["observation_turn_budget_s"] * 1e9:
                    # A goal route is not required to observe from a clear current
                    # footprint. Budget includes waits/transient route availability;
                    # only actual 30 cm translation earns a new observation episode.
                    self.turning, observing_for_route = True, True
                    end, linear, angular = self.turn_command(current, -1.0)
                    relative = [np.eye(4), end]
                    decision = motion_decision(relative)
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
        value = dict(
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
            goal_local_xy=None if self.goal is None else self.goal.tolist(),
            remaining_goal_m=remaining,
            completed_steps=0,
            step_replans=0,
            route_xy=[p.tolist() for p in path],
            lookahead_xy=None if target is None else target.tolist(),
            goal_distance_m=self.goal_distance,
            unknown_policy="finite_cost_no_positive_ground_required",
            navigation_authority=False,
            experiment_execution=True,
            stopping_allowance_m=0.02,
            stopping_allowance_calibrated=False,
        )
        if self.spec:
            value.update(
                protocol=ROUTE_PROTOCOL,
                scenario=self.scenario,
                goal_index=self.goal_index,
                goals_local_xy=[p.tolist() for p in self.goals],
                reached_goals=list(self.reached),
                home_local_xy=self.origin[:2, 3].tolist(),
                turn_id=self.turn_id if self.turn_origin is not None else None,
                turn_origin_xy=None if self.turn_origin is None else self.turn_origin.tolist(),
                turn_recentering=self.recentering,
                turn_control_policy=self.spec["turn_control_policy"],
                turn_translation_limit_m=self.spec["turn_translation_limit_m"],
                turn_command_drift_allowance_m=self.spec["turn_command_drift_allowance_m"],
                current_turn_assumption=turn_assumption,
                observing_for_route=observing_for_route,
                observation_started_ns=self.observation_started_ns,
                route_policy=self.spec["route_policy"],
                route_action=route_action,
                route_generation=self.route_generation,
                route_waypoints_xy=[p.tolist() for p in self.route_points],
                route_goal_xy=None if self.route_goal is None else self.route_goal.tolist(),
            )
        return value


def validate_proposal(value, capture, identity, now_ns, *, previous_sequence=0, scenario=None):
    """Independent driver-side bounds; never accept a legacy protocol as this one."""
    protocol = PROTOCOL if scenario is None else ROUTE_PROTOCOL
    if not isinstance(value, dict) or value.get("schema") != 1 or value.get("protocol") != protocol:
        raise ValueError("invalid costmap trial proposal")
    if scenario is not None and (scenario not in SCENARIOS or value.get("scenario") != scenario):
        raise ValueError("trial route does not match declared scenario")
    if value.get("capture") != capture or (value.get("session"), value.get("epoch")) != tuple(identity[:2]):
        raise ValueError("trial capture/session mismatch")
    if value.get("valid") is not True:
        raise ValueError("invalidated trial")
    seq, stamp = value["sequence"], value["query_ns"]
    if type(seq) is not int or seq <= previous_sequence or type(stamp) is not int:
        raise ValueError("trial sequence/timestamp")
    if not 0 <= now_ns - stamp <= 200_000_000:
        raise ValueError("trial stale/future proposal")
    if not 0 <= stamp - value["pose_ns"] <= 200_000_000 or not 0 <= stamp - value["evidence_ns"] <= 500_000_000:
        raise ValueError("trial stale pose/evidence")
    linear, angular = value["linear"], value["angular"]
    angular_limit = 0.04 if scenario is None else scenario_spec(scenario)["angular_speed"]
    allowed_linear = (0.0, 0.05)
    if linear not in allowed_linear or angular not in (-angular_limit, 0.0, angular_limit) or linear and angular:
        raise ValueError("trial command bounds / unchecked arc")
    if scenario is not None:
        assumption = value.get("current_turn_assumption", {})
        spec = scenario_spec(scenario)
        if value.get("route_policy") != spec["route_policy"]:
            raise ValueError("trial route retention policy mismatch")
        if (
            any(
                value.get(k) != spec[k]
                for k in ("turn_control_policy", "turn_translation_limit_m", "turn_command_drift_allowance_m")
            )
            or value.get("turn_recentering") is not False
        ):
            raise ValueError("trial drifting-turn policy mismatch")
        if (
            assumption.get("policy") != spec["turn_space_policy"]
            or assumption.get("radius_m") != spec["current_turn_assumption_radius_m"]
            or assumption.get("terrain_evidence_changed") is not False
            or not np.array_equal(assumption.get("center_xy"), np.asarray(value["local_from_base"])[:2, 3])
        ):
            raise ValueError("trial current-footprint assumption mismatch")
        observing, observation_start = value.get("observing_for_route"), value.get("observation_started_ns")
        if (
            type(observing) is not bool
            or observation_start is not None
            and (type(observation_start) is not int or not 0 <= observation_start <= stamp)
        ):
            raise ValueError("trial observation metadata")
        if observing and (
            observation_start is None
            or stamp - observation_start >= spec["observation_turn_budget_s"] * 1e9
            or angular > 0
            or linear
            and not value.get("turn_recentering")
        ):
            raise ValueError("trial unchecked observation motion")
        turn_id, origin, recenter = value.get("turn_id"), value.get("turn_origin_xy"), value.get("turn_recentering")
        if type(recenter) is not bool or (turn_id is None) != (origin is None):
            raise ValueError("trial turn metadata")
        if turn_id is not None and (
            type(turn_id) is not int or turn_id <= 0 or np.shape(origin) != (2,) or not np.isfinite(origin).all()
        ):
            raise ValueError("trial turn identity / anchor")
        if (angular or recenter) and turn_id is None or recenter and angular or linear < 0 and not recenter:
            raise ValueError("trial unchecked turn correction")
    if value["terminal"] not in (None, "no_observed_goal", "terrain_refused", "estimated_goal_reached", "time_limit"):
        raise ValueError("undeclared trial completion")
    if (linear or angular) and (value["terminal"] or value["decision"]["state"] != "available"):
        raise ValueError("trial motion without hazard check")
    validated_transform(value["local_from_base"])
    validated_transform(value["local_from_map"])
    poses = value["relative_poses"]
    if len(poses) not in (1, 2) or not np.array_equal(poses[0], np.eye(4)):
        raise ValueError("trial relative path")
    for pose in poses:
        validated_transform(pose)
    if linear or angular:
        expected = np.eye(4)
        if linear:
            expected[0, 3] = math.copysign(0.32, linear)
        else:
            angle = math.copysign(math.radians(5), angular)
            expected[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        if len(poses) != 2 or not np.array_equal(poses[-1], expected):
            raise ValueError("command differs from checked sweep")
    return linear, angular
