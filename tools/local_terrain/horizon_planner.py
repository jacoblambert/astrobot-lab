"""Tentative long route plus separately supported short prefix. No commands/GT.

Unknown in a route is an observation task, not permission to drive. Local
checks retain the full wheel/body geometry and actual-pose support index.
"""

from dataclasses import dataclass
import time

import numpy as np

from local_terrain.cost_planner import CostPlanner
from local_terrain.navigation_cost import navigation_costs


@dataclass(frozen=True)
class HorizonPolicy:
    unknown_route_cost: int = 90
    local_horizon_m: float = 0.5
    stopping_reserve_m: float = 0.02
    minimum_translation_m: float = 0.05
    endpoint_precheck: bool = True

    def __post_init__(self):
        if type(self.endpoint_precheck) is not bool:
            raise ValueError("explicit endpoint precheck policy required")
        if type(self.unknown_route_cost) is not int or not 70 < self.unknown_route_cost < 100:
            raise ValueError("bounded long-route uncertainty cost required")
        values = (self.local_horizon_m, self.stopping_reserve_m, self.minimum_translation_m)
        if not np.isfinite(values).all() or not 0 < self.stopping_reserve_m < self.local_horizon_m <= 1:
            raise ValueError("finite bounded local horizon/stopping reserve required")
        if not 0 < self.minimum_translation_m <= self.local_horizon_m - self.stopping_reserve_m:
            raise ValueError("bounded positive minimum translation required")


class FixedFreeGoals:
    """Observe for a declared duration, then choose once, including missing goals."""

    def __init__(self, delay_s=0.0):
        if not np.isfinite(delay_s) or not 0 <= delay_s <= 30:
            raise ValueError("bounded goal-selection delay required")
        self.delay_ns = round(delay_s * 1e9)
        self.first_ns = self.last_ns = self.selected_ns = None
        self.goals = {f"{d:.1f}m_{a}": None for d in (1.0, 2.0) for a in (0, 90, 180, 270)}
        self.nominal = {}

    def observe(self, planner, stamp):
        if type(stamp) is not int or self.last_ns is not None and stamp <= self.last_ns:
            raise ValueError("ordered integer goal-selection clock required")
        self.last_ns = stamp
        if self.first_ns is None:
            self.first_ns = stamp
        if self.selected_ns is not None or stamp - self.first_ns < self.delay_ns:
            return
        goals, nominal = {}, {}
        for distance in (1.0, 2.0):
            for angle in (0, 90, 180, 270):
                name = f"{distance:.1f}m_{angle}"
                target = planner.local.start[:2, 3] + distance * np.array(
                    [np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))]
                )
                nominal[name] = target.tolist()
                goals[name] = planner.select_goal(target)
        self.goals, self.nominal, self.selected_ns = goals, nominal, stamp


def length(poses):
    return sum(float(np.linalg.norm(np.asarray(b)[:2, 3] - np.asarray(a)[:2, 3])) for a, b in zip(poses, poses[1:]))


def translation_prefix(poses, distance):
    """Keep rotations before the cut, interpolate translations only.

    The lattice separates translation and rotation. Never create an unchecked
    shortcut across a corner or interpolate a translating/rotating arc.
    """
    if not poses or not np.isfinite(distance) or distance < 0:
        raise ValueError("nonempty path and finite nonnegative prefix required")
    result = [np.asarray(poses[0]).copy()]
    remaining = float(distance)
    for first, second in zip(poses, poses[1:]):
        a, b = np.asarray(first), np.asarray(second)
        d = float(np.linalg.norm(b[:2, 3] - a[:2, 3]))
        if d > 1e-12 and not np.allclose(a[:3, :3], b[:3, :3], atol=1e-12, rtol=0):
            raise ValueError("prefix expects separate translation/rotation edges")
        if remaining <= 1e-12:
            break
        if d > remaining:
            end = a.copy()
            end[:3, 3] += (b[:3, 3] - a[:3, 3]) * (remaining / d)
            result.append(end)
            break
        result.append(b.copy())
        remaining -= d
    return result


class HorizonPlanner:
    def __init__(self, layers, record, start, patches, body, *, support_index=None, policy=HorizonPolicy()):
        self.layers = np.asarray(layers).copy()
        if self.layers.ndim != 3 or self.layers.shape[2] != 6:
            raise ValueError("six-channel terrain required")
        if not np.isin(self.layers[:, :, 0], (-1, 0, 20, 100)).all():
            raise ValueError("valid terrain classes required")
        self.policy = policy
        self.local_costs, self.isolated_mask = navigation_costs(self.layers, resolution=record["resolution"])
        self.missing = (self.layers[:, :, 0] == -1) & np.isin(self.layers[:, :, 1], (0, 4, 9, 12))
        self.route_costs = self.local_costs.copy()
        self.route_costs[self.missing] = np.maximum(self.route_costs[self.missing], policy.unknown_route_cost)
        args = (record, start, patches, body)
        self.local = CostPlanner(self.local_costs, *args, support_index=support_index, support_eligible=self.missing)
        self.route = CostPlanner(self.route_costs, *args)

    def known_free(self, xy):
        xy = np.asarray(xy, float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            return False
        x, y = np.floor((xy - self.local.origin) / self.local.resolution).astype(int)
        h, w = self.layers.shape[:2]
        return bool(0 <= x < w and 0 <= y < h and self.layers[y, x, 0] in (0, 20))

    def select_goal(self, nominal_xy, *, max_offset_m=0.3):
        """Nearest observed-free cell, deterministic and independent of search/GT."""
        nominal = np.asarray(nominal_xy, float)
        if nominal.shape != (2,) or not np.isfinite(nominal).all() or not 0 < max_offset_m <= 0.5:
            raise ValueError("finite nominal goal and bounded selection radius required")
        yx = np.argwhere(np.isin(self.layers[:, :, 0], (0, 20)))
        xy = self.local.origin + (yx[:, ::-1] + 0.5) * self.local.resolution
        distances = np.linalg.norm(xy - nominal, axis=1)
        if not len(distances) or float(distances.min()) > max_offset_m:
            return None
        return xy[int(np.argmin(distances))].tolist()

    def lattice_goal(self, goal_xy):
        """Choose a known-free lattice endpoint within the original 7.5 cm tolerance.

        Do not let endpoint tolerance land on an unknown neighbour of a free
        goal. Selection uses distance/classes only, never search success.
        The fixed requested goal itself is not changed.
        """
        if goal_xy is None or not self.known_free(goal_xy):
            return None
        goal = np.asarray(goal_xy)
        center = np.rint((goal - self.local.start[:2, 3]) / self.local.resolution).astype(int)
        offsets = np.array([(x, y) for y in (-1, 0, 1) for x in (-1, 0, 1)])
        candidates = self.local.start[:2, 3] + (center + offsets) * self.local.resolution
        distance = np.linalg.norm(candidates - goal, axis=1)
        for i in np.argsort(distance, kind="stable"):
            if distance[i] <= 0.075 and self.known_free(candidates[i]):
                return candidates[i].tolist()
        return None

    def full_local_route(self, goal_xy):
        goal = self.lattice_goal(goal_xy)
        if goal is None:
            return dict(found=False, reason="goal_not_observed_free", poses=[], expanded=0, wall_ms=0.0)
        return self.local.plan(goal, tolerance_m=1e-6)

    def endpoint_headings(self, endpoint):
        """Necessary goal-footprint test, not a reachability certificate.

        Avoid exhausting search when the goal center is free but all eight
        permitted arrival orientations overlap a hazard or the map boundary.
        """
        offset = np.rint((np.asarray(endpoint) - self.route.start[:2, 3]) / self.route.resolution).astype(int)
        return [
            h for h in range(8) if self.route.check(self.route.geometry([self.route.pose((*offset, h))])) is not None
        ]

    def plan(self, goal_xy):
        began = time.perf_counter()
        empty = dict(found=False, reason="goal_not_observed_free", poses=[], expanded=0, wall_ms=0.0)
        endpoint = self.lattice_goal(goal_xy)
        if endpoint is None:
            return dict(
                route=empty,
                prefix=dict(empty),
                checked_poses=[],
                checked_translation_m=0.0,
                executable_translation_m=0.0,
                turn_available=False,
                goal_local_xy=goal_xy,
                goal_reached=False,
                navigation_authority=False,
            )
        if self.policy.endpoint_precheck and not self.endpoint_headings(endpoint):
            route = dict(empty, reason="goal_footprint_unavailable")
        else:
            route = self.route.plan(endpoint, tolerance_m=1e-6)
        route["wall_ms"] = (time.perf_counter() - began) * 1000
        checked = [self.local.start.copy()]
        reason = "no_tentative_route"
        if route["found"]:
            reason = "local_horizon"
            candidate = translation_prefix(route["poses"], self.policy.local_horizon_m)
            for a, b in zip(candidate, candidate[1:]):
                if self.local.check(self.local.geometry([a, b])) is None:
                    reason = "local_support_or_hazard_limit"
                    break
                checked.append(b)
        checked_distance = length(checked)
        usable = max(0.0, checked_distance - self.policy.stopping_reserve_m)
        # Translation reserve is not an angular stopping certificate. Report a
        # supported turn separately; this diagnostic does not authorize it.
        turn = any(not np.allclose(p[:3, :3], checked[0][:3, :3], atol=1e-12, rtol=0) for p in checked)
        found = usable + 1e-12 >= self.policy.minimum_translation_m
        poses = translation_prefix(checked, usable) if found else []
        # Check the actual cut prefix too; never rely on raster/template subset
        # assumptions when turning or an interpolated endpoint changes geometry.
        if any(self.local.check(self.local.geometry([a, b])) is None for a, b in zip(poses, poses[1:])):
            raise ValueError("cut prefix no longer passes original local checks")
        prefix = dict(
            found=found,
            reason="supported_prefix" if found else reason,
            expanded=0,
            poses=[p.tolist() for p in poses],
            wall_ms=(time.perf_counter() - began) * 1000,
            goal_local_xy=list(goal_xy),
        )
        return dict(
            route=route,
            prefix=prefix,
            checked_poses=[p.tolist() for p in checked],
            checked_translation_m=checked_distance,
            executable_translation_m=usable if found else 0.0,
            turn_available=turn,
            goal_local_xy=list(goal_xy),
            goal_reached=False,
            navigation_authority=False,
        )
