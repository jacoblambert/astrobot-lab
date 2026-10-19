"""Bounded cost-aware SE(2) lattice planner for inspection, never commands.

Eight headings, whole-cell translations and swept in-place rotations. Templates
check every body/wheel cell, including diagonal corner contacts. Ground support
is required only under wheels; body occupied/outside remains a veto. Assumed
support is evaluated on exact clipped polygons, not painted into terrain.
No GT or simulator geometry enters search. No dynamic tracking certificate.
"""

import heapq
import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter
from shapely import Polygon, box, difference, intersection, union_all
from shapely.affinity import translate
from scipy.spatial.transform import Rotation

from local_terrain.support_sweep import swept_polygons, touched_cells
from revision_evidence import validated_transform


class CostPlanner:
    def __init__(
        self, costs, record, start, patches, body, *, support_index=None, support_eligible=None, max_expansions=4000
    ):
        self.costs = np.asarray(costs).copy()
        if self.costs.ndim != 2 or not np.isin(self.costs, range(-1, 101)).all():
            raise ValueError("valid finite planner costs required")
        self.start = validated_transform(start).copy()
        self.origin = np.asarray(record["origin"], float)
        self.resolution = float(record["resolution"])
        if (
            self.resolution <= 0
            or not np.isfinite(self.resolution)
            or self.origin.shape != (2,)
            or not np.isfinite(self.origin).all()
        ):
            raise ValueError("valid planner grid required")
        if type(max_expansions) is not int or not 1 <= max_expansions <= 30000:
            raise ValueError("bounded search required")
        self.max_expansions = max_expansions
        self.index = support_index
        self.support_eligible = (
            np.zeros(self.costs.shape, bool) if support_eligible is None else np.asarray(support_eligible, bool)
        )
        if self.support_eligible.shape != self.costs.shape:
            raise ValueError("support eligibility/grid mismatch")
        if self.index is not None and (
            self.index.resolution != self.resolution or not np.array_equal(self.index.origin, self.origin)
        ):
            raise ValueError("support index/grid mismatch")
        self.patches, self.body = tuple(patches), body
        self.yaw = math.atan2(self.start[1, 0], self.start[0, 0])
        self.initial_heading = round(self.yaw / (math.pi / 4)) % 8
        self.templates, self.edges = {}, {}
        self.cell0 = np.floor((self.start[:2, 3] - self.origin) / self.resolution).astype(int)

    def pose(self, state):
        x, y, heading = state
        value = self.start.copy()
        value[:2, 3] += np.array([x, y]) * self.resolution
        value[:3, :3] = Rotation.from_euler("z", heading * math.pi / 4 - self.yaw).as_matrix() @ self.start[:3, :3]
        return value

    def template(self, heading, action):
        key = (heading, action)
        if key not in self.templates:
            dx, dy = np.rint([math.cos(heading * math.pi / 4), math.sin(heading * math.pi / 4)]).astype(int)
            if action in ("forward", "reverse"):
                sign = 1 if action == "forward" else -1
                delta = (sign * dx, sign * dy, heading)
            else:
                delta = (0, 0, (heading + (1 if action == "left" else -1)) % 8)
            poses = [self.pose((0, 0, heading)), self.pose(delta)]
            self.templates[key] = (delta, self.geometry(poses))
        return self.templates[key]

    def geometry(self, poses):
        wheels = list(swept_polygons(poses, self.patches))
        wheel_cells, _ = touched_cells(wheels, self.origin, self.resolution)
        body_cells, _ = touched_cells(swept_polygons(poses, [self.body]), self.origin, self.resolution)
        return wheel_cells, body_cells, union_all([Polygon(p) for p in wheels])

    def check(self, geometry, offset=(0, 0)):
        wheel, body, polygon = geometry
        offset = np.asarray(offset, int)
        wheel, body = wheel + offset, body + offset
        h, w = self.costs.shape
        for cells in (wheel, body):
            x, y = cells.T
            if (x < 0).any() or (y < 0).any() or (x >= w).any() or (y >= h).any():
                return None
            if (self.costs[y, x] == 100).any():
                return None
        x, y = wheel.T
        values = self.costs[y, x]
        unknown = wheel[values == -1]
        if len(unknown):
            shifted = None
            for ij in unknown:
                if not self.support_eligible[ij[1], ij[0]]:
                    return None
                if shifted is None:
                    shifted = translate(polygon, *(offset * self.resolution))
                low = self.origin + ij * self.resolution
                part = intersection(shifted, box(*low, *(low + self.resolution)), grid_size=0)
                # Match the existing exact-support query: touching the boundary
                # of unobserved ground is not a positive unsupported area.
                # Occupied/conflicting cells were already rejected above.
                if part.area <= 1e-12:
                    continue
                cover = None if self.index is None else self.index.unions.get(tuple(ij))
                if cover is None:
                    return None
                if difference(part, cover, grid_size=0).area > 1e-12:
                    return None
        # The maximum footprint cost makes an uncertain wheel crossing more
        # expensive even when it occupies just one cell among many body cells.
        positive = np.maximum(self.costs[body[:, 1], body[:, 0]], 0)
        return float(max(np.max(values, initial=0), np.max(positive, initial=0)))

    def plan(self, goal_xy, *, tolerance_m=0.075):
        started = time.perf_counter()
        goal = np.asarray(goal_xy, float)
        if goal.shape != (2,) or not np.isfinite(goal).all() or not 0 < tolerance_m <= self.resolution:
            raise ValueError("finite goal and bounded tolerance required")
        initial = (0, 0, self.initial_heading)
        connector = self.check(self.geometry([self.start, self.pose(initial)]))
        if connector is None:
            return dict(
                found=False,
                reason="start_or_heading_connector_unavailable",
                expanded=0,
                poses=[],
                wall_ms=(time.perf_counter() - started) * 1000,
            )

        # A holonomic obstacle-aware cost-to-go guides the heading search.
        # It is guidance ONLY: unreachable guide cells fall back to Euclidean
        # distance and never prune an otherwise legal swept-body motion.
        h, w = self.costs.shape
        xs = self.start[0, 3] + (np.arange(w) - self.cell0[0]) * self.resolution
        ys = self.start[1, 3] + (np.arange(h) - self.cell0[1]) * self.resolution
        near_goal = np.hypot(xs[None, :] - goal[0], ys[:, None] - goal[1]) <= tolerance_m
        guide_blocked = np.zeros((h, w), bool)
        if (self.costs == 100).any():
            guide_blocked = distance_transform_edt(self.costs != 100) * self.resolution <= min(
                self.body.half_extent[:2]
            )
        radius = min(self.body.half_extent[:2]) / self.resolution
        n = math.ceil(radius)
        dy, dx = np.mgrid[-n : n + 1, -n : n + 1]
        guide_cost = maximum_filter(
            np.maximum(self.costs, 0), footprint=dx * dx + dy * dy <= radius * radius, mode="constant", cval=0
        )
        guide = np.full((h, w), np.inf)
        guide_queue = []
        for y, x in np.argwhere(near_goal):
            guide[y, x] = 0
            heapq.heappush(guide_queue, (0.0, int(x), int(y)))
        while guide_queue:
            cost, x, y = heapq.heappop(guide_queue)
            if cost != guide[y, x]:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < w and 0 <= ny < h) or guide_blocked[ny, nx]:
                    continue
                value = cost + self.resolution * math.hypot(dx, dy) * (1 + 5 * guide_cost[ny, nx] / 100)
                if value < guide[ny, nx]:
                    guide[ny, nx] = value
                    heapq.heappush(guide_queue, (value, nx, ny))

        def euclidean(state):
            xy = self.start[:2, 3] + np.asarray(state[:2]) * self.resolution
            return max(0.0, float(np.linalg.norm(xy - goal)) - tolerance_m)

        def heuristic(state):
            x, y = self.cell0 + state[:2]
            if 0 <= x < w and 0 <= y < h and np.isfinite(guide[y, x]):
                return max(euclidean(state), guide[y, x])
            return euclidean(state)

        # Weighted A*: bounded inspection search, not an optimal-path claim.
        # A Euclidean lower bound alone otherwise spends the budget exploring
        # cheap space behind the rover before routing around an inflated rock.
        weight = 2.0
        queue = [(weight * heuristic(initial), 0.0, initial)]
        distance, parent = {initial: 0.0}, {}
        expanded = 0
        end = None
        while queue and expanded < self.max_expansions:
            _, cost, state = heapq.heappop(queue)
            if cost != distance[state]:
                continue
            expanded += 1
            if euclidean(state) == 0:
                end = state
                break
            for action in ("forward", "reverse", "left", "right"):
                key = (state, action)
                delta, geometry = self.template(state[2], action)
                following = (state[0] + delta[0], state[1] + delta[1], delta[2])
                if key not in self.edges:
                    self.edges[key] = self.check(geometry, state[:2])
                penalty = self.edges[key]
                if penalty is None:
                    continue
                length = self.resolution * math.hypot(*delta[:2]) if action in ("forward", "reverse") else 0.15
                candidate = cost + length * (1 + 5 * penalty / 100) * (1.2 if action == "reverse" else 1)
                if candidate < distance.get(following, float("inf")):
                    distance[following], parent[following] = candidate, state
                    heapq.heappush(queue, (candidate + weight * heuristic(following), candidate, following))
        states = []
        if end is not None:
            while end != initial:
                states.append(end)
                end = parent[end]
            states.append(initial)
            states.reverse()
        return dict(
            found=bool(states),
            reason="path" if states else "search_budget" if queue else "no_connected_path",
            expanded=expanded,
            heuristic_weight=weight,
            poses=[self.start.tolist(), *[self.pose(s).tolist() for s in states]] if states else [],
            wall_ms=(time.perf_counter() - started) * 1000,
        )
