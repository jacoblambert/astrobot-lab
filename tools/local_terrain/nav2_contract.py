"""Nav2 costmap semantics for terrain costs: an offline evaluator, never a controller.

Mirrors the installed configuration (astrobot_launch/config/nav2_phase3_bev.yaml):
non-trinary StaticLayer with lethal threshold 100, InflationLayer with
inflate_unknown false, the padded rectangular footprint, SmacPlanner2D centre-cell
checks (allow_unknown) and MPPI CostCritic with consider_footprint and
track_unknown_space. tools/nav2_shadow is the installed-Nav2 oracle that
`inflate` and `swept_area` are checked against (tools/test/test_nav2_contract.py).

Three acceptance semantics per motion primitive:
  area        no lethal or unknown cell under the swept footprint (nav2_shadow probe)
  controller  no lethal cell under the swept footprint; unknown tolerated (MPPI with
              track_unknown_space)
  planner     every centre-line cell below INSCRIBED, unknown only if allow_unknown

`clear_footprint` is a publisher-side requirement, not Nav2 behaviour: the
rover's own LiDAR blind disc is unknown, and a StaticLayer start cell of
NO_INFORMATION fails SmacPlanner2D with allow_unknown false. Unknown under the
unpadded body is the ground the rover stands on; occupied cells stay occupied.
"""

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt

NO_INFORMATION, LETHAL, INSCRIBED = 255, 254, 253
DEFAULT_CONFIG = (Path(__file__).resolve().parents[2]
                  / "astrobot-lab/src/astrobot_launch/config/nav2_phase3_bev.yaml")


@dataclass(frozen=True)
class Nav2Config:
    half_length: float = 0.595  # padded footprint half extents
    half_width: float = 0.435
    padding: float = 0.10
    inflation_radius: float = 0.75
    cost_scaling_factor: float = 2.0
    lethal_threshold: int = 100
    planner_allow_unknown: bool = False

    @property
    def inscribed(self):
        return min(self.half_length, self.half_width)

    @property
    def circumscribed(self):
        return math.hypot(self.half_length, self.half_width)

    @classmethod
    def from_yaml(cls, path=DEFAULT_CONFIG, costmap="local_costmap"):
        import yaml

        doc = yaml.safe_load(Path(path).read_text())
        params = doc[costmap][costmap]["ros__parameters"]
        corners = np.asarray(yaml.safe_load(params["footprint"]), dtype=float)
        pad = float(params.get("footprint_padding", 0.0))
        if corners.shape != (4, 2) or not np.allclose(np.abs(corners), np.abs(corners[0])):
            raise ValueError("centred rectangular footprint required")
        static = params["static_layer"]
        if static.get("trinary_costmap", True) or not params.get("track_unknown_space", False):
            raise ValueError("contract models a non-trinary, unknown-tracking costmap")
        inflation = params["inflation_layer"]
        planner = doc["planner_server"]["ros__parameters"]["GridBased"]
        return cls(
            half_length=float(abs(corners[0, 0])) + pad,
            half_width=float(abs(corners[0, 1])) + pad,
            padding=pad,
            inflation_radius=float(inflation["inflation_radius"]),
            cost_scaling_factor=float(inflation["cost_scaling_factor"]),
            lethal_threshold=int(static.get("lethal_cost_threshold", 100)),
            planner_allow_unknown=bool(planner.get("allow_unknown", True)),
        )


def static_costs(grid, config=Nav2Config()):
    """OccupancyGrid values [-1, 0..100] -> Nav2 master costs (StaticLayer::interpretValue)."""
    grid = np.asarray(grid)
    if not np.isin(grid, np.arange(-1, 101)).all():
        raise ValueError("occupancy values must lie in -1..100")
    out = np.floor(grid.astype(float) / config.lethal_threshold * LETHAL).astype(np.uint8)
    out[grid >= config.lethal_threshold] = LETHAL
    out[grid == -1] = NO_INFORMATION
    return out


def inflate(costs, resolution, config=Nav2Config()):
    """InflationLayer::updateCosts with inflate_unknown false (cell-centre distances)."""
    costs = np.asarray(costs, dtype=np.uint8)
    lethal = costs == LETHAL
    if not lethal.any():
        return costs.copy()
    cells = distance_transform_edt(~lethal)
    metres = cells * resolution
    factor = np.exp(-config.cost_scaling_factor * (metres - config.inscribed))
    cost = np.where(metres <= config.inscribed, INSCRIBED, np.floor((INSCRIBED - 1) * factor)).astype(np.uint8)
    cost[lethal] = LETHAL
    cost[cells > math.ceil(config.inflation_radius / resolution)] = 0
    out = np.maximum(costs, cost)
    unknown = costs == NO_INFORMATION
    out[unknown] = np.where(cost[unknown] >= INSCRIBED, cost[unknown], NO_INFORMATION)
    return out


def master_costs(grid, resolution, config=Nav2Config()):
    return inflate(static_costs(grid, config), resolution, config)


class Footprint:
    """Swept padded-rectangle overlap with grid squares (SAT, as footprint_area.hpp)."""

    STEP = 0.025  # metres of body-point travel per interval, as footprint_swept_area_clear

    def __init__(self, shape, origin, resolution, config=Nav2Config()):
        self.res = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.shape = shape
        rows, cols = np.indices(shape)
        # square lower corners in grid-local metres, row-major (layers[row, col])
        self.lower = np.stack([self.origin[0] + cols.ravel() * self.res, self.origin[1] + rows.ravel() * self.res], 1)
        self.centre = self.lower + self.res / 2
        self.config = config

    def overlap(self, x0, y0, yaw0, x1, y1, yaw1):
        """Flat mask of grid squares touched by the swept footprint, and whether it left the grid."""
        c = self.config
        travel = math.hypot(x1 - x0, y1 - y0) + c.circumscribed * abs(yaw1 - yaw0)
        intervals = max(1, math.ceil(travel / self.STEP))
        margin = travel / (2 * intervals)
        t = (np.arange(intervals) + 0.5) / intervals
        xs, ys, yaws = x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw0 + t * (yaw1 - yaw0)
        reach = c.circumscribed + margin + self.res
        lo = np.array([min(x0, x1), min(y0, y1)]) - reach
        hi = np.array([max(x0, x1), max(y0, y1)]) + reach
        cand = np.flatnonzero(((self.centre >= lo) & (self.centre <= hi)).all(1))
        half = self.res / 2 + margin
        d = self.centre[cand][None, :, :] - np.stack([xs, ys], 1)[:, None, :]  # (S, C, 2)
        cs, sn = np.cos(yaws)[:, None], np.sin(yaws)[:, None]
        # square (world) axes against the rectangle's world AABB
        ex = c.half_length * np.abs(cs) + c.half_width * np.abs(sn)
        ey = c.half_length * np.abs(sn) + c.half_width * np.abs(cs)
        # any sample whose margin-expanded AABB leaves the grid is rejected (out-of-map = hazard)
        grid_lo, grid_hi = self.origin, self.origin + np.array(self.shape[::-1]) * self.res
        outside = bool((xs - ex[:, 0] - margin < grid_lo[0]).any() or (ys - ey[:, 0] - margin < grid_lo[1]).any()
                       or (xs + ex[:, 0] + margin >= grid_hi[0]).any() or (ys + ey[:, 0] + margin >= grid_hi[1]).any())
        hit = (np.abs(d[..., 0]) <= ex + half + 1e-12) & (np.abs(d[..., 1]) <= ey + half + 1e-12)
        # rectangle axes against the (margin-expanded) square
        u = d[..., 0] * cs + d[..., 1] * sn
        v = -d[..., 0] * sn + d[..., 1] * cs
        proj = half * (np.abs(cs) + np.abs(sn))
        hit &= (np.abs(u) <= c.half_length + proj + 1e-12) & (np.abs(v) <= c.half_width + proj + 1e-12)
        mask = np.zeros(len(self.centre), bool)
        mask[cand[hit.any(0)]] = True
        return mask, outside

    def line_cells(self, x0, y0, x1, y1):
        """Flat indices of cells visited by a centre-line path (planner semantics)."""
        n = max(2, math.ceil(math.hypot(x1 - x0, y1 - y0) / (self.res / 4)) + 1)
        pts = np.stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n)], 1)
        ij = np.floor((pts - self.origin) / self.res).astype(int)
        valid = (ij >= 0).all(1) & (ij[:, 0] < self.shape[1]) & (ij[:, 1] < self.shape[0])
        return np.unique(ij[valid, 1] * self.shape[1] + ij[valid, 0]), bool(not valid.all())


def clear_footprint(grid, footprint, pose):
    """Unknown cells whose centre lies under the unpadded body become measured-free (0)."""
    c = footprint.config
    x, y, yaw = pose
    d = footprint.centre - (x, y)
    u = d[:, 0] * math.cos(yaw) + d[:, 1] * math.sin(yaw)
    v = -d[:, 0] * math.sin(yaw) + d[:, 1] * math.cos(yaw)
    under = (np.abs(u) <= c.half_length - c.padding) & (np.abs(v) <= c.half_width - c.padding)
    out = np.array(grid, copy=True)
    flat = out.reshape(-1)
    flat[under & (flat == -1)] = 0
    return out


def grid_pose(record):
    """Rover (x, y, yaw) in grid-corner coordinates of a terrain projection record."""
    local_from_base = np.asarray(record["local_from_base"], dtype=float)
    xy = local_from_base[:2, 3] - np.asarray(record["origin"], dtype=float)
    return (float(xy[0]), float(xy[1]), float(math.atan2(local_from_base[1, 0], local_from_base[0, 0])))


def planner_costs(layers, record, policy, footprint, *, resolution=0.1):
    """The planner costmap the publisher sends Nav2: gap policy, then the rover's own footprint.

    One function for the live publisher and the audit scorer, so the scored grid is the
    published grid. Returns (OccupancyGrid values, low-confidence mask without the cleared body).
    """
    from local_terrain.navigation_cost import GapPolicy, navigation_costs

    costs, uncertain = navigation_costs(layers, resolution=resolution, enabled=policy is not None,
                                        policy=policy or GapPolicy())
    cleared = clear_footprint(costs, footprint, grid_pose(record))
    return cleared, uncertain & ~((costs == -1) & (cleared == 0))


def primitives(heading, *, headings=16, length=0.9):
    """Rotate in place to each heading offset, then translate `length` (unwrapped yaw)."""
    for a in np.linspace(-np.pi, np.pi, headings, endpoint=False):
        yield float(a), heading, heading + a, length


def sweeps(footprint, pose, *, headings=16, length=0.9):
    """Cost-independent geometry of every primitive: reuse it across gap policies of one frame."""
    x, y, _ = pose
    start, _ = footprint.overlap(x, y, pose[2], x, y, pose[2])
    out = []
    for a, yaw0, yaw1, dist in primitives(pose[2], headings=headings, length=length):
        rot, rot_out = footprint.overlap(x, y, yaw0, x, y, yaw1)
        x1, y1 = x + dist * math.cos(yaw1), y + dist * math.sin(yaw1)
        tra, tra_out = footprint.overlap(x, y, yaw1, x1, y1, yaw1)
        line, line_out = footprint.line_cells(x, y, x1, y1)
        out.append(dict(offset=a, swept=rot | tra, outside=rot_out or tra_out, line=line, line_outside=line_out))
    return out, start


def accept(master, geometry, config):
    """Per-primitive acceptance under the three semantics for one master costmap."""
    prims, start = geometry
    flat = np.asarray(master).reshape(-1)
    lethal, blocked = flat == LETHAL, flat >= LETHAL
    planner_blocked = (flat >= INSCRIBED) & ((flat != NO_INFORMATION) | (not config.planner_allow_unknown))
    out = [dict(offset=g["offset"], swept=g["swept"],
                area=not (g["outside"] or blocked[g["swept"]].any()),
                controller=not (g["outside"] or lethal[g["swept"]].any()),
                planner=not (g["line_outside"] or planner_blocked[g["line"]].any())) for g in prims]
    return out, dict(start_lethal=bool(lethal[start].any()), start_blocked=bool(blocked[start].any()))


def evaluate(master, footprint, pose, *, headings=16, length=0.9):
    """Per-primitive acceptance under the three semantics plus the swept cell mask."""
    return accept(master, sweeps(footprint, pose, headings=headings, length=length), footprint.config)
