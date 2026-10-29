"""Persistent map-frame mosaic of published v2 planner windows (Nav2 global costmap input).

Nav2's StaticLayer resizes a non-rolling costmap to every map it receives, so the
60x60 rolling window alone limits global planning to ~2.5 m goals. The mosaic keeps
the latest observed value per cell (the world is static; the rover is the only mover).
Cells a window did not observe carry only its gap policy (bounded unknown, the cleared
footprint) or nothing: they fill a mosaic cell nothing has observed, and never overwrite
an observation. Otherwise the blind ring around a stationary rover would replace the
ground it saw on approach (seed-42 gate, Oct 22).
"""

import numpy as np


class GlobalStitch:
    def __init__(self, lo, hi, first_origin, resolution=0.1, tolerance=1e-3):
        """Covers map-frame box [lo, hi] on the lattice of the first published window."""
        self.resolution, self.tolerance = resolution, tolerance
        first = np.asarray(first_origin, dtype=float)
        self.origin = first + np.floor((np.asarray(lo, dtype=float) - first) / resolution) * resolution
        cells = np.ceil((np.asarray(hi, dtype=float) - self.origin) / resolution).astype(int)
        self.grid = np.full((cells[1], cells[0]), -1, dtype=np.int8)
        self.observed = np.zeros(self.grid.shape, dtype=bool)
        self.windows = self.clipped = 0

    def update(self, window, origin, observed=None):
        """observed: cells the window measured (classes != unknown); None = every known cell."""
        offset = (np.asarray(origin, dtype=float) - self.origin) / self.resolution
        index = np.rint(offset).astype(int)
        if np.abs(offset - index).max() > self.tolerance:
            raise ValueError("window is off the mosaic lattice")
        h, w = window.shape
        y0, x0 = max(0, index[1]), max(0, index[0])
        y1, x1 = min(self.grid.shape[0], index[1] + h), min(self.grid.shape[1], index[0] + w)
        if y1 <= y0 or x1 <= x0:
            self.clipped += 1
            return self.grid
        rows, cols = slice(y0 - index[1], y1 - index[1]), slice(x0 - index[0], x1 - index[0])
        part = window[rows, cols]
        target = self.grid[y0:y1, x0:x1]
        known = part != -1
        if observed is None:
            target[known] = part[known]
        else:
            seen = known & observed[rows, cols]
            policy = known & ~observed[rows, cols] & (self.observed[y0:y1, x0:x1] == 0)
            target[seen | policy] = part[seen | policy]
            self.observed[y0:y1, x0:x1] |= seen
        self.windows += 1
        self.clipped += int((y1 - y0, x1 - x0) != (h, w))
        return self.grid

    def window(self, origin, shape):
        """The mosaic over a window on its lattice; cells outside the mosaic are unknown (-1)."""
        offset = (np.asarray(origin, dtype=float) - self.origin) / self.resolution
        index = np.rint(offset).astype(int)
        if np.abs(offset - index).max() > self.tolerance:
            raise ValueError("window is off the mosaic lattice")
        out = np.full(shape, -1, dtype=np.int8)
        y0, x0 = max(0, index[1]), max(0, index[0])
        y1, x1 = min(self.grid.shape[0], index[1] + shape[0]), min(self.grid.shape[1], index[0] + shape[1])
        if y1 > y0 and x1 > x0:
            out[y0 - index[1]:y1 - index[1], x0 - index[0]:x1 - index[0]] = self.grid[y0:y1, x0:x1]
        return out
