"""Persistent map-frame mosaic of published v2 planner windows (Nav2 global costmap input).

Nav2's StaticLayer resizes a non-rolling costmap to every map it receives, so the
60x60 rolling window alone limits global planning to ~2.5 m goals. The mosaic keeps
the latest observed value per cell; a window's unknown cells never erase what an
earlier window observed (the world is static; the rover is the only mover).
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
        self.windows = self.clipped = 0

    def update(self, window, origin):
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
        part = window[y0 - index[1]:y1 - index[1], x0 - index[0]:x1 - index[0]]
        target = self.grid[y0:y1, x0:x1]
        seen = part != -1
        target[seen] = part[seen]
        self.windows += 1
        self.clipped += int((y1 - y0, x1 - x0) != (h, w))
        return self.grid
