"""NumPy interface to the bounded C++ experimental terrain mapper (no ROS)."""
import ctypes as C
from pathlib import Path
import numpy as np


class TerrainMapper:
    def __init__(self, library, *, width=200, height=200, resolution=.1, origin=(-10., -10.), lifetime=30., interpolate=True):
        self.shape = (height, width, 6)
        self.lib = C.CDLL(str(Path(library).resolve(strict=True)))
        self.lib.terrain_create.argtypes = [C.c_int, C.c_int, *[C.c_double]*4, C.c_int]
        self.lib.terrain_create.restype = C.c_void_p
        self.lib.terrain_destroy.argtypes = [C.c_void_p]
        self.lib.terrain_destroy.restype = None
        self.lib.terrain_error.restype = C.c_char_p
        self.lib.terrain_integrate.argtypes = [C.c_void_p, C.POINTER(C.c_double), C.c_size_t, *[C.c_double]*5]
        self.lib.terrain_project.argtypes = [C.c_void_p, C.c_double, C.POINTER(C.c_float)]
        self.handle = self.lib.terrain_create(width, height, resolution, *origin, lifetime, int(interpolate))
        if not self.handle:
            raise ValueError(self.lib.terrain_error().decode())

    def close(self):
        if self.handle:
            self.lib.terrain_destroy(self.handle)
            self.handle = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def integrate(self, points, stamp, sensor, yaw=0.):
        if self.handle is None:
            raise RuntimeError('closed terrain mapper')
        points = np.ascontiguousarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('expected Nx3 points')
        result = self.lib.terrain_integrate(self.handle, points.ctypes.data_as(C.POINTER(C.c_double)), len(points),
                                            stamp, *sensor, yaw)
        if result != 1:
            raise ValueError('invalid/nonmonotonic scan' if result == 0 else self.lib.terrain_error().decode())

    def project(self, stamp):
        if self.handle is None:
            raise RuntimeError('closed terrain mapper')
        output = np.empty(self.shape, dtype=np.float32)
        if self.lib.terrain_project(self.handle, stamp, output.ctypes.data_as(C.POINTER(C.c_float))) != 1:
            raise ValueError(self.lib.terrain_error().decode())
        return output

    def diagnostics(self):
        """Copy last projection's GT-free telemetry; diagnostic builds only."""
        if self.handle is None:
            raise RuntimeError('closed terrain mapper')
        if not hasattr(self.lib, 'terrain_diagnostics'):
            raise ValueError('library was not built with terrain diagnostics')
        height, width, _ = self.shape
        cells = np.empty((height, width, 4), dtype=np.uint32)
        corners = np.empty((height + 1, width + 1, 8), dtype=np.uint32)
        self.lib.terrain_diagnostics.argtypes = [C.c_void_p, C.POINTER(C.c_uint32), C.POINTER(C.c_uint32)]
        if self.lib.terrain_diagnostics(self.handle, cells.ctypes.data_as(C.POINTER(C.c_uint32)),
                                        corners.ctypes.data_as(C.POINTER(C.c_uint32))) != 1:
            raise ValueError('terrain diagnostics require a successful projection')
        return cells, corners

    def observations(self):
        """Per-rigid-input attribution; separate experimental diagnostic build."""
        if self.handle is None:
            raise RuntimeError('closed terrain mapper')
        if not hasattr(self.lib, 'terrain_observations'):
            raise ValueError('library was not built with observation diagnostics')
        channels = self.lib.terrain_observation_channels() if hasattr(self.lib, 'terrain_observation_channels') else 13
        if channels not in (13, 22):
            raise ValueError('unknown observation diagnostic layout')
        output = np.empty((*self.shape[:2], channels), dtype=np.float64)
        self.lib.terrain_observations.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
        if self.lib.terrain_observations(self.handle, output.ctypes.data_as(C.POINTER(C.c_double))) != 1:
            raise ValueError('observation diagnostics require a successful projection')
        return output
