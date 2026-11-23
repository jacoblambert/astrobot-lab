"""Opt-in rolling API adapter; the frozen terrain Python ABI stays unchanged."""

import ctypes as C

from layered_terrain import TerrainMapper


class RollingTerrainMapper(TerrainMapper):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not hasattr(self.lib, "terrain_shift_origin"):
            self.close()
            raise ValueError("rolling experiment requires a --rolling-grid library")
        self.lib.terrain_shift_origin.argtypes = [C.c_void_p, C.c_double, C.c_double]
        self.lib.terrain_shift_origin.restype = C.c_int

    def shift_origin(self, origin):
        if self.handle is None:
            raise RuntimeError("closed rolling terrain mapper")
        if len(origin) != 2:
            raise ValueError("two-dimensional origin required")
        if self.lib.terrain_shift_origin(self.handle, *origin) != 1:
            raise ValueError(self.lib.terrain_error().decode())
