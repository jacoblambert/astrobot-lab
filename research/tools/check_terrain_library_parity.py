"""Bitwise parity of two terrain libraries on randomized rolling-window scans (ctypes C API)."""
import ctypes as C
import sys

import numpy as np


def load(path):
    lib = C.CDLL(path)
    lib.terrain_create.argtypes = [C.c_int, C.c_int, *[C.c_double] * 4, C.c_int]
    lib.terrain_create.restype = C.c_void_p
    lib.terrain_destroy.argtypes = [C.c_void_p]
    lib.terrain_error.restype = C.c_char_p
    lib.terrain_integrate.argtypes = [C.c_void_p, C.POINTER(C.c_double), C.c_size_t, *[C.c_double] * 5]
    lib.terrain_project.argtypes = [C.c_void_p, C.c_double, C.POINTER(C.c_float)]
    lib.terrain_shift_origin.argtypes = [C.c_void_p, C.c_double, C.c_double]
    return lib


def scene(rng):
    rocks = [(rng.uniform(-8, 8), rng.uniform(-8, 8), rng.uniform(0.1, 0.5), rng.uniform(0.05, 0.4)) for _ in range(25)]
    return rocks


def height(x, y, rocks):
    z = 0.05 * np.sin(0.4 * x) + 0.03 * np.cos(0.3 * y)
    for rx, ry, r, h in rocks:
        d2 = (x - rx) ** 2 + (y - ry) ** 2
        z = np.where(d2 < r * r, z + h * np.sqrt(np.maximum(0, 1 - d2 / (r * r))), z)
    return z


def scan(rng, sensor, yaw, rocks, n=None):
    """Ray-marched 16-ring LiDAR over the height field (sensor-centred, 0.5 deg azimuth)."""
    el = np.deg2rad(np.linspace(-15, -1, 16))
    az = np.deg2rad(np.arange(0, 360, 0.5)) + yaw
    E, A = np.meshgrid(el, az, indexing="ij")
    d = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
    t = np.linspace(0.3, 7.0, 336)
    px = sensor[0] + d[:, None, 0] * t
    py = sensor[1] + d[:, None, 1] * t
    pz = sensor[2] + d[:, None, 2] * t
    below = pz <= height(px, py, rocks)
    hit = below.any(1)
    first = below.argmax(1)
    idx = np.flatnonzero(hit)
    rng_ = t[first[idx]] + rng.normal(0, 0.005, len(idx))
    pts = sensor + d[idx] * rng_[:, None]
    return np.ascontiguousarray(pts, dtype=np.float64)


def run(lib, seed):
    rng = np.random.default_rng(seed)
    rocks = scene(rng)
    handle = lib.terrain_create(60, 60, 0.1, -3.0, -3.0, 60.0, 1)
    assert handle, lib.terrain_error()
    outputs = []
    x = y = 0.0
    yaw = 0.0
    stamp = 1.0
    out = np.empty((60, 60, 6), dtype=np.float32)
    for step in range(45):
        x += 0.05 * np.cos(yaw)
        y += 0.05 * np.sin(yaw)
        yaw += rng.uniform(-0.05, 0.05)
        sensor = (x, y, 0.6)
        pts = scan(rng, sensor, yaw, rocks)
        ok = lib.terrain_integrate(handle, pts.ctypes.data_as(C.POINTER(C.c_double)), len(pts), stamp, *sensor, yaw)
        assert ok == 1, (ok, lib.terrain_error())
        if step % 3 == 2:
            origin = (np.floor(x / 0.1) * 0.1 - 3.0, np.floor(y / 0.1) * 0.1 - 3.0)
            assert lib.terrain_shift_origin(handle, *origin) == 1, lib.terrain_error()
            assert lib.terrain_project(handle, stamp, out.ctypes.data_as(C.POINTER(C.c_float))) == 1, lib.terrain_error()
            outputs.append(out.copy())
        stamp += 0.1
    lib.terrain_destroy(handle)
    return outputs


a, b = load(sys.argv[1]), load(sys.argv[2])
cells = 0
for seed in range(int(sys.argv[3]) if len(sys.argv) > 3 else 6):
    oa, ob = run(a, seed), run(b, seed)
    assert len(oa) == len(ob)
    for i, (u, v) in enumerate(zip(oa, ob)):
        if not np.array_equal(u.view(np.uint32), v.view(np.uint32)):
            diff = np.argwhere(u.view(np.uint32) != v.view(np.uint32))
            raise SystemExit(f"seed {seed} projection {i}: {len(diff)} differing values, first {diff[:3].tolist()}")
        cells += u.size
    classes = np.concatenate([o[:, :, 0].ravel() for o in oa])
    print(f"seed {seed}: {len(oa)} projections identical; class counts",
          {int(k): int(v) for k, v in zip(*np.unique(classes, return_counts=True))})
print(f"PARITY OK: {cells} float values bitwise identical")
