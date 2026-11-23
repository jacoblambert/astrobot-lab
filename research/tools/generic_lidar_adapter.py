"""Opt-in native Generic-ray -> timed GLIM input contract; no ROS/GT dependency.

Six complete acquisition ticks per window, not an invented instantaneous full scan.
Native tick times remain the only time resolution available. Moving use requires
an independent acquisition-pose audit; passing settled fixtures does not grant it.
"""

import numpy as np

from local_terrain.ray_design import native_pattern_indices, selected_setup
from local_terrain.self_filter import chassis_returns
from native_chunk_transport import acquisition_stamp_ns

# Small PhysX XYZ-versus-range differences are measurement diagnostics, not
# transport failures. User-approved replacement of the former 1 mm cutoff.
# Keep a gross corruption check; no point correction, filtering or retiming.
ASSOCIATION_POLICY = dict(version=2, diagnostic_m=0.001, gross_error_m=0.05, raw_points_unchanged=True)

FIELDS = (
    ("x", 0, 7, 1),
    ("y", 4, 7, 1),
    ("z", 8, 7, 1),
    ("intensity", 12, 7, 1),
    ("time", 16, 8, 1),
    ("ring", 24, 4, 1),
    ("beam_id", 28, 6, 1),
)
DTYPE = np.dtype(
    dict(
        names=[f[0] for f in FIELDS],
        formats=["<f4", "<f4", "<f4", "<f4", "<f8", "<u2", "<u4"],
        offsets=[f[1] for f in FIELDS],
        itemsize=32,
    )
)


class NativeWindow:
    """Bounded, fail-sticky assembler: a rejected tick invalidates the stream."""

    def __init__(self, setup_path=None, *, timestamp_policy="nearest_ns_v1"):
        if timestamp_policy not in ("nearest_ns_v1", "isaac_clock_ns_v2"):
            raise ValueError("unknown native timestamp policy")
        self.timestamp_policy = timestamp_policy
        self.setup, self.elevations = selected_setup(setup_path)
        self.azimuth = self.setup["horizontal_start_rad"] + np.arange(self.setup["horizontal_columns"]) * np.deg2rad(
            self.setup["horizontal_resolution_deg"]
        )
        self.next_sequence = 0
        self.previous_stamp = None
        self.previous_beam = None
        self.pending = []
        self.failed = False
        self.closed = False
        self.association_diagnostics = dict(ticks=0, hits=0, hits_over_1mm=0, max_residual_m=0.0)

    def stamp_ns(self, stamp):
        # Historical offline fixtures keep their original nearest-ns encoding.
        return acquisition_stamp_ns(stamp) if self.timestamp_policy == "isaac_clock_ns_v2" else round(stamp * 1e9)

    def push(self, sequence, stamp, points, depth, intensity, azimuth, zenith):
        if self.failed or self.closed:
            raise ValueError("native window is invalidated")
        try:
            return self._push(sequence, stamp, points, depth, intensity, azimuth, zenith)
        except Exception:
            self.failed = True
            self.pending.clear()
            raise

    def _push(self, sequence, stamp, points, depth, intensity, azimuth, zenith):
        if type(sequence) is not int or sequence != self.next_sequence:
            raise ValueError("native sequence gap/duplicate")
        if not np.isfinite(stamp) or stamp < 0 or stamp >= 2**31:
            raise ValueError("invalid native stamp")
        if self.previous_stamp is not None and abs(stamp - self.previous_stamp - 1 / 60) > 1e-7:
            raise ValueError("native time gap/reset; do not retime or join epochs")
        points, depth, intensity, azimuth, zenith = map(np.asarray, (points, depth, intensity, azimuth, zenith))
        n = len(points)
        if (
            n != 10661
            or points.shape != (n, 3)
            or any(v.shape != (n,) for v in (depth, intensity, azimuth, zenith))
            or not all(np.isfinite(v).all() for v in (points, depth, intensity, azimuth, zenith))
            or np.any(depth < 0)
            or not np.isin(intensity, [0, 255]).all()
        ):
            raise ValueError("invalid native sample count/arrays")
        ids = native_pattern_indices(azimuth, zenith, self.azimuth, self.elevations)
        revolution = self.setup["horizontal_columns"] * 32
        if not (np.diff(ids) % revolution == 1).all() or (
            self.previous_beam is not None and ids[0] != (self.previous_beam + 1) % revolution
        ):
            raise ValueError("native beam gap/reorder/duplicate")
        directions = np.column_stack(
            [np.cos(azimuth) * np.cos(zenith), np.sin(azimuth) * np.cos(zenith), -np.sin(zenith)]
        )
        hit = intensity > 0
        residual = np.linalg.norm(points[hit] - directions[hit] * depth[hit, None], axis=1)
        maximum = float(np.max(residual, initial=0))
        diagnostic = self.association_diagnostics
        diagnostic["ticks"] += 1
        diagnostic["hits"] += int(hit.sum())
        diagnostic["hits_over_1mm"] += int(np.count_nonzero(residual > ASSOCIATION_POLICY["diagnostic_m"]))
        diagnostic["max_residual_m"] = max(diagnostic["max_residual_m"], maximum)
        if maximum > ASSOCIATION_POLICY["gross_error_m"]:
            raise ValueError("gross native local XYZ/ray association mismatch (>5 cm)")
        valid = hit & (depth > self.setup["min_range_m"] + 1e-5) & (depth < self.setup["max_range_m"])
        valid &= ~chassis_returns(points, self.setup)
        # Original bytes remain in the native archive; GLIM receives valid hits,
        # without GT geometry, fitted transforms, smoothing or guessed beam times.
        rows = np.zeros(int(valid.sum()), dtype=DTYPE)
        for axis, name in enumerate(("x", "y", "z")):
            rows[name] = points[valid, axis]
        rows["intensity"], rows["ring"], rows["beam_id"] = intensity[valid], ids[valid] % 32, ids[valid]
        self.pending.append((stamp, sequence, rows, n))
        self.next_sequence += 1
        self.previous_stamp = stamp
        self.previous_beam = int(ids[-1])
        if len(self.pending) < 6:
            return None
        first_ns = self.stamp_ns(self.pending[0][0])
        for acquired, _, data, _ in self.pending:
            data["time"] = (self.stamp_ns(acquired) - first_ns) / 1e9
        # NumPy 1.26+ repacks structured concatenate outputs, dropping the two
        # padding bytes before beam_id (32 -> 30 bytes). ROS still advertises
        # point_step=32, so that silently corrupts XYZ/time and lets C++ readers
        # run beyond the payload. Allocate the wire dtype explicitly; zero padding
        # also makes archived payload hashes deterministic across environments.
        merged = np.zeros(sum(len(entry[2]) for entry in self.pending), dtype=DTYPE)
        offset = 0
        for _, _, data, _ in self.pending:
            merged[offset : offset + len(data)] = data
            offset += len(data)
        if not len(merged):
            raise ValueError("empty GLIM window")
        result = dict(
            stamp_ns=first_ns,
            last_tick_ns=self.stamp_ns(stamp),
            end_ns=first_ns + round(float(merged["time"].max()) * 1e9),
            sequences=[entry[1] for entry in self.pending],
            native_stamps=[entry[0] for entry in self.pending],
            native_samples=sum(entry[3] for entry in self.pending),
            points=len(merged),
            data=merged.tobytes(),
            moving_qualified=False,
        )
        self.pending.clear()
        return result

    def finish(self):
        if self.failed or self.pending or not self.next_sequence:
            self.failed = True
            raise ValueError("failed or incomplete native window stream")
        self.closed = True


def message(window):
    """No publication side effect; caller must opt into the experimental topic."""
    validate_wire_payload(window)
    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.header.frame_id = "vlp16"
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(window["stamp_ns"], 10**9)
    msg.height, msg.width = 1, window["points"]
    msg.fields = [PointField(name=n, offset=o, datatype=t, count=c) for n, o, t, c in FIELDS]
    msg.is_bigendian, msg.is_dense = False, True
    msg.point_step, msg.row_step, msg.data = 32, 32 * msg.width, window["data"]
    return msg


def validate_wire_payload(window):
    """Reject schema/stride drift before passing unchecked bytes to ROS/C++."""
    if (
        type(window["points"]) is not int
        or not 0 < window["points"] <= 63966
        or len(window["data"]) != window["points"] * DTYPE.itemsize
    ):
        raise ValueError("native window payload does not match declared 32-byte wire stride")
