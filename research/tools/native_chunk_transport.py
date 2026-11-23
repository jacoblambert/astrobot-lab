"""Versioned native LiDAR PointCloud2 wire contract; no GT/frame transforms.

Standard ROS messages keep Isaac independent of the custom-interface overlay.
Capture UUID and sequence are repeated fields (40 bytes/return), allowing one
atomic, self-identifying message without a separately timed metadata topic.
"""

import re

import numpy as np

FIELDS = (
    ("x", 0, 7, 1),
    ("y", 4, 7, 1),
    ("z", 8, 7, 1),
    ("depth", 12, 7, 1),
    ("intensity", 16, 2, 1),
    ("sequence", 20, 6, 1),
    ("capture_uuid", 24, 2, 16),
)
DTYPE = np.dtype(
    dict(
        names=[f[0] for f in FIELDS],
        formats=["<f4", "<f4", "<f4", "<f4", "u1", "<u4", ("u1", 16)],
        offsets=[f[1] for f in FIELDS],
        itemsize=40,
    )
)
MAX_POINTS = 100_000
FRAME = "vlp16"


def acquisition_stamp_ns(stamp):
    """Match Isaac 5.1 Ros2ClockMessageImpl's int64(timeStamp * 1e9).

    This quantizes the native acquisition time; it does not relabel a scan with
    callback/receipt time. Rounding upward can put a scan 1 ns past its own clock.
    """
    if not np.isfinite(stamp) or not 0 <= stamp < 2**31:
        raise ValueError("invalid native acquisition time")
    return int(stamp * 1e9)


def encode(capture, sequence, points, depths, intensities):
    if not isinstance(capture, str) or not re.fullmatch(r"revision-[0-9a-f]{32}", capture):
        raise ValueError("invalid native capture identity")
    if type(sequence) is not int or not 0 <= sequence < 2**32:
        raise ValueError("invalid native sequence")
    points, depths, intensities = map(np.asarray, (points, depths, intensities))
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not 0 < len(points) <= MAX_POINTS
        or depths.shape != (len(points),)
        or intensities.shape != (len(points),)
        or not np.isfinite(points).all()
        or not np.isfinite(depths).all()
        or intensities.dtype != np.uint8
    ):
        raise ValueError("invalid native return arrays")
    result = np.zeros(len(points), dtype=DTYPE)
    for i, name in enumerate(("x", "y", "z")):
        result[name] = points[:, i]
    result["depth"], result["intensity"], result["sequence"] = depths, intensities, sequence
    result["capture_uuid"] = np.frombuffer(bytes.fromhex(capture[9:]), dtype=np.uint8)
    return result.tobytes()


def decode(message):
    actual = tuple((f.name, f.offset, f.datatype, f.count) for f in message.fields)
    stamp = message.header.stamp
    if (
        message.header.frame_id != FRAME
        or stamp.sec < 0
        or not 0 <= stamp.nanosec < 10**9
        or message.height != 1
        or not 0 < message.width <= MAX_POINTS
        or actual != FIELDS
        or message.is_bigendian
        or not message.is_dense
        or message.point_step != 40
        or message.row_step != 40 * message.width
        or len(message.data) != message.row_step
    ):
        raise ValueError("invalid native wire layout/frame/stamp")
    rows = np.frombuffer(message.data, dtype=DTYPE)
    if (
        not (rows["sequence"] == rows["sequence"][0]).all()
        or not (rows["capture_uuid"] == rows["capture_uuid"][0]).all()
    ):
        raise ValueError("mixed native chunk identities")
    points = np.column_stack([rows[name] for name in ("x", "y", "z")])
    depths, intensities = rows["depth"].copy(), rows["intensity"].copy()
    if not np.isfinite(points).all() or not np.isfinite(depths).all():
        raise ValueError("nonfinite native returns")
    return dict(
        capture="revision-" + rows["capture_uuid"][0].tobytes().hex(),
        sequence=int(rows["sequence"][0]),
        stamp_ns=stamp.sec * 10**9 + stamp.nanosec,
        points=points,
        depths=depths,
        intensities=intensities,
    )


def message(capture, sequence, stamp_ns, points, depths, intensities):
    from sensor_msgs.msg import PointCloud2, PointField

    if type(stamp_ns) is not int or not 0 <= stamp_ns < 2**31 * 10**9:
        raise ValueError("invalid native acquisition stamp")
    payload = encode(capture, sequence, points, depths, intensities)
    cloud = PointCloud2()
    cloud.header.frame_id = FRAME
    cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    cloud.height, cloud.width = 1, len(points)
    cloud.fields = [PointField(name=n, offset=o, datatype=t, count=c) for n, o, t, c in FIELDS]
    cloud.is_bigendian, cloud.is_dense = False, True
    cloud.point_step, cloud.row_step, cloud.data = 40, len(payload), payload
    return cloud
