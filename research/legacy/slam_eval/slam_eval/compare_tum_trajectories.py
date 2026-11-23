import argparse
import bisect
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TrajectoryPoint:
    t: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def read_tum(path: str) -> list[TrajectoryPoint]:
    points = []
    with open(path, "r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) < 4:
                continue
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            if len(fields) >= 8:
                qx, qy, qz, qw = (float(fields[4]), float(fields[5]), float(fields[6]), float(fields[7]))
            points.append(
                TrajectoryPoint(
                    float(fields[0]),
                    float(fields[1]),
                    float(fields[2]),
                    float(fields[3]),
                    qx=qx,
                    qy=qy,
                    qz=qz,
                    qw=qw,
                )
            )
    if not points:
        raise RuntimeError(f"no trajectory points in {path}")
    return points


def yaw(point: TrajectoryPoint) -> float:
    return math.atan2(
        2.0 * (point.qw * point.qz + point.qx * point.qy),
        1.0 - 2.0 * (point.qy * point.qy + point.qz * point.qz),
    )


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def transform_xyz_yaw(point: TrajectoryPoint, yaw_offset: float, tx: float, ty: float, tz: float) -> tuple[float, float, float, float]:
    c = math.cos(yaw_offset)
    s = math.sin(yaw_offset)
    return (
        c * point.x - s * point.y + tx,
        s * point.x + c * point.y + ty,
        point.z + tz,
        wrap_angle(yaw(point) + yaw_offset),
    )


def align_translation_only(pair: tuple[TrajectoryPoint, TrajectoryPoint]) -> tuple[float, float, float, float]:
    reference, estimate = pair
    return (
        0.0,
        reference.x - estimate.x,
        reference.y - estimate.y,
        reference.z - estimate.z,
    )


def align_by_first_pose(pair: tuple[TrajectoryPoint, TrajectoryPoint]) -> tuple[float, float, float, float]:
    reference, estimate = pair
    yaw_offset = wrap_angle(yaw(reference) - yaw(estimate))
    c = math.cos(yaw_offset)
    s = math.sin(yaw_offset)
    tx = reference.x - (c * estimate.x - s * estimate.y)
    ty = reference.y - (s * estimate.x + c * estimate.y)
    tz = reference.z - estimate.z
    return yaw_offset, tx, ty, tz


def align_yaw_translation(
    pairs: list[tuple[TrajectoryPoint, TrajectoryPoint]],
) -> tuple[float, float, float, float]:
    ref_cx = sum(ref.x for ref, _ in pairs) / len(pairs)
    ref_cy = sum(ref.y for ref, _ in pairs) / len(pairs)
    est_cx = sum(est.x for _, est in pairs) / len(pairs)
    est_cy = sum(est.y for _, est in pairs) / len(pairs)
    est_cz = sum(est.z for _, est in pairs) / len(pairs)
    ref_cz = sum(ref.z for ref, _ in pairs) / len(pairs)

    sxx = 0.0
    sxy = 0.0
    for ref, est in pairs:
        ex = est.x - est_cx
        ey = est.y - est_cy
        rx = ref.x - ref_cx
        ry = ref.y - ref_cy
        sxx += ex * rx + ey * ry
        sxy += ex * ry - ey * rx

    yaw = math.atan2(sxy, sxx)
    c = math.cos(yaw)
    s = math.sin(yaw)
    tx = ref_cx - (c * est_cx - s * est_cy)
    ty = ref_cy - (s * est_cx + c * est_cy)
    tz = ref_cz - est_cz
    return yaw, tx, ty, tz


def nearest_index(times: list[float], t: float) -> int:
    idx = bisect.bisect_left(times, t)
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    return idx if abs(times[idx] - t) < abs(times[idx - 1] - t) else idx - 1


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True, help="Reference TUM trajectory, usually simulator GT.")
    parser.add_argument("--estimate", required=True, help="Estimated TUM trajectory, e.g. GLIM traj_lidar.txt.")
    parser.add_argument("--max-dt", type=float, default=0.1)
    parser.add_argument(
        "--align",
        choices=["translation", "first", "yaw"],
        default="yaw",
        help=(
            "translation uses first-position translation only, first uses first position+yaw, "
            "yaw optimizes a planar yaw+translation alignment over all matched poses"
        ),
    )
    args = parser.parse_args()

    reference = read_tum(args.reference)
    estimate = read_tum(args.estimate)
    ref_times = [point.t for point in reference]

    pairs = []
    for est in estimate:
        idx = nearest_index(ref_times, est.t)
        ref = reference[idx]
        if abs(ref.t - est.t) <= args.max_dt:
            pairs.append((ref, est))

    if not pairs:
        raise RuntimeError("no timestamp matches; check clocks or increase --max-dt")

    if args.align == "translation":
        yaw_offset, tx, ty, tz = align_translation_only(pairs[0])
    elif args.align == "first":
        yaw_offset, tx, ty, tz = align_by_first_pose(pairs[0])
    else:
        yaw_offset, tx, ty, tz = align_yaw_translation(pairs)

    position_errors = []
    yaw_errors = []
    for ref, est in pairs:
        x, y, z, est_yaw = transform_xyz_yaw(est, yaw_offset, tx, ty, tz)
        ex = x - ref.x
        ey = y - ref.y
        ez = z - ref.z
        position_errors.append(math.sqrt(ex * ex + ey * ey + ez * ez))
        yaw_errors.append(abs(wrap_angle(est_yaw - yaw(ref))))

    rmse = math.sqrt(sum(error * error for error in position_errors) / len(position_errors))
    mean = sum(position_errors) / len(position_errors)
    final_error = position_errors[-1]
    sorted_errors = sorted(position_errors)
    p50 = sorted_errors[int(0.50 * (len(sorted_errors) - 1))]
    p90 = sorted_errors[int(0.90 * (len(sorted_errors) - 1))]
    p95 = sorted_errors[int(0.95 * (len(sorted_errors) - 1))]
    yaw_rmse = math.sqrt(sum(error * error for error in yaw_errors) / len(yaw_errors))
    yaw_mean = sum(yaw_errors) / len(yaw_errors)
    print(
        f"matched={len(position_errors)} align={args.align} yaw={yaw_offset:.6f} "
        f"rmse={rmse:.4f} mean={mean:.4f} min={min(position_errors):.4f} "
        f"p50={p50:.4f} p90={p90:.4f} p95={p95:.4f} max={max(position_errors):.4f} final={final_error:.4f} "
        f"yaw_rmse={yaw_rmse:.4f} yaw_mean={yaw_mean:.4f}"
    )


if __name__ == "__main__":
    main()
