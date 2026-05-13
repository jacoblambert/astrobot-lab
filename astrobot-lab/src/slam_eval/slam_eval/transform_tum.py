#!/usr/bin/env python3
"""Apply a fixed body-frame translation to a TUM trajectory."""

from __future__ import annotations

import argparse
import math


def quat_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    qx, qy, qz, qw = q
    vx, vy, vz = v
    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx
    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx
    return (
        vx + 2.0 * (qw * uvx + uuvx),
        vy + 2.0 * (qw * uvy + uuvy),
        vz + 2.0 * (qw * uvz + uuvz),
    )


def normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in q))
    return tuple(value / norm for value in q)  # type: ignore[return-value]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--xyz", nargs=3, type=float, required=True, help="Translation from input frame to output frame, in input-frame coordinates.")
    parser.add_argument("--quat", nargs=4, type=float, default=[0.0, 0.0, 0.0, 1.0], help="Rotation from input frame to output frame, xyzw.")
    args = parser.parse_args()

    offset = tuple(args.xyz)
    q_offset = normalize(tuple(args.quat))
    count = 0
    with open(args.input, "r", encoding="utf-8") as src, open(args.output, "w", encoding="utf-8") as dst:
        for line in src:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) < 8:
                continue
            t = float(fields[0])
            x, y, z = (float(fields[1]), float(fields[2]), float(fields[3]))
            q = normalize((float(fields[4]), float(fields[5]), float(fields[6]), float(fields[7])))
            ox, oy, oz = quat_rotate(q, offset)
            q_out = normalize(quat_multiply(q, q_offset))
            dst.write(
                f"{t:.9f} {x + ox:.9f} {y + oy:.9f} {z + oz:.9f} "
                f"{q_out[0]:.9f} {q_out[1]:.9f} {q_out[2]:.9f} {q_out[3]:.9f}\n"
            )
            count += 1
    print(f"wrote {count} poses to {args.output}")


if __name__ == "__main__":
    main()
