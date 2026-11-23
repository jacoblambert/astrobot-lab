#!/usr/bin/env python3
"""Summarize and visualize Phase 4 GT-vs-collected resource samples."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def write_ppm(path: Path, image: np.ndarray) -> None:
    image = np.clip(image, 0, 255).astype(np.uint8)
    header = f"P6\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
    path.write_bytes(header + image.tobytes())


def downsample_max(grid: np.ndarray, max_size: int) -> tuple[np.ndarray, int]:
    factor = max(1, int(math.ceil(max(grid.shape) / max_size)))
    pad_rows = (-grid.shape[0]) % factor
    pad_cols = (-grid.shape[1]) % factor
    if pad_rows or pad_cols:
        grid = np.pad(grid, ((0, pad_rows), (0, pad_cols)), constant_values=0)
    reduced = grid.reshape(grid.shape[0] // factor, factor, grid.shape[1] // factor, factor).max(axis=(1, 3))
    return reduced, factor


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("resource_compare_npz")
    parser.add_argument("--summary", default="")
    parser.add_argument("--ppm", default="")
    parser.add_argument("--max-image-size", type=int, default=900)
    args = parser.parse_args()

    data = np.load(args.resource_compare_npz)
    gt = data["gt_resource_grid"].astype(np.float32)
    resolution = float(data["resolution"][0])
    origin = data["origin"].astype(np.float32)
    rows = data["collected_rows"].astype(np.int32)
    cols = data["collected_cols"].astype(np.int32)
    values = data["collected_values"].astype(np.float32)

    gt_max_index = int(np.argmax(gt)) if gt.size else 0
    gt_max_row = gt_max_index // gt.shape[1] if gt.size else 0
    gt_max_col = gt_max_index % gt.shape[1] if gt.size else 0
    gt_max_value = float(gt[gt_max_row, gt_max_col] / 100.0) if gt.size else 0.0
    gt_max_xy = [
        float(origin[0] + (gt_max_col + 0.5) * resolution),
        float(origin[1] + (gt_max_row + 0.5) * resolution),
    ]

    best_index = int(np.argmax(values)) if values.size else -1
    best_value = float(values[best_index]) if best_index >= 0 else 0.0
    best_row = int(rows[best_index]) if best_index >= 0 else -1
    best_col = int(cols[best_index]) if best_index >= 0 else -1
    best_xy = [
        float(origin[0] + (best_col + 0.5) * resolution),
        float(origin[1] + (best_row + 0.5) * resolution),
    ] if best_index >= 0 else None
    hotspot_distance = (
        math.hypot(best_xy[0] - gt_max_xy[0], best_xy[1] - gt_max_xy[1]) if best_xy is not None else None
    )

    summary = {
        "gt_max_value": gt_max_value,
        "gt_max_xy": gt_max_xy,
        "best_collected_value": best_value,
        "best_collected_xy": best_xy,
        "best_to_gt_hotspot_distance_m": hotspot_distance,
        "collected_cell_count": int(values.size),
        "best_collected_fraction_of_gt_max": best_value / gt_max_value if gt_max_value > 0 else 0.0,
    }

    text = json.dumps(summary, indent=2, sort_keys=True)
    print(text)
    if args.summary:
        Path(args.summary).write_text(text + "\n", encoding="utf-8")

    if args.ppm:
        reduced, factor = downsample_max(gt, args.max_image_size)
        norm = np.clip(reduced / max(1.0, float(reduced.max())), 0.0, 1.0)
        image = np.zeros((reduced.shape[0], reduced.shape[1], 3), dtype=np.float32)
        image[..., 0] = 35 + 190 * norm
        image[..., 1] = 35 + 130 * norm
        image[..., 2] = 45 + 25 * norm
        for row, col, value in zip(rows, cols, values):
            rr = int(row // factor)
            cc = int(col // factor)
            if 0 <= rr < image.shape[0] and 0 <= cc < image.shape[1]:
                image[max(0, rr - 1) : min(image.shape[0], rr + 2), max(0, cc - 1) : min(image.shape[1], cc + 2)] = [
                    20,
                    230,
                    80 + 120 * min(1.0, float(value)),
                ]
        if best_index >= 0:
            rr = int(best_row // factor)
            cc = int(best_col // factor)
            image[max(0, rr - 3) : min(image.shape[0], rr + 4), max(0, cc - 3) : min(image.shape[1], cc + 4)] = [
                255,
                255,
                255,
            ]
        rr = int(gt_max_row // factor)
        cc = int(gt_max_col // factor)
        image[max(0, rr - 3) : min(image.shape[0], rr + 4), max(0, cc - 3) : min(image.shape[1], cc + 4)] = [
            40,
            140,
            255,
        ]
        write_ppm(Path(args.ppm), image)


if __name__ == "__main__":
    main()
