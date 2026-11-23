"""Preserve rejected Generic sensor bytes without admitting them to either mapper."""

import hashlib
import json
from pathlib import Path

import numpy as np


def recast_rejected(arrays, pose, cast):
    """Failure-only evaluator diagnostic; never changes/adopts a sensor return.

    Compare both representations against the current physics scene, including
    PhysX's own distance/position consistency. Caller stops motion first and
    must not advance physics between acquisition and this diagnostic.
    """
    points, depths, intensities, azimuth, zenith = arrays
    native_dirs = np.column_stack([np.cos(azimuth) * np.cos(zenith), np.sin(azimuth) * np.cos(zenith), -np.sin(zenith)])
    residual = np.linalg.norm(points - native_dirs * depths[:, None], axis=1)
    bad = np.flatnonzero((intensities > 0) & (residual > 0.001))
    transform = np.asarray(pose["world_from_base"]) @ np.asarray(pose["base_from_lidar"])
    origin = transform[:3, 3]
    rows = []
    for index in bad[:20]:
        row = dict(index=int(index), native_residual_m=float(residual[index]), rays={})
        for label, local in (("beam", native_dirs[index]), ("xyz", points[index])):
            local = local.astype(float)
            direction = transform[:3, :3] @ (local / np.linalg.norm(local))
            direction /= np.linalg.norm(direction)
            hits = cast(tuple(origin), tuple(direction), float(depths[index]) + 0.1)
            row["rays"][label] = dict(
                origin=origin.tolist(),
                direction=direction.tolist(),
                hit_count=len(hits),
                first_hits=[
                    dict(
                        distance_m=float(hit[0]),
                        position=list(map(float, hit[1])),
                        normal=list(map(float, hit[2])),
                        collision=str(hit[3]),
                        native_depth_residual_m=float(hit[0] - depths[index]),
                        position_distance_residual_m=float(
                            np.linalg.norm(np.asarray(hit[1]) - (origin + direction * hit[0]))
                        ),
                    )
                    for hit in hits[:8]
                ],
            )
        rows.append(row)
    return dict(
        acceptance=False,
        association_bad_hits=len(bad),
        inspected=rows,
        note="Same-tick evaluator-only recasts; no sensor data repair or resumed capture.",
    )


def guarded_push(adapter, sequence, stamp, arrays, output, pose):
    try:
        return adapter.push(sequence, stamp, *arrays)
    except Exception as exc:
        # One failure per fail-sticky producer. Never overwrite a prior failure,
        # retry the tick, repair arrays, or mark this as an acquired native chunk.
        file = output / "rejected_native_tick.npz"
        with file.open("xb") as stream:
            np.savez(
                stream,
                **dict(zip(("points", "depths", "intensities", "azimuth", "zenith"), arrays)),
                stamp=stamp,
                producer_sequence=sequence,
                world_from_base=pose["world_from_base"],
                base_from_lidar=pose["base_from_lidar"],
            )
        with (output / "rejected_native_tick.json").open("x") as stream:
            json.dump(
                dict(
                    error=str(exc),
                    stamp=stamp,
                    producer_sequence=sequence,
                    file=file.name,
                    sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    acceptance=False,
                    published=False,
                    note="Rejected sensor tick only; excluded from valid native evidence and GLIM windows.",
                ),
                stream,
                indent=2,
            )
        raise
