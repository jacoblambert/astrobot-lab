"""Fixed loaded-rover support experiments. Geometry is evaluator-only.

These are stationary mechanics experiments, not synthetic sensor observations.
The floor at -0.30 m bounds the drop cases without hiding their height change.
"""

import numpy as np


def loaded_cases():
    floor = [[-1.5, -1.5, -0.40], [1.5, 1.5, -0.30]]
    flat = [[-1.5, -1.5, -0.10], [1.5, 1.5, 0.0]]

    def rails(width, offset):
        return [[[-1.0, y + offset - width / 2, -0.30], [1.0, y + offset + width / 2, 0.0]] for y in (-0.2854, 0.2854)]

    return [
        dict(name="flat_control", boxes=[flat], intent="baseline loaded rover"),
        dict(name="central_60mm_rails", boxes=[floor, *rails(0.06, 0)], intent="support core width only"),
        dict(name="offset_30mm_rails", boxes=[floor, *rails(0.03, 0.04)], intent="support off the wheel centre"),
        dict(
            name="missing_front_left",
            boxes=[floor, [[-1.5, -1.5, -0.3], [0.0, 1.5, 0]], [[0, -1.5, -0.3], [1.5, 0, 0]]],
            intent="one wheel over a 30 cm drop",
        ),
        dict(
            name="front_axle_drop",
            boxes=[floor, [[-1.5, -1.5, -0.3], [0.0, 1.5, 0]]],
            intent="both front wheels over a 30 cm drop",
        ),
        dict(
            name="front_left_8cm_block",
            boxes=[flat, [[0.10, 0.18, 0.0], [0.43, 0.40, 0.08]]],
            intent="known minimum hazard height; not a footprint threshold",
        ),
    ]


def summarize_case(rows):
    """Describe every case; instability is an outcome, not a dropped sample."""
    from scipy.spatial.transform import Rotation

    if len(rows) != 480:
        raise ValueError("all 480 declared physics ticks required")
    positions = np.asarray([r["positions"] for r in rows], float)
    orientations = np.asarray([r["orientations_wxyz"][0] for r in rows], float)
    forces = np.asarray([r["contact_forces_n"] for r in rows], float)
    velocities = np.asarray([r["velocities"][0] for r in rows], float)
    if positions.shape != (480, 5, 3) or forces.shape != (480, 5, 3) or velocities.shape != (480, 6):
        raise ValueError("complete base plus four-wheel measurements required")
    if not all(np.isfinite(v).all() for v in (positions, orientations, forces, velocities)):
        raise ValueError("nonfinite loaded state")
    rotations = Rotation.from_quat(orientations[:, [1, 2, 3, 0]])
    tilt = np.rad2deg(np.arccos(np.clip(rotations.as_matrix()[:, 2, 2], -1, 1)))
    wheel_forces = forces[-60:, 1:, 2]
    return dict(
        frames=len(rows),
        mean_base_height_m=float(positions[-60:, 0, 2].mean()),
        settled_max_tilt_deg=float(tilt[-60:].max()),
        all_ticks_max_tilt_deg=float(tilt.max()),
        settled_max_speed_m_s=float(np.linalg.norm(velocities[-60:, :3], axis=1).max()),
        mean_wheel_up_force_n=wheel_forces.mean(axis=0).tolist(),
        wheel_frames_without_positive_up_force=(wheel_forces <= 0.1).sum(axis=0).tolist(),
        mean_base_up_force_n=float(forces[-60:, 0, 2].mean()),
        acceptance=False,
        motion_tested=False,
        note="Settled contact forces do not certify a filled rectangular wheel patch or safe driving.",
    )
