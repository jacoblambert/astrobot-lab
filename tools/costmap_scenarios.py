"""Frozen opt-in navigation routes; no terrain truth or live GT inputs."""

import math

SCENARIOS = {
    "costmap_return4": [(2.0, 0.0), (0.0, 0.0)],
    "costmap_square8": [(2.0, 0.0), (2.0, -2.0), (0.0, -2.0), (0.0, 0.0)],
    "costmap_detour8": [(3.5, -1.5), (0.0, 0.0)],
}
COSTMAP_PATTERNS = ("costmap1", "costmap2", *SCENARIOS)
TRIAL_PATTERNS = ("terrain_step", *COSTMAP_PATTERNS)
PROTOCOL = "costmap_route_trial_v13"


def scenario_spec(pattern):
    goals = SCENARIOS[pattern]
    # Wide detour room, not a narrow corridor. Bound estimated AND GT position
    # in this declared rectangle, with additional footprint/turn survey margin.
    bounds = [-0.75, 4.75, -2.0, 2.0] if pattern == "costmap_detour8" else [-0.75, 2.75, -2.75, 0.75]
    if pattern == "costmap_return4":
        bounds = [-0.75, 2.75, -0.75, 0.75]
    return dict(
        goals_initial_xy=[list(p) for p in goals],
        route_length_m=sum(math.dist(a, b) for a, b in zip([(0, 0), *goals], goals)),
        position_bounds_initial_xy=bounds,
        goal_distance_m=max(math.hypot(*p) for p in goals),
        controller=PROTOCOL,
        route_policy="retain_until_blocked_or_consumed_v1",
        speed=0.05,
        angular_speed=0.1,
        turn_control_policy="clearance_checked_drifting_yaw_v1",
        turn_translation_limit_m=0.75,
        turn_command_drift_allowance_m=0.05,
        current_turn_assumption_radius_m=math.hypot(0.5037, 0.3454),
        turn_space_policy="current_footprint_over_uncertainty_occupied_always_veto",
        observation_turn_budget_s=90.0,
        maximum_trial_sim_s=600.0,
        max_wall_s=3600.0,
        survey_targets_only=True,
        controller_uses_gt=False,
        independent_gt_emergency_veto=True,
        # This protocol allows full turns, while retaining measured per-turn
        # drift, per-straight heading/lateral, footprint, stall and lease guards.
        heading_bound_rad=math.pi,
        survey_allows_obstacles=True,
        goal_policy="fixed_initial_planar_slam_frame_including_exact_home",
        route_layout="wide_goal_south_v2" if pattern == "costmap_detour8" else "original_v1",
        archive_limit_gib=32,
    )


def archive_limit(pattern, profile="default"):
    """New bounded trials have a stricter quota; the 50 GiB reserve is unchanged."""
    if profile == "turn45_bounded_v1":
        if pattern != "turn45":
            raise ValueError("bounded turn storage profile requires turn45")
        return 16 * 1024**3
    return (32 if pattern in SCENARIOS or profile == "turn15_gc_stress_v1" else 64) * 1024**3


def recorder_wall_maximum(pattern, profile, budget):
    return budget["max_wall_s"] if pattern in SCENARIOS or profile != "default" else 1800


def check_position(origin, current, bounds):
    """Fixed planar frame; never move a safety boundary with current drift."""
    angle = math.atan2(origin[1, 0], origin[0, 0])
    dx, dy = current[0, 3] - origin[0, 3], current[1, 3] - origin[1, 3]
    x, y = math.cos(angle) * dx + math.sin(angle) * dy, -math.sin(angle) * dx + math.cos(angle) * dy
    if not bounds[0] <= x <= bounds[1] or not bounds[2] <= y <= bounds[3]:
        raise ValueError("route left fixed initial-frame position envelope")
