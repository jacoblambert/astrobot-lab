"""Declared natural-scene acquisition routes and evaluator-only clearance contracts."""
import hashlib
import json
import math

import numpy as np

from natural_holdout import HOME, SEED
from revision_evidence import validated_transform
from costmap_scenarios import SCENARIOS, TRIAL_PATTERNS, scenario_spec

PATTERNS = ("straight_slow", "turn15", "turn45", "turn360", "peek_reverse", "peek_reverse20", "combined10", "combined15", "loop16", "terrain_step", "costmap1", "costmap2", *SCENARIOS)
GRID = .05
HALF = (.85, .70)  # Inflated footprint, same dimensions as guarded calibration.
ROTATION_RADIUS = math.hypot(*HALF) + .25
# Separate incline from local obstacles. This matches the retained mapper's
# experimental incline cap, not a claim of validated vehicle tip/traction limits.
MAX_SLOPE = .35
SURFACE_RESIDUAL = .02
PATCH_RADIUS = 4  # 9x9 at 5 cm: 40 cm span; steps also checked at every edge.


class SurveyLookup:
    """Immutable survey lookup with the original exact nearest-point policy.

    Bounds are static. Split XY arithmetic avoids repeated Nx2 reduction work,
    but evaluates the same two squared distances and first-index argmin. No
    approximate index, rounded grid coordinate, changed tie or relaxed boundary.
    """

    def __init__(self, points):
        points = np.array(points, dtype=float, copy=True)
        if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
            raise ValueError("invalid surveyed floor points")
        points.flags.writeable = False
        self.x, self.y = points.T
        self.low, self.high = points.min(axis=0), points.max(axis=0)

    def nearest(self, xy):
        xy = np.asarray(xy)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError("invalid floor query")
        if np.any(xy < self.low) or np.any(xy > self.high):
            raise ValueError("footprint left surveyed region")
        return int(np.argmin((self.x-xy[0])**2 + (self.y-xy[1])**2))


def route(pattern):
    if pattern not in PATTERNS:
        raise ValueError("undeclared natural motion pattern")
    if pattern == "loop16":
        # Local planar XY, body yaw, action. Distinct acquisition dose, never
        # substituted for the 4 m development route or used as navigation policy.
        return dict(pattern=pattern, speed=.10, angular_speed=.1,
            targets=[(2., 0., 0., "translate"), (2., 0., -math.pi/2, "rotate"),
                     (2., -6., -math.pi/2, "translate"), (2., -6., -math.pi, "rotate"),
                     (0., -6., -math.pi, "translate"), (0., -6., math.pi/2, "rotate"),
                     (0., 0., math.pi/2, "translate"), (0., 0., 0., "rotate")],
            route_length_m=16., coordinate_policy="initial_planar_base_xy_v1",
            translation_policy="frozen_post_turn_connection_v2",
            turn_translation_policy="anticipatory_corner_feedback_v2",
            turn_position_gain=.6, turn_max_linear_speed=.03,
            turn_reference_advance_m=.15,
            turn_control_policy="bounded_minimum_v1", turn_min_angular_speed=.04,
            turn_arrival_tolerance_rad=.012, pre_roll_s=3., post_roll_s=3., settle_s=1.,
            max_wall_s=1800., translation_leg_deadline_s=85., rotation_leg_deadline_s=75.,
            clearance_policy="relative_surface_v2", floor_max_slope=MAX_SLOPE,
            floor_relative_residual_m=SURFACE_RESIDUAL, floor_patch_radius_cells=PATCH_RADIUS,
            floor_sample_spacing_m=GRID, inflated_half_footprint_m=list(HALF),
            rotation_drift_limit_m=.25, translation_lateral_limit_m=.08,
            translation_heading_limit_rad=.15)
    angle = math.radians(45 if pattern == "turn45" else 15)
    speed = .05 if pattern == "straight_slow" else (.15 if pattern == "combined15" else .10)
    if pattern in ("peek_reverse", "peek_reverse20"):
        targets = [(-.20 if pattern == "peek_reverse20" else -.15, 0.), (0., 0.)]
    elif pattern == "turn360":
        # Eight ordered short arcs disambiguate a full revolution from doing
        # nothing at the final equivalent heading. New acquisition dose only.
        targets = [(.15, 0.)] + [(0., math.radians(d)) for d in (45, 90, 135, 180, -135, -90, -45, 0)] + [(0., 0.)]
    elif pattern.startswith("turn"):
        targets = [(0., angle), (0., -angle), (0., 0.)]
    elif pattern.startswith("combined"):
        targets = [(0., angle), (2., angle), (0., angle), (0., 0.)]
    else:
        targets = [(2., 0.), (0., 0.)]
    spec = dict(pattern=pattern, speed=speed, angular_speed=.1, targets=targets,
                turn_control_policy="bounded_minimum_v1", turn_min_angular_speed=.04,
                turn_arrival_tolerance_rad=.012,
                pre_roll_s=3., post_roll_s=3., settle_s=1., max_wall_s=900.,
                clearance_policy="relative_surface_v2", floor_max_slope=MAX_SLOPE,
                floor_relative_residual_m=SURFACE_RESIDUAL, floor_patch_radius_cells=PATCH_RADIUS,
                floor_sample_spacing_m=GRID,
                inflated_half_footprint_m=list(HALF), rotation_drift_limit_m=.25,
                translation_lateral_limit_m=.08, translation_heading_limit_rad=.15)
    if pattern == "turn360":
        spec.update(max_wall_s=1800., turn_translation_policy="paused_centering_observation_v3",
                    turn_position_gain=.6, turn_max_linear_speed=.03,
                    observation_radius_m=.15, turn_forward_compensation=.27,
                    turn_control_policy="bounded_minimum_with_recentering_v3",
                    recenter_radius_m=.18, recenter_min_projection_m=.04)
    if pattern == "peek_reverse":
        spec.update(speed=.03, settle_s=3., translation_policy="reverse_15cm_observe_return_v1",
                    translation_longitudinal_bounds_m=[-.17, .02])
    if pattern == "peek_reverse20":
        spec.update(speed=.03, settle_s=3., translation_policy="reverse_20cm_observe_return_v1",
                    translation_longitudinal_bounds_m=[-.22, .02], translation_arrival_tolerance_m=.005)
    if pattern == "terrain_step":
        # Targets bound the independent survey only. They MUST NOT be executed
        # by the legacy GT waypoint controller or count as reached waypoints.
        spec.update(targets=[(.30, 0.), (0., 0.)], speed=.02,
                    controller="terrain_step_trial_v1", goal_distance_m=.30,
                    maximum_trial_sim_s=45., survey_targets_only=True,
                    controller_uses_gt=False, independent_gt_emergency_veto=True)
    if pattern in ("costmap1", "costmap2"):
        distance = 1. if pattern == "costmap1" else 2.
        spec.update(targets=[(distance+.5, 0.), (0., 0.)], speed=.05,
                    controller="costmap_unknown_trial_v1", goal_distance_m=distance,
                    maximum_trial_sim_s=120., max_wall_s=1800., survey_targets_only=True,
                    controller_uses_gt=False, independent_gt_emergency_veto=True)
    if pattern in SCENARIOS:
        spec.update(scenario_spec(pattern))
        spec["targets"] = spec["goals_initial_xy"]
    return spec


def route_hash(pattern):
    return hashlib.sha256(json.dumps(route(pattern), sort_keys=True).encode()).hexdigest()


EXECUTION_PROFILES = ("default", "loop16_slow_host_v1", "turn15_gc_stress_v1", "turn45_bounded_v1")


def execution_budget(pattern, profile="default"):
    """Total wall-time envelope, separate from every motion/freshness guard.

    Keep the original route/hash and legacy budget unchanged. The opt-in slow
    host profile allows the same physical loop to finish below real-time speed.
    """
    expected_route = {"loop16_slow_host_v1": "loop16", "turn15_gc_stress_v1": "turn15",
                      "turn45_bounded_v1": "turn45"}
    if profile not in EXECUTION_PROFILES or profile != "default" and pattern != expected_route[profile]:
        raise ValueError("undeclared capture execution profile")
    maximum = route(pattern)["max_wall_s"] if pattern else 600.0
    if profile == "turn45_bounded_v1":
        # Same pure-yaw route, physics and guards; stricter storage cap only.
        # Preserve the separate 50 GiB filesystem reserve. Never fall back or
        # discard evidence if the 16 GiB run quota is exceeded.
        return dict(profile=profile, max_wall_s=maximum, archive_limit_gib=16)
    if profile == "turn15_gc_stress_v1":
        return dict(profile=profile, max_wall_s=maximum, diagnostic_only=True,
                    forced_gc_after_native_ticks=[240, 480, 720], storage_check_delay_s=.11,
                    archive_limit_gib=32)
    return dict(profile=profile, max_wall_s=3600.0 if profile == "loop16_slow_host_v1" else maximum)


def validate_execution_budget(request, expected_profile=None):
    declared = request.get("execution_budget")
    if declared is not None and not isinstance(declared, dict):
        raise ValueError("invalid capture execution budget")
    profile = declared.get("profile") if declared is not None else "default"
    expected = execution_budget(request["pattern"], profile)
    if declared is not None and declared != expected:
        raise ValueError("capture execution budget differs from declared profile")
    if expected_profile is not None and profile != expected_profile:
        raise ValueError("recorder and simulator execution profiles differ")
    return expected


def validate_motion_request(request):
    if "lidar_profile" in request:
        from selected_lidar_profile import validate_request
        validate_request(request)
    elif request.get("pitch_deg") != 5.:
        raise ValueError("legacy natural motion requires its declared 5-degree mount")
    if (request.get("schema") != 1 or request.get("seed") != SEED
            or request.get("home_xyz") != HOME or request.get("motion_authority") is not True
            or request.get("route_sha256") != route_hash(request.get("pattern"))):
        raise ValueError("undeclared natural motion acquisition")
    validate_execution_budget(request)
    for key in ("base_from_imu", "base_from_lidar", "world_from_base"):
        validated_transform(request[key])
    if not np.isfinite(request["seed_stamp"]) or request["seed_stamp"] < 0:
        raise ValueError("invalid initial GT measurement stamp")


def yaw(pose):
    return math.atan2(pose[1, 0], pose[0, 0])


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def world_target(origin, distance, heading):
    return origin[:2, 3] + distance * np.array([math.cos(yaw(origin)+heading), math.sin(yaw(origin)+heading)])


def survey_points(origin, pattern):
    """Conservative bounding region covers translation, turns and skid envelope."""
    spec = route(pattern)
    if pattern == "loop16" or pattern in SCENARIOS:
        angle = yaw(origin)
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        targets = spec["targets"]
        if pattern in SCENARIOS:
            x0, x1, y0, y1 = spec["position_bounds_initial_xy"]
            targets = [(x, y) for x in (x0, x1) for y in (y0, y1)]
        centers = np.array([origin[:2, 3], *[origin[:2, 3]+rotation @ np.array(t[:2]) for t in targets]])
    else:
        centers = np.array([origin[:2, 3], *[world_target(origin, d, h) for d, h in spec["targets"]]])
    # A straight route never spins. Bound the entire admitted +/-0.15 rad
    # heading, +/-8 cm lateral and +/-10 cm endpoint envelope instead.
    if pattern == "straight_slow":
        angles = np.linspace(yaw(origin)-.15, yaw(origin)+.15, 1001)
        direction = np.array([math.cos(yaw(origin)), math.sin(yaw(origin))])
        lateral = np.array([-direction[1], direction[0]])
        # Lipschitz padding also covers angles between the sampled extrema.
        extent = np.max(np.array([HALF[0]*abs(np.cos(angles))+HALF[1]*abs(np.sin(angles)),
                                  HALF[0]*abs(np.sin(angles))+HALF[1]*abs(np.cos(angles))]), axis=1)
        extent += .10*abs(direction)+.08*abs(lateral)+.001
    else:
        extent = ROTATION_RADIUS
    lo = np.floor((centers.min(axis=0)-extent)/GRID)*GRID
    hi = np.ceil((centers.max(axis=0)+extent)/GRID)*GRID
    x = np.arange(lo[0], hi[0]+GRID/2, GRID)
    y = np.arange(lo[1], hi[1]+GRID/2, GRID)
    return np.array([(a, b) for a in x for b in y]), (len(x), len(y))


def ground_sample(hits):
    hits = [h for h in hits if not h[3].startswith("/Robots/")]
    if not hits or not hits[0][3].startswith("/Lunaryard/Terrain/"):
        raise ValueError("swept region contains rock, missing floor or nonterrain support")
    normal = np.asarray(hits[0][2], dtype=float)
    position = np.asarray(hits[0][1], dtype=float)
    if (normal.shape != (3,) or position.shape != (3,) or not np.isfinite([normal, position]).all()
            or abs(np.linalg.norm(normal)-1.) > .001):
        raise ValueError("invalid swept floor geometry")
    if normal[2] < math.cos(math.atan(MAX_SLOPE)):
        raise ValueError(f"swept floor slope exceeds {MAX_SLOPE}")
    return float(position[2])


def validate_ground_tile(points, heights, shape, normals):
    points, heights, normals = (np.asarray(v, dtype=float) for v in (points, heights, normals))
    if (len(shape) != 2 or min(shape) < 3 or points.shape != (math.prod(shape), 2)
            or heights.shape != (len(points),) or normals.shape != (len(points), 3)
            or not np.isfinite(points).all() or not np.isfinite(heights).all()
            or not np.isfinite(normals).all() or np.any(abs(np.linalg.norm(normals, axis=1)-1.) > .001)
            or np.any(normals[:, 2] <= 0)):
        raise ValueError("invalid floor tile")
    xy, values = points.reshape(*shape, 2), heights.reshape(shape)
    gradients = (-normals[:, :2]/normals[:, 2:]).reshape(*shape, 2)
    if np.max(np.linalg.norm(gradients, axis=2)) > MAX_SLOPE:
        raise ValueError("swept floor exceeds separate absolute incline limit")
    maximum_jump = 0.
    for axis in (0, 1):
        delta = np.diff(xy, axis=axis)
        expected = np.zeros_like(delta); expected[..., axis] = GRID
        if not np.allclose(delta, expected, atol=1e-8, rtol=0):
            raise ValueError("floor tile is not the declared ordered grid")
        dz = np.diff(values, axis=axis)
        lo = [slice(None)]*2; hi = lo.copy()
        lo[axis], hi[axis] = slice(None, -1), slice(1, None)
        # Both surface tangents must explain the connection; averaging normals
        # would erase a step or an abrupt ramp-to-wall transition.
        for side in (tuple(lo), tuple(hi)):
            residual = abs(dz-np.sum(gradients[side]*delta, axis=2))
            maximum_jump = max(maximum_jump, float(residual.max()))
    if maximum_jump > SURFACE_RESIDUAL:
        raise ValueError("swept floor relative discontinuity exceeds 2 cm")
    maximum_roughness, maximum_incline = 0., 0.
    for i in range(shape[0]):
        for j in range(shape[1]):
            area = (slice(max(0, i-PATCH_RADIUS), min(shape[0], i+PATCH_RADIUS+1)),
                    slice(max(0, j-PATCH_RADIUS), min(shape[1], j+PATCH_RADIUS+1)))
            patch = xy[area].reshape(-1, 2)-xy[i, j]
            design = np.column_stack((patch, np.ones(len(patch))))
            z = values[area].ravel()
            plane = np.linalg.lstsq(design, z, rcond=None)[0]
            maximum_incline = max(maximum_incline, float(np.linalg.norm(plane[:2])))
            maximum_roughness = max(maximum_roughness, float(abs(z-design@plane).max()))
    if maximum_incline > MAX_SLOPE:
        raise ValueError("swept floor exceeds separate patch incline limit")
    if maximum_roughness > SURFACE_RESIDUAL:
        raise ValueError("swept floor detrended roughness exceeds 2 cm")
    return dict(samples=int(values.size), spacing_m=GRID, minimum_z=float(values.min()), maximum_z=float(values.max()),
                clearance_policy="relative_surface_v2", shape=list(shape),
                maximum_relative_discontinuity_m=maximum_jump, maximum_detrended_roughness_m=maximum_roughness,
                maximum_patch_incline=maximum_incline, absolute_incline_limit=MAX_SLOPE,
                geometry_only=True, navigation_labels=False, finite_sampling_not_mesh_proof=True)


def motion_command(origin, current, pattern, index, *, segment_start=None):
    if pattern in TRIAL_PATTERNS:
        raise ValueError("terrain step trial cannot use GT waypoint commands")
    origin, current = validated_transform(origin), validated_transform(current)
    spec = route(pattern)
    if pattern == "loop16":
        return loop_command(origin, current, index, spec, segment_start=segment_start)
    distance, heading = spec["targets"][index]
    target = world_target(origin, distance, heading)
    direction = np.array([math.cos(yaw(origin)+heading), math.sin(yaw(origin)+heading)])
    error = target-current[:2, 3]
    longitudinal = float(error@direction)
    lateral = abs(float(error@np.array([-direction[1], direction[0]])))
    angle = wrap(yaw(origin)+heading-yaw(current))
    rotating = pattern.startswith("turn") or (pattern.startswith("combined") and index in (0, 3))
    if pattern == "turn360":
        rotating = 1 <= index <= 8
        if np.linalg.norm(current[:2, 3]-origin[:2, 3]) > .25:
            raise ValueError("rotation skid exceeds 25 cm origin envelope")
    if rotating:
        if np.linalg.norm(current[:2, 3]-origin[:2, 3]) > .25:
            raise ValueError("rotation skid exceeds 25 cm origin envelope")
        arrived = abs(angle) < spec["turn_arrival_tolerance_rad"]
        angular = 0. if arrived else math.copysign(
            min(spec["angular_speed"], max(spec["turn_min_angular_speed"], abs(angle))), angle)
        linear = 0.
        if pattern == "turn360" and not arrived:
            # Recorded turn v1 exposed ~16 cm lateral instantaneous turning
            # offset plus backward slip. A 15 cm preparatory step centres the
            # resulting small orbit around the ORIGINAL deployment origin.
            # This is a declared acquisition manoeuvre, not a vehicle model
            # or a relaxed origin/skid bound. Never reanchor from actual skid.
            forward = np.array([math.cos(yaw(current)), math.sin(yaw(current))])
            projected_error = error @ forward
            if (np.linalg.norm(error) > spec["recenter_radius_m"] and
                    abs(projected_error) > spec["recenter_min_projection_m"]):
                # v2 showed that linear corrections WHILE skidding have weak,
                # variable authority. Pause yaw and make a straight correction
                # before the original 25 cm stop bound, then resume the arc.
                return math.copysign(spec["turn_max_linear_speed"], projected_error), 0., False
            radial_error = spec["observation_radius_m"] + error @ forward
            linear = float(np.clip(spec["turn_position_gain"] * radial_error +
                                   spec["turn_forward_compensation"] * angular,
                                   -spec["turn_max_linear_speed"], spec["turn_max_linear_speed"]))
        return linear, angular, arrived
    low, high = spec.get("translation_longitudinal_bounds_m", [-.10, 2.10])
    if lateral > .08 or abs(angle) > .15 or not low <= float((current[:2, 3]-origin[:2, 3])@direction) <= high:
        raise ValueError("translation exceeded heading/lateral/longitudinal envelope")
    linear = math.copysign(min(spec["speed"], max(.02, abs(longitudinal)*.5)), longitudinal)
    if pattern == "turn360":
        linear = float(np.clip(linear, -.03, .03))
    return linear, float(np.clip(angle, -.1, .1)), abs(longitudinal) < spec.get("translation_arrival_tolerance_m", .02)


def loop_command(origin, current, index, spec, *, segment_start=None):
    if isinstance(index, bool) or not isinstance(index, (int, np.integer)) or not 0 <= index < len(spec["targets"]):
        raise ValueError("invalid loop segment")
    x, y, heading, action = spec["targets"][index]
    base_yaw = yaw(origin)
    rotation = np.array([[math.cos(base_yaw), -math.sin(base_yaw)], [math.sin(base_yaw), math.cos(base_yaw)]])
    target = origin[:2, 3] + rotation @ [x, y]
    angle = wrap(base_yaw + heading - yaw(current))
    if action == "rotate":
        if np.linalg.norm(current[:2, 3]-target) > spec["rotation_drift_limit_m"]:
            raise ValueError("loop rotation skid exceeds waypoint envelope")
        arrived = abs(angle) < spec["turn_arrival_tolerance_rad"]
        angular = 0. if arrived else math.copysign(min(spec["angular_speed"],
            max(spec["turn_min_angular_speed"], abs(angle))), angle)
        # Counter contact-induced center drift while yaw changes. Only the
        # along-heading component is controllable by a skid-steer base. Never
        # reanchor the target or relax the original radial/swept-region bound.
        direction = np.array([math.cos(yaw(current)), math.sin(yaw(current))])
        # Reserve room for lateral skid before heading rotates away from the
        # direction in which it can be corrected. This is a fixed command
        # reference, not a new waypoint or a moving/reanchored safety envelope.
        # The arrival heading and radius above still use the nominal target.
        entry_heading = base_yaw + spec["targets"][index-1][2]
        reference = target + spec["turn_reference_advance_m"] * np.array(
            [math.cos(entry_heading), math.sin(entry_heading)])
        linear = 0. if arrived else float(np.clip(
            spec["turn_position_gain"] * ((reference-current[:2, 3]) @ direction),
            -spec["turn_max_linear_speed"], spec["turn_max_linear_speed"]))
        return linear, angular, arrived
    nominal_start = np.array(spec["targets"][index-1][:2]) if index else np.zeros(2)
    start = origin[:2, 3] + rotation @ nominal_start
    if segment_start is not None:
        actual = validated_transform(segment_start)[:2, 3]
        if np.linalg.norm(actual-start) > spec["rotation_drift_limit_m"]:
            raise ValueError("connection start exceeds admitted corner envelope")
        start = actual
    length = float(np.linalg.norm(target-start))
    direction = (target-start)/length
    error = target-current[:2, 3]
    longitudinal = float(error @ direction)
    lateral = abs(float(error @ np.array([-direction[1], direction[0]])))
    angle = wrap(math.atan2(direction[1], direction[0])-yaw(current))
    # Keep the same overall surveyed center envelope, even when an actual
    # corner offset and tracking error point in the same outward direction.
    local = rotation.T @ (current[:2, 3]-origin[:2, 3])
    vertices = np.array([[0., 0.], *[t[:2] for t in spec["targets"]]])
    outside = local-np.clip(local, vertices.min(0), vertices.max(0))
    if np.linalg.norm(outside) > spec["rotation_drift_limit_m"]:
        raise ValueError("connection left surveyed center envelope")
    if (lateral > spec["translation_lateral_limit_m"] or abs(angle) > spec["translation_heading_limit_rad"]
            or not -.10 <= length-longitudinal <= length+.10):
        raise ValueError("loop translation exceeded heading/lateral/longitudinal envelope")
    arrived = np.linalg.norm(error) < .02
    # Steer back toward the fixed endpoint, not just parallel to the leg.
    # Frozen cross-track guard above cannot be hidden by reanchoring each tick.
    desired = wrap(math.atan2(error[1], error[0])-yaw(current)) if longitudinal >= 0 else angle
    linear = math.copysign(min(spec["speed"], max(.02, abs(longitudinal)*.5)), longitudinal)
    return linear, float(np.clip(desired, -.1, .1)), arrived


def leg_deadline(pattern, index):
    spec = route(pattern)
    if pattern == "loop16":
        return spec["rotation_leg_deadline_s"] if spec["targets"][index][3] == "rotate" else spec["translation_leg_deadline_s"]
    return 65. if pattern == "straight_slow" else 40.


def validate_loop_turn_feedback(origin, current, index, linear, angular):
    expected = motion_command(origin, current, "loop16", index)
    if route("loop16")["targets"][index][3] != "rotate":
        raise ValueError("turn feedback audit requires a turn segment")
    if not np.allclose([linear, angular], expected[:2], atol=1e-9, rtol=0):
        raise ValueError("recorded loop turn differs from declared feedback policy")


def validate_loop_connection_log(rows):
    """Each actual leg start is a recorded pose, frozen until that leg ends."""
    starts = {}
    previous = -1
    for row in rows:
        index = row.get("arrived_segment")
        if index is None:
            index = row["index"]
        if not 0 <= index < 8 or index < previous or index > previous+1:
            raise ValueError("loop segment log order invalid")
        value = row.get("segment_start")
        if index % 2 == 0:
            anchor = validated_transform(value)
            if index not in starts:
                if not np.array_equal(anchor, np.asarray(row["world_from_base"])):
                    raise ValueError("connection was not anchored at actual first pose")
                starts[index] = anchor
            elif not np.array_equal(anchor, starts[index]):
                raise ValueError("connection anchor changed within leg")
        elif value is not None:
            raise ValueError("turn cannot redefine connection anchor")
        previous = index
    return starts


def seed_from_request(request):
    from scipy.spatial.transform import Rotation
    validate_motion_request(request)
    pose = np.array(request["world_from_base"]) @ np.array(request["base_from_imu"])
    pose[:3, 3] -= HOME
    return dict(schema=1, capture=request["capture"], stamp=request["seed_stamp"],
                pose=[*pose[:3, 3], *Rotation.from_matrix(pose[:3, :3]).as_quat()],
                one_time_only=True, stationary_hold_required=True)


def verify_live_start(request, first_frame, samples):
    from glim_gt_start import initial_state
    from shadow_terrain import pose_error
    validate_motion_request(request)
    if first_frame["frame_id"] != 0:
        raise ValueError("first published GLIM frame was not recorded")
    times, poses = zip(*samples)
    seed = initial_state(times, poses, first_frame["stamp_ns"], HOME,
                         request["base_from_imu"], request["base_from_lidar"])
    error = pose_error(np.asarray(first_frame["map_from_imu"]), np.asarray(seed["map_from_imu"]))
    return dict(schema=1, passed=error["position_m"] <= .01 and error["rotation_rad"] <= math.radians(.1),
                errors=error, first_frame=first_frame, measurement_time_gt=seed,
                initialization="one-time prior stationary GT seed", ongoing_gt=False)


def audit_motion_artifacts(capture, recorder, request, complete, grant, configs):
    """Recheck immutable seed, exact route completion and all logged command bounds."""
    def read(path):
        return json.loads(path.read_text())
    paths = [capture / "clearance.json", capture / "clearance_tile.npz", capture / "motion.jsonl",
             recorder / "live_gt_seed.json", recorder / "initial_output_check.json"]
    clearance, seed, first = read(paths[0]), read(paths[3]), read(paths[4])
    if seed != seed_from_request(request) or not first.get("passed") or first.get("ongoing_gt") is not False:
        raise ValueError("invalid one-time GT initialization evidence")
    selected = configs["config.json"]["global"]["config_odometry"]
    if configs[selected]["odometry_estimation"]["init_T_world_imu"] != seed["pose"]:
        raise ValueError("live estimator did not use declared GT seed")
    if (not clearance.get("passed") or clearance["pattern"] != request["pattern"]
            or hashlib.sha256(paths[1].read_bytes()).hexdigest() != clearance["tile_sha256"]):
        raise ValueError("invalid natural clearance evidence")
    spec = route(request["pattern"])
    reached = complete["reached"]
    if [r["index"] for r in reached] != list(range(len(spec["targets"]))):
        raise ValueError("natural route was not completed")
    origin = np.asarray(clearance["origin"])
    rows = [json.loads(line) for line in paths[2].read_text().splitlines()]
    starts = validate_loop_connection_log(rows) if request["pattern"] == "loop16" else {}
    for row in reached:
        anchor = starts.get(row["index"])
        if anchor is not None and not np.array_equal(anchor, np.asarray(row.get("segment_start"))):
            raise ValueError("reached target changed connection anchor")
        if not motion_command(origin, np.asarray(row["world_from_base"]), request["pattern"], row["index"],
                              segment_start=anchor)[2]:
            raise ValueError("claimed target not reached")
    if not rows or not any(abs(r["linear"])+abs(r["angular"]) > 0 for r in rows):
        raise ValueError("missing motion commands")
    times = np.asarray([r["stamp"] for r in rows])
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0) or times[0] < grant["stamp"]+spec["pre_roll_s"]:
        raise ValueError("motion preceded gate or command times invalid")
    if complete["last_stamp"] < reached[-1]["stamp"]+spec["post_roll_s"]:
        raise ValueError("missing stationary endpoint postroll")
    for row in rows:
        if (not np.isfinite([row["linear"], row["angular"]]).all()
                or abs(row["linear"]) > spec["speed"]+1e-12 or abs(row["angular"]) > spec["angular_speed"]+1e-12):
            raise ValueError("natural command exceeded limits")
        # A settled row increments its index after commanding zero. For loop
        # corner transitions, that pose has the PREVIOUS segment's heading.
        index = min(row["index"], len(spec["targets"])-1)
        if row.get("arrived_segment") is not None:
            index = row["arrived_segment"]
            if row["linear"] or row["angular"]:
                raise ValueError("settled transition has nonzero command")
            if not motion_command(origin, np.asarray(row["world_from_base"]), request["pattern"], index,
                                  segment_start=starts.get(index))[2]:
                raise ValueError("settled transition did not reach segment")
        else:
            expected = motion_command(origin, np.asarray(row["world_from_base"]), request["pattern"], index,
                                      segment_start=starts.get(index))
            if request["pattern"] == "turn360" and not np.allclose(
                    [row["linear"], row["angular"]], expected[:2], atol=1e-9, rtol=0):
                raise ValueError("recorded observation turn differs from fixed-origin feedback")
            if request["pattern"] == "loop16" and spec["targets"][index][3] == "rotate":
                validate_loop_turn_feedback(origin, np.asarray(row["world_from_base"]), index,
                                            row["linear"], row["angular"])
    return paths
