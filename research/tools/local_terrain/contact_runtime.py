"""Opt-in ROS advisory attached to the existing mapper; never a command owner.

Receives the SAME revision callbacks and clock fences as the mapper. Maintains
bounded historical poses separately from its five-second raw window. All raw
terrain topics and classification bytes remain unchanged. Five short candidate
queries are published at 1 Hz for inspection, not at a controller command rate.
"""

import json
import hashlib
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from export_shadow_trajectory import apply_record
from local_terrain.bookkeeping import BookkeepingLedger
from local_terrain.contact_advisory import ContactAdvisory
from local_terrain.physx_reference import BodyBox
from local_terrain_seal import OutputDigest
from shadow_terrain import PoseTimeline


def verify_contact_seal(archive, terrain_status):
    archive = Path(archive).resolve(strict=True)
    run = Path(terrain_status["archive"]).resolve(strict=True)
    if run.parent != archive:
        raise ValueError("contact archive escaped declared device")
    report = json.loads((run / "contact_collection.json").read_text())
    if not report["completed"] or report["failure"] or report["frames"] < 1 or report["query_batches"] < 1:
        raise ValueError("contact advisory did not complete successfully")
    if hashlib.sha256((run / "contact_advisory.jsonl").read_bytes()).hexdigest() != report["output_sha256"]:
        raise ValueError("contact advisory output changed")
    return report


def short_candidates():
    result = {"stationary": [np.eye(4)]}
    for name, distance, degrees in (
        ("forward_5cm", 0.05, 0),
        ("reverse_5cm", -0.05, 0),
        ("left_5deg", 0, 5),
        ("right_5deg", 0, -5),
    ):
        delta = np.eye(4)
        delta[0, 3] = distance
        delta[:3, :3] = Rotation.from_euler("z", degrees, degrees=True).as_matrix()
        result[name] = [np.eye(4), delta]
    return result


def startup_footprint(turn_area=False):
    """Nominal support geometry; placement is explicit at the call site.

    Nominal physical hull/wheel extent, NOT the inflated obstacle guard. The
    inscribed polygon never claims support outside the ideal turning circle.
    Actual skid is measured separately; it does not expand this assumption.
    Startup retains one fixed anchor. The opt-in current-turn policy reuses
    this geometry at the actual query pose, never at future planner poses.
    """
    if type(turn_area) is not bool:
        raise ValueError("explicit startup turning-area policy required")
    half = (0.5037, 0.3454)
    if not turn_area:
        return np.array([[-half[0], -half[1]], [half[0], -half[1]], list(half), [-half[0], half[1]]])
    # Sixteen sides stay within the 32-vertex index budget even after the
    # contact-height band is projected at a tilted pose. At most 11.74 mm
    # INSIDE the true circle; never expand the safe area to approximate it.
    theta = np.arange(16) * (2 * np.pi / 16)
    return np.hypot(*half) * np.column_stack((np.cos(theta), np.sin(theta)))


class ContactRuntime:
    def __init__(
        self,
        node,
        *,
        deployment_confirmed,
        support_core=False,
        startup_turn_area=False,
        current_turn_area=False,
        hazard_only=None
    ):
        from std_msgs.msg import String
        import shapely

        self.node, self.message = node, String
        if type(current_turn_area) is not bool:
            raise ValueError("explicit current-turn-area policy required")
        self.deployment_confirmed = deployment_confirmed is True
        patches = [
            BodyBox((x, y, -0.1449231063), (0.04, 0.06, 0.001), "nominal_loaded_wheel_patch")
            for x in (-0.256, 0.256)
            for y in (-0.2854, 0.2854)
        ]
        footprint = startup_footprint(startup_turn_area)
        if hazard_only is None:
            from costmap_scenarios import COSTMAP_PATTERNS

            hazard_only = getattr(node, "request", {}).get("pattern") in COSTMAP_PATTERNS
        if type(hazard_only) is not bool or hazard_only and support_core:
            raise ValueError("explicit full-patch hazard-only policy required")
        advisory_class = ContactAdvisory
        if hazard_only:
            from local_terrain.hazard_advisory import HazardAdvisory

            advisory_class = HazardAdvisory
        self.advisory = advisory_class(
            patches,
            footprint,
            BodyBox((0, 0, 0), (0.85, 0.70, 0.001), "original_body_guard"),
            support_core=support_core,
            current_turn_footprint=startup_footprint(True) if current_turn_area else None,
        )
        self.timeline = None
        self.ledger = BookkeepingLedger(node.core.config.max_bookkeeping_frames)
        self.pending, self.pending_frames = [], 0
        self.last_bin = None
        self.first_stamp = None
        self.failure = None
        self.frames = self.queries = 0
        self.output = OutputDigest(node.run / "contact_advisory.jsonl")
        self.publisher = node.create_publisher(String, "/terrain_shadow/contact_advisory", 10)
        self.trial = None
        pattern = getattr(node, "request", {}).get("pattern")
        from costmap_scenarios import COSTMAP_PATTERNS, TRIAL_PATTERNS, SCENARIOS

        if pattern in TRIAL_PATTERNS:
            from local_terrain.step_trial import StepTrial, TOPIC

            if not startup_turn_area or support_core:
                raise ValueError("terrain step trial requires unchanged full patches and startup turn area")
            self.trial = StepTrial(node.request["capture"])
            if pattern in COSTMAP_PATTERNS:
                from local_terrain.costmap_trial import CostmapTrial

                self.trial = CostmapTrial(
                    node.request["capture"],
                    1.0 if pattern == "costmap1" else 2.0,
                    scenario=pattern if pattern in SCENARIOS else None,
                )
            self.trial_publisher = node.create_publisher(String, TOPIC, 1)
            self.trial_output = OutputDigest(node.run / "step_trial.jsonl")
            self.trial_snapshots = set()
            self.trial_timings = []
        geometry_root = Path(shapely.__file__).parent
        self.native_hashes = {
            str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [*geometry_root.glob("*.so"), *(geometry_root.parent / "shapely.libs").glob("*.so*")]
        }
        node.hashes.update(self.native_hashes)
        with (node.run / "contact_manifest.json").open("x") as stream:
            json.dump(
                dict(
                    schema=1,
                    policy=(
                        "current_pose_hazards_unknown_permitted_v1"
                        if hazard_only
                        else "current_pose_lidar_confirmed_contacts_v1"
                    ),
                    hazard_only=hazard_only,
                    deployment_confirmed=self.deployment_confirmed,
                    query_hz=1,
                    patches=[dict(center=p.center, half_extent=p.half_extent) for p in patches],
                    footprint=footprint.tolist(),
                    history_erosion_m=0.02,
                    history_mode="disabled_unknown_permitted" if hazard_only else "lidar_confirmed",
                    support_history_used=not hazard_only,
                    support_core=support_core,
                    startup_turn_area=startup_turn_area,
                    current_turn_area=current_turn_area,
                    startup_radius_m=float(np.hypot(0.5037, 0.3454)) if startup_turn_area else None,
                    pose_age_limit_s=0.2,
                    map_age_limit_s=0.5,
                    navigation_authority=False,
                    acceptance=False,
                    shapely_version=shapely.__version__,
                    geos_version=shapely.geos_version_string,
                    native_geometry_sha256=self.native_hashes,
                    limits=[
                        "grounded operating assumption",
                        "nominal patch geometry",
                        "uncalibrated tracking/stopping/uncertainty",
                        "candidate availability is not goal progress",
                    ],
                ),
                stream,
                indent=2,
            )

    def emit(self, value):
        payload = json.dumps(dict(value, acceptance=False, navigation_authority=False), allow_nan=False)
        self.output.write(payload + "\n")
        self.publisher.publish(self.message(data=payload))

    def invalidate(self, reason):
        self.advisory.context = None
        if self.trial is not None:
            self.emit_trial(dict(valid=False, reason=reason, linear=0.0, angular=0.0))
        self.emit(
            dict(valid=False, reason=self.failure or reason, motions={}, query_ns=max(0, self.node.core.last_clock_ns))
        )

    def receive(self, record):
        if self.failure:
            return
        self.pending_frames += len(record["frames"])
        if len(self.pending) >= 100 or self.pending_frames > 16384:
            self.failure = "contact revision queue budget"
            self.pending.clear()
            self.pending_frames = 0
            self.invalidate(self.failure)
            return
        self.pending.append(record)

    def emit_trial(self, value):
        payload = json.dumps(value, allow_nan=False)
        self.trial_output.write(payload + "\n")
        self.trial_publisher.publish(self.message(data=payload))

    def record_trial(self, stamp, layers, record):
        started = time.perf_counter()
        value = self.trial.update(self.advisory, stamp)
        self.trial_timings.append((time.perf_counter() - started) * 1000)
        self.emit_trial(value)
        if value["decision"]["state"] == "available":
            return
        label = "terminal_refusal" if value["terminal"] else "first_refusal"
        if label in self.trial_snapshots:
            return
        from shapely import difference
        from shapely.geometry import mapping
        from local_terrain.contact_query import PreparedMotion

        prepared = PreparedMotion(
            layers,
            record,
            self.advisory.context.poses(value["relative_poses"]),
            self.advisory.patches,
            self.advisory.body,
        )
        fragments = []
        for ij, part in prepared.parts.items():
            cover = self.advisory.index.unions.get(ij)
            missing = part if cover is None else difference(part, cover)
            if missing.area > 1e-12:
                fragments.append(dict(cell=list(map(int, ij)), area_m2=float(missing.area), geometry=mapping(missing)))
        np.savez_compressed(
            self.node.run / (label + ".npz"),
            layers=layers,
            local_from_base=self.advisory.context.local_from_base,
            local_from_map=record["local_from_map"],
            origin=record["origin"],
        )
        (self.node.run / (label + ".json")).write_text(
            json.dumps(
                dict(
                    proposal=value,
                    resolution=record["resolution"],
                    fragments=fragments,
                    conflict_cells=[list(map(int, ij)) for ij in sorted(prepared.conflicts)],
                    terminal=prepared.terminal,
                ),
                indent=2,
            )
        )
        self.trial_snapshots.add(label)

    def clock(self, stamp, layers, record):
        if self.failure:
            return
        started = time.perf_counter()
        try:
            for revision in self.pending:
                if self.timeline is None:
                    self.timeline = PoseTimeline(revision["session"], revision["epoch"])
                if revision["kind"] in (2, 3):
                    self.ledger.accept(revision, stamp, self.timeline)
                elif revision["kind"] == 1 and not revision["snapshot"]:
                    apply_record(self.timeline, dict(revision, arrival_ns=stamp))
                    self.ledger.commit_odom([self.timeline.frames[f["frame_id"]][-1][2] for f in revision["frames"]])
                else:
                    raise ValueError("unsupported contact revision")
            self.pending.clear()
            self.pending_frames = 0
            if layers is None:
                return
            if self.timeline is None or len(self.timeline.frames) > self.node.core.config.max_bookkeeping_frames:
                raise ValueError("contact pose history missing or over budget")
            for frame, history in self.timeline.frames.items():
                self.timeline.frames[frame] = history[-1:]
            self.timeline.events = dict(list(self.timeline.events.items())[-128:])
            status = self.advisory.update(
                layers,
                record,
                self.timeline,
                self.node.core.base_from_imu,
                stamp,
                grounded=True,
                deployment_confirmed=self.deployment_confirmed,
            )
            self.frames += 1
            update_ms = (time.perf_counter() - started) * 1000
            if self.first_stamp is None:
                self.first_stamp = stamp
            current_bin = (stamp - self.first_stamp) // 10**9
            motions = None
            if current_bin != self.last_bin:
                self.last_bin = current_bin
                motions = {
                    name: self.advisory.query(poses, query_ns=stamp) for name, poses in short_candidates().items()
                }
                self.queries += 1
            self.emit(
                dict(
                    status,
                    session=record["session"],
                    epoch=record["epoch"],
                    revision=record["revision"],
                    evidence_ns=record["evidence_ns"],
                    motions=motions,
                    update_ms=update_ms,
                    total_ms=(time.perf_counter() - started) * 1000,
                )
            )
            if self.trial is not None:
                self.record_trial(stamp, layers, record)
        except Exception as exc:
            # Advisory failure cannot leave a previous "available" result live.
            # Raw mapper remains independent; the sealed advisory reports failure.
            self.failure = str(exc)
            self.pending.clear()
            self.pending_frames = 0
            self.node.get_logger().error("Contact advisory disabled: " + self.failure)
            self.invalidate(self.failure)

    def finish(self):
        self.output.close()
        if self.trial is not None:
            self.trial_output.close()
            (self.node.run / "step_trial_collection.json").write_text(
                json.dumps(
                    dict(
                        completed=self.node.core.completed and not self.node.core.failure and not self.failure,
                        outcome=self.trial.terminal,
                        proposals=self.trial.sequence,
                        steps=self.trial.steps,
                        controller_ms_p50_p95_max=(
                            np.percentile(self.trial_timings, [50, 95, 100]).tolist() if self.trial_timings else None
                        ),
                        output_sha256=self.trial_output.hexdigest(),
                        navigation_authority=False,
                    ),
                    indent=2,
                )
            )
        with (self.node.run / "contact_collection.json").open("x") as stream:
            json.dump(
                dict(
                    completed=self.node.core.completed and not self.node.core.failure and not self.failure,
                    failure=self.failure,
                    frames=self.frames,
                    query_batches=self.queries,
                    output_sha256=self.output.hexdigest(),
                    acceptance=False,
                    navigation_authority=False,
                ),
                stream,
                indent=2,
            )
