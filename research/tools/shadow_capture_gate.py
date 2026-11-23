"""Short-lived experiment lease. Never a navigation motion-permission message.

Standard JSON/String transport keeps the Isaac side independent of custom ROS
interfaces. A capture-specific token, estimator identity, advancing sequence and
both clock domains prevent an old ready message from authorizing movement.
"""
import math
import re
import threading
from collections import deque

# Wall/sim heartbeat window for the capture lease and its inputs. Was 0.5 s; transit50
# (Oct 21, recorder e19170de) tripped it at 510 sim-s when the recorder spent 0.63 s
# decoding a 4878-frame GLIM revision plus a storage check. Sim-only convenience guard.
HEARTBEAT_S = 2.


class PhysicsIMUReadiness:
    """Optional experiment gate; no estimator input or timing corrections."""

    def __init__(self):
        self.samples = deque(maxlen=120)
        self.clocks = deque(maxlen=120)
        self.sample_wall = self.clock_wall = -math.inf
        self.epoch = None
        self.armed = False

    def sample(self, stamp, wall, frame, orientation_covariance, values):
        if (frame != "Imu_Sensor" or orientation_covariance != -1
                or not all(math.isfinite(v) for v in (stamp, wall, *values))):
            raise ValueError("invalid physics IMU frame/orientation/payload")
        if self.samples and abs((stamp-self.samples[-1])/1e9-1/60) > 1e-5:
            if self.armed:
                raise ValueError("physics IMU cadence lost after arming")
            self.samples.clear()
        self.samples.append(stamp)
        self.sample_wall = wall

    def clock(self, status, wall):
        if status.get("terminal_fault"):
            raise ValueError("physics IMU publisher fault")
        if status.get("valid") is not True:
            if self.armed:
                raise ValueError("physics IMU invalid after arming")
            self.clocks.clear()
            return
        mono, world, sensor, dt = (status[k] for k in
                                   ("monotonic_time", "world_time", "sensor_time", "physics_dt"))
        epoch = status["epoch"]
        if (not all(math.isfinite(v) for v in (mono, world, sensor, dt, wall))
                or min(mono, world, sensor) < 0 or abs(dt-1/60) > 1e-8
                or abs(world-sensor-dt) > 1e-3 or type(epoch) is not int or epoch < 0):
            raise ValueError("invalid physics IMU clock phase")
        stamp = round(mono*1e9)
        changed = self.epoch is not None and self.epoch != epoch
        gap = self.clocks and abs((stamp-self.clocks[-1])/1e9-1/60) > 1e-5
        if changed or gap:
            if self.armed:
                raise ValueError("physics IMU clock reset/gap after arming")
            self.clocks.clear()
        self.epoch = epoch
        self.clocks.append(stamp)
        self.clock_wall = wall

    def ready(self, now_sim, now_wall):
        if not self.samples or not self.clocks:
            return False
        # Permit one topic to arrive ahead of the other; require an exact,
        # contiguous shared second, with both endpoints fresh in both clocks.
        common = sorted(set(self.samples) & set(self.clocks))
        return (len(common) >= 60
                and all(abs((b-a)/1e9-1/60) <= 1e-5 for a, b in zip(common[-60:], common[-59:]))
                and all(-.05 <= now_sim-stamp/1e9 <= .1 for stamp in (self.samples[-1], self.clocks[-1]))
                and all(0 <= now_wall-wall <= HEARTBEAT_S for wall in (self.sample_wall, self.clock_wall)))


class ShadowLease:
    def __init__(self, capture):
        if not re.fullmatch(r"revision-[0-9a-f]{32}", capture):
            raise ValueError("invalid capture identity")
        self.capture = capture
        self.identity = None
        self.sequence = -1
        self.sim_stamp = None
        self.wall_stamp = None
        self.failure = None
        self.first_grant = None
        self._lock = threading.RLock()

    def receive(self, grant, now_sim, now_wall):
        with self._lock:
            return self._receive(grant, now_sim, now_wall)

    def receive_checked(self, decode_and_check_publisher, now_sim, *, clock=None):
        """Timestamp callback entry, not completion of potentially slow validation.

        Validation cannot manufacture a fresh receipt or revive an expired
        lease. Serialize with ready() so a checked, on-time callback is committed
        before readers decide expiry; recheck actual completion time as well.
        """
        if clock is None:
            import time
            clock = time.monotonic
        received = clock()
        with self._lock:
            try:
                if self.identity is not None:
                    self._ready(now_sim, received)
                grant = decode_and_check_publisher()
                accepted = self._receive(grant, now_sim, received)
                if accepted:
                    self._ready(now_sim, clock())
                return accepted
            except Exception as exc:
                self.failure = str(exc)
                raise

    def _receive(self, grant, now_sim, now_wall):
        if not isinstance(grant, dict):
            self.failure = "invalid shadow grant payload"
            raise ValueError(self.failure)
        if grant.get("capture") != self.capture:
            return False  # A latched message from another experiment cannot arm us.
        if self.failure:
            raise ValueError(self.failure)
        try:
            if grant.get("schema") != 1 or grant.get("scope") != "guarded_terrain_capture":
                raise ValueError("invalid shadow grant schema/scope")
            if grant.get("ready") is not True:
                raise ValueError("shadow readiness revoked")
            seq, epoch = grant["sequence"], grant["epoch"]
            if (type(seq) is not int or seq < 0 or type(epoch) is not int or epoch < 1
                    or not isinstance(grant["session"], str) or not grant["session"]
                    or not re.fullmatch(r"revision-[0-9a-f]{32}", grant["recorder"])
                    or not re.fullmatch(r"[0-9a-f]{64}", grant["config_sha256"])):
                raise ValueError("invalid shadow grant identity")
            identity = (grant["session"], epoch, grant["recorder"], grant["config_sha256"])
            if self.identity is not None and identity != self.identity:
                raise ValueError("shadow session/epoch/config changed")
            if self.identity is not None:
                self._ready(now_sim, now_wall)  # A late heartbeat cannot revive an expired lease.
            stamp = grant["stamp"]
            if not all(math.isfinite(x) for x in (stamp, now_sim, now_wall)) or not -.05 <= now_sim-stamp <= HEARTBEAT_S:
                raise ValueError("shadow grant stale/future simulation time")
            if self.sim_stamp is not None and stamp < self.sim_stamp:
                raise ValueError("shadow grant clock reversed")
            if seq <= self.sequence:
                return False  # Replayed duplicates do not extend the lease.
            self.identity, self.sequence = identity, seq
            self.sim_stamp, self.wall_stamp = stamp, now_wall
            if self.first_grant is None:
                self.first_grant = dict(grant)
            return True
        except (ValueError, KeyError, TypeError) as exc:
            self.failure = str(exc)
            raise ValueError(self.failure) from exc

    def ready(self, now_sim, now_wall):
        with self._lock:
            return self._ready(now_sim, now_wall)

    def _ready(self, now_sim, now_wall):
        if self.failure:
            raise ValueError(self.failure)
        if self.identity is None:
            return False
        if (not all(math.isfinite(x) for x in (now_sim, now_wall))
                or not 0 <= now_wall-self.wall_stamp <= HEARTBEAT_S
                or not -.05 <= now_sim-self.sim_stamp <= HEARTBEAT_S):
            self.failure = "shadow heartbeat expired or clock reversed"
            raise ValueError(self.failure)
        return True


def validate_shadow_geometry(request, configured_lidar_from_imu, tf_base_from_imu,
                             tf_base_from_lidar, graph_imu_from_lidar, *, natural_stationary=False, natural_motion=False):
    """Cross-check declared experiment, USD, live TF and actual GLIM revision."""
    import numpy as np
    from revision_evidence import validated_transform

    if natural_motion:
        from natural_motion import validate_motion_request
        validate_motion_request(request)
    elif natural_stationary:
        from natural_holdout import validate_request
        validate_request(request)
    elif (request.get("schema") != 1 or request.get("pattern") != "wide_sweep15_return"
            or request.get("pitch_deg") != 5. or request.get("home_xyz") != [-20., -20., 0.]):
        raise ValueError("shadow request differs from frozen route/mount/home")
    ShadowLease(request["capture"])  # Validate archive/token name before resolving a path.
    usd_imu = validated_transform(request["base_from_imu"])
    usd_lidar = validated_transform(request["base_from_lidar"])
    expected = np.linalg.inv(usd_imu) @ usd_lidar
    comparisons = {
        "base_imu_tf": (validated_transform(tf_base_from_imu), usd_imu),
        "base_lidar_tf": (validated_transform(tf_base_from_lidar), usd_lidar),
        "glim_config": (np.linalg.inv(validated_transform(configured_lidar_from_imu)), expected),
        "glim_revision": (validated_transform(graph_imu_from_lidar), expected),
    }
    errors = {name: float(np.max(np.abs(a-b))) for name, (a, b) in comparisons.items()}
    if max(errors.values()) > 1e-4:
        raise ValueError(f"USD/TF/GLIM extrinsic mismatch: {errors}")
    return errors
