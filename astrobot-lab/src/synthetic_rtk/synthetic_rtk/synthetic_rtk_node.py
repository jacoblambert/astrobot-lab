"""Synthetic RTK / surface-beacon absolute-position source.

There is no GNSS on the Moon. This node is a stand-in for a *surface
absolute-positioning beacon* -- a surveyed lander / base station ("home") that
provides the rover with an absolute position fix whose accuracy degrades with
baseline distance from home and which can drop out (occlusion / loss of lock).
It is the additional information GLIM needs to bound the catastrophic pose
jumps that pure LiDAR-IMU odometry suffers in feature-degenerate terrain.

Input : ground-truth pose (`/gt/base_link_pose`, geometry_msgs/PoseStamped,
        frame gt_map, ~15 Hz from OmniLRS).
Output: absolute-position measurement (`/rtk/pose`,
        geometry_msgs/PoseWithCovarianceStamped) in the SLAM world frame.

Model applied to the GT position:
  * antenna lever arm   -- the fix is reported at a configurable sensor offset
    expressed in the base frame (default = the base->IMU origin, so the
    measurement is directly comparable to GLIM's X(i) = T_world_imu translation,
    letting the consumer skip lever-arm handling in early phases).
  * noise               -- zero-mean Gaussian, sigma(d) = sigma0 + sigma_per_m * d,
    where d is the planar baseline distance from home.
  * dropout             -- per-sample Bernoulli loss + optional timed occlusion
    windows (no message published -> the consumer simply adds no factor).
  * latency             -- publication delayed by a (jittered) latency; the
    message *stamp stays the measurement time*, so a timestamp-based consumer
    still binds the fix to the correct pose.

IMPORTANT: this node publishes a measurement TOPIC only. It never broadcasts a
TF (the gt_map->odom GT TF is deliberately disabled during nav so ground truth
does not leak into the SLAM TF tree; this node honours the same discipline).
"""

import math
from collections import deque

import numpy as np
from astrobot_core.integrity import PoseSample, PoseValidator, rotate_vector
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def _yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class SyntheticRtkNode(Node):
    def __init__(self):
        super().__init__("synthetic_rtk")

        # Topics
        self.gt_topic = self.declare_parameter("gt_topic", "/gt/base_link_pose").value
        self.output_topic = self.declare_parameter("output_topic", "/rtk/pose").value
        # Frame the measurement is expressed in. With GLIM's world frame pinned
        # to the real world (init_T_world_imu), this is GLIM's map/world frame.
        # Deliberately NOT gt_map, to avoid implying the GT TF.
        self.world_frame = self.declare_parameter("world_frame", "map").value

        # Home / base station (world XY). Rover spawns here.
        home = self.declare_parameter("home_xy", [20.0, 18.0]).value
        self.home = np.array([float(home[0]), float(home[1])], dtype=float)

        # Antenna / sensor lever arm in the BASE frame. Default = base->IMU
        # origin (0.264, 0.017, 0.262) so the reported fix coincides with GLIM's
        # X(i)=T_world_imu translation (lever-arm-free for early phases).
        lever = self.declare_parameter("antenna_offset_xyz", [0.264, 0.017, 0.262]).value
        self.lever = np.array([float(lever[0]), float(lever[1]), float(lever[2])], dtype=float)

        # Noise model: sigma(d) = sigma0 + sigma_per_m * baseline.
        self.sigma0 = float(self.declare_parameter("sigma0", 0.05).value)
        self.sigma_per_m = float(self.declare_parameter("sigma_per_m", 0.01).value)
        # Vertical noise is typically worse; scale relative to horizontal.
        self.sigma_z_scale = float(self.declare_parameter("sigma_z_scale", 2.0).value)

        # Dropout: per-sample loss probability + optional periodic occlusion.
        self.dropout_prob = float(self.declare_parameter("dropout_prob", 0.0).value)
        self.occlusion_period_s = float(self.declare_parameter("occlusion_period_s", 0.0).value)
        self.occlusion_duration_s = float(self.declare_parameter("occlusion_duration_s", 0.0).value)

        # Latency (publish delay); stamp remains the measurement time.
        self.latency_s = float(self.declare_parameter("latency_s", 0.0).value)
        self.latency_jitter_s = float(self.declare_parameter("latency_jitter_s", 0.0).value)

        # Absolute heading (GNSS-compass / dual-antenna model). Heading is reported in
        # the IMU frame to match GLIM's X(i)=T_world_imu, i.e. base heading + base->IMU
        # yaw offset (pi). Heading accuracy depends on antenna baseline, not range, so
        # sigma_yaw is constant. Emitted via the orientation + yaw covariance.
        self.emit_yaw = bool(self.declare_parameter("emit_yaw", True).value)
        self.sigma_yaw = float(self.declare_parameter("sigma_yaw", 0.01).value)
        self.imu_yaw_offset = float(self.declare_parameter("imu_yaw_offset", math.pi).value)
        self.yaw_dropout_prob = float(self.declare_parameter("yaw_dropout_prob", 0.0).value)

        seed = int(self.declare_parameter("seed", 0).value)
        self.rng = np.random.default_rng(seed)
        self.validator = PoseValidator()
        self._last_clock = None
        self._invalid = 0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, qos)
        self.sub = self.create_subscription(PoseStamped, self.gt_topic, self._on_gt, qos)

        # Compatibility parameter only: observation rate is owned by the source.
        self.declare_parameter("publish_rate_hz", 0.0)

        # Pending (release_time_sec, msg) buffer for latency simulation.
        self._pending = deque()
        self._delay_timer = self.create_timer(0.005, self._drain_pending)

        # Diagnostics.
        self._n_in = 0
        self._n_out = 0
        self._n_drop = 0
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f"synthetic_rtk: {self.gt_topic} -> {self.output_topic} "
            f"frame={self.world_frame} home={self.home.tolist()} "
            f"lever={self.lever.tolist()} sigma(d)={self.sigma0}+{self.sigma_per_m}*d "
            f"dropout={self.dropout_prob} latency={self.latency_s}+-{self.latency_jitter_s}s "
            f"emit_yaw={self.emit_yaw} sigma_yaw={self.sigma_yaw}rad yaw_dropout={self.yaw_dropout_prob} "
            f"(measurement topic only, no TF)"
        )

    # ---- core ------------------------------------------------------------
    def _on_gt(self, msg: PoseStamped):
        self._n_in += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_clock is not None and now < self._last_clock:
            self.validator.reset()
            self._pending.clear()
        self._last_clock = now
        p, q = msg.pose.position, msg.pose.orientation
        sample = PoseSample(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        reason = "wrong_frame" if msg.header.frame_id != "gt_map" else self.validator.validate(sample, now)
        if reason:
            self._invalid += 1
            self.get_logger().warning(f"rejected GT measurement: {reason}", throttle_duration_sec=5.0)
            return
        self._emit(msg)

    def _emit(self, msg: PoseStamped):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Occlusion windows: drop the whole window.
        if self.occlusion_period_s > 0.0 and self.occlusion_duration_s > 0.0:
            if (now % self.occlusion_period_s) < self.occlusion_duration_s:
                self._n_drop += 1
                return

        # Per-sample Bernoulli dropout.
        if self.dropout_prob > 0.0 and self.rng.random() < self.dropout_prob:
            self._n_drop += 1
            return

        p = msg.pose.position
        o = msg.pose.orientation
        yaw = _yaw_from_quat(o.x, o.y, o.z, o.w)
        offset = rotate_vector((o.x, o.y, o.z, o.w), self.lever)
        sx, sy, sz = p.x + offset[0], p.y + offset[1], p.z + offset[2]

        # Baseline-scaled noise (baseline = distance from the home beacon).
        d = float(np.hypot(sx - self.home[0], sy - self.home[1]))
        sigma = self.sigma0 + self.sigma_per_m * d
        sigma_z = sigma * self.sigma_z_scale
        nx, ny = self.rng.normal(0.0, sigma, size=2)
        nz = self.rng.normal(0.0, sigma_z)

        # Publish in the HOME-RELATIVE world frame (origin = home), matching GLIM's
        # pinned map frame (init_T_world_imu places base_link at map origin). z is
        # not shifted (home z = 0). This keeps the rover near map-origin so the
        # origin-centred nav costmaps stay valid, while the measurement remains a
        # direct world-aligned position prior.
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.world_frame
        out.pose.pose.position.x = (sx - self.home[0]) + nx
        out.pose.pose.position.y = (sy - self.home[1]) + ny
        out.pose.pose.position.z = sz + nz

        # 6x6 row-major covariance: position from sigma(d); roll/pitch always huge
        # (compass gives heading only; IMU gravity handles roll/pitch).
        cov = [0.0] * 36
        cov[0] = sigma * sigma       # xx
        cov[7] = sigma * sigma       # yy
        cov[14] = sigma_z * sigma_z  # zz
        big = 1e6
        cov[21] = big  # roll
        cov[28] = big  # pitch

        # Absolute heading: GT base yaw -> IMU yaw (+offset), with noise + dropout.
        emit_heading = self.emit_yaw and not (
            self.yaw_dropout_prob > 0.0 and self.rng.random() < self.yaw_dropout_prob
        )
        if emit_heading:
            meas_yaw = yaw + self.imu_yaw_offset + float(self.rng.normal(0.0, self.sigma_yaw))
            out.pose.pose.orientation.z = math.sin(meas_yaw * 0.5)
            out.pose.pose.orientation.w = math.cos(meas_yaw * 0.5)
            cov[35] = self.sigma_yaw * self.sigma_yaw  # yaw measured
        else:
            out.pose.pose.orientation.w = 1.0  # position-only fix this epoch
            cov[35] = big                       # yaw not measured
        out.pose.covariance = cov

        release = now + self.latency_s
        if self.latency_jitter_s > 0.0:
            release += float(self.rng.uniform(0.0, self.latency_jitter_s))
        if release <= now:
            self.pub.publish(out)
            self._n_out += 1
        else:
            self._pending.append((release, out))
            self._pending = deque(sorted(self._pending, key=lambda item: item[0]))

    def _drain_pending(self):
        if not self._pending:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        while self._pending and self._pending[0][0] <= now:
            _, msg = self._pending.popleft()
            self.pub.publish(msg)
            self._n_out += 1

    def _log_stats(self):
        self.get_logger().info(
            f"synthetic_rtk stats: in={self._n_in} out={self._n_out} "
            f"dropped={self._n_drop} invalid={self._invalid} pending={len(self._pending)}"
        )


def main():
    rclpy.init()
    node = SyntheticRtkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
