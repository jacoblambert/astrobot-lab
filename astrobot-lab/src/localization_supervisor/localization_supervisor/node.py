"""Own exactly one GLIM child; never restart the ROS stack or other host processes."""
from collections import deque
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ClearEntireCostmap
from astrobot_interfaces.msg import LocalizationStatus as Status
from astrobot_interfaces.srv import ResetMap
from astrobot_core.integrity import PoseSample, PoseValidator
from astrobot_core.recovery import RecoveryBudget, orientation_from_gravity


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# Absolute-fix gates (2026-10-20). Per axis, never one 3-D trace sigma: RTK's vertical
# variance (2x horizontal sigma) must not widen the horizontal gate. While GLIM runs, each
# fix is gated against GLIM's own position at the fix stamp (innovation), so an outlier
# burst cannot capture the reference the way a chain of fix-to-fix comparisons can (C3
# outlier1: one noisy fix inside the gate, then the whole 1 m burst). Without GLIM: fix to
# last accepted fix with a speed bound (the old 2 m/s * dt let a 1 m burst in after 0.5 s).
GATE_SIGMA = 6.  # per-axis sigma; a 2-D 6-sigma radius has p = exp(-18)
MAX_SPEED = .5  # m/s, rover motion bound over intervals GLIM does not cover
GLIM_ALLOWANCE = .1  # m, GLIM's own error against truth (C3 nominal max ~10 cm)


def horizontal_vertical_sigma(c):
    return math.sqrt((c[0]+c[7])/2), math.sqrt(c[14])


class LocalizationSupervisor(Node):
    def __init__(self):
        super().__init__("localization_supervisor")
        self.config_path = Path(self.declare_parameter("config_path", "").value)
        # Pose-only acquisition, never navigation readiness. Default stays false.
        self.shadow_mode = bool(self.declare_parameter("shadow_mode", False).value)
        self.shadow_seed_path = self.declare_parameter("shadow_seed_path", "").value
        if self.shadow_seed_path and not self.shadow_mode:
            raise ValueError("GT seed requires explicit shadow mode")
        self.shadow_seed_consumed = False
        self.state, self.reason, self.epoch = Status.STARTING, "waiting_for_reference", 0
        self.process = None
        self.run_directory = Path(tempfile.mkdtemp(prefix="astrobot-localization-"))
        self.budget = RecoveryBudget(2)
        self.pose_validator = PoseValidator(position_slack=0.25)
        self.fixes, self.imu = deque(maxlen=100), deque(maxlen=200)
        self.odom = None
        self.last_pose = None
        self.recent_poses = deque(maxlen=100)
        self.accepted_stamps = deque(maxlen=100)
        self.pending_clouds, self.pending_lidar = deque(maxlen=5), deque(maxlen=100)
        self.last_sim = 0.0
        self.clock_wall = time.monotonic()
        self.last_tick_wall = self.clock_wall
        self.clock_paused = True
        self.stationary_since = None
        self.minimum_stamp = 0.0
        self.attempt_wall = 0.0
        self.healthy_since = None
        self.map_ready = False
        self.map_stamp = None
        self.nav_active = False
        self.reset_future = None
        self.clear_futures = None
        self.health_pub = self.create_publisher(Status, "localization/status", 10)
        self.fix_pub = self.create_publisher(PoseWithCovarianceStamped, "localization/absolute_pose", 20)
        self.pose_pub = self.create_publisher(PoseStamped, "slam/pose_corrected", 10)
        self.lidar_pub = self.create_publisher(PoseStamped, "slam/lidar_pose_corrected", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "slam/aligned_points_corrected", qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/rtk/pose", self._fix, 20)
        self.create_subscription(Imu, "imu", self._imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", lambda m: setattr(self, "odom", m), 20)
        self.create_subscription(PoseStamped, "slam_raw/pose_corrected", self._pose, 20)
        self.create_subscription(PoseStamped, "slam_raw/lidar_pose_corrected", self._lidar, 20)
        self.create_subscription(PointCloud2, "slam_raw/aligned_points_corrected", self._cloud, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "slam/bev_costmap", self._map, 2)
        self.create_subscription(Bool, "slam/map_integrity", self._map_integrity, 10)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", self._nav_status, 10)
        self.reset_client = self.create_client(ResetMap, "slam/reset_map")
        self.cancel_client = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")
        self.clear_clients = [self.create_client(ClearEntireCostmap, name) for name in
                              ("local_costmap/clear_entirely_local_costmap", "global_costmap/clear_entirely_global_costmap")]
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self._tick, clock=self.wall_clock)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _nav_status(self, msg):
        self.nav_active = any(s.status in (1, 2, 3) for s in msg.status_list)

    def _imu(self, msg):
        a = msg.linear_acceleration
        if all(math.isfinite(v) for v in (a.x, a.y, a.z)):
            self.imu.append((stamp(msg), (a.x, a.y, a.z)))

    def _fix(self, msg):
        p, q, c = msg.pose.pose.position, msg.pose.pose.orientation, msg.pose.covariance
        values = (stamp(msg), p.x, p.y, p.z, q.x, q.y, q.z, q.w, c[0], c[7], c[14], c[35])
        if msg.header.frame_id != "map" or not all(math.isfinite(v) for v in values):
            return
        if min(c[0], c[7], c[14]) <= 0 or not 0 < c[35] < 0.1:
            return
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .02 or not -.05 <= self.now_sec()-stamp(msg) <= .5:
            return
        if self.fixes and stamp(msg) <= stamp(self.fixes[-1]):
            return
        expected = self._estimated_position(stamp(msg))
        reference = None
        if expected is not None:
            reference, uncovered = expected
            allowance, cov = GLIM_ALLOWANCE + MAX_SPEED*uncovered, c
        elif self.fixes:
            last = self.fixes[-1]
            old = last.pose.pose.position
            reference = (old.x, old.y, old.z)
            allowance = .05 + MAX_SPEED*(stamp(msg)-stamp(last))
            cov = [c[i]+last.pose.covariance[i] for i in range(15)]
        if reference is not None:
            sh, sz = horizontal_vertical_sigma(cov)
            if (math.hypot(p.x-reference[0], p.y-reference[1]) > allowance + GATE_SIGMA*sh
                    or abs(p.z-reference[2]) > allowance + GATE_SIGMA*sz):
                return
        self.fixes.append(msg)
        self.fix_pub.publish(msg)

    def _estimated_position(self, t):
        """GLIM position at stamp t (held at the nearest end) and the seconds it is held; None if unusable."""
        if not self.recent_poses:
            return None
        times = [s for s, _ in self.recent_poses]
        uncovered = max(0., times[0]-t, t-times[-1])
        if uncovered > .5:
            return None
        xyz = np.array([v for _, v in self.recent_poses])
        return tuple(float(np.interp(t, times, xyz[:, i])) for i in range(3)), uncovered

    def _fault(self, reason, terminal=False):
        terminal = terminal or self.shadow_mode
        if self.state == Status.FAULTED:
            return
        if self.state in (Status.HEALTHY, Status.DEGRADED):
            self.budget.reset()
        self.state = Status.FAULTED if terminal else Status.STOPPING
        self.reason = reason
        # Invalidate asynchronous work before stopping the estimator. A completed
        # reset from a failed attempt must never start another child later.
        if self.reset_future is not None:
            self.reset_future.cancel()
        self.reset_future = None
        if self.clear_futures is not None:
            for future in self.clear_futures:
                future.cancel()
        self.clear_futures = None
        self.map_ready, self.map_stamp = False, None
        self.last_pose = None
        self.recent_poses.clear()
        self.accepted_stamps.clear()
        self.pending_clouds.clear()
        self.pending_lidar.clear()
        self.healthy_since = self.stationary_since = None
        self._publish_health()
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())
        self._stop_child()
        self.get_logger().error(f"localization held: {reason}")

    def _stop_child(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)
        self.process = None

    def _pose(self, msg):
        if self.state not in (Status.REINITIALIZING, Status.REBUILDING, Status.HEALTHY, Status.DEGRADED):
            return
        if self.clock_paused or stamp(msg) < self.minimum_stamp:
            return
        if self.last_pose is not None and stamp(msg) <= stamp(self.last_pose):
            # GLIM can revise a published frame during smoothing. Do not pair a
            # revised pose with an already integrated cloud or treat it as motion.
            return
        p, q = msg.pose.position, msg.pose.orientation
        sample = PoseSample(stamp(msg), (p.x,p.y,p.z), (q.x,q.y,q.z,q.w))
        reason = "wrong_frame" if msg.header.frame_id != "map" else self.pose_validator.validate(sample, self.now_sec())
        if reason:
            self._fault(reason)
            return
        if self.fixes and abs(stamp(self.fixes[-1])-stamp(msg)) < .1:
            fix = self.fixes[-1]
            fp = fix.pose.pose.position
            sh, sz = horizontal_vertical_sigma(fix.pose.covariance)
            if (math.hypot(p.x-fp.x, p.y-fp.y) > max(.5, GATE_SIGMA*sh)
                    or abs(p.z-fp.z) > max(.5, GATE_SIGMA*sz)):
                self._fault("absolute_position_disagreement")
                return
        self.last_pose = msg
        self.recent_poses.append((stamp(msg), (p.x, p.y, p.z)))
        self.accepted_stamps.append((msg.header.stamp.sec,msg.header.stamp.nanosec))
        self.pose_pub.publish(msg)
        if self.healthy_since is None:
            self.healthy_since = stamp(msg)
        if self.state == Status.REINITIALIZING:
            self.state = Status.REBUILDING
        self._drain_outputs()

    def _lidar(self, msg):
        self.pending_lidar.append(msg)
        self._drain_outputs()

    def _cloud(self, msg):
        self.pending_clouds.append(msg)
        self._drain_outputs()

    def _drain_outputs(self):
        if self.state not in (Status.REBUILDING, Status.HEALTHY, Status.DEGRADED):
            return
        for queue, pub in ((self.pending_lidar,self.lidar_pub),(self.pending_clouds,self.cloud_pub)):
            waiting = []
            while queue:
                msg = queue.popleft()
                if stamp(msg) < self.minimum_stamp or self.now_sec()-stamp(msg) > .5:
                    continue
                if (msg.header.stamp.sec,msg.header.stamp.nanosec) in self.accepted_stamps:
                    pub.publish(msg)
                else:
                    waiting.append(msg)
            queue.extend(waiting)

    def _map(self, msg):
        self.map_ready, self.map_stamp = False, None
        info = msg.info
        if (msg.header.frame_id != "map" or self.last_pose is None
                or stamp(msg) < self.minimum_stamp or not 0 <= self.now_sec()-stamp(msg) <= .75
                or not math.isfinite(info.resolution) or info.resolution <= 0
                or len(msg.data) != info.width * info.height):
            return
        p = self.last_pose.pose.position
        # This is a map-rebuild readiness check, not a collision/path approval.
        # Nav2 still checks the complete footprint and route before driving.
        col = math.floor((p.x-info.origin.position.x)/info.resolution)
        row = math.floor((p.y-info.origin.position.y)/info.resolution)
        if not (0 <= row < info.height and 0 <= col < info.width):
            return
        radius = math.ceil(1.5/info.resolution)
        grid = np.asarray(msg.data).reshape(info.height, info.width)
        window = grid[max(0,row-radius):min(info.height,row+radius+1),
                      max(0,col-radius):min(info.width,col+radius+1)]
        self.map_ready = np.count_nonzero((window >= 0) & (window < 50)) >= 25
        self.map_stamp = stamp(msg)

    def _map_integrity(self, msg):
        if not msg.data:
            self._fault("map_capacity_exceeded", terminal=True)

    def _stationary_reference(self, now):
        if self.odom is None or not 0 <= now-stamp(self.odom) < .5:
            self.stationary_since = None
            return None
        t = self.odom.twist.twist
        velocities = (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)
        if (not all(math.isfinite(v) for v in velocities)
                or math.sqrt(sum(v*v for v in velocities[:3])) > .01
                or math.sqrt(sum(v*v for v in velocities[3:])) > .02):
            self.stationary_since = None
            return None
        if self.stationary_since is None:
            self.stationary_since = now
        fixes = [f for f in self.fixes if now-1 <= stamp(f) <= now]
        imu = [a for ts,a in self.imu if now-1 <= ts <= now]
        if now-self.stationary_since < 1 or len(fixes)<10 or len(imu)<10 or now-stamp(fixes[-1])>.25:
            return None
        xyz = np.mean([(f.pose.pose.position.x,f.pose.pose.position.y,f.pose.pose.position.z) for f in fixes],axis=0)
        yaws = [math.atan2(2*(f.pose.pose.orientation.w*f.pose.pose.orientation.z),
                          1-2*f.pose.pose.orientation.z**2) for f in fixes]
        yaw = math.atan2(sum(math.sin(y) for y in yaws),sum(math.cos(y) for y in yaws))
        try:
            q = orientation_from_gravity(np.mean(imu,axis=0),yaw)
        except ValueError:
            return None
        return [*map(float,xyz),*q]

    def _launch_child(self, pose):
        if getattr(self, "shadow_seed_path", ""):
            from astrobot_core.shadow_seed import validate_shadow_seed
            if self.shadow_seed_consumed:
                raise ValueError("experiment GT seed cannot be reused after restart")
            pose = validate_shadow_seed(json.loads(Path(self.shadow_seed_path).read_text()),
                                        self.shadow_mode, self.now_sec())
            self.shadow_seed_consumed = True
        # The current navigation consumer only accepts latest poses/clouds. A
        # revision exporter alone cannot make its accumulated evidence revisable.
        # Fail closed before spawning until a revision installation barrier exists.
        try:
            configs = json.loads((self.config_path / "config.json").read_text())["global"]
            for section in ("sub_mapping", "global_mapping"):
                selected = self.config_path / configs["config_" + section]
                settings = json.loads(selected.read_text())[section]
                if settings.get("enable_optimization") is not False:
                    self._fault("revision_consumer_required", terminal=True)
                    return
        except (OSError, ValueError, KeyError, TypeError):
            self._fault("invalid_mapping_revision_config", terminal=True)
            return
        directory = self.run_directory / str(self.epoch)
        shutil.copytree(self.config_path,directory)
        path = directory / "config_odometry_cpu.json"
        cfg = json.loads(path.read_text())
        cfg["odometry_estimation"]["init_T_world_imu"] = pose
        cfg["odometry_estimation"]["init_v_world_imu"] = [0.,0.,0.]
        path.write_text(json.dumps(cfg,indent=2))
        ros_path = directory / "config_ros.json"
        ros_cfg = json.loads(ros_path.read_text())
        ros_cfg["abs_constraint"]["rtk_topic"] = self.get_namespace().rstrip("/") + "/localization/absolute_pose"
        ros_cfg["revision_export"] = {"localization_epoch": self.epoch}
        ros_path.write_text(json.dumps(ros_cfg,indent=2))
        executable = str(Path(get_package_prefix("glim_ros")) / "lib/glim_ros/glim_rosnode")
        command = [executable,"--ros-args",
            "-r", "__node:=slam_raw", "-r", f"__ns:={self.get_namespace()}",
            "-p","use_sim_time:=true","-p",f"config_path:={directory}"]
        if self.shadow_mode and self.shadow_seed_path:
            # The guarded recorder creates this immutable seed inside its
            # external run. Keep GLIM's shutdown dump there, not /tmp/dump.
            from astrobot_core.shadow_seed import shadow_dump_path
            command.extend(["-p", f"dump_path:={shadow_dump_path(self.shadow_seed_path, self.epoch)}"])
        self.process = subprocess.Popen(command)
        self.state, self.reason = Status.REINITIALIZING, "estimator_starting"
        self.attempt_wall = time.monotonic()
        self.pose_validator.reset()

    def _publish_health(self):
        msg = Status()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.state, msg.epoch, msg.reason = self.state,self.epoch,self.reason
        msg.motion_permitted = (not self.shadow_mode and
                                self.state in (Status.HEALTHY,Status.DEGRADED) and not self.clock_paused)
        if self.last_pose is not None:
            msg.last_pose_stamp = self.last_pose.header.stamp
        self.health_pub.publish(msg)

    def _tick(self):
        now, wall = self.now_sec(),time.monotonic()
        elapsed_wall = wall - self.last_tick_wall
        self.last_tick_wall = wall
        if now < self.last_sim:
            self._fault("clock_reversed_new_mission_required",terminal=True)
        if now > self.last_sim:
            self.clock_wall = wall
        self.clock_paused = wall-self.clock_wall > .5 or now <= 0
        self.last_sim = now
        if self.shadow_mode and self.nav_active:
            self._fault("navigation_active_in_shadow_mode", terminal=True)
        if self.clock_paused:
            self.attempt_wall += elapsed_wall
            self._publish_health()
            return
        if self.process is not None and self.process.poll() is not None:
            self._fault("estimator_exited")
        if self.state in (Status.HEALTHY,Status.DEGRADED,Status.REBUILDING) and self.last_pose is not None:
            if now-stamp(self.last_pose) > .5:
                self._fault("pose_stale")
        if not self.shadow_mode and self.state in (Status.HEALTHY,Status.DEGRADED):
            if self.map_stamp is None or now-self.map_stamp > 1.0:
                self._fault("map_stale")
        if self.state in (Status.HEALTHY,Status.DEGRADED):
            age = now-stamp(self.fixes[-1]) if self.fixes else math.inf
            if age > 2:
                self._fault("absolute_sensor_outage")
            else:
                self.state = Status.DEGRADED if age > .25 else Status.HEALTHY
        if self.state in (Status.STARTING,Status.STOPPING):
            reference = self._stationary_reference(now)
            if reference is not None and not self.nav_active and (self.shadow_mode or self.reset_client.service_is_ready()):
                if not self.budget.consume():
                    self._fault("restart_budget_exhausted",terminal=True)
                else:
                    self.epoch += 1
                    self.minimum_stamp = now
                    self.map_ready = False
                    self.map_stamp = None
                    self.clear_futures = None
                    self.restart_pose = reference
                    self.state, self.reason = Status.REINITIALIZING,"resetting_map"
                    self.attempt_wall = wall
                    if self.shadow_mode:
                        try:
                            self._launch_child(reference)
                        except Exception as exc:
                            self._fault(f"shadow_start_failed:{exc}", terminal=True)
                    else:
                        request = ResetMap.Request(epoch=self.epoch,minimum_stamp=self.get_clock().now().to_msg())
                        self.reset_future = self.reset_client.call_async(request)
        if self.reset_future is not None and self.reset_future.done():
            future, self.reset_future = self.reset_future,None
            try:
                if not future.result().success or future.result().epoch != self.epoch:
                    raise RuntimeError("map reset rejected")
                self._launch_child(self.restart_pose)
            except Exception as exc:
                self._fault(f"restart_failed:{exc}")
        if self.state in (Status.REINITIALIZING,Status.REBUILDING) and wall-self.attempt_wall > 30:
            self.reset_future = None
            self._fault("restart_timeout")
        map_fresh = self.map_stamp is not None and 0 <= now-self.map_stamp <= .75
        if (self.shadow_mode and self.state == Status.REBUILDING and self.last_pose is not None
                and self.healthy_since is not None and now-self.healthy_since >= 2):
            self.state, self.reason = Status.HEALTHY, "shadow_pose_ready_not_navigation"
        if not self.shadow_mode and self.state == Status.REBUILDING and self.map_ready and map_fresh and self.healthy_since is not None and now-self.healthy_since>=2:
            if self.clear_futures is None and all(c.service_is_ready() for c in self.clear_clients):
                self.clear_futures = [c.call_async(ClearEntireCostmap.Request()) for c in self.clear_clients]
            if self.clear_futures is not None and all(f.done() and f.exception() is None for f in self.clear_futures) and not self.nav_active:
                self.state,self.reason = Status.HEALTHY,"validated"
                self.budget.reset()
        self._publish_health()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_child()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
