import math
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CommandMux(Node):
    def __init__(self) -> None:
        super().__init__("command_mux")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("nav_timeout_sec", 0.5)
        self.declare_parameter("teleop_timeout_sec", 0.5)
        self.declare_parameter("max_linear_x", 0.5)
        self.declare_parameter("max_angular_z", 1.2)
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("angular_scale_positive", 0.0)
        self.declare_parameter("angular_scale_negative", 0.0)
        self.declare_parameter("default_mode", "auto")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
        self.teleop_timeout_sec = float(self.get_parameter("teleop_timeout_sec").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.angular_scale_positive = float(self.get_parameter("angular_scale_positive").value)
        self.angular_scale_negative = float(self.get_parameter("angular_scale_negative").value)
        self.mode = str(self.get_parameter("default_mode").value).strip().lower()
        if self.mode not in {"auto", "nav", "teleop", "hold"}:
            self.mode = "auto"

        self.estop = False
        self._nav_cmd: Optional[Twist] = None
        self._teleop_cmd: Optional[Twist] = None
        self._nav_stamp = None
        self._teleop_stamp = None
        self._last_source = "hold"
        self._last_output = Twist()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(DiagnosticArray, "/control/status", 10)

        self.create_subscription(Twist, "/cmd_vel_nav", self._nav_callback, 10)
        self.create_subscription(Twist, "/cmd_vel_teleop", self._teleop_callback, 10)
        self.create_subscription(Bool, "/control/estop", self._estop_callback, 10)
        self.create_subscription(String, "/control/mode", self._mode_callback, 10)

        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._publish_loop)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            "basic_control ready: mode=%s nav_timeout=%.2fs teleop_timeout=%.2fs"
            % (self.mode, self.nav_timeout_sec, self.teleop_timeout_sec)
        )

    def _nav_callback(self, msg: Twist) -> None:
        self._nav_cmd = self._limit_twist(msg)
        self._nav_stamp = time.monotonic()

    def _teleop_callback(self, msg: Twist) -> None:
        self._teleop_cmd = self._limit_twist(msg)
        self._teleop_stamp = time.monotonic()

    def _estop_callback(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def _mode_callback(self, msg: String) -> None:
        requested_mode = msg.data.strip().lower()
        if requested_mode not in {"auto", "nav", "teleop", "hold"}:
            self.get_logger().warning("ignoring unsupported control mode '%s'" % msg.data)
            return
        if requested_mode != self.mode:
            self.mode = requested_mode
            self.get_logger().info("control mode -> %s" % self.mode)

    def _publish_loop(self) -> None:
        output, source = self._select_command()
        self._last_output = output
        self._last_source = source
        self.cmd_pub.publish(output)

    def _publish_status(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        nav_age = self._age_seconds(self._nav_stamp)
        teleop_age = self._age_seconds(self._teleop_stamp)

        if self.estop:
            level = DiagnosticStatus.ERROR
            text = "estop"
        elif self._last_source == "hold":
            level = DiagnosticStatus.WARN
            text = "holding"
        else:
            level = DiagnosticStatus.OK
            text = self._last_source

        status = DiagnosticStatus()
        status.name = "basic_control"
        status.hardware_id = "astrobot-dev"
        status.level = level
        status.message = text
        status.values = [
            KeyValue(key="mode", value=self.mode),
            KeyValue(key="estop", value=str(self.estop).lower()),
            KeyValue(key="active_source", value=self._last_source),
            KeyValue(key="nav_age_sec", value=self._format_age(nav_age)),
            KeyValue(key="teleop_age_sec", value=self._format_age(teleop_age)),
            KeyValue(key="linear_x", value=f"{self._last_output.linear.x:.3f}"),
            KeyValue(key="angular_z", value=f"{self._last_output.angular.z:.3f}"),
        ]
        msg.status = [status]
        self.status_pub.publish(msg)

    def _select_command(self) -> tuple[Twist, str]:
        zero = Twist()
        if self.estop or self.mode == "hold":
            return zero, "hold"

        has_nav = self._is_fresh(self._nav_stamp, self.nav_timeout_sec) and self._nav_cmd is not None
        has_teleop = self._is_fresh(self._teleop_stamp, self.teleop_timeout_sec) and self._teleop_cmd is not None

        if self.mode == "teleop":
            return (self._teleop_cmd if has_teleop else zero), ("teleop" if has_teleop else "hold")
        if self.mode == "nav":
            return (self._nav_cmd if has_nav else zero), ("nav" if has_nav else "hold")

        if has_teleop:
            return self._teleop_cmd, "teleop"
        if has_nav:
            return self._nav_cmd, "nav"
        return zero, "hold"

    def _limit_twist(self, msg: Twist) -> Twist:
        limited = Twist()
        limited.linear.x = self._clamp(msg.linear.x * self.linear_scale, -self.max_linear_x, self.max_linear_x)
        limited.linear.y = 0.0
        limited.linear.z = 0.0
        limited.angular.x = 0.0
        limited.angular.y = 0.0
        angular_scale = self.angular_scale
        if msg.angular.z > 0.0 and self.angular_scale_positive > 0.0:
            angular_scale = self.angular_scale_positive
        elif msg.angular.z < 0.0 and self.angular_scale_negative > 0.0:
            angular_scale = self.angular_scale_negative
        limited.angular.z = self._clamp(msg.angular.z * angular_scale, -self.max_angular_z, self.max_angular_z)
        return limited

    def _is_fresh(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        age = self._age_seconds(stamp)
        return age is not None and age <= timeout_sec

    def _age_seconds(self, stamp) -> Optional[float]:
        if stamp is None:
            return None
        return max(time.monotonic() - float(stamp), 0.0)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    @staticmethod
    def _format_age(age: Optional[float]) -> str:
        if age is None or math.isinf(age):
            return "n/a"
        return f"{age:.3f}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
