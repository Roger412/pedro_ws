"""
ROS 2 dashboard for the PEDRO STM32H7 L298N/PWM base.

This node does not talk to the STM32 serial port directly. It subscribes to
the topics published by serial_communication.py and serves a small Socket.IO
web dashboard that mirrors the latest ROS data.
"""

import math
import threading
import time
from importlib import resources
from typing import Any

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32, Int32MultiArray, String


ROBOT_STATES = {0: "IDLE", 1: "RUNNING", 2: "STOP", 3: "ESTOP"}


class PedroDashboardNode(Node):
    def __init__(self):
        super().__init__("pedro_dashboard_node")

        self.declare_parameter("enable_web_gui", True)
        self.declare_parameter("web_gui_port", 5000)
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("lost_timeout_s", 2.0)

        self._stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self._lost_timeout_s = float(self.get_parameter("lost_timeout_s").value)
        self._latest_lock = threading.Lock()
        self._last_raw_time: float | None = None
        self._sio = None
        self._latest = self._empty_telemetry()

        self.create_subscription(String, "stm32/raw", self._raw_cb, 10)
        self.create_subscription(TwistStamped, "stm32/cmd_setpoint", self._cmd_cb, 10)
        self.create_subscription(Imu, "imu/data", self._imu_cb, 10)
        self.create_subscription(Int32MultiArray, "stm32/encoders", self._enc_cb, 10)
        self.create_subscription(Float32MultiArray, "stm32/omegas", self._omega_cb, 10)
        self.create_subscription(Odometry, "odom", self._odom_cb, 10)
        self.create_subscription(Int32MultiArray, "stm32/pwm", self._pwm_cb, 10)
        self.create_subscription(Float32MultiArray, "stm32/ctrl_u", self._ctrl_u_cb, 10)
        self.create_subscription(Float32MultiArray, "stm32/errors", self._pose_err_cb, 10)
        self.create_subscription(Float32MultiArray, "stm32/u_errors", self._wheel_err_cb, 10)
        self.create_subscription(Int32, "stm32/robot_state", self._state_id_cb, 10)
        self.create_subscription(String, "stm32/robot_state_name", self._state_name_cb, 10)
        self.create_subscription(String, "stm32_debug", self._debug_cb, 10)

        self.create_timer(0.25, self._tick)

        if bool(self.get_parameter("enable_web_gui").value):
            self._start_web_server(int(self.get_parameter("web_gui_port").value))

        self.get_logger().info("PEDRO dashboard node ready.")

    def _start_web_server(self, port: int) -> None:
        try:
            from flask import Flask, render_template_string
            from flask_socketio import SocketIO
        except ImportError:
            self.get_logger().error(
                "enable_web_gui requires Flask and Flask-SocketIO "
                "(for example: pip install flask flask-socketio)"
            )
            return

        html = resources.files("serial_comm").joinpath("dashboard.html").read_text()
        app = Flask(__name__)
        app.config["SECRET_KEY"] = "pedro-dashboard"
        sio = SocketIO(
            app,
            cors_allowed_origins="*",
            async_mode="threading",
            logger=False,
            engineio_logger=False,
        )
        self._sio = sio

        @app.route("/")
        def index():
            return render_template_string(html)

        @sio.on("connect")
        def on_connect():
            with self._latest_lock:
                sio.emit("telemetry", dict(self._latest))

        def run_server():
            sio.run(app, host="0.0.0.0", port=port, use_reloader=False, log_output=False)

        threading.Thread(target=run_server, daemon=True).start()
        self.get_logger().info(f"PEDRO dashboard at http://localhost:{port}")

    def _raw_cb(self, msg: String) -> None:
        self._last_raw_time = time.monotonic()
        self._update(raw_preview=msg.data[:240])

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self._update(
            cmd=[
                self._finite(msg.twist.linear.x),
                self._finite(msg.twist.linear.y),
                self._finite(msg.twist.angular.z),
            ]
        )

    def _imu_cb(self, msg: Imu) -> None:
        roll, pitch, yaw = self._quat_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self._update(
            imu_rpy=[roll, pitch, yaw],
            imu_quat=[
                self._finite(msg.orientation.x),
                self._finite(msg.orientation.y),
                self._finite(msg.orientation.z),
                self._finite(msg.orientation.w),
            ],
            imu_gyro=[
                self._finite(msg.angular_velocity.x),
                self._finite(msg.angular_velocity.y),
                self._finite(msg.angular_velocity.z),
            ],
            imu_accel=[
                self._finite(msg.linear_acceleration.x),
                self._finite(msg.linear_acceleration.y),
                self._finite(msg.linear_acceleration.z),
            ],
        )

    def _enc_cb(self, msg: Int32MultiArray) -> None:
        self._update(encoders=self._pad_ints(msg.data, 4))

    def _omega_cb(self, msg: Float32MultiArray) -> None:
        self._update(omegas=self._pad_floats(msg.data, 4))

    def _odom_cb(self, msg: Odometry) -> None:
        self._update(
            odom=[
                self._finite(msg.pose.pose.position.x),
                self._finite(msg.pose.pose.position.y),
                self._yaw_from_quat(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ),
            ],
            odom_twist=[
                self._finite(msg.twist.twist.linear.x),
                self._finite(msg.twist.twist.linear.y),
                self._finite(msg.twist.twist.angular.z),
            ],
        )

    def _pwm_cb(self, msg: Int32MultiArray) -> None:
        self._update(pwm=self._pad_ints(msg.data, 4))

    def _ctrl_u_cb(self, msg: Float32MultiArray) -> None:
        self._update(ctrl_u=self._pad_floats(msg.data, 4))

    def _pose_err_cb(self, msg: Float32MultiArray) -> None:
        self._update(pose_error=self._pad_floats(msg.data, 3))

    def _wheel_err_cb(self, msg: Float32MultiArray) -> None:
        self._update(wheel_error=self._pad_floats(msg.data, 4))

    def _state_id_cb(self, msg: Int32) -> None:
        state_id = int(msg.data)
        self._update(
            robot_state=state_id,
            robot_state_name=ROBOT_STATES.get(state_id, f"UNKNOWN({state_id})"),
        )

    def _state_name_cb(self, msg: String) -> None:
        self._update(robot_state_name=msg.data)

    def _debug_cb(self, msg: String) -> None:
        flags = []
        text = msg.data.upper()
        for token in ("ESTOP", "STOP", "ERROR", "WATCHDOG", "TIMEOUT"):
            if token in text and token not in flags:
                flags.append(token)
        self._update(error_flags=flags)

    def _tick(self) -> None:
        self._update()

    def _update(self, **updates: Any) -> None:
        now = time.monotonic()
        with self._latest_lock:
            if self._last_raw_time is None:
                age_ms = None
            else:
                age_ms = int((now - self._last_raw_time) * 1000)
            self._latest.update(updates)
            self._latest["telemetry_age_ms"] = age_ms
            self._latest["connection_status"] = self._connection_status(age_ms)
            if self._latest["robot_state"] == 3 and "ESTOP" not in self._latest["error_flags"]:
                self._latest["error_flags"] = list(self._latest["error_flags"]) + ["ESTOP"]
            payload = dict(self._latest)

        if self._sio:
            self._sio.emit("telemetry", payload)

    def _connection_status(self, age_ms: int | None) -> str:
        if age_ms is None:
            return "UNKNOWN"
        age_s = age_ms / 1000.0
        if age_s <= self._stale_timeout_s:
            return "OK"
        if age_s <= self._lost_timeout_s:
            return "STALE"
        return "LOST"

    @staticmethod
    def _empty_telemetry() -> dict[str, Any]:
        return {
            "cmd": [0.0, 0.0, 0.0],
            "imu_rpy": [0.0, 0.0, 0.0],
            "imu_quat": [0.0, 0.0, 0.0, 1.0],
            "imu_gyro": [0.0, 0.0, 0.0],
            "imu_accel": [0.0, 0.0, 0.0],
            "encoders": [0, 0, 0, 0],
            "omegas": [0.0, 0.0, 0.0, 0.0],
            "odom": [0.0, 0.0, 0.0],
            "odom_twist": [0.0, 0.0, 0.0],
            "pwm": [0, 0, 0, 0],
            "ctrl_u": [0.0, 0.0, 0.0, 0.0],
            "pose_error": [0.0, 0.0, 0.0],
            "wheel_error": [0.0, 0.0, 0.0, 0.0],
            "robot_state": 0,
            "robot_state_name": "IDLE",
            "error_flags": [],
            "raw_preview": "",
            "telemetry_age_ms": None,
            "connection_status": "UNKNOWN",
        }

    @staticmethod
    def _finite(value: Any) -> float:
        try:
            out = float(value)
            return out if math.isfinite(out) else 0.0
        except (TypeError, ValueError):
            return 0.0

    @classmethod
    def _pad_floats(cls, values: Any, size: int) -> list[float]:
        out = [cls._finite(v) for v in list(values)[:size]]
        return out + [0.0] * (size - len(out))

    @staticmethod
    def _pad_ints(values: Any, size: int) -> list[int]:
        out = []
        for value in list(values)[:size]:
            try:
                out.append(int(value))
            except (TypeError, ValueError):
                out.append(0)
        return out + [0] * (size - len(out))

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @classmethod
    def _quat_to_rpy(cls, x: float, y: float, z: float, w: float) -> list[float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        yaw = cls._yaw_from_quat(x, y, z, w)
        return [roll, pitch, yaw]


def main(args=None):
    rclpy.init(args=args)
    node = PedroDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
