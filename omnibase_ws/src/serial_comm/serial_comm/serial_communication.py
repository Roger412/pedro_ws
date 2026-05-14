"""
ROS 2 serial bridge for STM32H7_PEDRO_OMNIBASE.

Wire format (USART3 @ 115200 8-N-1) is defined in
`firmware/STM32H7_PEDRO_OMNIBASE/status.md` sections
"FINAL UART COMMAND FORMAT" and "FINAL TELEMETRY FORMAT".

TX (PC -> STM32): 30 space-separated floats, \\r\\n terminated:
  x_desired y_desired phi_end d r
  u1_desired u2_desired u3_desired u4_desired
  x_Kp x_Ki x_Kd  y_Kp y_Ki y_Kd  phi_Kp phi_Ki phi_Kd
  u0_Kp u0_Ki u0_Kd  u1_Kp u1_Ki u1_Kd
  u2_Kp u2_Ki u2_Kd  u3_Kp u3_Ki u3_Kd

When the four u*_desired fields are zero and at least one of
x_desired/y_desired/phi_end is non-zero, the firmware interprets the
line as a body-frame twist (vx, vy, wz). Sending x_desired = -9999.0
latches ESTOP until MCU reset.

RX (STM32 -> PC): one line per ~100 Hz cycle, comma-separated
`key=value` pairs. See status.md for the complete key list. New keys
relative to the original PEDRO format: IMU_q*, IMU_w*, IMU_a*,
robot_state.
"""

import math
import re
import threading

import rclpy
import serial
from geometry_msgs.msg import Quaternion, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import (
    Float32,
    Float32MultiArray,
    Int32,
    Int32MultiArray,
    String,
)

ROBOT_STATE_NAMES = {0: "IDLE", 1: "RUNNING", 2: "STOP", 3: "ESTOP"}
ESTOP_CMD_KEY = -9999.0

# Telemetry key/value regex. Accepts optional sign, integers, decimals,
# and scientific notation so we don't silently drop floats the firmware
# may emit as 1e-3.
TELEM_KV_RE = re.compile(r'(\w+)=([-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)')


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')

        # ---- Parameters ----
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('tx_rate_hz', 10.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        # If true, /cmd_vel overrides parameter-driven setpoint (the
        # four u*_desired are zeroed and x/y/phi carry the twist).
        self.declare_parameter('use_cmd_vel', True)
        # Watchdog: if no /cmd_vel seen for this many seconds, the
        # bridge stops echoing the last twist and falls back to zeros.
        # The STM32 has its own 500 ms watchdog -- this is just to keep
        # the host side honest.
        self.declare_parameter('cmd_vel_timeout', 0.5)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud').value)
        tx_rate = float(self.get_parameter('tx_rate_hz').value)
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._imu_frame = self.get_parameter('imu_frame').value
        self._cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)

        self.ser = serial.Serial(port, baud, timeout=0.2)
        self._ser_lock = threading.Lock()

        # ---- Command-vector parameters ----
        self.pose_names = ['x', 'y', 'phi', 'd', 'r']
        self.velocity_names = ['u1_desired', 'u2_desired', 'u3_desired', 'u4_desired']
        self.gain_names = [
            'x_kp', 'x_ki', 'x_kd',
            'y_kp', 'y_ki', 'y_kd',
            'phi_kp', 'phi_ki', 'phi_kd',
            'u0_kp', 'u0_ki', 'u0_kd',
            'u1_kp', 'u1_ki', 'u1_kd',
            'u2_kp', 'u2_ki', 'u2_kd',
            'u3_kp', 'u3_ki', 'u3_kd',
        ]
        default_pose = [0.0, 0.0, 0.0, 0.195, 0.0762]   # x, y, phi, x_off (was d), wheel radius
        default_velocities = [0.0, 0.0, 0.0, 0.0]
        default_gains = [
            0.5, 0.1, 0.0,   # x
            0.5, 0.1, 0.0,   # y
            0.5, 0.1, 0.0,   # phi
            30.5, 0.5, 1.01, # u0
            30.5, 0.5, 1.01, # u1
            30.5, 0.5, 1.01, # u2
            30.5, 0.5, 1.01, # u3
        ]

        for i, name in enumerate(self.pose_names):
            self.declare_parameter(name, default_pose[i])
        for i, name in enumerate(self.velocity_names):
            self.declare_parameter(name, default_velocities[i])
        for i, name in enumerate(self.gain_names):
            self.declare_parameter(name, default_gains[i])

        # 30-float command vector laid out in the order the firmware sscanf expects.
        self.message_tx = [
            self.get_parameter(n).value
            for n in self.pose_names + self.velocity_names + self.gain_names
        ]
        self._tx_lock = threading.Lock()
        self.add_on_set_parameters_callback(self.update_parameters)

        # cmd_vel state
        self._cmd_vel_active = False
        self._cmd_vel_last_tick = 0.0
        self._cmd_vel_xyw = (0.0, 0.0, 0.0)

        # ---- Subscribers ----
        if bool(self.get_parameter('use_cmd_vel').value):
            self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        # ---- Publishers ----
        # Raw + human-readable
        self.state_pub      = self.create_publisher(String, 'stm32_debug', 10)
        self.raw_serial_pub = self.create_publisher(String, 'stm32/raw', 10)

        # Existing topics preserved (back-compat with anything already subscribed)
        self.pose_pub        = self.create_publisher(Float32MultiArray, 'stm32/pose', 10)
        self.imu_rpy_pub     = self.create_publisher(Float32MultiArray, 'stm32/imu', 10)
        self.omega_pub       = self.create_publisher(Float32MultiArray, 'stm32/omegas', 10)
        self.real_speeds_pub = self.create_publisher(Float32MultiArray, 'stm32/real_speeds', 10)
        self.odom_arr_pub    = self.create_publisher(Float32MultiArray, 'stm32/odom', 10)
        self.errors_pub      = self.create_publisher(Float32MultiArray, 'stm32/errors', 10)
        self.u_errors_pub    = self.create_publisher(Float32MultiArray, 'stm32/u_errors', 10)
        self.ctrl_speeds_pub = self.create_publisher(Float32MultiArray, 'stm32/ctrl_speeds', 10)
        self.ctrl_u_pub      = self.create_publisher(Float32MultiArray, 'stm32/ctrl_u', 10)
        self.pwm_pub         = self.create_publisher(Int32MultiArray,   'stm32/pwm', 10)
        self.ts_pub          = self.create_publisher(Int32MultiArray,   'stm32/timing', 10)
        self.encoders_pub    = self.create_publisher(Int32MultiArray,   'stm32/encoders', 10)
        self.x_pid_pub   = self.create_publisher(Float32MultiArray, 'stm32/pid_gains/x', 10)
        self.y_pid_pub   = self.create_publisher(Float32MultiArray, 'stm32/pid_gains/y', 10)
        self.phi_pid_pub = self.create_publisher(Float32MultiArray, 'stm32/pid_gains/phi', 10)
        self.u0_pid_pub  = self.create_publisher(Float32MultiArray, 'stm32/pid_gains/u0', 10)
        self.u1_pid_pub  = self.create_publisher(Float32MultiArray, 'stm32/pid_gains/u1', 10)
        self.u2_pid_pub  = self.create_publisher(Float32MultiArray, 'stm32/pid_gains/u2', 10)
        self.u3_pid_pub  = self.create_publisher(Float32MultiArray, 'stm32/pid_gains/u3', 10)

        # New standard-message topics
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_pub          = self.create_publisher(Imu, 'imu/data', sensor_qos)
        self.odom_pub         = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_echo_pub     = self.create_publisher(TwistStamped, 'stm32/cmd_setpoint', 10)
        self.state_id_pub     = self.create_publisher(Int32, 'stm32/robot_state', 10)
        self.state_name_pub   = self.create_publisher(String, 'stm32/robot_state_name', 10)
        # Per-wheel scalar fanout for plotting / RQT convenience
        self.per_wheel_enc_pubs = [
            self.create_publisher(Int32, f'stm32/encoder/wheel{i}', 10) for i in range(4)
        ]
        self.per_wheel_omega_pubs = [
            self.create_publisher(Float32, f'stm32/omega/wheel{i}', 10) for i in range(4)
        ]
        self.per_wheel_pwm_pubs = [
            self.create_publisher(Int32, f'stm32/pwm/wheel{i}', 10) for i in range(4)
        ]

        # ---- Reusable message buffers ----
        self.pose_data        = Float32MultiArray()
        self.imu_rpy_data     = Float32MultiArray()
        self.omega_data       = Float32MultiArray()
        self.real_speeds_data = Float32MultiArray()
        self.odom_arr_data    = Float32MultiArray()
        self.errors_data      = Float32MultiArray()
        self.u_errors_data    = Float32MultiArray()
        self.ctrl_speeds_data = Float32MultiArray()
        self.ctrl_u_data      = Float32MultiArray()
        self.pwm_data         = Int32MultiArray()
        self.ts_data          = Int32MultiArray()
        self.encoders_data    = Int32MultiArray()
        self.x_pid_data   = Float32MultiArray()
        self.y_pid_data   = Float32MultiArray()
        self.phi_pid_data = Float32MultiArray()
        self.u0_pid_data  = Float32MultiArray()
        self.u1_pid_data  = Float32MultiArray()
        self.u2_pid_data  = Float32MultiArray()
        self.u3_pid_data  = Float32MultiArray()

        # ---- Timer + RX thread ----
        timer_period = 1.0 / max(tx_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.send_message)
        self.count = 0

        self.recv_thread = threading.Thread(target=self.receiver, daemon=True)
        self.recv_thread.start()

    # ------------------------------------------------------------------
    # Parameter / cmd_vel handling
    # ------------------------------------------------------------------
    def update_parameters(self, params):
        with self._tx_lock:
            for param in params:
                if param.name in self.pose_names:
                    idx = self.pose_names.index(param.name)
                    self.message_tx[idx] = float(param.value)
                elif param.name in self.velocity_names:
                    idx = self.velocity_names.index(param.name)
                    self.message_tx[5 + idx] = float(param.value)
                elif param.name in self.gain_names:
                    idx = self.gain_names.index(param.name)
                    self.message_tx[9 + idx] = float(param.value)
        return SetParametersResult(successful=True)

    def cmd_vel_cb(self, msg: Twist):
        self._cmd_vel_xyw = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._cmd_vel_last_tick = self.get_clock().now().nanoseconds * 1e-9
        self._cmd_vel_active = True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def f(v) -> float:
        try:
            x = float(v)
            if not math.isfinite(x):
                return 0.0
            return x
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def i(v) -> int:
        try:
            x = int(float(v))
            return max(-2147483648, min(2147483647, x))
        except (TypeError, ValueError):
            return 0

    # ------------------------------------------------------------------
    # TX path
    # ------------------------------------------------------------------
    def send_message(self):
        with self._tx_lock:
            payload = list(self.message_tx)

        # cmd_vel takes priority when active and fresh.
        if self._cmd_vel_active:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._cmd_vel_last_tick > self._cmd_vel_timeout:
                # Stale: stop echoing the last twist.
                self._cmd_vel_active = False
                self._cmd_vel_xyw = (0.0, 0.0, 0.0)
            vx, vy, wz = self._cmd_vel_xyw
            # Twist mode requires u1..u4 == 0 per firmware semantics.
            payload[0] = float(vx)
            payload[1] = float(vy)
            payload[2] = float(wz)
            payload[5] = payload[6] = payload[7] = payload[8] = 0.0

        rounded = [round(float(v), 4) for v in payload]
        cmd_str = ' '.join(repr(v) if not float(v).is_integer() else f"{v:.1f}" for v in rounded)
        line = (cmd_str + "\r\n").encode()
        with self._ser_lock:
            try:
                self.ser.write(line)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"serial write failed: {e}")
                return

        self.count += 1
        if self.count % 50 == 1:
            self.get_logger().debug(f"TX[{self.count}] {cmd_str}")

    def send_estop(self):
        """Send the ESTOP sentinel (x_desired = -9999.0). Latches until MCU reset."""
        with self._tx_lock:
            self.message_tx[0] = ESTOP_CMD_KEY

    # ------------------------------------------------------------------
    # RX path
    # ------------------------------------------------------------------
    def receiver(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"serial read failed: {e}")
                continue
            if not line:
                continue

            self.raw_serial_pub.publish(String(data=line))

            try:
                data = {k: float(v) for k, v in TELEM_KV_RE.findall(line)}
                if not data:
                    continue
                self.publish_parsed(data)
            except Exception as e:
                self.get_logger().error(f"UART RX parse error: {e}")

    def publish_parsed(self, data: dict):
        g = data.get  # local alias for speed

        # ---- Back-compat Float32MultiArray / Int32MultiArray topics ----
        self.pose_data.data = [
            self.f(g('x_desired', 0)),
            self.f(g('y_desired', 0)),
            self.f(g('phi_desired', 0)),
            self.f(g('d', 0)),
            self.f(g('r', 0)),
        ]
        self.imu_rpy_data.data = [
            self.f(g('roll', 0)),
            self.f(g('pitch', 0)),
            self.f(g('yaw', 0)),
        ]
        self.encoders_data.data = [
            self.i(g('TIM1')), self.i(g('TIM2')),
            self.i(g('TIM4')), self.i(g('TIM8')),
        ]
        self.omega_data.data = [
            self.f(g('Enc_Wheel_Omega1', 0)),
            self.f(g('Enc_Wheel_Omega2', 0)),
            self.f(g('Enc_Wheel_Omega3', 0)),
            self.f(g('Enc_Wheel_Omega4', 0)),
        ]
        self.real_speeds_data.data = [
            self.f(g('Inertial_ang_vel_calc', 0)),
            self.f(g('Inertial_x_vel_calc', 0)),
            self.f(g('Inertial_y_vel_calc', 0)),
        ]
        self.odom_arr_data.data = [
            self.f(g('ODOM_phi', 0)),
            self.f(g('ODOM_x_pos', 0)),
            self.f(g('ODOM_y_pos', 0)),
        ]
        self.errors_data.data = [
            self.f(g('ODOM_Err_x', 0)),
            self.f(g('ODOM_Err_y', 0)),
            self.f(g('ODOM_Err_phi', 0)),
        ]
        self.u_errors_data.data = [
            self.f(g('U_Err_1', 0)), self.f(g('U_Err_2', 0)),
            self.f(g('U_Err_3', 0)), self.f(g('U_Err_4', 0)),
        ]
        self.ctrl_speeds_data.data = [
            self.f(g('Ctrl_Inertial_x_dot', 0)),
            self.f(g('Ctrl_Inertial_y_dot', 0)),
            self.f(g('Ctrl_Inertial_phi_dot', 0)),
        ]
        self.ctrl_u_data.data = [
            self.f(g('Ctrl_necc_u1', 0)), self.f(g('Ctrl_necc_u2', 0)),
            self.f(g('Ctrl_necc_u3', 0)), self.f(g('Ctrl_necc_u4', 0)),
        ]
        self.pwm_data.data = [
            self.i(g('Ctrl_duty_u1')), self.i(g('Ctrl_duty_u2')),
            self.i(g('Ctrl_duty_u3')), self.i(g('Ctrl_duty_u4')),
        ]
        self.ts_data.data = [
            self.i(g('ts_current')), self.i(g('ts_previous')), self.i(g('ts_delta')),
        ]
        self.x_pid_data.data   = [self.f(g('xKp', 0)), self.f(g('xKi', 0)), self.f(g('xKd', 0))]
        self.y_pid_data.data   = [self.f(g('yKp', 0)), self.f(g('yKi', 0)), self.f(g('yKd', 0))]
        self.phi_pid_data.data = [self.f(g('phiKp', 0)), self.f(g('phiKi', 0)), self.f(g('phiKd', 0))]
        self.u0_pid_data.data  = [self.f(g('u0Kp', 0)), self.f(g('u0Ki', 0)), self.f(g('u0Kd', 0))]
        self.u1_pid_data.data  = [self.f(g('u1Kp', 0)), self.f(g('u1Ki', 0)), self.f(g('u1Kd', 0))]
        self.u2_pid_data.data  = [self.f(g('u2Kp', 0)), self.f(g('u2Ki', 0)), self.f(g('u2Kd', 0))]
        self.u3_pid_data.data  = [self.f(g('u3Kp', 0)), self.f(g('u3Ki', 0)), self.f(g('u3Kd', 0))]

        self.pose_pub.publish(self.pose_data)
        self.imu_rpy_pub.publish(self.imu_rpy_data)
        self.encoders_pub.publish(self.encoders_data)
        self.omega_pub.publish(self.omega_data)
        self.real_speeds_pub.publish(self.real_speeds_data)
        self.odom_arr_pub.publish(self.odom_arr_data)
        self.errors_pub.publish(self.errors_data)
        self.u_errors_pub.publish(self.u_errors_data)
        self.ctrl_speeds_pub.publish(self.ctrl_speeds_data)
        self.ctrl_u_pub.publish(self.ctrl_u_data)
        self.pwm_pub.publish(self.pwm_data)
        self.ts_pub.publish(self.ts_data)
        self.x_pid_pub.publish(self.x_pid_data)
        self.y_pid_pub.publish(self.y_pid_data)
        self.phi_pid_pub.publish(self.phi_pid_data)
        self.u0_pid_pub.publish(self.u0_pid_data)
        self.u1_pid_pub.publish(self.u1_pid_data)
        self.u2_pid_pub.publish(self.u2_pid_data)
        self.u3_pid_pub.publish(self.u3_pid_data)

        # ---- Per-wheel scalar fanout ----
        for i, key in enumerate(('TIM1', 'TIM2', 'TIM4', 'TIM8')):
            self.per_wheel_enc_pubs[i].publish(Int32(data=self.i(g(key))))
        for i in range(4):
            self.per_wheel_omega_pubs[i].publish(
                Float32(data=self.f(g(f'Enc_Wheel_Omega{i + 1}', 0)))
            )
            self.per_wheel_pwm_pubs[i].publish(
                Int32(data=self.i(g(f'Ctrl_duty_u{i + 1}')))
            )

        # ---- IMU (sensor_msgs/Imu) ----
        # The firmware emits IMU_q* in ROS (x, y, z, w) order; orientation
        # is the SH2 rotation vector. Linear acceleration is gravity-removed.
        if any(k in data for k in ('IMU_qx', 'IMU_qy', 'IMU_qz', 'IMU_qw')):
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = self._imu_frame
            imu.orientation.x = self.f(g('IMU_qx', 0))
            imu.orientation.y = self.f(g('IMU_qy', 0))
            imu.orientation.z = self.f(g('IMU_qz', 0))
            imu.orientation.w = self.f(g('IMU_qw', 1.0))
            imu.angular_velocity.x = self.f(g('IMU_wx', 0))
            imu.angular_velocity.y = self.f(g('IMU_wy', 0))
            imu.angular_velocity.z = self.f(g('IMU_wz', 0))
            imu.linear_acceleration.x = self.f(g('IMU_ax', 0))
            imu.linear_acceleration.y = self.f(g('IMU_ay', 0))
            imu.linear_acceleration.z = self.f(g('IMU_az', 0))
            # Covariance unknown; first element = -1 signals "unknown"
            # per sensor_msgs/Imu convention.
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(imu)

        # ---- Odometry (nav_msgs/Odometry) ----
        if any(k in data for k in ('ODOM_x_pos', 'ODOM_y_pos', 'ODOM_phi')):
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = self._odom_frame
            odom.child_frame_id = self._base_frame
            odom.pose.pose.position.x = self.f(g('ODOM_x_pos', 0))
            odom.pose.pose.position.y = self.f(g('ODOM_y_pos', 0))
            odom.pose.pose.position.z = 0.0
            phi = self.f(g('ODOM_phi', 0))
            odom.pose.pose.orientation = yaw_to_quaternion(phi)
            # The firmware emits world-frame linear velocities
            # (`Inertial_{x,y}_vel_calc` = q_dot[1], q_dot[2] in
            # status.md §11). nav_msgs/Odometry.twist is conventionally
            # expressed in child_frame_id (base_link), so rotate
            # world -> body by -phi.
            vx_w = self.f(g('Inertial_x_vel_calc', 0))
            vy_w = self.f(g('Inertial_y_vel_calc', 0))
            c, s = math.cos(phi), math.sin(phi)
            odom.twist.twist.linear.x =  c * vx_w + s * vy_w
            odom.twist.twist.linear.y = -s * vx_w + c * vy_w
            odom.twist.twist.angular.z = self.f(g('Inertial_ang_vel_calc', 0))
            self.odom_pub.publish(odom)

        # ---- Commanded twist echo ----
        cmd_echo = TwistStamped()
        cmd_echo.header.stamp = self.get_clock().now().to_msg()
        cmd_echo.header.frame_id = self._base_frame
        cmd_echo.twist.linear.x = self.f(g('x_desired', 0))
        cmd_echo.twist.linear.y = self.f(g('y_desired', 0))
        cmd_echo.twist.angular.z = self.f(g('phi_desired', 0))
        self.cmd_echo_pub.publish(cmd_echo)

        # ---- Robot state ----
        if 'robot_state' in data:
            state_id = self.i(g('robot_state', 0))
            self.state_id_pub.publish(Int32(data=state_id))
            self.state_name_pub.publish(
                String(data=ROBOT_STATE_NAMES.get(state_id, f"UNKNOWN({state_id})"))
            )

        # ---- Human-readable debug summary ----
        state_id = self.i(g('robot_state', -1))
        state_name = ROBOT_STATE_NAMES.get(state_id, "?")
        summary = (
            "Parsed Robot State:\n"
            f"    Desired:  x={g('x_desired', 0):.2f} y={g('y_desired', 0):.2f} "
            f"phi={g('phi_desired', 0):.2f} d={g('d', 0):.2f} r={g('r', 0):.2f}\n"
            f"    IMU rpy:  roll={g('roll', 0):.2f} pitch={g('pitch', 0):.2f} yaw={g('yaw', 0):.2f}\n"
            f"    IMU quat: x={g('IMU_qx', 0):.3f} y={g('IMU_qy', 0):.3f} "
            f"z={g('IMU_qz', 0):.3f} w={g('IMU_qw', 0):.3f}\n"
            f"    IMU gyro: {g('IMU_wx', 0):.2f} {g('IMU_wy', 0):.2f} {g('IMU_wz', 0):.2f}\n"
            f"    IMU acc:  {g('IMU_ax', 0):.2f} {g('IMU_ay', 0):.2f} {g('IMU_az', 0):.2f}\n"
            f"    Enc raw:  {int(g('TIM1', 0))} {int(g('TIM2', 0))} "
            f"{int(g('TIM4', 0))} {int(g('TIM8', 0))}\n"
            f"    Omegas:   {g('Enc_Wheel_Omega1', 0):.2f} {g('Enc_Wheel_Omega2', 0):.2f} "
            f"{g('Enc_Wheel_Omega3', 0):.2f} {g('Enc_Wheel_Omega4', 0):.2f}\n"
            f"    Odom:     phi={g('ODOM_phi', 0):.2f} x={g('ODOM_x_pos', 0):.2f} "
            f"y={g('ODOM_y_pos', 0):.2f}\n"
            f"    PoseErr:  dx={g('ODOM_Err_x', 0):.2f} dy={g('ODOM_Err_y', 0):.2f} "
            f"dphi={g('ODOM_Err_phi', 0):.2f}\n"
            f"    U_err:    {g('U_Err_1', 0):.2f} {g('U_Err_2', 0):.2f} "
            f"{g('U_Err_3', 0):.2f} {g('U_Err_4', 0):.2f}\n"
            f"    PWM duty: {int(g('Ctrl_duty_u1', 0))} {int(g('Ctrl_duty_u2', 0))} "
            f"{int(g('Ctrl_duty_u3', 0))} {int(g('Ctrl_duty_u4', 0))}\n"
            f"    State:    {state_name} ({state_id})  "
            f"ts={int(g('ts_current', 0))} dt={int(g('ts_delta', 0))}\n"
        )
        self.state_pub.publish(String(data=summary))


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
