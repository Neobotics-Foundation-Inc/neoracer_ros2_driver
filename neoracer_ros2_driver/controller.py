#!/usr/bin/env python3

"""
ESP32 serial bridge for the Seeed OSRbot board on the Neoracer.

A single node that owns the USB-serial link to the ESP32, which carries the
IMU, wheel odometry, FlySky RC receiver, and the motor/steering ESC. It:

- subscribes to ``/motor`` (ackermann_msgs/AckermannDriveStamped, normalized
  [-1, 1]) from the throttle node and writes the firmware drive command
  ``v <speed_mps> <steer_deg>``,
- publishes ``/imu/fused`` (sensor_msgs/Imu), ``/odom`` (nav_msgs/Odometry),
  ``/mag`` (sensor_msgs/MagneticField), and ``/battery/state``
  (sensor_msgs/BatteryState, the racecar_neo name; ``/battery`` carries the
  same message for one more release) parsed from the board,
- publishes ``/joy`` (sensor_msgs/Joy) synthesized from the FlySky RC channels,
  so the same software pipeline (gamepad -> mux -> throttle) and the student
  controller API both work without a USB gamepad, and
- republishes ``/encoder/speed``, ``/battery/voltage`` (std_msgs/Float32) and
  ``/rc/channels`` (std_msgs/Float32MultiArray), the scalar sensor topics
  ``rc.physics`` reads on the racecar_neo platform, and ``/rc/link``
  (std_msgs/Bool), whether the transmitter is present, and
- publishes ``/odom/integrated`` (nav_msgs/Odometry) from each ``s`` frame,
  either the ESP32's own pose or odometry.py's integration of the body speed
  and the gyro yaw rate (``odometry.source``, config/odom.yaml).

The exact FlySky channel order depends on the transmitter mixer and is exposed
entirely as ROS parameters (see config/controller.yaml) for on-car tuning.

Dependencies: pyserial (``serial``), controller_lib, odometry.
"""

from collections.abc import Sequence
import math

# ===== IMPORT ROS2 CORE ======
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# ===== IMPORT ROS2 MESSAGE TYPES =====
from sensor_msgs.msg import BatteryState, Imu, Joy, MagneticField
from std_msgs.msg import Bool, Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ackermann_msgs.msg import AckermannDriveStamped

# ===== IMPORT OTHER DEPENDENCIES ======
import serial
import sys
import threading
import time
from . import controller_lib
from .odometry import OdomIntegrator, OdomState, yaw_quaternion

_INT_ARRAY = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)

# Where /odom/integrated takes its pose from (odometry.source).
ODOM_SOURCES = ('firmware', 'integrated')
# Odometry: logged when the gyro bias estimate moves by more than this.
ODOM_BIAS_LOG_STEP = math.radians(0.05)


def declare_float_param(node: Node, name: str, default: float) -> float:
    """
    Declare a double parameter that tolerates integer YAML values.

    A hand-edited ``steering_trim_deg: -5`` arrives from the YAML as an int,
    and a strictly-typed double parameter kills the node at startup over the
    missing ``.0``. Declare dynamically typed and cast instead; fall back to
    the default (loudly) if the value is not a number at all.
    """
    value = node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
    try:
        return float(value)
    except (TypeError, ValueError):
        node.get_logger().error(
            f"parameter '{name}' must be a number, got {value!r}; " f"using the default {default}"
        )
        return float(default)


def build_battery_state(volts: float, v_min: float, v_max: float) -> BatteryState:
    """
    Build a BatteryState from one pack voltage reading.

    The OSRbot base has no current shunt, so current is NaN (ROS counts
    discharge as negative; nothing here has a sign to give). percentage is the
    3S charge fraction between v_min and v_max, which the dashboard shows. The
    frame arrives at about 0.5 Hz, so there is nothing to average.
    """
    msg = BatteryState()
    msg.voltage = float(volts)
    msg.current = math.nan
    msg.charge = msg.capacity = msg.design_capacity = math.nan
    msg.temperature = math.nan
    msg.percentage = controller_lib.battery_fraction(volts, v_min, v_max)
    msg.present = True
    msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
    msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    return msg


def build_integrated_odom(
    state: OdomState, odom_frame: str, base_frame: str
) -> tuple[TransformStamped, Odometry]:
    """Return the odom -> base_link transform and its Odometry message for a pose."""
    assert state.t is not None
    stamp = Time(nanoseconds=round(state.t * 1e9)).to_msg()
    qx, qy, qz, qw = yaw_quaternion(state.theta)

    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = odom_frame
    tf.child_frame_id = base_frame
    tf.transform.translation.x = state.x
    tf.transform.translation.y = state.y
    tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
    tf.transform.rotation.z, tf.transform.rotation.w = qz, qw

    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = odom_frame
    odom.child_frame_id = base_frame
    odom.pose.pose.position.x = state.x
    odom.pose.pose.position.y = state.y
    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
    odom.twist.twist.linear.x = state.v
    odom.twist.twist.angular.z = state.omega
    return tf, odom


class ControllerNode(Node):
    """ROS2 node bridging the ESP32 serial firmware to the ROS graph."""

    def __init__(self):
        super().__init__('controller_node')

        # ===== DECLARE PARAMETERS =====
        # Serial link. /dev/osrbot_base is a stable udev symlink to the ESP32.
        # Baud is irrelevant for the native USB-CDC port but kept for clarity.
        self.port_name = self.declare_parameter('port_name', '/dev/osrbot_base').value
        self.baud_rate = self.declare_parameter('baud_rate', 115200).value

        # Drive command mapping (normalized /motor -> firmware m/s + degrees).
        self.max_speed_mps = declare_float_param(self, 'max_speed_mps', 6.0)
        self.max_steering_angle_deg = declare_float_param(self, 'max_steering_angle_deg', 30.0)
        self.steering_trim_deg = declare_float_param(self, 'steering_trim_deg', 0.0)

        # Frames.
        self.imu_frame = self.declare_parameter('imu_frame', 'imu_link').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # Broadcast odom -> base_link alongside /odom. SLAM and Nav2
        # need the transform, and this node is the single odometry source
        # (the vendor chassis node broadcast it the same way).
        self.publish_tf = self.declare_parameter('publish_tf', True).value

        # IMU axes: /imu/fused and /mag are REP-103 body axes (x forward, y
        # left, z up). out[i] = sign[i] * in[order[i]] on the firmware's
        # values; identity on the NeoRacer, see config/controller.yaml.
        self._ag_order, self._ag_sign = self._declare_axis_remap('imu.accel_gyro_axis')
        self._mag_order, self._mag_sign = self._declare_axis_remap('imu.mag_axis')
        # The firmware fuses its orientation in the board's own axes, so a
        # remap that moves them leaves that orientation in the wrong frame.
        self._orientation_valid = controller_lib.is_identity_remap(self._ag_order, self._ag_sign)
        if not self._orientation_valid:
            self.get_logger().warn(
                'imu.accel_gyro_axis remap is not the identity; /imu/fused marks the '
                "firmware's orientation unknown (orientation_covariance[0] = -1)"
            )

        # /mag feeds rc.physics.get_magnetic_field(); off only on a car
        # whose firmware does not emit the `m` frame.
        self.publish_mag = self.declare_parameter('publish_mag', True).value
        self.publish_joy = self.declare_parameter('publish_joy', True).value

        # FlySky RC -> Joy mapping (all tunable on-hardware; see config/controller.yaml).
        self._joy_cfg = self._declare_joy_params()

        # ===== INITIALIZE SERIAL PORT =====
        try:
            self.serial = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'[DEBUG] Connected to: {self.port_name}')
        except serial.SerialException as e:
            self.get_logger().fatal(f"[ERROR] Could not connect to device '{self.port_name}': {e}")
            rclpy.shutdown()
            return

        # ===== SET UP SUBSCRIBERS =====
        # /motor is the throttle node output (normalized, BEST_EFFORT).
        self.motor_sub = self.create_subscription(
            AckermannDriveStamped, '/motor', self.motor_callback, qos_profile_sensor_data
        )

        # ===== SET UP PUBLISHERS =====
        # /imu/fused and /odom are RELIABLE: the EKF (robot_localization) and the
        # complementary filter subscribe reliable, and a reliable publisher
        # still satisfies every BEST_EFFORT subscriber (student library,
        # dashboard, rviz).
        self.imu_pub = self.create_publisher(Imu, '/imu/fused', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.battery_pub = self.create_publisher(BatteryState, '/battery', qos_profile_sensor_data)
        # /battery/state is the racecar_neo name, which library v0.9.0-bot reads;
        # /battery carries the same message for one more release for the dashboard.
        self.battery_state_pub = self.create_publisher(BatteryState, '/battery/state', 10)
        # The scalar racecar_neo sensor topics the student library reads. They
        # carry no information /battery, /odom, and /joy do not already hold;
        # they exist so rc.physics reads the same names on both platforms.
        # /battery/current has no counterpart: the OSRbot base has no current
        # shunt, so nothing publishes it and rc.physics raises there.
        self.voltage_pub = self.create_publisher(
            Float32, '/battery/voltage', qos_profile_sensor_data
        )
        self.encoder_pub = self.create_publisher(
            Float32, '/encoder/speed', qos_profile_sensor_data
        )
        self.rc_pub = self.create_publisher(
            Float32MultiArray, '/rc/channels', qos_profile_sensor_data
        )
        # Link state from the raw SBUS counts; see controller_lib.rc_link_up.
        self.rc_link_pub = self.create_publisher(Bool, '/rc/link', qos_profile_sensor_data)
        self._unknown_tags = set()
        # /joy is RELIABLE so the student controller API (a RELIABLE subscriber)
        # connects; BEST_EFFORT pipeline subscribers accept a reliable publisher.
        self.joy_pub = self.create_publisher(Joy, '/joy', 10) if self.publish_joy else None
        self.mag_pub = (
            self.create_publisher(MagneticField, '/mag', qos_profile_sensor_data)
            if self.publish_mag
            else None
        )
        self._setup_odometry()

        # ===== STATE + SERIAL READ THREAD =====
        self.last_cmd_time = self.get_clock().now()
        self.serial_lock = threading.Lock()
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info('[DEBUG] Controller node initialized; entering main loop')

    def _declare_axis_remap(self, prefix):
        """Declare <prefix>_order and <prefix>_sign; return them, or the identity if invalid."""
        dynamic = ParameterDescriptor(dynamic_typing=True)
        order = self.declare_parameter(
            f'{prefix}_order', list(controller_lib.IDENTITY_AXIS_ORDER), dynamic
        ).value
        sign = self.declare_parameter(
            f'{prefix}_sign', list(controller_lib.IDENTITY_AXIS_SIGN), dynamic
        ).value
        problem = controller_lib.axis_remap_problem(order, sign)
        if problem:
            self.get_logger().error(f'{prefix}: {problem}; using the identity')
            return list(controller_lib.IDENTITY_AXIS_ORDER), list(
                controller_lib.IDENTITY_AXIS_SIGN
            )
        return [int(i) for i in order], [float(s) for s in sign]

    def _setup_odometry(self) -> None:
        """
        Declare the odometry.* parameters (config/odom.yaml) and set up /odom/integrated.

        A bad value is logged and replaced by its default rather than stopping
        the node, because this node also carries the drive commands. A wrong
        type, for example 2.0 for the axis, still stops the node at startup.
        """
        enable = bool(self.declare_parameter('odometry.enable', True).value)
        source = str(self.declare_parameter('odometry.source', 'firmware').value)
        # Off by default: the autonomy launch's EKF publishes odom -> base_link.
        publish_tf = bool(self.declare_parameter('odometry.publish_tf', False).value)
        topic = str(self.declare_parameter('odometry.topic', '/odom/integrated').value)
        axis = self.declare_parameter('odometry.yaw_rate_axis', 2).value
        yaw_rate_sign = declare_float_param(self, 'odometry.yaw_rate_sign', 1.0)
        speed_scale = declare_float_param(self, 'odometry.speed_scale', 1.0)
        yaw_rate_scale = declare_float_param(self, 'odometry.yaw_rate_scale', 1.0)
        rate_hz = declare_float_param(self, 'odometry.publish_rate_hz', 50.0)
        stale_sec = declare_float_param(self, 'odometry.stale_input_sec', 0.25)
        zero_hold_sec = declare_float_param(self, 'odometry.zero_hold_sec', 1.0)
        zero_max_rate_dps = declare_float_param(self, 'odometry.zero_max_rate_dps', 2.0)
        bias_tau_sec = declare_float_param(self, 'odometry.bias_tau_sec', 30.0)
        turn_gain = declare_float_param(self, 'odometry.turn_gain', 0.0)
        wheelbase_m = declare_float_param(self, 'odometry.wheelbase_m', 0.28764)
        max_steer_deg = declare_float_param(self, 'odometry.max_steer_deg', 30.0)

        self._odometry: OdomIntegrator | None = None
        self._pub_odom_integrated = None
        self._odom_tf = None
        self._odom_frames_warned = False
        if not enable:
            return
        if source not in ODOM_SOURCES:
            self.get_logger().error(
                f'odometry.source must be one of {ODOM_SOURCES}, got {source!r}; using firmware'
            )
            source = 'firmware'
        if axis not in (0, 1, 2):
            self.get_logger().error(
                f'odometry.yaw_rate_axis must be 0, 1 or 2, got {axis}; using 2'
            )
            axis = 2
        if rate_hz <= 0.0:
            self.get_logger().error('odometry.publish_rate_hz must be positive; using 50')
            rate_hz = 50.0
        if stale_sec <= 0.0:
            self.get_logger().error('odometry.stale_input_sec must be positive; using 0.25')
            stale_sec = 0.25
        if bias_tau_sec <= 0.0:
            self.get_logger().error('odometry.bias_tau_sec must be positive; using 30')
            bias_tau_sec = 30.0
        if zero_hold_sec < 0.0:
            self.get_logger().error('odometry.zero_hold_sec must not be negative; using 1')
            zero_hold_sec = 1.0
        self._yaw_axis = int(axis)
        self._yaw_gain = yaw_rate_sign * yaw_rate_scale
        self._speed_scale = speed_scale
        if source == 'integrated':
            self._odometry = OdomIntegrator(
                zero_hold_sec=zero_hold_sec,
                zero_max_rate=math.radians(zero_max_rate_dps),
                bias_tau_sec=bias_tau_sec,
                stale_sec=stale_sec,
                turn_gain=turn_gain,
                wheelbase=wheelbase_m,
                max_steer=math.radians(max_steer_deg),
            )
        # Published from the serial reader thread, not a timer, on a running
        # deadline at publish_rate_hz.
        self._odom_period = 1.0 / rate_hz
        self._odom_next_pub: float | None = None
        self._odom_gaps_logged = 0
        self._odom_bias_logged = 0.0
        self._pub_odom_integrated = self.create_publisher(Odometry, topic, qos_profile_sensor_data)
        if publish_tf:
            self._odom_tf = TransformBroadcaster(self)
            if self.tf_broadcaster is not None:
                # One writer for odom -> base_link: the firmware /odom TF steps aside.
                self.get_logger().warn(
                    'publish_tf and odometry.publish_tf are both on; '
                    f'odom -> base_link comes from {topic} only'
                )
                self.tf_broadcaster = None
        self.get_logger().info(
            f"{topic} from the {source} pose; TF odom -> base_link {'on' if publish_tf else 'off'}"
        )

    def _declare_joy_params(self):
        """Declare the FlySky->Joy mapping parameters and return them as a dict."""
        p = self.declare_parameter
        cfg = {
            'rc_min': p('rc_min', 1000).value,
            'rc_center': p('rc_center', 1500).value,
            'rc_max': p('rc_max', 2000).value,
            'rc_deadband': p('rc_deadband', 0.05).value,
            'rc_failsafe_below': p('rc_failsafe_below', 500).value,
            'num_axes': p('joy_num_axes', 6).value,
            'num_buttons': p('joy_num_buttons', 11).value,
            'trigger_axes': list(p('joy_trigger_axes', [2, 5]).value),
            'throttle_axis': p('throttle_axis', 1).value,
            'throttle_channel': p('throttle_channel', 2).value,
            'throttle_sign': p('throttle_sign', 1).value,
            'steering_axis': p('steering_axis', 3).value,
            'steering_channel': p('steering_channel', 0).value,
            'steering_sign': p('steering_sign', 1).value,
            'mode_channel': p('mode_channel', 4).value,
            'mode_mid_thresh': p('mode_mid_thresh', -0.5).value,
            'mode_high_thresh': p('mode_high_thresh', 0.5).value,
            'mode_idle_button': p('mode_idle_button', -1).value,
            'mode_manual_button': p('mode_manual_button', 4).value,
            'mode_autonomy_button': p('mode_autonomy_button', 5).value,
            'aux_button_channels': list(p('aux_button_channels', [], _INT_ARRAY).value or []),
            'aux_button_indices': list(p('aux_button_indices', [], _INT_ARRAY).value or []),
            'aux_button_thresh': p('aux_button_thresh', 0.5).value,
        }
        return cfg

    # Firmware lines that are expected but carry nothing we publish: admin
    # chatter (FW/DIAG/LINK banners, link acks).
    _IGNORED_EXACT = frozenset({'link', 'OK', 'ERROR'})
    _IGNORED_PREFIXES = ('FW', 'DIAG', 'LINK')

    # 3S LiPo bounds for the reported charge fraction.
    _BATTERY_V_MIN = controller_lib.BATTERY_3S_V_MIN
    _BATTERY_V_MAX = controller_lib.BATTERY_3S_V_MAX

    def read_serial_loop(self):
        """Read incoming serial lines and fan them out to the sensor topics."""
        while rclpy.ok():
            if self.serial.in_waiting > 0:
                try:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        try:
                            data, tag = controller_lib.parse_serial_data(line)
                            if tag is None:
                                self._note_unknown_line(line)
                            else:
                                self._dispatch(data, tag)
                        except (ValueError, IndexError) as e:
                            self.get_logger().warn(
                                f'Unable to parse message: [{line}], reason: {e}'
                            )
                except serial.SerialException:
                    self.get_logger().error('Serial Exception Error')
                    break
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f'Could not decode message: {e}')
            else:
                time.sleep(0.005)  # ~200 Hz idle poll

    def _note_unknown_line(self, line):
        """
        Warn once per unknown frame tag.

        A firmware protocol change stays visible in the log instead of being
        silently binned (the V1.1 's' frame went unnoticed exactly this way).
        """
        tag = line.split(None, 1)[0]
        if tag in self._IGNORED_EXACT or tag.startswith(self._IGNORED_PREFIXES):
            return
        if tag not in self._unknown_tags:
            self._unknown_tags.add(tag)
            self.get_logger().warn(
                f"Unknown serial frame tag '{tag}' (first line: [{line}]); "
                "firmware protocol may be newer than this driver"
            )

    def _dispatch(self, data, tag):
        """Route a parsed serial record to the matching publisher."""
        if data is None:
            return
        if tag == 's':
            # V1.1 firmware state frame carries both sensor sets.
            imu = self.pub_imu(data)
            self.pub_odom(data)
            self.pub_encoder(data)
            self.pub_odom_integrated(data, imu)
        elif tag == 'i':
            self.pub_imu(data)
        elif tag == 'o':
            self.pub_odom(data)
            self.pub_encoder(data)
            self._note_split_frames()
        elif tag == 'r':
            if self.joy_pub is not None:
                self.pub_joy(data)
            self.pub_rc(data)
            self.pub_rc_link(data)
        elif tag == 'm' and self.mag_pub is not None:
            self.pub_mag(data)
        elif tag == 'b':
            self.pub_battery(data)

    def pub_battery(self, data):
        """Publish the pack voltage from the firmware's battery frame on every battery topic."""
        msg = build_battery_state(data['voltage'], self._BATTERY_V_MIN, self._BATTERY_V_MAX)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.battery_state_pub.publish(msg)
        self.battery_pub.publish(msg)
        self.voltage_pub.publish(Float32(data=msg.voltage))

    def pub_encoder(self, data):
        """Publish forward speed (m/s) from a parsed state or odometry record."""
        # Signed body-frame x velocity, so reverse reads negative. The OSRbot
        # firmware integrates this from the same wheel encoder the Teensy
        # reports on the reference platform.
        self.encoder_pub.publish(Float32(data=float(data['v_x'])))

    def pub_imu(self, data):
        """Publish an Imu message from a parsed ``i`` or ``s`` record, and return it."""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame

        imu_msg.orientation.x = data['q_x']
        imu_msg.orientation.y = data['q_y']
        imu_msg.orientation.z = data['q_z']
        imu_msg.orientation.w = data['q_w']

        if not self._orientation_valid:
            imu_msg.orientation_covariance[0] = -1.0

        accel = controller_lib.remap_axes(
            (data['a_x'], data['a_y'], data['a_z']), self._ag_order, self._ag_sign
        )
        gyro = controller_lib.remap_axes(
            (data['g_x'], data['g_y'], data['g_z']), self._ag_order, self._ag_sign
        )
        a = imu_msg.linear_acceleration
        a.x, a.y, a.z = accel
        g = imu_msg.angular_velocity
        g.x, g.y, g.z = gyro

        self.imu_pub.publish(imu_msg)
        return imu_msg

    def pub_odom(self, data):
        """Publish an Odometry message from a parsed ``o`` record."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = data['p_x']
        odom_msg.pose.pose.position.y = data['p_y']
        odom_msg.pose.pose.position.z = data['p_z']

        q = controller_lib.quaternion_from_euler(0, 0, data['yaw'])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = data['v_x']
        odom_msg.twist.twist.linear.y = data['v_y']
        odom_msg.twist.twist.linear.z = data['v_z']
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = data['p_x']
            t.transform.translation.y = data['p_y']
            t.transform.translation.z = data['p_z']
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

    def pub_odom_integrated(self, data, imu):
        """
        Feed /odom/integrated from an ``s`` record and the /imu/fused message made from it.

        The speed is the frame's body speed vx and the yaw rate is the
        published gyro, after the REP-103 remap, on one stamp.
        """
        if self._pub_odom_integrated is None:
            return
        stamp_ns = imu.header.stamp.sec * 10**9 + imu.header.stamp.nanosec
        g = imu.angular_velocity
        gyro = (g.x, g.y, g.z)
        if self._odometry is not None:
            self._integrate_odometry(stamp_ns, float(data['v_x']), gyro)
        else:
            self._firmware_odometry(stamp_ns, data, gyro)

    def _integrate_odometry(self, stamp_ns: int, speed: float, gyro: Sequence[float]) -> None:
        assert self._odometry is not None
        omega = float(gyro[self._yaw_axis]) * self._yaw_gain
        t = stamp_ns * 1e-9
        self._odometry.step(t, speed * self._speed_scale, omega)
        if self._odometry_due(t):
            self._publish_odometry(self._odometry.state)
        gaps, bias = self._odometry.gaps, self._odometry.bias
        if gaps != self._odom_gaps_logged:
            self._odom_gaps_logged = gaps
            self.get_logger().warn(
                f'Odometry input gap over {self._odometry.stale_sec:g} s; pose held '
                f'({gaps} so far)'
            )
        if abs(bias - self._odom_bias_logged) > ODOM_BIAS_LOG_STEP:
            self._odom_bias_logged = bias
            self.get_logger().info(f'Gyro bias estimate {math.degrees(bias):+.3f} deg/s')

    def _firmware_odometry(self, stamp_ns: int, data, gyro: Sequence[float]) -> None:
        """Publish the ESP32's own pose, with the gyro yaw rate unscaled (source firmware)."""
        t = stamp_ns * 1e-9
        if self._odometry_due(t):
            state = OdomState(
                x=float(data['p_x']),
                y=float(data['p_y']),
                theta=float(data['yaw']),
                v=float(data['v_x']),
                omega=float(gyro[self._yaw_axis]),
                t=t,
            )
            self._publish_odometry(state)

    def _odometry_due(self, t: float) -> bool:
        """
        Return whether a sample at time t is published, and move the deadline.

        A running deadline holds the average at the configured rate; after a
        gap it restarts from now rather than publishing a burst.
        """
        due = self._odom_next_pub
        if due is not None and t < due:
            return False
        self._odom_next_pub = (
            t + self._odom_period
            if due is None or t - due > self._odom_period
            else due + self._odom_period
        )
        return True

    def _publish_odometry(self, state: OdomState) -> None:
        assert self._pub_odom_integrated is not None
        if state.t is None:
            return
        tf, odom = build_integrated_odom(state, self.odom_frame, self.base_frame)
        if self._odom_tf is not None:
            self._odom_tf.sendTransform(tf)
        self._pub_odom_integrated.publish(odom)

    def _note_split_frames(self):
        """Warn once that /odom/integrated needs the V1.1 ``s`` frame, not ``o`` and ``i``."""
        if self._pub_odom_integrated is None or self._odom_frames_warned:
            return
        self._odom_frames_warned = True
        self.get_logger().warn(
            'The firmware sends separate o and i frames; /odom/integrated is published '
            'from the NEORACER_V1.1 s frame only'
        )

    def pub_joy(self, data):
        """Publish a Joy message synthesized from a parsed ``r`` (FlySky) record."""
        axes, buttons = controller_lib.rc_to_joy(data['channels'], self._joy_cfg)
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [float(a) for a in axes]
        joy_msg.buttons = [int(b) for b in buttons]
        self.joy_pub.publish(joy_msg)

    def pub_rc(self, data):
        """Publish the raw FlySky channels from an ``r`` record, normalized."""
        # The student library reads the first eight; publishing all ten keeps
        # the aux switches reachable through `ros2 topic echo` while mapping them.
        msg = Float32MultiArray()
        msg.data = controller_lib.rc_to_channels(data['channels'], self._joy_cfg)
        self.rc_pub.publish(msg)

    def pub_rc_link(self, data):
        """Publish whether the transmitter link is up, from the raw ``r`` channels."""
        up = controller_lib.rc_link_up(data['channels'])
        self.rc_link_pub.publish(Bool(data=up))

    def pub_mag(self, data):
        """Publish a MagneticField message (Gauss -> Tesla) from an ``m`` record."""
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.imu_frame
        field = controller_lib.remap_axes(
            (data['mag_x'], data['mag_y'], data['mag_z']), self._mag_order, self._mag_sign, 1e-4
        )
        f = mag_msg.magnetic_field
        f.x, f.y, f.z = field
        self.mag_pub.publish(mag_msg)

    def motor_callback(self, msg: AckermannDriveStamped):
        """Map a normalized ``/motor`` command to the firmware ``v`` command."""
        command = controller_lib.motor_to_command(
            msg.drive.speed,
            msg.drive.steering_angle,
            self.max_speed_mps,
            self.max_steering_angle_deg,
            self.steering_trim_deg,
        )

        with self.serial_lock:
            try:
                self.serial.write(command.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f'[ERROR] Could not send [{command.strip()}]: {e}')

        self.last_cmd_time = self.get_clock().now()


# ===== INITIALIZE SYSTEM - DO NOT MODIFY ======


def main(args=None):
    """Spin the controller node until shutdown."""
    rclpy.init(args=args)
    try:
        node = ControllerNode()
    except Exception as exc:
        print(
            f'[controller_node] failed to start: {exc}\n'
            'Check config/controller.yaml. A common cause is a float parameter '
            'written as an integer: use -5.0, not -5.',
            file=sys.stderr,
        )
        rclpy.try_shutdown()
        raise

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
