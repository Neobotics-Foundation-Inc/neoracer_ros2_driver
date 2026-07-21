#!/usr/bin/env python3

"""
ESP32 serial bridge for the Seeed OSRbot board on the Neoracer.

A single node that owns the USB-serial link to the ESP32, which carries the
IMU, wheel odometry, FlySky RC receiver, and the motor/steering ESC. It:

- subscribes to ``/motor`` (ackermann_msgs/AckermannDriveStamped, normalized
  [-1, 1]) from the throttle node and writes the firmware drive command
  ``v <speed_mps> <steer_deg>``,
- publishes ``/imu`` (sensor_msgs/Imu) and ``/odom`` (nav_msgs/Odometry) parsed
  from the board, and
- publishes ``/joy`` (sensor_msgs/Joy) synthesized from the FlySky RC channels,
  so the same software pipeline (gamepad -> mux -> throttle) and the student
  controller API both work without a USB gamepad.

The exact FlySky channel order depends on the transmitter mixer and is exposed
entirely as ROS parameters (see config/controller.yaml) for on-car tuning.

Dependencies: pyserial (``serial``), controller_lib.
"""

# ===== IMPORT ROS2 CORE ======
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# ===== IMPORT ROS2 MESSAGE TYPES =====
from sensor_msgs.msg import Imu, Joy, MagneticField
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

# ===== IMPORT OTHER DEPENDENCIES ======
import serial
import sys
import threading
import time
from . import controller_lib

_INT_ARRAY = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)


def declare_float_param(node: Node, name: str, default: float) -> float:
    """Declare a double parameter that tolerates integer YAML values.

    A hand-edited ``steering_trim_deg: -5`` arrives from the YAML as an int,
    and a strictly-typed double parameter kills the node at startup over the
    missing ``.0``. Declare dynamically typed and cast instead; fall back to
    the default (loudly) if the value is not a number at all.
    """
    value = node.declare_parameter(
        name, default, ParameterDescriptor(dynamic_typing=True)
    ).value
    try:
        return float(value)
    except (TypeError, ValueError):
        node.get_logger().error(
            f"parameter '{name}' must be a number, got {value!r}; "
            f"using the default {default}"
        )
        return float(default)


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
        self.max_steering_angle_deg = declare_float_param(
            self, 'max_steering_angle_deg', 30.0)
        self.steering_trim_deg = declare_float_param(self, 'steering_trim_deg', 0.0)

        # Frames.
        self.imu_frame = self.declare_parameter('imu_frame', 'imu_link').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        # Optional magnetometer publication (the student library does not use it).
        self.publish_mag = self.declare_parameter('publish_mag', False).value
        self.publish_joy = self.declare_parameter('publish_joy', True).value

        # FlySky RC -> Joy mapping (all tunable on-hardware; see config/controller.yaml).
        self._joy_cfg = self._declare_joy_params()

        # ===== INITIALIZE SERIAL PORT =====
        try:
            self.serial = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"[DEBUG] Connected to: {self.port_name}")
        except serial.SerialException as e:
            self.get_logger().fatal(
                f"[ERROR] Could not connect to device '{self.port_name}': {e}")
            rclpy.shutdown()
            return

        # ===== SET UP SUBSCRIBERS =====
        # /motor is the throttle node output (normalized, BEST_EFFORT).
        self.motor_sub = self.create_subscription(
            AckermannDriveStamped, '/motor', self.motor_callback,
            qos_profile_sensor_data)

        # ===== SET UP PUBLISHERS =====
        self.imu_pub = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self._unknown_tags = set()
        # /joy is RELIABLE so the student controller API (a RELIABLE subscriber)
        # connects; BEST_EFFORT pipeline subscribers accept a reliable publisher.
        self.joy_pub = self.create_publisher(Joy, '/joy', 10) if self.publish_joy else None
        self.mag_pub = (
            self.create_publisher(MagneticField, '/mag', qos_profile_sensor_data)
            if self.publish_mag else None)

        # ===== STATE + SERIAL READ THREAD =====
        self.last_cmd_time = self.get_clock().now()
        self.serial_lock = threading.Lock()
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info("[DEBUG] Controller node initialized; entering main loop")

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
            'aux_button_channels': list(
                p('aux_button_channels', [], _INT_ARRAY).value or []),
            'aux_button_indices': list(
                p('aux_button_indices', [], _INT_ARRAY).value or []),
            'aux_button_thresh': p('aux_button_thresh', 0.5).value,
        }
        return cfg

    # Firmware lines that are expected but carry nothing we publish: admin
    # chatter (FW/DIAG/LINK banners, link acks) and the V1.1 battery frame,
    # which has no /battery publisher yet.
    _IGNORED_EXACT = frozenset({'b', 'link', 'OK', 'ERROR'})
    _IGNORED_PREFIXES = ('FW', 'DIAG', 'LINK')

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
                                f"Unable to parse message: [{line}], reason: {e}")
                except serial.SerialException:
                    self.get_logger().error("Serial Exception Error")
                    break
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f"Could not decode message: {e}")
            else:
                time.sleep(0.005)  # ~200 Hz idle poll

    def _note_unknown_line(self, line):
        """Warn once per unknown frame tag so a firmware protocol change is
        visible in the log instead of silently binned (the V1.1 's' frame
        went unnoticed exactly this way)."""
        tag = line.split(None, 1)[0]
        if tag in self._IGNORED_EXACT or tag.startswith(self._IGNORED_PREFIXES):
            return
        if tag not in self._unknown_tags:
            self._unknown_tags.add(tag)
            self.get_logger().warn(
                f"Unknown serial frame tag '{tag}' (first line: [{line}]); "
                "firmware protocol may be newer than this driver")

    def _dispatch(self, data, tag):
        """Route a parsed serial record to the matching publisher."""
        if data is None:
            return
        if tag == 's':
            # V1.1 firmware state frame carries both sensor sets.
            self.pub_imu(data)
            self.pub_odom(data)
        elif tag == 'i':
            self.pub_imu(data)
        elif tag == 'o':
            self.pub_odom(data)
        elif tag == 'r' and self.joy_pub is not None:
            self.pub_joy(data)
        elif tag == 'm' and self.mag_pub is not None:
            self.pub_mag(data)

    def pub_imu(self, data):
        """Publish an Imu message from a parsed ``i`` record."""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame

        imu_msg.orientation.x = data['q_x']
        imu_msg.orientation.y = data['q_y']
        imu_msg.orientation.z = data['q_z']
        imu_msg.orientation.w = data['q_w']

        imu_msg.linear_acceleration.x = data['a_x']
        imu_msg.linear_acceleration.y = data['a_y']
        imu_msg.linear_acceleration.z = data['a_z']

        imu_msg.angular_velocity.x = data['g_x']
        imu_msg.angular_velocity.y = data['g_y']
        imu_msg.angular_velocity.z = data['g_z']

        self.imu_pub.publish(imu_msg)

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

    def pub_joy(self, data):
        """Publish a Joy message synthesized from a parsed ``r`` (FlySky) record."""
        axes, buttons = controller_lib.rc_to_joy(data['channels'], self._joy_cfg)
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [float(a) for a in axes]
        joy_msg.buttons = [int(b) for b in buttons]
        self.joy_pub.publish(joy_msg)

    def pub_mag(self, data):
        """Publish a MagneticField message (Gauss -> Tesla) from an ``m`` record."""
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.imu_frame
        mag_msg.magnetic_field.x = data['mag_x'] * 1e-4
        mag_msg.magnetic_field.y = data['mag_y'] * 1e-4
        mag_msg.magnetic_field.z = data['mag_z'] * 1e-4
        self.mag_pub.publish(mag_msg)

    def motor_callback(self, msg: AckermannDriveStamped):
        """Map a normalized ``/motor`` command to the firmware ``v`` command."""
        command = controller_lib.motor_to_command(
            msg.drive.speed, msg.drive.steering_angle,
            self.max_speed_mps, self.max_steering_angle_deg, self.steering_trim_deg)

        with self.serial_lock:
            try:
                self.serial.write(command.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(
                    f"[ERROR] Could not send [{command.strip()}]: {e}")

        self.last_cmd_time = self.get_clock().now()


# ===== INITIALIZE SYSTEM - DO NOT MODIFY ======

def main(args=None):
    """Spin the controller node until shutdown."""
    rclpy.init(args=args)
    try:
        node = ControllerNode()
    except Exception as exc:
        print(
            f"[controller_node] failed to start: {exc}\n"
            "Check config/controller.yaml. A common cause is a float parameter "
            "written as an integer: use -5.0, not -5.",
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
