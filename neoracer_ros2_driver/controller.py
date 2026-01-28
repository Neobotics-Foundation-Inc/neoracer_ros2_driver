#!/usr/bin/env python3

"""
controller.py

Controller node to interface with custom Seeed OSRbot ESP32 firmware over serial.
Subscribes to the drive node from autonomy script (message type: ackermann_msgs/Drive),
converts commands to driver-specific serial protocol, and sends them to the ESP32.
Publishes IMU and odometry data as ROS2 topics after receiving from ESP32 board.

Dependencies:
- pyserial (as serial)
- controller_lib.py

"""

# ===== IMPORT ROS2 CORE ======
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
"""
qos_profile_sensor_data settings
- Reliability: BEST_EFFORT (attempt to deliver message but does not retry if lost, minimal latency)
- History: KEEP_LAST (always storre the most recent messages, depth = 5 default)
- Durability: VOLATILE (does not persist messages if disconnect)
"""

# ===== IMPORT ROS2 MESSAGE TYPES =====
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

# ===== IMPORT OTHER DEPENDENCIES ======
import serial, math, threading, time
from . import controller_lib

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # ===== DECLARE PARAMETERS =====
        self.port_name = '/dev/ttyACM1'
        self.baud_rate = 115200
        self.max_steering_angle_deg = 30 # in degrees, physical limitation of the vehicle
        self.wheelbase = 0.285 # in meters, physical limitation of the vehicle

        self.imu_frame = 'imu_link'
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        # ===== INITIALIZE SERIAL PORT =====
        try:
            self.serial = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"[DEBUG] Connected to: {self.port_name}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"[ERROR] Could not connect to device '{self.port_name}': {e}")
            rclpy.shutdown()
            return

        # ===== SET UP SUBSCRIBERS =====
        self.ackermann_cmd_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.ackermann_callback,
            qos_profile_sensor_data)

        # ===== SET UP PUBLISHERS =====
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu',
            qos_profile_sensor_data)

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            qos_profile_sensor_data)

        # ===== INITIALIZE THREADING =====
        self.last_cmd_time = self.get_clock().now()
        self.last_drive_cmd = ""
        self.serial_lock = threading.Lock() # gets called to prevent both pub/sub node from using this resource

        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True) # daemon=True, exit thread if program ends
        self.read_thread.start()

        self.get_logger().info("[DEBUG] Controller node finished initializing.. starting main loop")

    def read_serial_loop(self):
        """Read incoming serial messages and publish to sensor topics"""
        buffer = ""
        while rclpy.ok():
            if self.serial.in_waiting > 0:
                try:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        try:
                            data, tag = controller_lib.parse_serial_data(line)
                            if data is not None:
                                self.pub_imu(data) if tag == 'i' else self.pub_odom(data) if tag == 'o' else None
                        except (ValueError, IndexError) as e:
                            self.get_logger().warn(f"Unable to parse message: [{line}], reason: {e}")
                except serial.SerialException:
                    self.get_logger().error("Serial Exception Error")
                    break
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f"Could not decode message: {e}")
            else:
                time.sleep(0.005) # loop at about a rate of 200Hz

    def pub_imu(self, data):
        # Format and publish imu data to the /imu topic
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame

        # Orientation (accepts a quat, [x, y, z, w])
        imu_msg.orientation.x = data['q_x']
        imu_msg.orientation.y = data['q_y']
        imu_msg.orientation.z = data['q_z']
        imu_msg.orientation.w = data['q_w']

        # Linear Acceleration (m/s^2)
        imu_msg.linear_acceleration.x = data['a_x']
        imu_msg.linear_acceleration.y = data['a_y']
        imu_msg.linear_acceleration.z = data['a_z']

        # Angular Velocity (rad/s)
        imu_msg.angular_velocity.x = data['g_x']
        imu_msg.angular_velocity.y = data['g_y']
        imu_msg.angular_velocity.z = data['g_z']

        # Publish message
        self.imu_pub.publish(imu_msg)

    def pub_odom(self, data):
        # Format and publich odom data to the /odom topic
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Position [x, y, z] (m)
        odom_msg.pose.pose.position.x = data['p_x']
        odom_msg.pose.pose.position.y = data['p_y']
        odom_msg.pose.pose.position.z = data['p_z']

        # Orientation (quat, [x, y, z, w])
        q = controller_lib.quaternion_from_euler(0, 0, data['yaw'])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Linear Velocity [x, y, z] (m/s)
        odom_msg.twist.twist.linear.x = data['v_x']
        odom_msg.twist.twist.linear.y = data['v_y']
        odom_msg.twist.twist.linear.z = data['v_z']

        # Angular Velocity (not given by ESP32, so set to default value of 0)
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

    def ackermann_callback(self, msg: AckermannDriveStamped):
        """Read Ackermann message (speed/angle), map to values that the board expects, then send command via serial."""
        speed = msg.drive.speed
        angle = msg.drive.steering_angle

        # Map steering angle from abstract [-1, 1] to real [-30 deg, 30 deg] value via linear transform
        steering_angle_deg = self.max_steering_angle_deg * angle

        # Speed is a value from -6m/s to 6m/s, map based on max_speed param
        speed = speed * 6.0 # TODO param this later

        # Build command, ESP32 controller board expects key [v] to represent a drive command
        command = f"v {speed:.3f} {steering_angle_deg:.2f}\n"

        # Call dibs on serial thread to send command out to vehicle
        #if self.last_drive_cmd != command:
        with self.serial_lock:
            try:
                self.serial.write(command.encode('utf-8'))
                self.get_logger().info(f"[DEBUG] Sent command to controller board: {command.strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"[ERROR] Could not send message [{command}] out to controller board: {e}")
        
        self.last_drive_cmd = command
        self.last_cmd_time = self.get_clock().now() # track last commanded time to watch out for stale packets

# ===== INITIALIZE SYSTEM - DO NOT MODIFY ======

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
