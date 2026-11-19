"""
Topic contract: what the student library and the lab dashboards read and write.

Library v0.9.0-bot (MITRacecarNeo/racecar-neo-library), the Neobotics fork
of the library that deployed cars run until F45 resyncs it, and the seven
NeoRacer lab dashboards (camlabel, eps, linefollow, pursuit, smartfollow,
wallfollow, webteleop; in scripts/dashboards/ since F33, so their references
are checked like the driver's) reach the car only through the topics,
services, action and TF frames below. A
topic the library reads that nothing publishes does not fail: the library
reads zeros. So every row is checked against the driver's own sources: the
driver side exists with the message type, and the QoS on each end lets the
two connect (a RELIABLE subscriber hears nothing from a BEST_EFFORT
publisher; a TRANSIENT_LOCAL subscriber needs a TRANSIENT_LOCAL publisher).
Every read of the deployed fork must be live: the 0.9.0 driver may not break
the cars in the field. The one exception is /compute_path_to_pose: plan D1
(decided 2026-11-12) leaves navigation to upstream's 0.9.4, and the library
resync (F45) drops rc.nav in the same release (FORK_READS_NOT_PROVIDED).

Evidence is matched against code only: comments, docstrings and YAML or
XML comments are stripped first, so a commented-out publisher does not count.
A `path:line` reference into this repo must point at the line that sets the
topic up, so the references cannot drift. The readers' QoS comes from their
sources at the refs named in each End.
Publishers outside this repo are checked against the 0.6.1 baseline bag
recorded on the dev car on 2026-11-11 (BASELINE). Rows a later port feature
adds are skipped with that feature's name until it lands, so the table grows
as the port does.
"""

from dataclasses import dataclass
import io
from pathlib import Path
import re
import tokenize

import pytest

ROOT = Path(__file__).resolve().parent.parent

BE, REL = 'best_effort', 'reliable'
VOL, TL = 'volatile', 'transient_local'

LIB = 'library v0.9.0-bot'
AUTONOMY_NODE = 'neoracer_ros2_driver/autonomy_node.py'
DASH = 'dashboard'
FORK = 'deployed library fork'
# Neobotics-Foundation-Inc/racecar-neo-library, branch main, read at d8ca6ab.
FORK_REF = 'racecar-neo-library@d8ca6ab'


@dataclass(frozen=True)
class End:
    """One side of a topic: who it is, where it is set up, and the QoS it uses."""

    who: str
    source: str
    reliability: str
    durability: str = VOL


@dataclass(frozen=True)
class Row:
    """One topic, service, action or TF frame and everything that has to hold for it."""

    name: str
    msg_type: str
    publishers: tuple[End, ...] = ()
    subscribers: tuple[End, ...] = ()
    # (repo path, text) pairs that must appear in the driver's sources; spaces
    # and line breaks are ignored, so a call split over lines still matches.
    evidence: tuple[tuple[str, str], ...] = ()
    kind: str = 'topic'  # topic, service, action or frame
    pending: str = ''  # the port feature that adds the driver side
    absent: str = ''  # why the row does not exist on the NeoRacer
    note: str = ''


def _sensor(who, source):
    return End(who, source, BE)


def _fork(path, reliability=BE, durability=VOL):
    return End(FORK, f'{FORK_REF} library/real/{path}', reliability, durability)


# The mux subscribes to the RC topics only with rc_authority_enable (off by default).
MUX_RC = End('mux_node (rc_authority_enable)', 'neoracer_ros2_driver/mux_node.py:276', BE)
MUX_RC_LINK = End('mux_node (rc_authority_enable)', 'neoracer_ros2_driver/mux_node.py:273', BE)


def _dash(name, script, line, note='', reliability=BE):
    # A lab dashboard in scripts/dashboards/<name>/ (F33).
    ref = f'scripts/dashboards/{name}/{script}.py:{line}' + (f' ({note})' if note else '')
    return End(f'{DASH} {name}', ref, reliability)


def _viewer(line, qos, reliability=REL, durability=VOL):
    # The map viewer on 8088 (F26): subscribed only while a browser polls it.
    return End('map viewer', f'scripts/mapview.py:{line} ({qos})', reliability, durability)


def _overview(line):
    # The 8080 server's sensor panels (F30): opened while the page asks.
    return End('8080 dashboard', f'scripts/overview.py:{line} (qos_profile_sensor_data)', BE)


CAMERA_READERS = (
    End(LIB, 'library/real/camera_real.py:64', BE),
    _fork('camera_real.py:49 (depth 10)'),
    _dash('camlabel', 'camlabel', 81),
    _dash('linefollow', 'linefollow', 255),
    _dash('pursuit', 'pursuit', 123),
    _dash('smartfollow', 'smartfollow', 256),
    _dash('webteleop', 'teleop', 313),
    End('inference_node', 'neoracer_ros2_driver/inference_node.py:264', BE),
    _overview(60),
)
ODOM_READERS = (
    _dash('eps', 'eps', 400),
    _dash('linefollow', 'linefollow', 256),
    _dash('smartfollow', 'smartfollow', 255),
    _dash('webteleop', 'teleop', 322),
    _dash('wallfollow', 'wallfollow', 173),
)
SCAN_READERS = (
    End(LIB, 'library/real/lidar_real.py:36 (qos_profile_sensor_data)', BE),
    _fork('lidar_real.py:41 (qos_profile_sensor_data)'),
    _dash('eps', 'eps', 399),
    _dash('smartfollow', 'smartfollow', 254),
    _dash('webteleop', 'teleop', 321),
    _dash('wallfollow', 'wallfollow', 172),
    _overview(61),
    End('autonomy_node', f'{AUTONOMY_NODE}:188 (raw)', BE),
    End('slam_toolbox', 'launch/autonomy.launch.py (sensor data QoS)', BE),
    End('AMCL', 'launch/autonomy.launch.py localizer:=amcl (sensor data QoS)', BE),
    _viewer(416, 'qos_profile_sensor_data', BE),
)
DRIVE_WRITERS = (
    End(LIB, 'library/real/drive_real.py:34 (depth 1)', REL),
    _fork('drive_real.py:40 (depth 1)', REL),
) + tuple(
    _dash(name, script, line, 'depth 1', REL)
    for name, script, line in (
        ('eps', 'eps', 398),
        ('linefollow', 'linefollow', 254),
        ('pursuit', 'pursuit', 122),
        ('smartfollow', 'smartfollow', 253),
        ('webteleop', 'teleop', 312),
        ('wallfollow', 'wallfollow', 171),
    )
)
CONTROLLER = 'neoracer_ros2_driver/controller.py'
PHYSICS = 'library/real/physics_real.py:54'
FORK_PHYSICS = 'physics_real.py:57 (depth 1)'
AUTONOMY = 'launch/autonomy.launch.py'
FRAMES_URDF = 'urdf/neoracer.urdf.xacro'
TELEOP = 'launch/teleop.launch.py'
# The autonomy launch runs slam_toolbox under its own name and with no
# namespace while mapping (test_autonomy.py checks both), and map_server with
# AMCL, or the slam_toolbox localizer, in localization.
SLAM_NODE = (
    AUTONOMY,
    "'package': 'slam_toolbox', 'executable': 'async_slam_toolbox_node', 'name': 'slam_toolbox'",
)
SLAM_LOCALIZER = (
    AUTONOMY,
    "'package': 'slam_toolbox', 'executable': 'localization_slam_toolbox_node', "
    "'name': 'slam_toolbox'",
)
MAP_SERVER = (AUTONOMY, "'package': 'nav2_map_server', 'executable': 'map_server'")
AMCL_NODE = (AUTONOMY, "'package': 'nav2_amcl', 'executable': 'amcl'")
SLAM = 'live while the autonomy launch runs (racecar launch autonomy, neoracer-autonomy)'

TABLE = (
    # -- read by the library --
    Row(
        '/camera/color',
        'sensor_msgs/msg/Image',
        publishers=(End('camera_node', 'neoracer_ros2_driver/camera.py:78', BE),),
        subscribers=CAMERA_READERS,
        evidence=(
            (
                'neoracer_ros2_driver/camera.py',
                "create_publisher(Image, '/camera/color', qos_profile_sensor_data)",
            ),
        ),
    ),
    Row(
        '/camera/depth',
        'sensor_msgs/msg/Image',
        subscribers=(End(LIB, 'library/real/camera_real.py:58', BE),),
        absent='no depth camera; the library fork raises NotImplementedError (plan D7)',
    ),
    Row(
        '/joy',
        'sensor_msgs/msg/Joy',
        publishers=(End('controller_node', f'{CONTROLLER}:231', REL),),
        subscribers=(
            End(LIB, 'library/real/controller_real.py:89 (depth 1)', REL),
            _fork('controller_real.py:89 (depth 1)', REL),
            End('mux_node', 'neoracer_ros2_driver/mux_node.py:267', BE),
            End('gamepad_node', 'neoracer_ros2_driver/gamepad_node.py:51', BE),
        ),
        evidence=(
            (CONTROLLER, "create_publisher(Joy, '/joy', 10)"),
            ('config/controller.yaml', 'publish_joy: true'),
        ),
        note='both libraries subscribe RELIABLE, so /joy must stay RELIABLE',
    ),
    Row(
        '/scan',
        'sensor_msgs/msg/LaserScan',
        publishers=(End('lakibeam1 scan node', 'launch/lidar.launch.py', REL),),
        subscribers=SCAN_READERS,
        evidence=(
            ('launch/lidar.launch.py', 'lakibeam1'),
            (
                'launch/lidar.launch.py',
                "DeclareLaunchArgument('output_topic', default_value='scan')",
            ),
        ),
    ),
    Row(
        '/imu/fused',
        'sensor_msgs/msg/Imu',
        publishers=(End('controller_node', f'{CONTROLLER}:205', REL),),
        subscribers=(
            End(LIB, PHYSICS, BE),
            _fork(FORK_PHYSICS),
            _overview(62),
            _dash('webteleop', 'teleop', 324, 'qos_profile_sensor_data, raw'),
        ),
        evidence=((CONTROLLER, "create_publisher(Imu, '/imu/fused', 10)"),),
    ),
    Row(
        '/mag',
        'sensor_msgs/msg/MagneticField',
        publishers=(End('controller_node', f'{CONTROLLER}:233', BE),),
        subscribers=(
            End(LIB, PHYSICS, BE),
            _fork(FORK_PHYSICS),
            _dash('webteleop', 'teleop', 324, 'qos_profile_sensor_data, raw'),
        ),
        evidence=(
            (CONTROLLER, "create_publisher(MagneticField, '/mag', qos_profile_sensor_data)"),
            ('config/controller.yaml', 'publish_mag: true'),
        ),
    ),
    Row(
        '/encoder/speed',
        'std_msgs/msg/Float32',
        publishers=(End('controller_node', f'{CONTROLLER}:220', BE),),
        subscribers=(End(LIB, PHYSICS, BE), _fork(FORK_PHYSICS), _overview(63)),
        evidence=(
            (CONTROLLER, "create_publisher(Float32, '/encoder/speed', qos_profile_sensor_data)"),
        ),
    ),
    Row(
        '/battery/state',
        'sensor_msgs/msg/BatteryState',
        publishers=(End('controller_node', f'{CONTROLLER}:211', REL),),
        subscribers=(
            End(LIB, 'library/real/physics_real.py:78 (depth 10)', REL),
            _dash('webteleop', 'teleop', 328, 'depth 10', REL),
        ),
        evidence=((CONTROLLER, "create_publisher(BatteryState, '/battery/state', 10)"),),
        note='added by F07; the library subscribes RELIABLE, so /battery/state must stay RELIABLE',
    ),
    Row(
        '/rc/channels',
        'std_msgs/msg/Float32MultiArray',
        publishers=(End('controller_node', f'{CONTROLLER}:223', BE),),
        subscribers=(End(LIB, PHYSICS, BE), _fork(FORK_PHYSICS), MUX_RC),
        evidence=(
            (
                CONTROLLER,
                "create_publisher(Float32MultiArray, '/rc/channels', qos_profile_sensor_data)",
            ),
            (
                'neoracer_ros2_driver/mux_node.py',
                "declare_parameter('rc_channels_topic', '/rc/channels')",
            ),
            (
                'neoracer_ros2_driver/mux_node.py',
                "create_subscription(Float32MultiArray, self.get_parameter('rc_channels_topic')",
            ),
            ('config/mux.yaml', 'rc_channels_topic: "/rc/channels"'),
        ),
        note='the mux reads it only with rc_authority_enable, off by default (F12)',
    ),
    Row(
        '/edgetpu/inference',
        'vision_msgs/msg/Detection2DArray',
        publishers=(End('inference_node', 'neoracer_ros2_driver/inference_node.py:143', REL),),
        subscribers=(
            End(LIB, 'library/real/vision_real.py:34', BE),
            _fork('vision_real.py:34'),
            _dash('pursuit', 'pursuit', 124, 'depth 10', REL),
            _dash('smartfollow', 'smartfollow', 257, 'depth 10', REL),
            _dash('webteleop', 'teleop', 318, 'qos_profile_sensor_data'),
        ),
        evidence=(
            (
                'neoracer_ros2_driver/inference_node.py',
                'create_publisher(Detection2DArray, det_topic, 10)',
            ),
            (
                'neoracer_ros2_driver/inference_node.py',
                "declare_parameter('detections_topic', '/edgetpu/inference')",
            ),
            ('config/inference.yaml', 'detections_topic: "/edgetpu/inference"'),
        ),
        note='pursuit and smartfollow subscribe RELIABLE, so detections must stay RELIABLE',
    ),
    Row(
        '/diagnostics',
        'diagnostic_msgs/msg/DiagnosticArray',
        publishers=(End('inference_node', 'neoracer_ros2_driver/inference_node.py:147', REL),),
        subscribers=(
            End(LIB, 'library/real/telemetry_real.py:37', BE),
            _fork('telemetry_real.py:37'),
        ),
        evidence=(
            (
                'neoracer_ros2_driver/inference_node.py',
                "create_publisher(DiagnosticArray, '/diagnostics', 10)",
            ),
        ),
    ),
    Row(
        '/autonomy/status',
        'diagnostic_msgs/msg/DiagnosticStatus',
        publishers=(End('autonomy_node', f'{AUTONOMY_NODE}:209 (latched)', REL, TL),),
        subscribers=(
            End(LIB, 'library/real/slam_real.py:45 (latched)', REL, TL),
            End('throttle_node', 'neoracer_ros2_driver/throttle_node.py:75 (latched)', REL, TL),
            End('8080 dashboard', 'scripts/dashboard.py:196 (latched)', REL, TL),
            _viewer(413, 'latched', REL, TL),
            End('inference_node', 'neoracer_ros2_driver/inference_node.py:159 (latched)', REL, TL),
        ),
        evidence=(
            (AUTONOMY_NODE, "create_publisher(DiagnosticStatus, '/autonomy/status', latched)"),
            (
                'neoracer_ros2_driver/throttle_node.py',
                "create_subscription(DiagnosticStatus, '/autonomy/status', self._status_cb, "
                'latched)',
            ),
            (
                'scripts/dashboard.py',
                "create_subscription(DiagnosticStatus, '/autonomy/status', self._record_autonomy, "
                'latched)',
            ),
            (
                'neoracer_ros2_driver/inference_node.py',
                "create_subscription(DiagnosticStatus, '/autonomy/status', self._status_cb, "
                'latched)',
            ),
        ),
        note=(
            'added by F23; the throttle node reads it for the mapping caps (F14), the '
            'inference node for its pause while mapping (F27) and the 8080 page for its '
            'Autonomy card (F32)'
        ),
    ),
    Row(
        '/autonomy/pose',
        'geometry_msgs/msg/PoseWithCovarianceStamped',
        publishers=(End('autonomy_node', f'{AUTONOMY_NODE}:208', REL),),
        subscribers=(
            End(LIB, 'library/real/slam_real.py:54 (depth 10)', REL),
            _viewer(414, 'depth 10'),
        ),
        evidence=(
            (AUTONOMY_NODE, "create_publisher(PoseWithCovarianceStamped, '/autonomy/pose', 10)"),
        ),
        note='added by F23',
    ),
    Row(
        '/map',
        'nav_msgs/msg/OccupancyGrid',
        publishers=(
            End('slam_toolbox', f'{AUTONOMY} mode:=mapping', REL, TL),
            End('nav2 map_server', f'{AUTONOMY} mode:=localization', REL, TL),
            End('slam_toolbox localizer', f'{AUTONOMY} localizer:=slam_toolbox', REL, TL),
        ),
        subscribers=(
            End(LIB, 'library/real/slam_real.py:57 (latched)', REL, TL),
            _fork('slam_real.py:52 (latched)', REL, TL),
            End('autonomy_node', f'{AUTONOMY_NODE}:201 (latched, localization)', REL, TL),
            _viewer(415, 'latched', REL, TL),
        ),
        evidence=(SLAM_NODE, SLAM_LOCALIZER, MAP_SERVER, ('config/amcl.yaml', 'topic_name: map')),
        note=f'{SLAM}; slam_toolbox and map_server publish it latched, as upstream',
    ),
    Row(
        '/autonomy/save_map',
        'std_srvs/srv/Trigger',
        publishers=(End('autonomy_node', f'{AUTONOMY_NODE}:211 (server)', REL),),
        subscribers=(End(LIB, 'library/real/slam_real.py:59 (client)', REL),),
        evidence=(
            (
                AUTONOMY_NODE,
                "create_service(Trigger, '/autonomy/save_map', self._on_save_map, "
                'callback_group=clients)',
            ),
        ),
        kind='service',
        note='added by F23',
    ),
    Row(
        '/autonomy/global_localization',
        'std_srvs/srv/Trigger',
        publishers=(End('autonomy_node', f'{AUTONOMY_NODE}:217 (server)', REL),),
        subscribers=(_viewer(419, 'client'),),
        evidence=(
            (
                AUTONOMY_NODE,
                "create_service(Trigger, '/autonomy/global_localization', self._on_global, "
                'callback_group=clients)',
            ),
        ),
        kind='service',
        note='added by F23; called by the map viewer (Find me) and the SLAM validation (F41)',
    ),
    # -- written by the library and the dashboards --
    Row(
        '/drive',
        'ackermann_msgs/msg/AckermannDriveStamped',
        publishers=DRIVE_WRITERS,
        subscribers=(End('mux_node', 'neoracer_ros2_driver/mux_node.py:269', BE),),
        evidence=(
            (
                'neoracer_ros2_driver/mux_node.py',
                "create_subscription(AckermannDriveStamped, '/drive', self._auto_cb, qos)",
            ),
            ('neoracer_ros2_driver/mux_node.py', 'reliability=QoSReliabilityPolicy.BEST_EFFORT'),
        ),
    ),
    Row(
        '/dotmatrix/text',
        'std_msgs/msg/String',
        publishers=(
            End(LIB, 'library/real/display_real.py:59 (depth 1)', REL),
            _fork('display_real.py:54 (depth 1)', REL),
            _dash('webteleop', 'teleop', 329, 'depth 1', REL),
        ),
        subscribers=(End('led_matrix_node', 'neoracer_ros2_driver/led_matrix_node.py:62', REL),),
        evidence=(
            (
                'neoracer_ros2_driver/led_matrix_node.py',
                'create_subscription(String, self.input_topic, self._on_text, 10)',
            ),
            (
                'neoracer_ros2_driver/led_matrix_node.py',
                "declare_parameter('input_topic', '/dotmatrix/text')",
            ),
            ('config/led_matrix.yaml', 'input_topic: "/dotmatrix/text"'),
        ),
    ),
    Row(
        '/dotmatrix/pixels',
        'std_msgs/msg/UInt8MultiArray',
        publishers=(End(LIB, 'library/real/display_real.py:62', REL),),
        absent='the LED matrix node takes text only; the library fork stubs pixels (plan D7)',
    ),
    Row(
        '/led/pixels',
        'std_msgs/msg/UInt8MultiArray',
        publishers=(End(LIB, 'library/real/led_real.py:34', REL),),
        absent='no addressable LED strip; the library fork stubs it (plan D7)',
    ),
    # -- read by the dashboards --
    Row(
        '/odom',
        'nav_msgs/msg/Odometry',
        publishers=(End('controller_node', f'{CONTROLLER}:206', REL),),
        subscribers=(
            *ODOM_READERS,
            # The firmware pose beside the integrated estimate, on either source.
            End('calibrate_odom (F37)', 'scripts/calibration/calibrate_odom.py:488', BE),
        ),
        evidence=((CONTROLLER, "create_publisher(Odometry, '/odom', 10)"),),
    ),
    # -- the drive path and the other driver topics in the feature map --
    Row(
        '/gamepad_drive',
        'ackermann_msgs/msg/AckermannDriveStamped',
        publishers=(End('gamepad_node', 'neoracer_ros2_driver/gamepad_node.py:50', BE),),
        subscribers=(End('mux_node', 'neoracer_ros2_driver/mux_node.py:268', BE),),
        evidence=(
            (
                'neoracer_ros2_driver/gamepad_node.py',
                "create_publisher(AckermannDriveStamped, '/gamepad_drive', qos)",
            ),
        ),
    ),
    Row(
        '/mux_out',
        'ackermann_msgs/msg/AckermannDriveStamped',
        publishers=(End('mux_node', 'neoracer_ros2_driver/mux_node.py:266', BE),),
        subscribers=(End('throttle_node', 'neoracer_ros2_driver/throttle_node.py:69', BE),),
        evidence=(
            (
                'neoracer_ros2_driver/mux_node.py',
                "create_publisher(AckermannDriveStamped, '/mux_out', qos)",
            ),
            (
                'neoracer_ros2_driver/throttle_node.py',
                "create_subscription(AckermannDriveStamped, '/mux_out', self._drive_cb, qos)",
            ),
        ),
    ),
    Row(
        '/motor',
        'ackermann_msgs/msg/AckermannDriveStamped',
        publishers=(End('throttle_node', 'neoracer_ros2_driver/throttle_node.py:68', BE),),
        subscribers=(End('controller_node', f'{CONTROLLER}:196', BE),),
        evidence=(
            (
                'neoracer_ros2_driver/throttle_node.py',
                "create_publisher(AckermannDriveStamped, '/motor', qos)",
            ),
            (
                CONTROLLER,
                "create_subscription(AckermannDriveStamped, '/motor', self.motor_callback, "
                'qos_profile_sensor_data)',
            ),
        ),
    ),
    Row(
        '/battery',
        'sensor_msgs/msg/BatteryState',
        publishers=(End('controller_node', f'{CONTROLLER}:208', BE),),
        subscribers=(End('8080 dashboard', 'scripts/dashboard.py:203', BE),),
        evidence=(
            (CONTROLLER, "create_publisher(BatteryState, '/battery', qos_profile_sensor_data)"),
        ),
        note='kept one more release for the 8080 page (F07)',
    ),
    Row(
        '/battery/voltage',
        'std_msgs/msg/Float32',
        publishers=(End('controller_node', f'{CONTROLLER}:217', BE),),
        evidence=(
            (CONTROLLER, "create_publisher(Float32, '/battery/voltage', qos_profile_sensor_data)"),
        ),
        subscribers=(_fork(FORK_PHYSICS),),
        note='the deployed fork reads it (physics_real.py, physics.py); kept until F45',
    ),
    Row(
        '/rc/link',
        'std_msgs/msg/Bool',
        publishers=(End('controller_node', f'{CONTROLLER}:227', BE),),
        subscribers=(MUX_RC_LINK,),
        evidence=(
            (CONTROLLER, "create_publisher(Bool, '/rc/link', qos_profile_sensor_data)"),
            ('neoracer_ros2_driver/mux_node.py', "declare_parameter('rc_link_topic', '/rc/link')"),
            (
                'neoracer_ros2_driver/mux_node.py',
                "create_subscription(Bool, self.get_parameter('rc_link_topic').value",
            ),
            ('config/mux.yaml', 'rc_link_topic: "/rc/link"'),
        ),
        note=(
            'added by F08; the mux reads it only with rc_authority_enable, off by default (F12)'
        ),
    ),
    Row(
        '/odom/integrated',
        'nav_msgs/msg/Odometry',
        publishers=(End('controller_node', f'{CONTROLLER}:337', BE),),
        subscribers=(
            End(
                'calibrate_odom (F37)',
                'scripts/calibration/calibrate_odom.py:449',
                BE,
            ),
        ),
        evidence=(
            (CONTROLLER, 'create_publisher(Odometry, topic, qos_profile_sensor_data)'),
            ('config/odom.yaml', 'odometry.topic: "/odom/integrated"'),
            ('launch/controller.launch.py', "('odom_config', 'odom.yaml')"),
        ),
        note=(
            'added by F09; read by calibrate_odom (F37), calibrate_dynamics (F39) '
            'and analyze_odom (F40)'
        ),
    ),
    Row(
        'TF base_link -> laser',
        'tf2_msgs/msg/TFMessage',
        publishers=(End('robot_state_publisher (frames, /tf_static)', FRAMES_URDF, REL, TL),),
        subscribers=(End('slam_toolbox and AMCL', AUTONOMY, REL, TL),),
        evidence=((FRAMES_URDF, '<child link="laser" />'), (TELEOP, "'frames': 'true'")),
        kind='frame',
        note=(
            'added by F05; its yaw is laser_yaw_deg, which the lidar calibration (F36) '
            'writes to the car profile; library v0.9.2-bot reads its yaw in the 0.9.2 port'
        ),
    ),
    # -- read by the deployed library fork through the autonomy launch --
    Row(
        'TF map -> base_footprint',
        'tf2_msgs/msg/TFMessage',
        publishers=(
            End(
                'TransformBroadcaster (controller_node, odom -> base_link)',
                f'{CONTROLLER}:339',
                REL,
            ),
            End('robot_state_publisher (base_link -> base_footprint, static)', FRAMES_URDF, REL),
            End('slam_toolbox (map -> odom)', f'{AUTONOMY} mode:=mapping', REL),
            End('AMCL (map -> odom)', f'{AUTONOMY} mode:=localization', REL),
        ),
        subscribers=(
            _fork('slam_real.py:77 (lookup_transform)', REL),
            _fork('nav_real.py:64 (lookup_transform)', REL),
        ),
        evidence=(
            (CONTROLLER, 'self._odom_tf = TransformBroadcaster(self)'),
            ('config/odom.yaml', 'odometry.publish_tf: true'),
            ('launch/controller.launch.py', "('odom_config', 'odom.yaml')"),
            (FRAMES_URDF, '<child link="base_footprint" />'),
            (TELEOP, "'frames': 'true'"),
            SLAM_NODE,
            AMCL_NODE,
            ('config/amcl.yaml', 'tf_broadcast: true'),
        ),
        kind='frame',
        note=(
            f'{SLAM}; the controller publishes odom -> base_link (plan D1 retired the EKF), '
            'the teleop frames publish base_footprint as a static child of base_link for '
            'one release (D5), and slam_toolbox or AMCL publishes map -> odom'
        ),
    ),
    Row(
        '/slam_toolbox/serialize_map',
        'slam_toolbox/srv/SerializePoseGraph',
        subscribers=(
            _fork('slam_real.py:63 (client)', REL),
            End('autonomy_node', f'{AUTONOMY_NODE}:223 (client, saves)', REL),
        ),
        evidence=(SLAM_NODE,),
        kind='service',
        note=(
            f'{SLAM} in mapping; autonomy_node saves through it, and the fork reads it '
            'until F45'
        ),
    ),
    Row(
        '/slam_toolbox/deserialize_map',
        'slam_toolbox/srv/DeserializePoseGraph',
        subscribers=(_fork('slam_real.py:66 (client)', REL),),
        evidence=(SLAM_NODE,),
        kind='service',
        note=f'{SLAM} in mapping',
    ),
    Row(
        '/slam_toolbox/clear_changes',
        'slam_toolbox/srv/Clear',
        subscribers=(_fork('slam_real.py:69 (client)', REL),),
        evidence=(SLAM_NODE,),
        kind='service',
        note=f'{SLAM} in mapping',
    ),
    Row(
        '/compute_path_to_pose',
        'nav2_msgs/action/ComputePathToPose',
        subscribers=(_fork('nav_real.py:48 (action client)', REL),),
        kind='action',
        absent=(
            'not provided until upstream 0.9.4: plan D1 (decided 2026-11-12) has no '
            'navigation until then, and the library resync (F45) drops rc.nav'
        ),
    ),
)

# Topics the car published on 0.6.1, from the baseline bag's metadata.yaml
# (dev car, 2026-11-11): type and the publisher's offered reliability.
BASELINE = {
    '/odom': ('nav_msgs/msg/Odometry', REL),
    '/battery/voltage': ('std_msgs/msg/Float32', BE),
    '/camera/color': ('sensor_msgs/msg/Image', BE),
    '/diagnostics': ('diagnostic_msgs/msg/DiagnosticArray', REL),
    '/gamepad_drive': ('ackermann_msgs/msg/AckermannDriveStamped', BE),
    '/imu/fused': ('sensor_msgs/msg/Imu', REL),
    '/joy': ('sensor_msgs/msg/Joy', REL),
    '/mux_out': ('ackermann_msgs/msg/AckermannDriveStamped', BE),
    '/drive': ('ackermann_msgs/msg/AckermannDriveStamped', REL),
    '/mag': ('sensor_msgs/msg/MagneticField', BE),
    '/motor': ('ackermann_msgs/msg/AckermannDriveStamped', BE),
    '/rc/channels': ('std_msgs/msg/Float32MultiArray', BE),
    '/encoder/speed': ('std_msgs/msg/Float32', BE),
    '/battery': ('sensor_msgs/msg/BatteryState', BE),
    '/scan': ('sensor_msgs/msg/LaserScan', REL),
    '/edgetpu/inference': ('vision_msgs/msg/Detection2DArray', REL),
}

# The rows the feature map's F44 block lists.
FEATURE_MAP_ROWS = {
    '/scan', '/imu/fused', '/odom', '/odom/integrated', '/encoder/speed', '/battery/state',
    '/battery', '/battery/voltage', '/mag', '/camera/color', '/rc/channels', '/rc/link', '/joy',
    '/drive', '/gamepad_drive', '/mux_out', '/motor', '/edgetpu/inference', '/dotmatrix/text',
    '/autonomy/status', '/autonomy/pose', '/map', '/autonomy/save_map',
    '/autonomy/global_localization',
}  # fmt: skip

# Everything the deployed fork reads, and writes, on a real car.
FORK_READS = {
    '/camera/color', '/joy', '/scan', '/imu/fused', '/mag', '/encoder/speed',
    '/battery/voltage', '/rc/channels', '/edgetpu/inference', '/diagnostics', '/map',
    'TF map -> base_footprint', '/slam_toolbox/serialize_map', '/slam_toolbox/deserialize_map',
    '/slam_toolbox/clear_changes', '/compute_path_to_pose',
}  # fmt: skip
FORK_WRITES = {'/drive', '/dotmatrix/text'}
# Fork reads with no server until upstream 0.9.4 (decided 2026-11-12).
FORK_READS_NOT_PROVIDED = {'/compute_path_to_pose'}

LIVE = [r for r in TABLE if not r.pending and not r.absent]


def _squash(text):
    return re.sub(r'\s+', '', text)


def _python_code(text):
    """Return Python source without comments, docstrings or bare string statements."""
    tokens = list(tokenize.generate_tokens(io.StringIO(text).readline))
    skip = {tokenize.NL, tokenize.COMMENT}
    starts = {None, tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT}
    kept, prev = [], None
    for i, tok in enumerate(tokens):
        if tok.type == tokenize.COMMENT:
            continue
        if tok.type == tokenize.STRING and prev in starts:
            after = i + 1
            nxt = next((n for n in tokens[after:] if n.type not in skip), None)
            if nxt is None or nxt.type in (tokenize.NEWLINE, tokenize.ENDMARKER):
                prev = tok.type
                continue
        kept.append(tok.string)
        if tok.type not in skip:
            prev = tok.type
    return ' '.join(kept)


def _yaml_code(text):
    """Return YAML without `#` comments (a `#` at a line start or after a space)."""
    lines = []
    for line in text.splitlines():
        quote, cut = None, len(line)
        for i, ch in enumerate(line):
            if quote:
                quote = None if ch == quote else quote
            elif ch in '"\'':
                quote = ch
            elif ch == '#' and (i == 0 or line[i - 1].isspace()):
                cut = i
                break
        lines.append(line[:cut])
    return '\n'.join(lines)


def _code_text(path):
    """Return a source file's text with its comments and docstrings removed."""
    text = Path(path).read_text()
    name = Path(path).name
    if name.endswith('.py'):
        return _python_code(text)
    if name.endswith(('.yaml', '.yml')):
        return _yaml_code(text)
    if name.endswith(('.xacro', '.urdf', '.xml', '.launch')):
        return re.sub(r'<!--.*?-->', '', text, flags=re.S)
    if name.endswith('.sh'):
        return '\n'.join(ln for ln in text.splitlines() if not ln.lstrip().startswith('#'))
    return text


def _skip_unless_live(row):
    if row.absent:
        pytest.skip(f'{row.name}: not on the NeoRacer ({row.absent})')
    if row.pending:
        pytest.skip(f'{row.name}: arrives with {row.pending}')


def connects(publisher, subscriber):
    """Report whether a DDS subscriber with these QoS hears this publisher."""
    if subscriber.reliability == REL and publisher.reliability == BE:
        return False
    if subscriber.durability == TL and publisher.durability == VOL:
        return False
    return True


def test_qos_rule():
    pub_be, pub_rel = End('p', '', BE), End('p', '', REL)
    assert connects(pub_rel, End('s', '', BE))
    assert connects(pub_rel, End('s', '', REL))
    assert connects(pub_be, End('s', '', BE))
    assert not connects(pub_be, End('s', '', REL))
    assert not connects(pub_rel, End('s', '', REL, TL))
    assert connects(End('p', '', REL, TL), End('s', '', REL, TL))


@pytest.mark.parametrize('row', TABLE, ids=lambda r: r.name)
def test_the_driver_side_is_in_the_sources(row):
    # A pending row may already name a reader in this repo (as the throttle
    # node did on /autonomy/status before F23); that reader is checked before
    # the publisher lands.
    if not (row.pending and row.evidence):
        _skip_unless_live(row)
    assert row.evidence, f'{row.name} has no evidence in the driver sources'
    for path, text in row.evidence:
        source = _squash(_code_text(ROOT / path))
        assert _squash(text) in source, f'{row.name}: {path} lacks {text!r}'


@pytest.mark.parametrize('row', TABLE, ids=lambda r: r.name)
def test_every_reader_connects_to_every_writer(row):
    _skip_unless_live(row)
    if row.kind != 'topic':
        return
    assert row.publishers, f'{row.name} has no publisher'
    for pub in row.publishers:
        for sub in row.subscribers:
            assert connects(pub, sub), (
                f'{row.name}: {sub.who} ({sub.reliability}, {sub.durability}) hears nothing '
                f'from {pub.who} ({pub.reliability}, {pub.durability})'
            )


@pytest.mark.parametrize('row', [r for r in LIVE if r.name in BASELINE], ids=lambda r: r.name)
def test_the_table_matches_what_the_car_published(row):
    msg_type, reliability = BASELINE[row.name]
    assert row.msg_type == msg_type
    driver_side = [p for p in row.publishers if not p.who.startswith((LIB, DASH, FORK))]
    for pub in driver_side:
        assert pub.reliability == reliability, (row.name, pub.who)


def test_rows_the_port_has_not_added_name_their_feature():
    for row in TABLE:
        if row.pending:
            assert re.fullmatch(r'F\d\d', row.pending), row.name


def test_every_library_read_is_covered():
    library_reads = {
        '/camera/color', '/camera/depth', '/joy', '/scan', '/imu/fused', '/mag',
        '/encoder/speed', '/battery/state', '/rc/channels', '/edgetpu/inference',
        '/diagnostics', '/autonomy/status', '/autonomy/pose', '/map', '/autonomy/save_map',
    }  # fmt: skip
    rows = {r.name: r for r in TABLE}
    for name in library_reads:
        assert name in rows, name
        assert any(s.who == LIB for s in rows[name].subscribers), name


def test_every_fork_read_is_live():
    # Deployed cars run the fork until F45; the 0.9.0 driver must not break them.
    rows = {r.name: r for r in TABLE}
    assert FORK_READS_NOT_PROVIDED <= FORK_READS
    for name in FORK_READS:
        row = rows[name]
        assert any(s.who == FORK for s in row.subscribers), name
        if name in FORK_READS_NOT_PROVIDED:
            assert row.absent and not row.pending, f'{name}: marked not provided'
            continue
        assert not row.pending and not row.absent, f'{name}: deployed cars read it'
    for row in TABLE:
        if any(s.who == FORK for s in row.subscribers):
            assert row.name in FORK_READS, row.name


def test_every_fork_write_reaches_the_driver():
    rows = {r.name: r for r in TABLE}
    for name in FORK_WRITES:
        row = rows[name]
        assert any(p.who == FORK for p in row.publishers), name
        assert row in LIVE and row.subscribers, name


def test_comments_and_docstrings_do_not_count(tmp_path):
    py = tmp_path / 'node.py'
    py.write_text(
        '"""Module: create_publisher(Joy, \'/joy\', 10)."""\n\n\n'
        'def make(node):\n'
        '    """create_publisher(Imu, \'/imu/fused\', 10)"""\n'
        "    # node.create_publisher(Odometry, '/odom', 10)\n"
        "    return node.create_publisher(Bool, '/rc/link', 1)  # live\n"
    )
    code = _squash(_code_text(py))
    assert _squash("create_publisher(Joy, '/joy', 10)") not in code
    assert _squash("create_publisher(Imu, '/imu/fused', 10)") not in code
    assert _squash("create_publisher(Odometry, '/odom', 10)") not in code
    assert _squash("create_publisher(Bool, '/rc/link', 1)") in code
    yml = tmp_path / 'node.yaml'
    yml.write_text('# publish_joy: true\nlabel: "a # b"  # trailing\npublish_mag: true\n')
    code = _squash(_code_text(yml))
    assert 'publish_joy:true' not in code
    assert 'publish_mag:true' in code and 'label:"a#b"' in code
    xacro = tmp_path / 'car.urdf.xacro'
    xacro.write_text('<robot><!-- <child link="laser" /> --><child link="base_link" /></robot>')
    code = _squash(_code_text(xacro))
    assert '<childlink="laser"/>' not in code and '<childlink="base_link"/>' in code


def _repo_line_refs():
    for row in TABLE:
        for end in row.publishers + row.subscribers:
            m = re.match(r'([\w./-]+\.(?:py|yaml|xacro|sh)):(\d+)', end.source)
            if m and (ROOT / m.group(1)).is_file():
                yield row, end, m.group(1), int(m.group(2))


@pytest.mark.parametrize(
    'row,end,path,line',
    list(_repo_line_refs()),
    ids=lambda v: v if isinstance(v, (str, int)) else getattr(v, 'name', getattr(v, 'who', '')),
)
def test_line_references_point_at_the_topic(row, end, path, line):
    # The referenced line names the topic, the message type or the node. A
    # call split over lines opens on the referenced line, and the two after
    # it count; a reference one line early no longer passes on its neighbour.
    lines = (ROOT / path).read_text().splitlines()
    first, last = line - 1, line + 2
    window = lines[first]
    if window.rstrip().endswith('('):
        window = ' '.join(lines[first:last])
    needles = {row.name, row.msg_type.rsplit('/', 1)[-1], end.who.split()[0]}
    assert any(n in window for n in needles), f'{path}:{line} for {row.name} ({end.who})'


def test_the_table_covers_the_feature_map():
    assert FEATURE_MAP_ROWS <= {r.name for r in TABLE}


def test_names_are_unique():
    names = [r.name for r in TABLE]
    assert len(names) == len(set(names))
