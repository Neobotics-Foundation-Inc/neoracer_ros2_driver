"""
Topic contract: what the student library and the lab dashboards read and write.

Library v0.9.0-bot (MITRacecarNeo/racecar-neo-library), the Neobotics fork
of the library that deployed cars run until F45 resyncs it, and the seven
NeoRacer lab dashboards (camlabel, eps, linefollow, pursuit, smartfollow,
teleop, wallfollow) reach the car only through the topics, services, action
and TF frames below. A
topic the library reads that nothing publishes does not fail: the library
reads zeros. So every row is checked against the driver's own sources: the
driver side exists with the message type, and the QoS on each end lets the
two connect (a RELIABLE subscriber hears nothing from a BEST_EFFORT
publisher; a TRANSIENT_LOCAL subscriber needs a TRANSIENT_LOCAL publisher).
Every read of the deployed fork must be live: the 0.9.0 driver may not break
the cars in the field.

The readers' QoS comes from their sources at the refs named in each End.
Publishers outside this repo are checked against the 0.6.1 baseline bag
recorded on the dev car on 2026-11-11 (BASELINE). Rows a later port feature
adds are skipped with that feature's name until it lands, so the table grows
as the port does.
"""

from dataclasses import dataclass
from pathlib import Path
import re

import pytest

ROOT = Path(__file__).resolve().parent.parent

BE, REL = 'best_effort', 'reliable'
VOL, TL = 'volatile', 'transient_local'

LIB = 'library v0.9.0-bot'
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


CAMERA_READERS = (
    End(LIB, 'library/real/camera_real.py:64', BE),
    _fork('camera_real.py:49 (depth 10)'),
    _sensor(f'{DASH} camlabel', 'camlabel_dashboard/camlabel.py:62'),
    _sensor(f'{DASH} linefollow', 'linefollow_dashboard/linefollow.py:210'),
    _sensor(f'{DASH} pursuit', 'pursuit_dashboard/pursuit.py:91'),
    _sensor(f'{DASH} smartfollow', 'smartfollow_dashboard/smartfollow.py:177'),
    _sensor(f'{DASH} teleop', 'teleop_dashboard/teleop.py:170'),
    End('inference_node', 'neoracer_ros2_driver/inference_node.py:135', BE),
)
ODOM_READERS = tuple(
    _sensor(f'{DASH} {name}', f'{name}_dashboard/{name}.py:{line}')
    for name, line in (
        ('eps', 324),
        ('linefollow', 211),
        ('smartfollow', 176),
        ('teleop', 172),
        ('wallfollow', 116),
    )
)
SCAN_READERS = (
    End(LIB, 'library/real/lidar_real.py:36 (qos_profile_sensor_data)', BE),
    _fork('lidar_real.py:41 (qos_profile_sensor_data)'),
    _sensor(f'{DASH} eps', 'eps_dashboard/eps.py:322'),
    _sensor(f'{DASH} smartfollow', 'smartfollow_dashboard/smartfollow.py:175'),
    _sensor(f'{DASH} teleop', 'teleop_dashboard/teleop.py:171'),
    _sensor(f'{DASH} wallfollow', 'wallfollow_dashboard/wallfollow.py:115'),
)
DRIVE_WRITERS = (
    End(LIB, 'library/real/drive_real.py:34 (depth 1)', REL),
    _fork('drive_real.py:40 (depth 1)', REL),
) + tuple(
    End(f'{DASH} {name}', f'{name}_dashboard/{name}.py:{line} (depth 1)', REL)
    for name, line in (
        ('eps', 321),
        ('linefollow', 209),
        ('pursuit', 90),
        ('smartfollow', 174),
        ('teleop', 169),
        ('wallfollow', 114),
    )
)
CONTROLLER = 'neoracer_ros2_driver/controller.py'
PHYSICS = 'library/real/physics_real.py:54'
FORK_PHYSICS = 'physics_real.py:57 (depth 1)'
AUTONOMY = 'launch/autonomy.launch.py'
FRAMES_URDF = 'urdf/neoracer.urdf.xacro'
TELEOP = 'launch/teleop.launch.py'
SLAM_INCLUDE = (
    AUTONOMY,
    "get_package_share_directory('osracer_slam'), 'launch', 'slam_toolbox.launch.py'",
)
NAV_INCLUDE = (
    AUTONOMY,
    "get_package_share_directory('osracer_navigation'), 'launch', 'nav2.launch.py'",
)
VENDOR = 'live under racecar mapping and racecar navigation, through the vendor stack'

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
        publishers=(End('controller_node', f'{CONTROLLER}:176', REL),),
        subscribers=(
            End(LIB, 'library/real/controller_real.py:89 (depth 1)', REL),
            _fork('controller_real.py:89 (depth 1)', REL),
            End('mux_node', 'neoracer_ros2_driver/mux_node.py:96', BE),
            End('gamepad_node', 'neoracer_ros2_driver/gamepad_node.py:51', BE),
        ),
        evidence=((CONTROLLER, "create_publisher(Joy, '/joy', 10)"),),
        note='both libraries subscribe RELIABLE, so /joy must stay RELIABLE',
    ),
    Row(
        '/scan',
        'sensor_msgs/msg/LaserScan',
        publishers=(End('lakibeam1 scan node', 'launch/lidar.launch.py', REL),),
        subscribers=SCAN_READERS,
        evidence=(('launch/lidar.launch.py', 'lakibeam1'),),
    ),
    Row(
        '/imu/fused',
        'sensor_msgs/msg/Imu',
        publishers=(End('controller_node', f'{CONTROLLER}:150', REL),),
        subscribers=(End(LIB, PHYSICS, BE), _fork(FORK_PHYSICS)),
        evidence=((CONTROLLER, "create_publisher(Imu, '/imu/fused', 10)"),),
    ),
    Row(
        '/mag',
        'sensor_msgs/msg/MagneticField',
        publishers=(End('controller_node', f'{CONTROLLER}:178', BE),),
        subscribers=(End(LIB, PHYSICS, BE), _fork(FORK_PHYSICS)),
        evidence=(
            (CONTROLLER, "create_publisher(MagneticField, '/mag', qos_profile_sensor_data)"),
            ('config/controller.yaml', 'publish_mag: true'),
        ),
    ),
    Row(
        '/encoder/speed',
        'std_msgs/msg/Float32',
        publishers=(End('controller_node', f'{CONTROLLER}:165', BE),),
        subscribers=(End(LIB, PHYSICS, BE), _fork(FORK_PHYSICS)),
        evidence=(
            (CONTROLLER, "create_publisher(Float32, '/encoder/speed', qos_profile_sensor_data)"),
        ),
    ),
    Row(
        '/battery/state',
        'sensor_msgs/msg/BatteryState',
        publishers=(End('controller_node', f'{CONTROLLER}:156', REL),),
        subscribers=(End(LIB, PHYSICS, BE),),
        evidence=((CONTROLLER, "create_publisher(BatteryState, '/battery/state', 10)"),),
        note='added by F07',
    ),
    Row(
        '/rc/channels',
        'std_msgs/msg/Float32MultiArray',
        publishers=(End('controller_node', f'{CONTROLLER}:168', BE),),
        subscribers=(End(LIB, PHYSICS, BE), _fork(FORK_PHYSICS)),
        evidence=(
            (
                CONTROLLER,
                "create_publisher(Float32MultiArray, '/rc/channels', qos_profile_sensor_data)",
            ),
        ),
    ),
    Row(
        '/edgetpu/inference',
        'vision_msgs/msg/Detection2DArray',
        publishers=(End('inference_node', 'neoracer_ros2_driver/inference_node.py:130', REL),),
        subscribers=(
            End(LIB, 'library/real/vision_real.py:34', BE),
            _fork('vision_real.py:34'),
            End(f'{DASH} pursuit', 'pursuit_dashboard/pursuit.py:92 (depth 10)', REL),
            End(f'{DASH} smartfollow', 'smartfollow_dashboard/smartfollow.py:178 (depth 10)', REL),
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
        ),
        note='pursuit and smartfollow subscribe RELIABLE, so detections must stay RELIABLE',
    ),
    Row(
        '/diagnostics',
        'diagnostic_msgs/msg/DiagnosticArray',
        publishers=(End('inference_node', 'neoracer_ros2_driver/inference_node.py:134', REL),),
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
        publishers=(End('autonomy_node', 'upstream autonomy_node.py:209 (latched)', REL, TL),),
        subscribers=(End(LIB, 'library/real/slam_real.py:45 (latched)', REL, TL),),
        pending='F23',
    ),
    Row(
        '/autonomy/pose',
        'geometry_msgs/msg/PoseWithCovarianceStamped',
        publishers=(End('autonomy_node', 'upstream autonomy_node.py:208', REL),),
        subscribers=(End(LIB, 'library/real/slam_real.py:54 (depth 10)', REL),),
        pending='F23',
    ),
    Row(
        '/map',
        'nav_msgs/msg/OccupancyGrid',
        publishers=(
            End('slam_toolbox', f'{AUTONOMY} slam:=true (osracer_slam)', REL, TL),
            End('nav2 map_server', f'{AUTONOMY} nav:=true (osracer_navigation)', REL, TL),
        ),
        subscribers=(
            End(LIB, 'library/real/slam_real.py:57 (latched)', REL, TL),
            _fork('slam_real.py:52 (latched)', REL, TL),
        ),
        evidence=(SLAM_INCLUDE, NAV_INCLUDE),
        note=f'{VENDOR}; F24 replaces the vendor SLAM and must keep /map latched',
    ),
    Row(
        '/autonomy/save_map',
        'std_srvs/srv/Trigger',
        subscribers=(End(LIB, 'library/real/slam_real.py:59 (client)', REL),),
        kind='service',
        pending='F23',
    ),
    Row(
        '/autonomy/global_localization',
        'std_srvs/srv/Trigger',
        subscribers=(End('map viewer', 'upstream scripts/mapview.py:378 (client)', REL),),
        kind='service',
        pending='F23',
        note='called by the map viewer (F26) and the SLAM validation (F41)',
    ),
    # -- written by the library and the dashboards --
    Row(
        '/drive',
        'ackermann_msgs/msg/AckermannDriveStamped',
        publishers=DRIVE_WRITERS,
        subscribers=(End('mux_node', 'neoracer_ros2_driver/mux_node.py:98', BE),),
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
        publishers=(End('controller_node', f'{CONTROLLER}:151', REL),),
        subscribers=ODOM_READERS,
        evidence=((CONTROLLER, "create_publisher(Odometry, '/odom', 10)"),),
    ),
    # -- the drive path and the other driver topics in the feature map --
    Row(
        '/gamepad_drive',
        'ackermann_msgs/msg/AckermannDriveStamped',
        publishers=(End('gamepad_node', 'neoracer_ros2_driver/gamepad_node.py:50', BE),),
        subscribers=(End('mux_node', 'neoracer_ros2_driver/mux_node.py:97', BE),),
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
        publishers=(End('mux_node', 'neoracer_ros2_driver/mux_node.py:94', BE),),
        subscribers=(End('throttle_node', 'neoracer_ros2_driver/throttle_node.py:43', BE),),
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
        publishers=(End('throttle_node', 'neoracer_ros2_driver/throttle_node.py:42', BE),),
        subscribers=(End('controller_node', f'{CONTROLLER}:141', BE),),
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
        publishers=(End('controller_node', f'{CONTROLLER}:153', BE),),
        subscribers=(End('8080 dashboard', 'scripts/dashboard.py:123', BE),),
        evidence=(
            (CONTROLLER, "create_publisher(BatteryState, '/battery', qos_profile_sensor_data)"),
        ),
        note='kept one more release for the 8080 page (F07)',
    ),
    Row(
        '/battery/voltage',
        'std_msgs/msg/Float32',
        publishers=(End('controller_node', f'{CONTROLLER}:162', BE),),
        evidence=(
            (CONTROLLER, "create_publisher(Float32, '/battery/voltage', qos_profile_sensor_data)"),
        ),
        subscribers=(_fork(FORK_PHYSICS),),
        note='the deployed fork reads it (physics_real.py, physics.py); kept until F45',
    ),
    Row(
        '/rc/link',
        'std_msgs/msg/Bool',
        publishers=(End('controller_node', f'{CONTROLLER}:172', BE),),
        evidence=((CONTROLLER, "create_publisher(Bool, '/rc/link', qos_profile_sensor_data)"),),
        note='added by F08; the mux subscribes in F12',
    ),
    Row(
        '/odom/integrated',
        'nav_msgs/msg/Odometry',
        subscribers=(End('analyze_odom and calibrate_dynamics', 'upstream scripts', BE),),
        pending='F09',
        note='read by the calibrations (F39) and analyze_odom (F40)',
    ),
    Row(
        'TF base_link -> laser',
        'tf2_msgs/msg/TFMessage',
        publishers=(End('robot_state_publisher (frames, /tf_static)', FRAMES_URDF, REL, TL),),
        subscribers=(End('SLAM and the lidar calibration', 'upstream F24 and F36', REL, TL),),
        evidence=((FRAMES_URDF, '<child link="laser" />'), (TELEOP, "'frames': 'true'")),
        kind='frame',
        note='added by F05; library v0.9.2-bot reads its yaw in the 0.9.2 port',
    ),
    # -- read by the deployed library fork through the vendor autonomy stack --
    Row(
        'TF map -> base_footprint',
        'tf2_msgs/msg/TFMessage',
        publishers=(
            End('robot_localization EKF (odom -> base_link)', f'{AUTONOMY}:130', REL),
            End('robot_state_publisher (base_link -> base_footprint, static)', FRAMES_URDF, REL),
            End('slam_toolbox (map -> odom)', f'{AUTONOMY} slam:=true', REL),
        ),
        subscribers=(
            _fork('slam_real.py:77 (lookup_transform)', REL),
            _fork('nav_real.py:64 (lookup_transform)', REL),
        ),
        evidence=(
            (AUTONOMY, "'base_link_frame': 'base_link'"),
            (FRAMES_URDF, '<child link="base_footprint" />'),
            (TELEOP, "'frames': 'true'"),
            SLAM_INCLUDE,
        ),
        kind='frame',
        note=(
            f'{VENDOR}; since F05 the EKF publishes odom -> base_link and the teleop frames '
            'publish base_footprint as a static child of base_link for one release; '
            'F24 keeps map -> odom'
        ),
    ),
    Row(
        '/slam_toolbox/serialize_map',
        'slam_toolbox/srv/SerializePoseGraph',
        subscribers=(_fork('slam_real.py:63 (client)', REL),),
        evidence=(SLAM_INCLUDE,),
        kind='service',
        note=f'{VENDOR}; F24 must keep slam_toolbox under this name until F45',
    ),
    Row(
        '/slam_toolbox/deserialize_map',
        'slam_toolbox/srv/DeserializePoseGraph',
        subscribers=(_fork('slam_real.py:66 (client)', REL),),
        evidence=(SLAM_INCLUDE,),
        kind='service',
        note=VENDOR,
    ),
    Row(
        '/slam_toolbox/clear_changes',
        'slam_toolbox/srv/Clear',
        subscribers=(_fork('slam_real.py:69 (client)', REL),),
        evidence=(SLAM_INCLUDE,),
        kind='service',
        note=VENDOR,
    ),
    Row(
        '/compute_path_to_pose',
        'nav2_msgs/action/ComputePathToPose',
        subscribers=(_fork('nav_real.py:48 (action client)', REL),),
        evidence=(NAV_INCLUDE,),
        kind='action',
        note='live under racecar navigation (vendor Nav2); plan D1 decides how rc.nav is served',
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

LIVE = [r for r in TABLE if not r.pending and not r.absent]


def _squash(text):
    return re.sub(r'\s+', '', text)


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
    _skip_unless_live(row)
    assert row.evidence, f'{row.name} has no evidence in the driver sources'
    for path, text in row.evidence:
        source = _squash((ROOT / path).read_text())
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
    for name in FORK_READS:
        row = rows[name]
        assert any(s.who == FORK for s in row.subscribers), name
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


def test_the_table_covers_the_feature_map():
    assert FEATURE_MAP_ROWS <= {r.name for r in TABLE}


def test_names_are_unique():
    names = [r.name for r in TABLE]
    assert len(names) == len(set(names))
