"""
autonomy_node: the one SLAM interface for the CLI, the library, and dashboards.

Launched by launch/autonomy.launch.py beside slam_toolbox (mapping) or a
localizer (AMCL or the slam_toolbox localizer), so consumers see the same
topics whichever runs:

    /autonomy/pose       PoseWithCovarianceStamped, map frame, pose_rate_hz
    /autonomy/status     DiagnosticStatus, transient local, 1 Hz and on change
    /autonomy/save_map   Trigger: mapping only; pose graph, .pgm/.yaml, meta.yaml
    /autonomy/autosave   Trigger: mapping only; the same set into <map>/autosave/
    /autonomy/global_localization  Trigger: AMCL only; search the whole map

In mapping it autosaves every autosave_period_sec and keeps the car's map pose
in <map>/mapping_pose.yaml each second for a crash resume. A save into a
folder that holds another run's map moves those files to <map>/previous/
first. meta.yaml carries the run's start pose and last change; a run that
began from another map (`source`: a resume or an extension) carries them over
from that map's meta.yaml. In localization it writes
<map>/last_pose.yaml every last_pose_period_sec while localized, and on exit.
With pose 'global' (AMCL only) it asks AMCL to search the whole map once AMCL
is up. AMCL takes its start pose with no spread and would never correct it, so
once AMCL is up this node sends the start pose again on /initialpose with
start_spread_xy_m and start_spread_yaw_deg; the no-motion updates then match it
to the scan before `localized` turns true. On exit it publishes mode 'off',
which lifts the mapping caps in throttle_node. File layout and the
localized test: autonomy.py.
"""

from __future__ import annotations

from datetime import datetime
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
from typing import Any
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from . import autonomy as au
from .frames import load_frames
from .maps import map_stem, parse_pose

PACKAGE = 'neoracer_ros2_driver'
SERVICE_TIMEOUT_SEC = 60.0
MAP_SAVER_TIMEOUT_SEC = 60.0
# map_saver_cli's wait for /map; its 2 s default can run out under nice while driving.
MAP_WAIT_SEC = 10.0


def driver_version() -> str:
    try:
        share = Path(get_package_share_directory(PACKAGE))
        return ET.parse(share / 'package.xml').findtext('version') or 'unknown'
    except Exception:  # noqa: BLE001  meta.yaml still gets written
        return 'unknown'


def yaw_of(q: Any) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class AutonomyNode(Node):
    def __init__(self) -> None:
        super().__init__('autonomy_node')
        p = self.declare_parameter
        p('mode', 'mapping')
        p('map', '')
        p('localizer', 'amcl')
        p('session', '')
        p('pose', '')
        p('pose_source', '')
        p('resumed_from_autosave', False)
        p('source', '')
        p('extended_from', '')
        p('pose_rate_hz', 20.0)
        p('autosave_period_sec', 60.0)
        p('last_pose_period_sec', 5.0)
        p('scan_timeout_sec', 0.5)
        p('localized_max_xy_sd_m', 0.25)
        p('localized_max_yaw_sd_deg', 20.0)
        p('start_spread_xy_m', 0.25)
        p('start_spread_yaw_deg', 10.0)
        p('fit_rate_hz', 2.0)
        p('fit_beams', 180)
        p('fit_min_range_m', 0.3)
        p('fit_window_sec', 2.0)
        p('localized_min_fit', 0.5)
        p('localized_enter_fit', 0.65)

        def get(name: str) -> Any:
            return self.get_parameter(name).value

        self._mode = str(get('mode'))
        self._map = str(get('map'))
        self._localizer = str(get('localizer'))
        self._session = str(get('session'))
        self._pose_arg = str(get('pose'))
        self._pose_source = str(get('pose_source'))
        self._resumed = bool(get('resumed_from_autosave'))
        self._limits = au.Limits(
            float(get('scan_timeout_sec')),
            float(get('localized_max_xy_sd_m')),
            float(get('localized_max_yaw_sd_deg')),
        )
        self._stem = map_stem(self._map) if self._map else None
        source = str(get('source'))
        carried = au.read_record(Path(source).parent / au.META_FILE) if source else {}
        self._from_source = bool(source)
        self._start_pose = au.pose_of(carried.get('start_pose'))
        self._last_change = str(carried.get('last_change') or '')
        self._extended_from = str(get('extended_from')) or str(carried.get('extended_from') or '')
        self._global_pending = self._mode == 'localization' and self._pose_arg == 'global'
        self._global_requested = False
        self._global_spread_seen = False
        # AMCL from a given or saved pose: re-seeded with a spread once it is up.
        self._seed: tuple[float, float, float] | None = None
        if self._uses_amcl() and not self._global_pending:
            self._seed = parse_pose(self._pose_arg) or (0.0, 0.0, 0.0)
        self._seed_sent: float | None = None
        self._seed_cov = au.seed_covariance(
            float(get('start_spread_xy_m')), float(get('start_spread_yaw_deg'))
        )

        # Live scan-to-map fit (localization): a pose that passes the spread
        # test but not the scan is not localized; see autonomy.FitGate.
        # The map comes from /map, the one the localizer uses.
        self._walls: au.WallMask | None = None
        self._fit_period = 1.0 / float(get('fit_rate_hz'))
        self._fit_beams = int(get('fit_beams'))
        self._fit_min_range = float(get('fit_min_range_m'))
        self._fit_at = 0.0
        self._fit_wanted = self._mode == 'localization'
        self._gate = au.FitGate(
            float(get('fit_window_sec')),
            float(get('localized_min_fit')),
            float(get('localized_enter_fit')),
        )

        self._start = time.monotonic()
        self._last_scan: float | None = None
        self._sd: tuple[float, float] | None = None
        self._poses_seen = 0
        self._last_save = ''
        self._last_autosave = ''
        self._autosaved_at: float | None = None
        self._localized = False
        self._pose: tuple[float, float, float] | None = None
        self._latest_cov: list[float] | None = None
        self._map_base_stamp: Any = None
        self._save_lock = threading.Lock()
        self._stopping = False

        self._tf = Buffer()
        TransformListener(self._tf, self)
        clients = ReentrantCallbackGroup()
        self._saves = MutuallyExclusiveCallbackGroup()

        sensor = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        latched_map = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Raw: the arrival is recorded for every scan; only the fit_rate_hz
        # scans scored for the fit are deserialized.
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor, raw=True)
        if self._uses_amcl():
            # AMCL keeps its last pose (transient local); taking it covers an
            # update published before this node subscribed.
            kept = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, kept)
        else:
            self.create_subscription(PoseWithCovarianceStamped, '/pose', self._on_pose, 10)
        if self._fit_wanted:
            self.create_subscription(OccupancyGrid, '/map', self._on_map, latched_map)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/autonomy/pose', 10)
        self._pub_status = self.create_publisher(DiagnosticStatus, '/autonomy/status', latched)
        self._pub_initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_service(
            Trigger, '/autonomy/save_map', self._on_save_map, callback_group=clients
        )
        self.create_service(
            Trigger, '/autonomy/autosave', self._on_autosave, callback_group=clients
        )
        self.create_service(
            Trigger, '/autonomy/global_localization', self._on_global, callback_group=clients
        )

        from slam_toolbox.srv import SerializePoseGraph

        self._serialize = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map', callback_group=clients
        )
        self._reinit = self.create_client(
            Empty, '/reinitialize_global_localization', callback_group=clients
        )
        self._nomotion = self.create_client(
            Empty, '/request_nomotion_update', callback_group=clients
        )
        self._srv_type = SerializePoseGraph

        self.create_timer(1.0 / float(get('pose_rate_hz')), self._publish_pose)
        self.create_timer(1.0, self._tick)
        # localized can drop between ticks (a stopped lidar); publish the change
        # within a tenth of a second rather than at the next tick.
        self.create_timer(0.1, self._watch)
        if self._mode == 'mapping':
            self.create_timer(
                float(get('autosave_period_sec')), self._timed_autosave, callback_group=self._saves
            )
        else:
            self.create_timer(float(get('last_pose_period_sec')), self._write_last_pose)
        self._publish_status()
        self.get_logger().info(
            f'autonomy_node: {self._mode} on {self._map or "(no map)"}'
            + (f' with {self._localizer}' if self._mode == 'localization' else '')
        )

    def _uses_amcl(self) -> bool:
        return self._mode == 'localization' and self._localizer == 'amcl'

    # -- inputs --

    def _on_scan(self, raw: bytes) -> None:
        now = time.monotonic()
        self._last_scan = now
        if self._walls is not None and now - self._fit_at >= self._fit_period:
            self._fit_at = now
            self._score_scan(self._walls, deserialize_message(raw, LaserScan), now)

    def _score_scan(self, walls: au.WallMask, scan: LaserScan, now: float) -> None:
        try:
            stamp = Time.from_msg(scan.header.stamp)
            t = self._tf.lookup_transform('map', scan.header.frame_id, stamp)
        except TransformException:
            try:
                t = self._tf.lookup_transform('map', scan.header.frame_id, Time())
            except TransformException:
                return
        tr = t.transform.translation
        x, y = au.scan_points_in_map(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            (tr.x, tr.y, yaw_of(t.transform.rotation)),
            max(self._fit_min_range, scan.range_min),
            scan.range_max,
            self._fit_beams,
        )
        self._gate.add(now, walls.share(x, y))

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        self._walls = au.WallMask.from_grid(
            msg.data,
            info.width,
            info.height,
            (info.origin.position.x, info.origin.position.y),
            info.resolution,
        )

    def _fit_ok(self, now: float) -> bool:
        # Until /map arrives the fit cannot be judged; localized waits for it.
        return not self._fit_wanted or self._gate.passes(now, self._localized)

    def _seeding(self) -> bool:
        """Report whether AMCL's re-seeded start pose is still on its way."""
        if self._seed is not None:
            return True
        if self._seed_sent is None:
            return False
        if time.monotonic() - self._seed_sent > au.SEED_WAIT_SEC:
            self._seed_sent = None
        elif self._sd is not None and au.shows_seed(self._sd, self._seed_cov):
            self._seed_sent = None
        return self._seed_sent is not None

    def _send_seed(self) -> None:
        if self._seed is None or self._poses_seen == 0:
            return  # AMCL drops /initialpose until it is active and has published a pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        x, y, yaw = self._seed
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(
            yaw / 2
        )
        msg.pose.covariance = self._seed_cov
        self._pub_initial.publish(msg)
        self._seed, self._seed_sent = None, time.monotonic()
        self.get_logger().info('Start pose sent to AMCL with a spread; matching it to the scan.')

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._poses_seen += 1
        if self._mode == 'mapping':
            # slam_toolbox publishes a pose for each scan it adds to the map.
            self._last_change = au.change_time(datetime.now())
        self._sd = au.covariance_sd(list(msg.pose.covariance))
        if self._global_pending and self._global_requested and self._sd is not None:
            within = (
                self._sd[0] <= self._limits.max_xy_sd_m
                and self._sd[1] <= self._limits.max_yaw_sd_deg
            )
            # A pose from before the search started still has a small spread.
            if not within:
                self._global_spread_seen = True
            elif self._global_spread_seen:
                self._global_pending = False
                self.get_logger().info('Global localization converged.')
        self._latest_cov = list(msg.pose.covariance)

    # -- outputs --

    def _publish_pose(self) -> None:
        try:
            t = self._tf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            self._map_base_stamp = None
            return
        self._map_base_stamp = t.header.stamp
        out = PoseWithCovarianceStamped()
        out.header.stamp = t.header.stamp
        out.header.frame_id = 'map'
        tr, q = t.transform.translation, t.transform.rotation
        out.pose.pose.position.x, out.pose.pose.position.y = tr.x, tr.y
        out.pose.pose.orientation = q
        if self._latest_cov is not None:
            out.pose.covariance = self._latest_cov
        self._pose = (tr.x, tr.y, yaw_of(q))
        if self._mode == 'mapping' and self._start_pose is None and not self._from_source:
            self._start_pose = self._pose
        self._pub_pose.publish(out)

    def _has_map_odom(self) -> bool:
        return self._tf.can_transform('map', 'odom', Time())

    def _tf_age_ms(self) -> float | None:
        if self._map_base_stamp is None:
            return None
        age = (self.get_clock().now() - Time.from_msg(self._map_base_stamp)).nanoseconds * 1e-6
        return max(age, 0.0)

    def _localized_now(self) -> bool:
        scan_age = None if self._last_scan is None else time.monotonic() - self._last_scan
        return au.is_localized(
            self._mode,
            self._has_map_odom(),
            scan_age,
            self._sd,
            self._poses_seen,
            self._global_pending or self._seeding(),
            self._limits,
            self._localized,
            self._fit_ok(time.monotonic()),
        )

    def _watch(self) -> None:
        if self._localized_now() != self._localized:
            self._publish_status()

    def _tick(self) -> None:
        if self._mode == 'mapping' and self._pose is not None and self._stem is not None:
            # A crash restart resumes near here (autonomy.resume_point).
            au.write_yaml_atomic(
                self._stem.parent / au.MAPPING_POSE_FILE,
                au.pose_record(self._pose, datetime.now(), session=self._session),
            )
        if self._global_pending and not self._global_requested and self._reinit.service_is_ready():
            self._reinit.call_async(Empty.Request())
            self._global_requested = True
            self.get_logger().info('Searching the whole map (reinitialize_global_localization).')
        self._send_seed()
        # AMCL updates, and publishes the spread localized needs, only after the
        # car moves; from a given start pose this lets a parked car localize.
        # Not during a global search, where it could settle on a wrong match.
        if (
            self._uses_amcl()
            and not self._localized
            and not self._global_pending
            and self._nomotion.service_is_ready()
        ):
            self._nomotion.call_async(Empty.Request())
        self._publish_status()

    def _publish_status(self, mode: str | None = None) -> None:
        if self._stopping and mode is None:
            return
        now = time.monotonic()
        scan_age = None if self._last_scan is None else now - self._last_scan
        has_map_odom = self._has_map_odom()
        localized = mode is None and au.is_localized(
            self._mode,
            has_map_odom,
            scan_age,
            self._sd,
            self._poses_seen,
            self._global_pending or self._seeding(),
            self._limits,
            self._localized,
            self._fit_ok(now),
        )
        if localized != self._localized:
            self.get_logger().info(f'localized: {str(localized).lower()}')
        self._localized = localized
        msg = DiagnosticStatus()
        msg.name = 'autonomy'
        msg.hardware_id = self._map
        level = (
            au.LEVEL_OK if mode else au.status_level(localized, has_map_odom, now - self._start)
        )
        msg.level = bytes([level])
        msg.message = 'off' if mode else ('localized' if localized else 'not localized')
        sd = self._sd or (None, None)
        msg.values = [
            KeyValue(key=k, value=v)
            for k, v in au.status_values(
                mode=mode or self._mode,
                map=self._map,
                localizer=self._localizer if self._mode == 'localization' else '',
                localized=localized,
                tf_age_ms=self._tf_age_ms(),
                scan_age_ms=None if scan_age is None else scan_age * 1e3,
                pose_sd_m=sd[0],
                pose_sd_deg=sd[1],
                scan_fit=self._gate.mean(now) if self._fit_wanted else None,
                last_save=self._last_save,
                last_autosave=self._last_autosave,
                resumed_from_autosave=self._resumed,
                pose_source=self._pose_source,
                scans_processed=self._poses_seen,
                session=self._session,
            )
        ]
        self._pub_status.publish(msg)

    # -- saves --

    def _call(self, client: Any, request: Any) -> Any:
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        future = client.call_async(request)
        end = time.monotonic() + SERVICE_TIMEOUT_SEC
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _save_set(self, stem: Path) -> str:
        """Write the pose graph and the .pgm/.yaml at a stem; return '' or the failure."""
        stem.parent.mkdir(parents=True, exist_ok=True)
        result = self._call(self._serialize, self._srv_type.Request(filename=str(stem)))
        if result is None or result.result != self._srv_type.Response.RESULT_SUCCESS:
            return 'slam_toolbox could not write the pose graph'
        # Low priority: starting map_saver_cli costs seconds of CPU, which
        # slam_toolbox needs while the car moves.
        argv = [
            'nice', '-n', '10',
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(stem),
            '--ros-args', '-p', 'map_subscribe_transient_local:=true',
            '-p', f'save_map_timeout:={MAP_WAIT_SEC}',
        ]  # fmt: skip
        try:
            done = subprocess.run(
                argv, capture_output=True, timeout=MAP_SAVER_TIMEOUT_SEC, check=False
            )
        except subprocess.TimeoutExpired:
            return 'map_saver_cli timed out'
        if done.returncode == 0:
            return ''
        lines = (done.stderr or done.stdout or b'').decode(errors='replace').strip().splitlines()
        return 'map_saver_cli failed' + (f': {lines[-1]}' if lines else '')

    def _meta(self) -> dict[str, Any]:
        try:
            frames = load_frames(Path(get_package_share_directory(PACKAGE)) / 'config')
        except Exception:  # noqa: BLE001  meta.yaml still gets written
            frames = {}
        return au.meta_record(
            self._map,
            datetime.now(),
            driver_version(),
            frames,
            self._session,
            self._start_pose,
            self._last_change,
            self._extended_from,
        )

    def save_map(self) -> tuple[bool, str]:
        if self._mode != 'mapping' or self._stem is None:
            return False, 'not mapping'
        folder, name = self._stem.parent, self._stem.name
        tmp = folder / 'save.tmp'
        with self._save_lock:
            t0 = time.monotonic()
            # The new files are complete before the old ones move, so a failed
            # save leaves the earlier map in place.
            if tmp.exists():
                shutil.rmtree(tmp)
            tmp.mkdir(parents=True)
            error = self._save_set(tmp / name)
            if error:
                return False, error
            note = ''
            earlier = au.read_record(folder / au.META_FILE)
            if earlier.get('session') != self._session:
                moved = au.archive_map_files(folder, name, datetime.now())
                if moved is not None:
                    note = f'; the earlier map moved to {moved}'
            for f in tmp.iterdir():
                os.replace(f, folder / f.name)
            tmp.rmdir()
            au.write_yaml_atomic(folder / au.META_FILE, self._meta())
            self._last_save = datetime.now().strftime(au.TIME_FORMAT)
        self._publish_status()
        return True, f'saved to {folder} in {time.monotonic() - t0:.1f} s{note}'

    def autosave(self) -> tuple[bool, str]:
        if self._mode != 'mapping' or self._stem is None:
            return False, 'not mapping'
        folder = self._stem.parent
        tmp = folder / (au.AUTOSAVE_DIR + '.tmp')
        with self._save_lock:
            t0 = time.monotonic()
            if tmp.exists():
                shutil.rmtree(tmp)
            tmp.mkdir(parents=True)
            error = self._save_set(tmp / self._map)
            if error:
                shutil.rmtree(tmp, ignore_errors=True)
                return False, error
            record = au.pose_record(
                self._pose or (0.0, 0.0, 0.0), datetime.now(), session=self._session
            )
            au.write_yaml_atomic(tmp / au.AUTOSAVE_FILE, record)
            au.write_yaml_atomic(tmp / au.META_FILE, self._meta())
            au.replace_dir(tmp, folder / au.AUTOSAVE_DIR)
            self._last_autosave = record['time']
            self._autosaved_at = time.monotonic()
        self.get_logger().info(f'Autosaved in {time.monotonic() - t0:.1f} s.')
        self._publish_status()
        return True, f'autosaved to {folder / au.AUTOSAVE_DIR}'

    def _timed_autosave(self) -> None:
        if self._poses_seen == 0:
            return
        # Just autosaved on request (the stop sequence): this one would repeat it
        # and run into the stop.
        if self._autosaved_at is not None and time.monotonic() - self._autosaved_at < 10.0:
            return
        ok, message = self.autosave()
        if not ok:
            self.get_logger().warning(f'Autosave failed: {message}')

    def _on_save_map(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self.save_map()
        return response

    def _on_autosave(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self.autosave()
        return response

    def _on_global(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self._uses_amcl():
            response.success, response.message = False, 'needs localization with AMCL'
            return response
        # The tick asks AMCL to spread its particles over the map, as for pose:=global.
        self._seed, self._seed_sent = None, None
        self._global_pending, self._global_requested = True, False
        self._global_spread_seen = False
        self._pose_source = 'global'
        self._publish_status()
        response.success, response.message = True, 'searching the whole map; drive slowly'
        return response

    def _write_last_pose(self) -> None:
        if self._localized and self._pose is not None and self._stem is not None:
            au.write_yaml_atomic(
                self._stem.parent / au.LAST_POSE_FILE,
                au.pose_record(self._pose, datetime.now(), localizer=self._localizer),
            )

    def shutdown(self) -> None:
        """Keep the last pose (localization) and publish mode 'off' (lifts the speed cap)."""
        self._stopping = True
        if self._mode == 'localization' and self._stem is not None:
            if self._localized:
                self._write_last_pose()
            else:
                au.mark_pose_lost(self._stem.parent / au.LAST_POSE_FILE, datetime.now())
        self._publish_status(mode='off')
        # Reliable delivery needs the publisher alive until subscribers acknowledge.
        acked = self._pub_status.wait_for_all_acked(Duration(seconds=2.0))
        self.get_logger().info(
            'Stopped: status off' + ('' if acked else ' (not acknowledged by every subscriber)')
        )


def _interrupt(signum: int, frame: Any) -> None:
    raise KeyboardInterrupt


def main(args: list[str] | None = None) -> None:
    # rclpy's handlers would shut the context down before shutdown() can send
    # the final status that lifts the mapping caps. These replace them,
    # and also cover a SIGINT inherited as ignored and systemd's SIGTERM.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    node = AutonomyNode()
    # Three threads: a save's handler blocks on slam_toolbox's reply, which a
    # second thread receives, while the third keeps the timers running. Each
    # thread costs CPU on every wake-up, so no more than that.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    # The executor spins in a thread; the main thread stays in Python, where the
    # signal handlers run even while the executor waits (a stopped sim clock
    # leaves it waiting in C indefinitely).
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        while spinner.is_alive():
            spinner.join(timeout=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # Callbacks stop first, so no status tick lands after the final one.
        executor.shutdown(timeout_sec=1.0)
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
