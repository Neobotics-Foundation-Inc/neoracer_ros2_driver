#!/usr/bin/env python3
"""
Map lifecycle behind `racecar map` (scripts/neoracer_map.py), importable on its own.

A change of mode runs these steps: check the request, stop
neoracer-autonomy.service (its ExecStop autosaves a mapping run and the node
writes last_pose.yaml), resolve the start pose from the files that leaves,
write ~/.config/racecar/autonomy.conf through a rename, start the unit, and
wait for /autonomy/status to report the new session. The unit's
mode file keys are read by scripts/autonomy_args.py.

list, info, and recover read the map folder only (neoracer_ros2_driver/
autonomy.py, map_state); start, load, save, and status import rclpy on first
use. Nothing under the map root is deleted: replaced folders go to
`replaced/`, replaced files to `<map>/previous/`.

Failures raise MapError carrying the exit code: 1 failed, 2 usage.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime
import math
import os
from pathlib import Path
import shlex
import subprocess
import time
from typing import Any

from neoracer_ros2_driver import autonomy as au, data_root
from neoracer_ros2_driver.maps import map_root, parse_pose, ROOT_ENV, valid_name

UNIT = 'neoracer-autonomy.service'
TELEOP_UNIT = 'neoracer-teleop.service'
MODE_FILE = Path.home() / '.config' / 'racecar' / 'autonomy.conf'
MODE_FILE_ENV = 'RACECAR_AUTONOMY_CONF'
KEYS = (
    'AUTONOMY_MODE',
    'AUTONOMY_MAP',
    'AUTONOMY_SESSION',
    'AUTONOMY_FROM',
    'AUTONOMY_POSE',
    'AUTONOMY_POSE_SOURCE',
    'AUTONOMY_LOCALIZER',
    'AUTONOMY_FOXGLOVE',
)
LOCALIZERS = ('amcl', 'slam_toolbox')
RESERVED_NAMES = (au.REPLACED_DIR,)
READY_TIMEOUT_SEC = 30.0
SAVE_TIMEOUT_SEC = 90.0
STATUS_TIMEOUT_SEC = 3.0
# A last pose older than this may not be where the car is now.
STALE_POSE_SEC = 600.0
# frames.yaml values (mm, deg) that differ by more than this are reported.
FRAME_TOLERANCE = 0.05
LOG_HINT = '~/logs/latest/autonomy_*/autonomy.log'

Out = Callable[[str], None]


class MapError(Exception):
    def __init__(self, message: str, code: int = 1) -> None:
        super().__init__(message)
        self.code = code


@dataclass
class Plan:
    """The mode file for a start or load, and what to tell the operator."""

    values: dict[str, str]
    notes: list[str] = field(default_factory=list)
    replace: bool = False


# -- mode file --


def mode_file() -> Path:
    return Path(os.environ.get(MODE_FILE_ENV) or MODE_FILE)


def read_mode_file(path: Path | None = None) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        lines = (path or mode_file()).read_text().splitlines()
    except OSError:
        return values
    for line in lines:
        key, sep, value = line.strip().partition('=')
        if sep and not key.startswith('#'):
            values[key.strip()] = value.strip()
    return values


def write_mode_file(values: dict[str, str], path: Path | None = None) -> None:
    """Write every key in KEYS through a rename; AUTONOMY_FOXGLOVE carries over unless given."""
    path = path or mode_file()
    merged = dict.fromkeys(KEYS, '')
    merged['AUTONOMY_FOXGLOVE'] = read_mode_file(path).get('AUTONOMY_FOXGLOVE', '0') or '0'
    merged.update(values)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(''.join(f'{k}={merged[k]}\n' for k in KEYS))
    os.replace(tmp, path)


def new_session(now: datetime) -> str:
    return now.strftime('%Y%m%d-%H%M%S')


# -- checks and pose sources (no ROS) --


def check_name(name: str, what: str = 'map name') -> None:
    if not valid_name(name):
        raise MapError(f'{what} must be letters, digits, - and _: {name!r}', 2)
    if name in RESERVED_NAMES:
        raise MapError(f'{name!r} is reserved. Pick another {what}', 2)


def format_pose(pose: tuple[float, float, float]) -> str:
    return f'{pose[0]:.4f},{pose[1]:.4f},{pose[2]:.5f}'


def describe_pose(pose: tuple[float, float, float]) -> str:
    return f'x {pose[0]:.2f} m, y {pose[1]:.2f} m, heading {math.degrees(pose[2]):.0f} deg'


def given_pose(text: str) -> tuple[float, float, float]:
    try:
        pose = parse_pose(text)
    except ValueError:
        pose = None
    if pose is None:
        raise MapError(f'--pose must be x,y,yaw (meters, meters, radians): {text!r}', 2)
    return pose


def _age_sec(stamp: str, now: datetime) -> float | None:
    try:
        return (now - datetime.strptime(stamp, au.TIME_FORMAT)).total_seconds()
    except ValueError:
        return None


def saved_pose(
    state: au.MapState, now: datetime
) -> tuple[tuple[float, float, float] | None, str, str, bool]:
    """
    Return (pose, source, description, trusted) from the map's files.

    Order: last_pose.yaml (where the car last localized on it), the start pose
    recorded when the map was begun, then the map origin. A last pose from a
    run that stopped unlocalized is not trusted: the car drove on after it.
    """
    record = au.read_record(state.folder / au.LAST_POSE_FILE)
    pose = au.pose_of(record)
    if pose is not None:
        when = str(record.get('time', ''))
        text = f'the last pose on {state.name}, from {when}'
        if record.get(au.LOCALIZED_AT_STOP) is False:
            text += (
                f'. The car was not localized when the run on {state.name} stopped at '
                f'{record.get("stopped", "?")}. It may have moved since'
            )
            return pose, 'last_pose', text, False
        age = _age_sec(when, now)
        if age is not None and age > STALE_POSE_SEC:
            text += '. If the car has moved since, give --pose or load the map and drive first'
        return pose, 'last_pose', text, True
    meta = state.meta if state.saved else state.autosave_meta
    start = au.pose_of(meta.get('start_pose'))
    if start is not None:
        return start, 'start_pose', f'where mapping of {state.name} began', True
    return None, 'origin', f'the origin of {state.name}', True


def current_frames() -> dict[str, float] | None:
    try:
        from ament_index_python.packages import get_package_share_directory
        from neoracer_ros2_driver.frames import load_frames

        return load_frames(Path(get_package_share_directory('neoracer_ros2_driver')) / 'config')
    except Exception:  # noqa: BLE001  the comparison is advisory
        return None


def frame_changes(meta: dict[str, Any], current: dict[str, float] | None) -> list[str]:
    """List frames.yaml values that differ from the ones the map was built with."""
    built = meta.get('frames')
    if not isinstance(built, dict) or not current:
        return []
    out = []
    for key, now_value in current.items():
        then = built.get(key)
        if isinstance(then, int | float) and abs(float(then) - now_value) > FRAME_TOLERANCE:
            out.append(f'{key} {then} when built, {now_value} now')
    return out


def plan_start(
    root: Path,
    name: str,
    session: str,
    source: str | None = None,
    pose: str | None = None,
    force: bool = False,
    now: datetime | None = None,
) -> Plan:
    now = now or datetime.now()
    check_name(name)
    if source is not None:
        check_name(source, 'map to extend')
    if pose is not None and source is None:
        raise MapError('--pose applies with --from (the car on the map being extended)', 2)
    if force and source == name:
        raise MapError('--force replaces a map; it cannot also extend it', 2)
    exists = (root / name).exists()
    if exists and not force and source != name:
        raise MapError(
            f'{name} exists. Load it with `racecar map load {name}`, '
            f'extend it with `racecar map start {name} --from {name}`, '
            'or replace it with --force (the old folder is kept under '
            f'{root / au.REPLACED_DIR}).'
        )
    plan = Plan(
        {
            'AUTONOMY_MODE': 'mapping',
            'AUTONOMY_MAP': name,
            'AUTONOMY_SESSION': session,
            'AUTONOMY_FROM': source or '',
        },
        replace=exists and force,
    )
    if source is None:
        return plan
    src = au.map_state(root, source)
    if not src.exists:
        raise MapError(f'no map named {source} under {root}')
    if source == name and src.unsaved:
        raise MapError(
            f'{name} has changes that were never saved (autosave from {src.autosave_time}). '
            f'Keep them first with `racecar map recover {name}`, then extend it.'
        )
    if not src.saved:
        plan.notes.append(
            f'{source} was never saved; extending its autosave from {src.autosave_time}.'
        )
    elif src.unsaved:
        plan.notes.append(
            f'{source} has an autosave from {src.autosave_time} with changes its saved map lacks; '
            f'extending the saved map. To include them, run `racecar map recover {source}` first.'
        )
    if pose is not None:
        start, how, text = given_pose(pose), 'pose', 'given with --pose'
    else:
        found, how, text, trusted = saved_pose(src, now)
        if not trusted:
            raise MapError(
                f'not extending from {text}. Load {source}, drive slowly until '
                '`racecar map status` reports localized, then extend; or give the '
                "car's pose with --pose."
            )
        start = found or (0.0, 0.0, 0.0)
    plan.values['AUTONOMY_POSE'] = format_pose(start)
    plan.values['AUTONOMY_POSE_SOURCE'] = how
    plan.notes.append(f'Start pose on {source}: {describe_pose(start)} ({text}).')
    plan.notes.extend(
        f'Frames changed since {source} was built: {c}.'
        for c in frame_changes(src.meta, current_frames())
    )
    return plan


def plan_load(
    root: Path,
    name: str,
    session: str,
    pose: str | None = None,
    search: bool = False,
    localizer: str = 'amcl',
    now: datetime | None = None,
) -> Plan:
    now = now or datetime.now()
    check_name(name)
    if localizer not in LOCALIZERS:
        raise MapError(f'--localizer must be amcl or slam_toolbox: {localizer!r}', 2)
    if pose is not None and search:
        raise MapError('--pose and --global exclude each other', 2)
    if search and localizer != 'amcl':
        raise MapError('--global needs the AMCL localizer', 2)
    state = au.map_state(root, name)
    if not state.exists:
        raise MapError(f'no map named {name} under {root}')
    if not state.saved:
        raise MapError(
            f'{name} was never saved. Keep it with `racecar map recover {name}`, then load it.'
        )
    plan = Plan(
        {
            'AUTONOMY_MODE': 'localization',
            'AUTONOMY_MAP': name,
            'AUTONOMY_SESSION': session,
            'AUTONOMY_LOCALIZER': localizer,
        }
    )
    if state.unsaved:
        plan.notes.append(
            f'{name} has an autosave from {state.autosave_time} with unsaved changes. '
            f'Loading the saved map. To keep them, run `racecar map recover {name}`.'
        )
    if search:
        plan.values['AUTONOMY_POSE'] = 'global'
        plan.values['AUTONOMY_POSE_SOURCE'] = 'global'
        plan.notes.append('Start pose: searching the whole map (--global).')
    else:
        found: tuple[float, float, float] | None
        if pose is not None:
            found, how, text = given_pose(pose), 'pose', 'given with --pose'
        else:
            found, how, text, trusted = saved_pose(state, now)
            if not trusted:
                text += '. If it has, use --global or --pose'
        plan.values['AUTONOMY_POSE'] = format_pose(found) if found else ''
        plan.values['AUTONOMY_POSE_SOURCE'] = how
        plan.notes.append(f'Start pose: {describe_pose(found or (0.0, 0.0, 0.0))} ({text}).')
    plan.notes.extend(
        f'Frames changed since {name} was built: {c}.'
        for c in frame_changes(state.meta, current_frames())
    )
    return plan


# -- systemd --


# The verbs that change a unit. They run through `sudo -n`, which
# /etc/sudoers.d/neoracer allows with no password for each neoracer unit by its
# full name. Reading a unit's state needs no sudo.
SUDO_VERBS = ('start', 'stop', 'restart')


def systemctl(*args: str) -> subprocess.CompletedProcess[str]:
    """
    Run a systemd command; start, stop and restart go through sudo.

    `sudo -n` fails at once when no sudoers rule allows the command, instead
    of waiting for a password. RACECAR_SYSTEMCTL replaces the systemctl
    command and RACECAR_SUDO the sudo prefix (tests).
    """
    argv = shlex.split(os.environ.get('RACECAR_SYSTEMCTL') or 'systemctl')
    if args and args[0] in SUDO_VERBS:
        argv = [*shlex.split(os.environ.get('RACECAR_SUDO', 'sudo -n')), *argv]
    return subprocess.run([*argv, *args], capture_output=True, text=True, check=False)


def sudo_hint(detail: str) -> str:
    """Say why sudo failed when the sudoers file is the cause, else return ''."""
    if 'password is required' in detail or 'not allowed to execute' in detail:
        return (
            ' sudo needs a password for this command, so /etc/sudoers.d/neoracer is not'
            ' installed or does not list it. That file lets the racecar user start and'
            ' stop the neoracer units with no password.'
        )
    return ''


def unit_state(unit: str = UNIT) -> str:
    return systemctl('is-active', unit).stdout.strip() or 'unknown'


def unit_running() -> bool:
    return unit_state() in ('active', 'activating', 'reloading')


def _control(action: str) -> None:
    done = systemctl(action, UNIT)
    if done.returncode != 0:
        detail = (done.stderr or done.stdout).strip()
        raise MapError(f'sudo systemctl {action} {UNIT} failed: {detail}.{sudo_hint(detail)}')


# Lab apps that publish /drive, in port order (8081 to 8087). Autonomy and
# these run one at a time, as the apps page keeps them: together they leave no
# CPU for the localizer or slam_toolbox. Starting mapping or localization stops
# them; the map viewer and the apps page ask first. Camera Label (8082) only
# reads the camera, so it is not here. /etc/sudoers.d/neoracer allows stopping
# them.
DRIVING_APPS = (
    ('neoracer-wallfollow.service', 'Wall Follow'),
    ('neoracer-pursuit.service', 'Pure Pursuit'),
    ('neoracer-eps.service', 'Policy Search'),
    ('neoracer-smartfollow.service', 'Smart Follow'),
    ('neoracer-linefollow.service', 'Line Follow'),
    ('neoracer-webteleop.service', 'Web Teleop'),
)


def running_driving_apps() -> list[str]:
    return [name for unit, name in DRIVING_APPS if unit_state(unit) in ('active', 'activating')]


def stop_driving_apps(out: Out) -> None:
    for unit, name in DRIVING_APPS:
        if unit_state(unit) not in ('active', 'activating'):
            continue
        done = systemctl('stop', unit)
        if done.returncode == 0:
            out(f'Stopped {name}.')
        else:
            hint = sudo_hint((done.stderr or done.stdout).strip())
            out(f'Could not stop {name}.{hint} Stop it from the apps page.')


def require_teleop() -> None:
    if unit_state(TELEOP_UNIT) != 'active':
        raise MapError('teleop is not running. Start it with `racecar service start teleop`.')


# -- speed switch (NeoRacer only, plan D12) --

# The FlySky sticks drive the car through the ESP32 firmware while the mode
# switch is UP, so the driver's mapping cap does not apply to them. The only
# limit there is the firmware's own: with the speed switch UP, the firmware
# scales the stick speed to 15 percent, 0.9 m/s of its 6 m/s. Mapping
# therefore starts only with the speed switch UP.
#
# The speed switch is radio Ch8, index 7 on /rc/channels. With the shipped
# calibration (rc_min 240, rc_center 1023, rc_max 1807 in config/controller.yaml)
# it reads -1.0 UP (SBUS count 240) and +1.0 DOWN (1807). A frame with no
# signal reads 0.0 on every channel, and so does the frame of 1023 counts the
# receiver sends while the handset reconnects. A reading at or below -0.5 is
# the switch UP; 0.0 and +1.0 are not.
SPEED_SWITCH_INDEX = 7
SPEED_SWITCH_UP_AT_MOST = -0.5
# A new process on the car can take a few seconds to discover the controller.
RC_WAIT_SEC = 5.0


def speed_switch_problem(channels: list[float] | None, link: bool | None) -> str:
    """
    Return why mapping cannot start with this handset reading, or '' when it can.

    `channels` is the latest /rc/channels, or None when none arrived. `link`
    is the latest /rc/link, or None when none arrived. With no /rc/link the
    channel alone counts: a frame with no signal reads 0.0, which is not UP.
    """
    if channels is None:
        return (
            f'no /rc/channels from the controller within {RC_WAIT_SEC:.0f} s, so the speed '
            'switch cannot be read. Check that teleop runs and the ESP32 is connected '
            '(`racecar status`).'
        )
    if link is False:
        return (
            'the handset is not linked (/rc/link is false). Switch the handset on, '
            'set the speed switch UP (slow), and start again.'
        )
    if len(channels) <= SPEED_SWITCH_INDEX:
        return (
            f'/rc/channels has {len(channels)} channels, so the speed switch (index 7) is missing.'
        )
    value = channels[SPEED_SWITCH_INDEX]
    if value > SPEED_SWITCH_UP_AT_MOST:
        return (
            f'the speed switch (Ch8) reads {value:.2f}, not UP. Mapping starts only with '
            'it UP (slow), because the firmware caps the sticks at 15 percent only then. '
            'Set it UP and start again.'
        )
    return ''


def require_speed_switch_up() -> None:
    problem = speed_switch_problem(*read_rc())
    if problem:
        raise MapError(problem)


# -- ROS --


@contextmanager
def ros_node() -> Iterator[Any]:
    import rclpy

    rclpy.init()
    node = rclpy.create_node('racecar_map')
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def status_dict(msg: Any) -> dict[str, str]:
    out = {kv.key: kv.value for kv in msg.values}
    out['level'] = str(msg.level[0] if isinstance(msg.level, bytes | bytearray) else msg.level)
    out['message'] = msg.message
    return out


def wait_status(
    node: Any,
    timeout: float,
    accept: Callable[[dict[str, str]], bool],
    alive: Callable[[], bool] | None = None,
) -> tuple[dict[str, str] | None, dict[str, str] | None]:
    """
    Spin until a /autonomy/status message passes `accept`, or the timeout.

    Returns (accepted, last seen). `alive` is checked each second; when it
    turns false the wait ends early.
    """
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

    seen: dict[str, Any] = {'last': None, 'hit': None}

    def on_status(msg: Any) -> None:
        values = status_dict(msg)
        seen['last'] = values
        if seen['hit'] is None and accept(values):
            seen['hit'] = values

    latched = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    sub = node.create_subscription(DiagnosticStatus, '/autonomy/status', on_status, latched)
    end = time.monotonic() + timeout
    next_check = time.monotonic() + 1.0
    try:
        while seen['hit'] is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if alive is not None and time.monotonic() >= next_check:
                next_check += 1.0
                if not alive():
                    break
    finally:
        node.destroy_subscription(sub)
    return seen['hit'], seen['last']


def latest_pose(node: Any, timeout: float = 1.0) -> tuple[float, float, float] | None:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    got: list[tuple[float, float, float]] = []

    def on_pose(msg: Any) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        got.append((p.x, p.y, yaw))

    sub = node.create_subscription(PoseWithCovarianceStamped, '/autonomy/pose', on_pose, 10)
    end = time.monotonic() + timeout
    try:
        while not got and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(sub)
    return got[-1] if got else None


def read_rc(timeout: float = RC_WAIT_SEC) -> tuple[list[float] | None, bool | None]:
    """
    Return the latest /rc/channels and /rc/link, each None if none arrived in time.

    The controller publishes both on every frame from the ESP32 (10 Hz) with
    the sensor-data QoS, so the subscriptions use it too. The wait ends as
    soon as both have arrived.
    """
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool, Float32MultiArray

    got: dict[str, Any] = {'channels': None, 'link': None}

    def on_channels(msg: Any) -> None:
        got['channels'] = [float(v) for v in msg.data]

    def on_link(msg: Any) -> None:
        got['link'] = bool(msg.data)

    with ros_node() as node:
        subs = [
            node.create_subscription(
                Float32MultiArray, '/rc/channels', on_channels, qos_profile_sensor_data
            ),
            node.create_subscription(Bool, '/rc/link', on_link, qos_profile_sensor_data),
        ]
        end = time.monotonic() + timeout
        try:
            while None in got.values() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            for sub in subs:
                node.destroy_subscription(sub)
    return got['channels'], got['link']


def not_ready_reason(status: dict[str, str] | None) -> str:
    if status is None:
        return f'autonomy_node never reported; see {LOG_HINT}'
    if not status.get('scan_age_ms'):
        return 'no lidar scans (/scan); check `racecar status`'
    if status.get('mode') == 'mapping':
        return 'slam_toolbox has not published map -> odom; check that teleop publishes odometry'
    return (
        f'not localized yet (pose spread {status.get("pose_sd_m") or "?"} m, '
        f'{status.get("pose_sd_deg") or "?"} deg)'
    )


def _wait_ready(values: dict[str, str], out: Out, timeout: float) -> int:
    session, mode = values['AUTONOMY_SESSION'], values['AUTONOMY_MODE']
    searching = values.get('AUTONOMY_POSE') == 'global'

    def ours(s: dict[str, str]) -> bool:
        return s.get('session') == session and s.get('mode') == mode

    def ready(s: dict[str, str]) -> bool:
        return ours(s) and (searching or s.get('localized') == 'true')

    def alive() -> bool:
        return unit_state() not in ('failed', 'inactive')

    with ros_node() as node:
        hit, last = wait_status(node, timeout, ready, alive)
    if hit is not None:
        return 0
    if not alive():
        raise MapError(f'the autonomy unit stopped while starting; see {LOG_HINT}')
    if last is not None and ours(last) and mode == 'localization':
        reason = not_ready_reason(last)
        out(
            f'Running. {reason[:1].upper()}{reason[1:]}. Drive a short distance, '
            'then check `racecar map status`.'
        )
        return 0
    reason = not_ready_reason(last if last is not None and ours(last) else None)
    raise MapError(f'not ready after {timeout:.0f} s: {reason}')


# -- actions --


def _change_mode(
    make_plan: Callable[[], Plan],
    root: Path,
    out: Out,
    now: datetime,
    timeout: float,
) -> int:
    first = make_plan()  # usage and map errors before anything stops
    require_teleop()
    if first.values['AUTONOMY_MODE'] == 'mapping':
        require_speed_switch_up()  # NeoRacer only (plan D12)
    if not os.environ.get(ROOT_ENV):
        problem = data_root.session_problem()
        if problem:
            raise MapError(problem)
    stop_driving_apps(out)
    if unit_running():
        out('Stopping the current autonomy run.')
        _control('stop')
    # Planned again: the stop wrote the last pose and the final autosave.
    plan = make_plan()
    name = plan.values['AUTONOMY_MAP']
    if plan.replace:
        moved = au.replace_folder(root, name, now)
        out(f'Moved the existing {name} to {moved}.')
    for note in plan.notes:
        out(note)
    write_mode_file(plan.values)
    _control('start')
    code = _wait_ready(plan.values, out, timeout)
    if code == 0 and plan.values['AUTONOMY_MODE'] == 'mapping':
        out(f'Mapping {name}. Drive slowly around the space, then run `racecar map save`.')
    elif code == 0 and plan.values.get('AUTONOMY_POSE') == 'global':
        out(
            f'Localizing on {name}: searching. '
            'Drive slowly until `racecar map status` reports localized.'
        )
    elif code == 0:
        out(f'Localizing on {name}.')
    return code


def start(
    name: str,
    source: str | None = None,
    pose: str | None = None,
    force: bool = False,
    out: Out = print,
    timeout: float = READY_TIMEOUT_SEC,
) -> int:
    root, now = map_root(), datetime.now()
    session = new_session(now)
    return _change_mode(
        lambda: plan_start(root, name, session, source, pose, force, now), root, out, now, timeout
    )


def load(
    name: str,
    pose: str | None = None,
    search: bool = False,
    localizer: str = 'amcl',
    out: Out = print,
    timeout: float = READY_TIMEOUT_SEC,
) -> int:
    root, now = map_root(), datetime.now()
    session = new_session(now)
    return _change_mode(
        lambda: plan_load(root, name, session, pose, search, localizer, now),
        root,
        out,
        now,
        timeout,
    )


def resume(out: Out = print, timeout: float = READY_TIMEOUT_SEC) -> int:
    """
    Start the last run again: mapping continues its own autosave; localization loads the map.

    Localization goes through load() so it starts from the last pose rather
    than the one the run began with.
    """
    if unit_running():
        out('Autonomy is already running.')
        return 0
    values = read_mode_file()
    mode, name = values.get('AUTONOMY_MODE'), values.get('AUTONOMY_MAP', '')
    if mode not in ('mapping', 'localization') or not valid_name(name):
        raise MapError('no earlier run to resume. Start mapping or load a map in the map viewer.')
    if mode == 'localization':
        return load(
            name, localizer=values.get('AUTONOMY_LOCALIZER') or 'amcl', out=out, timeout=timeout
        )
    require_teleop()
    stop_driving_apps(out)
    _control('start')
    code = _wait_ready(values, out, timeout)
    if code == 0:
        out(f'Mapping {name} again from its last autosave.')
    return code


def stop(out: Out = print) -> int:
    if not unit_running():
        out('Autonomy is not running.')
        return 0
    values = read_mode_file()
    _control('stop')
    name = values.get('AUTONOMY_MAP', '')
    if values.get('AUTONOMY_MODE') != 'mapping' or not valid_name(name):
        out('Stopped.')
        return 0
    state = au.map_state(map_root(), name)
    if state.unsaved:
        out(
            f'Stopped. {name} has changes that were never saved (autosave from '
            f'{state.autosave_time}). To keep them, run `racecar map recover {name}`.'
        )
    else:
        out(f'Stopped. {name} is saved.')
    return 0


def save(out: Out = print, timeout: float = SAVE_TIMEOUT_SEC) -> int:
    values = read_mode_file()
    if not unit_running():
        raise MapError('autonomy is not running; start mapping with `racecar map start NAME`.')
    if values.get('AUTONOMY_MODE') != 'mapping':
        raise MapError(
            f'not mapping (localizing on {values.get("AUTONOMY_MAP", "?")}); nothing to save.'
        )
    import rclpy
    from std_srvs.srv import Trigger

    with ros_node() as node:
        client = node.create_client(Trigger, '/autonomy/save_map')
        if not client.wait_for_service(timeout_sec=10.0):
            raise MapError(f'/autonomy/save_map is not available; see {LOG_HINT}')
        out(f'Saving {values.get("AUTONOMY_MAP", "")}...')
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        result = future.result() if future.done() else None
    if result is None:
        raise MapError(f'no answer from /autonomy/save_map within {timeout:.0f} s')
    if not result.success:
        raise MapError(f'save failed: {result.message}')
    out(f'Saved: {result.message}')
    return 0


def recover(name: str, out: Out = print) -> int:
    check_name(name)
    root = map_root()
    state = au.map_state(root, name)
    if not state.autosaved:
        raise MapError(f'{name} has no autosave under {state.folder}')
    values = read_mode_file()
    if (
        values.get('AUTONOMY_MAP') == name
        and values.get('AUTONOMY_MODE') == 'mapping'
        and unit_running()
    ):
        raise MapError(f'{name} is being mapped. Stop the run first with `racecar map stop`.')
    if not state.unsaved:
        out(f'{name} is saved. Nothing to recover.')
        return 0
    moved = au.archive_map_files(state.folder, name, datetime.now())
    au.copy_autosave(state.folder, name)
    out(f'Recovered {name} from its autosave of {state.autosave_time}.')
    if moved is not None:
        out(f'The replaced map is in {moved}.')
    return 0


def set_foxglove(on: bool, out: Out = print) -> int:
    values = read_mode_file()
    values['AUTONOMY_FOXGLOVE'] = '1' if on else '0'
    write_mode_file({k: v for k, v in values.items() if k in KEYS})
    out(
        f'Foxglove bridge {"on" if on else "off"} (port 8765); '
        'applies at the next `racecar map start` or `racecar map load`.'
    )
    return 0


# -- reading maps (no ROS) --


def grid_size(folder: Path, name: str) -> tuple[int, int, float] | None:
    """Return (width, height, resolution) from <name>.pgm and <name>.yaml; None if unreadable."""
    resolution = au.read_record(folder / f'{name}.yaml').get('resolution')
    try:
        with open(folder / f'{name}.pgm', 'rb') as f:
            head = f.read(512)
    except OSError:
        return None
    tokens: list[bytes] = []
    for line in head.split(b'\n'):
        if line.startswith(b'#'):
            continue
        tokens.extend(line.split())
        if len(tokens) >= 3:
            break
    try:
        return int(tokens[1]), int(tokens[2]), float(str(resolution))
    except (IndexError, TypeError, ValueError):
        return None


def map_names(root: Path) -> list[str]:
    try:
        entries = sorted(p.name for p in root.iterdir() if p.is_dir())
    except OSError:
        return []
    return [
        n
        for n in entries
        if valid_name(n) and n not in RESERVED_NAMES and au.map_state(root, n).exists
    ]


def state_label(state: au.MapState) -> str:
    if not state.saved:
        return 'unsaved'
    return 'saved, newer autosave' if state.unsaved else 'saved'


def _when_saved(state: au.MapState) -> str:
    if state.meta.get('saved'):
        return str(state.meta['saved'])
    try:
        stamp = (state.folder / f'{state.name}.yaml').stat().st_mtime
    except OSError:
        return state.autosave_time
    return datetime.fromtimestamp(stamp).strftime(au.TIME_FORMAT)


def _size_text(folder: Path, name: str) -> str:
    size = grid_size(folder, name)
    if size is None:
        return '?'
    w, h, res = size
    return f'{w * res:.1f} x {h * res:.1f} m'


def map_rows(root: Path) -> list[dict[str, str]]:
    """One row per map: name, state, saved (time), size, and note (mapping or loaded now)."""
    values = read_mode_file()
    active = values.get('AUTONOMY_MAP') if unit_running() else None
    rows = []
    for name in map_names(root):
        state = au.map_state(root, name)
        folder = state.folder if state.saved else state.folder / au.AUTOSAVE_DIR
        when = _when_saved(state) if state.saved else f'autosave {state.autosave_time}'
        note = ''
        if name == active:
            note = 'mapping now' if values.get('AUTONOMY_MODE') == 'mapping' else 'loaded now'
        rows.append(
            {
                'name': name,
                'state': state_label(state),
                'saved': when,
                'size': _size_text(folder, name),
                'note': note,
            }
        )
    return rows


def list_maps(out: Out = print) -> int:
    root = map_root()
    found = map_rows(root)
    if not found:
        out(f'No maps under {root}.')
        return 0
    rows = [('NAME', 'STATE', 'SAVED', 'SIZE', '')]
    rows += [(r['name'], r['state'], r['saved'], r['size'], r['note']) for r in found]
    widths = [max(len(r[i]) for r in rows) for i in range(4)]
    for r in rows:
        out(
            '  '.join(c.ljust(w) for c, w in zip(r[:4], widths, strict=True)).rstrip()
            + (f'  {r[4]}' if r[4] else '')
        )
    out(f'Maps are in {root}.')
    return 0


def _dir_bytes(folder: Path) -> int:
    total = 0
    for dirpath, _, files in os.walk(folder):
        for f in files:
            try:
                total += (Path(dirpath) / f).stat().st_size
            except OSError:
                pass
    return total


def info(name: str, out: Out = print) -> int:
    check_name(name)
    state = au.map_state(map_root(), name)
    if not state.exists:
        raise MapError(f'no map named {name} under {map_root()}')
    meta = state.meta if state.saved else state.autosave_meta
    folder = state.folder if state.saved else state.folder / au.AUTOSAVE_DIR
    rows: list[tuple[str, str]] = [
        ('folder', str(state.folder)),
        ('state', state_label(state)),
        ('saved', _when_saved(state) if state.saved else 'never'),
    ]
    size = grid_size(folder, name)
    if size is not None:
        w, h, res = size
        rows.append(
            ('size', f'{w * res:.1f} x {h * res:.1f} m ({w} x {h} cells of {res * 100:.0f} cm)')
        )
    if state.autosaved:
        rows.append(
            (
                'autosave',
                f'{state.autosave_time}, session {state.autosave_record.get("session", "?")}',
            )
        )
    for key in ('session', 'driver_version', 'hostname', 'extended_from'):
        if meta.get(key):
            rows.append((key.replace('_', ' '), str(meta[key])))
    start_pose = au.pose_of(meta.get('start_pose'))
    if start_pose is not None:
        rows.append(('start pose', describe_pose(start_pose)))
    last = au.read_record(state.folder / au.LAST_POSE_FILE)
    last_pose = au.pose_of(last)
    if last_pose is not None:
        rows.append(('last pose', f'{describe_pose(last_pose)}, {last.get("time", "")}'))
    previous = state.folder / au.PREVIOUS_DIR
    if previous.is_dir():
        rows.append(
            (
                'previous',
                f'{sum(1 for p in previous.iterdir() if p.is_dir())} earlier versions '
                f'in {previous}',
            )
        )
    rows.append(('disk', f'{_dir_bytes(state.folder) / 1e6:.1f} MB'))
    width = max(len(k) for k, _ in rows)
    for key, value in rows:
        out(f'{key.ljust(width)}  {value}')
    for change in frame_changes(meta, current_frames()):
        out(f'warning: frames changed since this map was built: {change}')
    return 0


def status(out: Out = print, timeout: float = STATUS_TIMEOUT_SEC) -> int:
    values = read_mode_file()
    if not unit_running():
        last = values.get('AUTONOMY_MAP')
        out(
            'autonomy  off'
            + (f' (last run: {values.get("AUTONOMY_MODE", "")} {last})' if last else '')
        )
        return 0
    with ros_node() as node:
        found, _ = wait_status(node, timeout, lambda s: True)
        pose = latest_pose(node) if found else None
    if found is None:
        out(f'autonomy  starting (no status yet; unit {unit_state()})')
        return 1
    mode = found.get('mode', '')
    rows = [('autonomy', mode), ('map', found.get('map', ''))]
    if mode == 'localization':
        rows.append(('localizer', found.get('localizer', '')))
    rows.append(('localized', found.get('localized', '')))
    if pose is not None:
        rows.append(('pose', describe_pose(pose)))
    # slam_toolbox's mapping pose carries no usable spread.
    if found.get('pose_sd_m') and mode == 'localization':
        rows.append(('spread', f'{found["pose_sd_m"]} m, {found.get("pose_sd_deg", "")} deg'))
    for key in ('pose_source', 'last_save', 'last_autosave', 'session'):
        if found.get(key):
            rows.append((key.replace('_', ' '), found[key]))
    if found.get('resumed_from_autosave') == 'true':
        rows.append(('resumed', 'from autosave'))
    width = max(len(k) for k, _ in rows)
    for key, value in rows:
        out(f'{key.ljust(width)}  {value}')
    return 1 if found.get('level') == str(au.LEVEL_ERROR) else 0
