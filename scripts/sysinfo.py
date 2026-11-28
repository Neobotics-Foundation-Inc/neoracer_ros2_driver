#!/usr/bin/env python3
"""
Host facts shared by the dashboard, the watchdog and `racecar status`.

The NeoRacer runs on a Jetson Orin Nano. The readers keep upstream's names
and return types, so the tools that import them work unchanged; each one
reads the Jetson's own source (thermal zones, cpufreq, the SoC's
over-current throttle counters, nvpmodel, tegrastats). Upstream's Raspberry
Pi readers (vcgencmd, the PMIC's RTC cell and under-voltage alarm) have no
Jetson counterpart and return None.
"""

from __future__ import annotations

from collections.abc import Callable
from datetime import datetime
import json
import os
from pathlib import Path
import re
import select
import shutil
import subprocess
import time
from typing import Any

# Thresholds of upstream's rechargeable RTC cell (2.7-3.0 V), kept only so
# classify_rtc() keeps its interface. The Jetson exposes no RTC cell voltage,
# so read_rtc_voltage() is always None on the NeoRacer and these never apply.
RTC_OK_VOLTS = 2.8
RTC_LOW_VOLTS = 2.7

THERMAL_DIR = Path('/sys/class/thermal')
HWMON_DIR = Path('/sys/class/hwmon')
CPUFREQ_DIR = Path('/sys/devices/system/cpu/cpu0/cpufreq')
# The junction temperature, the zone the Orin's thermal throttling acts on.
SOC_TEMP_ZONE = 'tj-thermal'
# hwmon driver name of the SoC's over-current throttle event counters.
OC_HWMON_NAME = 'soctherm_oc'
# hwmon driver names of the board's power monitor, the fan's PWM output and
# the fan's tachometer.
POWER_HWMON_NAME = 'ina3221'
FAN_PWM_HWMON_NAME = 'pwmfan'
FAN_TACH_HWMON_NAME = 'pwm_tach'
DEVFREQ_DIR = Path('/sys/class/devfreq')
# The GPU's devfreq entry is named after its bus address, 17000000.gpu on the Orin.
GPU_DEVFREQ_SUFFIX = '.gpu'

# RealSense publishes its per-stream rates on /diagnostics under these names.
REALSENSE_DIAGNOSTIC_NAMES = {
    'camera: color': '/camera/color',
    'camera: depth': '/camera/depth',
    'camera: gyro': '/imu/realsense',
}
DIAGNOSTIC_RATE_KEY = 'Actual frequency (Hz)'

# Written by watchdog.py into the session log directory on every poll.
WATCHDOG_STATE_FILE = 'watchdog_state.json'
# A state file older than this many polls means the watchdog has stopped.
WATCHDOG_STALE_POLLS = 4


def run_cmd(cmd: list[str], timeout: float = 5.0) -> str:
    """Run a command and return stdout, or an empty string on any failure."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, OSError):
        return ''
    return r.stdout if r.returncode == 0 else ''


def first_line(cmd: list[str], timeout: float = 3.0) -> str:
    """
    Start a command that streams lines, return its first line, then stop it.

    Returns an empty string when the command is missing or prints no whole
    line within `timeout` seconds; a half-written line never holds it past
    the timeout. A command that exits without a newline gives what it printed.
    """
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except OSError:
        return ''
    deadline = time.monotonic() + timeout
    data = b''
    try:
        if proc.stdout is None:
            return ''
        fd = proc.stdout.fileno()
        while b'\n' not in data:
            left = deadline - time.monotonic()
            if left <= 0:
                return ''
            ready, _, _ = select.select([fd], [], [], left)
            if not ready:
                return ''
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            data += chunk
        return data.split(b'\n', 1)[0].decode(errors='replace').strip()
    finally:
        proc.kill()
        proc.wait()
        if proc.stdout is not None:
            proc.stdout.close()


def read_diagnostic_rates(msg: Any, into: dict[str, float]) -> None:
    """Copy the RealSense per-stream rates from one DiagnosticArray into `into`."""
    for status in msg.status:
        topic = REALSENSE_DIAGNOSTIC_NAMES.get(status.name.lower())
        if topic is None:
            continue
        for value in status.values:
            if value.key != DIAGNOSTIC_RATE_KEY:
                continue
            try:
                into[topic] = float(value.value)
            except (TypeError, ValueError):
                pass
            break


def read_rtc_voltage() -> float | None:
    """Return the RTC backup cell voltage; None on the Jetson, which does not expose it."""
    return None


def under_voltage_alarm_path() -> Path | None:
    """
    Return the path of a sticky under-voltage alarm, or None.

    The Jetson has no under-voltage alarm file like the Pi's PMIC. Power
    trouble shows up as over-current throttling; see read_throttled().
    """
    return None


def read_under_voltage_alarm() -> bool | None:
    """Return the sticky under-voltage alarm, or None when there is none to read."""
    alarm = under_voltage_alarm_path()
    if alarm is None:
        return None
    try:
        return alarm.read_text().strip() == '1'
    except OSError:
        return None


def classify_rtc(volts: float | None) -> tuple[str, str]:
    """Map an RTC cell voltage to a (status, label) pair for display."""
    if volts is None:
        return ('dead', 'NO READING')
    if volts >= RTC_OK_VOLTS:
        return ('healthy', f'{volts:.2f} V')
    if volts >= RTC_LOW_VOLTS:
        return ('stale', f'{volts:.2f} V, recharge soon')
    return ('dead', f'{volts:.2f} V, RECHARGE NOW')


def _read_millidegrees(path: Path) -> float | None:
    """
    Return one thermal zone's temperature in Celsius, or None.

    The cv*-thermal zones of a powered-down vision engine fail with EAGAIN,
    which Python's buffered reader surfaces as TypeError, not OSError.
    """
    try:
        celsius = int(path.read_text().strip()) / 1000.0
    except (OSError, ValueError, TypeError):
        return None
    return celsius if celsius > 0 else None


def read_thermal_zones(root: Path = THERMAL_DIR) -> dict[str, float]:
    """Return {zone type: Celsius} for every thermal zone that reads."""
    zones: dict[str, float] = {}
    for zone in sorted(root.glob('thermal_zone*')):
        try:
            kind = (zone / 'type').read_text().strip()
        except (OSError, TypeError):
            continue
        celsius = _read_millidegrees(zone / 'temp')
        if kind and celsius is not None:
            zones[kind] = celsius
    return zones


def read_soc_temp(root: Path = THERMAL_DIR) -> float | None:
    """Return the SoC temperature in Celsius: the junction zone, else the hottest zone."""
    zones = read_thermal_zones(root)
    if SOC_TEMP_ZONE in zones:
        return zones[SOC_TEMP_ZONE]
    return max(zones.values()) if zones else None


def hwmon_by_name(name: str, root: Path = HWMON_DIR) -> Path | None:
    """
    Locate the hwmon device whose driver is `name`, or None.

    hwmon numbering is not stable across boots, so match on the driver name.
    """
    for h in sorted(root.glob('hwmon*')):
        try:
            if (h / 'name').read_text().strip() == name:
                return h
        except OSError:
            continue
    return None


def oc_hwmon_dir(root: Path = HWMON_DIR) -> Path | None:
    """Locate the SoC's over-current throttle counters, or None."""
    return hwmon_by_name(OC_HWMON_NAME, root)


def read_throttled(root: Path = HWMON_DIR) -> tuple[int | None, list[str]]:
    """
    Return (raw flags, condition names) from the SoC's over-current throttle counters.

    Bit k-1 of the flags is set when alarm k has fired since boot. The counts
    only grow until reboot, so every name says "has occurred".
    """
    hwmon = oc_hwmon_dir(root)
    if hwmon is None:
        return (None, [])
    lines = []
    for counter in sorted(hwmon.glob('oc*_event_cnt')):
        try:
            lines.append(f'{counter.name}:{counter.read_text().strip()}')
        except OSError:
            continue
    return decode_throttled('\n'.join(lines))


def decode_throttled(text: str) -> tuple[int | None, list[str]]:
    """
    Parse over-current counter lines into (raw flags, set condition names).

    Each line is `oc<k>_event_cnt:<count>`, as `grep . oc*_event_cnt` prints
    it (a leading path is ignored).
    """
    counts = {}
    for line in text.splitlines():
        m = re.search(r'oc(\d+)_event_cnt\D*?(\d+)\s*$', line)
        if m:
            counts[int(m.group(1))] = int(m.group(2))
    if not counts:
        return (None, [])
    flags = 0
    active = []
    for k in sorted(counts):
        if counts[k] > 0:
            flags |= 1 << (k - 1)
            active.append(f'over-current throttling oc{k} has occurred ({counts[k]} events)')
    return (flags, active)


def _read_int(path: Path) -> int | None:
    try:
        return int(path.read_text().strip())
    except (OSError, ValueError, TypeError):
        return None


def read_power_rails(root: Path = HWMON_DIR) -> dict[str, dict[str, int]]:
    """
    Return {rail label: {'mv', 'ma', 'mw'}} from the board's power monitor.

    The INA3221 reports each channel's bus voltage (inN_input, mV) and current
    (currN_input, mA); VDD_IN is the whole board. A labelled input with no
    current, such as the sum of shunt voltages, is left out. Empty when the
    board has no power monitor.
    """
    rails: dict[str, dict[str, int]] = {}
    hwmon = hwmon_by_name(POWER_HWMON_NAME, root)
    if hwmon is None:
        return rails
    for label_file in sorted(hwmon.glob('in*_label')):
        channel = label_file.name.removeprefix('in').removesuffix('_label')
        try:
            label = label_file.read_text().strip()
        except OSError:
            continue
        mv = _read_int(hwmon / f'in{channel}_input')
        ma = _read_int(hwmon / f'curr{channel}_input')
        if label and mv is not None and ma is not None:
            rails[label] = {'mv': mv, 'ma': ma, 'mw': round(mv * ma / 1000)}
    return rails


def read_fan(root: Path = HWMON_DIR) -> dict[str, int | None] | None:
    """
    Return the fan's drive and speed, {'pwm': 0 to 255, 'percent', 'rpm'}, or None.

    The duty comes from the pwmfan driver and the speed from the pwm_tach
    tachometer; either value is None when its file does not read.
    """
    fan = hwmon_by_name(FAN_PWM_HWMON_NAME, root)
    tach = hwmon_by_name(FAN_TACH_HWMON_NAME, root)
    if fan is None and tach is None:
        return None
    pwm = _read_int(fan / 'pwm1') if fan is not None else None
    rpm = _read_int(tach / 'rpm') if tach is not None else None
    percent = round(pwm * 100 / 255) if pwm is not None else None
    return {'pwm': pwm, 'percent': percent, 'rpm': rpm}


def gpu_devfreq_dir(root: Path = DEVFREQ_DIR) -> Path | None:
    """Locate the GPU's devfreq entry, or None."""
    return next(iter(sorted(root.glob(f'*{GPU_DEVFREQ_SUFFIX}'))), None)


def read_gpu(root: Path = DEVFREQ_DIR) -> dict[str, float | None] | None:
    """
    Return the GPU's load and clock, {'load_pct', 'cur_mhz', 'max_mhz'}, or None.

    The clocks come from the GPU's devfreq entry in Hz. The load comes from the
    GPU device's `load` file in tenths of a percent, the value tegrastats shows
    as GR3D_FREQ. All three are file reads, cheap enough for a dashboard poll.
    """
    gpu = gpu_devfreq_dir(root)
    if gpu is None:
        return None
    cur = _read_int(gpu / 'cur_freq')
    top = _read_int(gpu / 'max_freq')
    load = _read_int(gpu / 'device' / 'load')
    return {
        'load_pct': load / 10 if load is not None else None,
        'cur_mhz': cur / 1e6 if cur is not None else None,
        'max_mhz': top / 1e6 if top is not None else None,
    }


def read_cpu_times() -> tuple[int, int] | None:
    """Return (busy, total) jiffies summed over every CPU since boot."""
    try:
        return parse_cpu_times(Path('/proc/stat').read_text())
    except OSError:
        return None


def parse_cpu_times(text: str) -> tuple[int, int] | None:
    """
    Reduce the aggregate `cpu` line of /proc/stat to (busy, total) jiffies.

    idle and iowait count as not busy. guest and guest_nice are already
    included in user and nice, so only the first eight fields are summed.
    """
    for line in text.splitlines():
        parts = line.split()
        if not parts or parts[0] != 'cpu':
            continue
        try:
            fields = [int(p) for p in parts[1:9]]
        except ValueError:
            return None
        if len(fields) < 5:
            return None
        total = sum(fields)
        return (total - fields[3] - fields[4], total)
    return None


def read_arm_clock(cpufreq: Path = CPUFREQ_DIR) -> tuple[int | None, int | None]:
    """Return (current, maximum) clock of CPU 0 in MHz from cpufreq; either may be None."""
    values: list[int | None] = []
    for name in ('scaling_cur_freq', 'cpuinfo_max_freq'):
        try:
            values.append(round(int((cpufreq / name).read_text()) / 1000))
        except (OSError, ValueError):
            values.append(None)
    return (values[0], values[1])


def parse_power_mode(text: str) -> tuple[str | None, int | None]:
    """Parse `nvpmodel -q` output into (mode name, mode id); either may be None."""
    name = None
    m = re.search(r'NV Power Mode:\s*(\S+)', text)
    if m:
        name = m.group(1)
    mode_id = None
    for line in text.splitlines():
        if line.strip().isdigit():
            mode_id = int(line.strip())
            break
    return (name, mode_id)


def read_power_mode() -> tuple[str | None, int | None]:
    """Return the Jetson power mode as (name, id) from `nvpmodel -q`."""
    return parse_power_mode(run_cmd(['nvpmodel', '-q'], timeout=3))


def parse_tegrastats(line: str) -> dict[str, Any] | None:
    """
    Parse one tegrastats line into RAM, swap, CPU, GPU, temperature and power figures.

    Returns None when the line has no RAM figure, which every tegrastats line has.
    """
    ram = re.search(r'RAM (\d+)/(\d+)MB', line)
    if not ram:
        return None
    out: dict[str, Any] = {'ram_used_mb': int(ram.group(1)), 'ram_total_mb': int(ram.group(2))}
    swap = re.search(r'SWAP (\d+)/(\d+)MB', line)
    if swap:
        out['swap_used_mb'], out['swap_total_mb'] = int(swap.group(1)), int(swap.group(2))
    cpu = re.search(r'CPU \[([^\]]*)\]', line)
    if cpu:
        cores: list[tuple[int, int] | None] = []
        for core in cpu.group(1).split(','):
            m = re.match(r'(\d+)%@(\d+)', core.strip())
            cores.append((int(m.group(1)), int(m.group(2))) if m else None)
        out['cpu'] = cores
    gpu = re.search(r'GR3D_FREQ (\d+)%', line)
    if gpu:
        out['gpu_load_pct'] = int(gpu.group(1))
    out['temps_c'] = {name: float(value) for name, value in re.findall(r'(\w+)@(-?[\d.]+)C', line)}
    out['power_mw'] = {
        name: (int(now), int(avg)) for name, now, avg in re.findall(r'(\w+) (\d+)mW/(\d+)mW', line)
    }
    return out


def read_tegrastats(timeout: float = 3.0) -> dict[str, Any] | None:
    """
    Return one parsed tegrastats line, or None.

    tegrastats prints its first line after about a second, so this suits a
    one-off check, not a fast polling loop.
    """
    return parse_tegrastats(first_line(['tegrastats', '--interval', '1000'], timeout))


def read_memory() -> dict[str, int] | None:
    """Return memory totals in MiB: total, available, used."""
    try:
        text = Path('/proc/meminfo').read_text()
    except OSError:
        return None
    return parse_meminfo(text)


def parse_meminfo(text: str) -> dict[str, int] | None:
    """Reduce /proc/meminfo text to MiB totals: total, available, used."""
    fields = {}
    for key in ('MemTotal', 'MemAvailable'):
        m = re.search(rf'^{key}:\s+(\d+) kB', text, re.MULTILINE)
        if not m:
            return None
        fields[key] = int(m.group(1)) // 1024
    return {
        'total': fields['MemTotal'],
        'available': fields['MemAvailable'],
        'used': fields['MemTotal'] - fields['MemAvailable'],
    }


def read_disk(path: str = '/') -> dict[str, int] | None:
    """Return filesystem usage for `path` in GiB, plus percent used."""
    try:
        total, used, free = shutil.disk_usage(path)
    except OSError:
        return None
    gib = 1024**3
    return {
        'total': total // gib,
        'used': used // gib,
        'free': free // gib,
        'percent_used': round(used * 100 / total) if total else 0,
    }


def read_uptime() -> float | None:
    """Return the system uptime in seconds."""
    try:
        return float(Path('/proc/uptime').read_text().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def ntp_synchronized() -> bool | None:
    """Return True when the clock is NTP-synchronized, or None if unreadable."""
    try:
        r = subprocess.run(
            ['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (subprocess.TimeoutExpired, OSError):
        return None
    if r.returncode != 0:
        return None
    return r.stdout.strip() == 'yes'


def format_uptime(seconds: float) -> str:
    """Render an uptime in seconds as a compact `1d 2h 3m` string."""
    total = int(seconds)
    days, rem = divmod(total, 86400)
    hours, rem = divmod(rem, 3600)
    minutes = rem // 60
    if days:
        return f'{days}d {hours}h {minutes}m'
    if hours:
        return f'{hours}h {minutes}m'
    return f'{minutes}m'


def read_watchdog_state(log_dir: Path | None = None) -> dict[str, Any] | None:
    """
    Return the watchdog's state file, or None when it is absent or unreadable.

    Adds `age_sec` (seconds since the watchdog last wrote it) and `stale`
    (older than WATCHDOG_STALE_POLLS polls).
    """
    path = (log_dir or Path.home() / 'logs' / 'latest') / WATCHDOG_STATE_FILE
    try:
        state: dict[str, Any] = json.loads(path.read_text())
        updated = datetime.fromisoformat(state['updated'])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    age = (datetime.now() - updated).total_seconds()
    state['age_sec'] = age
    state['stale'] = age > WATCHDOG_STALE_POLLS * float(state.get('poll_sec', 5))
    return state


# How often HostPoller reads the slow facts again; each is a subprocess.
SLOW_FACTS_SEC = 60.0


def cpu_load_pct(prev: tuple[int, int] | None, cur: tuple[int, int] | None) -> float | None:
    """Return the CPU load in percent between two read_cpu_times() samples, or None."""
    if prev is None or cur is None:
        return None
    busy, total = cur[0] - prev[0], cur[1] - prev[1]
    if total <= 0:
        return None
    return round(100.0 * busy / total, 1)


class HostPoller:
    """
    Every cheap host reading in one dict, for a page that polls every few seconds.

    Everything is a file read except the power mode and the NTP state, one
    subprocess each, which are read again at most every SLOW_FACTS_SEC. The
    CPU load is the busy share since the previous poll, None on the first.
    tegrastats is never run here: it takes about 2 s to print a line.
    """

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._cpu: tuple[int, int] | None = None
        self._slow: dict[str, Any] = {}
        self._slow_at: float | None = None

    def poll(self) -> dict[str, Any]:
        """Return the readings now; see the class docstring for what is cached."""
        now = self._clock()
        if self._slow_at is None or now - self._slow_at >= SLOW_FACTS_SEC:
            name, mode = read_power_mode()
            self._slow = {
                'power_mode': {'name': name, 'id': mode},
                'ntp_synchronized': ntp_synchronized(),
            }
            self._slow_at = now
        cpu = read_cpu_times()
        load = cpu_load_pct(self._cpu, cpu)
        self._cpu = cpu
        cur_mhz, max_mhz = read_arm_clock()
        flags, active = read_throttled()
        uptime = read_uptime()
        return {
            'soc_temp_c': read_soc_temp(),
            'temps_c': read_thermal_zones(),
            'cpu': {'load_pct': load, 'cur_mhz': cur_mhz, 'max_mhz': max_mhz},
            'memory_mib': read_memory(),
            'disk_gib': read_disk(),
            'gpu': read_gpu(),
            'power_rails': read_power_rails(),
            'fan': read_fan(),
            'throttle': {'flags': flags, 'active': active},
            'uptime_s': uptime,
            'uptime': format_uptime(uptime) if uptime is not None else None,
            **self._slow,
        }
