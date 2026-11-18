#!/usr/bin/env python3
"""
Measurements for SLAM runs on the car, alongside `racecar launch autonomy`.

slam_session.py in this folder runs a whole test session with these; the
subcommands below stay for single measurements.

    slam_eval.py save NAME                 # write the mapping run to <map root>/NAME/
    slam_eval.py load [--seconds N]        # CPU per node, the Jetson's load; Ctrl-C ends
    slam_eval.py mark LABEL [--tag T]      # pose in the map frame with the car stopped on a mark
    slam_eval.py report [--taped FILE]     # mark repeatability and distances against a tape
    slam_eval.py quality MAP               # occupied, free, and unknown cells of a saved map

Records go to slam_eval/ under the data root: one CSV per `load` run and marks.csv for
`mark`. A taped file has one line per pair of marks, `A B meters`.

`load` reads the Jetson through sysinfo.py: CPU, GPU, the junction
temperature, the board's power, memory, swap, and the SoC's over-current
throttling. A run with swap in use or throttling is to be discarded.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Iterable
import csv
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from typing import Any

from neoracer_ros2_driver import data_root

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import sysinfo  # noqa: E402

EVAL_DIR = data_root.current()[0] / 'slam_eval'
MARKS_FILE = 'marks.csv'
# Label -> substring of /proc/<pid>/cmdline. Python nodes run as python3, so
# match the executable path, not the process name. The driver installs its
# nodes under lib/neoracer_ros2_driver/; the controller is the ESP32 bridge,
# which also publishes /imu/fused.
PROCESSES = {
    'controller': 'lib/neoracer_ros2_driver/controller',
    'slam_toolbox': 'slam_toolbox_node',
    'amcl': 'nav2_amcl/amcl',
    'map_server': 'nav2_map_server/map_server',
    'lifecycle_manager': 'nav2_lifecycle_manager/lifecycle_manager',
    'autonomy_node': 'lib/neoracer_ros2_driver/autonomy_node',
    'camera': 'lib/neoracer_ros2_driver/camera',
    'inference': 'lib/neoracer_ros2_driver/inference_node',
    'lidar': 'lakibeam1_scan_node',
    'dashboard': 'dashboard.py',
    'foxglove_bridge': 'foxglove_bridge',
}
CLK_TCK = os.sysconf('SC_CLK_TCK')
MARK_SECONDS = 2.0
# pgm values written by map_saver_cli in trinary mode.
PGM_OCCUPIED, PGM_FREE = 0, 254
MEMINFO = Path('/proc/meminfo')
# The INA3221 rail that carries the whole board's input power.
BOARD_RAIL = 'VDD_IN'
# load.csv columns before the per-process ones.
LOAD_COLUMNS = (
    't_s', 'cpu_busy_pct', 'temp_c', 'throttled', 'arm_mhz',
    'gpu_pct', 'power_mw', 'mem_used_mib', 'mem_avail_mib', 'swap_used_mib',
)  # fmt: skip


# --- save ---------------------------------------------------------------------


def save(name: str) -> int:
    from neoracer_ros2_driver.maps import map_stem, valid_name

    if not valid_name(name):
        print(f'Map names use letters, digits, - and _: {name!r}', file=sys.stderr)
        return 2
    stem = map_stem(name)
    stem.parent.mkdir(parents=True, exist_ok=True)
    import rclpy
    from slam_toolbox.srv import SerializePoseGraph

    rclpy.init()
    node = rclpy.create_node('slam_eval_save')
    try:
        client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=5.0):
            print('slam_toolbox is not running: racecar launch autonomy', file=sys.stderr)
            return 1
        future = client.call_async(SerializePoseGraph.Request(filename=str(stem)))
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
        result = future.result()
        if result is None or result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            print('slam_toolbox could not write the pose graph.', file=sys.stderr)
            return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not run_map_saver(stem):
        print('map_saver_cli failed; the pose graph is saved.', file=sys.stderr)
        return 1
    for path in sorted(stem.parent.glob(stem.name + '.*')):
        print(path)
    return 0


def run_map_saver(stem: Path, log: Path | None = None) -> bool:
    """Write <stem>.pgm and <stem>.yaml from /map; True on success."""
    argv = [
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(stem),
        '--ros-args', '-p', 'map_subscribe_transient_local:=true',
    ]  # fmt: skip
    if log is None:
        return subprocess.run(argv, check=False).returncode == 0
    with log.open('a') as f:
        return (
            subprocess.run(argv, check=False, stdout=f, stderr=subprocess.STDOUT).returncode == 0
        )


# --- load ---------------------------------------------------------------------


def process_ticks(stat_text: str) -> int:
    """Return utime + stime from /proc/<pid>/stat; the name field may contain spaces."""
    fields = stat_text.rsplit(')', 1)[1].split()
    return int(fields[11]) + int(fields[12])


def find_processes(cmdlines: dict[int, str]) -> dict[str, list[int]]:
    found: dict[str, list[int]] = defaultdict(list)
    for pid, cmd in cmdlines.items():
        for label, needle in PROCESSES.items():
            if needle in cmd:
                found[label].append(pid)
                break
    return dict(found)


def read_cmdlines() -> dict[int, str]:
    out = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / 'cmdline').read_bytes()
        except OSError:
            continue
        if raw:
            out[int(entry.name)] = raw.replace(b'\0', b' ').decode(errors='replace')
    return out


def read_ticks(pids: Iterable[int]) -> dict[int, int]:
    """Return CPU ticks per live pid; a pid that has exited is left out."""
    ticks = {}
    for pid in pids:
        try:
            ticks[pid] = process_ticks(Path(f'/proc/{pid}/stat').read_text())
        except (OSError, IndexError, ValueError):
            continue
    return ticks


def tick_delta(before: dict[int, int], after: dict[int, int]) -> int:
    """Ticks used between two readings by pids alive at both."""
    return sum(max(after[pid] - before[pid], 0) for pid in after.keys() & before.keys())


@dataclass
class Sample:
    t: float
    cpu_busy: float  # percent of all cores
    temp_c: float | None  # the junction temperature, tj-thermal
    throttled: int | None  # bit k-1: over-current alarm k fired since recording began
    arm_mhz: int | None  # CPU 0's clock
    per_process: dict[str, float]  # percent of one core
    gpu_pct: float | None = None
    power_mw: int | None = None  # the board's input, BOARD_RAIL
    mem_used_mib: int | None = None
    mem_avail_mib: int | None = None
    swap_used_mib: int | None = None  # rounded up, so any swap in use reads at least 1


def _mean_max(label: str, values: list[float], unit: str = '') -> str:
    return f'{label:18s} {sum(values) / len(values):6.1f}  {max(values):6.1f}{unit}'


def summarize(samples: list[Sample]) -> list[str]:
    if not samples:
        return ['No samples.']
    lines = [f'{len(samples)} samples over {samples[-1].t - samples[0].t:.0f} s']
    labels = sorted({k for s in samples for k in s.per_process})
    lines.append('Process            mean %   max %  (one core = 100)')
    for label in labels:
        values = [s.per_process.get(label, 0.0) for s in samples]
        lines.append(f'{label:18s} {sum(values) / len(values):6.1f}  {max(values):6.1f}')
    busy = [s.cpu_busy for s in samples]
    lines.append(f'All cores busy     {sum(busy) / len(busy):6.1f}  {max(busy):6.1f}')
    gpu = [s.gpu_pct for s in samples if s.gpu_pct is not None]
    if gpu:
        lines.append(_mean_max('GPU load', gpu))
    temps = [s.temp_c for s in samples if s.temp_c is not None]
    if temps:
        lines.append(f'SoC temperature    {sum(temps) / len(temps):6.1f}  {max(temps):6.1f} C')
    power = [s.power_mw / 1e3 for s in samples if s.power_mw is not None]
    if power:
        lines.append(_mean_max('Board power', power, ' W'))
    used = [float(s.mem_used_mib) for s in samples if s.mem_used_mib is not None]
    if used:
        lines.append(_mean_max('Memory used', used, ' MiB'))
    avail = [s.mem_avail_mib for s in samples if s.mem_avail_mib is not None]
    if avail:
        lines.append(f'Memory available   min {min(avail)} MiB')
    flags = {s.throttled for s in samples if s.throttled}
    clocks = [s.arm_mhz for s in samples if s.arm_mhz]
    if clocks:
        lines.append(f'ARM clock          min {min(clocks)} MHz')
    swap = [s.swap_used_mib for s in samples if s.swap_used_mib is not None]
    if not swap:
        lines.append('Swap in use: not read.')
    elif max(swap):
        lines.append(f'Swap in use: up to {max(swap)} MiB. Discard this run.')
    else:
        lines.append('Swap in use: none for the whole run.')
    if flags:
        lines.append(
            'Throttled: '
            + ', '.join(f'0x{f:x}' for f in sorted(flags))
            + ' (over-current alarms during the run). Discard this run.'
        )
    elif all(s.throttled is None for s in samples):
        lines.append('Throttled: not read (no over-current counters).')
    else:
        lines.append('Throttled: 0x0 for the whole run.')
    return lines


def parse_swap_used(text: str) -> int | None:
    """Return swap in use in MiB from /proc/meminfo text, rounded up; None without the fields."""
    kb = {}
    for key in ('SwapTotal', 'SwapFree'):
        m = re.search(rf'^{key}:\s+(\d+) kB', text, re.MULTILINE)
        if not m:
            return None
        kb[key] = int(m.group(1))
    return -(-(kb['SwapTotal'] - kb['SwapFree']) // 1024)


def read_oc_counts(root: Path = sysinfo.HWMON_DIR) -> dict[int, int] | None:
    """Return {alarm k: events since boot} from the SoC's over-current counters, or None."""
    hwmon = sysinfo.oc_hwmon_dir(root)
    if hwmon is None:
        return None
    counts = {}
    for counter in sorted(hwmon.glob('oc*_event_cnt')):
        m = re.fullmatch(r'oc(\d+)_event_cnt', counter.name)
        if m is None:
            continue
        try:
            counts[int(m.group(1))] = int(counter.read_text().strip())
        except (OSError, ValueError):
            continue
    return counts


def throttle_flags(start: dict[int, int] | None, now: dict[int, int] | None) -> int | None:
    """
    Return bit k-1 set for each over-current alarm k whose count grew since `start`.

    The counters only grow until a reboot, so events from before the run are
    left out. None when either reading is missing.
    """
    if start is None or now is None:
        return None
    flags = 0
    for k, count in now.items():
        if count > start.get(k, 0):
            flags |= 1 << (k - 1)
    return flags


class LoadRecorder:
    """CPU per node and the Jetson's load, memory, and throttling, one sample per call."""

    def __init__(self) -> None:
        self.samples: list[Sample] = []
        self._ticks = {
            label: read_ticks(pids) for label, pids in find_processes(read_cmdlines()).items()
        }
        self._cpu = sysinfo.read_cpu_times()
        self._oc_start = read_oc_counts()
        self._start = self._last = time.monotonic()

    def sample(self) -> Sample:
        """Measure since the previous call; nodes started since are added."""
        now = time.monotonic()
        dt = max(now - self._last, 1e-3)
        per = {}
        for label, pids in find_processes(read_cmdlines()).items():
            ticks = read_ticks(pids)
            if label in self._ticks:
                per[label] = tick_delta(self._ticks[label], ticks) / CLK_TCK / dt * 100.0
            self._ticks[label] = ticks
        cpu = sysinfo.read_cpu_times()
        busy = 0.0
        if cpu and self._cpu and cpu[1] > self._cpu[1]:
            busy = (cpu[0] - self._cpu[0]) / (cpu[1] - self._cpu[1]) * 100.0
        self._cpu, self._last = cpu, now
        try:
            meminfo = MEMINFO.read_text()
        except OSError:
            meminfo = ''
        memory = sysinfo.parse_meminfo(meminfo)
        gpu = sysinfo.read_gpu()
        s = Sample(
            now - self._start,
            busy,
            sysinfo.read_soc_temp(),
            throttle_flags(self._oc_start, read_oc_counts()),
            sysinfo.read_arm_clock()[0],
            per,
            gpu_pct=None if gpu is None else gpu['load_pct'],
            power_mw=sysinfo.read_power_rails().get(BOARD_RAIL, {}).get('mw'),
            mem_used_mib=None if memory is None else memory['used'],
            mem_avail_mib=None if memory is None else memory['available'],
            swap_used_mib=parse_swap_used(meminfo),
        )
        self.samples.append(s)
        return s

    def write_csv(self, path: Path) -> None:
        labels = sorted({k for s in self.samples for k in s.per_process})
        with path.open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow([*LOAD_COLUMNS, *labels])
            for s in self.samples:
                w.writerow(
                    [f'{s.t:.1f}', f'{s.cpu_busy:.1f}', s.temp_c, s.throttled, s.arm_mhz]
                    + [s.gpu_pct, s.power_mw, s.mem_used_mib, s.mem_avail_mib, s.swap_used_mib]
                    + [f'{s.per_process.get(k, 0.0):.1f}' for k in labels]
                )


def record_load(seconds: float | None, out_dir: Path) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f'{datetime.now():%Y%m%d-%H%M%S}-load.csv'
    recorder = LoadRecorder()
    print(f'Recording to {path}; nodes started later are added. Ctrl-C ends.')
    try:
        while seconds is None or not recorder.samples or recorder.samples[-1].t < seconds:
            time.sleep(1.0)
            recorder.sample()
    except KeyboardInterrupt:
        pass
    recorder.write_csv(path)
    print('\n'.join(summarize(recorder.samples)))
    return 0


# --- mark and report ----------------------------------------------------------


@dataclass(frozen=True)
class Mark:
    label: str
    tag: str
    x: float
    y: float
    yaw: float


def mean_pose(poses: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    """Mean of (x, y, yaw) samples; yaw by its unit vector."""
    n = len(poses)
    x = sum(p[0] for p in poses) / n
    y = sum(p[1] for p in poses) / n
    yaw = math.atan2(sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses))
    return x, y, yaw


def yaw_of(q: Any) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def sample_map_pose(buffer: Any, spin: Any = None) -> list[tuple[float, float, float]]:
    """
    Sample map -> base_link for MARK_SECONDS; empty without the transform.

    `spin` runs one round of the node's callbacks; None when an executor
    thread already spins it.
    """
    from rclpy.time import Time
    from tf2_ros import TransformException

    poses: list[tuple[float, float, float]] = []
    end = time.monotonic() + MARK_SECONDS + 2.0
    while time.monotonic() < end and len(poses) < 40:
        if spin is not None:
            spin()
        try:
            t = buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            time.sleep(0.05)
            continue
        tr = t.transform.translation
        poses.append((tr.x, tr.y, yaw_of(t.transform.rotation)))
        time.sleep(MARK_SECONDS / 40)
    return poses


def append_mark(path: Path, label: str, tag: str, poses: list[tuple[float, float, float]]) -> str:
    """Append the mean of `poses` to a marks CSV; return a one-line summary."""
    x, y, yaw = mean_pose(poses)
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists()
    with path.open('a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['time', 'label', 'tag', 'x', 'y', 'yaw_deg', 'samples'])
        w.writerow(
            [f'{datetime.now():%Y-%m-%d %H:%M:%S}', label, tag, f'{x:.4f}', f'{y:.4f}',
             f'{math.degrees(yaw):.2f}', len(poses)]
        )  # fmt: skip
    return f'{label} ({tag}): x {x:.3f} m, y {y:.3f} m, yaw {math.degrees(yaw):.1f} deg'


def mark(label: str, tag: str, out_dir: Path) -> int:
    import rclpy
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('slam_eval_mark')
    buffer = Buffer()
    TransformListener(buffer, node)
    try:
        poses = sample_map_pose(buffer, lambda: rclpy.spin_once(node, timeout_sec=0.05))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not poses:
        print('No map -> base_link transform: is the localizer running?', file=sys.stderr)
        return 1
    print(append_mark(out_dir / MARKS_FILE, label, tag, poses))
    return 0


def read_marks(path: Path) -> list[Mark]:
    with path.open() as f:
        return [
            Mark(
                r['label'],
                r['tag'],
                float(r['x']),
                float(r['y']),
                math.radians(float(r['yaw_deg'])),
            )
            for r in csv.DictReader(f)
        ]


def read_taped(path: Path) -> list[tuple[str, str, float]]:
    pairs = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[0] != 'point' and not line.lstrip().startswith('#'):
            pairs.append((parts[0], parts[1], float(parts[2])))
    return pairs


def read_point(path: Path) -> tuple[float, float]:
    """
    Return the car point the tape was measured from, in base_link (m).

    A taped file line `point X Y` names it, for example the floor beside the
    left rear tire at the axle; base_link itself is under the car. Default
    (0, 0).
    """
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[0] == 'point':
            return float(parts[1]), float(parts[2])
    return 0.0, 0.0


def report_lines(
    marks: list[Mark],
    taped: list[tuple[str, str, float]],
    point: tuple[float, float] = (0.0, 0.0),
) -> list[str]:
    """
    Per tag: spread of repeated visits to each mark, then mark distances against the tape.

    Distances are between `point` (base_link coordinates) at each mark's mean pose.
    """
    lines = []
    by_tag: dict[str, dict[str, list[Mark]]] = defaultdict(lambda: defaultdict(list))
    for m in marks:
        by_tag[m.tag][m.label].append(m)
    for tag in sorted(by_tag):
        lines.append(f'== {tag}')
        lines.append('Mark   visits  spread mm  yaw spread deg')
        means = {}
        for label in sorted(by_tag[tag]):
            visits = by_tag[tag][label]
            mx, my, myaw = mean_pose([(v.x, v.y, v.yaw) for v in visits])
            c, s_ = math.cos(myaw), math.sin(myaw)
            means[label] = (mx + c * point[0] - s_ * point[1], my + s_ * point[0] + c * point[1])
            spread = max(math.hypot(v.x - mx, v.y - my) for v in visits)
            yaw_spread = max(
                abs(math.degrees(math.atan2(math.sin(v.yaw - myaw), math.cos(v.yaw - myaw))))
                for v in visits
            )
            lines.append(f'{label:6s} {len(visits):6d}  {spread * 1e3:9.0f}  {yaw_spread:14.1f}')
        rows = [(a, b, d) for a, b, d in taped if a in means and b in means]
        if rows:
            lines.append('Pair      taped m  map m   error mm')
            for a, b, d in rows:
                got = math.dist(means[a], means[b])
                lines.append(f'{a + "-" + b:9s} {d:7.3f}  {got:6.3f}  {(got - d) * 1e3:+8.0f}')
    return lines


# --- quality ------------------------------------------------------------------


def cell_counts(pixels: Iterable[int]) -> tuple[int, int, int]:
    occupied = free = unknown = 0
    for p in pixels:
        if p == PGM_OCCUPIED:
            occupied += 1
        elif p == PGM_FREE:
            free += 1
        else:
            unknown += 1
    return occupied, free, unknown


def quality(name: str) -> int:
    from PIL import Image
    import yaml

    from neoracer_ros2_driver.maps import map_stem

    stem = map_stem(name)
    meta = yaml.safe_load(Path(str(stem) + '.yaml').read_text())
    image = Image.open(stem.parent / meta['image'])
    occupied, free, unknown = cell_counts(image.getdata())
    res = float(meta['resolution'])
    print(f'{stem}: {image.width} x {image.height} cells at {res * 1e3:.0f} mm')
    print(f'occupied {occupied}  free {free}  unknown {unknown}')
    area = free * res * res
    print(
        f'free area {area:.2f} m^2; occupied cells per free m^2 {occupied / max(area, 1e-9):.1f}'
    )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('save', help='save the mapping run under the map root')
    p.add_argument('name')
    p = sub.add_parser('load', help='record CPU, GPU, temperature, power, memory, and throttling')
    p.add_argument('--seconds', type=float)
    p = sub.add_parser('mark', help='record the pose at a mark on the floor')
    p.add_argument('label')
    p.add_argument('--tag', default='run', help='groups marks, e.g. amcl or slam_toolbox')
    p = sub.add_parser('report', help='mark repeatability and taped distances')
    p.add_argument('--marks', type=Path, default=EVAL_DIR / MARKS_FILE)
    p.add_argument('--taped', type=Path)
    p = sub.add_parser('quality', help='cell counts of a saved map')
    p.add_argument('map')
    args = parser.parse_args(argv)

    if args.cmd == 'save':
        return save(args.name)
    if args.cmd == 'load':
        return record_load(args.seconds, EVAL_DIR)
    if args.cmd == 'mark':
        return mark(args.label, args.tag, EVAL_DIR)
    if args.cmd == 'report':
        taped = read_taped(args.taped) if args.taped else []
        point = read_point(args.taped) if args.taped else (0.0, 0.0)
        print('\n'.join(report_lines(read_marks(args.marks), taped, point)))
        return 0
    return quality(args.map)


if __name__ == '__main__':
    sys.exit(main())
