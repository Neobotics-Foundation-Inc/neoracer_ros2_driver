"""
A simulated NeoRacer driving a lane map, and the odometry it would report.

The car follows its lanes' centres, weaving a little in its lane; its
odometry carries what a real car's does: a gyro bias that grows and wanders,
vibration on the heading, and an encoder that may read distance a few
percent off. Used by the lane lock's tests (test/test_lane_lock.py), its
benchmark (bench_lane_lock.py) and sim_cars.py, which streams such cars over
V2V to v2v_view.py.
"""

import math
import random

from neoracer_ros2_driver.lane_lock import wrap


def route(lane_map, start, turns, laps_of_lanes):
    """Return the lanes a car drives, from `start`.

    At each junction it takes the path to the next of `turns` (a lane id),
    else straight on, until `laps_of_lanes` lanes have been driven.
    """
    turns = list(turns)
    out = [lane_map.by_id[start]]
    while len(out) < laps_of_lanes:
        lane = out[-1]
        if len(lane.next) == 1:
            out.append(lane.next[0])
            continue
        want = turns.pop(0) if turns else None
        paths = lane.next
        if want is not None:
            pick = next(p for p in paths if p.next_ids == [want])
        else:
            pick = next(p for p in paths
                        if lane_map.data['lanes'][p.index].get('turn') == 'straight')
        out.append(pick)
    return out


def drive(lane_map, lanes_driven, speed=1.0, rate=10.0, scale=1.0,
          bias_rate=math.radians(3) / 60, bias_walk=0.003, vibration=math.radians(2),
          wobble=0.05, start_s=0.0, seed=1):
    """Simulate a drive; return samples (t, odometry x, y, yaw, true lane, true s).

    The car follows its lanes' centres weaving `wobble` m to either side; its
    odometry measures distance in metres the map's are `scale` times, its
    heading off by a bias growing `bias_rate` rad/s and wandering `bias_walk`
    rad per sqrt(s), plus `vibration` rad of noise on each reading.
    """
    rng = random.Random(seed)
    step = speed / rate
    path = []   # (x, y, lane, s)
    for k, lane in enumerate(lanes_driven):
        s = start_s if k == 0 else 0.0
        while s < lane.length:
            x, y, h = lane.pose(s)
            path.append((x, y, h, lane, s))
            s += step
    true = []
    for i, (x, y, h, lane, s) in enumerate(path):
        u = i * step
        off = wobble * math.sin(2 * math.pi * u / 2.5)
        true.append((x + off * math.sin(h), y - off * math.cos(h), lane, s))
    samples = []
    ox = oy = 0.0
    bias = 0.0
    for i in range(1, len(true)):
        x0, y0, _, _ = true[i - 1]
        x1, y1, lane, s = true[i]
        t = i / rate
        heading = math.atan2(y1 - y0, x1 - x0)
        bias += bias_rate / rate + rng.gauss(0, bias_walk * math.sqrt(1 / rate))
        yaw = heading + bias
        d = math.hypot(x1 - x0, y1 - y0) / scale
        ox += d * math.cos(yaw)
        oy += d * math.sin(yaw)
        samples.append((t, ox, oy, wrap(yaw + rng.gauss(0, vibration)), lane, s))
    return samples
