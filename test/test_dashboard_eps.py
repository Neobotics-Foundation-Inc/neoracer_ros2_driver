"""
Learner, ledger and settings tests for the episodic policy search dashboard (eps.py).

Runs without a ROS graph or a lidar: the learner and the session are plain
Python, and the node's scan callback and timer are driven by hand with
Node.__init__ and the create_* factories stubbed.
"""

import json
import math

from conftest import load_script
import pytest
from sensor_msgs.msg import LaserScan
import yaml


class Recorder:
    """Stands in for the /drive publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(scope='module')
def eps():
    return load_script('eps')


@pytest.fixture
def cfg(eps, monkeypatch, tmp_path):
    """Shipped settings, a temporary car profile file and a temporary ledger."""
    monkeypatch.setattr(eps, 'YAML_PATH', tmp_path / 'profile' / 'eps.yaml')
    monkeypatch.setattr(eps, 'LEDGER', tmp_path / 'data' / 'ledger.jsonl')
    monkeypatch.setattr(eps, 'INIT_MARK', tmp_path / 'data' / '.init_set')
    eps._cfg.clear()
    eps.load_cfg()
    return dict(eps._cfg)


def test_the_ledger_lives_under_the_data_root(eps, data_root_dir):
    assert eps.LEDGER == data_root_dir / 'dashboards' / 'eps' / 'ledger.jsonl'
    assert eps.INIT_MARK.parent == eps.LEDGER.parent


def test_the_baseline_run_drives_the_starting_gains(eps, cfg):
    ln = eps.Learner(cfg)
    assert ln.arm(cfg) == {k: cfg['nominal'][k] for k in eps.GAIN_KEYS}


def test_proposals_stay_within_half_and_double(eps, cfg):
    ln = eps.Learner(cfg)
    ln.baselined = True
    ln.sigma = 50.0
    for _ in range(20):
        ln.arm(cfg)
        assert all(eps.NORM_CLAMP[0] <= v <= eps.NORM_CLAMP[1] for v in ln.proposal)


def test_success_keeps_the_proposal_and_grows_sigma(eps, cfg):
    ln = eps.Learner(cfg)
    ln.baselined = True
    ln.arm(cfg)
    proposal, sigma = list(ln.proposal), ln.sigma
    ln.label('success', cfg)
    assert ln.theta == proposal
    assert ln.sigma == pytest.approx(min(sigma * cfg['a_up'], cfg['sigma_max']))


def test_fail_keeps_the_old_gains_and_shrinks_sigma(eps, cfg):
    ln = eps.Learner(cfg)
    ln.baselined = True
    ln.arm(cfg)
    sigma = ln.sigma
    ln.label('fail', cfg)
    assert ln.theta == [1.0, 1.0, 1.0, 1.0]
    assert ln.sigma == pytest.approx(max(sigma * cfg['a_down'], cfg['sigma_min']))


def test_the_staircase_climbs_after_confirm_runs_and_backs_off_after_patience(eps, cfg):
    ln = eps.Learner(cfg)
    start = ln.speed
    for _ in range(int(cfg['confirm_runs'])):
        ln.arm(cfg)
        ln.label('success', cfg)
    assert ln.speed == pytest.approx(start + cfg['speed_step'])
    for _ in range(int(cfg['patience'])):
        ln.arm(cfg)
        ln.label('fail', cfg)
    assert ln.speed == pytest.approx(start)


def test_every_label_is_in_the_ledger_and_a_restart_resumes(eps, cfg):
    s = eps.Session(cfg)
    for result in ('fail', 'success'):
        assert s.arm(cfg) and s.go()
        assert s.label(result, cfg)
    rows = [json.loads(x) for x in eps.LEDGER.read_text().splitlines()]
    assert [r['outcome'] for r in rows] == ['fail', 'success']
    resumed = eps.Session(cfg)
    assert resumed.learner.episode == 2
    assert [h['outcome'] for h in resumed.history] == ['fail', 'success']


def test_undo_takes_back_one_label_across_a_restart(eps, cfg):
    s = eps.Session(cfg)
    s.arm(cfg)
    s.go()
    s.label('fail', cfg)
    assert s.undo()
    assert s.learner.episode == 0
    resumed = eps.Session(cfg)
    assert resumed.learner.episode == 0
    assert resumed.history == []


def test_the_car_profile_loads_over_the_defaults(eps, cfg):
    eps.YAML_PATH.parent.mkdir(parents=True)
    eps.YAML_PATH.write_text('speed_max: 0.6\nnominal:\n  kp: 3.0\n')
    eps.load_cfg()
    assert eps._cfg['speed_max'] == 0.6
    assert eps._cfg['nominal']['kp'] == 3.0
    assert eps._cfg['nominal']['kd'] == cfg['nominal']['kd']  # from the defaults


def test_save_and_the_starting_point_write_the_profile(eps, cfg):
    shipped = eps.DEFAULTS_PATH.read_text()
    eps._cfg['speed_max'] = 0.7
    eps.save_cfg()
    eps.write_init_to_yaml(
        {'kp': 1.5, 'kd': 0.5, 'speed_kp': 0.08, 'speed_kd': 0.03},
        0.2,
        {'width': 60.0, 'window': 10.0, 'side_weight': 0.0, 'lookahead': 8.0},
    )
    assert eps.DEFAULTS_PATH.read_text() == shipped
    saved = yaml.safe_load(eps.YAML_PATH.read_text())
    assert saved['speed_max'] == 0.7
    assert saved['nominal']['kp'] == 1.5
    assert saved['speed_floor'] == 0.2 and saved['width'] == 60.0
    assert 'Nominal gain vector' in eps.YAML_PATH.read_text()  # comments survive


def test_shipped_yaml_starts_at_the_floor_speed(eps, cfg):
    assert yaml.safe_load(eps.DEFAULTS_PATH.read_text())['speed_floor'] == cfg['speed_floor']


@pytest.fixture
def node(eps, cfg, monkeypatch):
    monkeypatch.setattr(eps.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(eps.EpsNode, 'create_publisher', lambda *a, **k: Recorder(), raising=False)
    monkeypatch.setattr(eps.EpsNode, 'create_subscription', lambda *a, **k: None, raising=False)
    monkeypatch.setattr(eps.EpsNode, 'create_timer', lambda *a, **k: None, raising=False)
    return eps.EpsNode(eps.Session(cfg))


def corridor_scan(left=1.0, right=3.0):
    """Build a LakiBeam scan between two walls, the right one further away."""
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_increment = 2 * math.pi / 1440
    msg.range_min, msg.range_max = 0.05, 25.0
    ranges = []
    for i in range(1440):
        raw = math.degrees(msg.angle_min + i * msg.angle_increment)
        ranges.append(left if raw > 0 else right)
    msg.ranges = ranges
    return msg


def test_the_car_drives_only_while_an_episode_runs(eps, node, cfg):
    node._scan_cb(corridor_scan())
    assert node._speed == 0.0 and node._steer == 0.0
    node.session.arm(cfg)
    node.session.go()
    node._scan_cb(corridor_scan())
    assert node._speed > 0.0


def test_more_room_on_the_right_steers_right_on_the_wire(eps, node, cfg):
    node.session.arm(cfg)
    node.session.go()
    node._scan_cb(corridor_scan(left=1.0, right=3.0))
    assert node._steer > 0.0  # student convention: positive is right
    node._actuate()
    assert node._pub.msgs[-1].drive.steering_angle == pytest.approx(-node._steer)


def test_an_empty_block_in_the_profile_keeps_the_shipped_one(eps, cfg):
    eps.YAML_PATH.parent.mkdir(parents=True)
    eps.YAML_PATH.write_text('nominal:\nspeed_max: 0.5\n')
    eps.load_cfg()
    assert eps._cfg['nominal'] == cfg['nominal']
    assert eps._cfg['speed_max'] == 0.5
