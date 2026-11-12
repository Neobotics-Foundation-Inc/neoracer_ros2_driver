"""Unit tests for the frame tree: frames.yaml loading, xacro render, transforms."""

import ast
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from neoracer_ros2_driver import frames
from neoracer_ros2_driver.frames import load_frames, xacro_mappings
import numpy as np
import pytest
import yaml

xacro = pytest.importorskip('xacro')

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / 'config'
URDF = ROOT / 'urdf' / 'neoracer.urdf.xacro'
LAUNCH = ROOT / 'launch'

# The NeoRacer spec (neoracer-mujoco, src/neoracer_spec/neoracer.py) in its
# body frame: origin at the middle of the wheelbase on the axle line, meters.
SPEC_WHEELBASE = 0.28764
SPEC_WHEEL_RADIUS = 0.0425
SPEC_LIDAR = (-0.0442, 0.0, 0.11995)
SPEC_CAMERA = (0.1692, 0.0, 0.06265)
SPEC_IMU = (0.0, 0.0, 0.0189)
SPEC_OUTLINE_FRONT_X = 0.25967
SPEC_OUTLINE_REAR_X = -0.22377
SPEC_OUTLINE_LEFT_Y = 0.11755
SPEC_OUTLINE_RIGHT_Y = -0.12352
SPEC_OUTLINE_BOTTOM_Z = -0.02298
# The dev car, 2026-11-12: nose square against a wall, the LakiBeam's nearest
# return straight ahead (m), and the LakiBeam's documented accuracy (m).
CAR_WALL_RANGE = 0.306
LIDAR_ACCURACY = 0.02
# Tape on the dev car, 2026-11-12 (mm above the floor): the chassis tub's
# underside, and the LakiBeam's top. The spec's 184 mm runs from the tub's
# underside to the LiDAR's top. The real body sits higher than the CAD.
CAR_TUB_UNDERSIDE_MM = 30.0
CAR_LIDAR_TOP_MM = 218.0
SPEC_TUB_TO_LIDAR_TOP_MM = 184.0


def _spec_to_base_link_mm(p: tuple[float, float, float]) -> list[float]:
    """Move a spec point into base_link: the floor point under the rear axle."""
    return [
        (p[0] + SPEC_WHEELBASE / 2) * 1e3,
        p[1] * 1e3,
        (p[2] + SPEC_WHEEL_RADIUS) * 1e3,
    ]


def _load_launch(name: str):
    spec = importlib.util.spec_from_file_location(name, LAUNCH / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _launch_ast(name: str) -> ast.Module:
    return ast.parse((LAUNCH / f'{name}.launch.py').read_text())


def _string_constants(tree: ast.AST) -> list[str]:
    return [
        n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)
    ]


def _render(values: dict[str, float]) -> ET.Element:
    return ET.fromstring(xacro.process_file(str(URDF), mappings=xacro_mappings(values)).toxml())


def _rot(rpy: list[float]) -> np.ndarray:
    """URDF rpy: fixed-axis roll, then pitch, then yaw (R = Rz Ry Rx)."""
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _transforms(robot: ET.Element) -> dict[str, np.ndarray]:
    """Return each link's 4x4 pose in base_link, following the fixed joints."""
    joints = {}
    for j in robot.iter('joint'):
        o = j.find('origin')
        xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
        tf = np.eye(4)
        tf[:3, :3] = _rot(rpy)
        tf[:3, 3] = xyz
        joints[j.find('child').get('link')] = (j.find('parent').get('link'), tf)
    poses = {'base_link': np.eye(4)}

    def pose(link: str) -> np.ndarray:
        if link not in poses:
            parent, tf = joints[link]
            poses[link] = pose(parent) @ tf
        return poses[link]

    for link in joints:
        pose(link)
    return poses


@pytest.fixture(scope='module')
def values(tmp_path_factory) -> dict[str, float]:
    # The committed fleet values only: a car profile's params/frames.yaml (a
    # calibrated lidar yaw, say) must not change what the tests check.
    fleet = tmp_path_factory.mktemp('fleet')
    (fleet / 'frames.yaml').write_text((CONFIG / 'frames.yaml').read_text())
    return load_frames(fleet)


@pytest.fixture(scope='module')
def robot(values) -> ET.Element:
    return _render(values)


@pytest.fixture(scope='module')
def poses(robot) -> dict[str, np.ndarray]:
    return _transforms(robot)


class TestLoadFrames:
    def test_repo_config_loads(self, values):
        assert values['laser_yaw_deg'] == 0.0

    def test_local_keys_win(self, tmp_path, car_profile_dir):
        (tmp_path / 'frames.yaml').write_text('laser_yaw_deg: 0.0\nlaser_x_mm: 1.0\n')
        (car_profile_dir / 'params').mkdir(parents=True)
        (car_profile_dir / 'params' / 'frames.yaml').write_text('laser_yaw_deg: 2.2\n')
        assert load_frames(tmp_path) == {'laser_yaw_deg': 2.2, 'laser_x_mm': 1.0}

    def test_driver_folder_override_is_ignored(self, tmp_path):
        (tmp_path / 'frames.yaml').write_text('laser_yaw_deg: 0.0\n')
        (tmp_path / 'frames.local.yaml').write_text('laser_yaw_deg: 2.2\n')
        assert load_frames(tmp_path) == {'laser_yaw_deg': 0.0}

    def test_unknown_local_key_is_an_error(self, tmp_path, car_profile_dir):
        (tmp_path / 'frames.yaml').write_text('laser_yaw_deg: 0.0\n')
        (car_profile_dir / 'params').mkdir(parents=True)
        (car_profile_dir / 'params' / 'frames.yaml').write_text('laser_yaw_dg: 2.2\n')
        with pytest.raises(ValueError, match='laser_yaw_dg'):
            load_frames(tmp_path)

    @pytest.mark.parametrize(
        'text', ['laser_x: 1.0\n', 'laser_x_mm: "near"\n', 'laser_x_mm: true\n']
    )
    def test_bad_entries_are_errors(self, tmp_path, text):
        (tmp_path / 'frames.yaml').write_text(text)
        with pytest.raises(ValueError):
            load_frames(tmp_path)

    def test_local_file_is_gitignored(self):
        lines = (ROOT / '.gitignore').read_text().splitlines()
        assert 'config/*.local.yaml' in lines


class TestXacroMappings:
    def test_units_converted_and_suffix_dropped(self):
        got = xacro_mappings({'laser_x_mm': 99.62, 'laser_yaw_deg': 180.0})
        assert float(got['laser_x']) == pytest.approx(0.09962)
        assert float(got['laser_yaw']) == pytest.approx(math.pi)

    def test_every_key_is_used_by_the_urdf(self, values):
        text = URDF.read_text()
        unused = [k for k in xacro_mappings(values) if f'$(arg {k})' not in text]
        assert unused == []

    def test_missing_key_fails_the_render(self, values):
        partial = dict(values)
        del partial['laser_x_mm']
        with pytest.raises(xacro.XacroException, match='laser_x'):
            _render(partial)


class TestFrameTree:
    def test_links(self, poses):
        # The frame_ids the nodes stamp: laser (launch/lidar.launch.py),
        # imu_link (config/controller.yaml), camera_link (config/camera.yaml).
        assert set(poses) == {'base_link', 'base_footprint', 'laser', 'imu_link', 'camera_link'}

    def test_laser_position(self, poses, values):
        mm = poses['laser'][:3, 3] * 1e3
        expected = [values['laser_x_mm'], values['laser_y_mm'], values['laser_z_mm']]
        assert mm == pytest.approx(expected)

    def test_laser_faces_forward(self, poses):
        # The LakiBeam's scan zero (laser +x) points along base +x.
        assert poses['laser'][:3, 0] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)
        assert poses['laser'][:3, 2] == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)

    def test_imu_link_has_base_link_axes(self, poses):
        # config/controller.yaml publishes /imu/fused in REP-103 body axes.
        assert poses['imu_link'][:3, :3] == pytest.approx(np.eye(3), abs=1e-9)

    def test_camera_link_at_the_lens(self, poses, values):
        expected = [values['camera_x_mm'], values['camera_y_mm'], values['camera_z_mm']]
        assert poses['camera_link'][:3, 3] * 1e3 == pytest.approx(expected)

    def test_camera_pitch_tips_the_lens_down(self, values):
        tilted = dict(values, camera_pitch_deg=10.0)
        p = _transforms(_render(tilted))
        assert p['camera_link'][:3, 3] * 1e3 == pytest.approx(
            [values['camera_x_mm'], values['camera_y_mm'], values['camera_z_mm']]
        )
        assert p['camera_link'][2, 0] == pytest.approx(-math.sin(math.radians(10.0)))


class TestLaunch:
    def test_robot_description_renders_from_package_layout(self):
        pytest.importorskip('launch_ros')
        module = _load_launch('frames')
        description = module.robot_description(str(ROOT))
        assert '<robot name="neoracer"' in description

    def test_node_respawns_without_a_retry_cap(self):
        # Humble's launch_ros has no respawn_max_retries.
        calls = [
            n
            for n in ast.walk(_launch_ast('frames'))
            if isinstance(n, ast.Call) and getattr(n.func, 'id', None) == 'Node'
        ]
        assert len(calls) == 1
        keywords = {k.arg: k.value for k in calls[0].keywords}
        assert isinstance(keywords['respawn'], ast.Constant) and keywords['respawn'].value is True
        assert 'respawn_delay' in keywords
        assert 'respawn_max_retries' not in keywords

    def test_teleop_includes_frames_behind_an_enable_arg(self):
        assigns = [
            n
            for n in ast.walk(_launch_ast('teleop'))
            if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', None) == '_SUBSYSTEMS'
        ]
        subsystems = ast.literal_eval(assigns[0].value)
        assert subsystems['frames'] == 'true'

    def test_module_names_frames_yaml(self):
        assert frames.FRAMES_YAML == 'frames.yaml'
        assert (CONFIG / frames.FRAMES_YAML).is_file()


class TestNeoRacer:
    """The NeoRacer's own frame rules: spec values, base_link, one parent each."""

    def test_base_footprint_is_base_link_on_the_floor(self, robot, poses):
        joints = {j.find('child').get('link'): j for j in robot.iter('joint')}
        assert joints['base_footprint'].find('parent').get('link') == 'base_link'
        assert poses['base_footprint'] == pytest.approx(np.eye(4), abs=1e-12)

    def test_every_link_has_one_parent(self, robot):
        links = {link.get('name') for link in robot.iter('link')}
        children = [j.find('child').get('link') for j in robot.iter('joint')]
        assert len(children) == len(set(children))
        assert links - set(children) == {'base_link'}

    def test_offsets_are_the_spec_in_base_link_lifted_as_measured(self, values):
        # x and y are the spec's; z adds the lift the tape measured on the car.
        spec_tub = _spec_to_base_link_mm((0.0, 0.0, SPEC_OUTLINE_BOTTOM_Z))[2]
        body_lift = CAR_TUB_UNDERSIDE_MM - spec_tub
        lidar_lift = CAR_LIDAR_TOP_MM - (spec_tub + SPEC_TUB_TO_LIDAR_TOP_MM)
        for key, point, lift in (
            ('laser', SPEC_LIDAR, lidar_lift),
            ('camera', SPEC_CAMERA, body_lift),
            ('imu', SPEC_IMU, body_lift),
        ):
            want = _spec_to_base_link_mm(point)
            want[2] += lift
            got = [values[f'{key}_x_mm'], values[f'{key}_y_mm'], values[f'{key}_z_mm']]
            assert got == pytest.approx(want, abs=0.01), key

    def test_footprint_is_the_spec_outline(self, values):
        front = _spec_to_base_link_mm((SPEC_OUTLINE_FRONT_X, 0.0, 0.0))[0]
        rear = _spec_to_base_link_mm((SPEC_OUTLINE_REAR_X, 0.0, 0.0))[0]
        assert values['footprint_front_x_mm'] == pytest.approx(front, abs=0.01)
        assert values['footprint_rear_x_mm'] == pytest.approx(rear, abs=0.01)
        assert values['footprint_left_y_mm'] == pytest.approx(SPEC_OUTLINE_LEFT_Y * 1e3, abs=0.01)
        assert values['footprint_right_y_mm'] == pytest.approx(
            SPEC_OUTLINE_RIGHT_Y * 1e3, abs=0.01
        )
        assert values['ride_height_mm'] == pytest.approx(CAR_TUB_UNDERSIDE_MM, abs=0.01)

    def test_lidar_matches_the_wall_check(self, values):
        # The CAD's front of the body ahead of the scan center, against the
        # car's nearest return with its nose on a wall.
        cad = (values['footprint_front_x_mm'] - values['laser_x_mm']) / 1e3
        assert abs(cad - CAR_WALL_RANGE) < LIDAR_ACCURACY

    def test_controller_odometry_is_rooted_at_base_link(self):
        params = yaml.safe_load((CONFIG / 'controller.yaml').read_text())
        assert params['controller_node']['ros__parameters']['base_frame'] == 'base_link'
        source = (ROOT / 'neoracer_ros2_driver' / 'controller.py').read_text()
        assert "declare_parameter('base_frame', 'base_link')" in source

    def test_autonomy_launch_adds_no_second_parent(self):
        # The vendor URDF roots at base_footprint, so it would form a loop with
        # base_link -> base_footprint and give laser, imu_link and camera_link a
        # second transform; the EKF must publish odom -> base_link, the root.
        strings = _string_constants(_launch_ast('autonomy'))
        assert 'osracer_description' not in strings
        assert 'base_footprint' not in strings
        tree = _launch_ast('autonomy')
        frames_set = [
            n.values[i]
            for n in ast.walk(tree)
            if isinstance(n, ast.Dict)
            for i, k in enumerate(n.keys)
            if isinstance(k, ast.Constant) and k.value == 'base_link_frame'
        ]
        assert [ast.literal_eval(v) for v in frames_set] == ['base_link']

    def test_the_urdf_and_its_tools_are_installed(self):
        setup = (ROOT / 'setup.py').read_text()
        assert "glob('urdf/*.xacro')" in setup
        manifest = (ROOT / 'package.xml').read_text()
        assert '<exec_depend>robot_state_publisher</exec_depend>' in manifest
        assert '<exec_depend>xacro</exec_depend>' in manifest
