from glob import glob

from setuptools import find_packages, setup

package_name = 'neoracer_ros2_driver'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Weights are gitignored; this ships whatever a given car has dropped in.
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chrisclai',
    maintainer_email='chrisclai02@gmail.com',
    description=(
        'Backend ROS2 driver for neoracer v1 with OSRbot software '
        'stack support, in collaboration with Seeed Studio'),
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = neoracer_ros2_driver.controller:main',
            'camera = neoracer_ros2_driver.camera:main',
            'inference_node = neoracer_ros2_driver.inference_node:main',
            'gamepad_node = neoracer_ros2_driver.gamepad_node:main',
            'mux_node = neoracer_ros2_driver.mux_node:main',
            'throttle_node = neoracer_ros2_driver.throttle_node:main',
            'led_matrix = neoracer_ros2_driver.led_matrix_node:main',
            'twist_bridge = neoracer_ros2_driver.twist_bridge_node:main',
        ],
    },
)
