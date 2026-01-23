from setuptools import find_packages, setup

package_name = 'neoracer_ros2_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
        ('share/' + package_name + '/launch', ['launch/lidar.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chrisclai',
    maintainer_email='chrisclai02@gmail.com',
    description='Backend ROS2 driver for neoracer v1 with support for OSRbot software stack in collaboration with Seeed Studio',
    license='CERN Open Hardware Licence Version 2 - Strongly Reciprocal',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera = neoracer_ros2_driver.camera:main',
        ],
    },
)
