# NeoRacer ROS 2 Driver

Everything that runs on a NeoRacer car lives in this repo. Clone it into the
car's workspace and build; there are no side-loaded packages.

## Packages

| Package | What it is |
|---------|------------|
| `neoracer_ros2_driver` | Vehicle driver stack: controller, gamepad, mux, throttle, camera, LED matrix, odometry TF. Launch entry point is `teleop.launch.py`. |
| `lakibeam1` | Vendored RichBeam LakiBeam1 lidar driver, publishes `/scan`. Carries local fixes; see `docs/lidar_scan_health.md`. |

## Provisioning a car

```
cd ~/ros2_ws/src
git clone https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver.git
cd ~/ros2_ws
colcon build --symlink-install
sudo systemctl restart neoracer-teleop
```

`docs/` holds the operational checklists. `scripts/` holds car setup tooling.
