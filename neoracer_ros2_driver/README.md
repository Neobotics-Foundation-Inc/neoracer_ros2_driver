# neoracer_ros2_driver

ROS 2 driver package for the NeoRacer V1 — nodes, launch files, and configuration.

Full documentation (hardware, architecture, provisioning, the `racecar` shell
tool, networking, autonomy, and troubleshooting) lives in the repository README:

**[../README.md](../README.md)**

| Path | Contents |
|---|---|
| `neoracer_ros2_driver/` | Node implementations: `controller`, `camera`, `gamepad_node`, `mux_node`, `throttle_node`, `led_matrix_node`, `twist_bridge_node` |
| `launch/` | Per-subsystem launch files plus `teleop.launch.py` and `autonomy.launch.py` |
| `config/` | YAML parameters loaded by each launch file |
| `test/` | Package test suite |

Build and run:

```sh
colcon build --packages-select neoracer_ros2_driver --symlink-install
source install/setup.bash
ros2 launch neoracer_ros2_driver teleop.launch.py
```
