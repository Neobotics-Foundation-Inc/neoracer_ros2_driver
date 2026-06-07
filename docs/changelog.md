# Changelog

All notable changes to this project. Format: Keep a Changelog
(keepachangelog.com). Versioning: Semantic Versioning (semver.org).

## [0.1.0] - 2026-06-06

Migration of the MIT racecar_neo_ros2_driver (1/14 RACECAR Neo, Raspberry Pi 5)
to the Neoracer (1/12 scale, Jetson Orin Nano, ROS 2 Humble). Satisfies the
Neobotics student-library topic contract and keeps the fleet toolchain.

### Added
- ESP32 bridge `controller`: subscribes `/motor`; publishes `/imu`, `/odom`,
  and `/joy` synthesized from FlySky RC channels (parameterized RC-to-Joy
  mapping in `controller_lib.py`, with unit tests).
- `led_matrix` node: USB-UART passthrough to the 8x8 display on
  `/led_matrix/command`.
- Fleet toolchain retargeted for Jetson/Humble: 7-phase setup orchestrator,
  `racecar` tool, watchdog, web dashboard, systemd services, networking.
- udev rules for `/dev/osrbot_base`, `/dev/osrbot_led_matrix`,
  `/dev/osrbot_usb_cam`.
- Docs: `docs/architecture.md`, `docs/esp32_protocol.md`, on-car checklist.

### Changed
- Control pipeline (`gamepad` to `mux` to `throttle` to `/motor`) retained so
  FlySky (manual) and the student library (`/drive`, autonomy) share one
  arbitrated, speed-capped path; a 3-position FlySky switch selects the mode.
- Camera: single `/camera` publisher (USB webcam, JPEG-in-Image), hardened with
  a stable device symlink, MJPG request, and reconnect.
- LIDAR: `lakibeam1` cloned into `src/` by `setup_workspace.sh` (sibling
  package); default `sensorip` `192.168.8.2` (USB-C). Replaces RPLIDAR/`sllidar`.
- Dashboard health check reads Jetson thermals; services target ROS Humble
  (Python 3.10).

### Removed
- LSM9DS1 I2C IMU node and Pololu Maestro PWM node (folded into the ESP32 bridge).
- Rear camera, MAX7219 SPI dotmatrix node, Coral EdgeTPU node.
- Raspberry Pi-specific setup (raspi-config) and PMIC/RTC health checks.
