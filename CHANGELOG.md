# Changelog

All notable changes to this project. Format: Keep a Changelog
(keepachangelog.com). Versioning: Semantic Versioning (semver.org).

## [Unreleased]

### Changed
- Flattened the package to the repository root, matching the
  `racecar_neo_ros2_driver` layout: `config/`, `launch/`, `resource/`, `test/`,
  `package.xml`, `setup.py`, and `setup.cfg` moved up out of
  `neoracer_ros2_driver/`, which now holds only the Python module. Reverses the
  multi-package nesting introduced in 0.2.0, which no longer has a second
  package to justify it since the lidar driver moved to a pinned fork.

### Fixed
- `racecar launch <TAB>` completion listed nothing. It looked for launch files
  in the repository root while they lived one level down; flattening puts them
  where the completion already expected.
- Four over-length lines in `scripts/dashboard.py`. `scripts/` was outside the
  lint scope while the package was nested and is inside it now.

## [0.2.0] - 2026-08-03

Autonomy release: SLAM/Nav2 fused behind the mux, EKF odometry, and the lidar
driver moved out of this repo to a pinned fork. Consolidates upstream work that
shipped without changelog entries between 0.1.0 and this tag.

### Added
- Autonomy layer: osracer SLAM/Nav2 fused behind the mux, with SLAM and
  navigation mutually exclusive and Python-cased args for Nav2 bringup
  (`autonomy.launch.py`, `neoracer-autonomy.service`, `launch_autonomy.sh`).
- EKF in the autonomy base; selectable mapping backends and navigation planners.
- `twist_bridge` node: `/cmd_vel` to `/drive` bridge, relays `/odom` to
  `/odometry/filtered` for osracer consumers, clean shutdown under systemd
  SIGTERM.
- `controller` broadcasts the `odom` to `base_footprint` TF.
- `controller` publishes `/battery` from the V1.1 firmware `b` frame; dashboard
  gains a battery card.
- Lidar all-inf scan watchdog and remote-logger crash fix, carried by the
  Neobotics `Lakibeam_ROS2_Driver` fork; `docs/lidar_scan_health.md`.
- `racecar mapping` / `racecar navigation` for on-demand SLAM/Nav, plus a
  `racecar update` that does repo reset, full setup, and service restart in one
  command.
- Dashboard: live node graph view (rqt-style), autonomy monitoring, EKF and IMU
  filter cards.
- Setup auto-installs the student library and labs on a fresh car; labs refresh
  preserves student files.

### Changed
- Restructured to a multi-package layout; the Python package moved into
  `neoracer_ros2_driver/`.
- Camera publishes native MJPG passthrough at 60 fps (no decode/re-encode).
- Throttle unlocks the full ESC swing for speed; steering stays at 0.625.
- `mux` gained autonomy passthrough for hardware-switched (FlySky) cars.
- Networking configures the Ethernet dual-IP through NetworkManager rather than
  netplan, creates a DHCP autoconnect profile for the native RJ45, and defaults
  to the real car values (`nr_eth0`, `192.168.10.100`).
- `racecar service start/stop/restart` with no unit now acts on all four
  services.
- Dashboard redesigned on the Neobotics 4-colour palette (white page, blue LED,
  kernel type, quiet status footer, solid status colours).
- One shared lakibeam across the neoracer and osracer workspaces; osracer's
  lidar points at the USB-C sensor IP (`192.168.8.2`).
- Student library installs from the Neobotics fork's `main` branch.

### Fixed
- `controller` parses the V1.1 firmware state frame; `/imu` and `/odom` were
  silently dead.
- `controller` tolerates integer YAML values for float parameters.
- Lidar remote logger nullptr crash; sensor config push re-enabled behind a
  bounded readiness wait.
- Dashboard survives unreadable thermal zones (`cv*` EAGAIN killed the monitor).
- Networking setup no longer aborts the run when a Cudy port is dark.
- Setup only passes `--break-system-packages` when pip supports it.

### Removed
- The third-party `lakibeam1` driver is no longer kept in this repo (52 files,
  including a bundled rapidjson). `setup_workspace.sh` clones it into
  `src/lakibeam1` from the Neobotics fork at a pinned tag instead, and
  re-points a factory image's upstream clone at the fork.

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
