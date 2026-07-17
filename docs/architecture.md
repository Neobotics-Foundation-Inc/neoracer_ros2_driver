# Architecture

## Overview

ROS 2 Humble driver for the Neoracer (1/12 scale, Jetson Orin Nano). One ESP32
serial bridge carries the IMU, wheel odometry, FlySky RC receiver, and motor
ESC; a software pipeline arbitrates manual and autonomy drive commands; a lidar,
camera, and 8x8 display round out the stack. Published topics match the Neobotics
student-library contract.

## Components

| Module (path) | Responsibility | Depends on |
| --- | --- | --- |
| `controller` (neoracer_ros2_driver/controller.py) | ESP32 serial bridge; maps `/motor` to the `v` command, publishes `/imu`, `/odom`, `/joy` | controller_lib, pyserial |
| `controller_lib` (neoracer_ros2_driver/controller_lib.py) | pure parsing + FlySky-to-Joy mapping (unit-tested) | - |
| `gamepad_node` (neoracer_ros2_driver/gamepad_node.py) | `/joy` to `/gamepad_drive` | ackermann_msgs |
| `mux_node` (neoracer_ros2_driver/mux_node.py) | gate `/gamepad_drive` (manual) or `/drive` (autonomy) to `/mux_out` | - |
| `throttle_node` (neoracer_ros2_driver/throttle_node.py) | speed/steer caps; `/mux_out` to `/motor` | - |
| `camera` (neoracer_ros2_driver/camera.py) | USB webcam to `/camera` (JPEG-in-Image) | opencv, numpy |
| `led_matrix` (neoracer_ros2_driver/led_matrix_node.py) | `/led_matrix/command` to USB-UART 8x8 | pyserial |
| `lakibeam1` (vendored in-repo) | Lakibeam lidar to `/scan` | rclcpp, libcurl |

## Data pipeline

```
FlySky RC -> controller -> /joy -> gamepad_node -> /gamepad_drive ->
   mux_node -> /mux_out -> throttle_node -> /motor -> controller -> ESP32 ESC
student lib -> /drive ------------------------^  (autonomy; gated by FlySky switch)
ESP32 -> /imu, /odom     Lakibeam -> /scan     USB cam -> /camera
student lib -> /led_matrix/command -> led_matrix -> 8x8 display
```

## Interfaces

- Serial: ESP32 on `/dev/osrbot_base` (controller.py); 8x8 display on
  `/dev/osrbot_led_matrix` (led_matrix_node.py). Wire protocol in
  `docs/esp32_protocol.md`.
- Network: Lakibeam lidar over UDP at `192.168.8.2` (launch/lidar.launch.py).
- USB video: camera at `/dev/osrbot_usb_cam` (camera.py).
- Topics: `/motor`, `/imu`, `/odom`, `/joy`, `/scan`, `/camera`,
  `/led_matrix/command`, `/drive`.

## Constraints

- The ESP32 USB-CDC port ignores baud. `/imu` streams ~170 Hz; `/scan` ~30 Hz.
- The ESP32 fails over to direct RC control on serial timeout, so the pipeline
  must stream `/motor` continuously to retain authority.
- All drive topics are normalized to `[-1, 1]`; the physical mapping lives in
  `config/controller.yaml` (m/s, degrees) and `config/throttle.yaml` (caps).
