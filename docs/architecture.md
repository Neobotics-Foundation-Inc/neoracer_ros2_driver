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
| `inference_node` (neoracer_ros2_driver/inference_node.py) | YOLO on `/camera` frames to `/detections` (Detection2DArray) | inference_lib, ultralytics, torch, vision_msgs |
| `inference_lib` (neoracer_ros2_driver/inference_lib.py) | pure frame decode + box geometry (unit-tested) | opencv, numpy |
| `led_matrix` (neoracer_ros2_driver/led_matrix_node.py) | `/led_matrix/command` to USB-UART 8x8 | pyserial |
| `lakibeam1` (pinned fork, cloned to `src/`) | Lakibeam lidar to `/scan` | rclcpp, libcurl |

## Data pipeline

```
FlySky RC -> controller -> /joy -> gamepad_node -> /gamepad_drive ->
   mux_node -> /mux_out -> throttle_node -> /motor -> controller -> ESP32 ESC
student lib -> /drive ------------------------^  (autonomy; gated by FlySky switch)
ESP32 -> /imu, /odom     Lakibeam -> /scan     USB cam -> /camera
student lib -> /led_matrix/command -> led_matrix -> 8x8 display

/camera -> inference_node -> /detections -> student lib / downstream perception
                |
                +-> /detections/image (annotated overlay; only while subscribed)
```

## Interfaces

- Serial: ESP32 on `/dev/osrbot_base` (controller.py); 8x8 display on
  `/dev/osrbot_led_matrix` (led_matrix_node.py). Wire protocol in
  `docs/esp32_protocol.md`.
- Network: Lakibeam lidar over UDP at `192.168.8.2` (launch/lidar.launch.py).
- USB video: camera at `/dev/osrbot_usb_cam` (camera.py).
- Topics: `/motor`, `/imu`, `/odom`, `/joy`, `/scan`, `/camera`, `/detections`,
  `/led_matrix/command`, `/drive`.
- Weights: `share/neoracer_ros2_driver/models/`, searched before the working
  directory and before Ultralytics' own cache (inference_lib.resolve_model_path).

## GPU stack

`inference_node` is the one driver node that uses the GPU, and it is off by
default in `teleop.launch.py`; every other node runs without it. The stack is
installed by `scripts/setup_ml.sh` into the user site so the systemd units (which
run as the same user) can import it.

```
.pt weights -> ultralytics -> ONNX -> TensorRT builder -> .engine (board-specific)
                    |                                          |
                torch/CUDA                              runtime inference
                (training, eager inference)             (deployment)
```

TensorRT comes from JetPack; PyTorch and torchvision come from the jetson-ai-lab
index for the image's CUDA version, because the PyPI aarch64 builds do not target
Tegra. Engines are not portable across boards or TensorRT versions.

## Constraints

- The ESP32 USB-CDC port ignores baud. `/imu` streams ~170 Hz; `/scan` ~30 Hz.
- The ESP32 fails over to direct RC control on serial timeout, so the pipeline
  must stream `/motor` continuously to retain authority.
- All drive topics are normalized to `[-1, 1]`; the physical mapping lives in
  `config/controller.yaml` (m/s, degrees) and `config/throttle.yaml` (caps).
- CPU and GPU share the Orin Nano's 8 GB. A TensorRT build peaks near 3 GB, so
  it competes with a running teleop stack.
- numpy is held below 2.0: `cv_bridge` and `sensor_msgs_py` ship binaries built
  against the numpy 1 C ABI.
