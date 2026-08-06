# Architecture

## Overview

ROS 2 Humble driver for the Neoracer (1/12 scale, Jetson Orin Nano). One ESP32
serial bridge carries the IMU, wheel odometry, FlySky RC receiver, and motor
ESC; a software pipeline arbitrates manual and autonomy drive commands; a lidar,
camera, and 8x8 display round out the stack. Published topics match the topic
contract of `MITRacecarNeo/racecar_neo_ros2_driver`, the reference platform, so
one student library serves both cars; see [Topic contract](#topic-contract).

## Components

| Module (path) | Responsibility | Depends on |
| --- | --- | --- |
| `controller` (neoracer_ros2_driver/controller.py) | ESP32 serial bridge; maps `/motor` to the `v` command, publishes `/imu/fused`, `/odom`, `/joy`, `/mag`, `/battery`, and the scalar `rc.physics` topics | controller_lib, pyserial |
| `controller_lib` (neoracer_ros2_driver/controller_lib.py) | pure parsing + FlySky-to-Joy mapping (unit-tested) | - |
| `gamepad_node` (neoracer_ros2_driver/gamepad_node.py) | `/joy` to `/gamepad_drive` | ackermann_msgs |
| `mux_node` (neoracer_ros2_driver/mux_node.py) | gate `/gamepad_drive` (manual) or `/drive` (autonomy) to `/mux_out` | - |
| `throttle_node` (neoracer_ros2_driver/throttle_node.py) | speed/steer caps; `/mux_out` to `/motor` | - |
| `camera` (neoracer_ros2_driver/camera.py) | USB webcam to `/camera/color` (JPEG-in-Image) | opencv, numpy |
| `inference_node` (neoracer_ros2_driver/inference_node.py) | YOLO on `/camera/color` frames to `/edgetpu/inference` (Detection2DArray) | inference_lib, ultralytics, torch, vision_msgs |
| `inference_lib` (neoracer_ros2_driver/inference_lib.py) | pure frame decode + box geometry (unit-tested) | opencv, numpy |
| `led_matrix` (neoracer_ros2_driver/led_matrix_node.py) | `/dotmatrix/text` to USB-UART 8x8 | pyserial |
| `lakibeam1` (pinned fork, cloned to `src/`) | Lakibeam lidar to `/scan` | rclcpp, libcurl |

## Data pipeline

```
FlySky RC -> controller -> /joy -> gamepad_node -> /gamepad_drive ->
   mux_node -> /mux_out -> throttle_node -> /motor -> controller -> ESP32 ESC
student lib -> /drive ------------------------^  (autonomy; gated by FlySky switch)
ESP32 -> /imu/fused, /odom   Lakibeam -> /scan   USB cam -> /camera/color
student lib -> /dotmatrix/text -> led_matrix -> 8x8 display

/camera/color -> inference_node -> /edgetpu/inference -> student lib / downstream perception
                |
                +-> /edgetpu/inference/image (annotated overlay; only while subscribed)
```

## Interfaces

- Serial: ESP32 on `/dev/osrbot_base` (controller.py); 8x8 display on
  `/dev/osrbot_led_matrix` (led_matrix_node.py). Wire protocol in
  `docs/esp32_protocol.md`.
- Network: Lakibeam lidar over UDP at `192.168.8.2` (launch/lidar.launch.py).
- USB video: camera at `/dev/osrbot_usb_cam` (camera.py).
- Topics: `/motor`, `/imu/fused`, `/odom`, `/joy`, `/mag`, `/battery`, `/scan`,
  `/camera/color`, `/edgetpu/inference`, `/dotmatrix/text`, `/drive`, and the
  scalar `/encoder/speed`, `/battery/voltage`, `/rc/channels`.
- Weights: `share/neoracer_ros2_driver/models/`, searched before the working
  directory and before Ultralytics' own cache (inference_lib.resolve_model_path).

## Topic contract

`MITRacecarNeo/racecar_neo_ros2_driver` is the authority on topic names. The
student library (`Neobotics-Foundation-Inc/racecar-neo-library`) subscribes to
that set, so any name this driver invents costs portability. The two platforms
run different hardware behind identical names:

| Topic | Reference source | NeoRacer source |
| --- | --- | --- |
| `/camera/color` | RealSense D435i color, remapped | USB webcam, JPEG-in-Image |
| `/imu/fused` | `imu_fusion_node` blending D435i and LSM9DS1 | ESP32 IMU, single source |
| `/mag`, `/encoder/speed`, `/battery/voltage`, `/rc/channels` | `pit_node` from the Teensy | `controller` from the ESP32 |
| `/edgetpu/inference` | Coral EdgeTPU, tflite | Orin iGPU, YOLO through TensorRT |
| `/dotmatrix/text` | MAX7219 chain, 24x8 | USB-UART 8x8 panel |
| `/scan`, `/joy`, `/drive`, `/motor`, `/mux_out`, `/gamepad_drive` | same | same |

Four reference topics have no NeoRacer hardware behind them, so nothing
publishes them and the matching library method raises `NotImplementedError`
naming the missing part: `/camera/depth` (no depth camera), `/battery/current`
(no current shunt), `/led/pixels` (no addressable strip), and
`/dotmatrix/pixels` (the panel firmware takes text only).

`/odom`, `/odometry/filtered`, `/map`, `/plan`, and `/cmd_vel` are NeoRacer
additions with no reference counterpart. They serve the SLAM and Nav2 layer
(`rc.slam`, `rc.nav`) that the reference platform does not carry.

## GPU stack

`inference_node` is the one driver node that uses the GPU. It is part of the
default teleop stack, so the GPU is held for the life of that stack; disable it
with `inference_enable:=false` to get the memory back. The stack is
installed by `scripts/setup_ml.sh` into the user site so the systemd units (which
run as the same user) can import it.

```
.pt weights -> ultralytics -> ONNX -> TensorRT builder -> .engine (config-specific)
                    |                                          |
                torch/CUDA                              runtime inference
                (training, eager inference)             (deployment)
```

TensorRT comes from JetPack; PyTorch and torchvision come from the jetson-ai-lab
index for the image's CUDA version, because the PyPI aarch64 builds do not target
Tegra. An engine is tied to its GPU, TensorRT version, and platform; the
committed one covers the fleet's stock configuration (SM 8.7, TensorRT
10.3.0.30, JetPack L4T R36.4.3). A car outside it falls back to the `.pt`
rather than failing, and logs that it did.

## Lab dashboards

Three lab tools live in repositories of their own and are cloned into
`scripts/dashboards/` by `scripts/setup_services.sh`, each tracking its default
branch. The driver ships the wiring, not the code: the checkouts are gitignored,
so a car's tuned dashboard yaml survives an update and a lab can be revised
without a driver release.

```
setup_services.sh
    |
    |-- core units -> /etc/systemd/system/ -> enabled at boot
    |
    `-- scripts/dashboards/<name>_dashboard/   (git clone, default branch)
              |
              `-- setup.sh -> renders neoracer-<name>.service.in with its own
                              path -> /etc/systemd/system/ -> installed disabled
```

Each dashboard is a single process that subscribes to driver topics and serves
its own web UI: camlabel (8082) on `/camera/color`, wallfollow (8081) on
`/scan`, pursuit (8083) on `/edgetpu/inference`. They stay off by default
because each holds the camera or the GPU for its whole run, and wallfollow and
pursuit both publish `/drive`, where a second publisher fights the mux. One at a
time, started per session through `racecar service start <name>`.

The unit is a template rather than a fixed file so the service follows the
checkout; that is what lets the driver own the install path without vendoring
the dashboards. `racecar service restart` acts only on enabled units, so a
field update never brings a dashboard up on its own.

## Constraints

- The ESP32 USB-CDC port ignores baud. `/imu/fused` streams ~170 Hz; `/scan` ~30 Hz.
- The ESP32 fails over to direct RC control on serial timeout, so the pipeline
  must stream `/motor` continuously to retain authority.
- All drive topics are normalized to `[-1, 1]`; the physical mapping lives in
  `config/controller.yaml` (m/s, degrees) and `config/throttle.yaml` (caps).
- CPU and GPU share the Orin Nano's 8 GB. A TensorRT build peaks near 3 GB, so
  it competes with a running teleop stack.
- numpy is held below 2.0: `cv_bridge` and `sensor_msgs_py` ship binaries built
  against the numpy 1 C ABI.
