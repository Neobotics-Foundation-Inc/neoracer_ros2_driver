# Changelog

All notable changes to this project. Format: Keep a Changelog
(keepachangelog.com). Versioning: Semantic Versioning (semver.org).

## [Unreleased]

## [0.3.2] - 2026-08-05

Aligns the topic contract with `MITRacecarNeo/racecar_neo_ros2_driver`, the
reference platform. The student library now reads one set of topic names on
both cars, so a lab written for either runs on the other unchanged; where the
NeoRacer has no matching hardware, the API is present and says so.

### Added
- `/encoder/speed` and `/battery/voltage` (std_msgs/Float32) and `/rc/channels` (std_msgs/Float32MultiArray) from `controller`: the scalar sensor topics `rc.physics` reads on the reference platform. Same data the ESP32 already reported through `/odom`, `/battery`, and `/joy`, under the names the reference publishes. `/rc/channels` carries every FlySky channel in firmware order, normalized by the same calibration `/joy` uses; the library reads the first eight.
- `controller_lib.rc_to_channels`, the pure normalization behind `/rc/channels`, with three unit tests.
- `racecar compile [model]`: TensorRT export as one command. With no argument it reads `model_path` and `imgsz` from `config/inference.yaml`, so the engine is built at the size it will be served at; a bare filename resolves in `models/`, and the engine is placed there whatever the source path. `--imgsz=`, `--device=`, and `--no-half` override the export, `--force` rebuilds over an existing engine. Refuses to start while teleop or autonomy is running, since the builder peaks near 3 GB of the Orin's 8 GB and the stack is already holding half of it.

### Changed
- `inference` defaults to true in `teleop.launch.py`, so `/edgetpu/inference` is on the graph without a launch argument and the student library's `rc.vision` works out of the box. This reverses the 0.3.1 default; the GPU cost that motivated that call has not changed, so pass `inference_enable:=false` on a car that never reads detections or to free the GPU for a `racecar compile` build.
- The dashboard's YOLO inference card is no longer `optional`, since a missing detection topic now means something is wrong rather than a subsystem being off. It stays unsupervised: the watchdog does not restart it, and a car where `racecar setup ml` never ran shows it red.
- Sensor topics renamed to the reference contract: `/camera` -> `/camera/color`, `/imu` -> `/imu/fused`, `/detections` -> `/edgetpu/inference`, `/detections/image` -> `/edgetpu/inference/image`, `/led_matrix/command` -> `/dotmatrix/text`. Anything reading the old names breaks at once rather than going quietly stale; there is no aliasing period.
- `/edgetpu/inference` names an accelerator this car does not have. It is kept because the reference hardcodes it and the library subscribes to it, so a NeoRacer-specific name would cost portability for nothing.
- `publish_mag` now defaults true. `rc.physics.get_magnetic_field()` returned a hardcoded zero vector while the firmware's `m` frame went nowhere.
- `autonomy.launch.py` remaps the complementary filter's `imu/data_raw` onto the absolute `/imu/fused` rather than the relative `imu` it resolved before. The EKF still reads `imu_filter`, so `chassis_ekf_params.yaml` is untouched.
- The reference repo contradicts itself on the color stream: `docs/realsense_topics.md` and one README line say `/camera/forward`, while `edgetpu.yaml`, `realsense.launch.py`, `dashboard.py`, and `watchdog.py` all say `/camera/color`. The code is the contract.
- `models/*.pt` is committed rather than gitignored, so a fresh clone runs inference without a download. The ONNX intermediate and the `.engine` stay ignored: an engine is built against one board's GPU and TensorRT and does not load on another, so each car still builds its own with `racecar compile`.
- `config/inference.yaml` defaults to `yolo26n.pt`, which every clone has. Point it at `yolo26n.engine` per car after compiling; the engine is 2.4x faster in service but cannot be a committed default while it stays per-car.
- TensorRT validated on-car through `teleop.launch.py inference_enable:=true`: `yolo26n.engine` averages 20.9 ms on `/diagnostics` against the fp32 model's 50.1 ms under the same load, with boxes matching fp32 to within a pixel or two at slightly higher scores. Standalone at 640: inference 35.5 -> 12.7 ms, end-to-end 39.6 -> 19.5 ms (25.2 -> 51.2 fps). The 640 build took 471.6 s.
- README benchmark table re-measured on YOLO26n, replacing the YOLO11n figures, and split into idle and full-stack columns since the two differ by more than the backend does.
- Export documentation in `README.md` and `models/README.md` leads with `racecar compile` and keeps the raw `yolo export` line as the manual equivalent; both switch from `yolo11n` to `yolo26n`.

### Fixed
- `neoracer-jupyter.service` sources `setup.bash` instead of hardcoding the ROS environment, matching the dashboard and watchdog units. The hardcoded `PYTHONPATH` listed `/opt/ros/humble/lib/python3.10/site-packages` but not `/opt/ros/humble/local/lib/python3.10/dist-packages`, where `rclpy` actually is, so every notebook failed on `import rclpy` unless its kernel happened to inherit a sourced shell. Predates this release; found while testing the topic rename from a notebook.

## [0.3.1] - 2026-08-04

Puts the 0.3.0 GPU stack to work: a detection node turns camera frames into
`/detections` on the graph, so student code and downstream perception read boxes
instead of each re-running a model on the same frames.

### Added
- `inference_node`: subscribes `/camera`, publishes `vision_msgs/Detection2DArray` on `/detections`. Adapted from the MIT `racecar_neo_ros2_driver` EdgeTPU node, retargeted from pycoral/Coral to Ultralytics on the Orin's integrated GPU. Loads either a `.pt` file or a TensorRT `.engine` through the same call, decodes the camera's JPEG passthrough directly, and takes class names from the weights, so there is no separate labels file to keep in sync.
- `inference_lib.py`: frame decode, weights-path resolution, and box geometry as pure functions, with 17 unit tests in `test/test_inference.py` that need neither a ROS graph nor a GPU.
- `config/inference.yaml`, `launch/inference.launch.py`, and a `models/` drop point with the TensorRT export recipe. `racecar launch inference` runs the node on its own.
- `inference_enable` arg on `teleop.launch.py`, defaulting to false: the model holds GPU memory for the life of the stack, which is wasted on a car whose student code never reads `/detections`.
- Optional annotated overlay on `/detections/image`, JPEG-encoded like `/camera` and drawn only while something is subscribed.
- Inference health on `/diagnostics`: per-frame and averaged latency, detection counts, and frame staleness. Dashboard gains a YOLO inference card.

### Changed
- The detection subscription keeps a depth-1 best-effort queue rather than the shared sensor-data profile. Inference at ~45 ms cannot follow a 60 fps camera, and a deeper queue would hand the model progressively staler frames; dropping to the newest one keeps detections on the present. `max_rate_hz` caps the work below that, leaving GPU headroom for the rest of the stack.
- `package.xml` declares `vision_msgs` and `diagnostic_msgs`. Ultralytics and torch stay outside rosdep, which has no key for the Tegra builds `scripts/setup_ml.sh` installs.

## [0.3.0] - 2026-08-04

GPU release: the Orin's integrated GPU becomes usable from setup, with PyTorch
for training and TensorRT for deployment. Also carries the package flattening
that landed after 0.2.0.

### Added
- Setup phase 6, `scripts/setup_ml.sh`: the on-board GPU stack. Installs PyTorch 2.8.0 and torchvision 0.23.0 built for Tegra, Ultralytics, and the ONNX tooling behind `YOLO.export(format='engine')`, then verifies the TensorRT bindings JetPack already ships. Reachable on its own as `racecar setup ml`. Wheels come from the jetson-ai-lab index keyed by the CUDA version read off the running image, not from PyPI, whose aarch64 torch builds are CPU-only or target datacenter GPUs and will not drive the Orin's integrated GPU.
- CUDA toolchain block in `setup_user_env.sh`: puts `nvcc` and `trtexec` on PATH and exports `CUDA_HOME`. JetPack installs both and leaves neither reachable from a login shell.

### Changed
- numpy in the user site moves to the last 1.x release. Ultralytics floors it at 1.23 and apt ships 1.21.5; numpy 2 is excluded because ROS 2 Humble's binary extensions (`cv_bridge`, `sensor_msgs_py`) are built against the numpy 1 C ABI.
- `cv2` resolves to pip's opencv-python 4.11 rather than apt's 4.5.4, pulled in by Ultralytics' floor of 4.6. `cv_bridge` links its own OpenCV in C++ and is unaffected; the camera node's V4L2 capture path is present in the pip build.
- Flattened the package to the repository root, matching the `racecar_neo_ros2_driver` layout: `config/`, `launch/`, `resource/`, `test/`, `package.xml`, `setup.py`, and `setup.cfg` moved up out of `neoracer_ros2_driver/`, which now holds only the Python module. Reverses the multi-package nesting introduced in 0.2.0, which no longer has a second package to justify it since the lidar driver moved to a pinned fork.

### Fixed
- `colcon test` died during plugin loading on any car that had run the Jupyter phase, before collecting a test. jupyterlab pulls anyio, whose pytest plugin imports `_pytest.scope` (pytest 7+), and Jammy's apt pytest is 6.2.5. `setup_jupyter.sh` now puts a 7.x in the user site alongside the dependency that needs it, held below 8 for ament_cmake_pytest.
- `racecar launch <TAB>` completion listed nothing. It looked for launch files in the repository root while they lived one level down; flattening puts them where the completion already expected.
- Four over-length lines in `scripts/dashboard.py`. `scripts/` was outside the lint scope while the package was nested and is inside it now.

## [0.2.0] - 2026-08-03

Autonomy release: SLAM/Nav2 fused behind the mux, EKF odometry, and the lidar
driver moved out of this repo to a pinned fork. Consolidates upstream work that
shipped without changelog entries between 0.1.0 and this tag.

### Added
- Autonomy layer: osracer SLAM/Nav2 fused behind the mux, with SLAM and navigation mutually exclusive and Python-cased args for Nav2 bringup (`autonomy.launch.py`, `neoracer-autonomy.service`, `launch_autonomy.sh`).
- EKF in the autonomy base; selectable mapping backends and navigation planners.
- `twist_bridge` node: `/cmd_vel` to `/drive` bridge, relays `/odom` to `/odometry/filtered` for osracer consumers, clean shutdown under systemd SIGTERM.
- `controller` broadcasts the `odom` to `base_footprint` TF.
- `controller` publishes `/battery` from the V1.1 firmware `b` frame; dashboard gains a battery card.
- Lidar all-inf scan watchdog and remote-logger crash fix, carried by the Neobotics `Lakibeam_ROS2_Driver` fork; `docs/lidar_scan_health.md`.
- `racecar mapping` / `racecar navigation` for on-demand SLAM/Nav, plus a `racecar update` that does repo reset, full setup, and service restart in one command.
- Dashboard: live node graph view (rqt-style), autonomy monitoring, EKF and IMU filter cards.
- Setup auto-installs the student library and labs on a fresh car; labs refresh preserves student files.

### Changed
- Restructured to a multi-package layout; the Python package moved into `neoracer_ros2_driver/`.
- Camera publishes native MJPG passthrough at 60 fps (no decode/re-encode).
- Throttle unlocks the full ESC swing for speed; steering stays at 0.625.
- `mux` gained autonomy passthrough for hardware-switched (FlySky) cars.
- Networking configures the Ethernet dual-IP through NetworkManager rather than netplan, creates a DHCP autoconnect profile for the native RJ45, and defaults to the real car values (`nr_eth0`, `192.168.10.100`).
- `racecar service start/stop/restart` with no unit now acts on all four services.
- Dashboard redesigned on the Neobotics 4-colour palette (white page, blue LED, kernel type, quiet status footer, solid status colours).
- One shared lakibeam across the neoracer and osracer workspaces; osracer's lidar points at the USB-C sensor IP (`192.168.8.2`).
- Student library installs from the Neobotics fork's `main` branch.

### Fixed
- `controller` parses the V1.1 firmware state frame; `/imu` and `/odom` were silently dead.
- `controller` tolerates integer YAML values for float parameters.
- Lidar remote logger nullptr crash; sensor config push re-enabled behind a bounded readiness wait.
- Dashboard survives unreadable thermal zones (`cv*` EAGAIN killed the monitor).
- Networking setup no longer aborts the run when a Cudy port is dark.
- Setup only passes `--break-system-packages` when pip supports it.

### Removed
- The third-party `lakibeam1` driver is no longer kept in this repo (52 files, including a bundled rapidjson). `setup_workspace.sh` clones it into `src/lakibeam1` from the Neobotics fork at a pinned tag instead, and re-points a factory image's upstream clone at the fork.

## [0.1.0] - 2026-06-06

Migration of the MIT racecar_neo_ros2_driver (1/14 RACECAR Neo, Raspberry Pi 5)
to the Neoracer (1/12 scale, Jetson Orin Nano, ROS 2 Humble). Satisfies the
Neobotics student-library topic contract and keeps the fleet toolchain.

### Added
- ESP32 bridge `controller`: subscribes `/motor`; publishes `/imu`, `/odom`, and `/joy` synthesized from FlySky RC channels (parameterized RC-to-Joy mapping in `controller_lib.py`, with unit tests).
- `led_matrix` node: USB-UART passthrough to the 8x8 display on `/led_matrix/command`.
- Fleet toolchain retargeted for Jetson/Humble: 7-phase setup orchestrator, `racecar` tool, watchdog, web dashboard, systemd services, networking.
- udev rules for `/dev/osrbot_base`, `/dev/osrbot_led_matrix`, `/dev/osrbot_usb_cam`.
- Docs: `docs/architecture.md`, `docs/esp32_protocol.md`, on-car checklist.

### Changed
- Control pipeline (`gamepad` to `mux` to `throttle` to `/motor`) retained so FlySky (manual) and the student library (`/drive`, autonomy) share one arbitrated, speed-capped path; a 3-position FlySky switch selects the mode.
- Camera: single `/camera` publisher (USB webcam, JPEG-in-Image), hardened with a stable device symlink, MJPG request, and reconnect.
- LIDAR: `lakibeam1` cloned into `src/` by `setup_workspace.sh` (sibling package); default `sensorip` `192.168.8.2` (USB-C). Replaces RPLIDAR/`sllidar`.
- Dashboard health check reads Jetson thermals; services target ROS Humble (Python 3.10).

### Removed
- LSM9DS1 I2C IMU node and Pololu Maestro PWM node (folded into the ESP32 bridge).
- Rear camera, MAX7219 SPI dotmatrix node, Coral EdgeTPU node.
- Raspberry Pi-specific setup (raspi-config) and PMIC/RTC health checks.
