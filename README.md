# neoracer_ros2_driver

ROS 2 driver for the NeoRacer V1, a 1:12-scale autonomous Ackermann-steering racing robot built by [Neobotics Foundation](https://github.com/Neobotics-Foundation-Inc) in collaboration with Seeed Studio.

## Contents

- [Hardware](#hardware)
- [Architecture](#architecture)
- [Quickstart (fresh JetPack 6.2 install)](#quickstart-fresh-jetpack-62-install)
- [The `racecar` shell tool](#the-racecar-shell-tool)
- [Networking (optional)](#networking-optional)
- [Autonomy: SLAM and Nav2](#autonomy-slam-and-nav2)
- [Web dashboard](#web-dashboard)
- [Jupyter notebooks and the student library](#jupyter-notebooks-and-the-student-library)
- [GPU stack](#gpu-stack)
- [Object detection](#object-detection)
- [Checking lidar health](#checking-lidar-health)
- [Coexisting with the vendor workspace](#coexisting-with-the-vendor-workspace)
- [Manual build](#manual-build)
- [Launch](#launch)
- [Troubleshooting](#troubleshooting)
- [Changelog](#changelog)
- [License](#license)
- [Citation](#citation)

## Hardware

| Subsystem | Component | Interface | udev symlink |
|---|---|---|---|
| Drive / steering / IMU / odometry / RC | Seeed OSRbot **ESP32** (OSCORE_NEO) | USB-CDC | `/dev/osrbot_base` |
| 2D LIDAR | RichBeam **LakiBeam1** | Ethernet over USB-C (UDP), sensor at `192.168.8.2` | — |
| Forward camera | USB webcam (MJPG) | USB | `/dev/osrbot_usb_cam` |
| Display | 8×8 LED dot matrix | USB-UART | `/dev/osrbot_led_matrix` |
| Manual control | **FlySky** RC transmitter | receiver wired to the ESP32 | — |
| Compute | Jetson Orin Nano (Seeed reComputer J4012) | — | — |

All `/dev/osrbot_*` paths are stable udev symlinks installed by `scripts/setup_udev.sh`, so devices won't shift between `ttyACM0` and `ttyACM1` across reboots.

One ESP32 carries the IMU, wheel odometry, the FlySky receiver, and the motor ESC. A single `controller` node owns that serial link; there is no separate IMU, PWM, or joystick driver.

## Architecture

```
FlySky RC ──(ESP32)──> controller ──/joy──> gamepad_node ──/gamepad_drive──┐
                          │                                                ├──> mux_node ──/mux_out──> throttle_node ──/motor──┐
                          │        student library / Nav2 ──────/drive─────┘   (FlySky switch: idle | manual | autonomy)       │
                          │                                                                                                   │
                          └──> /imu/fused, /odom, /battery                    controller writes "v <m/s> <deg>" to the ESP32 <───────┘

LakiBeam1 ──(UDP)──> lakibeam1_scan_node ──/scan
USB webcam ────────> camera ──/camera/color  (JPEG-in-Image) ──> inference_node ──/edgetpu/inference
Nav2 ──/cmd_vel──> twist_bridge ──/drive
student library ──/dotmatrix/text──> led_matrix ──(USB-UART)──> 8×8 display
```

Published topics:

- `/imu/fused` (sensor_msgs/Imu), `/odom` (nav_msgs/Odometry), `/battery`: from the ESP32 V1.1 firmware state frame
- `/joy` (sensor_msgs/Joy): synthesized from FlySky RC channels
- `/mag` (sensor_msgs/MagneticField): ESP32 magnetometer, Gauss converted to Tesla
- `/encoder/speed`, `/battery/voltage` (std_msgs/Float32), `/rc/channels` (std_msgs/Float32MultiArray): the scalar racecar_neo sensor topics the student library's `rc.physics` reads. Same data as `/odom`, `/battery`, and `/joy`, under the names the reference platform publishes
- `/scan` (sensor_msgs/LaserScan): LakiBeam1
- `/camera/color` (sensor_msgs/Image, `encoding: jpeg`): native MJPG passthrough at 60 fps. The `data` field is the raw JPEG byte stream, which the student library decodes with `cv2.imdecode`; the node does not decode and re-encode.
- `/edgetpu/inference` (vision_msgs/Detection2DArray): YOLO boxes for the frames on `/camera/color`, stamped with that frame's header. On by default; see [Object detection](#object-detection).

The control pipeline (`gamepad` → `mux` → `throttle` → `/motor`) enforces speed caps, arming, and manual/autonomy arbitration, so RC driving and autonomous code share one speed-limited path. A 3-position FlySky switch selects **idle / manual / autonomy**, mapped to the `/joy` buttons the mux reads. Every topic in the pipeline is normalized to `[-1, 1]`; top speed and steering limits live in `config/throttle.yaml`, and the normalized→m/s mapping in `config/controller.yaml`.

Safety and uptime layers:

- **Mux** gates commands behind the arming switch and zeroes output when the RC link drops.
- **Watchdog** (`scripts/watchdog.py`) supervises the control and sensor nodes, restarts a dead node, and monitors `/imu/fused` and `/scan` freshness.
- **Five systemd units** (`neoracer-{teleop,watchdog,dashboard,jupyter,autonomy}.service`) wired with `BindsTo=`/`Wants=` so the watchdog follows teleop up and down.
- **Launch wrapper** (`scripts/launch_teleop.sh`) creates `~/logs/<timestamp>/`, updates `~/logs/latest`, sweeps FastRTPS SHM orphans, and `exec`s `ros2 launch` so systemd tracks the launch PID.

## Quickstart (fresh JetPack 6.2 install)

Target: **Jetson Orin Nano** (Seeed reComputer J4012) running JetPack 6.2: Ubuntu 22.04.5 LTS (Jammy), Python 3.10, ROS 2 Humble. Humble is the only supported distro on Jammy.

### 1. Flash JetPack

Follow Seeed's [JetPack flashing guide](https://wiki.seeedstudio.com/reComputer_J4012_Flash_Jetpack/) for the reComputer J4012. Factory NeoRacer images ship with JetPack and this repo already present. Skip to step 3 and `git pull` instead of cloning.

### 2. First-boot configuration

Set these during the Ubuntu first-boot wizard. The `racecar` shell tool, udev groups, and systemd `User=` directives are hard-coded to the username; do not change it.

- **Username**: `racecar`
- **Hostname**: `neoracer`
- **Password**: your choice (fleet default is `neobotics`)

Then SSH in, or work on the console.

### 3. Clone and run the orchestrator

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver.git
bash neoracer_ros2_driver/scripts/setup_all.sh
```

`setup_all.sh` is idempotent. Each phase checks for existing state and skips when already applied, so re-running is safe.

### 4. Apply group memberships

Setup adds your user to `dialout` and the video groups. Group membership applies to new login sessions only:

```sh
exit                     # close SSH
ssh racecar@<ip>         # back in, groups now active
groups                   # verify dialout appears
```

`newgrp dialout` applies it to a single shell without reconnecting.

### 5. Plug in the hardware and reboot

With the car powered off, connect the ESP32 base, the LakiBeam1 (USB-C), the webcam, and the LED matrix. Power on, then:

```sh
sudo reboot
```

After reboot, verify and start the stack:

```sh
racecar status              # USB peripherals + /dev/osrbot_* symlinks + running nodes
racecar service status      # units enabled by setup, teleop not yet running
racecar service start       # starts neoracer-teleop (watchdog follows)
```

Browse to `http://neoracer.local:8080` for the live dashboard.

### 6. (Optional) Switch to AP-mode networking

Once the wired setup works, untether the car from your lab WiFi:

```sh
racecar setup networking
```

Run this from a wired session or the console. It reconfigures the WiFi interface and will drop SSH-over-WiFi. See [Networking (optional)](#networking-optional).

### What `setup_all.sh` does

Eight phases, all under `scripts/`:

1. **`setup_ros2.sh`**: ROS 2 Humble apt repo + message/driver packages
2. **`setup_dev_tools.sh`**: build tools and Python hardware libraries
3. **`setup_user_env.sh`**: joins device groups; sources ROS 2, the CUDA toolchain, and the `racecar` shell tool in `.bashrc`
4. **`setup_udev.sh`**: installs `/etc/udev/rules.d/99-racecar.rules` (stable `/dev/osrbot_*`)
5. **`setup_workspace.sh`**: clones the LakiBeam1 driver into `src/lakibeam1` at a pinned tag, then `colcon build --symlink-install`
6. **`setup_ml.sh`**: PyTorch/torchvision for Tegra, Ultralytics, ONNX export tooling; verifies the JetPack TensorRT bindings
7. **`setup_jupyter.sh`**: JupyterLab, `~/jupyter_ws/`, and the student library + labs
8. **`setup_services.sh`**: installs and enables the systemd units

Networking is not one of the phases: it reconfigures WiFi and would drop an SSH-over-WiFi install. Any phase script runs on its own to redo a step, e.g. `bash scripts/setup_udev.sh` after a hardware swap.

The lidar driver (package `lakibeam1`) is third-party and is not vendored in this repo. Phase 5 clones it from the [Neobotics fork](https://github.com/Neobotics-Foundation-Inc/Lakibeam_ROS2_Driver) at tag `v1.0-neobotics`, which carries the scan-health fixes described in [docs/lidar_scan_health.md](docs/lidar_scan_health.md). Override with `LAKI_REPO=` / `LAKI_PIN=` to test an unreleased revision.

## The `racecar` shell tool

`setup_user_env.sh` sources [`scripts/racecar-tool.sh`](scripts/racecar-tool.sh) into `~/.bashrc`. Re-open a shell, then:

```sh
racecar build               # colcon build --symlink-install + source overlay
racecar test                # package test suite, verbose
racecar source              # source the workspace overlay
racecar cd                  # chdir to the package source root
racecar teleop              # full stack in the foreground via launch_teleop.sh
racecar launch lidar        # ros2 launch neoracer_ros2_driver lidar.launch.py
racecar compile             # TensorRT-export the weights inference.yaml points at
racecar compile custom.pt   # ...or a specific .pt, resolved in models/
racecar watchdog            # run the supervisor in the foreground

racecar service status      # active/enabled snapshot for all units
racecar service install     # drop unit files in /etc/systemd/system/ + enable
racecar service start       # default: start teleop (watchdog follows)
racecar service stop        # default: stop teleop
racecar service logs teleop # journalctl -u neoracer-teleop -f

racecar mapping             # SLAM to build a map (slam_toolbox default)
racecar mapping save <name> # save the current map
racecar navigation          # Nav2 on a saved map (teb default)
racecar navigation rviz     # goal-setting RViz view (car desktop only)

racecar setup all           # the 7-phase orchestrator
racecar setup networking    # WiFi AP + Ethernet + lidar subnet
racecar update              # repo to latest main + full setup + service restart

racecar ws [neoracer|osracer]   # switch this shell between workspaces
racecar library --list          # student library folders in ~/jupyter_ws
racecar library --select <dir>  # point the student library at a notebook tree
racecar selftest --led[=TEXT]   # send test text to the 8x8 display
racecar clear --led             # clear the display
racecar udev                    # re-install the udev rules
racecar status                  # peripherals + symlinks + running nodes
racecar cleanup [--force]       # list / kill stale processes + SHM orphans
racecar help                    # full usage
```

`racecar update` resets the repo to `origin/main` (discarding local edits), runs the full setup, and restarts every service. Requires internet.

## Networking (optional)

```sh
racecar setup networking
```

Configures an isolated WiFi access point (so a laptop can reach the car headless), a static + DHCP address on Ethernet, and the LakiBeam1 lidar subnet.

> Warning: this reconfigures WiFi. Run it from a wired session or the console, or you will drop your own SSH connection.

Defaults, persisted to `~/.config/racecar/networking.env` and replayed on every re-run:

| Setting | Default | Flag |
|---|---|---|
| AP SSID | `neoracer-1` | `--ssid=NAME` |
| AP password | `neobotics` | `--psk=PASS` |
| AP channel | `6` | `--channel=N` |
| AP address | `10.42.0.1/24` | `--ap-addr=CIDR` |
| Ethernet static | `192.168.10.100/24` | `--eth-static=CIDR` |
| Lidar host | `192.168.8.1/24` | `--lidar-host=CIDR` |
| Lidar sensor | `192.168.8.2` | — |
| WiFi interface | `wlP1p1s0` | `--wifi-iface=NAME` |
| Ethernet interface | `nr_eth0` | `--eth-iface=NAME` |

Ethernet is configured through NetworkManager (not netplan), carrying both the static address and a DHCP autoconnect profile on the native RJ45. The car is reachable at a known address on a bare switch and via DHCP on a lab network.

Inspect or clear the saved overrides:

```sh
racecar setup networking --show     # print current persisted values
racecar setup networking --reset    # disable the AP + clear the saved car ID
```

Run `--reset` before capturing a golden image, so the clone ships with no active AP and no baked-in SSID.

## Autonomy: SLAM and Nav2

SLAM and navigation run on demand, not as services. The always-on autonomy base (`autonomy.launch.py`, `neoracer-autonomy.service`) runs only TF, the EKF, and the twist bridge.

```sh
racecar mapping                 # slam_toolbox (also: gmapping, cartographer)
racecar mapping save my_track   # save the map under that name
racecar navigation              # Nav2 with the teb planner (also: dwb)
racecar navigation rviz         # RViz goal-setting view, on the car's desktop
```

`twist_bridge` converts Nav2's `/cmd_vel` (Twist, m/s and rad/s) into the normalized `/drive` topic the mux arbitrates, dividing out the same `max_speed_mps` / `max_steering_angle_deg` constants the controller multiplies back in, so a commanded 1.5 m/s reaches the firmware as 1.5 m/s regardless of the throttle caps in between. Steering comes from the bicycle model.

Autonomy commands enter through the same mux as manual driving, so the FlySky arming switch still governs the car.

## Web dashboard

With `neoracer-dashboard` running, browse to `http://neoracer.local:8080`:

- **Nodes**: one card per monitored subsystem, coloured by liveness
- **System health**: Jetson thermals, battery, kernel type
- **Topic rates**: live Hz for the control and sensor topics, yellow when stale, red when missing
- **Node graph**: live rqt-style view of the running graph
- **Watchdog log**: tail of `~/logs/latest/watchdog.log`, so restart events are visible

## Jupyter notebooks and the student library

`http://neoracer.local:8888` serves JupyterLab from `~/jupyter_ws`, with `import rclpy` preconfigured.

Setup also installs the student side into `~/jupyter_ws/neoracer-os`: the `racecar-neo-library` and the NeoRacer labs, from the Neobotics forks. That library carries `rc.slam` / `rc.nav` and the FlySky auto-start fix. The FlySky RC has no START button, so `rc.go()` enters your program without one; the upstream MIT library waits for the Xbox START.

The `rc.*` API matches MIT's RACECAR Neo, so a lab written for either car runs on the other. Four capabilities have no hardware here and raise `NotImplementedError` naming the missing part rather than returning a placeholder: `rc.camera.get_depth_image()` (monocular webcam, no depth sensor), `rc.physics.get_battery_current()` (no current shunt), `rc.led.*` (no addressable strip), and `rc.display.set_matrix()` / `get_matrix()` / `set_matrix_intensity()` (the 8x8 panel takes text only; `show_text()` works). Topic-level detail is in [docs/architecture.md](docs/architecture.md#topic-contract).

```sh
racecar library --list             # valid folders in ~/jupyter_ws
racecar library --select <folder>  # choose which library is on the Python path
racecar library --status           # show the current selection
```

Refreshing the labs preserves student files.

## GPU stack

Phase 6 (`scripts/setup_ml.sh`, or `racecar setup ml` on its own) installs what the Orin needs to train and run vision models on-board:

| Package | Version | Source |
| --- | --- | --- |
| TensorRT | 10.3.0 | JetPack; verified, not installed |
| PyTorch | 2.8.0 | jetson-ai-lab `jp6/cu126` |
| torchvision | 0.23.0 | jetson-ai-lab `jp6/cu126` |
| Ultralytics | 8.4.x | PyPI |
| onnx, onnxslim | latest | PyPI |
| numpy | 1.26.x | PyPI, held below 2 |

PyTorch does **not** come from PyPI. Its aarch64 wheels are either CPU-only or built for datacenter GPUs (sbsa); neither drives the Orin's integrated GPU. The script reads the CUDA version off the running image and picks the matching jetson-ai-lab index (`jp6/cu126` for JetPack 6.2), so it follows a JetPack upgrade without edits.

Check the install:

```sh
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# True Orin
```

Deployment goes through TensorRT. Export once, then load the `.engine` the same way as the `.pt`:

```sh
racecar service stop        # the builder needs the memory the stack is holding
racecar compile             # exports what config/inference.yaml points at
```

`racecar compile` reads `model_path` and `imgsz` from [`config/inference.yaml`](config/inference.yaml), so the engine is built at the size it will be served at; an engine is fixed to its export size. Give it a filename (`racecar compile yolo26n.pt`, resolved in `models/`) or a path to override. `--imgsz=`, `--device=`, and `--no-half` override the rest; `--force` rebuilds over an existing engine and skips the stack-is-running guard. The engine lands in `models/` either way.

The equivalent by hand:

```sh
yolo export model=yolo26n.pt format=engine half=True imgsz=640 device=0
yolo predict model=yolo26n.engine source=bus.jpg
```

The export runs ONNX first and then a TensorRT builder pass. Budget the time: YOLO26n at 640 took 471.6 s (7m 52s) to build on a 25 W Orin Nano, and YOLO11n 8m 17s, both peaking near 2.6 GB. The resulting engine is tied to this board and this TensorRT version, so rebuild it after a JetPack upgrade rather than copying it between cars.

What the engine buys, measured on YOLO26n at 640:

| Backend | Inference | End-to-end, idle | `avg_inference_ms`, full stack |
| --- | --- | --- | --- |
| PyTorch fp32 | 35.5 ms | 39.6 ms (25.2 fps) | 50.1 ms |
| TensorRT fp16 | 12.7 ms | 19.5 ms (51.2 fps) | 20.9 ms |

Inference alone is 2.8x faster; end-to-end 2.0x, because letterboxing and NMS run on the CPU and dominate what TensorRT leaves behind. The last column is the node's own `/diagnostics` average with the whole teleop stack loaded, which is the number that matters in service.

FP16 costs nothing visible at this scale: the engine reproduces the fp32 boxes to within a pixel or two and scores them slightly higher.

Two things worth knowing before training on the car:

- The Orin Nano shares 8 GB between CPU and GPU. Training is viable for small models and short fine-tunes; anything larger belongs on a desktop, with only the exported engine copied over.
- `nvpmodel -q` reports the active power mode. The 25 W mode is the default; `sudo nvpmodel -m 0` unlocks MAXN for benchmarking, at the cost of thermals.

## Object detection

`inference_node` runs the frames from `/camera/color` through a YOLO model and publishes `vision_msgs/Detection2DArray` on `/edgetpu/inference`. Detection happens once, on the graph, so several consumers can read the same boxes instead of each holding its own copy of a model.

```sh
racecar launch inference                                   # on its own
racecar teleop                                             # with the rest of the stack (on by default)
racecar teleop inference_enable:=false                     # without it
ros2 topic echo /edgetpu/inference
```

It defaults to **on** in `teleop.launch.py`, so detections are on the graph without a launch argument. It is still the most expensive subsystem: the model holds GPU memory for the life of the stack. Launch with `inference_enable:=false` on a car whose student code never reads `/edgetpu/inference`, or to free the GPU for a `racecar compile` build.

The node exits at startup if `racecar setup ml` has never run on the car (it logs `ultralytics is not importable`). The rest of the stack comes up regardless; only the detection card goes red.

Each `Detection2D` carries a `bbox` in pixels (`center.position` and `size_x`/`size_y`, origin top-left) and one `results` entry with the class name and score. Class names come out of the weights, so a model trained on your own dataset publishes your own labels with no separate labels file:

```
bus     0.941  center=(399.9, 479.0)  size=(792.6 x 499.2)
person  0.889  center=(740.3, 636.9)  size=(138.9 x 483.8)
```

Settings live in `config/inference.yaml`:

| Parameter | Default | Notes |
| --- | --- | --- |
| `model_path` | `yolo11n.pt` | Absolute path, a filename in `models/`, or a stock Ultralytics name |
| `device` | `"0"` | CUDA index, or `"cpu"` |
| `imgsz` / `score_threshold` / `iou_threshold` | 640 / 0.5 / 0.45 | Model input size and NMS thresholds |
| `max_detections` | 10 | Cap per frame |
| `class_filter` | all | Class indices to keep, e.g. `[0, 2]`; omit the key for every class |
| `max_rate_hz` | 15.0 | Ceiling on inference rate |
| `publish_annotated` | false | Overlay on `/edgetpu/inference/image`, encoded only while subscribed |

Inference is slower than the 60 fps camera, so the subscription holds a single best-effort frame: a frame arriving while the model is busy replaces the one waiting behind it rather than queueing. Detections stay on the present instead of falling behind a backlog, at the cost of frames the model never sees. Measured on a 25 W Orin Nano with the teleop stack running, `yolo26n.pt` averages 50.1 ms per frame; the TensorRT engine cuts that to 20.9 ms, which clears the `max_rate_hz` ceiling of 15 Hz with headroom to spare.

Point `model_path` at an `.engine` to use it; both formats load through the same call. Building one is the export step in [GPU stack](#gpu-stack), and `models/README.md` has the recipe. The `.pt` weights are committed, so a fresh clone runs inference without a download. Engines are tied to the board and the TensorRT version that built them, so each car builds its own with `racecar compile` and `models/*.engine` stays gitignored.

Health lands on `/diagnostics` (latency, detection counts, frame staleness) and on the dashboard as a **YOLO inference** card.

## Checking lidar health

The lidar driver watches its own output. When a scan streams with (nearly) every range `inf` for 3 s, it logs `[scan-watchdog]` at ERROR and attaches the sensor's live state JSON (laser on/off, motor rpm, scan window). On any "the lidar looks dead" report, check this first:

```sh
journalctl -u neoracer-teleop -b | grep scan-watchdog
```

Empty output means the scan never went blind this boot. Add `-f` to watch live while driving; a recovery is logged at INFO with the total blind duration.

A bare `ros2 topic echo /scan` is not evidence of a dead lidar. `echo` truncates arrays to their first 128 values, which sit inside the always-blank rear crop, so a healthy scan prints a wall of `.inf`. Count the values, or use RViz with Fixed Frame `laser`. Full symptom table and the 60-second decision test: [docs/lidar_scan_health.md](docs/lidar_scan_health.md).

## Coexisting with the vendor workspace

Factory images ship the Seeed vendor stack preinstalled in `~/osracer_ws`, including its own copy of the `lakibeam1` package that has diverged from the pinned fork. Two copies of one package means the lidar you get depends on overlay order, and colcon may refuse to build ("underlay override").

Setup reconciles this to a single shared lidar: it masks the vendor copy with `COLCON_IGNORE`, symlinks `src/lakibeam1` into `~/osracer_ws/src`, and rebuilds it there. Both workspaces stay usable, one active per shell, and build from one source.

- Setup never edits existing `.bashrc` lines. It appends one marked block, `# Neoracer - default workspace`, which runs after the factory lines and resets the environment to the neoracer overlay. Nothing under `~/osracer_ws` is modified.
- `racecar ws osracer` switches the current shell to the vendor stack, `racecar ws neoracer` switches back, and a bare `racecar ws` shows which is active. The switch is per shell and leaves disk state alone.
- Restore the factory default by deleting that block from `~/.bashrc`; every factory line is still there, untouched.
- If the vendor stack autostarts on your image (`systemctl list-unit-files | grep -i osr`), stop it before running teleop; both stacks contend for the serial ports.

## Manual build

Without the shell tool:

```sh
cd ~/ros2_ws
colcon build --packages-select neoracer_ros2_driver --symlink-install
source install/setup.bash
```

## Launch

```sh
racecar teleop                  # or: ros2 launch neoracer_ros2_driver teleop.launch.py
racecar launch controller       # individual subsystems too
racecar launch lidar
racecar launch camera
racecar launch led_matrix
racecar launch inference
racecar launch autonomy
```

Any subsystem can be toggled at launch. `lidar`, `camera`, `led_matrix`, and `inference` all default to on:

```sh
ros2 launch neoracer_ros2_driver teleop.launch.py camera_enable:=false
ros2 launch neoracer_ros2_driver teleop.launch.py inference_enable:=false
```

The systemd service is the normal way to run the car: headless, started on every boot once enabled. `racecar teleop` runs the same stack in the foreground for interactive debugging. Startup logs stream, then it goes quiet and holds the terminal while the car is live; the quiet period is normal operation, not a hang. Ctrl+C stops it. Stop the service first, as the two cannot share the serial ports.

Configuration lives in `config/`, loaded by each node's launch file: `controller.yaml` (serial port, m/s mapping, FlySky channel map), `throttle.yaml` (speed and steering caps), `gamepad.yaml` and `mux.yaml` (axis indices, arming buttons), `twist_bridge.yaml`, `camera.yaml`, `led_matrix.yaml`, `inference.yaml` (weights, thresholds, inference rate).

## Troubleshooting

**`/dev/osrbot_*` symlink missing.** Re-run `racecar udev` and re-plug the device. Check IDs with `racecar status` or `udevadm info -n /dev/ttyACM0`.

**No `/scan`.** Confirm the lidar answers: `ping 192.168.8.2`. If it doesn't, the USB-C link should bring up an interface at `192.168.8.1`; see [Networking](#networking-optional). Then read [Checking lidar health](#checking-lidar-health) before concluding the sensor is dead.

**Car won't drive.** The mux only forwards commands when the FlySky switch arms manual or autonomy mode. Verify with `ros2 topic echo /joy`. Direction and the m/s mapping are tuned in `config/controller.yaml`.

**`/camera/color` won't decode.** The node publishes JPEG-in-`Image`. Confirm the webcam offers MJPG: `v4l2-ctl --list-formats-ext -d /dev/osrbot_usb_cam`.

**No `/edgetpu/inference`.** The node is on by default; confirm it was not disabled with `inference_enable:=false`. If it is running, `ros2 topic echo /diagnostics` reports what it is doing: `No images received yet` means `/camera/color` is dead, an `Inference failed` message names the model error. A node that exits at startup logs its one reason: `ultralytics is not importable` means phase 6 never ran (`racecar setup ml`), and a load failure names the weights path it tried.

**FlySky channels are wrong.** Channel order depends on your transmitter mixer. Start the controller, run `ros2 topic echo /joy` while moving each stick and switch, then set `throttle_channel` / `steering_channel` / `mode_channel` in `config/controller.yaml`. A no-signal channel (`-1`, transmitter off) maps to neutral so the car idles.

**JetPack dpkg errors.** Reset the dpkg info directory:

```sh
sudo mv /var/lib/dpkg/info/ /var/lib/dpkg/backup/
sudo mkdir /var/lib/dpkg/info/
sudo apt-get update && sudo apt-get -f install
sudo mv /var/lib/dpkg/backup/* /var/lib/dpkg/info/
sudo rm -rf /var/lib/dpkg/backup/
sudo apt update && sudo apt upgrade -y
```

## Changelog

See [CHANGELOG.md](./CHANGELOG.md).

## License

GPLv3. See [LICENSE](./LICENSE). Provided as-is, without express or implied warranty.

## Citation

If you use the NeoRacer ROS 2 driver in published research, please cite this repository. GitHub renders a "Cite this repository" button from [`CITATION.cff`](CITATION.cff) that produces BibTeX and APA automatically.

```bibtex
@software{neobotics_neoracer_ros2_driver_2026,
  author = {Lai, Chris and Bandyopadhyay, Koneshka},
  title  = {{NeoRacer ROS2 Driver}: Hardware Interface for the {NeoRacer V1} Autonomous Racing Platform},
  year   = {2026},
  url    = {https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver},
  note   = {Neobotics Foundation Inc.}
}
```
