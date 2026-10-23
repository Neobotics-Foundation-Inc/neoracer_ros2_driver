# neoracer_ros2_driver

ROS 2 driver for the NeoRacer V1, a 1:12-scale autonomous Ackermann-steering racing robot built by [Neobotics Foundation](https://github.com/Neobotics-Foundation-Inc) in collaboration with Seeed Studio.

## Contents

- [Hardware](#hardware)
- [Architecture](#architecture)
- [Quickstart (fresh JetPack 6.2 install)](#quickstart-fresh-jetpack-62-install)
- [The `racecar` shell tool](#the-racecar-shell-tool)
- [Networking (optional)](#networking-optional)
- [Car-to-car (V2V)](#car-to-car-v2v)
- [Localization and navigation (osracer)](#localization-and-navigation-osracer)
- [Autonomy: SLAM and Nav2](#autonomy-slam-and-nav2)
- [Web dashboard](#web-dashboard)
- [Lab dashboards](#lab-dashboards)
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
- `/v2v/peers` (std_msgs/String), `/v2v/carN/odom` (nav_msgs/Odometry), `/v2v/recv` (std_msgs/String): the other cars on the network, once this car is stamped; see [Car-to-car (V2V)](#car-to-car-v2v).

The control pipeline (`gamepad` → `mux` → `throttle` → `/motor`) enforces speed caps, arming, and manual/autonomy arbitration, so RC driving and autonomous code share one speed-limited path. A 3-position FlySky switch selects **idle / manual / autonomy**, mapped to the `/joy` buttons the mux reads. Every topic in the pipeline is normalized to `[-1, 1]`; top speed and steering limits live in `config/throttle.yaml`, and the normalized→m/s mapping in `config/controller.yaml`.

Safety and uptime layers:

- **Mux** gates commands behind the arming switch and zeroes output when the RC link drops.
- **Watchdog** (`scripts/watchdog.py`) supervises the control and sensor nodes, restarts a dead node, and monitors `/imu/fused` and `/scan` freshness.
- **Four systemd units** (`neoracer-{teleop,watchdog,dashboard,jupyter}.service`) wired with `BindsTo=`/`Wants=` so the watchdog follows teleop up and down. `neoracer-autonomy.service` is held and not installed; see [Autonomy: SLAM and Nav2](#autonomy-slam-and-nav2).
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
git clone -b neoracer_ros2_driver_purdue https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver.git
bash neoracer_ros2_driver/scripts/setup_all.sh
```

`setup_all.sh` is idempotent. Each phase checks for existing state and skips when already applied, so re-running is safe.

A car already set up from `main` moves to this branch once (this discards local edits in the driver repo):

```sh
racecar cd
git fetch origin && git checkout -f neoracer_ros2_driver_purdue
source scripts/racecar-tool.sh   # this branch's racecar tool in this shell
racecar update                   # full setup and restart; stays on this branch from now on
```

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
8. **`setup_services.sh`**: installs the systemd units; core stack enabled, lab dashboards cloned into `scripts/dashboards/` and left disabled

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

racecar service status      # active/enabled snapshot for every unit
racecar service install     # install units: core enabled, dashboards disabled
racecar service start       # no name: the core stack
racecar service stop        # no name: the core stack
racecar service restart     # every enabled unit; never starts a disabled one
racecar service start camlabel   # a lab dashboard, for this session
racecar service logs teleop # journalctl -u neoracer-teleop -f

racecar v2v id 1            # stamp this car as car 1: identity + ROS domain + v2v service
racecar v2v status          # the other cars on the network
racecar v2v ping 2          # round trip to car 2
racecar v2v send slow down  # one message to every car

racecar mapping             # SLAM to build a map (slam_toolbox default)
racecar mapping save <name> # save the current map
racecar navigation          # Nav2 on a saved map (teb default)
racecar navigation rviz     # goal-setting RViz view (car desktop only)

racecar setup all           # the 7-phase orchestrator
racecar setup networking    # WiFi AP + Ethernet + lidar subnet
racecar update              # repo to the latest of its branch + full setup + service restart

racecar ws [neoracer|osracer]   # switch this shell between workspaces
racecar mode [osracer|neoracer] # which stack owns the ESP32 and the lidar, across reboots
racecar library --list          # student library folders in ~/jupyter_ws
racecar library --select <dir>  # point the student library at a notebook tree
racecar selftest --led[=TEXT]   # send test text to the 8x8 display
racecar clear --led             # blank the display until something writes to it
racecar udev                    # re-install the udev rules
racecar status                  # peripherals + symlinks + running nodes
racecar cleanup [--force]       # list / kill stale processes + SHM orphans
racecar help                    # full usage
```

`racecar update` resets the repo to the latest commit of the branch it is on (`origin/neoracer_ros2_driver_purdue` here, `origin/main` on a stock car), discarding local edits, runs the full setup, and restarts every enabled service. A disabled lab dashboard stays down across an update. Requires internet.

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

## Car-to-car (V2V)

Every car sends its state to every other car ten times a second: pose in its own odom frame, speed, yaw rate, commanded speed and steering, battery. Cars also exchange text messages that arrive exactly once and in order. It runs as JSON on UDP 4747: a 1 Hz broadcast finds the cars and everything else goes unicast, which Wi-Fi delivers without loss. Each car runs its own ROS domain, so one car's `/drive` can never reach another.

Once per car, with every car on one network:

```sh
sudo nmcli dev wifi connect <ssid> password <psk>   # or Ethernet
racecar v2v id 1          # unique per car (2, 3, ...): sets the ROS domain, starts neoracer-v2v
```

Use it:

```sh
racecar v2v status            # other cars: age, loss, pose, speed, battery
racecar v2v ping 2            # round trip to car 2
racecar v2v send slow down    # to every car; prints delivered or NOT delivered per car
racecar v2v recv              # messages as they arrive
```

| Topic | Use |
| --- | --- |
| `/v2v/peers` | every other car as JSON, 10 Hz |
| `/v2v/carN/odom` | car N's pose (nav_msgs/Odometry, frame `carN/odom`) |
| `/v2v/carN/map_pose` | car N's pose on the map while it is localized (geometry_msgs/PoseStamped, frame `map`) |
| `/v2v/send`, `/v2v/recv` | send text (std_msgs/String); messages and delivery reports come back on `recv` |
| `/v2v/ping` | a car id to ping (std_msgs/Int32) |

Signing is optional: `racecar v2v key new` on one car, then `racecar v2v key set <hex>` on the others, and cars reject unsigned, forged and replayed packets. A laptop on the network joins as a virtual car with `python3 scripts/v2v_peer.py --as-car 9 --broadcast <subnet broadcast>`, or only listens with `--watch 90`. Positions are each car's own odometry, plus its pose on the map while it is localized (AMCL or slam_toolbox publishing `map -> odom`, step 3 of [Localization and navigation](#localization-and-navigation-osracer)). Cars localized on the same saved map share its frame: copy one car's map to the others before they localize.

### Watching the cars from a computer

`scripts/v2v_view.py` runs on any computer on the cars' network and reads their V2V state:

```sh
python3 scripts/v2v_view.py --broadcast 10.0.0.255                 # bird's-eye view at http://localhost:8470
python3 scripts/v2v_view.py --broadcast 10.0.0.255 --autodrive 2   # and car 2 in AutoDRIVE

scp racecar@<car>:maps/room.yaml racecar@<car>:maps/room.pgm .     # the map the cars localize on
python3 scripts/v2v_view.py --broadcast 10.0.0.255 --map room.yaml # the cars on that map
```

- The bird's-eye view draws every car to scale as the Playground's top-down NeoRacer, its rear wing and front bumper in the car's own colour, with its heading and the path it has driven, and lists speed, battery and how fresh its data is.
- With `--map`, the saved map is drawn under the cars (walls light, unexplored space dark), and a car localized on it (Frame `map` in the table) is drawn where it is on it. A car that is not (Frame `odom`) is drawn by its odometry from where its driver started; `--place 2:1.5,0,90` puts car 2's start at x 1.5 m, y 0, facing 90 degrees. When a car localizes, its path starts over in the map frame.
- `--autodrive CAR` also drives the NeoRacer in the AutoDRIVE simulator on the same computer: open the NeoRacer scene, press Connect, and the simulated car moves as the real one does, by its map pose while it is localized (it starts where it stands; `--at X Y HEADING` puts it elsewhere; when the car localizes or loses its map, the simulated car carries on from where it stands). It uses AutoDRIVE's co-simulation input, so the simulated car follows the real car's pose rather than its own physics; the scene's Bird's Eye camera follows it from above. It needs AutoDRIVE's devkit Python stack (Python 3.8, python-socketio 4.2.0, python-engineio 3.13.0, eventlet 0.33.3), and three changes to upstream's `NeoRacer - AuE-X930` scene (as of `75b7df52`), all on the `purdue-cosim` branch of [Neobotics-Foundation-Inc/AutoDRIVE](https://github.com/Neobotics-Foundation-Inc/AutoDRIVE/tree/purdue-cosim) (Unity 2022.3.52f1; run `Tools/unzip-and-clean.sh` after cloning, as upstream does): the socket lists the NeoRacer's co-simulation component (upstream leaves the list empty, so the pose is ignored), the posed car is kinematic (otherwise it falls and springs between poses, and respawns at a checkpoint wherever its pose crosses a wall), and its wheels roll and steer from its motion (`CoSimWheels`).
- `--autodrive 1 2` drives two cars at once, AutoDRIVE's vehicles 1 and 2, in a two-car scene (`NeoRacer - AuE-X930 - Two Cars`, or the testbed's below). Cars localized on the same map share it in the simulator too: the first pins the map where its simulated car stands, and the other goes where the map puts it.
- One program per computer can listen for V2V: run this or `v2v_peer.py`, not both.

### Cars locked onto a taped lane track

On a taped track the cars can be drawn on their lanes from their odometry alone, with no localization running on them:

```sh
python3 scripts/v2v_view.py --broadcast 10.0.0.255 --lanes scripts/lanes/purdue_testbed.json
python3 scripts/v2v_view.py --broadcast 10.0.0.255 --lanes scripts/lanes/purdue_testbed.json --autodrive 1 2
```

- A car's odometry drifts (the gyro's bias wanders, so the heading slowly turns, and the position with it) and shakes, but the distance it has driven and how its heading changes over a metre or two stay good, and on a known lane map they say where it is. The lane lock (`neoracer_ros2_driver/lane_lock.py`) is a particle filter over the lanes: each guess is a lane, how far along it, the gyro's bias and the odometry-to-map scale; as the car drives, each moves along its lane (at a junction, one of the ways on) and is weighed by how well the car's heading, less its bias, matches its lane there. Wrong guesses stop fitting at the next bend or turn. The car is always drawn on its lane centre, so drift and vibration cannot put it in the wrong lane or off the road.
- Without a known start the lock finds the car as it drives, typically within 10 m (it locks once the car has turned 180 degrees and its guess has fit for a metre or two; until then the car is drawn faded, "finding", and stays still in AutoDRIVE). `--start 2:loop2_fwd:3.5` gives car 2's start (lane, metres along it). A lock that stops fitting (a car picked up and put down elsewhere) is dropped within a few metres and the car found again.
- `--lane-scale` is the map's metres per metre of the cars' odometry (1 when the map is measured true); the lock learns the true ratio within 0.8 to 1.25 of it and the table shows it.
- The page draws the track itself (mat, white tape, dashed yellow centre lines, red stop lines). In AutoDRIVE, the testbed scene (`NeoRacer - Purdue Testbed`, or `- Two Cars`) is built from the same map in its own frame, so a locked car is placed on its lane as it is, on the `purdue-cosim` branch (menu AutoDRIVE > NeoRacer: Build Purdue Testbed Scene, or `-executeMethod NeoRacerTestbed.Build`).

The Purdue testbed's map was traced from one photo of the track (`scripts/lanes/trace_track_photo.py`): the mat straightened from its corners and tiles into a top view, the tape picked out by colour, the dashed yellow lines linked into each road's centre, two lanes per road (right-hand traffic, half a lane to the right of the centre), lanes ending at their stop lines, and every lane joined to every lane on through a junction but its own road's other way. Its shape and proportions are measured from the photo; its scale is not: the mat is taken as 45 ft on its longer side until the track is measured, and every length scales with it (`--mat-length`). Measured, re-run the tracer with the true size.

`scripts/lanes/sim_cars.py` sends simulated cars over V2V (odometry with gyro drift, shake, weave and encoder error) to try this without cars; `scripts/lanes/bench_lane_lock.py` scores the lock over many simulated drives:

| Simulated drive (6 seeds each, worst seed) | On the right lane | Along the lane, 95% within |
|---|---|---|
| Anticlockwise laps, known start | 100% | 0.3 m |
| Clockwise laps, known start | 100% | 0.4 m |
| Through both junctions, every turn | 99.9% | 0.3 m |
| Encoder reading 15% long | 100% | 0.3 m |
| Unknown start | found within 10 m, then 100% | 0.8 m |

Strong gyro drift (15 degrees a minute) with heavy weaving (12 cm either side) is past it: the car stays on its lane 95% of the time, but the scale it learns wanders and it can be drawn metres along the lane from where it is.

## Localization and navigation (osracer)

Mapping, localization and Nav2 run on the vendor stack, [osrbot/osracer](https://github.com/osrbot/osracer), in `~/osracer_ws`. Keep its checkout at commit `c70eb50`: upstream `main` launches its chassis from an `osracer_base` package the repository does not ship. osracer drives the same ESP32 and lidar as this driver, so switch the car first:

```sh
racecar mode osracer     # teleop off, ESP32 and lidar free, every terminal on ~/osracer_ws; survives reboots
racecar mode neoracer    # back to this driver
racecar mode             # current mode and who holds the ESP32 and the lidar
```

V2V keeps running in both modes. osracer drives the motors directly, outside the FlySky arming switch, so try navigation with the wheels off the ground first.

```sh
# 1. Base: chassis, TF, lidar (the lidar takes about 30 s)
ros2 launch osracer_bringup bringup.launch.py

# 2. Map: push or drive the car around the room and finish where you started
ros2 launch osracer_slam slam_toolbox.launch.py
ros2 launch osracer_debug debug_mapping.launch.py     # RViz, on the car's desktop (RustDesk or a monitor)
mkdir -p ~/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: /home/racecar/maps/room}}"

# 3. Localize: stop slam_toolbox, put the car back at the start
ros2 launch osracer_navigation nav2.launch.py use_planner:=teb map:=/home/racecar/maps/room.yaml use_rviz:=False
ros2 topic pub -t 3 -r 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
ros2 topic echo /amcl_pose --once

# 4. Navigate: 1 m ahead of the start
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
```

- Over SSH keep `use_rviz:=False`: RViz cannot open a window there, and its exit stops the whole launch. At the car's desktop leave it on and use RViz's 2D Pose Estimate and Nav2 Goal instead of the two `topic pub` lines.
- osracer's README writes `use_map:=/path/to/map.yaml`, but `use_map` takes a map name. Pass a path with `map:=`.
- "Timed out waiting for transform from base_link to map" repeats until AMCL has an initial pose.
- Mapping and navigation in one launch: `ros2 launch osracer_navigation bringup_launch.py slam:=True planner:=teb`.
- osracer's scripts in `~/osracer_ws/src/osracer/osracer_demo/scripts/` (`start_mapping_demo.sh`, `start_navigation_demo.sh`, `save_map_demo.sh`, `stop_all_demo.sh`) wrap the same steps.

## Autonomy: SLAM and Nav2

For working mapping, localization and Nav2 today, see [Localization and navigation (osracer)](#localization-and-navigation-osracer). SLAM and navigation run on demand, not as services. The autonomy base (`autonomy.launch.py`) runs only TF, the EKF, and the twist bridge.

`neoracer-autonomy.service` is **held**: the unit file stays in `scripts/`, but setup does not install it, and a setup run removes it from a car that has it from an earlier release. The service does not work yet and development is paused until further notice. Start the base by hand while it is held:

```sh
bash ~/ros2_ws/src/neoracer_ros2_driver/scripts/launch_autonomy.sh
```

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

## Lab dashboards

Seven lab tools ship as repositories of their own and are cloned into `scripts/dashboards/` by phase 8, each tracking its default branch:

| Dashboard | Port | Repository | Purpose |
| --- | --- | --- | --- |
| camlabel | 8082 | [camlabel_dashboard](https://github.com/Neobotics-Foundation-Inc/camlabel_dashboard) | capture frames and drag boxes; writes a YOLO dataset to `~/data/camlabel` |
| wallfollow | 8081 | [wallfollow_dashboard](https://github.com/Neobotics-Foundation-Inc/wallfollow_dashboard) | wall following on `/scan` with live PD tuning |
| pursuit | 8083 | [pursuit_dashboard](https://github.com/Neobotics-Foundation-Inc/pursuit_dashboard) | target following on `/edgetpu/inference` |
| eps | 8084 | [eps_dashboard](https://github.com/Neobotics-Foundation-Inc/eps_dashboard) | episodic policy search; a (1+1)-ES tunes the gap follower's gains from one success or fail press per run |
| smartfollow | 8085 | [smartfollow_dashboard](https://github.com/Neobotics-Foundation-Inc/smartfollow_dashboard) | wallfollow and pursuit combined; follows a detected car, falls back to wall following |
| linefollow | 8086 | [linefollow_dashboard](https://github.com/Neobotics-Foundation-Inc/linefollow_dashboard) | line following on `/camera/color`; HSV threshold, crop band, and PD steering tuned from trackbars, with a mask view and a stability trace |
| webteleop | 8087 | [teleop_dashboard](https://github.com/Neobotics-Foundation-Inc/teleop_dashboard) | manual driving from the browser: WASD, arrow keys, or buttons, two at once; camera, lidar, and encoder views; a command timeout drives zero when the browser goes quiet |

Setup installs each unit but leaves it disabled. A dashboard holds the camera or the GPU for its whole run, and wallfollow, pursuit, eps, smartfollow, linefollow, and webteleop all publish `/drive`, so they are started one at a time for a session:

```sh
racecar service start camlabel     # on for this session
racecar service stop camlabel
racecar service enable camlabel    # only if this car should boot into it
```

`racecar service restart` restarts what is enabled and never promotes a disabled dashboard into a running one, so a field update leaves the car's lab selection alone.

The checkouts are gitignored rather than vendored: a car's tuned `wallfollow.yaml`, `pursuit.yaml`, `eps.yaml`, `smartfollow.yaml`, `linefollow.yaml`, or `teleop.yaml` lives there, so setup fast-forwards them and reports a checkout it cannot advance instead of resetting over the tuning. Each repository's own README documents its dashboard.

## Jupyter notebooks and the student library

`http://neoracer.local:8888` serves JupyterLab from `~/jupyter_ws`, with `import rclpy` preconfigured.

Setup also installs the student side into `~/jupyter_ws/neoracer-os`: the `racecar-neo-library` and the NeoRacer labs, from the Neobotics forks. That library carries `rc.slam` / `rc.nav` and the FlySky auto-start fix. The FlySky RC has no START button, so `rc.go()` enters your program without one; the upstream MIT library waits for the Xbox START.

The `rc.*` API matches MIT's RACECAR Neo, so a lab written for either car runs on the other. Four capabilities have no hardware here and raise `NotImplementedError` naming the missing part rather than returning a placeholder: `rc.camera.get_depth_image()` (monocular webcam, no depth sensor), `rc.physics.get_battery_current()` (no current shunt), `rc.led.*` (no addressable strip), and `rc.display.set_matrix()` / `get_matrix()` / `set_matrix_intensity()` (the 8x8 panel takes text only; `show_text()` and `clear()` work). Topic-level detail is in [docs/architecture.md](docs/architecture.md#topic-contract).

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

**A lab dashboard won't come up.** It is disabled on purpose; start it with `racecar service start camlabel`. `racecar service status` shows `not installed` if phase 8 never cloned it, which happens on a car that had no internet during setup: re-run `racecar service install`. A dashboard that starts and dies is usually a port already held by a sibling dashboard, so stop the other one; `racecar service logs camlabel` names the bind failure.

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
