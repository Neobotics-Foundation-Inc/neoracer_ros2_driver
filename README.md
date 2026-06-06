# neoracer_ros2_driver

Backend ROS2 driver for the **Neoracer** (1/12-scale autonomous racing car) with
support for the OSRbot software stack, in collaboration with Seeed Studio.

It bridges the car's hardware to ROS2 and publishes the topic contract the
[Neobotics student library](https://github.com/MITRacecarNeo/racecar-neo-installer)
consumes (`/camera`, `/scan`, `/imu`, `/joy`, `/drive`, `/led_matrix/command`).
The repo also ships the full fleet toolchain: a one-shot setup script, the
`racecar` shell tool, a node watchdog, a web dashboard, JupyterLab, systemd
services, and networking.

## Changelog

**0.1.0** - migrated the MIT RACECAR Neo driver to the Neoracer (Jetson Orin
Nano, ROS Humble): ESP32 bridge (IMU/odom/FlySky joy/ESC), Lakibeam lidar,
USB-UART display, and a retargeted fleet toolchain. Full history in
[docs/changelog.md](docs/changelog.md); design in
[docs/architecture.md](docs/architecture.md).

---

## Table of Contents
1. [System Info](#1--system-info)
2. [Hardware & Topics](#2--hardware--topics)
3. [Architecture](#3--architecture)
4. [Installation](#4--installation)
5. [Usage](#5--usage)
6. [Networking](#6--networking)
7. [Configuration & Tuning](#7--configuration--tuning)
8. [Troubleshooting](#8--troubleshooting)
9. [Licensing](#9--licensing)
10. [Citation](#10--citation)

---

## 1 | System Info

The compute on the Neoracer is the **Jetson Orin Nano**, running JetPack 6.2.
To reflash, follow Seeed's [JetPack flashing guide](https://wiki.seeedstudio.com/reComputer_J4012_Flash_Jetpack/).
- **OS Version**: Ubuntu 22.04.5 LTS (codename: jammy)
- **Python Version**: 3.10.12
- **ROS2 Version**: Humble

Standard naming conventions across Neoracer systems (the password can change per
deployment):
- **Username**: racecar
- **Hostname**: neoracer
- **IP**: 192.168.1.[100 + Car ID]
- **Password**: neobotics
- **SSID**: neoracer-[Car ID]

---

## 2 | Hardware & Topics

| Subsystem | Hardware | Connection | udev symlink | ROS topic(s) |
|-----------|----------|-----------|--------------|--------------|
| Drive / IMU / odometry / RC | Seeed OSRbot **ESP32** | USB-CDC | `/dev/osrbot_base` | sub `/motor`; pub `/imu`, `/odom`, `/joy` |
| 2D LIDAR | **Lakibeam** | Ethernet/USB-C (UDP) | - (`192.168.8.2`) | `/scan` |
| Forward camera | USB webcam (MJPG) | USB | `/dev/osrbot_usb_cam` | `/camera` |
| Display | 8x8 dot matrix | USB-UART | `/dev/osrbot_led_matrix` | sub `/led_matrix/command` |
| Manual control | **FlySky** RC | via ESP32 | - | `/joy` |

The ESP32 carries the IMU, wheel odometry, the FlySky RC receiver, and the motor
ESC. The single `controller` node owns its serial link, so there is no separate
IMU/PWM/joystick driver. The camera publishes `/camera` as a `sensor_msgs/Image`
whose `data` is the raw JPEG byte stream (`encoding: jpeg`), which the student
library decodes with `cv2.imdecode`.

> EdgeTPU / Coral object detection from the upstream RACECAR Neo driver is not
> included - the Neoracer has no Coral accelerator. It can be restored from the
> upstream driver if one is ever added.

---

## 3 | Architecture

```
FlySky RC -(USB)-> controller --/joy---> gamepad_node --/gamepad_drive--+
                     |  ^                                              +-> mux_node --/mux_out---> throttle_node --/motor--+
                     |  |                  student library --/drive----+   (FlySky switch: manual vs autonomy)          |
                     |  +-------------------------- /motor <-------------------------------------------------------------+
                     +--/imu (sensor_msgs/Imu)        controller writes "v <m/s> <deg>" to the ESP32
                     +--/odom (nav_msgs/Odometry)
  Lakibeam -(UDP)-> lakibeam1_scan_node --/scan
  USB cam  ------> camera --/camera (JPEG-in-Image)
  student library --/led_matrix/command---> led_matrix -(USB-UART)-> 8x8 display
```

The control pipeline (`gamepad` -> `mux` -> `throttle`) enforces speed caps,
arming, and manual/autonomy arbitration. A 3-position FlySky switch selects
**idle / manual / autonomy** (mapped to the `/joy` LB/RB buttons the mux reads).
All values on every topic are normalized to `[-1, 1]`; the single tuning surface
for top speed/steer limits is `config/throttle.yaml`, and the ESP32 m/s mapping
is in `config/controller.yaml`.

Nodes & launch files: `controller`, `gamepad_node`, `mux_node`, `throttle_node`,
`camera`, `led_matrix`, plus the `lakibeam1` lidar (a sibling package in `src/`). `teleop.launch.py`
brings up the whole stack; each subsystem also has its own `<name>.launch.py`
(used by the watchdog for targeted restarts).

---

## 4 | Installation

The whole machine is provisioned by one idempotent orchestrator:

```sh
cd ~/ros2_ws/src
git clone https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver.git
bash neoracer_ros2_driver/scripts/setup_all.sh
```

`setup_all.sh` runs 7 idempotent phases (re-runs skip completed work):
1. **ROS2 Humble** + driver dependencies (`setup_ros2.sh`)
2. Robotics dev tools (`setup_dev_tools.sh`)
3. User environment - groups, `.bashrc`, `racecar` tool (`setup_user_env.sh`)
4. udev rules - stable `/dev/osrbot_*` symlinks (`setup_udev.sh`)
5. Workspace build - clones the **Lakibeam** lidar driver into `src/` and
   `colcon build`s the whole workspace (`setup_workspace.sh`)
6. JupyterLab + workspace (`setup_jupyter.sh`)
7. systemd services (`setup_services.sh`)

Networking is configured separately (it reconfigures Wi-Fi - see section 6).

The Lakibeam lidar driver (package `lakibeam1`) is a separate upstream project,
not vendored here; `setup_workspace.sh` clones it into `src/` alongside this
package. Override the source with `LIDAR_REPO=...`.

After install, log out/in (or `newgrp dialout`) so group changes apply.

---

## 5 | Usage

Bring up the full stack:

```sh
racecar teleop                 # logs to ~/logs/<timestamp>/, sweeps stale SHM
# or
ros2 launch neoracer_ros2_driver teleop.launch.py
# disable a subsystem:
ros2 launch neoracer_ros2_driver teleop.launch.py camera_enable:=false
```

The `racecar` shell tool (sourced into `~/.bashrc`):

| Command | Purpose |
|---------|---------|
| `racecar build` / `test` / `source` | colcon build / test / source the overlay |
| `racecar teleop` | full stack via the logging wrapper |
| `racecar launch <name>` | `ros2 launch neoracer_ros2_driver <name>.launch.py` |
| `racecar status` | USB peripherals, `/dev/osrbot_*` symlinks, running nodes |
| `racecar watchdog` | run the node supervisor in the foreground |
| `racecar service <action>` | install/start/stop/status the systemd units |
| `racecar setup <all\|networking>` | run a provisioning phase |
| `racecar library --select <folder>` | point the student library at a notebook tree |
| `racecar selftest --led[=TEXT]` | send test text to the 8x8 display |
| `racecar cleanup [--force]` | list/kill orphaned processes + FastRTPS SHM |

**Services** (auto-start on boot once installed via `racecar service install`):
`racecar-teleop`, `racecar-watchdog`, `racecar-dashboard` (web status, port
**8080**), `racecar-jupyter` (JupyterLab, port **8888**).

```sh
racecar service start              # starts teleop; watchdog follows
# web dashboard:   http://neoracer.local:8080
# jupyter:         http://192.168.1.101:8888
```

The **watchdog** supervises `controller`, `throttle`, `mux`, `gamepad`, `lidar`,
`camera` - restarting a dead node and monitoring `/imu` and `/scan` freshness.
The **dashboard** shows per-node liveness, topic rates, and Jetson temperature.

---

## 6 | Networking

```sh
racecar setup networking          # or: bash scripts/setup_networking.sh
```

Sets up an isolated Wi-Fi access point (so a laptop can reach the car headless),
an Ethernet static+DHCP address, and verifies the Lakibeam lidar subnet. Defaults
(override with flags or env vars; persisted to `~/.config/racecar/networking.env`):
- AP SSID `neoracer-1`, password `neobotics`, channel 6, `10.42.0.1/24`
- Ethernet static `192.168.1.101/24` (+ DHCP)
- Lidar host `192.168.8.1/24`, sensor `192.168.8.2`
- Interfaces default to `wlP1p1s0` (Wi-Fi) and `enP8p1s0` (Ethernet) - override
  with `--wifi-iface` / `--eth-iface` if your board enumerates differently.

> WARNING: this reconfigures Wi-Fi. Run it from a wired session or the console.

---

## 7 | Configuration & Tuning

YAML configs live in `config/` and load via each node's launch file.

- **`config/controller.yaml`** - ESP32 serial port, the normalized->m/s drive
  mapping (`max_speed_mps`, `steering_trim_deg`), and the **FlySky RC -> `/joy`
  mapping**. The exact FlySky channel order depends on your transmitter mixer:
  start the controller, run `ros2 topic echo /joy` while moving each stick/switch,
  and set `throttle_channel` / `steering_channel` / `mode_channel` accordingly.
  A no-signal channel (`-1`, transmitter off) maps to neutral so the car idles.
- **`config/throttle.yaml`** - single source of truth for top speed/steer caps.
- **`config/gamepad.yaml`**, **`config/mux.yaml`** - axis indices and the
  manual/autonomy enable buttons.
- **`launch/lidar.launch.py`** - Lakibeam `sensorip` (default `192.168.8.2`,
  USB-C mode), scan range, and frequency.

---

## 8 | Troubleshooting

**`/dev/osrbot_*` symlink missing** - re-run `racecar udev` and re-plug the
device. Check IDs with `racecar status` / `udevadm info -n /dev/ttyACM0`.

**No `/scan`** - confirm the lidar answers: `ping 192.168.8.2`. If not, see section 6
(the USB-C link should bring up a `usb*` interface at `192.168.8.1`).

**Car won't drive** - the mux only forwards commands when a FlySky switch arms
manual/autonomy mode (`/joy` buttons 4/5). Verify with `ros2 topic echo /joy`.
The ESP32 m/s mapping and direction are tuned in `config/controller.yaml`.

**`/camera` won't decode** - the node publishes JPEG-in-`Image`; confirm the
webcam offers MJPG (`v4l2-ctl --list-formats-ext -d /dev/osrbot_usb_cam`).

**JetPack/dpkg errors** - reset the dpkg info dir:

```sh
sudo mv /var/lib/dpkg/info/ /var/lib/dpkg/backup/
sudo mkdir /var/lib/dpkg/info/
sudo apt-get update && sudo apt-get -f install
sudo mv /var/lib/dpkg/backup/* /var/lib/dpkg/info/
sudo rm -rf /var/lib/dpkg/backup/
sudo apt update && sudo apt upgrade -y
```

---

## 9 | Licensing

All files in this repository are licensed under **GPLv3** (GNU GENERAL PUBLIC
LICENSE Version 3).

Source Location: https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver

Warranty disclaimer: provided **as-is**, without any express or implied warranty.

---

## 10 | Citation

If you use the **NeoRacer ROS2 driver** in published research, please cite this
repository. GitHub renders a "Cite this repository" button from the
[`CITATION.cff`](CITATION.cff) at the repo root that produces BibTeX/APA
automatically.

BibTeX:

```bibtex
@software{neobotics_neoracer_ros2_driver_2026,
  author       = {Lai, Chris and Bandyopadhyay, Koneshka},
  title        = {{NeoRacer ROS2 Driver}: Hardware Interface for the {NeoRacer V1} Autonomous Racing Platform},
  year         = {2026},
  url          = {https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver},
  note         = {Neobotics Foundation Inc.}
}
```
