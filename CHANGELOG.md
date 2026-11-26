# Changelog

All notable changes to this project. Format: Keep a Changelog
(keepachangelog.com). Versioning: the number follows the upstream release the driver ports.

## [Unreleased]

## [0.9.0] - 2026-11-19

This release v0.9.0 adds mapping and localization, guided calibration, a whole-car status check and bag recording to the NeoRacer. The 8080 page becomes an apps page, the seven lab dashboards now ship in the driver, and setup installs all of it.

The version jumps from 0.6.1 to 0.9.0. From this release, the version number follows the upstream release the driver ports. A NeoRacer-only fix between two ports gets a `v0.9.0-neo.N` tag and a line under the current version, and the package version stays the same.

### Added
- Mapping and localization. `autonomy_node` reports `/autonomy/status` and `/autonomy/pose` and serves `/autonomy/save_map`, `/autonomy/autosave` and `/autonomy/global_localization`. In localization, it counts the car as localized from the pose spread and how well the scan fits the map. `racecar launch autonomy` runs it with slam_toolbox for mapping, or with AMCL and the map server, or the slam_toolbox localizer, for localization on a saved map, configured for the LakiBeam1 from 0.12 to 15 m.
- `neoracer-autonomy.service` runs the autonomy launch beside teleop from `~/.config/racecar/autonomy.conf`. It autosaves the map before it stops, and after a crash a mapping run carries on from its last autosave. It starts on demand, never at boot.
- `racecar map` with start, save, stop, resume, load, recover, status, list, info and foxglove. A mapping start or resume runs only with the handset's speed switch UP (slow). Starting, loading or resuming stops the lab dashboards that publish `/drive`.
- The map viewer on port 8088 (`neoracer-mapview.service`). It shows the map, the car, its trail and the live scan, and its buttons run the `racecar map` start, save, stop, load and recover commands, Set pose and Find me. It publishes no `/drive`, and it holds no ROS subscriptions while no browser has it open.
- While mapping, the throttle node caps the commanded speed at `mapping_speed_cap` (0.083, about 0.5 m/s), and the inference node stops reading the camera. The model stays loaded but runs on no frames, so slam_toolbox gets the CPU time.
- `foxglove:=true` on the autonomy launch starts a Foxglove bridge on port 8765. Upstream's Lichtblick layout ships as `config/foxglove_layout.json`, to import in Lichtblick. Apps that connect to it can watch but cannot publish.
- SLAM validation tools in `scripts/validation/`. `slam_session.py` runs and records a mapping or localization session and writes a report under the data root, and `replay` runs a recording again with other settings. The report measures map to odom drift from the car's first move.
- `racecar calibrate`, a guided session for the lidar yaw, odometry, steering trim and, when named, dynamics, with `status` and `check`. Results go to the car profile. The steering trim goes to the ESP32 board, which also applies it to manual driving. The steering and dynamics steps, and the lidar step's driving method, drive the car themselves. Each starts only after the mode switch goes UP and then DOWN, and it stops when the switch goes UP. A person drives the odometry step with the handset, mode switch UP.
- `scripts/analyze_odom.py` grades an odometry run from a recording: a straight run, a circle back to a mark, or laps.
- `/odom/integrated` at 50 Hz, with `odometry.source` choosing the ESP32's own pose (the default) or upstream's integrator. The controller publishes `odom -> base_link`.
- A static frame tree for the NeoRacer, with `base_link` as the base frame. The positions come from the CAD, the heights add a lift measured with tape, and the IMU's position is an estimate. Teleop starts `robot_state_publisher`, which also publishes `/robot_description`, so the lidar, IMU and camera frames are on TF whenever teleop runs. The lidar's yaw comes from `laser_yaw_deg`.
- `/battery/state`, the battery topic the student library reads, with current reported as NaN. `/rc/link`, true while the ESP32 receives the handset's channels.
- `imu.accel_gyro_axis_order`, `imu.accel_gyro_axis_sign`, `imu.mag_axis_order` and `imu.mag_axis_sign` remap `/imu/fused` and `/mag` for a board mounted another way. The shipped values change nothing.
- The controller's `board.steering_trim_deg` parameter reads the ESP32's steering trim at start and sets it on a change. A change is accepted only after the `/motor` speed has been 0 for 1 s.
- `racecar status`, a whole-car check of the devices, sensors, actuators, autonomy, Jetson, services and network. It replaces the lists of USB devices, device links and ROS nodes the command printed before. It reads OK, WARN, FAIL or SKIP for each check and exits 1 when a check fails. With `--strict` it also exits 1 on a WARN.
- `racecar log`, which records every topic as MCAP under the data root and keeps recording after the SSH session ends. It also lists and analyzes recordings.
- The car profile at `~/.config/racecar`. It holds this car's overrides of `config/`, the lab dashboards' tuning and the calibration, with export and import between cars. When a calibration or a dashboard Save replaces a file in the profile, the old copy goes under `history/`. `racecar profile` manages it, and setup moves old `config/*.local.yaml` files into it.
- The data root `~/neoracer_data` for bags, images, datasets, maps, validation sessions and dashboard logs. It sits outside the folders the installer removes, programs find it through `RACECAR_DATA_DIR`, and `racecar data` prints it. `~/jupyter_ws/data` and `~/jupyter_ws/maps` link into it.
- The 8080 apps page. Its tiles open JupyterLab, the map viewer and the seven lab dashboards, and starting one driving app stops the others after a question. It switches the camera and detections, resumes or stops the last autonomy run, and shows the car's status. Turning the camera or detections on or off restarts teleop, and a running mapping or localization run stops first and resumes after the restart, unless a driving app is open by then. System Health at `/health` shows the Jetson's readings and live sensor panels.
- The seven lab dashboards ship in `scripts/dashboards/`: Wall Follow (8081), Camera Label (8082), Pure Pursuit (8083), Episodic Policy Search (8084), Smart Follow (8085), Line Follow (8086) and Web Teleop (8087). Wall Follow, Line Follow, Camera Label and Web Teleop are upstream's dashboards, adapted to the NeoRacer's lidar, data folders and look. Pure Pursuit, Episodic Policy Search and Smart Follow are the NeoRacer's own, which upstream retired in 0.8.1. Save writes a car's tuning to the car profile, and logs go under the data root, so a driver update keeps both. Camera Label saves its dataset to `datasets/camlabel` under the data root.
- `/etc/sudoers.d/neoracer` lets the racecar user start, stop and restart the NeoRacer services without a password. Setup checks it with `visudo -c` before installing it.
- Hardware tests for the lidar, ESP32, handset, camera and the Jetson's inference stack, run on the car with `python3 -m pytest -m hardware test/test_hardware.py`. A topic contract test checks every topic, service and frame the student library and the dashboards use against the driver's own sources, including their QoS.
- `racecar lint` runs ruff, black and mypy.
- `scripts/sysinfo.py` reads the Jetson for the watchdog, `racecar status` and the pages: the junction temperature, over-current throttling, power mode, board power, fan and GPU.

### Changed
- The FlySky channels are read as the SBUS counts the ESP32 sends (240, 1023 and 1807). A centred stick reads 0 instead of about -0.95. The mode switch is read on radio Ch7: up or middle sets LB and down sets RB on `/joy`. LB or RB stays set for as long as the switch stays in place. On 0.6.x neither button was ever set, so a program that waits for RB now starts as soon as the switch is DOWN. Steering on `/joy` reads +1 with the stick left.
- `base_link`, the floor point under the rear axle, is the base frame, and the controller's odometry names it.
- Detections are off at every boot, where 0.3.2 to 0.6.1 turned them on. Until they are on, `rc.vision` returns no detections, and Pure Pursuit and Smart Follow have no target. The apps page shows a prompt to turn them on when one of those two starts, and `racecar teleop inference_enable:=true` turns them on for one launch.
- The watchdog restarts a stopped node at once, then waits longer before each next restart. After five restarts that each fail within 60 s, it stops restarting that node. It leaves alone a node whose device is unplugged or that was switched off.
- Setup installs everything above. It enables the map viewer, installs the autonomy service without enabling it, installs the sudoers file and the lab dashboards, and adds the ROS packages mapping and localization need. It clones the student library at the tag `v0.9.0-neobotics`. It runs only as the racecar user from `~/ros2_ws/src/neoracer_ros2_driver`.
- `racecar service` runs one driving lab dashboard at a time, and `racecar service status` shows each page's address. A bare restart and `racecar update` restart at most one driving dashboard.
- The student library is pinned at `v0.9.0-neobotics` of [racecar-neo-library](https://github.com/Neobotics-Foundation-Inc/racecar-neo-library): upstream's v0.9.0 library with the NeoRacer's changes. It replaces the library's earlier `rc.slam` with upstream's, which reads `/autonomy/status` and `/autonomy/pose`. Code that calls the earlier `update`, `reset`, `load_map` or `save_map(name)` no longer runs. The library reads the battery from `/battery/state` and adds `rc_utils.get_data_dir()`.
- The mux is upstream's, with the NeoRacer's `/drive` passthrough kept. It still forwards any fresh `/drive` and publishes zero once it is older than 0.5 s, and the handset's mode switch, read by the ESP32, still selects who drives. Upstream's FlySky authority ships switched off.
- The teleop launch takes the control nodes' config files as arguments. `launch_teleop.sh` points `~/logs/latest` at the new log folder with one rename, so the link never goes missing during a restart.

### Removed
- `racecar mapping` and `racecar navigation`, which ran the vendor SLAM and Nav2 packages. Setup changes `~/osracer_ws` only for the shared lidar, as it did before 0.9.0. `racecar ws osracer` still switches a shell to it, and maps made with the vendor stack stay there.
- The vendor robot description, the vendor SLAM and Nav2 includes, the vendor EKF, the complementary filter and `twist_bridge_node` from the autonomy launch. Its `slam:=` and `nav:=` arguments give way to upstream's `mode:=`, `map:=`, `pose:=` and `localizer:=`.
- The seven lab dashboard repositories that setup cloned. Setup copies each old clone's tuning into the car profile, moves its logs under the data root, and then moves the clone to `scripts/dashboards/.migrated/`. It also moves Camera Label's dataset from `~/data/camlabel` to the data root.
- The node graph and the vendor autonomy rows of the old 8080 page.
- `rc.nav` from the student library. The driver ships no navigation until a later upstream release.

### Deprecated
- `/battery` and the `base_footprint` frame stay for one more release, for code that still reads them. `racecar status` reads `/battery/voltage`, so it stays.

### Fixed
- Stop on Line Follow, the speed slider at 0 on Pure Pursuit, and the car-follow speed slider at 0 on Smart Follow now stop a car that holds its last command after it loses the line or the target.
- Wall Follow, Episodic Policy Search and Smart Follow keep their heading search within 135 degrees of the nose on each side. Before, a wide setting searched the lidar's blind rear, which reads as open space.
- A notebook that calls `rc.go_async()` drives the car with `rc.drive.set_speed_angle()`. The library that 0.6.x cars install ran its own trigger drive beside the notebook and sent speed 0.
- The udev rules create `/dev/osrbot_base` and `/dev/osrbot_led_matrix` for the NeoRacer V1 board and its LED matrix on their own.
- `racecar build` removes links left by deleted files before it builds.

### Notes
- A car on 0.6.x updates with `racecar update`. Setup moves the car's per-car files into the car profile. It copies the tuning from the old dashboard clones, moves their logs under the data root, then moves the clones to `scripts/dashboards/.migrated/`. That folder can be deleted once the car profile and the data root hold what the car needs. After the first update, open a new terminal so the shell loads the 0.9.0 commands, such as `racecar map` and `racecar calibrate`.
- Switching the handset off does not stop the car. With the handset off or out of range, the ESP32 follows the Jetson, as it does with the mode switch DOWN. With the switch DOWN, the ESP32 keeps repeating the last channel values, so `/rc/link` stays true and RB stays set. Flip the mode switch UP to stop the car. The program keeps running until it is stopped.
- FlySky authority stays off in `config/mux.yaml`, so the mux never reads the handset. Keep it off on a NeoRacer. The handset's mode switch reads the other way from upstream's, so with it on and `autonomy_passthrough` off, switch DOWN would send the sticks through the Jetson.
- The mapping speed cap acts on commands from the Jetson only. When a person drives by hand, the ESP32 caps the sticks at 15 percent, about 0.9 m/s, only while the speed switch is UP. `racecar map` reads the switch once, when a run starts.
- `racecar service start autonomy` starts the last autonomy run with no speed switch check, where `racecar map start` and a mapping `racecar map resume` check it. `racecar service start` with a driving dashboard leaves a mapping or localization run going, where the apps page stops that run first, after a question, and saves a mapping run. The mapping speed cap still holds the dashboard's `/drive`.
- The odometry step's scales act only when `odometry.source` is `integrated`. With the default `firmware` source, `/odom` and `/odom/integrated` carry the ESP32's own pose, and the step reports its error without changing it.
- `rc.lidar`, Episodic Policy Search and Smart Follow do not apply the lidar yaw that `racecar calibrate` writes. The frame tree, SLAM, the map viewer, System Health, Wall Follow and Web Teleop do.
- If the autonomy node ends without a clean stop, the mapping speed cap and the inference pause stay on. `racecar service restart teleop` lifts both.
- The Foxglove bridge works with Lichtblick 1.22 or later.
- A car whose handset sends other counts sets its own in the car profile.
- Two upstream 0.9.0 bugs are fixed on the NeoRacer: a recording no longer takes over the terminal that started it, and `autonomy_node` no longer aborts at exit after Ctrl-C.
- Not ported, because the NeoRacer has different hardware: upstream's Teensy board and IMU fusion nodes, Pi network tools, safety override, and the IMU, magnetometer and RealSense calibrations.

## [0.6.1] - 2026-10-26

On a fresh Jetson image the first command of `setup_ros2.sh` was an
`apt-get install`, run before any `apt-get update`, and it failed with
`Unable to fetch some archives` because the image's package lists were stale.
Seen setting up a new car (Ansh, 2026-10-26).

### Fixed
- `scripts/setup_ros2.sh` refreshes the package lists before installing the bootstrap tools, so a new car's first run no longer stops at the ROS2 Humble install.

## [0.6.0] - 2026-09-04

Adds `teleop_dashboard` to the labs setup clones. It is the seventh lab and the
first that drives the car by hand: buttons and keys in a browser instead of a
controller, with the camera, the lidar, and the encoder on the same page, so a
student can feel out a track and read the car's response before automating it.

### Added
- `webteleop` (port 8087), the seventh lab dashboard: `scripts/setup_services.sh` clones `teleop_dashboard` into `scripts/dashboards/` and runs its `setup.sh`, and `racecar service start|stop|restart|enable|disable|logs webteleop` drives it like the others. Tab completion and `racecar service status` cover it. Four pad buttons, bound to WASD and the arrow keys and pressable two at once, set a forward/back and left/right command scaled by `speed` and `angle` sliders; a command older than `cmd_timeout` drives zero, so a closed tab or a dropped link coasts to a stop. It reads `/camera/color`, `/scan`, and `/odom` and publishes `/drive`.

### Changed
- The one-at-a-time rule now names six `/drive` publishers rather than five. webteleop drives the car whenever a button is down, so wallfollow, pursuit, eps, smartfollow, linefollow, and webteleop each exclude the other five.

### Notes
- The unit and CLI name is `webteleop`, not `teleop`, because `neoracer-teleop.service` is the core stack this dashboard rides on and `racecar service start <name>` maps straight to `neoracer-<name>`. The repository keeps the `teleop_dashboard` name; the `name:repo` pair in `DASHBOARDS` is what joins the two.

## [0.5.0] - 2026-09-04

Adds `linefollow_dashboard` to the labs setup clones. It is the sixth lab and
the first driven by the camera alone: the oneshot lab's `hsv-p_tuner.py` served
as a web page, so a student tunes the color threshold, the crop band, and the PD
steering from trackbars and reads the follower's stability off a live trace
instead of the car's desktop.

### Added
- `linefollow` (port 8086), the sixth lab dashboard: `scripts/setup_services.sh` clones `linefollow_dashboard` into `scripts/dashboards/` and runs its `setup.sh`, and `racecar service start|stop|restart|enable|disable|logs linefollow` drives it like the others. Tab completion and `racecar service status` cover it. It thresholds `/camera/color` in HSV, crops a band across the floor, takes the largest contour's center as the line, and steers with a PD on its offset from the frame center; throttle is constant or bled off in proportion to the offset. It reads `/camera/color` and `/odom` and publishes `/drive`.

### Changed
- The one-at-a-time rule now names five `/drive` publishers rather than four. linefollow drives the car whenever it is up, so wallfollow, pursuit, eps, smartfollow, and linefollow each exclude the other four.
- The `idle_text` comments in `led_matrix_node`, `config/led_matrix.yaml`, and `test/test_led_matrix.py` are cut to the point each one makes. No behavior change.
- The README's unsupported-hardware paragraph names `rc.display.clear()` alongside `show_text()` as a call that works on this panel. `clear()` returns it to the idle frame; the student library added it in `racecar-neo-library`.

## [0.4.3] - 2026-08-11

Gives the dot-matrix panel's idle frame an owner. The `N` on an untouched car
was the panel firmware's power-on state, not anything the stack maintained: the
node held the port open and wrote only what arrived on `/dotmatrix/text`, so the
first `show_text()` of a session replaced `N` for good and a lab that ended on a
blank frame left the car blank until the panel was power-cycled.

### Added
- `idle_text` parameter on `led_matrix_node` (default `"N"`, set in `config/led_matrix.yaml`): the frame the panel carries when no lab is driving it. Written once the serial port is open and again in `destroy_node()` before the port closes, so a stack restart returns the display to a known state. Set it to `" "` for a blank idle panel.
- `_write()` on `led_matrix_node`, the single serial path the subscription callback and both idle writes share. It no-ops when the port is closed, so an idle write during teardown cannot raise past `destroy_node()`.
- `test/test_led_matrix.py`: nine tests over a fake serial port, covering the startup and shutdown idle writes, newline handling, a write failure mid-session, the parameter overrides, and the signal handling `main()` installs.

### Fixed
- `led_matrix_node` exits on SIGTERM instead of running to systemd's stop timeout and dying on SIGKILL. `rclpy.init` now takes `signal_handler_options=ALL`, so SIGTERM goes through rclpy's handler and breaks the executor's wait set; `main()` catches `ExternalShutdownException` alongside `KeyboardInterrupt`. Without this the shutdown idle write is unreachable in service use, since only an interactive Ctrl-C reached `destroy_node()`. A plain `signal.signal()` handler does not work here: the Python handler does not run until `rcl_wait` returns, which for an idle node is never.

### Notes
- The idle frame is restored at node startup and shutdown only; there is no timeout that reclaims the panel from a running lab. A lab that intentionally holds a blank frame, such as `ultimate-wall-follower`, keeps it. Restoring mid-session is `racecar service restart led_matrix`, or a `show_text()` from the lab itself.
- rcl rejects a valueless parameter in a params file, so `idle_text:` with nothing after it is a launch failure rather than a way to switch the idle writes off; `" "` is how to ask for a blank panel. A `-p idle_text:=` style override can still resolve to no value, and the node treats that as "leave the panel alone" instead of failing at startup.

## [0.4.2] - 2026-08-07

Adds `smartfollow_dashboard` to the labs setup clones. It is the fifth lab and
the first that composes two earlier ones: students tune wallfollow and pursuit,
then run both gains together in one controller.

### Added
- `smartfollow` (port 8085), the fifth lab dashboard: `scripts/setup_services.sh` clones `smartfollow_dashboard` into `scripts/dashboards/` and runs its `setup.sh`, and `racecar service start|stop|restart|enable|disable|logs smartfollow` drives it like the others. Tab completion and `racecar service status` cover it. It runs the wallfollow dynamic gap follower on `/scan` and the pursuit controller on `/edgetpu/inference` at once, arbitrating between them: a target seen within `target_hold` seconds gets the car, otherwise the wall follower drives. It reads `/scan`, `/odom`, `/camera/color`, and `/edgetpu/inference` and publishes `/drive`.

### Changed
- The one-at-a-time rule now names four `/drive` publishers rather than three. smartfollow drives the car whenever it is up, so wallfollow, pursuit, eps, and smartfollow each exclude the other three.

## [0.4.1] - 2026-08-07

Adds `eps_dashboard` to the labs setup clones. It follows the contract the
other three already meet, so the driver's side of it is the service name, the
port, and the entry in the clone list.

### Added
- `eps` (port 8084), the fourth lab dashboard: `scripts/setup_services.sh` clones `eps_dashboard` into `scripts/dashboards/` and runs its `setup.sh`, and `racecar service start|stop|restart|enable|disable|logs eps` drives it like the others. Tab completion and `racecar service status` cover it. A (1+1) evolution strategy tunes the gap follower's four PID coefficients from one success or fail press per episode; it reads `/scan` and `/odom` and publishes `/drive`.

### Changed
- The one-at-a-time rule now names three `/drive` publishers rather than two. eps drives the car for the length of an episode, so wallfollow, pursuit, and eps each exclude the other two.

### Fixed
- `test_flake8` and `test_pep257` exclude `scripts/dashboards/`. The linters walked the dashboard checkouts, so a car that had run setup failed the driver's suite on style in code this repository does not own; each dashboard is linted in its own repository. Present since the checkouts landed in 0.4.0, and eps would have been a fourth source of it.

## [0.4.0] - 2026-08-06

Brings the three lab dashboards onto the car as part of setup. They were
installed by hand per car, each from a `scp` of its repository, which left every
car's set of labs slightly different and none of them visible to the driver.
Setup now clones them and installs their units; `racecar service` drives them
like any other service.

### Added
- `scripts/setup_services.sh` clones `camlabel_dashboard`, `wallfollow_dashboard`, and `pursuit_dashboard` into `scripts/dashboards/` and runs each one's `setup.sh` to install its unit. Each checkout tracks its default branch, so a lab is revised without a driver release.
- `racecar service start|stop|restart|enable|disable|logs <name>` accepts `camlabel`, `wallfollow`, and `pursuit` alongside the core units, and `racecar service status` lists them under their own heading. Tab completion covers the new names.
- `_rc_enabled_units` and `_rc_unit_installed` in `racecar-tool.sh`, the enable-state and installed-state queries the service actions share.

### Removed
- `neoracer-autonomy.service` is held: setup no longer installs it, and a setup run disables and removes it from a car that has it from an earlier release. The service does not work yet, so leaving it enabled meant every car booting a unit that fails; development is paused until further notice. The unit file stays in `scripts/`, and the hold lifts by moving the name from `HELD` back into `SERVICES`. SLAM and Nav2 are unaffected, still on demand through `racecar mapping` and `racecar navigation`; the base runs from `scripts/launch_autonomy.sh` in a terminal meanwhile.
- `autonomy` drops out of the `racecar service` unit list, its tab completion, and the blanket start/stop/enable/disable set. The autonomy checks in `racecar compile` and the SLAM/Nav2 environment now look for the `launch_autonomy.sh` process rather than the unit.

### Changed
- The dashboards install disabled and are started per session. Each holds the camera or the GPU for its whole run, and wallfollow and pursuit both publish `/drive`, where a second publisher fights the mux; one at a time is the working assumption, so none of them belong on the boot path.
- `racecar service restart` with no unit named now restarts only units that are enabled, dashboards included, instead of every unit in the list. `systemctl restart` starts a stopped unit, so the old behavior would have put a disabled lab dashboard on the graph on every field update.
- `racecar update` restarts the same enabled-only set rather than a hardcoded list of five.
- `racecar service enable` and `disable` with no unit named still cover the core stack only. A dashboard is enabled by name, which keeps the opt-in explicit: `racecar service enable camlabel`.
- `racecar service start|stop|restart <name>` reports an uninstalled unit and points at `racecar service install`, rather than passing systemd's error through.
- `racecar service status` prints `not installed` for an absent unit instead of systemd's "Failed to get unit file state" string.
- The dashboard checkouts are fast-forwarded, never reset. A car's tuned `wallfollow.yaml` or `pursuit.yaml` lives in the checkout, and a checkout that cannot fast-forward is reported and left alone rather than overwritten.
- `scripts/dashboards/` is gitignored. The driver ships the wiring rather than a vendored copy, and `racecar update`'s `git reset --hard` leaves the untracked checkouts in place.
- A dashboard that fails to clone or install no longer fails the setup run: the core stack finishes and the run ends by naming what to re-run once the car has internet.

### Fixed
- `pursuit.py` exits on SIGTERM instead of blocking in `serve_forever()` until systemd's stop timeout SIGKILLs it. Fixed in `pursuit_dashboard` and carried here as the version the driver installs; camlabel and wallfollow already had it.

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
- `models/yolo26n.engine` is committed and is the `model_path` default, so a fresh clone runs at TensorRT speed (about 2.4x the `.pt`) without the roughly eight-minute build. The fleet shares one configuration -- Orin SM 8.7, TensorRT 10.3.0.30, JetPack L4T R36.4.3 -- which is what makes a single engine valid on every car. It adds 8.1 MB to the repository, against a 7.7 MB `.git` before it.
- `inference_node` falls back to the `.pt` beside an `.engine` it cannot use, logging a `WARN` that names the reason, instead of exiting. A car that has drifted from the fleet configuration keeps publishing detections at the slower `.pt` latency until `racecar compile` is run there. Ultralytics defers engine deserialization to the first inference, so the failure appears during warmup rather than at load; both routes go through the same fallback.
- `inference_lib.fallback_weights` resolves that sibling path, with three unit tests.
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
