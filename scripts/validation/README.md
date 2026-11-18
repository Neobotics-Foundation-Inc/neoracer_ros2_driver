# Validation scripts

Test tooling for development on the car. Run from the source tree; nothing
here is installed or exposed through `racecar`.

## Contents

- [SLAM session](#slam-session)
- [Single measurements](#single-measurements)
- [Storage](#storage)

## SLAM session

```sh
cd ~/ros2_ws/src/neoracer_ros2_driver/scripts/validation
./slam_session.py map NAME                    # mapping; --camera off to drop the USB camera
./slam_session.py localize MAP                # AMCL; --localizer slam_toolbox, --pose x,y,yaw
./slam_session.py localize MAP --inference on # YOLO inference running too (off by default)
./slam_session.py analyze [SESSION_DIR]       # rewrite report.md; default the newest session
./slam_session.py replay SESSION_DIR TAG --set angle_variance_penalty=0.03   # offline tuning
./slam_session.py replay LOC_SESSION_DIR TAG --localizer slam_toolbox       # other localizer, same bag
./slam_session.py replay LOC_SESSION_DIR TAG --autonomy                     # with autonomy_node: time to localized
```

One terminal runs the whole session. You drive by hand with the handset, mode
switch UP (manual). In a mapping session `throttle_node` applies the mapping
caps to the Jetson's commands while `/autonomy/status` reports mapping, but
with the mode switch UP the ESP32 drives from the sticks and the caps do not
apply. Keep the speed switch UP (slow) in every session, because the firmware
caps the sticks at 15 percent only then. Saves go through
`/autonomy/save_map`. The script stops the teleop service
(sudo may ask for a password), starts teleop with inference off, the autonomy
launch, `foxglove_bridge` with `foxglove_validation.yaml`, an MCAP bag, and the
load recorder, then prints the Lichtblick address (`ws://<car>:8765`). On
Humble the MCAP plugin is the package `ros-humble-rosbag2-storage-mcap`, which
`scripts/setup_ros2.sh` installs; without it the session does not start.
A localization session starts the localizer at the map's start pose from its
`meta.yaml`, the pose where the map was begun, unless `--pose` gives another.
The ESP32 keeps its odometry pose through the session's teleop restart, so the
map origin is the mapping run's odometry origin and not the start mark. Give a
pose that starts with a minus sign as `--pose=-0.1,0.2,1.5`.

Keys while driving, read without Enter:

| Key | Action |
|---|---|
| `m` | Record the map pose with the car stopped on a mark; asks for a label |
| `e` | Note an event with the time (car set down, person walking); goes to `events.csv` and the report |
| `g` | Ask AMCL to search the whole map (localization with AMCL); logged as an event |
| `s` | Save the map now (mapping) |
| `t` | Timed save: the longest gap in `/pose` while slam_toolbox serializes (mapping; drive while pressing it) |
| `q` | Stop everything in order, save a mapping run, write the report |

Each session folder in `~/neoracer_data/slam_eval/sessions/` holds `session.yaml`
(settings, commit, start and end storage), `calibration/` (the car profile's
`params/` files), `load.csv`, `marks.csv`, `events.csv`, `status.csv`,
`stalls.csv`, `bag/`, `logs/`, `report.md`, and `map.png`. A mapping run's map
goes to `~/neoracer_data/maps/NAME/`. `~/neoracer_data` is the data root;
`racecar data` prints it.

`load.csv` has one sample a second: CPU per node (one core is 100), all cores
busy, GPU load, the junction temperature, the board's input power (`VDD_IN`),
memory used and available, swap in use, CPU 0's clock, and the SoC's
over-current alarms raised since the session started. The report's Load
section summarizes them with the Jetson power mode and says to discard the run
when swap in use grew during the run or an alarm fired. Swap already in use at
the start, such as pages a build pushed out, is reported and not counted. The
status line shows `THROTTLED` and `SWAP` as they happen.

Taped distances between marks go in `~/neoracer_data/slam_eval/taped.txt`, one pair per
line: `A B 2.35`. A line `point X Y` gives the point on the car the tape was
measured from, in `base_link` meters (default 0 0, the floor under the rear
axle midpoint), for example `point 0 0.112` at the middle of the left rear
tire (half the NeoRacer's 224 mm track).

`replay` runs a session's bag through a fresh mapper or localizer with the
`--set` overrides (repeatable, YAML values) applied to slam_toolbox or AMCL,
and writes `replay/TAG/` in the session folder: `overrides.yaml`,
`report.md`, `logs/`, `map_odom.csv`, with `--autonomy` `status.csv` (and the
seconds `localized` disagreed with the scan fit in the report), and for a mapping run `map.pgm`, `map.yaml`, and
`map.png`. A mapping run replays through slam_toolbox mapping
(`config/slam_mapping.yaml`). A localization run replays through its own
localizer, or the one `--localizer` names, on the run's map from its start
pose (AMCL with `config/amcl.yaml`, the slam_toolbox localizer with
`config/slam_localization.yaml`), so both localizers can be scored on the
same drive. As in the autonomy launch on Humble, slam_toolbox runs without a
lifecycle manager; map_server and AMCL run under `lifecycle_manager_localization`.
It needs ROS 2 Humble with slam_toolbox and the Nav2 packages, on the car or any
machine with the driver built. It runs on ROS
domain 77 and drops the bag's `map -> odom`, so it neither sees nor disturbs
a running car and the recorded solution does not leak in. It takes about as
long as the session. Compare variants by the rotation spread of `map -> odom`
(how often the scan matcher overrides the odometry) and by occupied cells per
free square meter (thinner walls give fewer).

The odometry lines of session and replay reports measure `map -> odom` from
its last value before the car first moved. A localization run starts the
transform at the start pose, so the raw value is not drift; a mapping run
starts it at zero, so there the two are the same.

Session and replay reports score the scan-to-map fit (`scan_fit.py`): each
scan is placed in the map at the localizer's pose and every point's distance
to the nearest wall cell is measured. The report gives the median distance,
the share of points within 100 mm, and the 90th percentile of each scan's
median (the bad stretches), with the time of the worst scan. A mapping run
scored against its own map gives the best fit the map allows. Only returns
inside slam_toolbox's range band count (`min_laser_range` to
`max_laser_range` in `config/slam_mapping.yaml`, with the car profile's
override); the LakiBeam's blind rear wedge reads inf and drops out.

## Single measurements

`slam_eval.py` runs one measurement beside `racecar launch autonomy`: `save`,
`load`, `mark`, `report`, `quality`. See its docstring.

`live_fit.py MAP` checks a localized pose with the car parked and autonomy
running on MAP: it scores a few live scans against the map at
`/autonomy/pose`, then searches around that pose (0.5 m and 20 deg by
default) for the best fit and prints the offset. An offset that does not
shrink as the localizer runs is a start pose it never corrected.

`compare_maps.py BASE EXTENDED` checks an extension (`racecar map start
EXTENDED --from BASE`) without ROS: the distance from EXTENDED's walls to
BASE's over the area BASE observed, and an overlay in `~/jupyter_ws/tmp/`
(BASE blue, EXTENDED red, dark gray where both draw a wall). A wrong start
pose shows as a second, shifted copy of the walls in red.

## Storage

A session uses about 120 MB for a 5 minute run, most of it the LakiBeam's
`/scan` at about 0.34 MB/s, plus about 7 MB per timed save (upstream's
figure). The session script warns below 10 GB free or above 80% used, lists the
largest folders under `~/logs`, the sessions, and the maps, and does not
start with less than 3 GB free.
