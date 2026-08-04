# NeoRacer ROS 2 Driver

Everything that runs on a NeoRacer car lives in this repo. Clone it into the
car's workspace and build; there are no side-loaded packages.

## Packages

| Package | What it is |
|---------|------------|
| `neoracer_ros2_driver` | Vehicle driver stack: controller, gamepad, mux, throttle, camera (native MJPG passthrough, 60 fps on `/camera`), LED matrix, odometry TF. Launch entry point is `teleop.launch.py`. |

The RichBeam LakiBeam1 lidar driver (package `lakibeam1`, publishes `/scan`) is
third-party and is **not** kept in this repo. `setup_workspace.sh` clones it
into `src/lakibeam1` from the Neobotics fork at a pinned tag; the fork carries
the scan-health fixes described in `docs/lidar_scan_health.md`.

## Provisioning a car

Brand-new car (fresh JetPack 6.x / Ubuntu 22.04 image):

```
# factory images pre-clone this repo; pull it (clone instead on a bare image)
cd ~/ros2_ws/src/neoracer_ros2_driver && git pull
bash scripts/setup_all.sh
```

Setup clones the lidar driver into `src/lakibeam1` at the pinned tag. If a
factory image already has a clone there pointing at RichbeamTechnology upstream,
setup re-points it at the Neobotics fork so the car does not run an unpatched
driver.

Then log out and back in (group changes) and bring the stack up as the
service, so it runs headless instead of holding a terminal:

```
racecar service status   # units enabled by setup, teleop not yet running
racecar service start    # starts neoracer-teleop (watchdog follows)
racecar service status   # confirm teleop active; it now also starts on boot
```

Wi-Fi AP + lidar subnet setup is separate: `racecar setup networking`.

### Running the stack

The systemd service is the normal way to run the car: `racecar service
start` / `racecar service logs` / `racecar service status` (wrapping
`neoracer-teleop`), headless and started automatically on every boot once
enabled. Verify with `ros2 topic hz /scan` or `ros2 topic echo /motor`
while moving the FlySky sticks (3-position switch in manual).

`racecar teleop` runs the same stack in the foreground of your terminal
for interactive debugging, like a dev server: startup logs stream, then it
goes quiet and holds the terminal while the car is live. That is it
working, not hanging. Ctrl+C stops it. Stop the service first - the two
can't share the serial ports.

### Coexisting with the vendor workspace

The factory image ships the vendor stack preinstalled in `~/osracer_ws`. It
shipped its own copy of the `lakibeam1` lidar package, divergent from the
pinned fork, so which lidar ran depended on overlay order and colcon could
refuse to build on the duplicate ("underlay override"). Setup reconciles this
to a single shared lidar: it masks the vendor copy with `COLCON_IGNORE`,
symlinks `src/lakibeam1` into `~/osracer_ws/src`, and rebuilds it there,
so both workspaces build the one canonical source (the copy with the
scan-health fixes). Setup also installs the `tf_transformations` /
`transforms3d` runtime deps the vendor bringup needs but the factory image
omits. Both workspaces stay usable, one active per shell.

- Setup never edits existing `.bashrc` lines (a factory file has unknown
  contents; commenting lines could orphan an `if`/`fi` or kill unrelated
  exports). It appends one marked block, `# Neoracer - default workspace`,
  at the end: it runs after whatever the factory lines set up and resets the
  environment to the neoracer overlay. Nothing under `~/osracer_ws` is
  modified.
- `racecar ws osracer` switches the current shell to the vendor stack;
  `racecar ws neoracer` switches back; `racecar ws` shows which is active.
  The switch is per shell and leaves disk state alone.
- Restore the factory default entirely by deleting the
  `# Neoracer - default workspace` block from `~/.bashrc`; every factory
  line is still there, untouched.
- If the vendor stack autostarts on your image
  (`systemctl list-unit-files | grep -i osr`), stop it before `racecar teleop`
  or the two stacks will fight over the serial ports.

Updating an already-provisioned car:

```
cd ~/ros2_ws/src/neoracer_ros2_driver && git pull
bash scripts/setup_workspace.sh          # rebuild + keep the shared lidar in sync
sudo systemctl restart neoracer-teleop
```

## Checking lidar health (the watchdog)

The lidar driver watches its own output: when a scan streams with (nearly) every
range inf for 3 s, it logs `[scan-watchdog]` at ERROR and attaches the sensor's
live state JSON - laser on/off, motor rpm, scan window - so the log names the
culprit. On any "lidar looks dead" report, check this first:

```
journalctl -u neoracer-teleop -b | grep scan-watchdog
```

Empty output means the scan never went blind this boot. Add `-f` to watch live
while driving. A recovery is logged at INFO with the total blind duration.

A bare `ros2 topic echo /scan` is NOT evidence of a dead lidar: echo truncates
arrays to their first 128 values, which sit entirely inside the always-blank
rear crop, so a healthy scan prints a wall of `.inf`. Count instead of looking,
or use RViz with Fixed Frame `laser`. Full symptom table, the 60-second decision
test, and the boot-time config push behavior: `docs/lidar_scan_health.md`.

## Student library and labs

`setup_all.sh` also installs the student side into `~/jupyter_ws/neoracer-os`:
the racecar-neo library and the NeoRacer labs, cloned from the Neobotics repos
(`racecar-neo-library`, branch `neobotics-slam-nav`, and `neoracer-labs`). That
branch carries `rc.slam` / `rc.nav` and the FlySky auto-start fix: the FlySky RC
has no START button, so `rc.go()` enters your program without one, unlike the
upstream MIT library that waits for the Xbox START. JupyterLab serves
`~/jupyter_ws` on port 8888; `racecar library --list` / `--select` choose which
library is on the Python path.

## Repo layout

`docs/` holds the operational checklists. `scripts/` holds car setup tooling.
