# NeoRacer ROS 2 Driver

Everything that runs on a NeoRacer car lives in this repo. Clone it into the
car's workspace and build; there are no side-loaded packages.

## Packages

| Package | What it is |
|---------|------------|
| `neoracer_ros2_driver` | Vehicle driver stack: controller, gamepad, mux, throttle, camera, LED matrix, odometry TF. Launch entry point is `teleop.launch.py`. |
| `lakibeam1` | Vendored RichBeam LakiBeam1 lidar driver, publishes `/scan`. Carries local fixes; see `docs/lidar_scan_health.md`. |

## Provisioning a car

Brand-new car (fresh JetPack 6.x / Ubuntu 22.04 image):

```
# factory images pre-clone this repo; pull it (clone instead on a bare image)
cd ~/ros2_ws/src/neoracer_ros2_driver && git pull
bash scripts/setup_all.sh
```

Setup removes the superseded sibling lidar clone factory images ship at
`src/lakibeam1` (lakibeam1 is vendored inside this repo, with local fixes).

Then log out and back in (group changes) and run `racecar teleop`. Wi-Fi AP +
lidar subnet setup is separate: `racecar setup networking`.

### Running the stack

`racecar teleop` runs the whole stack in the foreground of your terminal,
like a dev server: startup logs stream, then it goes quiet and holds the
terminal while the car is live. That is it working, not hanging. Ctrl+C
stops it. Verify from a second shell with `ros2 topic hz /scan` or
`ros2 topic echo /motor` while moving the FlySky sticks (3-position switch
in manual).

For deployment the systemd service runs the same stack on boot with no
terminal: `racecar service start` / `racecar service logs` (or
`sudo systemctl start neoracer-teleop`, `journalctl -u neoracer-teleop -f`).

### Coexisting with the vendor workspace

The factory image ships the vendor stack preinstalled in `~/osracer_ws`,
including its own copy of the `lakibeam1` lidar package. Both workspaces stay
installed side by side, but a shell can only have one active at a time: with
both sourced, the duplicate `lakibeam1` shadows and colcon refuses to build
("underlay override" on `~/osracer_ws/install/lakibeam1`).

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
cd ~/ros2_ws
colcon build --symlink-install
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

## Repo layout

`docs/` holds the operational checklists. `scripts/` holds car setup tooling.
