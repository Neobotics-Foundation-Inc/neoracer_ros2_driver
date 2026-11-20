# Wall Follow Dashboard

Wall following as a service for the NeoRacer, with a live tuning dashboard on port 8081.

One process subscribes to /scan, computes PD steering, publishes AckermannDriveStamped on /drive, and serves the dashboard. It runs as the systemd unit `neoracer-wallfollow`, installed with the driver and left off until a session starts it.

## Contents

- [Service control](#service-control)
- [Dashboard](#dashboard)
- [Modes](#modes)
- [Speed](#speed)
- [Parameters](#parameters)
- [Known conflicts on the car](#known-conflicts-on-the-car)
- [Tests](#tests)
- [Safety](#safety)
- [Car specifics](#car-specifics)

## Service control

Installed with the driver (`racecar service install`, which runs `scripts/setup_dashboards.sh`), stopped and disabled.
Dashboard: `http://<car-ip>:8081`.

| Command | Does |
|---|---|
| `racecar service start wallfollow` | start now |
| `racecar service stop wallfollow` | stop |
| `racecar service enable wallfollow` | start at every boot |
| `racecar service logs wallfollow` | follow the log |

Saved tuning goes to the car profile, `~/.config/racecar/dashboards/wallfollow.yaml`, which loads on top of the shipped `wallfollow.yaml`. The shipped file keeps the defaults, so a driver update keeps the car's tuning. Logs go to `dashboards/wallfollow/` under the data root (`~/neoracer_data`). Internals: `docs/dashboards.md` in the driver.

## Dashboard

- Lidar view: scan points, the averaging windows or search arc in gray, and the measured rays in red. Scroll or use + and - to zoom.
- Error chart: the last 30 s of error against the setpoint (cm in static mode, -1 to 1 in dynamic), with dashed lines at parameter changes.
- Tune panel: speed is a 0 to 1 slider that applies while dragging; the other values apply on Apply.
- Save and Load write and read the car profile's wallfollow.yaml. Reset (top bar) re-reads it. Stop (top bar) sets speed to 0.
- Save log snapshots the error chart to LogN.csv. Load log draws a saved one with the error and the measured speed.

## Modes

`mode` selects the controller: 0 = static, 1 = dynamic. It is switched only by editing the car profile's wallfollow.yaml (Save once creates it), then pressing Reset or Load. There is no UI toggle.

- `static`: mirrored side windows at look_angle. Error = right minus left in meters, setpoint 0 keeps the car centered.
- `dynamic`: steers toward the most open heading in a front arc, obstacles inflated by the car's half width. width opens and closes the arc, side_weight biases one side.

All parameters are documented in wallfollow.yaml.

## Speed

The speed slider is the throttle. With speed_kp and speed_kd at zero, the shipped values, the car holds that constant command. Nonzero gains bend the throttle toward a target set by how much road the chosen line has, using the measured speed from /odom: slower into corners, corrected for battery and surface. The live throttle shows in the summary.

## Parameters

wallfollow.yaml. Save rewrites the numbers in place, so the comments in the file survive.

| Key | Notes |
| --- | --- |
| mode | 0 static, 1 dynamic. File edit only; there is no UI toggle, because the two controllers take different parameters and mean different things by `error` |
| speed | 0..1 of `max_mps`; ships at 0.0, so the car cannot drive until the slider is raised |
| kp, kd | steering strength and its damper. Raise `kd` if the car wobbles |
| speed_kp, speed_kd | the speed correction toward the road-shaped target. 0 leaves speed constant |
| window | averaging window per ray, degrees; smooths a noisy scan |
| max_mps | the speed the slider's 1.0 stands for in the speed correction |
| lookahead | metres. Distances are capped here in both modes, so an open side reads as `lookahead` rather than as 12 m of empty room |
| look_angle | static mode: the side windows sit this many degrees off the nose. 90 is straight sideways |
| width, side_weight | dynamic mode: how far either side of the nose to search, and a bias toward one side |

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter. The webteleop, pursuit, eps, smartfollow and linefollow dashboards all publish it; stop them before starting this one:

```
racecar service stop webteleop
racecar service stop linefollow
```

Check with `ros2 topic info -v /drive`: there should be exactly one publisher.

A leftover Jupyter kernel that ever created a racecar object also keeps publishing /drive. Restart the jupyter service to clear them.

## Tests

`test/test_dashboard_wallfollow.py` in the driver; no ROS graph, camera, or browser
needed.

## Safety

The NeoRacer mux forwards /drive with no software deadman. The handset's mode switch is the physical gate: with the handset on, the ESP32 follows /drive only with the switch down. With the handset off or out of range it follows /drive too, so keep the handset on and flip the switch up to stop the car. The shipped yaml has speed 0.0, so the car cannot drive until the slider is raised. The speed command is hard capped at 1.0 in code.

## Car specifics

This dashboard is set for the NeoRacer: the LakiBeam faces forward over 360 degrees at 1440 points with the rear 90 degrees blank, and its measured mount yaw comes from frames.yaml (`laser_yaw_deg`, with the car profile's value on top) through LIDAR_MOUNT_YAW_DEG. Positive steering on /drive turns the car left (REP-103), speed feedback comes from /odom, and the unit runs on ROS Humble. Both signs were verified physically on the car.
