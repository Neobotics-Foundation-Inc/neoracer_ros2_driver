# Web Teleop Dashboard

Manual driving for the NeoRacer from a browser, on port 8087. Four buttons, bound to WASD and the arrow keys, drive the car forward, back, left, and right, two at once for a turn on the move, with the camera, the lidar, and the wheel encoder in view. Two sliders set how hard the buttons push.

The service name is `neoracer-webteleop`, not `neoracer-teleop`: that name belongs to the driver's core stack, which this dashboard rides on.

## Contents

- [Service control](#service-control)
- [Driving](#driving)
- [Dashboard](#dashboard)
- [Parameters](#parameters)
- [Known conflicts on the car](#known-conflicts-on-the-car)
- [Tests](#tests)
- [Safety](#safety)
- [Car specifics](#car-specifics)

## Service control

Installed with the driver (`racecar service install`, which runs `scripts/setup_dashboards.sh`), stopped and disabled.
Dashboard: `http://<car-ip>:8087`.

| Command | Does |
|---|---|
| `racecar service start webteleop` | start now |
| `racecar service stop webteleop` | stop |
| `racecar service enable webteleop` | start at every boot |
| `racecar service logs webteleop` | follow the log |

Saved tuning goes to the car profile, `~/.config/racecar/dashboards/teleop.yaml`, which loads on top of the shipped `teleop.yaml`. The shipped file keeps the defaults, so a driver update keeps the car's tuning. Logs go to `dashboards/teleop/` under the data root (`~/neoracer_data`). Internals: `docs/dashboards.md` in the driver.

## Driving

The browser keeps a set of what is held: each pad button under a pointer or finger, each of the eight keys that is down. That set collapses to two axes, forward/back and left/right, each -1, 0, or +1, and the pair is posted to the service as one command on every change and every 100 ms while anything is held. Forward and left pressed together are therefore one command, applied at the same instant; forward and back together cancel to 0.

```
buttons / keys --> held set --> {fwd, turn} --> POST /cmd (on change, 10 Hz while held)
                                                        |
        throttle = fwd  * speed                         v
        steering = turn * angle          /drive at 15 Hz from a timer
                                                        |
                              no command for cmd_timeout -> drive zero
```

`/drive` comes from a timer, not from the browser's posts, so the mux never sees a gap. A command older than `cmd_timeout` counts as released: a closed tab, a dropped link, or a page that lost focus with a key down all coast to zero instead of driving the last command forever. The browser also releases everything on its own when the window loses focus or the tab is hidden, so the timeout is the backstop rather than the first line.

Keys: W, A, S, D or the arrow keys. Key auto-repeat is ignored; a key drives from its keydown to its keyup. Keys are read anywhere on the page except inside a slider or the log picker.

## Dashboard

The page uses the same cards as the simulator's sensor dashboard, read from the
car's ROS topics:

- Camera: the color frame, the USB camera's own JPEG. The Detections switch
  overlays the object detector's boxes, each with its class and confidence;
  boxes older than 1.5 s are cleared. Inference is off at boot, so turn
  detections on from the apps page first. The NeoRacer has no depth camera.
- Lidar: scan points around the car, nose up, rings every meter. Scroll or
  use + and - to zoom. 0 degrees is the nose; see `docs/dashboards.md`,
  "Lidar convention", in the driver.
- Encoder: a state box (STOPPED, FORWARD LEFT, TIMED OUT, and so on) and the
  last 10 s of measured speed (m/s, left axis) against the commanded throttle
  (-1 to 1, right axis), with dashed lines at slider changes.
- IMU: linear acceleration and angular velocity over 10 s, x, y and z, from
  `/imu/fused`, and the current values.
- Battery: the voltage now and over the last minute. The NeoRacer has no
  current sensor, so the current chart stays empty.
- Compass: a dial with the car pointing up and magnetic north marked, the
  magnetic field over 10 s from `/mag`, and the heading (0 facing north,
  increasing counterclockwise).
- Dot matrix: type text and Show text (`show_text`). The NeoRacer's 8x8 panel
  takes text only; it scrolls the text and keeps it until something else
  writes to it. The NeoRacer has no LED strip.
- Drive: the pad, or WASD and the arrow keys.
- Tune: the speed and angle sliders. Save and Load write and read the car
  profile's teleop.yaml.
- Logs: Save log snapshots the encoder chart to LogN.csv; Load log draws a
  saved one with measured speed, throttle, and steer.

The page sends the text once, because the panel holds it; sending it again
would only restart the scroll.

Reset (top bar) re-reads the saved settings. Stop (top bar) releases every button
and sets speed to 0.

## Parameters

teleop.yaml:

| Key | Notes |
| --- | --- |
| speed | 0..1 throttle while forward or back is held; ships at 0.0 |
| angle | 0..1 steering while left or right is held; 1 is full lock |
| cmd_timeout | seconds without a command before the service drives zero; not on the dashboard |
| camera_topic, preview_width, preview_quality | `/camera/color` on the latest driver, `/camera` on older cars; live view size and jpeg quality |
| detections_topic | `vision_msgs/Detection2DArray` for the Detections switch, `/edgetpu/inference` on this driver. Empty on a car with no detector, which disables the switch |

Save rewrites the numbers in place, so the comments in the yaml survive.

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter:

- The wallfollow, pursuit, eps, smartfollow, and linefollow dashboards all publish /drive. Stop them before starting this one: `racecar service stop wallfollow`, and the same for the others.
- A leftover Jupyter kernel that ever created a racecar object keeps publishing /drive. Restart the jupyter service to clear them.

Check with: `ros2 topic info -v /drive` (there should be exactly one publisher: webteleop).

The camera and the lidar are read directly, so the driver's inference node does not need to be running, except for the Detections overlay. camlabel reads the same camera topic and does not publish /drive, so it can run alongside.

## Tests

`test/test_dashboard_teleop.py` in the driver; no ROS graph, camera, or browser
needed.

## Safety

The NeoRacer mux forwards /drive with no software deadman. The handset's mode switch is the physical gate: with the handset on, the ESP32 follows /drive only with the switch down. With the handset off or out of range it follows /drive too, so keep the handset on and flip the switch up to stop the car. The shipped yaml has speed at 0.0, so the buttons steer but the car cannot move until the slider is raised. The speed command is hard capped at 1.0 either way in code, and the service drives zero after `cmd_timeout` without a command.

## Car specifics

This dashboard is set for the NeoRacer: the USB camera's JPEG frames on /camera/color with no depth, the LakiBeam lidar (scan zero at the nose, 1440 points, the rear 90 degrees blank) with its measured mount yaw from frames.yaml, positive steering on /drive turning the car left (REP-103), speed feedback from /odom, ROS Humble. The steering sign was verified physically on the sibling dashboards and is the same here. The magnetometer's axes have not been checked on a car yet (config/controller.yaml), so the compass heading is to be confirmed.
