# Smart Follow Dashboard

Wall following and car following as one service for the NeoRacer, on port 8085. The car chases another car while one is in view and falls back to the gap follower when none is.

This is the wallfollow dashboard's dynamic controller and the pursuit dashboard's controller (`scripts/dashboards/wallfollow/` and `scripts/dashboards/pursuit/`) in a single process, unchanged, with an arbiter between them and /drive. Tune the two on their own dashboards first, then copy the numbers into smartfollow's tune panel.

## Contents

- [Service control](#service-control)
- [State machine](#state-machine)
- [Dashboard](#dashboard)
- [Parameters](#parameters)
- [Known conflicts on the car](#known-conflicts-on-the-car)
- [Tests](#tests)
- [Safety](#safety)
- [Car specifics](#car-specifics)

## Service control

Installed with the driver (`racecar service install`, which runs `scripts/setup_dashboards.sh`), stopped and disabled.
Dashboard: `http://<car-ip>:8085`.

| Command | Does |
|---|---|
| `racecar service start smartfollow` | start now |
| `racecar service stop smartfollow` | stop |
| `racecar service enable smartfollow` | start at every boot |
| `racecar service logs smartfollow` | follow the log |

Saved tuning goes to the car profile, `~/.config/racecar/dashboards/smartfollow.yaml`, which loads on top of the shipped `smartfollow.yaml`, block by block. The shipped file keeps the defaults, so a driver update keeps the car's tuning. Logs go to `dashboards/smartfollow/` under the data root (`~/neoracer_data`).

## State machine

Both controllers run continuously and each keeps its own error, steering, and throttle state. At 15 Hz an arbiter passes one of them to /drive:

```
                /scan  --->  gap follower  --+
                                             |
                                          arbiter ---> /drive
                                             |
   /edgetpu/inference --->  pure pursuit  ---+
                                  |
                          target seen within
                          target_hold seconds?
                            yes -> CAR FOLLOWING
                             no -> WALL FOLLOWING
```

A target is a detection above score_threshold, taken as the largest box among the best-scoring max_detections; the same rule the pursuit dashboard uses. `target_hold` is the only parameter this dashboard adds: detections arrive in bursts and a single dropped frame would otherwise flip the state, so a target stays valid for that many seconds after it was last seen. The state box on the dashboard shows which controller has the wheel and how long ago the target was seen.

Handover is on the command only. The gap follower keeps regulating throttle against measured odometry while pursuit drives, so it is already tracking the car's real speed when it takes over. The pursuit controller holds its last command while it waits, exactly as it does on its own dashboard, so a target that reappears inside the hold window resumes rather than restarts.

## Dashboard

Top row, the two sensor views side by side at the same width, then the state box across the rest:

- Lidar view: scan points, the search arc, the heading the gap follower picked, scroll to zoom. The heading dims while the car follower is driving, so the line the wall follower would have taken stays visible.
- Camera view: detection boxes, the target in red, the yellow dashed stop size (safety_area), and the center line the steering chases. It renders at the pursuit dashboard's width from the same 320 px preview, so it costs the browser no more than pursuit does.
- State box: CAR FOLLOWING (red) or WALL FOLLOWING (solid ink), why, and the `target_hold` input.

The state box carries the live error vs setpoint chart under it. Both controllers report a normalized -1 to +1 error, so one axis serves the whole run; the strip along the bottom is red where the car follower was driving. Yellow markers land at every parameter change. The chart is drawn from an even sample of the history rather than every row; the saved csv keeps all of them.

Second row, full width:

- Tune panel: the wallfollow fields and the pursuit fields in their original order, one group per column. Each has its own speed slider, applying live while dragging; the rest apply on Apply.
- Save and Load write and read the car profile's smartfollow.yaml. Reset (top bar) re-reads it.
- Save log snapshots what is on the live chart to LogN.csv, including a state column. Load log defaults to the latest save, markers and the state strip included.

## Parameters

smartfollow.yaml holds the two parameter sets in separate blocks, key for key with the dashboards they come from:

| Block | Keys | Source |
| --- | --- | --- |
| `wall:` | speed, kp, kd, speed_kp, speed_kd, window, max_mps, lookahead, width, side_weight | wallfollow.yaml with `mode: 1` |
| `car:` | speed, speed_kp, safety_area, angle_kp, angle_kd, score_threshold, max_detections | pursuit.yaml |
| top level | target_hold, camera_topic, detections_topic, preview_width, preview_quality | new, plus pursuit's topics and preview settings |

`speed` and `speed_kp` exist in both blocks and mean different things, which is why the blocks are nested rather than flattened. Copy tuned values straight across: the wallfollow keys go under `wall:`, the pursuit keys under `car:`. Static wall-follow mode is not carried over; this dashboard is always the dynamic gap follower, so `mode` and `look_angle` are gone.

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter:

- The wallfollow, pursuit, eps, linefollow and webteleop dashboards all publish /drive. Stop them before starting this one: `racecar service stop wallfollow`, and the same for the others.
- A leftover Jupyter kernel that ever created a racecar object keeps publishing /drive. Restart the jupyter service to clear them.

Check with: `ros2 topic info -v /drive` (there should be exactly one publisher: smartfollow).

The detections come from the driver's inference node, which is off at boot, so turn detections on for car following to ever engage. With no detections the service still works; it just stays in wall following.

## Tests

`test/test_dashboard_smartfollow.py` in the driver drives both callbacks and the arbiter with hand-built LaserScan and Detection2DArray messages, so no ROS graph and no car are needed. It covers the gap follower's steering sign and throttle ramp, target selection and the safety_area brake, the handover both ways across target_hold, the steering negation on /drive, and the car profile round trip that keeps `speed` and `speed_kp` in their own blocks.

## Safety

The NeoRacer mux forwards /drive with no software deadman. The handset's mode switch is the physical gate: with the handset on, the ESP32 follows /drive only with the switch down. With the handset off or out of range it follows /drive too, so keep the handset on and flip the switch up to stop the car. The shipped yaml has both speeds at 0.0, so the car cannot drive until a slider is raised. The speed command is hard capped at 1.0 in code.

Two throttle caps means two ways to move: raising `car.speed` alone gives a car that chases but will not wall follow, and raising `wall.speed` alone gives the reverse. Raise both to run the full behavior.

## Car specifics

This dashboard is calibrated for the NeoRacer: LakiBeam lidar angle mapping, steering sign, speed feedback from /odom, detections on /edgetpu/inference, ROS Humble paths. Both signs were verified physically on the car.
