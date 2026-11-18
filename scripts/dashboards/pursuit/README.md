# Pure Pursuit Dashboard

Target-following as a service for the NeoRacer, on port 8083. The car chases whatever the driver's YOLO inference node detects: PD steering on the target's horizontal offset, throttle that brakes as the target's bounding box grows toward a stop size.

## Contents

- [Service control](#service-control)
- [Detection and control](#detection-and-control)
- [Tuning](#tuning)
- [Known conflicts on the car](#known-conflicts-on-the-car)
- [Tests](#tests)
- [Safety](#safety)

## Service control

Installed with the driver (`racecar service install`, which runs `scripts/setup_dashboards.sh`), stopped and disabled.
Dashboard: `http://<car-ip>:8083`.

| Command | Does |
|---|---|
| `racecar service start pursuit` | start now |
| `racecar service stop pursuit` | stop |
| `racecar service enable pursuit` | start at every boot |
| `racecar service logs pursuit` | follow the log |

Saved tuning goes to the car profile, `~/.config/racecar/dashboards/pursuit.yaml`, which loads on top of the shipped `pursuit.yaml`. The shipped file keeps the defaults, so a driver update keeps the car's tuning.

## Detection and control

Detections come from the driver inference node (vision_msgs/Detection2DArray on /edgetpu/inference). The inference node is off at boot, so turn detections on before starting pursuit; with it off, the camera view shows no boxes and the car never moves. Detections below score_threshold are dropped, the best-scoring max_detections remain as candidates, and the largest box is the target. The model being served and the camera frame rate show in the summary; the model is whatever model_path the driver's inference.yaml points to, with the car profile's `params/inference.yaml` on top.

If no object is found the car KEEPS THE LAST ACTUATED COMMAND, speed and angle both, republished continuously. It does not stop on its own except by the safety_area brake. The speed slider at zero takes effect at the next detection, so with no target in view stop the app (Clear apps on 8080, or `racecar service stop pursuit`); `/drive` then goes to zero within the mux's 0.5 s timeout. The handset's mode switch is the physical gate.

## Tuning

All on the dashboard, live: speed (throttle cap, drag slider, applies while dragging), speed_kp (stopping gain), safety_area (bbox area fraction where the car fully stops; the yellow dashed outline on the camera view is this size), angle_kp, angle_kd, score_threshold, max_detections. Save writes the car profile's pursuit.yaml, Reset re-reads it.

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter:

- The wallfollow, eps, smartfollow, linefollow and webteleop dashboards all publish /drive. Stop them before starting this one: `racecar service stop wallfollow`, and the same for the others.
- A leftover Jupyter kernel that ever created a racecar object keeps publishing /drive. Restart the jupyter service to clear them.

Check with: `ros2 topic info -v /drive` (there should be exactly one publisher: pursuit).

## Tests

`test/test_dashboard_pursuit.py` in the driver; no ROS graph, camera, or browser needed.

## Safety

The NeoRacer mux forwards /drive with no software deadman; the handset's mode switch is the physical gate: with the handset on, the ESP32 follows /drive only with the switch down. With the handset off or out of range it follows /drive too, so keep the handset on and flip the switch up to stop the car. The shipped yaml has speed 0.0, so the car cannot drive until the slider is raised. Speed is hard capped at 1.0. Remember the keep-last rule: a car that loses its target mid-turn keeps turning until the switch is flipped or the box reappears.
