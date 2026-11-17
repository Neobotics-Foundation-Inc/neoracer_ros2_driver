# Line Follow Dashboard

Line following as a service for the NeoRacer, on port 8086, with the HSV threshold, the crop area, and the PD steering tuned from trackbars in a browser. The car steers toward the largest patch of a chosen color on the floor in front of it.

This is the oneshot lab's `hsv-p_tuner.py` moved off the car's desktop and onto the web: the same pipeline as the student library's `rc_utils.find_contours` and `get_largest_contour`, a mask view and a color view side by side, and a live trace of the line's position so the follower's stability can be read from a graph instead of by eye.

## Contents

- [Service control](#service-control)
- [Pipeline](#pipeline)
- [Dashboard](#dashboard)
- [Tuning](#tuning)
- [Parameters](#parameters)
- [Known conflicts on the car](#known-conflicts-on-the-car)
- [Tests](#tests)
- [Safety](#safety)
- [Car specifics](#car-specifics)

## Service control

Installed with the driver (`racecar service install`, which runs `scripts/setup_dashboards.sh`), stopped and disabled.
Dashboard: `http://<car-ip>:8086`.

| Command | Does |
|---|---|
| `racecar service start linefollow` | start now |
| `racecar service stop linefollow` | stop |
| `racecar service enable linefollow` | start at every boot |
| `racecar service logs linefollow` | follow the log |

Saved tuning goes to the car profile, `~/.config/racecar/dashboards/linefollow.yaml`, which loads on top of the shipped `linefollow.yaml`. The shipped file keeps the defaults, so a driver update keeps the car's tuning. Logs go to `dashboards/linefollow/` under the data root (`~/neoracer_data`). Internals: `docs/dashboards.md` in the driver.

## Pipeline

Frames from `/camera/color` are processed at 20 Hz; the camera publishes its own JPEG frames, faster than the follower needs, and decoding every frame would cost the CPU for nothing.

```
/camera/color --> resize to process_width --> HSV --> inRange(low, high) = mask
                                                         |
                                       rows crop_top..crop_bottom of the mask
                                                         |
                               largest contour with area >= min_area, its center
                                                         |
              offset = (center column - frame center) / (frame center)   -1..+1
                                                         |
         steer    = angle_kp * offset + angle_kd * (offset - last offset)
         throttle = speed * (1 - speed_kp * |offset|)
                                                         |
                                             /drive at 15 Hz
```

`/drive` is published from a timer, not from the camera callback, so the mux never sees a gap. With no contour in the band the last steer and throttle are held, as the lab script does, so a gap in a dashed line does not straighten the wheels. The state box shows how long the line has been lost.

A hue low set above its high wraps through 0, so red, which straddles hue 0 and 179 in OpenCV, is one range rather than two.

## Dashboard

Top row, the two views side by side at the same width, then the state box across the rest:

- Camera view: the color frame after the resize, with the crop area in gold, the winning contour and its center in red, a red tie from the center line to the contour center, and the grey setpoint line down the middle.
- Mask view: black and white, the whole frame, so the threshold can be judged on everything in view rather than only the band. The band is outlined in gold; only what is inside it is searched.
- State box: LINE FOUND in green, or LINE LOST in red. Hover it for the contour's center and area, or for how long the line has been lost.

Under the state box, the last 30 s of the line's offset against the setpoint at 0, from -1 (left edge) to +1 (right edge), on a fixed axis so a small wobble looks small. The strip along the bottom is red where the line was lost. Dashed lines mark parameter changes. The saved csv keeps every frame.

Names and numbers are on the page; short explanations are on hover. Hover a view, the chart, a group heading, or a slider.

Second row, full width, the four slider groups in the order they are tuned:

1. HSV threshold: two-thumb bars for hue, saturation, and value. The hue bar's track is the hue wheel, so the thumbs sit on the color they select.
2. Speed and angle: the constant throttle and the proportional steering gain (`angle_kp`).
3. Crop area: the top and bottom edges of the crop area, as fractions of the frame from the top. `min_area`, the smallest contour that counts as the line, sits here too.
4. Proportional speed and derivative angle: `speed_kp` slows the car as the line drifts from the setpoint; `angle_kd` damps the steering on the change in offset.

Every slider applies while it is dragged; the service folds the stream of values into one marker on the chart. Save and Load write and read the car profile's linefollow.yaml. Reset (top bar) re-reads it. Stop (top bar) sets speed to 0.

Save log snapshots what is on the live chart to LogN.csv, including the steer, the throttle, the measured speed, the lost flag, and the contour's column and area. Load log defaults to the latest save and draws the offset and the steer together, markers and the lost strip included.

## Tuning

With speed at 0 the car sits still and the pipeline still runs, so the first two steps are done parked over the line:

1. Threshold. Narrow the hue bar until only the line is white in the mask, then raise the saturation and value lows until the floor and its highlights drop out. The camera view's red contour should hug the line and nothing else.
2. Crop area. Put the crop area where the line is about a car length ahead; lower it if the car reacts late, raise it if the car cuts corners. A taller band averages more of the line and steadies the center at the cost of some lag.
3. Angle. Raise `speed` to a crawl and `angle_kp` until the car follows a curve without falling off it. The chart should settle back to 0 after every bend. A zigzag that grows is too much `angle_kp`.
4. Damping and speed. Add `angle_kd` until the zigzag dies out, then raise `speed`. If the car leaves the line in sharp turns, add `speed_kp` so it slows as the offset grows rather than lowering the constant speed everywhere.

## Parameters

linefollow.yaml, flat, in the order the dashboard shows them:

| Group | Keys | Notes |
| --- | --- | --- |
| 1. HSV | h_low, h_high, s_low, s_high, v_low, v_high | OpenCV ranges: hue 0..179, the rest 0..255; hue wraps when low > high |
| 2. speed and angle | speed, angle_kp | throttle 0..1; steer per unit offset |
| 3. crop area | crop_top, crop_bottom | fractions of the frame height from the top; the service keeps them ordered and at least 0.05 apart |
| 4. P speed, D angle | speed_kp, angle_kd | throttle drop per unit offset; damper on the change in offset |
| detection | min_area, process_width | smallest contour that counts, in pixels at process_width |
| topics and preview | camera_topic, preview_width, preview_quality | `/camera/color` on the latest driver, `/camera` on older cars |

Save rewrites the numbers in place, so the comments in the yaml survive. The values are the ones a lab passes to `rc_utils.find_contours` and `rc_utils.crop`; copy them straight into a student's line follower.

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter:

- The wallfollow, pursuit, eps, smartfollow and webteleop dashboards all publish /drive. Stop them before starting this one: `racecar service stop wallfollow`, and the same for the others.
- A leftover Jupyter kernel that ever created a racecar object keeps publishing /drive. Restart the jupyter service to clear them.

Check with: `ros2 topic info -v /drive` (there should be exactly one publisher: linefollow).

The camera is read directly, so the driver's inference node does not need to be running; the camera node does. camlabel reads the same topic and does not publish /drive, so it can run alongside.

## Tests

`test/test_dashboard_linefollow.py` in the driver; no ROS graph, camera, or browser
needed.

## Safety

The NeoRacer mux forwards /drive with no software deadman. The handset's mode switch is the physical gate: the ESP32 follows /drive only with it down. The shipped yaml has speed at 0.0, so the car cannot drive until the slider is raised. The speed command is hard capped at 1.0 in code. STOP in the top bar puts the slider back to 0.

A lost line holds the last command, throttle included. On a track where the line can end, keep `speed` low or be ready on the handset.

## Car specifics

This dashboard is set for the NeoRacer: the USB camera's JPEG frames on /camera/color (no depth), positive steering on /drive turning the car left (REP-103), speed feedback from /odom, ROS Humble. The steering sign was verified physically on the sibling dashboards and is the same here.
