# ESP32 (OSRbot) serial protocol

The Seeed OSRbot ESP32 (`/dev/osrbot_base`) is a native USB-CDC device, so the
serial baud rate is irrelevant - data streams regardless of the configured baud.
The `controller` node owns the link; the parsing/mapping lives in
`neoracer_ros2_driver/controller_lib.py` (pure, unit-tested) and the node wiring
in `controller.py`.

## Inbound (ESP32 -> host), newline-terminated ASCII

Firmware `NEORACER_V1.1` (2026-07) streams one combined state frame; older
firmware streamed IMU and odometry as separate lines. The driver parses both.

| Prefix | Format | Meaning | Published as |
|--------|--------|---------|--------------|
| `s` | `s px py pz vx vy vz yaw qx qy qz qw ax ay az gx gy gz` | state @ ~200 Hz: odometry + IMU in one frame (V1.1) | `/odom` + `/imu/fused` + `/encoder/speed` |
| `b` | `b volts` | battery voltage @ ~0.5 Hz (V1.1) | `/battery/state` (sensor_msgs/BatteryState: voltage, 3S charge fraction, current NaN), `/battery` (the same message, kept one more release) + `/battery/voltage` |
| `i` | `i qx qy qz qw ax ay az gx gy gz` | IMU: quaternion, accel (m/s^2), gyro (rad/s) (legacy, pre-V1.1) | `/imu/fused` (sensor_msgs/Imu) |
| `o` | `o px py pz vx vy vz yaw` | odometry: position (m), velocity (m/s), yaw (rad) (legacy, pre-V1.1) | `/odom` (nav_msgs/Odometry) + `/encoder/speed` |
| `r` | `r c1 c2 ... c10` | FlySky RC channels (ints; `-1` = no signal) | `/joy` (sensor_msgs/Joy) + `/rc/channels` + `/rc/link` (std_msgs/Bool, true while every channel is an SBUS count, false while the firmware sends `-1`) |
| `m` | `m x y z` | magnetometer (Gauss) | `/mag` (sensor_msgs/MagneticField) |

The firmware also emits admin chatter (`FW...`, `DIAG...`, `LINK...`, `link`
acks, `OK`, `ERROR`); the driver ignores it. Any other unknown prefix is
logged once per tag - if you see "firmware protocol may be newer than this
driver", the firmware grew a frame this parser does not know yet.

Example lines (captured live):
```
s 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0095 -0.0033 0.0020 0.0048 1.0000 -0.0418 -0.0636 9.9448 -0.0007 -0.0008 -0.0047
b 10.83
m 0.9402 0.0672 -1.2045
r -1 -1 -1 -1 -1 -1 -1 -1 -1 -1        # transmitter off -> all channels -1
```

## Outbound (host -> ESP32)

```
v <speed_mps> <steering_deg>\n
```
Sent from `controller.motor_callback`, mapped from the normalized `/motor`
command: `speed_mps = clamp(speed,-1,1) * max_speed_mps`, `steering_deg =
clamp(steering_angle,-1,1) * max_steering_angle_deg + steering_trim_deg`. The
ESP32 firmware fails over to direct RC control on its own serial timeout, so when
the ROS pipeline stops streaming `v` commands the car reverts to the transmitter.

## IMU axes

`/imu/fused` and `/mag` are published in REP-103 body axes: x forward, y left,
z up. The controller applies `out[i] = sign[i] * in[order[i]]` to the
firmware's accel, gyro and magnetometer values, with the
`imu.accel_gyro_axis_order` / `_sign` and `imu.mag_axis_order` / `_sign`
parameters in `config/controller.yaml`. The shipped values are the identity:
the firmware's axes are already REP-103 on the NeoRacer. Read on the dev car
on 2026-11-12, `/imu/fused`:

| Pose | accel x | accel y | accel z | Expected |
|---|---|---|---|---|
| Level, still | -0.03 | -0.06 | +10.00 | z up, about +9.81 |
| Nose down, about 27 deg | -4.50 | -0.01 | +8.97 | x negative |
| Left side down | -0.84 | -4.92 | +8.36 | y negative |
| Turning left, seen from above | | | | gyro z positive: read up to +0.45 rad/s |

Gravity reads 9.7 to 10.0 m/s^2 across these poses, about 2 percent above
9.81 when level; no accelerometer scale is applied. The magnetometer's axes
have not been checked yet. A board mounted another way is fixed in the car
profile with no code change for `/imu/fused` and `/mag`; the firmware's
`/odom` yaw stays in the board's own axes. A non-identity accel/gyro remap marks the
firmware's orientation unknown (`orientation_covariance[0] = -1`), because the
firmware fuses it in the board's own axes.

## FlySky RC -> `/joy` mapping

The 10 `r` channels are converted to a Joy message that satisfies three
consumers at once - `gamepad_node` (reads `axes[1]`/`axes[3]`), `mux_node`
(reads `buttons[4]`/`buttons[5]`, treats `axes[2,5]` as resting at +1.0), and the
student controller API (Xbox-style layout). Everything is parameterized in
`config/controller.yaml`:

- `rc_min/center/max` (1000/1500/2000 us) + `rc_deadband` calibrate each channel
  to `[-1, 1]`; a reading below `rc_failsafe_below` (e.g. `-1`) -> neutral.
- `throttle_channel` -> `axes[1]`, `steering_channel` -> `axes[3]`.
- `mode_channel` is a 3-position switch: low -> idle, mid -> manual (`buttons[4]`),
  high -> autonomy (`buttons[5]`).

**The channel indices are transmitter-mixer dependent and must be confirmed on
the car**: start the controller, `ros2 topic echo /joy` while moving each
stick/switch, and adjust the `*_channel` parameters until throttle, steering, and
the mode switch land on the right axes/buttons.
