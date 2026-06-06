# ESP32 (OSRbot) serial protocol

The Seeed OSRbot ESP32 (`/dev/osrbot_base`) is a native USB-CDC device, so the
serial baud rate is irrelevant - data streams regardless of the configured baud.
The `controller` node owns the link; the parsing/mapping lives in
`neoracer_ros2_driver/controller_lib.py` (pure, unit-tested) and the node wiring
in `controller.py`.

## Inbound (ESP32 -> host), newline-terminated ASCII

| Prefix | Format | Meaning | Published as |
|--------|--------|---------|--------------|
| `i` | `i qx qy qz qw ax ay az gx gy gz` | IMU: quaternion, accel (m/s^2), gyro (rad/s) | `/imu` (sensor_msgs/Imu) |
| `o` | `o px py pz vx vy vz yaw` | odometry: position (m), velocity (m/s), yaw (rad) | `/odom` (nav_msgs/Odometry) |
| `r` | `r c1 c2 ... c10` | FlySky RC channels (ints; `-1` = no signal) | `/joy` (sensor_msgs/Joy) |
| `m` | `m x y z` | magnetometer (Gauss) | `/mag` (optional, off by default) |

Example lines (captured live):
```
i -0.0072 -0.0061 0.7560 0.6545 -0.0179 -0.1542 9.8467 0.1297 0.0441 -0.0715
o 5.411874 7.240749 0.000000 0.000000 0.000000 0.000000 1.482815
m 0.4330 -0.0040 0.0290
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
