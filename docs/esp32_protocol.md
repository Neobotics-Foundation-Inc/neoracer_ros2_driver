# ESP32 (OSRbot) serial protocol

The Seeed OSRbot ESP32 (`/dev/osrbot_base`) is a native USB-CDC device, so the
serial baud rate is irrelevant - data streams regardless of the configured baud.
The `controller` node owns the link; the parsing/mapping lives in
`neoracer_ros2_driver/controller_lib.py` (pure, unit-tested) and the node wiring
in `controller.py`.

## Inbound (ESP32 -> host), newline-terminated ASCII

The cars run firmware `NEORACER_V1.2` (protocol 1.1, the same line formats as
V1.1). It streams one combined state frame; older firmware streamed IMU and
odometry as separate lines. The driver parses both.

| Prefix | Format | Meaning | Published as |
|--------|--------|---------|--------------|
| `s` | `s px py pz vx vy vz yaw qx qy qz qw ax ay az gx gy gz` | state @ ~200 Hz: odometry + IMU in one frame (V1.1) | `/odom` + `/imu/fused` + `/encoder/speed` |
| `b` | `b volts` | battery voltage @ ~0.5 Hz (V1.1) | `/battery` (sensor_msgs/BatteryState, with a 3S charge fraction) + `/battery/voltage` |
| `i` | `i qx qy qz qw ax ay az gx gy gz` | IMU: quaternion, accel (m/s^2), gyro (rad/s) (legacy, pre-V1.1) | `/imu/fused` (sensor_msgs/Imu) |
| `o` | `o px py pz vx vy vz yaw` | odometry: position (m), velocity (m/s), yaw (rad) (legacy, pre-V1.1) | `/odom` (nav_msgs/Odometry) + `/encoder/speed` |
| `r` | `r c1 c2 ... c10` | FlySky RC channels @ ~10 Hz, raw SBUS counts 0 to 2047 (`-1` on every channel = receiver failsafe) | `/joy` (sensor_msgs/Joy) + `/rc/channels` |
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
r -1 -1 -1 -1 -1 -1 -1 -1 -1 -1        # receiver failsafe -> all channels -1
```

## Outbound (host -> ESP32)

```
v <speed_mps> <steering_deg>\n
```
Sent from `controller.motor_callback`, mapped from the normalized `/motor`
command: `speed_mps = clamp(speed,-1,1) * max_speed_mps`, `steering_deg =
clamp(steering_angle,-1,1) * max_steering_angle_deg + steering_trim_deg`.

The firmware does not hand the car back to the transmitter when `v` lines stop.
If no `v` line arrives for 500 ms, it sets the target speed to 0 and centres the
steering, and the car coasts to a stop. Who drives is decided by the handset's
mode switch alone (radio Ch7, SBUS index 6):

- **Switch up, or the middle position** (a count below 1500): manual. The ESP32
  drives from the sticks and ignores every `v` line.
- **Switch down** (1500 or more): autonomy. The ESP32 follows the `v` lines.
- **Handset off or out of range with the switch up**: after about 200 ms without
  SBUS frames, or on receiver failsafe, the ESP32 hands the car to the `v` lines,
  and the `r` line reads `-1` on every channel.
- **Handset off or out of range with the switch down**: the loss is not detected.
  The `r` line repeats the last values, and the ESP32 keeps following `v` lines.

## FlySky RC -> `/joy` mapping

The 10 `r` channels are converted to a Joy message that satisfies three
consumers at once - `gamepad_node` (reads `axes[1]`/`axes[3]`), `mux_node`
(reads `buttons[4]`/`buttons[5]`, treats `axes[2,5]` as resting at +1.0), and the
student controller API (Xbox-style layout). Everything is parameterized in
`config/controller.yaml`:

- `rc_min/center/max` (240/1023/1807, raw SBUS counts measured on the stock
  handset) + `rc_deadband` calibrate each channel to `[-1, 1]`; a reading below
  `rc_failsafe_below` (0, so only the firmware's `-1`) -> neutral. Switches read
  240, 1023 or 1807, so `-1.0`, `0.0` or `1.0` on `/rc/channels`.
- `throttle_channel` (2) -> `axes[1]`, `steering_channel` (0) -> `axes[3]`.
- `mode_channel` (6, the mode switch) follows the firmware's two states: switch
  up or middle -> manual (`buttons[4]`), switch down -> autonomy (`buttons[5]`).
  The mux runs with `autonomy_passthrough` on, so these buttons do not gate
  driving; the firmware does.

**The channel indices are transmitter-mixer dependent and must be confirmed on
the car**: start the controller, `ros2 topic echo /joy` while moving each
stick/switch, and adjust the `*_channel` parameters until throttle, steering, and
the mode switch land on the right axes/buttons.
