# Neoracer - On-Car Verification Checklist

Run after a fresh build (`racecar build`) to confirm the stack works end to end.
Power the wheels off the ground for the first drive test.

## 1. Build & unit tests
```sh
cd ~/ros2_ws && colcon build --symlink-install
colcon list                      # expect: neoracer_ros2_driver + lakibeam1
racecar test                     # controller_lib + linters green
```

## 2. Per-node smoke tests
- [ ] **ESP32 / controller** - `racecar launch controller`, then:
  ```sh
  ros2 topic hz /imu              # ~170 Hz
  ros2 topic echo --once /odom    # live position
  ros2 topic echo /joy            # move FlySky sticks/switches; see section 4
  ```
- [ ] **Lidar** - `racecar launch lidar`, then `ros2 topic hz /scan` (~30 Hz)
  and `ros2 topic echo --once /scan` -> `frame_id: laser`, ~1440 ranges.
  If unreachable: `ping 192.168.8.2`.
- [ ] **Camera** - `racecar launch camera`, then `ros2 topic hz /camera`.
  Confirm a student `get_color_image()` returns a decoded frame.
- [ ] **Display** - `racecar launch led_matrix`, then
  `ros2 topic pub --once /led_matrix/command std_msgs/String "{data: 'HELLO'}"`.

## 3. Full stack
```sh
racecar teleop
ros2 node list   # controller_node, gamepad_node, mux_node, throttle_node,
                 # camera_node, led_matrix_node, richbeam_lidar_node0
```
- [ ] No tracebacks in `~/logs/latest/`.

## 4. FlySky -> /joy mapping (tune on-car)
With the transmitter ON, `ros2 topic echo /joy` and confirm:
- [ ] throttle stick moves `axes[1]`, steering stick moves `axes[3]`
- [ ] `axes[2]` and `axes[5]` rest at +1.0
- [ ] the mode switch sets `buttons[4]` (manual) / `buttons[5]` (autonomy)

Adjust `throttle_channel` / `steering_channel` / `mode_channel` in
`config/controller.yaml` until they line up.

## 5. Drive test (wheels off ground first)
- [ ] Flip the FlySky switch to **manual** -> stick drives the wheels
  (FlySky -> `/joy` -> gamepad -> mux -> throttle -> `/motor` -> ESP32).
- [ ] Flip to **autonomy** and publish `/drive` (or run a student script) -> wheels respond.
- [ ] Release the switch (idle) -> wheels stop.
- [ ] Verify drive **direction** and that top speed feels safe; tune
  `max_speed_mps` / `steering_trim_deg` (`config/controller.yaml`) and the caps
  in `config/throttle.yaml`.

## 6. Watchdog
```sh
racecar watchdog &               # or: racecar service start
pkill -f neoracer_ros2_driver/lib/.*/controller
```
- [ ] watchdog logs a restart and `/imu` resumes.

## 7. Services / dashboard / jupyter
```sh
racecar service install && racecar service start
racecar service status           # units active/enabled
```
- [ ] dashboard at `http://neoracer.local:8080` shows nodes green + Jetson temp
- [ ] jupyter at `http://<car-ip>:8888`

## 8. Networking (optional; reconfigures Wi-Fi - run wired/console)
```sh
racecar setup networking --show
racecar setup networking
```
- [ ] AP `neoracer-1` broadcasts; a client can reach the car's services
- [ ] `ip -br addr` shows the Ethernet static + the lidar `192.168.8.x`
