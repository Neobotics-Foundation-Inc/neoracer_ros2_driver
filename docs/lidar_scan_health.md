# Lidar /scan health

The LakiBeam keeps its own settings (laser on/off, scan window, motor speed) in
onboard flash, editable through its web panel at http://192.168.8.2. The driver
asserts the launch-file values (laser on, scan 45-315, 30 Hz) over the sensor's
REST API at every start, so a stray web-panel change cannot outlive a teleop
restart. The sensor's own OS takes about 65 s after power-on to serve http, so
at cold boot the driver waits up to 60 s and, if it still misses, retries every
10 s once scan packets flow. The `filter` setting is panel-managed on purpose
and not pushed.

## Reading the symptom

| /scan behavior | Meaning |
|----------------|---------|
| Streams, every range (or all but a few) `.inf` | Packets are arriving with zero-distance returns. The battery is sagging, everything in view is closer than the 10 cm minimum range, or the scan window is degenerate. Not a connection problem. |
| Silent, no new messages | No packets. The usb0 link or lidar power is down, or the laser is disabled: `laser_enable: false` stops the motor and the stream entirely (measured on hardware). |

The driver logs `[scan-watchdog]` at ERROR after 3 s of 100% inf ranges and
includes the sensor's live state. Check it first:

```
journalctl -u neoracer-teleop -b | grep scan-watchdog
```

## Do not trust a bare topic echo

`ros2 topic echo /scan` truncates arrays to their first 128 values, and index 0
points straight backwards into the always-blank rear crop. A perfectly healthy
scan therefore prints a wall of `.inf` and then `'...'` - anyone eyeballing the
echo will report "all inf" on a working lidar. Count instead of looking:

```
ros2 topic echo /scan --once --full-length | grep -c -- '- \.inf'
```

Roughly 460 is healthy (rear crop + out of range); 1440 is blind. RViz with
Fixed Frame `laser` is the honest visual check.

## The 60-second check

```
curl http://192.168.8.2/api/v1/sensor/overview
```

- `laser_enable: false` (topic silent) or a degenerate `scan_range` (topic
  streams inf): someone changed the web panel. Restart teleop; the driver
  re-asserts the correct values.
- `laser_enable: true`, `motor_rpm` near 1760, ranges still inf: physical.
  Wave a hand 30 cm from the sensor. If that sector comes alive, clean the
  window and check the battery. If nothing changes, check the battery first;
  a sagging pack keeps the motor and comms alive while the laser stops
  returning.
- No HTTP response: usb0 link or lidar power is down. /scan will be silent,
  not inf.
