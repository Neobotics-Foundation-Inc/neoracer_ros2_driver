# Lab dashboards

Internals of the seven lab dashboards in `scripts/dashboards/`: wallfollow
(8081), camlabel (8082), pursuit (8083), eps (8084), smartfollow (8085),
linefollow (8086) and webteleop (8087). Each one's README describes its page
and parameters. Install and use: README, "Lab dashboards". Unit install:
`scripts/setup_dashboards.sh`.

webteleop, linefollow, wallfollow and camlabel are upstream's v0.9.0
dashboards, which began as copies of the NeoRacer ones, adapted to the
NeoRacer's camera, lidar and display. pursuit, eps and smartfollow are the
NeoRacer's own; upstream retired them in 0.8.1.

## Contents

- [Shared structure](#shared-structure)
- [Tuning and logs](#tuning-and-logs)
- [Lidar convention](#lidar-convention)
- [Units](#units)
- [Cars set up before the move](#cars-set-up-before-the-move)
- [webteleop](#webteleop)
- [linefollow](#linefollow)
- [wallfollow](#wallfollow)
- [camlabel, pursuit, eps and smartfollow](#camlabel-pursuit-eps-and-smartfollow)
- [Tests](#tests)

## Shared structure

Each dashboard is one Python process: a ROS node plus a stdlib HTTP server,
with the whole browser side in one HTML file.

| Piece | Responsibility |
|---|---|
| `<name>.py` | The node (subscriptions, the `/drive` timer), the HTTP handler, and pure helpers the tests reach without a node |
| `<name>.html` | The page: controls, canvases, charts; polls `GET /state` |
| `<name>.yaml` | Shipped defaults for every tunable |
| `neoracer-<id>.service.in` | Unit template; `@DIR@` is substituted at install |
| `README.md` | The page, the parameters and the safety notes |

Every page loads the shared NeoRacer styles, fonts, logos, charts and theme
from `/ui/`, which each server answers through `scripts/webui.py`. Each page
has the Dark and Light themes, and `lost.js` covers a page whose server has
stopped with a link back to the apps page on 8080.

`/drive` comes from a timer, never from a browser post. In webteleop a
command older than `cmd_timeout` reads as released, so a closed tab, a
dropped link, or a page that lost focus with a key down coasts to a stop
instead of driving the last command forever.

Steering is negated on publish: the pages and controllers use the student
convention, positive right, and positive on `/drive` turns the NeoRacer left
(REP-103, verified on the car). Speed on `/drive` is capped at 1.0 in every
dashboard. The NeoRacer mux forwards `/drive` with no software deadman; the
handset's mode switch is the physical gate, and every shipped yaml has its
speed at 0.0. With the handset off or out of range the ESP32 follows `/drive`
too, so the handset stays on while a dashboard drives.

Three threads share the state: the ROS executor, the HTTP request threads, and
the main thread in `serve_forever`. Everything shared sits behind one `_lock`,
never held across an encode or a network write. `rclpy.init()` installs signal
handlers that stop ROS but leave `serve_forever` blocked, so `main` takes the
signals back and calls `shutdown` off the serving thread.

## Tuning and logs

Tuning loads the shipped `<name>.yaml`, then `dashboards/<name>.yaml` in the
car profile (`~/.config/racecar`, `neoracer_ros2_driver/car_profile.py`) on
top when it exists. Save writes only the profile file (seeded from the
defaults on the first save) and rewrites its numbers in place so the comments
survive. The file it replaces goes to the profile's `history/`. The shipped
file stays unchanged, so a driver update keeps a car's tuning. smartfollow's
`wall` and `car` blocks and eps's `nominal` block merge key by key.

The names are the scripts' own: webteleop's file is `dashboards/teleop.yaml`.
camlabel has no Save; its preview settings take the profile's
`dashboards/camlabel.yaml` on top in the same way.

CSV logs go to `dashboards/<name>/` under the data root (`~/neoracer_data`,
`neoracer_ros2_driver/data_root.py`) and are listed and downloaded through
the page. eps keeps its ledger there too, and camlabel's dataset is
`datasets/camlabel/` under the data root.

## Lidar convention

`LIDAR_MOUNT_YAW_DEG` is the raw scan angle that points at the car's nose:

```
student_deg = LIDAR_MOUNT_YAW_DEG - raw_deg
```

The NeoRacer's LakiBeam faces forward over 360 degrees at 1440 points, with
the rear 90 degrees always blank (inf). Its measured mount yaw is
`laser_yaw_deg` in `config/frames.yaml`, with the car profile's
`params/frames.yaml` on top, where the lidar calibration writes it; positive
turns left. A ray at raw angle r points at r + laser_yaw in base_link, so the
nose sits at raw -laser_yaw, and wallfollow and webteleop set the constant to
that when they start, reading frames.yaml the way the 8080 overview does
(`scripts/overview.py`). The fleet value is 0, which is the plain negation the
NeoRacer dashboards always used. Upstream's RPLIDAR faces aft, which is why
upstream ships 180 here.

Indices and angles come from each message's `angle_min` and `angle_increment`
and wrap modulo the point count, which is right for a full turn; the blank
rear rays read as open space. eps and smartfollow keep the plain negation,
which is the same at the fleet yaw. linefollow, camlabel and pursuit do not
read the lidar.

## Units

`setup_dashboards.sh` renders each `neoracer-<id>.service.in`, substituting
its folder for `@DIR@`, and installs it to `/etc/systemd/system/`. The
renderer checks for `@DIR@` and `Environment=HOME=` first and does not
install a template that has changed shape. A first install is stopped and disabled; an
installed unit keeps its enable state, and one that is running when its unit
file changes is restarted on the new file. On a car set up before the
dashboards moved into the driver, setup disables each dashboard whose old
clone it moves aside, because the old setup enabled it at boot. Upstream's
discovery range line is not added. `setup_services.sh` runs the script, so
`racecar service install` and `setup_all.sh` install the dashboards with no
network. Set `RACECAR_DASHBOARDS=0` to skip them.

## Cars set up before the move

Before the dashboards moved into the driver, `setup_services.sh` cloned each
one's repository to `scripts/dashboards/<repo>_dashboard/`
(`wallfollow_dashboard`, `camlabel_dashboard`, `pursuit_dashboard`,
`eps_dashboard`, `smartfollow_dashboard`, `linefollow_dashboard`,
`teleop_dashboard`), and each dashboard saved its tuning into its own yaml in
that checkout. The in-tree folders have other names, so a driver update
(`git reset --hard` in `racecar update`, or a branch switch) leaves those
checkouts and their tuning as they are; they stay gitignored and outside the
lint.

Once every unit has installed, `setup_dashboards.sh` runs
`scripts/dashboards/checkouts.py`. The units go first, so no unit is left
pointing at a checkout that has moved: when an install fails, the old
checkouts stay where they are, and the old units still run them, until the
next run. For each old checkout, `checkouts.py`:

1. Copies the values the car tuned into the car profile's
   `dashboards/<name>.yaml`, written over the new shipped defaults so new
   keys and comments come along. A tuned value is a number that differs from
   the yaml the checkout last took from its repository: the merge base with
   its upstream branch, so a commit made on the car still counts as tuning,
   or the new defaults when the checkout has no git history. A spelled-out
   wallfollow `mode` counts as its number. Text values such as topics are
   named in the output, not copied. A profile file that already exists is
   left alone, and the output says so; so is a yaml that cannot be read.
2. Moves `logs/*.csv`, and eps's `ledger*.jsonl` and `.init_set`, to
   `dashboards/<name>/` under the data root, never over a file already there.
   eps's ledger stays with the checkout when the profile already had its
   own eps.yaml, because the ledger's gains are relative to the old
   starting point.
3. Moves the checkout to `scripts/dashboards/.migrated/`.

It also moves camlabel's dataset from `~/data/camlabel/`, where the old
camlabel kept it and where the installer's erase list reaches, to
`datasets/camlabel/` under the data root, but only while the data root holds
no dataset: two datasets are never merged, because their image numbers would
collide. Nothing is deleted, a checkout that cannot be moved is left in place
and reported, and a second run finds nothing to move. The old units have the
same `neoracer-<id>` names, so the new units replace them and keep their
enable state.

The driver does not use `scripts/dashboards/.migrated/` after the move. The
folder is kept because an old checkout can hold something the steps above do
not carry, such as a code change. Once the car profile and the data root hold
what you need, you can delete it.

## webteleop

Drive from buttons or the keyboard, with the camera, the lidar, the encoder,
the IMU, the battery and the compass in view, and a text box for the dot
matrix.

```
/camera/color ─▶ _image_cb ──▶ resize ──▶ JPEG ──▶ _preview ──────▶ GET /frame

/edgetpu/inference ─▶ _det_cb ──▶ normalize against _state['res'] ─┐
/scan ─────────────▶ _scan_cb ─▶ _student_deg, thin by 3 ─────────┤
/odom ─────────────▶ _odom_cb ────────────────────────────────────┼─▶ _state ─▶ GET /state
                                                                   │
browser buttons/keys ─▶ POST /cmd ─▶ _cmd ─┐                       │
                                            ├─▶ _actuate (15 Hz) ──┘
                            tuning ─────────┘         │
                                                      ▼
                                              /drive  AckermannDriveStamped
```

- The camera node publishes the USB camera's own JPEG frames; the NeoRacer
  has no depth stream, so upstream's depth preview is not here.
- Detections leave as fractions of the frame and are scaled in the browser to
  the drawn preview. They expire after `DET_TIMEOUT_S`, so a stale box never
  sits on a live frame. Inference is off at boot, so the overlay is empty
  until detections are turned on.
- `/imu/fused` and `/mag` are raw subscriptions that keep only the newest
  message; `_actuate` decodes them at 15 Hz into `_sense` (10 s of
  acceleration, angular velocity, and field) and the state. The
  magnetometer's axes have not been checked on a car yet, so the compass
  heading is to be confirmed.
- `/battery/state` arrives at about 0.5 Hz (`build_battery_state` in `controller.py`). The NeoRacer has no current sensor and
  publishes current as NaN; `battery_reading` turns it into None, because
  JSON cannot carry NaN and the page would stop updating.
- POST /text records text for the 8x8 dot matrix, and `_actuate` sends it on
  `/dotmatrix/text` once. The panel holds and scrolls the last text, so
  sending it again would restart the scroll. The NeoRacer's matrix takes no
  pixels and the car has no LED strip, so upstream's pixel and LED editors
  are not here.
- Charts are drawn by the shared `/ui/charts.js`.

## linefollow

Follow a colored line with the camera; HSV thresholds tuned in the page.

```
/camera/color ─▶ _image_cb ─▶ resize to process_width ─▶ HSV ─▶ inRange(low, high)
                                                                     │
                                       ┌─────────────────────────────┤
                                       ▼                             ▼
                              crop to the crop area         mask preview JPEG
                                       │                             │
                        largest contour above min_area               ▼
                                       │                        GET /frame/mask
                              its center column
                                       │
                         error = (center - width/2) / (width/2)   -1 .. +1
                                       │
                    ┌──────────────────┴───────────────────┐
                    ▼                                      ▼
        steer = angle_kp * error                 annotated color JPEG
              + angle_kd * d(error)                        │
        speed = speed - speed_kp * |error|                 ▼
                    │                           GET /frame/color
                    ▼
          _actuate (15 Hz) ─▶ /drive  AckermannDriveStamped

/odom ─▶ _odom_cb ─▶ measured speed ─▶ _state ─▶ GET /state
```

- With no contour in the crop area the last command is held, speed
  included, but never above the speed slider, so the page's Stop also stops
  a car that has lost the line. Stop also drops the held throttle, so a
  slider raised again leaves the car still until the line is back.
- The mask preview shows the whole frame, not only the crop area: a threshold can
  only be judged against everything the camera sees.
- The offset chart's axis is fixed at -1 to 1, so a small wobble looks small.

## wallfollow

Follow walls with the lidar; two controllers, chosen by `mode` in the yaml (no
switch in the page, since they read the tune panel differently).

```
                          tuning  mode: 0 static | 1 dynamic
                                    │
/scan ─▶ _scan_cb ─▶ _student_deg ──┤
                                    │
      ┌─────────────────────────────┴───────────────────────────┐
      ▼                                                         ▼
   STATIC                                                    DYNAMIC
   _mean_at(+look_angle, window)          candidate headings every SEARCH_STEP
   _mean_at(-look_angle, window)          across +/- width of the nose
      │                                              │
   error = right - left  (setpoint 0)      obstacles inflated by HALF_CLEAR
      │                                    score = clearance
      │                                          - CENTER_BIAS * |deg|
      │                                          + side_weight * deg
      │                                              │
      │                                    error = best heading
      └──────────────────────┬───────────────────────┘
                             ▼
        steer = kp * error + kd * d(error)          overlay ─▶ _state
        speed = slider, bent toward a road-shaped           │
                target by speed_kp and speed_kd             ▼
                             │                        GET /state
                             ▼
                  /drive  AckermannDriveStamped

/odom ─▶ _odom_cb ─▶ measured speed ─▶ _state
```

- Distances are capped at `lookahead`, so an open doorway reads as `lookahead`
  rather than 12 m of empty room and does not dominate the score.
- The lidar view draws the rays the controller used and the heading it chose,
  so a wrong turn is diagnosed by seeing which ray it used.

## camlabel, pursuit, eps and smartfollow

- camlabel collects camera frames and labels them into a YOLO dataset under
  the data root; it only reads the camera, so it runs alongside a driving
  dashboard.
- pursuit chases the largest confident detection with PD steering and a
  throttle that brakes toward `safety_area`, and keeps its last command when
  the target is lost, never above the speed slider, so a slider at 0 stops
  the car at once. A slider at 0 also drops the held throttle, so a slider
  raised again leaves the car still until a target is seen.
- eps learns the dynamic gap follower's four gains with a (1+1) evolution
  strategy from one Success or Fail press per run; its ledger resumes a
  session after a restart.
- smartfollow runs wallfollow's dynamic gap follower and pursuit's car
  follower side by side, with an arbiter that hands the car to the car
  follower while a target was seen within `target_hold`.

Their READMEs have the details.

## Tests

`test/test_dashboard_teleop.py`, `test_dashboard_linefollow.py`,
`test_dashboard_wallfollow.py` (upstream's, adapted to the NeoRacer),
`test_dashboard_camlabel.py`, `test_dashboard_pursuit.py`,
`test_dashboard_eps.py` and `test_dashboard_smartfollow.py` load each module
by name through `load_script` and need no ROS graph, camera, or browser.
`test/test_setup_dashboards.py` runs `setup_dashboards.sh` against a
temporary unit folder and `checkouts.py` against old checkouts built in a
temporary folder.
