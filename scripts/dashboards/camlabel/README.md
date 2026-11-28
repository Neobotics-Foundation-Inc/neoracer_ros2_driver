# Camera Label Dashboard

Image capture and labeling as a service for the NeoRacer, on port 8082. Students capture frames from the car's camera, drag boxes on them, and the dashboard saves a YOLO-ready dataset for model training.

## Contents

- [Service control](#service-control)
- [Workflow](#workflow)
- [Settings](#settings)
- [Dataset](#dataset)

## Service control

Installed with the driver (`racecar service install`, which runs `scripts/setup_dashboards.sh`), stopped and disabled.
Dashboard: `http://<car-ip>:8082`.

| Command | Does |
|---|---|
| `racecar service start camlabel` | start now |
| `racecar service stop camlabel` | stop |
| `racecar service enable camlabel` | start at every boot |
| `racecar service logs camlabel` | follow the log |

Camlabel only reads the camera, so it runs alongside a lab dashboard.

## Workflow

Collection and labeling are separate stages:

1. Collect: Snap (button or spacebar) saves the current full-resolution frame to the unlabeled pool. Take as many as you want, fast.
2. Label: click any thumbnail in the unlabeled section. Click one corner of a box, move, and click again to place it (on a touch screen, drag); the class field names each box, and Enter or Save labels finishes the image. The image moves from unlabeled to labeled and the next unlabeled image loads automatically. Prev, Next, or the arrow keys move through the unlabeled images. Delete image throws a bad shot away, saving with no boxes makes a negative example.
3. Labeled images show as thumbnails with a click-to-view of their boxes.

## Settings

`camlabel.yaml` next to the script holds the shipped live-preview resolution and jpeg quality sent to the browser (saved images are always the full camera frame). A car's own values go in the car profile, `~/.config/racecar/dashboards/camlabel.yaml`, which loads on top; a driver update leaves that file alone. Restart the service after editing. Camera and preview resolutions show in the summary.

## Dataset

Saved on the car in `datasets/camlabel/` under the data root (`~/neoracer_data`; `racecar data` prints it), outside the driver, so updating or reinstalling the driver leaves it alone. The tree is created at startup when missing:

- `unlabeled/uN.jpg`: collected frames waiting for labels (not part of the training set)
- `images/imgN.jpg`: the frame as captured, full camera resolution JPEG, no boxes burned in
- `labels/imgN.txt`: YOLO format, one line per box: `class_id cx cy w h`, normalized 0..1
- `classes.txt`: one class name per line, id = line number

This layout trains directly with ultralytics by pointing a data.yaml at it.

Before the dashboards moved into the driver, the dataset lived in `~/data/camlabel/`. `setup_dashboards.sh` moves it to the data root once, when the data root holds no dataset yet. If both hold files, both are left in place and the setup output says so, since image numbers from two datasets would collide.
