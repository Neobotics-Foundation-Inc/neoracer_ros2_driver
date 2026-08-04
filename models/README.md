# Models

Drop point for the weights `inference_node` loads. `config/inference.yaml` sets
`model_path`; a bare filename is looked up here first (in the installed
`share/neoracer_ros2_driver/models/`), then in the working directory, and
finally handed to Ultralytics, which fetches its own stock names into a cache.

Weights are not committed. `.pt` files are large and `.engine` files are built
per car; `.gitignore` excludes both.

A stock name that is not here downloads on first run into the process's working
directory, which under systemd is whatever the unit happens to start in. Fetch
it once into this directory instead, so every later start finds the same file:

```sh
cd ~/ros2_ws/src/neoracer_ros2_driver/models
python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
```

## TensorRT export

An engine is built from a `.pt` on the car that will run it, against the
TensorRT and GPU it will run on. It is not portable to another board, another
JetPack, or another Ultralytics version, so rebuild after any of those move.

```
yolo export model=yolo11n.pt format=engine half=True imgsz=640 device=0
cp yolo11n.engine ~/ros2_ws/src/neoracer_ros2_driver/models/
```

The build peaks near 3 GB of the Orin Nano's 8 GB shared memory; stop the teleop
stack first (`racecar service stop`). Point `model_path` at `yolo11n.engine`
afterwards and rebuild the workspace so the file lands in `share/`.

## Custom classes

A model trained on your own dataset is loaded the same way. Ultralytics stores
class names inside the weights, so `/detections` carries real labels with no
separate labels file to keep in sync.
