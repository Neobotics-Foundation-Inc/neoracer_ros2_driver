# Models

Drop point for the weights `inference_node` loads. `config/inference.yaml` sets
`model_path`; a bare filename is looked up here first (in the installed
`share/neoracer_ros2_driver/models/`), then in the working directory, and
finally handed to Ultralytics, which fetches its own stock names into a cache.

`.pt` weights are committed, so a fresh clone can run inference without a
download. The `.onnx` export intermediate and the per-car `.engine` are not:
`.gitignore` excludes both, and an engine only runs on the board that built it.

A stock name that is not here downloads on first run into the process's working
directory, which under systemd is whatever the unit happens to start in. Fetch
it once into this directory instead, so every later start finds the same file:

```sh
cd ~/ros2_ws/src/neoracer_ros2_driver/models
python3 -c "from ultralytics import YOLO; YOLO('yolo26n.pt')"
```

## TensorRT export

An engine is built from a `.pt` on the car that will run it, against the
TensorRT and GPU it will run on. It is not portable to another board, another
JetPack, or another Ultralytics version, so rebuild after any of those move.

```sh
racecar service stop
racecar compile
```

With no argument, `racecar compile` exports the `model_path` in
`config/inference.yaml` at the `imgsz` set there, and leaves the `.engine` in
this directory. Name a file to export something else (`racecar compile
custom.pt`); `--imgsz=`, `--device=`, and `--no-half` override the export
settings, and `--force` rebuilds over an existing engine.

The build peaks near 3 GB of the Orin Nano's 8 GB shared memory, which is why it
refuses to start while the stack is running. Point `model_path` at the
`.engine` afterwards and rebuild the workspace so the file lands in `share/`.

By hand, without the CLI wrapper:

```sh
yolo export model=yolo26n.pt format=engine half=True imgsz=640 device=0
cp yolo26n.engine ~/ros2_ws/src/neoracer_ros2_driver/models/
```

## Custom classes

A model trained on your own dataset is loaded the same way. Ultralytics stores
class names inside the weights, so `/detections` carries real labels with no
separate labels file to keep in sync.
