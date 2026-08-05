# Models

Drop point for the weights `inference_node` loads. `config/inference.yaml` sets
`model_path`; a bare filename is looked up here first (in the installed
`share/neoracer_ros2_driver/models/`), then in the working directory, and
finally handed to Ultralytics, which fetches its own stock names into a cache.

`.pt` weights and the TensorRT `.engine` are both committed, so a fresh clone
runs inference at full speed without a download and without an eight-minute
build. The `.onnx` export intermediate is not; it is reproducible from the `.pt`
and only exists on the way to an engine.

The committed engine is built for the fleet's stock configuration:

| | |
|---|---|
| GPU | Orin, compute capability 8.7 |
| TensorRT | 10.3.0.30 (CUDA 12.5) |
| JetPack | L4T R36.4.3 |

Every car in the fleet shares that configuration, which is why one engine
serves all of them. A car that has drifted from any row cannot deserialize the
file. Nothing breaks quietly when that happens: `inference_node` falls back to
`yolo26n.pt` beside it, logs a `WARN` naming the reason, and keeps publishing
at roughly 2.4x the latency until somebody runs `racecar compile` there.

A stock name that is not here downloads on first run into the process's working
directory, which under systemd is whatever the unit happens to start in. Fetch
it once into this directory instead, so every later start finds the same file:

```sh
cd ~/ros2_ws/src/neoracer_ros2_driver/models
python3 -c "from ultralytics import YOLO; YOLO('yolo26n.pt')"
```

## TensorRT export

An engine is built from a `.pt` against the TensorRT version and GPU it will
run on. It carries across cars that match the configuration in the table above,
which is why the fleet ships one. It does not carry across a different board, a
different JetPack, or a different Ultralytics version, so rebuild and recommit
after any of those move.

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
