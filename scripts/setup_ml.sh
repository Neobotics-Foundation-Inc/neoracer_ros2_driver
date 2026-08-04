#!/bin/bash
# GPU stack for model training and deployment: PyTorch/torchvision built for
# Tegra, Ultralytics YOLO, and the ONNX tooling that feeds the TensorRT export
# path. TensorRT itself ships with JetPack; this script verifies it rather than
# fetching it.
#
# Idempotent: re-runs skip anything already at the pinned version.
# Target: Jetson Orin Nano, JetPack 6.x (L4T R36), CUDA 12.6, Python 3.10.
set -eo pipefail

# torchvision's CUDA extensions (NMS, ROI align) compile against a specific
# torch ABI, so the pair moves together. Both come from the jetson-ai-lab
# index below; the versions listed there are what constrains this choice.
TORCH_VERSION="2.8.0"
TORCHVISION_VERSION="0.23.0"

# numpy 2 changed the C ABI. ROS2 Humble's binary extensions (cv_bridge,
# sensor_msgs_py) are built against numpy 1, so a numpy-2 upgrade pulled in by
# ultralytics would break every node that touches an image or a point cloud.
NUMPY_SPEC="numpy>=1.23,<2"

# ---------------------------------------------------------------------------
# Platform probes.
# ---------------------------------------------------------------------------
if [ ! -f /etc/nv_tegra_release ]; then
    echo "ERROR: /etc/nv_tegra_release not found; this phase targets Jetson (L4T)." >&2
    echo "       On a non-Tegra machine, install PyTorch from pytorch.org instead." >&2
    exit 1
fi

L4T_MAJOR="$(awk -F'R| \\(release' '{print $2; exit}' /etc/nv_tegra_release)"
case "$L4T_MAJOR" in
    36) JP_DIR="jp6" ;;
    *)  echo "ERROR: unsupported L4T major release R$L4T_MAJOR (expected R36 / JetPack 6.x)." >&2
        exit 1 ;;
esac

# The wheel index is keyed by CUDA version, not JetPack version: jp6/cu126,
# jp6/cu128, jp6/cu129. Read the toolkit version off the CUDA install rather
# than assuming, since JetPack point releases move it.
CUDA_ROOT="/usr/local/cuda"
if [ ! -e "$CUDA_ROOT/version.json" ]; then
    echo "ERROR: no CUDA toolkit at $CUDA_ROOT; reflash JetPack with CUDA selected." >&2
    exit 1
fi
CUDA_VERSION="$(python3 -c "
import json
v = json.load(open('$CUDA_ROOT/version.json'))['cuda']['version'].split('.')
print(v[0] + '.' + v[1])
")"
CUDA_TAG="cu${CUDA_VERSION/./}"

TORCH_INDEX="https://pypi.jetson-ai-lab.io/${JP_DIR}/${CUDA_TAG}/+simple/"
echo "  L4T R$L4T_MAJOR, CUDA $CUDA_VERSION; wheel index $TORCH_INDEX"

# ---------------------------------------------------------------------------
# TensorRT (JetPack-supplied).
# ---------------------------------------------------------------------------
# There is no Tegra TensorRT wheel on PyPI; the aarch64 wheels published there
# are for datacenter GPUs and will not load on an iGPU. The bindings come from
# the apt package python3-libnvinfer, which JetPack installs and which lives in
# /usr/lib/python3.10/dist-packages - still on sys.path under a --user install.
if ! python3 -c "import tensorrt" >/dev/null 2>&1; then
    echo "  TensorRT bindings missing; installing from the JetPack apt repo"
    sudo apt-get install -y nvidia-tensorrt python3-libnvinfer
fi
if ! python3 -c "import tensorrt" >/dev/null 2>&1; then
    echo "ERROR: 'import tensorrt' still fails after apt install." >&2
    echo "       Check that the JetPack apt source (repo.download.nvidia.com) is enabled." >&2
    exit 1
fi
echo "  TensorRT $(python3 -c 'import tensorrt; print(tensorrt.__version__)')"

# ---------------------------------------------------------------------------
# pip.
# ---------------------------------------------------------------------------
# Per-user install, matching setup_jupyter.sh: the systemd units all run as this
# user, so ~/.local is on their sys.path, and nothing here has to fight apt for
# /usr/lib/python3/dist-packages.
#
# Jammy ships pip 22.0.2, which rejects wheels carrying Metadata-Version 2.4
# (ultralytics and several of its dependencies do). Upgrade in the user site
# before resolving anything. Invoke as `python3 -m pip` throughout so the
# upgraded copy is the one that runs, whatever `pip3` resolves to on PATH.
PIP_FLAGS=(--user)
if python3 -m pip install --help 2>/dev/null | grep -q -- '--break-system-packages'; then
    PIP_FLAGS+=(--break-system-packages)
fi

if ! python3 -c "
import sys
from pip import __version__ as v
sys.exit(0 if tuple(int(p) for p in v.split('.')[:2]) >= (23, 1) else 1)
" >/dev/null 2>&1; then
    echo "  upgrading pip (Jammy's 22.0.2 cannot read Metadata-Version 2.4 wheels)"
    python3 -m pip install "${PIP_FLAGS[@]}" -U pip
fi

# ---------------------------------------------------------------------------
# numpy.
# ---------------------------------------------------------------------------
# apt ships 1.21.5, below ultralytics' floor of 1.23. Pull the last 1.x into the
# user site, where it shadows the apt copy without removing it.
if ! python3 -c "
import sys
from importlib.metadata import version
major, minor = (int(p) for p in version('numpy').split('.')[:2])
sys.exit(0 if (1, 23) <= (major, minor) and major < 2 else 1)
" >/dev/null 2>&1; then
    python3 -m pip install "${PIP_FLAGS[@]}" "$NUMPY_SPEC"
fi

# ---------------------------------------------------------------------------
# PyTorch + torchvision.
# ---------------------------------------------------------------------------
# PyPI's aarch64 torch wheels are CPU-only or target sbsa (Grace/datacenter);
# neither drives the Orin's integrated GPU. The jetson-ai-lab index carries the
# Tegra builds.
#
# Two steps rather than one install: the download names that index as the sole
# source, so a same-named PyPI release cannot win the resolve, and --no-deps
# keeps the resolve to the two wheels. Installing the downloaded files then
# pulls their dependencies (sympy, networkx, fsspec, ...) from PyPI, which is
# where those belong.
TORCH_OK=0
python3 -c "
import sys, torch
sys.exit(0 if torch.__version__.split('+')[0] == '$TORCH_VERSION' and torch.cuda.is_available() else 1)
" >/dev/null 2>&1 && TORCH_OK=1

if [ "$TORCH_OK" -eq 0 ]; then
    echo "  installing torch $TORCH_VERSION + torchvision $TORCHVISION_VERSION (~230 MB)"
    WHEEL_DIR="$(mktemp -d)"
    trap 'rm -rf "$WHEEL_DIR"' EXIT
    python3 -m pip download --no-deps -d "$WHEEL_DIR" --index-url "$TORCH_INDEX" \
        "torch==$TORCH_VERSION" "torchvision==$TORCHVISION_VERSION"
    python3 -m pip install "${PIP_FLAGS[@]}" "$NUMPY_SPEC" "$WHEEL_DIR"/*.whl
    rm -rf "$WHEEL_DIR"
    trap - EXIT
fi

# ---------------------------------------------------------------------------
# Ultralytics + ONNX export tooling.
# ---------------------------------------------------------------------------
# onnx and onnxslim are ultralytics' [export-base] extra, minus the pieces that
# have no aarch64 wheel (openvino, nncf). They are what `YOLO.export(
# format='engine')` runs before handing the graph to TensorRT.
#
# ultralytics pulls opencv-python because its floor is 4.6 and apt's cv2 is
# 4.5.4. The pip build shadows the apt one in this user's sys.path; cv_bridge
# links its own libopencv 4.5.4 in C++ and is unaffected, and the camera node's
# V4L2 capture path is present in the pip build.
ML_DEPS=(ultralytics onnx onnxslim)
MISSING_DEPS=()
for dep in "${ML_DEPS[@]}"; do
    python3 -c "import importlib; importlib.import_module('$dep')" >/dev/null 2>&1 \
        || MISSING_DEPS+=("$dep")
done
if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo "  installing ${MISSING_DEPS[*]}"
    python3 -m pip install "${PIP_FLAGS[@]}" "$NUMPY_SPEC" "${MISSING_DEPS[@]}"
fi

# ---------------------------------------------------------------------------
# Verification.
# ---------------------------------------------------------------------------
python3 - <<'PY'
import sys

import numpy, torch, torchvision, tensorrt, ultralytics

if not torch.cuda.is_available():
    sys.exit("ERROR: torch.cuda.is_available() is False; the GPU build did not take.")

# Exercises the compiled torchvision CUDA extension, which is what a
# CPU-only wheel masquerading as a GPU one fails on.
boxes = torch.tensor([[0., 0., 10., 10.], [1., 1., 11., 11.]], device="cuda")
torchvision.ops.nms(boxes, torch.tensor([0.9, 0.8], device="cuda"), 0.5)

print(f"  torch {torch.__version__} on {torch.cuda.get_device_name(0)} "
      f"(sm_{''.join(str(c) for c in torch.cuda.get_device_capability(0))}), "
      f"CUDA {torch.version.cuda}, cuDNN {torch.backends.cudnn.version()}")
print(f"  torchvision {torchvision.__version__}, numpy {numpy.__version__}")
print(f"  ultralytics {ultralytics.__version__}, tensorrt {tensorrt.__version__}")
PY

echo "GPU stack ready. Export a YOLO model to a TensorRT engine with:"
echo "  yolo export model=yolo11n.pt format=engine half=True imgsz=640 device=0"
