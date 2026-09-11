#!/usr/bin/env bash
set -Eeuo pipefail

set +u
source /opt/ros/humble/setup.bash
source /opt/damgc_survivor_ws/install_survivor/setup.bash
set -u

python3 - <<'PY'
import torch
import torchvision
import ultralytics
from ultralytics import YOLO

print("torch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("ultralytics:", ultralytics.__version__)
print("CUDA:", torch.cuda.is_available())
print("CUDA version:", torch.version.cuda)
print("device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "none")
assert torch.cuda.is_available(), "CUDA is unavailable"
assert torchvision.__version__.startswith("0.20.0"), torchvision.__version__
assert ultralytics.__version__ == "8.4.147", ultralytics.__version__
assert hasattr(torch.ops.torchvision, "nms"), "torchvision NMS operator is unavailable"
boxes = torch.tensor([[0.0, 0.0, 10.0, 10.0]], device="cuda")
scores = torch.tensor([1.0], device="cuda")
assert torchvision.ops.nms(boxes, scores, 0.5).device.type == "cuda"
print("CUDA NMS: OK")
model = YOLO("yolo11n.pt")
print("YOLO load: OK")
PY

if [[ "${1:-}" == "--static" ]]; then
  python3 /usr/local/bin/survivor_static_inference.py "${2:-}"
fi

echo "ROS executable:"
ros2 pkg executables rescue_robot_survivor
