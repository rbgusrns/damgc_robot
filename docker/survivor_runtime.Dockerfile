FROM isaac_ros_dev-aarch64:latest

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    ROS_PYTHON_VERSION=3 \
    SURVIVOR_WS=/opt/damgc_survivor_ws \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1

# Keep the NVIDIA torch already supplied by the Jetson-compatible base image.
# torchvision is built against that exact torch instead of allowing pip to
# replace it with a generic PyPI wheel.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      libopenblas-dev \
      python3-dev \
      python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 - <<'PY'
import torch
print("base torch:", torch.__version__)
assert torch.__version__.startswith("2.5.0a0+")
PY

WORKDIR /tmp
RUN git clone --depth 1 --branch v0.20.0 \
      https://github.com/pytorch/vision.git torchvision-src \
    && cd torchvision-src \
    && export BUILD_VERSION=0.20.0 \
    && export TORCHVISION_INCLUDE=/tmp/torchvision-src \
    && export MAX_JOBS=1 \
    && export FORCE_CUDA=1 \
    && export TORCH_CUDA_ARCH_LIST=8.7 \
    && python3 setup.py bdist_wheel \
    && python3 -m pip install --no-deps dist/torchvision-0.20.0-*.whl \
    && cd / \
    && rm -rf /tmp/torchvision-src

# Install Ultralytics without dependency resolution. The Jetson torch and
# torchvision pair above must remain untouched. These are the runtime deps
# used by the verified YOLO11n path; torch packages are intentionally absent.
RUN python3 -m pip install --no-deps \
      ultralytics==8.4.147 \
      matplotlib \
      numpy \
      pillow \
      psutil \
      pyyaml \
      requests \
      scipy \
      polars \
      ultralytics-thop

WORKDIR ${SURVIVOR_WS}
COPY src/leader/rescue_robot_survivor src/leader/rescue_robot_survivor
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
         --packages-select rescue_robot_survivor \
         --build-base build_survivor \
         --install-base install_survivor

COPY scripts/check_survivor_ai_runtime.sh /usr/local/bin/check_survivor_ai_runtime
COPY scripts/survivor_static_inference.py /usr/local/bin/survivor_static_inference.py
RUN chmod +x /usr/local/bin/check_survivor_ai_runtime

ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["set +u; source /opt/ros/humble/setup.bash; source /opt/damgc_survivor_ws/install_survivor/setup.bash; set -u; ros2 run rescue_robot_survivor person_detector_node"]
