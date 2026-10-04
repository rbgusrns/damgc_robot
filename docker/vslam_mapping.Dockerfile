FROM isaac_ros_dev-aarch64:vslam-nvblox

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-diagnostic-updater \
      ros-humble-robot-localization \
      ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*
