# Isaac ROS Visual SLAM 및 nvblox 진행 기록

## 현재 완료 상태

- Jetson Orin / Ubuntu 22.04 / ROS 2 Humble
- Intel RealSense D435 연결 및 RGB/depth/infra 토픽 확인
- Isaac ROS Visual SLAM 실행 확인
- Visual SLAM `odom -> base_link` TF 확인
- RealSense 및 URDF TF 연결 확인
- nvblox 실행 확인
- nvblox mesh, TSDF, ESDF 관련 토픽 발행 확인
- RViz에서 PointCloud2와 TF/RobotModel/Path 시각화 설정 저장

## 확인된 입력 토픽

```text
/leader/camera/color/image_raw
/leader/camera/color/camera_info
/leader/camera/depth/image_rect_raw
/leader/camera/depth/camera_info
/leader/camera/infra1/image_rect_raw
/leader/camera/infra1/camera_info
/leader/camera/infra2/image_rect_raw
/leader/camera/infra2/camera_info
```

Visual SLAM은 일반 D435를 사용하므로 IMU fusion 없이 infra1/infra2 stereo 입력을
사용한다. nvblox는 RGB/depth와 TF를 사용한다.

## TF 구조

현재 목표 구조는 다음과 같다.

```text
map -> odom -> base_link -> camera_link
                              ├─ camera_infra1_optical_frame
                              ├─ camera_infra2_optical_frame
                              └─ camera_depth_optical_frame
```

`base_link -> camera_link`는 URDF의 `camera_joint`가 발행하고, `odom -> base_link`는
Visual SLAM이 발행한다.

## 호스트: RealSense 실행

카메라 드라이버는 호스트에서 한 번만 실행한다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=leader \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  enable_infra:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  publish_tf:=true \
  tf_publish_rate:=30.0
```

## 컨테이너: Isaac ROS workspace

`run_dev.sh`는 `--rm` 컨테이너를 사용하므로 Isaac ROS apt 패키지는 새 컨테이너마다
확인해야 한다. `damgc_robot` workspace를 마운트해 실행한다.

```bash
cd ~/isaac_ros_ws/src/isaac_ros_common
./scripts/run_dev.sh -d /home/maze/damgc_robot
```

현재 컨테이너에서 필요한 패키지 확인:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix isaac_ros_visual_slam
ros2 pkg prefix nvblox_ros
```

없으면 현재 컨테이너에서 설치한다.

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-isaac-ros-visual-slam \
  ros-humble-isaac-ros-nvblox
```

## 빌드 및 실행

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
colcon build --packages-select rescue_robot_bringup --symlink-install
source install/setup.bash
```

필요한 경우 GXF runtime 경로를 추가한다.

```bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger
```

Visual SLAM만 실행:

```bash
ros2 launch rescue_robot_bringup visual_slam_realsense.launch.py
```

Visual SLAM과 nvblox를 함께 실행:

```bash
ros2 launch rescue_robot_bringup visual_slam_nvblox_realsense.launch.py
```

## 출력 확인

```bash
ros2 topic echo /visual_slam/tracking/odometry --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /leader/camera/depth/image_rect_raw
ros2 topic hz /nvblox_node/mesh
```

확인된 nvblox 출력 토픽:

```text
/nvblox_node/mesh
/nvblox_node/tsdf_layer
/nvblox_node/static_esdf_pointcloud
/nvblox_node/static_map_slice
/nvblox_node/workspace_bounds
```

## RViz 시각화

저장된 설정 파일:

```text
rviz/vslam_nvblox.rviz
```

컨테이너에서 실행:

```bash
rviz2 -d /workspaces/isaac_ros-dev/rviz/vslam_nvblox.rviz
```

주요 표시 항목:

- Fixed Frame: `map`
- TF
- RobotModel
- `/nvblox_node/static_esdf_pointcloud` (PointCloud2)
- `/visual_slam/tracking/vo_path` (Path)
- `/visual_slam/tracking/odometry` (Odometry)
- `/nvblox_node/mesh` (nvblox Mesh plugin이 설치된 경우)

RViz 설정은 저장되지만, 실제 nvblox 지도 데이터는 자동 저장되지 않는다.

## 다음 작업

1. RViz에서 mesh와 path 장시간 유지 확인
2. nvblox map save/load 서비스 확인
3. 실제 저속 주행에서 tracking loss 시험
4. 카메라 USB 3 연결과 프레임 드롭 확인
5. `base_link` 기준 wheel odometry/BNO055를 `robot_localization`에 연결
6. Nav2 및 damgc_robot 전체 launch 통합
