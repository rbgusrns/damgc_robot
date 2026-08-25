# Isaac ROS Visual SLAM 및 nvblox 진행 기록

## 현재 완료 상태

- Jetson Orin / Ubuntu 22.04 / ROS 2 Humble
- Intel RealSense D435 연결 및 RGB/depth/infra 토픽 확인
- Isaac ROS Visual SLAM 실행 확인
- Visual SLAM `odom -> base_link` TF 확인
- RealSense 및 URDF TF 연결 확인
- nvblox 실행 확인
- nvblox mesh, TSDF, ESDF 관련 토픽 발행 확인
- STM32 wheel/IMU local EKF와 VSLAM global EKF 실행 확인
- `/visual_slam/tracking/odometry` 약 17 Hz 확인
- `/nvblox_node/mesh` 실행 조건에 따라 약 9.7~17 Hz 확인
- Docker RViz에서 카메라, PointCloud2, TF/RobotModel/Path와 nvblox mesh 표시 확인

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

`base_link -> camera_link`는 URDF의 `camera_joint`가 발행한다. 융합 구성에서는 local
EKF가 `odom -> base_link`, global EKF가 `map -> odom`을 발행한다. Visual SLAM의
`publish_odom_to_base_tf`와 `publish_map_to_odom_tf`는 모두 `false`이다.

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
  enable_gyro:=false \
  enable_accel:=false \
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
ros2 pkg prefix robot_localization
```

없으면 현재 컨테이너에서 설치한다.

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-isaac-ros-visual-slam \
  ros-humble-isaac-ros-nvblox \
  ros-humble-robot-localization \
  ros-humble-diagnostic-updater
```

## 빌드 및 실행

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
colcon --log-base log_docker build \
  --packages-up-to rescue_robot_bringup \
  --symlink-install \
  --build-base build_docker \
  --install-base install_docker
source install_docker/setup.bash
```

필요한 경우 GXF runtime 경로를 추가한다.

```bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger
```

STM32 엔코더·IMU와 VSLAM을 융합하려면 먼저 STM32 bridge를 실행한 뒤
localization launch를 실행한다. local EKF가 `odom → base_link`를 발행하고,
VSLAM odometry와 local EKF를 global EKF가 융합해 `map → odom`을 발행한다.
global EKF는 local EKF의 속도와 covariance adapter를 통과한 VSLAM map pose를
사용한다. 서로 다른 원점에서 시작하는 local/VSLAM absolute odometry를 같은
`odom` frame의 절대 pose로 직접 융합하면 안 된다.

```bash
ros2 launch rescue_robot_bringup localization.launch.py
```

Visual SLAM만 실행할 때도 이 launch가 발행하는 TF가 필요하므로, 기존처럼
VSLAM만 단독 실행하지 말고 localization을 함께 실행한다.

그 다음 별도 터미널에서 Visual SLAM을 실행:

```bash
ros2 launch rescue_robot_bringup visual_slam_realsense.launch.py
```

Visual SLAM, STM32 융합과 nvblox를 한 번에 실행:

```bash
ros2 launch rescue_robot_bringup visual_slam_nvblox_realsense.launch.py
```

이미 localization을 별도로 실행했다면 통합 launch를 사용하지 않고 VSLAM과 nvblox를
각각 실행한다. 그렇지 않으면 EKF 두 개가 중복 실행된다.

```bash
ros2 launch rescue_robot_bringup visual_slam_realsense.launch.py
ros2 launch rescue_robot_bringup nvblox_realsense.launch.py
```

## 출력 확인

```bash
ros2 topic echo /visual_slam/tracking/odometry --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map camera_depth_optical_frame
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

`nvblox_msgs/msg/Mesh`와 nvblox RViz plugin은 현재 Docker 환경에 있으므로 RViz와
mesh 진단 명령도 Docker에서 실행한다. `NvbloxMesh`가 Add 목록에 없다면
`nvblox_rviz_plugin`을 `install_docker` overlay에 빌드하고 RViz를 재시작한다.

현재 저장 설정의 카메라 화면은 Image display의
`/leader/camera/infra1/image_rect_raw`을 사용한다. color 영상을 사용하려면
`/leader/camera/color/image_raw`과 `Best Effort` reliability를 선택한다. TF 축이
과도하게 겹치면 TF display에서 `map`, `odom`, `base_link`만 남기거나 TF display를
끈다.

RViz 설정은 저장되지만, 실제 nvblox 지도 데이터는 자동 저장되지 않는다.

## 다음 작업

1. 줄자 기준 1 m 직진에서 wheel/VSLAM 거리 오차 비교
2. 제자리 회전에서 wheel/VSLAM yaw 오차 비교
3. RViz에서 mesh와 path 장시간 유지 확인
4. nvblox map save/load 서비스 확인
5. 카메라 USB 3 연결과 프레임 드롭 확인
6. BNO055 orientation fusion 전 장착 방향과 calibration 확인
7. Nav2 및 damgc_robot 전체 launch 통합
