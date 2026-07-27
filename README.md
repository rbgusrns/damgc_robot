# Rescue Robot ROS 2 Workspace

이 워크스페이스는 리더 Orin의 로봇 모델·RealSense D435·AprilTag 인식·Depth 측정과
팔로워의 AprilTag 접근 상태 인식을 함께 관리합니다.

프로젝트 구조와 GitHub 운영 기록은 [docs/README.md](docs/README.md)에서 확인할 수 있습니다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd /home/maze/damgc_robot
colcon build --symlink-install
source install/local_setup.bash
```

실제 장비를 사용할 때는 RealSense D435를 Orin에 연결한 뒤 다음 명령으로 장치가
인식되는지 먼저 확인합니다.

```bash
rs-enumerate-devices
```

## URDF만 확인

```bash
ros2 launch rescue_robot_description display.launch.py
```

## 리더 카메라 + URDF + image_proc + AprilTag 통합 실행

실제 D435가 연결된 상태에서 실행합니다.

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py
```

이 launch는 `/leader/camera` 아래에 RealSense RGB/depth 토픽을 만들고,
URDF TF, RGB 보정, CameraInfo QoS bridge, `/leader/apriltag/apriltag`를 함께 실행합니다.

USB 2.x 대역폭 때문에 RGB만 먼저 확인하려면:

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py enable_depth:=false
```

## Detection/TF 확인

```bash
ros2 topic echo --once /leader/apriltag/detections
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:0
```

## Depth CSV 측정

통합 launch를 실행한 뒤 별도 터미널에서 실행합니다.

```bash
ros2 run rescue_robot_tools depth_to_csv.py
```

기본 구독 토픽은 `/leader/camera/depth/image_rect_raw`입니다. 코드의 기본 저장 위치는
`~/jisu_ws/data/depth_distance.csv`이므로, 이 저장소의 `data` 폴더에 저장하려면 다음처럼
경로를 명시합니다.

```bash
ros2 run rescue_robot_tools depth_to_csv.py --ros-args \
  -p output_path:=/home/maze/damgc_robot/data/depth_distance.csv
```

## 리더 확인 명령

```bash
ros2 node list
ros2 topic list | grep leader
ros2 topic echo --once /leader/apriltag/detections
ros2 run rqt_image_view rqt_image_view /leader/camera/color/image_rect
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag36h11:0
```

AprilTag 확인 시 실제 태그 ID 0과 5 cm 크기의 `tag36h11` 태그를 카메라 앞에
두어야 검출 결과와 태그 TF가 출력됩니다.

현재 리더에는 주행 제어, 모터/엔코더, 실제 IMU 드라이버, 그리퍼 구동,
리더-팔로워 통신은 포함되어 있지 않습니다.
