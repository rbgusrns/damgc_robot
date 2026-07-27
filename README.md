# Rescue Robot ROS 2 Workspace

이 워크스페이스는 로봇 모델, RealSense D435, image_proc, AprilTag, Depth 측정 도구와
팔로워 접근 상태 인식을 함께 관리합니다.

프로젝트 구조와 GitHub 운영 기록은 [docs/README.md](docs/README.md)에서 확인할 수 있습니다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd /home/maze/damgc_robot
colcon build --symlink-install
source install/local_setup.bash
```

## URDF만 확인

```bash
ros2 launch rescue_robot_description display.launch.py
```

## 카메라 + URDF + image_proc + AprilTag 통합 실행

실제 D435가 연결된 상태에서 실행합니다.

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py
```

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

기본 구독 토픽은 `/leader/camera/depth/image_rect_raw`이고 기본 저장 위치는
`/home/maze/damgc_robot/data/depth_distance.csv`입니다.
