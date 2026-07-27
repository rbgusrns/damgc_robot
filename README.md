# 다중 로봇 재난 탐색·구호물품 운반 ROS 2 워크스페이스

`damgc_robot`은 소형 로봇 2대로 재난 환경을 탐색하고, 생존자 위치를 찾은 뒤
구호물품을 단독 또는 협동 운반하기 위한 ROS 2 Humble 워크스페이스입니다.

- 탐색·리더 로봇: RealSense D435 기반 인지, 3차원 지도, 자율주행, 임무 조정
- 운반 보조·팔로워 로봇: AprilTag 기반 상대 위치 보정과 협동 운반 지원
- 공통 하위 제어: STM32 기반 모터·엔코더·IMU·그리퍼 제어

최종 범위는 [개발 계획서](docs/Plan.md), 현재 완료 범위와 다음 작업은
[개발 현황 및 로드맵](docs/STATUS_AND_ROADMAP.md)을 기준으로 확인합니다.

## 현재 구현 상태

2026년 7월 27일 기준으로 저장소에서 확인되는 구현은 다음과 같습니다.

- 리더: URDF/RViz 모델, D435 RGB·depth, RGB 보정, AprilTag 검출, 중앙 depth CSV 측정
- 팔로워: USB 카메라, AprilTag 검출, TF 기반 거리·각도·정렬 상태 판정
- 아직 없음: Visual SLAM·nvblox, 사람 탐지, Nav2, wheel odometry·IMU 융합,
  STM32·그리퍼 연동, Mission Coordinator, 리더–팔로워 협동 운반

현재 AprilTag 상태 출력은 주행 명령이 아닙니다. 실제 이동 전에는 로봇 기준 TF,
속도 제한, 통신 유실 정지와 비상정지 경로를 먼저 연결해야 합니다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd /home/maze/damgc_robot
colcon build --symlink-install
source install/local_setup.bash
```

다른 경로에 clone했다면 `cd` 경로만 해당 저장소 루트로 바꿉니다. 두 Orin은
JetPack 6.2.2, Ubuntu 22.04, ROS 2 Humble과 동일한 의존성 버전을 사용하는 것이
계획의 기준입니다.

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

## 리더 DDS 협력 통신

리더 Orin에서 별도 터미널로 실행합니다. 기본 상태는 정지(`IDLE`)입니다.

```bash
ros2 launch leader_cooperation leader_cooperation.launch.py
ros2 service call /cooperation/enable std_srvs/srv/SetBool "{data: true}"
```

팔로워가 `/follower/status` (`std_msgs/msg/String`) heartbeat를 발행하고,
리더의 `/leader/cmd_vel`이 들어오는 동안에만 `/follower/cmd_vel`로 전달됩니다.
heartbeat 또는 명령이 끊기면 0 속도로 정지합니다. 상세 계약은
[leader_cooperation README](src/leader/leader_cooperation/README.md)를 참고합니다.

## 팔로워 인식 파이프라인

```bash
ros2 launch follower_supply_perception follower_apriltag.launch.py
```

기존 카메라·AprilTag 파이프라인을 유지하고 상태 판정 노드만 실행할 때는:

```bash
ros2 launch follower_supply_perception approach_only.launch.py
```

상세 토픽과 상태 정의는
[리더·팔로워 구조](docs/LEADER_FOLLOWER_ARCHITECTURE.md)에서 확인할 수 있습니다.

## 문서

- [문서 안내](docs/README.md)
- [프로젝트 개요](docs/PROJECT_OVERVIEW.md)
- [개발 계획서](docs/Plan.md)
- [개발 현황 및 로드맵](docs/STATUS_AND_ROADMAP.md)
- [1차 구현·시험 기록](docs/progress/week%201/README.md)
