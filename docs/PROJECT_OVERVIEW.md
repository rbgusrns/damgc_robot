# 프로젝트 개요

## 목적

`damgc_robot`은 ROS 2 Humble 기반의 구조·센서·AprilTag 인식 패키지를 한 워크스페이스에서 관리하는 프로젝트입니다.
현재 저장소에는 리더 Orin의 RealSense 파이프라인과 팔로워의 USB 카메라 기반 공급 대상 접근 상태 인식이 포함되어 있습니다.

## 디렉터리 구조

```text
damgc_robot/
├── src/
│   ├── leader/
│   │   ├── rescue_robot_description/   # URDF와 모델 표시 launch
│   │   ├── rescue_robot_bringup/       # 리더 통합 실행 launch
│   │   ├── rescue_robot_apriltag/      # CameraInfo QoS bridge, AprilTag 설정
│   │   └── rescue_robot_tools/         # 센서 측정 도구
│   └── follower/
│       └── follower_supply_perception/ # 팔로워 AprilTag 접근·정렬 상태
├── data/                               # 저장소에서 사용할 측정 결과 공간
├── build/ install/ log/                # colcon 생성물
└── docs/                               # 프로젝트 공통 문서
```

## 패키지 역할

| 패키지 | 빌드 타입 | 역할 |
| --- | --- | --- |
| `rescue_robot_description` | `ament_cmake` | URDF, robot state publisher, RViz 표시 |
| `rescue_robot_bringup` | `ament_cmake` | RealSense·image_proc·AprilTag 통합 launch |
| `rescue_robot_apriltag` | `ament_cmake` | 리더 CameraInfo QoS 연결 보조와 AprilTag 설정 |
| `rescue_robot_tools` | `ament_cmake` | Depth 영상을 CSV로 저장하는 측정 도구 |
| `follower_supply_perception` | `ament_python` | 팔로워 AprilTag 상대 위치와 접근 상태 판단 |

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd /home/maze/damgc_robot
colcon build --symlink-install
source install/local_setup.bash
```

리더 장비에서 실행하기 전 RealSense 연결을 확인합니다.

```bash
rs-enumerate-devices
```

다른 워크스페이스에서 복사한 뒤 CMake 캐시 경로 오류가 발생하면 다음 옵션으로 캐시를 재생성합니다.

```bash
colcon build --symlink-install --cmake-clean-cache
```

## 주요 실행 명령

```bash
ros2 launch rescue_robot_description display.launch.py
ros2 launch rescue_robot_bringup camera_apriltag.launch.py
ros2 launch follower_supply_perception follower_apriltag.launch.py
ros2 launch follower_supply_perception approach_only.launch.py
```

리더 통합 실행은 다음 명령입니다.

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py
```

Depth를 끄고 RGB/AprilTag만 확인하려면 `enable_depth:=false`를 추가합니다.

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py enable_depth:=false
```

## 현재 범위와 미구현 범위

- 리더 구현됨: URDF/RViz 모델, RealSense RGB/depth 발행, RGB 보정, CameraInfo QoS bridge,
  AprilTag 검출, 중앙 Depth 거리의 CSV 저장
- 팔로워 구현됨: USB 카메라 보정, AprilTag 검출, TF 기반 상대 위치와 접근·정렬 상태 발행
- 미구현: `cmd_vel` 주행 제어, 모터/엔코더, 실제 IMU 드라이버, STM32 연동, 그리퍼 제어,
  리더-팔로워 통신과 상위 행동 조정
- `target_distance=0.15 m` 등 접근 파라미터는 초기 시험값이며 실제 그리퍼/TCP 기준으로 재검증해야 합니다.

리더의 주요 확인 토픽은 `/leader/camera/color/image_rect`,
`/leader/camera/depth/image_rect_raw`, `/leader/apriltag/detections`이며,
태그 TF는 `camera_color_optical_frame -> tag36h11:0`입니다.
