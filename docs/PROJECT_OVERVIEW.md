# 프로젝트 개요

## 목적

`damgc_robot`은 ROS 2 Humble 기반의 소형 로봇 2대로 재난 환경을 탐색하고,
생존자를 찾은 뒤 규격 구호물품을 단독 또는 협동 운반하기 위한 프로젝트입니다.
탐색·리더 로봇은 3차원 지도, 생존자 인식, 자율주행과 임무 조정을 담당하고,
운반 보조·팔로워 로봇은 물품 접근과 협동 운반을 지원합니다.

이 문서에서 **목표 시스템**은 [개발 계획서](Plan.md)를, **현재 구현 상태**는
[개발 현황 및 로드맵](STATUS_AND_ROADMAP.md)을 뜻합니다. 현재 저장소는 목표
시스템 전체가 아니라 URDF, 카메라, depth 측정, AprilTag camera/base alignment와
software velocity pipeline까지 포함합니다. 실제 물품 파지와 하드웨어 주행 완료를
의미하지는 않습니다.

## 목표 기능 흐름

```text
D435 RGB-D → Visual SLAM·nvblox → 3차원 지도
       ├──→ 사람 탐지 + depth → 생존자 지도 좌표
       └──→ AprilTag → 물품 종류·상대 위치
                              ↓
Mission Coordinator → Nav2·정밀 접근 → 그리퍼
                              ↓
                    단독 운반 또는 리더–팔로워 협동 운반
```

현재는 이 흐름 중 D435 입력, depth 중앙값 측정, 양 로봇 AprilTag 검출·base alignment,
atomic approach command와 guarded software velocity 출력까지 구현되어 있습니다.

## 디렉터리 구조

```text
damgc_robot/
├── src/
│   ├── leader/
│   │   ├── rescue_robot_description/   # URDF와 모델 표시 launch
│   │   ├── rescue_robot_bringup/       # 리더 통합 실행 launch
│   │   ├── rescue_robot_apriltag/      # CameraInfo QoS bridge, AprilTag 설정
│   │   └── rescue_robot_tools/         # 센서 측정 도구
│   ├── follower/
│       ├── follower_alignment_msgs/    # atomic alignment command
│       ├── follower_supply_perception/ # 팔로워 AprilTag hybrid 접근·정렬
│       ├── follower_approach_control/   # raw 저속 접근 controller
│       ├── follower_command_selector/  # APPROACH/COOPERATION ownership
│       └── follower_control/            # final velocity safety guard
│   └── stm32_bridge/                    # 공통 STM32 transport/odometry bridge
├── data/                               # 실행 시 생성하는 측정 결과, Git 제외
├── build/ install/ log/                # colcon 생성물, Git 제외
└── docs/                               # 프로젝트 공통 문서
```

Visual SLAM·nvblox와 STM32 bridge 기반 기능은 저장소에 추가됐지만 실제 장비별
완료 여부는 별도 실행 기록을 따른다. 사람 탐지, Nav2, 그리퍼와 Mission Coordinator는
아직 완료되지 않았다. 새 패키지를 추가할 때는 리더 전용, 팔로워 전용, 공통 인터페이스
중 소유 범위를 먼저 정합니다.

## 패키지 역할

| 패키지 | 빌드 타입 | 역할 |
| --- | --- | --- |
| `rescue_robot_description` | `ament_cmake` | URDF, robot state publisher, RViz 표시 |
| `rescue_robot_bringup` | `ament_cmake` | RealSense·image_proc·AprilTag 통합 launch |
| `rescue_robot_apriltag` | `ament_cmake` | 리더 CameraInfo QoS 연결 보조와 AprilTag 설정 |
| `rescue_robot_tools` | `ament_cmake` | Depth 영상을 CSV로 저장하는 측정 도구 |
| `follower_alignment_msgs` | `ament_cmake` | Follower atomic pose/mode/state message |
| `follower_supply_perception` | `ament_python` | 팔로워 AprilTag 상대 위치와 hybrid 접근 상태 판단 |
| `follower_approach_control` | `ament_python` | atomic alignment command를 raw Twist로 변환 |
| `follower_command_selector` | `ament_python` | approach/cooperation 명령 소유권 선택 |
| `follower_control` | `ament_python` | timeout·clamp·slew를 포함한 final velocity guard |
| `stm32_bridge` | `ament_python` | STM32 transport, wheel/IMU state와 odometry bridge |

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
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py
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

- 리더 구현됨: URDF/RViz 모델, RealSense RGB/depth 발행, RGB 보정,
  CameraInfo QoS bridge, AprilTag 검출, 중앙 depth 거리의 CSV 저장
- 팔로워 구현됨: USB 카메라 보정, AprilTag 검출, exact-stamp TF 기반 상대 위치,
  hybrid base state, atomic command, approach controller, selector와 final safety guard
- Follower hybrid 안정화: `base_stable_time=0.30 s`, fresh confirmation 3회,
  FINAL_APPROACH/STABILIZING 0.30 s zero-command tag-loss grace, session별 ALIGNED reset
- 부분 완료: AprilTag 기반 정밀 접근 software는 구현됐지만 실제 로봇 주행·파지 검증은 남음
- 미구현/미완료: 사람 탐지와 3차원 위치, Nav2, 그리퍼, Mission Coordinator,
  Orin 간 실물 협동 운반
- `target_distance=0.15 m` 등 접근 파라미터는 초기 시험값이며 실제 그리퍼/TCP 기준으로 재검증해야 합니다.

리더의 주요 확인 토픽은 `/leader/camera/color/image_rect`,
`/leader/camera/depth/image_rect_raw`, `/leader/apriltag/detections`이며,
태그 TF는 `camera_color_optical_frame -> tag36h11:0`입니다.

`camera_link`와 RealSense optical frame은 아직 하나의 실물 기준 TF 체인으로
검증되지 않았습니다. 따라서 카메라 기준 pose를 주행이나 파지에 바로 사용하면 안 됩니다.

## 계획상 다음 통합 단위

2026년 7월 27일 시작한 3주차의 핵심은 다음과 같습니다.

1. 실제 센서 장착 기준 TF와 다중 로봇 frame 이름 고정
2. wheel odometry·BNO055·`robot_localization` 연결
3. 정적 지도 기반 리더 Nav2 목표점 이동
4. 사람 검출 ROI와 aligned depth를 결합한 카메라 좌표 출력
5. 팔로워 `/follower/cmd_vel`과 STM32 기본 구동
6. 두 로봇 비상정지와 30분 전원·발열 시험

세부 완료 조건과 선행 미완료 항목은
[개발 현황 및 로드맵](STATUS_AND_ROADMAP.md)에서 관리합니다.
