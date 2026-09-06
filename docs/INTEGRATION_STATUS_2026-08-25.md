# 통합 진행 기록 — 2026-08-25

## 결과 요약

RealSense D435, STM32 wheel/IMU, dual EKF, Isaac ROS Visual SLAM과 nvblox를
동시에 실행하고 Docker RViz에서 3차원 지도 토픽을 표시하는 단계까지 확인했다.

```text
호스트 RealSense ───────────────┐
                               ├─ Docker: VSLAM ───────┐
호스트 STM32 bridge            │                        ├─ global EKF: map → odom
  ├─ /leader/odom/raw ─ local EKF: odom → base_link ───┘
  └─ /leader/imu/data_raw       │
                               └─ Docker: nvblox → mesh/ESDF → RViz
```

현재 VSLAM의 자체 TF 발행은 꺼져 있고, TF 소유자는 dual EKF로 통일되어 있다.

## 이번 세션에서 확인한 항목

- D435 infra1 영상: 약 29~30 Hz
- `/leader/camera/infra1/camera_info`: 848×480, optical frame 및 calibration 수신 확인
- `/visual_slam_node`: 실행 확인
- `/visual_slam/tracking/odometry`: 약 17 Hz
- `/nvblox_node/mesh`: 실행 조건에 따라 약 9.7~17 Hz
- `nvblox_msgs/msg/Mesh`: Docker에서 interface 로드 확인
- Docker RViz에서 카메라 영상, TF, VSLAM 표시 및 nvblox 3D 지도 표시 단계 도달
- RViz 기본 설정 파일: `rviz/vslam_nvblox.rviz`

기존 STM32 시험에서 확인한 결과는 다음과 같다.

- UART frame count 증가, `crc_errors=0`, `sequence_drops=0`
- `/leader/imu/data_raw` 약 100 Hz
- 휠 계수:
  - `wheel_radius_m=0.0635`
  - `wheel_separation_m=0.23`
  - `ticks_per_revolution=5131`
- 1 m 직진 시 wheel odometry 약 1.05 m
- 제자리 회전 시 x/y 변화는 작고 yaw quaternion 변화 확인

## 정지 bag 분석

기록 파일: `data/vslam_static_01`

- 기록 시간: 29.76초
- 전체 메시지: 9,020개
- bag 크기: 5.1 MiB

로봇을 정지시킨 구간의 변화량은 다음과 같다.

| 출력 | x 범위 | y 범위 | yaw 범위 | 판정 |
|---|---:|---:|---:|---|
| wheel raw | 0 m | 0 m | 0° | 정상 |
| local EKF | 0 m | 0 m | 0.00375° | 정상 |
| VSLAM odometry | 0.000037 m | 0.000123 m | 0.00123° | 정상 |
| global EKF | 2.97 m | 4.27 m | 반복적인 약 55° 점프 | 비정상 |
| `map -> odom` TF | 2.30 m | 3.60 m | 반복적인 약 55° 점프 | 비정상 |

RViz에서 좌표축이 크게 움직인 현상은 단순 TF display 혼잡이 아니라 global EKF의
`map -> odom` 발산이었다. local EKF와 VSLAM 자체는 정지 상태에서 안정적이었다.

global EKF에 들어간 두 절대 odometry는 모두 `odom -> base_link`라고 선언하지만 시작
원점이 서로 달랐다.

```text
local EKF 시작: x=0.3516 m, y=0.0259 m, yaw=  8.60°
VSLAM 시작:     x=0.2109 m, y=-0.6681 m, yaw=-46.65°
초기 yaw 차이: 약 55.25°
```

global EKF가 이 약 55° 차이를 반복 보정하면서 회전이 누적되었다. 또한
`/visual_slam/tracking/odometry`의 정지 pose covariance는 0 또는 수치 오차 수준의
음수여서 측정값이 비현실적으로 강하게 반영될 수 있다.

따라서 실제 주행 전에 다음 수정과 재시험이 필요하다.

1. local/VSLAM 입력의 시작 좌표 충돌 제거
2. VSLAM map pose에 covariance 하한 적용
3. 수정 후 동일한 30초 정지 bag을 기록해 global EKF와 `map -> odom` 재검증

### 적용한 수정

global EKF가 continuous local pose와 continuous VO pose를 서로 다른 원점의 절대
좌표로 동시에 융합하던 구성을 변경했다.

- `/leader/odometry/local`: global EKF에는 `vx`, `vyaw`만 입력
- `/visual_slam/vis/slam_odometry`: `map` 기준 절대 x/y/yaw 입력
- `vslam_covariance_adapter.py`: VSLAM visualization odometry의 0 covariance를
  위치 표준편차 0.05 m, yaw 표준편차 0.05 rad의 보수적인 값으로 교체
- VSLAM 자체 TF는 계속 끄고 global EKF가 `map -> odom`을 발행

이 수정은 빌드·문법 검증 후 실제 센서 입력으로 정지 재시험했다.

### 수정 후 정지 재시험

`data/vslam_static_03`을 29.36초 기록해 수정 전 bag과 비교했다. adapter 출력은
753개, global odometry는 881개 수신했다.

| 출력 | x 범위 | y 범위 | yaw 범위 | 판정 |
|---|---:|---:|---:|---|
| wheel raw | 0 m | 0 m | 0° | 정상 |
| local EKF | 0 m | 0 m | 0.00337° | 정상 |
| global EKF | 0 m | 0 m | 0.01079° | 정상 |
| `map -> odom` TF | 0.000005 m | 0.000068 m | 0.01103° | 정상 |

수정 전 global EKF는 약 2.97 m, 4.27 m와 반복적인 약 55° 점프를 보였지만 수정
후에는 정지 상태 발산이 사라졌다. 이어서 nvblox를 다시 실행해
`/nvblox_node/mesh` 약 9.7 Hz 발행을 확인했다.

정지 시험은 통과했으며 다음 gate는 모터를 사용하지 않는 저속 수동 이동 bag이다.

### 저속 수동 이동 시험

`data/vslam_motion_01`을 48.91초 기록했다. 실제 wheel motion은 기록 시작 약 20.4초
후부터 감지됐고 여러 번 나뉜 이동이 기록됐다.

- Visual SLAM status: 성공 629개, 실패 0개
- VSLAM callback 평균/최대: 6.85/32.24 ms
- VSLAM tracking 평균/최대: 5.14/26.81 ms
- wheel/local EKF 누적 path: 약 0.61 m
- VSLAM 누적 path: 약 0.79 m
- global EKF는 VSLAM map pose를 안정적으로 추종했고 기존 반복 발산은 재발하지 않음

최종 직선 변위는 wheel 약 0.596 m, VSLAM 약 0.074 m로 달랐다. 다만 이번 기록은
직진과 회전이 명확히 분리되지 않았고 움직임이 여러 구간으로 나뉘었으므로 거리 보정
근거로 사용하지 않는다. 다음 시험에서는 줄자 기준 1 m 직진만 수행하고 시작·종료
정지 구간을 각각 10초 이상 확보한다.

## 최종 실행 구성

각 항목은 별도 터미널에서 실행하며, 모든 ROS 터미널은 같은 DDS 환경을 사용한다.

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

실행 프로세스는 다음과 같다.

```text
호스트 1: realsense2_camera
호스트 2: stm32_bridge
Docker 1: localization.launch.py
Docker 2: visual_slam_realsense.launch.py
Docker 3: nvblox_realsense.launch.py
Docker 4: RViz 또는 진단 명령
```

이미 `localization.launch.py`를 실행했다면
`visual_slam_nvblox_realsense.launch.py`를 추가로 실행하지 않는다. 통합 launch 안에도
localization이 포함되어 있어 EKF가 중복 실행된다.

## 호스트 실행

### STM32 bridge

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

source /opt/ros/humble/setup.bash
source /home/maze/stm32_bridge_install/setup.bash

ros2 launch stm32_bridge stm32_bridge.launch.py \
  port:=/dev/ttyTHS1 baudrate:=460800 namespace:=leader
```

### RealSense D435

D435에는 IMU가 없으므로 stereo infra, depth와 color만 사용한다.

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

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

## Docker 실행

### 컨테이너 접속과 공통 환경

```bash
cd /home/maze/isaac_ros_ws/src/isaac_ros_common
./scripts/run_dev.sh -d /home/maze/damgc_robot
```

Docker의 모든 새 터미널에서 다음을 다시 실행한다.

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install_docker/setup.bash
```

VSLAM 실행 시 GXF 라이브러리를 찾지 못하면 다음 경로를 추가한다.

```bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=/opt/ros/humble/share/isaac_ros_gxf/gxf/lib:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/logger:${LD_LIBRARY_PATH}
```

### 실행 순서

```bash
ros2 launch rescue_robot_bringup localization.launch.py
```

새 Docker 터미널:

```bash
ros2 launch rescue_robot_bringup visual_slam_realsense.launch.py
```

새 Docker 터미널:

```bash
ros2 launch rescue_robot_bringup nvblox_realsense.launch.py
```

## 발생한 문제와 해결

### 1. 호스트 토픽 이름만 보이고 Docker에서 샘플을 받지 못함

호스트와 Docker 양쪽에서 `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`,
`RMW_IMPLEMENTATION`을 맞추고 Fast DDS를 UDPv4로 제한한 뒤 샘플 수신을 확인했다.

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

환경변수는 ROS daemon이나 ROS 노드를 시작하기 전에 설정해야 하며, 새 터미널마다
다시 설정한다.

### 2. nvblox가 `base_link` TF를 찾지 못함

초기 메시지:

```text
Tried to clear map outside of radius but couldn't look up frame: base_link
Lookup transform failed for frame base_link
```

원인은 localization launch가 실행되지 않아 local EKF가 `odom → base_link`를
발행하지 않은 것이었다. EKF를 실행한 뒤 nvblox mesh 발행을 확인했다.

### 3. EKF가 exit code 127로 종료됨

오류:

```text
libdiagnostic_updater.so: cannot open shared object file
```

컨테이너에 누락된 runtime 의존성을 설치했다.

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-diagnostic-updater \
  ros-humble-robot-localization
```

검증:

```bash
ldd /opt/ros/humble/lib/robot_localization/ekf_node | grep "not found"
```

출력이 없으면 공유 라이브러리 의존성이 정상이다.

### 4. VSLAM 토픽은 보이지만 odometry가 발행되지 않음

카메라 infra 입력과 CameraInfo는 정상이었지만 `/visual_slam_node`가 없는 상태를
확인했다. Docker 터미널 환경과 GXF runtime 경로를 다시 적용하고 VSLAM을 재실행해
odometry 약 17 Hz를 확인했다.

`ros2 topic hz`처럼 foreground 명령이 실행 중일 때 뒤에 붙여 넣은 shell 명령은
실행되지 않는다. 먼저 `Ctrl+C`로 종료하고 prompt가 돌아온 뒤 다음 명령을 실행한다.

### 5. 호스트에서 nvblox Mesh 타입이 invalid로 표시됨

오류:

```text
The message type 'nvblox_msgs/msg/Mesh' is invalid
```

nvblox message와 RViz plugin은 Docker에만 설치되어 있으므로 Mesh 확인과 RViz 실행은
Docker에서 수행한다.

```bash
ros2 interface show nvblox_msgs/msg/Mesh
ros2 topic hz /nvblox_node/mesh
```

### 6. RViz의 Add 목록에 NvbloxMesh가 없음

현재 환경에 `nvblox_rviz_plugin`이 설치 또는 소싱되지 않은 경우다. 소스 패키지가
보이면 Docker overlay에 빌드한 뒤 RViz를 완전히 재시작한다.

```bash
colcon list | grep nvblox_rviz_plugin

colcon --log-base log_docker build \
  --packages-select nvblox_rviz_plugin \
  --symlink-install \
  --build-base build_docker \
  --install-base install_docker

source /workspaces/isaac_ros-dev/install_docker/setup.bash
```

## RViz

Docker에서 실행한다.

```bash
rviz2 -d /workspaces/isaac_ros-dev/rviz/vslam_nvblox.rviz
```

권장 주요 설정:

- Fixed Frame: `map`
- Image: `/leader/camera/infra1/image_rect_raw`
- NvbloxMesh: `/nvblox_node/mesh`
- PointCloud2: `/nvblox_node/static_esdf_pointcloud`
- Path: `/visual_slam/tracking/vo_path`
- Odometry: `/visual_slam/vis/slam_odometry`

빨강·초록·파랑 축이 뭉쳐 보이는 것은 여러 camera optical frame을 포함한 TF 표시다.
3D 지도만 볼 때는 TF display를 끄거나 `map`, `odom`, `base_link`만 선택한다. 화면
기본 설정은 시점이 로봇을 따라가도록 Views 패널의 Target Frame을 `base_link`로
설정한다. 지도 중심에 시점을 고정하려면 Target Frame을 `map`으로 바꾼다.

RViz display를 추가하거나 삭제한 뒤에는 `File → Save Config`로 설정을 저장해야 한다.
현재 저장소의 RViz 파일에는 NvbloxMesh, PointCloud2, Path, Odometry와 infra1 Image가
저장되어 있다. RViz 설정 파일은 저장되지만 실제 nvblox 지도 데이터는 자동 저장되지
않는다.

## 재현 확인 명령

```bash
ros2 topic hz /leader/odom/raw
ros2 topic hz /leader/odometry/local
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /nvblox_node/mesh

ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map camera_depth_optical_frame
```

## 남은 검증

1. 줄자 기준 1 m 직진 단일 동작으로 wheel/VSLAM 거리 오차 비교
2. 제자리 회전 단일 동작으로 wheel/VSLAM yaw 오차 비교
3. mesh와 path 장시간 누적 및 메모리 사용량 확인
4. nvblox map save/load 지원 서비스와 저장 경로 확인
5. BNO055 장착 방향 및 calibration 확인 후 orientation fusion 검토
6. Nav2와 전체 bringup 통합

## 주의

- host와 Docker의 기본 `build/install/log`를 공유하면 CMake cache 경로가
  `/home/maze`와 `/workspaces/isaac_ros-dev` 사이에서 충돌한다. Docker 전용
  `build_docker/install_docker/log_docker`를 사용한다.
- `run_dev.sh`가 일회성 컨테이너를 만들면 apt로 설치한 runtime 의존성을 새
  컨테이너마다 다시 설치해야 할 수 있다.
- BNO055 quaternion은 아직 EKF에서 사용하지 않고 z축 각속도만 사용한다.
