# VSLAM 키보드 매핑 진행 기록 — 2026-08-28

## 결과 요약

STM32 bridge, RealSense D435, dual EKF, Isaac ROS Visual SLAM, nvblox, RViz와
방향키 teleop을 한 명령으로 시작하고 종료하는 경로를 구성했다.

```bash
cd /home/maze/damgc_robot
./scripts/run_vslam_mapping.sh
```

필요한 센서·VSLAM 토픽이 준비된 뒤 현재 터미널이 방향키 조종기로 전환된다.
Space는 즉시 정지하고 Ctrl-C는 실행기가 시작한 전체 stack을 종료한다.

## 추가한 구성

### 방향키 teleop

`rescue_robot_bringup`에 `arrow_key_teleop.py`를 추가했다.

- 출력: `/leader/cmd_vel`
- 위/아래 방향키: 전진/후진
- 왼쪽/오른쪽 방향키: 제자리 좌/우 회전
- Space: 즉시 0속도
- 키 반복이 0.25초 끊기면 자동 0속도
- 노드 기본값: 직진 0.12 m/s, 회전 0.35 rad/s
- 통합 실행기 기본값: 직진 0.08 m/s, 회전 0.25 rad/s
- STM32 bridge에도 200 ms command watchdog이 적용됨

### 원클릭 실행기

`scripts/run_vslam_mapping.sh`가 다음 순서로 실행한다.

1. 호스트 STM32 bridge
2. 호스트 RealSense D435
3. 전용 Isaac ROS Docker container
4. dual EKF, VSLAM, nvblox
5. RViz
6. 방향키 teleop

중복 실행을 PID file로 막고, 호스트 launch는 독립 process group으로 실행한다.
Ctrl-C 또는 시작 실패 시 process group 전체를 종료해 camera/bridge 자식 노드가 남지
않도록 했다. 실행별 stdout/stderr는 다음 경로에 저장한다.

```text
log/vslam_mapping_YYYYMMDD_HHMMSS/
├── stm32_bridge.log
├── realsense.log
├── vslam_nvblox.log
└── rviz.log
```

### 영구 Docker 이미지

기존 `isaac_ros_dev-aarch64:vslam-nvblox` 위에 다음 runtime을 추가하는
`docker/vslam_mapping.Dockerfile`을 만들었다.

- `ros-humble-robot-localization`
- `ros-humble-diagnostic-updater`

생성 이미지:

```text
damgc-vslam-mapping:humble
```

이 이미지는 로컬 Docker에 유지되며 Git에는 Dockerfile만 저장한다. 따라서 `--rm`
container가 종료돼도 다음 실행에서 VSLAM, nvblox와 EKF 패키지를 다시 설치하지 않는다.

## 실제 통합 검증

`log/vslam_mapping_20260828_151610` 실행에서 다음을 확인했다.

- STM32 UART `/dev/ttyTHS1` open 성공
- RealSense D435 USB 3.2 연결 및 infra1/infra2/depth/color stream 시작 성공
- container의 host UID 사용자를 `admin`으로 정상 해석
- `rescue_robot_bringup`, `isaac_ros_visual_slam`, `nvblox_ros`,
  `robot_localization`, `diagnostic_updater` package 인식
- `/leader/odom/raw` 준비 확인
- `/leader/camera/infra1/image_rect_raw` 준비 확인
- `/visual_slam/tracking/odometry` 준비 확인
- cuVSLAM tracker 초기화 성공
- nvblox 실행 및 RViz OpenGL 초기화 성공
- 방향키 teleop 진입 성공
- Ctrl-C 후 Docker container와 관련 ROS node가 남지 않음

초기 구현 과정에서 발생한 다음 문제도 수정했다.

1. ROS setup과 `set -u` 충돌
2. Docker running 직후 `admin` 사용자가 아직 생성되지 않은 초기화 race
3. 일회성 container 종료 후 VSLAM/EKF runtime package가 사라지는 문제
4. launch 부모만 종료되고 RealSense·STM32 자식 node가 남는 문제

## 자동 저장 및 분석

원클릭 실행기는 각 process의 텍스트 로그와 함께 정확도 분석용 rosbag을 자동
저장한다. Ctrl-C 시 rosbag을 먼저 정상 마감하고 다음 결과를 bag 폴더에 생성한다.

- `analysis.md`: wheel/local/global/VSLAM 경로 길이, 변위, yaw, 정지 구간 흔들림,
  tracking 성공률과 wheel/VSLAM 차이
- `analysis.json`: 반복 시험 비교와 그래프 작성용 분석 데이터

bag에는 `/leader/cmd_vel`, wheel raw, IMU, local/global EKF, VSLAM odometry/status,
TF를 기록한다. 카메라 영상은 용량을 줄이기 위해 기본 기록에서 제외했다.

GPU 부하를 분리해서 확인할 수 있도록 다음 실행 모드도 추가했다.

- `VSLAM_HEADLESS=1`: RViz와 VSLAM debug rendering만 비활성화
- `VSLAM_HEADLESS=1 VSLAM_ONLY=1`: 위 기능에 더해 nvblox도 비활성화하고 VSLAM/EKF
  입력 처리만 측정

아직 자동 저장하지 않는 항목:

- nvblox 3D map
- VSLAM map database
- 카메라 영상을 포함한 완전 재생용 rosbag

## 관찰 사항

30 Hz camera의 정상 frame 간격은 약 33.3 ms지만 현재 VSLAM 설정의
`image_jitter_threshold_ms`는 22 ms이다. 통합 시험 로그에서 33~100 ms frame delta
경고가 반복됐다. tracking odometry는 발행됐지만, 실제 주행 정확도 시험 전에 threshold
의 의미와 camera frame drop 비율을 함께 확인해야 한다.

STM32 bridge는 종료 시 rclpy context가 먼저 닫혀 traceback과 exit code 1을 남길 수
있지만, process와 UART는 종료되고 잔류 node는 없었다. 정상 종료 로그 개선은 별도
정리 항목이다.

## 다음 작업

1. 줄자 기준 1 m 직진 및 제자리 회전 정량 시험
2. 큰 사각형 또는 폐루프 주행으로 loop closure 전후 오차 측정
3. camera frame drop과 `image_jitter_threshold_ms` 조정
4. nvblox map save/load 지원 확인
