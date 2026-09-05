# follower_supply_perception

- 파일 경로: `README.md`

Jetson Orin Nano의 ROS 2 Humble 환경에서 AprilTag 상대 위치를 사용해 공급 대상의
접근·정렬 상태를 판단하기 위한 Python 패키지입니다.

프로젝트 계획상 이 패키지는 팔로워의 “AprilTag 기반 상대 위치 보정”과
물품 정밀 접근 입력을 담당합니다. 전체 목표와 현재 통합 순서는
[`docs/Plan.md`](../../../docs/Plan.md)와
[`docs/STATUS_AND_ROADMAP.md`](../../../docs/STATUS_AND_ROADMAP.md)를 따릅니다.
이 패키지의 상태 출력만으로 계획의 정밀 접근·주행·파지가 완료된 것은 아닙니다.

현재 단계에서는 기존 camera-frame 상태를 유지하면서 TF2 exact-stamp 변환,
`base_link` pose·metric·상태, 별도 approach controller, deterministic command selector,
최종 Follower safety guard까지 software pipeline을 구성했습니다.
확정된 요구사항은
[`docs/TASK_SPEC_APRILTAG_APPROACH.md`](docs/TASK_SPEC_APRILTAG_APPROACH.md)에
기록되어 있습니다.

단위 테스트 결과와 하드웨어 미검증 범위는
[`docs/TEST_RESULTS_APRILTAG_APPROACH.md`](docs/TEST_RESULTS_APRILTAG_APPROACH.md)에
기록되어 있습니다.

실제 ROS 그래프 통합 확인 결과는
[`docs/INTEGRATION_CHECK_APRILTAG_APPROACH.md`](docs/INTEGRATION_CHECK_APRILTAG_APPROACH.md),
물리 태그 수동 시험은
[`docs/MANUAL_STATE_TEST.md`](docs/MANUAL_STATE_TEST.md)를 참고합니다.

launch 구성과 검증 결과는
[`docs/LAUNCH_VALIDATION_APRILTAG.md`](docs/LAUNCH_VALIDATION_APRILTAG.md)에
기록되어 있습니다.

종합 설계·운영 가이드는
[`docs/APRILTAG_APPROACH_NODE_GUIDE.md`](docs/APRILTAG_APPROACH_NODE_GUIDE.md),
전체 구현 변경 기록은
[`docs/IMPLEMENTATION_RECORD.md`](docs/IMPLEMENTATION_RECORD.md)를 참고합니다.

현재 base-link velocity pipeline의 설계, 전체 파라미터, 선택 빌드·자동시험 결과와
실카메라 검증 절차는
[`docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md`](docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를
기준으로 합니다.

노드 실행 진입점은 `apriltag_approach_node`입니다. 출력 토픽은 상대 이름을 사용하므로
요구된 `/follower/...` 이름으로 사용하려면 노드를 `follower` namespace에서 실행해야
합니다.

`config/approach.yaml`의 모든 수치는 초기 기능 시험용입니다. 특히
`target_distance`는 실제 그리퍼 동작 거리로 확정된 값이 아니며 현장 측정 후 조정해야
합니다.

## 환경 확인

- Ubuntu 22.04 / ROS 2 Humble
- Python 3.10
- 작업공간: `~/damgc_robot`
- 패키지: `~/damgc_robot/src/follower/follower_supply_perception`

향후 빌드와 테스트를 실행할 때는 각 셸에서 ROS 환경을 먼저 설정합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/damgc_robot
colcon build --packages-select follower_supply_perception
source install/local_setup.bash
colcon test --packages-select follower_supply_perception
colcon test-result --verbose
```

이 패키지 자체의 node는 perception과 상태 판단만 담당합니다. raw command, command
ownership, final safety는 각각 `follower_approach_control`,
`follower_command_selector`, `follower_control`이 담당합니다. 통합 launch는 이 패키지들을
`stm32_bridge`까지 연결하지만 STM32 firmware와 motor control 구현은 변경하지 않습니다.

## Launch 실행

기존 수동 카메라·Rectify·AprilTag 노드를 먼저 해당 터미널에서 종료한 다음 전체
주행 software pipeline을 실행합니다.

현재 Follower STM32 I2C slave firmware가 준비되지 않은 상태에서는 bridge만 제외한
software-only 모드를 사용합니다.

```bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py \
  use_stm32_bridge:=false
```

STM32 I2C firmware가 준비된 실제 robot에서는 기본 실행을 사용합니다. 기본 bridge
설정은 `follower` namespace, `i2c`, `/dev/i2c-7`, address `66 (0x42)`, I2C write enabled다.

```bash
ros2 launch follower_supply_perception follower_apriltag_drive.launch.py
```

두 모드 모두 approach controller는 enabled, selector는 `APPROACH`, velocity guard는
disabled 상태로 시작합니다. 따라서 controller 내부 command가 생성되더라도 실제 motor
command를 통과시키려면 사용자가 guard를 명시적으로 열어야 합니다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"
```

정지하거나 시험을 마칠 때는 즉시 guard를 다시 닫습니다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

Pipeline 상태와 최종 bridge 연결은 다음 명령으로 확인합니다.

```bash
ros2 topic echo /follower/base_alignment/state
ros2 topic echo /follower/approach/cmd_vel_raw
ros2 topic echo /follower/selected_cmd_vel
ros2 topic echo /follower/safe_cmd_vel
ros2 topic info /follower/safe_cmd_vel --verbose
```

STM32 I2C firmware가 준비된 뒤 수신 frame은 다음으로 확인합니다. I2C slave가 아직 ACK하지
않아 발생하는 timeout은 software launch integration 실패를 의미하지 않습니다.

```bash
ros2 topic echo /follower/stm32_rx/frame_count
```

Perception만 개별 실행하려면 기존 launch를 그대로 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 launch follower_supply_perception follower_apriltag.launch.py
```

카메라 보정값과 AprilTag·접근 상태 파라미터는 모두 패키지의 `config/`에 포함되어
있으며, 기본 실행에는 작업공간 밖의 설정 파일이 필요하지 않습니다.

기존 파이프라인을 유지하고 상태 판단 노드만 실행할 때는 다음을 사용합니다.

```bash
ros2 launch follower_supply_perception approach_only.launch.py
```

영상 확인은 별도 터미널에서 실행합니다.

```bash
ros2 run rqt_image_view rqt_image_view /follower/camera/image_rect
```

## 사용자 최종 확인

- `docs/MANUAL_STATE_TEST.md`에 따라 실제 태그를 좌우·전후로 이동해 모든 상태를 확인합니다.
- `rviz2`의 Fixed Frame을 `base_link` 또는
  `follower/follower_camera_optical_frame`으로 설정하고 TF와
  `/follower/supply/relative_pose`를 사용자가 직접 확인합니다.
- camera state의 `target_distance=0.15 m`와 base/controller target `0.25 m`는 서로 다른
  software-validation 값이다. 둘 다 실제 그리퍼/TCP 기준 grasp 거리로 확정하지 않습니다.

RIGHT/TARGET/HIDDEN 실카메라 시나리오는 아직 사용자가 직접 확인해야 하며 문서에서
`NOT VERIFIED`로 유지합니다. STM32 I2C slave firmware, motor algorithm과 그리퍼 연동은
이 launch 통합 작업의 구현 범위 밖입니다.
