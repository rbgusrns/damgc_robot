# Leader AprilTag Drive Run Guide

Tag orientation을 사용하는 최종 정렬의 target pose, parameter, RViz 및 단계별 실차
검증은 [Leader Tag-Normal 최종 정렬 검증 가이드](../../rescue_robot_apriltag/docs/LEADER_TAG_NORMAL_ALIGNMENT_VALIDATION_GUIDE.md)를
함께 따른다.

## 1. 목적

이 문서는 ROS 2 명령에 익숙하지 않은 사용자가 Leader 로봇의 AprilTag 실제
주행 파이프라인을 안전하게 실행하고 확인하는 절차를 설명한다. 통합 launch 한
번으로 다음 경로가 실행된다.

```text
RealSense D435
  -> rectified RGB image (image_proc)
  -> AprilTag detector and tag TF
  -> base_link 기준 pose와 base alignment state
  -> approach_controller
  -> /leader/approach/cmd_vel_raw
  -> velocity_guard
  -> /leader/cmd_vel
  -> stm32_bridge
  -> I2C (/dev/i2c-7, address 0x42)
  -> STM32
  -> motor
```

Depth stream도 향후 perception 사용을 위해 함께 켜진다. 현재 AprilTag 주행
controller는 rectified color image를 사용한다.

## 2. 가장 중요한 startup safety

통합 launch 직후의 상태는 다음과 같다.

| 구성요소 | 시작 상태 | 의미 |
|---|---|---|
| `approach_controller` | **ENABLED** | 태그 상태에 따라 raw command를 자동 계산한다. |
| `velocity_guard` | **DISABLED** | raw command를 모터 경로로 통과시키지 않는다. |
| motor | **STOP** | `/leader/cmd_vel`에는 zero command만 발행된다. |

통합 launch를 실행하는 것만으로는 주행이 시작되지 않는다. 주변 안전과 topic
상태를 확인한 사용자가 `/leader/velocity_guard/enable` 서비스를 직접 호출해야
실제 주행이 시작된다.

서비스 disable은 기본 software stop이다. 전원 차단 장치나 장비의 물리적 비상
정지 방법도 시험 전에 반드시 확인한다.

## 3. 실행 전 체크리스트

- 로봇 바퀴 주변에 사람, 케이블, 공구 및 장애물이 없는지 확인한다.
- 첫 확인은 가능하면 바퀴가 바닥에 닿지 않는 안전한 상태에서 수행한다.
- Jetson과 STM32 전원이 정상인지 확인한다.
- D435의 USB 케이블과 전원을 확인한다.
- Jetson과 STM32 사이의 I2C SDA, SCL, 공통 GND 연결을 확인한다. Jetson I2C는
  3.3 V 신호를 사용해야 한다.
- `/dev/i2c-7`을 사용할 권한이 있는지 확인한다.
- 물리적 비상 정지 또는 motor 전원 차단 방법을 작업자 모두가 알고 있어야 한다.
- 다른 teleop, Nav2 또는 control launch가 실행 중이지 않은지 확인한다.
- repository가 `~/damgc_robot`에 있는지 확인한다. 다른 위치라면 아래 `cd`
  경로를 실제 위치로 바꾼다.

## 4. 빌드

새 터미널을 열고 다음 명령을 순서대로 입력한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  rescue_robot_description \
  rescue_robot_apriltag \
  leader_approach_control \
  stm32_bridge \
  rescue_robot_bringup
source install/local_setup.bash
```

- `cd`는 ROS workspace로 이동한다.
- 첫 번째 `source`는 ROS 2 Humble 명령과 기본 package를 활성화한다.
- `colcon build`는 이 통합 파이프라인에 필요한 repository 내부 package를
  빌드한다.
- 마지막 `source`는 방금 빌드한 package와 launch 파일을 현재 터미널에서 찾을
  수 있게 한다.

빌드가 `Summary: 5 packages finished`와 함께 종료되고 `Failed`가 없어야 한다.
외부 package가 없다는 오류가 나면 먼저 프로젝트의 rosdep 의존성과 D435 및
AprilTag ROS package 설치 상태를 확인한다.

## 5. Terminal 1: 통합 launch 실행

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 launch rescue_robot_bringup leader_apriltag_drive.launch.py
```

launch 시작 부분에 다음 의미의 safety 메시지가 출력되어야 한다.

```text
approach controller ENABLED, velocity guard DISABLED, motor command held at zero
```

controller 로그에는 `enabled=True`, guard 로그에는 `enabled=False`가 표시된다.
STM32 bridge가 I2C를 열면 `/dev/i2c-7`과 address `0x42`를 사용했다는 로그가
표시된다. 이 터미널은 launch가 실행되는 동안 그대로 둔다.

## 6. Terminal 2: 상태 확인

별도 터미널에서도 ROS 환경을 먼저 설정한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

실행 중인 node를 확인한다.

```bash
ros2 node list
```

camera, `robot_state_publisher`, rectify, AprilTag detector,
`/leader/apriltag_approach`, `/leader/approach_controller`,
`/leader/velocity_guard`, `/leader/stm32_bridge`가 각각 한 번만 있어야 한다.

AprilTag 상태를 계속 확인하려면 다음을 실행한다. 종료하려면 `Ctrl+C`를 누른다.

```bash
ros2 topic echo /leader/base_alignment/state
```

| 상태 | 의미 |
|---|---|
| `TURN_LEFT` | pre-align target 방향으로 좌회전이 필요하다. |
| `TURN_RIGHT` | pre-align target 방향으로 우회전이 필요하다. |
| `APPROACH` | pre-align target을 향해 접근한다. |
| `FINE_ALIGN_LEFT` / `FINE_ALIGN_RIGHT` | tag normal 기반 final yaw 제자리 정렬이 필요하다. |
| `FINAL_APPROACH` | final target까지 낮은 속도로 접근한다. |
| `TOO_CLOSE` | final target을 지나 후진 없이 복구할 수 없어 정지한다. |
| `STABILIZING` | final position과 yaw의 연속 안정성을 확인한다. |
| `ALIGNED` | final position과 yaw 정렬 완료 상태이며 정지한다. |
| `TAG_LOST` | 유효한 태그를 찾지 못해 정지한다. |

controller의 startup 설정은 다음과 같이 확인한다.

```bash
ros2 param get /leader/approach_controller controller_enabled_on_startup
```

정상 결과는 `Boolean value is: True`다. 태그가 보이고 움직임이 필요한 상태라면
다음 raw command가 자동으로 non-zero가 될 수 있다.

```bash
ros2 topic echo /leader/approach/cmd_vel_raw
```

guard의 startup 설정은 다음과 같이 확인한다.

```bash
ros2 param get /leader/velocity_guard guard_enabled_on_startup
```

정상 결과는 `Boolean value is: False`다. 주행을 시작하기 전에는 다음 출력의
`linear`와 `angular` 모든 값이 반복해서 `0.0`이어야 한다.

```bash
ros2 topic echo /leader/cmd_vel
```

파라미터는 시작 설정을 나타내며 서비스 호출 이후의 내부 enable 상태를
동적으로 바꾸어 표시하지는 않는다. 서비스 호출 시 반환되는 메시지와 실제
`/leader/cmd_vel`을 함께 확인한다.

## 7. `/leader/cmd_vel` 연결 확인

```bash
ros2 topic info /leader/cmd_vel --verbose
```

정상 구조는 다음과 같다.

```text
Publisher count: 1
  Node name: velocity_guard
  Node namespace: /leader

Subscription count: 1
  Node name: stm32_bridge
  Node namespace: /leader
```

publisher가 둘 이상이면 주행을 시작하지 않는다. 다른 teleop, Nav2, 별도 guard
또는 이전 launch process를 먼저 종료한다.

## 8. 실제 주행 시작

주변, 상태, raw command, final zero command 및 publisher 관계를 모두 확인한 뒤
다음 한 줄을 실행한다.

```bash
ros2 service call /leader/velocity_guard/enable std_srvs/srv/SetBool "{data: true}"
```

정상 응답에는 `success=True`와 fresh raw command를 기다린다는 메시지가 포함된다.
서비스 호출 전에 수신한 오래된 raw command는 재사용하지 않는다. 이후 새 raw
command가 guard의 timeout, clamp, slew limit 및 reverse protection을 통과해
`/leader/cmd_vel`로 발행되고, stm32_bridge가 이를 I2C velocity frame으로 변환해
STM32에 전송한다.

## 9. 실제 주행 정지 및 종료

기본 software stop 명령은 다음 한 줄이다.

```bash
ros2 service call /leader/velocity_guard/enable std_srvs/srv/SetBool "{data: false}"
```

정상 응답은 `success=True`와 `velocity guard disabled` 메시지이며,
`/leader/cmd_vel`은 즉시 zero가 된다. 안전한 종료 순서는 다음과 같다.

1. 위 명령으로 velocity guard를 disable한다.
2. `/leader/cmd_vel`이 zero이고 실제 motor가 멈췄는지 확인한다.
3. Terminal 1에서 `Ctrl+C`를 눌러 통합 launch를 종료한다.

## 10. 예상 주행 동작

| AprilTag 조건 | state | raw command | guard enabled 후 final command | motor 동작 |
|---|---|---|---|---|
| pre-align target 왼쪽 | `TURN_LEFT` | `angular.z > 0` | `angular.z > 0` | 좌회전 |
| pre-align target 오른쪽 | `TURN_RIGHT` | `angular.z < 0` | `angular.z < 0` | 우회전 |
| pre-align target 정면 | `APPROACH` | `linear.x > 0` | `linear.x > 0` | 전진 |
| final yaw 오차 | `FINE_ALIGN_LEFT/RIGHT` | 해당 방향 angular command | 제한된 angular command | 제자리 미세 회전 |
| final yaw 정렬 후 | `FINAL_APPROACH` | 낮은 `linear.x > 0` | 제한된 저속 command | 최종 접근 |
| 숨김 또는 timeout | `TAG_LOST` | zero | zero | 정지 |
| 너무 가까움/안정화/정렬 완료 | `TOO_CLOSE`, `STABILIZING`, `ALIGNED` | zero | zero | 정지 |

guard가 disabled인 동안에는 state와 raw command에 관계없이 final command와
motor 동작은 항상 zero/정지여야 한다.

## 11. STM32와 I2C 확인

실행 중인 bridge가 받은 실제 파라미터를 확인한다.

```bash
ros2 param get /leader/stm32_bridge transport
ros2 param get /leader/stm32_bridge i2c_device
ros2 param get /leader/stm32_bridge i2c_address
ros2 param get /leader/stm32_bridge i2c_write_enabled
```

정상 값은 각각 `i2c`, `/dev/i2c-7`, `66`, `True`다.

현재 stm32_bridge가 제공하는 telemetry와 link counter는 다음과 같다.

```bash
ros2 topic echo /leader/stm32_rx/poll_count
ros2 topic echo /leader/stm32_rx/frame_count
ros2 topic echo /leader/stm32_rx/empty_poll_count
ros2 topic echo /leader/stm32_rx/crc_errors
ros2 topic echo /leader/stm32_rx/sequence_drops
ros2 topic echo /leader/imu/data_raw
ros2 topic echo /leader/odom/raw
ros2 topic echo /leader/system_state
```

- `poll_count`는 I2C read 시도 수이며 계속 증가해야 한다.
- `frame_count`는 유효한 새 STM32 frame 수이며 telemetry 송신 중 증가해야 한다.
- `empty_poll_count`는 읽을 frame이 없었던 poll 수다.
- 증가하는 `crc_errors`는 framing, 배선, 접지 또는 전기적 noise를 의심한다.
- `sequence_drops` 증가는 telemetry frame 누락을 뜻한다.
- `imu/data_raw`, `odom/raw`, `system_state`는 해당 종류의 STM32 frame을 실제로
  받았을 때 발행된다.

## 12. 실제 바닥 검증 절차

1. 로봇 주변과 즉시 정지 수단을 확보한다.
2. 통합 launch를 실행한다.
3. motor가 움직이지 않는지 확인한다.
4. `/leader/base_alignment/state`를 확인한다.
5. AprilTag를 보여주고 state 변화를 확인한다.
6. 필요하면 `/leader/approach/cmd_vel_raw`을 확인한다.
7. `/leader/cmd_vel`이 아직 zero인지 확인한다.
8. velocity guard를 `true`로 변경한다.
9. LEFT, RIGHT, FAR/CENTER 동작을 낮은 위험 조건에서 각각 확인한다.
10. 태그를 숨겼을 때 `TAG_LOST`, final zero 및 motor 정지를 확인한다.
11. velocity guard를 `false`로 변경한다.
12. motor 정지를 확인한 다음 launch를 `Ctrl+C`로 종료한다.

실제 motor 주행은 자동 테스트하지 않는다. 이 절차의 결과는 작업자가 별도로
기록한다.

## 13. Troubleshooting

### `package not found`

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 pkg prefix rescue_robot_bringup
```

마지막 명령이 경로를 출력하지 않으면 workspace를 다시 빌드하고 같은 터미널에서
overlay를 다시 source한다.

### `launch file not found`

```bash
ls ~/damgc_robot/install/rescue_robot_bringup/share/rescue_robot_bringup/launch/
```

`leader_apriltag_drive.launch.py`가 없으면 `rescue_robot_bringup`을 다시 빌드하고
`install/local_setup.bash`를 source한다.

### D435가 인식되지 않음

```bash
rs-enumerate-devices
ros2 topic list | grep /leader/camera
```

D435가 첫 명령에 없으면 USB 포트, 케이블, 전원과 RealSense udev 설정을 확인한다.
장치는 보이지만 topic이 없으면 Terminal 1의 RealSense 오류를 확인한다.

### AprilTag state가 나오지 않음

```bash
ros2 topic hz /leader/camera/color/image_rect
ros2 topic echo /leader/apriltag/detections
ros2 run tf2_ros tf2_echo base_link leader/tag36h11:0
```

rectified image, detection, tag TF 순서로 처음 끊기는 지점을 찾는다. 태그 family는
`36h11`, 운용 target ID는 `0`이며 태그 크기와 조명, 초점 및 시야를 확인한다.

### `/leader/cmd_vel` publisher가 둘 이상임

```bash
ros2 topic info /leader/cmd_vel --verbose
ros2 node list
```

`/leader/velocity_guard` 이외의 publisher를 만든 teleop, Nav2 또는 이전 launch를
종료한다. 원인을 제거하기 전에는 guard를 enable하지 않는다.

### `/dev/i2c-7`이 없음 또는 권한 오류

```bash
ls -l /dev/i2c-7
groups
```

device가 없으면 Jetson pinmux와 I2C kernel device 설정을 확인한다. 권한 오류면
사용자가 장비 정책에 맞는 `i2c` group 권한을 보유했는지 확인한다. bus scan이
필요하면 bridge launch를 먼저 종료한 뒤 `i2cdetect -y 7`에서 `42`가 보이는지
확인한다. 실행 중 bridge와 동시에 bus scan을 하지 않는다.

### stm32_bridge가 I2C를 열지 못함

Terminal 1의 `Cannot open STM32 transport` 로그를 확인한다. device 존재와 권한,
SDA/SCL/GND, 3.3 V level, STM32 전원 및 slave address `0x42`를 확인한다.
`poll_count` 또는 `frame_count`가 증가하는지도 확인한다.

### raw command는 있지만 final command가 zero임

launch 직후라면 정상이다. guard는 의도적으로 disabled다. 주변 안전 확인 후 enable
서비스를 호출하고 성공 응답을 확인한다. enable 후에도 zero이면 raw command가
계속 새로 들어오는지, `TAG_LOST`인지, controller/guard timeout을 넘기지 않는지
확인한다.

### guard를 enable했지만 motor가 움직이지 않음

```bash
ros2 topic echo /leader/approach/cmd_vel_raw
ros2 topic echo /leader/cmd_vel
ros2 topic info /leader/cmd_vel --verbose
ros2 topic echo /leader/stm32_rx/frame_count
```

raw, final, bridge 연결, STM32 수신 순서로 확인한다. final command가 non-zero인데
움직이지 않으면 bridge의 I2C write 오류, STM32 watchdog/control 상태, motor 전원과
물리적 비상 정지 상태를 확인한다.

### `TAG_LOST`인데 motor가 멈추지 않음

즉시 물리적 비상 정지 또는 motor 전원 차단을 사용하고 시험을 중단한다. 안전을
확보한 뒤 guard disable 서비스, state/raw/final topics, `/leader/cmd_vel`의 중복
publisher, stm32_bridge timeout 및 STM32 watchdog 동작을 순서대로 확인한다.

## 14. 알려진 제한 사항

현재 실제 주행에서 AprilTag 정렬 시 로봇이 태그 중심을 기준으로 한쪽에 치우쳐
정렬되는 현상이 있다. 이번 작업은 이미 검증된 주행 파이프라인을 하나의 launch로
통합하는 작업이므로 이 현상을 수정하지 않는다.

camera-to-base TF, lateral offset, camera yaw, alignment controller 및 target pose의
분석과 보정은 별도의 후속 작업에서 수행한다. 이번 통합 launch는 기존 TF 계산,
상태 판단, tolerance, gain, target distance, safety logic, STM32 firmware와 I2C
protocol을 그대로 사용한다.
