# Follower 실제 바퀴 주행 터미널 시험 절차

- 대상: Follower Jetson + STM32 + I2C motor system
- ROS 2: Humble
- workspace: `/home/kde/damgc_robot`
- 최초 작성일: 2026-09-06
- blind final: 이 절차 전체에서 **disabled 유지**

이 문서는 실제 바퀴를 굴리며 Follower의 다음 command path를 단계적으로 검증하기 위한
복사 가능한 터미널별 절차다.

```text
manual /follower/cmd_vel 또는 AprilTag controller
  -> follower_command_selector
  -> /follower/selected_cmd_vel
  -> velocity_guard
  -> /follower/safe_cmd_vel
  -> stm32_bridge
  -> I2C
  -> STM32
  -> motors
```

`/follower/safe_cmd_vel`에 직접 publish하지 않는다. Selector와 velocity guard를
우회하는 시험은 이 문서의 범위가 아니다.

## 0. 반드시 지킬 안전 조건

처음부터 지상에서 시험하지 않는다. 아래 순서를 바꾸지 않는다.

1. 첫 시험은 구동 바퀴가 지면에서 완전히 뜬 견고한 stand 위에서 수행한다.
2. 로봇 주변과 바퀴 주변에서 사람, 케이블, 공구를 치운다.
3. 물리 전원 차단 또는 emergency stop 담당자 한 명을 로봇 옆에 둔다.
4. 명령 담당자는 아래의 `Terminal 2 — SAFETY`를 항상 열어 둔다.
5. Tag 자동 접근 전에는 로봇 전방에 최소 1 m 이상의 빈 공간을 확보한다.
6. 최초 시험에서 `blind_final_approach_enabled`를 변경하지 않는다.
7. 명령 방향이 예상과 다르거나 통신 오류가 발생하면 tuning을 계속하지 말고 즉시
   guard disable 후 motor power를 차단한다.

ROS service는 물리 emergency stop을 대체하지 않는다. ROS graph나 Jetson이 멈추면
service 명령도 전달되지 않을 수 있다.

## 1. 공통 빌드 — Terminal 0

```bash
cd /home/kde/damgc_robot
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  follower_alignment_msgs \
  follower_approach_control \
  follower_command_selector \
  follower_control \
  follower_supply_perception \
  stm32_bridge

source install/setup.bash

colcon test --packages-select \
  follower_alignment_msgs \
  follower_approach_control \
  follower_command_selector \
  follower_control \
  follower_supply_perception

colcon test-result --verbose
```

Pass 기준:

- build 실패 없음
- test errors/failures 없음
- `follower_alignment_msgs/msg/FollowerAlignmentCommand`를 찾을 수 있음

```bash
ros2 interface show follower_alignment_msgs/msg/FollowerAlignmentCommand
```

## 2. I2C receive-only 사전 점검 — Terminal 1

이 단계에서는 bridge를 실행하지만 command write를 끈다. 바퀴가 움직이면 안 된다.

```bash
cd /home/kde/damgc_robot
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch follower_supply_perception follower_apriltag_drive.launch.py \
  use_stm32_bridge:=true \
  i2c_device:=/dev/i2c-7 \
  i2c_address:=66 \
  i2c_write_enabled:=false
```

Terminal 3에서 다음을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source /home/kde/damgc_robot/install/setup.bash

ros2 param get /follower/stm32_bridge transport
ros2 param get /follower/stm32_bridge i2c_device
ros2 param get /follower/stm32_bridge i2c_address
ros2 param get /follower/stm32_bridge i2c_write_enabled
ros2 topic echo /follower/stm32_rx/frame_count --once
ros2 topic echo /follower/stm32_rx/crc_errors --once
ros2 topic echo /follower/stm32_rx/sequence_drops --once
ros2 topic hz /follower/odom/raw
```

Pass 기준:

- `transport=i2c`, device `/dev/i2c-7`, address `66`, write `False`
- bridge log에 transport open 성공
- STM32가 telemetry를 보내는 firmware라면 frame count가 시간에 따라 증가
- CRC error와 sequence drop이 지속 증가하지 않음
- 바퀴가 움직이지 않음

Fail이면 Terminal 1에서 `Ctrl-C` 후 물리 전원을 끄고 I2C 배선, firmware, 권한부터
확인한다. `i2cdetect`는 동작 중인 slave에 영향을 줄 수 있으므로 이 절차에서 자동으로
실행하지 않는다.

점검이 끝나면 Terminal 1에서 `Ctrl-C`한다.

## 3. 실제 command-write launch — Terminal 1

Tag를 카메라에서 치우고 로봇을 stand에 올린 상태에서 실행한다. Launch는 controller를
enabled, selector를 `APPROACH`, velocity guard를 **disabled**로 시작한다.

```bash
cd /home/kde/damgc_robot
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch follower_supply_perception follower_apriltag_drive.launch.py \
  use_stm32_bridge:=true \
  i2c_device:=/dev/i2c-7 \
  i2c_address:=66 \
  i2c_write_enabled:=true
```

Launch log에서 다음 내용을 확인한다.

```text
approach controller ENABLED
command selector APPROACH
velocity guard DISABLED
```

## 4. 비상 정지 전용 — Terminal 2 (항상 열어 둘 것)

```bash
source /opt/ros/humble/setup.bash
source /home/kde/damgc_robot/install/setup.bash
```

가장 먼저 selector를 STOP으로 바꾸고 guard가 꺼져 있는지 확인한다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"

ros2 param set /follower/command_selector source_mode STOP

ros2 param get /follower/velocity_guard guard_enabled_on_startup
ros2 param get /follower/command_selector source_mode
ros2 param get /follower/apriltag_approach blind_final_approach_enabled
```

기대값은 guard `False`, selector `STOP`, blind final `False`다.

시험 중 정지가 필요하면 아래 두 명령을 이 순서로 실행한다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"

ros2 param set /follower/command_selector source_mode STOP
```

바퀴가 즉시 정지하지 않으면 명령을 반복하지 말고 물리 emergency stop 또는 motor
전원을 사용한다.

## 5. 토픽 감시 — Terminal 3

새 tab 또는 별도 terminal마다 공통 환경을 적용한다.

```bash
source /opt/ros/humble/setup.bash
source /home/kde/damgc_robot/install/setup.bash
```

먼저 command path ownership을 확인한다.

```bash
ros2 topic info /follower/approach/cmd_vel_raw --verbose
ros2 topic info /follower/selected_cmd_vel --verbose
ros2 topic info /follower/safe_cmd_vel --verbose
```

Pass 기준:

- `/follower/approach/cmd_vel_raw`: approach controller publisher 1
- `/follower/selected_cmd_vel`: command selector publisher 1, guard subscriber 1
- `/follower/safe_cmd_vel`: velocity guard publisher 1, STM32 bridge subscriber 1
- `/follower/safe_cmd_vel`에 다른 publisher가 없음

시험 중에는 필요에 따라 다음 명령을 각각 별도 tab에서 실행한다.

```bash
ros2 topic echo /follower/selected_cmd_vel
```

```bash
ros2 topic echo /follower/safe_cmd_vel
```

```bash
ros2 topic echo /follower/odom/raw
```

```bash
ros2 topic echo /follower/stm32_rx/frame_count
```

## 6. Stand 위 수동 저속 바퀴 pulse — Terminal 4

수동 명령도 selector와 guard를 통과시키기 위해 source를 `COOPERATION`으로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
source /home/kde/damgc_robot/install/setup.bash

ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
ros2 param set /follower/command_selector source_mode COOPERATION
```

각 pulse는 guard를 연 뒤 2초 동안 10 Hz 명령을 보내고 다시 guard를 닫는다. 명령 종료
후 selector와 guard watchdog도 각각 0.50/0.30 s 이내에 zero를 강제한다.

### 6.1 아주 느린 직진

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"

timeout --signal=INT --kill-after=1s 2s \
  ros2 topic pub --rate 10 /follower/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.02}, angular: {z: 0.0}}"

ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

기대 결과: 좌·우 바퀴가 모두 로봇 전진 방향으로 회전한다.

### 6.2 아주 느린 좌회전

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"

timeout --signal=INT --kill-after=1s 2s \
  ros2 topic pub --rate 10 /follower/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.15}}"

ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

기대 결과: 위에서 봤을 때 로봇이 반시계 방향(+yaw)으로 회전하는 wheel direction이다.

### 6.3 아주 느린 우회전

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"

timeout --signal=INT --kill-after=1s 2s \
  ros2 topic pub --rate 10 /follower/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.15}}"

ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

기대 결과: 위에서 봤을 때 로봇이 시계 방향(-yaw)으로 회전하는 wheel direction이다.

세 시험 중 하나라도 반대 방향이면 이후 지상 시험을 하지 않는다. STM32 motor polarity,
left/right mapping 또는 kinematic sign 문제를 별도 진단한다. 이 migration의 controller
gain이나 camera TF로 motor sign을 보정하지 않는다.

## 7. Fail-safe 정지 시험 — Stand 유지

### 7.1 Publisher 중단 watchdog

6.1의 2초 pulse가 끝난 뒤 별도 조작 없이 `/follower/selected_cmd_vel`과
`/follower/safe_cmd_vel`이 zero가 되는지 Terminal 3에서 확인한다.

### 7.2 Guard disable

2초 pulse 도중 Terminal 2에서 다음을 실행한다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

바퀴와 safe command가 즉시 zero로 수렴해야 한다.

### 7.3 Selector STOP

Guard를 enable하고 저속 명령을 발행하는 동안 Terminal 2에서 다음을 실행한다.

```bash
ros2 param set /follower/command_selector source_mode STOP
```

Source 변경 시 cache가 삭제되고 zero가 나와야 한다. 시험 후 guard도 닫는다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

### 7.4 음수 직진 거부

Guard의 `allow_reverse=false`를 확인한다.

```bash
ros2 param get /follower/velocity_guard allow_reverse
```

초기 하드웨어 시험에서는 reverse command를 실제 바퀴에 보내지 않는다.

## 8. 지상 수동 주행과 odometry 방향 시험

Stand 시험이 모두 통과한 뒤에만 평탄하고 넓은 바닥으로 내린다. 속도는 그대로
`0.02 m/s`, 회전은 `0.15 rad/s`, pulse는 최대 2초를 유지한다.

Terminal 4에서 source를 다시 수동으로 선택한다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
ros2 param set /follower/command_selector source_mode COOPERATION
```

Terminal 3에서 시작 odom을 기록한다.

```bash
ros2 topic echo /follower/odom/raw --once
```

Terminal 4에서 6.1의 직진 pulse를 한 번만 실행한 후 다시 기록한다.

```bash
ros2 topic echo /follower/odom/raw --once
```

Pass 기준:

- 로봇이 실제 전진
- odom의 전진 progress가 증가
- 정지 후 odom이 큰 폭으로 jump/reset되지 않음
- 눈에 띄는 lateral drift가 없음

이어서 6.2와 6.3의 회전 pulse를 각각 한 번 실행하고 odom quaternion 및 실제 회전
방향을 비교한다.

- `angular.z > 0`: 실제 좌회전, odom yaw 증가
- `angular.z < 0`: 실제 우회전, odom yaw 감소
- 회전 종료 후 position/yaw가 비정상적으로 reset되지 않음

odom sign 또는 방향이 맞지 않으면 blind-final을 절대 enable하지 않는다.

## 9. AprilTag 자동 주행 — Guard를 열기 전 관찰

수동 publisher가 모두 종료됐는지 확인하고 selector를 `APPROACH`로 바꾼다. Source 변경은
기존 수동 command cache를 삭제한다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
ros2 param set /follower/command_selector source_mode APPROACH
ros2 service call /follower/approach/enable \
  std_srvs/srv/SetBool "{data: true}"
```

로봇 정면 0.8--1.0 m 정도에 target tag ID 0을 놓는다. 뒤에는 최소 1 m의 빈 공간을
확보하고, 최종 0.25 m가 실제 기구와 충돌하지 않는지 먼저 줄자로 확인한다.

Terminal 3의 별도 tab에서 다음을 관찰한다.

```bash
ros2 topic echo /follower/alignment/command
```

```bash
ros2 topic echo /follower/base_alignment/state
```

```bash
ros2 topic echo /follower/alignment/control_mode
```

```bash
ros2 topic echo /follower/approach/cmd_vel_raw
```

Guard가 disabled인 동안 다음을 확인한다.

- FAR에서 `COARSE_TRACK`과 tag center 기반 좌/우 command
- 가까워지면 `NEAR_ALIGN`
- tag가 FOV 가장자리로 가면 `RECENTER`
- final 구간에서 `FINAL_YAW_ALIGN`, `FINAL_APPROACH`
- target에서 `STABILIZING` 후 `ALIGNED`
- atomic message의 pose, mode, state가 같은 cycle에서 일치
- tag를 가리면 raw zero. FINAL_APPROACH/STABILIZING이면 state/mode는 최대 0.30 s 유지된
  뒤 `TAG_LOST`, 다른 phase는 기존 loss 정책 적용

이 단계에서 상태/sign이 하나라도 틀리면 guard를 열지 않는다.

## 10. AprilTag 자동 저속 실주행

Tag와 로봇을 다시 안전한 시작 위치에 둔다. Terminal 2에서 즉시 정지할 준비를 하고,
최초 run은 5초 이내로 제한한다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"

sleep 5

ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

5초 전에 이상 동작이 보이면 기다리지 말고 Terminal 2에서 guard disable 또는 물리
emergency stop을 사용한다.

짧은 run을 반복하여 다음 순서로 확인한다.

1. 중앙 FAR tag를 향한 직진
2. 좌측/우측 tag center에 대한 올바른 회전 sign
3. NEAR에서 tag normal 방향으로 정렬
4. RECENTER 진입 시 전진 억제와 center 회복
5. FINAL_YAW_ALIGN에서 회전만 수행
6. FINAL_APPROACH에서 제한된 저속 전진
7. position과 yaw를 0.30 s 유지하고 fresh sample 3회 확인 뒤에만 `ALIGNED`
8. tag loss 시 즉시 zero. FINAL_APPROACH/STABILIZING의 state/control mode만 0.30 s grace
   동안 유지되고, pose나 blind-forward command가 나오지 않아야 함
9. grace 안에 strictly newer tag sample이 들어오면 visual control로 복구하고, duplicate
   TF만 반복될 때는 grace timer가 reset되지 않는지 확인
10. controller를 disable 후 enable해 새 approach session을 시작하면 이전 `ALIGNED` latch가
    남지 않고 fresh tag sample부터 다시 판정하는지 확인

각 run 사이에는 guard를 닫고 robot/tag 위치와 log를 확인한다. 한 번에 distance, gain,
camera extrinsic을 함께 tuning하지 않는다.

## 11. Blind-final 활성화 전 데이터 수집

Blind-final은 계속 `false`로 둔다.

```bash
ros2 param get /follower/apriltag_approach blind_final_approach_enabled
ros2 param get /follower/apriltag_approach stabilizing_tag_loss_grace_sec
ros2 param get /follower/apriltag_approach final_approach_tag_loss_grace_sec
ros2 topic echo /follower/approach/enabled
ros2 topic hz /follower/odom/raw
ros2 topic echo /follower/odom/raw
ros2 topic echo /follower/alignment/last_valid_tag_x
ros2 topic echo /follower/alignment/final_yaw_error
ros2 topic echo /follower/alignment/odom_forward_progress
```

최소한 다음을 기록한다.

- 직진 시 odom distance 증가 방향
- positive/negative yaw sign
- 0.02--0.10 m 직진에서 lateral drift
- 정상 frame 간 최대 odom step
- STM32/bridge 재시작 또는 통신 재연결 시 odom reset/jump 유무
- tag source generation 간 local arrival gap

다음 중 하나라도 미확인 또는 실패면 blind-final을 enable하지 않는다.

- visual-only hybrid 전체 동작
- 좌/우 motor와 직진/회전 sign
- 실제 odom topic과 update rate
- forward/yaw sign
- lateral drift margin
- odom jump/reset 안전성
- tag reacquisition과 odom loss에서 정지

Blind enable은 이 시험과 분리된 config 변경·review·별도 현장 시험으로 수행한다.

## 12. 정상 종료

항상 다음 순서로 종료한다.

Terminal 2:

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
ros2 param set /follower/command_selector source_mode STOP
ros2 service call /follower/approach/enable \
  std_srvs/srv/SetBool "{data: false}"
```

Terminal 3에서 zero를 확인한다.

```bash
ros2 topic echo /follower/safe_cmd_vel --once
```

그 다음 Terminal 1에서 `Ctrl-C`하고 모든 ROS process가 종료된 뒤 motor 전원을 끈다.

```bash
ros2 node list | sort
```

Follower drive 관련 node가 남아 있으면 다시 전원을 넣거나 주행 시험을 이어가지 않는다.

## 13. 시험 기록표

| 항목 | 기대 결과 | 결과 |
|---|---|---|
| I2C receive-only | frame 증가, motor 정지 | PASS / FAIL / NOT TESTED |
| command ownership | raw -> selector -> guard -> bridge 1:1 | PASS / FAIL / NOT TESTED |
| guard startup | disabled | PASS / FAIL / NOT TESTED |
| blind startup | disabled | PASS / FAIL / NOT TESTED |
| +linear wheel sign | 양쪽 전진 | PASS / FAIL / NOT TESTED |
| +angular wheel sign | 좌회전 | PASS / FAIL / NOT TESTED |
| -angular wheel sign | 우회전 | PASS / FAIL / NOT TESTED |
| publisher timeout | zero 정지 | PASS / FAIL / NOT TESTED |
| guard disable | 즉시 zero | PASS / FAIL / NOT TESTED |
| selector STOP | cache 삭제 후 zero | PASS / FAIL / NOT TESTED |
| ground straight | 실제 전진, odom forward 증가 | PASS / FAIL / NOT TESTED |
| ground rotation | 실제 방향과 odom yaw sign 일치 | PASS / FAIL / NOT TESTED |
| odom reset/jump | 허용 불가 | PASS / FAIL / NOT TESTED |
| visual COARSE/NEAR | 올바른 state와 command | PASS / FAIL / NOT TESTED |
| RECENTER | FOV 회복, 전진 억제 | PASS / FAIL / NOT TESTED |
| final phases | yaw -> approach -> stable -> aligned | PASS / FAIL / NOT TESTED |
| tag loss | zero command | PASS / FAIL / NOT TESTED |
| 종료 후 node | drive node 잔류 없음 | PASS / FAIL / NOT TESTED |

실제로 수행하지 않은 항목은 반드시 `NOT TESTED`로 남긴다.
