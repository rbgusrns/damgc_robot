# Leader Hybrid AprilTag Alignment Validation Guide

## 1. Prerequisite

- 로봇 주변과 진행 경로를 비운다.
- 첫 검증은 바퀴가 지면에 닿지 않은 상태에서 수행한다.
- emergency stop 또는 즉시 guard를 disable할 terminal을 준비한다.
- 보정된 camera TF, D435 설정, Tag size를 변경하지 않는다.

## 2. Build

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  rescue_robot_apriltag \
  leader_approach_control \
  rescue_robot_bringup
source install/local_setup.bash
```

자동 테스트:

```bash
colcon test --packages-select \
  rescue_robot_apriltag \
  leader_approach_control \
  rescue_robot_bringup
colcon test-result --verbose
```

## 3. Launch

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup leader_apriltag_drive.launch.py
```

## 4. Startup safety

통합 launch는 approach controller enabled, velocity guard disabled로 시작한다. 따라서
`/leader/approach/cmd_vel_raw`이 non-zero일 수 있지만 `/leader/cmd_vel`과 motor는 zero다.

```bash
ros2 param get /leader/approach_controller controller_enabled_on_startup
ros2 param get /leader/velocity_guard guard_enabled_on_startup
ros2 topic echo /leader/cmd_vel geometry_msgs/msg/Twist --once
```

예상값은 controller `true`, guard `false`, final Twist zero다.

## 5. Topic diagnostics

각 명령은 별도 terminal에서 실행한다.

```bash
ros2 topic echo /leader/base_alignment/state
ros2 topic echo /leader/alignment/control_mode
ros2 topic echo /leader/supply/base_bearing
ros2 topic echo /leader/alignment/tag_normal_heading
ros2 topic echo /leader/alignment/prealign_target_pose
ros2 topic echo /leader/alignment/final_target_pose
ros2 topic echo /leader/alignment/control_target_pose
ros2 topic echo /leader/alignment/final_position_error
ros2 topic echo /leader/alignment/final_yaw_error
ros2 topic echo /leader/approach/cmd_vel_raw
ros2 topic echo /leader/cmd_vel
```

`base_bearing`, normal heading, final yaw error 단위는 radian이다. Normal heading은 raw +Z
부호가 아니라 robot-facing 선택과 median filter가 끝난 방향이다. FOV recenter 활성은
`control_mode: RECENTER`로 확인한다.

## Atomic command 확인

Controller가 사용하는 authoritative 입력은 다음 typed topic이다.

```bash
ros2 topic type /leader/alignment/command
ros2 topic echo /leader/alignment/command
```

출력 한 건에서 `header.stamp`, `target_pose`, `control_mode`,
`alignment_state`가 함께 바뀌는지 확인한다. 기존 diagnostic topic은 호환성을 위해
남아 있지만 controller decision source가 아니다.

```bash
ros2 topic echo /leader/alignment/control_mode
ros2 topic echo /leader/base_alignment/state
ros2 topic echo /leader/alignment/control_target_pose
```

Command header timestamp가 `controller_pose_timeout`보다 오래되면 controller는
지연된 diagnostic 메시지의 도착 순서와 관계없이 safe zero Twist를 출력한다.

## 6. Guard-disabled static validation

Guard를 enable하지 않은 채 Tag를 좌우와 정면으로 천천히 옮긴다.

- 0.43 m 밖: `COARSE_TRACK`
- Tag 왼쪽: `TURN_LEFT`, raw angular positive
- Tag 오른쪽: `TURN_RIGHT`, raw angular negative
- Tag 중앙: `APPROACH`, raw linear positive
- 모든 경우 `/leader/cmd_vel`은 zero

FAR에서 `control_target_pose.position`이 pre-align pose가 아니라 base Tag center pose와
일치하는지 비교한다.

## 7. Normal sign validation

Tag를 정면에서 놓고 다음 두 heading을 확인한다.

```bash
ros2 topic echo /leader/supply/base_relative_pose --once
ros2 topic echo /leader/alignment/tag_normal_heading --once
```

선택된 normal은 Tag 위치에서 robot origin 쪽을 향해야 한다. Target은 다음 관계를 가져야
한다.

```text
prealign_target = tag + 0.30 × robot_facing_normal
final_target    = tag + 0.23 × robot_facing_normal
```

Detector quaternion 부호 표현이 반대여도 target이 Tag 뒤쪽으로 생성되면 안 된다.

## 8. Tilted Tag validation

Tag 면을 왼쪽과 오른쪽으로 각각 기울인다.

```text
                    TAG
                   / ■
                  /
ROBOT
```

- normal heading이 Tag 기울기에 따라 변해야 한다.
- pre-align/final target이 같은 tilted normal line 위에 있어야 한다.
- final target yaw는 normal과 약 π rad 반대여야 한다.
- Tag의 world-fixed 정면을 계속 가리키면 실패다.

## 9. FOV recenter test

0.40 m 안에서 Tag bearing을 천천히 키운다.

- 11° 이하: `NEAR_ALIGN`
- 11~18°: raw forward 감소, normal correction 감소
- 18° 이상: `RECENTER`, raw linear zero
- left edge: angular positive
- right edge: angular negative
- RECENTER 중 11°보다 클 때 mode 유지
- 11° 이하로 돌아오면 NEAR/final phase 재개

## 10. Wheels-up test

바퀴를 띄우고 publisher가 하나인지 확인한다.

```bash
ros2 topic info /leader/cmd_vel --verbose
```

Velocity guard 하나만 publisher인 경우에만 enable한다.

```bash
ros2 service call /leader/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"
```

TURN, APPROACH, RECENTER, FINAL_YAW_ALIGN, FINAL_APPROACH별 wheel 방향을 확인한다.

## 11. Ground test

바퀴 시험 통과 후 넓은 바닥에서 시작한다. 사람이 로봇 옆에서 stop terminal을 담당하고
낮은 속도 기본값을 유지한다. Tag loss, 예상 밖 회전, 반복 oscillation이 보이면 즉시
guard를 disable한다.

## 12. Left start

Tag를 로봇 왼쪽에 둔다.

- FAR에서 Tag center 쪽으로 좌회전
- Tag가 화면 반대쪽으로 밀려나지 않는지 확인
- 중앙에 들어오면 forward
- NEAR에서 normal correction이 0.10 rad/s 이하인지 확인

## 13. Right start

Tag를 로봇 오른쪽에 둔다.

- FAR에서 우회전
- raw/final angular sign이 negative
- TURN 8°/3° hysteresis로 state chatter가 감소하는지 확인

## 14. Frontal start

Tag를 정면 중앙에 둔다.

- FAR `APPROACH`
- 0.40 m 근처에서 `NEAR_ALIGN`
- 정면 normal이면 불필요한 좌우 correction이 작아야 함
- 0.30 m에서 final yaw 후 0.23 m 접근 (최종 약 7 cm)

## 15. Tilted-left Tag

Tag 면의 normal이 로봇 기준 왼쪽을 향하도록 기울인다.

- FAR은 여전히 center tracking
- NEAR부터 normal correction 발생
- 최종 robot heading이 tilted 면에 수직
- center만 보는 비스듬한 pose에서 ALIGNED가 나오지 않음

## 16. Tilted-right Tag

반대 방향으로 기울여 같은 항목을 검증한다. Correction과 final yaw sign이 좌우 배치에
맞게 반전돼야 한다.

## 17. Expected state sequence

대표 sequence:

```text
TURN_LEFT/RIGHT or APPROACH     mode=COARSE_TRACK
              ↓
APPROACH                        mode=NEAR_ALIGN
              ↓
FINE_ALIGN_LEFT/RIGHT           mode=FINAL_YAW_ALIGN
              ↓
FINAL_APPROACH                  mode=FINAL_APPROACH
              ↓
STABILIZING
              ↓
ALIGNED
```

언제든 FOV 18° 초과 시 `RECENTER`, Tag loss 시 `TAG_LOST`로 갈 수 있다.

## 18. Final yaw validation

Pre-align 근처에서 Tag를 약간 기울인다.

- `FINE_ALIGN_LEFT/RIGHT`에서 raw linear zero
- angular sign이 `/leader/alignment/final_yaw_error`와 일치
- 최대 final angular `0.08 rad/s`
- yaw 5° 이내에서 FINAL_APPROACH
- 접근 중 yaw 8° 초과 시 linear zero인 FINAL_YAW_ALIGN 복귀

## 19. Final distance validation

Tag plane과 `base_link` origin 간 normal 방향 거리를 측정한다.

- pre-align 약 0.30 m
- final target 약 0.23 m
- 마지막 이동량 약 0.10 m

Camera optical z만 측정하지 말고 tilted Tag에서는 Tag normal 방향 거리를 확인한다.

## 20. ALIGNED criteria

```text
final_position_error <= 0.020 m
abs(final_yaw_error) <= 5 deg
continuous duration >= 0.30 s
fresh valid confirmation samples >= 3
```

두 error 중 하나라도 벗어나면 `STABILIZING` timer가 reset되어야 한다. Tag center가
카메라 중앙이더라도 tilted surface의 position/yaw가 틀리면 ALIGNED가 아니어야 한다.

## 21. TAG_LOST safety

접근 중 Tag를 가린다. `STABILIZING`에서는 최대 `0.20 s` grace 동안 state를 유지하고
정지하며, 그 시간은 stability에 포함하지 않는다. 다른 phase에서는 즉시 `TAG_LOST`다.

```bash
ros2 topic echo /leader/base_alignment/state
ros2 topic echo /leader/approach/cmd_vel_raw
ros2 topic echo /leader/cmd_vel
```

즉시 `TAG_LOST`, raw zero, final zero가 되어야 한다. 이번 알고리즘은 detection dropout
debounce를 추가하지 않는다.

## 22. Stop command

정상 시험 종료 또는 이상 동작 시:

```bash
ros2 service call /leader/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

필요하면 controller도 disable한다.

```bash
ros2 service call /leader/approach/enable \
  std_srvs/srv/SetBool "{data: false}"
```

## 23. Troubleshooting

### Detection은 있지만 base state가 TAG_LOST

```bash
ros2 topic echo /leader/supply/detected --once
ros2 run tf2_ros tf2_echo base_link 'leader/tag36h11:0'
ros2 topic echo /leader/supply/base_relative_pose --once
```

Quaternion, TF timestamp, projected normal norm, robot-facing dot가 invalid인지 node warning을
확인한다. Camera TF offset을 임의로 바꾸지 않는다.

### Raw command가 항상 zero

```bash
ros2 topic echo /leader/alignment/control_target_pose --once
ros2 topic echo /leader/alignment/control_mode --once
ros2 topic echo /leader/base_alignment/state --once
ros2 param get /leader/approach_controller controller_enabled_on_startup
```

Pose→mode→state coherent sample, target Tag ID, timeout을 확인한다.

## 19. Close-Range Blind Final Approach Validation

### 19.1 Prerequisite and build

기존 prerequisite, D435/TF 설정, emergency stop 준비를 먼저 완료한다. Build와 테스트는
다음처럼 실행한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  rescue_robot_apriltag leader_approach_control
source install/local_setup.bash
colcon test --packages-select rescue_robot_apriltag leader_approach_control
colcon test-result --verbose
```

통합 환경에서는 기존 guide의 integrated launch command를 사용한다. Guard는 처음에 OFF로
두고 `/leader/cmd_vel`이 zero인지 확인한다.

### 19.2 Diagnostics and static check

```bash
ros2 param get /leader/apriltag_approach blind_final_approach_enabled
ros2 param get /leader/apriltag_approach blind_activation_max_tag_x
ros2 param get /leader/apriltag_approach blind_max_distance
ros2 param get /leader/apriltag_approach blind_last_tag_max_age
ros2 param get /leader/apriltag_approach blind_handoff_max_age
ros2 topic echo /leader/alignment/blind_final_approach_active
ros2 topic echo /leader/alignment/last_valid_tag_x
ros2 topic echo /leader/alignment/blind_planned_distance
ros2 topic echo /leader/alignment/odom_forward_progress
ros2 topic echo /leader/odom/raw nav_msgs/msg/Odometry
ros2 topic echo /leader/alignment/control_mode
ros2 topic echo /leader/base_alignment/state
ros2 topic echo /leader/approach/cmd_vel_raw
ros2 topic echo /leader/cmd_vel
```

`blind_last_tag_max_age`는 `0.25 s`, `blind_handoff_max_age`는 `0.40 s`이며 후자가 더
큰지 확인한다. Tag가 보이는 동안에는 기존 `COARSE_TRACK`, `NEAR_ALIGN`, `RECENTER`,
`FINAL_YAW_ALIGN`, `FINAL_APPROACH` 흐름과 command가 이전과 같아야 한다.

### 19.3 Close-range positive test

Final approach에서 다음을 확인한다.

```text
last_valid_tag_x ≈ 0.26 m
final_target_distance = 0.23 m
planned_blind_distance ≈ 0.06 m
```

Tag를 가까이 이동시켜 image에서 사라지게 하면, 직전 phase가 aligned
`FINAL_APPROACH`였을 때만 다음이 나타난다.

```text
control_mode = BLIND_FINAL_APPROACH
blind_active = true
linear.x > 0, low speed
angular.z = 0
```

새 source stamp가 약 `0.25 s` 동안 갱신되지 않아 loss candidate가 되어도, callback이
약 `0.30 s`에 실행된 경우 `blind_handoff_max_age=0.40 s` 안에 있으므로 blind handoff가
가능해야 한다. 반대로 source stamp가 계속 증가하면 visual FINAL_APPROACH를 유지하고
blind를 시작하지 않아야 한다.

`odom_forward_progress`가 증가하여 계획 거리 이상이 되면 command는 즉시
`linear.x=0`, `angular.z=0`이 되고 public state가 `ALIGNED`가 된다.

`/leader/supply/base_relative_pose.header.stamp` 또는 입력 TF의
`transform.header.stamp`를 함께 확인한다. TF lookup이 계속 성공하더라도 같은 source
stamp가 반복되면 새 AprilTag sample이 아니다. `blind_last_tag_max_age` 이후에는
`tag_timeout` 전체를 기다리지 않고 close-range eligibility가 평가되어야 한다.
정상적으로 source stamp가 증가하는 동안에는 기존 visual `FINAL_APPROACH`가 유지되고
blind mode가 시작되지 않아야 한다.

### 19.4 Wheels-up and ground test

먼저 바퀴를 들어 올린 상태에서 Tag를 근접 loss시켜 blind mode 진입, forward-only command,
progress, completion zero를 확인한다. 이후 ground low-speed test에서 약 6 cm만 추가 이동하는지
확인한다. Blind 중 회전이나 reverse가 나오면 즉시 emergency stop한다.

### 19.5 Negative regression tests

다음 모든 경우는 `TAG_LOST`, zero command, no `ALIGNED`여야 한다.

| 상황 | 기대 결과 |
|---|---|
| far Tag loss | 즉시 stop |
| TURN_LEFT/RIGHT loss | 즉시 stop |
| COARSE/APPROACH loss | 즉시 stop |
| RECENTER loss | 즉시 stop |
| close하지만 큰 yaw error | 즉시 stop |
| close하지만 큰 cross-track error | 즉시 stop |
| stale/invalid last pose | 즉시 stop |
| remaining distance 초과 | 즉시 stop |
| stale/invalid/NaN odom | blind abort, no ALIGNED |
| watchdog timeout | blind abort, no ALIGNED |

### 19.6 Re-acquisition and emergency stop

Blind 중 valid Tag를 다시 보이게 하면 `BLIND_FINAL_APPROACH`가 해제되고 현재 pose 기반
visual alignment로 복귀해야 한다. Invalid pose는 복귀를 유발하지 않는다.

Blind가 odometry 목표를 정상 완료한 뒤에는 `/leader/base_alignment/state`가 한 번만
`ALIGNED`가 아니라 다음 cycle들에서도 계속 `ALIGNED`여야 한다. completed 상태에서
Tag가 다시 검출되어도 명시적인 새 approach cycle 없이 재주행하지 않는다. 현재 구현의
새 cycle 시작 방법은 `apriltag_approach` node/process 재시작이다.

```bash
ros2 topic echo /leader/base_alignment/state
ros2 topic echo /leader/alignment/control_mode
ros2 topic echo /leader/cmd_vel
```

완료 후 기대 결과는 반복되는 `ALIGNED`, `control_mode=ALIGNED`,
`linear.x=0`, `angular.z=0`이다. FAR/TURN/RECENTER 또는 misaligned final loss는
기존처럼 `TAG_LOST`와 zero command를 유지해야 한다.

Guard OFF 상태에서 raw command를 확인한 뒤 velocity guard를 enable하면 guard가 false인
명령을 최종 `/leader/cmd_vel`에 내보내지 않는지 확인한다. Emergency stop과 controller/guard
disable은 blind completion보다 우선해야 한다.

### 19.7 Troubleshooting

- blind가 시작되지 않으면 `control_mode`, last valid timestamp, last x, yaw/cross-track,
  `/leader/odom/raw` freshness를 함께 확인한다.
- far loss에서 blind가 시작되면 eligibility gate 또는 last-valid phase cache가 잘못된 것이므로
  ground test를 중단한다.
- progress가 증가하지 않으면 odom topic, frame, wheel odometry와 watchdog을 확인한다.
- angular command가 발생하면 blind mode가 아닌 visual mode가 남아 있는지 atomic command와
  raw command를 같은 시각에 확인한다.
- visual path가 이전과 달라지면 blind 관련 변경 외의 hybrid diff를 먼저 조사한다.

## 24. Visual-Only Final Alignment Validation

이번 실차 tuning은 Tag 높이를 올려 final target 약 `0.23 m`에서도 AprilTag 전체가
camera image에 유지되는 조건에서 수행한다. 목적은 odometry blind fallback이 아닌
visual `FINAL_APPROACH`의 최종 정렬과 stabilization을 먼저 검증하는 것이다.

### 24.1 Parameter and launch check

Build 후 integrated launch를 실행한다. Launch만으로 motor가 구동되지 않으며,
처음에는 velocity guard가 disabled 상태여야 한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup leader_apriltag_drive.launch.py
```

다른 terminal에서 실제 node의 startup parameter를 확인한다.

```bash
ros2 param get /leader/apriltag_approach blind_final_approach_enabled
ros2 param get /leader/apriltag_approach final_position_tolerance
ros2 param get /leader/apriltag_approach final_yaw_tolerance_deg
ros2 param get /leader/apriltag_approach base_stable_time
ros2 param get /leader/apriltag_approach final_target_distance
ros2 param get /leader/apriltag_approach aligned_confirm_samples
ros2 param get /leader/apriltag_approach stabilizing_tag_loss_grace_sec
```

Expected output은 `false`, `0.020 m`, `5.0 deg`, `0.30 s`, `0.23 m`, `3`,
`0.20 s`다. 실제 ROS 2 배포판의 출력 표현이 다르더라도 값 자체를 확인한다.

### 24.2 Runtime validation

```bash
ros2 topic echo /leader/base_alignment/state
ros2 topic echo /leader/alignment/final_position_error
ros2 topic echo /leader/alignment/final_yaw_error
ros2 topic echo /leader/alignment/control_mode
```

안전 확인과 guard 상태 확인을 마친 뒤 실제 주행을 시작한다.

```bash
ros2 service call /leader/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"
```

정상 흐름은 다음과 같다.

```text
FAR / COARSE → NEAR ALIGN → FINE ALIGN → FINAL_APPROACH
→ STABILIZING → ALIGNED
```

`ALIGNED` 판정은 final planar position error가 `0.020 m` 이내이고 final yaw
error가 `±5°` 이내인 두 조건을 동시에 만족한 뒤 `0.30 s` 유지하고 새 source
timestamp의 valid observation 3개를 확인할 때 최초로 기대한다. 이후 latch되어 short
loss와 jitter에도 `ALIGNED`/zero를 유지한다. Perception timer는 20 Hz이고 source TF는
약 30 Hz이므로 동일 timestamp 중복 count가 없어야 한다.

### 24.3 Loss, failure, and follow-up criteria

`blind_final_approach_enabled`가 `false`이므로 close-range에서 Tag가 사라지면
odometry fallback으로 계속 전진하지 않고 기존 `TAG_LOST` 및 stop behavior가
나와야 한다. 이 설정은 safety logic을 우회하지 않는다.

`ALIGNED`가 나오지 않으면 다음을 기록한다.

- `final_position_error`의 범위와 2 cm 경계 통과 여부
- `final_yaw_error`의 범위와 5° 경계 통과 여부
- `STABILIZING` 진입 여부
- `STABILIZING`에서 이탈하는 패턴과 state transition

Position error가 2 cm 근처를 넘나들거나 yaw error가 5° 근처를 넘나드는지 먼저
확인한다. 두 조건을 모두 만족하는데 `STABILIZING`만 반복될 때에만 후속 tuning에서
`base_stable_time`을 더 낮추기 전에 source timestamp 중복과 3회 confirmation을 먼저
확인한다.

### Final command만 zero

```bash
ros2 param get /leader/velocity_guard guard_enabled_on_startup
ros2 topic echo /leader/approach/cmd_vel_raw --once
ros2 topic echo /leader/cmd_vel --once
```

Guard disabled이면 정상이다. 실제 주행은 주변 안전 확인 후에만 enable한다.

### 먼 거리에서 TAG_LOST 반복

작은 Tag detection 불안정은 별도 known issue다. Larger tag, resolution, detector tuning,
제한적 dropout 처리를 후속 작업으로 검토한다. 이번 controller 검증 중 Tag size나 detector
설정을 바꾸지 않는다.
