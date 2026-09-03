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
final_target    = tag + 0.20 × robot_facing_normal
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
- 0.30 m에서 final yaw 후 0.20 m 접근

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
- yaw 4° 이내에서 FINAL_APPROACH
- 접근 중 yaw 8° 초과 시 linear zero인 FINAL_YAW_ALIGN 복귀

## 19. Final distance validation

Tag plane과 `base_link` origin 간 normal 방향 거리를 측정한다.

- pre-align 약 0.30 m
- final target 약 0.20 m
- 마지막 이동량 약 0.10 m

Camera optical z만 측정하지 말고 tilted Tag에서는 Tag normal 방향 거리를 확인한다.

## 20. ALIGNED criteria

```text
final_position_error <= 0.015 m
abs(final_yaw_error) <= 4 deg
continuous duration >= 0.8 s
```

두 error 중 하나라도 벗어나면 `STABILIZING` timer가 reset되어야 한다. Tag center가
카메라 중앙이더라도 tilted surface의 position/yaw가 틀리면 ALIGNED가 아니어야 한다.

## 21. TAG_LOST safety

접근 중 Tag를 가린다.

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
