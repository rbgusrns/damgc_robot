# Follower hybrid AprilTag alignment migration

- 기준 저장소: `/home/kde/damgc_robot`
- 분석 기준 HEAD: `47843bb10423399adfd81a86641fc1c3798ed389`
- 작성일: 2026-09-06
- 상태: software implementation and automated tests complete; robot validation pending

## 1. 범위와 보존 경계

Leader의 현재 `rescue_robot_apriltag` geometry/state machine,
`leader_approach_control` mode controller, atomic command 및 restricted blind-final
원칙을 source of truth로 사용했다. Follower에는 알고리즘 구조만 이식했으며 다음
Follower 전용 구조는 그대로 유지한다.

- namespace와 frame: `/follower/...`,
  `follower/follower_camera_optical_frame`, `follower/tag36h11:{id}`
- camera extrinsic: `xyz=(0.042, 0.01, 0.120)`, 기존 rpy 전부
- USB camera launch, calibration, detector family/size
- command path:
  `/follower/approach/cmd_vel_raw` -> command selector ->
  `/follower/selected_cmd_vel` -> velocity guard ->
  `/follower/safe_cmd_vel` -> namespaced STM32 bridge
- velocity guard 기본 disabled와 explicit enable service
- STM32 firmware/protocol, I2C `/dev/i2c-7`, address `0x42`, motor conversion
- Follower final target baseline `base_target_forward=0.25 m`

`follower_apriltag_drive.launch.py`, command selector, velocity guard, STM32 bridge,
camera driver/calibration/AprilTag detector 설정은 이 migration에서 수정하지 않았다.

## 2. Leader와 Follower 구현 대응

| 기능 | Leader 구현 | 이전 Follower | 이식 결과 | runtime/위험/검증 |
|---|---|---|---|---|
| FAR tracking | tag center bearing, turn hysteresis | camera/base center 상태 | `COARSE_TRACK`, 8/3 deg turn hysteresis | 기존 좌/우 sign unit test |
| near orientation | projected tag +Z normal | 없음 | 0.40/0.43 m engage/disengage | hysteresis tests |
| normal 선택 | robot-facing sign + median | translation median만 존재 | finite quaternion, edge-on rejection, unique-stamp median | geometry tests |
| FOV protection | `RECENTER` 18/11 deg | 없음 | center-bearing 우선 recenter | enter/exit tests |
| final phases | yaw, approach, stable, aligned | 단일 position stable 판단 | `FINAL_YAW_ALIGN`, `FINAL_APPROACH`, `STABILIZING`, `ALIGNED` | transition tests |
| final decision | position and yaw | forward/lateral/bearing | Follower forward/lateral tolerance와 yaw 동시 적용 | stable/reset tests |
| atomic API | Leader-specific typed msg | pose/state 독립 callbacks | Follower-specific typed msg | generation coherence tests |
| command output | Leader controller path | Follower selector/guard path | 기존 Follower path 유지 | launch and package tests |
| visual freshness | Leader timing contract | source stamp 중심 | source stamp=sample identity, monotonic receipt=dropout | duplicate/receipt tests |
| blind final | final-only odom fallback | 없음 | exact final-only gate, default disabled | abort/completion tests |

Repository에는 generic alignment message package가 없고
`leader_alignment_msgs/LeaderAlignmentCommand`만 있었다. Follower runtime이 Leader
package에 의존하지 않도록 `follower_alignment_msgs/FollowerAlignmentCommand`를 추가했다.
그 필드는 `std_msgs/Header header`, `geometry_msgs/Pose target_pose`,
`string control_mode`, `string alignment_state`다.

기존 pose/state topics는 diagnostics와 migration 관찰을 위해 유지한다. Controller의
authoritative input은 오직 `/follower/alignment/command`이며, 독립 pose/state topic을
제어 입력으로 구독하지 않는다.

## 3. 실제 상태와 transition

외부 alignment state에는 기존 `TURN_LEFT`, `TURN_RIGHT`, `APPROACH`, `TOO_CLOSE`,
`TAG_LOST`도 유지된다. Atomic `control_mode`는 다음 phase를 표현한다.

1. `COARSE_TRACK`: 먼 거리에서 tag center의 bearing과 turn enter/exit hysteresis 사용
2. `NEAR_ALIGN`: tag normal 기반 pre-align target을 사용하되 center visibility로 보정 제한
3. `RECENTER`: tag bearing이 18 deg 이상이면 진입, 11 deg 이하면 이탈
4. `FINAL_YAW_ALIGN`: pre-align 위치에서 final normal yaw 정렬
5. `FINAL_APPROACH`: yaw가 4 deg 이내일 때 Follower의 0.25 m target으로 저속 접근
6. `STABILIZING`: forward/lateral/yaw가 모두 tolerance 안인 상태를 0.8 s 확인
7. `ALIGNED`: 안정 조건 완료
8. `TAG_LOST`: invalid, stale, TF/normal 실패의 기본 fail-safe zero
9. `BLIND_FINAL_APPROACH`: strict gate가 모두 참일 때만 가능한 예외적 저속 전진

Near에서 range가 0.43 m를 넘으면 coarse로 복귀한다. Final approach 중 yaw error가
8 deg를 넘으면 final yaw align으로 돌아간다. 새 visual sample이 invalid하면
`TAG_LOST`; blind가 활성화된 동안 새 visual generation이 들어오면 blind 계획을
취소하고 visual state machine으로 안전하게 복귀한다.

## 4. USB timestamp 계약

과거 Follower USB camera에서 source header가 receipt보다 약 0.31--0.79 s 오래된 사례가
있었다. 그러므로 두 clock의 역할을 분리했다.

- source header stamp: TF exact-stamp conversion, duplicate/out-of-order/new sample 식별,
  장시간 이상치 sanity bound(`tag_timeout=2.0`)
- `time.monotonic()` receipt: 실제 frame arrival 중단,
  blind handoff age, blind duration 판단

같은 source stamp를 반복 lookup해도 receipt time은 갱신하지 않는다. 고정 source offset만
보고 tag loss나 blind handoff를 만들지 않는다. 새 sample이 0.35 s 동안 도착하지 않으면
visual data는 lost 처리된다. Blind handoff 관련 0.25/0.40 s 값은 활성화 전 실제 camera
측정으로 다시 검증해야 한다.

## 5. Blind-final 안전 계약

기본값은 `blind_final_approach_enabled: false`다. 비활성 상태에서 모든 tag loss는 zero
command다. 활성화해도 직전의 새 visual generation이 정확히
`state=FINAL_APPROACH`, `mode=FINAL_APPROACH`였고 다음 조건을 모두 만족해야 한다.

- last tag x가 0.35 m 이하이며 0.25 m target까지 잔여 거리가 0--0.10 m
- local receipt handoff age 0.40 s 이하
- 직전 yaw error 5 deg 이하, cross-track 0.02 m 이하
- `/follower/odom/raw`의 source stamp와 local receipt가 모두 0.25 s 이내
- odometry가 finite이고 forward-only remaining distance가 유효

Blind 중에는 0.015 m/s만 controller에 요청하며 다음 중 하나면 즉시 `TAG_LOST`와 zero로
abort한다.

- odom missing/stale, one-step jump > 0.05 m, reverse progress > 0.01 m
- start heading 기준 lateral deviation > 0.03 m
- yaw deviation > 12 deg, total forward progress > 0.10 m
- duration > 5 s 또는 non-finite data

계획 거리 도달 시 `ALIGNED`로 latch한다. FAR/COARSE/NEAR/RECENTER/yaw-only 단계의 loss는
blind에 진입할 수 없다.

## 6. Dependency 변화

```text
follower_alignment_msgs
  -> std_msgs
  -> geometry_msgs

follower_supply_perception
  -> follower_alignment_msgs (publisher)
  -> nav_msgs (Odometry subscriber)

follower_approach_control
  -> follower_alignment_msgs (authoritative subscriber)
  -> /follower/approach/cmd_vel_raw
  -> follower_command_selector (unchanged)
  -> follower_control/velocity_guard (unchanged)
  -> stm32_bridge (unchanged)
```

Message package는 `ament_cmake`, `rosidl_default_generators`,
`rosidl_generate_interfaces`, `rosidl_default_runtime`을 사용한다. 따라서 clean build에서
message package가 두 Python consumer보다 먼저 build된다. `package.xml`에는
`build_type=ament_cmake`를 명시했다. 분석한 Leader message package는 이 export가 없어
현재 workspace의 `colcon list`에서 `ros.catkin`으로 분류되는 차이가 있었으며, 그 잘못된
분류를 Follower package에 복제하지 않았다. Clean shell에서 `ros2 pkg prefix`와
`ros2 interface show`까지 확인했다.

## 7. Build와 자동시험

```bash
cd /home/kde/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  follower_alignment_msgs follower_approach_control follower_supply_perception
source install/local_setup.bash
colcon test --packages-select \
  follower_alignment_msgs follower_approach_control follower_supply_perception \
  follower_command_selector follower_control
colcon test-result --verbose
```

2026-09-06 결과는 3개 수정 패키지 build 성공, 5개 패키지 235 tests,
0 errors/failures/skips다. 자동시험은 geometry sign/normal/filter, 모든 hybrid transition,
stability, stale/loss zero, atomic coherence, blind eligibility와 abort/completion,
selector/guard 기존 동작을 포함한다.

## 8. Software-only 검증

1. 기존 수동 camera/AprilTag process를 종료한다.
2. 다음 launch를 실행한다. STM32 bridge는 생성하지 않고 guard는 disabled다.

   ```bash
   ros2 launch follower_supply_perception follower_apriltag_drive.launch.py \
     use_stm32_bridge:=false
   ```

3. 다음 topic의 publisher/type를 확인한다.

   ```bash
   ros2 topic info /follower/alignment/command --verbose
   ros2 topic echo /follower/alignment/command
   ros2 topic echo /follower/base_alignment/state
   ros2 topic echo /follower/alignment/control_mode
   ros2 topic echo /follower/approach/cmd_vel_raw
   ros2 topic echo /follower/selected_cmd_vel
   ros2 topic echo /follower/safe_cmd_vel
   ```

4. Static tag를 중앙/좌/우, far/near/final 위치와 기울기로 이동한다. Atomic message의
   pose/mode/state 한 generation과 raw command 방향이 일치하는지 확인한다.
5. Tag를 가리면 다음 publish cycle부터 `TAG_LOST`와 raw zero가 나와야 한다.
6. `/follower/safe_cmd_vel` publisher는 velocity guard 하나, hardware subscriber는 0이어야
   한다. 이 단계에서 guard를 enable하지 않는다.
7. runtime topic/frame/parameter dump에 `/leader/`, `leader/tag`, Leader camera/odom 이름이
   없어야 한다.

## 9. 실제 로봇 검증 순서

Blind는 계속 disabled로 둔 채 다음을 한 항목씩 확인한다.

복사 가능한 전체 터미널 명령과 pass/fail 기록표는
[`FOLLOWER_WHEEL_DRIVE_TERMINAL_TEST.md`](FOLLOWER_WHEEL_DRIVE_TERMINAL_TEST.md)를 따른다.

1. 바퀴를 띄우거나 충분한 안전 공간에서 좌/우 motor polarity와 emergency stop 확인
2. guard를 짧게 열어 positive linear `cmd_vel`이 실제 직진 전진인지 확인
3. positive/negative angular command의 회전 방향 확인
4. visual-only coarse/near/recenter/final-yaw/final-approach/stabilizing/aligned 확인
5. tag loss, TF failure, node stop, selector timeout, guard disable에서 즉시 정지 확인
6. `/follower/odom/raw` source와 frequency, 직진 distance 증가 방향, yaw sign 확인
7. 반복 직진에서 lateral drift와 reset/jump 분포 측정
8. final target 0.25 m가 Follower 기구/TCP에서 안전한지 측정 후 별도 tuning change로 처리
9. bridge를 사용할 때 기존 I2C address/bus/protocol과 유일한 safe topic subscriber 확인

## 10. Blind enable 승인 조건

다음 조건을 모두 기록으로 남기기 전에는 YAML을 `true`로 바꾸지 않는다.

- visual-only hybrid 전 상태와 tag-loss zero 검증 통과
- 좌/우 motor, 직진, 회전 방향 검증 통과
- 실제 odom topic이 `/follower/odom/raw`임을 graph에서 확인
- 직진 progress와 yaw sign이 코드 좌표계와 일치
- normal driving의 odom step이 0.05 m보다 충분히 작고 reset/jump가 없음
- final 구간 lateral drift < 0.03 m, yaw drift < 12 deg에 충분한 margin
- camera frame arrival 분포로 0.25/0.40 s handoff timing 재검증
- physical stop distance를 포함해 0.10 m max blind travel과 0.015 m/s가 안전함을 확인
- tag reacquisition, odom disconnect, jump/yaw/lateral abort 실제 시험 통과
- operator stop/guard disable 절차 확인

이후에도 첫 enable은 별도 config commit으로 수행하고, 한 번에 알고리즘 또는 target
distance tuning을 함께 변경하지 않는다.

## 11. 주요 regression 위험과 방어

1. pose/state callback 세대 혼합: typed atomic command 하나만 controller가 구독
2. USB source offset을 dropout으로 오판: monotonic receipt clock 분리
3. duplicate TF가 freshness 연장: strictly newer source stamp만 receipt 갱신
4. Leader frame/topic 유출: Follower runtime tree case-insensitive grep audit
5. target 0.20 m 유입: Follower 0.25 m baseline과 config contract test
6. camera extrinsic 손상: `(0.042, 0.01, 0.120)` exact test
7. FAR loss에서 blind 전진: exact FINAL_APPROACH gate와 negative unit tests
8. odom 이상 중 blind 지속: source/receipt stale와 jump/yaw/lateral/distance abort tests
9. guard 우회 publisher 생성: raw output만 controller가 소유, launch path tests
10. 기존 config 이름 무시: `target_forward`, `sample_sync_tolerance`, `allow_reverse`를
    compatibility parameter로 유지하고 atomic/forward-only 제어에서는 no-op 처리

Hardware 동작은 이 문서 작성 시점에 검증하지 않았으며 software 성공으로 대신 기록하지
않는다.
