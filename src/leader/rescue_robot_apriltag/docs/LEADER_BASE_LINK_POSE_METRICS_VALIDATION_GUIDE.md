# Leader base_link AprilTag Pose·Metric 구현 및 재현 검증 가이드

- 작성일: 2026-08-28
- 대상 장치: Leader Jetson, Intel RealSense D435
- ROS 배포판: ROS 2 Humble
- 대상 태그: `tag36h11` ID 0, 한 변 `0.050 m`
- 구현 패키지: `rescue_robot_apriltag`
- 실행 노드: `/leader/apriltag_approach`

## 1. 문서 목적과 개발 범위

이 문서는 Leader의 filtered AprilTag pose를 카메라 optical frame에서 `base_link`로
변환하는 기능의 구현 내용, 실제 D435 시험 결과, 그리고 새 Jetson에서 같은 시험을
재현하는 절차를 기록한다.

이전 단계의 흐름은 다음과 같았다.

```text
D435
  → AprilTag detection
  → camera_color_optical_frame 기준 filtered PoseStamped
  → camera 기준 distance / lateral_error / angle
  → /leader/alignment/state
```

이번 단계에서는 기존 camera-frame 출력을 바꾸지 않고 다음 분기를 병렬로 추가했다.

```text
기존 filtered PoseStamped
  → 입력 pose의 frame_id와 timestamp를 사용한 TF2 lookup
  → base_link 기준 PoseStamped
  → base_forward_distance / base_lateral_error / base_bearing
```

`camera_color_optical_frame`은 AprilTag perception의 측정 기준이고, `base_link`는 실제
Leader 차체 기준 좌표계다. 이번 기능은 향후 이동 제어가 사용할 수 있는 perception
좌표 변환까지이며 motor control, `/leader/cmd_vel`, STM32 또는 gripper를 연결하지 않는다.

## 2. 좌표계와 metric 정의

현재 Leader URDF의 `base_link` 축은 다음과 같다.

```text
+X: 로봇 전방
+Y: 로봇 왼쪽
+Z: 위쪽
```

base pose의 위치를 `(x_base, y_base, z_base)`라고 할 때 신규 metric은 다음과 같다.

```text
base_forward_distance = x_base
base_lateral_error    = y_base
base_bearing          = atan2(y_base, x_base)
```

| 값 | 의미 |
|---|---|
| `base_forward_distance` | 로봇 본체 기준 tag의 전방 위치 `position.x` [m] |
| `base_lateral_error > 0` | tag가 로봇 왼쪽 |
| `base_lateral_error < 0` | tag가 로봇 오른쪽 |
| `base_bearing > 0` | tag가 로봇 왼쪽 방향 |
| `base_bearing < 0` | tag가 로봇 오른쪽 방향 |

`base_bearing`의 production 단위는 radian이다. 사람이 시험값을 이해할 때만
`degree = radian × 180 / π`로 참고 환산하며 topic 단위는 변경하지 않는다.

## 3. 실제 구현 구조

### 3.1 변경 package와 파일

| 경로 | 실제 역할 및 변경 |
|---|---|
| `src/leader/rescue_robot_apriltag/rescue_robot_apriltag/apriltag_approach_node.py` | 기존 filtered camera pose 생성, 신규 publisher, exact-time TF lookup과 base 출력 제어 |
| `src/leader/rescue_robot_apriltag/rescue_robot_apriltag/base_pose.py` | pose/transform 검증, `do_transform_pose()`, timestamp 보존, base metric 계산 |
| `src/leader/rescue_robot_apriltag/config/approach.yaml` | `base_frame: base_link`, `tf_lookup_timeout: 0.0` 추가 |
| `src/leader/rescue_robot_apriltag/test/test_base_pose.py` | metric 부호·경계, freshness, invalid 데이터, synthetic TF, node 분기 시험 |
| `src/leader/rescue_robot_apriltag/package.xml` | `tf2_geometry_msgs` runtime dependency 추가 |
| `src/leader/rescue_robot_apriltag/CMakeLists.txt` | `test_base_pose` pytest 등록 |

패키지는 `ament_cmake`와 `ament_cmake_python`을 사용한다. `setup.py`는 없으며 변경하지
않았다. `ament_python_install_package(${PROJECT_NAME})`가 신규 `base_pose.py`를 함께
설치한다.

다음 launch는 이미 approach node를 조건부 실행할 수 있었으므로 수정하지 않았다.

```text
src/leader/rescue_robot_bringup/launch/camera_apriltag.launch.py
```

실제 launch argument는 `enable_depth`, `enable_infra`, `enable_imu`, `enable_approach`,
`approach_config`다. base 출력 시험에는 `enable_depth:=false enable_approach:=true`를
사용한다.

### 3.2 같은 cycle의 데이터 흐름

`AprilTagApproachNode._publish_valid()`가 다음 camera pose를 만든다.

- `header.frame_id`: 설정된 `source_frame`, 현재 `camera_color_optical_frame`
- `header.stamp`: 선택 tag TF의 원본 timestamp
- `position`: `MedianTranslationFilter`의 filtered x/y/z
- `orientation`: 선택 observation의 검증·정규화된 quaternion

기존 camera pose와 metric/state를 먼저 발행한 뒤 같은 지역 `PoseStamped`를
`_publish_base_outputs()`에 직접 전달한다. 별도 pose subscriber를 만들지 않았으므로
선택 tag ID, filtered translation, quaternion과 timestamp가 다른 callback cycle에서
섞이지 않는다.

## 4. 신규 ROS 2 interface

launch에서 node에 `namespace="leader"`를 적용하므로 코드의 상대 topic 이름은 다음
전체 이름으로 해석된다.

| 실제 topic | 메시지 타입 | 의미 |
|---|---|---|
| `/leader/supply/base_relative_pose` | `geometry_msgs/msg/PoseStamped` | `base_link` 기준 선택 tag pose |
| `/leader/supply/base_forward_distance` | `std_msgs/msg/Float64` | base pose의 x [m] |
| `/leader/supply/base_lateral_error` | `std_msgs/msg/Float64` | base pose의 y [m], 왼쪽 양수 |
| `/leader/supply/base_bearing` | `std_msgs/msg/Float64` | `atan2(y_base,x_base)` [rad], 왼쪽 양수 |

기존 `/leader/supply/relative_pose`, camera 기준 metric, `/leader/supply/detected`,
`/leader/supply/tag_id`, `/leader/alignment/state`의 이름과 의미는 그대로 유지된다.

## 5. TF2 변환 구현

base 변환은 다음 실제 호출 구조를 사용한다.

```python
transform = tf_buffer.lookup_transform(
    base_frame,
    camera_pose.header.frame_id,
    Time.from_msg(camera_pose.header.stamp),
    timeout=Duration(seconds=tf_lookup_timeout),
)
```

- target frame: `base_frame`, 현재 config 기본값 `base_link`
- source frame: 상수로 다시 쓰지 않고 입력 `PoseStamped.header.frame_id`
- lookup time: 입력 `PoseStamped.header.stamp`
- lookup timeout: 현재 `0.0 s`, 다음 timer cycle에서 재시도하는 non-blocking 정책
- pose 변환 API: Humble의 `tf2_geometry_msgs.do_transform_pose(Pose, TransformStamped)`

Humble의 `do_transform_pose_stamped()`는 transform header를 결과에 복사하므로 사용하지
않았다. helper가 변환된 `Pose`를 새 `PoseStamped`에 넣고 다음 header를 직접 구성한다.

```text
result.header.frame_id = base_frame
result.header.stamp    = input_pose.header.stamp
```

따라서 변환 과정에서 `now()`로 stamp를 덮어쓰지 않는다. 또한
`base_x = camera_z`, `base_y = -camera_x` 같은 optical-axis 하드코딩은 전혀 사용하지
않고 URDF와 RealSense가 제공하는 전체 TF chain을 TF2가 적용한다.

`TransformException` 또는 startup 직후 TF 미준비 상황에서는 node를 종료하지 않고
해당 sample의 base 출력만 생략한다. warning은 5초 throttle을 적용한다. 실제 D435
시험에서는 TF buffer의 최신 시각이 pose 시각보다 잠시 뒤처져 future extrapolation이
간헐적으로 발생했지만 이후 sample에서 자동 회복했고 base pose는 약 20 Hz였다.

## 6. 유효성, stale와 tag lost 처리

### 6.1 base 변환 전후 검사

실제 helper와 node는 다음을 검사한다.

- pose 존재 여부
- non-empty input `frame_id`와 target frame
- zero, future 또는 기존 `tag_timeout`보다 오래된 timestamp 거부
- input position x/y/z의 NaN/infinity 거부
- input quaternion x/y/z/w의 NaN/infinity 및 영 노름 거부, 정상 값 정규화
- transform translation의 NaN/infinity 거부
- transform quaternion의 NaN/infinity 및 영 노름 거부, 정상 값 정규화
- transformed position의 NaN/infinity 거부
- transformed quaternion의 NaN/infinity 및 영 노름 거부, 정상 값 정규화
- base metric x/y의 NaN/infinity 거부

새로운 별도 freshness timeout을 만들지 않고 기존 camera observation의 `tag_timeout=1.0 s`
기준을 재사용한다.

### 6.2 tag lost와 TF failure

tag가 없거나 camera observation이 stale이면 기존 loss 경로가 다음만 발행한다.

```text
/leader/supply/detected = false
/leader/supply/tag_id   = -1
/leader/alignment/state = TAG_LOST
```

이때 camera pose/metric과 base pose/metric은 발행하지 않는다. 마지막 화면 숫자를 0으로
바꾸지도 않는다. TF lookup만 실패하고 camera observation은 유효한 경우에는 기존
`detected`와 alignment state 의미를 바꾸지 않고 그 sample의 base topic 네 개만 생략한다.

tag가 보이고 기존 freshness 범위 안에서는 timer가 같은 원본 stamp를 잠시 다시 발행할
수 있다. 이는 새 시각으로 위장한 sample이 아니며 header stamp가 그대로다. `tag_timeout`
이후에는 pose와 metric을 더 이상 발행하지 않는다.

## 7. 실제 빌드와 자동 시험 결과

다음 명령은 2026-08-28에 이 checkout에서 실제 실행했다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select rescue_robot_apriltag

source ~/damgc_robot/install/local_setup.bash

colcon test --packages-select rescue_robot_apriltag
colcon test-result \
  --test-result-base build/rescue_robot_apriltag \
  --verbose
```

실제 결과:

```text
Build: 1 package finished
test_approach_logic: 46 passed
test_base_pose:      28 passed
ament/CTest wrapper 포함: 76 tests, 0 errors, 0 failures, 0 skipped
```

전체 workspace와 STM32/DAMGC firmware는 빌드하지 않았다. 새 checkout에서도 이번 변경만
확인할 때는 위 `--packages-select rescue_robot_apriltag` 명령을 우선 사용한다. bringup
launch 파일 자체는 이번 기능에서 바뀌지 않았다.

## 8. 실제 D435 live 검증 결과

사용 장치는 RealSense D435 serial `109622073868`, firmware `5.13.0.50`이었다.
RGB-only pipeline과 approach node를 실행해 ID 0을 이동했다.

| 시험 | forward [m] | lateral [m] | bearing [rad] | 기대 결과 | 실제 결과 |
|---|---:|---:|---:|---|---|
| CENTER | 0.5198 | -0.0181 | -0.0349 | lateral≈0, bearing≈0 | PASS, 약 -1.8 cm/-2.0° |
| LEFT | 0.5081 | +0.0708 | +0.1385 | lateral>0, bearing>0 | PASS, state `TURN_LEFT` |
| RIGHT | 0.5373 | -0.1272 | -0.2324 | lateral<0, bearing<0 | PASS, state `TURN_RIGHT` |
| FARTHER | 0.7218 | -0.0772 | -0.1066 | forward 증가 | PASS, 0.5373→0.7218 m |
| HIDDEN | - | - | - | stale 재발행 없음 | PASS, 5.03초간 pose/metric 0건 |

동일 sample timestamp 검증값:

```text
base pose stamp   = 1787920616852332764 ns
camera pose stamp = 1787920616852332764 ns
base frame_id     = base_link
```

TF cross-check 대표값:

```text
tf2_echo translation x/y       ≈ 0.452 / -0.015 m
base_relative_pose position x/y = 0.4519 / -0.0145 m
```

기존 detection은 약 16 Hz, camera filtered pose와 신규 base pose는 약 20 Hz로 관찰됐다.
가림 후 5.03초 동안 camera pose, base pose, forward, lateral, bearing은 각각 0건이었고,
`detected=false`와 `TAG_LOST`는 약 20 Hz로 유지됐다. approach node crash와 TF exception
loop는 없었다.

## 수동 실기 검증 방법

아래 명령은 저장소를 `~/damgc_robot`에 clone하고 필요한 ROS dependency를 설치·빌드한
Jetson을 기준으로 한다. **새 터미널을 열 때마다** ROS 2 Humble과 workspace overlay를
모두 source해야 한다. 기존 D435 launch가 이미 실행 중이면 먼저 해당 터미널에서
`Ctrl-C`로 종료하여 카메라와 TF publisher를 중복 실행하지 않는다.

### 터미널 1: 환경 설정 및 Leader camera/AprilTag 실행

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_depth:=false \
  enable_approach:=true
```

이 터미널은 시험이 끝날 때까지 유지한다. `RealSense Node Is Up!`,
`/leader/apriltag/apriltag`, `/leader/apriltag_approach`가 시작되는지 확인한다. tag를
보여도 계속 `TAG_LOST`이거나 process가 종료되면 다음 시험으로 진행하지 않는다.

### 터미널 2: 신규 topic 존재와 타입 확인

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 node list | grep leader
ros2 topic list | grep '^/leader/'
ros2 topic list | grep '/leader/supply/base_'

ros2 topic type /leader/supply/base_relative_pose
ros2 topic type /leader/supply/base_forward_distance
ros2 topic type /leader/supply/base_lateral_error
ros2 topic type /leader/supply/base_bearing
```

네 base topic이 모두 존재하고 앞의 interface 표와 타입이 같아야 한다. topic이 없으면
overlay source와 `enable_approach:=true`를 먼저 확인한다.

### 터미널 3: base_relative_pose 확인

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/supply/base_relative_pose
```

다음을 확인한다.

- `header.frame_id: base_link`
- `header.stamp.sec/nanosec`가 존재하고 tag TF sample에 따라 갱신됨
- `position.x/y/z`가 tag 이동에 따라 변함
- orientation 네 성분이 finite이고 영 quaternion이 아님
- tag가 보이는데도 메시지가 계속 없으면 terminal 9의 detection/TF 순서를 점검

### 터미널 4: base_forward_distance 확인

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/supply/base_forward_distance
```

tag의 좌우 위치를 가능한 비슷하게 유지하고 로봇 정면 방향으로 멀리 옮기면 값이
증가하고 가까이 옮기면 감소해야 한다. 값 단위는 meter다.

### 터미널 5: base_lateral_error 확인

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/supply/base_lateral_error
```

- 정면 중앙: 0 근처
- 로봇 기준 왼쪽: 양수
- 로봇 기준 오른쪽: 음수

카메라 장착·tag pose 측정 오차가 있으므로 중앙에서 정확한 `0.0000`을 요구하지 않는다.
좌우 위치를 충분히 이동했을 때 부호가 정의와 일치하는지가 핵심이다.

### 터미널 6: base_bearing 확인

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/supply/base_bearing
```

- 정면: 0 rad 근처
- 왼쪽: 양수
- 오른쪽: 음수

topic 단위는 radian이다. 예를 들어 `0.1385 rad`는 이해를 위한 참고로 약 `7.9°`지만
production 값과 설정을 degree로 바꾸지 않는다.

### 터미널 7: 기존 camera-frame 기능 회귀 확인

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/apriltag/detections --once
ros2 topic echo /leader/supply/relative_pose --once
ros2 topic echo /leader/alignment/state
```

첫 두 명령은 각각 실제 detection과 기존 filtered camera `PoseStamped` 한 건을 확인하고
종료한다. 마지막 명령은 계속 실행된다. tag를 좌/우/앞/뒤로 움직였을 때 기존 state가
변하면서 별도 터미널의 신규 base metric도 동시에 출력돼야 한다. 기존 state는
camera-frame metric으로 계산되므로 base metric과 같은 숫자를 기대하지 않는다.

### 터미널 8: TF cross-check

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 run tf2_ros tf2_echo base_link 'leader/tag36h11:0'
```

같은 tag 위치에서 TF translation x/y와 `/leader/supply/base_relative_pose`의 position.x/y를
비교한다. filtered pose의 median window와 관찰 시각 차이 때문에 raw TF와 수 mm~수 cm
차이는 가능하다. X/Y가 서로 바뀌거나 부호가 반대이거나 지속적으로 큰 차이가 나면
실패로 판단한다.

### 터미널 9: tag hidden / stale 확인

먼저 새 터미널에서 loss 상태를 관찰한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/supply/detected
```

다른 새 터미널에서 base pose 수신 주기를 관찰한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic hz /leader/supply/base_relative_pose
```

필요하면 아래 metric을 각각 별도 새 터미널에서 같은 방식으로 확인한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic hz /leader/supply/base_forward_distance
# 또는 아래 topic 중 하나로 바꿔 실행한다.
# /leader/supply/base_lateral_error
# /leader/supply/base_bearing
```

tag가 보일 때 pose/metric 주기가 표시되는 것을 먼저 확인한 후 tag를 완전히 가린다.
기본 `tag_timeout=1.0 s` 이후 `detected=false`, `tag_id=-1`, `TAG_LOST`가 발행되고
camera/base pose와 metric의 새 출력은 중단돼야 한다. `/leader/apriltag/detections`의
현재 payload를 별도로 확인하려면 다음을 실행한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/apriltag/detections
```

`ros2 topic echo` 화면에 마지막 숫자가 남아 있는 것은 새 메시지가 아니다. header stamp가
계속 바뀌거나 `ros2 topic hz`가 계속 새 rate를 계산할 때만 새 sample이 수신되는 것이다.
가림 뒤 base topic의 새 출력이 멈추고 node가 살아 있어야 PASS다. 각 관찰 명령은 시험 후
`Ctrl-C`로 종료한다.

## 10. timestamp 수동 확인

터미널 A:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/supply/relative_pose --field header.stamp
```

터미널 B:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/supply/base_relative_pose --field header.stamp
```

같은 tag sample의 `sec`와 `nanosec` 조합이 같아야 한다. 두 topic이 약 20 Hz로 계속
흐르므로 사람이 두 화면의 정확히 같은 sample을 매칭하기 어려울 수 있다. `--once`를
순차 실행하면 서로 다른 sample을 잡을 수 있어 불일치 증거가 아니다. 의심될 때는 짧은
ROS subscriber로 stamp를 key로 매칭하거나 자동 test의 timestamp 보존 결과를 함께 본다.

## 11. 정상 판정 기준

다음 항목을 모두 만족해야 base-frame perception 수동 검증 PASS로 판정한다.

1. `/leader/supply/base_relative_pose`가 발행된다.
2. `header.frame_id == base_link`다.
3. 같은 camera/base sample의 header stamp가 같다.
4. 정면에서 lateral과 bearing이 0 근처다.
5. 왼쪽에서 lateral과 bearing이 모두 양수다.
6. 오른쪽에서 lateral과 bearing이 모두 음수다.
7. tag가 멀어지면 forward가 증가한다.
8. raw base-to-tag TF와 base pose의 X/Y 축과 부호가 일치한다.
9. 기존 `/leader/supply/relative_pose`와 `/leader/alignment/state`가 정상이다.
10. tag lost/stale 시 마지막 값을 새 timestamp의 valid sample처럼 계속 발행하지 않는다.
11. TF 미준비 또는 tag 가림 시 approach node가 crash하지 않는다.
12. `/leader/cmd_vel`, motor 또는 STM32가 이 시험에 연결되지 않는다.

## 12. 문제 발생 시 확인 순서

### 신규 topic 자체가 없음

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 pkg executables rescue_robot_apriltag
ros2 node list
ros2 topic list
ros2 topic info /leader/supply/base_relative_pose
```

`apriltag_approach_node`가 설치됐는지, launch에 `enable_approach:=true`를 전달했는지,
현재 checkout을 build한 뒤 overlay를 source했는지 순서대로 확인한다.

### base_relative_pose가 발행되지 않음

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic echo /leader/apriltag/detections --once
ros2 topic echo /leader/supply/detected --once
ros2 topic echo /leader/supply/relative_pose --once
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link 'leader/tag36h11:0'
```

위 명령을 실행하는 터미널에도 ROS와 overlay source가 선행돼야 한다. detection → camera
filtered pose → base-to-camera TF → tag TF 순서로 끊어진 지점을 찾는다.

### TransformException 또는 extrapolation warning

- warning의 requested/latest/earliest timestamp를 비교한다.
- `base_link`, input pose `frame_id`, `leader/tag36h11:0` 철자를 확인한다.
- startup 직후 한두 sample만 실패하고 자동 회복하는지 확인한다.
- 지속 실패하면 `ros2 topic hz /tf`, camera pose stamp와 system/ROS clock을 확인한다.
- 현재 `tf_lookup_timeout=0.0`이므로 lookup 실패 sample은 생략하고 다음 timer에서 재시도한다.
- timeout을 바꾸기 전 callback 지연과 실제 TF lag를 측정하고 별도 config로 시험한다.

### 종료 시 camera_info_qos_bridge traceback

2026-08-28 live 시험에서 launch `Ctrl-C` 시 기존 `camera_info_qos_bridge.py`가 중복
`rclpy.shutdown()`으로 exit code 1을 한 번 출력했다. D435, apriltag node,
`apriltag_approach_node`는 종료됐으며 base 변환 기능의 runtime 실패는 아니었다. 정상
실행 중 traceback이 발생하거나 process가 남는다면 별도 bridge shutdown 결함으로 추적한다.

## 13. 아직 하지 않은 작업과 NOT VERIFIED

이번 단계에서 완료하지 않았거나 별도 단계로 남긴 항목은 다음과 같다.

- `gripper_approach_link` 최종 calibration: **NOT VERIFIED**
- base-frame approach control policy와 정지 거리 확정: **NOT IMPLEMENTED**
- approach controller: **NOT IMPLEMENTED**
- velocity guard: **NOT IMPLEMENTED**
- `/leader/cmd_vel` 연결: **NOT IMPLEMENTED**
- STM32 motor integration: **NOT IMPLEMENTED**
- automatic approach: **NOT IMPLEMENTED**
- gripper actuation와 automatic grasp: **NOT IMPLEMENTED**
- 장시간 부하에서 exact-time TF miss 비율과 `tf_lookup_timeout` 최종 tuning: **NOT VERIFIED**
- 다중 tag ID 1/2의 base 출력: **NOT VERIFIED**
- tag orientation의 물리 방향 정확도: **NOT VERIFIED**

이번 결과를 motor control 또는 automatic grasp 완료로 표현하면 안 된다.

## 14. Git 확인과 종료

문서와 구현 변경을 확인한다.

```bash
cd ~/damgc_robot
git status
git diff
```

시험이 끝났으면 terminal 1과 모든 echo/hz terminal에서 `Ctrl-C`를 한 번 누르고 다음으로
남은 process를 확인한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 node list
```

이번 작업에서는 자동 commit/push를 수행하지 않는다. 추천 commit message:

```text
Add Leader base-frame AprilTag pose metrics and validation guide
```
