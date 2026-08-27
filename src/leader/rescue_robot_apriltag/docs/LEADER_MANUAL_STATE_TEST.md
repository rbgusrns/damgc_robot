# Leader AprilTag 상태판단 수동 시험

## 1. 목적과 자동 확인 범위

이 문서는 Leader D435 앞에서 실제 AprilTag를 움직여 상태 전이, 다중 ID 선택과 RViz
Pose를 사용자가 직접 확인하는 절차다. 상태 노드는 카메라 optical frame의 상대 위치만
발행하며 `cmd_vel`, 모터, STM32, 그리퍼 또는 Nav2 명령을 발행하지 않는다.

2026-08-27 자동 통합 검사에서 실제로 확인한 항목은 다음과 같다.

- 기존 camera/AprilTag 노드를 종료하거나 재시작하지 않고 상태 노드만 연결
- camera node `/leader/camera`
- rectify node `/RectifyNode`
- AprilTag node `/leader/apriltag/apriltag`
- CameraInfo frame `camera_color_optical_frame`
- AprilTag `size=0.05 m`
- detection family/ID `tag36h11/0`
- tag TF `camera_color_optical_frame -> leader/tag36h11:0`
- RGB 약 27 Hz, rectified RGB 약 18.5 Hz
- 5초 TF 표본의 중앙 timestamp 간격 약 0.033356초, 약 29.98 Hz
- 상태 node `/leader/apriltag_approach`
- 8개 출력 토픽과 메시지 타입
- 태그가 보이던 한 순간의 `detected=true`, `tag_id=0`
- 관찰값: distance 약 0.323555 m, lateral error 약 0.032603 m,
  straight distance 약 0.337547 m, angle 약 0.100428 rad
- 위 한 순간의 상태 `TURN_RIGHT`
- 상태 노드 종료 후 기존 camera/AprilTag pipeline이 계속 검출함

상태 노드 cleanup 직후에는 기존 네 pipeline 노드와 ID 0 detection이 계속 흐르는 것을
확인했다. 이후 문서 최종 감사 시점에는 사용자가 실행하던 pipeline node도 ROS graph에서
사라져 있었으며, 자동으로 재시작하지 않았다. 위 측정값은 pipeline이 정상 실행되던
통합 검사 구간의 기록이다.

이 결과는 태그를 물리적으로 움직여 각 상태 전이를 시험한 결과가 아니다. 특히
`TURN_LEFT`, `APPROACH`, `TOO_CLOSE`, `STABILIZING`, `ALIGNED`, 다중 ID 선택과
RViz Pose는 아래 절차를 사용자가 수행하기 전까지 실기 검증 완료로 기록하지 않는다.

## 2. 현재 시험 파라미터

`config/approach.yaml`의 현재 값은 다음과 같다.

| 파라미터 | 시험값 | 의미 |
|---|---:|---|
| `source_frame` | `camera_color_optical_frame` | D435 RGB optical frame |
| `tag_frame_pattern` | `leader/tag36h11:{id}` | 현재 namespaced tag TF |
| `target_tag_id` | `0` | ID 0만 추적 |
| `allowed_tag_ids` | `[0, 1, 2]` | 다중 모드 후보와 우선순위 |
| `selection_mode` | `priority` | 다중 모드 선택 방식 |
| `target_distance` | 0.15 m | 카메라 기준 시험 거리 |
| `distance_tolerance` | 0.02 m | 거리 허용오차 |
| `lateral_tolerance` | 0.02 m | 좌우 허용오차 |
| `angle_tolerance_deg` | 5.0° | 수평각 허용오차 |
| `tag_timeout` | 1.0 s | stale TF 시험 timeout |
| `stable_time` | 0.8 s | `ALIGNED` 전 연속 유지 시간 |
| `publish_rate` | 20.0 Hz | 상태 timer 주기 |
| `filter_window` | 5 | translation median 표본 수 |

`target_distance=0.15 m`는 Leader 그리퍼/TCP의 실제 파지 거리가 아니다. D435 장착
위치와 파지 대상 형상을 측정하기 전에는 시험용 카메라 기준 거리로만 사용한다.

Optical frame에서 x는 오른쪽, y는 아래쪽, z는 전방이다.

- `distance = z`
- `lateral_error = x`
- `straight_distance = sqrt(x²+y²+z²)`
- `angle = atan2(x,z)`

## 3. 사전 준비

별도 터미널에서 사용자가 기존 camera/AprilTag launch를 실행한다.

```bash
cd /home/maze/damgc_robot
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py
```

다른 터미널에서 필수 pipeline과 태그 TF를 확인한다.

```bash
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
ros2 node list | sort
ros2 topic echo /leader/apriltag/detections --once
ros2 run tf2_ros tf2_echo \
  camera_color_optical_frame leader/tag36h11:0
```

기존 상태 노드가 없는지 확인한 다음 Leader 상태 노드를 실행한다.

```bash
ros2 node list | grep apriltag_approach || true

ros2 run rescue_robot_apriltag apriltag_approach_node \
  --ros-args \
  -r __ns:=/leader \
  --params-file \
  /home/maze/damgc_robot/install/rescue_robot_apriltag/share/rescue_robot_apriltag/config/approach.yaml
```

관찰 터미널에서는 다음 토픽을 함께 확인한다.

```bash
ros2 topic echo /leader/supply/detected
ros2 topic echo /leader/supply/tag_id
ros2 topic echo /leader/supply/relative_pose
ros2 topic echo /leader/supply/distance
ros2 topic echo /leader/supply/lateral_error
ros2 topic echo /leader/supply/straight_distance
ros2 topic echo /leader/supply/angle
ros2 topic echo /leader/alignment/state
```

## 4. 단일 ID 물리 상태 시험

상태 우선순위는 angle, distance, lateral error 순서다. 거리 상태를 시험할 때는 먼저
태그를 화면 중앙에 놓아 angle을 ±5° 안으로 유지한다.

| 시험 | 실제 조작 | 예상 값과 상태 | 결과 기록 |
|---|---|---|---|
| 태그 가림 | 태그를 완전히 가리고 1.0초 이상 유지 | `detected=false`, `tag_id=-1`, `TAG_LOST` | 미확인 |
| 왼쪽 | 태그를 화면 왼쪽으로 옮겨 `angle < -5°` | x와 angle 음수, `TURN_LEFT` | 미확인 |
| 오른쪽 | 태그를 화면 오른쪽으로 옮겨 `angle > 5°` | x와 angle 양수, `TURN_RIGHT` | 한 위치만 자동 확인, 이동 시험 필요 |
| 먼 거리 | 태그를 중앙에 두고 `z > 0.17 m` | `APPROACH` | 미확인 |
| 가까운 거리 | 태그를 중앙에 두고 `z < 0.13 m` | `TOO_CLOSE` | 미확인 |
| 목표 위치 | angle ±5°, `0.13 <= z <= 0.17`, `abs(x) <= 0.02` | 즉시 `STABILIZING`, 0.8초 연속 유지 후 `ALIGNED` | 미확인 |

### TAG_LOST stale 출력 확인

태그를 가린 뒤에도 TF buffer에는 마지막 transform이 잠시 남을 수 있다. 1.0초 이후
다음을 확인한다.

- `/leader/supply/detected`가 `false`
- `/leader/supply/tag_id`가 `-1`
- `/leader/alignment/state`가 `TAG_LOST`
- relative pose, distance, lateral error, straight distance와 angle이 과거 값을 새로운
  timestamp로 계속 발행하지 않음

태그를 다시 보였을 때 filter와 stable timer가 초기화되어야 한다.

### Fine alignment 상태 참고

현재 시험값에서 angle 우선순위와 `lateral_tolerance=0.02 m`의 조합은 목표 거리 근처의
`FINE_ALIGN_LEFT/RIGHT` 도달 범위를 매우 좁게 만들거나 없앨 수 있다. 이 두 상태를
별도로 시험하려면 lateral tolerance를 임시로 더 작게 조정한 별도 params file을
사용하고, angle은 ±5° 안, 거리는 목표 범위 안에서 x만 lateral tolerance 밖으로
움직인다. 원본 `approach.yaml`을 현장 결과 없이 최종 튜닝값으로 덮어쓰지 않는다.

## 5. 여러 ID 시험 준비

현재 `apriltag_leader.yaml`은 ID 0 하나만 등록한다. ID 1과 2의 실제 priority/nearest
시험을 하려면 먼저 각 태그의 실제 크기를 확인하고 AprilTag 설정에 ID, namespaced
frame과 size를 등록해야 한다. 예시는 모두 0.050 m일 때만 사용할 수 있다.

```yaml
tag:
  ids: [0, 1, 2]
  frames:
    - "leader/tag36h11:0"
    - "leader/tag36h11:1"
    - "leader/tag36h11:2"
  sizes: [0.050, 0.050, 0.050]
```

AprilTag YAML 변경 후에는 사용자가 camera/AprilTag launch를 정상 종료하고 다시
실행해야 한다. 실행 중인 기존 pipeline을 자동으로 kill하거나 중복 실행하지 않는다.

상태 노드는 startup-only 파라미터를 사용하므로 시험 모드를 바꿀 때 상태 노드만
재시작한다.

## 6. 지정 ID와 다중 ID 시험

### 지정 ID

```bash
ros2 run rescue_robot_apriltag apriltag_approach_node \
  --ros-args -r __ns:=/leader \
  --params-file \
  /home/maze/damgc_robot/install/rescue_robot_apriltag/share/rescue_robot_apriltag/config/approach.yaml \
  -p target_tag_id:=1
```

- ID 1에서 `detected=true`, `tag_id=1` 확인
- ID 1을 가리고 ID 0만 보이면 `TAG_LOST` 유지 확인

### Priority

```bash
ros2 run rescue_robot_apriltag apriltag_approach_node \
  --ros-args -r __ns:=/leader \
  --params-file \
  /home/maze/damgc_robot/install/rescue_robot_apriltag/share/rescue_robot_apriltag/config/approach.yaml \
  -p target_tag_id:=-1 \
  -p allowed_tag_ids:="[0, 1, 2]" \
  -p selection_mode:=priority
```

- ID 0, 1, 2를 동시에 보이면 배열 앞쪽의 유효 ID가 선택되는지 확인
- 앞쪽 ID를 가리면 다음 유효 ID로 전환되는지 확인
- ID 변경 직후 이전 태그 translation이 median filter에 남지 않는지 확인
- ID 변경 후 `ALIGNED`가 유지되지 않고 `STABILIZING`부터 다시 시작하는지 확인

### Nearest

위 명령의 마지막 값을 다음과 같이 바꾼다.

```bash
-p selection_mode:=nearest
```

- 여러 태그 중 `straight_distance`가 가장 작은 ID를 선택하는지 확인
- 가까운 태그를 전후로 움직여 선택 ID가 바뀌는지 확인
- 같은 거리에서는 `allowed_tag_ids` 순서가 tie-breaker인지 확인
- 거리 차이가 작으면 hysteresis가 없어 ID가 흔들릴 수 있음을 기록

## 7. RViz Pose 확인

```bash
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
rviz2
```

1. Global Options의 Fixed Frame을 `camera_color_optical_frame`으로 설정한다.
2. `TF` display를 추가하고 `leader/tag36h11:<id>` frame을 표시한다.
3. `Pose` display를 추가한다.
4. Pose Topic을 `/leader/supply/relative_pose`로 설정한다.
5. 태그를 좌우로 움직여 TF와 Pose x 방향 및 `/leader/supply/angle` 부호를 비교한다.
6. 태그를 전후로 움직여 TF와 Pose z 및 `/leader/supply/distance`를 비교한다.
7. Pose header frame이 `camera_color_optical_frame`인지 확인한다.
8. 태그 유실 중 과거 Pose가 새 timestamp로 계속 표시되지 않는지 확인한다.

RViz 화면을 직접 확인하기 전에는 TF/Pose의 최종 육안 일치를 검증 완료로 기록하지
않는다.

## 8. 시험 기록표

| 항목 | 기록 |
|---|---|
| 시험 일시 | |
| D435 연결 방식/환경 | |
| 태그 family, ID, 실제 크기 | |
| target distance와 tolerance | |
| 실제 확인한 상태 | |
| TAG_LOST까지 걸린 시간 | |
| priority 선택 결과 | |
| nearest 선택 결과 | |
| RViz TF/Pose 일치 | |
| 조정한 파라미터와 이유 | |
| 남은 문제 | |

## 9. 종료와 cleanup

1. 상태 노드 터미널에서 한 번 `Ctrl-C`한다.
2. `/leader/apriltag_approach`가 node list에서 사라졌는지 확인한다.
3. 사용자가 계속 카메라를 사용할 경우 기존 camera/AprilTag launch는 종료하지 않는다.
4. 모든 시험을 끝낼 때만 사용자가 camera/AprilTag launch 터미널을 종료한다.

```bash
ros2 node list | sort
ps -ef | grep '[a]priltag_approach_node'
```
