# Leader AprilTag 상태판단 개발 및 운영 가이드

## 1. 개발 목적과 범위

이 기능은 Leader의 Intel RealSense D435가 검출한 AprilTag TF를 카메라 optical
frame 기준으로 해석하여, 태그의 상대 위치와 접근·정렬 상태를 ROS 2 토픽으로
발행한다.

처리 흐름은 다음과 같다.

```text
D435 RGB image / CameraInfo
  -> image_proc
  -> apriltag_ros
  -> camera_color_optical_frame -> leader/tag36h11:<id> TF
  -> /leader/apriltag_approach
  -> /leader/supply/* 및 /leader/alignment/state
```

현재 범위는 카메라 기준 상태판단까지다. `cmd_vel`, 모터, STM32, 그리퍼, Mission
Coordinator, Nav2는 연결하지 않았다. `base_link` 기준 변환과 실제 구동 제어는 다음
개발 단계다.

## 2. Leader 개발 환경

- 하드웨어: NVIDIA Jetson Orin Nano
- 카메라: Intel RealSense D435
- 운영체제: Ubuntu 22.04
- 미들웨어: ROS 2 Humble
- 저장소 및 ROS 2 workspace: `/home/maze/damgc_robot`
- 기본 브랜치: `main`
- Leader source root: `/home/maze/damgc_robot/src/leader`

`/home/maze/ros2_ws` 등 다른 workspace를 섞지 않는다. 명령을 실행할 때는 항상
Humble과 이 저장소의 install overlay를 순서대로 source한다.

## 3. 기존 D435 및 AprilTag 완료 상태

기존 launch는 다음 파일이다.

```text
src/leader/rescue_robot_bringup/launch/camera_apriltag.launch.py
```

이 launch의 D435, RGB image, CameraInfo, CameraInfo QoS bridge, rectification,
`apriltag_ros`, TF 출력은 Leader Jetson에서 이미 정상 동작하는 것이 확인됐다. 이번
개발에서는 해당 노드, remap, 해상도, frame 또는 AprilTag 설정을 변경하지 않고 상태
노드만 조건부로 추가했다.

`enable_approach` 기본값은 `false`다. 따라서 인자를 생략하면 검증된 기존 카메라와
AprilTag pipeline만 실행한다. `enable_approach:=true`를 지정할 때 상태 노드가 함께
실행된다. RViz는 운영 launch에 포함하지 않았다.

## 4. Follower 구현을 참고한 이유

Follower의 `src/follower/follower_supply_perception`에는 USB 카메라를 사용한 TF
유효성 검사, 다중 태그 선택, translation median filter, 상태 머신과 단위 테스트가
실기 검증돼 있었다. 동일한 상태 의미와 실패 처리 방식을 유지하기 위해 이 구조를
참고했다.

Follower 코드를 import하거나 공통 패키지로 옮기지 않았다. Leader 로직은
`rescue_robot_apriltag` 안에 별도로 작성했으며, Follower 패키지는 수정하지 않았다.

## 5. Leader에서 작성·변경한 파일

새로 작성한 구현 파일은 다음과 같다.

```text
src/leader/rescue_robot_apriltag/
├── config/approach.yaml
├── rescue_robot_apriltag/
│   ├── __init__.py
│   ├── approach_logic.py
│   └── apriltag_approach_node.py
├── test/test_approach_logic.py
└── docs/
    ├── LEADER_APRILTAG_APPROACH_SPEC.md
    ├── LEADER_APRILTAG_APPROACH_TEST_RESULTS.md
    ├── LEADER_MANUAL_STATE_TEST.md
    ├── LEADER_APRILTAG_APPROACH_GUIDE.md
    └── LEADER_IMPLEMENTATION_RECORD.md
```

수정한 기존 파일은 다음과 같다.

- `rescue_robot_apriltag/CMakeLists.txt`: Python package/executable과 pytest 등록
- `rescue_robot_apriltag/package.xml`: Python, ROS message, TF, test 의존성 등록
- `rescue_robot_bringup/launch/camera_apriltag.launch.py`: 조건부 상태 노드 통합

기존 `config/apriltag_leader.yaml`은 변경하지 않았다. 현재 이 파일에는 실제 검증에
사용한 ID 0, frame `leader/tag36h11:0`, tag size 0.050 m가 등록돼 있다.

## 6. 실제 ROS interface와 frame

실제 Leader graph와 메시지에서 확인한 입력은 다음과 같다.

| 구분 | 실제 값 |
|---|---|
| D435 node | `/leader/camera` |
| RGB 원본 | `/leader/camera/color/image_raw` |
| RGB CameraInfo | `/leader/camera/color/camera_info` |
| QoS 변환 CameraInfo | `/leader/camera/color/camera_info_transient` |
| rectified RGB | `/leader/camera/color/image_rect` |
| AprilTag node | `/leader/apriltag/apriltag` |
| detection | `/leader/apriltag/detections` |
| camera optical frame | `camera_color_optical_frame` |
| tag TF 형식 | `leader/tag36h11:<id>` |

`camera_color_optical_frame`의 축은 x 오른쪽, y 아래쪽, z 전방이다. 상태 노드의
`source_frame`과 `PoseStamped.header.frame_id`도 이 frame을 사용한다.

`tag36h11:<id>`에서 `tag36h11`은 AprilTag family이고 `<id>`는 family 안의 개별
태그 번호다. Leader 설정은 namespace 충돌을 피하기 위해 TF child frame 앞에
`leader/`를 붙인다. 예를 들어 ID 0의 실제 TF lookup은 다음과 같다.

```bash
ros2 run tf2_ros tf2_echo \
  camera_color_optical_frame leader/tag36h11:0
```

## 7. 상태판단 내부 구조

### 7.1 ROS 비의존 로직

`approach_logic.py`는 ROS import가 없는 순수 Python 모듈이다.

- `ApproachState`: 9개 출력 상태
- `ApproachThresholds`: 거리·각도·안정화 threshold와 검증
- `TagObservation`: 유효한 단일 tag TF 표본
- `RelativeMeasurement`: 필터 결과와 계산값
- `MedianTranslationFilter`: x/y/z 성분별 median filter
- `select_observation()`: priority/nearest 선택
- `ApproachStateMachine`: 판정 우선순위와 stable timer

### 7.2 ROS node wrapper

`apriltag_approach_node.py`의 노드 이름은 `apriltag_approach`다. launch에서
`namespace="leader"`를 적용하므로 전체 이름은 `/leader/apriltag_approach`다.

timer마다 후보 TF를 조회하고 다음 순서로 처리한다.

1. 후보 tag ID 결정
2. TF lookup과 fresh/finite/quaternion 검증
3. 하나의 태그 선택
4. 선택 ID가 바뀌면 filter와 stable timer 초기화
5. translation median filter
6. 기존 camera 기준 거리·각도 계산과 상태 판정
7. 기존 camera 기준 토픽 발행
8. 같은 camera PoseStamped의 원본 timestamp로 `base_link` TF lookup
9. TF2 변환 성공 시 base pose와 base metric 병렬 발행

## 8. 다중 ID와 선택 방식

- `target_tag_id >= 0`: 지정한 ID 하나만 조회한다.
- `target_tag_id == -1`: `allowed_tag_ids`에 등록된 유효 태그들을 조회한다.
- `selection_mode=priority`: `allowed_tag_ids` 배열의 앞쪽 태그를 선택한다.
- `selection_mode=nearest`: 3차원 `straight_distance`가 가장 짧은 태그를 선택한다.
- nearest 동률: `allowed_tag_ids` 앞쪽 ID를 결정적 tie-breaker로 사용한다.

선택 ID가 바뀌면 이전 태그의 translation 표본과 안정화 시간을 새 태그에 적용하지
않도록 median filter와 stable timer를 모두 초기화한다. nearest에는 hysteresis가
없으므로 거리가 비슷한 태그 사이에서 ID가 흔들릴 수 있다.

현재 `apriltag_leader.yaml`에는 ID 0만 등록돼 있다. ID 1, 2를 실기로 시험하려면 각
태그의 실제 크기를 확인한 뒤 `ids`, `frames`, `sizes`를 먼저 등록하고 카메라 launch를
재시작해야 한다.

## 9. TF 유효성, stale 처리와 median filter

다음 경우 해당 TF를 유효하지 않은 것으로 처리한다.

- `lookup_transform()`의 `TransformException`
- `now - TF stamp > tag_timeout`인 stale TF
- translation의 NaN 또는 infinity
- `z <= 0`
- quaternion 성분의 NaN/infinity, 길이 오류 또는 영 노름

유효한 quaternion은 정규화한다. 유효한 태그가 없으면 filter와 stable timer를
초기화하고 다음 세 토픽만 새 loss 값으로 발행한다.

```text
detected=false
tag_id=-1
state=TAG_LOST
```

유실 중에는 과거 pose, distance, lateral error, straight distance, angle을 새로운
측정값처럼 재발행하지 않는다. base pose와 base metric도 같은 `tag_timeout` freshness를
재사용하며, tag lost/stale 또는 base TF 실패 시 마지막 값이나 0을 다시 발행하지 않는다.

median filter는 최근 `filter_window`개의 x/y/z를 성분별로 계산해 순간 outlier를
억제한다. 상태 timer가 TF publish 주기보다 빠를 수 있으므로 동일 TF timestamp는 새
표본으로 중복 삽입하지 않는다. 태그 유실 또는 선택 ID 변경 시 filter를 비운다.

## 10. 상대 위치 계산

필터를 통과한 optical-frame translation `(x, y, z)`로 다음 값을 계산한다.

```text
distance          = z
lateral_error     = x
straight_distance = sqrt(x^2 + y^2 + z^2)
angle             = atan2(x, z)
```

단위는 translation과 distance가 m, angle이 rad다. `angle_tolerance_deg`만 설정에서
degree로 받고 내부에서 radian으로 변환한다.

같은 filtered camera PoseStamped를 TF2로 `base_link`에 변환한 뒤에는 다음 값을 별도로
계산한다. optical-frame 축을 수동으로 교환하지 않는다.

```text
base_forward_distance = x_base
base_lateral_error    = y_base
base_bearing          = atan2(y_base, x_base)
```

`base_link`의 +X는 로봇 전방, +Y는 왼쪽이므로 왼쪽에서 lateral/bearing이 양수이고
오른쪽에서 음수다. exact-time TF lookup에는 camera PoseStamped의 frame ID와 stamp를
사용하며, 결과 pose에도 원본 stamp를 복사한다.

## 11. 상태 머신

상태 판정은 아래 표의 위에서 아래 순서로 수행한다. 먼저 만족한 상태가 출력된다.

| 상태 | 조건 또는 의미 |
|---|---|
| `TAG_LOST` | fresh하고 유효한 선택 TF 없음 |
| `TURN_LEFT` | `angle < -angle_tolerance` |
| `TURN_RIGHT` | `angle > +angle_tolerance` |
| `APPROACH` | `distance > target_distance + distance_tolerance` |
| `TOO_CLOSE` | `distance < target_distance - distance_tolerance` |
| `FINE_ALIGN_LEFT` | 앞 조건 정상, `lateral_error < -lateral_tolerance` |
| `FINE_ALIGN_RIGHT` | 앞 조건 정상, `lateral_error > +lateral_tolerance` |
| `STABILIZING` | 모든 허용 범위 만족, 연속 유지 시간 미달 |
| `ALIGNED` | 모든 허용 범위를 `stable_time` 이상 연속 만족 |

허용 범위를 벗어나거나 태그가 유실되거나 선택 ID가 변경되면 stable timer가
초기화된다.

## 12. Leader 출력 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/leader/supply/detected` | `std_msgs/msg/Bool` | fresh하고 유효한 태그 여부 |
| `/leader/supply/tag_id` | `std_msgs/msg/Int32` | 선택 ID, 유실 시 -1 |
| `/leader/supply/relative_pose` | `geometry_msgs/msg/PoseStamped` | 필터 translation과 정규화 quaternion |
| `/leader/supply/distance` | `std_msgs/msg/Float64` | z 전방 거리 [m] |
| `/leader/supply/lateral_error` | `std_msgs/msg/Float64` | x 좌우 오차 [m] |
| `/leader/supply/straight_distance` | `std_msgs/msg/Float64` | 3차원 직선거리 [m] |
| `/leader/supply/angle` | `std_msgs/msg/Float64` | `atan2(x,z)` [rad] |
| `/leader/supply/base_relative_pose` | `geometry_msgs/msg/PoseStamped` | `base_link` 기준 선택 tag pose |
| `/leader/supply/base_forward_distance` | `std_msgs/msg/Float64` | base x 전방 거리 [m] |
| `/leader/supply/base_lateral_error` | `std_msgs/msg/Float64` | base y 오차 [m], 왼쪽 양수 |
| `/leader/supply/base_bearing` | `std_msgs/msg/Float64` | `atan2(y_base,x_base)` [rad], 왼쪽 양수 |
| `/leader/alignment/state` | `std_msgs/msg/String` | 9개 상태 문자열 |

consumer는 pose/metric만 보고 검출 여부를 판단하지 말고 반드시 `detected`와 메시지
수신 시각도 확인해야 한다.

## 13. 파라미터

현재 설치 기본값은 `config/approach.yaml`에 있다.

| 파라미터 | 현재값 | 설명 |
|---|---:|---|
| `source_frame` | `camera_color_optical_frame` | TF parent 및 출력 Pose frame |
| `base_frame` | `base_link` | 신규 base pose의 TF target/output frame |
| `tf_lookup_timeout` | `0.0` s | base exact-time lookup 대기시간, 기본 non-blocking |
| `tag_frame_pattern` | `leader/tag36h11:{id}` | ID별 TF child frame 형식 |
| `target_tag_id` | `0` | 0 이상은 고정 ID, -1은 다중 모드 |
| `allowed_tag_ids` | `[0, 1, 2]` | 다중 모드 후보와 priority 순서 |
| `selection_mode` | `priority` | `priority` 또는 `nearest` |
| `target_distance` | `0.15` m | 카메라 기준 목표 거리 시험값 |
| `distance_tolerance` | `0.02` m | 목표 거리 허용오차 |
| `lateral_tolerance` | `0.02` m | x 허용오차 |
| `angle_tolerance_deg` | `5.0` deg | 수평각 허용오차 |
| `tag_timeout` | `1.0` s | TF stale 판정 시험값 |
| `stable_time` | `0.8` s | `ALIGNED` 전 연속 유지 시간 |
| `publish_rate` | `20.0` Hz | 상태 timer 주기 |
| `filter_window` | `5` | median 표본 수 |

`target_distance=0.15 m`는 그리퍼/TCP 파지 거리가 아니라 카메라 기준 시험값이다.
D435 장착 위치, `base_link`와 TCP 변환, 대상 형상을 실제 측정한 뒤 조정해야 한다.

Follower의 0.3초 timeout은 실제 USB 카메라에서 false `TAG_LOST`를 발생시켰고 2.0초로
늘려 검증됐다. Leader는 이 값을 복사하지 않았다. D435의 10초 TF 측정에서 251개
서로 다른 timestamp, 중앙 간격 0.033357초(약 29.98 Hz), 최대 간격 0.166787초,
timestamp age 최대 0.067281초를 확인했다. 이를 바탕으로 충분한 여유가 있는
`tag_timeout=1.0 s`를 초기 시험값으로 선택했다. 장시간 부하, 조명, 가림 시험 후
false loss와 실제 loss 반응시간 사이의 절충을 측정하여 최종값을 승인해야 한다.

launch 자체의 추가 인자는 다음과 같다.

| launch 인자 | 기본값 | 설명 |
|---|---|---|
| `enable_approach` | `false` | 상태 노드 통합 실행 여부 |
| `approach_config` | 설치 share의 `config/approach.yaml` | 상태 노드 params file |

## 14. 빌드와 설치 확인

```bash
cd /home/maze/damgc_robot
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select rescue_robot_apriltag rescue_robot_bringup \
  --symlink-install
source /home/maze/damgc_robot/install/setup.bash

ros2 pkg executables rescue_robot_apriltag
ros2 launch rescue_robot_bringup camera_apriltag.launch.py --show-args
```

설치 경로는 다음과 같이 확인한다.

```bash
ls /home/maze/damgc_robot/install/rescue_robot_apriltag/share/\
rescue_robot_apriltag/config/approach.yaml
ls /home/maze/damgc_robot/install/rescue_robot_bringup/share/\
rescue_robot_bringup/launch/camera_apriltag.launch.py
```

## 15. 실행 방법

### 15.1 상태 노드 단독 실행

기존 camera/AprilTag launch가 실행 중이고 상태 노드가 없을 때만 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
ros2 node list | grep apriltag_approach || true

ros2 run rescue_robot_apriltag apriltag_approach_node \
  --ros-args \
  -r __ns:=/leader \
  --params-file \
  /home/maze/damgc_robot/install/rescue_robot_apriltag/share/\
rescue_robot_apriltag/config/approach.yaml
```

### 15.2 전체 camera/AprilTag/상태 launch

기존 수동 launch가 실행 중이면 먼저 그 터미널에서 한 번 `Ctrl-C`하고 노드가 사라진
것을 확인한다. 카메라 장치 충돌과 중복 node를 피하기 위해 기존 launch를 자동 kill한
뒤 새 launch를 시작하지 않는다.

```bash
cd /home/maze/damgc_robot
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
ros2 node list | sort
ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_approach:=true
```
### RGB 영상 화면 실행

```bash
cd /home/maze/damgc_robot
ros2 run rqt_image_view rqt_image_view
```

## 상태판단 확인
```bash
ros2 topic echo /leader/alignment/state
```

상태판단 없이 기존 pipeline만 실행하려면 `enable_approach:=false`를 지정하거나 해당
인자를 생략한다.

## 16. 단위 테스트

```bash
cd /home/maze/damgc_robot
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
colcon test --packages-select rescue_robot_apriltag
colcon test-result --verbose
```

기존 suite는 9개 상태, threshold 경계, stable timer reset, camera 계산식, 잘못된
translation/quaternion, median filter, 중복 timestamp, priority/nearest와 ID 변경을
검증한다. base suite는 metric 부호와 `atan2` 경계, freshness, NaN/inf와 quaternion
거부, known TF 변환, target frame 및 원본 timestamp 보존을 검증한다. 실제 D435에서의
base 출력 검증은 별도 현장 시험으로 남겨 둔다.

## 17. 실제 ROS graph 자동 검증 결과

2026-08-27에 기존 camera/AprilTag pipeline을 종료하지 않고 상태 노드만 연결해 다음을
확인했다.

- `/leader/camera`, `/RectifyNode`, `/leader/apriltag/apriltag`
- `camera_color_optical_frame -> leader/tag36h11:0`
- detection family/ID `tag36h11/0`, tag size 0.050 m
- RGB 약 27 Hz, rectified RGB 약 18.5 Hz
- tag TF 약 29.98 Hz
- `/leader/apriltag_approach`와 8개 출력 토픽 및 메시지 타입
- `detected=true`, `tag_id=0`
- 한 표본의 `(x,y,z)=(0.032603, 0.090484, 0.323555) m`
- distance 0.323555 m, lateral error 0.032603 m
- straight distance 0.337547 m, angle 0.100428 rad
- 해당 표본 상태 `TURN_RIGHT`
- 상태 노드 제거 뒤 기존 AprilTag detection publisher 유지

launch 통합 후 설치된 `camera_apriltag.launch.py --show-args`와 정적 launch 설명에서도
상태 executable이 정확히 한 번 포함됨을 확인했다. 이때 기존 전체 launch가 사용자가
실행 중이어서 D435를 중복 기동하는 전체 통합 runtime 시험은 의도적으로 수행하지
않았다.

문서 최종 감사 시점에는 사용자가 실행하던 pipeline이 이미 종료돼 Leader node와
`/leader/apriltag/detections`가 graph에 없었다. 이 상태를 바꾸기 위해 카메라 launch를
임의로 재시작하지 않았다.

위 결과는 한 위치에서의 자동 연결 검사다. 모든 상태의 물리 전이, 다중 ID, 실제 stale
loss 시간과 RViz 육안 검증을 성공했다고 의미하지 않는다.

## 18. 사용자가 수행할 물리 태그 시험

세부 기록표는 `LEADER_MANUAL_STATE_TEST.md`를 사용한다. 최소 시험 항목은 다음과 같다.

1. ID 0을 가리고 1.0초 이후 `false`, `-1`, `TAG_LOST`와 stale metric 미발행 확인
2. 태그를 좌우로 이동해 x/angle 부호와 `TURN_LEFT`, `TURN_RIGHT` 확인
3. 중앙에서 전후 이동해 `APPROACH`, `TOO_CLOSE` 확인
4. 허용 범위를 만족해 `STABILIZING` 후 0.8초 뒤 `ALIGNED` 확인
5. 시험용 tolerance로 `FINE_ALIGN_LEFT/RIGHT` 도달 확인
6. ID 1, 2를 실제 크기로 등록한 뒤 fixed/priority/nearest와 ID 변경 reset 확인
7. 장시간 부하에서 false `TAG_LOST`와 실제 가림 반응시간을 측정해 timeout 승인

현재 파라미터 조합에서는 angle 조건이 lateral 조건보다 먼저 판정되므로
`FINE_ALIGN_LEFT/RIGHT`의 물리적 도달 범위가 좁을 수 있다. 현장 시험에는 별도 params
file을 사용하고 원본 값을 결과 없이 최종 튜닝값으로 덮어쓰지 않는다.

## 19. RViz 확인

RViz는 별도 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
rviz2
```

1. Fixed Frame을 `camera_color_optical_frame`으로 설정한다.
2. TF display에서 `leader/tag36h11:<id>`를 표시한다.
3. Pose display topic을 `/leader/supply/relative_pose`로 설정한다.
4. 태그 좌우 이동 시 TF/Pose x와 angle 부호를 비교한다.
5. 태그 전후 이동 시 TF/Pose z와 distance를 비교한다.
6. Pose header frame과 TF timestamp를 확인한다.
7. 유실 중 과거 Pose가 새 timestamp로 재발행되지 않는지 확인한다.

이 RViz 시험은 아직 수행되지 않았으며 성공으로 기록하지 않는다.

## 20. 오류 해결

### 상태 노드가 보이지 않음

- 전체 launch 명령에 `enable_approach:=true`가 있는지 확인한다.
- `ros2 pkg executables rescue_robot_apriltag`에서
  `apriltag_approach_node`가 보이는지 확인한다.
- build 후 현재 저장소의 `install/setup.bash`를 다시 source한다.

### 계속 `TAG_LOST`

- `/leader/apriltag/detections`에 ID 0이 있는지 확인한다.
- TF 이름이 `leader/tag36h11:0`인지 확인한다. 비접두 `tag36h11:0`을 사용하지 않는다.
- `source_frame=camera_color_optical_frame`인지 확인한다.
- 태그 실제 한 변 길이와 `apriltag_leader.yaml`의 `sizes`가 일치하는지 확인한다.
- ROS time과 TF timestamp가 같은 clock을 쓰는지 확인한다.

### CameraInfo 또는 detection이 나오지 않음

- `/leader/camera/color/camera_info`와
  `/leader/camera/color/camera_info_transient`를 각각 확인한다.
- `/camera_info_qos_bridge`, `/RectifyNode`, `/leader/apriltag/apriltag` 노드를 확인한다.
- D435를 사용하는 기존 launch가 이미 실행 중인지 확인하고 중복 기동하지 않는다.

### 다중 ID가 선택되지 않음

`allowed_tag_ids`만 바꿔서는 `apriltag_ros`가 새 ID TF를 만들지 않는다.
`apriltag_leader.yaml`에도 각 ID, frame, 실제 size를 등록한 뒤 전체 카메라 launch를
재시작한다.

### timeout 종료 때 traceback

짧은 자동 검증에서 GNU `timeout`이 SIGINT를 보낼 때 Humble/rclpy executor 종료
경합으로 `Unable to convert call argument to Python object` traceback이 한 번
관찰됐다. 임시 상태 노드는 제거됐고 정상 동작 중에는 관찰되지 않았다. 운영 시에는
launch 터미널에서 한 번 `Ctrl-C`하고 node가 사라졌는지 확인한다. 정상 Ctrl-C에서도
재현되면 별도 shutdown 결함으로 추적한다.

## 21. 종료와 재실행

1. 전체 launch 터미널에서 한 번 `Ctrl-C`한다.
2. 다음 명령으로 관련 노드가 사라졌는지 확인한다.
3. D435 재연결이나 config 변경이 필요하면 종료 확인 후 다시 launch한다.

```bash
ros2 node list | sort
ps -ef | grep '[a]priltag_approach_node'
```

상태 노드만 단독 실행했다면 해당 터미널만 종료할 수 있다. 카메라를 계속 사용할
경우 기존 camera/AprilTag launch는 그대로 유지한다.

## 22. Git 작업 절차

현재 변경은 아직 commit하지 않은 작업 트리다. 검토와 물리 시험 결과를 확인한 뒤
Leader 경로만 명시적으로 stage한다.

```bash
cd /home/maze/damgc_robot
git status --short --branch
git diff --check
git diff -- src/follower/follower_supply_perception

git add \
  src/leader/rescue_robot_apriltag \
  src/leader/rescue_robot_bringup/launch/camera_apriltag.launch.py
git diff --cached --check
git diff --cached --stat
```

Follower diff가 비어 있는지 다시 확인하고 의미가 다른 기존 변경을 같은 commit에 섞지
않는다. 사용자가 검토한 뒤 commit과 push를 수행한다.

## 23. 다음 개발 단계

다음 단계는 camera optical frame의 상대 pose를 `base_link` 기준으로 변환하고, D435
장착 extrinsic과 Leader gripper/TCP 목표 거리를 실측하여 파라미터를 확정하는 것이다.
그 후에만 Mission Coordinator와 안전한 속도 제약을 포함한 구동 제어를 별도 단계로
설계한다.
