# Leader AprilTag 상대 위치 상태판단 조사 명세

## 1. 문서 목적과 조사 범위

이 문서는 Leader의 기존 RealSense D435/AprilTag 파이프라인에 카메라 optical
frame 기준 상태판단 노드를 추가하기 전에 수행한 저장소 및 ROS 2 환경 조사 결과를
기록한다. 이번 단계에서는 상태판단 Python 코드, 파라미터 YAML, 테스트 또는 launch
통합을 구현하지 않는다.

- 저장소: `/home/maze/damgc_robot`
- 운영체제/ROS: Ubuntu 22.04, ROS 2 Humble
- 대상 패키지: `src/leader/rescue_robot_apriltag`
- 통합 launch 패키지: `src/leader/rescue_robot_bringup`
- 참조 전용 패키지: `src/follower/follower_supply_perception`

Follower 패키지는 읽기 전용 참조로 조사했으며 수정하지 않았다. 상태판단 구현은 향후
Leader 패키지 안에 독립적으로 작성하고, 공통 패키지 리팩터링은 수행하지 않는다.

## 2. Git과 저장소 상태

조사 시점의 결과는 다음과 같다.

| 항목 | 확인 결과 |
|---|---|
| `pwd` | `/home/maze/damgc_robot` |
| Git top-level | `/home/maze/damgc_robot` |
| 현재 branch | `main` |
| upstream 상태 | `main...origin/main`, ahead/behind 표시 없음 |
| 기존 staged 변경 | 없음 |
| 기존 unstaged 변경 | 없음 |
| 기존 untracked 변경 | 없음 |

따라서 조사 시작 시점의 worktree는 깨끗했다. 사용자 변경을 삭제하거나 reset하는 작업은
수행하지 않았다.

## 3. Leader 패키지 현황

### 3.1 `rescue_robot_apriltag`

현재 `ament_cmake` 패키지이며 상태판단용 Python package/test/docs 구조는 아직 없다.

```text
rescue_robot_apriltag/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── apriltag_leader.yaml
└── scripts/
    └── camera_info_qos_bridge.py
```

- `CMakeLists.txt`는 config 디렉터리와 `camera_info_qos_bridge.py`만 설치한다.
- `package.xml`의 현재 실행 의존성은 `apriltag_ros`, `image_proc`, `rclpy`,
  `sensor_msgs`이다.
- `camera_info_qos_bridge.py`는
  `/leader/camera/color/camera_info`를 reliable/volatile로 구독하고
  `/leader/camera/color/camera_info_transient`에 reliable/transient-local로
  재발행한다.
- 현재 AprilTag 설정은 family `36h11`, ID 0, 크기 0.050 m이며 child frame을
  `leader/tag36h11:0`으로 지정한다.

### 3.2 `rescue_robot_bringup`

현재 `ament_cmake` 패키지이며 launch, config와 Python 보조 스크립트를 CMake에서
설치한다. `package.xml`에는 `rescue_robot_apriltag`, `realsense2_camera`,
`robot_state_publisher` 등이 실행 의존성으로 선언되어 있다.

상태판단 통합 대상은 기존에 D435와 AprilTag TF까지 검증된
`launch/camera_apriltag.launch.py`이다. 다른 localization, visual SLAM, nvblox
launch는 이번 카메라 기준 상태판단 범위에 포함하지 않는다.

## 4. `camera_apriltag.launch.py` 정적 분석

### 4.1 실행 구조

```text
realsense2_camera/rs_launch.py
  -> D435 color/depth 및 RealSense TF
camera_info_qos_bridge.py
  -> CameraInfo durability 변환
image_proc/rectify_node
  -> color image rectification
apriltag_ros/apriltag_node
  -> detections 및 tag TF
robot_state_publisher
  -> rescue_robot.urdf 발행
```

### 4.2 RealSense 설정

- include 대상: `realsense2_camera/launch/rs_launch.py`
- `camera_namespace`: `leader`
- `camera_name`: `camera`
- color: 항상 활성화
- depth: launch argument `enable_depth`, 기본 `true`
- infrared: `enable_infra`, 기본 `false`
- gyro/accel: `enable_imu`, 기본 `false`
- `publish_tf`: `true`
- `tf_publish_rate`: 30.0 Hz
- color/depth profile: 640×480×30

`tf_publish_rate=30.0`은 RealSense가 발행하는 카메라 TF 설정이며 AprilTag 검출 TF의
실제 갱신률을 보장하는 값으로 사용하면 안 된다.

### 4.3 영상과 CameraInfo

launch와 QoS bridge에서 직접 확인되는 이름은 다음과 같다.

| 용도 | 토픽 |
|---|---|
| D435 RGB 원본 | `/leader/camera/color/image_raw` |
| D435 RGB CameraInfo | `/leader/camera/color/camera_info` |
| QoS 변환 CameraInfo | `/leader/camera/color/camera_info_transient` |
| rectified RGB | `/leader/camera/color/image_rect` |

`image_proc` 노드 이름은 `/RectifyNode`로 구성되어 있고 별도 namespace는 없다.
입력 image는 원본 RGB, 입력 CameraInfo는 transient bridge 출력, 출력은
`image_rect`로 remap된다. 따라서 rectified image를 실제로 생성해 AprilTag에
전달하는 구조이다.

### 4.4 AprilTag 노드

- package/executable: `apriltag_ros/apriltag_node`
- namespace: `leader/apriltag`
- name: `apriltag`
- 정적으로 resolve되는 node 이름: `/leader/apriltag/apriltag`
- params file: `rescue_robot_apriltag/config/apriltag_leader.yaml`
- `image_rect` remap: `/leader/camera/color/image_rect`
- `camera_info` remap: `/leader/camera/color/camera_info`
- 설정된 tag family: `36h11`
- 설정된 tag child frame: `leader/tag36h11:0`

설치된 `apriltag_ros` 3.4.0의 상대 publisher 이름은 `detections`이므로 이 launch
namespace에서 예상되는 detection 토픽은 `/leader/apriltag/detections`이고 메시지
타입은 `apriltag_msgs/msg/AprilTagDetectionArray`이다. 다만 이번 조사 시 live graph가
없어 실제 publisher 연결은 아직 확인되지 않았다.

## 5. ROS graph 조사 결과와 미확정 항목

`/opt/ros/humble/setup.bash`와 존재하는 경우 저장소의 `install/setup.bash`를 source한
후 기존 프로세스를 종료하거나 시작하지 않고 다음을 조회했다.

```bash
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
```

이번 조사 세션에는 Leader 노드가 실행 중이지 않았고 기본 `/parameter_events`,
`/rosout` 외 프로젝트 토픽도 없었다. 다만 저장소의 기존 D435 실기 검증 문서
`docs/progress/week 1/03_AprilTag_인식/02_D435_RGB_AprilTag_실행.md`에는 image와
CameraInfo의 실제 `header.frame_id`가 `camera_color_optical_frame`이며, 확인된 TF가
`camera_color_optical_frame -> tag36h11:0`이라고 기록되어 있다. 저장소의 네 개
VSLAM rosbag에서도 `camera_color_frame -> camera_color_optical_frame` 정적 TF를
독립적으로 확인했다.

따라서 Leader 상태 노드의 확인된 기본 `source_frame`은
`camera_color_optical_frame`이다. 패키지 완성 단계의 live graph 재검증에서는 현재
`apriltag_leader.yaml`과 일치하는 child frame `leader/tag36h11:0`이 실제로 발행됐고,
과거 실기 기록의 비접두 `tag36h11:0` frame은 존재하지 않았다. 따라서 상태 노드의
현재 기본 pattern은 `leader/tag36h11:{id}`로 확정한다.

현재 연결된 D435에서는 image와 CameraInfo가 약 30 Hz로 발행됐고 CameraInfo의
`header.frame_id`도 `camera_color_optical_frame`으로 재확인됐다. frame 이름은 앞으로
카메라 driver/TF prefix 설정을 바꿀 때 다시 검증해야 한다.

사용자가 별도 터미널에서 기존 launch를 실행한 상태에서 아래처럼 읽기 전용으로
재확인한다.

```bash
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
ros2 node info /leader/apriltag/apriltag --no-daemon
ros2 param list /leader/apriltag/apriltag
timeout 5s ros2 topic echo /leader/camera/color/camera_info --once
timeout 5s ros2 topic echo /leader/apriltag/detections --once
timeout 5s ros2 topic echo /tf
```

CameraInfo/detections의 header와 `/tf`의 parent/child를 먼저 대조한 다음, 확인된 optical
frame을 사용하여 다음 명령을 실행한다.

```bash
timeout 5s ros2 run tf2_ros tf2_echo <확인된_camera_optical_frame> leader/tag36h11:0
```

패키지 완성 단계에서 보이는 ID 0 태그 TF를 10초간 직접 표본화했다. 251개의 distinct
timestamp가 수집됐으며 중앙 timestamp 간격은 0.033357초(약 29.98 Hz), 최대 간격은
0.166787초였다. 수신 시점 기준 TF age는 중앙 0.056085초, 최대 0.067281초였다.

이 실측을 바탕으로 상태 노드 초기 YAML에는 `tag_timeout=1.0`을 사용한다. 이는 관찰된
최대 간격의 약 6배, 최대 age의 약 15배인 보수적 시험값이며 Follower의 2.0초를
기계적으로 복사한 값이 아니다. 10초 단일 표본이므로 장시간 Jetson 부하 시험과 태그
가림 시험 전에는 최종값으로 간주하지 않는다. RealSense launch의
`tf_publish_rate=30.0`은 카메라 센서 TF 설정이므로 위 AprilTag TF 실측과 구분한다.

## 6. Follower 검증본 분석

### 6.1 순수 Python 로직

`approach_logic.py`는 ROS import가 없으며 다음 책임을 분리한다.

- `ApproachState`: 9개 상태 문자열 enum
- `ApproachThresholds`: 거리/좌우/각도/안정 시간 검증
- `TagObservation`: ID, translation, quaternion, TF timestamp
- `RelativeMeasurement`: 필터 좌표와 네 파생 metric
- `is_valid_translation()`: NaN, inf, `z<=0` 거부
- `normalize_quaternion()`: finite/영 노름 검사 및 정규화
- `compute_measurement()`: `z`, `x`, 3차원 거리, `atan2(x,z)` 계산
- `MedianTranslationFilter`: 서로 다른 TF timestamp의 x/y/z 중앙값 필터
- `select_observation()`: allowed ID의 priority/nearest 선택
- `ApproachStateMachine`: 판정 우선순위와 연속 stable timer 관리

상태 판정 순서는 `TAG_LOST`, `TURN_LEFT`, `TURN_RIGHT`, `APPROACH`, `TOO_CLOSE`,
`FINE_ALIGN_LEFT`, `FINE_ALIGN_RIGHT`, `STABILIZING`, `ALIGNED`이다. 임계 경계는
허용 범위에 포함하며 조건 이탈, 유실, 선택 ID 변경 시 stable timer를 초기화한다.

### 6.2 ROS 노드

`apriltag_approach_node.py`는 startup-only 파라미터를 검증하고 timer마다 최신 TF를
조회한다.

- `TransformException`을 정상적인 미검출 후보로 처리
- `now - TF stamp > tag_timeout`인 stale TF 제외
- translation과 quaternion 검증
- 지정 ID 또는 allowed ID 후보 조회
- priority 또는 straight distance 기반 nearest 선택
- 선택 ID 변경 시 median filter와 state machine reset
- 같은 TF timestamp를 필터에 중복 추가하지 않음
- 유효 태그에서 pose/metric/detected/ID/state 발행
- 유실 시 `detected=false`, `tag_id=-1`, `TAG_LOST`만 발행
- 유실 중 과거 pose/distance/angle을 새 값처럼 재발행하지 않음

### 6.3 파라미터와 launch

Follower는 `ament_python` 패키지이며 console script
`apriltag_approach_node`를 제공한다. 전체 launch는 USB camera, `image_proc`,
`apriltag_ros`, 상태 노드를 함께 시작하고 `approach_only.launch.py`는 기존 tag
pipeline에 상태 노드만 붙인다.

Follower의 현재 주요 값은 다음과 같다.

| 파라미터 | 값 |
|---|---|
| `source_frame` | `follower/follower_camera_optical_frame` |
| `tag_frame_pattern` | `follower/tag36h11:{id}` |
| `target_tag_id` | 0 |
| `allowed_tag_ids` | `[0, 1, 2]` |
| `selection_mode` | `priority` |
| `target_distance` | 0.15 m, 시험값 |
| `tag_timeout` | 2.0 s |
| `publish_rate` | 20.0 Hz |
| `filter_window` | 5 |

### 6.4 테스트와 점검 스크립트

`test_approach_logic.py`는 카메라 없이 다음을 검증한다.

- 8개 유효 상태와 `TAG_LOST`
- stable time과 reset
- 거리/좌우/각도 계산
- 임계 경계 포함
- priority, nearest, nearest 동률, 비허용 ID 제외
- median outlier 제거와 같은 timestamp 중복 방지
- 잘못된 threshold, translation, quaternion, filter window, selection mode 거부

`check_approach_topics.sh`는 실행 중 프로세스를 시작하거나 종료하지 않고 8개 출력 토픽
타입, 주요 파라미터와 짧은 메시지 샘플을 검사한다. Leader용 이식본에서는 과거 Follower
workspace 경로를 복사하지 않고 `/home/maze/damgc_robot/install/setup.bash`를 사용해야
한다.

## 7. Follower와 Leader 차이

| 구분 | Follower 검증본 | Leader 현재 환경 |
|---|---|---|
| 카메라 | `usb_cam` USB camera | Intel RealSense D435 |
| 카메라 namespace | `/follower/camera` | `/leader/camera` |
| RGB 원본 | `/follower/camera/image_raw` | `/leader/camera/color/image_raw` |
| CameraInfo | `/follower/camera/camera_info` | `/leader/camera/color/camera_info` |
| rectified RGB | `/follower/camera/image_rect` | `/leader/camera/color/image_rect` |
| CameraInfo 보조 처리 | 없음 | transient-local QoS bridge 사용 |
| camera optical frame | 설정 및 검증값 존재 | 기존 실기 기록상 `camera_color_optical_frame` |
| tag frame prefix | `follower/tag36h11:` | live graph 확인값 `leader/tag36h11:` |
| 상태 node namespace | `/follower` | `/leader` 예정 |
| build type | `ament_python` | 기존 `ament_cmake` 유지 예정 |
| 상태판단 구현 | 존재 및 카메라 검증 | 아직 없음 |

## 8. 그대로 재사용할 로직과 Leader 변경점

다음 알고리즘은 이름/동작을 유지해 Leader 패키지 안에 독립 구현할 수 있다.

- 9개 상태와 판정 우선순위
- threshold validation과 부동소수점 경계 처리
- translation/quaternion 유효성 검사
- metric 계산식
- distinct timestamp median translation filter
- single-ID 및 multi-ID 선택
- priority/nearest와 deterministic tie 처리
- ID 변경/유실/조건 이탈 reset
- stale TF와 `TransformException` 처리
- 유실 시 세 개 상태성 토픽만 발행하는 정책

Leader에서 반드시 바꿀 부분은 다음과 같다.

- Python import/package 이름을 `rescue_robot_apriltag`로 변경
- 실측한 D435 optical frame을 `source_frame`으로 사용
- tag pattern을 실제 TF와 일치하도록 Leader prefix로 설정
- node와 출력 토픽을 `/leader` namespace에 배치
- 기존 `ament_cmake`에 Python package와 pytest 설치/등록 방식 추가
- USB camera launch를 복사하지 않고 기존 `camera_apriltag.launch.py`에 조건부 통합
- D435의 실제 TF 갱신 특성으로 timeout 검증
- Leader의 실제 목표 거리와 허용오차를 물리 시험으로 재조정

## 9. 구현 예정 파일

향후 구현 단계에서 생성할 후보 파일은 다음과 같다.

```text
src/leader/rescue_robot_apriltag/
├── rescue_robot_apriltag/
│   ├── __init__.py
│   ├── approach_logic.py
│   └── apriltag_approach_node.py
├── config/
│   └── approach.yaml
├── launch/
│   └── approach_only.launch.py
├── test/
│   └── test_approach_logic.py
├── scripts/
│   └── check_approach_topics.sh
└── docs/
    ├── APRILTAG_APPROACH_NODE_GUIDE.md
    ├── MANUAL_STATE_TEST.md
    └── IMPLEMENTATION_RECORD.md
```

이번 조사 단계에서는 위 구현 파일을 생성하지 않는다.

## 10. 수정 예정 파일

- `rescue_robot_apriltag/CMakeLists.txt`
  - 기존 `ament_cmake`를 유지하며 Python package/executable, config, launch, docs 및
    pytest 설치/등록 추가
- `rescue_robot_apriltag/package.xml`
  - `ament_cmake_python`, `geometry_msgs`, `std_msgs`, `tf2_ros`, launch/test 의존성 추가
- `rescue_robot_apriltag/config/apriltag_leader.yaml`
  - 승인된 초기 구성에 따라 ID 0, 1, 2와 각 0.050 m frame/size 등록 예정
- `rescue_robot_bringup/launch/camera_apriltag.launch.py`
  - 상태 노드와 config launch argument를 추가하되 기존 카메라/tag pipeline 유지
  - `enable_approach` 기본값은 `false`로 두어 명시적으로 활성화할 때만 상태 노드 실행

## 11. 예상 Leader ROS 출력 인터페이스

상태 노드는 상대 토픽 이름을 사용하고 launch에서 `/leader` namespace를 적용한다.

| 전체 토픽 | 메시지 타입 | 의미 |
|---|---|---|
| `/leader/supply/detected` | `std_msgs/msg/Bool` | 유효하고 fresh한 선택 TF 존재 여부 |
| `/leader/supply/tag_id` | `std_msgs/msg/Int32` | 선택 ID, 유실 시 -1 |
| `/leader/supply/relative_pose` | `geometry_msgs/msg/PoseStamped` | 필터 위치와 최신 유효 quaternion |
| `/leader/supply/distance` | `std_msgs/msg/Float64` | 전방 거리 `z` [m] |
| `/leader/supply/lateral_error` | `std_msgs/msg/Float64` | 좌우 오차 `x` [m] |
| `/leader/supply/straight_distance` | `std_msgs/msg/Float64` | `sqrt(x²+y²+z²)` [m] |
| `/leader/supply/angle` | `std_msgs/msg/Float64` | `atan2(x,z)` [rad] |
| `/leader/alignment/state` | `std_msgs/msg/String` | 9개 상태 문자열 |

유실 중에는 detected, tag ID, state만 갱신하고 pose와 수치 metric은 재발행하지 않는다.

## 12. 테스트 계획

### 12.1 단위 테스트

Follower 테스트와 동등한 ROS 비의존 검증을 Leader import 경로로 작성한다.

- 모든 상태와 판정 우선순위
- 거리/좌우/직선거리/각도 계산
- threshold 경계값
- stable timer의 시작, 완료, 조건 이탈, ID 변경 및 유실 reset
- priority/nearest/동률/비허용 ID
- median filter의 outlier, timestamp 중복, reset
- NaN, inf, `z<=0`, invalid quaternion
- 잘못된 파라미터와 빈/중복 allowed ID 검증

### 12.2 build와 정적 통합 검사

```bash
cd /home/maze/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select rescue_robot_apriltag rescue_robot_bringup --symlink-install
source /home/maze/damgc_robot/install/setup.bash
colcon test --packages-select rescue_robot_apriltag
colcon test-result --verbose
ros2 launch rescue_robot_apriltag approach_only.launch.py --show-args
ros2 launch rescue_robot_bringup camera_apriltag.launch.py --show-args
```

### 12.3 실제 ROS graph 통합 시험

1. 기존 camera/AprilTag launch를 사용자가 실행한다.
2. 먼저 approach-only launch로 상태 노드만 추가한다.
3. node/토픽 타입/파라미터와 선택 TF를 확인한다.
4. 태그가 없을 때 `false`, `-1`, `TAG_LOST`만 갱신되는지 확인한다.
5. 태그를 가린 후 timeout 뒤 stale pose/metric이 다시 발행되지 않는지 확인한다.
6. 연속 가시 상태의 TF timestamp와 false `TAG_LOST` 여부를 기록한다.
7. 전체 launch의 조건부 통합을 활성화해 중복 노드 없이 같은 결과를 재검증한다.

### 12.4 D435 수동 이동 및 RViz 시험

- 태그 좌우 이동: `TURN_LEFT`, `TURN_RIGHT`, lateral/angle 부호 확인
- 태그 전후 이동: `APPROACH`, `TOO_CLOSE`, distance 확인
- 거리와 각도를 맞춘 좌우 이동: `FINE_ALIGN_LEFT/RIGHT`
- 모든 오차 정상 유지: `STABILIZING` 후 `ALIGNED`
- 태그 가림: timeout 뒤 `TAG_LOST`
- 여러 ID: fixed ID, priority, nearest와 ID 변경 reset 확인
- RViz Fixed Frame을 실측 optical frame으로 설정하고 TF와
  `/leader/supply/relative_pose`가 일치하는지 확인

## 13. Launch 통합 계획과 완료 조건

1. 실시간 graph에서 optical frame과 tag TF 이름을 먼저 확정한다.
2. 상태 로직과 ROS wrapper를 Leader 패키지에 독립 구현한다.
3. approach-only launch로 기존 검증 파이프라인을 변경하지 않고 통합 시험한다.
4. `camera_apriltag.launch.py`에 `approach_config`와 `enable_approach`를 추가한다.
5. 기본 `enable_approach=false`로 현재 검증된 launch 동작을 보존한다.
6. `enable_approach:=true`에서 `/leader/apriltag_approach`와 8개 출력 토픽을 확인한다.

구현 단계의 완료 조건은 다음과 같다.

- camera optical frame을 live 메시지와 TF로 확인해 문서에 기록
- 기존 D435/rectify/AprilTag 동작 회귀 없음
- Leader 단위 테스트와 colcon build/test 성공
- false `TAG_LOST` 없이 D435 TF를 추적하는 timeout 확인
- 9개 상태, 유실, 여러 ID와 reset을 실제 태그로 확인
- RViz에서 TF와 상대 Pose의 좌표 및 부호 확인
- `cmd_vel`, 모터, STM32, 그리퍼, Nav2 또는 base-link 제어를 추가하지 않음
- Follower 패키지에 변경 없음

## 14. 위험 요소와 rollback

- optical frame을 추측하면 TF lookup이 전부 실패할 수 있으므로 live 확인 전 설정을
  확정하지 않는다.
- TF buffer가 마지막 transform을 보관하므로 lookup 성공만으로 detected를 판정하면 안
  되며 timestamp timeout이 필수다.
- 1.0초 timeout은 10초 실측에 기반한 초기값이며 장시간 부하·가림 시험 후 재조정해야 한다.
- nearest에는 hysteresis가 없어 거리가 비슷한 태그 사이에서 선택이 흔들릴 수 있다.
- 유실 중 metric을 재발행하지 않으므로 consumer는 detected와 메시지 수신 시각을 함께
  확인해야 한다.

통합 문제 발생 시 상태 노드를 종료하거나 `enable_approach:=false`로 실행하면 기존
camera/AprilTag pipeline만 유지할 수 있다. 기존 launch 노드와 remap을 제거하지 않고
Leader 상태판단 변경을 독립 커밋으로 유지해 해당 변경만 되돌릴 수 있게 한다.
