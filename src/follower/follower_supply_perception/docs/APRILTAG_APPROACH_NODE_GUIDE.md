# AprilTag 상대 위치 기반 접근 상태 노드 가이드

- 문서 경로: `docs/APRILTAG_APPROACH_NODE_GUIDE.md`
- 패키지: `follower_supply_perception`
- 작업공간: `/home/kde/ros2_ws`
- 작성일: 2026-07-20

## 1. 목적과 완료 범위

USB 카메라로 검출한 AprilTag의 카메라 기준 상대 TF를 읽어 거리·좌우·각도 오차를
계산하고 접근 및 정렬 상태를 발행한다. 현재 완료된 범위는 다음과 같다.

- USB 카메라, image_proc, apriltag_ros와 상태 노드를 하나의 launch로 실행
- 단일 또는 여러 허용 ID 선택
- translation 중앙값 필터, TF timeout과 유실 처리
- 8개 상태/측정 ROS 토픽 발행
- 안정 조건 연속 유지 후 `ALIGNED` 판정
- 카메라 없는 단위 테스트 37개와 실제 ROS 그래프 통합 검사

이 버전은 인식과 상태 판정만 수행한다. 로봇 이동이나 물체 조작은 수행하지 않는다.

## 2. 개발 환경

- 장치: Jetson Orin Nano
- 운영체제: Ubuntu 22.04
- ROS: ROS 2 Humble
- Python: 3.10.12
- 패키지 형식: `ament_python`
- 카메라: `/dev/video0`, 640x480, 30 fps, `mjpeg2rgb`
- 보정 파일: 패키지 내부 `config/follower_usb_camera.yaml`
- 태그 family/크기: `36h11`, `0.050 m`

## 3. 데이터 흐름과 프레임

```text
/dev/video0
  -> usb_cam
     -> /follower/camera/image_raw
     -> /follower/camera/camera_info
  -> image_proc rectify_node
     -> /follower/camera/image_rect
  -> apriltag_ros
     -> /follower/apriltag/detections
     -> /tf: follower_camera_optical_frame -> tag36h11:<id>
  -> apriltag_approach_node
     -> /follower/supply/*
     -> /follower/alignment/state
```

`follower_camera_optical_frame`은 현재 모든 상대 위치의 기준 프레임이다. optical frame은
일반적으로 `x`가 영상 오른쪽, `y`가 아래, `z`가 카메라 전방을 뜻한다.
`tag36h11:<id>`는 36h11 family의 해당 정수 ID 태그 프레임이다. 예를 들어 ID 0은
`tag36h11:0`이다.

현재 결과는 카메라 기준이므로 로봇 차체 기준 제어에 바로 사용하면 안 된다. 추후
`base_link -> follower_camera_optical_frame`의 정확한 정적/URDF TF를 확보한 뒤
`base_link` 기준으로 변환해야 한다.

## 4. 여러 태그 ID 추적

- `target_tag_id >= 0`: 해당 ID만 추적한다.
- `target_tag_id == -1`: `allowed_tag_ids`에 포함된 보이는 태그를 후보로 사용한다.
- `selection_mode=priority`: `allowed_tag_ids` 배열에서 먼저 나오는 유효 ID를 선택한다.
- `selection_mode=nearest`: 카메라에서 3차원 직선거리가 가장 작은 유효 ID를 선택한다.
- nearest 동률은 `allowed_tag_ids` 순서로 결정한다.
- 태그 ID가 바뀌면 translation 필터와 안정화 타이머를 초기화한다.

## 5. 패키지 구조와 파일 역할

아래 경로는 모두 패키지 루트 기준 상대 경로다.

| 상대 경로 | 역할 |
|---|---|
| `follower_supply_perception/approach_logic.py` | ROS 비의존 계산, 필터, 선택과 상태 머신 |
| `follower_supply_perception/apriltag_approach_node.py` | TF 조회와 ROS publisher/timer |
| `config/approach.yaml` | 상태 노드 시험용 파라미터 |
| `config/apriltag.yaml` | 검증된 apriltag_ros 파라미터 복사본 |
| `launch/follower_apriltag.launch.py` | 카메라부터 상태 노드까지 전체 실행 |
| `launch/approach_only.launch.py` | 기존 태그 파이프라인에 상태 노드만 추가 |
| `test/test_approach_logic.py` | 카메라 없는 pytest 단위 테스트 37개 |
| `scripts/check_approach_topics.sh` | 실행 중 노드·토픽 타입·주요 값 점검 |
| `docs/MANUAL_STATE_TEST.md` | 사용자의 물리 태그 이동 시험표 |
| `docs/TEST_RESULTS_APRILTAG_APPROACH.md` | 단위 테스트 결과와 미검증 범위 |
| `docs/IMPLEMENTATION_RECORD.md` | 개발 변경·명령·실패 수정 기록 |
| `README.md` | 빠른 빌드와 실행 진입점 |

## 6. 클래스와 핵심 함수

### ROS 비의존 로직

- `ApproachState`: 9개 상태 문자열 enum.
- `ApproachThresholds`: 거리·좌우·각도·안정 시간 임계값과 유효성 검사.
- `TagObservation`: 한 태그의 원본 translation, quaternion, timestamp.
- `RelativeMeasurement`: 필터된 좌표와 계산된 네 metric.
- `is_valid_translation()`: NaN, 무한대와 `z<=0` 거부.
- `normalize_quaternion()`: quaternion 유한성/노름 검사와 정규화.
- `compute_measurement()`: 거리·좌우·직선거리·각도 계산.
- `MedianTranslationFilter`: 서로 다른 TF timestamp의 최근 x/y/z 중앙값 계산.
- `select_observation()`: priority 또는 nearest 후보 선택.
- `ApproachStateMachine.update()`: 상태 우선순위와 연속 안정 시간을 판정.

### ROS 노드

`AprilTagApproachNode`는 시작 시 파라미터를 검증하고 `tf2_ros.Buffer`와
`TransformListener`를 만든다. `publish_rate` timer마다 후보 태그의 최신 TF를 조회하고,
선택·필터·상태 머신을 호출한 뒤 상대 이름 publisher로 결과를 낸다. launch에서
`/follower` namespace를 적용하므로 실제 노드 이름은 `/follower/apriltag_approach`다.
`TransformException`은 정상 유실로 취급하며 상태가 바뀔 때만 info 로그를 남긴다.

## 7. TF 유실, stale 데이터와 필터

- `lookup_transform(source_frame, tag_frame, latest)`로 버퍼의 최신 TF를 조회한다.
- `now - transform.header.stamp > tag_timeout`이면 버퍼에 남은 오래된 TF를 거부한다.
- NaN, 무한대, `z<=0`, 영 quaternion 후보도 거부한다.
- 유효 후보가 없으면 `detected=false`, `tag_id=-1`, `TAG_LOST`만 발행하고 오래된 pose와
  수치 metric은 재발행하지 않는다.
- `filter_window` 크기의 x/y/z 성분별 중앙값으로 순간 outlier를 줄인다.
- 같은 timestamp를 timer마다 중복 샘플로 넣지 않는다.
- quaternion에는 성분 중앙값을 적용하지 않고 최신 유효 값을 정규화한다.
- 유실 또는 선택 ID 변경 시 필터와 안정화 이력을 초기화한다.

## 8. 계산식

카메라 optical frame의 필터된 좌표를 `(x, y, z)`라고 할 때:

- 전방 거리: `distance = z` [m]
- 좌우 오차: `lateral_error = x` [m]
- 직선거리: `straight_distance = sqrt(x² + y² + z²)` [m]
- 수평각: `angle = atan2(x, z)` [rad]

`angle<0`은 왼쪽, `angle>0`은 오른쪽 보정 방향이다.

## 9. 상태 판정 순서

최신 구현의 우선순위는 다음과 같다. 경계값은 허용 범위에 포함하고 부동소수점 경계
동등성을 고려한다.

1. `TAG_LOST`: 유효하고 timeout 이내인 선택 TF가 없음.
2. `TURN_LEFT`: `angle < -angle_tolerance`.
3. `TURN_RIGHT`: `angle > angle_tolerance`.
4. `APPROACH`: 각도는 정상이고 `z > target_distance + distance_tolerance`.
5. `TOO_CLOSE`: 각도는 정상이고 `z < target_distance - distance_tolerance`.
6. `FINE_ALIGN_LEFT`: 거리·각도 정상, `x < -lateral_tolerance`.
7. `FINE_ALIGN_RIGHT`: 거리·각도 정상, `x > lateral_tolerance`.
8. `STABILIZING`: 모든 오차 정상, 연속 유지 시간이 `stable_time` 미만.
9. `ALIGNED`: 모든 오차가 `stable_time` 이상 연속 정상.

조건 이탈, 유실 또는 선택 ID 변경은 안정화 시작 시각을 초기화한다.

## 10. ROS 인터페이스

### 출력 토픽

| 전체 토픽 | 메시지 타입 | 의미 |
|---|---|---|
| `/follower/supply/detected` | `std_msgs/msg/Bool` | 유효한 최신 TF 존재 여부 |
| `/follower/supply/tag_id` | `std_msgs/msg/Int32` | 선택 ID, 유실 시 -1 |
| `/follower/supply/relative_pose` | `geometry_msgs/msg/PoseStamped` | 필터 위치와 최신 quaternion |
| `/follower/supply/distance` | `std_msgs/msg/Float64` | 전방 z [m] |
| `/follower/supply/lateral_error` | `std_msgs/msg/Float64` | 좌우 x [m] |
| `/follower/supply/straight_distance` | `std_msgs/msg/Float64` | 3차원 거리 [m] |
| `/follower/supply/angle` | `std_msgs/msg/Float64` | 수평각 [rad] |
| `/follower/alignment/state` | `std_msgs/msg/String` | 9개 상태 문자열 |

### 파라미터

| 파라미터 | 기본 시험값 | 설명 |
|---|---:|---|
| `source_frame` | `follower_camera_optical_frame` | TF 기준 프레임 |
| `tag_frame_pattern` | `tag36h11:{id}` | ID를 치환할 태그 프레임 패턴 |
| `target_tag_id` | `0` | -1이면 다중 ID 모드 |
| `allowed_tag_ids` | `[0, 1, 2]` | 다중 모드 후보/우선순위 |
| `selection_mode` | `priority` | `priority` 또는 `nearest` |
| `target_distance` | `0.15` | 목표 z [m], 현장 확정값 아님 |
| `distance_tolerance` | `0.02` | 거리 허용 오차 [m] |
| `lateral_tolerance` | `0.02` | 좌우 허용 오차 [m] |
| `angle_tolerance_deg` | `5.0` | 수평각 허용 오차 [deg] |
| `tag_timeout` | `0.3` | TF stale 판정 시간 [s] |
| `stable_time` | `0.8` | ALIGNED 전 연속 유지 시간 [s] |
| `publish_rate` | `20.0` | timer 주기 [Hz] |
| `filter_window` | `5` | 중앙값 표본 수 |

`target_distance=0.15 m`를 실제 그리퍼 접촉 또는 파지 거리로 확정했다고 해석하면 안
된다. 카메라 장착 위치, base_link와 그리퍼 TCP를 기준으로 반드시 재측정해야 한다.

## 11. 빌드와 테스트

```bash
source /opt/ros/humble/setup.bash
cd /home/kde/ros2_ws
colcon build --packages-select follower_supply_perception
source /home/kde/ros2_ws/install/setup.bash
colcon test --packages-select follower_supply_perception
colcon test-result --verbose
```

최종 재검증 결과는 37 tests, 0 errors, 0 failures, 0 skipped다. 상세 범위는
`docs/TEST_RESULTS_APRILTAG_APPROACH.md`를 따른다.

## 12. 실행 방법

### 상태 노드 단독 launch

기존 카메라·Rectify·AprilTag 파이프라인이 실행 중일 때 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /home/kde/ros2_ws/install/setup.bash
ros2 launch follower_supply_perception approach_only.launch.py
```

### 전체 파이프라인 launch

중복 수동 노드를 먼저 각 터미널에서 `Ctrl-C`로 종료한 뒤 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/kde/ros2_ws/install/setup.bash
ros2 launch follower_supply_perception follower_apriltag.launch.py
```

RViz는 launch에 포함되지 않는다. 영상만 보려면 별도 터미널에서 실행한다.

```bash
ros2 run rqt_image_view rqt_image_view /follower/camera/image_rect
```

## 13. target_tag_id 변경

현재 노드는 파라미터를 시작할 때 한 번 읽어 내부 필드에 보관한다. 따라서 실행 중
`ros2 param set`이 성공하더라도 추적 대상은 즉시 바뀌지 않는다. 지원되는 변경 방법은
상태 노드만 종료하고 launch 또는 `ros2 run`을 override해 재실행하는 것이다.

```bash
ros2 run follower_supply_perception apriltag_approach_node \
  --ros-args \
  -r __ns:=/follower \
  --params-file /home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/approach.yaml \
  -p target_tag_id:=1
```

다중 ID는 `target_tag_id:=-1`과 `allowed_tag_ids`, `selection_mode`를 함께 override한다.
동적 파라미터 callback 지원은 후속 기능이다.

## 14. 실시간 통합 확인 결과와 명령

자동 통합 검사에서 전체 launch의 네 노드, 필수 영상·검출·TF·8개 출력 토픽,
AprilTag `size=0.05`, 보정 영상 약 11.8 Hz를 확인했다. 상태 노드 단독 검사에서는
`detected=false`, `tag_id=-1`, `TAG_LOST`를 실제 수신했다. 태그 유실 상태라 distance와
angle 표본은 설계대로 없었다.

```bash
ros2 node list
ros2 topic list -t
ros2 param get /follower/apriltag/apriltag size
ros2 run tf2_ros tf2_echo follower_camera_optical_frame tag36h11:0
/home/kde/ros2_ws/src/follower_supply_perception/scripts/check_approach_topics.sh
```

물리적인 TURN, APPROACH, TOO_CLOSE와 ALIGNED 상태는 자동 성공으로 기록하지 않았다.

## 15. 사용자 물리 시험과 RViz 확인

`docs/MANUAL_STATE_TEST.md`의 표에 따라 태그 가림, 좌우, 전후, 목표 위치 유지와 여러
ID 전환을 사용자가 직접 수행한다.

최종 Pose 확인도 사용자가 수행한다.

1. 전체 launch를 실행하고 태그를 보이게 한다.
2. 별도 터미널에서 `rviz2`를 실행한다.
3. Global Options의 Fixed Frame을 `follower_camera_optical_frame`으로 설정한다.
4. `TF` display를 추가해 `tag36h11:<id>` 프레임을 확인한다.
5. `Pose` display를 추가하고 Topic을 `/follower/supply/relative_pose`로 설정한다.
6. Pose 위치와 TF 위치가 일치하고 optical-frame 축 부호가 실제 이동과 맞는지 확인한다.
7. 태그 유실 시 Pose가 stale 값으로 계속 갱신되지 않는지 확인한다.

이번 자동 작업에서는 RViz 화면을 최종 판정하지 않았다.

## 16. 오류 해결

| 증상 | 확인 및 조치 |
|---|---|
| TF 없음 | 태그가 보이는지, `/tf`, `source_frame`, `tag36h11:<id>` 철자와 `tag_timeout` 확인 |
| size 오류 | `ros2 param get /follower/apriltag/apriltag size`; 기대값 `0.05` |
| image_rect 없음 | usb_cam의 image_raw/camera_info와 rectify_node remapping, 보정 파일 확인 |
| 출력 토픽 없음 | `/follower/apriltag_approach` 존재, `/follower` namespace와 설치 setup source 확인 |
| YAML 미적용 | 전체 노드 키가 `/follower/apriltag/apriltag` 또는 `/follower/apriltag_approach`인지 확인 |
| 중복 노드 | 기존 수동 터미널 노드를 `Ctrl-C`로 종료; 자동 kill이나 중복 카메라 접근 금지 |
| TAG_LOST 지속 | 최신 TF timestamp, target ID, allowed IDs, 카메라 시야와 조명 확인 |
| 카메라 control 경고 | 지원하지 않는 선택 V4L2 control일 수 있음; 영상/CameraInfo 갱신 여부로 판단 |

## 17. 안전 제한과 다음 단계

현재 코드와 launch에는 `cmd_vel`, 그리퍼 또는 STM32 명령이 없다. 상태 문자열은 제어
명령이 아니며, `ALIGNED`만으로 즉시 구동기를 작동시키면 안 된다.

다음 개발 단계 후보:

1. 카메라 extrinsic을 확정하고 `base_link` 기준 상대 pose 제공.
2. 여러 태그 상태를 동시에 표시하는 `visualization_msgs/MarkerArray` 추가.
3. 별도 안전 제어 계층에서 속도 제한·정지·watchdog를 갖춘 `cmd_vel` 연동.
4. 그리퍼 TCP 거리, 물체 존재, 정렬 유지와 interlock을 포함한 그리퍼 조건.

## 18. 종료와 재실행 순서

1. rqt_image_view와 RViz를 각각 `Ctrl-C`로 종료한다.
2. 전체 또는 approach-only launch 터미널에서 한 번 `Ctrl-C`한다.
3. `ros2 node list --no-daemon`으로 launch child가 사라졌는지 확인한다.
4. `/dev/video0`를 사용하는 다른 수동 카메라 노드가 없는지 확인한다.
5. 새 터미널에서 `/opt/ros/humble/setup.bash`, 작업공간 `install/setup.bash` 순서로 source한다.
6. 필요한 launch 명령을 다시 실행한다.

강제 `pkill`이나 기존 카메라 프로세스 자동 종료를 정상 운영 절차로 사용하지 않는다.
