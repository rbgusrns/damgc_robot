# AprilTag 상대 위치 기반 상태 판단 노드 작업 명세

## 1. 목적과 범위

USB 카메라와 `apriltag_ros`가 제공하는 TF를 읽어 카메라 기준 AprilTag 상대 위치를
계산하고, 접근·정렬 상태를 ROS 토픽으로 발행하는 Python ROS 2 노드를 개발한다.
카메라 드라이버, 영상 보정, AprilTag 검출 및 TF 발행은 이미 동작하는 외부 구성으로
취급한다.

이번 기능은 인식 및 상태 판단까지만 담당한다. 다음 기능은 절대 포함하지 않는다.

- `cmd_vel` 발행 또는 이동 제어
- STM32 명령이나 통신
- 그리퍼 명령
- 기존 카메라·AprilTag 프로세스의 시작, 종료 또는 설정 변경

## 2. 확인된 실행 환경과 현재 구조

- 장치: Jetson Orin Nano
- 운영체제: Ubuntu 22.04
- ROS: ROS 2 Humble
- Python: 3.10.12
- 작업공간: `/home/kde/ros2_ws`
- 기준 프레임: `follower_camera_optical_frame`
- 태그 family: `36h11`
- 실제 태그 한 변: `0.050 m`
- 기본 태그 TF 이름: `tag36h11:<id>`
- 현재 확인된 태그: `tag36h11:0`

조사 시점에 확인된 ROS 인터페이스는 다음과 같다.

| 용도 | 이름 | 타입 |
|---|---|---|
| 원본 영상 | `/follower/camera/image_raw` | `sensor_msgs/msg/Image` |
| 카메라 정보 | `/follower/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 보정 영상 | `/follower/camera/image_rect` | `sensor_msgs/msg/Image` |
| AprilTag 검출 | `/follower/apriltag/detections` | `apriltag_msgs/msg/AprilTagDetectionArray` |
| 동적 TF | `/tf` | `tf2_msgs/msg/TFMessage` |
| 정적 TF | `/tf_static` | `tf2_msgs/msg/TFMessage` |

`follower_camera_optical_frame`에서 `tag36h11:0`으로의 TF가 실제로 확인되었다.
예시 translation은 약 `x=-0.012 m`, `y=0.013 m`, `z=0.101 m`였다. optical frame
관례상 `x`는 좌우, `z`는 전방 거리로 사용한다.

## 3. 필수 기능

1. `tf2_ros.Buffer`와 `TransformListener`로 최신 태그 TF를 조회한다.
2. 단일 목표 ID 또는 허용된 여러 ID 중 하나를 선택한다.
3. 선택된 TF의 `x`, `y`, `z`, quaternion을 `PoseStamped`로 발행한다.
4. 다음 값을 계산한다.
   - 전방 거리: `z`
   - 좌우 오차: `x`
   - 직선거리: `sqrt(x² + y² + z²)`
   - 수평각: `atan2(x, z)` rad
5. TF timestamp와 `tag_timeout`을 이용해 오래된 TF와 검출 유실을 판정한다.
6. 태그별 최근 translation 표본에 중앙값 필터를 적용한다.
7. 모든 정렬 조건이 `stable_time` 동안 연속 유지된 뒤에만 `ALIGNED`를 출력한다.
8. 상태 판정 코드는 ROS 어댑터와 분리해 단위 테스트 가능하게 만든다.

## 4. 출력 토픽

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/follower/supply/detected` | `std_msgs/msg/Bool` | timeout 이내의 유효한 선택 TF 존재 여부 |
| `/follower/supply/tag_id` | `std_msgs/msg/Int32` | 선택 ID, 유실 시 `-1` |
| `/follower/supply/relative_pose` | `geometry_msgs/msg/PoseStamped` | 필터된 위치와 최신 정규화 quaternion |
| `/follower/supply/distance` | `std_msgs/msg/Float64` | 전방 거리 `z`, m |
| `/follower/supply/lateral_error` | `std_msgs/msg/Float64` | 좌우 오차 `x`, m |
| `/follower/supply/straight_distance` | `std_msgs/msg/Float64` | 3차원 직선거리, m |
| `/follower/supply/angle` | `std_msgs/msg/Float64` | `atan2(x,z)`, rad |
| `/follower/alignment/state` | `std_msgs/msg/String` | 상태 문자열 |

태그 유실 시 `detected=false`, `tag_id=-1`, `state=TAG_LOST`만 주기적으로 발행한다.
오래된 pose와 수치 값은 재발행하지 않는다.

## 5. 상태와 판정 우선순위

상태 문자열은 다음 아홉 개로 제한한다.

1. `TAG_LOST`: 유효하고 timeout 이내인 TF가 없음
2. `TOO_CLOSE`: `z < target_distance - distance_tolerance`
3. `TURN_LEFT`: 수평각이 음의 각도 허용 범위를 벗어남
4. `TURN_RIGHT`: 수평각이 양의 각도 허용 범위를 벗어남
5. `APPROACH`: `z > target_distance + distance_tolerance`
6. `FINE_ALIGN_LEFT`: 거리와 각도는 허용 범위이나 `x < -lateral_tolerance`
7. `FINE_ALIGN_RIGHT`: 거리와 각도는 허용 범위이나 `x > lateral_tolerance`
8. `STABILIZING`: 모든 오차가 허용 범위지만 유지 시간이 부족함
9. `ALIGNED`: 모든 오차가 `stable_time` 이상 연속으로 허용 범위에 있음

실제 판정 우선순위는 `TAG_LOST → TURN → APPROACH → TOO_CLOSE → FINE_ALIGN →
STABILIZING/ALIGNED`이다. 경계값은 허용 범위에 포함한다. 유실, ID 변경 또는 조건
이탈 시 안정화 타이머를 초기화한다. `angle > 0`과 `x > 0`은 오른쪽 보정 요구로
정의한다.

## 6. 여러 ID 선택 규칙

- `target_tag_id >= 0`: 해당 ID만 사용하는 단일 태그 모드
- `target_tag_id == -1`: `allowed_tag_ids`에 포함된 ID를 다중 후보로 사용
- `priority`: 유효 후보 중 `allowed_tag_ids`에 먼저 나오는 ID 선택
- `nearest`: `sqrt(x²+y²+z²)`가 가장 작은 유효 후보 선택
- nearest 동률은 `allowed_tag_ids` 순서로 결정
- 조회 실패, stale timestamp 또는 비정상 수치가 있는 후보는 제외
- ID별 필터 이력을 분리하고, 선택 ID 변경 시 안정화 시간을 초기화

`tag_frame_pattern`은 기본적으로 `tag36h11:{id}` 형식의 이름 있는 자리표시자를
사용한다.

## 7. 설정 파라미터

| 파라미터 | 초기 기본값 | 제약/의미 |
|---|---:|---|
| `source_frame` | `follower_camera_optical_frame` | TF 기준 프레임 |
| `tag_frame_pattern` | `tag36h11:{id}` | 태그 프레임 패턴 |
| `target_tag_id` | `0` | `-1`이면 다중 ID 모드 |
| `allowed_tag_ids` | `[0, 1, 2]` | 우선순위도 배열 순서를 사용 |
| `selection_mode` | `priority` | `priority` 또는 `nearest` |
| `target_distance` | `0.15` | m, 양수 |
| `distance_tolerance` | `0.02` | m, 음수 불가 |
| `lateral_tolerance` | `0.02` | m, 음수 불가 |
| `angle_tolerance_deg` | `5.0` | degree, 음수 불가 |
| `tag_timeout` | `0.3` | s, 음수 불가 |
| `stable_time` | `0.8` | s, 음수 불가 |
| `publish_rate` | `20.0` | Hz, 양수 |
| `filter_window` | `5` | 1 이상의 표본 수 |

기본값은 초기 기능 시험을 위한 값일 뿐이며 실제 그리퍼 동작 거리나 현장 확정값이
아니다. 실제 카메라 장착과 태그 배치에서 반드시 재측정하고 튜닝한다. 잘못된 값은
자동으로 보정하지 않고 시작 단계에서 명확한 오류로 거부한다.

## 8. 구현 코드 구조

- `follower_supply_perception/approach_logic.py`
  - 상태 enum, 설정 및 측정 dataclass
  - metric 계산, 중앙값 필터, 태그 선택, 안정화 상태 판정
- `follower_supply_perception/apriltag_approach_node.py`
  - 파라미터, TF 조회, stale 처리, publisher와 timer
- `config/approach.yaml`
  - 위 기본값을 담은 ROS 파라미터 파일
- `launch/approach_only.launch.py`
  - 기존 카메라·AprilTag 파이프라인은 별도로 유지하고 상태 노드만 실행
- `launch/follower_apriltag.launch.py`
  - USB 카메라, image rectification, AprilTag와 상태 노드를 통합 실행
- `test/test_approach_logic.py`
  - 상태, 수학, 필터와 태그 선택을 검증하는 ROS 비의존 단위 테스트

중앙값 필터는 서로 다른 TF timestamp의 `x/y/z`에만 적용한다. quaternion은 최신 TF
값의 유한성과 노름을 확인한 뒤 정규화한다. 같은 TF를 timer마다 중복 표본으로 넣지
않는다.

## 9. 테스트 및 통합 검증

단위 테스트에는 다음을 포함한다.

- 모든 상태의 대표값과 정확한 허용 경계
- 좌우 부호와 각도 계산
- 상태 우선순위가 겹치는 입력
- 안정화 성공, 순간 이탈, 유실 및 ID 변경
- priority/nearest 선택, 동률, stale 후보 제외
- 중앙값의 outlier 제거, ID별 이력 분리, timestamp 중복 방지
- 비정상 파라미터 및 quaternion 거부

실시간 통합 검증은 기존 프로세스를 종료하지 않고 별도 상태 노드만 실행해 수행한다.

1. 노드와 8개 출력 토픽의 이름·타입 확인
2. 출력 주기가 `publish_rate`와 일치하는지 확인
3. 태그를 가린 뒤 `tag_timeout` 후 `TAG_LOST` 확인
4. 태그를 전후·좌우로 움직여 모든 상태와 부호 확인
5. 허용 범위 유지 시 `STABILIZING → ALIGNED` 확인
6. 여러 태그로 priority와 nearest 모드 확인
7. 사용자가 RViz에서 기준 프레임, TF, pose를 최종 대조

물리 태그 이동 시험과 최종 RViz 확인은 사용자가 수행한다.

## 10. 완료 기준과 주의점

다음 단계의 전체 구현은 아래 조건을 모두 만족해야 완료로 본다.

- `colcon build --packages-select follower_supply_perception` 성공
- `colcon test --packages-select follower_supply_perception` 실패 없음
- 요청된 8개 토픽과 9개 상태가 명세대로 동작
- stale TF가 검출 상태로 잘못 유지되지 않음
- 다중 ID 선택과 태그 전환 시 안정화 초기화 검증
- 기존 카메라·AprilTag 프로세스와 독립적으로 launch 가능
- 코드·launch·문서 어디에도 이동, STM32, 그리퍼 명령이 없음
- README와 상세 문서에서 빌드, 실행, 튜닝, 검증 절차 재현 가능

TF buffer는 마지막 변환을 보존하므로 조회 성공만으로 검출을 판단하면 안 된다. ROS
clock과 TF timestamp 차이를 반드시 검사해야 한다. nearest 모드는 거리가 비슷한 태그
사이에서 전환될 수 있으므로, 최초 버전은 결정적 동률 규칙을 적용하고 필요 시 후속
버전에서 hysteresis를 추가한다.

추가 시스템 패키지 설치가 필요하면 `sudo`를 실행하지 않는다. 필요한 Ubuntu/ROS
패키지 이름과 필요 이유를 먼저 사용자에게 보고한다.
