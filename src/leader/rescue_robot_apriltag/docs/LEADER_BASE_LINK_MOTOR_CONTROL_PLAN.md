# Leader AprilTag base_link 변환 및 모터 구동 개발 계획

## 1. 문서 목적

이 문서는 Leader의 AprilTag 상대 위치를 D435 camera optical frame에서
`base_link` 기준으로 변환하고, 안전한 저속 접근 제어와 STM32 모터 구동까지 연결하기
위한 단계별 개발 계획이다.

현재 구현된 범위는 다음과 같다.

```text
D435
→ RGB image / CameraInfo
→ apriltag_ros
→ camera_color_optical_frame → leader/tag36h11:<id> TF
→ TF 유효성 검사·태그 선택·median filter
→ camera optical frame 기준 상태판단 및 8개 토픽
```

이 문서에서 설명하는 `base_link` 변환, 접근 제어기, Leader 속도 guard와 자동 모터
구동은 아직 구현하거나 실기 검증하지 않았다.

## 2. 목표 구조

최종적으로 다음 구조를 목표로 한다.

```text
AprilTag TF
→ camera optical 기준 검증·필터
→ base_link 기준 변환
→ base 기준 접근 오차·상태
→ 접근 제어기
→ Leader 속도 안전 guard
→ /leader/cmd_vel
→ stm32_bridge
→ UART
→ STM32 watchdog/E-stop
→ 좌·우 모터
```

인지, 제어, 안전 경계, 하드웨어 bridge를 서로 분리한다. AprilTag 상태 문자열을 바로
모터 명령으로 변환하거나 상태판단 패키지에서 UART를 직접 다루지 않는다.

## 3. 현재 저장소에서 확인한 전제

### 3.1 URDF의 base_link와 D435

현재 Leader URDF는 다음 파일이다.

```text
src/leader/rescue_robot_description/urdf/rescue_robot.urdf
```

좌표축 주석은 다음과 같다.

```text
base_link +X: 로봇 전방
base_link +Y: 로봇 왼쪽
base_link +Z: 위쪽
```

현재 카메라 장착 joint는 다음 모델값을 사용한다.

```xml
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0 0 0.070" rpy="0 0 0"/>
</joint>
```

이는 카메라가 `base_link`보다 70 mm 위에 있고, 전후·좌우 offset과 회전 오차가 없다는
가정이다. 실제 D435 장착 위치를 측정해 확정한 extrinsic이라고 간주하면 안 된다.

### 3.2 실제 camera와 tag frame

기존 Leader 실기 검사에서 다음 frame을 확인했다.

```text
source frame: camera_color_optical_frame
tag frame:    leader/tag36h11:<id>
```

camera optical frame은 다음 축을 사용한다.

```text
+X: 영상 오른쪽
+Y: 영상 아래쪽
+Z: 카메라 전방
```

카메라가 차체와 이상적으로 평행하다면 직관적인 축 관계는 다음과 같다.

```text
base x ≈ camera z
base y ≈ -camera x
base z ≈ -camera y
```

이 축 교환을 코드에 하드코딩하지 않는다. 실제 translation과 roll/pitch/yaw를 포함한
전체 변환은 TF2에 맡긴다.

### 3.3 현재 STM32 bridge

공통 패키지 `src/stm32_bridge`의 노드는 `/leader` namespace로 실행할 때 다음
interface를 사용한다.

- 구독: `/leader/cmd_vel`
- 발행: `/leader/odom/raw`
- 발행: `/leader/imu/data_raw`
- 발행: `/leader/system_state`
- 진단: `/leader/stm32_rx/frame_count`
- 진단: `/leader/stm32_rx/crc_errors`
- 진단: `/leader/stm32_rx/sequence_drops`

현재 주요 파라미터는 다음과 같다.

| 항목 | 현재값 |
|---|---:|
| wheel radius | 0.0635 m |
| wheel separation | 0.23 m |
| encoder | 5131 ticks/revolution |
| UART | 460800 baud |
| UART velocity 전송 | 50 Hz |
| command timeout | 200 ms |

bridge는 다음 식으로 차체 속도를 좌우 바퀴 선속도로 변환한다.

```text
left  = v - w × wheel_separation / 2
right = v + w × wheel_separation / 2
```

현재 Leader에는 Follower와 같은 독립적인 velocity guard가 없다. STM32 bridge가
`/leader/cmd_vel`을 직접 구독하므로 자동 제어를 연결하기 전에 별도 안전 경계가
필요하다.

## 4. base_link 변환 원리

전체 좌표 변환은 다음 행렬 곱으로 표현할 수 있다.

```text
T_base_tag
  = T_base_camera_link
  × T_camera_link_camera_color_optical
  × T_camera_color_optical_tag
```

예상 TF 체인은 다음과 같다.

```text
base_link
→ camera_link
→ camera_color_frame
→ camera_color_optical_frame
→ leader/tag36h11:<id>
```

`base_link` 변환 자체에는 `odom`이나 EKF가 필요하지 않다. `base_link → camera`가
고정 변환이기 때문이다. Wheel odometry는 실제 이동 피드백, 정지 확인과 주행 성능
평가에 사용한다.

## 5. TF 실측 및 검증 절차

### 5.1 D435 extrinsic 실측

다음 값을 실제 로봇에서 측정한다.

- `base_link` 원점에서 D435 기준점까지 x/y/z
- D435의 roll/pitch/yaw
- D435에서 optical center까지의 driver 제공 static TF
- 카메라가 차체 중심선에 대해 좌우로 치우쳤는지 여부
- 카메라가 위나 아래로 기울어졌는지 여부

초기에는 캘리퍼와 수평계를 사용할 수 있다. 이후 측정 위치가 알려진 AprilTag fixture를
여러 위치에 놓고 변환 결과의 systematic error를 확인한다.

### 5.2 URDF 반영

실측 승인을 받은 뒤에만 다음 값을 수정한다.

```xml
<origin xyz="MEASURED_X MEASURED_Y MEASURED_Z"
        rpy="MEASURED_ROLL MEASURED_PITCH MEASURED_YAW"/>
```

승인 전에는 현재 값을 최종값으로 덮어쓰지 않는다. 변경 전후 수치와 측정 방법을
문서에 기록한다.

### 5.3 TF graph 검사

카메라 launch가 실행된 상태에서 다음을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash

ros2 run tf2_tools view_frames

ros2 run tf2_ros tf2_echo \
  base_link camera_color_optical_frame

ros2 run tf2_ros tf2_echo \
  base_link leader/tag36h11:0
```

완료 기준은 다음과 같다.

- TF chain 전체 연결
- frame마다 parent 하나
- 동일 변환을 발행하는 중복 broadcaster 없음
- 태그 가시 상태에서 transform timestamp 갱신
- 태그 가림 후 과거 TF를 fresh 값으로 오인하지 않음

## 6. 권장 base pose 변환 구현

현재 `/leader/supply/relative_pose`는 다음 장점이 있다.

- `camera_color_optical_frame` 기준
- 선택 태그의 원본 TF timestamp 유지
- translation median filter 적용
- quaternion 유효성 검사와 정규화 적용
- 태그 유실 중 과거 pose를 새 값으로 재발행하지 않음

따라서 이 `PoseStamped`를 원본 timestamp에서 `base_link`로 변환하는 방식을 우선
검토한다.

개념 코드는 다음과 같다.

```python
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException


def transform_to_base(tf_buffer, camera_pose: PoseStamped) -> PoseStamped | None:
    try:
        base_from_camera = tf_buffer.lookup_transform(
            "base_link",
            camera_pose.header.frame_id,
            Time.from_msg(camera_pose.header.stamp),
            timeout=Duration(seconds=0.1),
        )
    except TransformException:
        return None

    base_pose = PoseStamped()
    base_pose.header.stamp = camera_pose.header.stamp
    base_pose.header.frame_id = "base_link"
    base_pose.pose = do_transform_pose(
        camera_pose.pose,
        base_from_camera,
    )
    return base_pose
```

구현 시 다음 조건을 추가한다.

- 입력 timestamp를 현재 시각으로 덮어쓰지 않음
- 입력 pose 및 변환 결과의 freshness 검사
- 변환 결과 NaN/inf 거부
- quaternion 재검증
- `TransformException` 시 invalid/lost 처리
- 유실 중 stale base pose와 metric 미발행
- 선택 ID와 pose의 동일 cycle 대응 보장

여러 개의 독립 토픽을 제어 입력으로 직접 조합하면 ID, pose, state가 서로 다른 cycle의
값일 수 있다. 모터 연결 단계에서는 stamp, detected, tag ID, pose와 state를 하나의
일관된 표본으로 전달하는 내부 구조 또는 custom message도 검토한다.

## 7. 제안 base_link 출력

기존 camera 기준 8개 토픽은 유지한다. 다음 토픽을 별도로 추가하는 방식을 검토한다.

| 제안 토픽 | 타입 | 의미 |
|---|---|---|
| `/leader/supply/base_relative_pose` | `geometry_msgs/msg/PoseStamped` | base 기준 태그 pose |
| `/leader/supply/base_forward_distance` | `std_msgs/msg/Float64` | base x [m] |
| `/leader/supply/base_lateral_error` | `std_msgs/msg/Float64` | base y [m], 왼쪽 양수 |
| `/leader/supply/base_planar_distance` | `std_msgs/msg/Float64` | `sqrt(x²+y²)` [m] |
| `/leader/supply/base_bearing` | `std_msgs/msg/Float64` | `atan2(y,x)` [rad] |

계산식은 다음과 같다.

```text
forward_distance = x_base
lateral_error    = y_base
planar_distance  = sqrt(x_base² + y_base²)
bearing          = atan2(y_base, x_base)
```

base frame에서는 다음 부호를 사용한다.

```text
y_base > 0 또는 bearing > 0: 태그가 왼쪽
y_base < 0 또는 bearing < 0: 태그가 오른쪽
```

기존 camera 상태 로직은 camera x가 음수일 때 왼쪽으로 판단한다. 따라서 base bearing을
기존 상태 머신에 그대로 전달하면 좌우가 반대로 판정될 수 있다. base 기준 판정은
다음처럼 정의해야 한다.

```text
bearing > +tolerance → TURN_LEFT
bearing < -tolerance → TURN_RIGHT
```

기존 camera-frame 상태는 회귀 검사용으로 유지하고, 모터 제어용 base-frame 오차와
상태를 별도로 검증한다.

## 8. 목표 pose와 그리퍼 기준 frame

현재 `target_distance=0.15 m`는 camera optical z 기준 시험값이다. 실제 그리퍼/TCP
목표 거리가 아니다.

현재 URDF에서 그리퍼 tip 중심은 대략 `base_link` 전방 0.305 m에 있다. 따라서 단순히
base x 목표를 0.15 m로 사용하면 모델상 태그가 그리퍼 끝보다 안쪽에 위치할 수 있다.
현재 값을 모터 정지 목표로 사용하지 않는다.

권장 방식은 URDF에 다음 고정 frame을 추가하는 것이다.

```text
base_link
└── gripper_approach_link
```

`gripper_approach_link`는 태그가 최종 정지해야 하는 위치와 방향을 나타낸다. 실제 물품
형상, 태그 부착면, 그리퍼 끝과 필요한 안전 여유를 측정해 위치를 정한다.

최종 오차는 다음 TF로 계산할 수 있다.

```text
T_gripper_approach_tag
```

개발 순서는 camera → base 변환을 먼저 완료하고, 이후 base → gripper approach 목표를
추가하는 방식으로 나눈다.

## 9. 패키지 분리 제안

인지 패키지에 모터 제어를 넣지 않는다.

```text
rescue_robot_apriltag
  └── TF 검증, 선택, filter, camera/base pose 출력

leader_approach_control              # 신규 패키지 제안
  ├── leader_approach_control/
  │   ├── base_approach_logic.py
  │   ├── approach_controller_node.py
  │   └── velocity_guard_node.py
  ├── config/approach_control.yaml
  ├── launch/approach_control.launch.py
  ├── test/
  └── docs/
```

구성 요소의 책임은 다음과 같다.

- `base_approach_logic.py`: base 오차와 상태, 제어 후보 계산
- `approach_controller_node.py`: fresh pose와 enable/fault gate를 확인하고 raw Twist 발행
- `velocity_guard_node.py`: finite 검사, 속도 제한, slew limit, command watchdog
- `stm32_bridge`: 최종 안전 Twist를 UART 바퀴 명령으로 변환

## 10. 권장 속도 토픽 구조

```text
/leader/approach/cmd_vel_raw
        ↓
leader_velocity_guard
        ↓
/leader/cmd_vel
        ├── stm32_bridge
        └── leader_cooperation
```

`/leader/cmd_vel`은 안전 guard 하나만 발행하는 최종 명령으로 정의하는 것이 좋다.
현재 `leader_cooperation`도 `/leader/cmd_vel`을 구독하므로 안전하게 제한된 명령이
협동 노드로 전달된다.

teleop, Nav2와 AprilTag controller가 동시에 같은 토픽을 발행하면 ROS 2가 자동으로
우선순위를 결정하지 않는다. 장기적으로 다음 command mux가 필요하다.

```text
teleop ─────┐
Nav2 ───────┼→ velocity mux → safety guard → /leader/cmd_vel
AprilTag ───┘
```

AprilTag 단독 시험 중에는 `leader_cooperation`을 비활성 상태로 두어 Follower로
명령이 전달되지 않게 한다.

## 11. 접근 제어 정책

Leader는 차동구동이므로 `linear.y`로 옆으로 이동할 수 없다. `FINE_ALIGN_LEFT` 또는
`FINE_ALIGN_RIGHT`를 lateral velocity로 직접 변환하지 않는다.

초기 상태별 정책은 다음과 같이 제한한다.

| 상태 | 초기 제어 정책 |
|---|---|
| `TAG_LOST` | `v=0`, `w=0` |
| `TURN_LEFT` | `v=0`, 작은 `w>0` |
| `TURN_RIGHT` | `v=0`, 작은 `w<0` |
| `APPROACH` | 작은 `v>0`, bearing 기반 `w` 보정 |
| `TOO_CLOSE` | 초기 버전은 정지, 후진 금지 |
| `FINE_ALIGN_LEFT/RIGHT` | 별도 저속 곡선 또는 rotate-drive-rotate 정책 |
| `STABILIZING` | `v=0`, `w=0` |
| `ALIGNED` | `v=0`, `w=0` |

고정 속도만 사용하는 대신 연속 오차 기반 후보 명령을 검토한다.

```text
v = clamp(k_v × forward_error)
w = clamp(k_bearing × bearing + k_lateral × lateral_error)
```

초기 실기 시험에서는 다음을 적용한다.

- 후진 비활성
- 회전만 먼저 시험
- 매우 낮은 최대 선속도와 각속도
- 가속도와 감속도 제한
- 목표에 가까워질수록 속도 감소
- 태그 유실 또는 stale 시 즉시 0속도
- enable 기본값 false

gain과 최대 속도는 실기 결과 없이 최종값으로 확정하지 않는다.

## 12. 독립적인 안전 정지 조건

다음 중 하나라도 만족하면 접근 제어기와 velocity guard는 0속도를 출력해야 한다.

- controller enable이 false
- `detected=false`
- pose timestamp 만료
- `TAG_LOST`
- TF 변환 실패
- NaN 또는 infinity
- 선택 ID 불일치
- 허용하지 않은 reverse 명령
- UART/STM32 heartbeat timeout
- STM32 E-stop 또는 fault
- command publisher timeout
- 장애물 또는 최소 안전거리 위반
- node shutdown

정지 경로는 여러 계층에 둔다.

```text
접근 제어기 stop
→ velocity guard watchdog
→ stm32_bridge command timeout
→ STM32 200 ms watchdog
→ 독립 하드웨어 E-stop
```

상위 노드의 정상 동작만 믿지 않고 각 하위 계층이 독립적으로 정지할 수 있어야 한다.

## 13. STM32 bridge 보완 항목

자동 모터 구동 전에 다음을 검토하고 시험한다.

1. Leader velocity guard 추가
2. bridge 입력의 NaN/inf와 허용 축 검사
3. 의미 있는 선속도·각속도 또는 wheel speed 제한
4. `cmd_timeout_ms`와 UART payload의 `watchdog_ms` 일치
5. 종료 시 0속도/controlled-stop frame 여러 회 송신
6. `control_flags`의 enable, controlled stop, emergency stop 반영
7. `/leader/system_state`를 구조화된 상태 interface로 교체 또는 별도 발행
8. estop, fault bits와 `last_cmd_age_ms`를 motor enable gate에 연결
9. UART 재연결 후 과거 명령이 다시 활성화되지 않는지 확인
10. STM32 clock과 ROS time 동기화 계획 수립

현재 URDF wheel radius는 0.060 m이고 bridge 값은 0.0635 m이므로 실제 바퀴 반지름을
측정해 하나의 기준값으로 통일해야 한다. 기존 기록의 1 m 직진에서 wheel odometry가
약 1.05 m였으므로 scale calibration도 필요하다.

## 14. 단계별 구현 및 시험 계획

| 단계 | 작업 | 완료 기준 |
|---|---|---|
| 1 | D435 x/y/z/rpy 실측 | 측정 방법·수치·오차 기록 |
| 2 | URDF extrinsic 반영 | 승인된 값만 반영, 기존 diff 최소화 |
| 3 | TF graph 검증 | base→optical→tag 연속 lookup, broadcaster 충돌 없음 |
| 4 | base pose 변환 구현 | timestamp 보존, stale/exception/NaN 처리 |
| 5 | ROS 비의존 단위 테스트 | 축·회전·translation·boundary·loss 테스트 통과 |
| 6 | synthetic TF 통합 시험 | 알려진 변환에서 기대 base pose와 일치 |
| 7 | 실제 태그 수동 시험 | base x/y/bearing 부호와 RViz 일치 |
| 8 | 그리퍼/TCP 목표 측정 | `gripper_approach_link` 위치 승인 |
| 9 | 제어 로직 작성 | motor 없이 raw Twist 계산 테스트 통과 |
| 10 | Leader velocity guard | timeout, clamp, finite, slew, shutdown stop 통과 |
| 11 | STM32 bridge hardening | watchdog·fault·E-stop·stop burst 검증 |
| 12 | 바퀴 공중 시험 | 좌우 방향, encoder 부호, E-stop 확인 |
| 13 | 수동 저속 지상 시험 | 직진·회전 반복, wheel geometry 보정 |
| 14 | AprilTag 회전 시험 | `v=0`, 좌우 bearing에 올바른 회전 |
| 15 | 저속 접근 시험 | 작은 구간별 전진, overshoot 없이 정지 |
| 16 | fault injection | tag 가림, node kill, UART 단절 모두 정지 |
| 17 | 통합 launch | 기본 motor disable, 명시적 enable 시에만 동작 |
| 18 | 장시간 시험 | 반복 접근 중 runaway·fault·과열 없음 |

## 15. 단계별 상세 완료 기준

### 단계 A: base_link 변환 완료

- `base_link → camera_color_optical_frame` 실측값 문서화
- `base_link → leader/tag36h11:0` live lookup
- base pose header stamp가 원본 tag stamp와 일치
- 태그 가림 후 stale base pose 미발행
- 태그 좌우 이동 시 base y와 bearing 부호 일치
- RViz에서 TF와 base pose 일치

이 단계에서는 `cmd_vel`을 발행하지 않는다.

### 단계 B: motor 없는 제어기 완료

- rosbag 또는 synthetic pose로 모든 상태 입력 재생
- 출력은 `/leader/approach/cmd_vel_raw`까지만 연결
- `TAG_LOST`, stale, invalid, aligned에서 항상 0
- 선속도·각속도 포화
- gain과 tolerance boundary 단위 테스트
- 후진 비활성 확인

### 단계 C: 안전 경계 완료

- raw publisher 종료 후 설정 timeout 안에 0속도
- NaN/inf 명령 거부
- 허용하지 않은 Twist 축 무시 또는 거부
- 과속 clamp
- 가속도·감속도 제한
- enable false 기본값
- shutdown 시 stop 메시지 반복 발행
- `/leader/cmd_vel` publisher가 안전 guard 하나뿐임을 확인

### 단계 D: STM32 및 공중 시험 완료

- 물리 E-stop이 UART와 독립적으로 모터 출력 차단
- STM32 watchdog 200 ms 이내 정지
- UART 분리와 bridge kill 시 정지
- 양수 `linear.x`의 실제 전진 방향 확인
- 양수 `angular.z`의 실제 왼쪽 회전 확인
- 좌우 wheel speed와 encoder 부호 일치
- CRC와 sequence drop 감시

### 단계 E: 지상 자동 접근 완료

- 작업자와 별도 E-stop 담당자 배치
- 넓고 장애물이 없는 구역
- 회전 시험 후 전진 시험
- 짧은 이동 구간부터 단계적으로 확대
- 태그 가림, 조명 변화, 잘못된 ID에서 정지
- 최종 위치 overshoot와 반복 오차 기록
- 배터리, motor temperature와 fault 기록

## 16. 물리 시험 순서

모터 시험은 다음 순서를 바꾸지 않는다.

1. 모터 전원 차단 상태에서 ROS command graph 확인
2. 물리 E-stop 누른 상태에서 UART frame 확인
3. 로봇을 들어 바퀴가 지면에 닿지 않게 함
4. 짧은 0속도 command부터 전송
5. 좌·우 바퀴에 매우 작은 속도를 각각 시험
6. 전진·후진 encoder 부호 확인
7. 제자리 좌·우 회전 부호 확인
8. publisher 중단과 UART 분리 watchdog 시험
9. E-stop 시험
10. 모든 정지 시험 통과 후 지상 수동 주행
11. 지상 회전 자동 제어
12. 마지막에만 저속 전진 자동 접근

## 17. 필수 fault injection 시험

| 시험 | 기대 결과 |
|---|---|
| 태그 가림 | timeout 후 0속도와 `TAG_LOST` |
| 태그 TF timestamp 정지 | stale 판정 후 0속도 |
| base TF 제거 | TransformException 후 0속도 |
| controller process kill | guard timeout 후 0속도 |
| guard process kill | STM32 watchdog 후 0속도 |
| stm32_bridge kill | STM32 watchdog 후 0속도 |
| UART TX 분리 | STM32 watchdog 후 0속도 |
| UART RX 분리 | feedback timeout에 의해 motor inhibit |
| NaN/inf 명령 | guard 거부, 0속도 |
| 최대값 초과 명령 | clamp 또는 거부 |
| hardware E-stop | 통신 상태와 무관하게 즉시 차단 |
| 재연결 | explicit enable 전까지 0속도 |

## 18. 제안 파라미터 구조

아래 이름은 설계 후보이며 값은 실기 시험 전 확정하지 않는다.

```yaml
/leader/approach_controller:
  ros__parameters:
    base_frame: base_link
    target_frame_pattern: "leader/tag36h11:{id}"
    pose_timeout: 0.0              # 실측 후 결정

    target_forward: 0.0            # gripper/TCP 실측 후 결정
    target_lateral: 0.0
    forward_tolerance: 0.0
    lateral_tolerance: 0.0
    bearing_tolerance_deg: 0.0
    stable_time: 0.0

    linear_gain: 0.0
    angular_gain: 0.0
    lateral_gain: 0.0
    max_linear_speed: 0.0
    max_angular_speed: 0.0
    max_linear_acceleration: 0.0
    max_angular_acceleration: 0.0

    allow_reverse: false
    enabled_on_startup: false
```

0 값은 미정 placeholder다. 이 파일을 그대로 실행 설정으로 사용하지 않는다.

## 19. launch 통합 원칙

최종 통합 시 기존 camera/AprilTag launch의 검증된 구성을 불필요하게 변경하지 않는다.

다음과 같은 명시적 인자를 검토한다.

```text
enable_approach=true
enable_base_transform=false
enable_approach_control=false
enable_motor_output=false
```

기본 운영에서는 motor output을 false로 둔다. 다음 조건을 모두 충족한 현장 시험에서만
명시적으로 true로 설정한다.

- base TF 실측 승인
- controller unit/integration test 통과
- velocity guard 통과
- STM32 watchdog과 E-stop 통과
- 바퀴 공중 시험 통과
- 작업 구역 안전 확보

## 20. rollback

문제 발생 시 다음 순서로 기능을 분리할 수 있어야 한다.

1. `enable_motor_output=false`
2. `enable_approach_control=false`
3. 상태 노드만 실행
4. 필요하면 `enable_approach=false`로 기존 camera/AprilTag pipeline만 실행

인지, base 변환, 제어, guard, STM32 변경을 서로 다른 commit으로 유지하면 단계별
rollback이 가능하다.

## 21. 다음 작업 우선순위

가장 먼저 수행할 작업은 다음 두 가지다.

1. D435 장착 extrinsic 실측과
   `base_link → camera_color_optical_frame` TF 검증
2. 현재 filtered `PoseStamped`를 `base_link`로 변환하고 태그 수동 이동으로 축과 부호
   검증

위 두 단계가 완료되기 전에는 `target_distance=0.15 m`를 모터 정지 목표로 사용하거나
AprilTag 상태를 `/leader/cmd_vel`에 연결하지 않는다.

그 다음 작업은 `leader_approach_control` 패키지와 Leader velocity guard의 상세
인터페이스를 확정하는 것이다.
