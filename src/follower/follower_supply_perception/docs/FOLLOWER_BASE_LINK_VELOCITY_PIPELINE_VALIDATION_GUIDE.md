# Follower base-link velocity software pipeline 재현·검증 가이드

## 1. 목적과 완료 범위

이 문서는 Follower Jetson에서 `damgc_robot` repository를 clone 또는 pull한 개발자가
AprilTag 기반 base-link velocity software pipeline을 선택 빌드하고, STM32와 motor를
연결하지 않은 상태에서 재현·검증하는 절차다. 명령과 이름은 2026-08-29 현재 repository의
실제 코드, launch 및 YAML을 기준으로 한다.

이번 단계에서 완료한 경계는 다음과 같다.

```text
USB camera
  → image_proc rectification
  → apriltag_ros
  → camera optical-frame pose/metric/state
  → exact-source-stamp TF2 transform
  → base_link pose/metric/state
  → Follower approach controller
  → deterministic command selector
  → Follower safety guard
  → final safe ROS 2 velocity topic
```

다음 항목은 완료 범위가 아니다.

- STM32 bridge 및 UART
- Motor 연결과 실제 주행
- Wheel-air 또는 ground driving test
- Gripper 제어와 실제 grasp

즉, 완료의 의미는 **USB camera에서 `/follower/safe_cmd_vel`까지의 ROS 2 software
pipeline 완료**이며, hardware motion 완료가 아니다.

## 2. 전체 architecture와 안전 경계

```text
/dev/video0 USB Camera (YUYV 640x480 @ 30 FPS)
                   ↓
          /follower/camera/usb_cam
                   ↓ image_raw + camera_info
        /follower/camera/rectify_node
                   ↓ image_rect
      /follower/apriltag/apriltag
                   ↓ TF: follower_camera_optical_frame ← tag36h11:0
      /follower/apriltag_approach
            ├─→ 기존 camera-frame pose/metric/state
            │      └─→ /follower/alignment/state
            │
            └─→ TF2 exact source stamp to base_link
                   ├─→ /follower/supply/base_relative_pose
                   ├─→ base_forward_distance
                   ├─→ base_lateral_error
                   ├─→ base_bearing
                   └─→ /follower/base_alignment/state
                                      ↓
                    /follower/approach_controller
                                      ↓
                    /follower/approach/cmd_vel_raw
                                      │
                                      ├──────────────┐
                                      ↓              │
                          /follower/command_selector │
                                      ↑              │
                                      │              │
      Leader cooperation node ─→ /follower/cmd_vel ─┘
                                      ↓
                         /follower/selected_cmd_vel
                                      ↓
                           /follower/velocity_guard
                                      ↓
                           /follower/safe_cmd_vel
                                      X
                              STM32 / UART / Motor
                           (이번 단계에서는 미연결)
```

Perception은 velocity를 발행하지 않는다. Controller는 raw `Twist`만 발행한다. Selector는
두 command source 중 하나만 명시적으로 소유하게 한다. Guard만 최종
`/follower/safe_cmd_vel`을 발행한다.

## 3. 구현 파일

| 경로 | 역할 |
|---|---|
| `src/follower/follower_supply_perception/config/approach.yaml` | camera/base state, TF timeout, target 및 tolerance |
| `src/follower/follower_supply_perception/config/apriltag.yaml` | tag36h11 detector, ID/frame/size |
| `src/follower/follower_supply_perception/config/follower_usb_camera.yaml` | 640×480 USB camera calibration |
| `src/follower/follower_supply_perception/follower_supply_perception/apriltag_approach_node.py` | camera output 유지, exact-stamp base TF, base output/state |
| `src/follower/follower_supply_perception/follower_supply_perception/approach_logic.py` | camera-frame filter/selection/state logic |
| `src/follower/follower_supply_perception/follower_supply_perception/base_pose.py` | pose/TF validation, stamp 보존, base metric 계산 |
| `src/follower/follower_supply_perception/follower_supply_perception/base_alignment_logic.py` | base state priority와 stable timer |
| `src/follower/follower_supply_perception/follower_supply_perception/camera_extrinsic.py` | measured body translation과 optical rotation 분리 정의 |
| `src/follower/follower_supply_perception/launch/follower_camera_tf.launch.py` | 두 static TF publisher 실행 |
| `src/follower/follower_supply_perception/launch/follower_apriltag.launch.py` | camera→rectify→tag→state 전체 perception launch |
| `src/follower/follower_approach_control/` | stamped pose/state 기반 raw controller와 테스트 |
| `src/follower/follower_command_selector/` | STOP/APPROACH/COOPERATION deterministic selector와 테스트 |
| `src/follower/follower_control/` | 기존 final guard를 강화하고 standalone/integrated launch 제공 |
| `src/follower/follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md` | 본 재현·검증 가이드 |

## 4. Camera extrinsic과 TF chain

### 4.1 측정 장착값

`camera_extrinsic.py`의 measured initial extrinsic은 다음과 같다.

| Parent → child | xyz `[m]` | fixed-axis RPY `[rad]` | 의미 |
|---|---|---|---|
| `base_link → follower/follower_camera_link` | `(0.042, 0.000, 0.120)` | `(0, 0, 0)` | camera body origin의 초기 실측 위치와 수평·정면 장착 가정 |
| `follower/follower_camera_link → follower/follower_camera_optical_frame` | `(0, 0, 0)` | `(-π/2, 0, -π/2)` | REP-103 body 축을 optical 축으로 변환 |

Base 축은 `+X=전방`, `+Y=왼쪽`, `+Z=위`다. `rpy=(0,0,0)`은
`base_link → camera body/link`에만 적용된다. Optical frame에 직접 적용되는 값이 아니다.
Body-to-optical 변환을 별도 TF로 유지하므로 실제 장착 translation과 optical convention을
섞지 않는다.

실행 중 기대 chain:

```text
base_link
  └─ follower/follower_camera_link
       └─ follower/follower_camera_optical_frame
            └─ follower/tag36h11:0  # apriltag_ros가 보일 때 동적 발행
```

확인 명령:

```bash
ros2 run tf2_ros tf2_echo base_link follower/follower_camera_link
ros2 run tf2_ros tf2_echo \
  follower/follower_camera_link follower/follower_camera_optical_frame
ros2 run tf2_ros tf2_echo \
  follower/follower_camera_optical_frame 'follower/tag36h11:0'
```

이 값은 정밀 hand-eye calibration 결과가 아니라 초기 실측 extrinsic이다. 최종 camera 및
gripper/TCP calibration은 TODO다.

### 4.2 Base pose의 timestamp 계약

`apriltag_approach_node`는 선택된 tag TF의 원본 stamp를 filtered camera
`PoseStamped`에 넣는다. Base 변환은 다음과 같이 수행한다.

1. input pose의 실제 `header.frame_id`를 사용한다.
2. source pose stamp가 `tag_timeout` 안인지 검사한다.
3. `lookup_transform(base_link, source_frame, source_stamp, timeout=0.05 s)`로 exact stamp를
   조회한다.
4. position, transform, input/output quaternion을 finite/zero-norm 검사한다.
5. output `header.frame_id=base_link`, output stamp=source pose stamp를 유지한다.

TF 실패나 invalid pose는 node를 종료시키지 않는다. 해당 base sample만 생략하고
`/follower/base_alignment/state=TAG_LOST`를 발행한다. Lost 동안 base pose와 metric을
현재 시각으로 다시 발행하지 않는다.

## 5. Camera state와 base state의 차이

기존 camera state `/follower/alignment/state`와 신규 base state
`/follower/base_alignment/state`는 같은 9개 문자열을 사용하지만 좌표와 목표값이 다르다.
따라서 두 문자열이 항상 같아야 하는 것은 아니다.

| 항목 | Camera optical frame | `base_link` frame |
|---|---|---|
| 축 | `+x=영상 오른쪽`, `+y=아래`, `+z=전방` | `+x=로봇 전방`, `+y=로봇 왼쪽`, `+z=위` |
| forward | camera `z` | base `x` |
| lateral | camera `x` | base `y` |
| bearing/angle | `atan2(camera_x, camera_z)` | `atan2(base_y, base_x)` |
| target | `target_distance=0.15 m` | `base_target_forward=0.25 m` |
| 태그가 로봇 왼쪽 | camera lateral/angle `<0` | base lateral/bearing `>0` |
| 태그가 로봇 오른쪽 | camera lateral/angle `>0` | base lateral/bearing `<0` |

Camera output은 기존 의미를 유지한다.

```text
distance          = filtered camera z
lateral_error     = filtered camera x
straight_distance = sqrt(x² + y² + z²)
angle             = atan2(x, z)
```

Base metric은 동일한 transformed pose 한 sample에서 계산한다.

```text
base_forward_distance = base_pose.x
base_lateral_error    = base_pose.y
base_bearing          = atan2(base_pose.y, base_pose.x)
```

## 6. ROS 2 interface

### 6.1 Nodes

| Node | Package | 역할 |
|---|---|---|
| `/follower/camera/usb_cam` | `usb_cam` | USB camera 및 CameraInfo |
| `/follower/camera/rectify_node` | `image_proc` | calibrated rectification |
| `/follower/apriltag/apriltag` | `apriltag_ros` | tag detection과 tag TF |
| `/follower/camera_mount_tf` | `tf2_ros` | base→camera body static TF |
| `/follower/camera_optical_tf` | `tf2_ros` | camera body→optical static TF |
| `/follower/apriltag_approach` | `follower_supply_perception` | camera/base pose, metric, state |
| `/follower/approach_controller` | `follower_approach_control` | raw approach command |
| `/follower/command_selector` | `follower_command_selector` | explicit source ownership |
| `/follower/velocity_guard` | `follower_control` | final software safety boundary |
| `/leader_cooperation` | `leader_cooperation` | 활성화 시 remote `/follower/cmd_vel` publisher |

### 6.2 Topics

| Topic | Type | Publisher | Subscriber/의미 |
|---|---|---|---|
| `/follower/camera/image_raw` | `sensor_msgs/msg/Image` | `/follower/camera/usb_cam` | rectify input; native `yuv422_yuy2` |
| `/follower/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `/follower/camera/usb_cam` | rectify와 AprilTag intrinsics |
| `/follower/camera/image_rect` | `sensor_msgs/msg/Image` | `/follower/camera/rectify_node` | `/follower/apriltag/apriltag` input |
| `/follower/apriltag/detections` | `apriltag_msgs/msg/AprilTagDetectionArray` | `/follower/apriltag/apriltag` | detection 관찰용 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `apriltag_ros` 등 | `/follower/apriltag_approach` TF buffer |
| `/follower/supply/detected` | `std_msgs/msg/Bool` | `/follower/apriltag_approach` | controller detection gate |
| `/follower/supply/tag_id` | `std_msgs/msg/Int32` | `/follower/apriltag_approach` | controller ID gate; lost는 `-1` |
| `/follower/supply/relative_pose` | `geometry_msgs/msg/PoseStamped` | `/follower/apriltag_approach` | filtered camera-frame pose |
| `/follower/supply/distance` | `std_msgs/msg/Float64` | `/follower/apriltag_approach` | filtered camera `z` `[m]` |
| `/follower/supply/lateral_error` | `std_msgs/msg/Float64` | `/follower/apriltag_approach` | filtered camera `x` `[m]` |
| `/follower/supply/straight_distance` | `std_msgs/msg/Float64` | `/follower/apriltag_approach` | camera Euclidean distance `[m]` |
| `/follower/supply/angle` | `std_msgs/msg/Float64` | `/follower/apriltag_approach` | camera `atan2(x,z)` `[rad]` |
| `/follower/alignment/state` | `std_msgs/msg/String` | `/follower/apriltag_approach` | 기존 camera-frame state |
| `/follower/supply/base_relative_pose` | `geometry_msgs/msg/PoseStamped` | `/follower/apriltag_approach` | controller authoritative stamped sample |
| `/follower/supply/base_forward_distance` | `std_msgs/msg/Float64` | `/follower/apriltag_approach` | base `x` 관찰값 `[m]` |
| `/follower/supply/base_lateral_error` | `std_msgs/msg/Float64` | `/follower/apriltag_approach` | base `y`, 왼쪽 양수 `[m]` |
| `/follower/supply/base_bearing` | `std_msgs/msg/Float64` | `/follower/apriltag_approach` | base `atan2(y,x)` `[rad]` |
| `/follower/base_alignment/state` | `std_msgs/msg/String` | `/follower/apriltag_approach` | controller high-level state |
| `/follower/approach/cmd_vel_raw` | `geometry_msgs/msg/Twist` | `/follower/approach_controller` | selector APPROACH input |
| `/follower/cmd_vel` | `geometry_msgs/msg/Twist` | `/leader_cooperation` 활성 시 | selector COOPERATION input; standalone guard input도 가능 |
| `/follower/selected_cmd_vel` | `geometry_msgs/msg/Twist` | `/follower/command_selector` | integrated guard input |
| `/follower/safe_cmd_vel` | `geometry_msgs/msg/Twist` | `/follower/velocity_guard` | 이번 단계의 final safe software velocity |
| `/follower/command_connected` | `std_msgs/msg/Bool` | `/follower/velocity_guard` | fresh upstream command 상태가 바뀔 때 발행 |
| `/follower/status` | `std_msgs/msg/String` | `/follower/velocity_guard` | 매 timer의 `ACTIVE`/`READY` heartbeat; cooperation이 구독 |

Header 없는 metric 세 개는 관찰용이다. Controller는 metric topic을 독립 조합하지 않고
`base_relative_pose` 한 sample에서 forward, lateral, bearing을 다시 계산한다.

### 6.3 Services와 runtime source control

| Interface | Type | 의미 |
|---|---|---|
| `/follower/approach/enable` | `std_srvs/srv/SetBool` | controller enable/disable; cache 삭제와 즉시 raw zero |
| `/follower/velocity_guard/enable` | `std_srvs/srv/SetBool` | final gate enable/disable; cache 삭제와 즉시 safe zero |
| `/follower/command_selector/set_parameters` | `rcl_interfaces/srv/SetParameters` | `source_mode`의 STOP/APPROACH/COOPERATION 전환 |

Selector source는 custom service가 아니라 표준 ROS parameter interface로 바꾼다.

```bash
ros2 param get /follower/command_selector source_mode
ros2 param set /follower/command_selector source_mode STOP
ros2 param set /follower/command_selector source_mode APPROACH
ros2 param set /follower/command_selector source_mode COOPERATION
```

## 7. Base state와 controller policy

State priority는 표의 위에서 아래 순서다. Exact tolerance boundary는 tolerance 안으로
처리한다. 기본 base target 구간은 `0.25 ± 0.03 m`, lateral tolerance는 `±0.02 m`,
bearing tolerance는 `±5 deg`, stable time은 `0.8 s`다.

| 우선순위/상태 | Base 조건 | 의미 | Controller output policy |
|---|---|---|---|
| 1 `TAG_LOST` | sample/ID 없음, invalid/stale, non-forward 또는 TF 실패 | 유효한 base sample 없음 | raw zero |
| 2 `TURN_LEFT` | `bearing > +5°` | 태그가 로봇 왼쪽 | `linear.x=0`, `angular.z>0` |
| 3 `TURN_RIGHT` | `bearing < -5°` | 태그가 로봇 오른쪽 | `linear.x=0`, `angular.z<0` |
| 4 `APPROACH` | bearing 허용, `forward > 0.28 m` | 목표 구간보다 멂 | `linear.x>0`, continuous angular correction |
| 5 `TOO_CLOSE` | bearing 허용, `forward < 0.22 m` | 목표 구간보다 가까움 | 현재 reverse policy에서 raw zero |
| 6 `FINE_ALIGN_LEFT` | forward/bearing 허용, `lateral > +0.02 m` | 목표 거리에서 왼쪽 오차 | `linear.x=0`, sign-valid positive angular |
| 7 `FINE_ALIGN_RIGHT` | forward/bearing 허용, `lateral < -0.02 m` | 목표 거리에서 오른쪽 오차 | `linear.x=0`, sign-valid negative angular |
| 8 `STABILIZING` | 모든 tolerance 안, 연속 0.8 s 미만 | 안정 조건 확인 중 | raw zero |
| 9 `ALIGNED` | 모든 tolerance를 0.8 s 연속 유지 | software alignment 완료 | raw zero |

Tag ID 변경, lost, tolerance 이탈 또는 clock rollback은 stability history를 reset한다.

## 8. Approach controller

State는 동작 종류를 고르고 한 stamped pose에서 계산한 연속 오차는 command 크기를 정한다.

```text
forward_error = base_forward_distance - target_forward

v_candidate = linear_gain * forward_error

w_candidate = angular_gain * base_bearing
              + lateral_gain * base_lateral_error
```

- `APPROACH`: `v_candidate`와 `w_candidate`를 raw limit로 clamp한다.
- `TURN_LEFT/RIGHT`: lateral term 없이 `angular_gain * bearing`만 사용하고 state와 sign이
  다르면 zero로 fail closed한다.
- `FINE_ALIGN_LEFT/RIGHT`: `linear.x=0`; `w_candidate` sign이 state와 맞을 때만 회전한다.
- `allow_reverse=false`이므로 negative linear candidate는 0이다.
- 모든 output은 `linear.x`, `angular.z`만 채운다. `linear.y/z`, `angular.x/y`는 zero다.

Non-zero raw에는 다음 조건이 모두 필요하다.

1. controller가 service로 enabled
2. `detected=true`
3. selected `tag_id=target_tag_id=0`
4. pose frame이 `base_link`
5. position/quaternion이 finite, quaternion non-zero, `x>0`
6. source stamp와 local receipt가 `pose_timeout=1.20 s` 안
7. state가 해당 pose 뒤 `sample_sync_tolerance=0.10 s` 안에 수신
8. state가 알려진 non-stop state이며 continuous error가 valid

Enable, detection 또는 ID 전환, stale, incoherent state는 cache를 폐기한다. Controller는
20 Hz로 non-zero 또는 명시적 zero raw command를 계속 발행하고 정상 종료 전에 zero를
발행한다.

### 8.1 Humble usb_cam source-stamp 주의

Follower 실기에서 ROS 2 Humble `usb_cam`의 V4L monotonic→epoch 변환으로 process 시작
fractional second에 따른 고정 source-age offset `0.31–0.79 s`를 확인했다. Source stamp를
`now()`로 덮어쓰지 않고 보존하기 위해 controller `pose_timeout`은 1.20 s로 설정했다.
Selector와 guard는 local receipt watchdog `0.35/0.30 s`를 별도로 유지하므로 downstream
publisher loss는 훨씬 빠르게 zero가 된다.

## 9. Command Selector — Leader와 다른 Follower 구조

Leader의 local approach pipeline은 controller에서 guard로 직접 들어간다. Follower에는
두 독립 source가 이미 존재한다.

- local AprilTag approach: `/follower/approach/cmd_vel_raw`
- Leader cooperation: `/follower/cmd_vel`

ROS 2는 같은 output topic의 여러 publisher에 priority를 부여하지 않는다. 따라서 Follower는
명시적 selector를 사용한다.

| Source | 입력 | 동작 |
|---|---|---|
| `STOP` | 사용 안 함 | 항상 selected zero; startup default |
| `APPROACH` | `/follower/approach/cmd_vel_raw` | 0.35 s 안의 fresh command만 전달 |
| `COOPERATION` | `/follower/cmd_vel` | 0.50 s 안의 fresh command만 전달 |

Selector는 선택되지 않은 source callback을 cache하지 않는다. Source가 바뀌면 두 cache를
모두 삭제하고 즉시 zero를 발행한 뒤, 전환 이후 선택 source에서 새 command가 올 때까지
zero를 유지한다. NaN/inf 또는 unused Twist axis는 reject하고 zero를 발행한다. Speed clamp,
reverse policy와 slew는 최종 guard 책임이다.

실제 cooperation publisher는 Leader Orin의 `/leader_cooperation` node다. Cooperation을 쓰지
않는 AprilTag 검증에서는 해당 node를 실행하지 않고 `/follower/cmd_vel` publisher count가
0인지 확인한다.

## 10. Follower Safety Guard

`follower_control`을 최종 safety boundary로 유지한다. 새 guard를 겹쳐 실행하지 않는다.

### 10.1 보존한 기존 기능/interface

- standalone input `/follower/cmd_vel`
- final output `/follower/safe_cmd_vel`
- `/follower/command_connected` freshness transition
- `/follower/status`의 `ACTIVE`/`READY` heartbeat
- `linear.x/angular.z` speed clamp
- 0.3 s local watchdog
- NaN/inf zero, publisher loss zero, shutdown zero

### 10.2 강화한 기능

| 기능 | 실제 동작 |
|---|---|
| explicit enable | startup false; enable 후 fresh upstream을 새로 기다림 |
| integrated input | `selected_velocity_guard.launch.py`가 input만 `/follower/selected_cmd_vel`로 override |
| all-axis validation | 6축 모두 finite, unused axes가 epsilon을 넘으면 전체 reject |
| reverse policy | `allow_reverse=false`에서 negative `linear.x` reject |
| clamp | planar command를 `±0.25 m/s`, `±0.8 rad/s`로 clamp |
| slew | linear `0.25 m/s²`, angular `0.8 rad/s²` 양방향 제한 |
| dt robustness | slew update `dt`를 최대 0.10 s로 제한; invalid/non-positive dt는 zero |
| timeout/invalid | 즉시 output과 slew state를 zero로 reset |
| shutdown | configurable 3회 zero burst |

Valid non-zero→zero command에는 감속 slew가 적용된다. Disabled, invalid 또는 timeout은
safety transition이므로 즉시 zero다.

Standalone cooperation compatibility:

```text
/follower/cmd_vel → velocity_guard.launch.py → /follower/safe_cmd_vel
```

Selector-integrated mode:

```text
/follower/selected_cmd_vel → selected_velocity_guard.launch.py
                           → /follower/safe_cmd_vel
```

두 guard launch를 동시에 실행하면 안 된다.

## 11. Parameter reference

아래 값은 별도 override가 없을 때 launch가 실제로 로드하는 값이다. `Final`은 interface나
현재 safety policy가 확정됐다는 뜻이며, motor/grasp tuning 완료를 뜻하지 않는다.

### 11.1 Camera launch와 measured extrinsic

| Parameter | Default | Unit | Role | Final/Provisional |
|---|---:|---|---|---|
| `video_device` | `/dev/video0` | device | V4L2 camera | 설치 구성 |
| `framerate` | `30.0` | Hz | native YUYV capture | 실기 검증 구성 |
| `io_method` | `mmap` | - | usb_cam I/O | 실기 검증 구성 |
| `pixel_format` | `yuyv` | - | native payload; output encoding `yuv422_yuy2` | 실기 검증 구성 |
| `av_device_format` | `YUV422P` | - | usb_cam format helper | 설치 구성 |
| `image_width` | `640` | px | calibrated width | calibration 구성 |
| `image_height` | `480` | px | calibrated height | calibration 구성 |
| `camera_name` | `follower_usb_camera` | - | CameraInfo name | interface |
| `frame_id` | `follower/follower_camera_optical_frame` | frame | image header frame | interface |
| mount `x/y/z` | `0.042/0/0.120` | m | base→camera body origin | measured initial, provisional |
| mount `roll/pitch/yaw` | `0/0/0` | rad | level/forward body assumption | provisional |
| optical `roll/pitch/yaw` | `-π/2/0/-π/2` | rad | REP-103 optical conversion | convention |

### 11.2 Camera calibration YAML

Source: `config/follower_usb_camera.yaml`

| Parameter | Value | Role | Final/Provisional |
|---|---|---|---|
| `image_width`, `image_height` | `640`, `480` | calibration resolution | current calibration |
| `camera_name` | `follower_usb_camera` | camera identity | interface |
| `distortion_model` | `plumb_bob` | lens model | current calibration |
| `camera_matrix.data` | `[428.37374,0,365.82626, 0,421.97669,196.55999, 0,0,1]` | intrinsic K | measured calibration |
| `distortion_coefficients.data` | `[-0.306573,0.073785,0.006517,-0.005485,0]` | D coefficients | measured calibration |
| `rectification_matrix.data` | identity 3×3 | R | current calibration |
| `projection_matrix.data` | `[332.95392,0,382.35232,0, 0,368.74911,188.1861,0, 0,0,1,0]` | projection P | measured calibration |

### 11.3 apriltag_ros YAML

Source: `config/apriltag.yaml`

| Parameter | Default | Unit | Role | Final/Provisional |
|---|---:|---|---|---|
| `image_transport` | `raw` | - | rectified raw transport | interface |
| `family` | `36h11` | - | tag family | mission 구성 |
| `size` | `0.050` | m | global tag size | measured tag |
| `max_hamming` | `0` | bits | reject corrected IDs | safety policy |
| `detector.threads` | `4` | threads | detector workers | compute tuning |
| `detector.decimate` | `1.0` | ratio | image decimation | quality tuning |
| `detector.blur` | `0.0` | px | detector blur | quality tuning |
| `detector.refine` | `true` | bool | edge refinement | quality tuning |
| `detector.sharpening` | `0.25` | - | decode sharpening | quality tuning |
| `detector.debug` | `false` | bool | debug output | software setting |
| `pose_estimation_method` | `pnp` | - | pose solver | software setting |
| `tag.ids` | `[0,1,2]` | ID list | configured IDs | mission 구성 |
| `tag.frames` | `follower/tag36h11:0..2` | frames | ID-specific TF children | interface |
| `tag.sizes` | `[0.050,0.050,0.050]` | m | ID-specific sizes | measured tag |

### 11.4 `/follower/apriltag_approach`

Source: `config/approach.yaml`

| Parameter | Default | Unit | Role | Final/Provisional |
|---|---:|---|---|---|
| `source_frame` | `follower/follower_camera_optical_frame` | frame | camera pose/TF source | interface |
| `base_frame` | `base_link` | frame | base output target | interface |
| `tf_lookup_timeout` | `0.05` | s | exact-stamp base TF wait | safety timing |
| `tag_frame_pattern` | `follower/tag36h11:{id}` | frame pattern | candidate TF frame | interface |
| `target_tag_id` | `0` | ID | fixed target; `-1` is multi-tag | mission 구성 |
| `allowed_tag_ids` | `[0,1,2]` | list | multi-tag candidates | mission 구성 |
| `selection_mode` | `priority` | - | `priority` or `nearest` | mission 구성 |
| `target_distance` | `0.15` | m | 기존 camera z target | provisional, grasp 아님 |
| `distance_tolerance` | `0.02` | m | camera z tolerance | provisional |
| `lateral_tolerance` | `0.02` | m | camera x tolerance | provisional |
| `angle_tolerance_deg` | `5.0` | deg | camera angle tolerance | provisional |
| `tag_timeout` | `2.0` | s | camera/base source freshness | 실기 timestamp 대응 trial |
| `stable_time` | `0.8` | s | camera stable duration | provisional |
| `publish_rate` | `20.0` | Hz | state/output timer | software setting |
| `filter_window` | `5` | samples | distinct-stamp translation median | software setting |
| `base_target_forward` | `0.25` | m | base stop target | provisional, grasp 아님 |
| `base_forward_tolerance` | `0.03` | m | base forward tolerance | provisional |
| `base_lateral_tolerance` | `0.02` | m | base lateral tolerance | provisional |
| `base_bearing_tolerance_deg` | `5.0` | deg | base bearing tolerance | provisional |
| `base_stable_time` | `0.8` | s | base stable duration | provisional |

### 11.5 `/follower/approach_controller`

Source: `follower_approach_control/config/approach_controller.yaml`

| Parameter | Default | Unit | Role | Final/Provisional |
|---|---:|---|---|---|
| `base_frame` | `base_link` | frame | accepted pose frame | interface |
| `target_tag_id` | `0` | ID | accepted selected ID | mission 구성 |
| `enabled_on_startup` | `false` | bool | startup raw gate | safety default |
| `publish_rate` | `20.0` | Hz | raw output rate | software setting |
| `pose_timeout` | `1.20` | s | source and receipt freshness | measured-offset trial |
| `sample_sync_tolerance` | `0.10` | s | state-after-pose receipt window | safety timing |
| `target_forward` | `0.25` | m | controller forward target | provisional, grasp 아님 |
| `linear_gain` | `0.20` | `(m/s)/m` | forward P gain | provisional tuning |
| `angular_gain` | `0.80` | `(rad/s)/rad` | bearing P gain | provisional tuning |
| `lateral_gain` | `0.50` | `(rad/s)/m` | lateral angular gain | provisional tuning |
| `max_raw_linear_speed` | `0.05` | m/s | raw linear saturation | software validation limit |
| `max_raw_angular_speed` | `0.20` | rad/s | raw angular saturation | software validation limit |
| `allow_reverse` | `false` | bool | reverse policy | current safety policy |

`base_target_forward`와 `target_forward`는 함께 변경해야 한다. 현재 0.25 m는 Leader의 현재
software target과 맞춘 값이지만 실제 gripper/TCP grasp target은 아니다.

### 11.6 `/follower/command_selector`

Source: `follower_command_selector/config/command_selector.yaml`

| Parameter | Default | Unit | Role | Final/Provisional |
|---|---:|---|---|---|
| `source_mode` | `STOP` | enum string | explicit source ownership | safety default |
| `publish_rate` | `50.0` | Hz | selected output rate | software setting |
| `approach_timeout` | `0.35` | s | local source receipt freshness | trial safety |
| `cooperation_timeout` | `0.50` | s | remote source receipt freshness | existing cooperation timing |
| `axis_epsilon` | `1.0e-9` | SI axis | unused-axis tolerance | software safety |

### 11.7 `/follower/velocity_guard`

Source: `follower_control/config/velocity_guard.yaml`

| Parameter | Default | Unit | Role | Final/Provisional |
|---|---:|---|---|---|
| `guard_enabled_on_startup` | `false` | bool | startup final gate | safety default |
| `command_timeout` | `0.30` | s | valid upstream receipt timeout | existing safety timing |
| `publish_rate` | `50.0` | Hz | safe/status output rate | software setting |
| `max_linear_speed` | `0.25` | m/s | final linear clamp | existing compatibility, motor tuning 전 |
| `max_angular_speed` | `0.80` | rad/s | final angular clamp | existing compatibility, motor tuning 전 |
| `max_linear_acceleration` | `0.25` | m/s² | linear slew | provisional motor tuning |
| `max_angular_acceleration` | `0.80` | rad/s² | angular slew | provisional motor tuning |
| `max_slew_dt` | `0.10` | s | maximum slew integration dt | software safety |
| `axis_epsilon` | `1.0e-9` | SI axis | unused-axis tolerance | software safety |
| `allow_reverse` | `false` | bool | negative linear reject | current safety policy |
| `shutdown_stop_count` | `3` | messages | shutdown zero burst | software safety |
| `command_topic` | `/follower/cmd_vel` | topic | standalone input | preserved interface |
| `safe_command_topic` | `/follower/safe_cmd_vel` | topic | final output | preserved interface |

Integrated launch는 `command_topic`만 `/follower/selected_cmd_vel`로 override한다.

## 12. Selective build

ROS 2 Humble, `usb_cam`, `image_proc`, `apriltag_ros`와 repository dependency가 설치돼 있다고
가정한다. 전체 workspace build는 필요하지 않다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
  follower_supply_perception \
  follower_approach_control \
  follower_command_selector \
  follower_control

source install/local_setup.bash
```

실제 Leader cooperation node까지 사용할 때만 별도로 `leader_cooperation`을 build한다.
Test-only COOPERATION source 검증에는 필요하지 않다.

## 13. Unit test

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

colcon test \
  --packages-select \
  follower_supply_perception \
  follower_approach_control \
  follower_command_selector \
  follower_control \
  --event-handlers console_direct+

colcon test-result --verbose
```

2026-08-29 최종 실제 실행 결과:

| Package | Passed | Failed | Errors | Skipped |
|---|---:|---:|---:|---:|
| `follower_supply_perception` | 101 | 0 | 0 | 0 |
| `follower_approach_control` | 44 | 0 | 0 | 0 |
| `follower_command_selector` | 33 | 0 | 0 | 0 |
| `follower_control` | 58 | 0 | 0 | 0 |
| 합계 | **236** | **0** | **0** | **0** |

자동 test는 invalid quaternion, NaN/inf, exact boundary, stale/lost/recovery, state stable timer,
controller state gate/saturation/coherence, selector ownership/freshness, guard clamp/slew/timeout과
legacy status interface를 다룬다. 자동 test는 수행하지 않은 물리 RIGHT/TARGET/HIDDEN 시험을
대체하지 않는다.

## 14. Live camera launch — 다중 terminal 재현

모든 terminal에서 공통으로 실행한다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

### Terminal 0 — 실행 전 안전·충돌 확인

```bash
pgrep -af 'stm32|uart|motor' || true
fuser -v /dev/video0 || true
ros2 node list
ros2 topic info /follower/cmd_vel --verbose
ros2 topic info /follower/safe_cmd_vel --verbose
```

- STM32/UART/Motor process를 실행하지 않는다.
- Motor power OFF를 물리적으로 확인한다.
- `/dev/video0`을 점유한 기존 camera launch가 없어야 한다.
- Cooperation을 사용하지 않으면 `/follower/cmd_vel` publisher는 0이어야 한다.
- `/follower/safe_cmd_vel`에 hardware subscriber가 없어야 한다.

### Terminal 1 — Camera, rectify, AprilTag, camera/base perception

```bash
ros2 launch follower_supply_perception follower_apriltag.launch.py
```

이 launch는 두 static camera TF publisher도 포함한다. Controller, selector, guard, STM32는
실행하지 않는다.

### Terminal 2 — Detection, base metric와 두 state

```bash
ros2 topic echo /follower/supply/detected std_msgs/msg/Bool
```

필요한 항목을 별도 pane에서 확인한다.

```bash
ros2 topic echo /follower/supply/base_relative_pose \
  geometry_msgs/msg/PoseStamped --once
ros2 topic echo /follower/supply/base_forward_distance \
  std_msgs/msg/Float64 --once
ros2 topic echo /follower/supply/base_lateral_error \
  std_msgs/msg/Float64 --once
ros2 topic echo /follower/supply/base_bearing \
  std_msgs/msg/Float64 --once
ros2 topic echo /follower/alignment/state std_msgs/msg/String
ros2 topic echo /follower/base_alignment/state std_msgs/msg/String
```

영상이 필요하면 software display만 별도로 실행한다.

```bash
ros2 run rqt_image_view rqt_image_view /follower/camera/image_rect
```

### Terminal 3 — Approach controller

```bash
ros2 launch follower_approach_control approach_controller.launch.py
```

Startup log의 `enabled=False`를 확인한다.

### Terminal 4 — Raw command와 controller enable

```bash
ros2 topic echo /follower/approach/cmd_vel_raw \
  geometry_msgs/msg/Twist --once

ros2 service call /follower/approach/enable \
  std_srvs/srv/SetBool "{data: true}"

ros2 topic echo /follower/approach/cmd_vel_raw geometry_msgs/msg/Twist
```

### Terminal 5 — Command selector

```bash
ros2 launch follower_command_selector command_selector.launch.py
```

Startup log의 `source=STOP`을 확인한다.

### Terminal 6 — Source와 selected command

```bash
ros2 param get /follower/command_selector source_mode
ros2 topic echo /follower/selected_cmd_vel geometry_msgs/msg/Twist --once

ros2 param set /follower/command_selector source_mode APPROACH
ros2 topic echo /follower/selected_cmd_vel geometry_msgs/msg/Twist
```

Source 전환 직후 zero가 먼저 나오고 전환 이후 fresh raw가 온 뒤에만 APPROACH command가
나와야 한다.

### Terminal 7 — Integrated safety guard

```bash
ros2 launch follower_control selected_velocity_guard.launch.py
```

이 launch만 selector output을 구독한다. `velocity_guard.launch.py`를 동시에 실행하지 않는다.
Startup log의 `enabled=False`, `input=/follower/selected_cmd_vel`을 확인한다.

### Terminal 8 — Safe output과 guard enable

```bash
ros2 topic echo /follower/safe_cmd_vel geometry_msgs/msg/Twist --once
ros2 topic info /follower/safe_cmd_vel --verbose

ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"

ros2 topic echo /follower/safe_cmd_vel geometry_msgs/msg/Twist
```

Final publisher는 `/follower/velocity_guard` 하나여야 하고 hardware subscriber는 없어야 한다.

## 15. 안전한 enable/source 순서

> **WARNING — 아래 순서를 지키고 hardware subscriber가 있으면 guard를 enable하지 않는다.**

1. STM32/UART/Motor process가 없고 motor power가 OFF인지 확인한다.
2. `/follower/safe_cmd_vel` hardware subscriber가 없는지 `--verbose`로 확인한다.
3. Controller startup log `enabled=False`와 raw zero를 확인한다.
4. Selector `source_mode=STOP`과 selected zero를 확인한다.
5. Guard startup log `enabled=False`와 safe zero를 확인한다.
6. Camera detection, base pose/metric/state와 sign을 먼저 확인한다.
7. Controller를 enable하고 raw만 확인한다. Selector STOP이므로 selected는 계속 zero다.
8. Selector를 `APPROACH`로 바꾸고 fresh raw 이후 selected를 확인한다.
9. Publisher/subscriber를 다시 확인한다.
10. Guard를 enable하고 final safe output을 확인한다.

종료 순서:

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
ros2 param set /follower/command_selector source_mode STOP
ros2 service call /follower/approach/enable \
  std_srvs/srv/SetBool "{data: false}"
```

그 뒤 각 launch terminal을 `Ctrl+C`로 종료한다.

## 16. Physical/software live test 결과

### 16.1 실제 USB camera + tag36h11 ID 0

값은 development validation session의 중앙값 또는 안정 구간 대표값이다. RIGHT, TARGET,
HIDDEN은 사용자 결정으로 수행하지 않았으며 자동 test를 근거로 PASS로 바꾸지 않는다.

| Test | forward `[m]` | lateral `[m]` | bearing `[rad/deg]` | base state | raw `v/w` | selected `v/w` | safe `v/w` | Result |
|---|---:|---:|---:|---|---|---|---|---|
| CENTER | 0.3909 | +0.0164 | +0.04192 / +2.40° | `APPROACH` | 0/0 (gates off) | 0/0 | 0/0 | PASS |
| LEFT | 0.3870 | +0.0750 | +0.19135 / +10.96° | `TURN_LEFT` | 0/+0.15308 | 0/+0.15308 | 0/+0.15309 | PASS |
| RIGHT | - | - | - | - | - | - | - | **NOT VERIFIED** |
| FAR | 0.3676 | +0.0294 | +0.07990 / +4.58° | `APPROACH` | +0.02353/+0.07863 | 동일 | 동일 | PASS |
| TARGET | - | - | - | - | - | - | - | **NOT VERIFIED** |
| HIDDEN | - | - | - | - | - | - | - | **NOT VERIFIED** |

CENTER와 FAR는 camera transport/freshness 최종 조정 전 같은 pose/state/controller logic으로
측정했고, LEFT는 native YUYV 30 Hz와 `pose_timeout=1.20 s` 최종 설정에서 다시 통과했다.
따라서 final camera setting의 CENTER/FAR 재측정은 권장하지만 위 값 자체는 실제 수행값이다.
실제 grasp target과 motor stopping distance를 검증한 결과는 아니다.

### 16.2 Selector와 guard software-only live 결과

Production controller와 충돌하지 않는 별도 ROS domain에서 test-only publisher로 수행했다.

| Test | Test input `v/w` | Selected `v/w` | Safe `v/w` | Result |
|---|---|---|---|---|
| `SELECTOR_STOP` | APPROACH +0.04/+0.10, COOP +0.08/-0.20 동시 | 0/0 | - | PASS |
| `APPROACH_SOURCE` | +0.04/+0.10 | +0.04/+0.10 | - | PASS |
| unselected COOPERATION | +0.08/-0.20 | 무시; APPROACH 유지 | - | PASS |
| `COOPERATION_SOURCE` | +0.08/-0.20 | +0.08/-0.20 | - | PASS |
| source switch before fresh input | 과거 command만 존재 | 0/0 | - | PASS |
| `GUARD_DISABLED` | selected +0.10/+0.20 | +0.10/+0.20 | 0/0 | PASS |
| guard enabled | selected +0.10/+0.20 | +0.10/+0.20 | slew 후 +0.10/+0.20 | PASS |
| `SOURCE_TIMEOUT` | selected publisher 중단 | publisher 없음 | 0.30 s 후 0/0 | PASS |

Invalid/over-limit/unused-axis/reverse는 236개 자동 test에 포함했다. 실기에서 억지로 NaN을
발행하지 않았다.

## 17. Selector test-only 재현

실제 controller와 cooperation publisher를 먼저 종료한다.

```bash
ros2 topic info /follower/approach/cmd_vel_raw --verbose
ros2 topic info /follower/cmd_vel --verbose
```

Selector만 실행한 뒤 test source를 하나씩 선택한다.

```bash
ros2 launch follower_command_selector command_selector.launch.py

ros2 param set /follower/command_selector source_mode APPROACH
ros2 topic pub --rate 20 \
  /follower/approach/cmd_vel_raw geometry_msgs/msg/Twist \
  "{linear: {x: 0.04}, angular: {z: 0.10}}"

ros2 param set /follower/command_selector source_mode COOPERATION
ros2 topic pub --rate 20 \
  /follower/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.08}, angular: {z: -0.20}}"
```

별도 terminal에서 selected를 본다.

```bash
ros2 topic echo /follower/selected_cmd_vel geometry_msgs/msg/Twist
```

각 source switch 직후 publisher를 시작하기 전에는 zero여야 한다. 두 test publisher를 동시에
실행해도 selected에는 명시적으로 선택한 source만 나타나야 한다. 종료 시 STOP으로 돌린다.

## 18. Guard disabled/enabled/timeout 재현

Selector-integrated guard를 실행하되 실제 selector는 종료하고 synthetic selected input만
사용한다.

```bash
ros2 launch follower_control selected_velocity_guard.launch.py
```

Guard disabled에서 valid input을 발행한다.

```bash
ros2 topic pub --rate 20 \
  /follower/selected_cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.20}}"
```

다른 terminal:

```bash
ros2 topic echo /follower/safe_cmd_vel geometry_msgs/msg/Twist
```

Disabled에서는 계속 zero다. Enable 후에는 slew 범위 안에서 input에 도달한다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"
```

Test publisher를 `Ctrl+C`로 중단하면 `command_timeout=0.30 s` 후 safe가 즉시 zero로
reset되고 status가 `READY`가 되어야 한다.

## 19. Troubleshooting

### Camera가 나오지 않음

```bash
fuser -v /dev/video0 || true
v4l2-ctl --get-fmt-video --get-parm -d /dev/video0
ros2 node list | grep camera
ros2 topic info /follower/camera/image_raw --verbose
```

중복 camera process를 종료한다. 현재 장치의 검증 모드는 YUYV 640×480 30 FPS다. 이 장치는
같은 해상도 MJPEG를 120.101 FPS로만 제공했으므로 launch 값을 임의로 `mjpeg2rgb` 저속
timer로 바꾸지 않는다.

### AprilTag가 검출되지 않음

```bash
ros2 topic echo /follower/camera/camera_info sensor_msgs/msg/CameraInfo --once
ros2 topic info /follower/camera/image_rect --verbose
ros2 topic echo /follower/apriltag/detections --once
ros2 run tf2_ros tf2_echo \
  follower/follower_camera_optical_frame 'follower/tag36h11:0'
```

Tag family `36h11`, ID 0, size 0.050 m, rectified image와 CameraInfo를 확인한다. Tag가 영상
가장자리에 있거나 작거나 가려지면 detection이 끊길 수 있다.

### Base pose/metric이 나오지 않음

```bash
ros2 topic echo /follower/supply/detected std_msgs/msg/Bool --once
ros2 topic echo /follower/supply/tag_id std_msgs/msg/Int32 --once
ros2 run tf2_ros tf2_echo base_link follower/follower_camera_optical_frame
ros2 topic info /follower/supply/base_relative_pose --verbose
ros2 topic echo /follower/base_alignment/state std_msgs/msg/String --once
```

Static TF 두 개, tag TF, source frame과 timestamp freshness를 순서대로 확인한다. Lost/TF
실패 중 pose와 metric이 재발행되지 않는 것은 정상이다.

### TF extrapolation 또는 exact-stamp lookup 실패

```bash
ros2 topic echo /follower/supply/relative_pose \
  geometry_msgs/msg/PoseStamped --once
ros2 topic echo /tf tf2_msgs/msg/TFMessage --once
ros2 param get /follower/apriltag_approach tf_lookup_timeout
```

Pose stamp를 `now()`로 덮어쓰거나 latest transform으로 우회하지 않는다. Static camera TF와
tag observation stamp의 clock domain을 확인한다. 실패한 base sample만 skip되는 것이
정상이다.

### 좌우 sign이 반대임

```bash
ros2 topic echo /follower/supply/lateral_error std_msgs/msg/Float64 --once
ros2 topic echo /follower/supply/angle std_msgs/msg/Float64 --once
ros2 topic echo /follower/supply/base_lateral_error std_msgs/msg/Float64 --once
ros2 topic echo /follower/supply/base_bearing std_msgs/msg/Float64 --once
```

로봇 왼쪽 tag는 camera `x/angle<0`이지만 base `y/bearing>0`이다. Camera sign을 base state에
그대로 복사하지 않는다. TF chain이 실제 장착과 다르면 임의 sign flip 대신 extrinsic부터
재측정한다.

### Base state가 나오지 않음

```bash
ros2 topic info /follower/base_alignment/state --verbose
ros2 topic echo /follower/supply/base_relative_pose \
  geometry_msgs/msg/PoseStamped --once
ros2 param get /follower/apriltag_approach tag_timeout
```

Pose가 없거나 stale/invalid이면 state만 `TAG_LOST`로 발행되고 metric은 나오지 않는다.

### Raw command가 항상 zero

```bash
ros2 topic echo /follower/supply/detected std_msgs/msg/Bool --once
ros2 topic echo /follower/supply/tag_id std_msgs/msg/Int32 --once
ros2 topic echo /follower/supply/base_relative_pose \
  geometry_msgs/msg/PoseStamped --once
ros2 topic echo /follower/base_alignment/state std_msgs/msg/String --once
ros2 param get /follower/approach_controller pose_timeout
```

Controller enable service 응답을 확인한다. Disabled, ID 불일치, source/receipt stale,
pose-state mismatch 또는 stop state이면 zero가 정상이다. Humble usb_cam offset 때문에 YAML의
1.20 s가 실제로 로드됐는지도 확인한다.

### Selector output이 항상 zero

```bash
ros2 param get /follower/command_selector source_mode
ros2 topic info /follower/approach/cmd_vel_raw --verbose
ros2 topic info /follower/cmd_vel --verbose
ros2 topic echo /follower/selected_cmd_vel geometry_msgs/msg/Twist
```

STOP이면 zero가 정상이다. 선택 source command가 0.35/0.50 s보다 stale하거나 switch 이후
fresh sample이 없으면 zero다.

### `/follower/safe_cmd_vel`이 항상 zero

```bash
ros2 topic echo /follower/selected_cmd_vel geometry_msgs/msg/Twist --once
ros2 topic info /follower/safe_cmd_vel --verbose
ros2 param get /follower/velocity_guard command_topic
ros2 param get /follower/velocity_guard command_timeout
ros2 topic echo /follower/status std_msgs/msg/String --once
```

Integrated launch가 `command_topic=/follower/selected_cmd_vel`인지 확인한다. Guard enable 직후
과거 command를 재사용하지 않고 fresh input을 기다린다. `READY`는 upstream stale/없음을
뜻한다.

### Publisher가 여러 개임

```bash
ros2 topic info /follower/approach/cmd_vel_raw --verbose
ros2 topic info /follower/cmd_vel --verbose
ros2 topic info /follower/selected_cmd_vel --verbose
ros2 topic info /follower/safe_cmd_vel --verbose
```

Raw는 controller 하나, selected는 selector 하나, safe는 guard 하나여야 한다. Test
publisher와 production publisher를 같은 source topic에 동시에 두지 않는다. ROS 2가 여러
publisher 중 priority를 자동 결정한다고 가정하지 않는다.

### Source stale 또는 guard timeout

```bash
ros2 topic hz /follower/approach/cmd_vel_raw
ros2 topic hz /follower/selected_cmd_vel
ros2 topic echo /follower/command_connected std_msgs/msg/Bool
ros2 topic echo /follower/status std_msgs/msg/String
```

Selector는 stale source를 zero로 바꾸지만 50 Hz selected publisher 자체는 계속 동작한다.
Guard input publisher 자체가 멈추면 0.30 s 뒤 `COMMAND_TIMEOUT_STOPPED`, safe zero,
`status=READY`가 된다. `command_connected`는 transition topic이므로 늦게 echo하면 새 전환
전까지 sample이 없을 수 있다.

### Duplicate node 또는 launch

```bash
ros2 node list | sort
pgrep -af 'follower_apriltag|usb_cam|apriltag_node|approach_controller|command_selector|velocity_guard'
```

기존 수동 camera pipeline과 `follower_apriltag.launch.py`를 동시에 실행하지 않는다.
`velocity_guard.launch.py`와 `selected_velocity_guard.launch.py`도 동시에 실행하지 않는다.

## 20. PASS/FAIL 기준과 현재 판정

| Criterion | Evidence | Result |
|---|---|---|
| camera perception regression | perception 101 tests, live CENTER/LEFT/FAR detection | PASS |
| exact-stamp base TF/validation | base pose unit tests와 live `base_link` pose | PASS |
| base metric sign | live LEFT positive; RIGHT physical test | PARTIAL — RIGHT NOT VERIFIED |
| base state | 9-state unit tests, live APPROACH/TURN_LEFT | PASS (physical subset) |
| controller | 44 tests, live LEFT/FAR raw | PASS |
| selector deterministic | 33 tests와 dual-source software live test | PASS |
| guard | 58 tests와 enable/timeout software live test | PASS |
| lost→zero | automatic tests | PASS; physical HIDDEN NOT VERIFIED |
| timeout→zero | guard publisher-stop live test | PASS |
| `/follower/safe_cmd_vel` hardware subscriber 없음 | `ros2 topic info --verbose` live 확인 | PASS |
| STM32/UART/Motor 미연결 | process/topic audit | PASS; physical motor power는 software로 NOT VERIFIED |

전체 physical table의 PASS 조건은 LEFT/RIGHT sign, FAR approach, TARGET stabilizing/aligned,
HIDDEN lost와 각 raw/selected/safe policy가 모두 일치하는 것이다. 현재 RIGHT/TARGET/HIDDEN은
NOT VERIFIED이므로 실제 closed-loop readiness를 주장하지 않는다.

## 21. 아직 하지 않은 작업

- [ ] precise Follower camera extrinsic calibration
- [ ] final gripper/TCP target와 tolerance 결정
- [ ] STM32 UART integration
- [ ] Motor power OFF wheel-air test
- [ ] Ground low-speed test
- [ ] 실제 AprilTag closed-loop motor test
- [ ] Hardware speed/acceleration tuning
- [ ] Cooperation + AprilTag full mission arbitration validation
- [ ] Gripper control
- [ ] Grasp validation
- [ ] Physical RIGHT/TARGET/HIDDEN scenario 재검증

## 22. Known limitations

- Camera mount translation과 level/forward orientation은 measured initial extrinsic이며 정밀
  calibration이 아니다.
- `base_target_forward=target_forward=0.25 m`는 software validation 값이다. 실제 grasp,
  TCP 또는 motor stopping distance가 아니다.
- ROS 2 Humble usb_cam source stamp에는 process-start dependent fixed offset가 관찰됐다.
  현재 timeout은 이를 수용하지만 장기적으로 driver fix 또는 검증된 timestamp source가
  바람직하다.
- Follower `base_link`는 현재 repository static TF chain의 root이며 namespaced frame이
  아니다. 다중 로봇 통합 전 TF frame ownership을 다시 검토해야 한다.
- Selector source는 runtime parameter다. Mission-level arbitration state machine과 operator
  authorization UI는 아직 없다.
- Guard limit은 기존 cooperation compatibility와 software validation 값이며 motor-safe tuning
  완료값이 아니다.
- `/follower/safe_cmd_vel` 이후 hardware bridge는 의도적으로 연결하지 않았다.

## 23. 종료와 repository 확인

현재 작업 diff의 기능별 요약은 다음과 같다. 이는 commit 목록이 아니라 working tree의
검토용 요약이며 자동 commit/push는 수행하지 않았다.

| 구분 | 변경 요약 |
|---|---|
| `follower_supply_perception` | 기존 camera state 유지, measured camera TF, exact-stamp base pose/metric와 병렬 base state 추가 |
| `follower_approach_control` | stamped base pose/state 기반 raw Twist controller 패키지와 tests 추가 |
| `follower_command_selector` | STOP/APPROACH/COOPERATION deterministic selector 패키지와 tests 추가 |
| `follower_control` | 기존 public status/output을 유지하며 enable, axis/reverse validation, clamp/slew/watchdog 강화 및 integrated launch 추가 |
| docs/README | 본 가이드, Follower package README와 1차 진행 요약을 실제 상태 및 `NOT VERIFIED` 결과에 맞게 갱신 |

상세 파일별 변경은 아래 `git status`와 `git diff`로 확인한다. 새 파일은 commit 전에는
`git diff` 기본 출력에 포함되지 않으므로 `git status --short`도 반드시 함께 확인한다.

모든 gate를 닫고 launch terminal을 `Ctrl+C`로 종료한 뒤 확인한다.

```bash
cd ~/damgc_robot
pgrep -af 'stm32|uart|motor|usb_cam|apriltag_node|approach_controller|command_selector|velocity_guard' || true
fuser -v /dev/video0 || true
git status --short
git diff --check
git diff
```

문서 절차는 STM32 bridge, UART 또는 motor를 시작하지 않는다. Hardware integration은 별도
E-stop, bridge watchdog, motor-safe limit 및 승인된 test procedure가 준비된 뒤 진행한다.
