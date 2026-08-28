# Leader D435 Extrinsic·base_link TF 검증 기록

> 이 문서는 extrinsic을 검증한 당시 기록이다. 후속 base pose/metric 구현과 실제
> CENTER/LEFT/RIGHT/FARTHER/HIDDEN 재검증 결과는
> [Leader base_link AprilTag Pose·Metric 구현 및 재현 검증 가이드](../../../../src/leader/rescue_robot_apriltag/docs/LEADER_BASE_LINK_POSE_METRICS_VALIDATION_GUIDE.md)를 따른다.

## 1. 작업 목적

Leader D435 장착 실측값을 URDF의 `base_link → camera_link` fixed joint에 반영하고,
실제 ROS 2 TF graph에서 카메라와 ID 0 AprilTag transform이 연결되는지 기록한다.
이번 기록에서는 motor, STM32, `/leader/cmd_vel`, approach controller를 실행하거나
수정하지 않았다.

## 2. 실측 기준점과 좌표축

- 기준 원점: Leader URDF의 `base_link`
- `+X`: 로봇 전방
- `+Y`: 로봇 왼쪽
- `+Z`: 위쪽
- 카메라 방향 후보: 차체 정면과 평행
- 초기 orientation 후보: roll=0, pitch=0, yaw=0 rad

## 3. D435 측정값

```text
x = +0.042 m
y =  0.000 m
z = +0.130 m
rpy = 0 0 0
```

D435 앞면까지의 전방 거리는 약 `0.053 m`로 측정했지만, URDF mounting reference
후보에는 D435 본체 기준 중심의 약 `0.042 m`를 사용했다. RealSense driver의
`camera_link` 원점이 본체 외형 중심과 완전히 동일하다고 단정하지 않았으며,
driver가 제공하는 센서 간/optical 변환을 URDF에 복제하지 않았다.

## 4. URDF 적용 내용

적용 파일:

```text
src/leader/rescue_robot_description/urdf/rescue_robot.urdf
```

`camera_joint`는 기존에도 `base_link → camera_link` fixed joint였다.

변경 전:

```xml
<origin xyz="0 0 0.070" rpy="0 0 0"/>
```

변경 후:

```xml
<origin xyz="0.042 0 0.130" rpy="0 0 0"/>
```

기존 카메라 높이 주석은 base 기준 실측 mounting 값임을 명확히 하도록 최소 수정했다.

## 5. RealSense 및 AprilTag TF chain

- `robot_state_publisher`: URDF의 `base_link → camera_link`
- `/leader/camera` (`realsense2_camera`): 카메라 내부 frame 및 optical frame
- `/leader/apriltag/apriltag`: 검출 tag TF

```text
base_link
  → camera_link
    → camera_color_frame
      → camera_color_optical_frame
        → leader/tag36h11:0
```

실제 CameraInfo의 `frame_id`는 `camera_color_optical_frame`이었다.

## 6. 빌드 및 실행 명령

```bash
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/local_setup.bash
colcon build --packages-select rescue_robot_description --symlink-install
```

결과:

```text
Finished <<< rescue_robot_description
Summary: 1 package finished
```

실기 실행:

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_depth:=false \
  enable_approach:=false
```

전체 workspace 또는 STM32/DAMGC package는 빌드하지 않았다.

## 7. 정적/카메라 TF 검증 결과

```text
base_link → camera_link
Translation: [0.042, 0.000, 0.130]
RPY:         [0.000, -0.000, 0.000]
```

```text
base_link → camera_color_optical_frame
Translation: [0.042, 0.015, 0.130]
RPY:         [-1.571, 0.006, -1.562]
```

두 번째 결과는 RealSense RGB 센서 offset과 optical frame 회전을 포함하므로
`camera_joint` 값과 동일할 필요가 없다.

## 8. ID 0 AprilTag 검출 및 TF 결과

실제 detection payload:

```text
family: tag36h11
id: 0
frame_id: camera_color_optical_frame
```

사용한 tag frame은 `leader/tag36h11:0`이다. 대표적인 지속 관측값:

```text
base_link → leader/tag36h11:0
Translation: 약 [0.372, -0.033, 0.036] m
```

`Y < 0`이므로 해당 관측은 로봇 기준 오른쪽 태그와 일치한다.

## 9. 위치별 부호 시험 결과

- center: **NOT VERIFIED** (정면 중앙 별도 sample 없음)
- farther: **NOT VERIFIED** (전후 pair를 기록하지 않음)
- left: **NOT VERIFIED** (왼쪽 별도 sample 없음)
- right: `y ≈ -0.033 m` 관측. `+Y=왼쪽` 정의와 부합
- tag hidden: **NOT VERIFIED** (가림 후 stale 동작 미시험)

따라서 farther에서 X 증가, left에서 Y 양수, hidden stale rejection은 완료된 검증으로
기록하지 않는다.

## 10. 중복 publisher 확인

단일 launch 기준 frame parent:

```text
camera_link                 parent: base_link
camera_color_frame          parent: camera_link
camera_color_optical_frame  parent: camera_color_frame
leader/tag36h11:0           parent: camera_color_optical_frame
```

검증 중 이전 launch와 새 launch가 동시에 남아 node가 2세트 보이는 상황이 한 번
발생했다. 이는 저장소의 고정 구조가 아니라 검증 프로세스 중복이었으며 두 세션을
종료했다. 단일 launch 기준으로 같은 transform을 중복 발행하는 구조는 확인되지 않았다.

## 11. 이 기록 시점에 아직 하지 않은 작업

- base-relative `PoseStamped` 추가 구현
- `base_forward_distance`
- `base_lateral_error`
- `base_bearing`
- approach controller
- velocity guard
- `/leader/cmd_vel` 연결
- STM32 motor integration
- gripper control
- center/farther/left/right 전체 순차 시험
- tag hidden 상태 stale transform/pose 시험

## 12. 작업 트리 및 commit 안내

```bash
git status
git diff
```

이번 작업에서는 commit 또는 push를 수행하지 않는다.

추천 commit message:

```text
Update Leader D435 mounting extrinsic and record TF validation
```
