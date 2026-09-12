# Stage 3 Camera XYZ 실행·검증 가이드

## 상태

현재 상태: **VERIFIED (2026-09-12)**

package build와 41개 자동 테스트를 통과했다. 2026-09-12 실제 Jetson + D435에서 RGB,
aligned depth, CameraInfo geometry, 실제 사람 XYZ, 좌/우 X 부호, distance/Z 일치, debug
overlay, 다중 사람 XYZ와 non-empty PoseArray를 확인해 hardware verification을 완료했다.

## 사전 조건과 build

```bash
cd ~/damgc_robot
git status --short
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_survivor
source install/local_setup.bash
colcon test --packages-select rescue_robot_survivor
colcon test-result --verbose
./scripts/build_survivor_runtime.sh
```

AI inference는 host Python이 아닌 `damgc-survivor-yolo:humble` NVIDIA container에서
실행한다. image를 다시 build해야 현재 source가 container install에 반영된다.

선택 runtime smoke test:

```bash
docker run --rm --runtime=nvidia --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -v ~/.cache/damgc-survivor-ultralytics:/root/.cache/ultralytics \
  damgc-survivor-yolo:humble 'check_survivor_ai_runtime --static'
```

## Terminal 1 — Camera

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_depth:=true enable_sync:=true align_depth.enable:=true
```

다음 topic이 생성되어야 한다.

```bash
ros2 topic list | grep /leader/camera
ros2 topic info -v /leader/camera/color/image_rect
ros2 topic info -v /leader/camera/aligned_depth_to_color/image_raw
ros2 topic info -v /leader/camera/color/camera_info
```

## Terminal 2 — Survivor detector

```bash
cd ~/damgc_robot
./scripts/run_survivor_detector.sh
```

정상 startup log에는 RGB, aligned depth, CameraInfo, debug image,
`/leader/survivor/camera_positions`, `CameraInfo.P` 선택과 CUDA device가 표시된다.

overlay만 숨기는 예:

```bash
./scripts/run_survivor_detector.sh show_camera_xyz:=false
```

이 설정에서도 positions topic은 계속 발행한다.

## Terminal 3 — Node, topic, rate와 parameter

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 node list
ros2 topic list | grep /leader/survivor
ros2 topic info -v /leader/survivor/debug_image
ros2 topic info -v /leader/survivor/camera_positions
ros2 topic hz /leader/camera/color/image_rect
ros2 topic hz /leader/camera/aligned_depth_to_color/image_raw
ros2 topic hz /leader/survivor/camera_positions
ros2 param list /leader/person_detector
ros2 param get /leader/person_detector camera_info_topic
ros2 param get /leader/person_detector camera_positions_topic
ros2 param get /leader/person_detector show_camera_xyz
```

## Terminal 4 — CameraInfo와 XYZ topic

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo --once /leader/camera/color/camera_info
ros2 topic echo /leader/survivor/camera_positions
```

확인 항목:

- CameraInfo width/height가 RGB와 같은가
- `p[0]`, `p[5]`가 양수이고 `p[2]`, `p[6]`가 finite인가
- `header.frame_id`가 실제 RGB optical frame인가
- positions의 stamp가 RGB stamp를 보존하는가
- 각 pose orientation의 `w`가 1인가
- person이 없거나 모든 XYZ가 invalid이면 `poses: []`인가

## Terminal 5 — rqt_image_view

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /leader/survivor/debug_image
```

각 person box에 confidence/distance 첫째 줄과 `XYZ (...) m` 또는 `XYZ N/A` 둘째 줄이
표시되는지 확인한다.

## 현재 장비에서 확인한 image geometry

| 데이터 | width×height | encoding | frame_id |
| --- | --- | --- | --- |
| rectified RGB | 640×480 | `rgb8` | `camera_color_optical_frame` |
| aligned depth | 640×480 | `16UC1` | `camera_color_optical_frame` |
| RGB CameraInfo | 640×480 | CameraInfo | `camera_color_optical_frame` |

확인한 P의 주요 값은 `fx=603.71155`, `fy=603.83417`, `cx=313.71130`,
`cy=235.27989`다. 장비 profile이나 resolution을 바꾸면 이 값을 복사해 쓰지 말고
CameraInfo를 다시 확인한다.

## 단일 사람 hardware verification 결과

카메라를 고정하고 bbox 중앙 ROI가 의도한 위치에 오도록 사람을 이동한다. X/Y는 ROI 중심
ray의 좌표이므로 사람 몸 전체 중심과 정확히 같을 필요는 없다.

| Test | 사람/ROI 위치 | 예상 | 실제 확인 결과 | 판정 |
| --- | --- | --- | --- | --- |
| left | 화면 왼쪽 | `X<0` | negative X 확인, 실제 수치 미기록 | PASS |
| right | 화면 오른쪽 | `X>0` | positive X 확인, 실제 수치 미기록 | PASS |
| distance/Z | 단일 사람 | distance와 XYZ Z 일치 | debug distance와 PoseArray/debug XYZ Z 일치 | PASS |
| debug overlay | 단일 사람 bbox | 해당 사람 XYZ 표시 | person label 아래 XYZ 정상 표시 | PASS |

별도 수치 로그가 없으므로 절대 오차나 특정 거리 정확도는 이 기록에서 주장하지 않는다.
추후 정밀도 튜닝이 필요하면 줄자로 `0.5`, `1.0`, `1.5`, `2.0`, `3.0 m`를 다시 측정한다.

## 다중 사람 PoseArray hardware verification 결과

| 검증 항목 | 실제 확인 결과 | 판정 |
| --- | --- | --- |
| 여러 사람 동시 검출 | 한 frame에서 여러 person bbox와 XYZ 표시 | PASS |
| 사람별 독립 XYZ | 각 detection에 서로 대응하는 XYZ 계산 | PASS |
| 다중 PoseArray | `/leader/survivor/camera_positions`에 여러 pose 발행 | PASS |
| debug/topic 대응 | debug image의 각 XYZ와 PoseArray XYZ 대응 | PASS |
| frame ID | `camera_color_optical_frame` | PASS |
| orientation | 모든 pose에서 identity quaternion, `w=1` | PASS |
| numbering | frame-local left-to-right `person1..N`, persistent ID 아님 | PASS |

PoseArray는 valid XYZ만 left-to-right 순서로 담는다. 중간 person이 `XYZ N/A`이면 array
index와 화면의 person 번호가 달라질 수 있으며, 이 interface는 tracking ID를 제공하지 않는다.

## Expected result와 판정

- Stage 1 bbox/confidence/좌→우 numbering이 유지된다.
- Stage 2 거리와 PoseArray Z가 같은 median measurement를 사용한다.
- optical axis 부호와 near/far 변화가 예상과 일치한다.
- debug image와 topic이 동시에 갱신된다.
- CameraInfo 또는 XYZ만 invalid일 때 detection/distance는 계속 나온다.
- header stamp와 frame ID가 원본 RGB measurement를 나타낸다.

위 단일·다중 사람 결과와 실제 topic 확인을 근거로 Stage 3를 `VERIFIED`로 판정했다.

현재 runtime 조사에서 기본 CameraInfo publisher가 `RELIABLE + VOLATILE`임을 확인했다.
초기 transient-local subscriber에서 발생한 durability incompatibility는 subscriber를 같은
reliable + volatile profile로 변경해 해결했다.

## 다음 개발 단계

Stage 4는 검증된 camera optical XYZ와 원본 RGB timestamp를 사용해 exact-timestamp TF2
transform을 수행하고 map frame XYZ를 만든다. Stage 3에서는 TF2 또는 map 좌표를 구현하지
않는다.

## Troubleshooting

### `XYZ N/A`만 표시됨

```bash
ros2 topic hz /leader/camera/color/camera_info
ros2 topic echo --once /leader/camera/color/camera_info
ros2 topic info -v /leader/camera/color/camera_info
```

P, dimensions와 frame ID를 확인한다. CameraInfo와 RGB의 frame/resolution mismatch warning이
있으면 topic을 억지로 조합하지 말고 camera/image_proc profile을 먼저 맞춘다.

### distance도 `N/A`

camera를 `enable_sync:=true align_depth.enable:=true`로 실행했는지 확인한다. RGB와 aligned
depth의 resolution/frame ID, `16UC1` encoding, timestamp 차이와 `sync_slop_sec`를 점검한다.

### positions가 빈 배열임

사람 검출이 없거나, 검출은 있지만 depth/CameraInfo/geometry가 invalid일 수 있다. debug
image에서 distance와 XYZ를 단계별로 구분한다. 빈 배열은 fake origin보다 안전한 정상
degradation 결과다.

### X 또는 Y 부호가 반대임

topic의 frame ID가 optical frame인지 확인한다. optical convention은 right/down/forward다.
`camera_link`의 일반 ROS body frame 축과 혼동하지 않는다.

### Z가 1000배 큼

aligned depth encoding을 확인한다. `16UC1`은 현재 검증한
`depth_scale_m_per_unit=0.001`, `32FC1`은 이미 meter로 처리한다. profile이 바뀌면 실제
known-distance raw 값을 다시 측정해 scale을 검증한다.

### topic rate가 낮음

positions는 YOLO RGB callback당 한 번 발행하므로 detector inference rate를 따른다. CUDA
선택, detector log, input/debug/positions hz를 함께 비교하고 오래된 frame backlog를 만들기
위해 queue를 키우지 않는다.

### container에서 새 parameter/topic이 없음

`./scripts/build_survivor_runtime.sh`로 image를 다시 build한 뒤 detector를 재실행한다.
host `install/`만 갱신해도 container 내부 `/opt/damgc_survivor_ws`는 자동 갱신되지 않는다.
