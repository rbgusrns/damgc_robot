# Stage 3 Camera Optical XYZ 구현

## 1. 상태와 목적

현재 상태는 **VERIFIED (2026-09-12)**다. 코드, package build와 synthetic unit test에 더해
실제 Jetson + D435에서 사람의 camera optical XYZ, 좌/우 X 부호, distance와 Z의 일치,
debug overlay, 다중 사람별 XYZ 및 PoseArray 발행을 확인했다.

Stage 3는 Stage 2가 만든 사람별 대표 측정 `(u, v, Z)`를 RGB camera optical frame의
metric 위치 `(X, Y, Z)`로 변환한다. TF2, `base_link`/`map` 변환, RViz marker, 중복 제거와
persistent tracking은 이 단계에 포함하지 않는다.

## 2. 입력과 처리 흐름

```text
/leader/camera/color/image_rect ─→ YOLO person bbox
                                      ↓
                         bbox 중앙 depth ROI
                                      ├─→ ROI center (u, v)
/leader/camera/aligned_depth_to_color/image_raw
                                      └─→ valid median depth Z [m]
/leader/camera/color/camera_info ─→ rectified intrinsics from P
                                      ↓
                         camera optical XYZ [m]
                              ├─→ debug image
                              └─→ /leader/survivor/camera_positions
```

Stage 2의 ROI와 median Z를 그대로 재사용한다. XYZ용으로 별도의 pixel이나 depth를 선택하지
않으므로 거리 표시와 `PoseArray`의 Z가 서로 다른 측정 경로에서 나오지 않는다.

## Hardware verification 결과

2026-09-12 실제 Jetson + RealSense D435에서 다음을 확인했다.

- 실제 사람의 camera optical XYZ 계산 및 debug image 표시 정상
- 사람이 화면 왼쪽에 있을 때 `X<0`, 오른쪽에 있을 때 `X>0`
- Stage 2 distance 표시와 Stage 3 XYZ의 Z가 정상적으로 일치
- 여러 사람 동시 검출 시 각 사람의 XYZ가 독립적으로 계산됨
- `/leader/survivor/camera_positions`에 다중 pose가 정상 발행됨
- `header.frame_id=camera_color_optical_frame`
- PoseArray XYZ와 같은 frame의 debug image XYZ가 대응됨
- 모든 pose의 orientation이 identity quaternion(`w=1`)
- `person1..N`이 frame-local left-to-right 번호이며 tracking ID가 아님

실제 좌표 수치는 별도로 보존하지 않았으므로 이 문서는 검증된 부호·일치·발행 동작만
기록하며 절대 위치 정확도를 과도하게 주장하지 않는다.

## 3. 실제 geometry 조사 결과

2026-09-12에 현재 D435 camera launch를 실행해 확인한 값은 다음과 같다.

| 항목 | 실제 확인값 |
| --- | --- |
| rectified RGB | `640×480`, `rgb8`, `camera_color_optical_frame` |
| aligned depth | `640×480`, `16UC1`, `camera_color_optical_frame` |
| RGB CameraInfo | `640×480`, `camera_color_optical_frame` |
| distortion model | `plumb_bob` |
| D | `[0, 0, 0, 0, 0]` |
| R | identity |
| K | `[603.71155, 0, 313.71130, 0, 603.83417, 235.27989, 0, 0, 1]` |
| P | `[603.71155, 0, 313.71130, 0, 0, 603.83417, 235.27989, 0, 0, 0, 1, 0]` |

이 실행에서는 rectified RGB와 color-aligned depth가 같은 해상도와 optical frame을
공유하며 K와 P의 핵심 intrinsics도 같았다. 그러나 코드는 이 관찰을 영구 가정하지 않는다.
매 RGB/depth frame에서 해상도와 frame ID를 검사하고, CameraInfo도 해상도·frame ID·P
유효성을 검사한다. 불일치하면 같은 pixel index를 억지로 사용하지 않고 distance 또는
XYZ를 `N/A`로 처리한다.

## 4. pixel 좌표와 대표점

`u`는 영상의 수평 좌표이며 왼쪽에서 오른쪽으로 증가한다. `v`는 수직 좌표이며 위에서
아래로 증가한다. `640×480` 영상의 유효 범위는 일반적으로 `u=0..639`, `v=0..479`다.
이 값은 아직 meter 좌표가 아니다.

Stage 2 ROI는 half-open bounds `(left, top, right, bottom)`을 사용한다. 대표 pixel은 실제
ROI pixel 집합의 중심으로 정의한다.

```text
u = (left + right - 1) / 2
v = (top + bottom - 1) / 2
```

중앙 ROI이므로 bbox 중심과 같은 방향의 대표점이지만, clamp와 integer rounding 때문에
sub-pixel 수준에서는 bbox의 실수 중심과 다를 수 있다. 위 정의는 ROI에서 실제 사용한
pixel 범위와 정확히 대응한다.

대표 Z는 같은 ROI에서 zero, NaN, Inf, 범위 밖 값을 제거한 뒤 계산한 median depth[m]다.
median은 한 개의 `(u,v)` pixel 값이 아니라 ROI의 강건한 대표 거리다.

## 5. K와 P 선택

ROS `CameraInfo.K`는 raw image의 intrinsic matrix이고 `P`는 rectified image를 위한
projection matrix다. person detector 입력은 `/leader/camera/color/image_rect`이므로 Stage
3는 다음 값을 사용한다.

```text
fx = P[0]
fy = P[5]
cx = P[2]
cy = P[6]
```

현재 장비에서는 K와 P 값이 같지만, 그것이 P 대신 K를 사용할 근거는 아니다. 입력 image가
rectified라는 인터페이스 의미를 기준으로 P를 선택했다. 계산은 단순 pinhole deprojection
뿐이어서 `image_geometry/PinholeCameraModel` dependency를 추가하지 않고 ROS 독립 순수
함수로 구현했다. 향후 binning, ROI 또는 stereo projection의 추가 처리가 필요해지면
`image_geometry` 도입을 다시 검토한다.

## 6. deprojection과 optical frame

유효한 `(u,v,Z)`와 rectified intrinsics로 다음을 계산한다.

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = Stage 2 median depth
```

ROS camera optical frame의 축은 다음과 같다.

- `+X`: 영상 오른쪽
- `+Y`: 영상 아래
- `+Z`: 카메라 전방

따라서 중앙에서는 `X≈0`, 왼쪽에서는 `X<0`, 오른쪽에서는 `X>0`이어야 한다. principal
point 위에서는 `Y<0`, 아래에서는 `Y>0`이며 사람이 가까워지면 Z가 감소한다.

## 7. source 구조와 역할

- `detection_logic.py`: person 필터·좌우 정렬과 distance/XYZ debug overlay
- `depth_logic.py`: 중앙 ROI, 단위 변환, invalid filtering과 median Z
- `geometry_logic.py`: P 검증, ROI 중심, pinhole deprojection 순수 함수
- `person_detector_node.py`: ROS subscriptions, caches, YOLO 호출, 단계별 graceful
  degradation, debug image 및 `PoseArray` 발행
- `config/person_detector.yaml`: Stage 1~3 기본 파라미터
- `person_detector.launch.py`: 모든 파라미터를 launch argument로 노출

CameraInfo callback은 최신 유효 P와 frame ID만 cache한다. 매 RGB callback에서는 image
resolution과 cache resolution, RGB frame과 CameraInfo frame을 다시 비교한다. deprojection은
사람당 사칙연산 몇 번뿐이며 큰 image array를 추가로 복사하지 않는다.

## 8. invalid 처리와 Stage 1/2 보호

다음 경우 해당 사람의 XYZ를 만들지 않는다.

- CameraInfo가 아직 없거나 frame ID가 비어 있음
- P 길이, `fx`, `fy`, `cx`, `cy`가 유효하지 않음
- CameraInfo와 RGB 해상도 또는 frame ID가 다름
- Stage 2 distance가 `N/A`, non-finite 또는 `Z<=0`
- ROI가 비었거나 대표 `(u,v)`가 image 밖임
- 계산 결과가 non-finite

실패를 `(0,0,0)`으로 표현하지 않는다. 원점은 실제 측정처럼 해석될 수 있기 때문이다.
debug image에는 기존 bbox/confidence/distance를 유지하고 둘째 줄만 `XYZ N/A`로 표시한다.
topic에는 invalid placeholder를 넣지 않는다. 반복 warning은 5초 throttle을 사용한다.

## 9. debug image와 다중 사람 처리

Stage 1의 왼쪽→오른쪽 `person1..N` 정렬을 먼저 수행하고, 각 detection의 기존 ROI와 median
Z로 XYZ를 독립 계산한다.

```text
person1 0.83 | 2.20 m
XYZ (-0.45, 0.10, 2.20) m
```

텍스트는 bbox 위 공간이 부족하면 아래로 옮기고 image 경계 안으로 clamp한다.
`show_camera_xyz:=false`는 둘째 줄만 숨기며 `PoseArray` 발행은 중단하지 않는다.

## 10. PoseArray interface

기본 출력은 `/leader/survivor/camera_positions`의
`geometry_msgs/msg/PoseArray`다.

- `header.stamp`: 해당 detection을 만든 원본 RGB timestamp
- `header.frame_id`: 검증된 원본 RGB optical frame
- `position`: meter 단위 camera optical XYZ
- `orientation`: 의미 없는 zero quaternion 대신 identity `(0,0,0,1)`
- `poses`: valid XYZ만, frame-local left-to-right detection 순서

예를 들어 person2의 XYZ만 invalid이면 `poses[0]=person1`, `poses[1]=person3`이 된다. 따라서
PoseArray index를 debug image의 person 번호나 persistent ID로 해석하면 안 된다. Stage 3는
간단한 표준 message와 Stage 4의 일괄 TF 변환 가능성을 우선해 이 정책을 선택했다.
placeholder `(0,0,0)`이나 별도 debug index topic은 만들지 않았다.

검토한 mapping 대안은 다음과 같다.

| 대안 | 판단 |
| --- | --- |
| A. PoseArray에 valid pose만 발행 | **채택**. 표준 message, 안전한 invalid 생략, Stage 4 일괄 TF 변환에 충분하다. |
| B. person index debug topic 추가 | 보류. 두 topic의 동기화 계약만 늘고 persistent ID를 제공하지 못한다. |
| C. 기존 standard detection interface 재사용 | 저장소에 bbox/confidence/index/3D를 함께 담는 적절한 기존 interface가 없다. |
| D. custom survivor message 추가 | Stage 5~7에서 tracking/confirmation/mission 필드가 정해질 때 재검토한다. |

향후 mission interface가 confidence, bbox, frame-local index, tracking ID 또는 validity reason을
동시에 요구하면 custom survivor detection message를 Stage 5~7 interface 설계와 함께
도입한다. 지금 custom message를 먼저 고정하면 tracking과 duplicate suppression 설계가
결정되기 전에 잘못된 계약을 만들 가능성이 있다.

## 11. QoS와 timestamp 근거

- RGB/aligned depth/debug image: sensor stream에 맞춘 best-effort, volatile, keep-last 1
- CameraInfo: 기본 `/leader/camera/color/camera_info` RealSense publisher와 맞춘 reliable,
  volatile, keep-last 1
- camera positions: downstream 변환 노드가 안정적으로 받도록 reliable, volatile,
  keep-last 1

원본 RGB timestamp는 depth 선택과 detection 시점을 나타내며 Stage 4 TF lookup의 기준이 된다.
현재 wall time으로 덮어쓰지 않는다. frame ID도 parameter로 만든 임의 문자열이 아니라 실제
RGB와 CameraInfo가 일치할 때의 optical frame을 사용한다.

## 12. 고려했지만 사용하지 않은 대안

- K 사용: raw image 기준이므로 rectified RGB 입력에는 P가 더 명확하다.
- `image_geometry`: 현재 수식에는 dependency 대비 이점이 작다.
- synchronized CameraInfo: intrinsics는 거의 정적이므로 최신 유효값 cache가 충분하다.
- invalid pose를 zero로 유지: 실제 원점과 구분할 수 없어 제외했다.
- custom detection message: Stage 4 전에는 요구 필드와 ID 의미가 확정되지 않았다.
- temporal smoothing: bbox와 depth의 시간 변화 특성을 바꾸므로 Stage 6 범위로 남겼다.

초기 구현에서는 별도 `/leader/camera/color/camera_info_transient` bridge의 QoS를 따라
transient-local subscriber를 사용했지만, 기본 입력 topic의 실제 publisher는 reliable +
volatile이었다. 실제 ROS graph에서 durability incompatibility warning을 확인한 뒤 기본 topic
계약에 맞게 volatile로 수정했다. CameraInfo는 계속 발행되므로 startup 이후 즉시 cache된다.

## 13. Known limitations

- ROI 중심은 사람의 물리적 중심이나 질량 중심이 아니다.
- median Z와 대표 `(u,v)`는 동일한 단일 depth pixel 측정이 아니다.
- bbox와 depth noise 때문에 XYZ가 frame마다 흔들릴 수 있다.
- `personN`은 persistent ID가 아니며 교차·누락 시 번호가 바뀔 수 있다.
- COCO person detector는 사람처럼 보이는 의류·마네킹을 오검출할 수 있다.
- PoseArray만으로는 confidence와 원래 person 번호를 복원할 수 없다.
- temporal smoothing, confirmation과 duplicate suppression은 아직 없다.

## 14. Stage 4로 전달하는 계약

다음 개발 단계인 Stage 4는 `/leader/survivor/camera_positions`를 입력으로 받아 각 pose를
`header.stamp` 시점에 `header.frame_id`에서 `map`으로 exact-timestamp TF2 변환한다. Stage 3가 보장하는
계약은 metric XYZ, 유효한 optical frame ID, 원본 RGB timestamp, identity orientation과
valid-only ordering이다. Stage 4는 원본 timestamp의 TF가 없을 때 최신 TF를 몰래 사용하거나
stale map position을 발행하면 안 된다.
