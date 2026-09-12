# 생존자 인식 전체 개발 계획

## 상태와 최종 흐름

현재 상태는 **Stage 1 VERIFIED (2026-09-11)** 및
**Stage 2·3 IMPLEMENTED - HARDWARE VERIFICATION REQUIRED (2026-09-12)**다. 실제 D435
화면에서 Stage 1의 1명, 2명, 3명 검출을 확인했고 Stage 2 거리 표시도 동작했다. Stage 3
코드·자동시험과 실제 RGB/depth/CameraInfo geometry 조사를 마쳤지만, 정식 줄자 거리표와
camera XYZ 축 방향·다중 사람 시험 기록은 남아 있다.

```text
D435 RGB → YOLO person bbox ─┐
D435 aligned depth ──────────┼→ distance → camera XYZ → map XYZ
CameraInfo ──────────────────┘              → marker/중복 제거
                                             → confirmation
                                             → Mission Coordinator
```

## Stage 1 — RGB YOLO person bounding box

- Purpose: COCO pretrained YOLO11n으로 한 프레임의 모든 사람을 시각화한다.
- Input: `/leader/camera/color/image_rect`, `sensor_msgs/Image`.
- Processing: BGR 변환, class 0/threshold 필터, 중심 x 좌우 정렬, frame-local 번호와 box 표시.
- Output: `/leader/survivor/debug_image`, 원본 header 유지.
- Completion: 0/1/다중 person 결과를 rqt에서 확인하고 비-person이 표시되지 않는다.
- Current: software/build/test, survivor 전용 Docker의 Jetson GPU 추론, 다중-person sample 및
  D435 실제 1명/2명/3명 검출을 확인했다. bounding box, confidence, 좌→우 frame-local
  numbering과 `/leader/survivor/debug_image`를 rqt에서 검증했다. Stage 2 이후도 같은
  image와 model cache를 재사용한다.

## Stage 2 — Bounding box + aligned depth → distance

**현재 상태: IMPLEMENTED - HARDWARE VERIFICATION REQUIRED**

- Purpose: 각 검출 사람까지의 강건한 대표 거리를 계산한다.
- Input: Stage 1 bbox, `/leader/camera/aligned_depth_to_color/image_raw`, RGB/depth stamp.
- Processing: 최근 aligned depth cache에서 RGB와 가장 가까운 stamp를 찾아 `sync_slop_sec` 이내인지 검사하고,
  bbox 중심 ROI에서 0/non-finite/range 밖 값을 제거한 뒤 median을 계산한다.
- Output: frame-local person별 거리[m]를 debug image에 표시한다. depth가 없거나 불일치하면
  YOLO 결과를 유지하고 `N/A`를 표시한다.
- Completion: 자동 테스트, Docker 실행, 실제 D435의 알려진 거리·다중 인원·이동 시험이 모두
  기록되어야 한다. synthetic test만으로 VERIFIED로 승격하지 않는다.

## Stage 3 — Depth + CameraInfo → camera XYZ

**현재 상태: IMPLEMENTED - HARDWARE VERIFICATION REQUIRED**

- Purpose: 2D pixel과 depth를 camera optical frame의 3D 위치로 변환한다.
- Input: Stage 2 중앙 ROI center `(u,v)`, median Z, rectified RGB CameraInfo.P.
- Processing: P의 `fx/fy/cx/cy` 검증, RGB/CameraInfo resolution·frame 일치 검사,
  pinhole deprojection과 finite/positive 검사.
- Output: debug image의 XYZ와 `/leader/survivor/camera_positions` `PoseArray`. 원본 RGB
  timestamp와 실제 optical frame을 유지하고 valid XYZ만 좌→우 순서로 포함한다.
- Dependency: Stage 1 frame-local detections, Stage 2 ROI/median Z, color CameraInfo.
- Completion: unit test와 Docker runtime에 더해 실제 D435 중앙 `X≈0`, 좌/우 X 부호,
  상/하 Y 부호, near/far Z, 다중 person과 positions topic을 기록해야 한다.

## Stage 4 — Camera XYZ + TF2 → map XYZ

**현재 상태: NOT IMPLEMENTED**

- Purpose: 측정 시점의 생존자 후보를 전역 map 좌표로 변환한다.
- Input: stamped camera point, camera↔base 및 map TF.
- Processing: 원본 timestamp의 TF2 lookup/transform, timeout과 unavailable 처리.
- Output: `map` frame의 stamped survivor 후보 좌표.
- Dependency: Stage 3 PoseArray의 correct frame ID와 original RGB timestamp, 유효 TF tree.
- Completion: TF가 있을 때 좌표가 일관되고 누락/지연 TF에서 stale 좌표를 발행하지 않는다.

## Stage 5 — RViz marker + duplicate suppression

**현재 상태: NOT IMPLEMENTED**

- Purpose: map에서 후보를 표시하고 같은 생존자의 반복 관측을 병합한다.
- Input: map-frame 후보 좌표와 confidence/관측 metadata.
- Processing: 공간 gate 기반 association, 관측 누적, marker lifetime 관리.
- Output: RViz Marker와 중복이 제거된 survivor candidate 목록.
- Dependency: Stage 4 map XYZ와 association 정책.
- Completion: 정지 생존자의 반복 관측이 한 후보로 유지되고 떨어진 사람은 분리된다.

## Stage 6 — Confirmation and stabilization

**현재 상태: NOT IMPLEMENTED**

- Purpose: 일시적 오검출을 줄이고 안정된 후보만 confirmed 상태로 승격한다.
- Input: 시간에 따른 survivor candidates.
- Processing: 최소 관측 수/시간, timeout, confidence 및 위치 안정성 조건.
- Output: confirmed/lost 상태와 안정화된 위치.
- Dependency: Stage 5 candidate identity/association 결과.
- Completion: 단발 검출은 확정되지 않고 지속 관측은 정의된 지연 내 확정된다.

## Stage 7 — Mission Coordinator integration

**현재 상태: NOT IMPLEMENTED**

- Purpose: confirmed survivor를 로봇 임무 판단과 구호물품 전달 흐름에 연결한다.
- Input: confirmed survivor 위치/ID/상태와 coordinator 상태.
- Processing: 인터페이스 계약, 중복 임무 방지, 취소·복구와 안전 gate.
- Output: coordinator가 소비할 survivor event/target.
- Dependency: Stage 6 confirmed interface와 mission state contract.
- Completion: simulation과 실기에서 한 후보가 한 번만 임무로 전환되고 실패를 복구한다.

## Stage 1에서 Stage 2로 전달된 계약

- bbox coordinates: color image pixel 좌표
- RGB timestamp/frame: 원본 `header.stamp`, `camera_color_optical_frame`
- aligned depth: `/leader/camera/aligned_depth_to_color/image_raw`, 실제 확인값 `16UC1`,
  640×480, `camera_color_optical_frame`
- raw depth 참고: `/leader/camera/depth/image_rect_raw`, `16UC1`, 640×480,
  `camera_depth_optical_frame`
- CameraInfo: `/leader/camera/color/camera_info`
- 실제 resolution, encoding과 timestamp 일치는 매 실행에서 재검증한다.

## Stage 2에서 Stage 3로 전달된 계약과 구현

- representative pixel: Stage 2 half-open central ROI의 실제 pixel 중심
- representative Z: 같은 ROI의 valid median distance[m]
- rectified intrinsics: `/leader/camera/color/camera_info`의 P 사용
- 실제 확인 geometry: RGB/depth/CameraInfo `640×480`, `camera_color_optical_frame`
- optical axes: right/down/forward
- invalid policy: detection/distance 유지, `XYZ N/A`, PoseArray에는 placeholder 미삽입

Stage 2는 `depth_logic.py`, Stage 3는 `geometry_logic.py`에 ROS-independent 순수 함수를 둔다.
`person_detector_node.py`가 depth cache, CameraInfo cache, frame별 계산과 ROS message 발행을
담당한다. 자세한 결정 근거는 [Stage 3 구현 문서](STAGE3_CAMERA_XYZ_IMPLEMENTATION.md), 실제
절차는 [Stage 3 검증 가이드](STAGE3_CAMERA_XYZ_VALIDATION.md)를 따른다.

## Stage 3에서 Stage 4로 전달할 계약

- topic: `/leader/survivor/camera_positions`
- type: `geometry_msgs/msg/PoseArray`
- header stamp: original RGB measurement time
- header frame: 검증된 RGB camera optical frame
- position: meter 단위 camera optical XYZ
- orientation: identity quaternion
- order: valid frame-local detections의 left-to-right 순서, persistent ID 아님

Stage 4는 이 header를 사용해 측정 시점 TF2 변환을 수행한다. map TF가 없거나 timestamp에
맞는 transform이 없을 때 최신 transform이나 fake origin으로 대체하지 않는다.
