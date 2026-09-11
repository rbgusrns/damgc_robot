# 생존자 인식 전체 개발 계획

## 상태와 최종 흐름

현재 상태는 **Stage 1 VERIFIED (2026-09-11)**다. 실제 D435 화면에서 1명, 2명, 3명
사람 검출과 bounding box, confidence, 좌→우 `person1..N` numbering을 확인했다.

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

- Purpose: 각 검출 사람까지의 강건한 대표 거리를 계산한다.
- Input: Stage 1 bbox, `/leader/camera/aligned_depth_to_color/image_raw`, RGB/depth stamp.
- Processing: timestamp 동기화, bbox 중심 주변 ROI, 0/non-finite/invalid 제거, median depth와
  유효 pixel 수 계산.
- Output: frame-local person별 거리[m]와 원본 bbox/stamp.
- Completion: 알려진 거리의 사람에 대해 유효 거리와 invalid-depth 처리가 재현된다.

## Stage 3 — Depth + CameraInfo → camera XYZ

- Purpose: 2D pixel과 depth를 camera optical frame의 3D 위치로 변환한다.
- Input: 대표 pixel, 거리, `/leader/camera/color/camera_info`의 K 행렬.
- Processing: pinhole deprojection과 입력 유효성/단위 검사.
- Output: `camera_color_optical_frame` 기준 X/Y/Z와 측정 timestamp.
- Completion: 합성 camera model과 실측점에서 축 방향 및 거리 오차가 확인된다.

## Stage 4 — Camera XYZ + TF2 → map XYZ

- Purpose: 측정 시점의 생존자 후보를 전역 map 좌표로 변환한다.
- Input: stamped camera point, camera↔base 및 map TF.
- Processing: 원본 timestamp의 TF2 lookup/transform, timeout과 unavailable 처리.
- Output: `map` frame의 stamped survivor 후보 좌표.
- Completion: TF가 있을 때 좌표가 일관되고 누락/지연 TF에서 stale 좌표를 발행하지 않는다.

## Stage 5 — RViz marker + duplicate suppression

- Purpose: map에서 후보를 표시하고 같은 생존자의 반복 관측을 병합한다.
- Input: map-frame 후보 좌표와 confidence/관측 metadata.
- Processing: 공간 gate 기반 association, 관측 누적, marker lifetime 관리.
- Output: RViz Marker와 중복이 제거된 survivor candidate 목록.
- Completion: 정지 생존자의 반복 관측이 한 후보로 유지되고 떨어진 사람은 분리된다.

## Stage 6 — Confirmation and stabilization

- Purpose: 일시적 오검출을 줄이고 안정된 후보만 confirmed 상태로 승격한다.
- Input: 시간에 따른 survivor candidates.
- Processing: 최소 관측 수/시간, timeout, confidence 및 위치 안정성 조건.
- Output: confirmed/lost 상태와 안정화된 위치.
- Completion: 단발 검출은 확정되지 않고 지속 관측은 정의된 지연 내 확정된다.

## Stage 7 — Mission Coordinator integration

- Purpose: confirmed survivor를 로봇 임무 판단과 구호물품 전달 흐름에 연결한다.
- Input: confirmed survivor 위치/ID/상태와 coordinator 상태.
- Processing: 인터페이스 계약, 중복 임무 방지, 취소·복구와 안전 gate.
- Output: coordinator가 소비할 survivor event/target.
- Completion: simulation과 실기에서 한 후보가 한 번만 임무로 전환되고 실패를 복구한다.

## Stage 1에서 Stage 2로 전달할 계약

- bbox coordinates: color image pixel 좌표
- RGB timestamp/frame: 원본 `header.stamp`, `camera_color_optical_frame`
- aligned depth: `/leader/camera/aligned_depth_to_color/image_raw`, `16UC1`, 640×480,
  `camera_color_optical_frame`
- raw depth 참고: `/leader/camera/depth/image_rect_raw`, `16UC1`, 640×480,
  `camera_depth_optical_frame`
- CameraInfo: `/leader/camera/color/camera_info`
- 실제 resolution, encoding과 timestamp 일치는 매 실행에서 재검증한다.

Stage 2의 다음 작업은 YOLO bounding box + aligned depth → 사람 영역의 유효 depth 추출
→ zero/invalid 제거 → median depth → person distance[m]이며, 이 문서의 뒤 단계를 동시에
구현하지 않는다.
