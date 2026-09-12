# Stage 2 RGB-aligned Depth 거리 검증 가이드

## 상태

현재 상태: **IMPLEMENTED - HARDWARE VERIFICATION REQUIRED**

Stage 1의 실제 D435 person detection은 `VERIFIED` 상태로 유지된다. Stage 2는 코드와
자동 테스트만으로 `VERIFIED`가 되지 않으며, 아래의 줄자 기준 거리·다중 사람·이동 시험을
실제 D435에서 기록한 뒤에만 상태를 변경한다.

## 목적과 구조

```text
RGB /leader/camera/color/image_rect ──→ YOLO person bbox ─┐
                                                          ├→ center ROI → filter → median → m
aligned depth /leader/camera/aligned_depth_to_color/image_raw ─┘
```

RGB callback은 depth가 없어도 계속 동작한다. aligned depth callback은 최근 message를
`sync_queue_size`개 보관하고, RGB timestamp에 가장 가까운 depth와의 차이가 `sync_slop_sec`
이내일 때만 사용한다.

## 입력과 단위

| 데이터 | Topic | 예상 검증값 |
|---|---|---|
| RGB | `/leader/camera/color/image_rect` | `rgb8`, 640×480 |
| aligned depth | `/leader/camera/aligned_depth_to_color/image_raw` | `16UC1`, 640×480 |
| raw depth 참고 | `/leader/camera/depth/image_rect_raw` | `16UC1` |
| RGB CameraInfo (Stage 3용) | `/leader/camera/color/camera_info` | `CameraInfo` |

실행 시 encoding, width/height, step, frame ID, timestamp와 raw 값을 반드시 기록한다.
정수 depth는 `depth_scale_m_per_unit`을 곱하고 기본값은 `0.001 m/unit`이다. 이 값은
parameter이며 실제 D435 raw sample과 측정 거리로 확인하기 전에는 hardware calibration으로
간주하지 않는다. `32FC1`은 meter 값으로 처리한다.

## 알고리즘

- bbox 중심의 `depth_roi_width_ratio=0.25`, `depth_roi_height_ratio=0.25` ROI를 사용한다.
- ROI는 depth image 경계에 clamp하고 빈 ROI는 invalid로 처리한다.
- 변환된 meter 값에서 zero, NaN, Inf, `min_depth_m` 미만, `max_depth_m` 초과를 제거한다.
- 유효 pixel 수가 `min_valid_depth_pixels` 미만이면 `N/A`다.
- 남은 값의 NumPy median을 사람 거리로 표시한다.
- 사람 번호는 Stage 1처럼 현재 frame에서만 왼쪽→오른쪽으로 부여한다.

## Parameters

| Parameter | 기본값 | 설명 |
|---|---:|---|
| `aligned_depth_topic` | aligned depth topic | RGB 정렬 depth |
| `depth_roi_width_ratio` / `height_ratio` | `0.25` / `0.25` | 중심 ROI 비율 |
| `min_depth_m` / `max_depth_m` | `0.2` / `6.0` | 유효 거리 범위 |
| `min_valid_depth_pixels` | `20` | 최소 유효 pixel |
| `depth_scale_m_per_unit` | `0.001` | `16UC1` raw 변환 |
| `show_depth_roi` | `false` | debug ROI 표시 |
| `sync_queue_size` | `5` | 최근 depth cache 크기 |
| `sync_slop_sec` | `0.12` | timestamp 허용 차이 |

## Build와 실행

### Terminal 1 — D435 camera

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_depth:=true enable_sync:=true align_depth.enable:=true
```

### Terminal 2 — survivor Docker detector

```bash
cd ~/damgc_robot
./scripts/run_survivor_detector.sh
```

runtime image가 없거나 코드가 변경된 경우에만 `./scripts/build_survivor_runtime.sh`를 먼저
실행한다.

### Terminal 3 — topic/metadata

```bash
source /opt/ros/humble/setup.bash
ros2 topic info -v /leader/camera/color/image_rect
ros2 topic info -v /leader/camera/aligned_depth_to_color/image_raw
ros2 topic echo --once /leader/camera/color/image_rect --field header
ros2 topic echo --once /leader/camera/color/image_rect --field encoding
ros2 topic echo --once /leader/camera/aligned_depth_to_color/image_raw --field header
ros2 topic echo --once /leader/camera/aligned_depth_to_color/image_raw --field encoding
ros2 topic echo --once /leader/camera/aligned_depth_to_color/image_raw --field height
ros2 topic echo --once /leader/camera/aligned_depth_to_color/image_raw --field width
ros2 topic hz /leader/camera/color/image_rect
ros2 topic hz /leader/camera/aligned_depth_to_color/image_raw
ros2 topic hz /leader/survivor/debug_image
```

### Terminal 4 — debug image

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /leader/survivor/debug_image
```

정상 출력 예: `person1 0.94 | 0.92 m`; depth가 없거나 invalid이면 `person1 0.94 | N/A`다.

## Hardware verification

### 단일 사람

| 기준 거리 | 표시 거리 | valid pixels | encoding | raw sample/scale | 결과/비고 |
|---:|---:|---:|---|---|---|
| 0.5 m |  |  |  |  |  |
| 1.0 m |  |  |  |  |  |
| 1.5 m |  |  |  |  |  |
| 2.0 m |  |  |  |  |  |
| 3.0 m |  |  |  |  |  |

### 다중 사람

가까운 사람부터 먼 사람을 배치하고 실제 거리와 표시값을 기록한다.

| 표시 번호 | 기준 거리 | 표시 거리 | valid pixels | 결과/비고 |
|---|---:|---:|---:|---|
| person1 |  |  |  |  |
| person2 |  |  |  |  |
| person3 |  |  |  |  |

가까운 순서로 배치했다면 `person1 distance < person2 distance < person3 distance`가
기대 결과다.

### 이동

한 사람이 약 3 m → 2 m → 1 m로 접근한 뒤 후퇴한다. 접근 시 distance가 전반적으로
감소하고 후퇴 시 증가해야 한다. Stage 2에는 temporal smoothing을 추가하지 않는다.

## Troubleshooting

| 증상 | 확인 명령/방법 | 원인 후보 |
|---|---|---|
| aligned topic 없음 | `ros2 topic list \| grep aligned` | camera depth/align 설정 또는 topic 이름 |
| RGB만 수신 | `ros2 topic info -v <topic>` | depth 장치, QoS, Docker domain |
| resolution mismatch | 각 Image의 width/height echo | RGB/depth profile 불일치 |
| timestamp mismatch | detector warning/header 비교 | sync, backlog, `sync_slop_sec` |
| distance 항상 N/A | encoding·valid pixel·ROI 확인 | scale/range/depth hole |
| distance 항상 0 | raw sample 확인 | zero depth 또는 잘못된 ROI |
| 1000배 차이 | raw value와 scale 비교 | mm↔m scale 오류 |
| 거리 튐 | `show_depth_roi:=true` | edge/background/반사/holes |
| 일부 사람만 거리 있음 | 각 ROI valid pixel 비교 | 가림, 사람 크기, depth hole |
| Docker에서 topic 없음 | `echo $ROS_DOMAIN_ID`, `ros2 topic list` | host network/domain/QoS |
| FPS 하락 | input/debug `topic hz`, GPU 상태 | YOLO 부하, CPU fallback, contention |

## Hardware verification 결과

시험 일시: 

장비/firmware: 

실제 encoding/resolution/frame ID: 

검증한 depth scale 근거: 

결과 및 실패 조건: 

## Next Stage

Stage 3에서 bbox center `(u, v)`, depth Z, RGB CameraInfo의 `fx/fy/cx/cy`를 이용해 camera
optical frame XYZ를 계산한다. Stage 2에서는 deprojection, TF2, map 좌표와 marker를 구현하지
않는다.
