# rescue_robot_survivor

## 개요와 현재 상태

`rescue_robot_survivor`는 Leader의 Intel RealSense D435 RGB 영상에서 생존자 후보인
사람을 검출하는 독립 ROS 2 Humble 패키지다. AprilTag는 구호물품 인식과 접근을,
이 패키지는 생존자 인식을 담당하므로 모델 의존성, 실행 주기와 이후 depth 처리를 서로
섞지 않도록 별도 패키지로 분리했다.

**현재 상태: VERIFIED (2026-09-11)**

코드, package build와 자동 테스트를 완료했다. survivor 전용 Jetson Docker GPU runtime에서
YOLO11n 추론을 수행하고 실제 D435 화면에서 1명, 2명, 3명 검출을 확인했다. 각 사람의
bounding box와 confidence, 왼쪽에서 오른쪽 순서의 `person1..N` 표시 및
`/leader/survivor/debug_image`를 rqt_image_view에서 확인했다. `personN`은 persistent
tracking ID가 아닌 현재 프레임의 표시 번호다.

Stage 2 거리 계산 코드는 구현되었지만 실제 줄자 기준의 D435 거리 검증 전이므로 현재 상태는
`IMPLEMENTED - HARDWARE VERIFICATION REQUIRED`이다. Stage 1의 `VERIFIED` 판정은 그대로
유지된다.

## Jetson GPU runtime

Host global Python에는 AI package를 설치하지 않는다. 재현 가능한 survivor 전용 image를
사용한다.

```bash
cd ~/damgc_robot
./scripts/build_survivor_runtime.sh
./scripts/run_survivor_detector.sh
```

Runtime smoke test:

```bash
docker run --rm --runtime=nvidia --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -v ~/.cache/damgc-survivor-ultralytics:/root/.cache/ultralytics \
  damgc-survivor-yolo:humble 'check_survivor_ai_runtime --static'
```

검증된 조합은 JetPack 6.2.3/L4T R36.5.2, Python 3.10, NVIDIA CUDA PyTorch
`2.5.0a0+872d972e41.nv24.08`, CUDA-enabled source-built torchvision `0.20.0`,
Ultralytics `8.4.147`이다. torchvision은 torch를 교체하지 않고 `FORCE_CUDA=1` 및
Orin `sm_87` 대상으로 build한다. check script는 CUDA NMS와 YOLO11n GPU inference까지
확인한다.

## Stage 1 기능과 범위

현재 구현은 다음 기능만 지원한다.

- `/leader/camera/color/image_rect`를 sensor-data QoS로 구독
- Ultralytics `yolo11n.pt` COCO pretrained detection 사용
- COCO class 0인 `person`만 confidence threshold로 필터링
- 한 프레임의 모든 사람을 중심 x 좌표 기준 왼쪽에서 오른쪽으로 정렬
- `person1 0.94 | 1.24 m` 형식의 번호, confidence, bounding box와 거리 표시
- 원본 `header.stamp`와 `header.frame_id`를 유지한 debug image 발행
- CUDA 사용 가능 시 GPU 선택, 불가능하면 CPU 선택(`device=auto`)

`person1`, `person2`는 **현재 프레임의 표시용 번호**다. tracking ID가 아니며 사람이
움직이거나 서로 교차하거나 검출이 누락되면 다음 프레임에서 번호가 달라질 수 있다.
ByteTrack, BoT-SORT, DeepSORT, re-identification과 persistent ID는 구현하지 않는다.

이번 Stage에는 사람 거리, bounding box depth median, Camera XYZ, TF2 map 변환, RViz
위치 marker, 중복 제거, survivor confirmation, Mission Coordinator, custom training과
TensorRT 최적화가 포함되지 않는다.

RGB, depth, aligned-depth를 포함한 D435 camera pipeline도 정상 기동 및 topic 발행을
확인했다. Stage 2는 YOLO bounding box와 aligned depth를 결합해 중심 ROI의 유효 depth만
추출하고, zero/NaN/Inf/range 밖 값을 제거한 median depth로 사람까지의 거리[m]를 계산한다.
aligned depth가 없거나 RGB와 timestamp 차이가 `sync_slop_sec`보다 크면 detection은 계속
발행하고 거리만 `N/A`로 표시한다.

## 구조

```text
rescue_robot_survivor/
├── rescue_robot_survivor/
│   ├── detection_logic.py       # person 필터, 좌표 보정, 정렬, drawing
│   ├── depth_logic.py           # ROI, scale, filtering, median
│   └── person_detector_node.py  # ROS 구독, YOLO/depth 처리, debug 발행
├── launch/person_detector.launch.py
├── config/person_detector.yaml
├── test/                        # 순수 로직과 launch/config 계약 테스트
├── docs/SURVIVOR_DEVELOPMENT_PLAN.md
├── docs/STAGE1_PERSON_DETECTION_VALIDATION.md
├── package.xml
├── setup.py
└── setup.cfg
```

launch는 detector만 실행한다. RealSense camera를 시작하거나 기존 AprilTag launch를
include하지 않는다. YAML 값은 launch argument의 기본값으로 읽히므로 config 수정과 실행
시 override를 모두 지원한다.

## ROS 인터페이스와 파라미터

| 구분 | 기본값 | 타입/설명 |
| --- | --- | --- |
| 입력 | `/leader/camera/color/image_rect` | `sensor_msgs/msg/Image`, 확인값 `rgb8` |
| 출력 | `/leader/survivor/debug_image` | `sensor_msgs/msg/Image`, `bgr8` |
| `image_topic` | 위 입력 | 다른 RGB 토픽으로 변경 |
| `debug_image_topic` | 위 출력 | debug 영상 토픽 변경 |
| `model_name` | `yolo11n.pt` | 모델 이름 또는 로컬 weight 경로 |
| `confidence_threshold` | `0.5` | 0.0–1.0 |
| `device` | `auto` | `auto`, `cpu`, `0` 등 |
| `aligned_depth_topic` | `/leader/camera/aligned_depth_to_color/image_raw` | RGB 정렬 depth |
| `depth_roi_width_ratio` / `height_ratio` | `0.25` / `0.25` | bbox 중심 ROI 비율 |
| `min_depth_m` / `max_depth_m` | `0.2` / `6.0` | 유효 거리 범위 |
| `min_valid_depth_pixels` | `20` | 최소 유효 pixel 수 |
| `depth_scale_m_per_unit` | `0.001` | integer depth raw 단위 변환값 |
| `show_depth_roi` | `false` | debug ROI 표시 |
| `sync_queue_size` | `5` | 최근 depth cache 크기 |
| `sync_slop_sec` | `0.12` | RGB/depth 허용 timestamp 차이 |

입출력은 `BEST_EFFORT`, `VOLATILE`, `KEEP_LAST(1)`을 사용한다. 느린 추론 중 오래된
camera frame이 쌓이는 것을 줄이고 RealSense sensor publisher와 호환하기 위한 설정이다.

## Dependencies

ROS 의존성은 `rclpy`, `sensor_msgs`, `cv_bridge`, `python3-opencv`, `python3-numpy`,
`launch`, `launch_ros`다. 추론에는 별도로 PyTorch, torchvision과 Ultralytics가 필요하다.

Jetson에서는 일반 PyPI torch/torchvision으로 기존 NVIDIA build를 교체하면 안 된다.
다음 확인을 먼저 수행한다.

```bash
python3 --version
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.version.cuda)"
python3 -c "import torchvision; print(torchvision.__version__)"
python3 -c "import ultralytics; print(ultralytics.__version__)"
```

현재 조사 결과는 Python 3.10.12, JetPack 6.2.3/CUDA 12.6이며 세 AI 패키지는 모두
미설치다. JetPack 6.2.3에 맞는 NVIDIA PyTorch/torchvision 조합을 먼저 준비한 다음,
Ultralytics 설치가 이를 교체하지 않는지 확인해야 한다. 이 패키지의 build 과정은 AI
패키지를 자동 설치하지 않는다.

설치 기준은 [NVIDIA PyTorch for Jetson 설치 문서](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)와
[Ultralytics Jetson 가이드](https://docs.ultralytics.com/guides/nvidia-jetson/)다. 확인 시점의
Ultralytics native 예시는 JetPack 6.1용 조합까지만 명시하므로, 현재 장비의 JetPack
6.2.3에 그 wheel을 그대로 설치하지 않는다. 정확한 6.2.3 호환 조합을 확인하거나 격리된
JetPack 6 container를 선택한 뒤 사용자가 설치해야 한다.

`yolo11n.pt`가 로컬에 없으면 Ultralytics가 최초 실행 시 다운로드할 수 있다. Jetson이
오프라인이면 weight를 미리 복사하고 `model_name:=/absolute/path/yolo11n.pt`로 지정한다.

## Host camera와 container detector 실행

```bash
cd ~/damgc_robot
./scripts/build_survivor_runtime.sh
```

기존 camera launch는 host에서 먼저 실행한다. detector는 host global Python이 아니라 전용
container에서 실행한다.

```bash
# Host terminal
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_depth:=true enable_sync:=true align_depth.enable:=true

# Another terminal
cd ~/damgc_robot
./scripts/run_survivor_detector.sh
```

파라미터 override 예시는 다음과 같다.

```bash
./scripts/run_survivor_detector.sh \
  confidence_threshold:=0.6 device:=cpu
```

화면은 다음 중 하나로 확인한다.

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
# GUI에서 /leader/survivor/debug_image 선택

ros2 run rqt_image_view rqt_image_view /leader/survivor/debug_image
```

정상 상태에서는 사람이 없을 때 원본 영상만 보이고, 사람이 있으면 각 사람 주위에 초록색
box와 `personN confidence`가 표시된다. 각 터미널에서 `Ctrl-C`로 detector, rqt와 camera를
종료한다.

## Known limitations

- frame-local 번호이므로 프레임 간 ID가 유지되지 않는다.
- COCO pretrained 모델은 누운 사람, 심한 가림, 작은 사람 또는 마네킹을 놓칠 수 있다.
- 마네킹 실패 시 Stage 1에서 custom training을 시작하지 않고 검증 결과에 기록한다.
- GPU/CPU 성능, camera와 AprilTag의 동시 사용에 따라 debug FPS가 낮아질 수 있다.
- Stage 1 node는 aligned depth를 구독하지 않는다.
- host Python에는 AI runtime이 없다. detector는 항상 survivor 전용 container에서 실행하고,
  host는 camera, ROS graph 확인과 rqt에 사용한다.

## 빠른 문제 해결

- RGB 없음: `rs-enumerate-devices`, `ros2 topic list | grep camera`, camera launch 로그 확인.
- callback 없음/QoS 의심: `ros2 topic info -v /leader/camera/color/image_rect`에서 endpoint와
  `BEST_EFFORT` 확인.
- debug 없음: `ros2 node list`, `ros2 topic info -v /leader/survivor/debug_image`, detector
  terminal의 model/import 오류 확인.
- Ultralytics/model load 실패: AI 패키지 import와 weight 경로/네트워크를 확인하되 torch를
  임의 업그레이드하지 않는다.
- CUDA false: JetPack용 torch인지 확인하고 우선 `device:=cpu`로 기능을 검증한다.
- rqt blank: debug topic을 명시하고 rqt의 QoS/transport 및 `ros2 topic hz`를 확인한다.
- 낮은 FPS/일부 사람 누락: 입력·출력 hz, GPU 사용 여부, 조명·거리·가림과 threshold를
  확인한다.
- aligned depth 없음: camera를 `enable_sync:=true align_depth.enable:=true`로 재실행한다.

명령별 상세 진단은
[Stage 1 validation](docs/STAGE1_PERSON_DETECTION_VALIDATION.md), 이후 전체 개발 순서는
[survivor development plan](docs/SURVIVOR_DEVELOPMENT_PLAN.md)을 따른다.

## Stage 2 distance algorithm

RGB callback은 YOLO inference를 계속 수행하고, depth callback은 최근 aligned depth message를
`sync_queue_size`개 보관한다. RGB timestamp에 가장 가까운 depth와의 차이가 `sync_slop_sec`
이내일 때만 depth를 사용한다.
`16UC1`/`MONO16`은 `depth_scale_m_per_unit`을 곱하고 `32FC1`은 meter로 처리한다. bbox
중심 ROI에서 유효 pixel을 추출하고 NumPy median을 계산한다. 유효 pixel이 20개 미만이거나
depth가 없으면 `N/A`를 표시한다.

## Next Stage — Stage 3

YOLO bounding box + aligned depth를 timestamp 기준으로 결합하고, 사람 영역에서 유효
depth를 추출한 뒤 zero/invalid 값을 제거한다. 이후 median depth로 사람까지의 거리[m]를
계산한다. 필요한 전달 정보는 bbox 좌표, RGB/aligned-depth timestamp, aligned-depth 및
CameraInfo 토픽, resolution, frame ID와 encoding이다.
