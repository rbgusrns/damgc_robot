# 독립형 Eye State 분류기

시스템 전체 구조, 현재 단계 판정과 실제 D435 시험 결과는 [Survivor Eye State 시스템 설계](../../docs/SURVIVOR_EYE_STATE_SYSTEM.md)를 참고한다. 단계별 실험 상세는 [개발 이력](../../docs/SURVIVOR_EYE_STATE_DEVELOPMENT.md)에 기록한다.

이 디렉터리는 눈 이미지 분류 및 standalone face/eye 검증 도구를 담는다. 분류 클래스는 `open`, `closed` 두 개다. ROS node는 별도 `rescue_robot_eye_state` package에 두며 기존 Survivor pipeline과 자동 연결하지 않는다.

## 2차 Goal 1: 얼굴·눈 ROI 독립 추출

`face_eye_roi.py`는 OpenCV `FaceDetectorYN`과 YuNet으로 얼굴 bounding box 및 5점 landmark를 얻고, subject 기준 좌우 눈 ROI를 계산한다. 얼굴 detector 호출은 `FaceEyeROIExtractor`가 담당하고, 검출 결과를 ROI로 변환하는 `faces_to_eye_rois`는 detector와 분리되어 단위 테스트할 수 있다. 여러 얼굴은 검출 결과별 항목으로 돌려주며 frame-local index 외의 추적은 하지 않는다.

### Backend와 모델 출처

- Backend: OpenCV DNN `FaceDetectorYN` + YuNet ONNX `face_detection_yunet_2022mar.onnx`. Jetson host와 검증된 Docker image의 OpenCV 4.5.4에서 실제 `detect()` 실행까지 확인했다. MediaPipe는 공식 Python 문서상 Linux aarch64 wheel이 제공되지 않고 직접 빌드가 필요해 이 단계에는 선택하지 않았다.
- 모델 원본: [OpenCV Zoo YuNet 디렉터리](https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet), [OpenCV Zoo README의 source 및 license](https://github.com/opencv/opencv_zoo/blob/main/models/face_detection_yunet/README.md).
- 고정 origin revision: `c97242ce7f2a554e288b50eabd9f5df957e78801`; 다운로드 URL은 `https://media.githubusercontent.com/media/opencv/opencv_zoo/c97242ce7f2a554e288b50eabd9f5df957e78801/models/face_detection_yunet/face_detection_yunet_2022mar.onnx`.
- Model version: `face_detection_yunet_2022mar.onnx`; 디렉터리 라이선스: MIT; 기대 SHA-256: `50ef07f702a31741ca46a4c0d947773b64143b9362780237bf0d427d6c79bab7`.
- 호환성 확인: OpenCV 4.5.4는 YuNet 2023 ONNX의 `detect()` 단계에서 `Layer with requested id=-1 not found` 오류가 났다. 동일 환경에서 고정한 2022 ONNX는 이미지 추론을 완료했다. production OpenCV/Ultralytics image는 수정하지 않았다.
- OpenCV Zoo 모델 바이너리는 Git에 추가하지 않는다. downloader는 기본적으로 `~/.cache/damgc-eye-state/`에 저장하고 checksum이 일치해야 사용한다.

모델 확보와 이미지 진단:

```bash
cd ~/damgc_robot/tools/eye_state
python3 fetch_yunet_model.py
python3 inspect_eye_rois.py --image /path/to/rgb.jpg --save-crops
```

디버그 사진과 valid crop은 기본 `eye_roi_debug/`에 기록된다. 다른 위치를 원하면 `--output-dir`를 지정한다. 실제 detector는 3채널 BGR 또는 RGB 배열을 받는다.

### ROI와 좌우 기준

OpenCV YuNet 행 순서는 `x, y, width, height, right eye x/y, left eye x/y, nose x/y, right/left mouth x/y, face confidence`다. `left_eye`와 `right_eye`는 **얼굴 주체 기준 해부학적 좌우**다. 정면 얼굴에서는 subject-left eye landmark가 보통 영상의 오른쪽에 나타난다. ([OpenCV output layout](https://github.com/opencv/opencv/blob/5.x/doc/tutorials/dnn/dnn_face/dnn_face.markdown), [OpenCV Zoo demo landmark ordering](https://github.com/opencv/opencv_zoo/blob/main/models/face_detection_yunet/demo.py))

각 눈 중심은 해당 semantic landmark이며 두 눈 사이의 유클리드 거리 `d`로 ROI 한 변을 계산한다. 기본은 `round(0.85 × d)`인 정사각 crop이다. 이미지 경계를 넘으면 box를 clamp하고, 잘린 비율이 25% 이하일 때만 edge replication으로 요청 square를 복원한다. 잘린 뒤 실제 crop 폭/높이의 최소값은 각각 24 px다. 눈 landmark가 유효하지 않거나, inter-eye 거리가 1 px 이하 또는 얼굴 크기에 비해 비정상적이면 두 ROI 모두 invalid 처리한다. Face confidence 기본 threshold는 0.60이다.

`VALID` 조건은 confidence 통과, 유효한 두 eye landmark, inter-eye geometry 정상, ROI가 비어 있지 않고 최소 폭/높이를 만족하며 severe clipping이 없는 경우다. 그 외에는 `INVALID` 및 `invalid_landmark`, `face_confidence_below_threshold`, `roi_below_minimum_size`, `severe_frame_clipping` 등 이유를 반환한다. Blur metric은 아직 적용하지 않았다. 24 px 및 0.85 비율은 초기값이며 D435 거리별 crop을 확인한 뒤 조정해야 한다.

### Goal 1 검증 기록

- Python unittest의 synthetic YuNet landmark 행으로 no-face, 다중 얼굴 구조, subject 좌우 순서, 비례 정사각 crop, 낮은 confidence, 잘못된/NaN landmark, 작은 ROI, 경계 clipping, 입력 행 형식 및 parameter 검사를 수행한다.
- 로컬의 D435 color frame에는 얼굴이 없어 검출 결과가 없는 경우 `NO_FACE` 처리를 확인했다. YuNet 2022 model load 및 `detect()` 호출은 OpenCV 4.5.4에서 통과했다. 얼굴을 포함한 정면/회전 이미지가 없어 face landmark와 crop의 실제 정확도 검증은 **NOT RUN**이다.
- Goal 1에서는 classifier checkpoint를 로드하지 않고 ROS2 node/topic도 만들지 않는다.

## 2차 Goal 2: standalone classifier 연결

`infer_eye_state_image.py`는 한 장의 RGB 사진에서 YuNet face/landmark 검출, 좌우 ROI 검사, valid crop 분류, debug image 기록을 차례로 수행한다. classifier는 process 안에서 한 번만 로드하고 valid eye crop만 한 batch로 추론한다. invalid ROI는 classifier에 전달하지 않으며 얼굴이 없으면 `NO_FACE`를 출력한다. 좌우 결과는 각각 독립적으로 출력하고 결합하지 않는다. debug image에는 face box, left/right eye ROI box, state와 confidence 또는 invalid 사유가 표시된다.

실행 예시는 검증된 Docker runtime에서 수행한다. 저장소는 읽기 전용이고, 입력 사진과 두 checkpoint는 읽기 전용 mount다. debug 결과만 `eye_roi_debug/`에 기록한다. 얼굴이 있는 입력 사진의 실제 경로로 `INPUT_IMAGE`를 바꾼다.

```bash
cd ~/damgc_robot
INPUT_IMAGE=/path/to/d435_rgb.jpg
OUTPUT_DIR="$PWD/eye_roi_debug"
mkdir -p "$OUTPUT_DIR"
docker run --rm --runtime=nvidia --ipc=host --network none \
  --user "$(id -u):$(id -g)" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -e YOLO_CONFIG_DIR=/tmp -e HOME=/tmp \
  -v "$PWD:/workspace:ro" \
  -v "$INPUT_IMAGE:/input/eye_state_input.jpg:ro" \
  -v "$HOME/eye_state_runs/eye_state/baseline/weights/eye_state_best.pt:/models/eye_state_best.pt:ro" \
  -v "$HOME/.cache/damgc-eye-state/face_detection_yunet_2022mar.onnx:/models/yunet.onnx:ro" \
  -v "$OUTPUT_DIR:/outputs" \
  --workdir /workspace/tools/eye_state \
  --entrypoint python3 damgc-survivor-yolo:humble \
  /workspace/tools/eye_state/infer_eye_state_image.py \
  --image /input/eye_state_input.jpg --model /models/eye_state_best.pt \
  --yunet-model /models/yunet.onnx --output-dir /outputs \
  --device cpu --preprocessing-mode gray --min-confidence 0.60 --save-crops
```

`--preprocessing-mode gray|rgb`로 입력 mode를 비교한다. `gray`는 OpenCV BGR ROI를 gray로 바꾼 다음 3채널로 복제한다. `rgb`는 color ROI를 유지하고 Ultralytics의 기존 BGR→RGB 변환과 checkpoint 저장 transforms를 사용한다. Checkpoint metadata에는 `Resize`, `CenterCrop`, `ToTensor`, `Normalize`가 기록돼 있다. 원본 dataset도 grayscale 640×640이므로 비교 전 기본 mode는 `gray`로 두었지만, 실제 D435 성능을 확인하기 전에는 더 나은 mode라고 간주하지 않는다.

검증된 checkpoint metadata의 class mapping은 **index 0=`closed`, index 1=`open`**이다. 실행 코드는 index 순서를 고정하지 않고 매 process의 `model.names`를 읽어 예측 index를 class 이름으로 변환하며, 이름이 정확히 `open`과 `closed`가 아니면 중단한다. 기존 `infer_eye_classifier.py`도 같은 `EyeStateClassifier` loader, preprocessing helper와 class mapping 검증을 사용한다.

`--min-confidence` 기본값 0.60 아래에서는 `OPEN`/`CLOSED` 확정을 피하고 `LOW_CONFIDENCE`를 표시한다. classifier가 반환한 원래 argmax class와 confidence/probability도 console에 남긴다. confidence 기준은 D435 검증 전 임시값이다. 각 결과 뒤에는 face+ROI, classifier batch, total inference latency(ms)가 출력된다. Total은 모델 초기 로드와 debug image write를 제외하고 한 장의 inference 처리를 잰다.

검증된 `damgc-survivor-yolo:humble` Docker image에서 실제 checkpoint load, class mapping, YuNet load, 전체 정지 이미지 flow 및 debug image 기록을 확인했다. 저장된 빈 사무실 이미지는 `NO_FACE`를 반환했다(전체 inference 105.46 ms). 실행 중이던 `/leader/camera/color/image_raw`를 구독해 받은 D435 frame에서는 YuNet이 17×18 px 얼굴 후보(confidence 0.8188)를 반환했지만 양쪽 eye ROI가 `roi_below_minimum_size`로 `INVALID`가 됐다. 이 frame에서 face detection+ROI는 116.88 ms, classifier 0.02 ms(crops=0), total 116.91 ms였다. classifier가 invalid ROI를 건너뛰는 동작은 확인했으나 유효한 눈 crop은 없어 실제 좌우 OPEN/CLOSED 예측, gray/rgb 비교, 분류 confidence와 ROI 품질의 성능 분리는 **NOT RUN**이다.

```bash
python3 -m pytest -q test_face_eye_roi.py test_eye_classifier.py
python3 infer_eye_state_image.py --help
```

## 2차 Goal 3: 독립 ROS 2 node

새 package `rescue_robot_eye_state`의 `survivor_eye_state_node`가 D435 RGB `sensor_msgs/msg/Image`를 구독하고 `/leader/survivor/eye_state/debug_image`를 발행한다. 현재 확인한 실시간 입력 topic은 `/leader/camera/color/image_raw`다. `/leader/camera/color/image_rect`는 실행 중인 graph에 없었다. 기존 `/leader/survivor/debug_image`나 `survivor_pipeline.launch.py`는 건드리지 않는다.

노드는 `rescue_robot_eye_state`를 선택적으로 build한 뒤, ROS와 Ultralytics/PyTorch가 함께 설치된 runtime에서 실행한다. 현재 검증된 `damgc-survivor-yolo:humble`은 ROS2 Humble/Ultralytics/PyTorch/OpenCV를 함께 제공하므로 이 image를 사용한다. classifier와 YuNet 파일을 읽기 전용으로 mount하고 Eye State package install prefix를 source한다. GPU를 공유하는 다른 작업을 고려해 기본 device는 `cpu`다. 사용 가능한 GPU를 확인한 뒤 필요하면 `device:=0`으로 바꾼다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rescue_robot_eye_state
```

실행 환경에서 package install과 모델 경로를 mount한 후 별도 launch를 시작한다. `PACKAGE_INSTALL`은 colcon install prefix 또는 그 내용을 담은 read-only mount로 지정한다.

```bash
PACKAGE_INSTALL="$PWD/install"
docker run --rm --runtime=nvidia --network host --ipc=host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e YOLO_CONFIG_DIR=/tmp \
  -v "$PACKAGE_INSTALL:/eye_state_install:ro" \
  -v "$HOME/eye_state_runs/eye_state/baseline/weights/eye_state_best.pt:/models/eye_state_best.pt:ro" \
  -v "$HOME/.cache/damgc-eye-state/face_detection_yunet_2022mar.onnx:/models/yunet.onnx:ro" \
  --entrypoint bash damgc-survivor-yolo:humble -lc '
    source /opt/ros/humble/setup.bash
    source /eye_state_install/local_setup.bash
    ros2 launch rescue_robot_eye_state survivor_eye_state.launch.py \
      image_topic:=/leader/camera/color/image_raw \
      model_path:=/models/eye_state_best.pt \
      yunet_model_path:=/models/yunet.onnx \
      inference_rate_hz:=5.0 preprocess_mode:=gray device:=cpu
  '
```

주요 launch 인자는 `image_topic`, `model_path`, `yunet_model_path`, `face_conf_threshold`, `eye_conf_threshold`, `min_eye_width`, `min_eye_height`, `inference_rate_hz`, `preprocess_mode`, `show_landmarks`, `device`다. ROS image callback은 수신 즉시 한 개의 최신 frame 슬롯만 갱신하고, 별도 worker가 최대 지정 rate로 처리하므로 inference 중 오래된 frame queue가 쌓이지 않는다. 두 모델은 node 시작 시 한 번 load한다.

raw 좌우 결과는 timestamp, face bbox, face index, 좌우 valid/state/confidence/ROI bbox를 가진 내부 dataclass로 보관한다. Face index는 frame-local 값이며 **Face index != Survivor ID**다. 아직 consumer나 persistent ID 연결이 없고 debug image만 ROS로 발행하므로 `rescue_robot_interfaces`를 변경하지 않았다. 향후 실제 consumer를 만드는 단계에서 재사용 요구와 메시지 구조가 확정되면 typed message를 추가한다.

debug image 확인은 별도 terminal에서 실행한다. launch는 GUI를 자동 실행하지 않는다.

```bash
ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/leader/survivor/eye_state/debug_image
```

### Goal 3 검증 범위

- `rescue_robot_eye_state`만 임시 build/install base(`/tmp`)에 빌드했고 package, executable, 별도 launch 인자 탐색을 확인했다.
- 검증된 YOLO ROS runtime에서 node를 launch하고 `/survivor_eye_state_node`와 전용 debug image publisher가 생성되는 것을 확인했다. 640×480 synthetic ROS input을 구독해 같은 크기의 `bgr8` debug image를 받았고 annotation으로 2,882 pixel이 바뀌었다. 빈 사무실 frame에서 나온 작은 background face candidate는 양쪽 ROI가 invalid였으므로 classifier는 실행되지 않았다.
- 실제 카메라 전체 pipeline의 장시간 실행 및 얼굴이 포함된 좌우 classifier output은 **NOT RUN**이다. 기존 runtime에는 사용자 운용 중인 VSLAM/센서 process가 있으며, 이 단계에서는 별도 camera driver를 시작하지 않는다.

## 데이터셋

Ultralytics 이미지 분류 형식에 따라 세 split 모두에 `open`, `closed` 디렉터리가 있어야 한다.

```text
DATASET_ROOT/
├── train/{open,closed}/
├── val/{open,closed}/
└── test/{open,closed}/
```

데이터 파일은 Git 밖에 둔다. 저장소의 기존 `data/` 경로는 이미 ignore 대상이다. 같은 사람의 연속 프레임이나 거의 같은 이미지가 train과 test에 동시에 들어가지 않도록 한다. 원본 데이터가 사람 ID를 제공하면 사람 단위 분할을 유지한다. 제공하지 않으면 분할 방법을 기록하고 test 결과가 실제 일반화 성능보다 높을 수 있음을 고려한다.

1차 공개 기준 데이터셋은 [Mujahid Raja의 Eye state Classification](https://universe.roboflow.com/mujahid-raja-epyoq/eye-state-classification/dataset/1) v1이다. 제공받은 export의 README에는 CC BY 4.0, 이미지 4,000장, 640×640 stretch 전처리와 증강 없음이 기록돼 있다. 실제 파일은 train 2,800장, valid 800장, test 400장이며 `Closed_Eyes`와 `Open_Eyes` 두 클래스다. 공개 데이터의 출처·라이선스·변환 내역은 `data/eye_state/dataset_source.json`에 기록한다.

파일명의 `s####` 접두사가 사람 ID라면 원본 split에는 사람 간 누수가 있다. 실제로 학습·시험에 공통 접두사 15개가 있다. 이번 baseline은 원본 split을 유지하므로 test 결과를 새 사람에 대한 성능으로 해석하지 않는다.

## Roboflow 내보내기 자료 준비

Roboflow 계정의 API 키가 있으면 **host의 별도 가상환경**에서 아래처럼 v1을 `multiclass` 형식으로 받을 수 있다. 키를 저장소나 명령 이력에 적지 않는다. 로그인한 계정에서 v1을 같은 형식으로 내려받아 `data/eye_state_raw/`에 압축 해제해도 된다.

```bash
cd ~/damgc_robot
python3 -m venv data/eye_state_download_venv
data/eye_state_download_venv/bin/pip install roboflow
read -rsp 'Roboflow API key: ' ROBOFLOW_API_KEY && echo
export ROBOFLOW_API_KEY
data/eye_state_download_venv/bin/python - <<'PY'
import os
from roboflow import Roboflow

rf = Roboflow(api_key=os.environ['ROBOFLOW_API_KEY'])
rf.workspace('mujahid-raja-epyoq').project('eye-state-classification').version(1).download(
    'multiclass', location='data/eye_state_raw/v1'
)
PY
unset ROBOFLOW_API_KEY
```

export의 `train`, `valid`, `test`가 있는 디렉터리를 source로 지정하고 날짜를 실제 다운로드 날짜로 바꾼다. 준비 도구는 원본 split을 유지하고 `Open_Eyes → open`, `Closed_Eyes → closed`, `valid → val`만 매핑한다. 출력 경로가 이미 있으면 덮어쓰지 않는다.

```bash
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD:/workspace:ro" -v "$PWD/data:/datasets" \
  --workdir /datasets damgc-survivor-yolo:humble \
  'python3 /workspace/tools/eye_state/prepare_dataset.py --source /datasets/eye_state_raw --output /datasets/eye_state --version 1 --download-date 2026-10-03'
```

데이터를 받은 날짜가 다르면 `--download-date` 값을 변경한다. `data/eye_state/dataset_source.json`에 출처·라이선스·매핑·실제 수를 기록한다. 원본 archive와 변환 결과는 기존 `data/` ignore 규칙으로 Git에서 제외된다.

## 실행 환경

Host Python은 검증된 AI 환경이 아니다. 기존 `damgc-survivor-yolo:humble` image 안에서 도구를 실행한다. 아래 명령은 저장소를 읽기 전용, dataset을 읽기 전용, 실험 출력 디렉터리만 쓰기 가능하게 연결한다. production image와 package 버전은 변경하지 않는다.

```bash
cd ~/damgc_robot
DATASET="$PWD/data/eye_state"
OUTPUT="$HOME/eye_state_runs"
mkdir -p "$OUTPUT"

eye_docker() {
  docker run --rm --runtime=nvidia --ipc=host \
    --user "$(id -u):$(id -g)" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -e YOLO_CONFIG_DIR=/tmp \
    -v "$PWD:/workspace:ro" \
    -v "$DATASET:/dataset:ro" \
    -v "$OUTPUT:/outputs" \
    --workdir /outputs \
    damgc-survivor-yolo:humble "$1"
}

eye_docker "python3 /workspace/tools/eye_state/validate_dataset.py --data /dataset"
```

Validator는 필수 split/class, 읽을 수 있는 지원 이미지, 표본 수와 크기 통계, 클래스 불균형, split 간 SHA-256 완전 중복을 확인한다. 한 클래스가 다른 클래스의 3배를 넘으면 경고하고, 구조 누락·빈 클래스·손상/미지원 이미지·split 간 완전 중복은 오류로 종료한다.

## 학습과 평가

```bash
docker ps --format '{{.Names}} {{.Image}} {{.Status}}'
eye_docker "python3 /workspace/tools/eye_state/train_eye_classifier.py --data /dataset --model yolo11n-cls.pt --epochs 2 --imgsz 224 --batch 4 --device 0 --workers 2 --project /outputs/eye_state --name smoke --seed 42"
eye_docker "python3 /workspace/tools/eye_state/train_eye_classifier.py --data /dataset --model yolo11n-cls.pt --epochs 40 --imgsz 224 --batch 4 --device 0 --workers 2 --project /outputs/eye_state --name baseline --seed 42"
eye_docker "python3 /workspace/tools/eye_state/evaluate_eye_classifier.py --model /outputs/eye_state/baseline/weights/best.pt --data /dataset --device auto --imgsz 224 --output /outputs/eye_state/baseline/test_misclassified.csv --all-output /outputs/eye_state/baseline/test_predictions.csv"
```

학습이 중단되면 같은 run의 `last.pt`로 재개한다. `--resume`은 checkpoint에 저장된 학습 설정과 epoch를 사용한다.

```bash
eye_docker "python3 /workspace/tools/eye_state/train_eye_classifier.py --data /dataset --model /outputs/eye_state/baseline/weights/last.pt --device 0 --project /outputs/eye_state --name baseline --resume"
```

40 epoch 학습이 중단된 경우에는 Docker 접근이 가능한 Jetson 셸에서 아래 명령으로 검증, 학습 재개, 시험 평가, OPEN/CLOSED 및 최저 confidence 이미지 추론을 차례로 실행할 수 있다. 스크립트는 실행 중인 Docker 컨테이너가 있으면 기본적으로 학습을 시작하지 않는다. 결과 로그와 `eye_state_best.pt`는 `~/eye_state_runs/eye_state/baseline/` 아래에 둔다.

```bash
cd ~/damgc_robot
bash tools/eye_state/finish_run.sh
```

실행 중인 컨테이너와 함께 학습해야 한다면 해당 작업의 지연 및 Jetson 공유 메모리 부족 가능성을 감수하고 아래처럼 명시적으로 허용한다. 스크립트는 컨테이너 목록과 호스트의 사용 가능 메모리를 출력하며, 사용 가능 메모리가 2 GiB 미만이면 중단한다. 이 옵션은 기존 컨테이너를 종료하거나 변경하지 않는다. 학습 도중 로봇 작업이 느려지거나 메모리가 급감하면 학습 명령을 중단하고 `last.pt`에서 재개한다.

```bash
cd ~/damgc_robot
EYE_STATE_ALLOW_ACTIVE_CONTAINERS=1 bash tools/eye_state/finish_run.sh
```

학습기는 학습 전에 dataset을 검사하고 장치 선택 및 사용 가능한 출력 디스크를 표시한다. 먼저 2 epoch smoke 학습에서 CUDA 실행과 checkpoint 생성을 확인한 뒤 40 epoch 전체 학습을 진행한다. 위 명령의 `--device 0`은 CUDA가 없으면 오류로 종료한다. CUDA 메모리가 부족하면 다른 GPU 작업을 멈추고 `--batch 4` 또는 `--batch 2`로 다시 시도한다. VSLAM 또는 Survivor workload와 동시 학습은 권장하지 않으며, 이번 재개 학습만 사용자 요청에 따라 병행했다. pretrained weight가 없다면 첫 학습 시 `/outputs`에 내려받는다. run과 weight는 저장소 바깥에 남으며 production 경로로 자동 복사하지 않는다.

평가기는 전체 표본 수, accuracy, OPEN/CLOSED 각각의 precision·recall, confusion matrix를 출력하고 오분류 이미지의 경로·정답·예측·confidence를 CSV로 저장한다. `--all-output`을 지정하면 전체 표본의 예측도 CSV로 기록하고 그중 confidence가 가장 낮은 표본을 출력한다. 이번 데이터셋의 실제 학습·평가·추론 결과와 해석은 [개발 문서](../../docs/SURVIVOR_EYE_STATE_DEVELOPMENT.md)에 기록했다.

학습을 다시 시작하지 않고 저장된 최종 모델만 재검증하려면 다음을 실행한다. 이 스크립트는 test 전체 재평가와 OPEN·CLOSED·최저 confidence 표본 추론을 수행한다.

```bash
cd ~/damgc_robot
bash tools/eye_state/verify_final.sh
```

Ultralytics 기본 분류 증강 및 validation/inference 전처리를 사용한다. 이 checkout에는 목표인 D435 눈 crop이 없어 가로로 긴 ROI가 과도하게 잘리는지 실제 확인하지 못했다. 대표 샘플로 눈 영역 손실이 확인되기 전에는 기본 전처리를 유지한다.

## 단일 이미지 추론

데이터셋 mount 내부의 샘플 경로를 지정한다.

```bash
eye_docker "python3 /workspace/tools/eye_state/infer_eye_classifier.py --model /outputs/eye_state/baseline/weights/eye_state_best.pt --image /dataset/test/open/s0014_07511_0_0_1_2_1_02_png.rf.4e54af10244805ec3195df0419803d40.jpg --device auto --imgsz 224"
```

출력에는 `OPEN` 또는 `CLOSED` 예측, confidence 및 두 클래스의 확률이 포함된다. `UNKNOWN`은 classifier class에 없다. 신뢰도·가시성에 따른 downstream 판단은 이번 단계에 포함하지 않는다.

## 한계

`OPEN`과 `CLOSED`는 이미지에서 관찰되는 눈 모양을 나타낸다. `CLOSED`는 의식이 없다는 뜻이 아니고, `OPEN`은 의식이 있다는 뜻이 아니다. 공개 데이터는 640×640으로 늘린 회색조 영상이며 D435의 RGB 눈 crop과 입력 분포가 다르다. 조명, 해상도, 카메라 거리, 안경, 가림, 머리 자세 차이도 크다. 유효한 후속 모델을 위해 동의를 거친 D435 눈 crop을 사람 단위로 나누어 추가 평가해야 한다.
