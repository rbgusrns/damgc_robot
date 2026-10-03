# 독립형 Eye State 분류기

이 디렉터리는 눈 이미지 분류를 위한 ROS 비의존 도구를 담는다. 분류 클래스는 `open`, `closed` 두 개다. 얼굴을 찾거나 눈 영역을 자동 추출하지 않으며, ROS topic 발행이나 Survivor ID 연결도 하지 않는다.

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
