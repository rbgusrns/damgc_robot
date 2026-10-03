# Survivor Eye State 개발 — 1차 독립 분류기

## 목적과 범위

눈 이미지 한 장을 YOLO11n-cls fine-tuned classifier에 입력해 `OPEN` 또는 `CLOSED`와 confidence를 반환하는 독립 개발 도구다. 이 결과는 관찰 가능한 눈 개폐 상태만 나타낸다. `CLOSED`가 의식이 없음을, `OPEN`이 의식이 있음을 뜻하지 않으며 의식 진단에 사용할 수 없다.

1차 범위는 dataset 검증, classifier 학습, 독립 test 평가, 단일 이미지 추론이다. 다음 기능은 구현하지 않았다: 얼굴 검출/landmark, 자동 Eye ROI 추출, ROS node/topic, Survivor ID 연결, 3초 temporal history, rqt/RViz overlay, pose/posture/activity. 2차 개발은 D435 RGB에서 얼굴 검출과 landmark로 좌우 Eye ROI를 자동 추출하고 이번 classifier에 전달하는 흐름을 독립 검증한다. Survivor ID 연결과 화면 표시는 이후 단계에서 다룬다.

## 모델과 실행 환경

- Base model은 `yolo11n-cls.pt`이며, fine-tuned checkpoint는 실험별 `weights/best.pt`로 둔다.
- 확인 환경은 `damgc-survivor-yolo:humble`, Ultralytics 8.4.147, NVIDIA PyTorch 2.5.0a0+872d972e41.nv24.08, torchvision 0.20.0, CUDA 12.6, Jetson Orin GPU다. 컨테이너에서 `yolo11n-cls.pt`를 분류 task로 로드했다.
- 기존 image와 production AI package는 변경하지 않았다. 기존 image에서 별도 `docker run`을 실행하고 저장소는 읽기 전용으로 mount했다. VSLAM/Survivor workload와 동시 학습은 권장하지 않으며, 이번 재개 학습만 사용자 요청에 따라 실행 중인 VSLAM 개발 컨테이너와 병행했다.
- 기본 classification 전처리를 baseline으로 사용한다. 제공된 export는 640×640으로 늘려진 회색조 영상이며, 표본 픽셀 조사에서도 4,000장 모두 회색조로 확인됐다. 실제 D435 RGB eye crop 표본이 없어 aspect ratio와 crop 영향은 아직 검증되지 않았다. 눈 영역 손실이 확인되기 전에는 custom trainer나 전처리를 추가하지 않는다.

## 데이터셋

필수 구조는 `train/{open,closed}`, `val/{open,closed}`, `test/{open,closed}`다. class 이름은 소문자 `open`, `closed`이며 평가/추론 시 모델에도 정확히 두 class가 있는지 확인한다. Validator는 split/class 누락, 빈 class, 지원하지 않는 확장자, 손상 이미지, 표본 수와 크기 통계, 3:1 초과 불균형 경고, split 간 SHA-256 완전 중복을 확인한다.

1차 공개 기준 데이터셋은 [Roboflow Universe의 Eye state Classification v1](https://universe.roboflow.com/mujahid-raja-epyoq/eye-state-classification/dataset/1)이다. [프로젝트 공개 페이지](https://universe.roboflow.com/mujahid-raja-epyoq/eye-state-classification)에는 현재 분류 작업, `Closed_Eyes`·`Open_Eyes` 두 클래스, CC BY 4.0, 이미지 약 4,000장, 데이터셋 버전 5개가 표시된다. 이번 실험에는 그중 사용자가 내려받은 **v1**만 사용했다. 제작자는 Mujahid Raja이고 원본 `README.dataset.txt`에도 CC BY 4.0 라이선스가 적혀 있다. 사용·공유 시 제작자, 자료명, 출처 URL, 라이선스와 변경 내용을 표시해야 한다. 제공받은 v1 export의 `README.roboflow.txt`에는 이미지 4,000장, 증강 없음, 640×640 stretch 전처리가 기록돼 있다. 실제 파일은 train 2,800장(open 1,373 / closed 1,427), valid 800장(open 426 / closed 374), test 400장(open 201 / closed 199)이다.

2026-10-03에 사용자가 인증을 통해 내려받아 `data/eye_state_raw/`에 압축 해제한 v1 export를 사용했다. 인증 정보는 저장소에 넣지 않았다. [`prepare_dataset.py`](../tools/eye_state/prepare_dataset.py)는 `Closed_Eyes → closed`, `Open_Eyes → open`, `valid → val`을 매핑하고 원본 split을 유지해 `data/eye_state/`를 생성했다. 이미지에 추가 resize나 증강을 하지 않고 복사했다. 출처·라이선스·원본 4,000장·분할별 수·변환·다운로드 날짜는 `data/eye_state/dataset_source.json`에 기록했고 실제 파일 수와 대조했다. 원본 README에는 [다른 Roboflow workspace 경로](https://universe.roboflow.com/mujahid-raja/eye-state-classification)도 적혀 있으며, 위 공개 버전 페이지와 자료명·버전·파일 수를 대조했다.

파일명의 `s####` 접두사를 사람 ID로 해석할 가능성이 있다. 학습에는 접두사 15개, 검증과 시험에는 각각 16개가 있다. 공통 접두사는 학습·검증 15개, 학습·시험 15개, 검증·시험 16개다. 따라서 접두사가 사람 ID라면 원본 split은 사람 단위 독립 분할이 아니다. 실제 사람 ID인지는 원본 메타데이터로 확인할 수 없으므로 **확인 불가**다. SHA-256 기준 동일 파일과 Roboflow의 `.rf.` 해시 앞 원본 파일명은 split 사이에 중복되지 않았다. 다만 같은 사람의 연속 프레임이나 같은 원본에서 파생된 유사 이미지가 분산됐는지는 확인할 수 없다. 이번 실험 수치는 **원본 split 기준 baseline**으로만 해석한다.

이전에 조사한 PopEYE는 촬영 도메인이 안과 NIR 영상이며 공개 subject split이 없어 이번 RGB 기준 자료로 사용하지 않는다. MRL Eye도 제공자의 연구용·비상업 및 재배포 제한 때문에 이번 자동 다운로드 대상이 아니다.

## 사용 방법

저장소와 분리된 dataset/output을 연결하는 전체 Docker 명령은 [`tools/eye_state/README.md`](../tools/eye_state/README.md)에 있다. 실행 흐름은 다음과 같다.

```bash
python3 /workspace/tools/eye_state/prepare_dataset.py --source /datasets/eye_state_raw --output /datasets/eye_state --version 1 --download-date 2026-10-03
python3 /workspace/tools/eye_state/validate_dataset.py --data /dataset
python3 /workspace/tools/eye_state/train_eye_classifier.py --data /dataset --model yolo11n-cls.pt --epochs 2 --imgsz 224 --batch 4 --device 0 --workers 2 --project /outputs/eye_state --name smoke --seed 42
python3 /workspace/tools/eye_state/train_eye_classifier.py --data /dataset --model yolo11n-cls.pt --epochs 40 --imgsz 224 --batch 4 --device 0 --workers 2 --project /outputs/eye_state --name baseline --seed 42
python3 /workspace/tools/eye_state/evaluate_eye_classifier.py --model /outputs/eye_state/baseline/weights/eye_state_best.pt --data /dataset --device auto --output /outputs/eye_state/baseline/final_misclassified.csv --all-output /outputs/eye_state/baseline/final_predictions.csv
python3 /workspace/tools/eye_state/infer_eye_classifier.py --model /outputs/eye_state/baseline/weights/eye_state_best.pt --image /dataset/test/open/s0014_07511_0_0_1_2_1_02_png.rf.4e54af10244805ec3195df0419803d40.jpg --device auto
```

평가기는 총 표본 수, accuracy, OPEN/CLOSED별 precision·recall, confusion matrix를 출력하고 오분류 경로·정답·예측·confidence를 CSV에 기록한다. 낮은 confidence를 UNKNOWN으로 처리하는 정책은 classifier에 넣지 않았다.

## 실제 실행 상태

확인한 학습 환경은 `damgc-survivor-yolo:humble`, Ultralytics 8.4.147, PyTorch 2.5.0a0+872d972e41.nv24.08, CUDA 12.6, Jetson Orin이다. GPU 메모리를 아끼기 위해 실제 실험은 YOLO11n-cls pretrained, 40 epoch, 224 px, batch 4, worker 2, seed 42, CUDA device 0으로 실행했다. 2 epoch smoke 학습은 CUDA에서 완료됐고 `smoke/weights/best.pt`가 생성됐다. Ultralytics 그래프 생성에서 `PyDataFrame` 오류가 출력됐지만 smoke 프로세스는 성공 코드로 종료되고 checkpoint가 생성됐다.

전체 학습은 35 epoch 기록 후 36 epoch 도중 중단됐다가 `last.pt`에서 재개해 **40/40 epoch 완료**했다. 재개 시 VSLAM 개발 컨테이너 `isaac_ros_dev-aarch64-container`가 실행 중이었고, 사용자의 동시 실행 요청에 따라 명시적 옵션으로 진행했다. 재개 전 호스트 사용 가능 메모리는 약 2.9 GiB, GR3D 사용률 표본은 8–14%였다. 재개 로그의 CUDA 사용량 표시는 약 0.15 GiB였으나 이는 Jetson 전체 메모리 사용량을 뜻하지 않는다. 학습 종료 후 `results.csv` 마지막 행은 40 epoch이며 validation top-1은 1.0이다. 그래프 생성의 `PyDataFrame` 오류는 재발했지만 checkpoint와 독립 test 평가가 생성됐다. 기존 VSLAM 기능 자체의 회귀 시험은 수행하지 않았다.

| 항목 | 2026-10-03 결과 |
| --- | --- |
| 공개 페이지의 이미지 수 | v1 총 4,000장; train 2,800 / val 800 / test 400 |
| 실제 다운로드·클래스별 수 | 4,000장; train 1,373/1,427, val 426/374, test 201/199 (open/closed) |
| dataset validation | PASS, 오류·경고 0개; 전체 640×640, split 간 동일 파일 없음 |
| smoke 학습·checkpoint | CUDA에서 2 epoch 완료, `smoke/weights/best.pt` 생성 |
| 전체 학습 | `last.pt`에서 재개 후 40/40 epoch 완료; 마지막 validation top-1 1.0 |
| 전체 학습 checkpoint | `/home/maze/eye_state_runs/eye_state/baseline/weights/best.pt`, `last.pt` 각 3,184,443 byte |
| 별도 이름 모델 | `/home/maze/eye_state_runs/eye_state/baseline/weights/eye_state_best.pt`; `best.pt`와 SHA-256 일치 |
| test 평가 | 400/400 정답, accuracy 1.0000; 오분류 0장 |
| OPEN 성능 | precision 1.0000, recall 1.0000, 201장 |
| CLOSED 성능 | precision 1.0000, recall 1.0000, 199장 |

독립 test split의 혼동 행렬은 행이 실제 클래스, 열이 예측 클래스다.

| 실제 / 예측 | OPEN | CLOSED |
| --- | ---: | ---: |
| OPEN | 201 | 0 |
| CLOSED | 0 | 199 |

단일 이미지 추론은 test의 OPEN 표본 `s0014_07511_0_0_1_2_1_02_png.rf.4e54af10244805ec3195df0419803d40.jpg`에서 OPEN 1.0000(CLOSED 0.0000), CLOSED 표본 `s0034_00018_0_0_0_0_0_02_png.rf.ae71e4fef7e1f995de2bed01f3ac3916.jpg`에서 CLOSED 1.0000(OPEN 0.0000)을 반환했다. 전체 test 중 confidence가 가장 낮은 표본은 `s0001_02879_0_1_1_0_1_01_png.rf.f62eccc363a97abfc6448816af1f64b3.jpg`이며, 실제 OPEN, 예측 OPEN, confidence 0.9984(CLOSED 0.0016)였다. 이 표본도 낮은 신뢰도의 진정한 애매한 사례라기보다는 이번 test에서 **상대적으로** 가장 낮은 confidence 사례다.

오분류 경로 파일 `test_misclassified.csv`에는 header만 있다. `test_predictions.csv`에는 test 전체 400장의 경로·정답·예측·confidence가 저장됐다. 정확도 100%는 **원본 Roboflow split에 한정된 측정값**이다. 사람 접두사 중복 가능성, 회색조 640×640 stretch 전처리, D435 RGB와의 촬영 환경 차이 때문에 이 값을 새 사람이나 실제 로봇 현장 성능으로 일반화하지 않는다.

최종 이름의 `eye_state_best.pt`를 사용해 학습 없이 독립 재평가했다. `final_evaluation.log`에서도 400/400 정답, accuracy 1.0000, OPEN·CLOSED precision/recall 각각 1.0000, 혼동 행렬 OPEN 201/0·CLOSED 0/199가 재현됐다. `final_predictions.csv`는 400행, `final_misclassified.csv`는 header만 있다. OPEN·CLOSED·최저 confidence 표본의 `final_*_inference.log`도 위 예측과 확률을 재현했다. 재평가는 [`verify_final.sh`](../tools/eye_state/verify_final.sh)로 실행했으며 학습·다운로드를 수행하지 않았다.

현재 Codex 작업 셸에서는 Docker 소켓 연결이 sandbox에 의해 차단돼, Docker 접근이 가능한 별도 Codex CLI 창에서 아래 명령을 실행했다. 해당 창의 결과가 공유 출력 디렉터리에 기록됐으며, 이 문서의 수치는 그 로그와 CSV를 직접 확인한 값이다.

```bash
cd ~/damgc_robot
bash tools/eye_state/finish_run.sh
```

이 스크립트는 검증, 40 epoch까지 학습 재개, test 평가, OPEN·CLOSED 표본 및 test 중 최저 confidence 표본 추론을 순서대로 실행하고 로그를 `~/eye_state_runs/eye_state/baseline/`에 남긴다.

VSLAM 컨테이너가 실행 중인 상태에서 학습을 요청한 경우, 명시적인 `EYE_STATE_ALLOW_ACTIVE_CONTAINERS=1` 환경변수로 동시 실행할 수 있다. Jetson은 CPU와 GPU가 메모리를 공유하므로 로봇 작업의 지연 및 메모리 부족 위험이 있다. 스크립트는 2 GiB 미만의 사용 가능 메모리에서는 학습을 시작하지 않으며, 실행 중인 컨테이너를 종료하지 않는다.

기존 코드 영향 검사는 Survivor 분류·거리·Registry 및 통합 launch 관련 단위 테스트 62개, 추가 person detector·Registry·주행 launch 테스트 16개로 총 **78개 통과**했다. 검사 시 ROS Humble과 이 저장소의 `install/setup.bash`를 소싱하고 로그는 `/tmp`에 썼다. Eye State 참조는 기존 Survivor, bringup, VSLAM 스크립트에서 발견되지 않았다. 이번 Eye State 작업으로 production 파일을 수정하지 않았으며, 작업 트리의 bringup/VSLAM 변경은 작업 전부터 존재한 다른 사용자 변경이다. 실제 카메라·로봇·Survivor/VSLAM/Nav2 전체 runtime 회귀 시험은 **미실행**이다.

## 산출물과 한계

기본 예시에서는 checkpoint가 `~/eye_state_runs/eye_state/baseline/weights/best.pt`에 저장된다. dataset과 원본 archive는 저장소의 ignore 대상인 `data/`에 두고 pretrained weight, run, CSV는 저장소 밖에 둔다. production 위치로 weight를 자동 복사하지 않는다.

핵심 한계는 공개 데이터가 회색조이며 640×640 stretch 처리돼 D435의 RGB 눈 ROI와 다르다는 점이다. 촬영 거리와 ROI 해상도, 안경·가림, 머리 회전, 조명 변화, 같은 사람 이미지의 split 누수 가능성도 남는다. 이번 원본 split의 test 수치는 새 사람이나 D435 입력에 대한 일반화 성능으로 해석할 수 없다. 실제 D435 자료를 추가할 때는 적절한 동의와 사람 단위 train/val/test 분리를 고려하고, 독립 test의 CLOSED recall을 반드시 확인한다.

## 2차 개발 연결

2차 개발은 `D435 RGB → 얼굴 검출 → Face Landmark → 좌우 Eye ROI 자동 추출 → 이번 eye_state_best.pt 추론`까지 독립 검증한다. **2차에서도 Survivor ID 연결은 하지 않는다.** 이후 단계에서 `3초 temporal history → OPEN/CLOSED/UNKNOWN → 기존 Survivor #ID 연결 → rqt_image_view/RViz 표시`를 진행한다. 이번 classifier에는 UNKNOWN class가 없다. 얼굴·눈 검출 실패, 작은 ROI, 낮은 confidence, 영상 품질 부족, 좌우 결과 불일치는 이후 application logic에서 UNKNOWN으로 판단한다.
