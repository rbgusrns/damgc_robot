# Survivor Eye State 시스템 설계 및 개발 문서

이 문서는 Eye State 기능의 현재 구현, 검증 범위, 후속 개발 설계를 설명하는 기준 문서다. 단계별 작업 기록은 [Survivor Eye State 개발 이력](SURVIVOR_EYE_STATE_DEVELOPMENT.md)을 참고한다.

## A. 목적과 사용 범위

구조 작업이나 재난 대응 중 카메라 영상에서 생존자의 눈이 보이는 경우, 영상에서 관찰되는 개폐 상태를 분류할 수 있도록 한다. 현재 classifier는 `OPEN`과 `CLOSED` 두 범주만 반환한다. 눈 개폐는 영상 관찰 결과이며 의식 진단이 아니다.

- `CLOSED`는 의식이 없음을 뜻하지 않는다.
- `OPEN`은 의식이 있음을 뜻하지 않는다.
- 안면·눈 검출 실패, 작은 ROI, 낮은 confidence, 가림이나 영상 품질 문제는 상태 판단의 근거로 사용하지 않는다.

이번 2차 개발은 D435 RGB frame에서 얼굴 검출, landmark 기반 양쪽 눈 ROI, 기존 classifier, 전용 ROS debug image를 별도 node에서 실행하는 범위다. Temporal history, blink 구분, Survivor ID 연결, RViz 표시, pose/posture/activity는 구현하지 않았다.

## B. 전체 시스템 아키텍처

```text
                         Intel RealSense D435
                              RGB frame
                                  │
                   ┌──────────────┴──────────────┐
                   │                             │
          기존 Survivor pipeline          Eye State pipeline
          (현재 코드·구조 유지)              (새 독립 node)
                   │                             │
          YOLO person detection         OpenCV FaceDetectorYN
                   │                    + YuNet 5-point landmark
          Depth / Camera XYZ                         │
                   │                    좌·우 eye ROI 계산 / 품질검사
          exact-time TF / Map XYZ                    │
                   │                    YOLO11n-cls eye_state_best.pt
          Persistent Survivor Registry               │
                   │                    Face별 L/R raw state + confidence
          persistent Survivor #ID                    │
                   │                    전용 debug image topic
                   │                             │
          기존 RViz / rqt 화면                 현재까지 구현 완료
                                                 │
                                   ┄┄┄ Stage 3 계획 ┄┄┄
                                   약 3초 temporal history
                                   blink / sustained closed 구분
                                   OPEN / CLOSED / UNKNOWN
                                                 │
                                   ┄┄┄ Stage 4 계획 ┄┄┄
                                   기존 Survivor #ID와 observation 연결
                                                 │
                                   ┄┄┄ Stage 5 계획 ┄┄┄
                                   rqt / RViz 표시
                                                 │
                                   ┄┄┄ Stage 6 계획 ┄┄┄
                                   D435 crop 수집·사람 검수·fine-tuning
```

Stage 1 standalone classifier의 학습 및 공개 test split 평가는 완료했다. Stage 2 standalone image 도구와 독립 ROS node는 구현·빌드됐다. Goal 4 실시간 시험에서 실제 D435 얼굴, 유효한 양쪽 눈 ROI, classifier 출력과 ROS debug image까지 확인했다. 사용자가 양쪽 눈 OPEN이라고 확인한 frame에서 왼쪽 눈이 CLOSED로 오분류되어 실물 정확도는 아직 충분하지 않다. 따라서 **2차 전체 상태는 PARTIAL**이다.

## C. 전체 개발 단계

| 단계 | 범위 | 상태 |
| --- | --- | --- |
| Stage 1 | crop된 눈 이미지 → YOLO11n-cls → OPEN/CLOSED와 confidence | 완료. 공개 test split 결과이며 실물 성능 보증은 아니다. |
| Stage 2 | D435 RGB → 얼굴/landmark → L/R eye ROI → classifier → debug image | 독립 node와 전체 연결을 실물 D435에서 확인. 유효 ROI와 결과를 확인했지만 양쪽 OPEN frame에서 왼쪽 오분류가 있어 정확도 검증은 PARTIAL. |
| Stage 3 | 약 3초 raw 결과 history, blink와 지속 감김 구분, OPEN/CLOSED/UNKNOWN | 계획. sample 수와 threshold는 실물 자료로 정한다. |
| Stage 4 | Eye State observation을 기존 persistent Survivor ID와 연결 | 계획. 새 ID를 만들지 않고 Survivor Registry ID를 authority로 쓴다. |
| Stage 5 | rqt와 RViz 시각화 | 계획. rqt에는 Survivor ID와 상태, RViz에는 기존 marker/text 확장을 검토한다. |
| Stage 6 | 실제 D435 crop 수집, 사람의 ground truth 검수, subject-separated data로 필요 시 fine-tuning | 계획. classifier 출력 자체를 label로 사용하지 않는다. |

## D. Stage 1: standalone classifier

### 데이터와 라이선스

- 출처: [Roboflow Universe, Eye state Classification](https://universe.roboflow.com/mujahid-raja-epyoq/eye-state-classification), [v1 export](https://universe.roboflow.com/mujahid-raja-epyoq/eye-state-classification/dataset/1)
- 제작자: Mujahid Raja
- 내려받은 자료의 표기 라이선스: CC BY 4.0. 재사용·배포 시 출처, 저작자, 라이선스와 변경 사항을 표시한다.
- 사용자 인증으로 내려받은 v1 export의 source/split/class 수와 provenance는 로컬 `data/eye_state/dataset_source.json`에 기록했다. dataset 이미지와 archive는 Git에 넣지 않는다.
- 원본 class `Open_Eyes`를 `open`, `Closed_Eyes`를 `closed`로 바꿨다. `valid` split 이름만 `val`로 변경하고 원본 split을 유지했다. 추가 resize나 augmentation 없이 이미지를 복사했다.

| split | open | closed | 합계 |
| --- | ---: | ---: | ---: |
| train | 1,373 | 1,427 | 2,800 |
| val | 426 | 374 | 800 |
| test | 201 | 199 | 400 |

Validator는 세 split 및 class directory, 비어 있지 않은 표본, 확장자, decode 가능성, 크기 통계, 3:1 초과 class 불균형, split 사이 SHA-256 완전 중복을 확인했다. 이번 dataset은 오류·경고 없이 통과했고 export 기록상 이미지는 640×640이다.

원본 파일명 `s####` 접두사가 사람 ID일 가능성이 있으나 제공된 자료에서 그 의미를 확정하지 못했다. train/val/test 사이 접두사 중복이 있으므로 그 접두사가 subject ID라면 subject-independent split이 아니다. 동일 이미지 hash와 `.rf.` hash 앞 원본 파일명 완전 중복은 split 사이에서 발견되지 않았지만, 같은 사람이나 연속 촬영·유사 frame 누수는 배제할 수 없다.

### 모델, 학습과 평가

- Base model: pretrained `yolo11n-cls.pt`.
- Ultralytics 분류 task로 fine-tune했다. 실제 학습 parameter: 40 epochs, image size 224, batch 4, worker 2, seed 42, deterministic, CUDA device 0. 중단된 run을 `last.pt`에서 재개해 40/40 epoch를 완료했다.
- 학습 시 Ultralytics 기본 classification preprocessing을 사용했다. 확인한 transform은 resize, center crop, tensor 변환, normalize다. dataset 이미지 자체는 grayscale 영상이 3채널로 저장된 형식이었다.
- best checkpoint의 class index는 `0=closed`, `1=open`이다. 코드에서는 숫자 순서를 하드코딩하지 않고 checkpoint의 `model.names`로 매핑한다.
- 최종 checkpoint: `/home/maze/eye_state_runs/eye_state/baseline/weights/eye_state_best.pt` (3,184,443 bytes). `best.pt`와 SHA-256이 일치한다. 이 파일은 Git에 포함하지 않는다.

최종 독립 test 결과는 400/400 정답, accuracy 1.0000이었다. OPEN precision/recall은 각각 1.0000 (201장), CLOSED precision/recall은 각각 1.0000 (199장)이다.

| 실제 / 예측 | OPEN | CLOSED | 실제 표본 |
| --- | ---: | ---: | ---: |
| OPEN | 201 | 0 | 201 |
| CLOSED | 0 | 199 | 199 |

오분류는 없었다. test에서 confidence가 가장 낮았던 표본도 OPEN 0.9984로 분류됐다. 이 성능은 위 Roboflow v1 원본 split의 결과다. 사람 간 split 독립성을 확인하지 못했고 grayscale public image와 D435 RGB crop 사이 domain gap도 크므로, 100%를 실제 현장 정확도로 해석하지 않는다.

단일 이미지 추론 도구는 `tools/eye_state/infer_eye_classifier.py`다. 학습·검증·평가 방법은 [Eye State 개발 이력](SURVIVOR_EYE_STATE_DEVELOPMENT.md)과 [도구 README](../tools/eye_state/README.md)를 참고한다. 출력 checkpoint는 실험 경로의 `weights/eye_state_best.pt`이며 Git 관리 대상이 아니다.

## E. Stage 2: 얼굴, landmark, ROI와 ROS node

### Face/Landmark backend

OpenCV DNN의 `FaceDetectorYN` API와 YuNet ONNX `face_detection_yunet_2022mar.onnx`를 사용한다. OpenCV 4.5.4에서 `FaceDetectorYN` 생성 및 실제 `detect()` 호출을 확인했다. MediaPipe나 임의 출처의 얼굴 model은 사용하지 않았다. 이 선택은 검증된 aarch64 runtime에 이미 포함된 OpenCV를 재사용하고, 얼굴 bbox와 눈 좌표를 포함한 5점 landmark를 한 단계에서 얻으며, 별도 Python package가 필요 없기 때문이다.

- Model source: [OpenCV Zoo YuNet](https://github.com/opencv/opencv_zoo/tree/models/face_detection_yunet)
- 고정한 Zoo source revision: [`c97242ce7f2a554e288b50eabd9f5df957e78801`](https://github.com/opencv/opencv_zoo/tree/c97242ce7f2a554e288b50eabd9f5df957e78801/models/face_detection_yunet)
- License: 해당 모델 directory의 MIT license
- SHA-256: `50ef07f702a31741ca46a4c0d947773b64143b9362780237bf0d427d6c79bab7`
- OpenCV Zoo에서 받은 ONNX weight는 cache/data에 두고 Git에 넣지 않는다.

YuNet은 얼굴 사각형과 5점(양쪽 눈 중심, 코 끝, 입 양쪽 끝점)을 반환한다. 구현에서 YuNet row의 첫 번째 눈 좌표를 subject-right, 두 번째 눈 좌표를 subject-left로 저장한다. `Left/Right`는 사람 본인의 해부학적 좌우다. 정면 영상에서는 subject-left가 화면 오른쪽에 보인다.

### ROI 계산과 품질 gate

`tools/eye_state/face_eye_roi.py`의 `FaceEyeROIExtractor.extract()`는 입력 BGR frame의 크기에 맞게 detector 입력 크기를 설정하고 모든 얼굴을 검출한다. 각 얼굴 landmark의 두 눈 중심 사이 거리 `d`를 구해 한 변이 `round(0.85 × d)`인 정사각형 ROI를 각 눈 중심에 배치한다.

- face confidence 기본 threshold는 0.60이다.
- crop 좌표는 image boundary에 맞춰 clamp한다.
- clamp로 잘린 면적 비율이 25%를 초과하면 `severe_frame_clipping`으로 invalid 처리한다. 경미한 clipping은 edge pixel 복제로 정사각형 크기를 채운다.
- 기본 최소 eye ROI 크기는 24×24 px다. 어느 한 변이라도 기준 미만이면 `roi_below_minimum_size`이고 classifier를 호출하지 않는다.
- 좌표가 frame 안에 없는 eye landmark, 잘못된 face/inter-eye geometry, 빈 ROI, confidence 미달은 invalid다. 두 eye landmark 중 하나가 없거나 inter-eye 거리를 계산할 수 없을 때는 두 눈 모두 invalid 처리한다.
- blur/조명 품질 점수나 head-pose threshold는 아직 없다. side face는 landmark 품질이 낮거나 ROI가 눈을 포함하지 못해도 detector가 좌표를 반환할 수 있어 별도 검증이 필요하다.

### Classifier 연결 및 ROS 구성

ROS package는 `src/leader/rescue_robot_eye_state/`의 `rescue_robot_eye_state`이며 node 이름은 `survivor_eye_state_node`다. 기존 person detector, Survivor Registry, interface package 안에는 코드를 삽입하지 않는다.

| 파일 | 실제 구현 역할 |
| --- | --- |
| `tools/eye_state/face_eye_roi.py` | `ExtractorConfig`로 품질 기준을 설정하고, `FaceEyeROIExtractor.extract()`로 YuNet을 호출한다. `faces_to_eye_rois()`는 landmark를 좌우 `EyeROI`로 바꾼다. |
| `tools/eye_state/eye_classifier.py` | `EyeStateClassifier`가 checkpoint와 class 이름을 확인해 모델을 한 번 로드한다. `prepare_eye_crop()`이 전처리를 선택하고 `classify_valid_eye_rois()`가 유효한 crop만 모아 추론한다. |
| `tools/eye_state/fetch_yunet_model.py` | 고정된 OpenCV Zoo YuNet 파일을 내려받고 SHA-256을 확인한다. |
| `tools/eye_state/inspect_eye_rois.py`, `infer_eye_state_image.py` | 저장된 RGB image에서 ROI만 확인하거나 ROI부터 L/R 분류까지 독립 실행한다. 후자는 `--save-crops`를 지원한다. |
| `src/leader/rescue_robot_eye_state/scripts/survivor_eye_state_node.py` | `SurvivorEyeStateNode`가 ROS 입력을 받고 최신 frame 하나를 worker에서 처리한다. `make_frame_result()`는 내부 결과를 만들고 `annotate_frame()`은 debug image를 그린다. |
| `src/leader/rescue_robot_eye_state/launch/survivor_eye_state.launch.py` | 독립 node와 아래 parameter를 실행한다. 기존 Survivor launch에 포함되지 않는다. |
| `src/leader/rescue_robot_eye_state/CMakeLists.txt`, `package.xml` | node·launch·공유 helper 파일의 설치와 ROS package 의존성을 선언한다. |

- Input: `/leader/camera/color/image_raw` (`sensor_msgs/msg/Image`). 실제 graph에서 후보 `/leader/camera/color/image_rect`는 없고 raw RGB topic을 확인했다.
- ROS image 변환: `cv_bridge`, `imgmsg_to_cv2(..., desired_encoding="bgr8")`.
- Output: `/leader/survivor/eye_state/debug_image` (`sensor_msgs/msg/Image`, `bgr8`). 기존 `/leader/survivor/debug_image`에는 publish하지 않는다.
- 별도 launch: `survivor_eye_state.launch.py`. 기존 `survivor_pipeline.launch.py`에 자동 연결하지 않는다.
- Model path는 launch/ROS parameter `model_path`로 설정하고 node 시작 시 한 번 load한다. YuNet도 `yunet_model_path`로 시작 때 한 번 load한다.
- Classifier는 valid한 좌우 ROI만 모아 frame 단위로 batch inference한다. `gray` mode는 BGR→grayscale→3채널 replicate 후 Ultralytics의 일반 classifier transform을 사용한다. `rgb` mode는 원래 BGR image를 전달하고 Ultralytics의 기본 BGR→RGB 처리를 사용한다.
- Classifier confidence threshold 기본값은 0.60이다. 이 미만이면 raw argmax/confidence는 보존하고 화면 state를 `LOW_CONFIDENCE`로 표시한다. 이것은 Stage 3의 temporal `UNKNOWN`이 아니다.
- 모델 class는 checkpoint `names`를 확인하며 정확히 `open`, `closed` 두 개여야 한다.

Launch parameter:

| 이름 | 기본값 | 의미 |
| --- | --- | --- |
| `image_topic` | `/leader/camera/color/image_raw` | RGB Image 입력 |
| `model_path` | `/home/maze/eye_state_runs/eye_state/baseline/weights/eye_state_best.pt` | fine-tuned classifier checkpoint |
| `yunet_model_path` | `/home/maze/.cache/damgc-eye-state/face_detection_yunet_2022mar.onnx` | YuNet ONNX model |
| `face_conf_threshold` | `0.60` | YuNet face threshold |
| `eye_conf_threshold` | `0.60` | classifier 최소 confidence |
| `min_eye_width`, `min_eye_height` | `24`, `24` | ROI 최소 크기(px) |
| `inference_rate_hz` | `5.0` | 최대 처리 rate; 허용 범위는 `(0, 30]` |
| `preprocess_mode` | `gray` | `gray` 또는 `rgb` |
| `show_landmarks` | `true` | debug image의 5점 표시 여부 |
| `device` | `cpu` | `cpu` 또는 CUDA index 문자열(예: `0`) |

### 여러 얼굴과 내부 출력 구조

한 frame의 모든 YuNet face detection을 처리하고 detection 순서로 `Face 0`, `Face 1`을 매긴다. 각 frame에서 새로 매기는 인덱스라 frame 사이 ID 지속성이 없다. **Face index는 Survivor ID가 아니다.** Stage 4 전에는 두 체계를 연결하지 않는다.

`survivor_eye_state_node.py`에는 `EyeStateFrameResult`, `FaceEyeStateResult`, `EyeStateSideResult` immutable dataclass가 있다. 결과는 이미지 timestamp(sec/nanosec), 얼굴 frame-local index/bbox/confidence, 좌우별 valid/state/confidence/ROI bbox/invalid reason을 가진다. `latest_result` property로 최신 결과를 node 내부에서 보관한다. 아직 raw message topic은 발행하지 않는다. 미래 소비자·장기 schema가 정해지지 않은 상태에서 `rescue_robot_interfaces`에 임시 public message를 추가하지 않았다. 현재 ROS 출력은 debug image 한 개다.

### Debug image와 확인 명령

Debug image에는 face bbox/index/confidence, L/R ROI box, `OPEN/CLOSED/LOW_CONFIDENCE`와 confidence, `INVALID (reason)`, 선택된 경우 landmark를 표시한다. 얼굴이 없으면 `NO_FACE`를 표시한다. 별도 rqt를 자동 실행하지 않는다.

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view --ros-args \
  -r image:=/leader/survivor/eye_state/debug_image
```

`rqt_image_view` 창이 뜨면 topic 선택 메뉴에서 `/leader/survivor/eye_state/debug_image`를 선택한다. GUI 자체는 이번 실물 시험에서 실행하지 않았다.

실제 D435에서 받은 동일 frame과 debug image는 시험 중 `/tmp/eye_state_goal4_gpu_matched/`에 저장했다. 유효한 36×36 crop 두 장은 `data/eye_state_d435_debug/`에 로컬 보관하며 Git에서 제외된다. 사용자가 이 frame의 양쪽 눈을 OPEN이라고 확인했으므로 이 표본의 수동 ground truth는 양쪽 OPEN이다. 자동 수집·학습셋 복사는 하지 않았다. 다른 캡처의 prediction은 ground truth로 간주하지 않는다.

## F. 한 frame의 처리 순서

1. ROS callback이 `sensor_msgs/msg/Image`를 받는다. QoS는 `KEEP_LAST`, depth 1, `BEST_EFFORT`, `VOLATILE`이다.
2. callback은 메시지 하나짜리 latest-frame slot만 교체하고 worker를 깨운다. inference 중 들어온 오래된 frame을 큐에 쌓지 않는다.
3. worker가 `inference_rate_hz` 간격으로 가장 최근 frame을 꺼낸다. 기본 제한은 5 Hz다.
4. `cv_bridge`가 이미지를 BGR8로 변환한다.
5. YuNet이 여러 얼굴의 bbox, confidence, 5점 landmark를 추출한다.
6. 각 얼굴에서 눈 중심 간 거리로 subject-left/right ROI 크기와 위치를 계산하고 boundary/크기/confidence 조건을 검사한다.
7. invalid ROI는 classifier에 전달하지 않는다. valid ROI는 설정한 gray 또는 rgb preprocessing을 거친다.
8. YOLO11n-cls가 valid한 양쪽 eye crop들을 한 번에 추론한다. class name과 class probability를 checkpoint mapping에 따라 변환한다.
9. timestamp, face, 좌우 validity/state/confidence를 내부 결과 dataclass에 보관한다.
10. 원본 frame에 얼굴·ROI·state/confidence 또는 invalid reason을 그린다.
11. 입력 header를 유지해 전용 debug image topic으로 publish한다.

Classifier/FaceDetector가 frame 처리 중 오류를 내면 worker는 subscription을 유지하고 오류를 제한적으로 log한다. Model load 오류는 startup 실패로 처리된다.

## G. 좌표와 ID 관계

`Face 0`, `Face 1`은 해당 frame에서 detector가 반환한 얼굴의 배열 index일 뿐이다. 같은 사람도 다음 frame에서 다른 index를 가질 수 있고, 여러 사람의 index 순서가 뒤바뀔 수 있다. `Survivor #1`, `Survivor #2`는 기존 Survivor Registry가 관리하는 persistent ID다.

Stage 2는 두 식별 공간을 연결하지 않는다. Stage 4에서 Registry가 만든 ID를 authority로 사용하고, camera/map 좌표 등 검증 가능한 observation association을 별도로 설계한다. Eye State가 Survivor ID를 만들거나 Registry를 수정하지 않는다.

## H. 오류·실패 동작

| 상황 | 현재 동작 |
| --- | --- |
| 얼굴 없음 | 얼굴 목록이 비어 있고 debug image에 `NO_FACE`; classifier 호출 없음 |
| 한쪽 눈 landmark가 frame 밖/누락 | inter-eye 거리를 만들 수 없어 현재 구현은 양쪽 ROI를 `invalid_eye_landmark`로 invalid 처리 |
| ROI가 image 밖에 일부 걸침 | clamp; 자른 비율이 25% 초과면 `severe_frame_clipping`; 이내면 edge replicate |
| ROI 한 변이 최소 크기 미달 | `roi_below_minimum_size`, classifier에 전달하지 않음 |
| face confidence 낮음 | detector threshold 아래 얼굴은 검출에서 제외; 변환 단계의 confidence gate도 존재 |
| classifier confidence 낮음 | valid eye의 state를 `LOW_CONFIDENCE`로 표시하고 confidence 보존 |
| 옆얼굴/회전 | 별도 pose gate는 없음; landmark/ROI 품질에 따라 invalid 또는 잘못된 crop 가능. 실물 검증 필요 |
| motion blur/가림/저조도/안경 | 별도 blur·occlusion 품질 모델 없음. confidence만으로 신뢰를 보장하지 않음 |
| 여러 얼굴 | 한 frame에서 모두 독립 처리하고 frame-local `Face N`으로 표시 |
| checkpoint 또는 YuNet 파일 없음 | 파일 확인/model 초기화 시 오류를 출력하고 node 시작 실패 |
| PyTorch/Ultralytics/OpenCV API 없음 | startup 오류. 검증된 YOLO runtime image에서 실행 |
| camera topic 없음 | node는 subscription을 대기하며 현재 별도 topic-timeout 진단이나 debug heartbeat는 없음. 입력/debug message가 나타나지 않음 |
| frame 변환/inference 오류 | 오류 log 후 worker가 다음 최신 frame 처리를 계속 시도 |
| node 종료 | 실물 시험에서는 SIGINT 종료가 한 번 정상 완료됐고, 다른 한 번은 ROS debug 출력 후 종료 중 `KeyboardInterrupt` traceback과 exit code `-2`가 남았다. 장시간 운용 전 종료 경로를 다시 확인해야 한다. |

## I. 실행 방법

아래는 repository에 있는 실제 package 및 `realsense2_camera/rs_launch.py` 이름을 사용한다. D435를 점유 중인 다른 driver가 있으면 그 driver를 먼저 정상 종료해야 한다. VSLAM/Nav2/nvblox는 Eye State 확인만을 위해 실행할 필요가 없다.

실행 전 Stage 1에서 만든 `~/eye_state_runs/eye_state/baseline/weights/eye_state_best.pt`가 있어야 한다. YuNet 파일이 없으면 저장소의 고정 revision 다운로드 도구로 준비한다. 이 파일은 SHA-256 검증 후 Git ignore 대상 경로에 저장된다.

```bash
cd ~/damgc_robot
python3 tools/eye_state/fetch_yunet_model.py \
  --output data/eye_state_models/face_detection_yunet_2022mar.onnx
```

### 1. package 빌드

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select rescue_robot_eye_state
```

이 package의 isolated install prefix는 보통 `install/rescue_robot_eye_state`다. Jetson host에 Ultralytics/PyTorch가 없는 현재 환경에서는 검증된 `damgc-survivor-yolo:humble` runtime에서 node를 실행한다. 기존 YOLO runtime image의 torch/torchvision/Ultralytics를 upgrade하지 않는다.

### 2. camera-only 실행

첫 terminal에서 D435 RGB만 켠다. 다음 명령은 depth/infra/TF와 전체 VSLAM·Nav2 stack을 시작하지 않는다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=leader camera_name:=camera \
  enable_color:=true rgb_camera.color_profile:=640,480,30 \
  enable_depth:=false enable_infra:=false \
  enable_infra1:=false enable_infra2:=false publish_tf:=false
```

다른 terminal에서 실제 입력 topic과 frame rate를 확인한다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list -t | grep '/leader/camera/color/image_raw'
ros2 topic hz /leader/camera/color/image_raw
```

### 3. Eye State node 실행

두 번째 container에서 ROS와 image를 연결하려면 기존 프로젝트의 Fast DDS 환경과 ROS domain을 맞춘다. 아래 mount의 model/YuNet 경로와 isolated install prefix를 장비 경로에 맞춘다.

```bash
cd ~/damgc_robot
docker run --rm -it --runtime=nvidia --network host --ipc=host \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e YOLO_CONFIG_DIR=/tmp \
  -v "$PWD/install/rescue_robot_eye_state:/eye_state_install/rescue_robot_eye_state:ro" \
  -v "$HOME/eye_state_runs/eye_state/baseline/weights/eye_state_best.pt:/models/eye_state_best.pt:ro" \
  -v "$PWD/data/eye_state_models/face_detection_yunet_2022mar.onnx:/models/yunet.onnx:ro" \
  --entrypoint bash damgc-survivor-yolo:humble -lc '
    source /opt/ros/humble/setup.bash
    export AMENT_PREFIX_PATH=/eye_state_install/rescue_robot_eye_state:/opt/ros/humble
    ros2 launch rescue_robot_eye_state survivor_eye_state.launch.py \
      image_topic:=/leader/camera/color/image_raw \
      model_path:=/models/eye_state_best.pt \
      yunet_model_path:=/models/yunet.onnx \
      inference_rate_hz:=5.0 preprocess_mode:=gray device:=cpu
  '
```

실제 검증은 다른 Docker workload가 없는 상태에서 카메라-only RGB stream을 사용했다. 별도 YOLO runtime container에서 CPU와 GPU device 0을 실행했고 GPU 실행은 성공했다. 측정된 짧은 시험은 `inference_rate_hz=2.0`이므로 기본 5 Hz 장시간 운용을 검증한 것은 아니다. GPU 추론은 VSLAM 등 다른 GPU workload와 함께 검증하지 않았다.

### 4. topic, node, image 확인

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 node info /survivor_eye_state_node
ros2 topic list -t | grep -E 'camera/color/image_raw|eye_state/debug_image'
ros2 topic hz /leader/survivor/eye_state/debug_image
ros2 run rqt_image_view rqt_image_view --ros-args \
  -r image:=/leader/survivor/eye_state/debug_image
```

## J. 성능 및 자원 측정

2026-10-03 실제 장비에서 640×480, 30 Hz color-only D435 stream을 확인했다. Eye State node는 `damgc-survivor-yolo:humble`, CUDA device 0, `inference_rate_hz=2.0`으로 실행했다. 8개 debug message의 유효 처리 간격은 중앙값 0.631초(약 1.594 Hz)였다. 입력 image header timestamp부터 debug message 수신까지 end-to-end 지연은 중앙값 196.80 ms, 최소 154.19 ms, 최대 1233.58 ms였다. 표본이 짧고 최대 지연 편차가 커서 안정된 처리율 보증으로 해석하지 않는다.

같은 D435 frame을 별도 프로세스에서 한 번씩 추론한 결과, gray mode는 얼굴 검출·ROI 94.69 ms, 두 눈 classifier batch 1027.75 ms, 전체 1122.45 ms였고 rgb mode는 각각 97.57 ms, 941.12 ms, 1038.69 ms였다. 이 값에는 단일 이미지 프로세스의 첫 inference 준비 비용이 포함될 수 있으므로 위 ROS 연속 처리 지연과 직접 비교하지 않는다. ROS node 내부 개별 FaceDetector/classifier 구간, CPU/GPU 사용률·메모리, 전력·열, 장시간 처리율은 **NOT MEASURED**다. 짧은 2 Hz 설정 시험을 5 Hz 장시간 운용이나 다른 GPU workload와 동시 운용 성능으로 일반화하지 않는다.

## K. 제한과 domain gap

- 공개 training image는 grayscale 계열이며 D435 입력은 RGB다. 같은 양쪽 OPEN D435 frame에서 gray mode는 왼쪽을 CLOSED(단일 이미지 재추론 confidence 0.9972), 오른쪽을 OPEN(0.9914)으로 예측했다. 실제 ROS overlay도 L:CLOSED, R:OPEN이었다. rgb mode로 같은 frame을 단독 추론했을 때 양쪽 모두 CLOSED(0.9076, 0.9360)로 나와 두 눈 모두 오분류했다. 이는 1개 frame의 진단 결과이며 전처리 mode 선택이나 일반 성능을 결정하기에는 부족하다. 기본값 gray를 유지한다.
- 거리 증가 시 face bbox와 inter-eye distance가 줄어든다. 24 px 기준 미달 crop은 invalid가 되어 먼 거리에서 classifier 판단을 하지 않는다.
- 옆얼굴, 눈 감김/가림, 안경 반사, 저조도, 역광, motion blur는 충분히 검증되지 않았다.
- Roboflow 파일명 접두사는 split 간 반복된다. 그 값이 subject ID인지 확인되지 않아 subject overlap 가능성이 남는다.
- 공개 split의 test 정확도 100%는 그 split에 대한 결과일 뿐 실제 사람·D435·현장 성능이 아니다.
- debug overlay 및 confidence는 분류 품질의 독립적인 ground truth가 아니다. D435 sample을 fine-tuning에 넣으려면 사람이 눈 상태를 직접 확인하고 subject 단위로 split해야 한다.
- 실제 사람 얼굴을 D435로 촬영해 classifier 연결까지 확인했다. 한 frame에서 YuNet face confidence 0.9953, face bbox 110×144 px, 좌우 ROI 각각 36×36 px였다. 사용자는 해당 frame의 눈이 양쪽 OPEN이라고 확인했다. gray mode에서 왼쪽은 CLOSED로 오분류되고 오른쪽은 OPEN으로 맞았다. 태블릿의 작고 가려진 얼굴 사진에서는 ROI가 21×21 px라 invalid 처리되고 classifier 호출을 생략했다. 얼굴/조명 조건과 거리 등은 통제되지 않았으며 한 사람·한 시점 결과다.
- `CLOSED`/`OPEN`은 관찰된 눈 모양만을 뜻하며 의식 또는 생존 여부 판단이 아니다.

### 실물 D435 확인 기록

| 시험 | 입력 및 ground truth | 출력 | 판정 |
| --- | --- | --- | --- |
| 작은 화면 사진 | D435 RGB, face conf 0.8817, ROI 21×21 px, 수동 눈 label 없음 | 양쪽 `roi_below_minimum_size`, classifier 미호출 | 품질 gate 동작 확인 |
| 실제 사람, 유효 ROI | 640×480 D435 RGB, 얼굴 bbox 110×144, face conf 0.9953, L/R ROI 36×36. 사용자가 두 눈 OPEN 확인 | gray ROS node: L `CLOSED` 약 0.95, R `OPEN` 약 1.00. 같은 frame 단독 gray: L `CLOSED` 0.9972, R `OPEN` 0.9914 | 왼쪽 오분류, 오른쪽 정답 |
| 동일 frame RGB 전처리 | 위와 같은 paired D435 frame, 양쪽 OPEN | standalone: L `CLOSED` 0.9076, R `CLOSED` 0.9360 | 양쪽 오분류. 한 장으로 mode 우열을 확정하지 않음 |

debug ROS image message 수신과 저장 frame 확인은 완료했다. `rqt_image_view` GUI 자체는 실행하지 않았다 (**NOT RUN**). 양쪽 눈 CLOSED 유지, blink, 좌/우·상/하 얼굴 회전, 안경 유무 비교, 0.5/1.0/1.5/2.0 m 거리 시험은 **NOT RUN**이다.

## L. 후속 개발 순서

1. **Stage 3 — Temporal History:** frame별 raw L/R 결과를 약 3초 유지한다. 5 Hz라면 약 15 sample이지만 이는 설계 출발점일 뿐이다. blink와 지속 감김 구분, `OPEN/CLOSED/UNKNOWN` threshold는 실물 자료와 failure 비용을 고려해 정한다.
2. **Stage 4 — Survivor ID Association:** 새 ID를 만들지 않는다. 기존 Survivor Registry의 persistent Survivor #ID를 authority로 사용하고 시간·영상·공간 정보의 association 규칙을 검증한다.
3. **Stage 5 — rqt/RViz Integration:** rqt에는 Survivor #ID와 Eyes 상태를, RViz에는 기존 Survivor marker/text 주변에 Eye State observation을 표시하는 방식을 검토한다.
4. **Stage 6 — D435 Dataset and Fine-tuning:** 실제 ROI를 수집하고 사람이 OPEN/CLOSED label을 검수한다. subject-separated train/val/test를 만든 뒤 domain gap이 확인될 때 fine-tune하고 CLOSED recall을 평가한다.
5. 그 이후 별도 cue로 **YOLO Pose → posture → activity**를 단계적으로 검토한다. Eye State와 pose는 독립된 관찰 cue로 유지하고, 이후 검증된 규칙을 통해 multi-cue Survivor visual state로 결합할 수 있다.

consciousness 판단, 임상 진단, 단일 classifier output으로 Survivor ID 또는 Registry state를 바꾸는 동작은 이 개발 계획의 범위가 아니다.

## 현재 완료 판정

- **Stage 1: COMPLETE.** 40/40 epoch 학습, checkpoint 저장, 공개 test split 평가 및 standalone inference 기록이 있다.
- **Stage 2 구현/build: COMPLETE.** 독립 ROS package/node, parameter launch, YuNet/ROI/classifier 코드와 전용 debug topic을 구현했고 package 단독 build와 synthetic ROS message 전달을 확인했다.
- **Stage 2 실물 연결: COMPLETE. 정확도/조건 검증: PARTIAL.** 실제 D435에서 얼굴→landmark→유효한 36×36 양쪽 ROI→YOLO classifier→ROS debug image 흐름을 확인했다. 사용자가 양쪽 OPEN으로 확인한 한 frame에서 gray mode 왼쪽은 오분류했고 rgb mode는 양쪽을 CLOSED로 예측했다. 작은 얼굴의 INVALID gate도 확인했다. 통제된 CLOSED, blink, 자세, 거리 시험은 수행하지 않았다.
- **2차 전체 판정: PARTIAL.** CLOSED, blink, pose 변화, 거리별 검증과 실제 valid eye crop 성능이 남아 있다.
