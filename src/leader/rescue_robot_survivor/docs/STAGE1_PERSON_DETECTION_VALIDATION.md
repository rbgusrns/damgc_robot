# Stage 1 Jetson/D435 검증 가이드

## 판정 기준

현재 코드는 **IMPLEMENTED - HARDWARE VERIFICATION REQUIRED**다. 아래 절차로 실제 사람
bounding box를 사용자가 확인하기 전까지 `VERIFIED`로 기록하지 않는다. 각 터미널은 새
shell이며 공통으로 ROS와 workspace overlay를 source한다.

## Survivor Docker runtime check

Host Python에는 `torch`, `torchvision`, `ultralytics`를 설치하지 않는다. 먼저 전용 image를
build하고 CUDA runtime을 확인한다.

```bash
cd ~/damgc_robot
./scripts/build_survivor_runtime.sh
docker run --rm --runtime=nvidia --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  -e YOLO_CONFIG_DIR=/tmp \
  -v ~/.cache/damgc-survivor-ultralytics:/root/.cache/ultralytics \
  --workdir /root/.cache/ultralytics \
  damgc-survivor-yolo:humble 'check_survivor_ai_runtime --static'
```

정상 결과에는 `CUDA: True`, `CUDA NMS: OK`, `YOLO load: OK`, `GPU inference: OK`와
`rescue_robot_survivor person_detector_node`가 포함되어야 한다.

## 2026-09-11 개발 검증 기록

- 신규 package 단독 build와 ROS package/executable/launch discovery 성공.
- unit/node/launch 테스트 12개 통과, flake8/pydocstyle/diff check 통과.
- host Python에는 torch/torchvision/Ultralytics가 없음.
- 기존 Isaac ROS container의 NVIDIA `torch 2.5.0a0+nv24.08`과 CUDA 12.6을 격리
  runtime으로 사용하고, `torchvision 0.20.0`을 Orin sm_87용으로 source build했다.
- 격리 runtime의 Ultralytics 8.4.147에서 CPU/GPU NMS와 YOLO11n GPU 추론 성공.
- 공개 bus sample에서 person 4명을 검출하고 왼쪽부터 `person1`, `person2`, `person3`,
  `person4`와 confidence/bounding box가 그려진 ROS debug image를 확인했다.
- 실제 D435 빈 장면에서는 예상대로 box가 없었고 debug image가 약 14 Hz로 발행됐다.
- 실제 D435 입력 55개와 debug 출력 50개를 5초간 수집했으며 48개 output stamp가 입력과
  정확히 일치했다. frame은 `camera_color_optical_frame`, 출력 encoding은 `bgr8`이었다.
- 같은 실행에서 raw depth는 약 27 Hz, aligned depth는 약 29 Hz로 확인됐다.

위 기록은 코드와 장비 파이프라인 검증 근거지만 D435 앞의 실제 사람/마네킹 검증을
대체하지 않는다. 따라서 상태는 계속 `IMPLEMENTED - HARDWARE VERIFICATION REQUIRED`다.

## 사전 점검 및 build

```bash
cd ~/damgc_robot
git status
git branch --show-current
git log -10 --oneline

python3 --version
python3 -c "import torch; print('torch:', torch.__version__); print('CUDA available:', torch.cuda.is_available()); print('torch CUDA:', torch.version.cuda)"
python3 -c "import torchvision; print('torchvision:', torchvision.__version__)"
python3 -c "import ultralytics; print('ultralytics:', ultralytics.__version__)"

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_survivor
source install/local_setup.bash
ros2 pkg prefix rescue_robot_survivor
ros2 pkg executables rescue_robot_survivor
ros2 launch rescue_robot_survivor person_detector.launch.py --show-args
```

Expected result: branch와 변경사항을 확인하고 세 AI import가 모두 성공하며, build가 성공하고
`person_detector_node`와 5개 launch argument가 나타난다.

Failure symptoms/troubleshooting: import가 실패하면 detector는 실행할 수 없다. JetPack 6.2.3
호환 NVIDIA PyTorch와 torchvision을 먼저 준비한다. `pip install --upgrade torch`,
`pip uninstall torch` 같은 일반 교체 명령은 실행하지 않는다. build만 실패하면 첫 error부터
수정하고 `build/install/log`를 삭제하지 않는다.

호환성 판단에는 [NVIDIA 공식 Jetson PyTorch 문서](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)와
[Ultralytics 공식 Jetson 문서](https://docs.ultralytics.com/guides/nvidia-jetson/)를 사용한다.
현재 Ultralytics 문서의 명시적 native 조합은 JetPack 6.1 기준이므로 JetPack 6.2.3 장비에
추측하여 설치하지 않는다. 검증된 6.2.3 wheel 조합 또는 격리된 JetPack 6 container가
확정되기 전에는 위 build/static test까지만 수행한다.

## Terminal 1 — D435 RGB + depth + aligned depth

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
rs-enumerate-devices -s
ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_depth:=true \
  enable_sync:=true \
  align_depth.enable:=true
```

Expected result: D435가 USB 3.x로 인식되고 `leader.camera`가 color/depth 640×480@30을 열며
`RealSense Node Is Up!`이 출력된다. 이 기존 launch는 AprilTag도 실행하지만 접근 제어는
기본 `false`다.

Failure symptoms/troubleshooting: 장치가 없으면 cable/전원/USB와 다른 RealSense process를
확인한다. bandwidth 오류면 USB 3.x 연결과 다른 고대역폭 노드를 확인한다. 기존 RealSense
launch가 상위 custom argument에 대해 출력하는 unsupported warning은 알려진 기존 경고다.

## Terminal 2 — RGB/depth metadata와 rate

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 topic list | grep camera
ros2 topic list | grep depth
ros2 topic info -v /leader/camera/color/image_rect
ros2 topic info -v /leader/camera/depth/image_rect_raw
ros2 topic info -v /leader/camera/aligned_depth_to_color/image_raw
ros2 topic info -v /leader/camera/color/camera_info
ros2 topic hz /leader/camera/color/image_rect
ros2 topic hz /leader/camera/depth/image_rect_raw
ros2 topic hz /leader/camera/aligned_depth_to_color/image_raw
```

각 `topic hz`는 확인 후 `Ctrl-C`하고 다음 명령으로 넘어간다. 별도 터미널에서 다음 metadata도
한 건씩 확인한다.

```bash
ros2 topic echo --once /leader/camera/color/image_rect --field header
ros2 topic echo --once /leader/camera/color/image_rect --field encoding
ros2 topic echo --once /leader/camera/depth/image_rect_raw --field encoding
ros2 topic echo --once /leader/camera/aligned_depth_to_color/image_raw --field header
ros2 topic echo --once /leader/camera/aligned_depth_to_color/image_raw --field encoding
ros2 topic echo --once /leader/camera/color/camera_info --field header
```

Expected result: 조사 시 RGB는 `rgb8`, raw/aligned depth는 `16UC1`, 모두 640×480이었다.
RGB/aligned depth frame은 `camera_color_optical_frame`, raw depth는
`camera_depth_optical_frame`이었다. raw/aligned depth는 약 28–29 Hz였다. 실행 부하에 따라
수치는 달라질 수 있으므로 이번 실행값을 기록한다.

Failure symptoms/troubleshooting: aligned 토픽만 없으면 Terminal 1의 두 override 철자와
`ros2 launch ... --show-args`를 확인한다. RGB는 있는데 detector callback이 없다면 topic
철자와 `ros2 topic info -v`의 endpoint/QoS를 비교한다.

## Terminal 3 — Container person detector

```bash
cd ~/damgc_robot
./scripts/run_survivor_detector.sh
```

Expected result: model, input/output topic, threshold와 selected device가 한 번씩 출력된다.
프레임마다 INFO 로그는 출력되지 않는다. 처음 `yolo11n.pt`를 사용할 때는 weight 다운로드가
필요할 수 있다.

Failure symptoms/troubleshooting:

- `ultralytics` import error: 같은 `python3`에서 import되는지 확인한다.
- model load/download 실패: 네트워크/cache를 확인하거나 로컬 weight 절대 경로를 전달한다.
- CUDA false: JetPack용 torch 여부를 확인하고 `device:=cpu`로 기능을 우선 검증한다.
- FPS가 낮음: `nvidia-smi`, 입력/output hz, 다른 AprilTag/SLAM 부하를 비교한다.

파라미터 확인 명령:

```bash
ros2 node list | grep person_detector
ros2 param list /leader/person_detector
ros2 param get /leader/person_detector image_topic
ros2 param get /leader/person_detector debug_image_topic
ros2 param get /leader/person_detector model_name
ros2 param get /leader/person_detector confidence_threshold
ros2 param get /leader/person_detector device
```

## Terminal 4 — Survivor output

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 topic list | grep survivor
ros2 topic info -v /leader/survivor/debug_image
ros2 topic hz /leader/survivor/debug_image
ros2 topic echo --once /leader/survivor/debug_image --field header
```

Expected result: publisher가 하나 나타나고 debug rate가 출력되며 header stamp/frame이 입력
RGB와 동일한 원본 camera header를 사용한다.

Failure symptoms/troubleshooting: debug topic이 없으면 Terminal 3 process와 fatal 로그를
확인한다. 토픽은 있지만 message가 없으면 입력 topic endpoint, QoS와 inference 오류를
확인한다. subscriber는 `BEST_EFFORT`, `VOLATILE`과 호환되어야 한다.

## Terminal 5 — rqt_image_view

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run rqt_image_view rqt_image_view
```

GUI의 topic 목록에서 `/leader/survivor/debug_image`를 선택한다. 직접 지정도 가능하다.

```bash
ros2 run rqt_image_view rqt_image_view /leader/survivor/debug_image
```

Expected result: 사람이 없으면 box 없는 color 영상, 사람이 있으면 각 사람에 초록색 box와
`personN 0.xx`가 표시된다.

Failure symptoms/troubleshooting: blank 화면이면 Terminal 4의 rate를 먼저 확인하고 rqt topic을
다시 선택한다. `ros2 topic info -v`에서 rqt subscriber가 생성됐는지 확인한다. remote desktop
환경이면 `DISPLAY`와 Qt 오류도 확인한다.

## 실물 검출 테스트 기록

| Test | 장면 | Expected result |
| --- | --- | --- |
| A | 사람 없음 | box 없음 |
| B | 사람 1명 | `person1`, confidence, box |
| C | 사람 2명 | 왼쪽 `person1`, 오른쪽 `person2` |
| D | 사람 3명 이상 | 왼쪽부터 `person1..N` |
| E | 사람 이동/교차 | box가 현재 위치를 따름; 번호 변경 허용 |
| F | 책상·상자·의자만 | person box 없음 |
| G | 일부만 보이는 사람 | 검출 성공률과 조건 기록 |
| H | 최종 시연 마네킹 | 성공/실패와 거리·조명·자세 기록 |

여러 사람 중 일부가 누락되면 threshold, 조명, 사람 크기/가림과 입력 해상도를 기록한다.
마네킹 실패는 COCO model의 known limitation으로 기록하며 Stage 1에서 custom training을
시작하지 않는다. `personN`은 frame-local 번호이므로 프레임 간 동일 ID를 기대하지 않는다.

## 종료

Terminal 5→3→1 순서로 각 process에 `Ctrl-C`를 입력한다. 종료 후 다음을 확인한다.

```bash
ros2 node list
git status
```

카메라, detector와 rqt가 종료되었는지 확인한다. 기존 `camera_info_qos_bridge.py`는 launch
종료 시 중복 shutdown traceback을 출력할 수 있으며 이는 기존 알려진 문제다.

## Troubleshooting checklist

1. **Camera topic이 없음**
   - `rs-enumerate-devices -s`, `ros2 node list | grep camera`와 Terminal 1 로그를 확인한다.
2. **RGB는 있지만 detector가 image를 못 받음**
   - `ros2 param get /leader/person_detector image_topic`과 실제 topic 철자를 비교한다.
3. **QoS mismatch**
   - `ros2 topic info -v /leader/camera/color/image_rect`에서 publisher와 detector endpoint가
     `BEST_EFFORT`, `VOLATILE`로 연결 가능한지 확인한다.
4. **Ultralytics import error**
   - `which python3`, `python3 -m pip show ultralytics`와 `python3 -c "import ultralytics"`를
     같은 sourced shell에서 실행한다.
5. **YOLO model download/load 실패**
   - 네트워크와 파일 권한을 확인하거나 `model_name:=/absolute/path/yolo11n.pt`를 사용한다.
6. **CUDA가 false**
   - `python3 -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"`
     와 `nvidia-smi`를 확인한다. 기능 확인은 `device:=cpu`로 가능하지만 generic torch를
     설치하지 않는다.
7. **rqt_image_view가 blank**
   - output hz, rqt subscriber endpoint, 선택 topic과 `DISPLAY`/Qt 오류를 확인한다.
8. **Debug topic이 없음**
   - `ros2 node list`, detector fatal 로그, `ros2 topic list | grep survivor`를 확인한다.
9. **FPS가 지나치게 낮음**
   - RGB와 debug hz를 나란히 측정하고 GPU 선택, `nvidia-smi`, AprilTag/SLAM 부하를
     기록한다.
10. **사람을 검출하지 못함**
    - color 원본 화질, 조명, 거리, 가림을 확인하고 threshold를 소폭 낮춰 비교하되 Stage 1
      범위에서 training을 시작하지 않는다.
11. **여러 사람 중 일부만 검출됨**
    - 각 사람의 pixel 크기/가림과 confidence를 기록하고 위치를 바꿔 COCO model 한계인지
      확인한다.
12. **마네킹 검출 실패**
    - 자세·거리·조명과 실패 영상을 Known Limitations에 기록한다. custom model은 별도
      후속 Goal로만 검토한다.
13. **Aligned depth topic이 없음**
    - `ros2 launch rescue_robot_bringup camera_apriltag.launch.py --show-args`에서
      `align_depth.enable`, `enable_sync`를 확인하고 Terminal 1 명령으로 camera를 재시작한다.

## Stage 2에 남길 결과

검증 기록에 bbox 좌표 기준, RGB와 aligned-depth timestamp, aligned-depth topic,
`/leader/camera/color/camera_info`, 실제 resolution, frame ID와 encoding을 남긴다. 다음
Stage는 bbox 중심 주변 ROI에서 유효 depth를 추출하고 zero/invalid 제거 후 median
person distance[m]를 계산한다. Camera XYZ, TF, marker와 tracking은 그 이후 범위다.
