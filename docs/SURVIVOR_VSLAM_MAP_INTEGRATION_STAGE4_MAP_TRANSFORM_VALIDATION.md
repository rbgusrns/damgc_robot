# Survivor–VSLAM Map Integration Stage 4: Phase A 사전 검증

## 범위와 판정

2026-09-14 Jetson Orin Nano + D435에서 기존 VSLAM, dual EKF, nvblox,
survivor preprocessing, YOLO detector를 동시에 실행하여 `camera_positions`의
frame/timestamp 계약과 `map`까지의 TF 연결을 확인했다. **Phase A: PASS**.
`/goal 2`의 exact-timestamp transform node 구현으로 진행할 수 있다.

이번 goal에서는 transform node, launch, 설정, TF architecture 및 기존 코드를
수정하지 않았다. 이 파일만 추가했다. `/leader/survivor/map_positions`는 아직 없다.
Phase A의 PASS는 map 변환의 기술적 사전 조건을 뜻한다. 고정된 사람의 이동 중
map 좌표 안정성이나 map 좌표의 절대 정확도를 증명한 것은 아니다.

## 시작 상태와 인계된 구조

| 항목 | 실제 확인값 |
| --- | --- |
| branch | `main` |
| short commit | `db1e715 feat(survivor): integrate camera preprocessing with VSLAM pipeline` |
| full commit | `db1e71543236bd94328b33c0dd68634a08c97e9c` |
| 초기 `git status` | clean; 기존 uncommitted changes 없음 |
| 장비 | `aarch64` Jetson, `lsusb`에서 `8086:0b07 Intel Corp. RealSense D435` |
| 시작 ROS graph | `/parameter_events`, `/rosout`만 존재 |
| 실행 컨테이너 | 기존 `damgc-vslam-mapping:humble`, `damgc-survivor-yolo:humble` 이미지 |

조사한 현재 코드: `person_detector_node.py`, `person_detector.launch.py`,
`survivor_camera_processing.launch.py`, `scripts/run_survivor_detector.sh`,
`scripts/run_vslam_mapping.sh`, `visual_slam_nvblox_realsense.launch.py`,
`visual_slam_realsense.launch.py`, `localization.launch.py`,
`nvblox_realsense.launch.py`. Stage 1/2 통합 validation 문서와 survivor Stage
1/2/3 validation 문서도 확인했다. 과거 문서는 인계 정보로만 사용했고 아래
판정은 이번 live message/TF/rosbag 증거를 우선한다.

기존 실행기는 호스트의 STM32 bridge와 **한 개의** `realsense2_camera`를 시작한다.
D435 infra1/infra2는 Isaac ROS Visual SLAM에, color/depth는 nvblox와
survivor pipeline에 공급된다. `visual_slam_nvblox_realsense.launch.py`가
localization, VSLAM, nvblox launch를 포함한다. local EKF는
`odom → base_link`, global EKF는 `map → odom`을 소유한다. VSLAM의
`publish_odom_to_base_tf`와 `publish_map_to_odom_tf`는 모두 `False`다.
`robot_state_publisher_vslam`이 URDF의 `base_link → camera_link`, 기존
RealSense driver가 `camera_link → camera_color_optical_frame`을 발행한다.
`camera_apriltag.launch.py`는 이번 실행에 사용하지 않았다.

## 실제 `camera_positions` 계약

`person_detector_node.py`는 `/leader/camera/color/image_rect`의 RGB callback에서
`geometry_msgs/msg/PoseArray`를 한 frame당 한 번 만든다. 코드의
`positions.header = source_message.header`가 원본 RGB header 전체를 복사한다.
따라서 `header.stamp`는 YOLO 완료 시각이 아닌 해당 rectified RGB image의
capture timestamp이고, `header.frame_id`는 검증된 RGB optical frame이다.
RGB/CameraInfo frame 불일치 시 유효 XYZ를 만들지 않는다.

각 `Pose.position`은 rectified RGB의 `CameraInfo.P`, bbox 중앙 ROI와 aligned
depth median Z로 계산한 meter 단위 camera optical XYZ다. orientation은 측정된
사람의 방향이 아니며 identity quaternion `(0,0,0,1)`로 설정된다. 여러 사람은
현재 frame에서 왼쪽→오른쪽 순서로 처리하지만 유효 XYZ만 `poses`에 들어간다.
따라서 array index는 지속 ID 또는 화면의 `personN`과 항상 같지 않다.
publisher 코드의 QoS는 reliable/volatile/keep-last 1이고,
`ros2 topic info -v`에서도 reliable/volatile을 확인했다. CLI는 history depth를
`UNKNOWN`으로 표시했으므로 depth 1은 현재 코드에 근거한다.

실제 사람 1명의 live sample:

```yaml
header:
  stamp: {sec: 1789367640, nanosec: 74820557}
  frame_id: camera_color_optical_frame
poses:
- position: {x: 0.4975316581865145, y: 0.1518972069026802, z: 2.761000156402588}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

다른 one-shot에서도 한 개의 유효 pose와 nonzero stamp가 발행됐다. 13초 동안
rectified RGB와 positions를 동시에 구독한 읽기 전용 probe는 RGB 105개,
positions 70개, non-empty positions 67개를 받았다. 이 중 42개 position stamp가
probe가 수신한 RGB stamp와 **정확히 동일**했다. 나머지 25개는 probe의 RGB
best-effort 수신 누락 가능성이 있어 stamp 불일치 증거로 해석하지 않는다.
원본 header 직접 복사는 위 코드에서도 확인된다.

Stage 4 다음 구현의 입력 계약은 다음과 같다.

```text
target_frame = map
source_frame = msg.header.frame_id  # 이번 실측: camera_color_optical_frame
time         = msg.header.stamp     # 원본 rectified RGB capture timestamp
```

YOLO/depth 처리 중 로봇이 움직일 수 있으므로 최신 TF는 촬영 시점의 camera pose를
대표하지 않는다. 다음 goal의 transform node는 위 stamp의 TF만 사용하고,
조회 실패 시 map position을 만들지 않아야 한다.

## TF tree와 exact-stamp 조회 결과

실제 `tf2_echo`에서 아래 네 구간이 각각 연결됐다.

```text
map → odom → base_link → camera_link → camera_color_optical_frame
  global EKF  local EKF      URDF          RealSense
```

`map → base_link`와 `map → camera_color_optical_frame` 조회도 성공했다.
관찰 중 `map → base_link`는 약 `(-0.131, 0.097, 0.000) m`, yaw 약
`-54.7°`; `base_link → camera_color_optical_frame` translation은 약
`(0.042, 0.025, 0.130) m`였다. `base_link → camera_link`의 URDF
translation은 `(0.042, 0.010, 0.130) m`이다. 별도 `tf2_echo` 시작 직후의
`Invalid frame ID`는 listener cache 준비 전 출력이며, 동일 명령에서 이어서
transform을 수신했다.

`tf2_echo`는 최신 TF 확인이므로 detection 시점 사용 가능성을 따로 검증했다.
동일 ROS node가 live `PoseArray`와 TF listener를 받아 buffer에 쌓은 뒤,
`lookup_transform('map', msg.header.frame_id,
Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.0))`를 실행했다.
최근 non-empty detection **10/10개**의 exact timestamp 조회가 성공했다.
예를 들어 detection `1789367750.817945557`의 lookup도 성공했고 반환
transform header의 stamp는 같은 값이었다. 이는 실제 detection timestamp에서
TF가 이용 가능하다는 증거이며 map position node를 구현한 결과는 아니다.

## 기존 시스템 회귀 확인

| 항목 | 이번 실행의 증거 |
| --- | --- |
| D435 single owner | ROS node `/leader/camera` 1개; infra1/infra2/color/depth 대표 topic publisher 각 1개 |
| VSLAM | infra1/infra2 발행, `/visual_slam/tracking/odometry` 발행; container `/visual_slam/status` sample `vo_state: 1`(Success); 종료 bag 1,585 Success/0 Failed/0 Unknown |
| dual EKF | `/leader/odometry/local` 9,529 samples/29.6 Hz, `/leader/odometry/global` 9,369 samples/29.1 Hz; `map → base_link` 연결 |
| nvblox | `/nvblox_node` alive, 기존 RGB/depth subscription 연결, mesh 10초 probe 39 messages 중 27개에 vertex 존재, 최대 18,200 vertices; RViz 3D mesh 화면 확인 |
| 3D ESDF | `/nvblox_node/get_esdf_and_gradient` 요청 `success=true`, voxel 0.05 m, 35,301 values |
| Survivor | `/leader/person_detector`, rectification과 CameraInfo bridge alive; `/leader/survivor/debug_image`와 non-empty `/leader/survivor/camera_positions` 발행; detector CUDA device `0` |

기존 `esdf_mode: 3d`에서 `static_esdf_pointcloud` topic의 존재만으로 실제
cloud 발행을 주장하지 않는다. 이전 Stage 2 문서와 마찬가지로 3D ESDF는
기존 서비스의 실제 응답으로 검증했다. RViz 화면에는 갱신된 회색 3D mesh가
보였고, `/nvblox_node/mesh`에 RViz subscription이 연결되어 있었다.

이번 종료 bag은 `data/vslam_mapping_20260914_153251/`에 있고 분석은
`analysis.md`, 실행 로그는 `log/vslam_mapping_20260914_153251/`에 있다.
bag 기간은 322.3초다. wheel odometry는 순이동 0 m이었으나 VSLAM/global
map pose는 기록 시작 후 약 10초 안에 `(0,0,0)`에서 약
`(-0.131, 0.097, yaw=-54.7°)`로 바뀌었다. 20초 이후 종료까지 해당 pose는
거의 고정됐고 마지막 5초 global 위치 jitter는 약 8.4 nm, yaw range 약
0.011°였다. 따라서 **초기 pose 정렬 변화는 있었다**. 원인 자체를 이번
Phase A에서 단정하지 않으며, 다음 Stage의 map stability 측정에서는
시작 직후 최소 20초를 제외하고 tracking/TF 안정화 후 측정해야 한다.
전체 bag의 VSLAM 평균 4.9 Hz, 최대 gap 약 4.0초는 처리율 관찰사항이다.
Success 상태와 exact-stamp 조회가 확인됐지만 이를 고속 주행 보증으로
해석해서는 안 된다.

detector에는 RGB/aligned-depth timestamp 차이 `0.133–0.467 s`로
`XYZ N/A`를 사용하는 warning이 간헐적으로 반복됐다. 유효 XYZ도 지속
발행됐다. rectifier에는 image/CameraInfo sync warning이 간헐적으로
출력됐다. Ctrl+C 종료 시 기존 CameraInfo bridge가 `rcl_shutdown already
called` 예외를 냈지만 실행 중 데이터 경로는 동작했고 종료 후 프로세스는
남지 않았다. 이 Phase에서는 기존 설정/코드를 변경하지 않았다.

## 수동 재현 명령

각 실행 명령은 **별도 terminal**에서 사용한다. ROS_DOMAIN_ID와 RMW 설정은
기존 `run_vslam_mapping.sh`의 환경을 따른다. 이 실행기는 작업 로그와 rosbag을
생성하고 마지막에 방향키 teleop을 띄운다. Phase A에서 Codex는 방향키나
모터 명령을 보내지 않았다. `camera_apriltag.launch.py`를 함께 실행하면
RealSense owner가 중복되므로 실행하지 않는다.

Terminal 1 — 기존 통합 실행:

```bash
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh
```

Terminal 2 — survivor preprocessing:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py
```

Terminal 3 — 기존 detector 전용 container:

```bash
cd ~/damgc_robot
./scripts/run_survivor_detector.sh
```

Terminal 4 — 사람이 D435 앞에 있을 때 확인. 현재 Humble CLI에서 `echo
--once` 지원을 `ros2 topic echo --help`로 확인했다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 node list
ros2 topic info -v /leader/survivor/camera_positions
timeout 12 ros2 topic echo --once /leader/survivor/camera_positions
timeout 12 ros2 topic echo --once /leader/camera/color/image_rect --field header
ros2 topic info -v /leader/survivor/debug_image
timeout 8 ros2 topic echo --once /leader/survivor/debug_image --field header
timeout 8 ros2 run tf2_ros tf2_echo map base_link
timeout 8 ros2 run tf2_ros tf2_echo map camera_color_optical_frame
timeout 8 ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
timeout 4 ros2 run tf2_ros tf2_echo map odom
timeout 4 ros2 run tf2_ros tf2_echo odom base_link
timeout 4 ros2 run tf2_ros tf2_echo base_link camera_link
timeout 4 ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

원본 RGB stamp 일치와 detection 시점 TF를 동시에 검사한 읽기 전용 probe:

```bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import time
from collections import deque
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

rclpy.init()
node = Node('stage4_phase_a_contract_probe')
buffer = Buffer()
listener = TransformListener(buffer, node)
images = deque(maxlen=300)
positions = deque(maxlen=100)
node.create_subscription(
    Image, '/leader/camera/color/image_rect',
    lambda m: images.append((m.header.stamp.sec, m.header.stamp.nanosec)),
    qos_profile_sensor_data)
node.create_subscription(
    PoseArray, '/leader/survivor/camera_positions',
    lambda m: positions.append(m), 1)
deadline = time.monotonic() + 13.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
nonempty = [m for m in positions if m.poses]
image_stamps = set(images)
matches = sum(
    (m.header.stamp.sec, m.header.stamp.nanosec) in image_stamps
    for m in nonempty)
success = 0
for m in nonempty[-10:]:
    try:
        transform = buffer.lookup_transform(
            'map', m.header.frame_id, Time.from_msg(m.header.stamp),
            timeout=Duration(seconds=0.0))
        print('exact TF:', m.header.stamp, transform.header.stamp)
        success += 1
    except TransformException as error:
        print('TF failure:', error)
print('RGB samples:', len(images))
print('nonempty positions:', len(nonempty))
print('exact RGB stamp matches:', matches)
print('exact timestamp TF successes:', success, '/', min(10, len(nonempty)))
node.destroy_node()
rclpy.shutdown()
PY
```

TF listener cache가 준비될 시간을 주기 위해 13초 수집 후 최근 10개를
조회한다. 이 probe는 topic을 발행하거나 robot 제어 명령을 보내지 않는다.

회귀 topic 확인:

```bash
ros2 topic info -v /leader/camera/infra1/image_rect_raw
ros2 topic info -v /leader/camera/infra2/image_rect_raw
ros2 topic info -v /leader/camera/color/image_raw
ros2 topic info -v /leader/camera/depth/image_rect_raw
ros2 topic info -v /leader/camera/aligned_depth_to_color/image_raw
ros2 topic info -v /visual_slam/tracking/odometry
ros2 topic info -v /leader/odometry/local
ros2 topic info -v /leader/odometry/global
ros2 topic info -v /nvblox_node/mesh
ros2 service type /nvblox_node/get_esdf_and_gradient
```

host에는 Isaac ROS 전용 `VisualSlamStatus`와 `nvblox_msgs/Mesh` Python
message type이 없어 해당 메시지의 echo/내용 검사는 실행 중인 Isaac
container에서 수행했다.

```bash
docker exec isaac_ros_dev-aarch64-container bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  timeout 8 ros2 topic echo --once /visual_slam/status
  timeout 8 ros2 topic echo --once /nvblox_node/mesh --field header
'
```

3D ESDF 실제 값은 container에서 `nvblox_msgs/srv/EsdfAndGradients` Python
client로 `update_esdf=false`, `visualize_esdf=false`, `use_aabb=true`,
`frame_id=odom`, AABB min `(-1,-1,-0.3) m`, size `(2,2,1) m`를 요청해
`success`, `voxel_size_m`, `len(esdf_and_gradients.data)`를 확인했다.
위 구체적 request는 repo의 Stage 2 통합 validation에서 사용한 범위와 같다.

종료 순서: Terminal 3 detector Ctrl+C → Terminal 2 preprocessing Ctrl+C →
Terminal 1 통합 실행기 Ctrl+C. 마지막 실행기는 rosbag을 정상 마감·분석하고
자신이 시작한 호스트/컨테이너 프로세스를 종료한다. 종료 후 `ros2 node list`,
`docker ps`, `git status`를 확인했다. ROS node와 실행 중 container는 남지
않았다.

## Phase A 최종 게이트

| 조건 | 결과 |
| --- | --- |
| camera_positions 정상, 실제 사람 pose | PASS |
| type/frame/stamp 명확, stamp nonzero | PASS |
| RGB 원본 stamp 복사 및 live stamp 일치 | PASS |
| `map → camera_color_optical_frame` TF | PASS |
| detection exact timestamp TF lookup | PASS, 최근 10/10 |
| `map → base_link` TF | PASS |
| VSLAM/dual EKF/Survivor Detector | PASS, 초기 정렬·처리율 관찰사항 있음 |
| nvblox mesh/3D ESDF | PASS |

**Phase A: PASS. `/goal 2` 진행 가능: YES.** 다음 goal은 현재
`camera_positions` 계약을 그대로 소비하는 별도 map transform node를 추가한다.
이 문서의 초기 정렬·rate·RGB/depth warning은 다음 실측 결과와 함께 추적한다.

## Phase B — exact-timestamp map transform 구현

2026-09-14 Phase B 시작 시 branch/commit은 위 Phase A와 같은 `main` /
`db1e71543236bd94328b33c0dd68634a08c97e9c`였다. `git status`에는
Phase A에서 작성한 이 문서만 untracked로 있었고 `git diff`와 staged diff는
비어 있었다. 기존 사용자 변경을 reset·삭제하지 않았다.

이번 Phase B의 신규 파일:

- `src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_map_transform_node.py`
- `src/leader/rescue_robot_survivor/launch/survivor_map_transform.launch.py`
- `src/leader/rescue_robot_survivor/test/test_survivor_map_transform_node.py`

수정 파일은 `src/leader/rescue_robot_survivor/setup.py`와
`src/leader/rescue_robot_survivor/package.xml`, 그리고 이 문서다. `setup.py`에는
`survivor_map_transform_node` console entry point를 추가했다. `package.xml`에는
실제로 import하는 `tf2_ros`와 `tf2_geometry_msgs`만 추가했다. 기존
`rclpy`, `geometry_msgs`, launch dependency는 재사용한다. package의
`glob("launch/*.launch.py")`가 새 launch를 자동 설치한다.

보호 대상인 `scripts/run_vslam_mapping.sh`, VSLAM/dual EKF/nvblox 설정·launch,
`person_detector_node.py`의 XYZ 계산, survivor preprocessing, RealSense owner,
`camera_apriltag.launch.py`는 수정하지 않았다. 새 launch는 transform consumer
하나만 실행하며 camera, VSLAM, nvblox, detector, robot_state_publisher를
중복 실행하지 않는다.

### Node의 데이터 계약

| 항목 | 기본값 / 처리 |
| --- | --- |
| `input_topic` | `/leader/survivor/camera_positions`, `PoseArray` |
| `output_topic` | `/leader/survivor/map_positions`, `PoseArray` |
| `target_frame` | `map` |
| `tf_timeout_sec` | `0.2`초; 유한 양수만 허용 |
| 입력/출력 QoS | reliable, volatile, keep-last 1; 현재 detector publisher와 동일 |
| 출력 header | `frame_id=target_frame`, `stamp=input.header.stamp` |

callback은 빈 poses를 정상적인 no-detection frame으로 보고 발행하지 않는다.
non-empty 메시지는 source frame, stamp, 모든 입력 XYZ가 유효한지 먼저
검사한다. 그 후 `Buffer.lookup_transform(target_frame,
msg.header.frame_id, Time.from_msg(msg.header.stamp),
timeout=Duration(seconds=tf_timeout_sec))`를 **메시지당 한 번** 호출한다.
`Time()` 또는 최신 transform으로 fallback하는 경로는 없다.

Stage 3의 orientation은 사람 방향이 아니라 identity placeholder이므로 각
`pose.position`을 `PointStamped`로 바꿔 `do_transform_point`로 변환한다.
출력 pose orientation은 다시 identity `(0,0,0,1)`로 둔다. 여러 사람은 입력
순서를 유지한다. 하나의 PoseArray에서 모든 point에 동일한 조회 TF를 적용하며
전체 변환이 성공한 뒤에만 출력 PoseArray를 발행한다.

source frame이 비거나 stamp가 zero/invalid, 입력 XYZ가 NaN/Inf, TF
lookup·연결·외삽·timeout 실패, TF translation/quaternion 또는 변환 결과가
비정상이면 **그 메시지 전체를 skip**한다. 부분 map 배열이나 fake origin을
발행하지 않는다. 오류 경고는 5초 throttle을 적용한다. TF listener는 별도
spin thread를 사용하므로 callback의 짧은 timed lookup 중에도 `/tf`와
`/tf_static`을 계속 수신할 수 있다.

### Build, 정적 검증, 설치 확인

실제로 실행한 명령:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_survivor
source install/local_setup.bash
colcon test --packages-select rescue_robot_survivor --event-handlers console_direct+
colcon test-result --verbose
ros2 pkg executables rescue_robot_survivor
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py --show-args
python3 -m flake8 \
  src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_map_transform_node.py \
  src/leader/rescue_robot_survivor/launch/survivor_map_transform.launch.py \
  src/leader/rescue_robot_survivor/test/test_survivor_map_transform_node.py
git diff --check
git status
git diff
```

build는 성공했다. 설치된 `rescue_robot_survivor` executable 목록에서 기존
`person_detector_node`와 새 `survivor_map_transform_node`를 확인했다.
새 launch의 4개 argument도 `--show-args`에서 발견됐다. 첫 test run은
비정상 TF translation `Inf`에서 Humble `do_transform_point` 내부 NumPy가
`LinAlgError`를 낸 사례를 발견해 45/46 PASS였다. TF 수치·quaternion
사전 검증과 변환 예외 처리를 추가한 뒤 **46/46 PASS**로 재실행했다.
flake8와 `git diff --check`도 통과했다. 테스트는 exact stamp 전달,
multi-pose당 lookup 1회, map frame/input stamp 보존, 회전+translation,
identity orientation, malformed 입력, TF 실패 시 무발행, non-finite TF
무발행을 검사한다.

### 실제 통합 smoke test

Phase A와 같은 기존 통합 실행기를 먼저 시작한 다음 기존 preprocessing과
detector를 실행했다. 새 node는 별도 terminal에서 다음처럼 실행했다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py
```

확인 명령:

```bash
ros2 node list
ros2 node info /leader/survivor_map_transform
ros2 topic info -v /leader/survivor/camera_positions
ros2 topic info -v /leader/survivor/map_positions
timeout 15 ros2 topic echo --once /leader/survivor/camera_positions
timeout 15 ros2 topic echo --once /leader/survivor/map_positions
timeout 8 ros2 topic hz /leader/survivor/map_positions
```

실제 `/leader/survivor_map_transform` node, camera_positions subscription,
map_positions publisher가 생겼고 양쪽 endpoint는 reliable/volatile이었다.
사람이 D435 앞에 있는 동안 non-empty map PoseArray가 발행됐으며
`frame_id=map`, XYZ finite, orientation identity였다. 독립적인 `--once`
두 명령은 서로 다른 image frame을 잡으므로 stamp 일치 판정에는 사용하지
않았다. 두 topic을 동시에 12초 구독해 stamp를 key로 비교한 결과
**non-empty camera 52개, map 49개, 동일 stamp pair 49개**였다. 첫 pair는
다음과 같다.

```text
stamp: 1789393892.922213623
camera frame: camera_color_optical_frame
camera XYZ: (0.4594544955, 0.1333944554, 2.6220002174) m
map frame: map
map XYZ: (2.6675035611, -0.4133901861, -0.0052690942) m
poses: 1 → 1, 출력 XYZ finite
```

2명 검출 pair도 `poses: 2 → 2`와 동일 stamp를 확인했다. 이 smoke test는
실제 data flow와 output contract 확인이며, Phase C의 정지 상태 위치 정확도
또는 Phase D의 이동 중 map stability PASS를 대신하지 않는다.

TF 실패 무발행도 실제 graph에서 별도 failure probe로 확인했다. 기존
node/output을 건드리지 않고 존재하지 않는 target frame 및 별도 output
topic으로 잠시 실행했다.

```bash
ros2 run rescue_robot_survivor survivor_map_transform_node --ros-args \
  -r __node:=survivor_map_transform_failure_probe \
  -p target_frame:=stage4_missing_frame \
  -p output_topic:=/leader/survivor/map_positions_failure_probe \
  -p tf_timeout_sec:=0.1
timeout 8 ros2 topic echo --once \
  /leader/survivor/map_positions_failure_probe
```

경고는 약 5초 간격으로 나왔고 echo는 8초간 아무 메시지도 받지 못했다
(`timeout` exit 124). probe는 Ctrl+C로 종료했다. 실제 map output의 짧은
`ros2 topic hz` 관찰은 약 4.2–4.5 Hz였으나 장기 처리율 판정은 아니다.

Phase B smoke의 log/rosbag 경로는
`log/vslam_mapping_20260914_225012/`,
`data/vslam_mapping_20260914_225012/`이다. detector에는 이전 Phase와
같은 RGB/aligned-depth timestamp warning이 간헐적으로 있었지만 유효
camera/map XYZ가 계속 발행됐다. 이번 goal에서 robot 주행·motor command는
실행하지 않았다.

### VSLAM 처리 공백 조사와 같은 실행 내 OFF/ON 비교

첫 Phase B smoke의 종료 rosbag 분석에서 VSLAM tracking odometry는
163.8초 동안 111건/평균 **0.7 Hz**, 최대 수신 공백 **108.6초**였다.
Success 110/Failed 0/Unknown 0이라는 status count만으로 이 공백을
정상이라고 볼 수 없다. bag의 수신 시각을 읽기 전용으로 조사한 결과
`1789393866.166508401` 다음 메시지는 `1789393974.771603804`에
수신됐고, 재개 메시지의 원본 stamp는 `1789393915.9797`로 수신 시점보다
약 58.8초 뒤처져 있었다. 이 구간에도 global EKF의 TF 발행과 survivor
map output은 있었지만, 그것만으로 VSLAM pose의 신선도를 보증할 수 없다.
새 node가 원인인지 여부는 이 한 번의 실행만으로 판단할 수 없다.

이를 분리하기 위해 VSLAM 관련 코드·설정을 수정하지 않고 기존
`run_vslam_mapping.sh` + preprocessing + detector를 새로 실행했다.
첫 30초에는 transform node를 **실행하지 않고**, 같은 실행 중 뒤 30초에는
transform node만 **추가**했다. 두 구간 모두 작은 `Odometry` subscriber로
5초 구간별 수신 수와 `wall_time - message.header.stamp`를 측정했다.

| 구간 | 30초 수신 | 5초 구간 수신 범위 | 마지막 stamp 지연 범위 |
| --- | ---: | ---: | ---: |
| transform OFF | 162건 (약 5.4 Hz) | 22–38건 | 0.07–0.24초 |
| transform ON | 147건 (약 4.9 Hz) | 15–33건 | 0.09–0.70초 |

ON 구간에서 별도로 읽은 `/visual_slam/status`는 `vo_state: 1`(Success),
`/leader/survivor/map_positions`도 non-empty로 발행됐다. 이 짧은 비교에서
108초급 공백은 재현되지 않았다. OFF/ON 평균 차이를 새 node의 인과적
부하로 단정하지 않으며, 첫 smoke에서 관찰된 긴 공백의 원인은 **미확정**이다.
두 번째 실행의 log/rosbag은 `log/vslam_mapping_20260914_225511/`,
`data/vslam_mapping_20260914_225511/`에 있다. 종료 bag 143.9초의
VSLAM tracking odometry는 729건/평균 **5.1 Hz**, 최대 수신 공백
**5.28초**, status Success 729/Failed 0/Unknown 0이었다. 즉 긴
108초 공백은 재현되지 않았지만 짧은 간헐 공백은 여전히 관찰됐다.
다음 Phase C에서는
VSLAM 수신 rate뿐 아니라 image stamp 지연과 장시간 공백을 별도
회귀 항목으로 검사해야 한다. VSLAM이 뒤처진 구간의 map XYZ는
TF lookup이 성공하더라도 실제 localization 정확도 판정에 사용하지 않는다.

**Phase B: 구현·build·실제 map output contract PASS. `/goal 3` 진행 가능:
YES, 단 VSLAM 긴 공백은 재검증 대상.** 다음 단계는 실제 정지 상태 좌표
검증과 VSLAM/EKF/nvblox 장시간 동시 회귀 측정이다. 그 다음 사용자 수동
주행 중 map stability를 별도로 검증한다.

## Phase C — 정지 상태 실제 동시 통합 검증

2026-09-14 Jetson + D435에서 기존 통합 실행기, survivor preprocessing,
YOLO detector, 새 transform node를 **동시에** 실행했다. 시작 branch/commit은
`main` / `db1e71543236bd94328b33c0dd68634a08c97e9c`다. Phase A/B의
uncommitted 문서·코드·launch·package 변경을 그대로 보존했고 추가 코드
수정은 하지 않았다. 이번에는 robot 방향키 또는 motor command를 보내지
않았으며 사람이 D435 앞에 정지했다.

### 실제 실행 terminal과 수동 확인 명령

Terminal 1 — 기존 single-owner D435, STM32, VSLAM, dual EKF, nvblox,
RViz와 metrics bag:

```bash
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh
```

Terminal 2 — 기존 survivor RGB preprocessing:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py
```

Terminal 3 — 기존 NVIDIA detector container:

```bash
cd ~/damgc_robot
./scripts/run_survivor_detector.sh
```

Terminal 4 — 처음 30초 baseline에서는 실행하지 않고, 그 다음 시작한
transform consumer:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py
```

Terminal 5 — 실제 camera XYZ와 endpoint:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 topic info -v /leader/survivor/camera_positions
timeout 10 ros2 topic echo --once /leader/survivor/camera_positions
```

Terminal 6 — 실제 map XYZ와 endpoint:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 topic info -v /leader/survivor/map_positions
timeout 10 ros2 topic echo --once /leader/survivor/map_positions
```

Terminal 7 — TF와 전체 graph:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 node list
ros2 topic info -v /leader/camera/infra1/image_rect_raw
ros2 topic info -v /leader/camera/infra2/image_rect_raw
ros2 topic info -v /leader/camera/color/image_raw
ros2 topic info -v /leader/camera/depth/image_rect_raw
timeout 6 ros2 run tf2_ros tf2_echo map base_link
timeout 6 ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```

`tf2_echo`는 자체 buffer가 처음 채워지기 전 `Invalid frame ID`를 잠시
출력했지만 같은 probe에서 이어서 transform을 수신했다. `map → base_link`
translation은 정지 상태에서 거의 `(0,0,0) m`, `map →
camera_color_optical_frame`은 약 `(0.042,0.025,0.130) m`였다. 전체
node 목록에서 camera driver `/leader/camera`는 한 개였다. infra1/infra2,
color/raw depth의 대표 topic에는 각각 RealSense publisher가 한 개였다.
`/leader/person_detector`, `/leader/survivor_map_transform`, 두 EKF,
`/visual_slam_node`, `/nvblox_node`, `/rviz`도 실행 중임을 확인했다.

Isaac 전용 `VisualSlamStatus` type은 호스트 overlay에 없으므로 기존
container에서 다음과 같이 확인했다.

```bash
docker exec isaac_ros_dev-aarch64-container bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  timeout 8 ros2 topic echo --once /visual_slam/status
'
```

### 동일 image stamp의 camera/map 좌표

독립적인 `topic echo --once`는 서로 다른 RGB frame을 포착할 수 있으므로
두 topic을 동시에 `PoseArray`로 구독해 `(stamp.sec, stamp.nanosec)`를
key로 짝지었다. 45초간 camera 메시지 278개, map 메시지 260개,
**동일 stamp pair 260개**를 수신했다. 단일 pose·예상 frame·finite XYZ
조건을 만족한 pair는 **255개**였다. 독립적인 one-shot에서 빈
`camera_positions.poses`도 한 번 보였는데, 이는 detector의 정상적인
frame-local no-valid-XYZ 출력이며 map node는 빈 출력 pose를 만들지 않는다.

동일 stamp와 raw 좌표 통계를 다시 확인하는 명령:

```bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import math
import statistics
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

rclpy.init()
node = Node('stage4_phase_c_stationary_probe')
camera, mapped = {}, {}
def key(message):
    return message.header.stamp.sec, message.header.stamp.nanosec
node.create_subscription(
    PoseArray, '/leader/survivor/camera_positions',
    lambda message: camera.__setitem__(key(message), message), 1)
node.create_subscription(
    PoseArray, '/leader/survivor/map_positions',
    lambda message: mapped.__setitem__(key(message), message), 1)
deadline = time.monotonic() + 45.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
paired = sorted(set(camera) & set(mapped))
valid = []
for stamp in paired:
    source, target = camera[stamp], mapped[stamp]
    if len(source.poses) != 1 or len(target.poses) != 1:
        continue
    cp, mp = source.poses[0].position, target.poses[0].position
    values = (cp.x, cp.y, cp.z, mp.x, mp.y, mp.z)
    if source.header.frame_id == 'camera_color_optical_frame' and target.header.frame_id == 'map' and all(map(math.isfinite, values)):
        valid.append((stamp, values[:3], values[3:]))
print('camera/map/paired/valid:', len(camera), len(mapped), len(paired), len(valid))
if valid:
    print('first:', valid[0])
    print('last:', valid[-1])
    for label, index in (('camera', 1), ('map', 2)):
        axes = list(zip(*(row[index] for row in valid)))
        print(label, 'mean/min/max:',
              tuple(statistics.mean(axis) for axis in axes),
              tuple(min(axis) for axis in axes),
              tuple(max(axis) for axis in axes))
    print('max adjacent map jump:', max(
        (math.dist(valid[i-1][2], valid[i][2]) for i in range(1, len(valid))),
        default=0.0))
node.destroy_node()
rclpy.shutdown()
PY
```

첫 유효 pair:

```text
stamp: 1789394608.577992188 (camera와 map에서 동일)
camera frame: camera_color_optical_frame
camera XYZ: (0.4281873923, 0.1440854151, 2.8790001869) m
map frame: map
map XYZ: (2.9244408773, -0.3786105352, -0.0157058254) m
```

마지막 유효 pair:

```text
stamp: 1789394653.310234131 (camera와 map에서 동일)
camera XYZ: (0.4328417770, 0.1365283121, 2.7280001640) m
map XYZ: (2.7735073804, -0.3843877443, -0.0082182265) m
```

별도 one-shot map 메시지에서 `header.frame_id=map`, 유효 stamp,
finite XYZ와 orientation `(0,0,0,1)`도 확인했다. map node는 종료 시
정상 종료했고, 실제 TF 실패가 발생한 것은 새 listener buffer가 준비되기
전의 첫 시작 경고뿐이었다. 계속된 map 출력을 확인했으며 latest TF
fallback 경로는 Phase B 코드 검토·테스트에서도 없다. 시스템 고장을
의도적으로 유발하지 않았다.

정지 상태 단일 사람 유효 pair 255개의 **raw** 좌표 통계:

| 측정 | Camera X/Y/Z [m] | Map X/Y/Z [m] |
| --- | --- | --- |
| mean | `(0.4327, 0.1423, 2.8247)` | `(2.8702, -0.3833, -0.0139)` |
| min | `(0.4146, 0.1299, 2.6020)` | `(2.6478, -0.4314, -0.0253)` |
| max | `(0.4800, 0.1536, 2.9160)` | `(2.9614, -0.3651, -0.0016)` |

map XY 전체 축별 range의 radial 크기는 약 **0.3206 m**, 연속 유효
map point의 최대 3D 인접 jump는 **0.2098 m**였다. NaN/Inf나
meter 단위 폭주는 없었지만 수십 cm 흔들림은 정밀 map stability라고
평가하기 어렵다. Camera Z range 0.314 m가 map X range 약 0.314 m로
거의 그대로 반영됐다. 이는 입력 camera XYZ의 depth/ROI 측정 변화가
map spread의 주요 원인일 가능성을 보여 주는 **추론**이며, 사람이
완전히 움직이지 않았다는 독립 계측은 없다. 이 Stage에서는 averaging,
filtering, 중복 제거를 추가하지 않았다.

### VSLAM, EKF 및 camera rate 전후 비교

같은 통합 실행 중 하나의 읽기 전용 ROS probe로 infra1/infra2 image,
VSLAM tracking odometry, local/global EKF, camera_positions를 10초
구간별로 수집했다. 첫 30초는 transform OFF, 다음 10초는 launch
전환 구간으로 제외하고, 이후 80초를 transform ON으로 비교했다.

| Topic | OFF 30초 | ON 80초 | 판단 |
| --- | ---: | ---: | --- |
| infra1 `image_rect_raw` | 410 / 13.67 Hz | 1,106 / 13.83 Hz | 유지 |
| infra2 `image_rect_raw` | 469 / 15.63 Hz | 1,305 / 16.31 Hz | 유지 |
| VSLAM tracking odometry | 152 / 5.07 Hz | 413 / 5.16 Hz | 평균률 유지 |
| local EKF | 871 / 29.03 Hz | 2,362 / 29.53 Hz | 유지 |
| global EKF | 879 / 29.30 Hz | 2,321 / 29.01 Hz | 유지 |

OFF 구간 VSLAM 최대 수신 gap은 1.77초였다. ON 80초 구간까지의
최대 gap은 4.58초였고 한 10초 구간 끝의 stamp age는 2.92초였다.
평균률 유지와 `vo_state: 1`(Success) sample을 확인했지만, 간헐 지연은
관찰사항으로 남긴다. 이전 Phase B의 108초 공백 재발 여부를 보려면
더 긴 ON 구간도 필요하므로 별도 연속 probe를 이어서 수행했다.
transform ON으로 추가 140초 연속 관찰한 VSLAM은 **678건/약 4.84 Hz**,
최대 수신 gap **2.36초**였고 매 10초 구간에서 메시지를 수신했다.
관찰된 10초 경계의 마지막 stamp age는 최대 **0.91초**였다. 따라서
이번 장시간 ON 구간에서 Phase B의 108초 공백은 재발하지 않았다.
다만 짧은 공백의 근본 원인은 아직 확인되지 않았다.

### nvblox, RViz, RealSense 회귀

`/nvblox_node`는 기존 `/leader/camera/color/image_raw` 및
`/leader/camera/depth/image_rect_raw`를 구독했고 두 입력의 RealSense
publisher는 각각 한 개였다. 기존 `esdf_mode: 3d` 정책을 유지하며
`/nvblox_node/get_esdf_and_gradient`에 읽기 전용 AABB 요청
(`update_esdf=false`, `visualize_esdf=false`, `frame_id=odom`, min
`(-1,-1,-0.3) m`, size `(2,2,1) m`)을 보냈다.

| 상태 | mesh probe | 3D ESDF 서비스 |
| --- | --- | --- |
| transform ON | 18 messages, vertex 포함 16, 최대 18,333 vertices | `success=true`, 0.05 m, 35,301 values |
| transform OFF | 20 messages, vertex 포함 14, 최대 17,901 vertices | `success=true`, 35,301 values |
| transform 다시 ON | 21 messages, vertex 포함 14, 최대 18,459 vertices | `success=true`, 35,301 values |

RViz의 fixed frame은 `map`이고 3D mesh가 화면에 보였다. 정지 상태라
새로운 공간으로 map이 확장되는지는 시험하지 않았다. 기존 3D 모드에서
`static_esdf_pointcloud` 발행 여부로 ESDF를 판정하지 않고 서비스의
실제 값을 사용했다. screenshot 임시 확인 파일은
`/tmp/stage4_phase_c_rviz_20260914.png`다.

Mesh의 실제 vertex와 3D ESDF를 직접 확인한 container 명령은 다음과
같다. 현재 ROS2 메시지 구현에서 `Point`/`Vector3` 필드에는 `-1` 같은
정수 대신 `-1.0` 형식의 float를 대입해야 한다.

```bash
docker exec -i isaac_ros_dev-aarch64-container bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  python3 -
' <<'PY'
import time
import rclpy
from rclpy.node import Node
from nvblox_msgs.msg import Mesh
from nvblox_msgs.srv import EsdfAndGradients

rclpy.init()
node = Node('stage4_phase_c_nvblox_probe')
meshes = []
node.create_subscription(
    Mesh, '/nvblox_node/mesh',
    lambda m: meshes.append(sum(len(block.vertices) for block in m.blocks)), 10)
client = node.create_client(
    EsdfAndGradients, '/nvblox_node/get_esdf_and_gradient')
print('service available:', client.wait_for_service(timeout_sec=5.0))
request = EsdfAndGradients.Request()
request.update_esdf = False
request.visualize_esdf = False
request.use_aabb = True
request.frame_id = 'odom'
request.aabb_min_m.x, request.aabb_min_m.y, request.aabb_min_m.z = -1.0, -1.0, -0.3
request.aabb_size_m.x, request.aabb_size_m.y, request.aabb_size_m.z = 2.0, 2.0, 1.0
future = client.call_async(request)
deadline = time.monotonic() + 12.0
while not future.done() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
if future.done():
    result = future.result()
    print('ESDF success/voxel/values:', result.success,
          result.voxel_size_m, len(result.esdf_and_gradients.data))
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
print('mesh samples/nonzero/max vertices:', len(meshes),
      sum(value > 0 for value in meshes), max(meshes, default=0))
node.destroy_node()
rclpy.shutdown()
PY
```

### 종료 및 남은 검증

실행 종료는 Terminal 4 transform Ctrl+C → Terminal 3 detector Ctrl+C →
Terminal 2 preprocessing Ctrl+C → Terminal 1 mapping Ctrl+C 순서다.
기존 mapping launcher는 rosbag을 마감·분석한 뒤 자신이 시작한
camera/bridge/컨테이너를 종료한다. 이번 실행의 log/rosbag은
`log/vslam_mapping_20260914_230100/`,
`data/vslam_mapping_20260914_230100/`에 기록됐다. 다음 Phase D는
사람을 고정하고 사용자가 로봇을 아주 천천히 이동시킬 때 raw camera/map
좌표의 안정성을 따로 측정한다. Codex는 모터 명령을 보내지 않는다.

정상 마감된 469.0초 rosbag 분석:

| Topic | Samples / 평균률 | 최대 수신 gap | 순이동 |
| --- | ---: | ---: | ---: |
| wheel odometry | 16,472 / 35.2 Hz | 1.01초 | 0 m |
| local EKF | 13,886 / 29.6 Hz | 0.43초 | 0 m |
| global EKF | 13,687 / 29.2 Hz | 0.45초 | 0 m |
| VSLAM tracking odometry | 2,412 / 5.1 Hz | 4.58초 | 0 m |

VSLAM status는 **Success 2,413 / Failed 0 / Unknown 0**이었다.
처음 Phase B의 108초 공백은 이번 469초 실행에서 재현되지 않았지만
4.58초 공백은 짧은 관찰사항으로 남는다. 이 관찰만으로 이후 이동 중
tracking이나 map 안정성을 보증하지 않는다. preprocessing rectifier의
image/CameraInfo sync warning과 detector의 RGB/aligned-depth 차이
warning은 이번에도 간헐적으로 보였다. 기존 CameraInfo bridge는
Ctrl+C 종료 시 `rcl_shutdown already called`를 다시 출력했지만 실행
중 topic은 유지됐고 종료 후 ROS node/실행 중 container는 남지 않았다.

### Phase C 판정

| 요구 조건 | 판정 |
| --- | --- |
| camera_positions와 실제 사람 XYZ | PASS |
| map_positions 지속 발행, frame `map` | PASS |
| 동일 detection timestamp 유지 | PASS, 260 paired stamps |
| finite XYZ, node crash 없음 | PASS, 255 유효 단일 사람 pairs 및 정상 종료 |
| VSLAM infra1/infra2/odometry | PASS, 평균률 유지; 간헐 gap 관찰 |
| local/global EKF, `map → base_link` | PASS |
| nvblox mesh/3D ESDF, RViz 3D map | PASS, 정지 상태 map 확장 미시험 |
| Survivor Detector와 single D435 owner | PASS |

**Phase C: PASS. `/goal 4` 진행 가능: YES.** 정지 상태의 raw map
좌표는 최대 약 0.32 m planar range로 흔들렸으므로 다음 이동 시험에서
정확도 수치를 과장하지 않는다. Phase D에서 map 위치가 크게 움직이면
filter를 먼저 넣지 않고 입력 depth/ROI, timestamp, TF, VSLAM/EKF를
분리 조사한다.

## Phase D — 고정 생존자·로봇 이동 Map Stability 검증

2026-09-14에 동일한 D435, VSLAM, dual EKF, nvblox, preprocessing,
detector, `survivor_map_transform_node`를 실행한 상태에서 Phase D를
수행했다. Codex는 모터 명령을 보내지 않았고, 사용자가 통합 실행기의
기존 `arrow_key_teleop`에서 짧게 전진한 뒤 `Space`로 정지했다. 사람은
가능한 같은 자리에 유지했다.

### A 기준 위치

안정화 후 20초 동안 동일 detection timestamp로 짝지어진 유효 단일
사람 pair 129개를 수집했다.

| 위치 | Camera mean X/Y/Z [m] | Map mean X/Y/Z [m] | 해당 위치 내부 map XY 최대 편차 |
| --- | --- | --- | ---: |
| A | `(0.084, -0.465, 2.566)` | `(2.609, -0.040, 0.595)` | `0.024 m` |

### A → B 이동

사용자가 사람을 같은 위치에 둔 채 로봇을 짧게 전진했다. wheel/local
odometry의 최종 위치는 약 `(0.351, -0.001) m`였고, global/VSLAM은 약
`(0.343, 0.009) m`로 변했다. 따라서 실제로 측정 가능한 robot motion이
발생했다.

| 위치 | Camera mean X/Y/Z [m] | Map mean X/Y/Z [m] | 해당 위치 내부 map XY 최대 편차 | paired samples |
| --- | --- | --- | ---: | ---: |
| A | `(0.084, -0.465, 2.566)` | `(2.609, -0.040, 0.595)` | `0.024 m` | 129 |
| B | `(0.043, -0.379, 2.149)` | `(2.533, -0.058, 0.509)` | `0.040 m` | 135 |

Camera 좌표 변화량은 평균 기준 약 `(-0.041, +0.086, -0.418) m`로
명확했다. A와 B map 평균의 3D 차이는 약 `0.115 m`였다. 즉 map 좌표가
카메라 좌표처럼 약 0.4 m 크게 따라 움직이지 않고 같은 생존자 주변에
남았다는 핵심 동작은 확인했다. 동시에 0.115 m의 raw 이동은 정밀한
절대 위치 안정성으로 보기에는 크므로, 이번 Stage에서는 filtering이나
averaging으로 숨기지 않았다. 가능한 원인은 detector depth/ROI 변화,
카메라-생존자 상대 자세 변화, VSLAM/EKF 초기 map 정렬 차이이며, 이
측정만으로 하나를 단정하지 않는다.

### 이동 중 회귀 검증

이동 및 B 정지 구간에서 다음을 확인했다.

| 항목 | 결과 |
| --- | --- |
| VSLAM status | `vo_state: 1` Success |
| wheel/local odometry | 최종 약 `0.351 m`, B에서 정지 |
| global/VSLAM pose | 최종 약 `(0.343, 0.009) m`, B에서 안정 |
| Survivor camera/map pairs | B 유효 pair 135개, finite XYZ |
| nvblox mesh | 11 messages, vertex 포함 8개, 최대 17,944 vertices |
| nvblox 3D ESDF | service `success=true`, voxel `0.05 m`, 35,301 values |
| map transform | node 유지, map frame/동일 timestamp contract 유지 |

B 구간의 실제 통계 명령은 다음과 같은 read-only probe로 수행했다.

```bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import math, statistics, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
rclpy.init(); node = Node('stage4_phase_d_b_probe')
camera, mapped = {}, {}
def key(msg): return msg.header.stamp.sec, msg.header.stamp.nanosec
node.create_subscription(PoseArray, '/leader/survivor/camera_positions', lambda msg: camera.__setitem__(key(msg), msg), 1)
node.create_subscription(PoseArray, '/leader/survivor/map_positions', lambda msg: mapped.__setitem__(key(msg), msg), 1)
deadline = time.monotonic() + 25.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
pairs = []
for stamp in set(camera) & set(mapped):
    source, target = camera[stamp], mapped[stamp]
    if len(source.poses) != 1 or len(target.poses) != 1:
        continue
    a, b = source.poses[0].position, target.poses[0].position
    values = (a.x, a.y, a.z, b.x, b.y, b.z)
    if (source.header.frame_id == 'camera_color_optical_frame'
            and target.header.frame_id == 'map'
            and all(math.isfinite(value) for value in values)):
        pairs.append(values)
print('B valid paired samples:', len(pairs))
for name, indices in (('camera', (0, 1, 2)), ('map', (3, 4, 5))):
    values = [tuple(row[i] for i in indices) for row in pairs]
    mean = tuple(statistics.mean(row[i] for row in values) for i in range(3))
    lo = tuple(min(row[i] for row in values) for i in range(3))
    hi = tuple(max(row[i] for row in values) for i in range(3))
    spread = max((math.dist(row[:2], mean[:2]) for row in values), default=0.0)
    print(name, 'mean/min/max/max_xy_radius:', mean, lo, hi, spread)
node.destroy_node(); rclpy.shutdown()
PY
```

VSLAM과 ESDF 확인 명령:

```bash
docker exec isaac_ros_dev-aarch64-container bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  timeout 7 ros2 topic echo --once /visual_slam/status
'

docker exec -i isaac_ros_dev-aarch64-container bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  python3 -
' <<'PY'
import time, rclpy
from rclpy.node import Node
from nvblox_msgs.srv import EsdfAndGradients
rclpy.init(); node = Node('stage4_phase_d_esdf_probe')
client = node.create_client(EsdfAndGradients, '/nvblox_node/get_esdf_and_gradient')
print('service available:', client.wait_for_service(timeout_sec=5.0))
request = EsdfAndGradients.Request()
request.update_esdf = False; request.visualize_esdf = False
request.use_aabb = True; request.frame_id = 'odom'
request.aabb_min_m.x, request.aabb_min_m.y, request.aabb_min_m.z = -1.0, -1.0, -0.3
request.aabb_size_m.x, request.aabb_size_m.y, request.aabb_size_m.z = 2.0, 2.0, 1.0
future = client.call_async(request)
deadline = time.monotonic() + 12.0
while not future.done() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
if future.done():
    result = future.result()
    print('ESDF success/voxel/values:', result.success, result.voxel_size_m, len(result.esdf_and_gradients.data))
node.destroy_node(); rclpy.shutdown()
PY
```

### Phase D 판정

| 요구 조건 | 판정 |
| --- | --- |
| 고정 생존자에 대해 camera XYZ 변화 | PASS, A→B에서 Z 약 0.418 m 변화 |
| 고정 생존자에 대해 map XYZ 유지 | PASS with raw-drift caveat, A→B map 평균 차이 약 0.115 m |
| A/B 위치 내부 map 안정성 | PASS, A 0.024 m / B 0.040 m planar max deviation |
| 이동 중 VSLAM/dual EKF | PASS |
| 이동 중 nvblox mesh/ESDF | PASS |
| 이동 중 Survivor camera/map output | PASS |

**Phase D: PASS with raw-coordinate caveat.** map 좌표는 로봇 이동을
상쇄하여 같은 실제 생존자 주변에 유지되었고, camera XYZ는 명확히
변했다. 다만 `0.115 m`의 A→B raw map 변화가 있으므로 위치 정밀도를
보증하지 않는다. 후속 Stage에서 이 값을 개선할 필요가 있으면 먼저
depth/ROI, image-depth synchronization, exact timestamp TF, VSLAM drift,
global EKF 및 extrinsic을 분리 분석해야 하며, 이번 Stage의 acceptance를
위해 filter를 추가하지 않았다.

## Stage 4 최종 판정

Phase A, B, C, D의 live/runtime 및 정적 증거를 종합하면 Stage 4의
요구 범위는 충족했다. exact detection timestamp TF2 변환, 안전한 TF
실패 처리, `map_positions` 출력, 정지 상태 검증, 실제 A→B 이동 안정성,
기존 VSLAM/EKF/nvblox/Survivor 회귀를 모두 확인했다.

**Stage 4: PASS with documented raw map-coordinate stability caveat.**
다음 Stage는 요청 범위대로 RViz Survivor Marker와 map-coordinate
visualization이며, 이번 Stage에서 의도적으로 구현하지 않았다.

## 최종 요구사항 완전성 대조표

이 표는 최종 문서 요구사항과 실제 근거를 번호별로 대조한다. C 위치는
안전·공간 제약으로 수행하지 않았으며 임의의 C 결과를 만들지 않고 N/A로
기록한다.

| 번호 | 요구사항 | 근거 / 상태 |
| ---: | --- | --- |
| 1 | Stage 4 개요 | Phase A~D 및 최종 판정 |
| 2 | 개발 목표 | Phase A 범위와 Phase D 목적 |
| 3 | Stage 3 인계 상태 | 시작 상태와 인계된 구조 |
| 4 | 시작 Git 상태 | `main`, `db1e715...`, 초기 status |
| 5 | 기존 Camera XYZ 구조 | 실제 `camera_positions` 계약 |
| 6 | VSLAM/EKF/nvblox 구조 | 인계 구조 및 회귀 표 |
| 7 | 실제 TF tree | `map → odom → base_link → camera_link → camera_color_optical_frame` |
| 8 | Exact timestamp 설계 | 입력 계약과 `Time.from_msg` |
| 9 | Latest TF를 쓰지 않는 이유 | 처리 지연과 촬영시각 pose 설명 |
| 10 | transform node 구조 | Phase B node 계약 |
| 11 | Input topic/message/QoS/frame/timestamp | `camera_positions`, `PoseArray`, reliable/volatile, optical frame, RGB stamp |
| 12 | Output topic/message/frame/timestamp | `map_positions`, `PoseArray`, `map`, 입력 stamp 보존 |
| 13 | Position transform 방식 | `PointStamped` + `do_transform_point` |
| 14 | Orientation 처리 | 출력 identity quaternion |
| 15 | TF lookup 구현 | exact timestamp `lookup_transform` |
| 16 | TF timeout | 기본 `0.2 s` |
| 17 | 오류 처리 | invalid input/TF, lookup/connectivity/extrapolation/timeout, NaN/Inf |
| 18 | 신규/수정 파일 | Phase B 파일 목록 |
| 19 | diff 요약 | package/setup dependency 및 entry point |
| 20 | Build 절차 | 실제 colcon build 명령 |
| 21 | 전체 실행 순서 | Phase C Terminal 1~7 및 종료 순서 |
| 22 | Camera XYZ 수동 확인 | camera `topic echo --once` |
| 23 | Map XYZ 수동 확인 | map `topic echo --once` |
| 24 | TF 수동 확인 | `tf2_echo` 명령과 실제 frame |
| 25 | VSLAM 수동 확인 | tracking hz 및 status probe |
| 26 | EKF 수동 확인 | local/global hz와 echo 명령 |
| 27 | nvblox 수동 확인 | mesh, ESDF service, RViz |
| 28 | ROS node/topic 확인 | `ros2 node list`, `topic info -v`, single owner |
| 29 | 정지 상태 검증 | 260 paired stamps, 255 valid pairs |
| 30 | 로봇 이동 검증 절차 | Phase D A→B 절차 |
| 31 | 위치 A/B/C 결과표 | 아래 표; C는 미수행 N/A |
| 32 | Map XYZ 안정성 통계 | A/B mean/min/max/spread |
| 33 | VSLAM 이동 결과 | status Success, 약 0.343 m 이동 |
| 34 | EKF 이동 결과 | wheel/local 약 0.351 m |
| 35 | nvblox 이동 결과 | mesh/ESDF 수치 |
| 36 | 오류와 해결 | sync/TF/test 오류 및 대응 |
| 37 | 알려진 제한사항 | raw drift, 짧은 VSLAM gap, C 미수행 |
| 38 | Stage 4 PASS/FAIL | PASS with raw-coordinate caveat |
| 39 | 최종 Architecture | Phase A 구조 및 consumer architecture |
| 40 | Quick Reference | 아래 명령 모음 |
| 41 | Rollback | 아래 non-destructive 절차 |
| 42 | 다음 Stage | Stage 5 RViz Marker + map visualization |

### 위치 A/B/C 결과표

| 위치 | Camera XYZ 평균 [m] | Map XYZ 평균 [m] | paired samples | 상태 |
| --- | --- | --- | ---: | --- |
| A | `(0.084, -0.465, 2.566)` | `(2.609, -0.040, 0.595)` | 129 | PASS |
| B | `(0.043, -0.379, 2.149)` | `(2.533, -0.058, 0.509)` | 135 | PASS |
| C | — | — | — | 미수행: 안전·공간 제약으로 A/B만 수행 |

### 최종 결과 보고

| 필드 | 실제 결과 |
| --- | --- |
| Stage 4 | PASS with raw-coordinate caveat |
| 시작 commit | `db1e71543236bd94328b33c0dd68634a08c97e9c` |
| 신규 node | `rescue_robot_survivor/survivor_map_transform_node.py` |
| Input / Output | `/leader/survivor/camera_positions` → `/leader/survivor/map_positions` |
| Source / Target | `camera_color_optical_frame` → `map` |
| Exact timestamp TF / latest fallback | 정상 / 없음 |
| Stationary / movement test | PASS / A→B 수행 |
| Camera XYZ 변화 | 정상; Z 평균 약 0.418 m 변화 |
| Map XYZ stability | 정상 동작; A→B raw 평균 차이 약 0.115 m |
| Map statistics | A/B 129/135쌍, 내부 planar 편차 0.024/0.040 m |
| VSLAM / Local EKF / Global EKF | 정상 |
| nvblox / Survivor Detector | 정상 |
| TF failure handling | 정상; failure probe에서 output 미발행 |
| 발견된 문제와 대응 | raw drift·sync warning·짧은 VSLAM gap을 기록하고 filter는 추가하지 않음 |
| 문서 / 명령 | 본 문서 / 완료 |
| 다음 단계 | Stage 5 — RViz Marker 및 map-coordinate visualization |

## 수동 검증 Quick Reference

각 실행 블록은 별도 terminal에서 사용한다. Terminal 1의 통합 실행기는
single RealSense owner, STM32, VSLAM, dual EKF, nvblox, RViz와 rosbag을
시작한다.

### Build 및 실행

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_survivor
source install/local_setup.bash
colcon test --packages-select rescue_robot_survivor --event-handlers console_direct+
colcon test-result --verbose

# Terminal 1
./scripts/run_vslam_mapping.sh

# Terminal 2
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py

# Terminal 3
./scripts/run_survivor_detector.sh

# Terminal 4
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py
```

Terminal 2~4에서는 실행 전에 `cd ~/damgc_robot`, ROS Humble setup,
`source install/local_setup.bash`를 수행한다.

### Camera/Map/TF/VSLAM/EKF/ROS

```bash
timeout 10 ros2 topic echo --once /leader/survivor/camera_positions
timeout 10 ros2 topic echo --once /leader/survivor/map_positions
ros2 topic info -v /leader/survivor/camera_positions
ros2 topic info -v /leader/survivor/map_positions
timeout 8 ros2 run tf2_ros tf2_echo map base_link
timeout 8 ros2 run tf2_ros tf2_echo map camera_color_optical_frame
timeout 8 ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
timeout 10 ros2 topic hz /visual_slam/tracking/odometry
timeout 10 ros2 topic hz /leader/odometry/local
timeout 10 ros2 topic hz /leader/odometry/global
timeout 10 ros2 topic echo --once /leader/odometry/local
timeout 10 ros2 topic echo --once /leader/odometry/global
ros2 node list
ros2 topic info -v /leader/camera/infra1/image_rect_raw
ros2 topic info -v /leader/camera/infra2/image_rect_raw
```

Isaac ROS status 확인:

```bash
docker exec isaac_ros_dev-aarch64-container bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  timeout 8 ros2 topic echo --once /visual_slam/status
'
```

### nvblox

`static_esdf_pointcloud`가 3D mode에서 없더라도 FAIL로 처리하지 않는다.
mesh, ESDF service, RViz map을 함께 확인한다.

```bash
ros2 node list | rg nvblox
ros2 topic info -v /nvblox_node/mesh
ros2 service list | rg 'nvblox_node/get_esdf_and_gradient'
ros2 topic info -v /leader/camera/color/image_raw
ros2 topic info -v /leader/camera/depth/image_rect_raw
```

실제 ESDF probe는 `EsdfAndGradients.Request`를 사용하고
`update_esdf=false`, `visualize_esdf=false`, `frame_id=odom`, AABB
min `(-1.0,-1.0,-0.3)`, size `(2.0,2.0,1.0)`을 요청한다. 관측 결과는
`success=true`, voxel `0.05 m`, `35,301 values`였다. Phase D mesh는
11 messages, vertex 포함 8개, 최대 17,944 vertices였다.

### 종료

이동 후 teleop에서 `Space`로 정지한다. 종료 순서는 Terminal 4 transform,
Terminal 3 detector, Terminal 2 preprocessing, Terminal 1 mapping 순으로
각각 `Ctrl+C`이다. 마지막에 `ros2 node list`와 `docker ps`가 비어 있는지
확인한다.

## Rollback 및 Git 권장 절차

Stage 4 변경 파일은 `package.xml`, `setup.py`,
`survivor_map_transform_node.py`, `survivor_map_transform.launch.py`,
`test_survivor_map_transform_node.py`, 이 문서다. 기존 사용자 변경을
보존하기 위해 `git reset --hard`나 broad recursive 삭제는 사용하지 않는다.

```bash
cd ~/damgc_robot
git status
git diff
git diff --check
```

새 기능만 비활성화하려면 Terminal 4를 실행하지 않고 기존 Terminal 1~3만
실행한다. 파일 복구는 사용자가 승인한 파일만 개별 검토 후 수행한다.
자동 commit과 `git push`는 수행하지 않았다. 사용자가 commit을 승인할 때:

```bash
git add src/leader/rescue_robot_survivor/package.xml \
  src/leader/rescue_robot_survivor/setup.py \
  src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_map_transform_node.py \
  src/leader/rescue_robot_survivor/launch/survivor_map_transform.launch.py \
  src/leader/rescue_robot_survivor/test/test_survivor_map_transform_node.py \
  docs/SURVIVOR_VSLAM_MAP_INTEGRATION_STAGE4_MAP_TRANSFORM_VALIDATION.md
git commit -m "Add exact-timestamp survivor camera-to-map transform"
# git push는 별도 승인 없이는 실행하지 않는다.
```
