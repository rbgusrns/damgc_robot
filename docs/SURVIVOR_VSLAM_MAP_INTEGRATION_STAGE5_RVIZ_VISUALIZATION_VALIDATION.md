# Survivor–VSLAM Map Integration Stage 5: RViz Visualization Validation

## Stage 5 목적과 현재 범위

`/leader/survivor/map_positions`의 현재 검출 후보를 RViz의 nvblox 3D map 위에
위치 marker와 좌표 text로 표시한다. Stage 5에는 persistent survivor ID,
spatial association, 중복 제거, 위치 평균·filter, registry를 넣지 않는다.

이 문서는 Phase A 사전 조사부터 Phase D 실물 검증까지의 누적 기록이다.
Visualizer, MarkerArray topic 및 RViz Survivor display는 Phase B에서
구현했고, Phase C와 Phase D에서 실제 Jetson/D435로 검증했다. **최종 Stage 5
판정은 문서 끝의 PASS/FAIL 표를 기준으로 한다.** Stage 4의 exact-timestamp
camera→map 변환을 비롯한 보호 대상은 변경하지 않았다.

## Phase A — 시작 상태와 보호 대상

2026-09-18 KST, Jetson Orin Nano에서 확인했다.

| 항목 | 확인값 |
| --- | --- |
| branch | `main` |
| `git log -1 --oneline` | `2608e90 Move follower_Leader Tracking to follower root` |
| `git rev-parse HEAD` | `2608e90ddd8a47e7e3412e11d54d4e3be1ca29ba` |
| 시작 `git status --short --branch` | `## main...origin/main`; 기존 uncommitted change 없음 |
| 시작 ROS graph / Docker | ROS node 없음 / 실행 중인 container 없음 |
| 장비 | `aarch64`; USB `8086:0b07 Intel Corp. RealSense D435` |

확인에 사용한 명령:

```bash
cd ~/damgc_robot
git status --short --branch
git branch --show-current
git log -1 --oneline
git rev-parse HEAD
source /opt/ros/humble/setup.bash
ros2 node list
docker ps --format '{{.Names}} {{.Status}}'
lsusb
```

보호 대상은 `scripts/run_vslam_mapping.sh`, Visual SLAM/dual EKF/nvblox 구성,
TF ownership, RealSense single-owner 실행, survivor preprocessing·detector,
`survivor_map_transform_node.py`의 timestamp TF 변환, 기존 두 PoseArray 계약,
`camera_apriltag.launch.py`이다. Phase A에서는 이들에 손대지 않았다.

## Stage 4 인계 및 실제 `map_positions` 계약

`person_detector_node.py`는 rectified RGB 한 프레임마다
`/leader/survivor/camera_positions` (`geometry_msgs/msg/PoseArray`)를 발행한다.
유효한 depth/XYZ가 없어도 **빈 `poses`를 발행**한다. header는 원본 RGB
image header이며 camera optical frame과 촬영 시각을 담는다. orientation은
측정된 사람 방향이 아닌 identity placeholder다. 유효 XYZ만 배열에 넣으므로
detector 화면의 `personN`과 PoseArray 인덱스가 항상 일치하지는 않는다.

`survivor_map_transform_node.py`는 이 배열을 소비해 입력 header stamp 시각의
TF만 조회한다. 출력 `/leader/survivor/map_positions`의 타입은
`geometry_msgs/msg/PoseArray`, frame은 `map`, stamp는 입력 RGB stamp 그대로다.
pose는 meter 단위 map XYZ이며 orientation은 `(0,0,0,1)`이다. 여러 pose의
순서는 입력 순서를 유지하지만 persistent identity를 뜻하지 않는다.
실제 graph의 publisher는 1개, QoS는 reliable/volatile이며 코드의 depth는 1이다
(CLI의 history depth 표기는 `UNKNOWN`).

**No-detection 동작:** transform callback은 빈 camera PoseArray를 받으면
즉시 return한다. TF 실패, 잘못된 header 또는 비정상 XYZ에서도 출력하지
않는다. 따라서 빈 map PoseArray로 검출 소실을 알리는 계약이 아니다.
이번 live 25초 probe에서도 camera 152건 중 빈 배열 85건, map 64건 중
빈 배열 0건이었다. 빈 camera 메시지와 같은 stamp의 map 메시지는 0건이었다.

실제 `ros2 topic echo --once /leader/survivor/map_positions` sample:

```yaml
header:
  stamp: {sec: 1789729665, nanosec: 338680908}
  frame_id: map
poses:
- position: {x: 2.583513285470672, y: 0.19190136627622575, z: 0.5644829231509233}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

이 25초 probe의 map 수신 간격은 평균 0.358초, 중앙값 0.273초,
최대 1.470초였고 첫 메시지부터 마지막 메시지까지의 평균 수신률은
약 **2.79 Hz**였다. 사람 검출과 유효 depth가 간헐적이므로 이 값은
카메라 FPS나 지속 검출 보장치가 아니다. Stage 4의 짧은 실측은
약 4.2–4.5 Hz였다. Phase B에서 lifetime은 정상 구간의 실제 최대 간격을
다시 측정해 재검증한다.

빈 camera 배열과 map 출력을 같은 25초 동안 비교한 probe는 다음과 같다.
별도 터미널에서 Humble setup 후 실행한다.

```bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import time
import statistics
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

rclpy.init()
node = rclpy.create_node('stage5_phase_a_probe')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST)
camera, mapped = [], []
def capture(dst):
    def callback(msg):
        dst.append((time.monotonic(), msg))
    return callback
node.create_subscription(PoseArray, '/leader/survivor/camera_positions',
                         capture(camera), qos)
node.create_subscription(PoseArray, '/leader/survivor/map_positions',
                         capture(mapped), qos)
start = time.monotonic()
while time.monotonic() - start < 25:
    rclpy.spin_once(node, timeout_sec=0.1)
def stamp(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)
print('elapsed_sec', round(time.monotonic() - start, 2))
print('camera_total', len(camera), 'camera_empty',
      sum(not msg.poses for _, msg in camera), 'camera_nonempty',
      sum(bool(msg.poses) for _, msg in camera))
print('map_total', len(mapped), 'map_empty',
      sum(not msg.poses for _, msg in mapped), 'map_nonempty',
      sum(bool(msg.poses) for _, msg in mapped))
if mapped:
    gaps = [b[0] - a[0] for a, b in zip(mapped, mapped[1:])]
    print('map_gap_mean_median_max_sec',
          tuple(round(v, 3) for v in (statistics.mean(gaps),
                                      statistics.median(gaps), max(gaps)))
          if gaps else 'n/a')
    print('map_rate_hz',
          round((len(mapped) - 1) / (mapped[-1][0] - mapped[0][0]), 3)
          if len(mapped) > 1 else 'n/a')
    msg = mapped[0][1]
    print('first_map', stamp(msg), msg.header.frame_id,
          [(p.position.x, p.position.y, p.position.z, p.orientation.w)
           for p in msg.poses])
empty_stamps = {stamp(msg) for _, msg in camera if not msg.poses}
map_stamps = {stamp(msg) for _, msg in mapped}
print('empty_camera_stamp_in_map', len(empty_stamps & map_stamps))
print('empty_camera_examples', list(sorted(empty_stamps))[:3])
node.destroy_node()
rclpy.shutdown()
PY
```

## Phase A 당시 RViz와 nvblox 구조

`rviz/vslam_nvblox.rviz`의 Fixed Frame은 `map`이다. 활성 display에는
TF, RobotModel, VSLAM `/visual_slam/tracking/vo_path` Path,
`/visual_slam/vis/slam_odometry` Odometry,
`/nvblox_node/mesh` NvbloxMesh, infra1 rectified Image가 있다.
`/nvblox_node/static_esdf_pointcloud` PointCloud2 display도 현재 활성 상태다.
RobotModel display는 활성화되어 있으나 config의 Description Topic 값이
비어 있으므로, Phase A는 RobotModel 렌더링 성공까지 주장하지 않는다.
Survivor MarkerArray display는 아직 없다.

`visual_slam_nvblox_realsense.launch.py`는 localization, VSLAM 및 nvblox
launch를 포함한다. `nvblox_realsense.launch.py`는 단일 depth/color 카메라,
`static_tsdf`, voxel `0.05 m`, `esdf_mode=3d`, mesh update `5 Hz`를
설정한다. 실제 graph에서 `/nvblox_node/mesh`는 publisher 1개와 RViz
subscriber 1개가 있었고, 14개 mesh sample 모두 vertex를 포함했다
(최대 13,957개). `/nvblox_node/get_esdf_and_gradient` 호출은
`success=true`, voxel `0.05 m`, 35,301 values였다. RViz 화면에 3D mesh가
보였다. 임시 화면 확인 파일은 `/tmp/stage5_phase_a_rviz_20260918.png`이며
repository 산출물은 아니다.

`static_esdf_pointcloud`는 graph에 publisher/subscriber 각각 1개가
광고됐지만 8초간 `echo --once --field header`에서 메시지를 받지 못했다
(`timeout` exit 124). 현재 3D ESDF의 PASS 근거는 이 topic이 아니라
mesh와 ESDF service 응답이다. 이 display는 Phase B에서 현재 모드에 맞게
정리할 예정이다.

실제 mesh vertex와 ESDF service 확인에 사용한 probe:

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
node = Node('stage5_phase_a_nvblox_probe')
meshes = []
node.create_subscription(
    Mesh, '/nvblox_node/mesh',
    lambda m: meshes.append(sum(len(block.vertices) for block in m.blocks)), 10)
client = node.create_client(
    EsdfAndGradients, '/nvblox_node/get_esdf_and_gradient')
available = client.wait_for_service(timeout_sec=5.0)
print('service_available', available)
if available:
    request = EsdfAndGradients.Request()
    request.update_esdf = False
    request.visualize_esdf = False
    request.use_aabb = True
    request.frame_id = 'odom'
    request.aabb_min_m.x, request.aabb_min_m.y, request.aabb_min_m.z = -1.0, -1.0, -0.3
    request.aabb_size_m.x, request.aabb_size_m.y, request.aabb_size_m.z = 2.0, 2.0, 1.0
    future = client.call_async(request)
    deadline = time.monotonic() + 12
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if future.done():
        result = future.result()
        print('esdf_success_voxel_values', result.success,
              result.voxel_size_m, len(result.esdf_and_gradients.data))
    else:
        print('esdf_timeout')
deadline = time.monotonic() + 5
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
print('mesh_samples_nonzero_max_vertices', len(meshes),
      sum(v > 0 for v in meshes), max(meshes, default=0))
node.destroy_node()
rclpy.shutdown()
PY
```

## 실제 전체 runtime 조사

기존 Stage 4 실행 구성대로 각각 별도 터미널에서 실행했다. Terminal 2~4는
실행 전 `cd ~/damgc_robot`, Humble setup과 `source install/local_setup.bash`를
사용했다. detector는 기존 전용 Docker 실행기를 사용했고 CUDA device `0`을
선택했다. 주행 키를 누르지 않고 정지 상태로 관찰했다.

```bash
# Terminal 1
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh

# Terminal 2
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py

# Terminal 3
cd ~/damgc_robot
./scripts/run_survivor_detector.sh

# Terminal 4
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py
```

실제로 사용한 주요 확인 명령:

```bash
ros2 node list
ros2 topic info -v /leader/survivor/camera_positions
ros2 topic info -v /leader/survivor/map_positions
timeout 15 ros2 topic echo --once /leader/survivor/map_positions
timeout 10 ros2 topic hz /visual_slam/tracking/odometry
timeout 8 ros2 topic hz /leader/odometry/local
timeout 8 ros2 topic hz /leader/odometry/global
ros2 topic info -v /nvblox_node/mesh
ros2 service list | rg nvblox_node
timeout 8 ros2 topic echo --once /nvblox_node/static_esdf_pointcloud --field header
timeout 8 ros2 run tf2_ros tf2_echo map base_link
```

실제 node에는 단일 `/leader/camera`, `/visual_slam_node`, local/global EKF,
`/nvblox_node`, `/leader/person_detector`, `/leader/survivor_map_transform`가
함께 있었다. infra1/infra2 topic의 publisher는 각각 1개였고,
`/leader/camera/color/image_raw` publisher도 `/leader/camera` 1개였다.
VSLAM tracking odometry의 짧은 `topic hz`는 약 5.02 Hz,
`/visual_slam/status` sample은 `vo_state: 1`(Success)이었다.
local/global EKF는 각각 약 29.5/29.2 Hz였고 `map→base_link` TF가
조회됐다. detector의 `camera_positions`와 transform의 `map_positions`는
실제 발행됐다.

이 실행에서 rectifier의 image/CameraInfo sync warning과 detector의
RGB/aligned-depth timestamp 차이 warning이 간헐적으로 나타났다.
정상적인 map XYZ sample도 계속 나왔다. 종료 Ctrl+C에서 기존 TF listener와
CameraInfo bridge에 `rcl_shutdown already called` 류 예외가 보였으나
실행 중 데이터 경로의 실패 근거는 아니며 Stage 5 Phase A에서는 수정하지 않았다.
실행 log/bag 시작 경로는 각각 `log/vslam_mapping_20260918_200707/`,
`data/vslam_mapping_20260918_200707/`이다.
종료 시 자동 분석된 134.0초 bag에는 VSLAM tracking odometry 571건
(평균 4.5 Hz, 최대 수신 공백 5.65초), status Success 571건/Failed 0건,
local/global EKF 평균 29.4/27.7 Hz가 기록됐다. 이 간헐 공백 때문에
2초 marker lifetime에서도 일시적인 표시 소실이 가능한지는 Phase C에서
실제 RViz로 확인해야 한다. 분석 파일은 위 bag의 `analysis.md`이다.
종료 후 ROS node와 Docker container가 남지 않은 것도 확인했다.

## Phase B 구현 지침: marker와 stale 처리

`/leader/survivor/map_positions`만 구독하는 visualizer를 추가한다.
유효한 각 pose의 배열 인덱스 `i`마다 map XYZ에 `SPHERE` 위치 marker
(ID `2*i`), 위쪽에 `TEXT_VIEW_FACING` 좌표 marker (ID `2*i+1`)를 만든다.
두 marker의 namespace는 `survivor_current`, header frame/stamp는 입력을
보존하고 orientation은 identity로 한다. text는 `Survivor candidate N`과
원래 map X/Y/Z를 표시한다. `N=i+1`은 프레임 내 후보 번호이며 영구 ID가
아니다. color alpha는 0보다 큰 유효값으로 설정한다.

**권장 기본 `marker_lifetime_sec=2.0`**. 이번 실측의 최대 map 수신 간격
1.47초보다 여유가 있고, 입력이 끊겼을 때 무한히 남지 않는다. 1초 기본값은
현재 지속 검출 중에도 flicker를 만들 수 있다. Phase C/D에서 실제 지속
검출과 FOV 이탈을 보고 필요 시 값을 재평가한다. `lifetime=0`은 금지한다.

stale marker 정책은 두 경로를 함께 사용한다.

1. 새 PoseArray에서 후보 수가 감소하거나 유효 후보 인덱스가 사라지면 이전
   활성 인덱스를 추적해 해당 sphere/text ID 모두에 `DELETE`를 발행한다.
   빈 PoseArray가 들어오면 모두 삭제한다.
2. transform이 아무 메시지도 내지 않는 no-detection/TF 실패 구간에는
   RViz의 finite marker lifetime으로 자동 삭제한다.

`DELETEALL`을 매 callback에 사용하지 않는다. NaN/Inf XYZ는 해당 후보만
건너뛰되 다른 유효 후보는 표시한다. 빈 frame ID는 marker를 만들지 않고
경고를 제한한다. input QoS는 map publisher와 호환되는
reliable/volatile/keep-last 1을 기본으로 한다.

예상 파일은 `rescue_robot_survivor/survivor_map_visualizer_node.py`,
`launch/survivor_map_visualizer.launch.py`, unit test, package `setup.py`와
`package.xml`의 필요한 entry point/의존성, `rviz/vslam_nvblox.rviz`,
그리고 이 문서다. README 상태 표기는 **Stage 5 최종 PASS 후**에만 갱신한다.

## Phase A 판정과 다음 단계

| PASS 조건 | 결과 |
| --- | --- |
| map_positions type/frame/stamp/poses/orientation | PASS: PoseArray, map, 입력 RGB stamp, meter XYZ, identity orientation |
| no-detection 동작 | PASS: camera empty 발행, transform은 map 출력 중단 |
| 실제 map sample/rate | PASS: 1 pose sample, 25초 64건/약 2.79 Hz |
| RViz 현재 config | PASS: Fixed Frame map, 각 display 확인 |
| nvblox visualization | PASS: vertex mesh, RViz 화면, ESDF service 확인 |
| finite lifetime 및 stale 전략 | PASS: 기본 2.0초, 감소 시 DELETE + 무발행 시 lifetime |
| 수정 예정 파일 | PASS: 상기 파일로 확정 |

**Stage 5 Phase A: PASS.** 다음 Phase B는 visualizer 구현과 자동 테스트,
RViz 설정 변경이다. 이번 PASS는 Stage 5 전체 PASS나 marker의 실제 RViz
표시 검증을 의미하지 않는다.

## Phase B — Survivor Map Visualizer 구현

2026-09-18 Phase B 시작 시 branch/HEAD는 Phase A와 같은 `main` /
`2608e90ddd8a47e7e3412e11d54d4e3be1ca29ba`였다. 기존 uncommitted
change는 Phase A에서 작성한 이 문서 한 개뿐이었으며 보존했다.
전체 VSLAM/Jetson 화면 통합 검증은 Phase C에서 수행한다.

### 신규 node, 메시지 계약과 marker 구조

`rescue_robot_survivor/survivor_map_visualizer_node.py`는
`/leader/survivor/map_positions` (`geometry_msgs/msg/PoseArray`)만 구독하고
`/leader/survivor/map_markers` (`visualization_msgs/msg/MarkerArray`)를
발행한다. 별도 `survivor_map_visualizer.launch.py`는 이 노드 하나만 실행한다.
RealSense, VSLAM, EKF, nvblox, detector 및 Stage 4 transform을 시작하지 않는다.

| 항목 | Phase B 구현 |
| --- | --- |
| marker namespace | `survivor_current` |
| 위치 marker | `SPHERE`, 입력 map XYZ, `id=2*i`, 지름 기본 `0.20 m` |
| 글자 marker | `TEXT_VIEW_FACING`, X/Y 동일, Z에 기본 `0.30 m` 추가, `id=2*i+1`, 글자 높이 `0.18 m` |
| 글자 | `Survivor candidate N`과 원래 map 좌표 `X=… Y=… Z=…`(소수 둘째 자리) |
| header | 두 marker 모두 입력 `frame_id`와 원본 RGB `stamp` 보존 |
| orientation | 두 marker 모두 identity `(0,0,0,1)`; 입력 pose의 방향은 사용하지 않음 |
| 색/alpha | sphere 주황색 alpha `0.95`, text 흰색 alpha `1.0` |
| 활성 marker lifetime | 기본 `2.0 s`; Humble `rclpy.duration.Duration(...).to_msg()` |
| 입출력 QoS | reliable/volatile/keep-last 1; 기존 map publisher와 호환 |

인덱스 `i`는 **현재 PoseArray의 배열 위치**다. detector의 `personN`이나
영구 생존자 ID와 동일하다고 주장하지 않는다. 일부 pose가 NaN/Inf여서
건너뛰어지면 남은 후보는 원래 인덱스를 유지하므로 ID 사이에 빈 번호가
있을 수 있다. 한 pose의 오류로 다른 valid candidate를 버리지 않는다.

이전 입력에서 사용한 활성 인덱스를 보관하고 새 입력에서 사라진 인덱스의
sphere와 text에 `DELETE`를 발행한다. 빈 PoseArray를 받으면 모두 DELETE한다.
입력 자체가 끊기면 활성 marker의 유한 lifetime이 RViz에서 만료된다.
`DELETEALL`은 사용하지 않는다. DELETE 메시지 자체의 기본 lifetime 0은
표시 객체가 아니라 삭제 명령이므로, 활성 marker의 finite lifetime 정책과
다르다. 빈 `frame_id`는 marker 생성 없이 5초 제한 경고를 남긴다.
나쁜 XYZ는 개별 skip하며 경고도 5초로 제한한다.

### 파라미터, 설치 및 변경 파일

| parameter | 기본값 | 검증 |
| --- | --- | --- |
| `input_topic` | `/leader/survivor/map_positions` | 비어 있지 않고 출력과 달라야 함 |
| `output_topic` | `/leader/survivor/map_markers` | 비어 있지 않아야 함 |
| `marker_namespace` | `survivor_current` | 비어 있지 않아야 함 |
| `marker_lifetime_sec` | `2.0` | 유한 양수; 0 금지 |
| `marker_scale` | `0.20` | 유한 양수, sphere X/Y/Z |
| `text_height` | `0.18` | 유한 양수, text scale Z |
| `text_z_offset` | `0.30` | 유한한 0 이상 |

신규 파일은 visualizer node, 단독 launch, `test/test_survivor_map_visualizer_node.py`다.
기존 `setup.py`에는 console entry point만, `package.xml`에는 실제 사용하는
`visualization_msgs` exec dependency만 추가했다. 기존
`glob("launch/*.launch.py")`가 신규 launch를 설치한다.
`rviz/vslam_nvblox.rviz`에는 `Survivors` MarkerArray display만 추가했다.
Fixed Frame `map`과 기존 nvblox Mesh, TF, RobotModel, VSLAM display는
그대로 유지했다. `static_esdf_pointcloud` display는 Phase A에서 8초간
메시지를 받지 못했으나, Phase B 범위의 대규모 RViz 정리를 피하려고
기존 상태로 두었다. 3D ESDF의 성공 기준은 계속 mesh와 service다.

### Build, 자동 테스트, 설치 확인

실제로 실행한 명령:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_survivor
source install/local_setup.bash
colcon test --packages-select rescue_robot_survivor --event-handlers console_direct+
colcon test-result --verbose
ros2 pkg executables rescue_robot_survivor
ros2 launch rescue_robot_survivor survivor_map_visualizer.launch.py --show-args
python3 -m flake8 \
  src/leader/rescue_robot_survivor/rescue_robot_survivor/survivor_map_visualizer_node.py \
  src/leader/rescue_robot_survivor/launch/survivor_map_visualizer.launch.py \
  src/leader/rescue_robot_survivor/test/test_survivor_map_visualizer_node.py
git diff --check
```

Build 성공, package pytest **53/53 PASS**(신규 visualizer 테스트 7개 포함),
flake8 및 `git diff --check` 통과했다. `colcon test-result --verbose`는
전체 workspace 누적 결과 `605 tests, 0 errors, 0 failures, 0 skipped`를
표시했다. 신규 테스트는 1명/다중 후보의 `2*N` marker, frame/stamp,
XYZ, label, offset, identity quaternion, alpha, finite lifetime,
NaN/±Inf 개별 skip, 후보 감소·빈 배열 DELETE, callback 상태,
잘못된 파라미터 거부를 검사한다.

설치된 실행 파일 목록에서 기존 detector·map transform과 새
`survivor_map_visualizer_node`가 확인됐다. `--show-args`는 위 7개
파라미터의 launch 기본값을 표시했다. RViz config는 YAML로 파싱됐고
`Fixed Frame: map`, 기존 NvbloxMesh, 새 Survivors MarkerArray topic을
확인했다.

### 전체 시스템 없이 실행한 ROS smoke test

다음 단독 실행에서 `/leader/survivor_map_visualizer`가 실제로 나타났고,
`map_positions` PoseArray subscription과 `map_markers` MarkerArray publisher는
reliable/volatile이었다. 이때 upstream map publisher나 RViz는 실행하지
않았으므로 endpoint 발견만으로 실제 3D 표시를 PASS 처리하지 않는다.

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_visualizer.launch.py

# 별도 terminal
source /opt/ros/humble/setup.bash
ros2 node info /leader/survivor_map_visualizer
ros2 topic info -v /leader/survivor/map_positions
ros2 topic info -v /leader/survivor/map_markers
```

같은 단독 실행에서 합성 PoseArray를 1명→2명→빈 배열 순서로 실제 ROS
topic에 publish하고 MarkerArray subscriber로 수신했다. 관측값:

실제로 사용한 합성 probe 명령은 별도 terminal에서 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

rclpy.init()
node = rclpy.create_node('stage5_phase_b_smoke')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE)
pub = node.create_publisher(PoseArray, '/leader/survivor/map_positions', qos)
received = []
node.create_subscription(MarkerArray, '/leader/survivor/map_markers',
                         lambda m: received.append(m), qos)
deadline = time.monotonic() + 5
while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
print('input_subscribers', pub.get_subscription_count())
frames = (
    [(2.53, -0.06, 0.51)],
    [(2.53, -0.06, 0.51), (3.0, 0.2, 0.6)],
    [],
)
for seq, points in enumerate(frames, start=1):
    msg = PoseArray()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 100 + seq
    for x, y, z in points:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        msg.poses.append(pose)
    target = len(received) + 1
    pub.publish(msg)
    deadline = time.monotonic() + 5
    while len(received) < target and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(received) < target:
        print('missing_output_for_seq', seq)
        break
    markers = received[-1].markers
    print('seq', seq, 'input_poses', len(points), 'markers',
          [(m.id, m.action, m.type, m.header.frame_id,
            m.header.stamp.sec, m.lifetime.sec) for m in markers])
node.destroy_node()
rclpy.shutdown()
PY
```

| 입력 | 출력 ID/action/type | frame/stamp/lifetime |
| --- | --- | --- |
| 1 pose | `0 ADD SPHERE`, `1 ADD TEXT_VIEW_FACING` | `map`, sec `101`, `2 s` |
| 2 poses | `0…3 ADD`, 위치·글자 각각 2개 | `map`, sec `102`, `2 s` |
| 0 poses | `0…3 DELETE` | `map`, sec `103`; DELETE lifetime은 무관 |

합성 probe는 새 노드의 메시지 왕복과 명시적 삭제를 입증한다.
실제 사람·RViz 화면·FOV 이탈 후의 시간 기반 소멸은 Phase C/D의 검증 대상이다.
smoke 종료 시 노드는 Ctrl+C로 정상 종료됐다.

### Static review 및 Phase B 판정

`git diff`와 `git status`에서 변경 범위는 새 node/launch/test, 이 문서,
`setup.py`, `package.xml`, RViz config뿐이었다. VSLAM·dual EKF·nvblox,
`person_detector_node.py`, `survivor_map_transform_node.py`, RealSense owner,
survivor preprocessing 및 기존 launch에 diff가 없었다. 자동 commit/push는
수행하지 않았다.

| Phase B 조건 | 결과 |
| --- | --- |
| Visualizer node / launch | PASS: 단독 실행 및 endpoint 확인 |
| MarkerArray 계약 / multi-person | PASS: pure tests + 1→2명 ROS smoke |
| 2초 finite lifetime / stale cleanup | PASS: 활성 ADD lifetime 및 감소·empty DELETE 구현; 무입력 만료의 실제 RViz 확인은 Phase C/D |
| invalid XYZ / parameters | PASS: unit test |
| package build / unit tests | PASS: build, 53/53 tests |
| RViz config | PASS: Survivors display 추가, Fixed Frame map 유지 |
| 보호 대상 변경 | 없음 |

**Stage 5 Phase B: PASS. `/goal 3` 진행 가능: YES.** 이는 구현과 단독
ROS smoke의 판정이다. Stage 5 최종 PASS는 실제 Jetson의 mesh·marker 동시
표시, 사람 이동과 FOV 이탈 후 만료, 기존 시스템 회귀를 확인한 후에만 한다.

## Phase C — 정지 상태 실제 통합 검증

2026-09-18 Jetson + D435에서 로봇을 움직이지 않고 기존 mapping stack,
preprocessing, detector, map transform과 visualizer를 동시에 실행했다.
single RealSense owner는 `/leader/camera` 한 개였고 VSLAM, dual EKF,
nvblox, survivor 경로가 함께 존재했다. Detector debug image에는 실제
사람 1명이 검출됐으며 `/tmp/stage5_phase_c_color.png`에 임시 저장했다.

### 실제 Terminal별 Quick Reference

```text
Terminal 1: ./scripts/run_vslam_mapping.sh
Terminal 2: ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py
Terminal 3: ./scripts/run_survivor_detector.sh
Terminal 4: ros2 launch rescue_robot_survivor survivor_map_transform.launch.py
Terminal 5: ros2 launch rescue_robot_survivor survivor_map_visualizer.launch.py
Terminal 6: timeout 15 ros2 topic echo --once /leader/survivor/map_positions
Terminal 7: timeout 15 ros2 topic echo --once /leader/survivor/map_markers
Terminal 8: ros2 topic hz /visual_slam/tracking/odometry
             ros2 topic hz /leader/odometry/local
             ros2 topic hz /leader/odometry/global
             ros2 run tf2_ros tf2_echo map base_link
             ros2 topic info -v /nvblox_node/mesh
```

For Terminals 2, 4, 5, 6, 7 and 8, the exact setup prefix used was:
`cd ~/damgc_robot && source /opt/ros/humble/setup.bash && source install/local_setup.bash`.
Terminal 3 used the existing `./scripts/run_survivor_detector.sh` container
launcher without additional arguments.

The exact Phase C visualizer and topic commands were:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_visualizer.launch.py

source /opt/ros/humble/setup.bash
timeout 15 ros2 topic echo --once /leader/survivor/map_positions
timeout 15 ros2 topic echo --once /leader/survivor/map_markers
ros2 topic info -v /leader/survivor/map_positions
ros2 topic info -v /leader/survivor/map_markers
timeout 10 ros2 topic hz /visual_slam/tracking/odometry
timeout 10 ros2 topic hz /leader/odometry/local
timeout 10 ros2 topic hz /leader/odometry/global
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

Terminal 1 used the existing `rviz/vslam_nvblox.rviz`; the visualizer launch
started no camera, VSLAM, EKF, nvblox, detector or map transform node.

### MarkerArray and coordinate result

The live graph showed `/leader/survivor/map_markers` as
`visualization_msgs/msg/MarkerArray`, with the visualizer as publisher and RViz
as subscriber, both reliable/volatile. A paired 20-second subscriber received
eight non-empty `map_positions` messages and eight MarkerArray messages. The
last pair was:

```text
map_positions: frame_id=map, stamp=1789730962.873122803, poses=1
XYZ=(2.530186, -0.124492, 0.530441) m

map_markers: count=2
id=0, ns=survivor_current, action=ADD, type=SPHERE
  XYZ=(2.530186, -0.124492, 0.530441) m, lifetime=2.0 s
id=1, ns=survivor_current, action=ADD, type=TEXT_VIEW_FACING
  XYZ=(2.530186, -0.124492, 0.830441) m, lifetime=2.0 s
  text="Survivor candidate 1 | X=2.53 Y=-0.12 Z=0.53"
```

Sphere XYZ matched the source map pose exactly. Only text received the
intentional visual Z offset of `0.30 m`; no map-coordinate offset was added.
The marker scale was `0.20 m`. Marker and text were visible with the gray
nvblox mesh in RViz; the clean marker capture is
`/tmp/stage5_phase_c_rviz_marker.png`. Fixed Frame was `map` and the
`Survivors` display was enabled.

The same probe observed a maximum map update gap of `5.193 s` and about
`0.67 Hz` over that short interval. This upstream gap can let a 2-second marker
expire temporarily; Phase D may reassess the finite value without making it
infinite.

### Static map and nvblox result

The live node list included `/visual_slam_node`, both EKF nodes,
`/nvblox_node`, `/leader/person_detector`, `/leader/survivor_map_transform`,
`/leader/survivor_map_visualizer` and `/rviz`. Short rate observations were:

| stream | observed rate |
| --- | ---: |
| `/visual_slam/tracking/odometry` | 5.2 Hz in the 767.1 s bag |
| `/leader/odometry/local` | 29.5 Hz in the 767.1 s bag |
| `/leader/odometry/global` | 23.3 Hz in the 767.1 s bag |

VSLAM status reported `vo_state: 1` (Success). The Isaac ROS nvblox probe
returned:

```text
service_available True
esdf True 0.05000000074505806 35301
mesh_samples 53 nonzero 40 max_vertices 15818
```

RViz showed the nvblox mesh together with the survivor sphere and text.
`static_esdf_pointcloud` remains optional in this 3D mode; mesh and ESDF
service are the authoritative checks.

The mapping launcher finalized `data/vslam_mapping_20260918_202053/analysis.md`.
That 767.1 s stationary bag recorded VSLAM 3993/3993 Success, local EKF 22576
samples, global EKF 17853 samples, and VSLAM maximum gap 8.695 s. The longer
bag confirms the same pipeline remained alive, while also showing that upstream
processing gaps can exceed the 2-second marker lifetime.

### Input-loss / stale-marker check and reappearance

The camera view contained a real person before the loss check. Since a person
could not be physically moved during this unattended run, the first FOV check
used the exact downstream condition produced by that event: the detector was
stopped, so no new camera/map positions arrived. This is a controlled
input-loss test, not a claim that a human physically walked out of frame.

The detector stop was followed by a marker monitor that observed 59 MarkerArray
messages before the stop and no further messages; the last marker was followed
by `4.767 s` of quiet. With each ADD marker carrying a finite `2.0 s` lifetime,
RViz removed the sphere and text. The clean post-loss screen is
`/tmp/stage5_phase_c_after_loss.png`. A normal detector restart then produced
a new ADD pair, confirming reappearance (`frame_id=map`, stamp
`1789730732.078159180`, IDs 0 and 1, lifetime 2 s).

The controlled loss passed stale cleanup. A physical person walking out of the
D435 FOV remains a manual operator follow-up.

### Regression and load observations

With visualizer ON, VSLAM, local/global EKF, nvblox mesh/ESDF, detector, map
transform and RViz remained alive. The detector used CUDA device `0`.
Short `tegrastats --interval 1000` samples varied with the full workload and
showed no clear visualizer-specific increase:

```text
visualizer ON:  RAM 5453/7607 MB, CPU mostly 86–95%, GR3D 57–80%
visualizer OFF: RAM 5409/7607 MB, CPU mostly 86–97%, GR3D 33–98%
```

This is qualitative, not a performance benchmark.

### Phase C 판정

| 조건 | 결과 |
| --- | --- |
| map_markers publish | PASS: live MarkerArray publisher and RViz subscriber |
| frame/stamp/IDs/namespace/geometry | PASS: map, input stamp, IDs 0/1, exact XYZ |
| sphere and text visible | PASS: RViz screen with nvblox mesh |
| nvblox + marker | PASS: mesh vertex stream and ESDF service |
| stale marker removal | PASS for controlled upstream input loss; 2 s lifetime and 4.767 s quiet |
| marker reappearance | PASS: detector restart produced a new ADD pair |
| VSLAM / EKF / TF | PASS: status Success, short rates, map→base_link |
| survivor detector / map transform | PASS: real person debug image, pose and marker pair |
| physical person FOV exit | NOT RUN in unattended test; operator follow-up required |

**Stage 5 Phase C: PASS with a documented physical-FOV limitation.** Stationary
integrated marker display and downstream stale cleanup passed. The next Phase
is robot/person movement plus an operator-assisted physical FOV test.

## Phase D — 실제 이동 검증 및 수동 후속 항목

### 실행 세션과 위치 A

2026-09-18, Phase C와 동일한 다섯 실행 경로를 사용했다. 처음 실행한
`./scripts/run_vslam_mapping.sh`의 bag은
`data/vslam_mapping_20260918_204222`이며, 운영자에게 방향키를 넘기기
위해 `Ctrl+C`로 정상 종료했다. 실행기는 bag 분석을 마친 후 exit code 0으로
종료됐다. VSLAM을 재시작하면 새로운 `map` 원점이 정해질 수 있으므로, 첫
세션에서 얻은 A 값은 B와 비교하지 않는다. 운영자가 새 터미널에서 동일
스크립트를 다시 실행한 뒤, 새 세션의 A를 다시 측정했다.

새 세션에서 로봇 정지 중 15초 동안 `camera_positions`, `map_positions`,
`map_markers`를 구독하고 **같은 header stamp**의 단일 pose만 짝지었다.
11개의 정확히 일치하는 triplet을 얻었다. 평균 좌표는 다음과 같다(단위 m).

| 위치 | Camera X | Camera Y | Camera Z | Map X | Map Y | Map Z | Marker X | Marker Y | Marker Z |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| A (새 세션) | 0.1473 | -0.4023 | 2.3534 | 2.4813 | -0.1191 | 0.5322 | 2.4813 | -0.1191 | 0.5322 |
| B | 0.1133 | -0.3635 | 2.1808 | 2.4012 | -0.0102 | 0.4936 | 2.4012 | -0.0102 | 0.4936 |
| C (선택) | 수행하지 않음 | | | | | | | | |

A의 camera XYZ 범위는 `(0.0272, 0.0117, 0.0160)` m, map XYZ 범위는
`(0.0160, 0.0276, 0.0117)` m였다. 이 구간의 global odometry 첫/끝 XY는
각각 `(0.0857727076, 0.0028214804)` 및
`(0.0857727081, 0.0028214818)` m로 정지 상태였다. 모든 marker sphere
좌표는 대응하는 map pose와 정확히 일치했다.

운영자가 안전한 짧은 거리를 이동하고 정지한 B에서 같은 방법으로 15초간
34개의 정확히 일치하는 triplet을 측정했다. B의 camera XYZ 범위는
`(0.0989, 0.0145, 0.0140)` m, map XYZ 범위는
`(0.0143, 0.0996, 0.0145)` m였다. B 구간의 global odometry 첫/끝
XY는 `(0.1766425849, -0.0002068432)` 및
`(0.1764589005, -0.0003669411)` m로 정지 상태였다. A→B의 robot
odometry XY 차이는 약 `0.091 m`, camera XYZ 변화는
`(-0.0340, +0.0388, -0.1726)` m, map XYZ 변화는
`(-0.0801, +0.1089, -0.0386)` m이다. Map의 수평 차이는 약
`0.135 m`로, marker는 map pose와 정확히 같지만 해당 위치 오차를
포함한다. 사람의 실제 위치가 동일했다고 해도 이 데이터만으로 map 위치가
정확히 불변이라고 주장하지 않는다. 짧은 이동에 대한 map 좌표 잔차와
detector 중심점 변동 가능성을 제한사항으로 남긴다.

같은 시점에 graph에는 D435 한 노드, VSLAM, local/global EKF, nvblox,
detector, map transform, visualizer, RViz가 존재했다. `map_markers`는
`MarkerArray` publisher 1개 및 RViz subscriber 1개(reliable/volatile)였다.
`/visual_slam/status`는 `vo_state: 1`, ESDF service는
`/nvblox_node/get_esdf_and_gradient`로 확인했다. 이동 후 최종 회귀
측정과 물리 FOV·다중 사람 검증은 별도로 수행해야 한다.

운영자에게 안내한 수동 이동 명령은 다음과 같다. 실행기의 방향키 teleop은
20 Hz로 `/leader/cmd_vel`을 발행하므로 동시에 두 개를 실행하지 않는다.

```bash
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh
# 실행 터미널에 포커스를 둔 뒤 ↑를 아주 짧게 누르고 Space로 정지
# B 측정 전에는 Ctrl+C를 누르지 않는다. Ctrl+C는 매핑 전체를 종료한다.
```

### 사람 이동 (로봇 B에서 정지)

운영자가 화면 속 사람을 이동한 뒤 15초간 112개의 동일 stamp
camera/map/marker triplet을 얻었다. 이동 후 평균 camera XYZ는
`(0.2745, -0.4713, 2.7917)` m, map 및 marker XYZ는
`(3.0246, -0.1562, 0.6005)` m였다. 이동 전 B에 비해 camera는
`(+0.1612, -0.1078, +0.6109)` m, map과 marker는
`(+0.6234, -0.1460, +0.1069)` m 변화했다. 방향과 크기는 카메라 축과
map 축이 다르므로 각 축끼리 같을 필요는 없지만, map 및 marker는 같은
방향과 크기로 이동했다. 같은 측정 구간의 global odometry 첫/끝 XY는
`(0.1847624440, 0.0147128203)` 및 `(0.1847821539, 0.0147625372)` m로
거의 정지했다. B 측정과 사람 이동 측정 사이의 global odometry 차이
약 `(0.0081, 0.0151)` m는 localization 변동을 포함한다.

### 이동 후 회귀 단기 샘플

10초 read-only subscriber 관찰에서 infra1 149개(14.89 Hz), infra2
169개(16.89 Hz), VSLAM tracking odometry 48개(4.80 Hz), local EKF
285개(28.47 Hz), global EKF 252개(25.18 Hz), survivor camera
positions 80개(비어 있지 않은 것 69개), map positions 68개, marker
arrays 69개를 얻었다. `/visual_slam/status`는 `vo_state: 1`이었다.
이 값은 짧은 구간 관찰이며 전체 경로 안정성의 장시간 보증은 아니다.

물리 FOV 이탈·재진입, 다중 사람, candidate 감소와 최종 회귀 검증이
끝나기 전에는 Stage 5 전체 PASS로 판정하지 않는다.

### 물리 FOV 이탈과 재진입

운영자가 실제로 카메라 시야 밖으로 나갔다. `camera_positions`의 다음
메시지는 `poses: []`였고, 각각 5초 동안 `map_positions` 및
`map_markers`의 새 메시지를 기다리는 명령은 출력 없이 timeout(124)으로
끝났다. 이후 20초 관찰에서는 비어 있는 camera 메시지 178건,
비어 있지 않은 camera/map/marker 이벤트 0건이었다. 단, 첫 90초짜리
관찰에서는 이탈 전후를 한 구간에 섞어 수집했고 마지막에는 사람이
재진입하여 marker가 다시 갱신됐다. 이를 90초 연속 무검출로 해석하지
않는다.

운영자가 재진입했다고 확인한 후 10초 관찰에서 비어 있지 않은 camera
62건, map 56건, sphere marker ADD 60건을 받았다. 최신 map XYZ와
marker XYZ는 모두 `(2.452323, -0.267993, 0.492805)` m였고 marker
lifetime은 `2.0 s`였다. 재진입에 따른 marker 복구가 확인됐다.

화면 삭제를 직접 검증하기 위해 운영자가 다시 FOV 밖으로 나간 후,
마지막 ADD 수신부터 `5.033 s` 동안 새 ADD가 없고 camera empty가
지속된 순간 RViz를 캡처했다:
`/tmp/stage5_phase_d_quiet_5s.png`. 그 화면에는 기존 nvblox mesh와
RobotModel이 남아 있으나 survivor sphere/text는 없다. 따라서 실제
RViz 삭제는 **늦어도 5.033초까지** 완료됐다. Marker의 명시적 lifetime은
2.0초지만 RViz가 정확히 몇 밀리초에 삭제했는지는 측정하지 않았으므로
2.0초를 실측 삭제 지연으로 표기하지 않는다. FOV 밖에서도 짧은
간헐 검출이 발생할 수 있으므로, stale 조건은 마지막 유효 ADD부터의
무갱신 시간을 기준으로 판정한다.

재현 명령은 다음과 같다(사람이 시야 밖에 머무는 동안 실행).

```bash
source /opt/ros/humble/setup.bash
timeout 8 ros2 topic echo --once /leader/survivor/camera_positions --field poses
timeout 5 ros2 topic echo --once /leader/survivor/map_positions --field poses
timeout 5 ros2 topic echo --once /leader/survivor/map_markers --field markers
```

위의 timeout은 카메라가 빈 PoseArray를 발행하고 transform이 빈
map PoseArray를 발행하지 않는 현재 계약을 확인하기 위한 것이다.
아무 출력도 없다는 한 구간만으로 RViz 삭제를 주장하지 않고,
finite lifetime 및 실제 RViz 캡처와 함께 판정한다.

### Phase D 후반 회귀 점검 (다중 사람 테스트 전)

동시 실행 중 nvblox ESDF 요청은 `success=True`, voxel size
`0.05000000074505806 m`, 반환 데이터 35,301개였다. 5초 mesh 구독에서
20개 샘플 중 12개에 vertex가 있었고 최대 19,295 vertex였다.
`map → base_link` TF는 `tf2_echo`에서 실제 값을 반환했다(예: translation
`[0.229, -1.738, 0.000]`, yaw `119.9°`). `tf2_echo` 시작 직후
`map` frame discovery 메시지가 한 번 있었지만 이후 연속 transform을
받았으므로 이를 지속적인 TF error로 해석하지 않는다. 이 수치는
mapping/VSLAM의 실제 추정값이며 이전 B의 global odometry 수치와
좌표계를 혼동하지 않는다.

동일 작업공간에서 자동 테스트를 재실행했다:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select rescue_robot_survivor --event-handlers console_direct+
```

결과는 **53/53 PASS**(visualizer 신규 테스트 7개 포함)였다. 실물 다중
사람 및 candidate 감소가 완료되지 않았다면 자동 테스트 결과만으로
Stage 5 최종 PASS를 선언하지 않는다.

다중 사람 관찰 후 최종 10초 read-only sample에서는 infra1 80건
(`8.0 Hz`), infra2 76건(`7.6 Hz`), VSLAM tracking odometry 23건
(`2.3 Hz`), local EKF 294건(`29.38 Hz`), global EKF 249건
(`24.89 Hz`), camera positions 68건(유효 47건), map positions 46건,
marker arrays 46건이었다. 별도 status sample은 다시 `vo_state: 1`이었다.
이는 모든 stream이 살아 있다는 근거지만, 앞선 정지 구간보다 VSLAM/infra
rate가 낮아진 점은 기록한다. Mapping log에는 검사 시점까지 EKF의
`Failed to meet update rate` 경고가 288회 있었다. Phase C log에도
같은 유형 189회가 있어 visualizer가 새로 만든 오류라고 단정하지
않는다. 반면 `vslam_nvblox.log`의 검색에서는 반복된 TF lookup/
extrapolation 오류 및 `[ERROR]` 항목을 찾지 못했다. 장시간 성능 회귀를
완전히 배제하려면 별도 부하 비교가 필요하다.

### 다중 사람 실물 관찰

10초 구간의 실제 detector 출력에서 camera PoseArray 길이 분포는
`0:54, 1:22, 2:5`, map PoseArray는 `1:18, 2:4`였다.
MarkerArray에는 ADD 4개(IDs 0–3; 각 후보 sphere/text)가 5회,
2→1 감소 시 ADD 2개와 DELETE 2개(IDs 2/3)가 2회 관찰됐다.
map의 2인 예시 stamp `(1789732703, 342138672)`에는
`(-2.299, 0.959, 1.290)` m 및 `(-2.329, 1.510, 1.522)` m pose가
있었다. 그러나 이어진 15초 구간에는 2인 동시 pose가 없었다. 당시
detector debug image(`/tmp/stage5_phase_d_debug_latest.png`)에서는
사람이 책상·의자에 가려져 보였으므로, 이 10초 예비 이벤트만으로
다중 사람 실물 PASS를 선언하지 않았다.

운영자가 두 사람을 열린 시야에 위치시킨 뒤 60초 동시 구독에서
**camera 307개, map 306개, marker 306개의 2인 이상 이벤트 중
map/marker stamp-matched pair 306개**를 얻었다. 현재 배열 순서는
프레임마다 바뀔 수 있으므로 같은 candidate 번호를 고정 사람으로
해석하지 않는다. 대표적인 2인 stamp `(1789733061, 127071289)`:

| frame-local candidate | camera XYZ (m) | map XYZ (m) | sphere marker XYZ (m) | text marker Z (m) |
| --- | --- | --- | --- | ---: |
| 1 | (-0.701, -0.616, 3.451) | (-2.166, 0.906, 0.751) | (-2.166, 0.906, 0.751) | 1.051 |
| 2 | (0.461, -0.551, 3.132) | (-1.003, 1.220, 0.680) | (-1.003, 1.220, 0.680) | 0.980 |

이 pair의 marker는 `survivor_current` namespace의 sphere IDs `0,2`,
text IDs `1,3`이었다. text는 각 후보의 `Survivor candidate N` 및
소수 둘째 자리 XYZ를 포함했고, 두 sphere XYZ는 source map pose와
정확히 일치했다. 일부 프레임은 3 pose/6 ADD marker까지 발생했고,
3→1 전환에서는 IDs `2,3,4,5` 모두에 DELETE가 나왔다. 60초 동안
24개 DELETE 이벤트를 관찰했다. 세 번째 후보가 독립된 제3자인지는
검증하지 않았으므로 3인 실물 PASS는 주장하지 않는다.

RViz 화면 `/tmp/stage5_phase_d_multi_rviz_try.png`에는 nvblox 3D mesh와
복수 후보의 text가 동시에 나타났다. 그러나 현재 RViz 거리·시점과
사람 배치에서는 text가 시각적으로 겹쳐 일부 숫자를 읽기 어려웠다.
MarkerArray 메시지의 각 label과 좌표는 정상이며, 화면 확대/시점 변경이나
후속 label collision avoidance가 필요한 알려진 UI 제한으로 기록한다.

### 운영자 수동 후속 검증 — 2명 유지 → 1명 퇴장

운영자 요청으로 여기서 사람 이동 실험을 멈췄다. 최종 수동 확인 시에는
로봇을 안전하게 정지시키고 두 사람의 몸이 D435 영상에서 서로 및
책상·의자에 가려지지 않도록 간격을 둔다. RViz의 Fixed Frame `map`,
`NvbloxMesh`, `Survivors`를 켜고 두 sphere 및 두 text가 동시에 보이는
시점을 확인한다. 글자가 겹치면 RViz 화면을 확대·회전하거나 두 사람의
간격을 넓힌 뒤 각 좌표가 읽히는지 확인한다. 한 명만 FOV 밖으로 나간
뒤 남은 한 명의 marker/text만 남고, 떠난 후보의 marker/text가
DELETE 또는 최대 2초 lifetime 후 사라지는지 확인한다. 이때 후보
번호 변경은 정상이며 동일 인물의 persistent ID로 해석하지 않는다.

ROS 메시지 수/DELETE를 보는 실제 재현 명령:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node('stage5_manual_multi_check')
def mapped(msg):
    if msg.poses:
        print('MAP', msg.header.stamp.sec, msg.header.stamp.nanosec,
              'candidates', len(msg.poses), flush=True)
def markers(msg):
    adds = [m for m in msg.markers if m.action == 0]
    deletes = [m.id for m in msg.markers if m.action == 2]
    if adds or deletes:
        print('MARKERS', 'ADD', len(adds), 'DELETE IDs', deletes,
              flush=True)
node.create_subscription(PoseArray, '/leader/survivor/map_positions',
                         mapped, qos_profile_sensor_data)
node.create_subscription(MarkerArray, '/leader/survivor/map_markers',
                         markers, 10)
end = time.monotonic() + 30
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
PY
```

두 사람 구간은 `MAP ... candidates 2`, `MARKERS ADD 4`가 반복되고,
한 사람 퇴장 후에는 `MAP ... candidates 1`, `MARKERS ADD 2 DELETE IDs
[2, 3]`가 나타나야 한다. 3개 이상 검출됐다가 1개로 줄면 추가 IDs도
DELETE된다. ROS 로그와 RViz 화면을 같이 확인해야 하며, 메시지 수만으로
글자 가독성을 PASS 처리하지 않는다.

## 전체 architecture 및 실행 순서

```text
D435 single RealSense owner
 ├─ infra1/infra2 → Isaac ROS Visual SLAM ──────┐
 ├─ color/depth → nvblox 3D mesh/ESDF ───────────┤→ RViz (Fixed Frame: map)
 └─ color/depth → detector → camera_positions     │
                              │                    │
                   exact image stamp TF2           │
                              ↓                    │
                         map_positions             │
                              ↓                    │
                     map visualizer → MarkerArray ─┘
STM32 wheel odometry + IMU → dual EKF → map→odom→base_link TF
```

실제 실행은 각 명령을 **별도 터미널**에서 수행한다. mapping 실행기가
방향키 teleop과 단일 D435 owner를 포함하므로 다시 실행하거나 별도 teleop을
중복 실행하지 않는다. 실행 전 다른 매핑·RealSense 인스턴스가 없는지
`ros2 node list`와 `docker ps`로 확인한다.

```bash
# Terminal 1: mapping + RViz + manual arrow-key teleop
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh

# Terminal 2: RGB/depth preprocessing
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py

# Terminal 3: detector (dedicated GPU container)
cd ~/damgc_robot
./scripts/run_survivor_detector.sh

# Terminal 4: exact-timestamp camera → map
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_transform.launch.py

# Terminal 5: map PoseArray → MarkerArray
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_survivor survivor_map_visualizer.launch.py
```

정지·이동·FOV·다중 사람 검증에는 다음 명령이 빠른 확인용이다. `timeout`
종료 코드 124는 이탈 시 map/marker 무출력 확인에서는 기대되는 결과일 수
있지만, 평상시 스트림 검증에서는 원인 조사가 필요하다.

```bash
source /opt/ros/humble/setup.bash
timeout 10 ros2 topic echo --once /leader/survivor/camera_positions
timeout 10 ros2 topic echo --once /leader/survivor/map_positions
timeout 10 ros2 topic echo --once /leader/survivor/map_markers
ros2 topic info -v /leader/survivor/map_markers
timeout 10 ros2 topic hz /visual_slam/tracking/odometry
timeout 10 ros2 topic hz /leader/odometry/local
timeout 10 ros2 topic hz /leader/odometry/global
timeout 6 ros2 run tf2_ros tf2_echo map base_link
```

`map_positions`와 marker의 **동일 stamp**를 짝지어 좌표를 비교해야 한다.
서로 다른 시각의 `ros2 topic echo --once` 두 출력은 사람이 움직이는
동안 직접 수치 비교하면 안 된다. Phase D의 A/B/사람 이동 수치는 실제
동시 rclpy 구독에서 stamp를 key로 사용해 얻었다. RViz에서는
`Survivors` MarkerArray display와 `NvbloxMesh`를 함께 켜고 Fixed Frame이
`map`인지 확인한다. `static_esdf_pointcloud`는 현재 3D ESDF mode의
필수 성공 조건이 아니다.

## 문제, 제한 및 다음 단계

- Mapping 세션을 중간에 재시작하면 VSLAM의 map 좌표 기준이 바뀔 수
  있다. 같은 A/B 비교는 반드시 한 세션 안에서 새 A를 측정해 수행한다.
- A→B 고정 사람의 map XY에는 약 0.135 m의 잔차가 있었다. Visualizer는
  source pose를 수정하지 않으므로 이 오차를 보정하지 않는다.
- 유효 detector/map 출력이 간헐적일 수 있어 2.0초 finite lifetime은
  지속 검출 중에도 잠깐 사라지는 현상을 만들 수 있다. 입력이 끊긴 뒤
  stale marker가 영구 잔류하는 것보다 보수적인 선택이며, 추후 측정으로
  lifetime을 재조정할 수 있다.
- Candidate 번호는 프레임 내 배열 인덱스다. 2→1 전환 또는 후보 위치
  교차 시 같은 번호가 다른 사람을 가리킬 수 있다. 이것은 현재 Stage의
  정상적인 제한이다. Persistent ID, spatial association, 중복 제거,
  averaging, survivor registry는 구현하지 않았다.
- 텍스트는 실제 map XYZ를 소수 둘째 자리로 반올림해 표시한다. 후보가
  가까우면 text가 겹칠 수 있으며, 이번 Stage에는 label collision
  avoidance를 넣지 않는다.
- 다음 Stage에서만 persistent survivor identity/association, 중복 제거,
  안정 위치 추정 및 registry를 설계한다. 현재 `personN` 또는
  `Survivor candidate N`을 영구 ID로 저장하면 안 된다.

## 안전한 rollback

현재 실행 중 문제를 만나면 visualizer launch 터미널만 `Ctrl+C`로
종료하고 RViz의 `Survivors` display를 끄면 Stage 4의
`/leader/survivor/map_positions` 경로는 그대로 사용할 수 있다. 이는
RealSense/VSLAM/EKF/nvblox/transform 구성에 손대지 않는다. 작업 파일을
되돌릴 필요가 생기면 먼저 `git status --short`와 `git diff`로 대상과
사용자 변경사항을 확인하고 변경 파일을 별도 백업한 후 이 Stage의 변경만
선택적으로 되돌린다. 이 문서의 검증 과정에서는 `git reset --hard`,
`git restore .`, `git checkout .` 또는 자동 commit/push를 실행하지 않았다.

## Stage 5 최종 판정 (운영자 수동 검증 대기)

| 조건 | 현재 증거와 판정 |
| --- | --- |
| Visualizer, MarkerArray, map frame/stamp/text | PASS: Phase B 테스트 및 Phase C live pair |
| 한 사람 + nvblox 동시 RViz 표시 | PASS: Phase C 화면 및 mesh/ESDF |
| 로봇 A→B, camera 변화, marker=map | PASS: Phase D 동일 세션 11/34 triplet; map 수평 잔차 0.135 m 제한 |
| 사람 이동, robot 정지, marker 이동 | PASS: Phase D 112 triplet, map/marker 동일 이동 |
| 실제 FOV 이탈 및 stale 제거 | PASS: camera empty, map/marker 무출력, 5.033초 무갱신 RViz 화면에서 사라짐 |
| 재진입 후 복구 | PASS: camera/map/marker 갱신 62/56/60건 |
| 2인 동시 출력 | PASS: 실물 배치 후 60초간 stamp-matched map/marker 306쌍, 2명당 ADD 4개 |
| 2인 RViz text 가독성 | **MANUAL FOLLOW-UP:** 복수 text 동시 표시를 봤으나 현재 시점에서 일부 겹침; 운영자가 시점/간격을 조절해 최종 판정 |
| 2→1 old marker 정리 | PASS for observed DELETE IDs 2/3; 한 사람만 물리적으로 퇴장시키는 통제 실험은 운영자 요청으로 수동 후속 검증 |
| VSLAM / EKF / TF | PASS for short live sample: odometry 흐름, `vo_state:1`, TF 조회; EKF update-rate 경고는 아래 제한 참고 |
| nvblox mesh / ESDF / RViz | PASS: mesh stream, service success, 3D 화면 |
| Survivor detector / map transform | PASS: 실제 camera/map positions 및 stamp-matched pairs |
| D435 single owner | PASS: `/leader/camera` 한 노드; 중복 매핑 실행 없음 |
| 자동 테스트 | PASS: 53/53 |
| 최종 문서와 README | 검증 문서 완료; README의 VERIFIED 표기는 최종 PASS 뒤 업데이트 |

**Stage 5 전체: 아직 최종 PASS 아님.** 운영자 요청에 따라 추가적인
실물 다중 사람 조작은 멈췄다. UI text 가독성 및 통제된 한 사람 퇴장
실험을 수동 후속 검증으로 남긴다. 이 결과가 확인되기 전에는 README에
`RViz visualization VERIFIED`를 기록하지 않는다.
