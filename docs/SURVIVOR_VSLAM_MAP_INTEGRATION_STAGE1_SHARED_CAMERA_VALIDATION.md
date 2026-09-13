# Survivor–VSLAM map 통합 Stage 1: shared-camera validation

## 범위와 판정

- 목적: 현재 Leader VSLAM을 수정하지 않은 상태에서 D435, STM32, dual EKF, TF의 실제 동작을 기록하고 이후 shared-camera 변경의 비교 기준으로 사용한다.
- 통합 단계: Phase A baseline 검증 후 Phase B shared-camera 최소 변경까지 수행했다. survivor detector 연결과 camera XYZ의 map 변환은 수행하지 않았다.
- 측정 시작: 2026-09-13 16:57:42 KST. 정지 상태 실행은 약 217초간 기록했다.
- **Phase A 판정: 기능 데이터 경로 PASS, 엄격한 전체 baseline FAIL/보류.** 실제 이동 중 모든 필수 입력·출력과 tracking은 유지됐지만, 정지 이후 `map → odom`에 약 4.0 cm 단일 step이 관찰되어 TF 연속성의 정상 여부를 확정할 수 없다. 자세한 근거는 아래 이동 검증 절에 있다.
- Phase B에서 수정한 코드는 `scripts/run_vslam_mapping.sh` 하나이며, Phase C 실제 회귀 검증까지 완료했다.

## Git 및 하드웨어 기준점

| 항목 | 시작 시 실제 결과 |
| --- | --- |
| branch | `main` (`main...origin/main`) |
| short commit | `de62113 docs(survivor): mark Stage 3 hardware verification complete` |
| full commit | `de621133b64fbad4ae94ec911cd8e9d3a4b1d3b1` |
| working tree | clean 아님. 기존 untracked `frames_2026-09-12_22.42.53.gv`, `frames_2026-09-12_22.42.53.pdf`가 있으며 보존함 |
| D435 | `lsusb`: `8086:0b07 Intel Corp. RealSense D435`; 드라이버 로그: USB 3.2, RealSense ROS 4.58.3 / librealsense 2.58.3 |
| STM32 연결 | `/dev/i2c-7` 존재; bridge 로그: STM32 I²C `0x42` open 성공. 연결된 보드의 모델/펌웨어 버전은 이번 실행에서 직접 확인하지 않음 |
| Jetson/컨테이너 | `/proc/device-tree/model`: `NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super`; `aarch64`, 호스트 ROS 2 Humble, `damgc-vslam-mapping:humble` 이미지의 `isaac_ros_dev-aarch64-container` 실행 확인 |

시작 직전 명령은 `date -Is`, `git status --short --branch`, `git branch --show-current`, `git log -1 --oneline`, `git rev-parse HEAD`, `lsusb`, `ls -l /dev/i2c-7 /dev/video0`, `docker ps`, `ros2 node list`, `ros2 topic list`였다. 시작 전 ROS node는 없고 topic은 `/parameter_events`, `/rosout`뿐이었다.

## 현재 코드와 실행 구조

현재 checkout에서 `scripts/run_vslam_mapping.sh`, `visual_slam_realsense.launch.py`, `visual_slam_nvblox_realsense.launch.py`, `dual_ekf.yaml`, `camera_apriltag.launch.py`, `localization.launch.py`, `nvblox_realsense.launch.py`, `vslam_covariance_adapter.py`를 확인했다. 현재 실행 명령은 다음과 같다.

```bash
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh
```

실행기는 호스트에서 `stm32_bridge`를 `namespace:=leader`, I²C `/dev/i2c-7`/`0x42`로 시작하고, 같은 호스트에서 **한 번** `realsense2_camera rs_launch.py`를 시작한다. Phase A baseline 카메라 인자는 `camera_namespace:=leader`, `camera_name:=camera`, color/depth/infra1/infra2 활성, D435 IMU 비활성, `publish_tf:=true`, `tf_publish_rate:=30.0`였고 `enable_sync`와 `align_depth.enable`은 `False`였다. Phase B는 여기에 `enable_sync:=true`와 `align_depth.enable:=true`만 추가한다. 카메라 해상도/FPS는 별도 지정하지 않는다.

컨테이너의 `visual_slam_nvblox_realsense.launch.py`가 `localization.launch.py`, `visual_slam_realsense.launch.py`, `nvblox_realsense.launch.py`를 포함한다. `robot_state_publisher_vslam`은 VSLAM launch 내부에서 URDF를 읽어 실행된다. `camera_apriltag.launch.py`는 별도 카메라 launch를 포함하지만 이번 VSLAM 실행에는 포함되지 않았고 실제 node 목록에도 없었다.

```text
호스트 D435 /leader/camera
  ├─ infra1 image_rect_raw + camera_info ┐
  └─ infra2 image_rect_raw + camera_info ┴─> Isaac ROS cuVSLAM (컨테이너)
      ├─ /visual_slam/tracking/odometry (odom, base_link)
      └─ /visual_slam/vis/slam_odometry (map, base_link)
           └─ covariance adapter → /visual_slam/slam_odometry_with_covariance
호스트 STM32 I²C → /leader/odom/raw + /leader/imu/data_raw
  └─ local EKF → /leader/odometry/local
local EKF velocity + VSLAM covariance adapter → global EKF → /leader/odometry/global
D435 color/image_raw + depth/image_rect_raw → 기존 nvblox
```

TF 소유권은 코드와 런타임 segment echo가 일치한다. local EKF가 `odom → base_link`, global EKF가 `map → odom`을 발행한다. VSLAM의 두 TF publish 옵션은 `False`다. `robot_state_publisher_vslam`의 URDF가 `base_link → camera_link`를, RealSense driver가 `camera_link → camera_color_optical_frame` 등 센서 frame을 발행한다. `base_link → camera_link` URDF 원점은 `(0.042, 0.010, 0.130) m`이다.

nvblox 기존 입력은 `/leader/camera/depth/image_rect_raw`, `/leader/camera/depth/camera_info`, `/leader/camera/color/image_raw`, `/leader/camera/color/camera_info`다.

## 정지 상태 런타임 검증

실행 기록: [`log/vslam_mapping_20260913_165742`](../log/vslam_mapping_20260913_165742/), [`data/vslam_mapping_20260913_165742/analysis.md`](../data/vslam_mapping_20260913_165742/analysis.md). rosbag은 Ctrl-C로 정상 마감됐고 `metadata.yaml` 및 `analysis.json`이 생성됐다. 이 실행에서 Codex는 방향키, teleop 주행 명령 또는 모터 구동 명령을 보내지 않았다.

검증에 사용한 명령은 다음과 같다. 각 `timeout`의 exit 124는 관찰 시간이 끝난 결과이며, 아래에는 그 전에 실제 출력된 값만 기록한다.

```bash
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list
ros2 node info /leader/camera
ros2 topic info -v /leader/camera/infra1/image_rect_raw
ros2 topic info -v /leader/camera/infra2/image_rect_raw
ros2 topic info -v /leader/camera/color/image_raw
ros2 topic info -v /leader/camera/depth/image_rect_raw
ros2 param get /leader/camera enable_sync
ros2 param get /leader/camera align_depth.enable
timeout 8 ros2 topic hz /leader/camera/infra1/image_rect_raw
timeout 8 ros2 topic hz /leader/camera/infra2/image_rect_raw
timeout 8 ros2 topic hz /leader/camera/color/image_raw
timeout 8 ros2 topic hz /leader/camera/depth/image_rect_raw
timeout 8 ros2 topic hz /visual_slam/tracking/odometry
timeout 6 ros2 topic hz /leader/odom/raw
timeout 6 ros2 topic hz /leader/imu/data_raw
timeout 6 ros2 topic hz /leader/odometry/local
timeout 6 ros2 topic hz /leader/odometry/global
timeout 8 ros2 topic echo /visual_slam/tracking/odometry --once
timeout 8 ros2 topic echo /visual_slam/vis/slam_odometry --once
timeout 8 ros2 topic echo /leader/odom/raw --once
timeout 8 ros2 topic echo /leader/imu/data_raw --once
timeout 8 ros2 topic echo /leader/odometry/local --once
timeout 8 ros2 topic echo /leader/odometry/global --once
timeout 8 ros2 run tf2_ros tf2_echo map base_link
timeout 8 ros2 run tf2_ros tf2_echo map camera_color_optical_frame
timeout 5 ros2 run tf2_ros tf2_echo map odom
timeout 5 ros2 run tf2_ros tf2_echo odom base_link
timeout 5 ros2 run tf2_ros tf2_echo base_link camera_link
timeout 5 ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

호스트 ROS에는 `isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus`가 없어 호스트 `ros2 topic echo /visual_slam/status --once`는 메시지 타입 오류였다. 컨테이너에서는 아래 명령이 성공했고 `vo_state: 1`을 받았다. 설치된 메시지 정의에서 `1 = Success`, `2 = Failed`, `0 = Unknown`임을 확인했다.

```bash
docker exec isaac_ros_dev-aarch64-container bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  ros2 interface show isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus
  timeout 8 ros2 topic echo /visual_slam/status --once
'
```

| 항목 | 실제 관찰 결과 |
| --- | --- |
| infra1 / infra2 | 둘 다 존재하고 실제 메시지 발행. 단독 8초 측정 마지막 평균 각각 **24.1 / 20.5 Hz**. 장치 open profile은 각각 `848×480 Y8 @30 FPS` |
| color / raw depth | `/leader/camera/color/image_raw`, `/leader/camera/depth/image_rect_raw` 존재, 각각 publisher 1개. 동시 probe 중 마지막 평균 **19.1 / 17.8 Hz**이므로 단독 rate와 직접 비교하지 않음. open profile은 color `640×480 RGB8 @30`, depth `848×480 Z16 @30` |
| aligned depth | `/leader/camera/aligned_depth_to_color/image_raw` topic 없음. 현재 align 설정이 `False`이므로 Phase A의 실패 조건이 아님 |
| VSLAM | `/visual_slam/tracking/odometry`, `/visual_slam/status`, `/visual_slam/vis/slam_odometry` 존재. tracking odometry 단독 8초 측정 마지막 평균 **13.0 Hz**, 실제 메시지의 `frame_id=odom`, `child_frame_id=base_link` |
| STM32 wheel | `/leader/odom/raw` 실제 메시지(`odom`, `base_link`) 수신. 단독 6초 측정 마지막 평균 **47.1 Hz** |
| STM32 IMU | `/leader/imu/data_raw` 실제 메시지(`imu_link`) 수신. 단독 6초 측정 마지막 평균 **86.1 Hz** |
| local / global EKF | 양쪽 실제 메시지 지속 발행. 각각 약 **30.0 / 30.0 Hz**. local `odom/base_link`, global `map/base_link`. 관찰 샘플은 유한 값·거의 0 위치였고 명백한 폭주 없음 |
| `map → base_link` | transform 수신 성공, 정지 중 `(x,y)≈(0,0)`, yaw 약 0°. 시작 직후 TF cache 준비 전의 `Invalid frame ID` 출력 후 정상 수신 |
| `map → camera_color_optical_frame` | transform 수신 성공, 정지 중 translation 약 `(0.042,0.025,0.130) m`. 두 EKF/URDF/RealSense 구간이 연결됨 |
| TF segment | `map → odom`, `odom → base_link`, `base_link → camera_link`, `camera_link → camera_color_optical_frame` 각각 `tf2_echo`에서 수신 확인 |
| RealSense owner | node 목록에 `/leader/camera` **1개**. infra1, infra2, color, depth 대표 topic은 각각 publisher count **1**, publisher node `/leader/camera` |

실행 중 node 목록에는 `/leader/camera`, `/leader/stm32_bridge`, `/leader/ekf_localization_node`, `/leader/ekf_globalization_node`, `/leader/vslam_covariance_adapter`, `/visual_slam_node`, `/robot_state_publisher_vslam`, `/nvblox_node`, `/rviz`, `/rosbag2_recorder`, `/arrow_key_teleop`와 TF listener 보조 node가 있었다. AprilTag 또는 survivor detector node는 없었다.

종료 후 rosbag 분석은 `/visual_slam/tracking/odometry` **2,418 samples / 11.2 Hz**, status **Success 2,418 / Failed 0 / Unknown 0**, local EKF **6,468 / 29.8 Hz**, global EKF **6,416 / 29.7 Hz**, wheel **9,661 / 44.7 Hz**, IMU **99.4 Hz**를 보고했다. 각 경로의 정지 상태 path/net displacement는 `0.000 m`였다. 짧은 `ros2 topic hz` 측정과 217초 전체 bag rate가 다르므로 둘 다 기록한다.

### Warning/error 기준점

- VSLAM 로그에서 `Delta between current and previous frame ... above threshold [22 ms]` 경고 **2,633건**. 30 FPS 정상 간격도 약 33.3 ms이므로 이 threshold와 실제 입력 간격 차이는 이미 baseline에 존재한다. 최대 관찰 경고는 200 ms 이상도 있었다. 원인이나 주행 중 영향은 이번 정지 검사만으로 단정할 수 없다.
- STM32 bridge 로그에서 `STM32 RX sequence gap` 경고 **851건**. wheel/IMU는 계속 publish됐지만 누락 경고를 정상이라고 간주하지 않는다.
- RealSense 로그: 기본 설정 파일 없음으로 defaults 로드, `rgb_camera.power_line_frequency=3`가 허용 범위 `[0,2]`를 벗어나 설정 실패 경고. `Publishing dynamic camera transforms at 30 Hz` 경고도 있었고 camera node는 계속 동작했다.
- host에서 status echo 시 메시지 타입 미설치 오류가 있었고 컨테이너에서 성공했다. `timeout` 종료 시 일부 CLI shutdown 경고가 출력됐으며 이는 관찰 명령 종료에서 나온 것이다.

## 사용자 주행을 통한 이동 검증

정지 상태 217초 bag에서는 wheel/local/global/VSLAM 경로가 모두 0 m였다. Codex가 연 통합 실행기의 teleop은 Codex의 PTY에 연결되어 사용자 방향키를 받을 수 없어 정상 종료했다. 이후 사용자가 자기 터미널에서 **동일한 기존 명령** `./scripts/run_vslam_mapping.sh`를 2026-09-13 17:03:06 KST에 재실행하고 짧게 전진한 다음 Space 및 Ctrl-C로 종료했다. Codex는 모터 명령을 보내지 않았다. 두 번째 실행의 로그는 [`log/vslam_mapping_20260913_170306`](../log/vslam_mapping_20260913_170306/), 정상 마감된 rosbag과 분석은 [`data/vslam_mapping_20260913_170306/analysis.md`](../data/vslam_mapping_20260913_170306/analysis.md), [`analysis.json`](../data/vslam_mapping_20260913_170306/analysis.json)에 있다. 종료 후 해당 ROS/카메라/컨테이너 프로세스는 남지 않았다.

검증 중 다음 live 명령도 실행했다. `map → base_link`는 이동 후 약 `(0.159, 0.004, 0.000) m`, yaw 약 `-0.9°`로 갱신됐고 VSLAM status echo는 `vo_state: 1`이었다. wheel, VSLAM, local/global EKF의 실제 메시지 위치도 모두 0에서 변했다.

```bash
ros2 topic echo /leader/odom/raw --once
ros2 topic echo /visual_slam/tracking/odometry --once
ros2 topic echo /leader/odometry/local --once
ros2 topic echo /leader/odometry/global --once
timeout 5 ros2 run tf2_ros tf2_echo map base_link
docker exec isaac_ros_dev-aarch64-container bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  timeout 8 ros2 topic echo /visual_slam/status --once
'
```

두 번째 bag의 기간은 130.2초다. `/leader/cmd_vel`에는 `0.08 m/s` 이하 선속도 명령 61개가 17:03:37.330–17:03:48.733에 기록됐고 각속도 명령은 0이었다. 이 수치는 명령이며 실제 물리 거리의 독립 계측값은 없다.

| 측정 | 샘플 / 평균 rate | 기록된 path | 순이동 | 판정 |
| --- | ---: | ---: | ---: | --- |
| wheel `/leader/odom/raw` | 5,935 / 45.9 Hz | 0.247 m | 0.240 m | 이동 확인 |
| local EKF | 3,889 / 29.9 Hz | 0.252 m | 0.240 m | 갱신 확인, `odom/base_link` |
| VSLAM tracking odometry | 1,451 / 11.3 Hz | 0.238 m | 0.104 m | 갱신 확인, `odom/base_link` |
| VSLAM map pose | 1,451 / 11.3 Hz | 0.298 m | 0.161 m | 갱신 확인, `map/base_link` |
| global EKF | 3,882 / 29.8 Hz | 0.493 m | 0.161 m | 갱신 확인, `map/base_link` |

VSLAM status는 **Success 1,451 / Failed 0 / Unknown 0**, tracking success 100%였다. IMU는 13,148 samples / 평균 101.6 Hz였고 시작·종료 5초 gyro Z 평균은 각각 약 `-0.00010`, `0.00001 rad/s`다. 종료 전 5초 position range는 wheel 0, local 약 `4 nm`, global 약 `0.35 mm`, VSLAM tracking 약 `0.32 mm`였다. 검사한 odometry 및 `map → odom`/`odom → base_link` TF 샘플에는 NaN/Inf가 없었다. RealSense driver는 두 번째 실행에서도 host process와 ROS node `/leader/camera` 하나였다.

### TF 연속성 검토와 미해결 문제

정상 마감된 rosbag을 `rosbag2_py.SequentialReader`와 `rclpy.serialization.deserialize_message`로 **읽기 전용** 조회해 odometry 및 `/tf`의 인접 샘플 사이 3D translation 거리와 yaw 차이를 계산했다. 다른 엔지니어는 같은 bag에서 아래 핵심 절차로 재계산할 수 있다.

```python
# source /opt/ros/humble/setup.bash 후 Python 3에서 실행
import math
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri="data/vslam_mapping_20260913_170306", storage_id="sqlite3"
    ),
    rosbag2_py.ConverterOptions("", ""),
)
previous = None
while reader.has_next():
    topic, raw, timestamp_ns = reader.read_next()
    if topic != "/tf":
        continue
    message = deserialize_message(raw, TFMessage)
    for transform in message.transforms:
        if (transform.header.frame_id, transform.child_frame_id) != ("map", "odom"):
            continue
        xyz = transform.transform.translation
        current = (timestamp_ns / 1e9, xyz.x, xyz.y, xyz.z)
        if previous is not None:
            step_m = math.dist(current[1:], previous[1:])
            if step_m >= 0.04:
                print(current[0], step_m, current[0] - previous[0])
        previous = current
```

| 인접 샘플 최대 translation step | 실제 값 | 간격 |
| --- | ---: | ---: |
| wheel odometry | 0.0023 m | 0.017 s |
| local EKF / `odom → base_link` | 0.0044 m | 0.035 s |
| VSLAM tracking odometry | 0.0277 m | 0.219 s |
| VSLAM map pose | **0.0601 m** | 0.096 s |
| global EKF | **0.0401 m** | 0.032 s |
| `map → odom` | **0.0402 m** | 0.032 s |

최대 map 보정은 **2026-09-13 17:04:52.49 KST**, 마지막 활성 주행 명령 약 64초 뒤에 나타났다. 그 시점 전후 ±1초의 bag 샘플 79개에서 wheel `x=0.240391 m`, `y=-0.000331 m`는 일정했고 `/leader/cmd_vel` 샘플 40개도 모두 0이었다. 같은 구간 VSLAM tracking odometry `x` 범위는 약 **0.00048 m**, VSLAM map pose와 global EKF `x` 범위는 각각 약 **0.058 m**였다. VSLAM 로그에는 직전 약 **434 ms** frame 간격 경고가 있으나 이것이 보정의 원인인지 확인되지 않았다. tracking failure, NaN, TF 단절은 확인되지 않았으며 map pose 보정과 `map → odom`/global EKF 변화가 동시 발생했다. 정상 loop closure인지, 지연된 map 보정인지, 결함인지 현재 로그만으로 분류할 수 없다.

두 번째 실행 로그에도 frame 간격 경고 **1,521건**, STM32 RX sequence gap 경고 **349건**이 있다. wheel과 VSLAM의 누적 path 차이는 0.009 m지만 순이동 차이는 **0.137 m**이며, map 보정이 들어간 경로를 단순 누적 거리만으로 정확하다고 평가할 수 없다. 줄자 등 외부 ground truth가 없어 실제 위치 정확도 판정은 하지 않는다.

## Phase B 정적 변경 기록

### 시작 상태

Phase A의 기능 데이터 경로는 PASS였고 실제 이동 시 VSLAM status는 1,451건 모두 Success였다. 다만 정지 후 map pose 보정과 `map → odom` step이 발견되어 전체 baseline은 FAIL로 남아 있다. 이 문제를 해결하거나 설정으로 숨기지 않고, Phase B는 요청된 shared-camera 인자 추가만 수행했다. Phase C runtime 회귀 검증은 아래와 같이 완료했다.

### 실제 변경

변경 파일은 [`scripts/run_vslam_mapping.sh`](../scripts/run_vslam_mapping.sh) 하나다.

```diff
     enable_infra:=true enable_infra1:=true enable_infra2:=true \\
+    enable_sync:=true align_depth.enable:=true \\
     enable_gyro:=false enable_accel:=false \\
```

- `enable_sync:=true`: RealSense의 color/depth 및 관련 stream 동기화 기능을 활성화해 향후 RGB와 aligned depth를 같은 시간 기준으로 사용할 수 있게 한다.
- `align_depth.enable:=true`: depth를 color 좌표계로 정렬하고 `/leader/camera/aligned_depth_to_color/image_raw` 생성을 요청한다.
- 기존 `infra1`, `infra2`, color, depth 인자와 VSLAM IR remapping은 그대로 유지한다. VSLAM 입력 경로의 호환성을 보존하기 위해서다.
- nvblox remapping은 그대로 유지한다. 기존 depth/color 입력 계약을 보존하기 위해서다.
- `person_detector_node.py`는 수정하지 않는다. Stage 1은 detector 연결과 map 변환을 포함하지 않는다.
- `camera_apriltag.launch.py`는 수정하지 않는다. 별도 실행 구조와 단일 owner 원칙을 유지한다.

설치된 ROS 2 Humble의 `ros2 launch realsense2_camera rs_launch.py --show-args`에서 두 인자의 정확한 이름과 기본값(`False`)을 재확인했다. RealSense는 계속 host의 단일 `/leader/camera` node만 실행하며, 두 번째 D435 launch나 추가 camera node를 만들지 않았다. 해상도/FPS, calibration, TF ownership, VSLAM/EKF/nvblox 설정은 변경하지 않았다.

### 영향 및 rollback

변경은 RealSense filter/synchronization 동작을 추가하므로 USB bandwidth, 입력 timestamp와 stream rate, VSLAM frame jitter에 영향을 줄 가능성이 있다. Phase C에서 infra rate, VSLAM rate/status, EKF, TF, aligned depth, single-owner와 duplicate publisher를 baseline과 비교했다.

rollback은 이 변경 한 줄을 제거하는 방식으로 수행한다. 전체 파일 복원이나 destructive Git 명령은 사용하지 않는다.

```bash
git diff -- scripts/run_vslam_mapping.sh
# 검토 후 추가된 enable_sync/align_depth 줄만 제거
git diff --check
```

예상 추가 topic:

```text
/leader/camera/aligned_depth_to_color/image_raw
```

## Phase A 최종 판정과 다음 단계

**기능 동작 PASS:** D435의 IR/RGB/depth와 단일 RealSense owner, STM32 wheel/IMU, VSLAM odometry/status, dual EKF, `map → base_link → camera_color_optical_frame`가 실행됐고 짧은 이동에서도 tracking 성공이 유지됐다.

**전체 baseline FAIL (정상 기준점 미확정):** 정지 상태에서 4.0 cm `map → odom` TF step과 6.0 cm VSLAM map pose 보정이 실제로 기록됐다. 이것을 무시한 채 “비정상 TF jump 없음” 또는 “정량적 VSLAM 정확도 정상”으로 기록할 수 없다. Phase B의 두 인자 변경은 이 문제를 해결하지 않으며, Phase C에서 변경 후 재검증한다.

**Phase B 정적 변경 PASS:** 설치된 launch API와 실제 diff를 확인했고, `scripts/run_vslam_mapping.sh`에 RealSense 인자 한 줄만 추가했다. Phase C에서 실제 aligned depth 메시지와 기존 VSLAM 파이프라인의 하드웨어 회귀 여부도 검증했다.

## Phase C 실제 회귀 검증

### 실행과 관찰 범위

2026-09-13 17:20:59 KST에 사용자가 사용자 터미널에서 기존 실행 명령을 **한 번** 실행했다. 실행기는 host에서 단일 RealSense driver를 시작하고 container에서 기존 VSLAM/EKF/nvblox를 시작했다. 로봇은 정지 구간 후 기존 시험과 같은 저속 직진으로 약 0.19 m 이동하고 Space로 정지한 뒤 Ctrl-C로 종료했다. 최종 bag은 [`data/vslam_mapping_20260913_172059/analysis.md`](../data/vslam_mapping_20260913_172059/analysis.md)와 [`analysis.json`](../data/vslam_mapping_20260913_172059/analysis.json)에 있다.

이번 실행의 RealSense log에는 `Sync Mode: On`, D435 USB 3.2, serial `109622073868`, infra1/infra2 `848×480 Y8 @30 FPS`, depth `848×480 Z16 @30 FPS`, color `640×480 RGB8 @30 FPS`, `RealSense Node Is Up!`가 기록됐다. Phase C의 수정 설정으로 실행한 live topic 관찰(중복 실행 사건이 발생하기 전)에서는 aligned depth의 실제 `sensor_msgs/msg/Image` 메시지를 받았다. 메시지는 `640×480`, `16UC1`, `frame_id=camera_color_optical_frame`이고 단독 rate 마지막 평균은 약 **22.5 Hz**였다. 해당 topic의 publisher count는 **1**, publisher는 `/leader/camera`였다. 이 topic은 metrics bag에 저장하지 않아 clean bag의 message count에는 포함되지 않는다. 따라서 aligned depth의 직접 증거는 live topic 관찰에, clean bag은 단일 실행의 VSLAM/EKF/TF 회귀 증거에 각각 근거한다.

### 수정 후 결과

| 항목 | 수정 후 실제 결과 | 판정 |
| --- | --- | --- |
| infra1 / infra2 | open profile `848×480@30`; 수정 설정 live graph에서 publisher `/leader/camera` 1개. clean bag 실행기 readiness도 infra1과 VSLAM에 도달 | 정상 |
| RGB | `/leader/camera/color/image_raw` 유지, open profile `640×480 RGB8@30`, publisher 1개 | 정상 |
| raw depth | `/leader/camera/depth/image_rect_raw` 유지, open profile `848×480 Z16@30`, publisher 1개 | 정상 |
| aligned depth | 수정 설정 live 관찰에서 실제 메시지 수신, 약 22.5 Hz, `640×480 16UC1`, color optical frame | 정상 |
| VSLAM tracking | clean bag 207 samples / **8.7 Hz**, status **207 success, 0 failed, 0 unknown** | 정상, rate 비교 필요 |
| wheel odom | 1,117 samples / 47.1 Hz, path 0.189 m, 순이동 0.186 m | 정상 |
| IMU | 2,472 samples / 104.2 Hz | 정상 |
| local EKF | 711 samples / 30.0 Hz, `odom/base_link`, 순이동 0.186 m | 정상 |
| global EKF | 710 samples / 29.9 Hz, `map/base_link`, 순이동 0.051 m | 정상 갱신 |
| `map → base_link` | runtime에서 수신, clean bag에 `odom→base_link`와 `map→odom`이 모두 기록 | 정상 |
| `map → camera_color_optical_frame` | runtime에서 수신, camera extrinsic 약 `(0.042,0.025,0.130) m` 유지 | 정상 |
| RealSense owner | clean execution log에 camera launch 1개와 `/leader/camera` 1개. AprilTag/survivor node 없음 | 정상 |

`/visual_slam/tracking/odometry`의 clean bag rate는 8.7 Hz로 Phase A 정지 bag 11.2 Hz보다 낮고, 이전 이동 bag 11.3 Hz보다도 낮다. 그러나 status 성공률은 100%이고 tracking execution mean/max는 `5.01/27.03 ms`, camera profile은 유지됐다. 따라서 hard tracking failure는 없지만, VSLAM rate 저하는 명백한 성능 차이로 기록하며 “성능 완전 동일”이라고 주장하지 않는다. clean bag의 짧은 duration(23.7 s)과 이동 구간 때문에 rate를 최종 정량 결론으로 쓰지 않고, 후속 Stage에서 반복 측정한다.

### 이동 결과와 TF 연속성

clean bag의 `/leader/cmd_vel`에는 최대 `0.08 m/s`, 각속도 0인 활성 명령이 기록됐다. wheel/local EKF는 약 `0.186 m` 이동했고 VSLAM tracking/map pose 및 global EKF도 0에서 각각 약 `0.051 m`로 변했으며 status는 끝까지 Success였다. aligned depth는 Phase C 수정 설정의 live 관찰에서 실제 publish를 확인했고, clean bag에서는 이미지 자체를 저장하지 않았으므로 image message count로 재검증하지 않았다.

읽기 전용 bag 분석 결과 인접 샘플 최대 translation step은 `map→odom 0.01998 m / 0.031 s`, `odom→base_link 0.00424 m / 0.033 s`였고 NaN/Inf는 없었다. Phase A의 `map→odom 0.0402 m` step보다 작지만, VSLAM map pose와 global map 위치가 wheel/local과 완전히 일치하지 않는 기존 현상은 남아 있다. 이 결과는 shared-camera 변경으로 TF chain이 끊어지지 않았음을 보여주지만 map pose 정확도나 loop closure 정상성을 증명하지는 않는다.

## Before / After 비교

| 항목 | Phase A baseline | Phase C shared-camera | 판정 / 비고 |
| --- | ---: | ---: | --- |
| infra1 rate | 24.1 Hz | clean bag readiness; live modified observation 약 21.6 Hz | publish 유지, rate 저하 가능성 기록 |
| infra2 rate | 20.5 Hz | live modified observation 약 21.4 Hz | 유지 |
| RGB raw | 약 19.1 Hz probe | open `640×480@30`; 실제 topic 유지 | 정상 |
| raw depth | 약 17.8 Hz probe | open `848×480@30`; 실제 topic 유지 | 정상 |
| aligned depth | 없음 | 실제 메시지, 약 22.5 Hz, 1 publisher | 추가 기능 정상 |
| VSLAM tracking odometry | 11.2 Hz stationary bag | 8.7 Hz, 207 samples | status 100%; rate 반복 측정 필요 |
| wheel odometry | 44.7 Hz stationary bag | 47.1 Hz, 0.186 m 이동 | 정상 |
| IMU | 99.4 Hz stationary bag | 104.2 Hz | 정상 |
| local EKF | 29.8 Hz | 30.0 Hz | 정상 |
| global EKF | 29.7 Hz | 29.9 Hz | 정상 |
| `map → base_link` | 수신, stationary 0 m | 수신, 이동 후 map pose 갱신 | chain 유지 |
| `map → camera_color_optical_frame` | 수신, extrinsic 확인 | 수신, extrinsic 유지 | chain 유지 |
| RealSense driver | 1 | 1 | duplicate 없음 |

### 중복 실행 사건과 제외 기록

17:13 실행(`171339`) 중 별도 동일 script가 다시 시작되어 `/dev/video0` busy와 재연결 로그가 발생했다. 이 기록은 최종 회귀 수치에서 제외했다. 해당 사건으로부터 “동일 script를 두 번 실행하면 single-owner 조건이 깨지고 camera device conflict가 발생할 수 있다”는 운영 주의를 확인했다. 이후 모든 프로세스와 container를 종료하고, `172059` 단일 실행을 새로 수행했다. `172059`에는 camera device busy 재연결이 없었다.

## Stage 1 최종 판정

**Stage 1: PASS WITH OBSERVATION.** 다음 필수 동작은 실제 하드웨어에서 확인됐다: 단일 D435 owner, infra1/infra2, RGB, raw depth, 실제 aligned depth message, VSLAM odometry/status, STM32 wheel/IMU, local/global EKF, `map→base_link`, camera TF, 짧은 이동 중 VSLAM tracking 유지. 변경 범위도 `scripts/run_vslam_mapping.sh`의 인자와 의도 주석만으로 최소화됐다.

단, VSLAM tracking rate는 baseline보다 낮은 8.7 Hz였고 기존 map pose 보정 현상도 남아 있다. 따라서 이 PASS는 shared-camera 기능과 기존 파이프라인의 hard regression이 없다는 의미이며, VSLAM rate·map pose 정확도가 baseline과 완전히 동일하다는 의미가 아니다. aligned depth와 VSLAM이 동시에 동작한 것은 확인됐고, 후속 반복 시험에서 rate 저하 원인을 계속 관찰한다.

Stage 1 완료 범위는 shared D435 검증까지다. 다음 단계는 이미 실행 중인 D435를 다시 실행하지 않고 기존 RGB와 CameraInfo를 구독하는 `survivor_camera_processing.launch.py` 설계·구현이다. CameraInfo QoS bridge와 필요한 preprocessing만 포함하고, RealSense 재실행과 `robot_state_publisher` 중복 실행은 포함하지 않는다. 이후 단계는 detector/XYZ, timestamp TF2, map transform, marker와 tracking 순서로 진행한다.

## 최종 Architecture

```text
                              Jetson Orin Nano
                                      │
                                  Intel D435
                                      │
                         RealSense Driver (1개)
                                      │
              ┌───────────────────────┼───────────────────────┐
              │                       │                       │
       infra1 / infra2          color + depth          aligned depth to color
              │                       │                       │
              └───────┐       ┌───────┴───────┐       ┌───────┘
                      │       │               │       │
                  cuVSLAM  future Survivor  nvblox  future survivor XYZ input
                      │      detector       │
          VSLAM tracking/   RGB + depth      │
          slam odometry        │             │
                      │        │             │
STM32 wheel ───────────┴──> Local EKF <──── STM32 IMU
                                  │
                         local odometry output
                                  │
                   VSLAM map pose + local velocity
                                  │
                            Global EKF
                                  │
                           map → odom
                                  │
                             base_link
                                  │
                         camera_link / optical frames

Future survivor map path:
camera XYZ
    ↓
TF2 using image timestamp
    ↓
survivor map XYZ
    ↓
RViz Marker
```

## 향후 단계

1. **Stage 2:** 실행 중인 D435를 재실행하지 않는 `survivor_camera_processing.launch.py` 생성. 기존 RGB/CameraInfo 구독, CameraInfo QoS bridge와 필요한 preprocessing만 연결한다.
2. **Stage 3:** 기존 person detector 동시 실행과 camera XYZ 검증.
3. **Stage 4:** image timestamp 기준 `map → camera` TF 검증.
4. **Stage 5:** `survivor_map_transform_node` 구현.
5. **Stage 6:** 고정 생존자와 로봇 이동 상황에서 map XYZ 안정성 검증.
6. **Stage 7:** RViz Marker 표시.
7. **Stage 8:** 중복 제거, tracking, 저장.
