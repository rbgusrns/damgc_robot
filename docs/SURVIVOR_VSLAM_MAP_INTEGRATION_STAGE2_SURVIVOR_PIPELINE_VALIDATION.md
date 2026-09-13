# Survivor–VSLAM 통합 Stage 2: Survivor camera preprocessing 검증

## 1. Stage 2 범위와 현재 판정

Stage 1에서 실행 중인 single shared D435를 재실행하지 않고, 기존 RGB와
CameraInfo로 Survivor Detector 입력인 rectified RGB를 만든다. Phase A는 정적 구현
PASS다. Phase B는 2026-09-13 실제 하드웨어에서 두 번 검증했고, 기존 3D ESDF
서비스를 합격 기준으로 유지한다는 사용자 결정에 따라 **기능 PASS, 처리율
관찰사항 있음**으로 판정한다. 기존 `esdf_mode: 3d`에서는 이 버전의 nvblox가
`static_esdf_pointcloud`를 발행하지 않지만, 3D ESDF 서비스는 실제 값을 반환했다.
Survivor Detector와 실제 사람 검출은 Phase C에서 실행했으며, 정지 상태
통합은 확인했다. 이동 및 구간별 수동 검증은 후속으로 보류한다.

구현 범위는 다음으로 제한한다.

```text
/leader/camera/color/image_raw
/leader/camera/color/camera_info
        │
        ├─ CameraInfo QoS bridge
        │       └─ /leader/camera/color/camera_info_transient
        │
        └─ image_proc/rectify_node
                └─ /leader/camera/color/image_rect
```

## 2. 시작 상태 및 Stage 1 인계

| 항목 | 실제 값 |
| --- | --- |
| repository | `/home/maze/damgc_robot` |
| branch | `main` |
| 시작 commit | `7da1410d21ba33b0870a608c136864410b8579a1` |
| 시작 working tree | clean |
| Phase A 시작 시각 | `2026-09-13T17:54:08+09:00` |
| Stage 1 상태 | single D435, sync, aligned depth 및 기존 VSLAM 회귀 검증 완료 |
| Stage 1 실행기 | `scripts/run_vslam_mapping.sh` |

Stage 1의 RealSense 실행부는 현재 다음 shared-camera 설정을 유지한다.

```text
enable_color:=true
enable_depth:=true
enable_infra:=true
enable_infra1:=true
enable_infra2:=true
enable_sync:=true
align_depth.enable:=true
```

이번 Phase A에서는 `scripts/run_vslam_mapping.sh`를 수정하지 않는다.

## 3. 기존 camera_apriltag.launch.py 구조

기존 launch에는 다음 구성요소가 모두 포함되어 있다.

1. `realsense2_camera/launch/rs_launch.py`
2. `robot_state_publisher`
3. `rescue_robot_apriltag/camera_info_qos_bridge.py`
4. `image_proc/rectify_node`
5. `apriltag_ros/apriltag_node`
6. 조건부 `apriltag_approach_node`

Survivor preprocessing에 재사용한 부분은 CameraInfo bridge와 rectify remapping뿐이다.

전체 `camera_apriltag.launch.py`를 재사용하지 않는 이유는 다음과 같다.

- 이미 VSLAM 실행기에서 RealSense driver가 하나 실행 중이다.
- RealSense launch를 다시 include하면 `/dev/video*` 충돌 또는 duplicate publisher가 발생할 수 있다.
- `robot_state_publisher`를 중복 실행하면 camera TF ownership이 깨질 수 있다.
- AprilTag와 approach node는 Survivor preprocessing 범위가 아니다.
- 기존 VSLAM, EKF, nvblox 구조에 불필요한 node를 추가하지 않아야 한다.

## 4. 신규 구현 파일

신규 파일:

```text
src/leader/rescue_robot_bringup/launch/survivor_camera_processing.launch.py
```

역할:

- 기존 `/leader/camera/color/camera_info`를 bridge에 입력한다.
- bridge는 `/leader/camera/color/camera_info_transient`를 발행한다.
- `image_proc/rectify_node`는 기존 `/leader/camera/color/image_raw`와 bridge된 CameraInfo를 사용한다.
- `/leader/camera/color/image_rect`만 발행한다.

포함하지 않은 것:

- `realsense2_camera`
- RealSense launch include
- `robot_state_publisher`
- cuVSLAM
- dual EKF
- nvblox
- AprilTag
- approach
- person detector

## 5. Build/install 확인 계획

`rescue_robot_bringup/CMakeLists.txt`가 이미 `launch` 디렉터리 전체를
`share/rescue_robot_bringup`에 설치하므로 별도 install 설정 변경은 필요하지 않다.

실행 명령:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_bringup
source install/local_setup.bash
```

실제 실행 결과: `rescue_robot_bringup` build 성공 (`Finished <<< rescue_robot_bringup`).
기존 `CMakeLists.txt`의 `install(DIRECTORY launch ...)`로 신규 launch가 설치되었고,
`ros2 launch ... --show-args`에서 launch discovery가 성공했다. 설치 경로는 다음
symlink로 확인했다.

```text
/home/maze/damgc_robot/install/rescue_robot_bringup/share/rescue_robot_bringup/launch/
survivor_camera_processing.launch.py
```

정적 및 discovery 확인:

```bash
python3 -m py_compile \
  src/leader/rescue_robot_bringup/launch/survivor_camera_processing.launch.py

ros2 launch rescue_robot_bringup \
  survivor_camera_processing.launch.py --show-args

find install/rescue_robot_bringup/share/rescue_robot_bringup/launch \
  -name 'survivor_camera_processing.launch.py' -print
```

Phase A 정적 구현 결과: PASS. `python3 -m py_compile`와 `git diff --check`도
성공했다. Phase B와 C에서 VSLAM, RealSense, preprocessing launch, Survivor
Detector를 실제 runtime으로 검증했다.

## 6. Runtime 실행 명령과 검증 절차

Phase A에서는 runtime PASS를 선언하지 않았다. Phase B에서는 아래 순서로
실행했다. Phase C 명령은 Phase B가 PASS로 재검증된 뒤에만 사용한다.

### Phase B: VSLAM + preprocessing

Terminal 1:

```bash
cd ~/damgc_robot
./scripts/run_vslam_mapping.sh
```

Terminal 2:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py
```

확인 명령:

```bash
ros2 node list
ros2 topic info -v /leader/camera/color/image_raw
ros2 topic info -v /leader/camera/color/image_rect
ros2 topic info -v /leader/camera/color/camera_info
ros2 topic info -v /leader/camera/color/camera_info_transient
ros2 topic info -v /leader/camera/aligned_depth_to_color/image_raw

timeout 8 ros2 topic hz /leader/camera/infra1/image_rect_raw
timeout 8 ros2 topic hz /leader/camera/infra2/image_rect_raw
timeout 8 ros2 topic hz /leader/camera/color/image_raw
timeout 8 ros2 topic hz /leader/camera/color/image_rect
timeout 8 ros2 topic hz /leader/camera/aligned_depth_to_color/image_raw
timeout 8 ros2 topic hz /visual_slam/tracking/odometry
timeout 8 ros2 topic hz /leader/odometry/local
timeout 8 ros2 topic hz /leader/odometry/global

timeout 8 ros2 run tf2_ros tf2_echo map base_link
```

Phase B에서는 Survivor Detector를 실행하지 않는다.

### Phase C: Survivor Detector 동시 실행

Phase B가 정상일 때만 다음 명령을 실행한다.

```bash
cd ~/damgc_robot
./scripts/run_survivor_detector.sh
```

이 script는 RealSense를 실행하지 않고 Survivor detector container만 실행해야 한다.
사람 검출 후 다음을 확인한다.

```bash
ros2 topic info -v /leader/survivor/debug_image
ros2 topic info -v /leader/survivor/camera_positions
timeout 15 ros2 topic hz /leader/survivor/debug_image
timeout 15 ros2 topic hz /leader/survivor/camera_positions
timeout 10 ros2 topic echo /leader/survivor/camera_positions
```

`camera_positions`에서 `header.frame_id`, `header.stamp`, `poses` 개수 및 각
pose의 `x/y/z`를 기록한다. map 변환은 이 Stage의 범위가 아니다.

## 7. Phase A 정적 판정 기준

현재 구현이 만족해야 할 기준:

- 신규 launch가 CameraInfo bridge와 `image_proc/rectify_node`만 실행한다.
- 신규 launch에 RealSense, robot_state_publisher, VSLAM, EKF, nvblox, AprilTag, detector가 없다.
- 기존 VSLAM 실행 파일과 보호 대상 파일에 diff가 없다.
- 기존 `camera_apriltag.launch.py`의 bridge/remapping 계약을 유지한다.
- build system을 불필요하게 변경하지 않는다.
- 신규 launch가 install overlay에서 discover 가능하다.

Stage 2 최종 판정은 Phase C 정지 상태 결과와 후속 수동 검증 보류 여부를 함께
기록한다.

## 8. Phase B 실제 실행과 단일 카메라 구조

실행 ID: `vslam_mapping_20260913_175618`. 사용한 명령은 위의 Terminal 1과
Terminal 2에 적은 그대로다. `VSLAM_ONLY=1` 또는 nvblox 제외 모드를 사용하지
않았다. 로봇은 전체 514.6초 기록 중 정지 상태였으며 Codex는 모터 명령을 보내지
않았다. Survivor Detector도 실행하지 않았다. rosbag은 Ctrl-C로 정상 마감됐고
`data/vslam_mapping_20260913_175618/analysis.md`, `analysis.json`, `metadata.yaml`
이 생성됐다. 종료 후 VSLAM/RealSense/preprocessing/container 프로세스는 남지 않았다.
Phase B 시작 시 branch는 `main`, HEAD는 Stage 1 commit `7da1410d21ba33b0870a608c136864410b8579a1`이었다.
working tree에는 Phase A의 신규 launch와 이 문서 두 파일만 untracked로 있었고
기존 tracked 파일 변경은 없었다.

실제 node 목록의 주요 구성은 다음과 같았다.

```text
/leader/camera                         # RealSense 1개
/leader/stm32_bridge
/visual_slam_node
/leader/ekf_localization_node
/leader/ekf_globalization_node
/robot_state_publisher_vslam
/nvblox_node
/rviz
/survivor_camera_info_qos_bridge      # ON에서만
/survivor_color_rectify               # ON에서만
```

`ros2 node info /nvblox_node`, `ros2 topic info -v`에서 확인한 ownership:

| Topic | Publisher | Subscriber |
| --- | --- | --- |
| `/leader/camera/infra1/image_rect_raw` | `/leader/camera` | cuVSLAM |
| `/leader/camera/infra2/image_rect_raw` | `/leader/camera` | cuVSLAM |
| `/leader/camera/color/image_raw` | `/leader/camera` 1개 | `/nvblox_node`, ON에서는 `/survivor_color_rectify` 추가 |
| `/leader/camera/depth/image_rect_raw` | `/leader/camera` 1개 | `/nvblox_node` |
| `/leader/camera/color/camera_info` | `/leader/camera` 1개 | `/nvblox_node`, ON에서는 bridge 추가 |
| `/leader/camera/color/camera_info_transient` | ON에서 bridge 1개 | ON에서 rectify node |
| `/leader/camera/color/image_rect` | ON에서 rectify node 1개 | 이번 Phase에서 detector 미실행 |
| `/leader/camera/aligned_depth_to_color/image_raw` | `/leader/camera` 1개 | 이번 Phase에서 detector 미실행 |

중복 RealSense node, camera topic의 중복 publisher, node/service 이름 충돌은
확인되지 않았다. 새 preprocessing launch에는 camera driver가 없다.

## 9. Survivor 입력과 image_proc 결과

OFF/ON 모두 raw RGB와 aligned depth의 실제 message를 받았다. aligned depth는
`16UC1`, `camera_color_optical_frame`; raw RGB header 역시
`camera_color_optical_frame`이었다. ON에서 CameraInfo는 원본과 bridge output
모두 실제 메시지가 발행됐고 약 30 Hz였다. `ros2 node info
/survivor_color_rectify`에서 입력 subscriber가 실제로
`/leader/camera/color/image_raw`와
`/leader/camera/color/camera_info_transient`에 연결된 것을 확인했다.

ON에서 `/leader/camera/color/image_rect`의 실제 메시지는 `640×480 rgb8`,
`frame_id=camera_color_optical_frame`였다. 단기 `ros2 topic hz`의 마지막 평균은
약 **5.4 Hz**였다. 영상 한 프레임을 `/tmp/damgc_phaseb_rect_20260913.png`로
캡처해 직접 확인했다. 방 내부 가구와 바닥이 선명하게 보였고 검은 화면이나
눈에 띄는 심한 왜곡은 없었다. 픽셀 평균/표준편차는 각각 118.9/47.5,
범위는 3–255였다. 이 스크린샷은 임시 검증 파일이며 repository에 포함하지
않았다. `image_proc`의 경고에는 CameraInfo 대비 RGB 이미지 수가 적은
10초 구간이 여러 번 기록됐다. 따라서 rectified RGB의 처리율은 후속 반복 시험이
필요하다.

## 10. VSLAM·EKF·TF 전후 비교

rosbag의 message 수는 read-only SQLite 조회로 preprocessing 시작
`2026-09-13 17:59:35.645 KST`와 종료 `18:04:27.833 KST`를 경계로 나눴다.
rate는 각 구간의 `(샘플 수−1)/(마지막 수신 시각−첫 수신 시각)`이다.
카메라 image는 이 metrics bag에 포함되지 않으므로 image rate는 별도의 짧은
live probe 결과다.

| 항목 | OFF 전 | ON | OFF 후 | 해석 |
| --- | ---: | ---: | ---: | --- |
| VSLAM tracking odometry, bag | 1,807 / **10.22 Hz** | 2,276 / **7.79 Hz** | 448 / **10.72 Hz** | ON에서 약 24% 낮음; OFF 후 회복 |
| VSLAM status | 1,807 Success | 2,276 Success | 448 Success | 전체 Failed 0, Unknown 0 |
| local EKF, bag | 29.91 Hz | 29.80 Hz | 29.93 Hz | 유지 |
| global EKF, bag | 29.82 Hz | 29.56 Hz | 29.81 Hz | 유지 |
| wheel odom, bag | 44.44 Hz | 42.32 Hz | 46.09 Hz | 계속 publish |
| STM32 IMU, bag | 99.22 Hz | 96.50 Hz | 101.62 Hz | 계속 publish |
| infra1, 짧은 live probe | 18.8 Hz | 20.7 Hz | 미측정 | 큰 하락 확인 안 됨 |
| infra2, 독립 재측정 | 23.5 Hz | **21.8 Hz** | **22.5 Hz** | 동시 다중 CLI probe 때의 15.2 Hz는 비교값에서 제외 |
| VSLAM, 독립 재측정 | 초기 7.8 Hz | 7.3 Hz | 11.0 Hz | 짧은 window 편차 큼; bag 비교 우선 |
| `map → base_link` | tf2_echo 수신 | tf2_echo 수신 | 미측정 | ON 중 chain 유지 |

`/visual_slam/status`의 `vo_state=1`(Success)을 ON 중 live echo로도 확인했다.
전체 bag은 Success 4,531 / Failed 0 / Unknown 0이다. 정지 상태였으므로
wheel/local/global/VSLAM의 기록된 path/net displacement는 모두 0 m였다.
VSLAM frame delta `>22 ms` 경고는 기존 실행에서도 있었으며 ON 구간만의
신규 오류로 분류하지 않았다.

preprocessing ON과 VSLAM throughput 저하는 시간상 연관되지만 단독 원인으로
확정할 수 없다. 실행 중 여러 CLI probe와 화면 캡처가 있었고 OFF 전후 window도
길이가 다르다. tegrastats에서 CPU는 OFF에도 대체로 79–100%, ON에도
82–100%였으므로 CPU contention 가능성을 우선 조사한다. VSLAM/EKF parameter는
변경하지 않았다.

## 11. nvblox 전후 비교와 3D ESDF 출력 조건

실제 launch의 입력은 기존 `/leader/camera/depth/image_rect_raw`,
`/leader/camera/depth/camera_info`, `/leader/camera/color/image_raw`,
`/leader/camera/color/camera_info`다. `/nvblox_node`는 OFF와 ON 모두
실행됐고 이 네 topic에 subscriber를 유지했다. 카메라 publisher도 계속
하나였다. nvblox 로그에는 TF lookup 반복 오류나 새 depth 처리 오류가
없었다. 단, startup에서 weighting/workspace bounds 문자열 관련
기본값 경고와 GXF scheduler 경고가 있었다.

| 항목 | Preprocessing OFF | Preprocessing ON | 판정 |
| --- | --- | --- | --- |
| `/nvblox_node` alive | 예 | 예 | 유지 |
| color/depth input | 기존 topic 구독 및 camera publish 확인 | 같은 구독 및 publish 유지 | 연결 유지; 소비 rate 직접 계측 안 함 |
| `/nvblox_node/mesh` | 실제 publish 약 **3.5 Hz** | 실제 publish 약 **6.5 Hz**; 한 메시지에 220 blocks, 11,858 vertices, frame `odom` | 유지 |
| `/nvblox_node/static_esdf_pointcloud` | publisher 1, RViz subscriber 1; 8초 probe에서 메시지 없음 | 같은 graph지만 20초 probe와 one-shot echo에서 메시지 없음 | **현재 3D 모드에서 발행되지 않는 2D slice** |
| `/nvblox_node/pessimistic_static_esdf_pointcloud` | 미측정 | 8초 probe에서 메시지 없음 | 추가 진단 |
| `/nvblox_node/static_map_slice` | 8초 one-shot에서 메시지 없음 | 컨테이너에서 12초 rate probe 중 메시지 없음 | 추가 진단 |
| `/nvblox_node/esdf_slice_bounds` | 미측정 | Marker 약 3.8 Hz | bounds 표시이며 ESDF grid 데이터 증거는 아님 |
| `/nvblox_node/get_esdf_and_gradient` | 미측정 | 3D grid 요청 성공, 35,301 values | 사용자 승인 3D ESDF 검증 기준 충족 |
| RViz 3D mesh | 화면에 부분 map/mesh 보임 | 화면에 부분 map/mesh 보임, mesh topic 지속 갱신 | mesh 표시 유지 |
| RViz ESDF PointCloud2 | display와 subscription 존재 | display와 subscription 존재, 실제 cloud 수신 미확인 | **정상 표시로 판정 불가** |
| TF/depth warning | 반복 TF lookup 오류 없음 | 반복 TF lookup 오류 또는 새 depth 오류 없음 | 새 오류 확인 안 됨 |

RViz 설정의 `NvbloxMesh`와 PointCloud2 display는 모두 enabled였다.
`gnome-screenshot`으로 OFF/ON 화면을 각각 `/tmp/damgc_phaseb_off_20260913.png`,
`/tmp/damgc_phaseb_on_20260913.png`에 임시 저장해 보았다. 로봇을 움직이지
않았으므로 RViz의 공간적 map 확장은 확인하지 않았다. mesh는 실제 message와
non-empty blocks/vertices로 갱신을 확인했지만 ESDF pointcloud는 topic의
존재와 publisher/subscriber만으로 정상이라고 판단할 수 없다. 이 증상은
preprocessing을 켜기 전부터 있었다. nvblox 설정은 변경하지 않았다.

### 3D ESDF와 pointcloud 미출력 원인

현재 launch의 `esdf_mode`는 `3d`이며 runtime `ros2 param get /nvblox_node
esdf_mode`도 `3d`였다. 현재 설치 소스
`/home/maze/isaac_ros_ws/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp`
의 `NvbloxNode::processEsdf()`는 ESDF 자체는 `multi_mapper_->updateEsdf()`로
갱신하지만 `static_esdf_pointcloud`, `static_map_slice`, occupancy grid를
발행하는 `sliceAndPublishEsdf()`는 `EsdfMode::k2D` 조건 안에서만 호출한다.
따라서 3D 모드에서 해당 topic의 publisher가 graph에 있어도 message가 없는
것은 이 코드의 예상 동작이다. 이동만 해도 이 topic이 생성될 것이라고
가정해서는 안 된다.

현재 3D ESDF의 내부 데이터는 nvblox의 기존
`/nvblox_node/get_esdf_and_gradient` 서비스에 `update_esdf=false`,
`visualize_esdf=false`, `use_aabb=true`, `frame_id=odom`으로 요청해 읽기
전용으로 확인했다. AABB 최소점 `(-1,-1,-0.3) m`, 크기 `(2,2,1) m`의 응답은
`success=true`, voxel size `0.05 m`, 41×41×21 = **35,301 values**였다.
값 범위는 `-1000`(unknown sentinel 포함)부터 `2.0`이었다. 이는 3D ESDF
조회 경로가 살아 있다는 증거지만, RViz pointcloud가 발행된다는 뜻은 아니다.

## 12. 시스템 부하와 종료 기록

8초 tegrastats probe 결과를 대략적으로 기록한다. 여러 ROS CLI 검사와 함께
진행한 관찰이므로 통제된 benchmark 수치는 아니다.

| 항목 | OFF 관찰 범위 | ON 관찰 범위 |
| --- | --- | --- |
| RAM | 약 4.32–4.40 / 7.61 GB | 약 4.50–4.59 / 7.61 GB |
| CPU 6 cores | 대체로 79–100% | 대체로 82–100% |
| GR3D GPU | 약 19–89% | 약 8–72% |
| 최고 표시 온도 `tj` | 약 56°C | 약 59°C |

preprocessing 종료 시 기존 `camera_info_qos_bridge.py`가 SIGINT cleanup 중
`KeyboardInterrupt` traceback을 남겼고 rectify node는 정상 종료됐다. 실행 중
bridge의 CameraInfo 발행과 rectified RGB 생성에는 영향이 확인되지 않았다.
VSLAM 스크립트는 rosbag을 정상 마감하고 전체 프로세스를 종료했다. 보호 대상
VSLAM/RealSense/nvblox/detector 파일 및 설정에는 코드 변경이 없다.

### VSLAM rate 통제 재시험

첫 실행은 많은 ROS CLI probe가 ON 구간에 몰려 있었다. 이 교란을 줄이기 위해
`vslam_mapping_20260913_180952`를 기존 통합 명령으로 한 번 더 실행했다.
동일한 정지 상태에서 약 1분씩 OFF→ON→OFF를 유지하고 각 측정 구간에는
별도 `ros2 topic hz`를 실행하지 않았다. 이 실행의 bag duration은 253.3초이며
`data/vslam_mapping_20260913_180952/analysis.md`와 `analysis.json`이
정상 생성됐다. ON 시작은 bridge 로그 시각 18:11:24.453 KST, ON 종료는
rectify 종료 로그 시각 18:12:40.350 KST로 정의하고 전환 전후 5초를 제외했다.

| 구간 | VSLAM odometry | VSLAM status | local EKF | global EKF | wheel | IMU |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| OFF 전 55초 | 206 / **3.78 Hz** | 205 / 3.76 Hz | 28.90 Hz | 26.16 Hz | 36.91 Hz | 88.55 Hz |
| ON 약 66초 | 229 / **3.49 Hz** | 228 / 3.46 Hz | 28.75 Hz | 25.36 Hz | 36.15 Hz | 86.91 Hz |
| OFF 후 55초 | 200 / **3.65 Hz** | 193 / 3.51 Hz | 28.80 Hz | 26.03 Hz | 39.42 Hz | 91.00 Hz |

두 번째 bag 전체 status는 Success **866**, Failed 0, Unknown 0이다.
ON 차이는 OFF 전 대비 약 8%, OFF 후 대비 약 4%로 첫 실행의 24% 차이를
재현하지 않았다. 반면 OFF부터 VSLAM 3–4 Hz, wheel 37–39 Hz, IMU
87–91 Hz, global EKF 25–26 Hz로 전체 처리율이 첫 실행보다 낮았다.
시험 후 `ps`에서는 GNOME Shell 약 38% CPU와 GNOME Remote Desktop 약
22% CPU가 상주했다. 이 백그라운드 부하가 유일한 원인이라고 단정하지는
않는다. 현재 증거로는 preprocessing만의 확정적 VSLAM regression을
주장할 수 없으며, 동시에 기존 Stage 1과 동일한 처리율을 입증하지도 못한다.

## 13. Phase B 판정과 다음 조치

2026-09-13 18:18 KST 사용자는 **기존 3D 모드와 ESDF 서비스 검증 유지**를
선택했다. 따라서 2D slice 전용 `static_esdf_pointcloud`의 미출력을
3D 모드 실패로 판정하지 않는다. 기존 nvblox 설정과 3D ESDF 의미는 유지한다.

**Phase B: PASS (처리율 관찰사항 포함).** Shared camera ownership과
raw/rectified RGB, CameraInfo, aligned depth, infra1/infra2, VSLAM tracking,
local/global EKF, `map → base_link`, nvblox mesh 및 3D ESDF service가
실제로 동작했다. `/nvblox_node`의 기존 color/depth 구독과 camera source도
유지됐다. RViz에는 3D mesh가 보였고 mesh message가 갱신됐다.
3D 모드에서는 RViz의 2D ESDF PointCloud2 display가 실제 데이터를 받지
않는 것이 현재 코드의 예상 동작이며, 3D ESDF 검증에는 service 응답을 사용했다.

첫 실행의 VSLAM rate ON 저하(10.22→7.79 Hz)는 probe를 최소화한 두 번째
실행에서 같은 크기로 재현되지 않았다(3.78→3.49→3.65 Hz). 두 실행 모두
tracking status는 전 구간 100% Success였다. 다만 두 번째 실행의 전체
처리율은 Stage 1보다 낮았으므로 `성능 완전 동일`이나 `절대 rate 정상`이라고
주장하지 않는다. 이는 goal 3에서 detector를 추가할 때 계속 관찰할 항목이다.

**goal 3 진행 가능:** Phase B의 기능 통합 gate는 통과했다. 다음 단계에서도
동일한 단일 D435 구조를 유지하고, detector 실행 전/후 VSLAM rate·status와
nvblox mesh 및 3D ESDF service를 다시 비교한다. 실제 로봇 이동은 사용자만
수행하며 이번 Phase B에서는 수행하지 않았다.

## 14. Phase C 실제 실행

Phase C 실행 ID는 `vslam_mapping_20260913_182114`이다. 기존
`./scripts/run_vslam_mapping.sh`를 먼저 실행한 뒤, 별도 터미널에서 다음
preprocessing과 detector만 추가했다.

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch rescue_robot_bringup survivor_camera_processing.launch.py
```

```bash
./scripts/run_survivor_detector.sh
```

`scripts/run_survivor_detector.sh`는 `docker run`으로
`person_detector.launch.py`만 실행하며 RealSense launch/include가 없다. Phase C
실행 중 host의 RealSense는 `/leader/camera` 하나(PID 기준 launch와 node 한 쌍)였고,
preprocessing은 bridge와 `survivor_color_rectify`만, detector container는
`/leader/person_detector`만 추가했다. 두 번째 D435, 두 번째
`realsense2_camera_node`, duplicate camera publisher는 확인되지 않았다.

검출기 startup 설정은 다음 입력/output으로 확인했다.

```text
input RGB       /leader/camera/color/image_rect
input depth     /leader/camera/aligned_depth_to_color/image_raw
input info      /leader/camera/color/camera_info
debug output    /leader/survivor/debug_image
pose output     /leader/survivor/camera_positions
device          CUDA device 0
```

## 15. Phase C topic ownership 및 정지 상태 결과

detector ON 상태에서 확인한 publisher/subscriber 핵심 결과는 다음과 같다.

| Topic | Publisher | Subscriber | 결과 |
| --- | --- | --- | --- |
| `/leader/camera/infra1/image_rect_raw` | `/leader/camera` 1개 | cuVSLAM | single owner |
| `/leader/camera/infra2/image_rect_raw` | `/leader/camera` 1개 | cuVSLAM | single owner |
| `/leader/camera/color/image_raw` | `/leader/camera` 1개 | nvblox, rectify | 정상 |
| `/leader/camera/color/image_rect` | `survivor_color_rectify` 1개 | `/leader/person_detector` | 정상 |
| `/leader/camera/aligned_depth_to_color/image_raw` | `/leader/camera` 1개 | `/leader/person_detector` | 정상 |
| `/leader/survivor/debug_image` | `/leader/person_detector` 1개 | probe | 정상, best-effort |
| `/leader/survivor/camera_positions` | `/leader/person_detector` 1개 | probe | 정상 |

15초 단일 rclpy probe에서 측정한 detector ON rate는 다음과 같다. 짧은 live
probe이며 시스템 부하와 DDS QoS에 영향을 받을 수 있으므로 절대 성능 기준이
아니다.

| Topic | 측정 rate |
| --- | ---: |
| infra1 | 13.67 Hz |
| infra2 | 14.20 Hz |
| VSLAM tracking odometry | 4.13 Hz |
| local EKF | 25.93 Hz |
| global EKF | 29.40 Hz |
| camera_positions | 6.33 Hz |
| debug_image | 1.87 Hz |

ON 중 `/visual_slam/status`는 `vo_state=1`로 유지되었고, `map → base_link`
TF lookup도 성공했다. detector ON 상태의 TF 예시 translation은
`(2.768, -0.186, 0.000) m`였으며, 이는 사람이 화면에 들어오도록 로봇을
들어 재배치한 뒤의 상태다. 이 재배치 구간은 주행 odometry 회귀 증거로
사용하지 않는다.

## 16. 실제 person detection 및 camera XYZ

사람이 화면에 들어온 뒤 `/leader/survivor/camera_positions`는 빈 배열이
아닌 1 pose를 반복 발행했다. `ros2 topic echo --once` 예시는 다음과 같다.

```text
header.frame_id: camera_color_optical_frame
poses: 1
position: x=-0.6752, y=-0.2343, z=1.5170
```

다른 안정 구간에서는 다음과 같은 1 pose가 반복되었다.

```text
frame_id: camera_color_optical_frame
position: x=0.8029, y=0.1276, z=2.8300
orientation: (0, 0, 0, 1)
```

`/tmp/phasec_debug.png`를 best-effort QoS로 캡처해 확인했으며, 640×480
debug image에 사람 bounding box와 `XYZ (0.78, 0.13, 3.02) m` overlay가
실제로 표시됐다. detector는 CUDA device 0을 사용했다.

10초 XYZ probe에서는 69개의 non-empty pose를 수신했고 모두 pose 1개였다.
그 구간 통계는 `x=0.7853–0.8177 m`, `y=0.1146–0.1261 m`,
`z=2.8660–3.0470 m`이었다. 사람의 가까이/멀리/좌우 이동을 분리한 추가
검증은 사용자의 요청으로 이번 실행에서 종료했으며 후속 수동 검증으로
이관한다.

| 사람 위치 구간 | pose 수 | mean x | mean y | mean z | 비고 |
| --- | ---: | ---: | ---: | ---: | --- |
| 기준/가까이/멀리/좌우 | 미수행 | — | — | — | 후속 수동 검증으로 이관 |

## 17. camera_positions timestamp contract

rectified RGB와 `camera_positions`를 동시에 구독한 12초 probe에서 non-empty
pose 67개를 확인했다. 첫 pose를 제외한 매칭 pose는 nearest rectified RGB
timestamp delta가 `0.0 ns`로 출력되었다. 첫 pose도 직전 이미지와 약 `-100 ms`
였고, 이후에는 정확히 일치했다. 따라서 현재 detector의
`camera_positions.header`는 원본 rectified image header를 보존하는 것으로
확인했다.

대표 timestamp:

```text
camera_positions stamp: 1789291596.726391315
frame_id: camera_color_optical_frame
matching image_rect stamp delta: 0.0 ns
```

## 18. Detector ON 회귀 및 nvblox 재확인

Detector 실행 전 OFF 상태에서 camera, VSLAM, EKF, TF, nvblox graph를 확인한
뒤 detector를 켰다. detector ON 후에도 RealSense driver 수는 1개였고,
infra publisher 중복은 없었다. `person_detector`가 추가한 subscription은
rectified RGB와 aligned depth뿐이며 VSLAM IR input remap은 변경하지 않았다.

| 항목 | Detector OFF 기준 | Detector ON | 판정 |
| --- | --- | --- | --- |
| infra1 | 약 23.7 Hz live probe | 13.67 Hz / 205 samples | 추가 부하 관찰; duplicate 없음 |
| infra2 | 약 23.7 Hz live probe | 14.20 Hz / 213 samples | 추가 부하 관찰; duplicate 없음 |
| VSLAM odometry | status `vo_state=1` | 4.13 Hz / 62 samples, status `vo_state=1` | tracking 유지 |
| local EKF | publish | 25.93 Hz / 389 samples | 유지 |
| global EKF | publish | 29.40 Hz / 441 samples | 유지 |
| `map → base_link` | lookup 성공 | lookup 성공 | 유지 |
| nvblox mesh | non-empty, 약 3.5–6.5 Hz | node/subscriber graph 유지 | 유지 |
| 3D ESDF service | 35,301 values | 설정 변경 없이 기존 service 유지 | 유지 |

OFF/ON rate는 서로 다른 짧은 측정 window이고 Jetson background load의 영향을
받으므로 동일-rate benchmark로 해석하지 않는다. Detector ON 뒤 VSLAM status의
실패/unknown 반복은 발견되지 않았다. nvblox 설정, EKF parameter, VSLAM
parameter는 변경하지 않았다. detector 로그에는 RGB/aligned-depth timestamp
차이가 `0.133–0.200 s`라는 warning이 반복되어 이슈로 기록한다. 그럼에도
camera_positions와 debug image는 실제로 생성되었다.

## 19. Phase C 이동 검증

로봇을 사람이 직접 들어 화면에 맞게 재배치한 것은 person detection 준비를
위한 것이며, VSLAM 주행 검증 구간에서는 제외했다. 별도의 바닥 주행 검증은
다음 조건으로 수행한다.

```text
사람은 가능한 고정된 위치에 둔다.
로봇은 안전한 공간에서 사용자가 아주 천천히 짧은 거리만 이동한다.
Codex는 motor command를 보내지 않는다.
```

이번 Phase C에서는 person XYZ 구간 검증을 종료하면서 별도의 바닥 주행을
추가로 수행하지 않았다. 로봇을 들어서 카메라 화면을 재배치한 구간은
주행 검증으로 사용할 수 없으므로 이동 결과도 미수행으로 기록한다.

| 항목 | 결과 |
| --- | --- |
| VSLAM tracking 유지 | 이번 Phase C 주행 구간 미수행 |
| camera_positions 지속 출력 | 정지 상태에서 6.33 Hz, non-empty pose 확인 |
| aligned depth | detector ON 입력 및 warning 동반 수신 확인 |
| local/global EKF | detector ON에서 각각 25.93/29.40 Hz 확인 |
| map → base_link | detector ON lookup 성공; 주행 검증은 미수행 |

## 20. Stage 2 현재 판정 및 다음 단계

현재까지 Phase A, Phase B, Phase C 정지 상태의 핵심 통합은 PASS다.
single RealSense ownership, raw/rectified RGB, CameraInfo, aligned depth,
person detection, debug image, non-empty camera XYZ, frame_id 및 timestamp
contract, VSLAM status, EKF, TF가 실제 Jetson runtime에서 확인됐다.

다만 다음 항목은 이번 실행에서 완료하지 않았고 후속 수동 검증으로 남긴다.

1. 사람의 기준/가까이/멀리/좌우 구간별 XYZ 평균 기록
2. 사용자가 수행하는 안전한 짧은 바닥 이동 중 VSLAM/EKF/TF와
   `camera_positions` 지속성 확인

따라서 본 문서의 최종 판정은 **Stage 2 기능 통합 PASS, 이동/구간별 수동
검증 보류**로 기록한다. 이는 camera XYZ 출력 자체의 실패가 아니라 사용자가
요청한 수동 검증 종료에 따른 범위 기록이다.

이번 Stage에서 구현하지 않은 항목은 그대로 유지한다.

- camera XYZ → map XYZ
- `/leader/survivor/map_positions`
- `survivor_map_transform_node.py`
- RViz Survivor Marker
- survivor ID tracking, deduplication, position storage

다음 Stage는 `map → odom → base_link → camera_link →
camera_color_optical_frame` TF chain을 집중 검증한 뒤, message header timestamp
기반 `survivor_map_transform_node.py`를 구현하는 것이다.
