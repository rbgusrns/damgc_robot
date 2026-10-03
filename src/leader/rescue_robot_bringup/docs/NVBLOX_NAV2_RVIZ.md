# nvblox 기반 Nav2 목표 확인

2026-10-03 주행 문제 후 현재 전용 launch는 wheel odometry가 `odom -> base_link`를
발행하고 nvblox가 `odom` 프레임의
`/nvblox_node/static_map_slice`를 발행한다. Nav2 global/local costmap 모두 이 slice를
사용한다. Nav2 위치·속도와 frontier 위치는 `/leader/odom/raw`를 기준으로 한다.
VSLAM은 비교 기록용이고 TF/시각 위치 보정은 발행하지 않는다. VSLAM translation
정지나 wheel/VSLAM 차이만으로 주행을 끊지 않는다. wheel 데이터 자체의 freshness와
유효성은 selector가 확인한다. Nav2 자체는 `map` 프레임이나 AMCL을 요구하지 않는다.
이 전용 wheel 모드에는 `map -> odom`이 없으므로 Survivor의 map 기반 장기 위치
등록은 별도 localization 구성을 사용해야 한다. 아래 과거 검증 기록의 VSLAM TF 구성은
현재 전용 launch의 동작을 설명하지 않는다.

매번 새 mapping 실행을 시작하면 새 STM32 bridge process가 wheel odometry의
`x/y/yaw`를 0으로 초기화하고 첫 encoder sample을 tick 기준점으로 잡는다. 주행 후
mapping stack을 완전히 종료한 다음 로봇을 손으로 시작 위치에 옮기면 다음 실행의
wheel odom은 다시 원점에서 시작한다. stack이 살아 있는 동안 들어 옮겨도 pose reset은
일어나지 않는다.

카메라가 이미 실행 중일 때 컨테이너에서 다음 launch를 사용한다.

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install_docker/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 DAMGC_VSLAM_HEADLESS=1
ros2 pkg prefix rescue_robot_bringup
ros2 launch rescue_robot_bringup nvblox_vslam_realsense.launch.py
```

`ros2 pkg prefix` 결과가 `/workspaces/isaac_ros-dev/install_docker/rescue_robot_bringup`인지
확인한다. 호스트의 `/home/maze/damgc_robot/install`이 나오면 컨테이너 안의 새 터미널에서
위 두 setup 파일을 순서대로 다시 source한다. 새 launch가 `install_docker`에 없다면
컨테이너에서 다음 명령으로 설치한다.

```bash
cd /workspaces/isaac_ros-dev
colcon --log-base log_docker build --packages-select rescue_robot_bringup \
  --symlink-install --build-base build_docker --install-base install_docker
source install_docker/local_setup.bash
```

기존 `nvblox_costmap.launch.py` 또는 다른 VSLAM/nvblox launch와 동시에 실행하지 않는다.
카메라·VSLAM·nvblox가 이미 실행 중이면 `nvblox_nav2.launch.py`만 실행할 수 있다.
특히 `visual_slam_nvblox_realsense.launch.py`는 dual EKF가 TF를 발행하므로
VSLAM의 `publish_odom_to_base_tf` 값이 `false`다. 위 전용 launch에서도
`publish_odom_to_base_tf`와 `publish_map_to_odom_tf`를 모두 `false`로 두며,
`wheel_odometry_tf` 노드가 wheel pose와 timestamp를 보존해 TF를 발행한다.
따라서 이 launch와 dual EKF launch를 동시에 실행하면 TF publisher가 중복된다.

별도 컨테이너 터미널에서 같은 setup 파일을 source한 뒤
`rviz2 -d /workspaces/isaac_ros-dev/rviz/vslam_nvblox.rviz`를 실행한다.
RViz의 Fixed Frame은 `odom`이고
`Nav2 Goal` 도구가 있다. 이 도구는 `/navigate_to_pose` action으로 목표를 보낸다.
계산된 경로는 주황색 `Nav2 Plan` 표시(`/plan`)에서 확인한다.
로봇 가까이, 현재 확인된 자유 공간에 작은 목표를 지정한다. `/nav2/cmd_vel`은
바퀴 브리지의 `/leader/cmd_vel`과 연결되지 않아 이 구성만으로 바퀴는 움직이지 않는다.
`Nav2 Plan`은 메시 위에 그리도록 RViz에서 Z 오프셋 0.2 m로 설정했다.
RViz를 이미 실행 중이었다면 변경된 `vslam_nvblox.rviz` 파일로 다시 실행한다.

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action list -t | grep navigate_to_pose
ros2 topic info /nvblox_node/static_map_slice
ros2 topic info /plan
ros2 topic info /nav2/cmd_vel
```

`Nav2 Goal`로 목표를 지정하기 전에 다른 터미널에서
`ros2 topic echo --once /plan --field header.frame_id`를 실행하면
계획 성공 시 `odom`이 출력된다. 목표는 global costmap의 알려진 자유 공간에
설정해야 하며, 경로가 없으면 planner 로그와 목표 좌표를 확인한다.

활성 상태는 각각 `active [3]`이어야 하고 slice 구독자는 두 costmap이다.
`/nav2/cmd_vel`에는 controller publisher 하나와 subscriber 0개가 있어야 한다.

로컬 costmap은 `/local_costmap/costmap`에서 발행하고, 호환 토픽
`/costmap/costmap`에도 같은 `nav_msgs/msg/OccupancyGrid`를 발행한다.
두 토픽 모두 `header.frame_id`가 `odom`이며, 호환 토픽은 전체 `data` 배열을
2 Hz로 제공한다.
통합 launch를 재시작한 뒤 다음 명령으로 확인한다.

```bash
ros2 topic echo --once /costmap/costmap
ros2 topic hz /costmap/costmap
```

## Survivor와 함께 실행할 때: TF 및 depth 동기화

이 절은 wheel 기반 전용 launch로 변경하기 전의 통합 검증 기록이다.

2026-09-26의 Nav2 통합(`1b201a8`)에서 전용 VSLAM launch의
`publish_map_to_odom_tf`가 `true`에서 `false`로 바뀌었다. 전용 실행 경로에는
global EKF가 없어 `map` 프레임 자체가 사라졌고, Survivor Stage 4의 영상 시각
`camera -> map` 조회가 실패했다. Stage 5 marker와 Stage 6 registry도 따라서
비어 있었다. 전용 VSLAM launch에서 `map -> odom`을 다시 켰으며 소유권은
`map -> odom`: VSLAM, `odom -> base_link`: VSLAM,
`base_link -> camera_link`: robot_state_publisher의 고정 관절,
`camera_link -> optical frame`: RealSense이다. Nav2와 nvblox의 `odom` 설정 및
RViz Fixed Frame은 유지한다. 다른 VSLAM/EKF launch를 동시에 실행하지 않는다.
VSLAM TF는 영상 처리 시점에 간헐적으로 늦게 도착한다. 실측 328개 sample에서
영상 stamp 대비 `odom` TF 지연은 95백분위 133 ms, 99백분위 300 ms,
최대 467 ms였다. Stage 4는 같은 영상 stamp의 TF를 최대 0.6초 기다린다.
lookup timestamp를 0으로 바꾸거나 `odom` 좌표를 `map`으로 간주하지 않는다.

또한 YOLO가 단일 스레드 RGB callback에서 실행되는 동안 aligned depth callback이
지연될 수 있었다. D435를 매핑 스크립트와 같은 인자로 18초 실행해 원본 RGB 478개와
aligned depth 490개를 비교한 결과 최근접 header stamp 차이는 중앙값과 95백분위
모두 0 ms, 최대 33.36 ms, 120 ms 초과 0건이었다. 따라서 이 장비의 해당 실행에서
관찰된 133~167 ms 경고의 원인은 카메라 프로파일보다 detector의 callback/cache
경로다. 실제 로그에는 depth가 RGB보다 133 ms 오래된 경우와 RGB 처리가
589 ms 늦어 depth보다 400 ms 오래된 경우가 모두 있었다. detector의 ROS
callback은 최신 RGB 한 장과 depth cache만 갱신하고, 별도 worker가 최대 150 ms
동안 더 가까운 depth를 기다린 뒤 YOLO를 실행한다. stamp 기준 0.5초와
`sync_queue_size`로 depth cache를 제한하며 기존 `sync_slop_sec=0.12`는 유지한다.
callback에 250 ms 이상 늦게 도착한 RGB는 추론 전에 버려 밀린 영상을 새 depth와
잘못 짝짓지 않는다.
카메라 자동 선택 프로파일
color `640x480x30`, depth/infra `848x480x30`, `enable_sync` 및 depth alignment도
유지한다.

변경 파일은 `launch/nvblox_vslam_realsense.launch.py`, Survivor detector,
Stage 4 launch와 노드의 TF 대기 기본값, 관련 회귀 테스트 및 이 문서와 Survivor
설명 문서다. `run_vslam_mapping.sh`, AprilTag, STM32, Nav2 설정,
Survivor Stage 4의 exact-time 조회 정책은 그대로다.

빌드 및 실행 순서:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rescue_robot_bringup rescue_robot_survivor
source install/local_setup.bash
# detector 코드가 이미지에 포함되므로 해당 이미지는 다시 빌드
./scripts/build_survivor_runtime.sh
```

매핑 컨테이너의 기존 `install_docker`가 저장소 launch를 symlink로 참조하는지
확인한다. 설치본이 오래된 경우 매핑 컨테이너를 띄운 뒤 위 문서 앞부분의
`colcon --build-base build_docker --install-base install_docker` 명령으로
`rescue_robot_bringup`만 다시 빌드하고 매핑 launch를 재시작한다.
Terminal 1에서 `./scripts/run_vslam_mapping.sh`, Terminal 2에서 ROS setup과
`install/local_setup.bash`를 source하고
`ros2 launch rescue_robot_bringup survivor_pipeline.launch.py`, Terminal 3에서
`./scripts/run_survivor_detector.sh`를 실행한다.

Terminal 4 검증:

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/local_setup.bash
ros2 node list | grep -E 'visual_slam|ekf|nvblox|planner|controller|bt_navigator'
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo --once /leader/survivor/camera_positions --field header.frame_id
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
ros2 topic echo --once /leader/survivor/camera_positions
ros2 topic echo --once /leader/survivor/map_positions
ros2 topic echo --once /leader/survivor/tracks --qos-durability transient_local
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic info /nvblox_node/mesh
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_tools view_frames
```

`camera_positions`의 실제 `frame_id`가 위 예시와 다르면 그 값으로 `tf2_echo`를
실행한다. 사람이 보이는 상태에서 camera/map positions에 유효한 XYZ, raw marker,
충분한 관측 뒤 tracks에 ID가 있어야 한다. RViz Nav2 Goal로 경로와 `/plan`의
`odom` frame을 확인한다. `/tf`·`/tf_static`의 publisher와 생성된 TF tree에서
두 dynamic TF의 중복이나 cycle도 확인한다. Stage 4는 정확한 영상 timestamp의
TF만 사용하므로 `latest` 조회로 우회하지 않는다.

관련 자동 회귀 테스트는 `colcon test --packages-select rescue_robot_bringup
rescue_robot_survivor`와 `colcon test-result --verbose`로 실행한다.

2026-10-02 현장 검증: 두 변경 패키지 빌드와 Python syntax 검사가 성공했다.
Survivor 전체 테스트 및 bringup Survivor launch 테스트 125개가 통과했다. 두 패키지의
전체 pytest 결과는 128 passed, 2 failed였으며 두 실패는 이번 변경 전부터
존재한 AprilTag launch 테스트의 이전 그리퍼 닫힘값 `450`과 접근 거리 `0.16`
기대값이다. 현재 main의 실제 기본값은 각각 다른 값이므로 이 작업에서
AprilTag 파일이나 해당 테스트는 변경하지 않았다. 실장비에서는 `map -> odom`,
`odom -> base_link`, `map -> camera_color_optical_frame`이 지속 출력됐고,
`view_frames`에 하나의 `map -> odom -> base_link -> camera_link` 경로가 있었다.
nvblox mesh는 12초에 76건(최대 11,390 vertices)이 발행됐고 Nav2
`ComputePathToPose`는 `odom` 경로를 반환하며 성공했다. 사람 관측 20초에
camera_positions 163건과 map_positions 150건이 non-empty였으며 raw/registry
marker도 발행되고 registry에 Survivor ID가 나타났다. detector의 마지막
5분 로그에는 RGB/depth mismatch 경고가 없었다. 0.6초 TF 대기 설정으로
워밍업 이후 실제 영상 stamp 240회 중 exact-time TF 조회 실패는 0건이었다.
`NavigateToPose` 목표는 수락됐으나 현장 costmap에서 controller가 유효한
trajectory를 찾지 못해 중단됐다. 따라서 주행 완료는 확인되지 않았으며
Nav2 경로 생성과 노드 활성 상태까지만 검증됐다.
로봇을 이동시키며 map 좌표 안정성을 확인하는 시험은 별도로 필요하다.

## 다음 주행 시험: controller 실패 원인 분리

현재 기록에는 실패 요약만 있고 당시 controller 로그 원문과 local costmap 수치가 없어
원인을 특정할 수 없다. `robot_radius`를 줄이거나 DWB 설정을 바꾸기 전에 다음 자료를
같은 실패 시각에 모은다. 실제 모터 시험은 E-stop과 bridge watchdog을 확인하고 낮은
속도의 짧은 목표부터 별도 안전 절차로 수행한다.

### 목표를 보내기 전

아래 항목을 저장해 controller가 실제 로봇 위치와 비용 지도를 어떻게 보고 있는지 확인한다.

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic echo --once /visual_slam/tracking/odometry
ros2 topic echo --once /local_costmap/costmap
ros2 topic echo --once /local_costmap/published_footprint
ros2 topic echo --once /nvblox_node/static_map_slice
ros2 topic info /nav2/cmd_vel
ros2 topic info /leader/cmd_vel
```

RViz에서 로봇 footprint가 local costmap의 자유 셀에 놓이는지, 목표와 계획 경로가
장애물/인플레이션 영역을 통과하지 않는지 캡처한다. `NavigateToPose`를 보낼 때는
controller server 로그 전체와 `/plan`, local costmap, `/nav2/cmd_vel`을 함께 기록한다.
Nav2의 “no valid trajectories” 메시지가 발생한 시각을 표시하고, 그때 costmap이 로봇
주변을 lethal/unknown으로 표시했는지 확인한다.

### 판별 순서

1. `odom` pose와 속도 갱신이 멈추거나 튀는지 확인한다. 문제가 있으면 controller 튜닝보다
   VSLAM/odometry 입력을 먼저 다룬다.
2. footprint가 local costmap에 완전히 들어가 있는지, footprint 주변이 lethal 또는
   unknown인지 확인한다. 그렇다면 nvblox slice의 값/범위, inflation과 footprint 설정을
   검토한다.
3. `/plan`이 있고 로봇에서 첫 경로 점까지 연결되는지 확인한다. 경로가 없으면 planner와
   costmap을 조사하고, 경로는 있으나 controller만 실패하면 DWB 평가 결과를 조사한다.
4. 장애물 없는 바닥에서 로봇 가까이의 짧은 목표로 재현한다. 목표 방향, 회전 필요 여부,
   로봇 전방 축과 `base_link` 축 일치 여부를 기록한다.
5. 원인과 관측 자료가 일치할 때만 footprint/radius 또는 controller 파라미터를 조정하고
   동일 조건에서 다시 비교한다. 통과 뒤에는 저속 구동, 정지, 반복 목표 순으로 확장한다.

CAD 모델 치수는 약 `0.373 × 0.226 m`다. 기존 Nav2 설정은 `robot_radius: 0.3 m`,
`inflation_radius: 0.5 m`였으며 2026-10-03 14시대 주행 때 이 값으로 실행됐다. 이후 source
설정이 `robot_radius: 0.2 m`, `inflation_radius: 0.3 m`로 바뀌었고 15:44 이후 raw/filtered
정지 비교는 이 값으로 실행했다. 이어서 이 문서 마지막의 다각형 footprint로 변경했으며,
그 footprint 설정은 아직 ROS 실행에서 확인하지 않았다.

## 2026-10-03 실물 주행 진단

`./scripts/run_vslam_mapping.sh`로 D435, VSLAM, nvblox, Nav2, Leader selector와 STM32
bridge를 함께 실행했다. RViz는 GPU 자원 사용을 줄이기 위해 시험 중 종료했으며 ROS 노드와
rosbag 기록은 유지했다. VSLAM 위치는 시작 시 `(0, 0)` 부근이었고, 사용자가 앞쪽 약 1 m가
비어 있는 것을 육안으로 확인했다.

Nav2 controller 속도를 시험용으로 `max_vel_x=0.05 m/s`, `max_speed_xy=0.05 m/s`,
`max_vel_theta=0.15 rad/s`로 낮추고 selector를 `NAV2`로 바꿨다. `odom` frame에서 정면
`+x=0.8 m` 목표가 수락됐지만, controller가 약 0.83초 뒤 `No valid trajectories out of 209!`
로그와 함께 abort했다. 사용자가 실제 로봇이 약 8 cm 움직인 것을 확인했다. 시험 종료 후
selector를 `STOP`으로 설정했다. 전체 0.8 m 목표 주행은 실패했으며, 속도 상한은 runtime 설정이라
재실행 때 기본 설정으로 돌아간다.

전체 mapping rosbag 분석은 약 31분 동안 VSLAM tracking success 100%를 기록했다. 다만
`/visual_slam/tracking/odometry` 누적 경로 길이는 `0.711 m`, net displacement는 `0.084 m`였고,
wheel odometry는 각각 `0.034 m`, `0.032 m`였다. VSLAM의 최대 sample gap은 `838 ms`였으며
마지막 정지 5초의 pose 변화는 `0.003 m / 0.07°`였다. tracking status가 Success였다는 사실만으로
저속 경로의 정밀 odometry를 보장하지 않으므로, wheel/VSLAM 불일치와 frame interval 경고도
후속 주행 시험에서 함께 다룬다. 전체 수치는 rosbag의 `analysis.md`에 있다.

### costmap 장애물과 그리퍼의 일치

목표 전후 odometry pose에서 로봇 중심 costmap cell은 cost `99`였다. 같은 costmap에서 가장
가까운 lethal cell은 로봇 기준 `(x=+0.15 m, y=0.00 m)`였다. 현재 Leader URDF의 그리퍼 중앙
연결부는 중심 `x=0.185 m`, 폭 `0.050 m`로 앞면이 `x=0.160 m`에 놓이며, 높이 범위는
`z=0.030–0.090 m`다. 따라서 그리퍼는 costmap lethal cell 위치와 수 cm 이내로 일치한다.
costmap은 `robot_radius=0.30 m`, `inflation_radius=0.50 m`를 사용하므로 이 셀은 로봇 footprint
안쪽에서 중심 주변까지 높은 비용을 만든다. 이 위치 대응과 camera depth 입력 때문에
**그리퍼 자기 점유가 209개 궤적 거부의 유력 원인**이다.

현재 `nvblox_realsense.launch.py`는 D435 depth를 nvblox에 직접 연결하고, launch에는 로봇
형상 기반 depth mask/self-filter가 없다. ESDF height 범위 `0.01–0.30 m`에는 URDF 그리퍼의
`0.03–0.09 m` 높이가 포함된다. nvblox slice 원점은 `x=0`이고 시작 당시 로봇도 `x≈0`이어서
후방 footprint 일부가 slice의 미관측 경계에 놓이는 문제도 함께 있다. 그리퍼 위치 일치는 직접
관측됐지만, 경계/unknown이 기여한 양은 아직 분리되지 않았다.

실행 중 ESDF 최소 높이를 `0.01`에서 `0.10 m`로 바꿔도 local costmap은 변하지 않았다. Static
mapper가 이미 적분한 점유를 유지할 수 있어 이 변경은 그리퍼 가설을 반증하지 않는다. 원래 값
`0.01 m`로 되돌렸다. 실제 해결은 depth의 그리퍼 픽셀을 mask/self-filter한 뒤 새 nvblox map을
만들어 같은 목표를 다시 보내는 A/B 검증으로 진행한다. 낮은 장애물을 놓치지 않도록 ESDF 높이
범위를 영구적으로 높이는 방식은 해결책으로 채택하지 않는다.

### 수집 파일

- VSLAM/STM32 rosbag: [`data/vslam_mapping_20261003_144710`](../../../../data/vslam_mapping_20261003_144710)
- VSLAM/wheel 궤적 비교: [`analysis.md`](../../../../data/vslam_mapping_20261003_144710/analysis.md)
- Nav2/costmap rosbag: [`data/nav2_diagnostic_20261003_144710`](../../../../data/nav2_diagnostic_20261003_144710)
- Nav2 controller와 VSLAM launch 로그: `log/vslam_mapping_20261003_144710/vslam_nvblox.log`
- bridge/카메라 로그: `log/vslam_mapping_20261003_144710/stm32_bridge.log`, `realsense.log`
- 촬영한 camera frame: `log/vslam_mapping_20261003_144710/color.png`, `depth.png`

이번 run의 속도 제한은 시험 도중 parameter service로 설정한 값이며 YAML에는 저장하지 않았다.
이 기록은 실제 모터 E-stop 작동 시험이 아니다.
## 로봇 부품 self-filter

`nvblox_realsense.launch.py`는 이제 D435 깊이 영상과 nvblox 사이에서
`robot_self_filter.py`를 실행한다. 현재 필터는 camera intrinsics와 base-to-camera TF로
깊이 픽셀을 `base_link` 좌표로 바꾸고, 다음 그리퍼 swept volume 안에 들어오는 점을 지운다:
`x=0.08..0.43 m`, `y=-0.20..+0.20 m`, `z=0.00..0.22 m`. 기존보다 여유를 늘려 URDF 그리퍼,
손가락 개폐와 흔들림을 포함한다. 이 영역 안의 실제 장애물도 함께 지워지는
의도된 tradeoff다. 전체 영상의 고정된 아래쪽 띠나 낮은 장애물을 일괄 제거하지 않고,
원본 토픽은 유지하며 nvblox에는 `/leader/camera/depth/self_filtered`를 연결한다.

`SELF_FILTER_ENABLED=0`은 raw depth A/B 비교용 우회 설정이다. 필터 사용 중 TF/처리 오류가
발생하면 해당 프레임은 nvblox로 보내지 않고 throttled warning을 남긴다. 한 프레임이라도
raw depth가 누적돼 static map에 자기 점유가 남는 일을 막는다. 주행 전
`robot_self_filter` 로그와 filtered 토픽을 확인한다.

### 2026-10-03 첫 표면투영 필터 비교 이력

필터 추가 뒤 host와 Isaac ROS overlay를 빌드하고, `VSLAM_HEADLESS=1`
`STM32_I2C_WRITE_ENABLED=0`으로 카메라/VSLAM/nvblox/Nav2를 실행했다. 모터 bridge 쓰기는
비활성화했고 navigation 목표는 보내지 않았다. Filtered depth 토픽은 평균 약 18 Hz였다.
한 쌍의 848×480 frame에서 원본의 유효 픽셀 345,727개 중 3,234개(0.94%)가 제거됐다.

로봇 중심 costmap cell은 직전 원본 depth 시험 때 99였고 이번 필터 실행에서는 52였다.
하지만 두 실행 사이 Nav2 설정도 `robot_radius=0.30/inflation_radius=0.50 m`에서 현재
`0.20/0.30 m`로 바뀌었으므로 이 차이를 필터 효과로 단정할 수 없다. 이번 필터 실행에서
lethal cost 100 셀은 남았으며 가장 가까운 셀은 로봇 기준 전방 오른쪽 약
`(+0.23, -0.12) m`였다. 필터가 깊이 픽셀 일부를 제거하는 것까지는 확인했지만 costmap
개선 여부는 동일 Nav2 설정으로 raw/filtered 입력을 각각 새 map에서 비교해야 한다.

측정 자료: `log/vslam_mapping_20261003_153237/depth_raw.png`,
`depth_self_filtered.png`, `depth_filter_comparison.png`, `analysis.md`와
`data/vslam_mapping_20261003_153237` rosbag. 비교 이미지는 제거된 깊이 픽셀을 빨간색으로
표시한다.

### 첫 표면투영 버전: 같은 Nav2 설정 비교 이력

2026-10-03에 `SELF_FILTER_ENABLED=0`과 `=1`로 각각 fresh nvblox map을 만들었다. 두 run
모두 `robot_radius=0.20 m`, `inflation_radius=0.30 m`였고 selector는 TELEOP, STM32 bridge
쓰기는 꺼져 있었으며 주행 목표는 보내지 않았다. 두 rosbag의 VSLAM pose sample은 모두
`(0, 0)`이었다.

- Raw 입력: 중심 costmap cell 표본 10개의 중앙값 `60`; 가장 가까운 lethal 셀은
  `(+0.225, -0.075) m`.
- 필터 입력: 중심 cell 중앙값 `52`; 가장 가까운 lethal 셀은 `(+0.225, -0.125) m`.
- TF 처리 수정 뒤 pass-through 경고는 없었다. depth frame에서 유효 픽셀 약 0.96%를 제거했다.

필터가 그리퍼로 추정되는 점유와 중심 비용을 줄인 신호는 있지만, lethal 셀이 남았고 완전한
원인 제거는 입증하지 못했다. 설정은 동일하지만 두 map은 연속 재시작에서 생성됐으므로
환경/적분 차이도 남는다. 이번 결과를 주행 가능 판정으로 사용하지 않는다.
스냅샷은 `log/self_filter_ab_20261003/raw.json`, `filtered_tf_fixed.json`에 있다.

### 2026-10-03 확장 swept-volume 정지 확인

사용자 지시에 따라 그리퍼 고정 표면 대신 `x=0.12..0.38`, `y=±0.15`,
`z=0.00..0.16 m` (`base_link`) 안에 측정된 depth 점을 모두 제외하도록 넓혔다. TF 처리에
실패한 프레임은 raw로 통과시키지 않고 drop해 nvblox static map에 자기 점유가 남는 일을
막는다. Bridge write off, Nav2 goal 없음, 새 nvblox map에서 10개 local costmap 표본을 얻었다.
중심 비용은 모두 `0`; 가장 가까운 lethal 셀은 `(+0.575, +0.475) m`였다. 직전 좁은 구역 시험의
가까운 셀 `(+0.175, -0.075) m`는 더 이상 가까이 나타나지 않았다. 이번 정지 확인에서는
gripper self-occupancy가 제거된 것으로 판단한다.

단, 이 넓은 3D 구역 안에 놓인 실제 장애물도 depth map에서 제거된다. 정지 상태로 검증했으며
실제 그리퍼를 흔들거나 주행시키는 시험은 하지 않았다. 주행 전 작은 장애물이 그 영역 안에
있을 때 놓치는지 확인해야 한다. snapshot: `log/self_filter_ab_20261003/filtered_swept_volume.json`,
`filtered_swept_volume_wide.json`; run data: `data/vslam_mapping_20261003_162742`.

### 2026-10-03 swept-volume 추가 여유

후속 stopped costmap 표본에서 lethal cell이 로봇 기준 전방 오른쪽 footprint 모서리 가까이에
관측됐다. 사용자가 그리퍼 자기 점유로 판단해 margin을 조금 더 늘려 달라고 요청했다. 기본
필터 범위를 `x=0.08..0.43`, `y=±0.20`, `z=0.00..0.22 m` (`base_link`)로 변경했다.
새 mapping session에서 parameter 적용을 확인했다. 중심과 주변 5×5 costmap 셀은 0이었고
최근접 lethal cell은 약 `(+0.525,+0.475) m`였다. 확장된 volume 안의 실제 장애물도 함께
필터될 수 있다.

### 2026-10-03 확장 filter 후 frontier 주행 재시험

`data/vslam_mapping_20261003_202448`에서 시작 pose 기준 전방 2 m, 반경 2 m disk를 탐색했다.
속도 제한은 선속도 `0.10 m/s`, 각속도 `0.20 rad/s`였다. 첫 목표는 허용 오차 내 즉시 성공,
두 번째 목표는 약 4.6초 주행 후 성공했다. 세 번째 목표는 약 10초 후 `Failed to make
progress`로 abort했다. 이때 `/nav2/cmd_vel`은 우회전 방향 각속도 `-0.20 rad/s`까지 명령해,
관찰된 급우회전이 Nav2 출력에도 포함된 것을 확인했다. 종료 pose는 약 `(0.236,-0.069) m`,
yaw `-17.8°`였다. 원인은 확정하지 않았다. Controller 10 Hz loop miss와 BT tick warning도
발생했다. 실패 후 selector를 STOP으로 두고 zero command와 5초 정지 odom을 확인한 다음
mapping을 종료했다. 탐색은 불완전이며 성공으로 판정하지 않는다. 기록 bag에 VSLAM odometry와
status 표본이 없어 비교할 수 없다.

## Nav2 footprint: 차체와 그리퍼를 포함한 다각형

Nav2 costmap 설정은 `robot_radius` 원 대신 `base_link` 기준 6점 convex polygon과
`footprint_padding: 0.01`을 사용한다. 단순화 URDF의 바퀴 뒤쪽부터 그리퍼 끝까지
`x=-0.060..+0.3175 m`, 좌우 바퀴 바깥까지 `y=±0.145 m`를 감싼다. 그리퍼 두 손가락
사이의 빈 공간도 다각형 안에 포함되므로 좁은 틈에서는 보수적으로 동작한다.

이 footprint는 Nav2가 장애물과의 경로/충돌 여유를 계산할 때 쓴다. nvblox 지도에서
로봇 자신의 점을 지우지는 않으므로 앞 절의 depth self-filter는 별도로 유지한다.
간략화 URDF와 실물 치수 기록 사이에 차이가 있으므로 하드웨어 외곽 대조가 남아 있다.
2026-10-03 정지 bringup에서 `/local_costmap/published_footprint`와
`/global_costmap/published_footprint`를 확인했다. 두 토픽 모두 `odom` frame으로 발행됐고
padding 적용 꼭짓점은 `x=-0.070..+0.3275 m`, `y=±0.155 m`였다. Bridge write는 비활성화했고
Nav2 목표도 보내지 않았다. 실제 외곽과 치수는 하드웨어에서 한 번 더 대조해야 하며, 이
정지 확인만으로 주행 안전성을 보증하지 않는다.

## 2026-10-03 후속 실물 Nav2 주행: 0.10 m/s

확장 swept-volume 필터와 polygon footprint 적용 후 headless mapping/Nav2를 올려 실제 목표를
보냈다. 두 시험 모두 mapping bag을 기록했고, RViz는 실행하지 않았다.

- `vslam_mapping_20261003_163319`: 선속도 상한 `0.05 m/s`, 각속도 상한
  `0.15 rad/s`, `odom x=0.6 m` 목표. 약 10초 뒤 `Failed to make progress`로 abort했다.
  VSLAM 위치가 목표 진행 중 흔들렸고 `Unknown Tracker Error 2`가 기록됐다.
- `vslam_mapping_20261003_163828`: 선속도 상한 `0.10 m/s`, 각속도 상한
  `0.20 rad/s`. `odom x=0.5 m`, `x=0.9 m`의 두 action 목표가 모두 `SUCCEEDED`로 끝났다.
  selector는 각 시험 직전에 `NAV2`, 종료 뒤 `STOP`이었다.

성공 실행 bag에서 active `/leader/cmd_vel` 481개는 선속도 `0.0667..0.10 m/s`
(평균 `0.09 m/s`)였고 각속도는 `-0.20..+0.1579 rad/s` 범위였다. active command 전체
구간의 VSLAM pose 변화는 대략 `(+0.712,+0.014) m`, yaw `-0.09° → -7.64°`였다. wheel
odometry 변화는 대략 `(+0.82,-0.054) m`, yaw `-0.02° → -5.81°`였다. 이 구간에서 Nav2는
직진 목표에도 heading 보정을 보냈다. `/cmd_vel.linear.y`는 0이었으므로 직접 횡속도를 명령한
것은 아니다. 좌우 바퀴 개별 encoder/속도 telemetry가 rosbag에 없어, 관찰된 움직임 중
얼마가 controller 보정이고 얼마가 모터 편차인지는 분리하지 못했다.

두 목표의 성공은 실물 command 경로와 짧은 목표 action이 작동했음을 확인하지만, 정밀한
도착 오차나 반복성까지 의미하지 않는다. 기존 2026-10-03 초기 8 cm/trajectory 실패 기록은
그 당시 조건의 이력으로 유지하고, 최신 결과와 합쳐 단일 시험 결과처럼 취급하지 않는다.
상세 run data 및 분석은 `data/vslam_mapping_20261003_163319/analysis.md`,
`data/vslam_mapping_20261003_163828/analysis.md`에 있다. 이 rosbag과 log 디렉터리는 로컬
장비에 남겨뒀으며 Git에 포함하지 않는다.

다음 검증은 각속도 0인 짧은 직진으로 구동계 편향을 분리하고, 좌우 encoder/속도 telemetry를
기록하는 것이다. 이후 반복 직진·회전, 장애물 정지, cancel/failure 정지 동작을 확인한다.
E-stop 및 bridge watchdog은 아직 실물 시험하지 않았다.

### 2026-10-03 20:45 계측 재시험

계측을 추가한 뒤 새 run에서 목표 2개는 통과했지만, 세 번째 목표는 `/nav2/cmd_vel` 선속도
`0`과 제자리 회전만 10초 지속되어 `Failed to make progress`로 abort했다. wheel pose 변화는
약 4 mm였다. 선택 frontier의 ESDF clearance는 `0.354 m`로 기준 `0.35 m`를 겨우 넘었다.
현재 증거로는 DWB가 footprint 충돌/비용 때문에 전진 궤적을 선택하지 못했을 가능성이 크지만,
critic별 점수는 당시 bag에 없어 원인을 확정하지 못한다. 실행 중 controller loop miss는 없었다.
Explorer는 자기 입력 global OccupancyGrid가 stale됐다고 알렸으나, 그 topic 자체가 bag에 없어
Nav2 raw grid 기록과 비교할 수 없었다.

다음 run의 bag은 `/global_costmap/costmap`, local/global costmap update topic 및 DWB
`/evaluation`을 포함한다. DWB evaluation은 trajectory별 critic 점수를 보존하므로 정지 궤적이
선택된 이유를 가르는 데 쓴다. Filter는 약 20–27 Hz로 돌고 frame drop은 0이었다. 계측 중
selector 요약 logger에 severity 선택 버그가 발견되어 수정했고, selector와 두 ROS overlay를
다시 빌드했다. 이 주행 자료는 `data/vslam_mapping_20261003_204257/`와
`log/vslam_mapping_20261003_204257/`에 있다.

# Bounded frontier exploration

The explorer chooses its search disk once from the first wheel odometry sample
(`/leader/odom/raw`): the center is 2 m ahead in the robot's startup heading,
and the radius is 2 m.
It reads Nav2's `/global_costmap/costmap` (in `odom`), selects reachable free
cells next to unknown nvblox slice cells inside that disk, checks the Nav2
costmap for traversability and nvblox ESDF for 0.35 m obstacle clearance, and
sends them one at a time to `/navigate_to_pose`. The Nav2 OccupancyGrid alone
does not preserve the unobserved state in this setup, so the explorer reads
`/nvblox_node/static_map_slice` for frontier detection. Reached goals are
skipped on later scans. A rejected, aborted, or otherwise failed goal stops the
explorer for operator review. This bounded first-pass policy does not guarantee
complete coverage behind walls or beyond the current observed map slice. An
early physical trial ended after wall contact. Later controlled repeats verified
selector stopping, but this frontier policy still has an unresolved controller
stall described below and is not yet validated for coverage.

Start the existing mapping stack and wait until Nav2 is active. The repository
runner applies the requested speed ceiling, records the frontier node output,
and returns the selector to `STOP` when interrupted or when launch exits:

```bash
./scripts/run_frontier_exploration.sh
```

When exploration completes or is interrupted, stop the base through the
selector:

```bash
ros2 param set /leader/command_selector source_mode STOP
```

The explorer expects `/global_costmap/costmap`, `/leader/odom/raw`,
`/nvblox_node/static_map_slice`, and the Nav2 `NavigateToPose` action. The
current rolling global costmap is 10 m square, so it can represent the 4 m
diameter target disk while the robot moves within it. The first repeated
physical trial is recorded above; it reached two targets and then aborted on
the third. Treat the behavior as in-progress, not validated coverage.

The mapping runner stores per-process logs under `log/vslam_mapping_<run_id>/`,
including controller/Nav2 output, selector state/output summaries, RealSense,
STM32 bridge, rosbag status, and Jetson `tegrastats`. The frontier runner adds
`frontier_exploration.log` and ROS client logs in `ros_logs/`. The bag includes
commands from Nav2 and the selector, wheel odometry, action feedback/status,
global/local costmaps, footprint, plans, TF, `/rosout`, `/diagnostics`, and
parameter events. The explorer log reports frontier filter counts, selected
target/cost/ESDF clearance, goal duration/status, and periodic Nav2 feedback.
