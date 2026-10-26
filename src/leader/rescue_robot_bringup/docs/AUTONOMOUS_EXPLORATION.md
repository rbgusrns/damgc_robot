# 3m × 3m 지도 탐색 — 안정화 구성 복구

## 짧은 이동과 도착 방향 맞추기

기본 `maximum_step_distance_m=0.60`으로 Nav2가 계산한 경로를 경로 길이
60cm씩 나눠 이동한다. 장애물을 가로지르는 직선 중간 목표를 만들지 않는다.
목표마다 지도에서 다음 관측 후보를 고르고, 주행 중 경로도 0.5Hz로 갱신한다.
초록색은 관측 후보를 고르는 영역이며 경로 또는 중간 waypoint를 막는 벽이
아니다. 이동·복귀 경로는 충돌 검사를 통과하면 경계 밖을 경유할 수 있다.

자동 주행에는 위치 전용 goal checker(12cm)를 사용한다. 위치에 도착한 뒤
남은 방향 오차가 0.25rad보다 크면 Nav2 `Spin`으로 제자리에서 각도만 맞춘다.
수동 RViz 목표는 기존 general goal checker를 사용한다. 복귀 마지막 구간도
이동 방향과 최종 출발 방향을 분리해 출발점에서 전후진을 반복하지 않게 한다.
Nav2 `BackUp` 플러그인을 연결해 탐색기의 후진 복구 요청을 처리한다.

2026-10-27 실물 시험에서 360도 스캔과 짧은 목표 8개 도착을 확인했다.
마지막 복귀가 경계 검사에 걸려 중단돼 검사를 제거하고 복귀만 재실행했으며,
출발점 복귀와 마지막 방향 회전이 성공했다. 이후 융합 odometry 기준 거리는
출발점에서 약 13.5cm, 방향 차이는 약 0.95°였다(외부 계측 오차가 아님).
지도 셀 기준 관측률은 74.2%여서 3m × 3m 전체 관측 완료로 해석하지 않는다.
이번 시험의 관측·복귀 결과와 남은 한계는 복구 보고서에 기록했다.

## 2026-10-27 복구 기준

GitHub `origin/main`을 fetch해 비교했다. 최신 `c411ca2`는 자율탐색과 사람 마커를
함께 추가한 커밋이며, 3m × 3m 변경은 별도 커밋으로 남아 있지 않다.
매핑 경로는 그 전의 `f7255ad` 트리를 기준으로 복구했다. 해당 경로는
`f342249`의 바퀴·IMU 보정 및 매핑 안정화 작업 이후 구성이다.

`nvblox_realsense.launch.py`, `nvblox_vslam_realsense.launch.py`,
`visual_slam_realsense.launch.py`는 `f7255ad`와 동일하다. 원래 스테레오
RealSense/VSLAM, dual EKF, nvblox 경로를 사용하고, 2D RViz로 표시한다.
기본 카메라는 30 FPS, Nav2 inflation radius는 0.30m, footprint padding은
0.01m다. 요청한 부하 지연 여유에 따라 TF tolerance 1초, action 응답 5초,
진행 검사 대기 20초는 유지한다. I²C 주기는 변경하지 않았다.

RGB-D 위치추정·사용자 DDS XML·localhost 전용 격리·색상/메시 계산 중지
실험은 이 실행 경로에 적용하지 않는다. 관련 실험 파일은 복구 백업의
`retired_experiments`로 옮겼다. 사람 인식은 지도 탐색에 연결하지 않는다.

현재 변경사항의 백업:
`/home/maze/backups/damgc_mapping_recovery_20261027_010105/`.
소스 snapshot, 전체 working-tree patch, Git 상태와 기준 commit이 들어 있다.

## 한 번에 실행

로봇을 새 시작 위치에 둔 뒤 실행한다. 실행 전의 지도·위치추정 상태를
다시 사용하지 않는다.

```bash
bash scripts/run_map_only_exploration.sh
```

새 지도 → 최신 입력 확인 → 초기 360도 스캔 → 3m × 3m 탐색 → 시작점 복귀를
순서대로 진행한다. 스캔 전 pose로 초록색 영역과 복귀 위치·방향을 고정한다.
입력 검사 또는 스캔 실패 시 탐색은 활성화되지 않고 STOP으로 남는다.
`PREPARING`, `SCANNING`, `EXPLORING`, 정지·완료 원인은 해당 세션의
`automatic_exploration.log`에 기록된다. 이 스크립트는 YOLO와 rosbag을 기본으로
실행하지 않는다. 기록이 필요하면 `MAPPING_RECORD_BAG=1`을 붙인다.

영상의 짧은 끊김은 허용하며 10초 이상 영상이 없거나 10초 이상 오래된
영상만 들어오면 탐색을 멈춘다. 지도 freshness는 10초, robot TF freshness는
3초다. TF 수신은 별도 노드·스레드에서 처리한다. Nav2 충돌 검사는 유지한다.

로봇을 손으로 옮기지 않았고 기존 세션이 실행 중이라면 `--retry`로 원래
영역과 복귀 pose를 현재 로봇 좌표계로 보존하며 새 지도를 시작할 수 있다.
새 위치에 손으로 옮긴 경우에는 옵션 없이 실행한다.

```bash
bash scripts/run_map_only_exploration.sh --retry
```

종료는 Ctrl-C다. 종료 신호는 실행기를 실제 종료하며 다음 기동 단계로
계속 진행하지 않는다. 실행기는 파일 잠금으로 중복 기동을 거부하고,
기존 로봇 노드가 남아 있어도 출발하지 않는다. Docker 내부 기동도 독립
프로세스 그룹으로 관리하여 부모가 먼저 끝나도 남은 자식까지 종료한다.
잠금 획득에 실패한 두 번째 실행기가 첫 번째 세션을 정리하지 않는다.

코드·TF 및 프로세스 회귀 검증은 장치 없는 환경에서 수행했다.
복구 후 실제 주행·지도 품질은 아직 재시험하지 않았다.

## 기본 설정: 시작점 앞의 3m × 3m 정사각형 탐색

launch 기본값은 시작 pose와 방향을 기준으로 전방 3m, 우측 3m인 3m × 3m
사각형이다. 시작 위치는 뒤쪽 변의 좌측 끝, 즉 사각형의 뒤쪽 왼쪽 모서리에 놓인다.
경계는 시작 yaw에 맞춰 회전하며, 사각형 안에서 접근 가능한 frontier와 관측 지점을
순서대로 방문한다. 기본 실행은
`ros2 launch rescue_robot_bringup frontier_exploration.launch.py dry_run:=true`로 후보를
미리 보는 것이다. 실제 주행은 같은 명령에 `dry_run:=false`를 준다.

## 현재 구현 범위

기존 D435 → nvblox 3D 매핑과 Nav2 위에서 ESDF slice의 free/unknown 경계를
선택하여 `NavigateToPose` 목표를 한 번에 하나씩 보낸다. 현재 pose에서 사방으로 연결된 영역 중 기본 0.60m 장애물 clearance를 만족하는 cell을 목표 후보로 쓴다.
연결성 검사에는 차체 반폭과 여유를 고려한 0.20m를 사용하며, 실제 경로의 회전·footprint 충돌은 Nav2가 판단한다.
Nav2는 실제 footprint와 costmap으로 최종 경로 가능 여부를 판단한다.
주행 중 앞의 장애물이 costmap에 반영되면 Nav2가 경로를 다시 계산한다. 목표가 abort되면
실패 목표 주변 기본 1.0m를 후보에서 제외하고, 2초 뒤 갱신된 지도에서 다른 목표를 고른다.
주행 abort 뒤에는 Nav2의 costmap 검사형 `BackUp` 동작으로 최대 0.20m를 0.05m/s로 후진한다.
후진 경로가 막히면 0.10m, 0.05m로 줄여 두 번 더 안전 검사를 한다. 모두 막히면 현재 위치에서
다른 frontier 경로를 고른다. 연속 3회 실패하면 마지막 recovery 뒤 운영자 확인을 위해 멈춘다.
연속 3회 실패하면 현재 위치가 장애물 셀로 잘못 처리됐거나 통로가 막혔을 수 있으므로
정지해 운영자가 확인한다. 시작 셀 자체가 lethal인 경우에는 새 경로를 만들 수 없으므로
장애물 회피만으로 복구된다고 보장하지 않는다.

탐색 후보가 모두 소진되면 기본 동작은 시작 시 저장한 위치와 방향으로 `NavigateToPose`를
보내 복귀하는 것이다. 복귀에 성공한 뒤 `COMPLETE`를 발행한다. 복귀 경로가 거부·중단되거나
시간 초과가 나면 재시도하지 않고 `STOP`으로 끝난다. `return_to_start:=false`로 자동 복귀를
끄고 탐색 목표 소진 시 바로 완료 처리할 수 있다.

사각형은 관측 후보 목표만 제한하며, 연결성 검사와 Nav2 이동 경로는 영역 밖을 경유할 수 있다.
Nav2는 실제 footprint와 costmap으로 충돌을 검사하므로, 경로가 영역 밖으로 나갈 수 있는
장애물이 있다면 별도 keepout 설정이 필요하다. 탐색 완료는 현재 연결되어 관측된 공간에서
더 방문할 frontier/관측 목표가 없다는 뜻이며, 가려진 공간까지 탐색했다는 보장은 아니다.
생존자 발견·복귀·파지는 아직 이 탐색에 연결하지 않았다.

## 빌드와 기본 매핑

Isaac ROS 컨테이너 안에서 기존 overlay를 읽고 bringup 패키지를 갱신한다.

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install_docker/setup.bash
colcon build --build-base build_frontier --install-base install_docker --packages-select rescue_robot_bringup
source install_docker/setup.bash
```

호스트의 기존 매핑 터미널에서는 평소 실행법을 사용한다.

```bash
cd /home/maze/damgc_robot
MAPPING_INITIAL_SCAN=0 bash scripts/run_vslam_mapping.sh
```

D435, STM32 bridge, selector, Nav2를 중복 실행하지 않는다.
이 스크립트의 초기 source는 STOP이며, 탐색 노드가 자신의 목표를 수락받으면 NAV2로 전환한다.
기존 Nav2 설정의 기본 선속도는 0.10m/s다. 별도 설정을 적용했다면 실제 parameter를 확인한다.

```bash
ros2 param get /controller_server FollowPath.desired_linear_vel
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

## 1. 목표 미리보기

컨테이너의 별도 터미널에서 위 ROS/overlay를 source한다.

```bash
ros2 launch rescue_robot_bringup frontier_exploration.launch.py \
  dry_run:=true start_rviz:=false
```

RViz의 초록 테두리가 시연 영역이며 출발 위치는 뒤쪽 왼쪽 모서리다. 로봇 정면으로 3m,
로봇 기준 오른쪽으로 3m가 영역 안에 들어간다. 청록 점은 후보,
빨간 화살표는 다음 목표다. `area_align_with_start_heading:=true`가 기본이므로 정사각형은
odom 축 대신 시연 시작 시 로봇이 바라보는 방향을 따른다. 다른 직사각형/오프셋은 아래처럼
조정할 수 있다.

```bash
ros2 launch rescue_robot_bringup frontier_exploration.launch.py \
  dry_run:=true start_rviz:=false \
  area_width_m:=4.0 area_height_m:=3.0 \
  area_offset_x_m:=2.0 area_offset_y_m:=0.0
```

`area_offset_x_m`과 `area_offset_y_m`은 로봇 기준 전방·좌측 축에서 사각형 중심의
오프셋이다. 기본값 `(1.5, -1.5)`라 시작 위치가 뒤쪽 왼쪽 모서리에 놓인다. 미리보기
자체는 목표나 selector 변경을 보내지 않는다.

시야가 좁아 후보가 없으면 수동으로 천천히 주변을 관측해 초기 지도를 만든다.
`restart_fresh_mapping.sh`는 기본으로 초기 360도 scan을 요청한다. scan 완료 후 탐색을 실행한다.
기존 `run_vslam_mapping.sh`의 기본 initial scan 설정은 그대로이며,
`FRESH_INITIAL_SCAN=0`을 주면 fresh wrapper에서도 회전을 생략한다.

## 2. 반복 탐색

미리보기 launch를 Ctrl+C로 종료한 후 같은 로봇 시작 위치·방향에서 `dry_run:=false`로
실행한다. 실제 주행에서도 그 시작 pose와 yaw를 기준으로 영역을 한 번 고정한다.

```bash
ros2 launch rescue_robot_bringup frontier_exploration.launch.py \
  dry_run:=false start_rviz:=false
```

목표 도착 → source STOP → 갱신된 지도에서 다음 목표 선택을 반복한다.
지도 수신이 2초 넘게 끊기거나 pose TF가 오래됐거나, 한 목표에 90초가 지나면
탐색을 정지하고 자신의 Nav2 목표를 취소한다. 기본 시간 제한은 `goal_timeout_sec`으로 설정한다.
수동 source 변경 요청도 탐색을 종료한다. 재개하려면 launch를 다시 실행한다.

```bash
ros2 service call /frontier_explorer/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /frontier_explorer/status
ros2 topic echo /leader/command_selector/status
```

첫 실물 통과 조건은 서로 다른 목표 2개 이상을 자동 선택·이동하며 지도가 확장되는 것,
그리고 탐색 중 stop 서비스로 다음 목표 생성과 주행을 중단하는 것이다.
ROS 미들웨어·실물 주행은 현장에서 확인해야 한다. 합성 지도와 콜백 테스트만으로
실물 도착 정확도나 정지 거리를 검증했다고 판단하지 않는다.


## 빠른 재시험과 2026-10-26 실물 결과

로봇을 손으로 옮겼다면 이전 지도/위치 추정을 사용하지 말고 호스트의 제어 터미널에서:

```bash
cd /home/maze/damgc_robot
bash scripts/restart_fresh_mapping.sh
```

이 명령은 이전 관리 세션이 있으면 종료를 기다리고, 저장 지도/anchor를 읽지 않은 채
새 nvblox·VSLAM·EKF 세션을 시작한다. 초기 source는 STOP이고 rosbag 준비 뒤 360도
제자리 scan을 수행한다. 기존 bag/저장 지도를 삭제하지 않는다.
scan 완료 후 컨테이너 터미널에서 탐색한다:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install_docker/setup.bash
ros2 launch rescue_robot_bringup frontier_exploration.launch.py dry_run:=false
```

탐색은 전방 30도 안의 후보를 우선하고, 기본 1.5m 거리와 넉넉한 장애물 여유를 선호한다.
`preferred_goal_distance_m`은 선호 거리이며 탐색 범위 제한이 아니다.
도착 방향은 이동 방향을 따른다. 프론티어가 없으면 관측된 빈 공간의 미방문 지점을 선택한다.
관측 지점은 출발점/성공한 목표에서 0.75m 이상 떨어진 후보를 사용한다.
0.10m² 미만의 작은 미관측 구멍은 프론티어로 취급하지 않는다.

실물 확인:

- 첫 시험은 약 (0.38, 0.33), 도착 yaw 135도 목표였다. 기록된 경로가 왼쪽으로 휘었고
  실제 yaw 약 91도에서 controller collision check로 abort, selector STOP을 확인했다.
- 출발 위치로 수동 재배치한 뒤 새 지도/위치 추정을 시작했다.
- 360도 제자리 scan이 성공했고 selector가 STOP으로 복귀했다.
- 수정한 로직은 약 (1.47, 0.07), yaw 약 3도 목표를 선택했다.
- ComputePathToPose 사전 점검에서 전방 약 1.47m의 거의 직진 경로를 확인했다.
- 실물 NavigateToPose가 SUCCEEDED로 종료했고 selector STOP을 확인했다.
- 후속 목표로 남은 약 0.0125m²의 출발점 미관측 구멍을 발견하여 작은 구멍 제외를 추가했다.
- 최종 helper/콜백 테스트 23개 통과. 전체 공간 반복 순회 성공으로 판정하지 않는다.

기록: `data/vslam_mapping_20261026_192643`, `data/vslam_mapping_20261026_193514`.
두 번째 기록의 기본 매핑 세션은 주행 종료 후에도 살아 있을 수 있다. 로봇을 재배치하면
반드시 fresh 재시작을 사용한다. 실제 주행은 단독 미션/생존자 전달과 아직 연결하지 않았다.

## 2026-10-26 저녁 추가 진행상황

- 사람 등록 기준을 카메라 좌표계 거리 2.0m로 설정했다. 유효한 깊이 좌표가 이 범위에
  들어오면 1회 검출만으로 tentative track을 확정하도록 registry core/node를 수정했다.
- 사람 감지 표시와 별도로 registry marker를 RViz에 추가해 트랙이 화면에서 사라져도
  마지막 위치를 계속 표시하도록 했다.
- 새 세션 `data/vslam_mapping_20261026_212819`에서 맵과 위치 추정을 초기화하고,
  제자리 360도 scan을 완료했다. command selector는 STOP 상태였다.
- 이 세션에서는 사람 상자 검출은 관찰됐지만 detector가 RGB와 depth 사이에 큰 timestamp
  차이를 기록해 유효한 XYZ/map 위치를 얻지 못했다. 따라서 2m 1회 등록과 지속 마커는
  코드에 반영됐지만, 이 세션의 실물 주행으로 동작을 확인하지 못했다.
- 사용자가 중단을 요청한 뒤 탐색기를 실행하지 않고 mapping, 카메라, Nav2와 survivor
  pipeline을 종료했다. rosbag은 정상 finalize되어 위 경로에 저장됐다.
- 이 세션은 전체 폐공간 탐색 또는 생존자 위치 표시의 성공으로 집계하지 않는다.
  재시험은 robot 위치를 고정하고 fresh mapping 및 360도 scan 후 진행한다. 사람 marker를
  확인하려면 detector RGB/depth cache의 timestamp 불일치를 먼저 해결하고 map 좌표가
  나오는지 확인해야 한다.

## 2026-10-26 시작점 기준 3m × 3m 실물 시험

- 시작 위치를 다시 잡고 fresh mapping 및 360도 scan 뒤 2D RViz에서 초록색 영역 경계를
  확인했다. 시작 pose는 약 `(0, 0)`, yaw 약 `2.7°`였다. 로봇 기준 앞쪽 3m·오른쪽 3m,
  뒤쪽 왼쪽 모서리 시작으로 설정했다.
- 자동 탐색은 `(1.37, -0.03)`, `(1.82, -0.63)`, `(1.38, -1.28)` m 목표 3개를
  순서대로 `SUCCEEDED`로 마쳤다. 모두 지정 영역 안이었다.
- 이후 한 목표는 RPP collision-ahead로 중단됐고, 두 목표는 Nav2 planner가 현재 로봇
  시작 셀을 lethal로 판단해 경로를 만들지 못했다. 3회 연속 실패 제한에 도달해 탐색기가
  중단됐으며 command selector는 `STOP`으로 복귀했다.
- 이동 중 Visual SLAM frame 간격이 최대 약 2.1초까지 늘고 EKF update-rate 경고가
  반복됐다. 제한 영역의 경계·후보 표시와 여러 목표 자동 주행은 실물에서 확인했지만,
  3m × 3m 전체 커버리지 완료로 판정하지 않는다. 위치/비용지도 지연과 Nav2 시작 셀의
  lethal 판정 원인을 해결한 뒤 다시 시험해야 한다.

rosbag: `data/vslam_mapping_20261026_221250/`. 현재 2D 화면은 투영 지도, green frontier
boundary, survivor marker display를 사용하며 3D mesh/pointcloud display는 없다.

## 2026-10-26 obstacle-clearance 설정 실물 확인

- 사용자가 시작점에 로봇을 재배치해 새 mapping/VSLAM 세션을 시작했다. 첫 새 세션의
  자동 360도 scan은 Nav2 `Collision Ahead`로 중단됐다. 재배치 후 두 번째 세션은 초기
  회전을 건너뛰고 시작 pose를 다시 잡았다.
- 2D dry run에서 시작 pose `(0, 0)`, yaw 약 `0.5°`, 첫 후보 `(1.73, -0.83)`m,
  clearance `0.68m`를 확인했다. 실제 실행에서는 최소 clearance `0.60m`, 실패 목표 회피
  반경 `1.0m` 설정이 로드됐다.
- 실제 자동 주행 목표 4개가 연속 `SUCCEEDED`했다. 마지막 목표 뒤 탐색기는 현재 연결된
  지도에서 더 방문할 후보가 없다고 보고 `COMPLETE`를 냈고 selector는 `STOP`으로 복귀했다.
- 이번 주행에서는 Nav2 경로 중 장애물로 인한 abort가 발생하지 않았다. 따라서 실패 목표를
  제외하고 다른 목표를 고르는 동작은 코드/로직에 반영됐지만, 장애물을 실제로 만나 우회
  경로로 바뀌는 상황을 이번 실물 주행에서 재현하지는 않았다. 전체 공간 커버리지 보장도 아니다.

기록: `data/vslam_mapping_20261026_223906/`.
