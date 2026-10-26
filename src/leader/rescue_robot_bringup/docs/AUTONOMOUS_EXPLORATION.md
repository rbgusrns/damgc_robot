# 작은 실내 공간의 3D 매핑·자율 탐색

## 기본 설정: 범위 제한 없이 폐공간 탐색

사용자 요청에 따라 launch 기본값은 `area_radius_m=0`, `area_width_m=0`,
`area_height_m=0`이며 인위적인 범위를 두지 않는다. 아래 사각형/반경 인수는
선택 사항이다. 벽과 장애물, 도달 가능한 관측 영역을 기준으로 후보를 고른다.
기본 실행은 `ros2 launch rescue_robot_bringup frontier_exploration.launch.py dry_run:=true`다.
실제 주행은 같은 명령의 `dry_run:=false`로 실행한다.

## 현재 구현 범위

기존 D435 → nvblox 3D 매핑과 Nav2 위에서 ESDF slice의 free/unknown 경계를
선택하여 `NavigateToPose` 목표를 한 번에 하나씩 보낸다. 현재 pose에서 사방으로 연결된 영역 중 기본 0.36m 장애물 clearance를 만족하는 cell을 목표 후보로 쓴다.
연결성 검사에는 차체 반폭과 여유를 고려한 0.20m를 사용하며, 실제 경로의 회전·footprint 충돌은 Nav2가 판단한다.
Nav2는 실제 footprint와 costmap으로 최종 경로 가능 여부를 판단한다.
목표가 거부되거나 주행이 실패하면 자동 재시도하지 않고 정지한다.

사각형은 slice 좌표계(현재 3D 구성은 odom)의 축 방향이며 탐색 시작 pose에 고정한다.
**목표와 연결 영역의 범위이며, Nav2 경로 전체를 제한하는 가상 벽은 아니다.**
실물 외벽 밖으로 경로가 나가는 구조라면 costmap의 keepout 설정이 추가로 필요하다.
범위 안에 후보가 없다는 사실만으로 전체 지도 완성을 판정하지 않는다.
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
  dry_run:=true start_rviz:=true \
  area_width_m:=3.0 area_height_m:=4.0 area_radius_m:=3.0
```

사각 범위를 명시한 예제는 출발 위치가 직사각형 중앙이다. 출발점에서 odom +X 방향으로 3m,
Y 방향으로 좌우 2m씩인 배치라면 다음과 같이 준다. 축은 로봇의 현재 바라보는 방향이
아니라 RViz의 odom 축을 기준으로 확인한다.

```bash
ros2 launch rescue_robot_bringup frontier_exploration.launch.py \
  dry_run:=true start_rviz:=true \
  area_width_m:=3.0 area_height_m:=4.0 \
  area_offset_x_m:=1.5 area_offset_y_m:=0.0 area_radius_m:=4.0
```

반경 제한과 사각 제한은 동시에 적용된다. 큰 연습 공간에서는 두 치수와 반경을 함께 늘린다.
초록 테두리, 청록 후보, 빨간 선택 목표를 확인한다. 반경을 작게 주면 사각형 일부가 제외된다.
미리보기 자체는 목표나 selector 변경을 보내지 않는다. RViz에서 별도로 클릭한 Nav2 목표는
기존 selector 설정에 따라 실제로 주행하므로 자동 탐색 미리보기와 구분한다.

시야가 좁아 후보가 없으면 수동으로 천천히 주변을 관측해 초기 지도를 만든다.
`restart_fresh_mapping.sh`는 기본으로 초기 360도 scan을 요청한다. scan 완료 후 탐색을 실행한다.
기존 `run_vslam_mapping.sh`의 기본 initial scan 설정은 그대로이며,
`FRESH_INITIAL_SCAN=0`을 주면 fresh wrapper에서도 회전을 생략한다.

## 2. 반복 탐색

미리보기 launch를 Ctrl+C로 종료한 후 **동일한 범위 인수**로 `dry_run:=false`를 실행한다.
탐색 시작 위치를 범위 원점으로 다시 저장하므로 미리보기 중 로봇을 옮겼다면 범위를 재확인한다.

```bash
ros2 launch rescue_robot_bringup frontier_exploration.launch.py \
  dry_run:=false start_rviz:=true \
  area_width_m:=3.0 area_height_m:=4.0 area_radius_m:=3.0
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
