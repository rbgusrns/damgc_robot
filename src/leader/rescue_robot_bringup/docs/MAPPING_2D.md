# 수동 2D 매핑과 목표 이동

실행: `./scripts/run_2d_mapping.sh` (GUI 터미널). 기존 3D 실행은
`./scripts/run_vslam_mapping.sh`로 유지한다. 두 실행은 같은 관리 잠금과 센서를
사용하므로 동시에 실행하지 않는다. 종료는 실행 터미널의 Ctrl-C이며 rosbag을
닫고 분석을 저장한다. 로그와 bag은 `log/mapping_2d_*`, `data/mapping_2d_*`에 남는다.

방향키/Q/W/A/S는 수동 주행으로 전환하며 진행 중인 Nav2 목표를 취소한다.
RViz `Nav2 Goal`을 누르고 위치에서 드래그하면 목표와 도착 방향을 지정한다.
새 목표는 selector를 NAV2로 전환한다. Space와 목표 종료는 STOP으로 전환한다.
자동 탐색과 자동 시작 회전은 실행하지 않는다.

2D 실행에서는 RealSense depth 424x240@15만 켠다. 색상·IR·정렬·pointcloud·VSLAM·
nvblox·3D mesh는 실행하지 않는다. `depth_obstacle_scan.py`가 4픽셀 간격으로
샘플링하며 최대 10Hz로 `/scan`을 발행한다. base_link 기준 높이 0.01..0.30 m,
평면 거리 0.20..4.0 m의 점만 사용하고 몸체/그리퍼 영역을 제외한다. 높이 범위는
현재 3D costmap의 기준을 가져온 초기값이며 실물 장애물로 조정해야 한다.
거리나 높이 필터를 통과하지 못한 방향과 카메라 밖 방향은 NaN으로 남기며
자유 공간으로 가정하지 않는다. 카메라와 로봇 변환은 depth 이미지 시각을 사용한다.

TF 소유권: local wheel/IMU EKF가 `odom -> base_link`, slam_toolbox가
`map -> odom`을 발행한다. global EKF와 VSLAM은 실행하지 않는다. `/map`은
5cm 해상도 2D occupancy grid이며 global costmap은 static/obstacle/inflation,
local costmap은 obstacle/inflation을 사용한다. RPP와 후진 가능한 Smac planner를 유지한다.

카메라는 전방만 관측한다. 사용자가 회전/주행해 관측 범위를 넓혀야 한다.
이 가상 scan은 여러 높이의 반사점을 평면으로 투영하므로 실제 2D 라이다와 다르며,
지도 정합과 낮은/높은 장애물 검출을 실차로 검증해야 한다. 바닥 낙차는 검출하지 않는다.
맵은 프로세스가 종료되면 초기화된다. 별도 저장은 `/slam_toolbox/save_map`과
`/slam_toolbox/serialize_map` 서비스를 사용할 수 있다. rosbag은 자동 재시작 지도 파일이 아니다.

Nav2 장애물 레이어의 유효 스캔과 ray tracing 설정은
[Nav2 공식 문서](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/),
SLAM 설정은 [slam_toolbox 공식 설정](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml)을 참고한다.

## 2026-10-04 초기 실행 확인

실행 `mapping_2d_20261004_150524`에서 RealSense depth 424x240@15,
Nav2 lifecycle 전체 active, `/scan` 및 `/map` 생성을 확인했다. 관측 12초 동안
scan 약 7.50Hz, 수신 최대 간격 약 144ms, 마지막 scan의 유효 방향 69개였다.
15fps 입력에 최대 10Hz 제한을 적용해 주로 2프레임마다 출력되는 주기다.
관측 중 사용자 TELEOP 명령도 수신되어 정지 조건의 측정은 아니다.
`readiness.json`에 당시 지도 크기, 명령, CPU/RSS를 저장했다. CPU 값은 1코어
100% 기준으로 scan 노드 17.5%, slam_toolbox 5.4%, RViz 28.5%였다.
동일 조건의 3D 비교 측정은 수행하지 않아 전체 부하 감소율을 단정하지 않는다.

관측된 빈 공간 약 0.38m 거리로 `/compute_path_to_pose`를 요청해 SUCCEEDED와
6개 path pose를 받았다. 모터 주행 목표는 보내지 않았다. 결과는
`log/mapping_2d_20261004_150524/planning_check.json`에 저장했다. 실제 목표 추종,
후진 경로, 반복 매핑 정합은 추가 확인이 필요하다. 이 초기 실행에서는 이전 3D
실행의 planner exit -11을 재현하지 않았으며 원인이 해결됐다고 단정하지 않는다.
깊이 투영의 최근접 장애물/미지 방향/잘못된 depth/높이 및 자기 몸체 제거/카메라
설치 변환 검증과 기존 관련 테스트 총 10개가 통과했다.
