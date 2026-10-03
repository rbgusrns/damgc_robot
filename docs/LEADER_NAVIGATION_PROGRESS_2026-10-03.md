# 리더 로봇 VSLAM·Nav2 실물 진행 기록

기록일: 2026-10-03
브랜치: `main`
목표: D435 기반 3D mapping을 유지하면서 Nav2 목표를 실제 Leader 로봇에 보내고,
이후 자율 탐색·생존자 발견·시작 위치 복귀 시나리오의 주행 기준선을 만든다.

## 이번에 확인한 구성

- RealSense D435 depth → robot self-filter → nvblox mapping/costmap
- VSLAM `odom` pose/TF → Nav2 planner/controller
- Nav2 `/nav2/cmd_vel` → Leader command selector `NAV2` → `/leader/cmd_vel` → STM32 bridge
- RViz는 GPU 자원 부담 때문에 끄고(`VSLAM_HEADLESS=1`) topic, action, rosbag으로 확인
- 자율주행 중 STM32 I²C write를 켰다. 시험 종료 시 selector를 `STOP`으로 전환하고
  mapping stack을 종료했다.

## 그리퍼 depth self-filter와 footprint

nvblox 입력 앞에 `robot_self_filter.py`를 추가했다. CameraInfo 및 camera-to-`base_link` TF를
사용해 depth 픽셀을 로봇 좌표로 변환하고, 측정 점이 아래 swept volume에 들어오면 depth를
제거한다.

```text
x: 0.08..0.43 m
y: -0.20..+0.20 m
z: 0.00..0.22 m
```

이는 흔들리거나 움직이는 그리퍼를 넉넉히 덮도록 의도적으로 크게 잡은 범위다. 따라서 그
공간에 실제 장애물이 있으면 함께 지워질 수 있다. TF/처리에 실패한 frame은 raw depth로
통과시키지 않고 drop해 미필터 데이터가 nvblox static map에 누적되지 않게 한다.
`SELF_FILTER_ENABLED=0`은 raw depth A/B용이다.

Nav2 global/local costmap은 원형 `robot_radius` 대신 차체와 그리퍼 바깥을 근사하는 convex
polygon 및 1 cm padding을 사용한다. footprint는 경로 충돌 계산용이며, nvblox에 쌓인 자기
점유 데이터를 제거하지 않으므로 depth self-filter를 별도로 유지한다.

확장 swept-volume 적용 후 정지 상태 costmap에서 중심 비용은 `0`이었고 가장 가까운 lethal
cell은 로봇 기준 약 `(+0.525, +0.475) m`였다. 직전 시험보다 여유를 조금 더 늘린 범위다.
실제 장애물 누락이나 주행 중 흔들림까지 검증한 것은 아니다.

## 실물 Nav2 목표 주행

Mapping 실행:

```bash
SELF_FILTER_ENABLED=1 VSLAM_HEADLESS=1 STM32_I2C_WRITE_ENABLED=1 \
  ./scripts/run_vslam_mapping.sh
```

RViz 없이 짧은 `NavigateToPose` 목표로 시험했다. Controller runtime 속도 상한은 YAML에
저장하지 않고 시험 중 parameter로만 설정했다.

| 실행 | Nav2 속도 상한 | 결과 |
| --- | ---: | --- |
| `vslam_mapping_20261003_163319` | 선속도 `0.05 m/s`, 각속도 `0.15 rad/s` | `odom x=0.6 m` 목표가 약 10초 후 `Failed to make progress`로 abort. 중간 VSLAM pose가 흔들렸고 `Unknown Tracker Error 2` 경고가 있었다. |
| `vslam_mapping_20261003_163828` | 선속도 `0.10 m/s`, 각속도 `0.20 rad/s` | `odom x=0.5 m`, `x=0.9 m` 목표 action이 모두 `SUCCEEDED`. active command 구간에서 실제 VSLAM 전진이 기록됐다. |

두 번째 실행의 active `/leader/cmd_vel` 샘플은 481개였고, 선속도는 `0.0667..0.10 m/s`,
평균 `0.09 m/s`였다. 각속도는 `-0.20..+0.1579 rad/s` 범위였고 부호/무보정 구간 전환이
10회 관찰됐다. 두 목표 사이의 전체 active command 시간대에서 VSLAM pose는 대략
`(+0.712, +0.014) m`, 방향은 `-0.09° → -7.64°`로 변했다. 같은 구간 wheel odometry는
대략 `(+0.82, -0.054) m`, `-0.02° → -5.81°`였다. 전체 bag 기준 경로 길이는 wheel
`0.855 m`, VSLAM `0.908 m`였고 차이는 `0.053 m`였다.

Nav2는 직진 목표에서도 각속도를 변경했으므로 관찰된 좌우 움직임 일부는 heading/path 보정에
따른 것이다. `/cmd_vel.linear.y`는 0이었다. 다만 bag에는 좌우 모터 개별 속도나 encoder가
없어 모터 간 출력 차이가 함께 기여했는지는 판정하지 못한다. action 성공은 목표 허용 오차
안에 들어왔다는 뜻이며, 정밀 도착 오차가 확인됐다는 뜻은 아니다.

## 산출물

| 항목 | 위치 |
| --- | --- |
| 첫 저속 실패 rosbag 및 분석 | `data/vslam_mapping_20261003_163319/`, `analysis.md` |
| 성공 실행 rosbag 및 분석 | `data/vslam_mapping_20261003_163828/`, `analysis.md` |
| 실행 로그 | `log/vslam_mapping_20261003_163319/`, `log/vslam_mapping_20261003_163828/` |
| 구현·설정 및 통합 설명 | [`NVBLOX_NAV2_RVIZ.md`](../src/leader/rescue_robot_bringup/docs/NVBLOX_NAV2_RVIZ.md) |
| 전체 임무 단계 계획 | [`LEADER_SCENARIO_V2_DEVELOPMENT_PLAN.md`](LEADER_SCENARIO_V2_DEVELOPMENT_PLAN.md) |

rosbag과 실행 로그는 장비의 로컬 작업 공간에 있고 이 문서 변경과 함께 GitHub에 올리지
않는다. 분석 수치와 재현 경로를 이 기록에 남긴다.

## 다음 단계

1. 각속도 `0`인 짧은 저속 직진을 수행하고 실제 경로가 휘는지 기록한다. 이 결과로 기계적
   구동 편향과 Nav2 heading 보정을 구분한다.
2. 좌우 wheel encoder/속도 telemetry를 rosbag에 추가해 명령 대비 각 바퀴 응답을 비교한다.
3. 고정된 직선 및 회전 목표를 반복해 goal 허용 오차, 누적 yaw, lateral drift를 측정한다.
4. 작은 장애물 정지와 Nav2 cancel/failure 시 zero command를 확인한다. E-stop과 bridge
   watchdog은 별도 실물 검증으로 기록한다.
5. 이 기준선이 확보된 뒤에야 mapping 중 탐색, survivor 위치 보존, 시작 pose 복귀로 진행한다.

현재까지 확인된 것은 짧은 실물 목표 주행이다. 반복 자율 탐색, 사람 발견 후 복귀,
구급함 파지·전달 end-to-end는 아직 수행하지 않았다.

## 제한 구역 frontier 탐색 구현 (2026-10-03)

시연 요구에 맞춰 시작 시 VSLAM pose와 heading으로부터 전방 2 m 지점을 원 중심으로
고정하고, 반경 2 m 안의 주행 가능 셀 중 미지 셀과 맞닿은 frontier를 Nav2
`NavigateToPose`로 하나씩 방문하는 `frontier_exploration.py`를 추가했다. 방문/실패한
목표 주변 0.35 m는 다음 선택에서 제외하고, 새 frontier가 없으면 종료한다. 매핑 및 Nav2
실행 후 controller 상한을 검증된 선속도 0.10 m/s, 각속도 0.20 rad/s로 설정하고
selector를 `NAV2`로 바꿔 노드를 실행하는 절차는
[`NVBLOX_NAV2_RVIZ.md`](../src/leader/rescue_robot_bringup/docs/NVBLOX_NAV2_RVIZ.md)에
기록했다.

이는 frontier 선택·주행 루프의 구현 완료이지 실물 탐색 검증 완료가 아니다. Isaac ROS
컨테이너에서 패키지를 빌드하고, 개방된 시험 구역에서 목표가 지정 원 안에 머무는지,
장애물 앞에서 Nav2가 정지하는지, 더 탐색할 곳이 없을 때 멈추는지를 확인해야 한다.

### 첫 실물 탐색 시도: 실패, 벽 접촉 보고

같은 날 Isaac ROS 컨테이너에서 `rescue_robot_bringup`과 누락된
`leader_command_selector`를 빌드했다. RViz를 종료하고 VSLAM, nvblox, Nav2,
rosbag을 실행했다. 첫 구현은 Nav2 OccupancyGrid의 미관측 영역을 구분하지 못해 즉시
종료했다. 해당 topic의 40,000개 cell에는 unknown 값이 없고 free `0`이 36,318개였다.
그래서 nvblox `/nvblox_node/static_map_slice`의 `unknown_value=1000` 및 ESDF clearance
기반 탐지로 수정했다.

재실행할 때 selector를 `STOP`으로 둔 상태에서 frontier 노드가 여러 목표를 보내게 했다.
이 action들은 약 10초 후 abort됐고, 코드는 실패 위치도 visited로 기록했다. 그 뒤
`NAV2`로 전환해 제한 시간 동안 주행했고 사용자가 벽 접촉을 보고했다. `/leader/cmd_vel`
표본에서 선속도 `0.10 m/s`, 각속도 `-0.158 rad/s`가 확인됐다. selector를 `STOP`으로
바꾼 뒤 zero Twist를 확인하고 mapping/Nav2/bridge를 종료했다. 탐색은 성공하지 않았다.

기록 bag `data/vslam_mapping_20261003_170827/`에서 VSLAM odometry는 6.5 Hz, 최대 간격
1.857 s였고 wheel odometry와 경로 길이도 크게 달랐다 (VSLAM 8.302 m, wheel 2.745 m).
이 저하가 주행 판단에 영향을 줬을 가능성은 있지만 충돌 원인으로 확정할 수 없다.
Nav2 costmap update 지연, ESDF와 실제 장애물의 차이, planner/controller 동작도 이번
bag에 저장되지 않아 분리 분석할 수 없다. 벽 접촉은 obstacle avoidance가 충분하지 않았음을
보여주는 실패 결과로 기록한다. 원인과 정지 성능을 별도 확인하기 전까지 이 탐색 경로를
다시 실물 주행에 사용하지 않는다.

### Depth self-filter 최적화 및 정지 상태 확인

벽 접촉 후 3D 좌표를 매 frame 계산하던 gripper self-filter를 수정했다. 카메라 intrinsics,
고정 camera-to-base TF, ROI와 픽셀별 swept-volume 깊이 구간을 한 번 계산해 재사용하고,
frame마다 depth ROI 구간 비교만 수행한다. Isaac ROS 컨테이너에서 패키지 빌드가
성공했고, `VSLAM_HEADLESS=1 STM32_I2C_WRITE_ENABLED=0`으로 selector `STOP` 상태에서
센서 mapping stack을 실행해 확인했다. ROI는 `848x409` (frame의 85.2%)였다. 프로세스 CPU
표본은 이전 약 77%에서 약 33%로 감소했다. VSLAM odometry 관측 주기는 이전 5.7–15 Hz
(최대 간격 약 0.8 s)에서 약 20–30 Hz (최대 간격 약 0.13 s)로 개선됐다.

이는 정지 상태의 짧은 측정이며 통제된 benchmark나 주행 검증이 아니다. 현재 필터 출력은
frame 간격이 변동했고, 전체 시스템 CPU도 여전히 높았다. 최적화가 depth 제거의 기하학적
동등성이나 obstacle avoidance를 보증하지 않는다. 해당 세션은 bag 저장 후 종료했다.

### 17:57 실행의 직진 목표 실패와 제어 입력 보완

`data/vslam_mapping_20261003_175701`에서 RViz 없이 전방 0.8m 목표를 실행했다.
사용자는 우측/좌측 조향 후 벽에 거의 닿을 때 정지했다고 보고했다. 주행 중 명령은
최대 0.08m/s, 각속도 ±0.25rad/s였고, 목표 완료가 아닌 외부 감시 스크립트의 취소로
정지했다. 취소 시각은 epoch 1791018213.621, 최종 출력 zero는 약 1791018213.667,
selector의 STALE_NAV2는 그 뒤 1791018214.169였다. 장애물 정지 성공 사례가 아니다.

실제 주행 구간의 wheel 순변위는 약 1.08m, VSLAM 순변위는 약 0.53m였다. 마지막
약 5초 동안 wheel은 약 0.35m 이동했지만 VSLAM x/y는 거의 정지했다. VSLAM header와
odom->base_link TF는 계속 갱신됐으므로 수신 timeout만으로는 검출할 수 없다.
wheel twist 최대 전진 속도는 0.0875m/s인 반면 VSLAM twist는 전진 약 0.255m/s,
횡방향 약 0.197m/s까지 나왔다. 당시 Nav2의 현재 속도 입력도 VSLAM이었다.

보완 사항 (사용자의 wheel odometry 우선 지시 반영):

- `nvblox_vslam_realsense.launch.py`에서 wheel pose의 timestamp를 보존해
  `odom -> base_link` TF를 발행한다. VSLAM의 odom/map TF 발행은 끈다.
  nvblox, Nav2, frontier는 모두 wheel 기반 odom 좌표와 위치를 사용한다.
  현재 VSLAM은 비교 기록용이며 시각 위치 보정은 적용하지 않는다.
  wheel 적분 오차는 누적되므로 장거리 운용의 보정은 별도 검증이 필요하다.
- Nav2 `odom_topic`도 `/leader/odom/raw`로 변경해 현재 속도를 wheel에서 읽는다.
- VSLAM과 4cm/4mm 이동 차이가 난다는 이유로 차단하려던 로직은 사용자 지시에 따라
  제거했다. selector는 wheel 측정 자체의 유효성/0.5초 freshness만 확인한다.
  VSLAM 누락/정지/불일치는 주행 중단 조건이 아니다.
- DWB 속도 상한을 0.08m/s, 0.25rad/s로 저장하고 가속도 상한을 0.2m/s²,
  0.5rad/s²로 낮췄다. 경로 정렬 가중치는 그대로 둔다.
- 기존 BaseObstacle에 ObstacleFootprint를 추가해 집게까지 포함한 footprint 경계를
  검사한다. 중심 경로에 lethal 셀이 없다는 것만으로 차체 통과가 검증되지는 않는다.
- mapping launcher source 기본값은 STOP, 종료 시 bag 분석 전에 STOP을 요청한다.
  plan/local_plan, costmap, ESDF, 명령, selector/action 상태를 추가 기록한다.
  `NAV_DIAGNOSTIC_IMAGES=1`이면 IR stereo와 raw/filtered depth를 저장한다.
  영상 기록은 저장 대역폭/CPU 부하를 늘릴 수 있으므로 측정 조건에 표기한다.

이 단계에서는 실물 주행을 재개하지 않았다. 실물 직진/회피 성공은 별도 확인해야 한다.

### wheel odometry 기준 짧은 실물 주행 및 취소 검증 (19:28 실행)

`SELF_FILTER_ENABLED=1 VSLAM_HEADLESS=1 MAPPING_SOURCE_MODE=STOP`으로 mapping,
nvblox, Nav2와 계측 rosbag을 시작했다. 시작은 `STM32_I2C_WRITE_ENABLED=0`으로
센서와 정지 costmap을 확인했고, 주행 직전에 bridge write를 켰다. 속도 상한은
설정 파일의 선속도 `0.08 m/s`, 각속도 `0.25 rad/s`를 사용했다. 시험 사이마다 selector를
`STOP`으로 바꿨고, 각 정지 확인 후 종료 전에 bridge write를 다시 껐다.

`/compute_path_to_pose`는 `odom` 프레임에서 `x=0.5 m`, `x=0.75 m`, 그리고 제자리 회전
목표를 모두 성공적으로 계산했다. 이어진 실물 action 결과:

| 동작 | Action 결과 | wheel odometry 결과 |
| --- | --- | --- |
| 직진 목표 `x=0.5 m` | `SUCCEEDED` | 정지 확인 시 `x=0.330 m`, `y=0.001 m` (목표 허용 오차 `0.20 m`) |
| 직진 목표 `x=0.75 m` | `SUCCEEDED` | 정지 확인 시 `x=0.564 m`, `y=0.002 m`; 직전 정지점에서 전진 약 `0.234 m`, 횡방향 변화 약 `1.1 mm` |
| 같은 위치의 yaw 약 `+0.60 rad` 목표 | `SUCCEEDED` | yaw 약 `2.0°`에서 `23.0°`로 변함; 위치 변화 약 `5 mm` |
| 목표 `x=1.0, y=0.18 m` 취소 | `CANCELED` 응답 확인 | 취소 이후 정지 확인까지 약 `5.7 cm` 이동; selector `STOP` 뒤 `/leader/cmd_vel`과 wheel twist 모두 0 |

전체 bag은 647.2초, 221,833개 메시지다. wheel odometry는 46.0 Hz, 최대 표본 간격
385.8 ms, 경로 길이 `0.636 m`, 최종 변위 `0.622 m`, 누적 yaw `17.80°`였다. VSLAM
tracking status는 성공 100%였지만 odometry는 19.5 Hz, 최대 간격 930.6 ms였고 경로
길이는 `0.856 m`, 최종 변위는 `0.375 m`였다. 따라서 이 실행에서는 VSLAM 누적 경로가
wheel 경로보다 `0.221 m` 길고, 순변위도 크게 달랐다. Nav2 속도 입력과 TF 기준으로
사용한 wheel odometry에는 이 구간의 위치 점프가 보이지 않았지만, 최대 간격 385.8 ms는
selector freshness 제한 0.5초에 가깝다.

STM32 system state의 `fault_bits=2`가 실행 중 관찰됐다. 현재 firmware 정의상 UART 오류 또는
수신 overrun 표시이며, 동시에 I2C bridge CRC 오류는 0이었다. 원인을 분리하지 않았으므로
상태 비트가 해소됐다고 판단하지 않는다. 이번 실행은 Nav2 짧은 직진·회전 및 취소 동작을
확인했지만, 물체를 둔 장애물 정지와 반복 주행은 수행하지 않았다. 벽 접촉 이력이 있으므로
장애물 회피를 성공으로 간주하지 않는다.

산출물: `data/vslam_mapping_20261003_192804/analysis.md`,
`data/vslam_mapping_20261003_192804/metadata.yaml`,
`log/vslam_mapping_20261003_192804/`. 이번 bag은 이미지 토픽을 포함하지 않는다.

참고: [Nav2 Humble footprint critic 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp),
[Controller odometry velocity 입력](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/controller_server/).

### 추가 self-filter 여유 적용 후 제한 구역 탐색 재시험 (20:24 실행)

사용자 지시에 따라 필터 범위만 `x=0.08..0.43`, `y=±0.20`, `z=0.00..0.22 m`로 넓힌 뒤
새 mapping session에서 탐색을 반복했다. ROS parameter에서 이 값이 적용된 것을 확인했다.
정지 costmap은 중심 및 주변 5×5 셀이 모두 cost `0`이었고 최근접 lethal cell은 로봇 기준
약 `(+0.525,+0.475) m`였다. 이 측정은 self-filter 효과를 뒷받침하지만 gripper 기하 전체가
완전히 제거됐음을 증명하지는 않는다.

탐색 disk는 시작 위치 기준 전방 `2 m`, 반경 `2 m`였다. Nav2 속도 제한은 사용자 지정대로
`0.10 m/s`, `0.20 rad/s`를 runtime에 적용했다. 첫 목표 `(0.08,0.03)`은 허용 오차 내라 즉시
성공 처리됐고, `(0.33,-0.22)`는 약 4.6초 주행해 성공했다. 세 번째 `(0.38,0.28)`은 약 10초
뒤 `Failed to make progress`로 실패했다. rosbag의 `/nav2/cmd_vel`과 selector 출력에는 목표
주행 중 우회전 명령이 있었고 각속도는 `-0.20 rad/s`까지 내려갔다. 따라서 관찰한 우회전은
Nav2 출력 명령에도 있었다. 실패 후 wheel odom은 `(x=0.236,y=-0.069) m`, yaw 약 `-17.8°`를
보였다. 이 결과만으로 오른쪽 회전 원인(경로 형상, local costmap, 제어 지연 등)을 확정할 수
없다. Controller log에는 10 Hz control loop miss와 BT tick-rate warning도 반복 기록됐다.

실패 직후 selector를 `STOP`으로 바꿔 `/leader/cmd_vel=(0,0)`을 확인했고, 마지막 5초 wheel
odom jitter도 `0`이었다. 탐색은 목표 disk를 충분히 훑지 못했으므로 성공 판정이 아니다.
실행은 종료했고 bag은 정상 finalize됐다. 산출물은
`data/vslam_mapping_20261003_202448/analysis.md`, rosbag 및
`log/vslam_mapping_20261003_202448/`이다. bag에는 VSLAM status/odometry 표본이 없어 이번
실행의 wheel-VSLAM 비교는 불가능하다.

### 다음 재시험용 원인 진단 로그 보강

사용자 요청으로 재시험 전에 계측을 여러 지점에 추가했다. Frontier 노드는 입력 topic 누락과
staleness, costmap/ESDF/frontier 필터 단계별 후보 수, 선택 좌표·cost·clearance, 현재 pose에서
목표까지 거리, Nav2 feedback 및 각 목표 action의 상태/소요시간을 기록한다. Command selector는
주행 중 매초 mux 출력 범위와 선택 source/status, 입력 age를 요약한다. Mapping launcher는 run
설정 manifest와 Jetson `tegrastats`를 남기며 bag에 `/rosout`, `/diagnostics`,
`/parameter_events`를 추가했다. `scripts/run_frontier_exploration.sh`는 speed parameter 변경,
NAV2 선택, frontier 출력, 종료 시 STOP을 `frontier_exploration.log`로 남긴다. 본 주행에서
계측을 실행했고, 후속 분석은 아래와 같다.

### 계측 주행 분석 (20:42 run, 목표 실행 20:45)

첫 helper 실행은 host에 Nav2 의존성이 없어(`nav2_msgs` import 실패) action 전 종료됐다.
selector는 STOP으로 돌아왔고 주행은 없었다. helper를 수정해 frontier를 Isaac ROS 컨테이너에서
실행했고 같은 mapping bag에 본 주행을 기록했다. 시작 disk는 `(2.00,0.00)`, 반경 `2.00 m`였고
속도 상한 `0.10 m/s`, `0.20 rad/s`는 parameter readback으로 확인했다. 첫 목표는 거리 `0.079 m`
로 허용 오차 내 즉시 성공, 두 번째는 약 `0.395 m` 떨어진 목표까지 4.68초 주행해 성공했다.
두 번째 목표는 남동쪽 `(0.325,-0.225) m`였으므로 그 구간의 큰 음수 각속도(우회전)는 목표
방향에 맞는 heading 조정이었다. 실패한 세 번째 목표의 문제는 이 초기 우회전과 별개다.

세 번째 목표는 `(0.375,0.275) m`, 시작 pose 약 `(0.239,-0.069) m`였다. frontier 후보는
costmap cost `11`, ESDF clearance `0.354 m`로 선택됐다. clearance 임계치 `0.35 m`보다 겨우
`0.004 m` 여유다. 이 지점에서 controller의 `/local_plan`은 위치가 시작점 주변에 고정된 채
yaw만 약 `-13.7°..-17.3°`를 오갔다. `/nav2/cmd_vel.linear.x`는 action 종료까지 10초 동안
전부 `0`; 각속도도 처음 대부분 `-0.032 rad/s`, 이후 `-0.011 rad/s` 또는 `+0.011 rad/s`에
머물렀다. wheel odom pose는 약 `(0.239,-0.069)`에서 `(0.243,-0.070) m`로만 변했고 Nav2
feedback 거리도 `0.360 m`에서 거의 줄지 않았다. 따라서 이 실패는 회전 명령을 받아도 바퀴가
따르지 않은 문제가 아니다. Nav2 DWB가 사실상 정지/미세 제자리 회전 궤적만 선택해 10초 동안
요구된 `0.2 m` 진행량을 채우지 못했고, `SimpleProgressChecker`가 설정대로 abort했다. 이번
실행에서는 controller loop miss나 BT tick warning은 관찰되지 않았다.

왜 DWB가 전진 궤적을 선택하지 않았는지는 아직 확정할 수 없다. 목표 clearance가 footprint와
회전 여유에 비해 경계였고, local costmap/footprint critic이 전진 후보를 거부했을 가능성이
가장 높다. Explorer도 이 목표 도중 입력 `/global_costmap/costmap`이 4.16초 및 9.36초 stale라고
기록했다. 다만 bag에는 그 정확한 `OccupancyGrid` 입력이 없고 Nav2 raw global/local costmap은
계속 기록됐으므로 explorer topic staleness가 DWB 정지의 원인이라고 단정할 수는 없다. 다음
run부터 explorer가 실제 읽는 OccupancyGrid와 DWB evaluation을 직접 rosbag에 넣도록 목록을
보강했다.

Self-filter는 5초 표본마다 약 20–27 frame/s, 평균 5.2–7.3 ms 처리, frame drop `0`을 기록했다.
이 구간에서 filter 처리 실패나 controller loop miss가 보여 실패 원인으로 지목할 근거는 없다.
실패 후 action abort 시각은 `1791027937.4`; selector logger는 약 0.5초 뒤 `STALE_NAV2` 경고 후
종료됐다. 원인은 추가한 로그 코드에서 `info`와 `warning`을 동적으로 선택해 rclpy의
`Logger severity cannot be changed between calls` 예외를 낸 것이다. selector 종료는 목표 실패의
원인은 아니지만 안전 경로 계측 코드의 결함이다. `/leader/cmd_vel`에 zero를 반복 발행하고
mapping process group을 종료했다. selector 로그는 단일 severity를 쓰도록 고쳤다.

이 run의 bag/분석은 `data/vslam_mapping_20261003_204257/`, frontier 출력은
`log/vslam_mapping_20261003_204257/frontier_exploration.log`다. 정확한 DWB critic 점수와 explorer
input grid 변화는 다음 rosbag에서 수집하게 설정했다.

### OccupancyGrid·DWB 계측 재시험 (20:58 run)

앞선 계측 개선을 적용해 `/evaluation`과 explorer가 직접 구독하는 global OccupancyGrid를
기록했다. 속도 parameter readback은 `max_vel_x=0.10`, `max_speed_xy=0.10`,
`max_vel_theta=0.20 m/s, rad/s`였다. 첫 목표는 시작 pose 근처라 즉시 성공했고, 두 번째
목표 `(0.275,0.325) m`는 0.447 m 떨어져 있었으며 약 7초 주행해 성공했다. 세 번째 목표는
`(0.575,1.125) m`, 당시 robot pose는 `(0.250,0.180) m`였고 10.06초 뒤 실패했다.

이번 `/evaluation`에는 실패 action 구간의 DWB 100개 표본이 담겼다. 매 주기 best로 표시된
궤적은 전부 `vx=0`; 각속도는 `-0.20 rad/s`에서 점차 0에 가까워졌고 마지막에는 작은
양수로 바뀌었다. 실제 wheel odom도 약 10초 동안 `(0.250,0.180)` 부근에 머물렀으며 최종
feedback 거리는 약 `1.819 m`로 줄지 않았다. Controller의 `SimpleProgressChecker`가
`required_movement_radius=0.2 m`, `movement_time_allowance=10 s` 설정에 따라 실패 처리했다.
따라서 관찰된 좌우 흔들림은 Nav2가 거의 제자리 회전 명령만 내는 것과 맞는다.

계획도 직진하지 않았다. global plan은 로봇 `(0.25,0.20)`에서 목표 `(0.575,1.125)`로 가면서
초반에 x를 감소시켜 로봇 뒤쪽으로 돌아간 뒤 목표에 접근하는 73점 경로였다. 이 경로 굴곡이
DWB의 회전 우선 선택과 이어졌을 가능성이 있다. 기록된 best 궤적의 점수에서
`GoalAlign`·`GoalDist`가 대부분을 차지했고 `ObstacleFootprint` 기여는 약 `0.041`이었다.
이번 점수만 보면 footprint 충돌 critic이 best 궤적을 직접 강하게 벌점 준 정황은 없다.
그러나 planner가 왜 뒤쪽 우회 경로를 만들었는지까지는 costmap 좌표와 장애물 셀을 더
대조해야 확정할 수 있다.

Explorer 입력 `/global_costmap/costmap`은 bag에서 5회만 기록됐고 updates는 97회 기록됐다.
따라서 explorer의 staleness 경고는 실제 업데이트 topic 활용 여부를 별도로 고쳐야 한다.
이 경고는 탐색 후보 선정에는 영향을 줄 수 있지만, Nav2가 받아 수행한 현재 목표의 local
control 실패를 직접 일으켰다고 단정할 근거는 없다. Self-filter는 평균 약 5.3–6.7 ms로
계속 동작했고 frame drop은 대부분 0이었다.

시험 종료 때 frontier runner가 STOP을 요청하고 mapping launcher가 bag을 finalize했다.
wheel odometry는 46.8 Hz, 경로 길이 `0.363 m`, 최종 변위 `0.333 m`; 종료 후 5초 jitter는
`0`이었다. bag은 `data/vslam_mapping_20261003_205844/`, Nav2/frontier 출력은
`log/vslam_mapping_20261003_205844/`에 있다. DWB의 제어 파라미터는 config에 이미 존재하며
이번 runtime 변경은 속도 상한뿐이었다. 다음 튜닝은 제한 속도만 바꾸기보다 global plan의
우회 원인을 costmap에서 찾고, `PathAlign`/`GoalAlign`, 가속 한계, footprint·inflation과
실제 구동 응답을 함께 확인해야 한다.

### 다음 원인 분석용 계측 보강

사용자 요청에 따라 주행 명령이 생성되는 단계부터 모터 응답까지 빠질 수 있는 관측값을
늘렸다. Rosbag은 기존 DWB `/evaluation`, 전역·지역 costmap 및 update, 계획/action/feedback,
Nav2 명령, selector 최종 명령, wheel odometry와 IMU 외에 teleop/approach 입력, gripper 명령,
STM32 `system_state`, 수신 frame/poll/empty-poll/CRC/sequence-drop 누적치를 기록한다.
따라서 Nav2 입력과 selector 출력 차이, wheel 속도 응답, 구동기 fault/통신 품질을 같은 bag에서
비교할 수 있다. 카메라 raw/filtered 이미지는 기존대로 `NAV_DIAGNOSTIC_IMAGES=1`에서 선택
기록한다.

Frontier 로그는 5초마다 각 입력의 수신 횟수와 수신 age, message stamp, costmap geometry 및
unknown/lethal 셀 수, wheel pose/twist를 함께 남기고 2초 이상 stale 입력을 경고한다. 실행 helper는
주행 시작 전에 Nav2·costmap·nvblox·selector·bridge의 실제 parameter dump와 핵심 topic QoS/
publisher 연결 상태를 run별 snapshot 파일에 저장한다. Global costmap에는 주기적 full map 발행을
켜 explorer가 갱신 update만 받고 OccupancyGrid snapshot은 오래됐다고 판단하던 관측 문제도
보완했다. 장비 성능은 기존 `tegrastats`, ROS diagnostics와 노드 로그로 확인한다.

계측 변경은 이후 `211214` 및 후속 run에서 사용했다. 아래에는 run별 목표 결과, controller 점수,
입력 freshness 및 wheel/VSLAM 응답을 연결해 원인을 갱신한다. 각 bag과 로그 위치는 해당 실행
분석 절에 적었다.

### 키보드 주행 대비 DWB 실패 확인 (21:12 실행)

키보드 teleop 기록 `data/vslam_mapping_20260828_160008/`는 명령 선속도 최대
`0.08 m/s`, 각속도 최대 `0.25 rad/s`에서 wheel 경로 `1.105 m`, 최종 변위 `1.102 m`,
yaw 변화 `-2.67°`였다. 이번 NAV2 run의 실패 구간은 조건이 달랐으므로 둘만으로 controller
성능을 완전히 비교할 수는 없지만, STM32와 wheel odometry 구동 경로가 수동 저속 주행에서
직선 이동을 수행한 선례는 확인된다.

새 계측 run은 정면 2 m 중심·반경 2 m, 최고 속도 `0.10 m/s`/`0.20 rad/s`로 시작했다.
첫 frontier는 시작점과 가까워 즉시 처리됐고 두 번째 `(0.325,-0.225)`는 4.57초 만에
도착했다. 세 번째 `(0.375,0.275)`는 시작 pose `(0.239,-0.064)`에서 직접 거리 `0.366 m`,
ESDF clearance `0.424 m`였다. Planner의 14점 경로는 시작점에서 목표까지 곧게 올라갔다.
실패 때 costmap/ESDF/wheel odom 입력 나이는 약 `0.81/0.18/0.01 s`였고, 전체 costmap도
117회 기록돼 이전 run의 snapshot staleness는 재현되지 않았다.

DWB `/evaluation` 100회 모두 `vx=0`을 best로 선택했다. 선속도 `0.022 m/s` 후보의 점수는
best보다 약 `0.4` 높았고 이 표본에서 obstacle/footprint critic 점수는 0이었다. 즉 이 구간은
장애물이 전진 궤적을 거부한 흔적보다 DWB가 격자 costmap의 경로 점수에서 정지 회전을 근소하게
선호한 결과와 맞는다. `/nav2/cmd_vel`은 `vx=0`과 `wz=-0.053..+0.011 rad/s`를 냈고 selector
출력도 이를 그대로 전달했다. wheel yaw는 이 실패 구간에서 추가로 `-4.78°` 우회전했으며,
pose 변위는 약 `4 mm`였다. 사용자가 본 우측 방향은 bag의 최종 yaw `-18.65°`와 일치한다.
종료 후 selector status는 STOP, 최종 `/leader/cmd_vel`은 zero였고 마지막 5초 odom jitter는
0이었다. 주행 node는 종료했다.

후속 `212410` run은 helper의 runtime parameter setter에서 controller 서비스 응답이 멈춰 목표를
내기 전에 중단됐고, 실제 주행은 없었다. 이를 피하도록 속도 설정을 시작 YAML에서 읽도록 바꿨다.
`212911` run에서는 설정 readback이 성공했지만 `min_vel_x=0.04` 때문에 첫 0.079 m 목표에서
DWB가 `No valid trajectories out of 210!`으로 abort했다. odometry 이동은 0 m였고 selector STOP,
정지 bag tail도 안정적이었다. 즉 전진 최저속도 제한은 RotateToGoal의 정지 궤적과 충돌해
가까운 목표에서 사용할 수 없다. `min_vel_x=0.0`으로 되돌리고, 정지 궤적을 없애려 하지 않고
경로 진행을 더 반영하도록 `PathDist.scale=48`, `GoalDist.scale=36`으로 높여 다음 run에서 critic
선택과 실제 직진 응답을 다시 본다. 최고 속도는 `0.10 m/s`, 각속도는 `0.20 rad/s`로 유지한다.
이번 STM32 `fault_bits=2`는 firmware에서 UART 오류/RX overrun 표시지만,
I2C CRC 누적치는 0이고 selector command age도 약 9 ms여서 현재 주행 실패의 직접 원인으로
확정할 수 없다. 통신 상태는 후속 시험에서도 추적한다.

### DWB 정지 선택 원인과 다음 실험 (21:33 실행)

`213358` run은 첫 근접 frontier를 0.20초에 성공 처리했고, 다음 목표 `(0.325,-0.225)`까지
0.395 m 주행해 4.60초 만에 도착했다. 그 다음 목표 `(0.375,0.275)`는 10.06초 동안 실패했고,
wheel odom pose는 약 `(0.230,-0.067)`에서 `(0.233,-0.068)`로만 바뀌었다. costmap·ESDF·wheel
odom 입력 수신은 계속됐으며 receipt age는 대체로 0.5초 미만이었다. 실패 구간 rosbag의
`/nav2/cmd_vel` 109개는 전부 `vx=0`; `wz`는 대부분 `-0.011..+0.011 rad/s`에 머물렀다.
controller log는 `Failed to make progress`를 기록했다. 이는 선택된 제어가 거의 정지 회전이었다는
점에서 앞선 실패와 같다. `/evaluation`은 101개 기록돼 실패 구간 critic 점수도 비교할 수 있었다.

이 run의 실패 구간에서는 정지 best sample이 40회, `vx=0.011`인 sample이 21회 선택됐다.
대표 표본에서 정지와 전진 후보의 `GoalDist` raw score는 모두 10, `PathDist`는 모두 0으로
같았고 총점도 16.6으로 동점이었다. 따라서 DWB가 동점에서 0 속도를 선택해 다음 cycle에도
같은 격자 점수에 머무는 것이 정지 반복의 직접 설명과 맞는다. `/evaluation`에서 obstacle 및
footprint 점수는 0이었다.

`214016` run에서는 점수 문제를 가속도 창으로 풀어보려고 `acc_lim_x=0.5 m/s²`로 올렸다.
첫 근접 goal은 성공했지만 두 번째 목표에 0.064 m만 이동한 뒤 멈춰 10.18초에 abort했다.
wheel yaw 변화는 0.66°였고 visual SLAM tracking success rate는 100%였지만 wheel path는
`0.064 m`, VSLAM path는 `0.045 m`라 짧은 구간만으로 드리프트를 평가하기 어렵다. VSLAM 최대
message gap도 1.43초였다. `/evaluation`에서는 정지 궤적과 낮은 전진 궤적이 같은 격자 점수를
내는 표본이 확인돼 가속도 상한만으로는 선택 문제를 해결하지 못했다. 다음 run은 가속도 기본값으로
복구했다.

`214509` run은 `min_vel_x=0.02 m/s`에서 첫 0.079 m 목표가 `No valid trajectories out of 210!`으로
abort해 정지 샘플은 필수임을 재확인했다.

### Goal heading과 DWB 경로 추종 (21:51 실행)

`215144` run에서 첫 두 frontier는 성공했으나 세 번째 북동쪽 목표는 x방향으로 약 0.10 m 이동한 뒤
y방향 진행이 멈춰 progress checker에 abort됐다. Bag의 14 pose global plan은 좌표가
`(0.25,-0.05)`에서 `(0.375,0.275)`로 북동쪽으로 바뀌었지만, 모든 pose quaternion은 yaw `0°`였다.
Frontier explorer가 goal orientation을 identity로 보내고 NavFn 기본값이 이를 유지한 결과다.
DWB `/evaluation`에서는 정지/동쪽 이동 궤적을 선택했고 작은 우회전 명령이 기록됐다. 바뀐 critic
가중치는 이 목표 진행을 충분히 교정하지 못했다.

가중치를 기본값으로 되돌리고 NavFn `use_final_approach_orientation=true`를 켰다.
Nav2 Humble NavFn은 이 옵션에서 마지막 두 path pose의 차이로 endpoint 방향을 계산한다
([Humble NavFn API](https://api.nav2.org/nav2-humble/html/navfn__planner_8cpp_source.html)).
하지만 `215754` bag에서 두 번째 목표 `(0.325,-0.225)`의 path 끝부분은 `(.300,-.150)`,
`(.350,-.200)`, `(.325,-.225)` 순서라 endpoint yaw가 `-135°`가 됐다. 로봇과 목표 사이의
직접 진행 방향은 `-34.7°`였다. 로봇은 실제 yaw `-32°` 부근, 목표 거리 약 `0.20 m`까지
접근했지만 endpoint의 반대 방향을 요구받아 회전 중 멈춘 뒤 14.26초에 abort됐다. 입력 나이는
비교적 정상(costmap 최대 0.86 s, ESDF 0.14 s, wheel odom 0.03 s)이었다. 누적 wheel 경로는
`0.348 m`, 최종 변위 `0.315 m`, yaw `-32.39°`; Nav2 goal 1은 3.09초에 성공, goal 2는 실패했다.
이 결과에서는 NavFn의 마지막 grid segment 방향이 실제 접근 방향을 대표하지 못했다.

따라서 `use_final_approach_orientation`은 기본 false로 복구하고, frontier explorer가 goal을 보낼 때
현재 wheel pose에서 target까지의 bearing을 goal quaternion으로 설정하도록 반영했다. 이 방향은
위 사례에서 `-34.7°`로 기록된 로봇 접근 방향과 일치한다. 로그에도 goal yaw를 출력한다.
이 마지막 수정은 물리 주행으로 검증하지 않았다. 사용자가 지정한 마지막 test는 `215754`였고,
이번 수정은 그 bag 분석에서 확인한 방향 불일치를 바로잡기 위해 반영한 후속 변경이다. 다음 실행은
goal yaw와 `/plan` 끝 orientation을 확인한 뒤 전체 frontier 탐색을 반복해야 한다.

### 2026-10-03 22:21 실차 직진 목표 재시험

Headless mapping stack을 시작하고 selector를 `STOP`으로 두어 준비 상태를 확인한 다음, 전방 costmap이
free인 구간에서 `odom x=0.40 m`, 이어서 `x=0.70 m` 목표를 보냈다. 두 action은 `SUCCEEDED`였지만
wheel odometry는 각각 `x=0.215 m`, `x=0.515 m`에서 멈췄다. 두 경우 모두 목표까지 약 `0.185 m`가
남았다. 기본 `general_goal_checker.xy_goal_tolerance=0.20 m`와 일치하므로, 이전에 관찰된 짧은
이동 후 정지는 조기 goal success로 설명된다.

원인 확인을 위해 controller와 goal checker의 허용 오차를 실행 중에만 `0.05 m`로 낮추고 같은
`x=0.70 m` 목표를 재시도했다. 로봇은 `x=0.519 m`까지 약 `4 mm`만 더 움직인 뒤 10초 progress
검사에서 abort됐다. 실패 구간 100개의 `/evaluation` 표본은 전부 `vx=0`인 trajectory를 선택했고,
모든 양의 `vx` candidate는 `total=-1`로 무효였다. 대표 sample에서 무효 사유는
`RotateToGoal` critic이었다. DWB Humble 구현은 `FollowPath.xy_goal_tolerance`를 초기화 때
critic 내부에 읽어 보관하므로, 실행 중 parameter를 바꿔도 기존 critic에는 반영되지 않는다.
따라서 이 재시험은 시작 시점의 `0.20 m` 내부 창이 남은 상태에서 goal checker만 `0.05 m`로
좁힌 결과로 해석한다. controller를 재시작할 때 goal checker와 DWB tolerance가 함께 적용되도록
source YAML의 두 값을 `0.05 m`로 맞췄다. 새 Nav2에서의 물리 검증은 아래 22:42 시험에서
완료했다.

시험 후 selector는 `STOP`, `/leader/cmd_vel`은 zero였고 마지막 5초 wheel odometry jitter는 0이었다.
rosbag은 `data/vslam_mapping_20261003_222105/`, 실행 로그는
`log/vslam_mapping_20261003_222105/`에 저장했다. bag 집계에서 wheel 경로 `0.522 m`, 순변위
`0.519 m`, VSLAM 경로 `0.876 m`가 기록됐다. 좌우 I2C CRC 오류는 0이었으며 STM32 sequence drop
누적치는 실행 중 증가해 구동 통신 계측을 계속 확인해야 한다.

새 시작 시 설정 반영과 목표 허용 오차 안쪽 도달은 아래 22:42 시험에서 확인했다. 다음 검증은
반복 주행과 회전·장애물 정지 동작이다.

### 2026-10-03 22:42 goal tolerance 정합성 재검증

Nav2 bringup을 `install_docker`에 다시 빌드하고 새로운 mapping stack에서 시작했다. 시작 wheel pose는
`(-0.00008, -0.00000) m`였고, controller parameter readback은 goal checker와 `FollowPath` 양쪽
모두 `0.05 m`였다. 전방 costmap의 `0.15..0.45 m` 구간은 측정한 차체 폭 전체에서 cost `0`이었다.

`odom x=0.40 m` 직진 목표는 `SUCCEEDED`였고 정지 wheel pose는 `(0.3525, 0.0022) m`, 목표와의
거리는 `0.0476 m`였다. 이전 `0.20 m` tolerance의 약 `0.185 m` 오차 정지보다 목표에 가까워졌고,
실제 시작부터 약 `0.353 m` 이동했다. selector는 `STOP`, 최종 `/leader/cmd_vel`은 zero였으며 종료
후 5초 wheel jitter는 0이었다. 이 한 번의 짧은 주행은 새 설정이 초기화될 때 적용되고 `0.05 m`
내 goal 도착이 가능함을 확인한다. 반복성과 회전·장애물 정지는 아직 검증하지 않았다.

실행 bag은 `data/vslam_mapping_20261003_224243/`, 로그는
`log/vslam_mapping_20261003_224243/`에 저장했다. wheel odometry는 46.7 Hz, 최대 sample gap
283.4 ms, path/net displacement `0.353/0.353 m`; VSLAM path/net displacement는
`0.302/0.189 m`였다. BNO055 fault bit와 e-stop은 각각 clear/0이었지만 firmware `fault_bits=2`와
STM32 sequence drop 증가는 남았다. I2C CRC error는 0이었다.

IMU raw stream은 약 103.2 Hz였고 정지 구간 gyro-z 평균/표준편차는 `-0.00069/0.00143 rad/s`로
관측됐다. 다만 최신 bag에서도 `fault_bits=2`는 UART error/RX overrun을 뜻하고 sequence drop이
`104→137`로 증가했다. 이는 BNO055 fault bit(0)와는 별개다. wheel odometry에는 불연속이 보이지
않았지만 bag에 raw `WHEEL_STATE`의 `encoder_status`가 기록되지 않아 엔코더 자체의 상태까지
정상이라고 단정할 수 없다.

### 2026-10-03 22:52 직진 목표 반복 시험

새 mapping stack은 wheel odom `(-0.00012, 0.00000) m`에서 시작했다. goal checker/DWB tolerance는
각각 `0.05 m`, controller 상한은 `0.10 m/s`와 `0.20 rad/s`로 확인했다. 목표 중앙선은 costmap에서
치명 장애물 없이 free였고 정지 시 e-stop은 `0`이었다. bridge는 시험 중 UART sequence gap을
보고했고 system `fault_bits=2`도 남아 통신 상태는 깨끗하지 않았다.

`odom (0.40, 0.00) m` 목표를 보냈다. 로봇은 wheel odom `(0.35350, -0.04523) m`, yaw `-15.81°`까지
움직인 뒤 약 12.5초 시점에 controller의 `Failed to make progress`로 abort됐다. 목표까지 남은
거리는 `0.0648 m`로 `0.05 m` 허용 범위 밖이었다. 따라서 직전 한 번의 성공은 반복 재현되지
않았다. launcher cleanup이 selector를 `STOP`으로 바꾸고 주행 stack을 종료했다.

bag `data/vslam_mapping_20261003_225235/`의 wheel odom은 46.9 Hz, 최대 간격 169.8 ms, 경로/순변위
`0.363/0.356 m`였다. IMU는 103.3 Hz, 최대 간격 210 ms였고 정지 구간 gyro-z 평균/표준편차는
`-0.00059/0.00138 rad/s`; 마지막 5초 odom jitter는 0이었다. VSLAM tracking 표본은 전부 success였지만
rate는 18.0 Hz, 최대 간격은 807 ms, 처리 최대시간은 약 499 ms였다. VSLAM과 wheel 누적 yaw 차이는
`0.44°`였으나 path는 각각 `0.668 m`와 `0.363 m`로 차이가 컸다. raw encoder 상태는 bag에 없어
확인할 수 없다.

확인된 abort 로그 원인은 progress checker다. 기록만으로 측면 편향, controller 후보 평가, UART
sequence gap 중 어느 항목이 직접 원인인지는 분리되지 않았다. 로그와 bag은 각각
`log/vslam_mapping_20261003_225235/`, `data/vslam_mapping_20261003_225235/`에 보존했다.
