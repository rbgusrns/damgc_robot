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
x: 0.12..0.38 m
y: -0.15..+0.15 m
z: 0.00..0.16 m
```

이는 흔들리거나 움직이는 그리퍼를 넉넉히 덮도록 의도적으로 크게 잡은 범위다. 따라서 그
공간에 실제 장애물이 있으면 함께 지워질 수 있다. TF/처리에 실패한 frame은 raw depth로
통과시키지 않고 drop해 미필터 데이터가 nvblox static map에 누적되지 않게 한다.
`SELF_FILTER_ENABLED=0`은 raw depth A/B용이다.

Nav2 global/local costmap은 원형 `robot_radius` 대신 차체와 그리퍼 바깥을 근사하는 convex
polygon 및 1 cm padding을 사용한다. footprint는 경로 충돌 계산용이며, nvblox에 쌓인 자기
점유 데이터를 제거하지 않으므로 depth self-filter를 별도로 유지한다.

확장 swept-volume 적용 후 정지 상태 costmap 10개 표본에서 중심 비용은 모두 `0`이었고 가장
가까운 lethal cell은 로봇 기준 `(+0.575, +0.475) m`였다. 이전 좁은 필터 시험보다 그리퍼로
의심했던 가까운 lethal cell이 멀어졌다. 이는 정지 map에서의 결과이며 실제 장애물 누락이나
주행 중 흔들림까지 검증한 것은 아니다.

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
