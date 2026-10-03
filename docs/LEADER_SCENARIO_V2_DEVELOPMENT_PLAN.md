# 리더 로봇 탐색·구호품 전달 시나리오 개발 계획

## 목적

리더 로봇이 3D 지도를 만들며 자율 탐색하고, 생존자를 발견하면 위치를 기록한 뒤
시작 위치로 복귀해 구급함을 파지하고, 생존자에게 전달하는 시나리오를 단계별로
구현·검증한다. 이 문서는 목표 시나리오와 진행 순서를 관리한다. 완료 판정은 실제
시험 결과를 근거로 갱신하며, 코드가 있다는 이유만으로 실물 기능 완료로 표시하지 않는다.

## 목표 시나리오

```text
시작 위치 설정
 → VSLAM + nvblox로 3D 매핑하며 자율 탐색
 → 생존자 감지 및 유효한 map 위치/Registry ID 기록
 → 탐색 정지, 생존자 목표와 map session 보존
 → 시작 위치(구급함 위치)로 복귀
 → AprilTag 정렬 및 구급함 파지 확인
 → 기록한 생존자 위치의 안전한 전달 지점으로 이동
 → 도착 확인 및 전달 완료
```

현재 문서상 시작 위치가 구급함을 파지하는 위치라고 가정한다. 실제 배치가 다르면
`시작 위치`, `구급함 위치`, `탐색 복귀 위치`를 별도 waypoint로 정의한다. 사람의 검출
좌표를 그대로 Nav2 목표로 쓰지 않고, 안전 거리와 로봇 footprint를 고려한 접근 목표를
계산한다.

## 현재 기준선 (2026-10-03)

| 기능 | 상태 | 근거 및 남은 조건 |
| --- | --- | --- |
| VSLAM 위치 추정 및 map/odom TF | 부분 완료(범위 내 검증) | D435 기반 mapping 중 TF와 Survivor map 변환을 확인. 이동 경로 전반의 장시간 위치 안정성과 복귀 오차는 별도 검증 필요 |
| nvblox 3D map/ESDF | 부분 완료(범위 내 검증) | mesh/ESDF와 RViz 표시를 확인. 자율 탐색 중 map 품질·장애물 반영을 검증해야 함 |
| Survivor 감지 및 map 위치 | 완료(범위 내) | camera/map positions와 persistent Registry를 확인. 임무 목표로 쓸 track 선택·유효성 정책 연결은 남음 |
| Nav2 경로 생성 | 완료(범위 내) | Nav2 활성화와 `ComputePathToPose`의 `odom` 경로 확인 |
| Nav2 실물 목표 주행 | 부분 완료 | 2026-10-03 0.10 m/s 속도 상한에서 짧은 `NavigateToPose` 목표 2개가 성공했다. Active 명령 구간 VSLAM 전진은 약 0.71 m. 반복 경로, 목표 도착 오차, 장애물 정지와 localization 복귀 정확도는 미검증 |
| `/nav2/cmd_vel` → 모터 구동 | 부분 완료 | Selector `NAV2`를 거쳐 STM32 bridge까지 연결하고 실물 주행을 확인했다. 좌우 바퀴별 telemetry는 기록하지 않아 모터 간 편차는 판별 불가; E-stop과 정지 성능도 미검증 |
| 탐색 중 생존자 발견 후 복귀 | 미구현 | 탐색 상태, 중단·재개, 시작 pose 복귀를 조정하는 mission 흐름 필요 |
| 구급함 파지 후 생존자 전달 | 부분 구현/통합 미완료 | AprilTag 접근과 gripper sequence는 존재하나 본 시나리오의 복귀·파지·전달 end-to-end는 미검증 |

상세 Nav2 현장 결과는
[`NVBLOX_NAV2_RVIZ.md`](../src/leader/rescue_robot_bringup/docs/NVBLOX_NAV2_RVIZ.md),
VSLAM 준비 절차는 [`VISUAL_SLAM_SETUP.md`](VISUAL_SLAM_SETUP.md), 기존 단독
파지·전달 계획은 [`LEADER_SCENARIO_V1_DEVELOPMENT_PLAN.md`](LEADER_SCENARIO_V1_DEVELOPMENT_PLAN.md)를
참고한다. 이 문서는 기존 v1의 operator가 물품 앞에 수동 이동하는 데모와 달리,
탐색 자율주행 및 발견 후 시작 위치 복귀를 목표로 한다.

## 단계별 진행 순서

| 단계 | 목표 | 완료 조건 |
| --- | --- | --- |
| 0. 안전·기준선 고정 | 현 장비/브랜치, E-stop, bridge watchdog, command source와 실제 모터 경로 확인 | 시험 구성과 정지 수단 기록, 단일 command owner 확인 |
| 1. Nav2 실물 주행 | `/nav2/cmd_vel`로 짧은 목표와 반복 목표를 주행 | 0.05 m/s에서는 progress checker abort, 0.10 m/s 상한에서는 짧은 목표 2개 성공. 반복성, 도착 오차, straight-line 편향, 장애물 정지 및 cancel/failure 정지 검증이 남음 |
| 2. 매핑 중 주행 안정성 | 저속 이동하며 VSLAM·nvblox 동시 실행 | 경로 중 tracking loss 없음, 지도와 TF 지속, 시작 pose 대비 복귀 오차 측정 |
| 3. 시작 pose와 복귀 | mission 시작 시 pose를 저장하고 Nav2로 돌아오기 | 같은 map session에서 복귀 성공, 목표 pose 허용 오차와 실패 정지 확인 |
| 4. 탐색과 생존자 목표 보존 | 탐색 중 valid Registry track을 선택하고 임무를 중지/전환 | track ID, 원본 시각, map pose, map session을 기록하고 stale/LOST track 거부 |
| 5. 구급함 접근·파지 | 복귀 뒤 AprilTag 접근과 gripper close/lift 수행 | `ALIGNED → CLOSE → LIFT` 순서와 파지 성공 확인을 정의하고 반복 시험 |
| 6. 생존자 전달 목표 주행 | 파지 완료를 확인한 뒤 안전 offset 목표로 이동 | NAV2 전환, 도착 판정, cancel/failure 시 정지 확인 |
| 7. 임무 조정 | 위 기능을 상태도와 fault 처리로 연결 | 허용 전이만 실행, 각 단계 timeout/cancel/fault 시 안전 정지 및 재시작 가능 |
| 8. 통합 반복 시험 | 실제 전체 시나리오와 중단·복구 시험 | 반복 성공 횟수와 실패 복구 결과를 기록한 뒤 완료 판정 |

## 지금 진행할 첫 작업: 단계 1

우선 mapping/exploration 자동화보다 Nav2 실물 주행의 반복성과 방향 정확도를 확인한다.
2026-10-03 bag에서 직진 목표 중 `/nav2/cmd_vel.angular.z`가 `-0.20..+0.16 rad/s`로
변했으므로 Nav2 heading 보정이 좌우 움직임에 기여했다. 반면 바퀴별 encoder/속도는 기록되지
않아 모터 편차는 아직 분리할 수 없다. 다음은 각속도 0의 짧은 저속 직진으로 구동계 편향을
확인하고, Nav2가 각속도를 보정한 구간과 비교한다. 그 다음 반복 직진·회전·장애물 정지와
도착 오차를 기록한다. E-stop과 bridge watchdog 확인도 독립된 실기 검증으로 남긴다.

상세 측정값과 rosbag 위치는 [2026-10-03 실물 진행 기록](LEADER_NAVIGATION_PROGRESS_2026-10-03.md)에 있다.

## 완료 상태 갱신 규칙

- 각 단계는 구현, 시뮬레이션/자동 시험, 실물 시험을 구분해 기록한다.
- VSLAM의 순간 TF 출력만으로 장거리 위치 안정성을 완료 처리하지 않는다.
- 실제 구동, E-stop, watchdog 결과 없이 실물 주행 완료로 표시하지 않는다.
- 생존자 목표는 Registry track의 유효성, map frame/session 일치, 안전 접근 offset을 확인한다.
- 시험하지 않은 gripper 파지 성공은 actuator 명령 완료와 구분한다.
- 단계가 끝날 때 이 표, `STATUS_AND_ROADMAP.md`, 관련 세부 검증 기록을 함께 갱신한다.
