# 1차 진행상황 한눈에 보기

> 이 폴더는 첫 번째 구현·시험 묶음의 **과거 실행 근거**다. 프로젝트 전체 일정과
> 완료 여부는 [개발 계획서](../../Plan.md)와
> [개발 현황 및 로드맵](../../STATUS_AND_ROADMAP.md)을 우선한다.
> 일부 AprilTag 작업은 계획의 2~4주차 항목을 선행 구현한 것이며, 1주차의 모든
> 종료 조건을 달성했다는 뜻은 아니다.

## 현재 상태 한 줄 요약

**Leader와 Follower 모두 AprilTag base-link 상태에서 최종 software velocity topic까지 구현·자동시험했으며, 실제 로봇 이동·STM32·그리퍼 제어는 연결하지 않았다.**

## 전체 기능 흐름

```text
로봇 형상·센서 위치 정의
        ↓
카메라 영상 및 깊이값 발행
        ↓
AprilTag 검출 및 상대 위치 TF 계산
        ↓
거리·좌우·각도 오차 계산
        ├→ 접근 상태 판정
        └→ TF2 기반 Leader base_link pose·metric 계산
                ↓
        base alignment state
                ↓
        approach controller → cmd_vel_raw
                ↓
        Leader: velocity guard → /leader/cmd_vel
        Follower: command selector
                  ├─ AprilTag approach
                  └─ cooperation /follower/cmd_vel
                         ↓
                  velocity guard → /follower/safe_cmd_vel
                X
        STM32·motor 미연결
```

## 어디까지 완료됐나

| 단계 | 상태 | 결과 |
|---|---|---|
| 로봇 URDF·TF 구성 | 완료 | 실제 치수를 반영하고 RViz2에서 검증 |
| D435 컬러·뎁스 토픽 | 완료 | 컬러 및 뎁스 영상 발행 확인 |
| D435 거리 CSV 저장 | 완료 | 중앙 20×20 픽셀의 중앙값을 1초마다 저장 |
| USB 카메라 AprilTag | 완료 | ID 0 검출 및 카메라 기준 TF 출력 |
| D435 RGB AprilTag | 완료 | ID 0 검출 및 카메라 기준 TF 출력 |
| 접근 상태 판정 노드 | 구현·자동시험 완료 | 9개 상태 발행, camera-frame 단위 테스트 46개 통과 |
| 실제 태그 이동 시험 | 부분 완료 | Leader 완료; Follower CENTER/LEFT/FAR 완료, RIGHT/TARGET/HIDDEN `NOT VERIFIED` |
| 로봇 차체 기준 좌표 변환 | 구현·자동시험 완료 | Leader/Follower TF2 exact-stamp 변환; Follower LEFT 실측 부호 확인 |
| ROS 2 velocity command pipeline | software 완료 | Leader `/leader/cmd_vel`; Follower selector·guard → `/follower/safe_cmd_vel` |
| STM32·실제 motor 제어 | 미완료 | software final topic과 hardware를 아직 연결하지 않음 |
| 그리퍼 제어 | 미완료 | 안전 조건과 파지 거리 확정 필요 |

## 기능별 문서

### 1. 로봇 모델링

- [URDF·TF·RViz 작업 정리](01_로봇_모델링/URDF_TF_RViz_작업_정리.md)
  - 로봇 치수, `base_footprint`, `base_link`, 카메라·IMU 위치
  - RViz2 표시 및 `tf2_echo` 검증

### 2. D435 깊이카메라

- [D435 토픽 발행 결과](02_D435_깊이카메라/01_토픽_발행_결과.md)
  - RealSense 실행, 컬러·뎁스 토픽과 발행 주기 확인
- [D435 거리값 CSV 저장 결과](02_D435_깊이카메라/02_거리값_CSV_저장_결과.md)
  - 뎁스 영상 중앙 영역에서 거리 계산
  - `/home/maze/depth_distance.csv`에 1초 간격 저장

### 3. AprilTag 인식

- [USB 카메라 AprilTag 실행](03_AprilTag_인식/01_USB카메라_AprilTag_실행.md)
  - follower USB 카메라 보정 및 AprilTag ID 0 검출
- [D435 RGB AprilTag 실행](03_AprilTag_인식/02_D435_RGB_AprilTag_실행.md)
  - leader D435의 RGB 영상으로 AprilTag ID 0 검출
  - CameraInfo QoS bridge와 재실행 절차

### 4. 접근 상태 판정

- [AprilTag 접근 상태 노드 가이드](04_접근_상태판정/AprilTag_접근_상태노드_가이드.md)
  - 상대 위치의 거리·좌우·각도 계산
  - TF 유실, 오래된 데이터, 중앙값 필터 처리
  - 다중 태그 선택 및 9개 접근 상태 발행
  - 실행, 시험, 파라미터와 남은 안전 과제

### 5. Leader base_link Pose와 metric

- [base_link AprilTag Pose·Metric 구현 및 재현 검증 가이드](../../../src/leader/rescue_robot_apriltag/docs/LEADER_BASE_LINK_POSE_METRICS_VALIDATION_GUIDE.md)
  - filtered camera pose의 원본 timestamp를 사용하는 TF2 변환
  - base forward/lateral/bearing interface와 유효성·유실 처리
  - D435 ID 0 실측값과 Jetson 다중 터미널 재현 절차

### 6. Leader velocity command software pipeline

- [base state → controller → velocity guard 상세 재현·검증 가이드](../../../src/leader/rescue_robot_apriltag/docs/LEADER_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)
  - `/leader/base_alignment/state`, `/leader/approach/cmd_vel_raw`, `/leader/cmd_vel`
  - 상태 priority, 제어식, enable gate, clamp, slew, timeout 및 publisher 충돌 점검
  - STM32와 motor를 연결하지 않는 Jetson 수동 검증 절차

### 7. Follower base-link velocity software pipeline

- [Follower base-link velocity pipeline 상세 재현·검증 가이드](../../../src/follower/follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)
  - 측정 camera extrinsic, camera/base 상태 병렬 유지와 exact-stamp TF2 변환
  - approach controller, STOP/APPROACH/COOPERATION selector, 기존 final safety guard
  - 관련 4개 패키지 자동시험 236개와 실카메라 PASS/NOT VERIFIED 구분
  - 현재 후속 구현은 base 안정화 0.30 s + fresh sample 3회, FINAL_APPROACH/STABILIZING
    0.30 s zero-command grace와 approach-session ALIGNED latch reset을 포함

### 8. 이전 기록

- [기존 진행 메모 원문](99_기존_메모/1차_진행상황_원문.txt)
  - 당시 작성한 시간순 메모
  - 최신 상태 판단에는 위의 기능별 문서를 우선 사용

## 접근 상태 9개

| 상태 | 의미 |
|---|---|
| `TAG_LOST` | 태그가 없거나 TF가 오래됨 |
| `TURN_LEFT` | 왼쪽으로 크게 회전 필요 |
| `TURN_RIGHT` | 오른쪽으로 크게 회전 필요 |
| `APPROACH` | 목표보다 멀어서 전진 필요 |
| `TOO_CLOSE` | 목표보다 가까워 후진 필요 |
| `FINE_ALIGN_LEFT` | 왼쪽으로 미세 정렬 필요 |
| `FINE_ALIGN_RIGHT` | 오른쪽으로 미세 정렬 필요 |
| `STABILIZING` | 오차 범위 안에서 안정화 확인 중 |
| `ALIGNED` | 정렬 조건을 정해진 시간 동안 유지 |

이 상태는 high-level 판단 결과다. 별도 controller·selector·velocity guard가 software
Twist를 계산하지만, `/leader/cmd_vel`과 `/follower/safe_cmd_vel`은 STM32 또는 motor에
연결되어 있지 않다.

## 헷갈리기 쉬운 구분

| 구분 | 사용하는 데이터 | 목적 |
|---|---|---|
| D435 거리 CSV | Depth 영상 | 화면 중앙 물체까지의 거리 기록 |
| AprilTag 인식 | RGB 영상과 CameraInfo | 태그 ID와 3차원 상대 위치 계산 |
| 접근 상태 노드 | AprilTag TF | 회전·접근·정렬 상태 판단 |
| URDF·TF | 로봇 치수와 센서 장착 위치 | 카메라 기준 값을 로봇 기준으로 연결 |

## 다음 작업 순서

계획상 현재는 3주차이므로 다음 순서를 사용한다.

1. 두 Orin 빌드·네트워크·시간 동기화와 namespace를 확인한다.
2. 다중 로봇 TF frame 규칙과 `base_link → camera optical frame → gripper TCP`를 확정한다.
3. Orin–STM32 패킷, timeout, 비상정지와 `/leader|follower/cmd_vel` 경로를 고정한다.
4. wheel odometry와 BNO055를 연결하고 `robot_localization` 출력을 검증한다.
5. 리더의 정적 지도 기반 Nav2 목표점 이동과 팔로워 원격 주행을 시험한다.
6. 사람 탐지 ROI와 aligned depth로 카메라 기준 3차원 위치를 출력한다.
7. 태그 물리 시험과 목표 거리 재측정은 4주차 저속 정렬 연결 전에 완료한다.

## 통합 전에 통일할 항목

- 작업공간: 사용자별 절대 경로를 코드에 넣지 않고 `damgc_robot` 저장소 루트를 기준으로 사용
- 역할 namespace: `/leader`, `/follower`
- 로봇별 TF frame 이름과 카메라 optical frame–`base_link` 연결
- 최종 software 주행 토픽: `/leader/cmd_vel`, `/follower/safe_cmd_vel`
- Follower cooperation 입력: 기존 `/follower/cmd_vel`; AprilTag controller가 직접 publish하지 않음
- STM32 패킷, wheel odometry와 IMU 메시지·단위·주기
- 통신 단절 watchdog과 하드웨어·소프트웨어 비상정지 우선순위

Follower camera target `0.15 m`와 base/controller target `0.25 m`는 software-validation
값이다. 실제 파지 거리로 확정하지 말고 카메라 장착 위치와 그리퍼 끝점(TCP)을 기준으로
다시 측정해야 한다.
