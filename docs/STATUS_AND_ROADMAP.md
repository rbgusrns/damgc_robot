# 개발 현황 및 로드맵

## 문서 목적

이 문서는 [개발 계획서](Plan.md)를 실행 상태로 변환한 관리 문서다. 기준일은
**2026년 7월 27일**이며, 일정상 3주차(7월 27일~8월 2일)의 첫날이다. 2026년 9월 6일
AprilTag software pipeline 후속 구현 상태는 아래 관련 행과 팔로워 절에 추가 반영했다.

상태 표시는 다음 기준을 사용한다.

- **완료**: 저장소 구현과 시험 기록으로 확인됨
- **부분 완료**: 하위 기능은 있으나 계획의 종료 조건을 충족하지 못함
- **미구현**: 저장소에서 구현을 확인할 수 없음
- **확인 필요**: 기구·전장·다른 장비 작업처럼 이 저장소만으로 판단할 수 없음

## 목표 대비 현재 상태

| 계획의 세부 목표 | 상태 | 현재 근거 또는 부족한 조건 |
| --- | --- | --- |
| D435 실시간 3차원 지도 | 부분 완료 | RGB·depth 발행은 확인, Visual SLAM·nvblox 지도는 없음 |
| Visual SLAM 위치·자세 추정 | 미구현 | 관련 패키지·launch·시험 기록 없음 |
| BNO055와 wheel odometry 보정 | 미구현 | URDF의 `imu_link`만 있으며 실제 데이터와 융합 없음 |
| 카메라 기반 생존자 탐지 | 미구현 | 사람 탐지 노드 없음 |
| depth 기반 생존자 3차원 위치 | 미구현 | 중앙 depth CSV 도구는 있으나 사람 검출 결과와 결합되지 않음 |
| Nav2 자율주행 | 미구현 | Nav2 구성·지도·주행 시험 없음 |
| AprilTag 물품 인식·정밀 접근 | 부분 완료 | 양 로봇 camera/base alignment와 guarded software velocity 구현; 실제 접근·파지 검증 필요 |
| 그리퍼 물품 파지 | 확인 필요 | URDF 형상만 있고 제어 코드·실물 시험 근거 없음 |
| 경량 물품 단독 운반 | 미구현 | 접근·파지·주행 연결 없음 |
| 중량 물품 협동 운반 | 부분 완료 | 리더 DDS 상태·속도 게이트 구현, 팔로워 heartbeat/하드웨어 주행 시험 필요 |
| 지도·생존자·로봇 상태 시각화 | 부분 완료 | URDF/RViz와 카메라 확인만 가능, 통합 화면 없음 |

## 현재 저장소에서 재현 가능한 범위

### 리더

- 실제 치수를 반영한 URDF와 RViz 표시
- D435 RGB·depth 토픽 발행
- RGB 보정과 CameraInfo QoS 보조
- `tag36h11`, ID 0, 0.050 m 기준 AprilTag 검출
- depth 영상 중앙 20×20 영역의 거리 CSV 저장

### 팔로워

- USB 카메라 보정과 AprilTag 검출
- `follower_camera_optical_frame` 기준 태그 상대 pose
- 거리·좌우 오차·수평각 계산
- TF timeout, 중앙값 필터와 9개 접근 상태
- ROS 비의존 단위 시험과 전체/상태 전용 launch
- exact-stamp `base_link` hybrid alignment와 atomic alignment command
- approach controller, command selector, final velocity guard와 `/follower/safe_cmd_vel`
- `base_stable_time=0.30 s`, fresh confirmation 3회와 ALIGNED session latch/reset
- FINAL_APPROACH/STABILIZING 0.30 s tag-loss grace; tag 미검출 중 velocity zero
- source stamp/monotonic receipt freshness 분리와 blind-final 기본 OFF

이 범위는 software command pipeline까지다. 실제 모터 주행, 그리퍼 또는 협동 운반이
검증됐다는 의미가 아니다.

## 주차 게이트 점검

### 1주차 게이트: 인터페이스와 개발환경 고정

| 종료 조건 | 상태 | 조치 |
| --- | --- | --- |
| 두 Orin에서 같은 workspace 빌드 | 확인 필요 | 두 장비의 커밋·의존성·빌드 결과 기록 |
| `/leader`, `/follower` namespace 통신 | 부분 완료 | 개별 토픽은 사용, 두 Orin 네트워크 시험 필요 |
| 차체–카메라–IMU TF 확인 | 부분 완료 | 리더 URDF는 있음; 실제 센서 optical frame 연결과 팔로워 차체 TF 필요 |
| RGB-D, IMU, 모터 데이터 확인 | 부분 완료 | RGB-D만 확인됨 |
| STM32 패킷·시간 동기화·Mission 상태도 | 미구현 | 공통 인터페이스 문서와 가짜 노드 필요 |
| 차체·그리퍼·부품 확정 | 확인 필요 | 기구/BOM 담당 기록 연결 필요 |

1주차 게이트가 닫히지 않았으므로 네임스페이스, TF frame, STM32 패킷, 비상정지
경로를 3주차 기능과 병행해 먼저 고정해야 한다.

### 2주차 게이트: 로봇 A 주행과 독립 기능

| 종료 조건 | 상태 | 조치 |
| --- | --- | --- |
| 로봇 A 원격 직진·회전 | 확인 필요 | `/leader/cmd_vel`부터 STM32까지 실물 시험 |
| 3차원 지도 5분 이상 생성 | 미구현 | Visual SLAM·nvblox rosbag 시험부터 수행 |
| 사람과 AprilTag 검출 | 부분 완료 | AprilTag 완료, 사람 탐지 없음 |
| 규격 물품 수동 파지 | 확인 필요 | 그리퍼 시험 기록 필요 |
| 가짜 노드로 임무 상태 전체 순환 | 미구현 | Mission Coordinator 상태·전이 정의 필요 |

### 3주차 게이트: 로봇 A 자율주행과 로봇 B 기본 구동

| 종료 조건 | 상태 | 이번 주 산출물 |
| --- | --- | --- |
| 로봇 A 목표점 반복 이동 | 미구현 | 정적 지도 Nav2 최소 구성과 반복 시험 |
| 사람 위치를 카메라 좌표로 출력 | 미구현 | 사람 검출 ROI와 aligned depth 결합 |
| 로봇 B 원격 속도 주행 | 미구현/확인 필요 | STM32 통신과 `/follower/cmd_vel` 연결 |
| 두 로봇 긴급정지 | 미구현/확인 필요 | 하드웨어 E-stop과 소프트웨어 정지 경로 검증 |
| 전원·발열 30분 시험 | 확인 필요 | 전압·온도·재부팅 여부 기록 |

## 우선순위

### P0 — 통합 전에 반드시 고정

1. 두 Orin의 커밋, ROS 환경, 의존성, ROS domain과 시간 동기화 확인
2. `/leader`, `/follower` 토픽 규칙과 양쪽 TF frame 충돌 방지 규칙 확정
3. `base_link`에서 실제 camera optical frame과 `imu_link`까지의 TF 측정
4. Orin–STM32 패킷, 50 Hz 명령·상태, timeout과 비상정지 동작 정의
5. Mission Coordinator 상태와 가짜 토픽 시험 정의

### P1 — 3주차 종료 조건

1. wheel odometry와 BNO055를 `robot_localization`에 연결
2. 로봇 A의 정적 지도 기반 Nav2 목표점 반복 이동
3. 사람 검출과 aligned depth를 결합한 카메라 좌표 출력
4. 로봇 B의 `/follower/cmd_vel` 주행
5. 두 로봇 E-stop과 전원·발열 30분 시험

### P2 — 이미 앞서 구현된 기능의 현장 검증

1. 태그를 좌우·전후로 이동해 팔로워 hybrid 상태와 부호 확인
2. FINAL_APPROACH/STABILIZING 0.30 s grace, zero velocity와 재검출 복구 실측
3. 실제 물품과 그리퍼 형상으로 목표 거리·허용 오차 재측정
4. 4주차 저속 정렬 제어에 사용할 안전 제한과 watchdog 확정

## 계획 인터페이스와 현재 구현의 관계

계획의 로봇별 odometry·IMU와 실제 배터리·fault 구조화 상태는 아직 목표
인터페이스다. 리더 저장소에는 `leader_cooperation` 패키지가 추가되어
`/follower/cmd_vel`, `/cooperation/*`, `/mission/state`의 리더 측 발행과
`/follower/status` heartbeat 구독을 제공한다.

메시지 타입, QoS, 발행 주기, timeout, 담당 노드와 fault 동작이 확정되기 전까지
목표 인터페이스를 구현 완료로 표시하지 않는다. 자세한 구분은
[리더·팔로워 구조](LEADER_FOLLOWER_ARCHITECTURE.md)를 따른다.

## 갱신 규칙

- 기능 상태를 바꿀 때 실행 명령, 날짜, 장비와 결과 문서 링크를 함께 남긴다.
- “노드가 실행됨”과 “계획의 종료 조건을 만족함”을 구분한다.
- 하드웨어 시험을 하지 않은 기능은 코드가 있어도 부분 완료로 표시한다.
- 매주 종료일에 해당 주차 게이트와 다음 주 우선순위를 갱신한다.
- 8월 30일 이후에는 계획대로 신규 기능을 추가하지 않고 안정화 기록만 갱신한다.
