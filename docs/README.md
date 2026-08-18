# 프로젝트 문서

이 폴더는 `damgc_robot`의 계획, 현재 상태, 시스템 인터페이스와 시험 근거를
구분해 관리합니다. 계획에 적혀 있다는 이유만으로 구현 완료로 간주하지 않습니다.

## 먼저 읽을 문서

1. [개발 계획서](Plan.md) — 2026년 7월 13일~9월 14일의 목표, 역할, 9주 일정과 평가 기준
2. [개발 현황 및 로드맵](STATUS_AND_ROADMAP.md) — 2026년 7월 27일 기준 구현 상태, 주차 게이트와 우선순위
3. [Visual SLAM 준비 및 검증 절차](VISUAL_SLAM_SETUP.md) — 현재 입력·TF 점검, rosbag 기록과 SLAM 연동 순서
4. [프로젝트 개요](PROJECT_OVERVIEW.md) — 현재 저장소 구조, 패키지 역할과 실행 방법
5. [리더·팔로워 구조](LEADER_FOLLOWER_ARCHITECTURE.md) — 목표 아키텍처와 현재 연결 상태
6. [Orin–STM32 UART 통신 규격](STM32_UART_PROTOCOL.md) — 명령, IMU, 엔코더, 상태 패킷 초안

## 문서 목록

- [1차 구현·시험 기록](progress/week%201/README.md) — URDF, D435, AprilTag와 접근 상태 판정의 실행 근거
- [GitHub 등록 및 운영 기록](GITHUB_SETUP.md) — 저장소 연결과 안전한 변경 등록 절차

팔로워의 AprilTag 접근 상태 노드 상세 명세와 시험 기록은
`src/follower/follower_supply_perception/docs/`에서 관리합니다.

## 문서 기준

문서가 충돌하면 다음 순서로 해석합니다.

1. 최종 목표·범위·일정: `Plan.md`
2. 완료 여부·현재 우선순위: `STATUS_AND_ROADMAP.md`
3. 실제 ROS 이름·파라미터: 현재 소스 코드와 패키지 README
4. 특정 장비에서 수행한 결과: `docs/progress/`와 패키지별 시험 기록

상태 문서는 기능이 검증될 때 갱신하고, 실험 기록의 과거 명령과 결과는 임의로
현재 사실처럼 바꾸지 않습니다.
