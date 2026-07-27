# AprilTag 접근 상태 로직 단위 테스트 결과

- 문서 경로: `docs/TEST_RESULTS_APRILTAG_APPROACH.md`
- 테스트 파일: `test/test_approach_logic.py`

## 실행 환경

- 실행일: 2026-07-20
- ROS 2: Humble
- Python: 3.10.12
- 작업공간: `/home/kde/ros2_ws`
- 패키지: `follower_supply_perception`
- 테스트 방식: 외부 카메라, 실제 TF, 인터넷 및 sudo를 사용하지 않는 pytest

## 실행 명령

```bash
cd /home/kde/ros2_ws

python3 -c 'from pathlib import Path; root=Path("src/follower_supply_perception"); files=list((root/"follower_supply_perception").glob("*.py"))+list((root/"launch").glob("*.launch.py"))+list((root/"test").glob("test_*.py"))+[root/"setup.py"]; [compile(p.read_text(), str(p), "exec") for p in files]; print("Python compile OK:", len(files), "files")'

source /opt/ros/humble/setup.bash
colcon build --packages-select follower_supply_perception
source /home/kde/ros2_ws/install/setup.bash
colcon test --packages-select follower_supply_perception
colcon test-result --verbose
```

## 최종 결과

- Python compile: 최종 소스·launch·test·setup 7개 파일 성공
- colcon 빌드: 1개 패키지 성공
- pytest 수집: 37개
- 통과: 37개
- 실패: 0개
- 오류: 0개
- 건너뜀: 0개
- JUnit 결과: `/home/kde/ros2_ws/build/follower_supply_perception/pytest.xml`

최종 `colcon test-result --verbose` 출력:

```text
Summary: 37 tests, 0 errors, 0 failures, 0 skipped
```

## 수정 과정

첫 번째 테스트 실행에서는 37개 중 36개가 통과하고 거리 상한 `0.17 m` 경계 테스트
1개가 실패했다. `0.15 + 0.02`가 이진 부동소수점에서 약
`0.16999999999999998`로 표현되어, 수학적으로 허용 경계인 `0.17`을 `APPROACH`로
판정한 것이 원인이었다.

`approach_logic.py`의 각도·거리·좌우 허용 경계 비교에 작은 부동소수점 동등성 처리를
추가했다. 수정 후 compile, symlink 빌드, 전체 테스트와 결과 집계를 다시 실행했으며
37개 모두 통과했다.

최종 문서 작성 후에는 일반 `colcon build`와 전체 테스트를 다시 실행해 같은
`37 tests, 0 errors, 0 failures, 0 skipped` 결과를 확인했다.

## 단위 테스트가 검증한 범위

- 음·양 수평각 초과에 대한 `TURN_LEFT`, `TURN_RIGHT`
- 목표 거리보다 먼/가까운 경우의 `APPROACH`, `TOO_CLOSE`
- 거리·각도 정상 상태의 `FINE_ALIGN_LEFT`, `FINE_ALIGN_RIGHT`
- `STABILIZING` 시작과 연속 `stable_time` 후 `ALIGNED`
- 조건 이탈, 태그 ID 변경 및 태그 유실 시 안정화 타이머 초기화
- `distance=z`, `lateral_error=x`, `atan2(x,z)`, 3차원 직선거리 계산
- 거리·각도·좌우 허용 경계의 inclusive 처리
- 허용 ID priority, nearest, nearest 동률과 비허용 ID 제외
- 중앙값 필터의 outlier 억제와 동일 timestamp 중복 제외
- 잘못된 임계값, filter window, selection mode, translation과 quaternion 방어

## 검증하지 못한 실시간 범위

다음 항목은 카메라 없는 단위 테스트의 범위가 아니며 실제 장비 통합 시험이 필요하다.

- USB 카메라 영상 수신 및 CameraInfo 정확성
- `apriltag_ros` 검출률, 실제 태그 크기 설정과 pose 정확도
- `/tf` 수신, frame 이름 및 TF timestamp/timeout의 실시간 동작
- ROS namespace 적용 후 8개 출력 토픽의 실제 연결과 주기
- Jetson Orin Nano 부하 상태에서 20 Hz timer 지연
- 여러 물리 태그가 동시에 보일 때 선택 전환과 거리 노이즈
- RViz pose/TF 시각 확인 및 실제 카메라 좌표축 부호 확인
- 실제 목표 거리와 lateral/angle/stable time의 현장 튜닝

이 결과는 순수 상태·수학 로직의 결정적 동작을 입증한다. 실시간 파이프라인 시작과
필수 토픽은 별도 통합 검사에서 확인했지만, 물리 상태 전이와 최종 RViz Pose 확인은
여전히 사용자가 직접 수행해야 한다.
