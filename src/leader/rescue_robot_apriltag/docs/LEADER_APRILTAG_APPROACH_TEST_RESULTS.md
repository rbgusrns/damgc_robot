# Leader AprilTag 상태판단 단위 테스트 결과

## 1. 실행 환경과 대상

- 실행 일시: 2026-08-27 KST
- 플랫폼: Jetson Orin Nano, Ubuntu 22.04, ROS 2 Humble
- 저장소: `/home/maze/damgc_robot`
- 패키지: `rescue_robot_apriltag`
- 테스트 파일: `test/test_approach_logic.py`
- 대상 모듈: `rescue_robot_apriltag/approach_logic.py`
- Python: 3.10.12
- pytest: 6.2.5

이 테스트는 순수 Python 상태 로직만 대상으로 한다. 카메라, 인터넷, ROS graph,
실제 `/tf`와 AprilTag 하드웨어를 사용하지 않았다. Follower 테스트는 참조만 했으며
수정하거나 Leader 테스트에서 import하지 않았다.

## 2. 실행 명령

### Python compile

소스에 `__pycache__`를 만들지 않도록 `compile()`로 Leader Python package와 테스트
파일을 메모리에서 컴파일했다.

```bash
cd /home/maze/damgc_robot
python3 - <<'PY'
from pathlib import Path

root = Path('src/leader/rescue_robot_apriltag')
files = sorted((root / 'rescue_robot_apriltag').glob('*.py'))
files += sorted((root / 'test').glob('test_*.py'))
for path in files:
    compile(path.read_text(encoding='utf-8'), str(path), 'exec')
print(f'Python compile OK: {len(files)} files')
PY
```

### Package build와 test

```bash
cd /home/maze/damgc_robot
source /opt/ros/humble/setup.bash
colcon build --packages-select rescue_robot_apriltag --symlink-install
source /home/maze/damgc_robot/install/setup.bash
colcon test --packages-select rescue_robot_apriltag
colcon test-result --verbose
```

## 3. 결과

| 단계 | 결과 |
|---|---|
| Python compile | 4 files 성공 |
| 선택 package build | 1 package 성공 |
| pytest 수집 | 46 cases |
| pytest 결과 | 46 passed, 0 failed, 0 skipped |
| pytest 실행 시간 | 0.23 s |
| CTest wrapper | 1 passed |
| `colcon test-result --verbose` | 47 tests, 0 errors, 0 failures, 0 skipped |

`colcon test-result`의 47은 pytest의 46개 testcase와 이를 실행하는 ament/CTest
wrapper 1개를 함께 집계한 값이다. 실제 로직 testcase 수는 46개이며 실패를 skip하거나
제거하지 않았다.

생성된 결과 파일:

- `build/rescue_robot_apriltag/test_results/rescue_robot_apriltag/test_approach_logic.xunit.xml`
- `build/rescue_robot_apriltag/Testing/20260827-1407/Test.xml`
- `build/rescue_robot_apriltag/ament_cmake_pytest/test_approach_logic.txt`

## 4. 자동 검증 범위

### 상태와 안정화

- 음수 angle 허용 범위 초과 시 `TURN_LEFT`
- 양수 angle 허용 범위 초과 시 `TURN_RIGHT`
- 정면 원거리에서 `APPROACH`
- 정면 근거리에서 `TOO_CLOSE`
- 거리·각도 정상이고 x가 음수 허용 범위를 벗어나면 `FINE_ALIGN_LEFT`
- 거리·각도 정상이고 x가 양수 허용 범위를 벗어나면 `FINE_ALIGN_RIGHT`
- 모든 조건 최초 만족 시 `STABILIZING`
- `stable_time` 연속 유지 후 `ALIGNED`
- 조건 이탈 후 stable timer 초기화
- measurement 또는 tag ID가 없을 때 `TAG_LOST` 및 timer 초기화
- 선택 tag ID 변경 시 stable timer 초기화
- non-finite state-machine time 거부

### 계산과 경계

- `distance = z`
- `lateral_error = x`
- `straight_distance = sqrt(x²+y²+z²)`
- `angle = atan2(x,z)`
- distance, lateral, angle 허용오차 경계값을 정상 범위에 포함

### 입력 유효성

- translation의 NaN과 infinity 거부
- `z==0`과 `z<0` 거부
- quaternion 정규화
- 잘못된 길이, 영 노름, NaN, infinity quaternion 거부
- 0 이하 filter window 거부
- 잘못된 selection mode 거부
- 0 target distance, 음수 tolerance/stable time, non-finite threshold 거부

### 필터와 태그 선택

- x/y/z 성분별 median filter가 outlier를 억제
- 같은 TF timestamp를 중복 표본으로 넣지 않음
- filter reset 후 같은 timestamp도 새로운 표본으로 수용
- `priority`가 `allowed_tag_ids` 순서를 따름
- `nearest`가 3차원 직선거리를 사용
- nearest 동률에서 `allowed_tag_ids` 순서를 사용
- allowed 목록 밖 ID 제외

## 5. 실제 D435/ROS graph에서만 검증 가능한 범위

다음 항목은 이 카메라 비의존 단위 테스트의 성공으로 검증됐다고 간주하지 않는다.

- `tf2_ros.Buffer`와 `TransformListener`의 실제 lookup 성공 여부
- `camera_color_optical_frame -> leader/tag36h11:<id>`의 live TF 연결
- `TransformException`, stale TF와 `tag_timeout`의 실제 시간 동작
- D435 부하 변화·조명·가림·검출 누락 중 false `TAG_LOST` 여부
- `/leader/supply/*`와 `/leader/alignment/state`의 실제 ROS 메시지 타입·주기·값
- 유실 중 pose와 수치 metric이 실제 graph에서 재발행되지 않는지 여부
- 여러 실제 tag ID의 priority/nearest 선택과 전환 노이즈
- 태그 좌우·전후 이동에 따른 9개 상태의 물리적 전이
- `PoseStamped`와 TF의 frame, timestamp, translation, quaternion 일치
- RViz에서 optical-frame 축 방향과 상대 Pose의 육안 일치
- 장시간 실행 시 CPU 부하, TF 지연과 초기 시험용 `tag_timeout=1.0`의 최종 적합성

위 항목은 이후 camera/AprilTag launch 통합 시험과 사용자의 실제 D435 수동 시험에서
별도로 기록해야 한다.
