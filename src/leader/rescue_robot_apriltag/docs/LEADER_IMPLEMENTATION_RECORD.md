# Leader AprilTag 상태판단 구현 기록

## 1. 작업 정보

- 작업 날짜: 2026-08-27 KST
- 저장소: `/home/maze/damgc_robot`
- 브랜치: `main`
- 대상: Leader D435 AprilTag 상대 위치 기반 상태판단
- 구현 범위: camera optical frame 기준 상태 및 상대 측정 토픽 발행

Follower의 검증된 설계는 참조했지만
`src/follower/follower_supply_perception`은 수정하거나 import하지 않았다.

## 2. 생성 파일

```text
src/leader/rescue_robot_apriltag/config/approach.yaml
src/leader/rescue_robot_apriltag/rescue_robot_apriltag/__init__.py
src/leader/rescue_robot_apriltag/rescue_robot_apriltag/approach_logic.py
src/leader/rescue_robot_apriltag/rescue_robot_apriltag/apriltag_approach_node.py
src/leader/rescue_robot_apriltag/test/test_approach_logic.py
src/leader/rescue_robot_apriltag/docs/LEADER_APRILTAG_APPROACH_SPEC.md
src/leader/rescue_robot_apriltag/docs/LEADER_APRILTAG_APPROACH_TEST_RESULTS.md
src/leader/rescue_robot_apriltag/docs/LEADER_MANUAL_STATE_TEST.md
src/leader/rescue_robot_apriltag/docs/LEADER_APRILTAG_APPROACH_GUIDE.md
src/leader/rescue_robot_apriltag/docs/LEADER_IMPLEMENTATION_RECORD.md
```

직접 작성한 파일은 모두 `/home/maze/damgc_robot/src/leader` 아래에 있다.

## 3. 수정 파일

### `src/leader/rescue_robot_apriltag/CMakeLists.txt`

- `ament_cmake_python`과 Python package 설치 추가
- `apriltag_approach_node` executable 설치 추가
- `ament_cmake_pytest`와 `test_approach_logic.py` 등록

### `src/leader/rescue_robot_apriltag/package.xml`

- `ament_cmake_python` buildtool dependency 추가
- `geometry_msgs`, `std_msgs`, `tf2_ros` 실행 dependency 추가
- `ament_cmake_pytest`, `python3-pytest` test dependency 추가

### `src/leader/rescue_robot_bringup/launch/camera_apriltag.launch.py`

- 기존 D435, robot state publisher, CameraInfo bridge, rectify, AprilTag node 보존
- 설치 share의 `config/approach.yaml`을 가리키는 `approach_config` 추가
- 기본값 `false`인 `enable_approach` 추가
- `/leader/apriltag_approach`를 조건부로 정확히 한 번 추가

기존 `config/apriltag_leader.yaml`과 Follower 파일은 수정하지 않았다.

## 4. 주요 구현 결정

- 실제 `source_frame`: `camera_color_optical_frame`
- 실제 tag frame pattern: `leader/tag36h11:{id}`
- 상태 노드: `/leader/apriltag_approach`
- translation: finite이고 `z>0`인 값만 수용
- quaternion: finite, 비영 노름 확인 후 정규화
- stale: `now - TF stamp > tag_timeout`이면 loss
- 유실: `false`, `-1`, `TAG_LOST`만 발행하고 stale metric 미발행
- filter: distinct timestamp의 x/y/z 성분별 median, window 5
- ID 변경: filter와 stable timer 초기화
- 선택: 고정 ID 또는 allowed ID의 priority/nearest
- 측정: z, x, 3차원 거리, `atan2(x,z)`
- 상태: 9개 Follower 호환 상태 문자열
- 제어: `cmd_vel`, STM32, gripper, Nav2 미구현

`target_distance=0.15 m`와 `tag_timeout=1.0 s`는 현장 승인을 기다리는 초기 시험값이다.
timeout은 Leader tag TF 약 29.98 Hz, 최대 표본 간격 0.166787초와 최대 timestamp age
0.067281초의 10초 측정에 근거한다.

## 5. 실행한 조사·검증 명령

### 저장소와 파일 조사

```bash
cd /home/maze/damgc_robot
git status --short --branch
git diff --stat
git diff -- src/follower/follower_supply_perception
rg -n "apriltag_approach|camera_color_optical_frame|tag36h11" src/leader
```

### ROS graph와 TF 조사

```bash
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
ros2 node list
ros2 topic list -t
ros2 topic echo /leader/camera/color/camera_info --once
ros2 topic echo /leader/apriltag/detections --once
ros2 run tf2_ros tf2_echo \
  camera_color_optical_frame leader/tag36h11:0
```

기존 사용자의 camera/AprilTag launch가 실행 중일 때는 이를 자동 kill하거나 전체
launch를 중복 실행하지 않았다.

### Python compile

소스 트리에 `__pycache__`를 새로 만들지 않도록 파일 내용을 메모리에서 compile한다.

```bash
cd /home/maze/damgc_robot
python3 - <<'PY'
from pathlib import Path

paths = [
    Path("src/leader/rescue_robot_apriltag/rescue_robot_apriltag/__init__.py"),
    Path("src/leader/rescue_robot_apriltag/rescue_robot_apriltag/approach_logic.py"),
    Path("src/leader/rescue_robot_apriltag/rescue_robot_apriltag/apriltag_approach_node.py"),
    Path("src/leader/rescue_robot_apriltag/test/test_approach_logic.py"),
    Path("src/leader/rescue_robot_bringup/launch/camera_apriltag.launch.py"),
]
for path in paths:
    compile(path.read_text(encoding="utf-8"), str(path), "exec")
print(f"Python compile OK: {len(paths)} files")
PY
```

### build, test와 launch 정적 확인

```bash
cd /home/maze/damgc_robot
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select rescue_robot_apriltag rescue_robot_bringup \
  --symlink-install
source /home/maze/damgc_robot/install/setup.bash
colcon test --packages-select rescue_robot_apriltag
colcon test-result --verbose
ROS_LOG_DIR=/home/maze/damgc_robot/log/ros2_launch_validation \
  ros2 launch rescue_robot_bringup camera_apriltag.launch.py --show-args
```

## 6. 최종 build와 test 결과

2026-08-27 문서 작성 후 동일 workspace에서 재실행한 결과다.

| 검사 | 결과 |
|---|---|
| Python compile | 5 files 성공 |
| `rescue_robot_apriltag` build | 성공 |
| `rescue_robot_bringup` build | 성공 |
| pytest testcase | 46 passed |
| CTest wrapper | 1 passed |
| `colcon test-result --verbose` | 47 tests, 0 errors, 0 failures, 0 skipped |
| `git diff --check` | 성공 |

47은 pytest testcase 46개와 ament/CTest wrapper 1개를 합친 집계다. 카메라 독립
테스트가 실제 물리 이동이나 RViz까지 검증한다고 해석하지 않는다.

## 7. 자동 ROS graph 검증

실제 Leader D435 pipeline에서 확인한 항목은 다음과 같다.

- `/leader/camera`, `/RectifyNode`, `/leader/apriltag/apriltag`
- image `/leader/camera/color/image_raw`
- CameraInfo `/leader/camera/color/camera_info`
- detections `/leader/apriltag/detections`
- `camera_color_optical_frame -> leader/tag36h11:0`
- family/ID `tag36h11/0`, 설정 size 0.050 m
- RGB 약 27 Hz, rectified RGB 약 18.5 Hz
- tag TF timestamp 약 29.98 Hz
- `/leader/apriltag_approach` 실행
- Leader 8개 출력 토픽과 타입
- `detected=true`, `tag_id=0`, 상태 `TURN_RIGHT`
- 표본 distance 0.323555 m, lateral 0.032603 m
- 표본 straight distance 0.337547 m, angle 0.100428 rad
- 상태 노드 제거 후 기존 detection publisher 유지

통합 launch 수정 후 다음도 확인했다.

- 설치된 launch와 설치된 `approach.yaml` 경로 존재
- `ros2 pkg executables`에 `apriltag_approach_node` 존재
- `camera_apriltag.launch.py --show-args`에 `enable_approach`와 설치 config 경로 표시
- 정적 launch description에 approach executable이 한 번 포함
- 현재 pipeline에 상태 노드만 짧게 연결했을 때 `TAG_LOST -> TURN_RIGHT` 전환
- 임시 상태 노드 정리 후 기존 AprilTag publisher count 1 유지

전체 통합 launch runtime은 당시 기존 D435 launch가 실행 중이어서 중복 카메라 기동을
피하려고 수행하지 않았다.

문서 최종 감사 시점에는 사용자가 실행하던 pipeline이 이미 종료돼 ROS graph에 Leader
node와 `/leader/apriltag/detections`가 없었다. 이 최종 조회를 위해 카메라 launch를
임의로 재시작하지 않았다.

## 8. 사용자 검증 필요 항목

다음 항목은 아직 성공으로 기록하지 않는다.

- 태그 가림 후 실제 `TAG_LOST` 시간과 stale pose/metric 미발행
- 물리 이동에 따른 `TURN_LEFT`, `TURN_RIGHT`, `APPROACH`, `TOO_CLOSE`
- 실제 `FINE_ALIGN_LEFT`, `FINE_ALIGN_RIGHT` 도달
- 연속 안정 상태의 `STABILIZING -> ALIGNED`
- ID 1, 2 등록 후 fixed ID, priority, nearest와 ID 변경 reset
- 장시간 부하와 조명 변화에서 `tag_timeout=1.0 s`의 적합성
- RViz TF와 `/leader/supply/relative_pose`의 위치·방향·timestamp 일치
- D435 장착 위치와 그리퍼/TCP를 반영한 `target_distance` 확정
- 기존 launch 종료 후 `enable_approach:=true` 전체 통합 runtime

시험 절차와 기록표는 `LEADER_MANUAL_STATE_TEST.md`에 있다.

## 9. 알려진 제한

- 출력은 `camera_color_optical_frame` 기준이며 `base_link` 기준이 아니다.
- `target_distance=0.15 m`는 실제 파지 목표가 아니다.
- `tag_timeout=1.0 s`는 초기 시험값이다.
- nearest에는 hysteresis가 없다.
- 현재 `apriltag_leader.yaml`에는 ID 0만 등록돼 있다.
- angle 판정이 lateral 판정보다 먼저라 현재 tolerance에서 fine-align 상태의 물리적
  도달 범위가 좁을 수 있다.
- timeout SIGINT 종료 중 Humble/rclpy executor traceback이 한 번 관찰됐으며 정상
  Ctrl-C 재현 여부를 별도 확인해야 한다.
- RViz, `cmd_vel`, gripper, STM32, Mission Coordinator, Nav2는 통합하지 않았다.

## 10. 전체 launch 명령

기존 camera launch를 먼저 정상 종료한 뒤 실행한다.

```bash
cd /home/maze/damgc_robot
source /opt/ros/humble/setup.bash
source /home/maze/damgc_robot/install/setup.bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py \
  enable_approach:=true
```

## 11. 최종 Git 상태

문서 작성과 최종 검사 시점의 `git status --short --untracked-files=all`은 다음과 같다.

```text
 M src/leader/rescue_robot_apriltag/CMakeLists.txt
 M src/leader/rescue_robot_apriltag/package.xml
 M src/leader/rescue_robot_bringup/launch/camera_apriltag.launch.py
?? src/leader/rescue_robot_apriltag/config/approach.yaml
?? src/leader/rescue_robot_apriltag/docs/LEADER_APRILTAG_APPROACH_GUIDE.md
?? src/leader/rescue_robot_apriltag/docs/LEADER_APRILTAG_APPROACH_SPEC.md
?? src/leader/rescue_robot_apriltag/docs/LEADER_APRILTAG_APPROACH_TEST_RESULTS.md
?? src/leader/rescue_robot_apriltag/docs/LEADER_IMPLEMENTATION_RECORD.md
?? src/leader/rescue_robot_apriltag/docs/LEADER_MANUAL_STATE_TEST.md
?? src/leader/rescue_robot_apriltag/rescue_robot_apriltag/__init__.py
?? src/leader/rescue_robot_apriltag/rescue_robot_apriltag/approach_logic.py
?? src/leader/rescue_robot_apriltag/rescue_robot_apriltag/apriltag_approach_node.py
?? src/leader/rescue_robot_apriltag/test/test_approach_logic.py
```

`git diff --stat`은 tracked 수정 세 파일에 대해 다음을 표시한다. untracked 생성 파일은
stage하기 전에는 이 통계에 포함되지 않는다.

```text
src/leader/rescue_robot_apriltag/CMakeLists.txt         | 15 +++++++++++++++
src/leader/rescue_robot_apriltag/package.xml            |  8 ++++++++
src/leader/rescue_robot_bringup/launch/camera_apriltag.launch.py | 17 +++++++++++++++++
3 files changed, 40 insertions(+)
```

`git diff -- src/follower/follower_supply_perception` 출력은 비어 있다.

## 12. 다음 개발 단계

1. 사용자 물리 태그 이동과 RViz 시험을 기록한다.
2. 장시간 TF 지연을 측정해 `tag_timeout`을 승인한다.
3. D435 extrinsic과 그리퍼/TCP 목표를 측정한다.
4. TF2로 `camera_color_optical_frame` pose를 `base_link` 기준으로 변환한다.
5. 안전 요구와 속도 제한을 별도로 설계한 뒤 구동 계층과 연동한다.
