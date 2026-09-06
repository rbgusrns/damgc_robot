# follower_supply_perception 구현 기록

- 문서 경로: `docs/IMPLEMENTATION_RECORD.md`
- 작업 날짜: 2026-07-20
- 작업공간: `/home/kde/ros2_ws`

## 1. 생성·수정 파일

아래는 패키지 기준 상대 경로다. `build/`, `install/`, `log/`는 colcon/ROS 생성물이며
직접 작성 소스가 아니다.

| 상대 경로 | 작업 내용 |
|---|---|
| `package.xml` | ament_python 및 ROS/launch 런타임 의존성 구성 |
| `setup.py` | console script와 config/launch 설치 규칙 구성 |
| `setup.cfg` | ROS executable 설치 경로 구성 |
| `resource/follower_supply_perception` | ament index marker 생성 |
| `follower_supply_perception/__init__.py` | Python 패키지 생성 |
| `follower_supply_perception/approach_logic.py` | 순수 계산·필터·선택·상태 머신 구현 및 경계 수정 |
| `follower_supply_perception/apriltag_approach_node.py` | TF/ROS 노드 구현 및 interrupt-safe cleanup 수정 |
| `config/approach.yaml` | 상태 판단 시험 파라미터 생성 |
| `config/apriltag.yaml` | 검증된 원본 AprilTag 설정 복사 |
| `launch/follower_apriltag.launch.py` | 전체 카메라/태그/상태 파이프라인 생성 |
| `launch/approach_only.launch.py` | 상태 노드 전용 launch 생성 |
| `test/test_approach_logic.py` | 카메라 없는 37개 테스트 생성 |
| `scripts/check_approach_topics.sh` | ROS 인터페이스 점검 스크립트 생성·보강 |
| `README.md` | 빌드·실행·문서 링크 갱신 |
| `docs/TASK_SPEC_APRILTAG_APPROACH.md` | 요구 명세 생성·정합화 |
| `docs/TEST_RESULTS_APRILTAG_APPROACH.md` | 실제 테스트 결과 생성·갱신 |
| `docs/MANUAL_STATE_TEST.md` | 사용자 물리 시험 절차 생성·갱신 |
| `docs/INTEGRATION_CHECK_APRILTAG_APPROACH.md` | 실제 ROS 통합 결과 생성 |
| `docs/LAUNCH_VALIDATION_APRILTAG.md` | launch 검증 결과 생성·갱신 |
| `docs/APRILTAG_APPROACH_NODE_GUIDE.md` | 최종 종합 운영 가이드 생성 |
| `docs/IMPLEMENTATION_RECORD.md` | 이 변경 기록 생성 |
| `config/.gitkeep`, `launch/.gitkeep`, `test/.gitkeep` | 초기 디렉터리 placeholder 생성 |

## 2. 실행한 주요 명령

```bash
source /opt/ros/humble/setup.bash
cd /home/kde/ros2_ws

colcon list
colcon build --packages-select follower_supply_perception
source /home/kde/ros2_ws/install/setup.bash
colcon test --packages-select follower_supply_perception
colcon test-result --verbose

ROS_LOG_DIR=/home/kde/ros2_ws/log \
  ros2 launch follower_supply_perception follower_apriltag.launch.py --show-args
ROS_LOG_DIR=/home/kde/ros2_ws/log \
  ros2 launch follower_supply_perception approach_only.launch.py --show-args
```

통합 검사에서는 `ros2 node list`, `ros2 topic list -t`, `ros2 topic hz`,
`ros2 param get`, `tf2_echo`와 timeout 제한 launch를 사용했다. 기존 카메라·AprilTag
노드를 자동 종료하지 않았다.

## 3. 성공 결과

- 최종 compile 대상 7개 파일 성공.
- ROS 2 Humble에서 패키지 인식과 build 성공.
- Python 소스와 launch compile 성공.
- console executable `apriltag_approach_node` 설치 확인.
- 단위 테스트 최종 37개 통과, 오류·실패·건너뜀 0.
- 두 launch의 `--show-args`와 install 공간 발견 확인.
- approach-only와 전체 launch 실제 시작/정리 확인.
- 전체 launch에서 네 노드, 영상·보정·검출·TF·8개 상태 출력 확인.
- `/follower/camera/image_rect` 약 11.8 Hz와 tag `size=0.05` 확인.
- 종료 후 관련 프로세스와 ROS 노드가 남지 않음.
- 원본·패키지·설치 AprilTag YAML SHA-256 일치.

## 4. 실패와 수정 내용

1. 초기 `rosdep check`는 시스템 rosdep 미초기화로 실패했다. `sudo rosdep init`은 사용자
   승인 없이 실행하지 않았으며 설치된 의존성으로 build가 성공했다.
2. 일반 설치 후 새 YAML을 추가한 첫 symlink build에서 Humble `symlink_data`가 없는
   대상 파일을 제거하려 해 실패했다. 패키지 일반 build로 대상 설치 파일을 생성한 뒤
   symlink build를 재검증했다.
3. 첫 단위 테스트는 `0.15+0.02` 부동소수점 표현 때문에 거리 상한 1건이 실패했다.
   경계 비교에 `isclose` 기반 동등성 처리를 적용해 37개 모두 통과했다.
4. sandbox에서 `ros2 launch --show-args`가 `/home/kde/.ros/log`를 쓰지 못했다.
   `ROS_LOG_DIR=/home/kde/ros2_ws/log`로 재실행해 성공했다.
5. approach-only timeout 종료 중 두 번째 SIGINT가 cleanup에 들어가 exit -2가 발생했다.
   main cleanup을 interrupt-safe하게 수정해 child clean 종료를 확인했다.
6. ROS discovery가 순간적으로 node/topic을 누락해 점검 스크립트가 오판했다. 토픽 타입
   재시도와 파라미터 서비스 기반 node 확인을 추가했다.
7. 전체 launch 첫 시도 전 기존 수동 노드 중복을 발견했다. 자동 kill하지 않고 사용자
   종료 후 실행해 카메라 장치와 토픽 충돌을 피했다.
8. USB 카메라는 일부 V4L2 선택 control에 `unknown control` 경고를 냈다. 필수 영상과
   CameraInfo가 정상 갱신돼 비치명 경고로 기록했다.

## 5. 자동 검증된 항목

- 계산식, 9개 상태와 우선순위, 안정화 reset, 허용 경계
- priority/nearest와 ID 전환, 중앙값 필터, 잘못된 입력 방어
- 패키지 build/install, executable, config와 launch 설치
- launch arguments, 네 파이프라인 노드와 필수 토픽 타입
- image_rect 갱신, tag size, 일부 TF/상태 메시지
- timeout/interrupt cleanup과 잔여 프로세스 부재

## 6. 사용자가 검증해야 하는 항목

- 태그 물리 이동에 따른 TURN_LEFT/RIGHT, APPROACH, TOO_CLOSE
- 목표 위치에서 STABILIZING 후 ALIGNED 유지
- 여러 실제 ID의 priority/nearest 전환과 노이즈
- 실제 그리퍼/TCP 기준 목표 거리 재측정
- Jetson 장시간 운용 부하와 카메라 프레임률
- RViz에서 TF와 `/follower/supply/relative_pose`의 최종 육안 확인

자동 작업은 물리 상태 전이와 최종 RViz 시험을 성공으로 기록하지 않았다.

## 7. 알려진 한계

- source frame이 카메라 optical frame이며 base_link 기준 변환이 아직 없다.
- target_distance 0.15 m는 시험값이다.
- 파라미터는 시작 시 읽으므로 target_tag_id 동적 적용 callback이 없다.
- nearest 선택 hysteresis가 없어 비슷한 거리 태그 사이에서 전환될 수 있다.
- cmd_vel, STM32, 그리퍼와 MarkerArray는 구현되지 않았다.
- TAG_LOST일 때 이전 pose/metric을 재발행하지 않으므로 consumer가 detected를 확인해야 한다.

## 8. Git status와 diff 요약

`/home/kde/ros2_ws`에서 실행한 `git status --short --branch`는 다음 오류로 종료됐다.

```text
fatal: not a git repository (or any of the parent directories): .git
```

`git diff --stat`도 동일하게 저장소가 아니어서 diff 기준을 만들 수 없었다. 따라서 Git
변경 수나 patch 통계는 제공하지 않는다. 이 문서의 파일 표, 최종 `find` 결과와
build/test 결과를 현재 상태의 근거로 사용한다. 자동 커밋은 수행하지 않았다.

## 9. 2026-09-06 hybrid tag-loss tolerance 갱신

현재 Git 저장소 `/home/kde/damgc_robot`의 HEAD `1f4d2ed`를 기준으로 Leader의 최신
tag-loss handling을 Follower 구조에 맞게 최소 이식했다.

- `base_stable_time=0.30 s`, `aligned_confirm_samples=3`
- `stabilizing_tag_loss_grace_sec=0.30 s`
- `final_approach_tag_loss_grace_sec=0.30 s`
- Grace 동안 state/control mode만 유지하고 stale target pose를 발행하지 않아 raw
  `cmd_vel`을 zero로 유지
- Strictly newer source stamp만 valid observation/reacquisition/grace reset으로 인정
- `tag_timeout=2.0 s` source sanity와 `tag_receipt_timeout=0.35 s` monotonic dropout 분리
- blind가 disabled일 때 `blind_last_tag_max_age`가 일반 visual loss를 앞당기던 경로 분리
- `blind_final_approach_enabled=false` 기본값 유지
- selected tag 변경, explicit reset과 `/follower/approach/enabled` session event에서
  `ALIGNED` latch reset

관련 targeted perception/controller tests 81개와 `follower_supply_perception` 전체
135개가 통과했고, `follower_supply_perception` 및 `follower_approach_control` build가
성공했다. 실제 로봇/카메라 runtime grace timing은 아직 물리 검증 결과로 기록하지 않는다.
