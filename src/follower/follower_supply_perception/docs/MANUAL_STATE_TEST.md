# AprilTag 접근 상태 수동 시험

- 문서 경로: `docs/MANUAL_STATE_TEST.md`
- 관련 launch: `launch/follower_apriltag.launch.py`, `launch/approach_only.launch.py`
- 점검 스크립트: `scripts/check_approach_topics.sh`

이 절차는 사용자가 실제 태그를 움직이며 상태 판정과 좌표 부호를 확인하기 위한 것이다.
자동 통합 검사나 단위 테스트만으로 물리 상태 전이를 성공했다고 판단하지 않는다.

## 1. 사전 조건

기존 USB 카메라, RectifyNode와 `apriltag_ros`는 별도 터미널에서 실행한 상태로 둔다.
이 문서의 명령은 해당 프로세스를 종료하거나 재시작하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source /home/kde/ros2_ws/install/setup.bash

ros2 node list
ros2 topic hz /follower/camera/image_rect
ros2 run tf2_ros tf2_echo follower_camera_optical_frame tag36h11:0
```

태그가 보일 때 마지막 명령에서 timestamp와 translation이 계속 바뀌어야 한다.
`/follower/apriltag/apriltag`의 `size`도 확인한다.

```bash
ros2 param get /follower/apriltag/apriltag size
```

예상값은 `0.05`이다.

## 2. 상태 노드 실행과 인터페이스 확인

새 터미널에서 다음을 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/kde/ros2_ws/install/setup.bash

ros2 run follower_supply_perception apriltag_approach_node \
  --ros-args \
  -r __ns:=/follower \
  --params-file /home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/approach.yaml
```

다른 터미널에서 점검 스크립트를 실행한다.

```bash
/home/kde/ros2_ws/src/follower_supply_perception/scripts/check_approach_topics.sh
```

필요하면 상태와 측정치를 연속 관찰한다.

```bash
ros2 topic echo /follower/alignment/state
ros2 topic echo /follower/supply/detected
ros2 topic echo /follower/supply/tag_id
ros2 topic echo /follower/supply/relative_pose
```

## 3. 물리 상태 시험

시험값 기준은 `target_distance=0.15 m`, `distance_tolerance=0.02 m`,
`lateral_tolerance=0.02 m`, `angle_tolerance=5 deg`, `tag_timeout=0.3 s`,
`stable_time=0.8 s`이다. 이 값들은 실제 그리퍼 동작 거리로 확정된 값이 아니다.

optical frame에서 `x<0`은 왼쪽, `x>0`은 오른쪽, `z`는 전방 거리다.

| 시험 | 조작 | 예상 출력 | 사용자 기록 |
|---|---|---|---|
| 태그 유실 | 태그를 완전히 가리고 0.3초 이상 대기 | `detected=false`, `tag_id=-1`, `TAG_LOST` | 미확인 |
| 왼쪽 각도 | 태그를 영상 왼쪽으로 충분히 이동 | `TURN_LEFT` | 미확인 |
| 오른쪽 각도 | 태그를 영상 오른쪽으로 충분히 이동 | `TURN_RIGHT` | 미확인 |
| 먼 거리 | 태그를 정면에 두고 `z>0.17 m` | `APPROACH` | 미확인 |
| 너무 가까움 | 태그를 정면에 두고 `z<0.13 m` | `TOO_CLOSE` | 미확인 |
| 목표 위치 | 거리·좌우·각도 오차를 모두 허용 범위에 유지 | 즉시 `STABILIZING`, 0.8초 후 `ALIGNED` | 미확인 |

각 시험에서 `/follower/supply/distance`, `lateral_error`, `angle`도 함께 기록한다.
상태 우선순위 때문에 각도가 5도를 벗어나면 거리보다 TURN 상태가 먼저 출력된다.

## 4. ID 변경 시험

기본 설정의 `target_tag_id=0`은 ID 0만 추적한다. ID 0을 가리고 다른 ID를 보여도
`TAG_LOST`가 유지되는지 먼저 확인한다.

`target_tag_id`는 현재 구현에서 시작 시 한 번 읽는 파라미터다. 실행 중
`ros2 param set`으로 바꾸지 말고 상태 노드만 `Ctrl-C`로 종료한 뒤 다음처럼 다시
실행한다. 카메라, RectifyNode와 AprilTag 노드는 종료하지 않는다.

```bash
ros2 run follower_supply_perception apriltag_approach_node \
  --ros-args \
  -r __ns:=/follower \
  --params-file /home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/approach.yaml \
  -p target_tag_id:=1
```

ID 1을 보여 `tag_id=1`과 상태 출력을 확인한다. 다중 ID priority 시험은 다음과 같다.

```bash
ros2 run follower_supply_perception apriltag_approach_node \
  --ros-args \
  -r __ns:=/follower \
  --params-file /home/kde/ros2_ws/install/follower_supply_perception/share/follower_supply_perception/config/approach.yaml \
  -p target_tag_id:=-1 \
  -p allowed_tag_ids:="[0, 1, 2]" \
  -p selection_mode:=priority
```

여러 태그를 동시에 보여 배열 앞쪽의 보이는 ID가 선택되는지 확인한다. `nearest` 시험은
마지막 인자를 `-p selection_mode:=nearest`로 바꾸고 가장 가까운 태그가 선택되는지
확인한다. 선택 ID가 바뀐 직후에는 안정화가 다시 `STABILIZING`부터 시작해야 한다.

## 5. 완료 기록

- 시험 일시:
- 카메라/태그 배치:
- 확인한 태그 ID:
- 각 상태의 실제 관찰 결과:
- RViz에서 확인한 frame과 pose:
- 조정한 파라미터와 이유:
- 남은 문제:

물리 시험을 마치기 전에는 TURN, APPROACH, TOO_CLOSE 또는 ALIGNED의 현장 동작이
검증됐다고 기록하지 않는다.

## 6. 사용자 RViz Pose 최종 확인

```bash
source /opt/ros/humble/setup.bash
source /home/kde/ros2_ws/install/setup.bash
rviz2
```

Fixed Frame을 `follower_camera_optical_frame`으로 정하고 `TF` display와
`Pose` display를 추가한다. Pose Topic은 `/follower/supply/relative_pose`로 설정한다.
태그를 좌우·전후로 움직여 TF와 Pose 위치 및 optical-frame 부호가 일치하는지 사용자가
직접 확인한다. 태그 유실 시 오래된 Pose가 새 값처럼 계속 발행되지 않는지도 확인한다.
이 RViz 검사는 자동 수행 완료로 기록하지 않는다.
