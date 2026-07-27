# AprilTag 접근 노드 통제 통합 검사 기록

## 검사 환경과 보호 조건

- 검사일: 2026-07-20
- 기존 USB 카메라, RectifyNode와 AprilTag 프로세스를 재시작하거나 종료하지 않음
- 상태 노드만 timeout 제한 세션에서 실행하고 검사 후 종료
- 인터넷, sudo 및 시스템 설정 변경 없음

## 자동 확인 결과

사전 검사에서 다음 항목을 실제 ROS 그래프에서 확인했다.

- `/follower/camera/usb_cam`, `/RectifyNode`, `/follower/apriltag/apriltag`
- `/follower/camera/image_rect` 메시지 약 12–14 Hz
- `/follower/apriltag/detections` 토픽 존재
- AprilTag `size=0.05`
- `follower_camera_optical_frame -> tag36h11:0` TF의 연속 timestamp 갱신
- 관찰 TF translation 예: `x=-0.013 m`, `y=0.020 m`, `z=0.089 m`

통제 실행 중 `/follower/apriltag_approach` 노드와 다음 publisher 타입을 확인했다.

| 토픽 | 확인된 타입 |
|---|---|
| `/follower/supply/detected` | `std_msgs/msg/Bool` |
| `/follower/supply/tag_id` | `std_msgs/msg/Int32` |
| `/follower/supply/relative_pose` | `geometry_msgs/msg/PoseStamped` |
| `/follower/supply/distance` | `std_msgs/msg/Float64` |
| `/follower/supply/lateral_error` | `std_msgs/msg/Float64` |
| `/follower/supply/straight_distance` | `std_msgs/msg/Float64` |
| `/follower/supply/angle` | `std_msgs/msg/Float64` |
| `/follower/alignment/state` | `std_msgs/msg/String` |

확인된 주요 파라미터는 `target_tag_id=0`,
`source_frame=follower_camera_optical_frame`, `target_distance=0.15`이다.

실행 시점에는 태그 TF가 다시 유실되어 다음 값을 관찰했다.

- `detected=false`
- `tag_id=-1`
- `state=TAG_LOST`
- distance와 angle은 유실 상태에서 발행하지 않으므로 표본 없음

상태 노드는 timeout 및 명시적 interrupt로 종료했으며 이후 프로세스 목록과 ROS
그래프에서 잔여 상태 노드가 없음을 확인했다.

## 관찰 중 주의 사항

카메라/AprilTag 그래프가 검사 도중 일시적으로 discovery에서 보이지 않는 구간이
있었다. 필수 노드, 영상 갱신과 TF가 함께 확인된 시점에만 통합 실행을 진행했다.
실시간 운용 전 DDS discovery 안정성과 카메라 처리율을 별도로 관찰해야 한다.

## 사용자 확인 필요

이번 자동 검사에서는 태그를 물리적으로 움직이지 않았다. 따라서 다음 상태는 성공으로
판정하지 않았다.

- `TURN_LEFT`, `TURN_RIGHT`
- `APPROACH`, `TOO_CLOSE`
- `STABILIZING -> ALIGNED`
- target ID 변경 및 priority/nearest 전환

수동 절차는 `docs/MANUAL_STATE_TEST.md`를 따른다.
