# 협동 운반 Leader 구현 응답

2026-10-04, 팔로워의
[COOPERATIVE_TRANSPORT_LEADER_HANDOFF.md](https://github.com/kmhuh1525/damgc_robot/blob/codex/cooperative-mission-20260930/docs/COOPERATIVE_TRANSPORT_LEADER_HANDOFF.md)
검토 결과입니다. 확정 wire 규격은
[COOP_TRANSPORT_FOLLOWER_HANDOFF.md](COOP_TRANSPORT_FOLLOWER_HANDOFF.md)에 정리했습니다.

## 팔로워 요청에 대한 반영

| 팔로워 요청 | 리더 구현 |
|---|---|
| selector가 COOPERATION을 받아야 함 | 추가 완료. `/leader/cooperation/cmd_vel` 구독, COOPERATION source 및 watchdog 지원. 모터 topic 직통 우회 없음. |
| enable_nav2_goal_selection gate | 현재 리더에는 선언되어 있음. BOOL 런타임 변경 지원과 cooperative mode 차단을 구현. 이전 fork selector와 코드 차이 있음. |
| B/N/중지 전달 | B/N 및 Space/teleop abort 추가. 아래 public→manager→peer 경로 사용. |
| 새 목표의 plan만 채택 | B 이후 새 NavigateToPose goal status stamp와 plan stamp를 확인. 이전 goal 취소, 이전 plan 사용하지 않음. |
| frame/시작 정렬 | 동일: leader odom frame, 시작 위치 5cm / heading 3° / 간격 8cm / 3–4000 pose. |
| follower feasible 경로만 허용 | PR 기하 수학 사용, 80% 힌지 reserve, 비측방/전후진 일정 방향 검증. 실패하면 원인과 함께 WAIT_PLAN. 자동 대체 경로 생성은 미구현. |
| FollowPath와 RPP gate | N 후 확정 경로를 FollowPath에 제출, 수락/ARM/COMMIT ACK 이후 공통 예정 시각까지 0 유지. RPP 유지, 실패 시 공동 STOP. |
| costmap의 합체 외곽 확인 | leader odom global costmap + updates, unknown/범위 밖/장애물 거절. 샘플 사이도 촘촘하게 검사. 힌지 기울어진 chassis 모서리까지 포함. |

## 반드시 맞춰야 하는 제어 토픽 차이

팔로워 인계 문서의 'peer가 `/leader/mapping/control`을 받는다'는 표현은,
현재 리더에서는 **mapping mode manager가 public 키 요청을 받아 peer로 전달**하는
구조입니다. 이것은 mapping manager의 주기적인 자동 Nav2 선택 갱신이 협동
차단을 다시 풀어버리지 않게 하기 위한 것입니다.

```text
arrow_key_teleop / transport_keys
  └─ /leader/mapping/control : COOP_PREPARE / COOP_START / COOP_ABORT
       └─ mapping_mode_manager
            ├─ automatic Nav2 source gate, HOLD map, combined footprint
            └─ /cooperation/transport/control : PREPARE / START / ABORT
                 └─ leader transport_peer
                      └─ /cooperation/transport/leader : wire protocol
                           └─ follower transport_peer
```

팔로워는 `/leader/mapping/control`이나 내부 control topic을 subscribe할 필요가
없습니다. **leader wire topic만 처리**하고 자기 wire topic으로 응답하면 됩니다.
peer가 public과 internal control을 동시에 받아 B를 중복 처리하지 않게 하세요.

## 경로/속도/상태 세부 확정

- `body.leader`는 **원래 Nav2 경로**. 변환한 leader 경로를 다시 invert하는
  double conversion을 하지 않습니다. 양쪽 같은 원본으로 formation을 계산.
- `body.follower`는 이 원본에서 생성한 follower axle path, leader odom 좌표.
- 최초 독립 검증 후 follower SE(2) 기준을 **한 번만** 고정.
- 준비는 selector STOP 성공 ACK 뒤 READY. 기존 path_tracking/enable 호출 금지.
- `READY`/clock ping은 nonce와 t1/t2/t3 포함. t1은 요청과 그대로 일치.
- 같은 speed 설정이어야 함. 현재 양쪽 0.05m/s.
- HB 100ms, PREPARE/PING 500ms. COMMIT start는 leader epoch seconds,
  offset 부호는 follower-minus-leader. monotonic deadline 저장.
- COMMIT 2초 전 예약, ACK는 출발 0.5초 전까지, 출발 callback 지연 >150ms 거절.
- 출발할 때 peer가 이미 RUNNING인 경우도 허용: 두 callback의 작은 시차가
  정상적인 출발을 오류로 만들지 않게 합니다.
- 출발 직전 150ms 내에는 상대 로봇이 먼저 움직여 합체체가 밀릴 수 있으므로
  stationary 속도 조건을 생략하고 시작 위치/heading 조건은 유지.
- 이미 도착점 7.5cm 이내인 경로는 준비 단계에서 거절.
- RPP ratio는 follower 감속/충돌 정지 동기화에 사용.
- 시작 0.2초 공통 ramp, path progress 차이와 heartbeat watchdog.
- 한쪽 ARRIVED 때 다른 쪽 goal 오차 ≤7cm이면 공동 DONE/STOP.
- 자기 selector source가 바뀌면 `/parameter_events`를 통해 소유권 상실을 감지.
- 리더 cooperative footprint는 PR 기본 기하의 힌지 reserve ±12°를 고려해
  x[-.08,.75]m / y±.32m. 보수적이라 좁은 경로가 거절될 수 있습니다.
  기하 변경 시 이 footprint도 같이 맞춰야 합니다.

## 공유 방식과 현재 검증 범위

공유 저장소에 새 `cooperative_transport` 패키지를 올립니다. 기존 follower
mission/gripper 파일은 이번 리더 변경에 포함하지 않습니다. 팔로워에서 기존
미커밋 수정을 보존하고, 참고 패키지를 그대로 쓰거나 동일 protocol을 기존
실행기에 구현하면 됩니다. 팔로워에 먼저 복사한 untracked 패키지는 초기 초안이며,
확정 규격과 최신 참고 코드는 이번 GitHub 커밋을 기준으로 확인해야 합니다.

리더 host/Docker와 초기 참고 코드 follower 빌드 완료. 실제 cooperative loaded
주행, PREPARE→READY 양방향 인수, 출발 skew 측정은 아직 완료하지 않았습니다.
이번 작업에서 N/START를 누르거나 자동 grasp/lift를 실행하지 않았습니다.

먼저 두 로봇을 정지 상태로 두고 B→새 RViz goal→READY를 확인하는 흐름에
동의합니다. N은 작업자가 양쪽 준비를 확인한 뒤 명시적으로 누르는 키입니다.
소프트웨어 scheduling을 물리적 동시 출발 보장으로 표현하지 않습니다.
