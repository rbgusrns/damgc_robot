# 팔로워 인계: 수동 합체 + 협동 경로 준비 / 출발 키

작성: 2026-10-04. **이 문서가 리더와 팔로워의 통신 계약입니다.**
기존 팔로워 PR #1 (`dbc24b9`)의 경로 수학을 기반으로 하며, grasp/lift
미션 전체를 실행하는 기능과 분리합니다.

## 요구사항과 현재 상태

사용자가 직접 로봇을 합체하고 물품을 잡습니다. 이번 흐름은 **다이나믹셀을
실행하거나 파지/리프트 토픽·서비스를 호출하지 않습니다.** 소프트웨어 READY는
물리적 합체를 센서로 확인했다는 뜻이 아닙니다.

- 리더 **B**: 기존 주행 정지 → 자동 Nav2 명령 선택 차단 → 새 RViz 목표 대기.
- RViz에서 **새** 목표 선택: Nav2가 생성한 경로를 검증하고 팔로워에 전달.
- 팔로워: 경로/오도메트리/명령 소유권 검증 → 정지 상태로 READY 응답.
- 리더: 자신과 팔로워의 준비가 확인되면 터미널에 **COOP leader READY** 표시.
- 사용자가 **N**: 양쪽을 0속도로 arm → 공통 출발 시각 합의 → 주행 시작.
- Space, 통신 끊김, 제어 실패, 목표 도착: 양쪽 명령을 정지.

리더는 기존 **Nav2 RPP**를 유지합니다. `NavigateToPose`를 준비 단계의 경로
생성에 쓰고, 실행 때는 확정 경로를 `/follow_path`에 제출합니다. 자동 재계획으로
팔로워와 다른 경로를 실행하지 않습니다. 팔로워는 역주행 가능한 pure pursuit로
자신의 차축 경로를 추종합니다.

참고 구현은 [`src/cooperative_transport`](../src/cooperative_transport)에 있습니다.
리더 로컬 및 Docker 빌드, 팔로워 호스트 빌드는 완료했습니다. 협동 실차
주행/출발 시간차는 검증하지 않았습니다. 인계 시점 팔로워에서는 이 패키지를
복사·빌드만 했으며, 노드를 실행하거나 다이나믹셀을 조작하지 않았습니다.
팔로워 기존 미커밋 수정은 보존했습니다.

## 기존 팔로워 코드에서 주의할 부분

`cooperative_mission/follower_mission_node.py`의
`/follower/path_tracking/enable` (`SetBool`)은 현재 enable하면 selector와 guard를
arm하고 **주행을 시작**합니다. **PREPARE를 이 서비스에 연결하면 안 됩니다.**
준비와 실제 출발을 분리한 상태 머신이 필요합니다.

가장 간단한 적용은 새 `cooperative_transport` peer를 사용하고 기존
`cooperative_mission` 실행기를 함께 실행하지 않는 것입니다. 기존 실행기에
통합한다면 아래 계약을 그대로 구현하고, 준비 중에는 파지/리프트 미션과
standalone tracking 모두 정지 상태를 유지해야 합니다. 실행기는 IDLE이어야
하며 명령 토픽은 하나의 publisher만 소유해야 합니다.

## 환경과 ROS 연결

| 항목 | 리더 | 팔로워 |
|---|---|---|
| SSH / tailnet | 100.76.60.55 | `kde@100.86.100.99` |
| 로컬 Wi-Fi | 192.168.0.6 | 192.168.0.7 |
| ROS_DOMAIN_ID | **0** | **0**으로 맞출 것 |
| ROS_LOCALHOST_ONLY | 0 | 0 |
| RMW_IMPLEMENTATION | rmw_fastrtps_cpp | rmw_fastrtps_cpp |
| FASTDDS_BUILTIN_TRANSPORTS | UDPv4 | UDPv4 |
| 현재 작업 경로 | `/home/maze/damgc_robot` | `/home/kde/damgc_robot_mission` |
| 오도메트리 | `/leader/odometry/local` | `/follower/odom/raw` |

SSH 비밀번호는 저장소에 기록하지 않습니다. SSH 접속 성공과 DDS 통신 성공은
별개입니다. 두 로봇은 현재 같은 Wi-Fi에 있으므로 DDS는 해당 LAN을 사용합니다.
기존 `scripts/ros2_dds_env.sh`는 기본 domain 42이므로 domain 0을 명시해야 합니다.
기존 팔로워의 휠/IMU 보정값을 리더 값으로 덮어쓰지 않습니다.

## ROS 인터페이스

모든 wire 메시지는 `std_msgs/msg/String` 안의 JSON입니다.

| 토픽 | 방향 / 의미 | QoS |
|---|---|---|
| `/cooperation/transport/leader` | 리더 → 팔로워 | reliable / volatile / depth 10 |
| `/cooperation/transport/follower` | 팔로워 → 리더 | reliable / volatile / depth 10 |
| `/cooperation/transport/control` | 리더 내부 PREPARE / START / ABORT | reliable / volatile / depth 10 |
| `/cooperation/transport/leader/status` | 사람용 상태 문자열 | reliable / transient_local / depth 1 |
| `/cooperation/transport/follower/status` | 사람용 상태 문자열 | reliable / transient_local / depth 1 |
| `/cooperation/transport/leader/path` | 리더 차축 경로 `nav_msgs/Path` | reliable / transient_local / depth 1 |
| `/cooperation/transport/follower/path` | 팔로워 **자기 odom** 차축 경로 `nav_msgs/Path` | reliable / transient_local / depth 1 |
| `/leader/cooperation/cmd_vel` | 리더 selector COOPERATION 입력 | reliable / volatile / depth 1 |
| `/follower/mission/cmd_vel` | 팔로워 selector COOPERATION 입력 | reliable / volatile / depth 1 |

팔로워 필요한 기존 서비스:

- `/follower/command_selector/set_parameters`: `rcl_interfaces/srv/SetParameters`.
  **한 요청에** `source_mode` 하나만 변경. STOP / COOPERATION 사용.
- `/follower/velocity_guard/enable`: `std_srvs/srv/SetBool`.
  준비는 비활성/0속도, N 이후 arm 단계에서 활성화. 실패/종료 시 비활성화.
- selector의 `cmd_vel` 구독을 `/follower/mission/cmd_vel`로 remap.
- guard 입력 `/follower/selected_cmd_vel`, 출력 `/follower/safe_cmd_vel`.
- STM32 bridge의 `cmd_vel`을 `/follower/safe_cmd_vel`로 remap.
- 반대 방향으로 물품을 잡으므로 **guard의 `allow_reverse:=true`가 필수**.

`/follower/mission/cmd_vel`에 이전 mission executor와 새 peer가 동시에
publish하면 준비를 거절합니다. `/follower/cmd_vel` 직통으로 우회하지 않습니다.

## 패킷 공통 필드와 경로 식별

```json
{"v":1,"kind":"HB","session":"리더가 만든 UUID hex","hash":"경로 SHA-256","state":"READY","progress":0.0,"speed_ratio":0.0}
```

- `v`: 정수 1.
- `session`: B마다 새 UUID. 지난 세션의 ACK/START/STOP을 무시.
- `hash`: 아래 `body` 전체의 SHA-256 hex. 경로 변경 시 새 준비 절차 필요.
- `kind`: 아래 표의 패킷 종류.
- ACK, COMMIT, heartbeat, STOP은 **session과 hash가 모두 일치**해야 처리.
- 새 PREPARE는 팔로워 IDLE/STOPPED/DONE/ERROR에서만 받아들임.
  실행 중 다른 session의 PREPARE가 주행 소유권을 가져가면 안 됨.
- 같은 PREPARE 재수신 시 이미 정지 확인된 READY를 재응답할 뿐,
  기준 pose / progress / 경로를 다시 초기화하거나 움직이면 안 됨.

해시 계산은 Python 표현을 그대로 사용합니다. 해시 확인 전에 int/float 형식을
바꾸지 마세요. 다른 언어 구현은 같은 JSON canonicalization을 맞춰야 합니다.

```python
canonical = json.dumps(body, sort_keys=True, separators=(',', ':'), allow_nan=False)
path_hash = hashlib.sha256(canonical.encode()).hexdigest()
```

### PREPARE와 body

리더는 WAIT_READY에서 0.5초마다 재전송합니다.

```json
{
  "v":1,"kind":"PREPARE","session":"...","hash":"...",
  "nonce":"매 요청마다 새 UUID hex","t1":1791100000.0,
  "body":{
    "frame":"odom",
    "geometry":[0.125,0.1325,0.0525,0.2617993877991494],
    "leader":[[0.0,0.0,0.0],[0.05,0.0,0.0],[0.10,0.0,0.0]],
    "follower":[[0.62,0.0,3.141592653589793],[0.67,0.0,3.141592653589793],[0.72,0.0,3.141592653589793]],
    "expected_follower":[0.62,0.0,3.141592653589793],
    "speed":0.05
  }
}
```

- `leader`: **원래 Nav2 차축 경로** `[x metres, y metres, yaw radians]` 배열.
- `follower`: 리더가 협동 기하로 생성한 팔로워 차축 경로. **리더 odom 좌표**.
- `frame`: 리더 경로의 좌표계. 팔로워 자기 odom과 같다는 의미가 아님.
- `geometry`: axle→hinge, hinge→contact, centre→contact (m), hinge limit (rad).
  팔로워 설정과 1e-6 이내 일치해야 함.
- `expected_follower`: 실제 리더 준비 pose와 중립 합체 기하로 계산한 팔로워
  차축 pose. 전달 경로의 첫 pose를 실제 기준으로 무조건 간주하지 말 것.
- `speed`: 현재 0.05m/s. 유한한 양수이며 팔로워 제한과 같아야 함.
- 배열 3~4000개 pose, 경로 간격 최대 8cm, 유한한 값만 허용.
- 이미 goal 오차 7.5cm 이내인 경로는 준비하지 않고 거절.

팔로워는 `leader_path_to_object_path(leader, geometry, curvature_margin=.8,
lateral_tolerance=.01)`를 독립 실행하고 그 `formation.follower`가 전달 배열과
일치하는지 확인합니다. 참고 구현 허용차는 위치 5mm, yaw 0.01rad입니다.
MIXED 전후진 / lateral orientation mismatch 경로는 거절합니다.

### READY와 clock ping

STOP 파라미터 변경의 **성공 응답을 받은 뒤** READY 전송:

```json
{"v":1,"kind":"READY","session":"...","hash":"...","nonce":"PREPARE/PING nonce 그대로","t1":1791100000.0,"t2":1791100000.003,"t3":1791100000.004}
```

- `t1`: 요청의 값 그대로 echo.
- `t2`: 요청 수신 즉시 팔로워 `time.time()` (UTC Unix seconds).
- `t3`: 응답 발행 직전 팔로워 `time.time()`.
- 리더 `t4`는 응답 수신 시각. offset = `((t2-t1)+(t3-t4))/2`.
- 리더는 monotonic RTT에서 `(t3-t2)` 처리 시간을 뺀 값을 확인.
  현재 계약은 네트워크 RTT ≤200ms.
- READY 이후 리더는 body 없는 `PING` (`nonce`,`t1`)을 0.5초마다 전송.
  팔로워 READY 상태에서 같은 READY 응답을 재전송하여 시계를 확인.
- 팔로워가 기준점에서 움직였거나 odom이 오래됐으면 READY 재응답 대신 STOP.

### ARM → COMMIT → 공통 시각

1. 사용자가 N. 리더는 자기 selector를 COOPERATION으로 바꾸되 출력은 0 유지.
2. 리더 WAIT_ARM에서 `ARM` 반복. 추가 필드 없음.
3. 팔로워 READY에서 ARM을 받으면 상태 ARMING:
   stationary/odom/소유권 재확인 → selector COOPERATION 성공 응답 →
   guard enable 성공 응답 → ARMED. **여전히 0만 publish**.
4. 팔로워 ARMED에서 `ARM_ACK`를 100ms마다 재전송. 추가 필드 없음.
5. 리더는 확정 경로의 Nav2 FollowPath 수락을 받은 뒤 출발을 2초 뒤로 예약:

```json
{"v":1,"kind":"COMMIT","session":"...","hash":"...","start":1791100002.0,"offset":0.003}
```

`start`는 리더 wall-clock 기준 Unix seconds, `offset`은 **팔로워 시각-리더 시각**.

6. 팔로워 ARMED에서 `delay = start + offset - time.time()`.
   `0.5 < delay < 4.0`이어야 수락. stationary를 재확인하고
   `local_start_monotonic = time.monotonic()+delay`를 **한 번만** 저장.
   상태 SCHEDULED로 바꾸고 `COMMIT_ACK` 전송. 중복 COMMIT은 ACK만 재전송.
7. 리더는 출발 0.5초 전까지 COMMIT_ACK가 도착해야 SCHEDULED. 아니면 STOP.
8. 양쪽 출발 시 자신의 monotonic deadline 도달, peer 상태
   SCHEDULED/RUNNING, heartbeat age ≤300ms를 확인 후 RUNNING으로 전환.
   출발 callback이 deadline보다 150ms 넘게 늦으면 출발하지 말고 STOP.
9. RUNNING 전에 속도를 발행하지 않음. 초기 0.2초는 공통 속도 ramp 적용.

이 합의는 **best effort**입니다. 무선 비대칭 지연/ROS callback/모터 반응 시간차가
남으며, 네트워크 단절 상황에서 두 로봇의 원자적 동시 출발을 보장할 수 없습니다.
출발 시간차를 실제 로그로 측정하기 전에는 '정확히 동시에 검증 완료'로 표시하면
안 됩니다. 통신 손실 때 로컬 watchdog로 0속도를 유지합니다.

### HB: 실행 동기화

세션 중 100ms 주기로 발행:

```json
{"v":1,"kind":"HB","session":"...","hash":"...","state":"RUNNING","progress":0.2,"speed_ratio":1.0}
```

- `progress`: `nearest_progress_index/(pose_count-1)`, [0,1].
- `speed_ratio`: 리더 RPP 속도 크기 / 설정 속도, [0,1] clamp.
  follower HB에서는 현재 참고 구현이 0을 발행; 리더는 follower ratio를 쓰지 않음.
- follower 명령은 leader ratio로 스케일하여 RPP가 collision 등으로 0을 낼 때
  follower만 계속 진행하지 않게 함. linear와 angular에 같은 배율 적용.
- 자기 progress가 peer보다 0.02 넘게 앞서면 감속:
  `factor=clamp(1-10*max(0,own_progress-peer_progress-.02),0,1)`.
- progress 차이 >0.15 또는 peer HB age >0.6초면 양쪽 정지 요청.
- READY/ARMED/SCHEDULED에서도 fresh odom과 stationary를 계속 확인.
  단, 예정 출발 직전 150ms는 peer 출발 시차 때문에 물리적으로 밀릴 수 있으므로
  stationary 속도 검사는 생략하되 출발 위치/heading 허용차는 다시 확인.

### STOP와 완료

```json
{"v":1,"kind":"STOP","session":"...","hash":"...","detail":"peer heartbeat lost"}
```

즉시 0 publish → selector STOP → guard disable → 상태 STOPPED.
경로 추종기/FollowPath도 취소. 다음 B 전까지 자동 재출발 금지.
종료 상태에서도 STOP 재전송하여 손실된 단일 패킷에 의존하지 않음.
`detail="DONE"`은 완료를 의미하며 peer도 DONE으로 전환합니다.

도착 허용차는 5cm. 한쪽 ARRIVED가 확인됐을 때 다른 쪽의 현재 goal 오차가
7cm 이내면 공동 DONE으로 정지; 더 크면 실패로 정지합니다. 마지막 제자리
heading rotation을 물리적으로 합체한 채 수행하면 안 됩니다.

## 좌표 초기화: odom 문자열을 공유 좌표로 착각하지 말 것

독립 bridge의 frame_id가 둘 다 `odom`이어도 좌표 원점은 다릅니다.
기존 follower 코드는 frame 이름이 같으면 TF 변환을 생략하므로 그대로 사용할
경우 리더 좌표의 경로를 자기 원점에서 잘못 따라갈 수 있습니다.

현재 범위는 사용자가 **중립 직선 힌지 + 반대 방향**으로 합체한 뒤 B를 누르는
경우입니다. PR 기하 기준 centre→axle = 0.31m, axle 간격 = 0.62m.
centre→contact 0.0525m는 PR의 물품 두께 10.5cm 설정이며, 다른 물체/팔 위치를
사용할 때는 양쪽 기하 설정과 물리 배치를 같이 맞춰야 합니다.

팔로워 READY 준비 때 실제 자기 wheel odom `F_local`을 한 번 저장합니다.
리더가 보내준 `expected_follower=F_leader`에서 이 pose로 가는 SE(2) 변환:

```python
a = normalize_angle(F_local.yaw - F_leader.yaw)
dx, dy = P_leader.x - F_leader.x, P_leader.y - F_leader.y
P_local.x = F_local.x + cos(a)*dx - sin(a)*dy
P_local.y = F_local.y + sin(a)*dx + cos(a)*dy
P_local.yaw = normalize_angle(P_leader.yaw + a)
```

모든 follower 경로 pose를 이 변환으로 자기 odom으로 바꾸고 고정합니다.
첫 pose 오차 ≤5cm, heading 오차 ≤3° 확인. 준비 뒤 위치가 2.5cm 이상 또는
heading이 3° 이상 바뀌면 B부터 다시 준비합니다. 이것은 물리 배치를 전제로 한
초기화이며, 임의 힌지각/비대칭 합체를 측정해주는 localization이 아닙니다.

## 팔로워 실행 구성과 구현 범위

참고 패키지 그대로 사용할 경우:

```bash
cd /home/kde/damgc_robot_mission
source /opt/ros/humble/setup.bash
colcon build --packages-select cooperative_transport --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch cooperative_transport manual_transport.launch.py \
  role:=follower follower_drive:=true use_stm32_bridge:=true
```

이 launch는 peer, selector, final guard, STM32 drive/odom bridge만 실행합니다.
초기 selector STOP, guard disabled, reverse enabled. Dynamixel/AprilTag approach/
자동 grasp/lift 미션은 실행하지 않습니다.

기존 selector/guard/bridge가 같은 remap으로 실행 중이면 `follower_drive:=false`로
peer만 추가해야 합니다. 명령 publisher를 중복 실행하지 마세요.

기존 follower_mission_node에 통합할 경우의 필수 변경:

1. standalone enable과 별개인 prepare/armed/scheduled gate 추가.
2. PREPARE 수신은 freeze/STOP/READY까지만 처리.
3. 두 robot odom의 SE(2) 초기 기준을 명시적으로 저장.
4. nonce/session/hash와 중복 메시지 처리를 유지.
5. ARM service 성공 후에도 예정 출발까지 0속도 유지.
6. peer heartbeat / leader velocity ratio / progress 동기화 및 STOP 구현.
7. mission 파지/리프트 state machine을 이번 모드에서 호출하지 않음.
8. 휠 odom age ≤350ms, 경로 오차 ≤10cm, 충돌 없는 단일 cmd_vel 소유권 확인.
9. 역주행 부호와 곡률 일관성을 유지; 제자리 회전 금지.
10. terminal READY/RUNNING/STOPPED/DONE과 원인을 기록.
11. `/parameter_events`에서 자기 selector의 source_mode 소유권 변경을 감시.
    실행 중 COOPERATION이 아닌 모드가 되면 peer에도 STOP을 알림.

참고 follower 추종 제한: lookahead .20m, speed .05m/s, angular .20rad/s,
path error .10m, goal tolerance .05m. 실제 `angular/linear`이 협동 hinge curvature
limit의 1.1배를 넘는 명령은 정지합니다. 선속도와 각속도를 따로 부호 반전하지
말고 생성된 follower pose 경로의 drive_direction을 사용하세요.

## 남은 한계와 인계 기준

- 임의 모양 물체/힌지각을 자동 인식하지 않음. 수동 중립 합체가 전제.
- solo Nav2의 radius .20m 경로를 그대로 허용하지 않음. 현재 cooperative
  object radius 제한은 PR 기하/12° reserve 기준 약 1.47m.
- mixed 전후진 cusp나 급격한 곡률은 거절. 새 협동 경로를 자동 재탐색하지 않음.
- 전체 swept footprint가 알려진 global costmap 내부이고 장애물과 겹치지
  않아야 함. 시작 시와 N 때 확인하며 Nav2에는 합체 footprint도 설정.
  현재 Nav2 envelope는 리더 기준 x[-.08,.75]m, y±.32m로 힌지 12°까지
  포함하는 보수적 설정. 중립 직선의 실제 폭보다 커서 좁은 통로가 거절될 수 있음.
  기하/차체 치수를 변경하면 이 footprint도 함께 갱신해야 함.
- 초기 map이 없거나 follower 자체가 장애물로 저장됐으면 경로가 거절될 수 있음.
  이 경우 원인 로그를 확인해야 하며 임의로 collision 확인을 제거하지 않음.
- 센서가 가려진 HOLD 상태에서는 저장 지도와 오도메트리에 의존. 미지의
  동적 장애물을 완전 검출하는 기능은 이번 통신 구현 범위가 아님.
- 현재는 빌드 완료, 실차 loaded 동기 주행 및 출발 skew 미검증.

팔로워 구현 완료 후 리더에서 먼저 idle 노드/토픽/odom 상태를 확인하고,
사용자가 B와 새 목표를 보내 READY까지 확인합니다. **N은 사용자가 누릅니다.**
리더 로그에는 session/hash, clock RTT, state transition, 실행 command와
STOP 원인이 남아야 합니다. rosbag 항목은 리더 launcher에 추가했습니다.

## 최신 문서를 기존 팔로워 작업을 덮어쓰지 않고 읽기

팔로워 repo가 fork의 이전 main을 기준으로 하고 미커밋 수정도 있으므로,
이 기능을 위해 작업 트리를 강제로 main으로 교체하지 마세요.

```bash
# remote 'leader'가 없을 때 한 번만 추가
 git remote add leader https://github.com/rbgusrns/damgc_robot.git
 git fetch leader main
 git show leader/main:docs/COOP_TRANSPORT_FOLLOWER_HANDOFF.md
```

참고 구현 위치:

- `src/cooperative_transport/cooperative_transport/transport_peer.py`: 양쪽 protocol/gate.
- `src/cooperative_transport/cooperative_transport/hinged_formation.py`: PR 경로 기하.
- `src/cooperative_transport/cooperative_transport/path_tracking.py`: follower pure pursuit.
- `src/cooperative_transport/launch/manual_transport.launch.py`: 소프트웨어/drive-only 실행.
- `scripts/run_manual_cooperative_transport.sh`: domain 0 실행 wrapper.
- `src/leader/rescue_robot_bringup/scripts/mapping_mode_manager.py`: B/N와 지도 모드 연동.
- `src/leader/leader_command_selector`: 독립 COOPERATION 명령 입력.
