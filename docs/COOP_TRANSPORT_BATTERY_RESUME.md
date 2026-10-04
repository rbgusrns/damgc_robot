# 협동 운반 배터리 교체 / 재시작 인계 — 2026-10-04

## 목표와 실제 진행 상태

범위는 **경로 전달 → 양쪽 준비 응답 → READY 표시 → 작업자 N으로 공동 시작**.
합체·파지는 작업자가 수동으로 준비하며 Dynamixel은 이번 흐름에서 실행하지 않습니다.

| 항목 | 배터리 교체 직전 확인 |
|---|---|
| 공유 코드 | `b5f5c63`: B/N, path/session/hash, READY/ARM/COMMIT, RPP gate, follower reference |
| 리더 실행 | 최신 peer / mode manager / STOP selector, 별도 B/N 키 창을 실행했음 |
| 팔로워 실행 | 기존 작업과 분리한 overlay에서 peer + selector + guard + STM32 bridge 실행했음 |
| DDS | 양쪽 domain 0, 서로의 node/status/odom 발견 확인 |
| 팔로워 odom | frame `odom`, 관측 시 최신 수신 age 약 26ms |
| 구동 상태 | leader/follower selector STOP 서비스 성공, follower guard disable 성공 |
| 준비 세션 | **IDLE. 합체 전이라 새 경로 PREPARE→READY는 아직 수행하지 않음** |
| 실제 출발 | **N/START를 보내지 않았음. 협동 실차 주행/출발 skew 미검증** |
| 저장 및 종료 | 아래 지도 체크포인트 저장, cooperative 노드 종료 요청 및 follower 프로세스 종료 확인 |

팔로워 로그에는 STM32 RX sequence gap 경고가 관측됐습니다. 최신 odom 수신은
확인했지만 엔코더/IMU 전체 정확도나 CRC 무오류를 이번 준비로 검증한 것은 아닙니다.

## 보존한 로컬 자료

- 지도: `data/maps/battery_coop_checkpoint_20261004/`
  - `map.nvblx`: **26,206,208 bytes**, 저장 성공 응답과 nonempty 파일 확인.
  - `manifest.json`: 저장 시 software odom x=0.0603337433m,
    y=0.6846558400m, yaw=-114.8386595°.
  - 저장 시각: 2026-10-04 18:19:59 KST.
  - 원래 `home_anchor` 별도 보존: x=-0.6164289293m, y=-0.0677489809m,
    yaw=10.3590795°.
  - **체크포인트 pose는 합체 측정 기준이나 자동 재위치 추정 결과가 아님.**
    그대로 복원할 때는 저장 당시 물리 위치와 방향이 같아야 합니다.
- 리더 실행 로그: `log/cooperative_prepare_20261004/`
  - `leader_peer.log`, `manager.log`, `battery_map_save.json`.
- 팔로워 실행 로그: `/home/kde/coop_transport_runtime/prepare.log`.
- 이전 장시간 rosbag: `data/vslam_mapping_20261004_165546/`.
  - 디스크 부족으로 recorder가 종료됐었음.
  - `ros2 bag reindex` 완료, `metadata.yaml` 복구.
  - 이후 새 협동 준비를 전부 기록한 rosbag이라고 간주하면 안 됨.
  - 정상 종료 처리에서 `analysis.md` 생성 완료.

종료 후 leader/follower의 cooperative peer, selector, guard, STM32 bridge 프로세스가
남아 있지 않은 것을 확인했습니다. 기존 실행 중이던 launcher는 종료 시
`unexpected EOF`를 기록했지만 cleanup은 `Stopped`까지 완료했습니다. 실행 중인
스크립트를 수정한 상황이었으며, 현재 저장소의 launcher와 saved-map wrapper는
`bash -n` 구문 확인을 통과했습니다. 새 옵션으로 전체 stack을 재실행하는 것은
배터리 교체 후 진행할 항목입니다.

지도 archive와 rosbag은 로컬 데이터이며 소스 GitHub 커밋에 큰 바이너리로
넣지 않았습니다. `.bak` 등 다른 작업 파일과 팔로워의 기존 미커밋 변경도 보존합니다.

## 팔로워 적용 위치

기존 `/home/kde/damgc_robot_mission/src/cooperative_transport` untracked 초안을
덮어쓰지 않기 위해 최신 공유 코드를 별도 overlay에 복사·빌드했습니다:

```text
/home/kde/coop_transport_runtime/src/cooperative_transport
/home/kde/coop_transport_runtime/install
```

실행할 때는 base workspace를 먼저, 이 overlay를 마지막으로 source합니다.
이번 기록의 팔로워 base branch 마지막 확인 commit은 `739a9d8`입니다.

```bash
source /opt/ros/humble/setup.bash
source /home/kde/damgc_robot_mission/install/setup.bash
source /home/kde/coop_transport_runtime/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch cooperative_transport manual_transport.launch.py \
  role:=follower follower_drive:=true use_stm32_bridge:=true
```

이 launch는 selector STOP / guard disabled / reverse enabled로 시작합니다.
기존 자동 grasp/lift mission과 함께 실행하지 않습니다.

## 리더 재부팅 후

새 `MAPPING_COOPERATIVE_ON_STARTUP=1` 옵션은 다음을 함께 적용합니다:

- host selector STOP, 자동 Nav2 goal source 선택 false.
- mapping manager `cooperative_on_startup=true`, 초기 HOLD / 협동 mode.
- leader peer는 IDLE. 새 경로를 받거나 READY라고 표시하며 시작하지 않음.

저장 당시 위치/방향에 놓았을 때의 체크포인트 복원 예:

```bash
cd /home/maze/damgc_robot
MAPPING_COOPERATIVE_ON_STARTUP=1 \
  ./scripts/run_saved_mapping.sh \
  /home/maze/damgc_robot/data/maps/battery_coop_checkpoint_20261004
```

원래 home 기준점에 놓는다면 그 물리 배치에 맞는 saved-map manifest를 선택해야
합니다. 로봇을 손으로 옮겼는데 battery checkpoint 초기 pose를 무조건 사용하면
지도와 실제 로봇이 어긋납니다. 이번 작업은 임의 위치의 자동 relocalization을 추가하지
않았습니다. 기존 `data/maps/latest` 링크는 자동으로 battery checkpoint로 바꾸지 않았습니다.

소스 업데이트 후 bringup 빌드가 필요하면:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cooperative_transport leader_command_selector \
  rescue_robot_bringup --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

Docker overlay에도 bringup/cooperative_transport 최신 소스를 빌드해야 합니다.
새 launch 자체를 위해 전체 맵을 삭제하거나 초기 회전을 자동 수행하지 않습니다.

## 다음 작업: 출발 전까지

1. 배터리 교체 후 map/odom 기준에 맞게 로봇 위치·방향을 잡습니다.
2. 양쪽 drive/peer를 실행하고 source STOP, guard disabled, odom 신선도를 확인합니다.
3. 작업자가 상자를 사이에 두고 반대 방향 / 중립 직선 힌지로 합체 완료를 알립니다.
4. 리더 키 창에서 **B**, `WAIT_PLAN` 확인 후 **새 RViz Nav2 goal**을 선택합니다.
5. geometry/costmap 검증과 session/hash 전달, follower READY 및 clock 응답을 확인합니다.
6. 터미널에 **COOP leader READY**가 떠야 'N만 남은 상태'입니다.
7. **N은 작업자가 누릅니다.** 이번 배터리 교체 인계에서는 누르지 않았습니다.

통신/기하 계약: [팔로워 인계](COOP_TRANSPORT_FOLLOWER_HANDOFF.md).
팔로워 요청 반영: [리더 응답](COOP_TRANSPORT_LEADER_RESPONSE.md).
