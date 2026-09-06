# 리더·팔로워 협동 이동 실행 가이드

## 범위

이 실행기는 기존 방향키 이동과 STM32 bridge를 재사용하고, 두 Orin의 DDS 연결과
협동 명령 전달을 한 경로로 묶는다. 현재는 리더 명령을 팔로워에 그대로 전달하는
open-loop 방식이며 wheel odometry 기반 상대 오차 보정은 포함하지 않는다.

```text
Leader arrow-key teleop -> /leader/cmd_vel -> leader_cooperation
                                           -> /follower/cmd_vel (DDS)
Follower command selector (COOPERATION) -> selected velocity guard
                                        -> /follower/safe_cmd_vel
                                        -> STM32 bridge -> motors
```

두 장비에서 같은 커밋을 빌드하고 동일한 `ROS_DOMAIN_ID`를 사용해야 한다. 팔로워를
먼저 실행한다.

## 1. 네트워크 전용 점검

모터 없이 DDS 경로부터 확인하려면 양쪽 모두 STM32 bridge를 끈다.

Follower Orin:

```bash
cd ~/damgc_robot
ROS_DOMAIN_ID=42 COOP_USE_STM32_BRIDGE=0 \
  ./scripts/run_cooperative_transport.sh follower
```

Leader Orin:

```bash
cd ~/damgc_robot
ROS_DOMAIN_ID=42 COOP_USE_STM32_BRIDGE=0 \
  COOP_PEER_IP=192.168.0.7 \
  ./scripts/run_cooperative_transport.sh leader
```

Leader 실행기는 peer ping을 확인하고 `/follower/status` heartbeat를 기다린다. heartbeat가
발견되어야 `/cooperation/enable`을 호출하고 기존 방향키 teleop을 시작한다. 제한 시간 안에
발견하지 못하면 주행기로 진입하지 않고 종료한다.

## 2. 실제 장비 실행

첫 시험은 두 로봇의 바퀴를 지면에서 띄우고 물리 비상정지를 준비한다.

Follower Orin:

```bash
ROS_DOMAIN_ID=42 COOP_PEER_IP=192.168.0.6 \
  ./scripts/run_cooperative_transport.sh follower
```

Leader Orin:

```bash
ROS_DOMAIN_ID=42 COOP_PEER_IP=192.168.0.7 \
  ./scripts/run_cooperative_transport.sh leader
```

팔로워 실행기는 selector를 `COOPERATION`으로 설정하지만 최종 velocity guard는 닫힌
상태로 시작한다. Leader가 `COOPERATING` 상태이고 양쪽 command topic이 정상임을 확인한
후 Follower의 별도 터미널에서만 다음을 호출한다.

```bash
source /opt/ros/humble/setup.bash
source ~/damgc_robot/install/setup.bash
source ~/damgc_robot/scripts/ros2_dds_env.sh
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"
```

정지는 Follower guard를 먼저 닫은 뒤 Leader 방향키 터미널에서 Space를 누른다.

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

그 다음 양쪽 실행기에서 `Ctrl-C`한다. Leader 실행기는 종료하면서
`/cooperation/enable=false`를 호출하고 background launch를 정리한다.

## 환경 변수

| 변수 | 기본값 | 의미 |
|---|---:|---|
| `ROS_DOMAIN_ID` | `42` | 양쪽에서 동일해야 하는 DDS domain |
| `COOP_PEER_IP` | 미설정 | 설정 시 시작 전에 peer ping 검사 |
| `COOP_USE_STM32_BRIDGE` | `1` | `0`이면 네트워크·ROS 경로만 시험 |
| `COOP_I2C_DEVICE` | `/dev/i2c-7` | 역할별 STM32 I2C device |
| `COOP_I2C_ADDRESS` | `66` | STM32 7-bit address |
| `COOP_I2C_WRITE_ENABLED` | `1` | `0`이면 bridge receive-only |
| `COOP_DISCOVERY_TIMEOUT` | `30` | Leader의 heartbeat 대기 시간(초) |

현재 heartbeat는 구조화된 fault가 아니라 `/follower/status`의 주기적 String이다. 실제
협동 운반 전에는 양쪽 모터 부호, timeout 정지, 동시 E-stop과 물체 파지 상태를 별도로
검증해야 한다.
