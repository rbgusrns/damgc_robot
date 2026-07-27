가장 단순한 방법은 두 Orin을 같은 공유기/스위치에 연결하고 ROS 2 DDS로 직접 통신시키는 것입니다. ROS 1처럼 별도 Master나 TCP 서버를 만들 필요는 없습니다. 같은 DDS domain의 노드는 자동으로 서로를 발견합니다.

## 리더와 팔로워의 역할

| 구분 | 리더 Orin | 팔로워 Orin |
|---|---|---|
| 네트워크 | 팔로워 IP로 ping | 리더 IP로 ping |
| DDS 설정 | 리더·팔로워와 동일한 값 설정 | 리더·팔로워와 동일한 값 설정 |
| 멀티캐스트 시험 | `send` 또는 `receive` | 리더와 반대 역할로 실행 |
| 협동 노드 | `leader_cooperation` 실행 | `/follower/status` heartbeat 발행 및 주행 노드 연결 |
| 주요 송신 토픽 | `/leader/cmd_vel`, `/cooperation/target_velocity` | `/follower/status` |
| 주요 수신 토픽 | `/follower/status` | `/follower/cmd_vel` |

아래 명령에서 `<리더_IP>`와 `<팔로워_IP>`는 각 장비에서 `hostname -I`로 확인한 주소로 바꿉니다.

## 1. 양쪽 공통 준비

리더와 팔로워의 모든 ROS 2 터미널에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/maze/damgc_robot
source install/local_setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

두 장비에서 다음 값이 같아야 합니다.

```bash
printenv ROS_DOMAIN_ID
printenv ROS_LOCALHOST_ONLY
printenv RMW_IMPLEMENTATION
```

IP 확인:

```bash
hostname -I
```

## 2. 리더 Orin에서 할 일

먼저 팔로워와 네트워크가 연결되는지 확인합니다.

```bash
ping -c 4 <팔로워_IP>
```

멀티캐스트 시험에서 리더를 송신자로 사용할 경우:

```bash
ros2 multicast send
```

팔로워가 보낸 테스트 토픽을 확인할 경우:

```bash
ros2 topic echo /test/orin_link_back
```

DDS 협동 노드를 실행합니다.

```bash
ros2 launch leader_cooperation leader_cooperation.launch.py
```

협동 운반을 활성화하고 상태를 확인합니다.

```bash
ros2 service call /cooperation/enable \
  std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /cooperation/state
```

## 3. 팔로워 Orin에서 할 일

먼저 리더와 네트워크가 연결되는지 확인합니다.

```bash
ping -c 4 <리더_IP>
```

멀티캐스트 시험에서 팔로워를 수신자로 사용할 경우:

```bash
ros2 multicast receive
```

리더가 발행한 테스트 토픽을 확인합니다.

```bash
ros2 topic echo /test/orin_link
```

팔로워 heartbeat와 실제 팔로워 주행 노드를 연결합니다. 현재 저장소에서는 heartbeat 메시지 계약을 `std_msgs/msg/String`으로 정의합니다.

```bash
ros2 topic pub -r 2 /follower/status \
  std_msgs/msg/String "{data: 'follower_alive'}"
```

리더의 명령 토픽 수신 여부는 다음으로 확인합니다.

```bash
ros2 topic echo /follower/cmd_vel
```

heartbeat 또는 리더 명령이 timeout되면 리더 협동 노드는 팔로워 속도를 0으로 발행합니다. 실제 운용에서는 위 `ros2 topic pub` 시험 명령 대신 팔로워 상태 노드가 heartbeat를 계속 발행해야 합니다.

## 4. 양방향 DDS 토픽 시험

리더에서 실행:

```bash
ros2 topic pub -r 2 /test/orin_link \
  std_msgs/msg/String "{data: 'leader_alive'}"
```

팔로워에서 확인:

```bash
ros2 topic echo /test/orin_link
```

팔로워에서 실행:

```bash
ros2 topic pub -r 2 /test/orin_link_back \
  std_msgs/msg/String "{data: 'follower_alive'}"
```

리더에서 확인:

```bash
ros2 topic echo /test/orin_link_back
```

양쪽에서 `leader_alive`, `follower_alive`가 보이면 기본 DDS 통신이 정상입니다.

## 5. 네트워크가 안 될 때 확인

- 두 Orin이 같은 공유기·스위치에 연결되어 있는지 확인합니다.
- 공유기의 AP isolation 또는 client isolation을 끕니다.
- `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, `RMW_IMPLEMENTATION` 값이 같은지 확인합니다.
- `ROS_LOCALHOST_ONLY`는 반드시 `0`이어야 합니다.
- 방화벽이 UDP 멀티캐스트를 차단하지 않는지 확인합니다.

## 프로젝트 통신 구조

통신이 확인되면 다음 구조로 연결합니다.
리더 Orin
 ├─ /mission/state
 ├─ /cooperation/state
 └─ /cooperation/target_velocity
                 ↓
팔로워 Orin
 ├─ /follower/status
 ├─ /follower/odom
 ├─ /follower/imu
 └─ /follower/alignment/state
연속 데이터는 topic, 즉시 확인이 필요한 단발성 명령은 service, 이동처럼 오래 걸리고 취소가 필요한 임무는 action이 적합합니다. ROS 2 topic/service/action 구분
중요한 현재 문제
현재 저장소의 리더와 팔로워 AprilTag 노드를 같은 ROS graph에서 동시에 실행하면 tag36h11:0 같은 TF frame 이름이 충돌할 수 있습니다.
따라서 전체 launch를 동시에 켜기 전에 최소한 다음처럼 frame을 분리해야 합니다.
leader/base_link
leader/camera_color_optical_frame
leader/tag36h11:0

follower/base_link
follower/follower_camera_optical_frame
follower/tag36h11:0
먼저 /test/orin_link로 통신만 검증하고, TF 이름을 분리한 다음 실제 카메라·AprilTag 파이프라인을 같이 실행하는 것이 안전합니다.
또한 /cmd_vel을 연결할 때는 통신이 끊기면 팔로워가 자체적으로 정지하는 watchdog이 반드시 있어야 합니다. QoS만으로 정지를 보장하면 안 됩니다. 센서 데이터에는 일반적으로 최신성을 우선하는 best-effort가, 임무 상태에는 reliable이 적합하며 publisher와 subscriber의 QoS가 호환되어야 합니다
