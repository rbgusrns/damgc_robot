가장 단순한 방법은 두 Orin을 같은 공유기/스위치에 연결하고 ROS 2 DDS로 직접 통신시키는 것입니다. ROS 1처럼 별도 Master나 TCP 서버를 만들 필요는 없습니다. 같은 DDS domain의 노드는 자동으로 서로를 발견합니다. ROS 2 DDS 구조
1. 두 Orin의 네트워크 확인
예시:
리더 Orin:   192.168.10.11
팔로워 Orin: 192.168.10.12
리더에서:
hostname -I
ping -c 4 192.168.10.12
팔로워에서:
hostname -I
ping -c 4 192.168.10.11
양쪽 모두 ping이 되어야 합니다. Wi-Fi를 사용한다면 공유기의 AP isolation 또는 client isolation 기능은 꺼져 있어야 합니다.
2. 양쪽 ROS 환경을 동일하게 설정
두 Orin의 모든 터미널에서:
source /opt/ros/humble/setup.bash
source /path/to/damgc_robot/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
핵심은 다음 세 값이 양쪽에서 같아야 한다는 것입니다.
printenv ROS_DOMAIN_ID
printenv ROS_LOCALHOST_ONLY
printenv RMW_IMPLEMENTATION
ROS_DOMAIN_ID가 같은 노드끼리만 서로 발견합니다. 다른 ROS 장비와 섞이지 않도록 팀 전용 번호를 정하면 됩니다. ROS_DOMAIN_ID 설명
시험에 성공한 다음 위 환경변수를 양쪽 ~/.bashrc에 넣으면 됩니다.
3. DDS multicast부터 확인
팔로워 Orin:
ros2 multicast receive
리더 Orin:
ros2 multicast send
팔로워에 Hello World!가 표시되어야 합니다. 반대 방향도 시험합니다.
리더: ros2 multicast receive
팔로워: ros2 multicast send
이 명령은 DDS 노드 검색에 필요한 UDP multicast가 두 장비 사이를 통과하는지 확인합니다. ROS 2 multicast 도구
4. 실제 ROS 토픽 시험
리더 Orin:
ros2 topic pub -r 2 /test/orin_link \
  std_msgs/msg/String "{data: 'leader_alive'}"
팔로워 Orin:
ros2 topic echo /test/orin_link
다음과 같이 나오면 리더 → 팔로워 통신이 된 것입니다.
data: leader_alive
반대 방향도 확인합니다.
팔로워:
ros2 topic pub -r 2 /test/orin_link_back \
  std_msgs/msg/String "{data: 'follower_alive'}"
리더:
ros2 topic echo /test/orin_link_back
5. 프로젝트에 적용
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