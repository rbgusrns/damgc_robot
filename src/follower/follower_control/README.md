# follower_control

팔로워 Orin에서 리더의 협동 속도 명령과 실제 구동 명령 사이에 두는 안전
경계입니다.

## 인터페이스

- 입력: `/follower/cmd_vel` (`geometry_msgs/msg/Twist`, reliable)
- 안전 출력: `/follower/safe_cmd_vel` (`geometry_msgs/msg/Twist`, reliable, 50 Hz)
- 상태: `/follower/command_connected` (`std_msgs/msg/Bool`)
- 상태 설명: `/follower/status` (`std_msgs/msg/String`)

입력의 `linear.x`, `angular.z`만 사용합니다. 기본 제한은 각각 `0.25 m/s`,
`0.8 rad/s`이며, 명령이 0.3초 이상 끊기거나 NaN/Inf가 입력되면 0 속도를
발행합니다. 이 watchdog은 팔로워 Orin에서 실행해야 하며 QoS 연결 여부와
무관하게 로컬 타이머로 정지를 결정합니다.

## 실행

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ros2_dds_env.sh
ros2 launch follower_control velocity_guard.launch.py
```

실제 모터를 연결하기 전 다음처럼 제한과 timeout을 검증할 수 있습니다.

```bash
ros2 topic pub -r 10 /follower/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.2}}"
ros2 topic echo /follower/safe_cmd_vel
ros2 topic echo /follower/status
```

publisher를 중지하면 0.3초 이내에 `/follower/safe_cmd_vel`이 0으로 바뀌어야 합니다.
현재 출력은 ROS 토픽까지만 구현되어 있으며 STM32 모터 bridge는 포함하지 않습니다.
