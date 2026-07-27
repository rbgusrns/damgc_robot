# leader_cooperation

리더 Orin에서 실행하는 ROS 2 DDS 협동 운반 노드입니다.

`/leader/cmd_vel`을 제한한 뒤, `/cooperation/enable` 서비스로 협동 운반이
활성화되고 `/follower/status` heartbeat가 신선할 때만
`/cooperation/target_velocity`와 `/follower/cmd_vel`로 전달합니다.
팔로워 heartbeat 또는 리더 명령이 timeout되면 0 속도를 발행합니다.

```bash
ros2 launch leader_cooperation leader_cooperation.launch.py
ros2 service call /cooperation/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /cooperation/state
```

현재 `/follower/status`는 `std_msgs/msg/String` heartbeat로 정의했습니다.
실제 배터리·fault 메시지 계약이 정해지면 별도 인터페이스 메시지로 교체해야 합니다.
