# Leader Command Selector

Leader의 최종 `/leader/cmd_vel` source를 하나로 선택한다.

| Mode | Input |
|---|---|
| `STOP` | 항상 zero |
| `TELEOP` | `/leader/teleop/cmd_vel` |
| `APPROACH` | `/leader/approach/cmd_vel_safe` |
| `NAV2` | `/nav2/cmd_vel` |

Output은 `/leader/cmd_vel`, status는 transient-local
`/leader/command_selector/status` (`std_msgs/msg/String`)다. Generic launch default는 `STOP`이다.

```bash
ros2 param get /leader/command_selector source_mode
ros2 param set /leader/command_selector source_mode STOP
ros2 param set /leader/command_selector source_mode TELEOP
ros2 param set /leader/command_selector source_mode APPROACH
ros2 param set /leader/command_selector source_mode NAV2
ros2 topic echo /leader/command_selector/status \
  --qos-durability transient_local
```

Mode 변경 시 모든 command cache를 버리고 zero를 즉시 발행한다. 선택된 source에서 fresh
command를 받기 전까지 zero를 유지한다. Source별 timeout, 잘못된 Twist와 shutdown은 zero로
fail closed한다. Status는 `STOP`, `WAITING_*`, `ACTIVE_*`, `STALE_*`를 보고한다.

NAV2 모드는 `/leader/odom/raw`의 유효한 `odom -> base_link` 측정과 최신 수신을
확인한다. header 또는 수신 시각이 0.5초 이상 오래되거나 frame/pose가 잘못되면
출력을 zero로 유지하고 `NAV2_WHEEL_ODOM_*` 상태를 기록한다. 정상 wheel 측정과 fresh
명령이 다시 들어오면 진행한다. VSLAM 누락이나 wheel/VSLAM 변위 차이는 정지 조건이 아니다.

`nav_wheel_guard_enabled`와 `nav_wheel_odom_timeout`은 시작 시 설정한다. 이 검사는
wheel slip이나 장애물 충돌을 검출하는 장치가 아니다.
