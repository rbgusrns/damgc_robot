# follower_control

Follower의 최종 software velocity safety boundary다. STM32/UART/motor output은
이 패키지 범위에 포함하지 않는다.

## 보존된 public interface

- 최종 출력: `/follower/safe_cmd_vel` (`geometry_msgs/msg/Twist`)
- command freshness: `/follower/command_connected` (`std_msgs/msg/Bool`)
- heartbeat/status: `/follower/status` (`std_msgs/msg/String`)
- enable service: `/follower/velocity_guard/enable` (`std_srvs/srv/SetBool`)

`command_connected=true`와 `status=ACTIVE`는 enable gate와 별개로 유효하고 신선한
upstream command를 수신 중이라는 기존 의미를 유지한다. `status`는 command가 없거나
timeout이면 `READY` heartbeat를 계속 발행한다.

## 두 입력 모드

기존 cooperation standalone 경로는 그대로 유지한다.

```text
/follower/cmd_vel -> velocity_guard -> /follower/safe_cmd_vel
```

```bash
ros2 launch follower_control velocity_guard.launch.py
```

AprilTag/selector 통합 모드에서는 guard의 parameterized input만 변경한다.

```text
/follower/selected_cmd_vel -> velocity_guard -> /follower/safe_cmd_vel
```

```bash
ros2 launch follower_control selected_velocity_guard.launch.py
```

두 guard launch를 동시에 실행하면 안 된다. 어느 모드에서도
`/follower/safe_cmd_vel` publisher는 guard 하나여야 한다.

## Safety contract

- startup disabled; enable 전에는 항상 final zero
- enable/disable 전환 시 command cache 삭제 후 즉시 zero
- enable 후 새로운 upstream sample이 도착해야 출력 재개
- 모든 Twist 축 finite 검사
- `linear.y/z`, `angular.x/y` 사용 시 즉시 reject/zero
- reverse 기본 금지; 음의 `linear.x`는 reject/zero
- linear/angular symmetric clamp
- linear/angular acceleration/deceleration slew limit
- monotonic watchdog 및 slew timing, 비정상 dt fail-closed
- 기존 0.3초 local command watchdog 유지
- invalid, timeout, publisher loss 시 즉시 zero
- shutdown 시 configurable zero burst

기본 speed clamp `0.25 m/s`, `0.8 rad/s`는 기존 cooperation compatibility를 위해
유지했다. acceleration limit을 포함한 모든 값은 motor tuning 최종값이 아닌
software topic-level validation용 임시값이다.

Guard enable:

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"
```

Disable:

```bash
ros2 service call /follower/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```

전체 AprilTag·cooperation 통합 흐름과 재현 검증은
[`Follower pipeline validation guide`](../follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를
참고합니다.
