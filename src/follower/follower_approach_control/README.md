# follower_approach_control

This package converts one coherent
`follower_alignment_msgs/FollowerAlignmentCommand` generation into the
software-only raw topic `/follower/approach/cmd_vel_raw`. The message carries
the target pose, control mode, and alignment state together, so the controller
cannot combine a new pose with an older state callback.

It does not select command ownership, publish `/follower/cmd_vel`, apply the final
safety guard, or communicate with STM32/motors.

The controller starts disabled. Enable it with:

```bash
ros2 service call /follower/approach/enable \
  std_srvs/srv/SetBool "{data: true}"
```

Enable 상태는 `/follower/approach/enabled` (`std_msgs/msg/Bool`)에도 발행됩니다.
Perception은 enable/disable 전환을 새 approach session 경계로 사용해 이전
`ALIGNED` latch와 안정화/tag-loss grace 이력을 초기화합니다. 전환 시 controller는
cached command를 폐기하고 즉시 zero `Twist`를 발행한 뒤 fresh coherent command를
기다립니다.

The Follower final target remains `0.25 m` in perception. The controller's
`target_forward` parameter is retained only for existing launch/config
compatibility; the authoritative control error is in the atomic command.
`TAG_LOST`, stale data, invalid pose/state combinations, and all non-motion
states produce zero velocity. Blind-final commands are accepted only as the
exact `BLIND_FINAL_APPROACH` mode with `FINAL_APPROACH` state.

`FINAL_APPROACH` tag-loss grace에서는 state와 mode 이름은 유지되지만 detection/tag가
invalid이고 pose가 비어 있으므로 controller 출력은 zero입니다. Grace 자체는
blind-forward 권한을 뜻하지 않습니다.

전체 pipeline, 파라미터와 재현 절차는
[`follower_supply_perception` validation guide](../follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를
참고합니다.
