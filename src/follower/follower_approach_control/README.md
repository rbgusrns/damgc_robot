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

The Follower final target remains `0.25 m` in perception. The controller's
`target_forward` parameter is retained only for existing launch/config
compatibility; the authoritative control error is in the atomic command.
`TAG_LOST`, stale data, invalid pose/state combinations, and all non-motion
states produce zero velocity. Blind-final commands are accepted only as the
exact `BLIND_FINAL_APPROACH` mode with `FINAL_APPROACH` state.

전체 pipeline, 파라미터와 재현 절차는
[`follower_supply_perception` validation guide](../follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를
참고합니다.
