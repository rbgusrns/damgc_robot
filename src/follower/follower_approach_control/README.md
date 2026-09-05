# follower_approach_control

This package converts one coherent Follower base-frame AprilTag pose/state sample
into the software-only raw topic `/follower/approach/cmd_vel_raw`.

It does not select command ownership, publish `/follower/cmd_vel`, apply the final
safety guard, or communicate with STM32/motors.

The controller starts disabled. Enable it with:

```bash
ros2 service call /follower/approach/enable \
  std_srvs/srv/SetBool "{data: true}"
```

`target_forward=0.25 m` is the configured approach stop target matching the
Leader. It must remain equal to `follower_supply_perception`'s
`base_target_forward`; controller gains and raw limits remain conservative
software-validation values.

전체 pipeline, 파라미터와 재현 절차는
[`follower_supply_perception` validation guide](../follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)를
참고합니다.
