# follower_command_selector

This package gives explicit ownership of `/follower/selected_cmd_vel` to exactly
one input source:

- `STOP`: always zero
- `APPROACH`: `/follower/approach/cmd_vel_raw`
- `COOPERATION`: existing `/follower/cmd_vel`

Select the source through the existing ROS parameter service:

```bash
ros2 param set /follower/command_selector source_mode APPROACH
```

A string parameter is used because `std_srvs/SetBool` cannot represent three
mutually exclusive states safely. A custom message/service would add unnecessary
interface and build dependencies. Source changes clear all caches, publish zero,
and require a new command received after the switch.

This package does not modify the cooperation publisher, final guard, STM32, or
motor interfaces. The integrated guard consumes `/follower/selected_cmd_vel` and
publishes the software-only final topic `/follower/safe_cmd_vel`.

See the
[`Follower pipeline validation guide`](../follower_supply_perception/docs/FOLLOWER_BASE_LINK_VELOCITY_PIPELINE_VALIDATION_GUIDE.md)
for the complete ownership and fault-validation procedure.
