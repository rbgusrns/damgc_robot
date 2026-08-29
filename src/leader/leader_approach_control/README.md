# leader_approach_control

Leader의 검증된 `base_link` AprilTag pose와 alignment state를 사용해
`/leader/approach/cmd_vel_raw`을 계산하는 software-only controller다.

이 패키지에는 raw command를 최종 software topic으로 제한하는 독립
`/leader/velocity_guard` node도 포함된다.

STM32 UART 또는 motor output은 생성하지 않는다.
`max_raw_*`는 잘못된 gain이나 큰 오차로 controller candidate가 과도해지는 것을 막는
1차 saturation이다. 최종 `/leader/cmd_vel`의 안전 clamp, slew limit와 watchdog은
velocity guard가 담당한다.

Velocity guard가 `/leader/cmd_vel`의 clamp, slew limit와 watchdog을 담당한다.

## Interface

- Node: `/leader/approach_controller`
- Inputs:
  - `/leader/supply/base_relative_pose` (`geometry_msgs/msg/PoseStamped`)
  - `/leader/base_alignment/state` (`std_msgs/msg/String`)
  - `/leader/supply/detected` (`std_msgs/msg/Bool`)
  - `/leader/supply/tag_id` (`std_msgs/msg/Int32`)
- Output: `/leader/approach/cmd_vel_raw` (`geometry_msgs/msg/Twist`)
- Enable service: `/leader/approach/enable` (`std_srvs/srv/SetBool`)

Velocity guard interface:

- Node: `/leader/velocity_guard`
- Input: `/leader/approach/cmd_vel_raw` (`geometry_msgs/msg/Twist`)
- Output: `/leader/cmd_vel` (`geometry_msgs/msg/Twist`)
- Enable service: `/leader/velocity_guard/enable` (`std_srvs/srv/SetBool`)

Header가 없는 세 개의 base metric 토픽을 따로 조합하지 않는다. Controller는
`base_relative_pose` 한 sample의 x/y에서 forward, lateral, bearing을 함께 계산하고,
그 pose 직후 `sample_sync_tolerance` 안에 도착한 base state만 결합한다. Pose/state 순서가
뒤바뀌거나 source stamp 또는 local receipt time이 만료되면 zero command를 발행한다.

## Provisional parameters

`target_forward=0.25 m`, gains와 raw speed limits는 topic-level software validation용
임시값이다. 실제 grasp distance, gripper target, motor tuning 또는 최종 safety limit가
아니다. State와 controller가 같은 forward error 기준을 사용하도록
`rescue_robot_apriltag`의 `base_target_forward`와 이 패키지의 `target_forward`는 항상
같이 변경해야 한다.

## Build and test

```bash
source /opt/ros/humble/setup.bash
cd ~/damgc_robot
colcon build --symlink-install \
  --packages-select rescue_robot_apriltag leader_approach_control
source install/setup.bash
colcon test \
  --packages-select rescue_robot_apriltag leader_approach_control \
  --event-handlers console_direct+
```

Guard만 실행할 때는 다음 launch를 사용한다.

```bash
ros2 launch leader_approach_control velocity_guard.launch.py
```

## D435 topic-only validation

STM32 bridge, teleop, cooperation node와 motor power를 모두 끈 상태에서 실행한다.

```bash
ros2 launch rescue_robot_bringup camera_apriltag.launch.py enable_approach:=true
ros2 launch leader_approach_control approach_controller.launch.py
ros2 launch leader_approach_control velocity_guard.launch.py
```

Enable 전에 다른 터미널에서 hardware/control 충돌이 없는지 확인한다.

```bash
ros2 node list | grep -E 'stm32_bridge|arrow_key_teleop|leader_cooperation' || true
ros2 topic info /leader/cmd_vel --verbose
ros2 topic info /leader/approach/cmd_vel_raw --verbose
```

첫 명령은 아무것도 출력하지 않아야 한다. 최종 topic publisher는
`/leader/velocity_guard` 하나여야 하며, raw topic publisher는
`/leader/approach_controller` 하나여야 한다.

시작 시 controller는 disabled이므로 raw output은 zero다. 다음 service로 enable하면 기존
sample을 재사용하지 않고 다음 fresh pose/state pair부터 command를 계산한다.

```bash
ros2 service call /leader/approach/enable \
  std_srvs/srv/SetBool "{data: true}"

ros2 topic echo /leader/approach/cmd_vel_raw
```

Guard는 시작 시 disabled이므로 다음 명령으로 명시적으로 enable한다.

```bash
ros2 service call /leader/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: true}"

ros2 topic echo /leader/cmd_vel
```

Guard disabled 상태에서는 raw가 non-zero여도 최종 `/leader/cmd_vel`은 zero다.
raw publisher를 중지하면 `command_timeout` 후 final command가 zero가 된다.

확인할 결과:

- `TURN_LEFT`: `linear.x=0`, `angular.z>0`
- `TURN_RIGHT`: `linear.x=0`, `angular.z<0`
- `APPROACH`: `linear.x>0`, bearing에 따른 angular correction
- `FINE_ALIGN_LEFT/RIGHT`: `linear.x=0`, 낮은 angular correction
- `TAG_LOST`, `TOO_CLOSE`, `STABILIZING`, `ALIGNED`: 모든 축 zero

시험이 끝나면 먼저 disable한다.

```bash
ros2 service call /leader/approach/enable \
  std_srvs/srv/SetBool "{data: false}"

ros2 service call /leader/velocity_guard/enable \
  std_srvs/srv/SetBool "{data: false}"
```
