# STM32 UART bridge

The `stm32_bridge_node` receives STM32 binary frames from UART and publishes:

- `imu/data_raw` (`sensor_msgs/Imu`)
- `odom/raw` (`nav_msgs/Odometry`)
- `system_state` (`std_msgs/String`)
- `stm32_rx/frame_count`, `stm32_rx/crc_errors`, `stm32_rx/sequence_drops` (`std_msgs/UInt32`)

It also subscribes to `cmd_vel` and sends `CMD_VELOCITY` at 50 Hz. If the UART
device disappears, the node retries the connection every second.

## Hardware test

From the ROS workspace root (`damgc_robot`):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select stm32_bridge
source install/setup.bash
ros2 launch stm32_bridge stm32_bridge.launch.py \
  port:=/dev/ttyTHS1 baudrate:=460800 namespace:=leader
```

The default UART speed is `460800` baud. The STM32 UART must use the same
speed; `8-N-1` and hardware flow control disabled are expected. If an overlay
was built in another install directory, source that overlay instead, for
example `source /home/maze/stm32_bridge_install/setup.bash`.

On Jetson Orin systems, the 40-pin header is commonly exposed as
`/dev/ttyTHS1` on JetPack 6. Verify the actual mapping with
`ls -l /dev/ttyTHS*`; some carrier/JetPack combinations use `/dev/ttyTHS0`.

Check that packets are arriving:

```bash
ros2 topic echo /leader/stm32_rx/frame_count
ros2 topic echo /leader/imu/data_raw
ros2 topic echo /leader/odom/raw
ros2 topic echo /leader/system_state
```

The diagnostic topics report received frames and parser/link errors:

```bash
ros2 topic echo /leader/stm32_rx/frame_count
ros2 topic echo /leader/stm32_rx/crc_errors
ros2 topic echo /leader/stm32_rx/sequence_drops
```

`frame_count` should increase continuously. Occasional sequence gaps indicate
individual missing frames; increasing `crc_errors` points to baudrate,
wiring, grounding, or electrical noise. A verified Jetson test at `460800`
baud showed increasing frame counts with both `crc_errors` and
`sequence_drops` equal to zero.

The UART must be 3.3 V, 8-N-1, with STM32 TX connected to the adapter/Orin RX,
STM32 RX to TX, and a shared GND. Do not connect a 5 V UART signal directly.
