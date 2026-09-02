# STM32 I2C bridge

The `stm32_bridge_node` receives STM32 binary frames from I2C and publishes:

- `imu/data_raw` (`sensor_msgs/Imu`)
- `odom/raw` (`nav_msgs/Odometry`)
- `system_state` (`std_msgs/String`)
- `stm32_rx/frame_count`, `stm32_rx/poll_count`, `stm32_rx/empty_poll_count`,
  `stm32_rx/crc_errors`, `stm32_rx/sequence_drops` (`std_msgs/UInt32`)

It also subscribes to `cmd_vel` and sends `CMD_VELOCITY` at 50 Hz. If the I2C
device disappears, the node retries the connection every second. The original
UART transport remains available as an explicit fallback.

## Hardware test

From the ROS workspace root (`damgc_robot`):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select stm32_bridge
source install/setup.bash
ros2 launch stm32_bridge stm32_bridge.launch.py \
  transport:=i2c i2c_device:=/dev/i2c-7 i2c_address:=66 \
  i2c_write_enabled:=true namespace:=leader
```

The defaults select Jetson Orin 40-pin header pins 3 (SDA) and 5 (SCL), Linux
device `/dev/i2c-7`, and STM32 7-bit address `0x42` (decimal ROS parameter
value `66`). Each poll reads one fixed 66-byte queue slot in a single I2C
transaction: a two-byte mailbox header followed by up to 64 bytes of framed
protocol data. The first header byte selects the valid frame length. Reading
the complete slot atomically avoids producer races and pops the STM32 queue
head. The `i2c_read_size` parameter is the maximum accepted frame size (64
bytes by default). `i2c_poll_hz` defaults to 500 Hz, leaving scheduling margin above the
STM32 queue's required minimum drain rate of 200 Hz at 400 kHz. I2C command
writes default to enabled for the validated bidirectional STM32 firmware.
Disable them with `i2c_write_enabled:=false` for receive-only diagnostics. If
an overlay was built in another install directory, source that overlay instead,
for example `source /home/maze/stm32_bridge_install/setup.bash`.

The mapping launcher passes the I2C settings explicitly and enables command
writes by default:

```bash
./scripts/run_vslam_mapping.sh
```

For a receive-only diagnostic run, disable command writes with:

```bash
STM32_I2C_WRITE_ENABLED=0 ./scripts/run_vslam_mapping.sh
```

`STM32_I2C_DEVICE`, `STM32_I2C_ADDRESS`, and `STM32_I2C_POLL_HZ` can override
`/dev/i2c-7`, `66`, and `500.0`.

## STM32 firmware contract

The Orin side is complete when the STM32 implements these transactions at
7-bit address `0x42`:

- A 66-byte read returns `[frame_length, generation_or_status,
  complete_frame..., padding...]` and pops exactly one queue entry.
- A write contains exactly one complete binary protocol frame, starting with
  `AA 55`. For velocity commands it is 20 bytes total: header with message type
  `0x01`, 8-byte little-endian payload `<left_mm_s, right_mm_s, watchdog_ms,
  control_flags>`, then CRC16-CCITT-FALSE over bytes from version through the
  end of the payload.

The STM32 must reject bad version/length/CRC, refresh its watchdog only after a
valid `CMD_VELOCITY`, and stop both motors when the watchdog expires. It should
also update telemetry sequence and timestamp on every published frame; a
constant frame is intentionally de-duplicated by the ROS bridge.

For the UART fallback, launch with `transport:=uart port:=/dev/ttyTHS1
baudrate:=230400`. UART expects 3.3 V, 8-N-1, hardware flow control disabled,
and a shared GND.

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
individual missing frames; increasing `crc_errors` points to framing, wiring,
grounding, or electrical noise.

The I2C bus must use 3.3 V logic, a shared GND, and suitable SDA/SCL pull-ups.
Do not connect 5 V I2C signals directly to the Jetson header.
