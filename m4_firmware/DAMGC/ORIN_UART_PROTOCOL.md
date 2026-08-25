# Orin–STM32 UART contract

## Link

- STM32 USART2: PA2 TX, PA3 RX
- 460800 baud, 8 data bits, no parity, 1 stop bit, no flow control
- 3.3 V UART; cross TX/RX and connect a common ground
- All integer and IEEE-754 float fields are little-endian

## Frame

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | Sync `AA 55` |
| 2 | 1 | Protocol version, currently `1` |
| 3 | 1 | Message type |
| 4 | 2 | Payload length |
| 6 | 2 | Global transmit sequence |
| 8 | 2 | Flags |
| 10 | N | Payload |
| 10+N | 2 | CRC-16/CCITT-FALSE |

CRC parameters are polynomial `0x1021`, initial value `0xFFFF`, no reflection,
and no final XOR. CRC covers bytes from version through the end of payload; it
does not cover sync or the CRC field.

## Orin to STM32

### `0x01 CMD_VELOCITY`, 8 bytes, `<hhHH`

1. left wheel target, mm/s (`int16`)
2. right wheel target, mm/s (`int16`)
3. watchdog, ms (`uint16`, zero selects 200 ms)
4. control flags (`uint16`)

Control flags: bit 0 motor enable, bit 1 controlled stop, bit 2 latched e-stop,
bit 3 clear fault. The Orin bridge transmits this frame at 50 Hz. Loss of valid
enabled commands stops the motor controller after the requested watchdog time.

## STM32 to Orin

### `0x10 IMU_DATA`, 52 bytes, `<Q6f4fhH`

- timestamp in microseconds since STM32 boot
- linear acceleration XYZ in m/s² (BNO055 gravity-removed output)
- angular velocity XYZ in rad/s
- quaternion XYZW
- temperature in centi-degrees Celsius
- status: BNO `CALIB_STAT` in bits 7:0, `SYS_STATUS` in bits 11:8, and
  `SYS_ERR` in bits 15:12

Nominal period: 9 ms.

### `0x11 WHEEL_STATE`, 34 bytes, `<QqqiiH`

- timestamp in microseconds since STM32 boot
- signed left and right cumulative encoder ticks (`int64`)
- signed left and right measured speeds in mm/s (`int32`)
- status bits: bit 0 sample valid, bit 1 speed PID active, bit 2 remote control active

Nominal period: 20 ms. Physical constants are 5131 ticks/revolution and a
127 mm wheel diameter (63.5 mm radius, 398.982 mm circumference).

### `0x12 SYSTEM_STATE`, 22 bytes, `<QHhhBBIH`

- timestamp, battery mV, battery mA, motor temperature in centi-degrees Celsius
- mode, e-stop state, fault bits, last command age in ms

Battery and motor-temperature fields are currently zero because those sensors
are not connected. Nominal period: 100 ms.

## Orin parameters to correct before odometry

The current `damgc_robot/src/stm32_bridge` defaults do not match this robot.
Set `wheel_radius_m` to `0.0635`, `ticks_per_revolution` to `5131`, and
`wheel_separation_m` to the measured wheel contact-center distance `0.23`.
The repository's current `0.0325`, `4096`, and `0.20` defaults do not match.

The STM32 sequence is global across IMU, wheel, and system frames. Packet-loss
diagnostics must therefore update their previous sequence for every valid frame,
not only for one message type.

Before fusion, verify the IMU mounting transform against ROS REP-103: X forward,
Y left, Z up, and positive yaw counter-clockwise when viewed from above.
