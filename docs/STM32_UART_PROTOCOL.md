# Orin–STM32 UART 통신 규격 초안

상태: 1차 설계안

이 문서는 각 로봇의 Jetson Orin과 STM32G474VET6 사이의 유선 UART 통신을
정의한다. 리더와 팔로워는 같은 규격을 사용하고, ROS namespace는 UART
패킷에 포함하지 않는다.

## 1. 전송 규칙

- 물리 계층: 3.3 V UART, 921600 baud, 8-N-1
- 바이트 순서: little-endian
- 멀티바이트 정수: 고정 폭 정수 사용
- 실수: IEEE-754 `float32`를 사용하되, 제어·상태 값은 가능한 한 정수 단위 사용
- 프레임 경계: sync + header의 payload 길이 + CRC로 결정
- 명령 timeout: 마지막 유효 `CMD_VELOCITY` 수신 후 200 ms
- timeout 또는 CRC 오류가 연속되면 STM32는 모터를 0 속도로 출력
- 하드웨어 E-stop은 UART와 독립적으로 모터 출력을 차단

## 2. 바이너리 프레임

모든 패킷은 다음 형식이다. `payload_len`은 payload 바이트 수이며, CRC는
`sync`를 제외한 `version`부터 payload 끝까지 계산한다.

| 필드 | 크기 | 설명 |
| --- | ---: | --- |
| `sync` | 2 | `0xAA 0x55` |
| `version` | 1 | 현재 `1` |
| `msg_type` | 1 | 메시지 종류 |
| `payload_len` | 2 | 0–512 bytes |
| `seq` | 2 | 송신자별 증가 sequence, wrap 허용 |
| `flags` | 2 | 메시지별 상태 플래그 |
| `payload` | N | 메시지 payload |
| `crc16` | 2 | CRC-16/CCITT-FALSE, 초기값 `0xFFFF` |

수신기는 sync를 다시 찾을 수 있어야 하며, 길이 초과·버전 불일치·CRC 오류
패킷은 폐기한다. `seq`가 끊긴 경우 상태 진단에 기록하지만, 센서 데이터는
timestamp 기준으로 처리한다.

## 3. 메시지 종류

| 값 | 이름 | 방향 | 주기 |
| ---: | --- | --- | ---: |
| `0x01` | `CMD_VELOCITY` | Orin → STM32 | 50 Hz |
| `0x02` | `CMD_GRIPPER` | Orin → STM32 | 이벤트 |
| `0x03` | `CMD_ESTOP_RESET` | Orin → STM32 | 이벤트 |
| `0x10` | `IMU_DATA` | STM32 → Orin | 100–200 Hz |
| `0x11` | `WHEEL_STATE` | STM32 → Orin | 50 Hz |
| `0x12` | `SYSTEM_STATE` | STM32 → Orin | 10–50 Hz |
| `0x20` | `TIME_SYNC_RESPONSE` | STM32 → Orin | 요청 응답 |
| `0x21` | `TIME_SYNC_REQUEST` | Orin → STM32 | 1 Hz |
| `0x7F` | `ACK_NACK` | 양방향 | 필요 시 |

## 4. Payload 정의

### 4.1 `CMD_VELOCITY` (`0x01`)

| 필드 | 타입 | 단위 |
| --- | --- | --- |
| `left_mm_s` | `int16` | 좌측 바퀴 목표 선속도 |
| `right_mm_s` | `int16` | 우측 바퀴 목표 선속도 |
| `watchdog_ms` | `uint16` | 명령 유효기간, 기본 200 |
| `control_flags` | `uint16` | enable/reset 등의 명령 |

`control_flags`:

- bit 0: motor enable
- bit 1: controlled stop
- bit 2: emergency stop request
- bit 3: clear latched fault

### 4.2 `IMU_DATA` (`0x10`)

모든 IMU 값의 기준 frame은 `imu_link`이며, 값은 측정 시점의 MCU monotonic
clock 기준이다.

| 필드 | 타입 | 단위 |
| --- | --- | --- |
| `timestamp_us` | `uint64` | STM32 monotonic timestamp |
| `ax, ay, az` | `float32[3]` | m/s² |
| `gx, gy, gz` | `float32[3]` | rad/s |
| `qx, qy, qz, qw` | `float32[4]` | quaternion, 유효하지 않으면 NaN 금지 대신 status로 표시 |
| `temperature_cdeg` | `int16` | 0.01 °C |
| `imu_status` | `uint16` | sensor/calibration 상태 |

`imu_status`에는 BNO055 system/calibration 상태와 데이터 유효 비트를 포함한다.
EKF에 사용할 때 orientation이 보정되지 않았으면 orientation covariance를
크게 하거나 해당 측정을 제외한다.

### 4.3 `WHEEL_STATE` (`0x11`)

Orin은 누적 encoder tick으로 wheel odometry를 계산한다. 로봇 기하 파라미터
(wheel radius, wheel separation, ticks/rev)는 STM32 패킷에 넣지 않고 ROS 설정에
둔다.

| 필드 | 타입 | 단위 |
| --- | --- | --- |
| `timestamp_us` | `uint64` | 측정 시점 |
| `left_ticks` | `int64` | 누적 tick |
| `right_ticks` | `int64` | 누적 tick |
| `left_mm_s` | `int32` | 측정 바퀴 속도 |
| `right_mm_s` | `int32` | 측정 바퀴 속도 |
| `encoder_status` | `uint16` | overflow/disconnect 상태 |

### 4.4 `SYSTEM_STATE` (`0x12`)

| 필드 | 타입 | 단위 |
| --- | --- | --- |
| `timestamp_us` | `uint64` | STM32 monotonic timestamp |
| `battery_mv` | `uint16` | mV |
| `battery_ma` | `int16` | mA |
| `motor_temp_cdeg` | `int16` | 0.01 °C |
| `mode` | `uint8` | boot/idle/manual/auto/fault |
| `estop_state` | `uint8` | released/pressed |
| `fault_bits` | `uint32` | 모터·IMU·encoder·통신 fault |
| `last_cmd_age_ms` | `uint16` | 마지막 명령 이후 경과 |

## 5. 시간 동기화

STM32의 `timestamp_us`는 부팅 후 증가하는 monotonic clock이다. Orin bridge는
수신 시각과 timestamp를 이용해 offset을 추정하고 ROS header stamp로 변환한다.
IMU와 encoder를 Orin 수신 시각으로 각각 timestamp하면 지연 차이가 생기므로
사용하지 않는다.

`TIME_SYNC_REQUEST`에는 Orin monotonic 송신 시각 `t1`을 넣고, 응답에는 STM32
수신 시각 `t2`, 송신 시각 `t3`를 넣는다. Orin은 왕복 지연을 이용해 offset을
추정하며, 최소 1 Hz로 갱신한다.

## 6. ROS bridge 출력

리더 기준 예시는 다음과 같다.

| UART 데이터 | ROS 출력 |
| --- | --- |
| `IMU_DATA` | `/leader/imu/data_raw` (`sensor_msgs/Imu`) |
| `WHEEL_STATE` | `/leader/wheel/state` 또는 `/leader/odom/raw` |
| `SYSTEM_STATE` | `/leader/system_state` |

bridge가 `WHEEL_STATE`에서 계산한 odometry와 IMU는 이후
`robot_localization` EKF 입력으로 사용한다. 센서 frame은 각각 `imu_link`와
`base_link`이며, `base_link → imu_link`의 정적 TF는 실제 장착 측정값으로
설정한다.

## 7. 구현 순서

1. STM32에서 frame parser, CRC, sequence, timeout만 먼저 구현
2. `SYSTEM_STATE` heartbeat 송수신 시험
3. encoder tick과 BNO055 raw 데이터를 각각 송신
4. Orin bridge에서 ROS `Imu`와 wheel state 발행
5. timestamp/sequence/dropout 시험
6. 모터 명령 연결은 마지막에 하고, timeout·E-stop을 먼저 검증

