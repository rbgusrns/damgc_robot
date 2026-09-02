# STM32 I2C·3D mapping 통합 진행 기록 — 2026-09-02

## 결과 요약

Orin과 STM32 사이의 ROS 2 bridge를 UART 기본 경로에서 I2C 기본 경로로
전환했다. STM32 telemetry 수신, `CMD_VELOCITY` 송신, wheel odometry, IMU,
dual EKF, Isaac ROS Visual SLAM, nvblox, RViz와 방향키 teleop을 한 실행기에
연결했다.

기본 설정은 다음과 같다.

- Linux I2C device: `/dev/i2c-7`
- STM32 7-bit address: `0x42` (`66`)
- bus speed: 400 kHz
- queue slot: 66 bytes (`length`, `generation/status`, frame 64 bytes)
- host polling: 500 Hz
- command write: 기본 활성화
- command rate / timeout: 50 Hz / 200 ms

## I2C queue 수신 수정

초기 host 구현은 한 바이트 길이를 읽고 실제 길이만큼 다시 읽었다. STM32
queue는 전체 66-byte slot을 clock한 뒤에만 head를 pop하므로 이 방식에서는 같은
frame이 반복 관측됐다. telemetry가 갱신되기 시작한 뒤에는 두 transaction 사이에
queue head가 바뀌어 `46 -> 34 -> 64` mailbox length race도 발생했다.

현재 구현은 poll마다 66 bytes를 단일 I2C transaction으로 읽고 첫 바이트의
`frame_length`만큼만 parser에 전달한다. 이 방식으로 queue pop과 producer race를
동시에 해결했다.

STM32 queue는 400 kHz에서 최소 200 Hz drain이 필요하다. Python/ROS timer jitter를
고려해 기본 `i2c_poll_hz`를 500 Hz로 설정했다. 누적 poll counter로 실제 read가
500 Hz 수준으로 진행되는 것도 확인했다.

진단 토픽:

- `/leader/stm32_rx/frame_count`
- `/leader/stm32_rx/poll_count`
- `/leader/stm32_rx/empty_poll_count`
- `/leader/stm32_rx/crc_errors`
- `/leader/stm32_rx/sequence_drops`

## 모터 명령 검증

I2C write를 활성화한 bridge에서 `/leader/cmd_vel`에 `0.03 m/s` 전진 명령을 한 번
발행했다. bridge command timeout은 200 ms이다.

- command 전 odom x: `0.00000 m`
- command 후 odom x: `0.00521 m`
- 1초 후 wheel odom linear/angular velocity: 모두 0
- CRC error: 0

따라서 Orin `Twist` -> I2C `CMD_VELOCITY` -> STM32 motor control -> encoder
telemetry -> ROS odometry의 왕복 경로와 timeout 정지를 확인했다.

## 매핑 실행기 수정

`scripts/run_vslam_mapping.sh`는 bridge를 다음 값으로 명시 실행한다.

- `transport:=i2c`
- `i2c_device:=/dev/i2c-7`
- `i2c_address:=66`
- `i2c_poll_hz:=500.0`
- `i2c_write_enabled:=true`

환경 변수 `STM32_I2C_WRITE_ENABLED`는 사용 편의를 위해 `0/1`을 받지만 ROS launch
parameter에는 `false/true`로 변환한다. 숫자 `1`을 그대로 넘겨 bridge가
`InvalidParameterTypeException`으로 종료되던 문제를 수정했다.

수신 전용 진단 실행:

```bash
STM32_I2C_WRITE_ENABLED=0 ./scripts/run_vslam_mapping.sh
```

방향키 teleop의 escape sequence 입력과 non-zero `Twist` 발행도 별도 test topic에서
확인했다. 조종할 때는 RViz가 아니라 `Arrow-key control`이 표시된 터미널에 focus가
있어야 한다.

## 2026-09-02 통합 기록

결과 위치:

- log: `log/vslam_mapping_20260902_225400`
- bag: `data/vslam_mapping_20260902_225400`
- analysis: `data/vslam_mapping_20260902_225400/analysis.md`

약 321.8초 기록의 주요 수치는 다음과 같다.

| 항목 | 결과 |
|---|---:|
| Wheel odometry | 9443 samples, 46.9 Hz |
| IMU | 103.4 Hz |
| VSLAM tracking odometry | 4813 samples, 15.0 Hz |
| VSLAM tracking success | 100.0% |
| Wheel path / net displacement | 0.236 m / 0.049 m |
| VSLAM path / net displacement | 9.159 m / 7.317 m |

RealSense는 USB 3.2로 연결됐다. VSLAM tracking은 끊기지 않았지만 wheel과 VSLAM
경로 차이가 매우 크므로 이번 기록은 데이터 경로와 통합 기동 검증으로만 사용한다.
주행 정확도 검증 완료 결과로 보지 않는다.

## 남은 확인 사항

1. VSLAM `image_jitter_threshold_ms=22`보다 큰 33~333 ms frame delta 경고와 최대
   tracking gap 약 862 ms의 원인을 확인한다.
2. 직선 1 m와 제자리 360도 시험으로 wheel/VSLAM scale과 camera extrinsic을 검증한다.
3. 동일한 이동체에 camera와 wheel이 고정된 상태에서 폐루프 주행을 다시 기록한다.
4. 시작 backlog 이후에도 관측되는 I2C sequence gap의 빈도와 queue overflow counter를
   장시간 측정한다.
5. nvblox mesh/3D map 저장 경로를 추가하고 실제 map 품질을 별도로 평가한다.

## 검증

- STM32 bridge protocol/transport unit tests: 9 passed
- `stm32_bridge` colcon build: passed
- mapping launcher `bash -n`: passed
- bridge Ctrl-C clean shutdown: passed

이 변경에서는 STM32 firmware source를 수정하지 않았다.
