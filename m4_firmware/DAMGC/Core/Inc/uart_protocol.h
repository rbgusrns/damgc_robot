#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "main.h"

#define UART_PROTOCOL_VERSION 0x01U

#define UART_MSG_CMD_VELOCITY 0x01U
#define UART_MSG_IMU_DATA 0x10U
#define UART_MSG_WHEEL_STATE 0x11U
#define UART_MSG_SYSTEM_STATE 0x12U

#define UART_CONTROL_MOTOR_ENABLE 0x0001U
#define UART_CONTROL_CONTROLLED_STOP 0x0002U
#define UART_CONTROL_ESTOP 0x0004U
#define UART_CONTROL_CLEAR_FAULT 0x0008U

/* IMU status payload: CALIB_STAT[7:0], SYS_STATUS[11:8], SYS_ERR[15:12]. */
#define UART_IMU_STATUS_CALIB_MASK 0x00FFU
#define UART_IMU_STATUS_SYS_SHIFT 8U
#define UART_IMU_STATUS_ERROR_SHIFT 12U

/* Wheel-state status bits. */
#define UART_WHEEL_STATUS_SAMPLE_VALID 0x0001U
#define UART_WHEEL_STATUS_PID_ACTIVE 0x0002U
#define UART_WHEEL_STATUS_REMOTE_ACTIVE 0x0004U

typedef struct
{
  int16_t left_mm_s;
  int16_t right_mm_s;
  uint16_t watchdog_ms;
  uint16_t control_flags;
  uint16_t sequence;
} UARTVelocityCommand;

typedef struct
{
  uint32_t valid_rx_frames;
  uint32_t crc_errors;
  uint32_t malformed_frames;
  uint32_t rx_overruns;
  uint32_t uart_errors;
  uint32_t tx_frames;
  uint32_t tx_errors;
} UARTProtocolStats;

void UARTProtocol_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef UARTProtocol_StartReceive(void);
void UARTProtocol_Process(void);
uint8_t UARTProtocol_TakeVelocityCommand(UARTVelocityCommand *command);
uint32_t UARTProtocol_GetLastCommandAgeMs(void);
uint64_t UARTProtocol_GetTimestampUs(void);
const UARTProtocolStats *UARTProtocol_GetStats(void);

HAL_StatusTypeDef UARTProtocol_SendIMU(const float accel_mps2[3],
                                       const float gyro_rps[3],
                                       const float quaternion_xyzw[4],
                                       int16_t temperature_cdeg,
                                       uint16_t imu_status);
HAL_StatusTypeDef UARTProtocol_SendWheel(int64_t left_ticks,
                                         int64_t right_ticks,
                                         int32_t left_mm_s,
                                         int32_t right_mm_s,
                                         uint16_t encoder_status);
HAL_StatusTypeDef UARTProtocol_SendSystem(uint16_t battery_mv,
                                          int16_t battery_ma,
                                          int16_t motor_temp_cdeg,
                                          uint8_t mode,
                                          uint8_t estop_state,
                                          uint32_t fault_bits,
                                          uint16_t last_cmd_age_ms);

void UARTProtocol_RxCpltCallback(UART_HandleTypeDef *uart);
void UARTProtocol_ErrorCallback(UART_HandleTypeDef *uart);

#endif
