/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "drive.h"
#include "speed_pid.h"
#include "uart_protocol.h"
#include "calibration_store.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint32_t magic;
  uint32_t state;
  uint32_t address_7bit;
  uint32_t chip_id;
  uint32_t calib_stat;
  uint32_t sys_status;
  uint32_t sys_error;
  int32_t heading_x16;
  int32_t roll_x16;
  int32_t pitch_x16;
  uint32_t sample_count;
  uint32_t i2c_error_count;
  uint32_t last_hal_status;
  uint32_t uart_tx_count;
  uint32_t uart_tx_error_count;
  uint32_t uart_last_sequence;
} BNO055_TestData;

typedef enum
{
  SERIAL_TEST_IDLE = 0,
  SERIAL_TEST_ENCODER,
  SERIAL_TEST_IMU
} SerialTestMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BNO055_ADDRESS_LOW        0x28U
#define BNO055_ADDRESS_HIGH       0x29U

#define BNO055_REG_CHIP_ID        0x00U
#define BNO055_REG_GYR_DATA_X     0x14U
#define BNO055_REG_EUL_HEADING    0x1AU
#define BNO055_REG_CALIB_STAT     0x35U
#define BNO055_REG_SYS_STATUS     0x39U
#define BNO055_REG_SYS_ERR        0x3AU
#define BNO055_REG_PAGE_ID        0x07U
#define BNO055_REG_OPR_MODE       0x3DU
#define BNO055_REG_PWR_MODE       0x3EU
#define BNO055_REG_SYS_TRIGGER    0x3FU
#define BNO055_REG_ACC_OFFSET_X   0x55U

#define BNO055_CHIP_ID_VALUE      0xA0U
#define BNO055_MODE_CONFIG        0x00U
#define BNO055_MODE_NDOF          0x0CU
#define BNO055_POWER_NORMAL       0x00U

#define BNO055_STATE_STARTING     0U
#define BNO055_STATE_NOT_FOUND    1U
#define BNO055_STATE_BAD_CHIP_ID  2U
#define BNO055_STATE_INIT_ERROR   3U
#define BNO055_STATE_RUNNING      4U

#define UART_IMU_PERIOD_MS        9U
#define UART_WHEEL_PERIOD_MS      20U
#define UART_SYSTEM_PERIOD_MS     100U
#define UART_DEFAULT_WATCHDOG_MS  200U

#define SERIAL_REPORT_PERIOD_MS   200U
#define PID_PLOT_PERIOD_MS        20U
#define MOTOR_TEST_DURATION_MS    2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile BNO055_TestData bno055_test =
{
  .magic = 0xB0550550U,
  .state = BNO055_STATE_STARTING
};

static uint16_t bno055_device_address;
static float bno055_accel_mps2[3];
static float bno055_gyro_rps[3];
static float bno055_quaternion[4];
static int16_t bno055_temperature_cdeg;
static SerialTestMode serial_test_mode;
static uint8_t motor_test_duty_percent = 25U;
static uint8_t motor_test_active;
static uint32_t motor_test_stop_ms;
static uint32_t serial_last_report_ms;
static int32_t encoder_left_previous;
static int32_t encoder_right_previous;
static uint32_t encoder_last_sample_ms;
static uint32_t imu_last_send_ms;
static uint32_t wheel_last_send_ms;
static uint32_t system_last_send_ms;
static uint32_t bno055_next_retry_ms;
static uint32_t pid_last_report_ms;
static uint16_t remote_watchdog_ms = UART_DEFAULT_WATCHDOG_MS;
static uint8_t remote_control_active;
static uint8_t remote_estop_latched;
static uint8_t bno055_profile_loaded;
static uint8_t bno055_profile_mismatch_index = 0xFFU;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

static HAL_StatusTypeDef BNO055_Read(uint8_t reg, uint8_t *data, uint16_t length);
static HAL_StatusTypeDef BNO055_Write(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef BNO055_WriteBuffer(uint8_t reg,
                                            const uint8_t *data,
                                            uint16_t length);
static HAL_StatusTypeDef BNO055_ApplyCalibrationProfile(
    const BNO055CalibrationProfile *profile);
static HAL_StatusTypeDef BNO055_Init(void);
static HAL_StatusTypeDef BNO055_Update(void);
static void BNO055_PrintCalibrationStore(void);
static void BNO055_SaveCalibration(void);
static void BNO055_LoadCalibration(void);
static void BNO055_EraseCalibration(void);
static HAL_StatusTypeDef UART_SendIMU(void);
static void JetsonProtocol_Process(void);
static void UART_SendWheelState(void);
static void UART_SendSystemState(void);
static void SerialConsole_Write(const char *text);
static void SerialConsole_PrintHelp(void);
static void SerialConsole_PrintStatus(void);
static void SerialConsole_PrintPID(void);
static void SerialConsole_PrintLink(void);
static void SerialConsole_Process(void);
static void SerialTest_Process(void);
static void SerialTest_Encoder(void);
static void SerialTest_IMU(void);
static void SerialTest_MotorPWM(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef BNO055_Read(uint8_t reg, uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c3, bno055_device_address,
                                               reg, I2C_MEMADD_SIZE_8BIT,
                                               data, length, 100U);
  bno055_test.last_hal_status = (uint32_t)status;
  if (status != HAL_OK)
  {
    bno055_test.i2c_error_count++;
  }
  return status;
}

static HAL_StatusTypeDef BNO055_Write(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c3, bno055_device_address,
                                                reg, I2C_MEMADD_SIZE_8BIT,
                                                &value, 1U, 100U);
  bno055_test.last_hal_status = (uint32_t)status;
  if (status != HAL_OK)
  {
    bno055_test.i2c_error_count++;
  }
  return status;
}

static HAL_StatusTypeDef BNO055_WriteBuffer(uint8_t reg,
                                            const uint8_t *data,
                                            uint16_t length)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
      &hi2c3, bno055_device_address, reg, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)data, length, 100U);

  bno055_test.last_hal_status = (uint32_t)status;
  if (status != HAL_OK)
  {
    bno055_test.i2c_error_count++;
  }
  return status;
}

static HAL_StatusTypeDef BNO055_ApplyCalibrationProfile(
    const BNO055CalibrationProfile *profile)
{
  uint8_t verify[BNO055_CALIBRATION_PROFILE_SIZE];

  bno055_profile_mismatch_index = 0xFFU;
  for (uint32_t attempt = 0U; attempt < 3U; attempt++)
  {
    if (BNO055_WriteBuffer(BNO055_REG_ACC_OFFSET_X, profile->data,
                           BNO055_CALIBRATION_PROFILE_SIZE) != HAL_OK)
    {
      continue;
    }
    HAL_Delay(10U);
    if (BNO055_Read(BNO055_REG_ACC_OFFSET_X, verify,
                    BNO055_CALIBRATION_PROFILE_SIZE) != HAL_OK)
    {
      continue;
    }
    bno055_profile_mismatch_index = 0xFFU;
    for (uint8_t index = 0U; index < BNO055_CALIBRATION_PROFILE_SIZE; index++)
    {
      if (verify[index] != profile->data[index])
      {
        bno055_profile_mismatch_index = index;
        break;
      }
    }
    if (bno055_profile_mismatch_index == 0xFFU)
    {
      return HAL_OK;
    }
    HAL_Delay(10U);
  }
  return HAL_ERROR;
}

static HAL_StatusTypeDef BNO055_Init(void)
{
  HAL_StatusTypeDef status;
  BNO055CalibrationProfile profile;
  uint8_t chip_id = 0U;
  uint8_t address;

  bno055_test.state = BNO055_STATE_STARTING;
  bno055_test.address_7bit = 0U;
  bno055_test.chip_id = 0U;
  HAL_Delay(700U);

  address = BNO055_ADDRESS_LOW;
  bno055_device_address = (uint16_t)(address << 1U);
  status = HAL_I2C_IsDeviceReady(&hi2c3, bno055_device_address, 3U, 100U);
  if (status != HAL_OK)
  {
    address = BNO055_ADDRESS_HIGH;
    bno055_device_address = (uint16_t)(address << 1U);
    status = HAL_I2C_IsDeviceReady(&hi2c3, bno055_device_address, 3U, 100U);
  }

  bno055_test.last_hal_status = (uint32_t)status;
  if (status != HAL_OK)
  {
    bno055_test.state = BNO055_STATE_NOT_FOUND;
    bno055_test.i2c_error_count++;
    return status;
  }
  bno055_test.address_7bit = address;

  for (uint32_t retry = 0U; retry < 10U; retry++)
  {
    status = BNO055_Read(BNO055_REG_CHIP_ID, &chip_id, 1U);
    if ((status == HAL_OK) && (chip_id == BNO055_CHIP_ID_VALUE))
    {
      break;
    }
    HAL_Delay(100U);
  }
  bno055_test.chip_id = chip_id;
  if (chip_id != BNO055_CHIP_ID_VALUE)
  {
    bno055_test.state = BNO055_STATE_BAD_CHIP_ID;
    return HAL_ERROR;
  }

  if ((BNO055_Write(BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG) != HAL_OK))
  {
    bno055_test.state = BNO055_STATE_INIT_ERROR;
    return HAL_ERROR;
  }
  HAL_Delay(25U);

  if ((BNO055_Write(BNO055_REG_PAGE_ID, 0x00U) != HAL_OK) ||
      (BNO055_Write(BNO055_REG_PWR_MODE, BNO055_POWER_NORMAL) != HAL_OK) ||
      (BNO055_Write(BNO055_REG_SYS_TRIGGER, 0x00U) != HAL_OK))
  {
    bno055_test.state = BNO055_STATE_INIT_ERROR;
    return HAL_ERROR;
  }
  HAL_Delay(10U);

  bno055_profile_loaded = 0U;
  if ((CalibrationStore_Load(&profile) != 0U) &&
      (BNO055_ApplyCalibrationProfile(&profile) == HAL_OK))
  {
    bno055_profile_loaded = 1U;
  }

  if (BNO055_Write(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF) != HAL_OK)
  {
    bno055_test.state = BNO055_STATE_INIT_ERROR;
    return HAL_ERROR;
  }
  HAL_Delay(30U);
  bno055_test.state = BNO055_STATE_RUNNING;
  return HAL_OK;
}

static HAL_StatusTypeDef BNO055_Update(void)
{
  uint8_t sensor_data[33];
  uint8_t value;
  int16_t raw_value;

  if (BNO055_Read(BNO055_REG_GYR_DATA_X, sensor_data, sizeof(sensor_data)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    raw_value = (int16_t)(((uint16_t)sensor_data[(axis * 2U) + 1U] << 8U) |
                          sensor_data[axis * 2U]);
    bno055_gyro_rps[axis] = (float)raw_value * 0.0010908308f;

    raw_value = (int16_t)(((uint16_t)sensor_data[21U + (axis * 2U)] << 8U) |
                          sensor_data[20U + (axis * 2U)]);
    bno055_accel_mps2[axis] = (float)raw_value * 0.01f;
  }

  bno055_test.heading_x16 = (int16_t)(((uint16_t)sensor_data[7] << 8U) | sensor_data[6]);
  bno055_test.roll_x16 = (int16_t)(((uint16_t)sensor_data[9] << 8U) | sensor_data[8]);
  bno055_test.pitch_x16 = (int16_t)(((uint16_t)sensor_data[11] << 8U) | sensor_data[10]);

  raw_value = (int16_t)(((uint16_t)sensor_data[15] << 8U) | sensor_data[14]);
  bno055_quaternion[0] = (float)raw_value / 16384.0f;
  raw_value = (int16_t)(((uint16_t)sensor_data[17] << 8U) | sensor_data[16]);
  bno055_quaternion[1] = (float)raw_value / 16384.0f;
  raw_value = (int16_t)(((uint16_t)sensor_data[19] << 8U) | sensor_data[18]);
  bno055_quaternion[2] = (float)raw_value / 16384.0f;
  raw_value = (int16_t)(((uint16_t)sensor_data[13] << 8U) | sensor_data[12]);
  bno055_quaternion[3] = (float)raw_value / 16384.0f;

  bno055_temperature_cdeg = (int16_t)((int8_t)sensor_data[32]) * 100;

  if ((bno055_test.sample_count % 10U) == 0U)
  {
    if (BNO055_Read(BNO055_REG_CALIB_STAT, &value, 1U) == HAL_OK)
    {
      bno055_test.calib_stat = value;
    }
    if (BNO055_Read(BNO055_REG_SYS_STATUS, &value, 1U) == HAL_OK)
    {
      bno055_test.sys_status = value;
    }
    if (bno055_test.sys_status == 0x01U)
    {
      if (BNO055_Read(BNO055_REG_SYS_ERR, &value, 1U) == HAL_OK)
      {
        bno055_test.sys_error = value;
      }
    }
    else
    {
      bno055_test.sys_error = 0U;
    }
  }

  bno055_test.sample_count++;
  return HAL_OK;
}

static uint8_t BNO055_CalibrationAccessAllowed(void)
{
  if ((SpeedPID_GetStatus()->active != 0U) || (motor_test_active != 0U))
  {
    SerialConsole_Write("Calibration denied: stop motors first\r\n> ");
    return 0U;
  }
  if (bno055_test.state != BNO055_STATE_RUNNING)
  {
    SerialConsole_Write("Calibration denied: BNO055 is not running\r\n> ");
    return 0U;
  }
  return 1U;
}

static void BNO055_PrintCalibrationStore(void)
{
  char line[224];
  BNO055CalibrationProfile profile;
  uint8_t stored_calib;

  memset(&profile, 0, sizeof(profile));
  uint8_t valid = CalibrationStore_Load(&profile);
  stored_calib = (valid != 0U) ? profile.calibration_status : 0U;

  (void)snprintf(line, sizeof(line),
                 "CAL store[type=internal addr=0x%08lX valid=%u loaded=%u mismatch=%u] "
                 "saved[S%u G%u A%u M%u] "
                 "live[S%lu G%lu A%lu M%lu fusion=%lu err=%lu]\r\n> ",
                 (unsigned long)CalibrationStore_GetAddress(),
                 valid,
                 bno055_profile_loaded,
                 bno055_profile_mismatch_index,
                 (stored_calib >> 6U) & 0x03U,
                 (stored_calib >> 4U) & 0x03U,
                 (stored_calib >> 2U) & 0x03U,
                 stored_calib & 0x03U,
                 (unsigned long)((bno055_test.calib_stat >> 6U) & 0x03U),
                 (unsigned long)((bno055_test.calib_stat >> 4U) & 0x03U),
                 (unsigned long)((bno055_test.calib_stat >> 2U) & 0x03U),
                 (unsigned long)(bno055_test.calib_stat & 0x03U),
                 (unsigned long)bno055_test.sys_status,
                 (unsigned long)bno055_test.sys_error);
  SerialConsole_Write(line);
}

static void BNO055_SaveCalibration(void)
{
  BNO055CalibrationProfile profile;
  HAL_StatusTypeDef read_status;

  if (BNO055_CalibrationAccessAllowed() == 0U)
  {
    return;
  }
  if (CalibrationStore_IsPresent() == 0U)
  {
    SerialConsole_Write("CAL save failed: internal Flash unavailable\r\n> ");
    return;
  }
  if ((bno055_test.sys_status != 5U) ||
      ((bno055_test.calib_stat & 0x3FU) != 0x3FU))
  {
    SerialConsole_Write("CAL save denied: require fusion=5 and G3 A3 M3\r\n> ");
    return;
  }

  profile.calibration_status = (uint8_t)bno055_test.calib_stat;
  if (BNO055_Write(BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG) != HAL_OK)
  {
    SerialConsole_Write("CAL save failed: cannot enter CONFIG mode\r\n> ");
    return;
  }
  HAL_Delay(25U);
  read_status = BNO055_Read(BNO055_REG_ACC_OFFSET_X, profile.data,
                           BNO055_CALIBRATION_PROFILE_SIZE);
  (void)BNO055_Write(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF);
  HAL_Delay(30U);

  if (read_status != HAL_OK)
  {
    SerialConsole_Write("CAL save failed: BNO055 profile read error\r\n> ");
    return;
  }
  if (CalibrationStore_Save(&profile) == 0U)
  {
    SerialConsole_Write("CAL save failed: internal Flash write/verify error\r\n> ");
    return;
  }
  bno055_profile_loaded = 1U;
  SerialConsole_Write("CAL saved and verified; next boot will auto-load it\r\n> ");
}

static void BNO055_LoadCalibration(void)
{
  BNO055CalibrationProfile profile;
  HAL_StatusTypeDef write_status;

  if (BNO055_CalibrationAccessAllowed() == 0U)
  {
    return;
  }
  if (CalibrationStore_Load(&profile) == 0U)
  {
    SerialConsole_Write("CAL load failed: no valid saved profile\r\n> ");
    return;
  }
  if (BNO055_Write(BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG) != HAL_OK)
  {
    SerialConsole_Write("CAL load failed: cannot enter CONFIG mode\r\n> ");
    return;
  }
  HAL_Delay(25U);
  write_status = BNO055_ApplyCalibrationProfile(&profile);
  (void)BNO055_Write(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF);
  HAL_Delay(30U);

  if (write_status == HAL_OK)
  {
    bno055_profile_loaded = 1U;
    SerialConsole_Write("CAL profile loaded into BNO055\r\n> ");
  }
  else
  {
    bno055_profile_loaded = 0U;
    SerialConsole_Write("CAL load failed: BNO055 profile read-back mismatch\r\n> ");
  }
}

static void BNO055_EraseCalibration(void)
{
  if ((SpeedPID_GetStatus()->active != 0U) || (motor_test_active != 0U))
  {
    SerialConsole_Write("CAL erase denied: stop motors first\r\n> ");
    return;
  }
  if (CalibrationStore_Erase() != 0U)
  {
    bno055_profile_loaded = 0U;
    SerialConsole_Write("CAL saved profile erased; current BNO offsets unchanged\r\n> ");
  }
  else
  {
    SerialConsole_Write("CAL erase failed\r\n> ");
  }
}

static HAL_StatusTypeDef UART_SendIMU(void)
{
  uint16_t imu_status;
  HAL_StatusTypeDef status;

  imu_status = (uint16_t)(bno055_test.calib_stat &
                          UART_IMU_STATUS_CALIB_MASK) |
               (uint16_t)((bno055_test.sys_status & 0x0FU) <<
                          UART_IMU_STATUS_SYS_SHIFT) |
               (uint16_t)((bno055_test.sys_error & 0x0FU) <<
                          UART_IMU_STATUS_ERROR_SHIFT);
  status = UARTProtocol_SendIMU(bno055_accel_mps2, bno055_gyro_rps,
                                bno055_quaternion, bno055_temperature_cdeg,
                                imu_status);
  bno055_test.uart_last_sequence = UARTProtocol_GetStats()->tx_frames;
  if (status == HAL_OK)
  {
    bno055_test.uart_tx_count++;
  }
  else
  {
    bno055_test.uart_tx_error_count++;
  }
  return status;
}

static void JetsonProtocol_Process(void)
{
  UARTVelocityCommand command;

  UARTProtocol_Process();
  if (UARTProtocol_TakeVelocityCommand(&command) != 0U)
  {
    remote_watchdog_ms = (command.watchdog_ms != 0U) ?
                         command.watchdog_ms : UART_DEFAULT_WATCHDOG_MS;
    motor_test_active = 0U;

    if ((command.control_flags & UART_CONTROL_CLEAR_FAULT) != 0U)
    {
      remote_estop_latched = 0U;
    }

    if ((command.control_flags & UART_CONTROL_ESTOP) != 0U)
    {
      remote_estop_latched = 1U;
      SpeedPID_Stop();
      remote_control_active = 0U;
    }
    else if (remote_estop_latched != 0U)
    {
      SpeedPID_Stop();
      remote_control_active = 0U;
    }
    else if ((command.control_flags & UART_CONTROL_CONTROLLED_STOP) != 0U)
    {
      SpeedPID_SetTargets(0.0f, 0.0f);
      if (SpeedPID_GetStatus()->active == 0U)
      {
        SpeedPID_Start();
      }
      remote_control_active = 1U;
    }
    else if (((command.control_flags & UART_CONTROL_MOTOR_ENABLE) != 0U) &&
             (remote_estop_latched == 0U))
    {
      SpeedPID_SetTargets((float)command.left_mm_s,
                          (float)command.right_mm_s);
      if (SpeedPID_GetStatus()->active == 0U)
      {
        SpeedPID_Start();
      }
      remote_control_active = 1U;
    }
    else
    {
      SpeedPID_Stop();
      remote_control_active = 0U;
    }
  }

  if ((remote_control_active != 0U) &&
      (UARTProtocol_GetLastCommandAgeMs() > remote_watchdog_ms))
  {
    SpeedPID_Stop();
    remote_control_active = 0U;
  }
}

static void UART_SendWheelState(void)
{
  const EncoderState *encoder = Encoder_GetState();
  uint16_t encoder_status = 0U;

  if (encoder->sample_period_ms != 0U)
  {
    encoder_status |= UART_WHEEL_STATUS_SAMPLE_VALID;
  }
  if (SpeedPID_GetStatus()->active != 0U)
  {
    encoder_status |= UART_WHEEL_STATUS_PID_ACTIVE;
  }
  if (remote_control_active != 0U)
  {
    encoder_status |= UART_WHEEL_STATUS_REMOTE_ACTIVE;
  }

  (void)UARTProtocol_SendWheel(
      encoder->left_total_ticks, encoder->right_total_ticks,
      (int32_t)encoder->left_speed_mm_s,
      (int32_t)encoder->right_speed_mm_s, encoder_status);
}

static void UART_SendSystemState(void)
{
  const UARTProtocolStats *uart_stats = UARTProtocol_GetStats();
  uint32_t fault_bits = 0U;
  uint32_t command_age_ms = UARTProtocol_GetLastCommandAgeMs();
  uint8_t mode;

  if (bno055_test.state != BNO055_STATE_RUNNING)
  {
    fault_bits |= 0x00000001U;
  }
  if ((uart_stats->uart_errors != 0U) || (uart_stats->rx_overruns != 0U))
  {
    fault_bits |= 0x00000002U;
  }
  mode = (remote_estop_latched != 0U) ? 4U :
         ((remote_control_active != 0U) ? 3U :
          ((SpeedPID_GetStatus()->active != 0U) ? 2U : 1U));
  if (command_age_ms > 65535U)
  {
    command_age_ms = 65535U;
  }
  (void)UARTProtocol_SendSystem(0U, 0, 0, mode, remote_estop_latched,
                                fault_bits,
                                (uint16_t)command_age_ms);
}

static void SerialConsole_Write(const char *text)
{
  (void)HAL_UART_Transmit(&huart1, (const uint8_t *)text,
                          (uint16_t)strlen(text), 100U);
}

static void SerialConsole_PrintHelp(void)
{
  SerialConsole_Write(
      "\r\n=== DAMGC UART1 test console (115200 8N1) ===\r\n"
      "h/? : help\r\n"
      "s   : print one-shot status\r\n"
      "e   : toggle live encoder test\r\n"
      "i   : toggle live IMU test\r\n"
      "u   : print BNO calibration store status\r\n"
      "U   : save current G3/A3/M3 profile to internal Flash\r\n"
      "L   : load saved BNO calibration profile now\r\n"
      "X   : erase saved BNO calibration profile\r\n"
      "z   : zero both encoder counters\r\n"
      "+/- : change motor test PWM by 5%\r\n"
      "m   : run both motors forward for 2 seconds\r\n"
      "c   : toggle continuous closed-loop speed PID\r\n"
      "[/] : target speed -/+ 25 mm/s\r\n"
      "p/P : Kp -/+ 0.01\r\n"
      "k/K : Ki -/+ 0.05\r\n"
      "d/D : Kd -/+ 0.001\r\n"
      "a/A : acceleration -/+ 500 mm/s^2\r\n"
      "b/B : deceleration -/+ 1000 mm/s^2\r\n"
      "g   : print PID gains and live values\r\n"
      "j   : print Jetson UART2 link diagnostics\r\n"
      "0/q : emergency stop and leave live test\r\n"
      "=========================================\r\n> ");
}

static void SerialTest_Encoder(void)
{
  char line[224];
  uint32_t now_ms = HAL_GetTick();
  uint32_t elapsed_ms = now_ms - encoder_last_sample_ms;
  int32_t left = Encoder_GetLeftCount();
  int32_t right = Encoder_GetRightCount();
  int32_t left_delta = left - encoder_left_previous;
  int32_t right_delta = right - encoder_right_previous;
  int32_t left_distance_mm;
  int32_t right_distance_mm;
  int32_t left_speed_mm_s = 0;
  int32_t right_speed_mm_s = 0;

  left_distance_mm = (int32_t)(((int64_t)left * WHEEL_CIRCUMFERENCE_UM) /
                               ((int64_t)ENCODER_COUNTS_PER_REV * 1000));
  right_distance_mm = (int32_t)(((int64_t)right * WHEEL_CIRCUMFERENCE_UM) /
                                ((int64_t)ENCODER_COUNTS_PER_REV * 1000));
  if (elapsed_ms != 0U)
  {
    left_speed_mm_s = (int32_t)(((int64_t)left_delta * WHEEL_CIRCUMFERENCE_UM) /
                                ((int64_t)ENCODER_COUNTS_PER_REV * elapsed_ms));
    right_speed_mm_s = (int32_t)(((int64_t)right_delta * WHEEL_CIRCUMFERENCE_UM) /
                                 ((int64_t)ENCODER_COUNTS_PER_REV * elapsed_ms));
  }
  encoder_left_previous = left;
  encoder_right_previous = right;
  encoder_last_sample_ms = now_ms;
  (void)snprintf(line, sizeof(line),
                 "ENC L[tick=%ld d=%ld dist=%ldmm speed=%ldmm/s] "
                 "R[tick=%ld d=%ld dist=%ldmm speed=%ldmm/s] dt=%lums\r\n",
                 (long)left, (long)left_delta, (long)left_distance_mm,
                 (long)left_speed_mm_s, (long)right, (long)right_delta,
                 (long)right_distance_mm, (long)right_speed_mm_s,
                 (unsigned long)elapsed_ms);
  SerialConsole_Write(line);
}

static void SerialTest_IMU(void)
{
  char line[224];
  int32_t heading_cdeg = (bno055_test.heading_x16 * 100) / 16;
  int32_t roll_cdeg = (bno055_test.roll_x16 * 100) / 16;
  int32_t pitch_cdeg = (bno055_test.pitch_x16 * 100) / 16;
  uint32_t calib = bno055_test.calib_stat;

  (void)snprintf(line, sizeof(line),
                 "IMU state=%lu id=0x%02lX H=%ld.%02ld R=%ld.%02ld P=%ld.%02ld "
                 "T=%ld.%02ldC cal[S%lu G%lu A%lu M%lu] "
                 "fusion=%lu bnoErr=%lu i2cErr=%lu\r\n",
                 (unsigned long)bno055_test.state,
                 (unsigned long)bno055_test.chip_id,
                 (long)(heading_cdeg / 100), (long)(heading_cdeg < 0 ? -(heading_cdeg % 100) : heading_cdeg % 100),
                 (long)(roll_cdeg / 100), (long)(roll_cdeg < 0 ? -(roll_cdeg % 100) : roll_cdeg % 100),
                 (long)(pitch_cdeg / 100), (long)(pitch_cdeg < 0 ? -(pitch_cdeg % 100) : pitch_cdeg % 100),
                 (long)(bno055_temperature_cdeg / 100),
                 (long)(bno055_temperature_cdeg < 0 ? -(bno055_temperature_cdeg % 100) : bno055_temperature_cdeg % 100),
                 (unsigned long)((calib >> 6U) & 0x03U),
                 (unsigned long)((calib >> 4U) & 0x03U),
                 (unsigned long)((calib >> 2U) & 0x03U),
                 (unsigned long)(calib & 0x03U),
                 (unsigned long)bno055_test.sys_status,
                 (unsigned long)bno055_test.sys_error,
                 (unsigned long)bno055_test.i2c_error_count);
  SerialConsole_Write(line);
}

static void SerialTest_MotorPWM(void)
{
  char line[128];

  remote_control_active = 0U;
  SpeedPID_Stop();
  Drive_SetBothPercent((float)motor_test_duty_percent);
  motor_test_active = 1U;
  motor_test_stop_ms = HAL_GetTick() + MOTOR_TEST_DURATION_MS;
  (void)snprintf(line, sizeof(line),
                 "MOTOR both forward, PWM=%u%%, auto-stop=2s (0=STOP)\r\n",
                 motor_test_duty_percent);
  SerialConsole_Write(line);
}

static void SerialConsole_PrintStatus(void)
{
  char line[192];
  int32_t left = Encoder_GetLeftCount();
  int32_t right = Encoder_GetRightCount();

  (void)snprintf(line, sizeof(line),
                 "STATUS tick=%lu ENC[L=%ld R=%ld] IMU[state=%lu id=0x%02lX samples=%lu i2cErr=%lu] "
                 "MOTOR[pwm=%u%% running=%u]\r\n> ",
                 (unsigned long)HAL_GetTick(), (long)left, (long)right,
                 (unsigned long)bno055_test.state,
                 (unsigned long)bno055_test.chip_id,
                 (unsigned long)bno055_test.sample_count,
                 (unsigned long)bno055_test.i2c_error_count,
                 motor_test_duty_percent, motor_test_active);
  SerialConsole_Write(line);
}

static void SerialConsole_PrintPID(void)
{
  char line[224];
  const SpeedPIDStatus *pid = SpeedPID_GetStatus();

  (void)snprintf(line, sizeof(line),
                 "PID active=%u cmd[L=%ld R=%ld] ramp[L=%ld R=%ld]mm/s acc=%ld dec=%ld Kp=%ld/1000 Ki=%ld/1000 Kd=%ld/1000 "
                 "L[v=%ld pwm=%ld] R[v=%ld pwm=%ld]\r\n> ",
                 pid->active, (long)pid->left_target_mm_s,
                 (long)pid->right_target_mm_s,
                 (long)pid->left_ramped_target_mm_s,
                 (long)pid->right_ramped_target_mm_s,
                 (long)pid->acceleration_mm_s2,
                 (long)pid->deceleration_mm_s2,
                 (long)(pid->kp * 1000.0f + 0.5f),
                 (long)(pid->ki * 1000.0f + 0.5f),
                 (long)(pid->kd * 1000.0f + 0.5f),
                 (long)pid->left_speed_mm_s, (long)pid->left_output_pct,
                 (long)pid->right_speed_mm_s, (long)pid->right_output_pct);
  SerialConsole_Write(line);
}

static void SerialConsole_PrintLink(void)
{
  char line[224];
  const UARTProtocolStats *stats = UARTProtocol_GetStats();

  (void)snprintf(line, sizeof(line),
                 "UART2 rx=%lu crc=%lu malformed=%lu overrun=%lu uartErr=%lu "
                 "tx=%lu txErr=%lu cmdAge=%lums watchdog=%ums remote=%u estop=%u\r\n> ",
                 (unsigned long)stats->valid_rx_frames,
                 (unsigned long)stats->crc_errors,
                 (unsigned long)stats->malformed_frames,
                 (unsigned long)stats->rx_overruns,
                 (unsigned long)stats->uart_errors,
                 (unsigned long)stats->tx_frames,
                 (unsigned long)stats->tx_errors,
                 (unsigned long)UARTProtocol_GetLastCommandAgeMs(),
                 remote_watchdog_ms, remote_control_active,
                 remote_estop_latched);
  SerialConsole_Write(line);
}

static void SerialConsole_Process(void)
{
  uint8_t command;
  char line[192];

  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
  }

  if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET) ||
      (HAL_UART_Receive(&huart1, &command, 1U, 1U) != HAL_OK))
  {
    return;
  }

  switch (command)
  {
    case 'h':
    case 'H':
    case '?':
      SerialConsole_PrintHelp();
      break;

    case 's':
    case 'S':
      SerialConsole_PrintStatus();
      break;

    case 'e':
    case 'E':
      serial_test_mode = (serial_test_mode == SERIAL_TEST_ENCODER) ?
                         SERIAL_TEST_IDLE : SERIAL_TEST_ENCODER;
      encoder_left_previous = Encoder_GetLeftCount();
      encoder_right_previous = Encoder_GetRightCount();
      encoder_last_sample_ms = HAL_GetTick();
      SerialConsole_Write(serial_test_mode == SERIAL_TEST_ENCODER ?
                          "ENC live test ON (e/q=exit)\r\n" : "ENC live test OFF\r\n> ");
      break;

    case 'i':
    case 'I':
      serial_test_mode = (serial_test_mode == SERIAL_TEST_IMU) ?
                         SERIAL_TEST_IDLE : SERIAL_TEST_IMU;
      SerialConsole_Write(serial_test_mode == SERIAL_TEST_IMU ?
                          "IMU live test ON (i/q=exit)\r\n" : "IMU live test OFF\r\n> ");
      break;

    case 'u':
      BNO055_PrintCalibrationStore();
      break;

    case 'U':
      BNO055_SaveCalibration();
      break;

    case 'L':
      BNO055_LoadCalibration();
      break;

    case 'X':
      BNO055_EraseCalibration();
      break;

    case 'z':
    case 'Z':
      Encoder_Zero();
      encoder_left_previous = 0;
      encoder_right_previous = 0;
      encoder_last_sample_ms = HAL_GetTick();
      SerialConsole_Write("ENC counters zeroed\r\n> ");
      break;

    case '+':
      motor_test_duty_percent = (motor_test_duty_percent <= 95U) ?
                                (uint8_t)(motor_test_duty_percent + 5U) : 100U;
      (void)snprintf(line, sizeof(line), "Motor test PWM=%u%%\r\n> ", motor_test_duty_percent);
      SerialConsole_Write(line);
      break;

    case '-':
      motor_test_duty_percent = (motor_test_duty_percent >= 10U) ?
                                (uint8_t)(motor_test_duty_percent - 5U) : 5U;
      (void)snprintf(line, sizeof(line), "Motor test PWM=%u%%\r\n> ", motor_test_duty_percent);
      SerialConsole_Write(line);
      break;

    case 'm':
    case 'M':
      SerialTest_MotorPWM();
      break;

    case 'c':
    case 'C':
      motor_test_active = 0U;
      remote_control_active = 0U;
      if (SpeedPID_GetStatus()->active != 0U)
      {
        SpeedPID_Stop();
        SerialConsole_Write("PID stopped\r\n> ");
      }
      else
      {
        SpeedPID_Start();
        pid_last_report_ms = HAL_GetTick();
      }
      break;

    case '[':
      SpeedPID_SetTarget(SpeedPID_GetStatus()->left_target_mm_s - 25.0f);
      SerialConsole_PrintPID();
      break;

    case ']':
      SpeedPID_SetTarget(SpeedPID_GetStatus()->left_target_mm_s + 25.0f);
      SerialConsole_PrintPID();
      break;

    case 'p':
      SpeedPID_SetGains(SpeedPID_GetStatus()->kp - 0.010f,
                        SpeedPID_GetStatus()->ki, SpeedPID_GetStatus()->kd);
      SerialConsole_PrintPID();
      break;

    case 'P':
      SpeedPID_SetGains(SpeedPID_GetStatus()->kp + 0.010f,
                        SpeedPID_GetStatus()->ki, SpeedPID_GetStatus()->kd);
      SerialConsole_PrintPID();
      break;

    case 'k':
      SpeedPID_SetGains(SpeedPID_GetStatus()->kp,
                        SpeedPID_GetStatus()->ki - 0.050f,
                        SpeedPID_GetStatus()->kd);
      SerialConsole_PrintPID();
      break;

    case 'K':
      SpeedPID_SetGains(SpeedPID_GetStatus()->kp,
                        SpeedPID_GetStatus()->ki + 0.050f,
                        SpeedPID_GetStatus()->kd);
      SerialConsole_PrintPID();
      break;

    case 'd':
      SpeedPID_SetGains(SpeedPID_GetStatus()->kp, SpeedPID_GetStatus()->ki,
                        SpeedPID_GetStatus()->kd - 0.001f);
      SerialConsole_PrintPID();
      break;

    case 'D':
      SpeedPID_SetGains(SpeedPID_GetStatus()->kp, SpeedPID_GetStatus()->ki,
                        SpeedPID_GetStatus()->kd + 0.001f);
      SerialConsole_PrintPID();
      break;

    case 'g':
    case 'G':
      SerialConsole_PrintPID();
      break;

    case 'j':
    case 'J':
      SerialConsole_PrintLink();
      break;

    case 'a':
      SpeedPID_SetAcceleration(SpeedPID_GetStatus()->acceleration_mm_s2 - 500.0f,
                               SpeedPID_GetStatus()->deceleration_mm_s2);
      SerialConsole_PrintPID();
      break;

    case 'A':
      SpeedPID_SetAcceleration(SpeedPID_GetStatus()->acceleration_mm_s2 + 500.0f,
                               SpeedPID_GetStatus()->deceleration_mm_s2);
      SerialConsole_PrintPID();
      break;

    case 'b':
      SpeedPID_SetAcceleration(SpeedPID_GetStatus()->acceleration_mm_s2,
                               SpeedPID_GetStatus()->deceleration_mm_s2 - 1000.0f);
      SerialConsole_PrintPID();
      break;

    case 'B':
      SpeedPID_SetAcceleration(SpeedPID_GetStatus()->acceleration_mm_s2,
                               SpeedPID_GetStatus()->deceleration_mm_s2 + 1000.0f);
      SerialConsole_PrintPID();
      break;

    case '0':
    case 'q':
    case 'Q':
      remote_control_active = 0U;
      SpeedPID_Stop();
      Drive_Stop();
      motor_test_active = 0U;
      serial_test_mode = SERIAL_TEST_IDLE;
      SerialConsole_Write("STOP: motor off, live test off\r\n> ");
      break;

    case '\r':
    case '\n':
      break;

    default:
      SerialConsole_Write("Unknown command. Send h for help.\r\n> ");
      break;
  }
}

static void SerialTest_Process(void)
{
  uint32_t now_ms = HAL_GetTick();
  const SpeedPIDStatus *pid;

  (void)Encoder_Update();

  if ((motor_test_active != 0U) &&
      ((int32_t)(now_ms - motor_test_stop_ms) >= 0))
  {
    Drive_Stop();
    motor_test_active = 0U;
    SerialConsole_Write("MOTOR auto-stopped\r\n> ");
  }

  if (SpeedPID_Process() != 0U)
  {
    pid = SpeedPID_GetStatus();
    if ((uint32_t)(now_ms - pid_last_report_ms) >= PID_PLOT_PERIOD_MS)
    {
      char line[160];
      pid_last_report_ms = now_ms;
      (void)snprintf(line, sizeof(line), "%ld,%ld,%ld\r\n",
                     (long)((pid->left_ramped_target_mm_s +
                             pid->right_ramped_target_mm_s) * 0.5f),
                     (long)pid->left_speed_mm_s,
                     (long)pid->right_speed_mm_s);
      SerialConsole_Write(line);
    }
  }

  if ((serial_test_mode != SERIAL_TEST_IDLE) &&
      ((uint32_t)(now_ms - serial_last_report_ms) >= SERIAL_REPORT_PERIOD_MS))
  {
    serial_last_report_ms = now_ms;
    if (serial_test_mode == SERIAL_TEST_ENCODER)
    {
      SerialTest_Encoder();
    }
    else if (serial_test_mode == SERIAL_TEST_IMU)
    {
      SerialTest_IMU();
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Drive_Bind(&htim1, &htim3, &htim2);
  if (Drive_Start() != HAL_OK)
  {
    Error_Handler();
  }
  SpeedPID_Init();
  UARTProtocol_Init(&huart2);
  CalibrationStore_Init();
  UART_SendWheelState();
  (void)UARTProtocol_SendSystem(0U, 0, 0, 0U, 0U, 0U, 0U);
  SerialConsole_PrintHelp();
  SerialConsole_Write("DAMGC boot: initializing BNO055...\r\n");
  (void)BNO055_Init();
  if (UARTProtocol_StartReceive() != HAL_OK)
  {
    Error_Handler();
  }
  imu_last_send_ms = HAL_GetTick();
  wheel_last_send_ms = HAL_GetTick();
  system_last_send_ms = HAL_GetTick();
  bno055_next_retry_ms = HAL_GetTick() + 2000U;
  SerialConsole_Write(bno055_test.state == BNO055_STATE_RUNNING ?
                      "BNO055 ready\r\n> " : "BNO055 not ready; automatic retry enabled\r\n> ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();

    JetsonProtocol_Process();
    SerialConsole_Process();
    SerialTest_Process();

    if ((uint32_t)(now_ms - wheel_last_send_ms) >= UART_WHEEL_PERIOD_MS)
    {
      wheel_last_send_ms = now_ms;
      UART_SendWheelState();
    }
    if ((uint32_t)(now_ms - system_last_send_ms) >= UART_SYSTEM_PERIOD_MS)
    {
      system_last_send_ms = now_ms;
      UART_SendSystemState();
    }

    if ((bno055_test.state == BNO055_STATE_RUNNING) &&
        ((uint32_t)(now_ms - imu_last_send_ms) >= UART_IMU_PERIOD_MS))
    {
      imu_last_send_ms = now_ms;
      if (BNO055_Update() != HAL_OK)
      {
        bno055_test.state = BNO055_STATE_INIT_ERROR;
        bno055_next_retry_ms = now_ms + 2000U;
        SerialConsole_Write("IMU read error; retry scheduled\r\n> ");
      }
      else
      {
        (void)UART_SendIMU();
      }
    }
    else if ((bno055_test.state != BNO055_STATE_RUNNING) &&
             ((int32_t)(now_ms - bno055_next_retry_ms) >= 0) &&
             (SpeedPID_GetStatus()->active == 0U))
    {
      bno055_next_retry_ms = now_ms + 2000U;
      SerialConsole_Write("Retrying BNO055 initialization...\r\n");
      (void)BNO055_Init();
    }
    HAL_Delay(1U);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 34, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(170000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40B285C2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7U;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(INA1_GPIO_Port, INA1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(INA2_GPIO_Port, INA2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(INB2_GPIO_Port, INB2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(INB1_GPIO_Port, INB1_Pin);

  /**/
  LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);

  /**/
  GPIO_InitStruct.Pin = INA1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(INA1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = INA2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(INA2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = INB2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(INB2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = INB1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(INB1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
