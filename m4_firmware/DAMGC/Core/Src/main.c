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

#define BNO055_CHIP_ID_VALUE      0xA0U
#define BNO055_MODE_CONFIG        0x00U
#define BNO055_MODE_NDOF          0x0CU
#define BNO055_POWER_NORMAL       0x00U

#define BNO055_STATE_STARTING     0U
#define BNO055_STATE_NOT_FOUND    1U
#define BNO055_STATE_BAD_CHIP_ID  2U
#define BNO055_STATE_INIT_ERROR   3U
#define BNO055_STATE_RUNNING      4U

#define UART_PROTOCOL_VERSION     0x01U
#define UART_MSG_IMU_DATA         0x10U
#define UART_IMU_PAYLOAD_SIZE     52U
#define UART_IMU_FRAME_SIZE       64U
#define UART_IMU_PERIOD_MS        9U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

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
static uint16_t imu_tx_sequence;
static uint32_t timestamp_last_ms;
static uint64_t timestamp_epoch_ms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static HAL_StatusTypeDef BNO055_Read(uint8_t reg, uint8_t *data, uint16_t length);
static HAL_StatusTypeDef BNO055_Write(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef BNO055_Init(void);
static HAL_StatusTypeDef BNO055_Update(void);
static HAL_StatusTypeDef UART_SendIMU(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef BNO055_Read(uint8_t reg, uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, bno055_device_address,
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
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, bno055_device_address,
                                                reg, I2C_MEMADD_SIZE_8BIT,
                                                &value, 1U, 100U);
  bno055_test.last_hal_status = (uint32_t)status;
  if (status != HAL_OK)
  {
    bno055_test.i2c_error_count++;
  }
  return status;
}

static HAL_StatusTypeDef BNO055_Init(void)
{
  HAL_StatusTypeDef status;
  uint8_t chip_id = 0U;
  uint8_t address;

  bno055_test.state = BNO055_STATE_STARTING;
  bno055_test.address_7bit = 0U;
  bno055_test.chip_id = 0U;
  HAL_Delay(700U);

  address = BNO055_ADDRESS_LOW;
  bno055_device_address = (uint16_t)(address << 1U);
  status = HAL_I2C_IsDeviceReady(&hi2c2, bno055_device_address, 3U, 100U);
  if (status != HAL_OK)
  {
    address = BNO055_ADDRESS_HIGH;
    bno055_device_address = (uint16_t)(address << 1U);
    status = HAL_I2C_IsDeviceReady(&hi2c2, bno055_device_address, 3U, 100U);
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

static void PutU16LE(uint8_t *destination, uint16_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8U);
}

static void PutU32LE(uint8_t *destination, uint32_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8U);
  destination[2] = (uint8_t)(value >> 16U);
  destination[3] = (uint8_t)(value >> 24U);
}

static void PutU64LE(uint8_t *destination, uint64_t value)
{
  PutU32LE(destination, (uint32_t)value);
  PutU32LE(destination + 4U, (uint32_t)(value >> 32U));
}

static void PutFloatLE(uint8_t *destination, float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  PutU32LE(destination, bits);
}

static uint16_t CRC16_CCITT_FALSE(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t index = 0U; index < length; index++)
  {
    crc ^= (uint16_t)data[index] << 8U;
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 0x8000U) != 0U) ?
              (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

static uint64_t GetTimestampUs(void)
{
  uint32_t now_ms = HAL_GetTick();
  if (now_ms < timestamp_last_ms)
  {
    timestamp_epoch_ms += (1ULL << 32U);
  }
  timestamp_last_ms = now_ms;
  return (timestamp_epoch_ms + now_ms) * 1000ULL;
}

static HAL_StatusTypeDef UART_SendIMU(void)
{
  uint8_t frame[UART_IMU_FRAME_SIZE];
  uint16_t offset = 0U;
  uint16_t imu_status;
  uint16_t crc;
  HAL_StatusTypeDef status;

  frame[offset++] = 0xAAU;
  frame[offset++] = 0x55U;
  frame[offset++] = UART_PROTOCOL_VERSION;
  frame[offset++] = UART_MSG_IMU_DATA;
  PutU16LE(&frame[offset], UART_IMU_PAYLOAD_SIZE);
  offset += 2U;
  PutU16LE(&frame[offset], imu_tx_sequence);
  offset += 2U;
  PutU16LE(&frame[offset], 0U);
  offset += 2U;

  PutU64LE(&frame[offset], GetTimestampUs());
  offset += 8U;
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    PutFloatLE(&frame[offset], bno055_accel_mps2[axis]);
    offset += 4U;
  }
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    PutFloatLE(&frame[offset], bno055_gyro_rps[axis]);
    offset += 4U;
  }
  for (uint32_t component = 0U; component < 4U; component++)
  {
    PutFloatLE(&frame[offset], bno055_quaternion[component]);
    offset += 4U;
  }
  PutU16LE(&frame[offset], (uint16_t)bno055_temperature_cdeg);
  offset += 2U;

  imu_status = (uint16_t)(bno055_test.calib_stat & 0xFFU) |
               (uint16_t)((bno055_test.sys_status & 0x0FU) << 8U) |
               (uint16_t)((bno055_test.sys_error & 0x0FU) << 12U);
  PutU16LE(&frame[offset], imu_status);
  offset += 2U;

  crc = CRC16_CCITT_FALSE(&frame[2], (uint16_t)(offset - 2U));
  PutU16LE(&frame[offset], crc);
  offset += 2U;

  status = HAL_UART_Transmit(&huart2, frame, offset, 10U);
  (void)HAL_UART_Transmit(&huart1, frame, offset, 10U);
  bno055_test.uart_last_sequence = imu_tx_sequence;
  imu_tx_sequence++;
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  (void)BNO055_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (bno055_test.state == BNO055_STATE_RUNNING)
    {
      uint32_t loop_started_ms = HAL_GetTick();
      if (BNO055_Update() != HAL_OK)
      {
        bno055_test.state = BNO055_STATE_INIT_ERROR;
      }
      else
      {
        (void)UART_SendIMU();
      }
      uint32_t elapsed_ms = HAL_GetTick() - loop_started_ms;
      if (elapsed_ms < UART_IMU_PERIOD_MS)
      {
        HAL_Delay(UART_IMU_PERIOD_MS - elapsed_ms);
      }
    }
    else
    {
      HAL_Delay(1000U);
      (void)BNO055_Init();
    }
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x40B285C2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Init.BaudRate = 460800;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

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
