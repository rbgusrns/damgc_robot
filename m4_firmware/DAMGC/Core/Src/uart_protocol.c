#include "uart_protocol.h"
#include "i2c_protocol.h"

#include <string.h>

#define UART_SYNC_0 0xAAU
#define UART_SYNC_1 0x55U
#define UART_MAX_PAYLOAD 512U
#define UART_MAX_FRAME_SIZE (12U + UART_MAX_PAYLOAD)
#define UART_RX_RING_SIZE 256U
#define UART_RX_DMA_SIZE 256U
#define UART_TX_QUEUE_DEPTH 8U
#define UART_CMD_VELOCITY_PAYLOAD_SIZE 8U
#define UART_IMU_PAYLOAD_SIZE 52U
#define UART_WHEEL_PAYLOAD_SIZE 34U
#define UART_SYSTEM_PAYLOAD_SIZE 22U

static UART_HandleTypeDef *protocol_uart;
static volatile uint8_t rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t rx_dma_buffer[UART_RX_DMA_SIZE];
static volatile uint16_t rx_dma_position;
static volatile uint8_t rx_restart_pending;
typedef struct
{
  uint8_t data[64];
  uint16_t length;
} UARTTxFrame;
static UARTTxFrame tx_queue[UART_TX_QUEUE_DEPTH];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_dma_active;
static uint8_t parser_frame[UART_MAX_FRAME_SIZE];
static uint16_t parser_length;
static uint16_t parser_expected_length;
static uint16_t tx_sequence;
static uint16_t last_rx_sequence;
static uint8_t have_rx_sequence;
static UARTVelocityCommand velocity_command;
static uint8_t velocity_command_pending;
static uint32_t last_command_ms;
static uint32_t timestamp_last_ms;
static uint64_t timestamp_epoch_ms;
static UARTProtocolStats protocol_stats;

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

static uint16_t GetU16LE(const uint8_t *source)
{
  return (uint16_t)source[0] | ((uint16_t)source[1] << 8U);
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

uint64_t UARTProtocol_GetTimestampUs(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (now_ms < timestamp_last_ms)
  {
    timestamp_epoch_ms += (1ULL << 32U);
  }
  timestamp_last_ms = now_ms;
  return (timestamp_epoch_ms + now_ms) * 1000ULL;
}

static void RestoreInterruptState(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void StartNextTx(void)
{
  if ((protocol_uart == NULL) || (tx_dma_active != 0U) ||
      (tx_tail == tx_head))
  {
    return;
  }
  if (HAL_UART_Transmit_DMA(protocol_uart, tx_queue[tx_tail].data,
                            tx_queue[tx_tail].length) == HAL_OK)
  {
    tx_dma_active = 1U;
  }
  else
  {
    protocol_stats.tx_errors++;
  }
}

static void PushRxByte(uint8_t byte)
{
  uint16_t next_head = (uint16_t)((rx_head + 1U) % UART_RX_RING_SIZE);

  if (next_head == rx_tail)
  {
    protocol_stats.rx_overruns++;
  }
  else
  {
    rx_ring[rx_head] = byte;
    rx_head = next_head;
  }
}

static HAL_StatusTypeDef SendFrame(uint8_t message_type,
                                   const uint8_t *payload,
                                   uint16_t payload_length,
                                   uint16_t flags)
{
  uint8_t frame[64];
  uint16_t offset = 0U;
  uint16_t crc;
  uint8_t next_head;
  uint32_t primask;

  if ((protocol_uart == NULL) || (payload_length > (sizeof(frame) - 12U)))
  {
    return HAL_ERROR;
  }

  frame[offset++] = UART_SYNC_0;
  frame[offset++] = UART_SYNC_1;
  frame[offset++] = UART_PROTOCOL_VERSION;
  frame[offset++] = message_type;
  PutU16LE(&frame[offset], payload_length);
  offset += 2U;
  PutU16LE(&frame[offset], tx_sequence++);
  offset += 2U;
  PutU16LE(&frame[offset], flags);
  offset += 2U;
  if (payload_length != 0U)
  {
    memcpy(&frame[offset], payload, payload_length);
    offset += payload_length;
  }
  crc = CRC16_CCITT_FALSE(&frame[2], (uint16_t)(offset - 2U));
  PutU16LE(&frame[offset], crc);
  offset += 2U;

  I2CProtocol_PublishFrame(frame, offset);

  primask = __get_PRIMASK();
  __disable_irq();
  next_head = (uint8_t)((tx_head + 1U) % UART_TX_QUEUE_DEPTH);
  if (next_head == tx_tail)
  {
    protocol_stats.tx_errors++;
    RestoreInterruptState(primask);
    return HAL_BUSY;
  }
  memcpy(tx_queue[tx_head].data, frame, offset);
  tx_queue[tx_head].length = offset;
  tx_head = next_head;
  StartNextTx();
  RestoreInterruptState(primask);
  return HAL_OK;
}

static void DispatchFrame(void)
{
  uint8_t message_type = parser_frame[3];
  uint16_t payload_length = GetU16LE(&parser_frame[4]);
  uint16_t sequence = GetU16LE(&parser_frame[6]);
  const uint8_t *payload = &parser_frame[10];

  if (have_rx_sequence != 0U)
  {
    /* Sequence gaps are tolerated. The bridge resends velocity at 50 Hz. */
    (void)last_rx_sequence;
  }
  last_rx_sequence = sequence;
  have_rx_sequence = 1U;

  if ((message_type == UART_MSG_CMD_VELOCITY) &&
      (payload_length == UART_CMD_VELOCITY_PAYLOAD_SIZE))
  {
    velocity_command.left_mm_s = (int16_t)GetU16LE(&payload[0]);
    velocity_command.right_mm_s = (int16_t)GetU16LE(&payload[2]);
    velocity_command.watchdog_ms = GetU16LE(&payload[4]);
    velocity_command.control_flags = GetU16LE(&payload[6]);
    velocity_command.sequence = sequence;
    last_command_ms = HAL_GetTick();
    velocity_command_pending = 1U;
    protocol_stats.valid_rx_frames++;
  }
}

static void ParserResetWithByte(uint8_t byte)
{
  parser_length = 0U;
  parser_expected_length = 0U;
  if (byte == UART_SYNC_0)
  {
    parser_frame[0] = byte;
    parser_length = 1U;
  }
}

static void ParserFeed(uint8_t byte)
{
  uint16_t payload_length;
  uint16_t received_crc;
  uint16_t calculated_crc;

  if (parser_length == 0U)
  {
    if (byte == UART_SYNC_0)
    {
      parser_frame[parser_length++] = byte;
    }
    return;
  }
  if (parser_length == 1U)
  {
    if (byte == UART_SYNC_1)
    {
      parser_frame[parser_length++] = byte;
    }
    else
    {
      ParserResetWithByte(byte);
    }
    return;
  }

  parser_frame[parser_length++] = byte;
  if (parser_length == 6U)
  {
    payload_length = GetU16LE(&parser_frame[4]);
    if ((parser_frame[2] != UART_PROTOCOL_VERSION) ||
        (payload_length > UART_MAX_PAYLOAD))
    {
      protocol_stats.malformed_frames++;
      ParserResetWithByte(byte);
      return;
    }
    parser_expected_length = (uint16_t)(12U + payload_length);
  }

  if ((parser_expected_length != 0U) &&
      (parser_length == parser_expected_length))
  {
    received_crc = GetU16LE(&parser_frame[parser_length - 2U]);
    calculated_crc = CRC16_CCITT_FALSE(&parser_frame[2],
                                       (uint16_t)(parser_length - 4U));
    if (received_crc == calculated_crc)
    {
      DispatchFrame();
    }
    else
    {
      protocol_stats.crc_errors++;
    }
    ParserResetWithByte(byte);
  }
}

void UARTProtocol_Init(UART_HandleTypeDef *uart)
{
  protocol_uart = uart;
  rx_head = 0U;
  rx_tail = 0U;
  rx_dma_position = 0U;
  rx_restart_pending = 0U;
  tx_head = 0U;
  tx_tail = 0U;
  tx_dma_active = 0U;
  parser_length = 0U;
  parser_expected_length = 0U;
  tx_sequence = 0U;
  have_rx_sequence = 0U;
  velocity_command_pending = 0U;
  last_command_ms = HAL_GetTick();
  timestamp_last_ms = HAL_GetTick();
  timestamp_epoch_ms = 0U;
  memset(&protocol_stats, 0, sizeof(protocol_stats));
}

HAL_StatusTypeDef UARTProtocol_StartReceive(void)
{
  if (protocol_uart == NULL)
  {
    return HAL_ERROR;
  }
  __HAL_UART_CLEAR_OREFLAG(protocol_uart);
  __HAL_UART_FLUSH_DRREGISTER(protocol_uart);
  rx_dma_position = 0U;
  rx_restart_pending = 0U;
  return HAL_UARTEx_ReceiveToIdle_DMA(protocol_uart, rx_dma_buffer,
                                      UART_RX_DMA_SIZE);
}

void UARTProtocol_Process(void)
{
  uint32_t primask;

  if (rx_restart_pending != 0U)
  {
    (void)HAL_UART_AbortReceive(protocol_uart);
    if (UARTProtocol_StartReceive() != HAL_OK)
    {
      protocol_stats.uart_errors++;
      rx_restart_pending = 1U;
    }
  }

  primask = __get_PRIMASK();
  __disable_irq();
  StartNextTx();
  RestoreInterruptState(primask);

  while (rx_tail != rx_head)
  {
    uint8_t byte = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % UART_RX_RING_SIZE);
    ParserFeed(byte);
  }
}

void UARTProtocol_PushRxData(const uint8_t *data, uint16_t length)
{
  if (data == NULL)
  {
    return;
  }

  for (uint16_t index = 0U; index < length; index++)
  {
    PushRxByte(data[index]);
  }
}

uint8_t UARTProtocol_TakeVelocityCommand(UARTVelocityCommand *command)
{
  if ((velocity_command_pending == 0U) || (command == NULL))
  {
    return 0U;
  }
  *command = velocity_command;
  velocity_command_pending = 0U;
  return 1U;
}

uint32_t UARTProtocol_GetLastCommandAgeMs(void)
{
  return HAL_GetTick() - last_command_ms;
}

const UARTProtocolStats *UARTProtocol_GetStats(void)
{
  return &protocol_stats;
}

HAL_StatusTypeDef UARTProtocol_SendIMU(const float accel_mps2[3],
                                       const float gyro_rps[3],
                                       const float quaternion_xyzw[4],
                                       int16_t temperature_cdeg,
                                       uint16_t imu_status)
{
  uint8_t payload[UART_IMU_PAYLOAD_SIZE];
  uint16_t offset = 0U;

  PutU64LE(&payload[offset], UARTProtocol_GetTimestampUs());
  offset += 8U;
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    PutFloatLE(&payload[offset], accel_mps2[axis]);
    offset += 4U;
  }
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    PutFloatLE(&payload[offset], gyro_rps[axis]);
    offset += 4U;
  }
  for (uint32_t component = 0U; component < 4U; component++)
  {
    PutFloatLE(&payload[offset], quaternion_xyzw[component]);
    offset += 4U;
  }
  PutU16LE(&payload[offset], (uint16_t)temperature_cdeg);
  offset += 2U;
  PutU16LE(&payload[offset], imu_status);
  return SendFrame(UART_MSG_IMU_DATA, payload, sizeof(payload), 0U);
}

HAL_StatusTypeDef UARTProtocol_SendWheel(int64_t left_ticks,
                                         int64_t right_ticks,
                                         int32_t left_mm_s,
                                         int32_t right_mm_s,
                                         uint16_t encoder_status)
{
  uint8_t payload[UART_WHEEL_PAYLOAD_SIZE];
  uint16_t offset = 0U;

  PutU64LE(&payload[offset], UARTProtocol_GetTimestampUs());
  offset += 8U;
  PutU64LE(&payload[offset], (uint64_t)left_ticks);
  offset += 8U;
  PutU64LE(&payload[offset], (uint64_t)right_ticks);
  offset += 8U;
  PutU32LE(&payload[offset], (uint32_t)left_mm_s);
  offset += 4U;
  PutU32LE(&payload[offset], (uint32_t)right_mm_s);
  offset += 4U;
  PutU16LE(&payload[offset], encoder_status);
  return SendFrame(UART_MSG_WHEEL_STATE, payload, sizeof(payload), 0U);
}

HAL_StatusTypeDef UARTProtocol_SendSystem(uint16_t battery_mv,
                                          int16_t battery_ma,
                                          int16_t motor_temp_cdeg,
                                          uint8_t mode,
                                          uint8_t estop_state,
                                          uint32_t fault_bits,
                                          uint16_t last_cmd_age_ms)
{
  uint8_t payload[UART_SYSTEM_PAYLOAD_SIZE];
  uint16_t offset = 0U;

  PutU64LE(&payload[offset], UARTProtocol_GetTimestampUs());
  offset += 8U;
  PutU16LE(&payload[offset], battery_mv);
  offset += 2U;
  PutU16LE(&payload[offset], (uint16_t)battery_ma);
  offset += 2U;
  PutU16LE(&payload[offset], (uint16_t)motor_temp_cdeg);
  offset += 2U;
  payload[offset++] = mode;
  payload[offset++] = estop_state;
  PutU32LE(&payload[offset], fault_bits);
  offset += 4U;
  PutU16LE(&payload[offset], last_cmd_age_ms);
  return SendFrame(UART_MSG_SYSTEM_STATE, payload, sizeof(payload), 0U);
}

void UARTProtocol_RxEventCallback(UART_HandleTypeDef *uart, uint16_t position)
{
  uint16_t index;

  if ((protocol_uart == NULL) || (uart->Instance != protocol_uart->Instance))
  {
    return;
  }
  if (position > UART_RX_DMA_SIZE)
  {
    protocol_stats.malformed_frames++;
    return;
  }

  if (position > rx_dma_position)
  {
    for (index = rx_dma_position; index < position; index++)
    {
      PushRxByte(rx_dma_buffer[index]);
    }
  }
  else if (position < rx_dma_position)
  {
    for (index = rx_dma_position; index < UART_RX_DMA_SIZE; index++)
    {
      PushRxByte(rx_dma_buffer[index]);
    }
    for (index = 0U; index < position; index++)
    {
      PushRxByte(rx_dma_buffer[index]);
    }
  }
  rx_dma_position = position;
}

void UARTProtocol_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if ((protocol_uart == NULL) || (uart->Instance != protocol_uart->Instance))
  {
    return;
  }
  if (tx_dma_active != 0U)
  {
    tx_tail = (uint8_t)((tx_tail + 1U) % UART_TX_QUEUE_DEPTH);
    tx_dma_active = 0U;
    protocol_stats.tx_frames++;
  }
  StartNextTx();
}

void UARTProtocol_ErrorCallback(UART_HandleTypeDef *uart)
{
  if ((protocol_uart == NULL) || (uart->Instance != protocol_uart->Instance))
  {
    return;
  }
  protocol_stats.uart_errors++;
  rx_restart_pending = 1U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t position)
{
  UARTProtocol_RxEventCallback(uart, position);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  UARTProtocol_TxCpltCallback(uart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  UARTProtocol_ErrorCallback(uart);
}
