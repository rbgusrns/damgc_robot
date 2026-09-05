#include "i2c_protocol.h"
#include "uart_protocol.h"

#include <string.h>

#define I2C_PROTOCOL_QUEUE_DEPTH 32U

typedef struct
{
  uint8_t data[I2C_PROTOCOL_MAX_FRAME_SIZE];
  uint8_t length;
} I2CProtocolFrame;

static I2C_HandleTypeDef *protocol_i2c;
static I2CProtocolFrame frame_queue[I2C_PROTOCOL_QUEUE_DEPTH];
static volatile uint8_t queue_head;
static volatile uint8_t queue_tail;
static volatile uint8_t tx_pending;
static uint8_t tx_record[I2C_PROTOCOL_RECORD_SIZE];
static uint8_t rx_command[I2C_PROTOCOL_COMMAND_SIZE];
static I2CProtocolStats protocol_stats;

static void RestoreInterruptState(uint32_t primask)
{
  if ((primask & 1U) == 0U)
  {
    __enable_irq();
  }
}

static uint8_t QueueCount(void)
{
  if (queue_head >= queue_tail)
  {
    return (uint8_t)(queue_head - queue_tail);
  }
  return (uint8_t)(I2C_PROTOCOL_QUEUE_DEPTH - queue_tail + queue_head);
}

static void CompletePendingTx(void)
{
  if ((tx_pending != 0U) && (queue_tail != queue_head))
  {
    queue_tail = (uint8_t)((queue_tail + 1U) % I2C_PROTOCOL_QUEUE_DEPTH);
    protocol_stats.read_frames++;
  }
  tx_pending = 0U;
}

HAL_StatusTypeDef I2CProtocol_Init(I2C_HandleTypeDef *i2c)
{
  protocol_i2c = i2c;
  queue_head = 0U;
  queue_tail = 0U;
  tx_pending = 0U;
  memset(&protocol_stats, 0, sizeof(protocol_stats));
  return HAL_I2C_EnableListen_IT(protocol_i2c);
}

void I2CProtocol_PublishFrame(const uint8_t *frame, uint16_t length)
{
  uint8_t next_head;
  uint32_t primask;

  if ((frame == NULL) || (length == 0U) ||
      (length > I2C_PROTOCOL_MAX_FRAME_SIZE))
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  next_head = (uint8_t)((queue_head + 1U) % I2C_PROTOCOL_QUEUE_DEPTH);
  if (next_head == queue_tail)
  {
    protocol_stats.dropped_frames++;
    RestoreInterruptState(primask);
    return;
  }

  memcpy(frame_queue[queue_head].data, frame, length);
  frame_queue[queue_head].length = (uint8_t)length;
  queue_head = next_head;
  protocol_stats.published_frames++;
  RestoreInterruptState(primask);
}

const I2CProtocolStats *I2CProtocol_GetStats(void)
{
  return &protocol_stats;
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *i2c,
                          uint8_t transfer_direction,
                          uint16_t address_match_code)
{
  HAL_StatusTypeDef status;

  (void)address_match_code;
  if ((protocol_i2c == NULL) || (i2c->Instance != protocol_i2c->Instance))
  {
    return;
  }

  if (transfer_direction == I2C_DIRECTION_RECEIVE)
  {
    protocol_stats.read_requests++;
    memset(tx_record, 0, sizeof(tx_record));
    if (queue_tail != queue_head)
    {
      tx_record[0] = frame_queue[queue_tail].length;
      tx_record[1] = QueueCount();
      memcpy(&tx_record[2], frame_queue[queue_tail].data,
             frame_queue[queue_tail].length);
      tx_pending = 1U;
    }
    else
    {
      tx_pending = 0U;
    }

    status = HAL_I2C_Slave_Seq_Transmit_IT(
        i2c, tx_record, sizeof(tx_record), I2C_LAST_FRAME);
  }
  else
  {
    tx_pending = 0U;
    memset(rx_command, 0, sizeof(rx_command));
    status = HAL_I2C_Slave_Seq_Receive_IT(
        i2c, rx_command, sizeof(rx_command), I2C_LAST_FRAME);
  }

  if (status != HAL_OK)
  {
    tx_pending = 0U;
    protocol_stats.bus_errors++;
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *i2c)
{
  if ((protocol_i2c == NULL) || (i2c->Instance != protocol_i2c->Instance))
  {
    return;
  }

  UARTProtocol_PushRxData(rx_command, sizeof(rx_command));
  protocol_stats.command_writes++;
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *i2c)
{
  if ((protocol_i2c == NULL) || (i2c->Instance != protocol_i2c->Instance))
  {
    return;
  }

  CompletePendingTx();
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *i2c)
{
  if ((protocol_i2c != NULL) &&
      (i2c->Instance == protocol_i2c->Instance))
  {
    (void)HAL_I2C_EnableListen_IT(i2c);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c)
{
  uint32_t error;
  uint16_t transferred;
  uint16_t required;

  if ((protocol_i2c == NULL) || (i2c->Instance != protocol_i2c->Instance))
  {
    return;
  }

  error = HAL_I2C_GetError(i2c);
  if (((error & HAL_I2C_ERROR_AF) != 0U) && (tx_pending != 0U))
  {
    transferred = (i2c->XferCount < I2C_PROTOCOL_RECORD_SIZE) ?
                  (uint16_t)(I2C_PROTOCOL_RECORD_SIZE - i2c->XferCount) : 0U;
    required = (uint16_t)(2U + tx_record[0]);
    if (transferred >= required)
    {
      CompletePendingTx();
    }
    else
    {
      tx_pending = 0U;
    }
  }
  else
  {
    tx_pending = 0U;
  }

  if ((error & ~HAL_I2C_ERROR_AF) != 0U)
  {
    protocol_stats.bus_errors++;
  }
  (void)HAL_I2C_EnableListen_IT(i2c);
}
