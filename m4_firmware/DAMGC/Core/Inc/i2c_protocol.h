#ifndef I2C_PROTOCOL_H
#define I2C_PROTOCOL_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define I2C_PROTOCOL_ADDRESS_7BIT 0x42U
#define I2C_PROTOCOL_MAX_FRAME_SIZE 64U
#define I2C_PROTOCOL_RECORD_SIZE (2U + I2C_PROTOCOL_MAX_FRAME_SIZE)
#define I2C_PROTOCOL_COMMAND_SIZE 20U

typedef struct
{
  uint32_t published_frames;
  uint32_t read_requests;
  uint32_t read_frames;
  uint32_t dropped_frames;
  uint32_t bus_errors;
  uint32_t command_writes;
} I2CProtocolStats;

HAL_StatusTypeDef I2CProtocol_Init(I2C_HandleTypeDef *i2c);
void I2CProtocol_PublishFrame(const uint8_t *frame, uint16_t length);
const I2CProtocolStats *I2CProtocol_GetStats(void);

#endif
