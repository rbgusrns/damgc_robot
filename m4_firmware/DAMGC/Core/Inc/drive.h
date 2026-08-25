#ifndef DRIVE_H
#define DRIVE_H

#include "main.h"

#define ENCODER_COUNTS_PER_REV 5131U
#define WHEEL_CIRCUMFERENCE_UM 398982U
#define DRIVE_MEASURED_MAX_SPEED_MM_S 570.0f
#define ENCODER_SAMPLE_PERIOD_MS 20U

typedef struct
{
  int64_t left_total_ticks;
  int64_t right_total_ticks;
  int32_t left_delta_ticks;
  int32_t right_delta_ticks;
  float left_speed_mm_s;
  float right_speed_mm_s;
  uint32_t sample_period_ms;
  uint32_t sample_sequence;
} EncoderState;

void Drive_Bind(TIM_HandleTypeDef *pwm_timer,
                TIM_HandleTypeDef *left_encoder,
                TIM_HandleTypeDef *right_encoder);
HAL_StatusTypeDef Drive_Start(void);
void Drive_Stop(void);
void Drive_SetLeftPercent(float percent);
void Drive_SetRightPercent(float percent);
void Drive_SetBothPercent(float percent);
void Encoder_Zero(void);
uint8_t Encoder_Update(void);
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
int64_t Encoder_GetLeftTotalTicks(void);
int64_t Encoder_GetRightTotalTicks(void);
const EncoderState *Encoder_GetState(void);

#endif
