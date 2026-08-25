#include "drive.h"

static TIM_HandleTypeDef *pwm_htim;
static TIM_HandleTypeDef *left_htim;
static TIM_HandleTypeDef *right_htim;
static EncoderState encoder_state;
static uint16_t encoder_left_raw;
static uint32_t encoder_right_raw;
static uint32_t encoder_sample_ms;

static void SetCompare(uint32_t channel, float percent)
{
  uint32_t pulse;

  if (percent < 0.0f)
  {
    percent = -percent;
  }
  if (percent > 100.0f)
  {
    percent = 100.0f;
  }
  pulse = (uint32_t)((float)(__HAL_TIM_GET_AUTORELOAD(pwm_htim) + 1U) *
                     percent / 100.0f);
  __HAL_TIM_SET_COMPARE(pwm_htim, channel, pulse);
}

void Drive_Bind(TIM_HandleTypeDef *pwm_timer,
                TIM_HandleTypeDef *left_encoder,
                TIM_HandleTypeDef *right_encoder)
{
  pwm_htim = pwm_timer;
  left_htim = left_encoder;
  right_htim = right_encoder;
}

HAL_StatusTypeDef Drive_Start(void)
{
  if ((HAL_TIM_PWM_Start(pwm_htim, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(pwm_htim, TIM_CHANNEL_2) != HAL_OK) ||
      (HAL_TIM_Encoder_Start(left_htim, TIM_CHANNEL_ALL) != HAL_OK) ||
      (HAL_TIM_Encoder_Start(right_htim, TIM_CHANNEL_ALL) != HAL_OK))
  {
    return HAL_ERROR;
  }
  Encoder_Zero();
  Drive_Stop();
  return HAL_OK;
}

void Drive_SetLeftPercent(float percent)
{
  if (percent > 0.0f)
  {
    LL_GPIO_SetOutputPin(INB1_GPIO_Port, INB1_Pin);
    LL_GPIO_ResetOutputPin(INB2_GPIO_Port, INB2_Pin);
  }
  else if (percent < 0.0f)
  {
    LL_GPIO_ResetOutputPin(INB1_GPIO_Port, INB1_Pin);
    LL_GPIO_SetOutputPin(INB2_GPIO_Port, INB2_Pin);
  }
  SetCompare(TIM_CHANNEL_2, percent);
}

void Drive_SetRightPercent(float percent)
{
  if (percent > 0.0f)
  {
    LL_GPIO_SetOutputPin(INA1_GPIO_Port, INA1_Pin);
    LL_GPIO_ResetOutputPin(INA2_GPIO_Port, INA2_Pin);
  }
  else if (percent < 0.0f)
  {
    LL_GPIO_ResetOutputPin(INA1_GPIO_Port, INA1_Pin);
    LL_GPIO_SetOutputPin(INA2_GPIO_Port, INA2_Pin);
  }
  SetCompare(TIM_CHANNEL_1, percent);
}

void Drive_SetBothPercent(float percent)
{
  Drive_SetLeftPercent(percent);
  Drive_SetRightPercent(percent);
}

void Drive_Stop(void)
{
  SetCompare(TIM_CHANNEL_1, 0.0f);
  SetCompare(TIM_CHANNEL_2, 0.0f);
  LL_GPIO_ResetOutputPin(INA1_GPIO_Port, INA1_Pin);
  LL_GPIO_ResetOutputPin(INA2_GPIO_Port, INA2_Pin);
  LL_GPIO_ResetOutputPin(INB1_GPIO_Port, INB1_Pin);
  LL_GPIO_ResetOutputPin(INB2_GPIO_Port, INB2_Pin);
}

void Encoder_Zero(void)
{
  __HAL_TIM_SET_COUNTER(left_htim, 0U);
  __HAL_TIM_SET_COUNTER(right_htim, 0U);
  encoder_left_raw = 0U;
  encoder_right_raw = 0U;
  encoder_sample_ms = HAL_GetTick();
  encoder_state.left_total_ticks = 0;
  encoder_state.right_total_ticks = 0;
  encoder_state.left_delta_ticks = 0;
  encoder_state.right_delta_ticks = 0;
  encoder_state.left_speed_mm_s = 0.0f;
  encoder_state.right_speed_mm_s = 0.0f;
  encoder_state.sample_period_ms = 0U;
  encoder_state.sample_sequence++;
}

uint8_t Encoder_Update(void)
{
  uint16_t left_now;
  uint32_t right_now;
  uint32_t now_ms = HAL_GetTick();
  uint32_t elapsed_ms = now_ms - encoder_sample_ms;
  float left_raw_speed;
  float right_raw_speed;

  if (elapsed_ms < ENCODER_SAMPLE_PERIOD_MS)
  {
    return 0U;
  }

  left_now = (uint16_t)__HAL_TIM_GET_COUNTER(left_htim);
  right_now = (uint32_t)__HAL_TIM_GET_COUNTER(right_htim);
  encoder_state.left_delta_ticks = -(int32_t)(int16_t)(left_now - encoder_left_raw);
  encoder_state.right_delta_ticks = (int32_t)(encoder_right_raw - right_now);
  encoder_state.left_total_ticks += encoder_state.left_delta_ticks;
  encoder_state.right_total_ticks += encoder_state.right_delta_ticks;

  left_raw_speed = ((float)encoder_state.left_delta_ticks *
                    (float)WHEEL_CIRCUMFERENCE_UM) /
                   ((float)ENCODER_COUNTS_PER_REV * (float)elapsed_ms);
  right_raw_speed = ((float)encoder_state.right_delta_ticks *
                     (float)WHEEL_CIRCUMFERENCE_UM) /
                    ((float)ENCODER_COUNTS_PER_REV * (float)elapsed_ms);
  encoder_state.left_speed_mm_s +=
      0.35f * (left_raw_speed - encoder_state.left_speed_mm_s);
  encoder_state.right_speed_mm_s +=
      0.35f * (right_raw_speed - encoder_state.right_speed_mm_s);
  encoder_state.sample_period_ms = elapsed_ms;
  encoder_state.sample_sequence++;
  encoder_left_raw = left_now;
  encoder_right_raw = right_now;
  encoder_sample_ms = now_ms;
  return 1U;
}

int32_t Encoder_GetLeftCount(void)
{
  return (int32_t)encoder_state.left_total_ticks;
}

int32_t Encoder_GetRightCount(void)
{
  return (int32_t)encoder_state.right_total_ticks;
}

int64_t Encoder_GetLeftTotalTicks(void)
{
  return encoder_state.left_total_ticks;
}

int64_t Encoder_GetRightTotalTicks(void)
{
  return encoder_state.right_total_ticks;
}

const EncoderState *Encoder_GetState(void)
{
  return &encoder_state;
}
