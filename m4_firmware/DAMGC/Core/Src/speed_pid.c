#include "speed_pid.h"

/* Best result saved before the comparison test:
 * target=275 mm/s, Kp=1.000, Ki=1.100, Kd=0.015,
 * acceleration=deceleration=1000 mm/s^2.
 */
#define SAVED_TUNING_KP 1.000f
#define SAVED_TUNING_KI 1.100f
#define SAVED_TUNING_KD 0.015f

static SpeedPIDStatus status =
{
  .kp = SAVED_TUNING_KP,
  .ki = SAVED_TUNING_KI,
  .kd = SAVED_TUNING_KD,
  .left_target_mm_s = 275.0f,
  .right_target_mm_s = 275.0f,
  .acceleration_mm_s2 = 1000.0f,
  .deceleration_mm_s2 = 1000.0f
};
static float left_integral;
static float right_integral;
static float left_previous_speed;
static float right_previous_speed;
static uint32_t last_encoder_sequence;

static float AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float StepToward(float current, float target, float max_step)
{
  if (target > current)
  {
    current += max_step;
    return (current > target) ? target : current;
  }
  if (target < current)
  {
    current -= max_step;
    return (current < target) ? target : current;
  }
  return current;
}

static float UpdateTargetRamp(float current, float command, float dt_s)
{
  float rate;

  /* Decelerate to zero before changing travel direction. */
  if ((current * command) < 0.0f)
  {
    return StepToward(current, 0.0f, status.deceleration_mm_s2 * dt_s);
  }

  rate = (AbsFloat(command) > AbsFloat(current)) ?
         status.acceleration_mm_s2 : status.deceleration_mm_s2;
  return StepToward(current, command, rate * dt_s);
}

static float Compute(float target, float measured, float previous,
                     float *integral, float dt_s)
{
  float error = target - measured;
  float derivative = -(measured - previous) / dt_s;
  float output;

  *integral += error * dt_s;
  output = (status.kp * error) + (status.ki * (*integral)) +
           (status.kd * derivative);

  /* PWM itself is physically limited to -100..100%. */
  if (output > 100.0f)
  {
    output = 100.0f;
    if (error > 0.0f)
    {
      *integral -= error * dt_s;
    }
  }
  else if (output < -100.0f)
  {
    output = -100.0f;
    if (error < 0.0f)
    {
      *integral -= error * dt_s;
    }
  }
  return output;
}

void SpeedPID_Init(void)
{
  status.active = 0U;
  Drive_Stop();
}

void SpeedPID_Start(void)
{
  left_integral = 0.0f;
  right_integral = 0.0f;
  left_previous_speed = 0.0f;
  right_previous_speed = 0.0f;
  status.left_speed_mm_s = 0.0f;
  status.right_speed_mm_s = 0.0f;
  status.left_ramped_target_mm_s = 0.0f;
  status.right_ramped_target_mm_s = 0.0f;
  status.left_output_pct = 0.0f;
  status.right_output_pct = 0.0f;
  last_encoder_sequence = Encoder_GetState()->sample_sequence;
  status.active = 1U;
}

void SpeedPID_Stop(void)
{
  status.active = 0U;
  status.left_output_pct = 0.0f;
  status.right_output_pct = 0.0f;
  Drive_Stop();
}

uint8_t SpeedPID_Process(void)
{
  const EncoderState *encoder = Encoder_GetState();
  float dt_s;

  if ((status.active == 0U) ||
      (encoder->sample_sequence == last_encoder_sequence) ||
      (encoder->sample_period_ms == 0U))
  {
    return 0U;
  }
  last_encoder_sequence = encoder->sample_sequence;
  dt_s = (float)encoder->sample_period_ms * 0.001f;
  status.left_ramped_target_mm_s = UpdateTargetRamp(
      status.left_ramped_target_mm_s, status.left_target_mm_s, dt_s);
  status.right_ramped_target_mm_s = UpdateTargetRamp(
      status.right_ramped_target_mm_s, status.right_target_mm_s, dt_s);
  status.left_speed_mm_s = encoder->left_speed_mm_s;
  status.right_speed_mm_s = encoder->right_speed_mm_s;
  status.left_output_pct = Compute(status.left_ramped_target_mm_s,
                                   status.left_speed_mm_s,
                                   left_previous_speed, &left_integral, dt_s);
  status.right_output_pct = Compute(status.right_ramped_target_mm_s,
                                    status.right_speed_mm_s,
                                    right_previous_speed, &right_integral, dt_s);
  left_previous_speed = status.left_speed_mm_s;
  right_previous_speed = status.right_speed_mm_s;
  Drive_SetLeftPercent(status.left_output_pct);
  Drive_SetRightPercent(status.right_output_pct);
  return 1U;
}

void SpeedPID_SetTarget(float target_mm_s)
{
  SpeedPID_SetTargets(target_mm_s, target_mm_s);
}

void SpeedPID_SetTargets(float left_target_mm_s, float right_target_mm_s)
{
  status.left_target_mm_s = left_target_mm_s;
  status.right_target_mm_s = right_target_mm_s;
}

void SpeedPID_SetGains(float kp, float ki, float kd)
{
  status.kp = (kp < 0.0f) ? 0.0f : kp;
  status.ki = (ki < 0.0f) ? 0.0f : ki;
  status.kd = (kd < 0.0f) ? 0.0f : kd;
}

void SpeedPID_SetAcceleration(float acceleration_mm_s2,
                              float deceleration_mm_s2)
{
  if (acceleration_mm_s2 > 0.0f)
  {
    status.acceleration_mm_s2 = acceleration_mm_s2;
  }
  if (deceleration_mm_s2 > 0.0f)
  {
    status.deceleration_mm_s2 = deceleration_mm_s2;
  }
}

const SpeedPIDStatus *SpeedPID_GetStatus(void)
{
  return &status;
}
