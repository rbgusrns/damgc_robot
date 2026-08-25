#ifndef SPEED_PID_H
#define SPEED_PID_H

#include "drive.h"

typedef struct
{
  float kp;
  float ki;
  float kd;
  float left_target_mm_s;
  float right_target_mm_s;
  float left_ramped_target_mm_s;
  float right_ramped_target_mm_s;
  float acceleration_mm_s2;
  float deceleration_mm_s2;
  float left_speed_mm_s;
  float right_speed_mm_s;
  float left_output_pct;
  float right_output_pct;
  uint8_t active;
} SpeedPIDStatus;

void SpeedPID_Init(void);
void SpeedPID_Start(void);
void SpeedPID_Stop(void);
uint8_t SpeedPID_Process(void);
void SpeedPID_SetTarget(float target_mm_s);
void SpeedPID_SetTargets(float left_target_mm_s, float right_target_mm_s);
void SpeedPID_SetGains(float kp, float ki, float kd);
void SpeedPID_SetAcceleration(float acceleration_mm_s2,
                              float deceleration_mm_s2);
const SpeedPIDStatus *SpeedPID_GetStatus(void);

#endif
