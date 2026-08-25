#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include <stdint.h>

#define MOTOR_PWM_PERIOD 2000U
#define MOTOR_ESC_MIN_US 1000U
#define MOTOR_ESC_MAX_US 2000U
#define MOTOR_ESC_SAFE_US 1000U

typedef struct {
    volatile int16_t i16_qep_delta;
    volatile int32_t i32_qep_total;
    volatile float fp32_tick_dist;
    volatile float fp32_dist_sum;
    volatile float fp32_gone_distance;
    volatile float fp32_turnmark_check_dist;
    volatile float fp32_cross_check_dist;
    volatile float fp32_total_dist;
    volatile float fp32_err_dist;
    volatile float fp32_user_dist;
    volatile float fp32_decel_dist;
    volatile float fp32_decel_vel;
    volatile float fp32_cur_vel[2];
    volatile float fp32_cur_vel_avr;
    volatile float fp32_user_vel;
    volatile float fp32_next_vel;
    volatile float fp32_accel;
    volatile float fp32_decel;
    volatile float fp32_handle;
    volatile float fp32_kp;
    volatile float fp32_kd;
    volatile float fp32_err_vel[4];
    volatile float fp32_proportional;
    volatile float fp32_derivative;
    volatile float fp32_pid_out;
    volatile float fp32_pid_result;
    volatile uint16_t u16_decel_flag;
} motor_axis_t;

typedef struct {
    volatile int16_t i16_left_delta;
    volatile int16_t i16_right_delta;
    volatile int32_t i32_left_count;
    volatile int32_t i32_right_count;
    volatile uint32_t u32_tick_500us;
} motor_feedback_t;

extern motor_feedback_t g_motor_feedback;
extern motor_axis_t g_lm;
extern motor_axis_t g_rm;

void motor_init(void);
void motor_vari_init(motor_axis_t *pm);
void motor_stop(void);
void motor_set_pwm(int16_t left_pwm, int16_t right_pwm);
void motor_enable_control(uint8_t enable);
void motor_reset_motion_state(void);
void move_to_move(float dist, float dec_dist, float tar_vel, float dec_vel, float acc);
void move_to_end(float dist, float fp32_vel, float acc);
void decel_dist_compute(float fp32_cur_vel, float tar_vel, float acc, volatile float *fp32_decel_dist);
void diffvel_to_remaindist_cpt(float fp32_cur_vel, float tar_vel, float acc, volatile float *fp32_decel_dist);
void max_vel_compute(float dist, float minus_dist, float fp32_cur_vel, float acc, float limit_vel, float min_vel, volatile float *max_vel);
void suction_esc_set_us(uint16_t pulse_us);
void suction_esc_calibration(void);
void suction_esc_test(void);
void motor_500us_irq_handler(void);
void motor_encoder_test(void);
void motor_pwm_test(void);

#endif /* __MOTOR_H__ */
