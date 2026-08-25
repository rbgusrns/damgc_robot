#include "motor.h"
#include "oled.h"
#include "sensor.h"
#include "variable.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MOTOR_SAMPLE_SEC             0.0005f
#define MOTOR_PULSE_TO_DIST_MM       0.016686047734493f
#define MOTOR_PULSE_TO_VEL_MM_S      33.37209546898672f
#define MOTOR_DEFAULT_ACCEL_MM_S2    3500.0f
#define MOTOR_DEFAULT_DECEL_MM_S2    15000.0f
#define MOTOR_DEFAULT_KP             0.42f
#define MOTOR_DEFAULT_KD             0.62f
#define MOTOR_MAX_PID_OUT            7700.0f
#define MOTOR_MIN_PID_OUT           -7700.0f
#define MOTOR_ESC_TEST_START_US      MOTOR_ESC_MIN_US
#define MOTOR_ESC_TEST_MAX_US        1900U
#define MOTOR_ESC_TEST_STEP_US       200U
#define MOTOR_PWM_TEST_STEP          1000U
#define MOTOR_LEFT_ENCODER_SIGN     -1
#define MOTOR_RIGHT_ENCODER_SIGN    -1

motor_feedback_t g_motor_feedback;
motor_axis_t g_lm;
motor_axis_t g_rm;

static float motor_fabsf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float motor_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static uint16_t motor_abs_clamp(int16_t pwm)
{
    int32_t value = pwm;

    if (value < 0) {
        value = -value;
    }

    if (value > (int32_t)MOTOR_PWM_PERIOD) {
        value = (int32_t)MOTOR_PWM_PERIOD;
    }

    return (uint16_t)value;
}

static int16_t motor_counter_to_delta(uint32_t count)
{
    uint16_t raw = (uint16_t)count;

    if (raw > 32767U) {
        return (int16_t)(raw - 65536U);
    }

    return (int16_t)raw;
}

static void motor_set_left_direction(int16_t pwm)
{
    if (pwm > 0) {
        LL_GPIO_SetOutputPin(L_DIR_GPIO_Port, L_DIR_Pin);
    } else if (pwm < 0) {
        LL_GPIO_ResetOutputPin(L_DIR_GPIO_Port, L_DIR_Pin);
    } else {
        LL_GPIO_ResetOutputPin(L_DIR_GPIO_Port, L_DIR_Pin);
    }
}

static void motor_set_right_direction(int16_t pwm)
{
    if (pwm > 0) {
        LL_GPIO_ResetOutputPin(R_DIR_GPIO_Port, R_DIR_Pin);
    } else if (pwm < 0) {
        LL_GPIO_SetOutputPin(R_DIR_GPIO_Port, R_DIR_Pin);
    } else {
        LL_GPIO_ResetOutputPin(R_DIR_GPIO_Port, R_DIR_Pin);
    }
}

void motor_set_pwm(int16_t left_pwm, int16_t right_pwm)
{
    motor_set_left_direction(left_pwm);
    motor_set_right_direction(right_pwm);

    LL_TIM_OC_SetCompareCH2(TIM1, motor_abs_clamp(left_pwm));
    LL_TIM_OC_SetCompareCH3(TIM1, motor_abs_clamp(right_pwm));
}

void motor_stop(void)
{
    motor_set_pwm(0, 0);
}

void suction_esc_set_us(uint16_t pulse_us)
{
    uint16_t clamped = pulse_us;

    if (clamped < MOTOR_ESC_MIN_US) {
        clamped = MOTOR_ESC_MIN_US;
    } else if (clamped > MOTOR_ESC_MAX_US) {
        clamped = MOTOR_ESC_MAX_US;
    }

    LL_TIM_OC_SetCompareCH1(TIM1, clamped);
}

static void motor_wait_switch_release(void)
{
    while ((SW_U == 0U) || (SW_D == 0U) || (SW_R == 0U) || (SW_L == 0U)) {
        LL_mDelay(10U);
    }
}

static void suction_esc_test_oled(uint16_t pulse_us, const char *line2, const char *line3)
{
    char pwm_line[22];

    snprintf(pwm_line, sizeof(pwm_line), "PWM: %4uus", (unsigned int)pulse_us);
    OLED_ClearBuffer();
    OLED_PrintCentered(0U, "SUCTION TEST");
    OLED_Print(1U, 0U, pwm_line);
    OLED_Print(2U, 0U, line2);
    OLED_Print(3U, 0U, line3);
    OLED_Update();
}

void suction_esc_calibration(void)
{
    if (SW_D != 0U) {
        return;
    }

    suction_esc_set_us(MOTOR_ESC_MAX_US);
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);

    while (SW_R != 0U) {
        if (SW_L == 0U) {
            suction_esc_set_us(MOTOR_ESC_MIN_US);
            LL_TIM_GenerateEvent_UPDATE(TIM1);
            while (SW_L == 0U) {
            }
        } else if (SW_U == 0U) {
            suction_esc_set_us(MOTOR_ESC_MAX_US);
            LL_TIM_GenerateEvent_UPDATE(TIM1);
            while (SW_U == 0U) {
            }
        }
    }

    suction_esc_set_us(MOTOR_ESC_MIN_US);
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    while (SW_R == 0U) {
    }
}

void suction_esc_test(void)
{
    uint16_t pulse_us = MOTOR_ESC_TEST_START_US;
    uint16_t last_print_us = 0U;

    motor_enable_control(OFF);
    motor_stop();
    suction_esc_set_us(MOTOR_ESC_SAFE_US);

    printf("SUCTION TEST START\r\n");
    printf("ARM LOW=%uus for 3s\r\n", (unsigned int)MOTOR_ESC_SAFE_US);
    suction_esc_test_oled(MOTOR_ESC_SAFE_US, "ARMING...", "WAIT 0.5 SEC");
    LL_mDelay(500U);

    printf("START=%uus MAX=%uus STEP=%uus\r\n",
           (unsigned int)MOTOR_ESC_TEST_START_US,
           (unsigned int)MOTOR_ESC_TEST_MAX_US,
           (unsigned int)MOTOR_ESC_TEST_STEP_US);
    printf("UP:+%uus LEFT:-%uus DOWN:STOP/EXIT\r\n",
           (unsigned int)MOTOR_ESC_TEST_STEP_US,
           (unsigned int)MOTOR_ESC_TEST_STEP_US);
    motor_wait_switch_release();
    suction_esc_test_oled(pulse_us, "UP:+1 LEFT:-1", "DOWN:STOP");

    while (SW_D != 0U) {
        if (SW_U == 0U) {
            if (pulse_us <= (MOTOR_ESC_TEST_MAX_US - MOTOR_ESC_TEST_STEP_US)) {
                pulse_us = (uint16_t)(pulse_us + MOTOR_ESC_TEST_STEP_US);
            } else {
                pulse_us = MOTOR_ESC_TEST_MAX_US;
            }
            LL_mDelay(120U);
        } else if (SW_L == 0U) {
            if (pulse_us >= (MOTOR_ESC_TEST_START_US + MOTOR_ESC_TEST_STEP_US)) {
                pulse_us = (uint16_t)(pulse_us - MOTOR_ESC_TEST_STEP_US);
            } else {
                pulse_us = MOTOR_ESC_TEST_START_US;
            }
            LL_mDelay(120U);
        }

        suction_esc_set_us(pulse_us);
        if (last_print_us != pulse_us) {
            printf("SUCTION PWM: %uus\r\n", (unsigned int)pulse_us);
            suction_esc_test_oled(pulse_us, "UP:+10 LEFT:-10", "DOWN:STOP");
            last_print_us = pulse_us;
        }
        LL_mDelay(40U);
    }

    suction_esc_set_us(MOTOR_ESC_SAFE_US);
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    printf("SUCTION TEST END, PWM=%uus\r\n", (unsigned int)MOTOR_ESC_SAFE_US);
    suction_esc_test_oled(MOTOR_ESC_SAFE_US, "TEST END", "SAFE OUTPUT");

    motor_wait_switch_release();
    LL_mDelay(300U);
}

void motor_vari_init(motor_axis_t *pm)
{
    memset((void *)pm, 0, sizeof(motor_axis_t));

    pm->fp32_accel = MOTOR_DEFAULT_ACCEL_MM_S2;
    pm->fp32_decel = (g_i32_decel > 0) ? (float)g_i32_decel : MOTOR_DEFAULT_DECEL_MM_S2;
    pm->fp32_handle = 1.0f;
    pm->fp32_kp = (g_fp32_motor_kp > 0.0f) ? g_fp32_motor_kp : MOTOR_DEFAULT_KP;
    pm->fp32_kd = (g_fp32_motor_kd > 0.0f) ? g_fp32_motor_kd : MOTOR_DEFAULT_KD;
}

void motor_reset_motion_state(void)
{
    motor_vari_init(&g_lm);
    motor_vari_init(&g_rm);

    g_motor_feedback.i16_left_delta = 0;
    g_motor_feedback.i16_right_delta = 0;
    g_motor_feedback.i32_left_count = 0;
    g_motor_feedback.i32_right_count = 0;
    g_motor_feedback.u32_tick_500us = 0U;
    g_u32_motor_isr_cnt = 0U;
    g_i32_timer_cnt = 0;

    LL_TIM_SetCounter(TIM3, 0U);
    LL_TIM_SetCounter(TIM4, 0U);
}

void decel_dist_compute(float fp32_cur_vel, float tar_vel, float acc, volatile float *fp32_decel_dist)
{
    float denom;

    if ((fp32_decel_dist == NULL) || (acc <= 0.0f)) {
        return;
    }

    denom = 2.0f * acc;
    *fp32_decel_dist = motor_fabsf((fp32_cur_vel * fp32_cur_vel) - (tar_vel * tar_vel)) / denom;
}

void diffvel_to_remaindist_cpt(float fp32_cur_vel, float tar_vel, float acc, volatile float *fp32_decel_dist)
{
    decel_dist_compute(fp32_cur_vel, tar_vel, acc, fp32_decel_dist);
}

void max_vel_compute(float dist, float minus_dist, float fp32_cur_vel, float acc, float limit_vel, float min_vel, volatile float *max_vel)
{
    float usable_dist;
    float fp32_vel;

    if ((max_vel == NULL) || (acc <= 0.0f)) {
        return;
    }

    usable_dist = dist - minus_dist;
    if (usable_dist < 0.0f) {
        usable_dist = 0.0f;
    }

    fp32_vel = sqrtf((fp32_cur_vel * fp32_cur_vel) + (acc * usable_dist));
    fp32_vel = motor_clampf(fp32_vel, min_vel, limit_vel);
    *max_vel = fp32_vel;
}

void move_to_move(float dist, float dec_dist, float tar_vel, float dec_vel, float acc)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    g_rm.fp32_accel = acc;
    g_lm.fp32_accel = acc;
    g_rm.fp32_decel = (g_i32_decel > 0) ? (float)g_i32_decel : MOTOR_DEFAULT_DECEL_MM_S2;
    g_lm.fp32_decel = g_rm.fp32_decel;

    g_rm.fp32_decel_dist = dec_dist;
    g_lm.fp32_decel_dist = dec_dist;
    g_rm.fp32_user_dist = dist;
    g_lm.fp32_user_dist = dist;
    g_rm.fp32_user_vel = tar_vel;
    g_lm.fp32_user_vel = tar_vel;
    g_rm.fp32_err_dist = g_rm.fp32_user_dist - g_rm.fp32_total_dist;
    g_lm.fp32_err_dist = g_lm.fp32_user_dist - g_lm.fp32_total_dist;
    g_rm.fp32_decel_vel = dec_vel;
    g_lm.fp32_decel_vel = dec_vel;
    g_rm.u16_decel_flag = ON;
    g_lm.u16_decel_flag = ON;
    g_Flag.move_state = ON;
    g_Flag.motor = ON;

    if (primask == 0U) {
        __enable_irq();
    }
}

void move_to_end(float dist, float fp32_vel, float acc)
{
    float fp32_decel_dist = 0.0f;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    diffvel_to_remaindist_cpt(fp32_vel, 0.0f, acc, &fp32_decel_dist);

    g_rm.fp32_accel = acc;
    g_lm.fp32_accel = acc;
    g_rm.fp32_decel_dist = fp32_decel_dist;
    g_lm.fp32_decel_dist = fp32_decel_dist;
    g_rm.fp32_user_dist = dist;
    g_lm.fp32_user_dist = dist;
    g_rm.fp32_user_vel = fp32_vel;
    g_lm.fp32_user_vel = fp32_vel;
    g_rm.fp32_err_dist = g_rm.fp32_user_dist - g_rm.fp32_total_dist;
    g_lm.fp32_err_dist = g_lm.fp32_user_dist - g_lm.fp32_total_dist;
    g_rm.fp32_decel_vel = 0.0f;
    g_lm.fp32_decel_vel = 0.0f;
    g_rm.u16_decel_flag = ON;
    g_lm.u16_decel_flag = ON;
    g_Flag.move_state = OFF;
    g_Flag.motor = ON;

    if (primask == 0U) {
        __enable_irq();
    }
}

void motor_enable_control(uint8_t enable)
{
    if (enable != 0U) {
        g_Flag.motor_start = ON;
        g_Flag.motor = ON;
    } else {
        g_Flag.motor_start = OFF;
        motor_stop();
        g_lm.fp32_pid_out = 0.0f;
        g_rm.fp32_pid_out = 0.0f;
    }
}

void motor_init(void)
{
    motor_reset_motion_state();

    motor_stop();

    LL_TIM_SetCounter(TIM3, 0U);
    LL_TIM_SetCounter(TIM4, 0U);
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_EnableCounter(TIM4);

    LL_TIM_OC_SetCompareCH2(TIM1, 0U);
    LL_TIM_OC_SetCompareCH3(TIM1, 0U);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);

    LL_TIM_ClearFlag_UPDATE(TIM6);
    LL_TIM_SetCounter(TIM6, 0U);
    LL_TIM_EnableIT_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);
}

static void motor_update_axis_feedback(motor_axis_t *pm, int16_t delta)
{
    pm->i16_qep_delta = delta;
    pm->i32_qep_total += delta;
    pm->fp32_tick_dist = (float)delta * MOTOR_PULSE_TO_DIST_MM;
    pm->fp32_dist_sum += pm->fp32_tick_dist;
    pm->fp32_gone_distance += pm->fp32_tick_dist;
    pm->fp32_turnmark_check_dist += pm->fp32_tick_dist;
    pm->fp32_cross_check_dist += pm->fp32_tick_dist;

    pm->fp32_cur_vel[1] = pm->fp32_cur_vel[0];
    pm->fp32_cur_vel[0] = (float)delta * MOTOR_PULSE_TO_VEL_MM_S;
    pm->fp32_cur_vel_avr = (pm->fp32_cur_vel[0] + pm->fp32_cur_vel[1]) * 0.5f;
}

static void motor_update_distance_state(void)
{
    const float total = (g_lm.fp32_dist_sum + g_rm.fp32_dist_sum) * 0.5f;

    g_lm.fp32_total_dist = total;
    g_rm.fp32_total_dist = total;
    g_lm.fp32_err_dist = g_lm.fp32_user_dist - g_lm.fp32_total_dist;
    g_rm.fp32_err_dist = g_rm.fp32_user_dist - g_rm.fp32_total_dist;
}

static void motor_update_decel_state(void)
{
    if ((g_rm.u16_decel_flag == ON) && (g_rm.fp32_decel_dist >= motor_fabsf(g_rm.fp32_err_dist))) {
        g_rm.fp32_user_vel = g_rm.fp32_decel_vel;
        g_lm.fp32_user_vel = g_lm.fp32_decel_vel;
        g_rm.u16_decel_flag = OFF;
        g_lm.u16_decel_flag = OFF;
        g_Flag.speed_up = OFF;
        g_Flag.speed_up_start = OFF;
        g_rm.fp32_accel = g_rm.fp32_decel;
        g_lm.fp32_accel = g_lm.fp32_decel;
    }

    if ((g_lm.u16_decel_flag == ON) && (g_lm.fp32_decel_dist >= motor_fabsf(g_lm.fp32_err_dist))) {
        g_rm.fp32_user_vel = g_rm.fp32_decel_vel;
        g_lm.fp32_user_vel = g_lm.fp32_decel_vel;
        g_rm.u16_decel_flag = OFF;
        g_lm.u16_decel_flag = OFF;
        g_Flag.speed_up = OFF;
        g_Flag.speed_up_start = OFF;
        g_rm.fp32_accel = g_rm.fp32_decel;
        g_lm.fp32_accel = g_lm.fp32_decel;
    }
}

static void motor_update_velocity_ramp(motor_axis_t *pm)
{
    const float step = pm->fp32_accel * MOTOR_SAMPLE_SEC;

    if (pm->fp32_user_vel > pm->fp32_next_vel) {
        pm->fp32_next_vel += step;
        if (pm->fp32_user_vel < pm->fp32_next_vel) {
            pm->fp32_next_vel = pm->fp32_user_vel;
        }
    } else if (pm->fp32_user_vel < pm->fp32_next_vel) {
        pm->fp32_next_vel -= step;
        if (pm->fp32_user_vel > pm->fp32_next_vel) {
            pm->fp32_next_vel = pm->fp32_user_vel;
        }
    }
}

static void motor_update_pid(motor_axis_t *pm)
{
    uint8_t i;
    float diff_mix;
    const float target_vel = pm->fp32_next_vel * pm->fp32_handle;

    for (i = 3U; i > 0U; i--) {
        pm->fp32_err_vel[i] = pm->fp32_err_vel[i - 1U];
    }

    pm->fp32_err_vel[0] = target_vel - pm->fp32_cur_vel_avr;
    diff_mix = ((pm->fp32_err_vel[0] - pm->fp32_err_vel[3]) +
                ((pm->fp32_err_vel[1] - pm->fp32_err_vel[2]) * 3.0f));
    pm->fp32_proportional = pm->fp32_kp * pm->fp32_err_vel[0];
    pm->fp32_derivative = pm->fp32_kd * diff_mix;
    pm->fp32_pid_out += pm->fp32_proportional + pm->fp32_derivative;
    pm->fp32_pid_out = motor_clampf(pm->fp32_pid_out, MOTOR_MIN_PID_OUT, MOTOR_MAX_PID_OUT);

    pm->fp32_pid_result = (pm->fp32_pid_out / MOTOR_MAX_PID_OUT) * (float)MOTOR_PWM_PERIOD;
}

static int16_t motor_pid_to_pwm(float fp32_pid_result)
{
    if (fp32_pid_result > (float)MOTOR_PWM_PERIOD) {
        return (int16_t)MOTOR_PWM_PERIOD;
    }

    if (fp32_pid_result < -(float)MOTOR_PWM_PERIOD) {
        return -(int16_t)MOTOR_PWM_PERIOD;
    }

    return (int16_t)fp32_pid_result;
}

static void motor_position_to_vel(void)
{
    if (g_Flag.start_flag == OFF) {
        return;
    }

    if (g_Flag.err == ON) {
        g_Flag.fast_flag = OFF;

        if (g_fp32_user_vel > 2200.0f) {
            g_fp32_user_vel = 2200.0f;
        }

        g_rm.fp32_user_vel = g_fp32_user_vel;
        g_lm.fp32_user_vel = g_fp32_user_vel;
    }

    if ((g_Flag.speed_up == ON) &&
        (g_i32_mark_cnt >= 0) &&
        (g_i32_mark_cnt < 256) &&
        (g_fast_info[g_i32_mark_cnt].fp32_vel > 0.0f)) {
        g_lm.fp32_user_vel = g_fast_info[g_i32_mark_cnt].fp32_vel;
        g_rm.fp32_user_vel = g_lm.fp32_user_vel;
    }
}

void motor_500us_irq_handler(void)
{
    int16_t i16_left_delta;
    int16_t i16_right_delta;

    i16_left_delta = (int16_t)(MOTOR_LEFT_ENCODER_SIGN * motor_counter_to_delta(LL_TIM_GetCounter(TIM3)));
    i16_right_delta = (int16_t)(MOTOR_RIGHT_ENCODER_SIGN * motor_counter_to_delta(LL_TIM_GetCounter(TIM4)));

    LL_TIM_SetCounter(TIM3, 0U);
    LL_TIM_SetCounter(TIM4, 0U);

    g_motor_feedback.i16_left_delta = i16_left_delta;
    g_motor_feedback.i16_right_delta = i16_right_delta;
    g_motor_feedback.i32_left_count += i16_left_delta;
    g_motor_feedback.i32_right_count += i16_right_delta;
    g_motor_feedback.u32_tick_500us++;

    g_Flag.motor_ISR_flag = ON;
    g_u32_motor_isr_cnt++;

    motor_update_axis_feedback(&g_lm, i16_left_delta);
    motor_update_axis_feedback(&g_rm, i16_right_delta);
    motor_update_distance_state();
    motor_update_decel_state();
    position_PID();
    motor_position_to_vel();

    if (g_Flag.motor_start == ON) {
        motor_update_velocity_ramp(&g_rm);
        motor_update_velocity_ramp(&g_lm);
        motor_update_pid(&g_rm);
        motor_update_pid(&g_lm);
        motor_set_pwm(motor_pid_to_pwm(g_lm.fp32_pid_result), motor_pid_to_pwm(g_rm.fp32_pid_result));
    } else {
        g_lm.fp32_pid_out = 0.0f;
        g_rm.fp32_pid_out = 0.0f;
        g_lm.fp32_pid_result = 0.0f;
        g_rm.fp32_pid_result = 0.0f;
        motor_stop();
    }

    if (g_Flag.race_start == ON) {
        g_i32_timer_cnt++;
    }

    sensor_scan_cycle_start();
}

static void motor_pwm_test_apply(uint8_t selected_motor, uint16_t pwm)
{
    if (selected_motor == 0U) {
        motor_set_pwm((int16_t)pwm, 0);
    } else {
        motor_set_pwm(0, (int16_t)pwm);
    }
}

static void motor_pwm_test_oled(uint8_t selected_motor, uint16_t pwm)
{
    char status_line[22];

    snprintf(status_line,
             sizeof(status_line),
             "SEL:%c PWM:%5u",
             (selected_motor == 0U) ? 'L' : 'R',
             (unsigned int)pwm);
    OLED_ClearBuffer();
    OLED_PrintCentered(0U, "MOTOR PWM TEST");
    OLED_Print(1U, 0U, status_line);
    OLED_Print(2U, 0U, "U:+1000 L:L R:R");
    OLED_Print(3U, 0U, "D:STOP/EXIT");
    OLED_Update();
}

void motor_pwm_test(void)
{
    uint8_t selected_motor = 0U;
    uint16_t pwm = 0U;
    uint32_t tim6_it_enabled = LL_TIM_IsEnabledIT_UPDATE(TIM6);
    uint32_t tim6_irq_enabled = NVIC_GetEnableIRQ(TIM6_DAC_IRQn);

    motor_enable_control(OFF);
    LL_TIM_DisableIT_UPDATE(TIM6);
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    LL_TIM_ClearFlag_UPDATE(TIM6);
    motor_stop();
    motor_wait_switch_release();

    printf("MOTOR PWM TEST START\r\n");
    printf("UP:+%u LEFT:L RIGHT:R DOWN:STOP/EXIT\r\n",
           (unsigned int)MOTOR_PWM_TEST_STEP);
    motor_pwm_test_oled(selected_motor, pwm);

    while (SW_D != 0U) {
        uint8_t changed = 0U;

        if (SW_U == 0U) {
            if (pwm <= (MOTOR_PWM_PERIOD - MOTOR_PWM_TEST_STEP)) {
                pwm = (uint16_t)(pwm + MOTOR_PWM_TEST_STEP);
            } else {
                pwm = MOTOR_PWM_PERIOD;
            }
            changed = 1U;
            while (SW_U == 0U) {
            }
        } else if (SW_L == 0U) {
            selected_motor = 0U;
            changed = 1U;
            while (SW_L == 0U) {
            }
        } else if (SW_R == 0U) {
            selected_motor = 1U;
            changed = 1U;
            while (SW_R == 0U) {
            }
        }

        if (changed != 0U) {
            motor_pwm_test_apply(selected_motor, pwm);
            motor_pwm_test_oled(selected_motor, pwm);
            printf("MOTOR PWM SEL=%c PWM=%u\r\n",
                   (selected_motor == 0U) ? 'L' : 'R',
                   (unsigned int)pwm);
        }

        LL_mDelay(10U);
    }

    motor_stop();
    printf("MOTOR PWM TEST END\r\n");
    OLED_ClearBuffer();
    OLED_PrintCentered(0U, "MOTOR PWM TEST");
    OLED_Print(1U, 0U, "STOP PWM:0");
    OLED_Print(3U, 0U, "TEST END");
    OLED_Update();

    while (SW_D == 0U) {
    }

    LL_TIM_ClearFlag_UPDATE(TIM6);
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    if (tim6_it_enabled != 0U) {
        LL_TIM_EnableIT_UPDATE(TIM6);
    }
    if (tim6_irq_enabled != 0U) {
        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
    LL_mDelay(300U);
}

void motor_encoder_test(void)
{
    int32_t prev_left_count;
    int32_t prev_right_count;
    uint32_t prev_tick;

    motor_enable_control(OFF);
    motor_reset_motion_state();

    prev_left_count = g_motor_feedback.i32_left_count;
    prev_right_count = g_motor_feedback.i32_right_count;
    prev_tick = g_motor_feedback.u32_tick_500us;

    printf("ENC TEST START\r\n");
    printf("rotate wheels by hand, press DOWN to exit\r\n");

    while (SW_D != 0U) {
        int32_t i32_left_count;
        int32_t i32_right_count;
        int32_t i32_left_delta;
        int32_t i32_right_delta;
        uint32_t tick;
        uint32_t tick_delta;
        int32_t left_vel;
        int32_t right_vel;
        char left_line[OLED_TEXT_COLUMNS + 1U];
        char right_line[OLED_TEXT_COLUMNS + 1U];

        LL_mDelay(100U);

        i32_left_count = g_motor_feedback.i32_left_count;
        i32_right_count = g_motor_feedback.i32_right_count;
        tick = g_motor_feedback.u32_tick_500us;

        i32_left_delta = i32_left_count - prev_left_count;
        i32_right_delta = i32_right_count - prev_right_count;
        tick_delta = tick - prev_tick;

        if (tick_delta == 0U) {
            tick_delta = 1U;
        }

        left_vel = (int32_t)(((float)i32_left_delta * MOTOR_PULSE_TO_DIST_MM) /
                             ((float)tick_delta * MOTOR_SAMPLE_SEC));
        right_vel = (int32_t)(((float)i32_right_delta * MOTOR_PULSE_TO_DIST_MM) /
                              ((float)tick_delta * MOTOR_SAMPLE_SEC));

        printf("ENC L cnt=%ld d=%ld v=%ldmm/s | R cnt=%ld d=%ld v=%ldmm/s\r\n",
               (long)i32_left_count,
               (long)i32_left_delta,
               (long)left_vel,
               (long)i32_right_count,
               (long)i32_right_delta,
               (long)right_vel);

        snprintf(left_line,
                 sizeof(left_line),
                 "L:%ld V:%ldmm/s",
                 (long)i32_left_count,
                 (long)left_vel);
        snprintf(right_line,
                 sizeof(right_line),
                 "R:%ld V:%ldmm/s",
                 (long)i32_right_count,
                 (long)right_vel);
        OLED_ClearBuffer();
        OLED_PrintCentered(0U, "ENCODER TEST");
        OLED_Print(1U, 0U, left_line);
        OLED_Print(2U, 0U, right_line);
        OLED_Print(3U, 0U, "DOWN: EXIT");
        OLED_Update();

        prev_left_count = i32_left_count;
        prev_right_count = i32_right_count;
        prev_tick = tick;
    }

    while (SW_D == 0U) {
        LL_mDelay(10U);
    }

    printf("ENC TEST END\r\n");
}
