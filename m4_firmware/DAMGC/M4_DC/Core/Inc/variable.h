#ifndef __APP_VARIABLE_H__
#define __APP_VARIABLE_H__

#include "main.h"
#include "struct.h"
#include <stdint.h>

#ifndef ON
#define ON  1U
#endif

#ifndef OFF
#define OFF 0U
#endif

#define SENSOR_ON_THRESHOLD   35.0f
#define SENSOR_STATE_THRESHOLD 60.0f
#define POS_END 14500.0f
#define HANDLE_CENTER 7250.0f
#define MAX_SPEED_HANDLE 1.15f
#define POS_KP_UP 0.7f
#define POS_KD_UP 4.4f
#define POS_KI_UP 0.0f

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} led_pin_t;

#define SW_U (((SW_U_GPIO_Port->IDR & SW_U_Pin) != 0U) ? 1U : 0U)
#define SW_D (((SW_D_GPIO_Port->IDR & SW_D_Pin) != 0U) ? 1U : 0U)
#define SW_L (((SW_L_GPIO_Port->IDR & SW_L_Pin) != 0U) ? 1U : 0U)
#define SW_R (((SW_R_GPIO_Port->IDR & SW_R_Pin) != 0U) ? 1U : 0U)

extern sen_t g_sen[16];
extern race_info search_info[256];
extern fast_run_str g_fast_info[256];
extern error_str g_err;
extern position_t g_pos;
extern turnmark_t g_lmark;
extern turnmark_t g_rmark;
extern bit_field_flag_t g_Flag;
extern volatile uint32_t g_u32_isr_cnt;
extern volatile uint32_t g_u32_adc_error_cnt;
extern volatile uint32_t g_u32_motor_isr_cnt;
extern volatile int32_t g_i32_timer_cnt;
extern volatile int32_t g_i32_mark_cnt;
extern volatile int32_t g_i32_total_cnt;
extern volatile int32_t g_i32_err_cnt;
extern volatile int32_t g_i32_speed_up_cnt;
extern volatile uint16_t g_u16_pos_cnt;
extern volatile uint16_t g_u16_sen_enable;
extern volatile uint16_t g_u16_sen_state;
extern volatile uint16_t g_u16_turnmark_cnt;
extern volatile uint16_t g_u16_turn_dist;
extern volatile uint16_t g_u16_turnmark_limit;
extern volatile float g_fp32_user_vel;
extern volatile float g_fp32_user_acc;
extern volatile float g_fp32_end_vel;
extern volatile float g_fp32_end_acc;
extern volatile float g_fp32_endturn_acc;
extern volatile float g_fp32_fast_vel_limit;
extern volatile float g_fp32_straight_dist;
extern volatile float g_fp32_turn_angle;
extern volatile float g_fp32_current_angle;
extern volatile float g_fp32_gyro_dps_z;
extern volatile float g_fp32_gyro_tick_z;
extern volatile float g_fp32_sensor_value_max;
extern volatile float g_fp32_turnmark_dist;
extern volatile float g_fp32_straight_mark_dist;
extern volatile float g_fp32_mark_dist;
extern volatile float g_fp32_turn_threshold;
extern volatile float g_fp32_out_corner_limit;
extern volatile float g_fp32_in_corner_limit;
extern volatile float g_fp32_out_corner_fast_limit;
extern volatile float g_fp32_in_corner_fast_limit;
extern volatile float g_fp32_motor_kp;
extern volatile float g_fp32_motor_kd;
extern volatile int32_t g_i32_decel;
extern volatile int32_t g_i32_fast_error_flag;
extern volatile float g_fp32_left_handle;
extern volatile float g_fp32_right_handle;
extern volatile float g_fp32_handle_dec_max;
extern volatile float g_fp32_handle_acc_max;
extern volatile float g_fp32_handle_dec_step;
extern volatile float g_fp32_handle_acc_step;
extern volatile float g_fp32_left_handle_temp;
extern volatile float g_fp32_right_handle_temp;

void Variable_Init(void);

#endif /* __APP_VARIABLE_H__ */
