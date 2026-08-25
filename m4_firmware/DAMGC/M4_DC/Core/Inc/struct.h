#ifndef __APP_STRUCT_H__
#define __APP_STRUCT_H__

#include <stdint.h>

typedef volatile struct {
    float fp32_4095_value;
    float fp32_4095_min_value;
    float fp32_4095_max_value;
    float fp32_127_value;
    float fp32_on_off_value;
    float fp32_weight;
    uint16_t u16_active_arr;
    uint16_t u16_passive_arr;
} sen_t;

typedef volatile struct {
    float fp32_sensor_sum;
    float fp32_position_sum;
    float fp32_weighted_sum;
    float fp32_temp_pos;
    float fp32_temp_position;
    float fp32_pos_iir_puted;
    float fp32_pos_iir_puting;
    float fp32_pos_iir_output;
    float fp32_past_pos[4];
    float fp32_kp;
    float fp32_ki;
    float fp32_kd;
    float fp32_proportion_val;
    float fp32_differential_val;
    float fp32_integral_val;
    float fp32_pid_out;
    uint16_t u16_enable;
    uint16_t u16_state;
    uint16_t u16_current_state;
    uint16_t u16_past_state;
} position_t;

typedef struct {
    int32_t i32_dist;
    int32_t i32_turn_way;
    int32_t i32_left_dist;
    int32_t i32_right_dist;
    int32_t i32_cross_check_dist;
    int32_t i32_turn_dir;
    int32_t i32_turn_cnt;
    int32_t i32_accel;
    float fp32_in_vel;
    float fp32_vel;
    float fp32_out_vel;
    float fp32_decel_dist;
    float fp32_motion_dist;
} race_info;

typedef volatile struct {
    float fp32_left_dist;
    float fp32_right_dist;
    float fp32_angle;
    float fp32_decel_dist;
    float fp32_motion_dist;
    float fp32_accel;
    float fp32_in_vel;
    float fp32_vel;
    float fp32_out_vel;
    uint16_t u16_dist;
    uint16_t u16_turn_way;
    uint16_t u16_turn_dir;
    uint16_t u16_turn_cnt;
} fast_run_str;

typedef volatile struct {
    float fp32_over_dist;
    float fp32_err_dist[256];
    float fp32_under_dist[256];
} error_str;

typedef volatile struct {
    float fp32_turnmark_dist;
    float fp32_check_dist;
    float fp32_limit_dist;
    float fp32_turnmark_check_dist;
    uint16_t u16_state;
    uint16_t u16_mark_enable;
    uint16_t u16_turn_flag;
    uint16_t u16_single_flag;
    uint16_t u16_cross_flag;
} turnmark_t;

typedef volatile struct {
    uint16_t sensor_ready : 1;
    uint16_t lineout_flag : 1;
    uint16_t adc_error : 1;
    uint16_t motor : 1;
    uint16_t motor_start : 1;
    uint16_t motor_ISR_flag : 1;
    uint16_t move_state : 1;
    uint16_t start_flag : 1;
    uint16_t speed_up : 1;
    uint16_t speed_up_start : 1;
    uint16_t stop_check : 1;
    uint16_t end_flag : 1;
    uint16_t cross_flag : 1;
    uint16_t Lturnmark_flag : 1;
    uint16_t Rturnmark_flag : 1;
    uint16_t race_start : 1;
    uint16_t first_race : 1;
    uint16_t second_race : 1;
    uint16_t err : 1;
    uint16_t fast_flag : 1;
} bit_field_flag_t;

#endif /* __APP_STRUCT_H__ */
