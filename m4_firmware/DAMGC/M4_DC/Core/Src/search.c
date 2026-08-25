#include "search.h"
#include "motor.h"
#include "sensor.h"
#include "oled.h"
#include "rom.h"
#include <stdio.h>
#include <string.h>

static uint32_t s_lineout_cnt;
static uint8_t s_route_save_pending;

#define SEARCH_MENU_DELAY_MS 125U
#define D_STR 1U
#define D_45A 10U
#define D_90A 30U
#define D_180A 40U
#define D_270A 50U
#define LONG_DIST 2000U
#define SHORT_DIST 400U
#define MID_DIST 600U
#define TURN_45_DIST 250U
#define TURN_90_DIST 460U
#define TURN_180_DIST 720U
#define TURN_270_DIST 1400U

static void turn_division_compute(fast_run_str *pinfo, int32_t mark_cnt, error_str *perr);
static void search_route_save_prompt(void);

static void search_wait_key_release(void)
{
    while ((SW_U == 0U) || (SW_D == 0U) || (SW_R == 0U) || (SW_L == 0U)) {
        LL_mDelay(10U);
    }
}

static const char *search_setting_title(const char *tag)
{
    if (strcmp(tag, "VEL") == 0) return "USER VELOCITY";
    if (strcmp(tag, "LMIT") == 0) return "END MARK LIMIT";
    if (strcmp(tag, "THOLD") == 0) return "SENSOR THRESHOLD";
    if (strcmp(tag, "MARKD") == 0) return "TURN MARK DISTANCE";
    if (strcmp(tag, "MARKT") == 0) return "TURN MARK THRESHOLD";
    if (strcmp(tag, "errflg") == 0) return "FAST ERROR CHECK";
    if (strcmp(tag, "IS") == 0) return "SEARCH INNER HANDLE";
    if (strcmp(tag, "OS") == 0) return "SEARCH OUTER HANDLE";
    if (strcmp(tag, "IF") == 0) return "FAST INNER HANDLE";
    if (strcmp(tag, "OF") == 0) return "FAST OUTER HANDLE";
    if (strcmp(tag, "EDV") == 0) return "END VELOCITY";
    if (strcmp(tag, "ET") == 0) return "END TURN ACCEL";
    if (strcmp(tag, "EA") == 0) return "END ACCEL";
    if (strcmp(tag, "AC") == 0) return "USER ACCELERATION";
    if (strcmp(tag, "Pkp") == 0) return "POSITION KP";
    if (strcmp(tag, "Pkd") == 0) return "POSITION KD";
    if (strcmp(tag, "Mkp") == 0) return "MOTOR KP";
    if (strcmp(tag, "Mkd") == 0) return "MOTOR KD";
    return tag;
}

static const char *search_setting_adjust_hint(const char *tag)
{
    if ((strcmp(tag, "VEL") == 0) ||
        (strcmp(tag, "EDV") == 0)) {
        return "UP/DOWN: +/-100";
    }
    if (strcmp(tag, "AC") == 0) return "UP/DOWN: +/-500";
    if (strcmp(tag, "ET") == 0) return "UP/DOWN: +/-1000";
    if (strcmp(tag, "EA") == 0) return "UP/DOWN: +/-100";
    if ((strcmp(tag, "IS") == 0) ||
        (strcmp(tag, "OS") == 0) ||
        (strcmp(tag, "IF") == 0) ||
        (strcmp(tag, "OF") == 0) ||
        (strcmp(tag, "Mkp") == 0) ||
        (strcmp(tag, "Mkd") == 0)) {
        return "UP/DOWN: +/-0.01";
    }
    if ((strcmp(tag, "Pkp") == 0) || (strcmp(tag, "Pkd") == 0)) {
        return "UP/DOWN: +/-0.1";
    }
    if (strcmp(tag, "errflg") == 0) return "UP: ON / DOWN: OFF";
    return "UP/DOWN: +/-1";
}

static void search_show_setting(const char *tag, const char *value_line)
{
    OLED_ClearBuffer();
    OLED_PrintCentered(0U, search_setting_title(tag));
    OLED_PrintCentered(1U, value_line);
    OLED_Print(2U, 0U, search_setting_adjust_hint(tag));
    OLED_Print(3U, 0U, "RIGHT: SAVE / NEXT");
    OLED_Update();
}

static void search_print_float_setting(const char *tag, float value)
{
    char value_line[OLED_TEXT_COLUMNS + 1U];
    long scaled_value = (long)(value * 1000.0f);
    unsigned long absolute_value;

    if (scaled_value < 0L) {
        absolute_value = (unsigned long)(-scaled_value);
        snprintf(value_line,
                 sizeof(value_line),
                 "VALUE: -%lu.%03lu",
                 absolute_value / 1000UL,
                 absolute_value % 1000UL);
    } else {
        absolute_value = (unsigned long)scaled_value;
        snprintf(value_line,
                 sizeof(value_line),
                 "VALUE: %lu.%03lu",
                 absolute_value / 1000UL,
                 absolute_value % 1000UL);
    }

    printf("%s:%s\r\n", tag, &value_line[7]);
    search_show_setting(tag, value_line);
}

static void search_print_int_setting(const char *tag, long value)
{
    char value_line[OLED_TEXT_COLUMNS + 1U];

    if (strcmp(tag, "errflg") == 0) {
        snprintf(value_line,
                 sizeof(value_line),
                 "VALUE: %s",
                 (value != 0L) ? "ON" : "OFF");
    } else {
        snprintf(value_line, sizeof(value_line), "VALUE: %ld", value);
    }

    printf("%s:%ld\r\n", tag, value);
    search_show_setting(tag, value_line);
}

void Race_Init(void)
{
    g_Flag.motor = OFF;
    g_Flag.motor_start = OFF;
    g_Flag.motor_ISR_flag = OFF;
    g_Flag.move_state = OFF;
    g_Flag.start_flag = OFF;
    g_Flag.speed_up = OFF;
    g_Flag.speed_up_start = OFF;
    g_Flag.stop_check = OFF;
    g_Flag.end_flag = OFF;
    g_Flag.lineout_flag = OFF;
    g_Flag.cross_flag = OFF;
    g_Flag.Lturnmark_flag = OFF;
    g_Flag.Rturnmark_flag = OFF;
    g_Flag.race_start = OFF;
    g_Flag.first_race = ON;
    g_Flag.second_race = OFF;
    g_Flag.err = OFF;
    g_Flag.fast_flag = OFF;
    s_lineout_cnt = 0U;
    s_route_save_pending = 0U;
    g_u16_turnmark_cnt = 0U;
    g_i32_mark_cnt = 0;
    g_i32_err_cnt = 0;
    g_i32_speed_up_cnt = 0;
    g_fp32_straight_dist = 0.0f;
    g_fp32_turn_angle = 0.0f;
    g_fp32_current_angle = 0.0f;
    g_pos.fp32_temp_pos = 0.0f;
    g_pos.fp32_temp_position = 0.0f;
    g_pos.u16_enable = 0xFFFFU;
    g_pos.u16_current_state = STRAIGHT;
    g_pos.u16_past_state = STRAIGHT;
    g_lmark.u16_mark_enable = LEFT_MARK_CHECK;
    g_lmark.u16_cross_flag = OFF;
    g_lmark.u16_single_flag = OFF;
    g_lmark.u16_turn_flag = OFF;
    g_rmark.u16_mark_enable = RIGHT_MARK_CHECK;
    g_rmark.u16_cross_flag = OFF;
    g_rmark.u16_single_flag = OFF;
    g_rmark.u16_turn_flag = OFF;

    motor_reset_motion_state();
}

void search_route_save_request(void)
{
    s_route_save_pending = 1U;
}

static void search_route_save_prompt(void)
{
    OLED_ShowTextScreen("FIRST RACE COMPLETE",
                        "ROUTE DATA READY",
                        "RIGHT: SAVE ROUTE",
                        "DOWN: DISCARD");

    while (1) {
        if (SW_R == 0U) {
            search_wait_key_release();

            if (s_route_save_pending != 0U) {
                sensor_scan_stop();
                fast_infor_write_rom();
                mark_write_rom();
                sensor_scan_start();
                s_route_save_pending = 0U;
                OLED_ShowTextScreen("ROUTE DATA SAVED",
                                    "TURN MARKS SAVED",
                                    "DISTANCES SAVED",
                                    "RETURNING TO MENU");
            } else {
                OLED_ShowTextScreen("ROUTE SAVE SKIPPED",
                                    "NO ROUTE DATA",
                                    "NOTHING WAS WRITTEN",
                                    "RETURNING TO MENU");
            }

            LL_mDelay(700U);
            motor_enable_control(OFF);
            return;
        }

        if (SW_D == 0U) {
            s_route_save_pending = 0U;
            OLED_ShowTextScreen("ROUTE NOT SAVED",
                                "NEW DATA DISCARDED",
                                "FLASH NOT CHANGED",
                                "RELEASE DOWN KEY");
            search_wait_key_release();
            LL_mDelay(500U);
            motor_enable_control(OFF);
            return;
        }

        LL_mDelay(10U);
    }
}

void search_run(void)
{
    char speed_line[OLED_TEXT_COLUMNS + 1U];

    printf("Sch_%ld\r\n", (long)g_fp32_user_vel);
    snprintf(speed_line, sizeof(speed_line), "SPEED: %ld mm/s", (long)g_fp32_user_vel);
    OLED_ShowTextScreen("FIRST RACE", speed_line, "READY TO START", "DOWN: ABORT");
    LL_mDelay(1000U);
    OLED_ShowTextScreen("FIRST RACE", speed_line, "RUNNING...", "DOWN: ABORT");
    LL_mDelay(500U);

    memset((void *)g_fast_info, 0, sizeof(g_fast_info));
    memset((void *)&g_err, 0, sizeof(g_err));
    memset((void *)search_info, 0, sizeof(search_info));
    g_i32_total_cnt = 0;

    Race_Init();
    handle_ad_make(g_fp32_out_corner_limit, g_fp32_in_corner_limit);
    move_to_move(SEARCH_DEFAULT_DIST_MM,
                 0.0f,
                 g_fp32_user_vel,
                 g_fp32_user_vel,
                 SEARCH_DEFAULT_ACC_MM_S2);
    g_Flag.move_state = ON;
    g_Flag.start_flag = ON;
    motor_enable_control(ON);

    while (1) {
        if (g_Flag.move_state == ON) {
            g_lmark.fp32_turnmark_dist =
                (g_lm.fp32_turnmark_check_dist + g_rm.fp32_turnmark_check_dist) * 0.5f;
            g_rmark.fp32_turnmark_dist = g_lmark.fp32_turnmark_dist;
            turn_decide(&g_lmark, &g_rmark);
            turn_decide(&g_rmark, &g_lmark);
        }

        if (g_Flag.motor_ISR_flag == ON) {
            g_Flag.motor_ISR_flag = OFF;

            if (race_stop_check() != 0) {
                search_route_save_prompt();
                return;
            }
        }

        if (SW_D == 0U) {
            search_run_stop();
            OLED_ShowTextScreen("FIRST RACE",
                                "USER ABORT",
                                "MOTOR STOPPED",
                                "RELEASE DOWN KEY");
            search_wait_key_release();
            return;
        }
    }
}

void turn_info_compute(fast_run_str *pinfo, int32_t mark_cnt)
{
    int32_t temp;
    float user_vel_2000;
    float max_dist;
    float min_dist;

    if (pinfo == NULL) {
        return;
    }

    user_vel_2000 = g_fp32_user_vel / 2000.0f;

    if (mark_cnt == 0) {
        pinfo->u16_turn_way = STRAIGHT;
    }

    if (((pinfo->u16_turn_way & STRAIGHT) != 0U) && ((pinfo->u16_turn_way & ETURN) == 0U)) {
        pinfo->u16_turn_dir = STRAIGHT;
        pinfo->u16_turn_cnt = D_STR;

        if (mark_cnt != 0) {
            if (pinfo->u16_dist > SHORT_DIST) {
                temp = (int32_t)pinfo->u16_dist -
                       (int32_t)(user_vel_2000 * (float)((pinfo - 1)->u16_turn_cnt));
                if (temp <= 0) {
                    temp = (int32_t)pinfo->u16_dist;
                    (pinfo - 1)->u16_turn_cnt = D_STR;
                }
            } else {
                temp = (int32_t)pinfo->u16_dist;
                (pinfo - 1)->u16_turn_cnt = D_STR;
            }

            pinfo->u16_dist = (uint16_t)temp;
        }
    } else if (((pinfo->u16_turn_way & STRAIGHT) == 0U) && ((pinfo->u16_turn_way & ETURN) == 0U)) {
        if (pinfo->u16_dist <= TURN_45_DIST) {
            pinfo->u16_turn_dir = TURN_45 | pinfo->u16_turn_way;
            pinfo->u16_turn_cnt = (((pinfo + 1)->u16_turn_way & STRAIGHT) != 0U) ? D_45A : D_STR;
        } else if (pinfo->u16_dist <= TURN_90_DIST) {
            pinfo->u16_turn_dir = TURN_90 | pinfo->u16_turn_way;
            pinfo->u16_turn_cnt = (((pinfo + 1)->u16_turn_way & STRAIGHT) != 0U) ? D_90A : D_STR;
        } else if (pinfo->u16_dist <= TURN_180_DIST) {
            pinfo->u16_turn_dir = TURN_180 | pinfo->u16_turn_way;
            pinfo->u16_turn_cnt = (((pinfo + 1)->u16_turn_way & STRAIGHT) != 0U) ? D_180A : D_STR;
        } else if (pinfo->u16_dist <= TURN_270_DIST) {
            pinfo->u16_turn_dir = TURN_270 | pinfo->u16_turn_way;
            pinfo->u16_turn_cnt = (((pinfo + 1)->u16_turn_way & STRAIGHT) != 0U) ? D_270A : D_STR;
        } else {
            max_dist = (pinfo->fp32_left_dist > pinfo->fp32_right_dist) ? pinfo->fp32_left_dist : pinfo->fp32_right_dist;
            min_dist = (pinfo->fp32_left_dist > pinfo->fp32_right_dist) ? pinfo->fp32_right_dist : pinfo->fp32_left_dist;

            if ((min_dist > 0.0f) && ((max_dist / min_dist) < 3.0f)) {
            pinfo->u16_turn_dir = LARGE_TURN | pinfo->u16_turn_way;
            pinfo->u16_turn_cnt = D_STR;

            if (mark_cnt != 0) {
                if (pinfo->u16_dist > SHORT_DIST) {
                    temp = (int32_t)pinfo->u16_dist -
                           (int32_t)(user_vel_2000 * (float)((pinfo - 1)->u16_turn_cnt));
                    if (temp <= 0) {
                        temp = (int32_t)pinfo->u16_dist;
                        (pinfo - 1)->u16_turn_cnt = D_STR;
                    }
                } else {
                    temp = (int32_t)pinfo->u16_dist;
                    (pinfo - 1)->u16_turn_cnt = D_STR;
                }

                pinfo->u16_dist = (uint16_t)temp;
            }
            } else {
                pinfo->u16_turn_dir = TURN_270 | pinfo->u16_turn_way;
                pinfo->u16_turn_cnt = (((pinfo + 1)->u16_turn_way & STRAIGHT) != 0U) ? D_270A : D_STR;
            }
        }
    } else {
        pinfo->u16_turn_dir = pinfo->u16_turn_way;
    }
}

void turn_info_func(void)
{
    int32_t i;

    for (i = 0; i <= g_i32_total_cnt; i++) {
        turn_info_compute((fast_run_str *)&g_fast_info[i], i);
    }
}

void print_second_info(void)
{
    int32_t i;
    int32_t stop = g_i32_total_cnt + 5;

    if (stop < 5) {
        stop = 5;
    }

    if (stop > 255) {
        stop = 255;
    }

    printf("SECOND INFO total=%ld\r\n", (long)g_i32_total_cnt);
    for (i = 0; i <= stop; i++) {
        printf("%ld| dst:%5u| dec:%5ld| mdst:%5ld| turn_dir:0x%04X| way:0x%04X| acc:%5ld| in:%5ld| vel:%5ld| out:%5ld| cnt:%u| RDIST:%ld| LDIST:%ld| angle:%ld\r\n",
               (long)i,
               (unsigned int)g_fast_info[i].u16_dist,
               (long)g_fast_info[i].fp32_decel_dist,
               (long)g_fast_info[i].fp32_motion_dist,
               (unsigned int)g_fast_info[i].u16_turn_dir,
               (unsigned int)g_fast_info[i].u16_turn_way,
               (long)g_fast_info[i].fp32_accel,
               (long)g_fast_info[i].fp32_in_vel,
               (long)g_fast_info[i].fp32_vel,
               (long)g_fast_info[i].fp32_out_vel,
               (unsigned int)g_fast_info[i].u16_turn_cnt,
               (long)g_fast_info[i].fp32_right_dist,
               (long)g_fast_info[i].fp32_left_dist,
               (long)g_fast_info[i].fp32_angle);

        if (i == g_i32_total_cnt) {
            printf("-----------------------------------\r\n");
        }
    }
}

void fst_info(void)
{
    printf("fst_info\r\n");
    mark_read_rom();
    fast_infor_read_rom();
    turn_info_func();
    turn_division_func();
    print_second_info();
}

void fast_run(void)
{
    char speed_line[OLED_TEXT_COLUMNS + 1U];

    printf("fast_run\r\n");
    snprintf(speed_line, sizeof(speed_line), "SPEED: %ld mm/s", (long)g_fp32_user_vel);
    OLED_ShowTextScreen("SECOND RACE", speed_line, "PREPARING ROUTE", "DOWN: ABORT");
    LL_mDelay(500U);
    second_run((fast_run_str *)g_fast_info);
}

static void straight_compute(fast_run_str *pinfo, int32_t mark_cnt, error_str *perr)
{
    float big_vel;
    float small_vel;

    pinfo->fp32_in_vel = (mark_cnt > 0) ? (pinfo - 1)->fp32_out_vel : 0.0f;

    if ((pinfo->u16_turn_dir & ETURN) == 0U) {
        turn_division_compute(pinfo + 1, mark_cnt + 1, perr);
        pinfo->fp32_out_vel = g_fp32_user_vel;
    } else {
        (pinfo + 1)->fp32_in_vel = g_fp32_end_vel;
        pinfo->fp32_out_vel = g_fp32_end_vel;
    }

    pinfo->fp32_accel = ((pinfo->u16_turn_dir & ETURN) != 0U) ? g_fp32_endturn_acc : g_fp32_user_acc;

    if ((mark_cnt == 0) && (pinfo->fp32_accel > 10000.0f)) {
        pinfo->fp32_accel = 10000.0f;
    }

    big_vel = (pinfo->fp32_in_vel > pinfo->fp32_out_vel) ? pinfo->fp32_in_vel : pinfo->fp32_out_vel;
    small_vel = (pinfo->fp32_in_vel > pinfo->fp32_out_vel) ? pinfo->fp32_out_vel : pinfo->fp32_in_vel;

    decel_dist_compute(pinfo->fp32_in_vel,
                       pinfo->fp32_out_vel,
                       pinfo->fp32_accel,
                       &pinfo->fp32_motion_dist);

    if (pinfo->fp32_motion_dist >= (float)pinfo->u16_dist) {
        pinfo->fp32_decel_dist = (float)pinfo->u16_dist;
        max_vel_compute((float)pinfo->u16_dist,
                        0.0f,
                        small_vel,
                        pinfo->fp32_accel,
                        g_fp32_fast_vel_limit,
                        g_fp32_user_vel,
                        &pinfo->fp32_vel);

        if (pinfo->fp32_in_vel > pinfo->fp32_out_vel) {
            pinfo->fp32_in_vel = pinfo->fp32_vel;
        } else {
            pinfo->fp32_out_vel = pinfo->fp32_vel;
        }

        if (mark_cnt == 0) {
            pinfo->fp32_in_vel = 0.0f;
        }
    } else {
        max_vel_compute((float)pinfo->u16_dist,
                        pinfo->fp32_motion_dist,
                        big_vel,
                        pinfo->fp32_accel,
                        g_fp32_fast_vel_limit,
                        g_fp32_user_vel,
                        &pinfo->fp32_vel);
        decel_dist_compute(pinfo->fp32_vel,
                           pinfo->fp32_out_vel,
                           pinfo->fp32_accel,
                           &pinfo->fp32_decel_dist);
    }

    perr->fp32_err_dist[mark_cnt] = (float)(pinfo->u16_dist << 2);
    if (perr->fp32_err_dist[mark_cnt] > (float)(MID_DIST + SHORT_DIST)) {
        perr->fp32_err_dist[mark_cnt] = (float)(MID_DIST + SHORT_DIST);
    }

    perr->fp32_err_dist[mark_cnt] += (float)pinfo->u16_dist;
    perr->fp32_under_dist[mark_cnt] = (float)pinfo->u16_dist * 0.7f;
}

static void default_turn_compute(fast_run_str *pinfo, int32_t mark_cnt, error_str *perr)
{
    pinfo->fp32_accel = g_fp32_user_acc;
    pinfo->fp32_in_vel = g_fp32_user_vel;
    pinfo->fp32_vel = pinfo->fp32_in_vel;
    pinfo->fp32_out_vel = pinfo->fp32_in_vel;

    perr->fp32_err_dist[mark_cnt] = ((float)pinfo->u16_dist * 0.5f) + (float)pinfo->u16_dist;
    perr->fp32_under_dist[mark_cnt] = (float)pinfo->u16_dist * 0.65f;

}

static void turn_division_compute(fast_run_str *pinfo, int32_t mark_cnt, error_str *perr)
{
    if (((pinfo->u16_turn_way & STRAIGHT) != 0U) || ((pinfo->u16_turn_dir & ETURN) != 0U)) {
        straight_compute(pinfo, mark_cnt, perr);
    } else if ((pinfo->u16_turn_dir & LARGE_TURN) != 0U) {
        default_turn_compute(pinfo, mark_cnt, perr);
    } else {
        default_turn_compute(pinfo, mark_cnt, perr);
    }
}

void turn_division_func(void)
{
    int32_t i;

    for (i = 0; i < g_i32_total_cnt; i++) {
        turn_division_compute((fast_run_str *)&g_fast_info[i], i, (error_str *)&g_err);
    }
}

void speed_up_compute(fast_run_str *pinfo)
{
    if ((g_Flag.speed_up_start == OFF) || (g_Flag.err == ON)) {
        return;
    }

    if ((g_i32_mark_cnt == 0) ||
        (g_fp32_straight_dist >= (float)(pinfo + g_i32_mark_cnt - 1)->u16_turn_cnt)) {
        g_Flag.speed_up = ON;
        g_Flag.speed_up_start = OFF;
        g_i32_speed_up_cnt = 0;
    }
}

void second_infor(fast_run_str *pinfo, error_str *perr)
{
    perr->fp32_over_dist = (g_rm.fp32_gone_distance + g_lm.fp32_gone_distance) * 0.5f;

    if ((g_Flag.err == OFF) &&
        (perr->fp32_over_dist < perr->fp32_under_dist[g_i32_mark_cnt])) {
        return;
    }

    g_i32_mark_cnt++;

    if ((g_Flag.err == OFF) && (g_i32_mark_cnt > g_i32_total_cnt)) {
        g_Flag.err = ON;
        g_lm.fp32_gone_distance = 0.0f;
        g_rm.fp32_gone_distance = 0.0f;
        return;
    }

    if (((pinfo + g_i32_mark_cnt)->u16_turn_dir & (STRAIGHT | LARGE_TURN | ETURN)) != 0U) {
        g_Flag.speed_up_start = ON;
    }

    move_to_move((float)(pinfo + g_i32_mark_cnt)->u16_dist,
                 (pinfo + g_i32_mark_cnt)->fp32_decel_dist,
                 (pinfo + g_i32_mark_cnt)->fp32_vel,
                 (pinfo + g_i32_mark_cnt)->fp32_out_vel,
                 (pinfo + g_i32_mark_cnt)->fp32_accel);

    perr->fp32_over_dist = 0.0f;
    g_lm.fp32_gone_distance = 0.0f;
    g_rm.fp32_gone_distance = 0.0f;
    g_lm.fp32_total_dist = 0.0f;
    g_rm.fp32_total_dist = 0.0f;
    g_lm.fp32_dist_sum = 0.0f;
    g_rm.fp32_dist_sum = 0.0f;
    g_pos.fp32_integral_val = 0.0f;
    g_pos.u16_past_state = (pinfo + g_i32_mark_cnt)->u16_turn_dir;
}

void fast_error_compute(error_str *perr, fast_run_str *pinfo, int32_t mark_cnt)
{
    fast_run_str *pnow;
    fast_run_str *pnext;
    int32_t dist;
    float big_vel;

    if ((mark_cnt < 0) || (mark_cnt >= 255)) {
        return;
    }

    perr->fp32_over_dist = (g_rm.fp32_gone_distance + g_lm.fp32_gone_distance) * 0.5f;

    if (g_Flag.err == ON) {
        g_Flag.fast_flag = OFF;
        return;
    }

    if (perr->fp32_over_dist <= perr->fp32_err_dist[mark_cnt]) {
        return;
    }

    g_i32_err_cnt++;

    if ((g_i32_fast_error_flag != 0) &&
        ((g_i32_err_cnt > 10) || (mark_cnt > (g_i32_total_cnt - 1)))) {
        printf("FAST ERROR\r\n");
        OLED_ShowTextScreen("SECOND RACE ERROR",
                            "ROUTE SYNC FAILED",
                            "SEARCH MODE",
                            "DOWN: RETURN");
        g_Flag.err = ON;
        g_Flag.fast_flag = OFF;
        if (g_fp32_user_vel > 2200.0f) {
            g_fp32_user_vel = 2200.0f;
        }
        return;
    }

    pnow = pinfo + mark_cnt;
    pnext = pinfo + mark_cnt + 1;
    dist = (int32_t)perr->fp32_err_dist[mark_cnt] - (int32_t)pnow->u16_dist;

    if ((int32_t)pnext->u16_dist > dist) {
        pnext->u16_dist = (uint16_t)((int32_t)pnext->u16_dist - dist);
    } else {
        pnext->u16_dist = 10U;
    }

    big_vel = (pinfo->fp32_in_vel > pinfo->fp32_out_vel) ? pinfo->fp32_in_vel : pinfo->fp32_out_vel;
    max_vel_compute((float)pnext->u16_dist,
                    pnext->fp32_motion_dist,
                    big_vel,
                    pnext->fp32_accel,
                    g_fp32_fast_vel_limit,
                    g_fp32_user_vel,
                    &pnext->fp32_vel);
    decel_dist_compute(pnext->fp32_vel,
                       pnext->fp32_out_vel,
                       pnext->fp32_accel,
                       &pnext->fp32_decel_dist);

    perr->fp32_under_dist[mark_cnt + 1] = (float)(pnext->u16_dist >> 1);
    second_infor(pinfo, perr);
}

void second_run(fast_run_str *pinfo)
{
    mark_read_rom();
    fast_infor_read_rom();

    if ((pinfo == NULL) || (g_i32_total_cnt <= 0)) {
        printf("FAST NO ROUTE total=%ld\r\n", (long)g_i32_total_cnt);
        OLED_ShowTextScreen("SECOND RACE",
                            "NO ROUTE DATA",
                            "RUN FIRST RACE",
                            "DOWN: RETURN");
        LL_mDelay(500U);
        return;
    }

    Race_Init();
    turn_info_func();
    turn_division_func();

    char speed_line[OLED_TEXT_COLUMNS + 1U];

    printf("FAST START total=%ld vel=%ld\r\n", (long)g_i32_total_cnt, (long)g_fp32_user_vel);
    snprintf(speed_line, sizeof(speed_line), "SPEED: %ld mm/s", (long)g_fp32_user_vel);
    OLED_ShowTextScreen("SECOND RACE", speed_line, "RUNNING...", "DOWN: ABORT");
    LL_mDelay(500U);

    handle_ad_make(g_fp32_out_corner_fast_limit, g_fp32_in_corner_fast_limit);
    move_to_move((float)pinfo->u16_dist, pinfo->fp32_decel_dist, pinfo->fp32_vel, pinfo->fp32_out_vel, pinfo->fp32_accel);

    g_Flag.first_race = OFF;
    g_Flag.second_race = ON;
    g_Flag.fast_flag = ON;
    g_Flag.motor_start = ON;
    g_Flag.start_flag = ON;

    while (1) {
        g_fp32_straight_dist =
            (g_rm.fp32_gone_distance + g_lm.fp32_gone_distance) * 0.5f;

        if (g_Flag.move_state == ON) {
            g_lmark.fp32_turnmark_dist =
                (g_lm.fp32_turnmark_check_dist + g_rm.fp32_turnmark_check_dist) * 0.5f;
            g_rmark.fp32_turnmark_dist = g_lmark.fp32_turnmark_dist;
            turn_decide(&g_lmark, &g_rmark);
            turn_decide(&g_rmark, &g_lmark);
        }

        if (g_Flag.motor_ISR_flag == ON) {
            g_Flag.motor_ISR_flag = OFF;

            if (line_out_func() != 0) {
                printf("FAST LINEOUT\r\n");
                OLED_ShowTextScreen("SECOND RACE",
                                    "LINE OUT DETECTED",
                                    "MOTOR STOPPED",
                                    "DOWN: RETURN");
                return;
            }

            speed_up_compute(pinfo);
            fast_error_compute((error_str *)&g_err, pinfo, g_i32_mark_cnt);

            if (race_stop_check() != 0) {
                printf("FAST STOP\r\n");
                OLED_ShowTextScreen("SECOND RACE",
                                    "END MARK DETECTED",
                                    "RACE COMPLETE",
                                    "DOWN: RETURN");
                return;
            }
        }

        if (SW_D == 0U) {
            search_run_stop();
            printf("FAST ABORT\r\n");
            OLED_ShowTextScreen("SECOND RACE",
                                "USER ABORT",
                                "MOTOR STOPPED",
                                "RELEASE DOWN KEY");
            search_wait_key_release();
            return;
        }
    }
}

void Set_Velocity(void)
{
    search_wait_key_release();

    while (1) {
        if (SW_U == 0U) {
            g_fp32_user_vel += 100.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_user_vel -= 100.0f;
            if (g_fp32_user_vel < 0.0f) {
                g_fp32_user_vel = 0.0f;
            }
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            turnvel_write_rom();
            break;
        }

        search_print_float_setting("VEL", g_fp32_user_vel);
        LL_mDelay(20U);
    }
}

void Set_TurnMark(void)
{
    search_wait_key_release();

    while (1) {
        if (SW_U == 0U) {
            g_u16_turnmark_limit++;
            LL_mDelay(50U);
        } else if (SW_D == 0U) {
            if (g_u16_turnmark_limit > 0U) {
                g_u16_turnmark_limit--;
            }
            LL_mDelay(50U);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_int_setting("LMIT", (long)g_u16_turnmark_limit);
    }

    while (1) {
        if (SW_U == 0U) {
            g_fp32_sensor_value_max += 1.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_sensor_value_max -= 1.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_float_setting("THOLD", g_fp32_sensor_value_max);
    }

    while (1) {
        if (SW_U == 0U) {
            g_fp32_turnmark_dist += 1.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_turnmark_dist -= 1.0f;
            if (g_fp32_turnmark_dist < 0.0f) {
                g_fp32_turnmark_dist = 0.0f;
            }
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            g_fp32_straight_mark_dist = g_fp32_turnmark_dist + 80.0f;
            g_fp32_mark_dist = g_fp32_turnmark_dist;
            g_u16_turn_dist = (uint16_t)g_fp32_turnmark_dist;
            search_wait_key_release();
            break;
        }
        search_print_float_setting("MARKD", g_fp32_turnmark_dist);
    }

    while (1) {
        if (SW_U == 0U) {
            g_fp32_turn_threshold += 1.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_turn_threshold -= 1.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_float_setting("MARKT", g_fp32_turn_threshold);
    }

    while (1) {
        if (SW_U == 0U) {
            g_i32_fast_error_flag = 1;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_i32_fast_error_flag = 0;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            turnmark_info_write_rom();
            break;
        }
        search_print_int_setting("errflg", (long)g_i32_fast_error_flag);
    }
}

void Set_Handle(void)
{
    search_wait_key_release();

    while (1) {
        if (SW_U == 0U) {
            g_fp32_in_corner_limit += 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_in_corner_limit -= 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_float_setting("IS", g_fp32_in_corner_limit);
    }

    while (1) {
        if (SW_U == 0U) {
            g_fp32_out_corner_limit += 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_out_corner_limit -= 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_float_setting("OS", g_fp32_out_corner_limit);
    }

    while (1) {
        if (SW_U == 0U) {
            g_fp32_in_corner_fast_limit += 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_in_corner_fast_limit -= 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_float_setting("IF", g_fp32_in_corner_fast_limit);
    }

    while (1) {
        if (SW_U == 0U) {
            g_fp32_out_corner_fast_limit += 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_out_corner_fast_limit -= 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            handle_ad_make(g_fp32_out_corner_limit, g_fp32_in_corner_limit);
            handle_write_rom();
            break;
        }
        search_print_float_setting("OF", g_fp32_out_corner_fast_limit);
    }
}

void SET_END(void)
{
    search_wait_key_release();

    while (1) {
        if (SW_U == 0U) {
            g_fp32_end_vel += 100.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_end_vel -= 100.0f;
            if (g_fp32_end_vel < 0.0f) {
                g_fp32_end_vel = 0.0f;
            }
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_float_setting("EDV", g_fp32_end_vel);
    }

    while (1) {
        if (SW_U == 0U) {
            g_fp32_end_acc += 100.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_end_acc -= 100.0f;
            if (g_fp32_end_acc < 0.0f) {
                g_fp32_end_acc = 0.0f;
            }
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            acc_info_write_rom();
            break;
        }
        search_print_float_setting("EA", g_fp32_end_acc);
    }
}

void Set_Accel(void)
{
    search_wait_key_release();

    while (1) {
        if (SW_U == 0U) {
            g_fp32_user_acc += 500.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_user_acc -= 500.0f;
            if (g_fp32_user_acc < 0.0f) {
                g_fp32_user_acc = 0.0f;
            }
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_float_setting("AC", g_fp32_user_acc);
    }

    while (1) {
        if (SW_U == 0U) {
            g_fp32_endturn_acc += 1000.0f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_endturn_acc -= 1000.0f;
            if (g_fp32_endturn_acc < 0.0f) {
                g_fp32_endturn_acc = 0.0f;
            }
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            SET_END();
            break;
        }
        search_print_float_setting("ET", g_fp32_endturn_acc);
    }
}

void Set_PosPID(void)
{
    search_wait_key_release();

    while (1) {
        if (SW_U == 0U) {
            g_pos.fp32_kp += 0.1f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_pos.fp32_kp -= 0.1f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_float_setting("Pkp", g_pos.fp32_kp);
    }

    while (1) {
        if (SW_U == 0U) {
            g_pos.fp32_kd += 0.1f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_pos.fp32_kd -= 0.1f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            printf("POS PID SAVE RAM\r\n");
            break;
        }
        search_print_float_setting("Pkd", g_pos.fp32_kd);
    }
}

void Set_MotorPID(void)
{
    search_wait_key_release();

    while (1) {
        if (SW_U == 0U) {
            g_fp32_motor_kp += 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_motor_kp -= 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            break;
        }
        search_print_float_setting("Mkp", g_fp32_motor_kp);
    }

    while (1) {
        if (SW_U == 0U) {
            g_fp32_motor_kd += 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_D == 0U) {
            g_fp32_motor_kd -= 0.01f;
            LL_mDelay(SEARCH_MENU_DELAY_MS);
        } else if (SW_R == 0U) {
            search_wait_key_release();
            g_lm.fp32_kp = g_rm.fp32_kp = g_fp32_motor_kp;
            g_lm.fp32_kd = g_rm.fp32_kd = g_fp32_motor_kd;
            printf("MOTOR PID SAVE RAM\r\n");
            break;
        }
        search_print_float_setting("Mkd", g_fp32_motor_kd);
    }
}

void search_run_stop(void)
{
    move_to_end(100.0f, 0.0f, 12500.0f);
    motor_enable_control(OFF);
    g_Flag.move_state = OFF;
    g_Flag.start_flag = OFF;
    g_Flag.stop_check = ON;
}

int line_out_func(void)
{
    if (g_Flag.stop_check == ON) {
        return 0;
    }

    if (g_Flag.lineout_flag == ON) {
        s_lineout_cnt++;

        if (s_lineout_cnt >= 200U) {
            s_lineout_cnt = 0U;
            g_Flag.move_state = OFF;
            g_Flag.stop_check = ON;
            move_to_end(100.0f, 0.0f, 13000.0f);

            while ((g_lm.fp32_next_vel >= 30.0f) &&
                   (g_rm.fp32_next_vel >= 30.0f)) {
            }

            motor_enable_control(OFF);
            g_Flag.start_flag = OFF;
            return 1;
        }
    } else {
        s_lineout_cnt = 0U;
    }

    return 0;
}

int race_stop_check(void)
{
    if (g_Flag.stop_check == OFF) {
        return 0;
    }

    if ((g_lm.fp32_next_vel < 30.0f) && (g_rm.fp32_next_vel < 30.0f)) {
        if ((s_route_save_pending == 0U) || (g_Flag.fast_flag != OFF)) {
            motor_enable_control(OFF);
        }
        return 1;
    }

    return 0;
}
