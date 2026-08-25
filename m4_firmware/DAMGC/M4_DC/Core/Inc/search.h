#ifndef __SEARCH_H__
#define __SEARCH_H__

#include "variable.h"

#define SEARCH_DEFAULT_DIST_MM 1000.0f
#define SEARCH_DEFAULT_VEL_MM_S 2300.0f
#define SEARCH_DEFAULT_ACC_MM_S2 5000.0f

void Race_Init(void);
void search_route_save_request(void);
void search_run_stop(void);
void search_run(void);
void fast_run(void);
void second_run(fast_run_str *pinfo);
void second_infor(fast_run_str *pinfo, error_str *perr);
void fast_error_compute(error_str *perr, fast_run_str *pinfo, int32_t mark_cnt);
void speed_up_compute(fast_run_str *pinfo);
void turn_division_func(void);
void fst_info(void);
void print_second_info(void);
void turn_info_func(void);
void turn_info_compute(fast_run_str *pinfo, int32_t mark_cnt);
int line_out_func(void);
int race_stop_check(void);
void Set_TurnMark(void);
void Set_Velocity(void);
void Set_Handle(void);
void Set_Accel(void);
void Set_MotorPID(void);
void Set_PosPID(void);

#endif /* __SEARCH_H__ */
