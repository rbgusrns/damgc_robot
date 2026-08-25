#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "variable.h"

#define SEN_NUM 8U
#define ADC_NUM 16U
#define LEFT_ENABLE  0x003FU
#define RIGHT_ENABLE 0xFC00U
#define LEFT_MARK_CHECK  0xF000U
#define RIGHT_MARK_CHECK 0x000FU
#define FRONT_MARK_CHECK 0x0004U

#define STRAIGHT   0x0001
#define LTURN      0x0002
#define RTURN      0x0004
#define ETURN      0x0008
#define CTURN      0x000C
#define TURN_45    0x0010
#define TURN_90    0x0020
#define TURN_180   0x0040
#define TURN_270   0x0080
#define LARGE_TURN 0x0100

#define LEFT_TURN  LTURN
#define RIGHT_TURN RTURN
#define END_TURN   ETURN

void sen_vari_init(void);
extern void sensor_scan_start(void);
extern void sensor_scan_stop(void);
void sensor_scan_cycle_start(void);
void sensor_tim2_irq_handler(void);
void adc_timer_ISR(void);
void F_4095(void);
void F_Max_min(void);
void F_127(void);
void make_position(void);
void position_PID(void);
void handle_ad_make(float acc_rate, float dec_rate);
void turn_decide(turnmark_t *pmark, turnmark_t *premark);

#endif /* __SENSOR_H__ */
