#include "menu.h"
#include "motor.h"
#include "sensor.h"
#include "oled.h"
#include "search.h"
#include "variable.h"

#include <stdio.h>
#include <string.h>

#define Down_SW   (SW_D == 0U)
#define Up_SW     (SW_U == 0U)
#define Left_SW   (SW_L == 0U)
#define Right_SW  (SW_R == 0U)

#define Down_W    0
#define Up_W      0
#define Left_W    0
#define Right_W   0

#define DELAY     135U

int32_t row = 0;
int32_t column = 0;

char menu_sel[ROW][COLUMN][OLED_TEXT_COLUMNS + 1U] = {
    {"SENSOR CALIBRATION", "RAW SENSOR VALUES", "SENSOR BAR GRAPH", "TURN MARK SETUP", "ENCODER TEST"},
    {"VELOCITY SETUP", "ACCELERATION SETUP", "HANDLE SETUP", "MOTOR PID SETUP", "POSITION PID SETUP"},
    {"FIRST RACE", "SECOND RACE", "FAST RUN INFO", "MOTOR PWM TEST", "NOT USED"}
};

static const char *const menu_category[ROW] = {
    "SENSOR SETTINGS",
    "MOTOR SETTINGS",
    "RACE OPERATIONS"
};

static void Set_Max_Min(void);
static void menu_serial_print(void);

void (* menu_functions[ROW][COLUMN])(void) =
{
    {Set_Max_Min,    F_4095,        F_127,        Set_TurnMark,   motor_encoder_test},
    {Set_Velocity,   Set_Accel,     Set_Handle,   Set_MotorPID,   Set_PosPID},
    {search_run,     fast_run,      fst_info,     motor_pwm_test, NULL}
};

void menu(void)
{
    if (Down_SW || Down_W) {
        LL_mDelay(DELAY);
        row++;
        column = 0;
        if (row > ROW - 1) {
            row = 0;
        }
    }

    if (Right_SW || Right_W) {
        LL_mDelay(DELAY);
        column++;
        if (column > COLUMN - 1) {
            column = 0;
        }
    }

    if (Left_SW || Left_W) {
        LL_mDelay(DELAY);
        column--;
        if (column < 0) {
            column = COLUMN - 1;
        }
    }

    if (Up_SW) {
        LL_mDelay(DELAY);
        if (menu_functions[row][column] != NULL) {
            printf("ENTER [%ld,%ld] %s\r\n",
                   (long)row,
                   (long)column,
                   menu_sel[row][column]);
            menu_functions[row][column]();
            OLED_Clear();
        }
    }

    OLED_ClearBuffer();

    char dots[10] = "o o o o o";
    dots[column * 2] = '*';

    OLED_PrintCentered(0U, menu_category[row]);
    OLED_PrintCentered(1U, dots);
    OLED_PrintCentered(2U, menu_sel[row][column]);
    OLED_PrintCentered(3U, "UP:OPEN L/R:SELECT");
    OLED_Update();
    menu_serial_print();
}

void menu_start(void)
{
    OLED_Init();
    sensor_scan_start();
    printf("MENU START\r\n");

    while (1) {
        menu();
        LL_mDelay(1U);
    }
}

static void menu_serial_print(void)
{
    static int32_t prev_row = -1;
    static int32_t prev_column = -1;
    if ((prev_row == row) && (prev_column == column)) {
        return;
    }

    printf("MENU [%ld,%ld] %-16s %s\r\n",
           (long)row,
           (long)column,
           menu_category[row],
           menu_sel[row][column]);

    prev_row = row;
    prev_column = column;
}

static void Set_Max_Min(void)
{
    F_Max_min();
}
