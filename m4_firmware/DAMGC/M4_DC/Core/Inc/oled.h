#ifndef __OLED_H__
#define __OLED_H__

#include <stdint.h>

#define OLED_WIDTH 128U
#define OLED_HEIGHT 32U
#define OLED_PAGE_COUNT (OLED_HEIGHT / 8U)
#define OLED_TEXT_COLUMNS 21U

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);
uint8_t OLED_IsBusy(void);
void OLED_Print(uint8_t row, uint8_t col, const char *str);
void OLED_PrintCentered(uint8_t row, const char *str);
void OLED_Printf(uint8_t row, uint8_t col, const char *fmt, ...);
void OLED_ShowTextScreen(const char *title,
                         const char *line1,
                         const char *line2,
                         const char *line3);
void OLED_DrawSensorBars(const float values[16]);
void OLED_ClearBuffer(void);

extern const uint8_t font5x7[95][5];

#endif
